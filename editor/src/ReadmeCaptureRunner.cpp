#include <quantum/editor/ReadmeCapture.hpp>

#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/editor/AuthoredTrackEditTransaction.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/editor/EditorUi.hpp>
#include <quantum/editor/RiderLoadDiagnostics.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <SDL3/SDL.h>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>

namespace quantum::editor
{
    namespace
    {
        struct LoadedCapture
        {
            coaster::AuthoredTrack track;
            CenterlineVisualization centerline;
            coaster::RiderLoadHistory loads;
        };

        [[noreturn]] void sdlError(const char* operation)
        {
            throw std::runtime_error(std::string(operation) + " failed: " + SDL_GetError());
        }

        LoadedCapture loadCapture(const ReadmeCaptureScenario& scenario)
        {
            std::ifstream input(scenario.document, std::ios::binary);
            if (!input)
                throw std::runtime_error("Cannot read capture document: " + scenario.document.string());
            const std::string json{std::istreambuf_iterator<char>(input), {}};
            auto track = coaster::deserializeCoasterDocument(json);
            if (!track)
                throw std::invalid_argument(scenario.document.string() + ": " + track.error());
            validateReadmeCaptureDocument(scenario, *track);
            // The same solve/load acceptance as File > Open, with no document mutation.
            auto centerline = createCenterlineVisualization(*track);
            auto loads = evaluateRiderLoadDiagnostics(*track);
            AuthoredTrackEditTransaction transaction{*track};
            transaction.requireAcceptableRiderLoads(loads);
            if (scenario.kind == ReadmeCaptureKind::ForceDiagnostics)
            {
                RiderLoadDiagnosticsModel model;
                model.update(*track, loads);
                model.selectSection(scenario.region);
                if (model.selectedSection().samples.size() < 2)
                    throw std::invalid_argument("force-diagnostics: selected region has no plottable rider loads.");
            }
            return {std::move(*track), std::move(centerline), std::move(loads)};
        }

        void savePng(renderer::FrameImage& image, const std::filesystem::path& path)
        {
            std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> surface(
                SDL_CreateSurfaceFrom(static_cast<int>(image.width), static_cast<int>(image.height),
                    SDL_PIXELFORMAT_RGBA32, image.pixels.data(), static_cast<int>(image.width * 4)),
                SDL_DestroySurface);
            if (!surface)
                sdlError("SDL_CreateSurfaceFrom for PNG");
            const auto utf8Path = path.u8string();
            if (!SDL_SavePNG(surface.get(), reinterpret_cast<const char*>(utf8Path.c_str())))
                sdlError("SDL_SavePNG");
        }

        void captureScenario(const ReadmeCaptureManifest& manifest,
            const ReadmeCaptureScenario& scenario, const LoadedCapture& loaded)
        {
            std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window(
                SDL_CreateWindow("QUANTUM README capture", manifest.width, manifest.height,
                    SDL_WINDOW_VULKAN | SDL_WINDOW_BORDERLESS), SDL_DestroyWindow);
            if (!window)
                sdlError("SDL_CreateWindow for capture");

            // Reverse destruction order: ImGui, Vulkan (including surface), then SDL window.
            renderer::VulkanContext vulkan;
            vulkan.initialize(window.get(), loaded.centerline.vertices,
                loaded.centerline.verticesPerCurve,
                loaded.centerline.renderableTrack, true);
            EditorUi ui;
            ui.initialize(window.get(), vulkan, loaded.track,
                loaded.centerline.minimumPosition, loaded.centerline.maximumPosition, &scenario);
            ui.setCenterlineSections(loaded.centerline.sectionSlices);
            ui.setCenterlineVisualization(loaded.centerline);
            ui.setRiderLoadHistory(loaded.loads);

            int stableFrames = 0;
            auto generation = vulkan.swapchainGeneration();
            VkExtent2D previousViewport{};
            renderer::FrameImage image;
            // Bounded retries allow initial layout/font uploads and swapchain recreation,
            // without depending on wall-clock sleeps or capturing a partial first frame.
            for (int frame = 0; frame < manifest.settleFrames + 240; ++frame)
            {
                SDL_Event event{};
                while (SDL_PollEvent(&event))
                {
                    if (event.type == SDL_EVENT_QUIT
                        || event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED)
                        throw std::runtime_error("Screenshot capture cancelled before completion.");
                    // Deliberately do not forward input to the read-only capture UI.
                }
                int width = 0;
                int height = 0;
                if (!SDL_GetWindowSizeInPixels(window.get(), &width, &height))
                    sdlError("SDL_GetWindowSizeInPixels for capture");
                if (width != manifest.width || height != manifest.height)
                    throw std::runtime_error("Capture drawable differs from requested dimensions; "
                        "choose a size supported by the current display.");

                ui.beginFrame(vulkan);
                const auto viewport = vulkan.viewportExtent();
                if (viewport.width == 0 || viewport.height == 0
                    || viewport.width != previousViewport.width || viewport.height != previousViewport.height
                    || generation != vulkan.swapchainGeneration())
                    stableFrames = 0;
                previousViewport = viewport;
                generation = vulkan.swapchainGeneration();
                const bool capture = stableFrames >= manifest.settleFrames;
                vulkan.drawFrame([](VkCommandBuffer command, void* data)
                    { static_cast<EditorUi*>(data)->render(command); }, &ui, capture ? &image : nullptr);
                if (generation != vulkan.swapchainGeneration())
                    stableFrames = 0;
                else
                    ++stableFrames;
                if (!image.pixels.empty())
                    break;
            }
            if (image.pixels.empty())
                throw std::runtime_error("Capture did not reach a stable renderable frame.");
            if (image.width != static_cast<std::uint32_t>(manifest.width)
                || image.height != static_cast<std::uint32_t>(manifest.height))
                throw std::runtime_error("Captured swapchain dimensions differ from the manifest.");
            const auto output = readmeCaptureOutputPath(manifest, scenario);
            savePng(image, output);
            std::printf("Captured %s (%u x %u)\n", output.string().c_str(), image.width, image.height);
        }
    }

    int runReadmeCapture(const ReadmeCaptureManifest& manifest)
    {
        // Validate every supplied document before opening a window or writing any images.
        std::vector<LoadedCapture> loaded;
        loaded.reserve(manifest.scenarios.size());
        for (const auto& scenario : manifest.scenarios)
            loaded.push_back(loadCapture(scenario));
        std::filesystem::create_directories(manifest.outputDirectory);
        if (!SDL_Init(SDL_INIT_VIDEO))
            sdlError("SDL_Init for capture");
        SDL_SetLogPriority(SDL_LOG_CATEGORY_RENDER, SDL_LOG_PRIORITY_WARN);
        try
        {
            for (std::size_t index = 0; index < manifest.scenarios.size(); ++index)
                captureScenario(manifest, manifest.scenarios[index], loaded[index]);
        }
        catch (...)
        {
            SDL_Quit();
            throw;
        }
        SDL_Quit();
        return 0;
    }
}
