#include <quantum/editor/SimulationPreview.hpp>
#include <quantum/editor/RigidBodyMechanismProof.hpp>
#include <quantum/engine/Logging.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <SDL3/SDL.h>

#include <iostream>
#include <memory>
#include <stdexcept>

namespace
{
    using namespace quantum;

    void require(const bool condition, const char* message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    struct LogCounts
    {
        int trainUploads = 0;
        int mechanismUploads = 0;
        int assetErrors = 0;
        int vulkanErrors = 0;
        int frames = 0;
    };

    void countLog(const logging::LogLevel level, const std::string_view category,
        const std::string_view message, void* data) noexcept
    {
        auto& counts = *static_cast<LogCounts*>(data);
        if (category == "ASSET" && message.starts_with("Uploaded static mesh once:"))
        {
            if (message.find(editor::placeholderTrainCarAssetId) != std::string_view::npos)
                ++counts.trainUploads;
            else
                ++counts.mechanismUploads;
        }
        if (category == "ASSET" && level == logging::LogLevel::Error)
            ++counts.assetErrors;
        if (category.starts_with("VK") && level == logging::LogLevel::Error)
            ++counts.vulkanErrors;
    }

    void rendererLifecycle(LogCounts& counts)
    {
        std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window{
            SDL_CreateWindow("Train Visual M0 regression", 128, 128,
                SDL_WINDOW_HIDDEN | SDL_WINDOW_VULKAN), &SDL_DestroyWindow};
        if (!window)
            throw std::runtime_error(std::string{"SDL_CreateWindow: "} + SDL_GetError());
        renderer::VulkanContext renderer;
        coaster::RenderableTrack geometry;
        geometry.continuousMesh.vertices = {
            {{0, 0, 0}, {0, 0, 1}}, {{1, 0, 0}, {0, 0, 1}}, {{0, 1, 0}, {0, 0, 1}}};
        geometry.continuousMesh.triangleIndices = {0, 1, 2};
        geometry.continuousMesh.edgeIndices = {0, 1, 1, 2, 2, 0};
        geometry.continuousMesh.submeshes = {{0, 3, 0}};
        geometry.materials.emplace_back();
        renderer.initialize(window.get(), {}, 0, geometry, false);
        renderer.resizeViewportTarget(128, 128, nullptr, nullptr, false);
        auto track = coaster::createNewDocument();
        coaster::setSectionLength(track.section(0), 100.0);
        editor::RigidBodyMechanismProof mechanism;
        editor::SimulationPreview preview;
        preview.setRigidBodyWorld(&mechanism.world());
        require(preview.rebuild(track), "A usable specialized train must coexist with the mechanism.");
        std::vector<renderer::StaticMeshInstance> combined;
        const auto draw = [&] { renderer.drawFrame(); ++counts.frames; };
        const auto publish = [&](const bool train, const bool proof) {
            const auto cars = preview.meshInstances();
            if (train)
                combined.assign(cars.begin(), cars.end());
            else
                combined.clear();
            if (proof)
            {
                const auto parts = mechanism.meshInstances();
                combined.insert(combined.end(), parts.begin(), parts.end());
            }
            renderer.updateDynamicMeshInstances(combined);
        };
        for (int cycle = 0; cycle < 20; ++cycle)
        {
            preview.reset();
            preview.play();
            for (const auto [train, proof] : {std::pair{true, false}, std::pair{true, true},
                std::pair{false, true}, std::pair{false, false}, std::pair{true, true}})
            {
                preview.update(1.5 * physics::defaultFixedTimeStepSeconds);
                publish(train, proof);
                require(combined.size() == (train ? 4 : 0) + (proof ? 2 : 0),
                    "Train/proof transitions must publish all active instances.");
                if (train)
                {
                    const auto status = renderer.dynamicMeshAssetLoadStatus(editor::placeholderTrainCarAssetId);
                    require(status && status->state == renderer::HardwareAssetLoadState::Loaded,
                        "Four shells must share the loaded placeholder.");
                }
                if (proof)
                {
                    for (const auto& binding : mechanism.meshBindings())
                    {
                        const auto status = renderer.dynamicMeshAssetLoadStatus(binding.assetIdentifier);
                        require(status && !status->usingDiagnosticFallback,
                            "Both mechanism assets must coexist with train shells.");
                    }
                }
                draw();
            }
            preview.pause();
            const auto tick = preview.dynamicsState()->tick;
            const auto matrix = preview.meshInstances().front().transform;
            preview.update(0.1);
            require(preview.dynamicsState()->tick == tick
                && preview.meshInstances().front().transform == matrix,
                "Pause must freeze the submitted shells and specialized physics.");
            draw();
        }
        for (const auto identifier : {"assets://train/missing-m0-test.glb",
            "assets://train/invalid-m0-test.glb"})
        {
            preview.play();
            const auto tick = preview.dynamicsState()->tick;
            for (int frame = 0; frame < 8; ++frame)
            {
                preview.update(1.5 * physics::defaultFixedTimeStepSeconds);
                const auto cars = preview.meshInstances();
                combined.assign(cars.begin(), cars.end());
                for (auto& instance : combined)
                    instance.assetIdentifier = identifier;
                const auto parts = mechanism.meshInstances();
                combined.insert(combined.end(), parts.begin(), parts.end());
                renderer.updateDynamicMeshInstances(combined);
                const auto status = renderer.dynamicMeshAssetLoadStatus(identifier);
                require(status && status->usingDiagnosticFallback && !status->detail.empty(),
                    "Failed shells require explicit asset errors and diagnostic fallback.");
                require(status->state == (std::string_view{identifier}.find("missing") != std::string_view::npos
                    ? renderer::HardwareAssetLoadState::MissingAsset : renderer::HardwareAssetLoadState::InvalidGlb),
                    "Missing and invalid assets must be classified correctly.");
                preview.setPhysicsDiagnosticsVisible(status->usingDiagnosticFallback);
                require(!preview.vertices().empty() && preview.meshInstances().size() == 4,
                    "Asset failure restores train body/bogie/connector diagnostics.");
                renderer.updateTrainPreviewVertices(preview.vertices());
                const auto healthy = renderer.dynamicMeshAssetLoadStatus(editor::mechanicalGondolaAssetId);
                require(healthy && !healthy->usingDiagnosticFallback,
                    "Failed shells must not hide healthy mechanism instances.");
                draw();
                // Re-enable after a collection reset: failure identity still
                // suppresses duplicate I/O/logging, even for four repeated cars.
                renderer.updateDynamicMeshInstances({});
                draw();
            }
            require(preview.isAvailable() && preview.dynamicsState()->tick > tick,
                "Missing/invalid GLBs must not stop or replace specialized train physics.");
        }
        coaster::setSectionLength(track.section(0), 1.0);
        require(!preview.rebuild(track),
            "A one-meter document has no legal four-car placement.");
        publish(true, true);
        require(combined.size() == 2, "Unavailable preview removes shells while retaining an active proof.");
        draw();
        preview.setRigidBodyWorld(nullptr);
        renderer.updateDynamicMeshInstances({});
        draw();
        renderer.shutdown();
    }
}

int main()
{
    LogCounts counts;
    logging::setLogSinkForTesting(countLog, &counts);
    try
    {
        if (!SDL_Init(SDL_INIT_VIDEO))
            throw std::runtime_error(std::string{"SDL_Init: "} + SDL_GetError());
        rendererLifecycle(counts);
        SDL_Quit();
        require(counts.trainUploads == 1 && counts.mechanismUploads == 2,
            "Four shells upload once; arm/carrier each upload once across all transitions.");
        require(counts.assetErrors == 2, "Missing/invalid shells each log once across four cars and re-enable.");
        require(counts.vulkanErrors == 0, "Train/proof frame-buffer lifecycle must have no Vulkan errors.");
        logging::resetLogSinkForTesting();
        std::cout << "Train Visual M0 Vulkan: " << counts.frames
            << " frames, 20 collection cycles, 1 train + 2 mechanism uploads, 2 retained asset errors, 0 Vulkan errors.\n";
        return 0;
    }
    catch (const std::exception& exception)
    {
        SDL_Quit();
        logging::resetLogSinkForTesting();
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
