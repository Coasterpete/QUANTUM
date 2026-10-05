#include <quantum/engine/Logging.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <SDL3/SDL.h>
#include <glm/gtc/matrix_transform.hpp>

#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

namespace
{
    void require(const bool condition, const char* message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    struct LogCounts
    {
        int meshUploads = 0;
        int validationErrors = 0;
        int assetErrors = 0;
    };

    void countLog(const quantum::logging::LogLevel level, const std::string_view category,
        const std::string_view message, void* data) noexcept
    {
        auto& counts = *static_cast<LogCounts*>(data);
        if (category == "ASSET" && message.starts_with("Uploaded static mesh once:"))
            ++counts.meshUploads;
        if (category == "ASSET" && level == quantum::logging::LogLevel::Error)
            ++counts.assetErrors;
        if (category.starts_with("VK") && level == quantum::logging::LogLevel::Error)
            ++counts.validationErrors;
    }

    void rendererLifecycle()
    {
        using namespace quantum::renderer;
        std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window{
            SDL_CreateWindow("M4 renderer regression", 128, 128,
                SDL_WINDOW_HIDDEN | SDL_WINDOW_VULKAN), &SDL_DestroyWindow};
        if (!window)
            throw std::runtime_error(std::string{"SDL_CreateWindow: "} + SDL_GetError());
        VulkanContext renderer;
        quantum::coaster::RenderableTrack track;
        track.continuousMesh.vertices = {
            {{0, 0, 0}, {0, 0, 1}}, {{1, 0, 0}, {0, 0, 1}}, {{0, 1, 0}, {0, 0, 1}}};
        track.continuousMesh.triangleIndices = {0, 1, 2};
        track.continuousMesh.edgeIndices = {0, 1, 1, 2, 2, 0};
        track.continuousMesh.submeshes = {{0, 3, 0}};
        track.materials.emplace_back();
        renderer.initialize(window.get(), {}, 0, track, false);
        renderer.resizeViewportTarget(128, 128, nullptr, nullptr, false);
        StaticMeshInstance instance{"assets://mechanical/rotating-arm-placeholder.glb"};
        for (int reset = 0; reset < 20; ++reset)
        {
            renderer.updateDynamicMeshInstance(instance);
            const auto status = renderer.dynamicMeshAssetLoadStatus();
            require(status && status->state == HardwareAssetLoadState::Loaded
                && !status->usingDiagnosticFallback, "Actual mechanical GLB must load.");
            for (int frame = 0; frame < 6; ++frame)
            {
                // Fixed test pose exercises repeated publication and buffer
                // reuse. Actual Jolt motion is tested by RigidBodyMeshBinding.
                instance.transform = glm::translate(glm::mat4{1}, glm::vec3{0, 0, 2});
                renderer.updateDynamicMeshInstance(instance);
                renderer.drawFrame();
            }
            renderer.updateDynamicMeshInstance(std::nullopt);
            require(!renderer.dynamicMeshAssetLoadStatus(), "Disable must clear active asset status.");
            renderer.drawFrame();
            renderer.drawFrame();
        }
        // Grow/shrink through 1/2/3 transforms while frames are in flight.
        // The third instance shares the arm GLB and must reuse its geometry.
        std::array<StaticMeshInstance, 3> assembly{{
            {"assets://mechanical/rotating-arm-placeholder.glb"},
            {"assets://mechanical/hanging-carrier-placeholder.glb"},
            {"assets://mechanical/rotating-arm-placeholder.glb"}}};
        for (int reset = 0; reset < 20; ++reset)
        {
            for (const std::size_t count : {1, 2, 3, 2})
            {
                assembly[0].transform = glm::translate(glm::mat4{1}, glm::vec3{-2, 0, 3});
                assembly[1].transform = glm::translate(glm::mat4{1}, glm::vec3{2, 0, 3})
                    * glm::rotate(glm::mat4{1}, 0.1F * reset, glm::vec3{0, 1, 0});
                assembly[2].transform = glm::translate(glm::mat4{1}, glm::vec3{-2, 1, 5});
                renderer.updateDynamicMeshInstances(std::span{assembly}.first(count));
                for (std::size_t index = 0; index < count; ++index)
                {
                    const auto status = renderer.dynamicMeshAssetLoadStatus(assembly[index].assetIdentifier);
                    require(status && status->state == HardwareAssetLoadState::Loaded
                        && !status->usingDiagnosticFallback, "Every independently placed GLB must load.");
                }
                renderer.drawFrame();
                renderer.drawFrame();
            }
            renderer.updateDynamicMeshInstances({});
            require(!renderer.dynamicMeshAssetLoadStatus(assembly[1].assetIdentifier),
                "Disable must clear every active instance status.");
            renderer.drawFrame();
        }
        auto invalid = instance;
        invalid.transform[0][0] = std::numeric_limits<float>::quiet_NaN();
        bool rejected = false;
        try { renderer.updateDynamicMeshInstance(invalid); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Nonfinite transform must be rejected before publication.");
        instance.assetIdentifier = "assets://mechanical/missing-m4-test.glb";
        renderer.updateDynamicMeshInstance(instance);
        const auto missing = renderer.dynamicMeshAssetLoadStatus();
        require(missing && missing->requestedIdentifier == instance.assetIdentifier
            && missing->state == HardwareAssetLoadState::MissingAsset
            && missing->usingDiagnosticFallback && !missing->detail.empty(),
            "Missing asset must retain explicit failure and request physics bounds.");
        for (int frame = 0; frame < 4; ++frame)
        {
            renderer.updateDynamicMeshInstance(instance);
            renderer.drawFrame();
        }
        // Failure in the first slot must not suppress a good second mesh or
        // shift its transform index. Repeat poses must retain the failure result.
        assembly[0] = instance;
        for (int frame = 0; frame < 4; ++frame)
        {
            renderer.updateDynamicMeshInstances(std::span{assembly}.first(2));
            const auto good = renderer.dynamicMeshAssetLoadStatus(assembly[1].assetIdentifier);
            require(good && !good->usingDiagnosticFallback, "One failed asset must not hide a healthy sibling.");
            renderer.drawFrame();
        }
        renderer.updateDynamicMeshInstance(std::nullopt);
        instance.assetIdentifier = "assets://mechanical/invalid-m4-test.glb";
        renderer.updateDynamicMeshInstance(instance);
        const auto invalidGlb = renderer.dynamicMeshAssetLoadStatus();
        require(invalidGlb && invalidGlb->state == HardwareAssetLoadState::InvalidGlb
            && invalidGlb->usingDiagnosticFallback, "Invalid GLB must request explicit fallback.");
        renderer.drawFrame();
        renderer.updateDynamicMeshInstance(std::nullopt);
        renderer.drawFrame();
        // An empty requested ID is still a retained failure, not an uninitialized
        // entry that should be retried/logged on every pose update.
        instance.assetIdentifier.clear();
        for (int frame = 0; frame < 4; ++frame)
        {
            renderer.updateDynamicMeshInstance(instance);
            const auto empty = renderer.dynamicMeshAssetLoadStatus();
            require(empty && empty->usingDiagnosticFallback && !empty->detail.empty(),
                "Empty asset identity must report a retained explicit failure.");
            renderer.drawFrame();
        }
        renderer.updateDynamicMeshInstances({});
        renderer.drawFrame();
        renderer.shutdown();
    }
}

int main()
{
    LogCounts counts;
    quantum::logging::setLogSinkForTesting(countLog, &counts);
    try
    {
        if (!SDL_Init(SDL_INIT_VIDEO))
            throw std::runtime_error(std::string{"SDL_Init: "} + SDL_GetError());
        rendererLifecycle();
        SDL_Quit();
        require(counts.meshUploads == 2, "Two assets/shared instances must each upload immutable geometry only once.");
        require(counts.assetErrors == 3, "Missing/invalid/empty asset must each log once across pose updates.");
        require(counts.validationErrors == 0, "Dynamic mesh lifetime must produce no Vulkan errors.");
        quantum::logging::resetLogSinkForTesting();
        std::cout << "Dynamic mesh renderer: 20 single + 20 multi cycles, 355 frames, two mesh uploads; growth/shrink/shared mesh and sibling/empty-ID fallback passed.\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        SDL_Quit();
        quantum::logging::resetLogSinkForTesting();
        std::cerr << error.what() << '\n';
        return 1;
    }
}
