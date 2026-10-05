#include <quantum/editor/SimulationPreview.hpp>
#include <quantum/editor/RigidBodyMechanismProof.hpp>
#include <quantum/engine/Logging.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <SDL3/SDL.h>

#include <array>
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
        std::array<int, 3> perTrainAssetUploads{};
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
            if (message.find("assets://train/") != std::string_view::npos)
            {
                ++counts.trainUploads;
                const std::array identifiers{editor::placeholderTrainLeadAssetId,
                    editor::placeholderTrainCarAssetId, editor::placeholderTrainRearAssetId};
                for (std::size_t index = 0; index < identifiers.size(); ++index)
                    if (message.find(identifiers[index]) != std::string_view::npos)
                        ++counts.perTrainAssetUploads[index];
            }
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
            SDL_CreateWindow("Train Visual M0/M1 regression", 128, 128,
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
        // Retain M0's four copies/one upload proof before mixed identities.
        const auto initialCars = preview.meshInstances();
        combined.assign(initialCars.begin(), initialCars.end());
        for (auto& instance : combined)
            instance.assetIdentifier = editor::placeholderTrainCarAssetId;
        renderer.updateDynamicMeshInstances(combined);
        require(counts.trainUploads == 1, "M0's four repeated shells must still upload one mesh.");
        draw();
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
                    for (const auto& instance : preview.meshInstances())
                    {
                        const auto status = renderer.dynamicMeshAssetLoadStatus(instance.assetIdentifier);
                        require(status && status->state == renderer::HardwareAssetLoadState::Loaded,
                            "Every heterogeneous train identity must be loaded.");
                    }
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
        // Count transitions keep the same three meshes while resizing poses.
        for (const std::size_t count : {4, 2, 6, 1, 4})
        {
            auto setup = track.coasterSetup();
            setup.carsPerTrain = count;
            track.setCoasterSetup(setup);
            require(preview.rebuild(track), "Changed consist count must rebuild.");
            publish(true, true);
            require(combined.size() == count + 2, "Count changes must retain mechanism instances.");
            draw();
        }
        for (const int failedRole : {0, 1, 2})
        {
            const auto identifier = failedRole == 0 ? "assets://train/missing-lead-m1-test.glb"
                : failedRole == 1 ? "assets://train/invalid-m0-test.glb"
                : "assets://train/missing-rear-m1-test.glb";
            preview.play();
            const auto tick = preview.dynamicsState()->tick;
            for (int frame = 0; frame < 8; ++frame)
            {
                preview.update(1.5 * physics::defaultFixedTimeStepSeconds);
                publish(true, true);
                for (std::size_t index = 0; index < 4; ++index)
                    if ((failedRole == 0 && index == 0)
                        || (failedRole == 1 && (index == 1 || index == 2))
                        || (failedRole == 2 && index == 3))
                        combined[index].assetIdentifier = identifier;
                renderer.updateDynamicMeshInstances(combined);
                const auto failed = renderer.dynamicMeshAssetLoadStatus(identifier);
                require(failed && failed->usingDiagnosticFallback && !failed->detail.empty(),
                    "Failed lead/shared middle/rear must request diagnostics.");
                require(failed->state == (failedRole == 1 ? renderer::HardwareAssetLoadState::InvalidGlb
                    : renderer::HardwareAssetLoadState::MissingAsset), "M1 failure classification must be precise.");
                for (std::size_t index = 0; index < combined.size(); ++index)
                {
                    if (combined[index].assetIdentifier == identifier)
                        continue;
                    const auto healthy = renderer.dynamicMeshAssetLoadStatus(combined[index].assetIdentifier);
                    require(healthy && !healthy->usingDiagnosticFallback,
                        "Healthy train and mechanism siblings must remain loaded beside failed shells.");
                    if (index < 4)
                        require(combined[index].transform == preview.meshInstances()[index].transform,
                            "Failed siblings must not shift healthy transform indices.");
                }
                preview.setPhysicsDiagnosticsVisible(true);
                require(!preview.vertices().empty(), "A shell failure must leave physics diagnostics available.");
                renderer.updateTrainPreviewVertices(preview.vertices());
                draw();
                renderer.updateDynamicMeshInstances({});
                draw();
            }
            require(preview.isAvailable() && preview.dynamicsState()->tick > tick,
                "Partial visual failure must leave specialized physics stepping.");
        }
        coaster::setSectionLength(track.section(0), 1.0);
        require(!preview.rebuild(track),
            "A one-meter document has no legal four-car placement.");
        publish(true, true);
        require(combined.size() == 2, "Unavailable preview removes shells while retaining an active proof.");
        draw();
        coaster::setSectionLength(track.section(0), 100.0);
        require(preview.rebuild(track), "Recovered preview must restore the heterogeneous consist.");
        publish(true, true);
        require(combined.size() == 6, "Recovery must restore train and mechanism publication.");
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
        require(counts.trainUploads == 3 && counts.perTrainAssetUploads == std::array{1, 1, 1}
            && counts.mechanismUploads == 2,
            "Lead, shared middle, rear and arm/carrier must each upload once across all transitions.");
        require(counts.assetErrors == 4,
            "M0 missing/invalid and M1 missing lead/rear each log once; repeated invalid middle reuses M0 failure.");
        require(counts.vulkanErrors == 0, "Train/proof frame-buffer lifecycle must have no Vulkan errors.");
        logging::resetLogSinkForTesting();
        std::cout << "Train Visual M0/M1 Vulkan: " << counts.frames
            << " frames, 20 collection cycles, 3 train + 2 mechanism uploads, 4 retained asset errors, "
               "0 Vulkan errors; mixed failures, count changes and recovery passed.\n";
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
