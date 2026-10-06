#include <quantum/coaster/TrainConfiguration.hpp>
#include <quantum/editor/SimulationPreview.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace
{
    using namespace quantum;

    void require(const bool condition, const char* message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    void near(const glm::dvec3& actual, const glm::dvec3& expected)
    {
        require(std::isfinite(glm::length(actual - expected))
            && glm::length(actual - expected) < 1e-5,
            "Per-car visual points must follow their own physical pose/local adjustment.");
    }

    void requireOrder(const std::span<const renderer::StaticMeshInstance> instances)
    {
        for (std::size_t index = 0; index < instances.size(); ++index)
        {
            const auto expected = index == 0 ? editor::placeholderTrainLeadAssetId
                : index + 1 == instances.size() ? editor::placeholderTrainRearAssetId
                : editor::placeholderTrainCarAssetId;
            require(instances[index].assetIdentifier == expected,
                "Temporary visual defaults must retain lead/middle(s)/rear order.");
        }
    }

    void mappingAndCache(const std::filesystem::path& root)
    {
        renderer::StaticMeshAssetCache cache{root};
        renderer::StaticMeshGpuHandleCache gpuCache;
        std::array<int, 3> uploads{};
        std::shared_ptr<const renderer::StaticMeshAsset> middle;
        for (const std::size_t count : {0, 1, 2, 4, 8})
        {
            auto visuals = editor::makeDefaultTrainVisualConsist(count);
            require(visuals.cars.size() == count, "Default generator must produce exactly N entries.");
            if (count == 0)
                continue;
            auto configuration = coaster::createDefaultTrainConfiguration();
            configuration.carCount = count;
            const auto train = coaster::resolveTrainConfiguration(configuration);
            const auto forward = glm::normalize(glm::dvec3{1, 0.3, 0.4});
            const auto lateral = glm::normalize(glm::cross(glm::dvec3{0, 0, 1}, forward));
            const auto frame = geometry::applyRoll({forward, lateral, glm::cross(forward, lateral)}, 0.55);
            const physics::CompiledPhysicsTrack track{
                std::vector<coaster::TrackKinematicState>{
                    {0.0, {3, -5, 7}, frame, {}},
                    {200.0, glm::dvec3{3, -5, 7} + 200.0 * forward, frame, {}}},
                1.0, coaster::TopologyKind::OpenLinear};
            std::vector<renderer::StaticMeshInstance> instances;
            for (const auto direction : {physics::TravelDirection::IncreasingStation,
                physics::TravelDirection::DecreasingStation})
            {
                const auto pose = physics::solveTrainPose(track, train,
                    {physics::primaryTrackPathId, 80.0, direction});
                editor::updateTrainMeshInstances(instances, pose, visuals.cars, 0.5);
                requireOrder(instances);
                for (std::size_t index = 0; index < count; ++index)
                {
                    const auto asset = cache.load(instances[index].assetIdentifier);
                    const int identity = index == 0 ? 0 : index + 1 == count ? 2 : 1;
                    if (identity == 1)
                    {
                        if (middle)
                            require(middle == asset, "Repeated middle entries must share immutable CPU geometry.");
                        middle = asset;
                    }
                    const auto handle = gpuCache.getOrUpload(*asset,
                        [&](const renderer::StaticMeshAsset&) {
                            ++uploads[identity];
                            return renderer::StaticMeshGpuHandle{static_cast<std::uint32_t>(identity)};
                        });
                    require(handle.value == identity, "Each identity must retain its own geometry handle.");
                    require(asset->submeshes.size() == 3, "Each shell must retain untextured PBR finishes.");
                    for (const auto& vertex : asset->vertices)
                        near(glm::dvec3{glm::dmat4{instances[index].transform}
                            * glm::dvec4{vertex.position, 1.0}},
                            0.5 * pose.cars()[index].carPose().transformLocalPoint(vertex.position));
                }
                auto adjusted = visuals;
                for (std::size_t index = 0; index < count; ++index)
                {
                    adjusted.cars[index].localAssetTransform =
                        glm::translate(glm::dmat4{1.0}, glm::dvec3{0.1 * index, -0.2 * index, 0.3})
                        * glm::rotate(glm::dmat4{1.0}, 0.2 * index, glm::dvec3{0, 0, 1});
                    adjusted.cars[index].assetIdentifier = "per-car-" + std::to_string(index);
                }
                editor::updateTrainMeshInstances(instances, pose, adjusted.cars, 2.0);
                const glm::dvec4 local{0.7, -0.3, 0.5, 1.0};
                for (std::size_t index = 0; index < count; ++index)
                {
                    require(instances[index].assetIdentifier == adjusted.cars[index].assetIdentifier,
                        "The mapping must accept arbitrary per-car identities, independent of roles.");
                    near(glm::dvec3{glm::dmat4{instances[index].transform} * local},
                        2.0 * pose.cars()[index].carPose().transformLocalPoint(
                            glm::dvec3{adjusted.cars[index].localAssetTransform * local}));
                }
                const auto before = instances;
                for (const std::size_t badCount : {count - 1, count + 1})
                {
                    auto malformed = visuals;
                    malformed.cars.resize(badCount);
                    bool rejected = false;
                    try { editor::updateTrainMeshInstances(instances, pose, malformed.cars, 1.0); }
                    catch (const std::invalid_argument& error)
                    {
                        rejected = std::string_view{error.what()}.find("render-pose cars") != std::string_view::npos;
                    }
                    require(rejected && instances.size() == before.size(), "Count mismatch must reject clearly.");
                    for (std::size_t index = 0; index < count; ++index)
                        require(instances[index].assetIdentifier == before[index].assetIdentifier
                            && instances[index].transform == before[index].transform,
                            "Rejected visual inputs must leave the previous output unchanged.");
                }
            }
        }
        require(cache.size() == 3 && uploads == std::array{1, 1, 1},
            "Lead, shared middle and rear must each load/upload once across counts and motion.");
        std::cout << "M1 cache: 3 immutable CPU entries, 3 upload callbacks (one per identity).\n";
    }

    void countChanges()
    {
        auto track = coaster::createNewDocument();
        coaster::setSectionLength(track.section(0), 200.0);
        editor::SimulationPreview preview;
        for (const std::size_t count : {4, 2, 6, 1})
        {
            auto setup = track.coasterSetup();
            setup.carsPerTrain = count;
            track.setCoasterSetup(setup);
            require(preview.rebuild(track) && preview.meshInstances().size() == count,
                "Rebuild must replace the visual count without stale entries.");
            requireOrder(preview.meshInstances());
            const auto initial = preview.meshInstances().front().transform;
            preview.play();
            preview.update(1.5 * physics::defaultFixedTimeStepSeconds);
            require(preview.dynamicsState()->tick == 1 && preview.interpolationAlpha() > 0.49,
                "Heterogeneous visuals must retain existing fractional-tick interpolation.");
            requireOrder(preview.meshInstances());
            preview.pause();
            const auto paused = preview.meshInstances().front().transform;
            preview.update(0.1);
            require(preview.meshInstances().front().transform == paused, "Pause must freeze visual transforms.");
            preview.reset();
            require(preview.meshInstances().front().transform == initial, "Reset must restore visual transforms.");
            requireOrder(preview.meshInstances());
        }
        coaster::setSectionLength(track.section(0), 1.0);
        require(!preview.rebuild(track) && preview.meshInstances().empty(), "Unavailable previews must clear visuals.");
        coaster::setSectionLength(track.section(0), 200.0);
        require(preview.rebuild(track) && preview.meshInstances().size() == 1,
            "Recovered preview must regenerate its current visual consist.");
        requireOrder(preview.meshInstances());
    }
}

int main(int argc, char** argv)
{
    try
    {
        require(argc == 2, "Provide repository asset root.");
        mappingAndCache(argv[1]);
        countChanges();
        std::cout << "Train Visual M1: ordered defaults, arbitrary mapping/local transforms, mismatch rejection, "
            "pitched/rolled/reverse alignment and 4->2->6->1 lifecycle passed.\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
