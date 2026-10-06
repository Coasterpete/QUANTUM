#include <quantum/coaster/TrainConfiguration.hpp>
#include <quantum/editor/SimulationPreview.hpp>
#include <quantum/editor/RigidBodyMechanismProof.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <filesystem>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>

namespace
{
    using namespace quantum;

    void require(const bool condition, const char* message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    void near(const glm::dvec3& actual, const glm::dvec3& expected, const char* message)
    {
        require(std::isfinite(glm::length(actual - expected))
            && glm::length(actual - expected) < 1e-5, message);
    }

    glm::dvec3 point(const renderer::StaticMeshInstance& instance, const glm::dvec3& local)
    {
        return glm::dvec3{glm::dmat4{instance.transform} * glm::dvec4{local, 1.0}};
    }

    coaster::AuthoredTrack authoredStraight(const double scale = 1.0,
        const double speed = 20.0, const double length = 100.0)
    {
        auto track = coaster::createNewDocument();
        coaster::setSectionLength(track.section(0), length);
        auto settings = track.physicalSettings();
        settings.metersPerCoordinateUnit = scale;
        settings.initialSpeed = speed;
        track.setPhysicalSettings(settings);
        return track;
    }

    physics::CompiledPhysicsTrack straight(const glm::dvec3& tangent, const double roll)
    {
        const auto forward = glm::normalize(tangent);
        const auto lateral = glm::normalize(glm::cross(glm::dvec3{0, 0, 1}, forward));
        const auto frame = geometry::applyRoll(
            {forward, lateral, glm::cross(forward, lateral)}, roll);
        const std::vector<coaster::TrackKinematicState> samples{
            {0.0, {3, -5, 7}, frame, {}},
            {100.0, glm::dvec3{3, -5, 7} + 100.0 * forward, frame, {}}};
        return {samples, 1.0, coaster::TopologyKind::OpenLinear};
    }

    void transformsAndAsset(const std::filesystem::path& root)
    {
        const auto train = coaster::resolveTrainConfiguration(
            coaster::createDefaultTrainConfiguration());
        renderer::StaticMeshAssetCache cache{root};
        renderer::StaticMeshGpuHandleCache gpuCache;
        std::shared_ptr<const renderer::StaticMeshAsset> shared;
        int uploads = 0;
        for (int index = 0; index < 4; ++index)
        {
            const auto asset = cache.load(editor::placeholderTrainCarAssetId);
            if (shared)
                require(shared == asset, "Four cars must share one immutable CPU mesh.");
            shared = asset;
            const auto handle = gpuCache.getOrUpload(*asset,
                [&](const renderer::StaticMeshAsset&) {
                    ++uploads;
                    return renderer::StaticMeshGpuHandle{0};
                });
            require(handle.value == 0, "All cars must share one geometry handle.");
        }
        require(cache.size() == 1 && uploads == 1, "Repeated cars must load/upload once.");
        require(shared->submeshes.size() == 3, "Placeholder retains its three PBR finishes.");
        glm::dvec3 minimum{1e10}, maximum{-1e10};
        for (const auto& vertex : shared->vertices)
        {
            minimum = glm::min(minimum, glm::dvec3{vertex.position});
            maximum = glm::max(maximum, glm::dvec3{vertex.position});
        }
        near(minimum, {-2, -0.675, -0.7}, "Asset lower bounds must match the physical body origin.");
        near(maximum, {2, 0.675, 0.7}, "Asset upper bounds must match the default physical body.");

        std::vector<renderer::StaticMeshInstance> instances;
        for (const glm::dvec3 tangent : {glm::dvec3{1, 0, 0},
            glm::dvec3{1, 1, 0}, glm::dvec3{1, 0, 0.5}})
        for (const double roll : {0.0, 0.55})
        for (const auto direction : {physics::TravelDirection::IncreasingStation,
            physics::TravelDirection::DecreasingStation})
        {
            const auto track = straight(tangent, roll);
            const auto pose = physics::solveTrainPose(track, train,
                {physics::primaryTrackPathId, 35.0, direction});
            std::vector<editor::TrainVisualPrototype> visual(pose.carCount());
            editor::updateTrainMeshInstances(instances, pose, visual, 1.0);
            require(instances.size() == 4, "Every solved car needs a separate transform.");
            const auto* storage = instances.data();
            for (std::size_t index = 0; index < instances.size(); ++index)
            {
                const auto& car = pose.cars()[index].carPose();
                require(pose.cars()[index].carIndex() == index,
                    "Instance order must remain lead to rear.");
                near(point(instances[index], {}), car.bodyWorldPositionMeters(),
                    "Asset origin must be the body origin, not the loaded COG.");
                const glm::dmat3 rotation{instances[index].transform};
                near(rotation[0], car.bodyFrame().tangent, "+X must follow physical forward.");
                near(rotation[1], car.bodyFrame().lateral, "+Y must follow physical lateral.");
                near(rotation[2], car.bodyFrame().up, "+Z must follow physical up/bank.");
                for (const auto& vertex : shared->vertices)
                    near(point(instances[index], vertex.position),
                        car.transformLocalPoint(vertex.position),
                        "Imported shell must follow the entire physical frame, including reverse facing.");
                if (index > 0)
                    require(glm::length(point(instances[index], {}) - point(instances[index - 1], {})) > 1.0,
                        "Different cars must not reuse the lead car's transform.");
            }
            const auto adjustment = glm::translate(glm::dmat4{1.0}, glm::dvec3{0.2, -0.1, 0.3})
                * glm::rotate(glm::dmat4{1.0}, 0.4, glm::dvec3{0, 0, 1});
            for (auto& prototype : visual)
                prototype.localAssetTransform = adjustment;
            editor::updateTrainMeshInstances(instances, pose, visual, 0.5);
            require(instances.data() == storage, "Motion should reuse instance storage.");
            const glm::dvec3 local{0.7, -0.3, 0.5};
            const glm::dvec3 adjusted{adjustment * glm::dvec4{local, 1.0}};
            for (std::size_t index = 0; index < instances.size(); ++index)
                near(point(instances[index], local),
                    0.5 * pose.cars()[index].carPose().transformLocalPoint(adjusted),
                    "Local adjustment acts before physical pose and document-unit conversion.");
        }
        std::vector<coaster::TrackKinematicState> curveSamples;
        constexpr double radius = 30.0;
        for (int index = 0; index <= 360; ++index)
        {
            const double angle = 2.0 * std::numbers::pi * index / 360.0;
            const auto frame = geometry::applyRoll(
                {{std::cos(angle), std::sin(angle), 0},
                 {-std::sin(angle), std::cos(angle), 0}, {0, 0, 1}},
                0.4 * std::sin(2.0 * angle));
            curveSamples.push_back({radius * angle,
                {radius * std::sin(angle), radius * (1.0 - std::cos(angle)), 0}, frame,
                {-std::sin(angle) / radius, std::cos(angle) / radius, 0}});
        }
        const physics::CompiledPhysicsTrack curve{curveSamples, 1.0, coaster::TopologyKind::ClosedCircuit};
        for (const auto direction : {physics::TravelDirection::IncreasingStation,
            physics::TravelDirection::DecreasingStation})
        {
            const auto previous = physics::solveTrainPose(curve, train,
                {physics::primaryTrackPathId, 25.0, direction});
            const auto current = physics::solveTrainPose(curve, train,
                {physics::primaryTrackPathId, 25.2, direction});
            const auto rendered = editor::interpolateTrainPreviewPose(curve, train, previous, current, 0.5);
            const std::vector<editor::TrainVisualPrototype> visual(rendered.carCount());
            editor::updateTrainMeshInstances(instances, rendered, visual, 1.0);
            for (std::size_t index = 0; index < instances.size(); ++index)
                for (const auto& vertex : shared->vertices)
                    near(point(instances[index], vertex.position),
                        rendered.cars()[index].carPose().transformLocalPoint(vertex.position),
                        "Every shell vertex must stay aligned on interpolated curved/banked track.");
        }
        const physics::TrainPose empty{{}, {}, {}, 0.0, {}, 0.0};
        editor::updateTrainMeshInstances(instances, empty, {}, 1.0);
        require(instances.empty(), "An empty presentation pose must produce no stale instances.");
        std::cout << "Placeholder: " << shared->vertices.size() << " vertices, "
            << shared->triangleIndices.size() << " indices; one cached mesh/upload callback.\n";
    }

    void requirePresentation(const editor::SimulationPreview& preview, const double units)
    {
        require(preview.renderPose() && preview.meshInstances().size() == preview.renderPose()->carCount(),
            "Meshes must consume the current render pose.");
        const auto instances = preview.meshInstances();
        for (std::size_t index = 0; index < instances.size(); ++index)
        {
            const auto& car = preview.renderPose()->cars()[index].carPose();
            near(point(instances[index], {}), units * car.bodyWorldPositionMeters(),
                "Mesh must use interpolated body position.");
            if (!preview.vertices().empty())
            {
                const auto& vertex = preview.vertices()[index * 36];
                near(point(instances[index], {-2, -0.675, -0.7}), {vertex.x, vertex.y, vertex.z},
                    "Mesh and diagnostic box must use exactly the same presentation pose.");
            }
        }
    }

    void previewLifecycle()
    {
        editor::SimulationPreview preview;
        auto track = authoredStraight(2.0);
        require(preview.rebuild(track), "Default four-car preview must be available.");
        requirePresentation(preview, 0.5);
        const auto initial = point(preview.meshInstances().front(), {});
        const auto* storage = preview.meshInstances().data();
        preview.play();
        preview.update(1.5 * physics::defaultFixedTimeStepSeconds);
        require(preview.dynamicsState()->tick == 1 && preview.interpolationAlpha() > 0.49
            && preview.interpolationAlpha() < 0.51, "Fractional tick must use existing interpolation.");
        requirePresentation(preview, 0.5);
        require(glm::length(point(preview.meshInstances().front(), {})
            - 0.5 * preview.pose()->cars().front().carPose().bodyWorldPositionMeters()) > 0.001,
            "Meshes must not bypass the render pose for the committed tick.");
        require(preview.meshInstances().data() == storage, "Playback retains per-car value storage.");
        const auto generation = preview.vertexGeneration();
        preview.update(0.25 * physics::defaultFixedTimeStepSeconds);
        require(preview.dynamicsState()->tick == 1 && preview.vertexGeneration() > generation,
            "Presentation-only interpolation must refresh meshes without a committed tick.");
        requirePresentation(preview, 0.5);
        preview.pause();
        const auto paused = point(preview.meshInstances().front(), {});
        preview.update(0.1);
        near(point(preview.meshInstances().front(), {}), paused, "Pause freezes presentation.");
        preview.setPhysicsDiagnosticsVisible(false);
        require(preview.vertices().empty() && preview.meshInstances().size() == 4,
            "Solid shells remain when diagnostic overlays are hidden.");
        requirePresentation(preview, 0.5);
        preview.setPhysicsDiagnosticsVisible(true);
        require(!preview.vertices().empty(), "Diagnostic boxes/bogies/connectors remain available.");
        preview.reset();
        near(point(preview.meshInstances().front(), {}), initial, "Reset restores initial shell transforms.");

        auto setup = track.coasterSetup();
        setup.carsPerTrain = 2;
        track.setCoasterSetup(setup);
        require(preview.rebuild(track) && preview.meshInstances().size() == 2,
            "Rebuild resizes the repeated-shell consist.");
        requirePresentation(preview, 0.5);
        require(!preview.rebuild(authoredStraight(1.0, 20.0, 1.0))
            && preview.meshInstances().empty() && preview.vertices().empty(),
            "Unavailable preview clears both presentation streams.");
        preview.play(); preview.reset(); preview.update(0.1);
        require(preview.meshInstances().empty(), "Unavailable controls must not resurrect shells.");

        auto uphill = authoredStraight(1.0, 1.0);
        uphill.setLayoutMode(coaster::LayoutMode::Shuttle);
        uphill.setStartPose({{}, glm::angleAxis(-0.3, glm::dvec3{0, 1, 0})});
        require(preview.rebuild(uphill), "Uphill rollback preview must rebuild.");
        preview.play();
        for (int tick = 0; tick < 240 && preview.isAvailable()
            && preview.dynamicsState()->signedVelocityMetersPerSecond >= 0.0; ++tick)
            preview.update(1.5 * physics::defaultFixedTimeStepSeconds);
        require(preview.isAvailable() && preview.dynamicsState()
            && preview.dynamicsState()->signedVelocityMetersPerSecond < 0.0,
            "Uphill train must roll back to exercise signed backward speed.");
        requirePresentation(preview, 1.0);
        require(preview.meshInstances().front().transform[0][0] > 0.9F
            && preview.renderPose()->cars().front().referenceLocation().direction
                == physics::TravelDirection::IncreasingStation,
            "Backward velocity must not flip physical car facing.");

        // Application uses this same value composition before its one setter.
        editor::RigidBodyMechanismProof mechanism;
        std::vector<renderer::StaticMeshInstance> combined;
        for (const bool enabled : {true, false, true, false})
        {
            const auto cars = preview.meshInstances();
            combined.assign(cars.begin(), cars.end());
            if (enabled)
            {
                const auto parts = mechanism.meshInstances();
                combined.insert(combined.end(), parts.begin(), parts.end());
            }
            require(combined.size() == (enabled ? 6 : 4),
                "Proof enable/disable must preserve all train instances.");
            for (std::size_t index = 0; index < cars.size(); ++index)
                near(point(combined[index], {}), point(cars[index], {}),
                    "Mechanism publication must not replace train transforms.");
        }
    }
}

int main(int argc, char** argv)
{
    try
    {
        require(argc == 2, "Provide repository asset root.");
        transformsAndAsset(argv[1]);
        previewLifecycle();
        std::cout << "Train Visual M0: transform/axes/units/alignment, interpolation, lifecycle and coexistence passed.\n";
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
