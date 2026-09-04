#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/editor/SimulationPreview.hpp>
#include <quantum/physics/TrackFollower.hpp>

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    using quantum::editor::SimulationPreview;

    void require(const bool condition, const std::string& message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    void requireNear(
        const double actual,
        const double expected,
        const double tolerance,
        const std::string& message)
    {
        if (!std::isfinite(actual)
            || std::abs(actual - expected) > tolerance)
        {
            throw std::runtime_error(message);
        }
    }

    [[nodiscard]] quantum::coaster::AuthoredTrack straightTrack(
        const double length = 60.0,
        const double initialSpeed = 20.0,
        const double metersPerCoordinateUnit = 1.0)
    {
        quantum::coaster::AuthoredTrack track =
            quantum::coaster::createNewDocument();
        quantum::coaster::setSectionLength(track.section(0), length);
        auto settings = track.physicalSettings();
        settings.initialSpeed = initialSpeed;
        settings.metersPerCoordinateUnit = metersPerCoordinateUnit;
        track.setPhysicalSettings(settings);
        return track;
    }

    void initializesFromAuthoredPhysicalSettings()
    {
        SimulationPreview preview;
        require(preview.rebuild(straightTrack(60.0, 7.25)),
            "preview should initialize on the default straight track");
        require(preview.isAvailable(), "initialized preview availability");
        require(preview.playbackState()
                == SimulationPreview::PlaybackState::Stopped,
            "initial preview playback state");
        require(preview.trainDefinition().cars.size() == 4
                && preview.trainDefinition().connections.size() == 3,
            "preview train shape");
        require(preview.pose() != nullptr
                && preview.pose()->carCount() == 4,
            "initial Core train pose");
        requireNear(preview.speedMetersPerSecond(), 7.25, 0.0,
            "authored initial speed policy");
        require(preview.dynamicsState() != nullptr
                && preview.dynamicsState()
                    ->generalizedReferenceLocation.stationMeters > 0.0,
            "open-track placement must keep the complete consist legal");
        require(preview.vertices().size() == 150,
            "four boxes, eight bogie markers, and three connectors");
    }

    void playbackUsesFixedStepsAndResetIsDeterministic()
    {
        SimulationPreview preview;
        require(preview.rebuild(straightTrack()), "playback fixture");
        const auto initialLocation =
            preview.dynamicsState()->generalizedReferenceLocation;
        const auto initialVertices =
            std::vector(preview.vertices().begin(), preview.vertices().end());

        preview.play();
        const std::uint64_t initialVertexGeneration =
            preview.vertexGeneration();
        require(preview.playbackState()
                == SimulationPreview::PlaybackState::Playing,
            "Play should enter Playing");
        preview.update(0.49 * quantum::physics::defaultFixedTimeStepSeconds);
        require(preview.dynamicsState()->tick == 0,
            "substep frame must not use variable-dt physics");
        require(preview.vertexGeneration() == initialVertexGeneration,
            "a render frame without a physics step must retain geometry");
        preview.update(0.51 * quantum::physics::defaultFixedTimeStepSeconds);
        require(preview.dynamicsState()->tick == 1,
            "accumulated frame time should execute one Core fixed step");
        require(preview.vertexGeneration() == initialVertexGeneration + 1,
            "a physics update should rebuild preview geometry once");

        preview.pause();
        const auto pausedState = *preview.dynamicsState();
        const std::uint64_t pausedVertexGeneration =
            preview.vertexGeneration();
        preview.update(1.0);
        require(preview.dynamicsState()->generalizedReferenceLocation
                    == pausedState.generalizedReferenceLocation
                && preview.dynamicsState()->signedVelocityMetersPerSecond
                    == pausedState.signedVelocityMetersPerSecond
                && preview.dynamicsState()
                    ->generalizedAccelerationMetersPerSecondSquared
                    == pausedState
                        .generalizedAccelerationMetersPerSecondSquared
                && preview.dynamicsState()->tick == pausedState.tick
                && preview.dynamicsState()->runState
                    == pausedState.runState,
            "Pause must preserve the current physics state");
        require(preview.vertexGeneration() == pausedVertexGeneration,
            "Pause must not rebuild preview geometry");
        preview.play();
        preview.update(quantum::physics::defaultFixedTimeStepSeconds);
        require(preview.dynamicsState()->tick == pausedState.tick + 1,
            "Play should resume a paused preview");

        preview.reset();
        require(preview.playbackState()
                == SimulationPreview::PlaybackState::Stopped,
            "Reset should return to Stopped");
        require(preview.dynamicsState()->tick == 0
                && preview.dynamicsState()->generalizedReferenceLocation
                    == initialLocation,
            "Reset should restore the initial Core state");
        require(preview.vertices().size() == initialVertices.size(),
            "Reset geometry size");
        for (std::size_t index = 0; index < initialVertices.size(); ++index)
        {
            requireNear(preview.vertices()[index].x,
                initialVertices[index].x, 0.0, "reset vertex x");
            requireNear(preview.vertices()[index].y,
                initialVertices[index].y, 0.0, "reset vertex y");
            requireNear(preview.vertices()[index].z,
                initialVertices[index].z, 0.0, "reset vertex z");
        }
    }

    void catchUpIsBounded()
    {
        SimulationPreview preview;
        require(preview.rebuild(straightTrack()), "catch-up fixture");
        preview.play();
        const std::uint64_t vertexGeneration = preview.vertexGeneration();
        preview.update(10.0);
        require(preview.dynamicsState()->tick
                == SimulationPreview::maximumStepsPerFrame,
            "one frame must execute at most the catch-up guard step count");
        require(preview.vertexGeneration() == vertexGeneration + 1
                && preview.vertices().size() == 150,
            "catch-up must publish one fixed-size geometry update per frame");
    }

    void rendererVerticesRespectDocumentScale()
    {
        SimulationPreview preview;
        constexpr double scale = 2.0;
        require(preview.rebuild(straightTrack(60.0, 10.0, scale)),
            "scaled preview fixture");

        const auto& car = preview.trainDefinition().cars.front().car;
        const glm::dvec3 expectedMeters =
            preview.pose()->cars().front().carPose().transformLocalPoint(
                -0.5 * car.bodyDimensionsMeters);
        requireNear(preview.vertices().front().x,
            expectedMeters.x / scale, 1.0e-5,
            "preview x must map SI pose back to document coordinates");
        requireNear(preview.vertices().front().y,
            expectedMeters.y / scale, 1.0e-5,
            "preview y must map SI pose back to document coordinates");
        requireNear(preview.vertices().front().z,
            expectedMeters.z / scale, 1.0e-5,
            "preview z must map SI pose back to document coordinates");
    }

    void rebuildAndShortTrackInvalidationAreSafe()
    {
        SimulationPreview preview;
        auto track = straightTrack();
        require(preview.rebuild(track), "rebuild fixture");
        preview.play();
        preview.update(quantum::physics::defaultFixedTimeStepSeconds);

        track.setStartPose({{5.0, -2.0, 1.0}, track.startPose().orientation});
        require(preview.rebuild(track), "edited track should rebuild");
        require(preview.playbackState()
                == SimulationPreview::PlaybackState::Stopped
                && preview.dynamicsState()->tick == 0,
            "track edit should stop and reset playback");

        require(!preview.rebuild(straightTrack(8.0)),
            "too-short track should be unavailable");
        require(!preview.isAvailable() && preview.vertices().empty()
                && !preview.error().empty(),
            "failed rebuild must clear every stale track-dependent value");
    }

    void openTrackPausesAtItsLegalEndpoint()
    {
        SimulationPreview preview;
        require(preview.rebuild(straightTrack(25.0, 20.0)),
            "open-end fixture");
        preview.play();
        for (int frame = 0;
            frame < 20
                && preview.playbackState()
                    == SimulationPreview::PlaybackState::Playing;
            ++frame)
        {
            preview.update(0.25);
        }
        require(preview.isAvailable()
                && preview.playbackState()
                    == SimulationPreview::PlaybackState::Paused,
            "Core boundary intervention should pause an open-track preview; "
            "state=" + std::to_string(static_cast<int>(
                preview.playbackState())) + " tick="
                + std::to_string(preview.dynamicsState()
                    ? preview.dynamicsState()->tick : 0)
                + " error=" + preview.error());
    }

}

int main()
{
    try
    {
        initializesFromAuthoredPhysicalSettings();
        playbackUsesFixedStepsAndResetIsDeterministic();
        catchUpIsBounded();
        rendererVerticesRespectDocumentScale();
        rebuildAndShortTrackInvalidationAreSafe();
        openTrackPausesAtItsLegalEndpoint();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "SimulationPreviewTests failed: "
                  << exception.what() << '\n';
        return EXIT_FAILURE;
    }

    std::cout << "SimulationPreviewTests passed\n";
    return EXIT_SUCCESS;
}
