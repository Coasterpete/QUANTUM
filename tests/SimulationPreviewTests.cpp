#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/editor/SimulationPreview.hpp>
#include <quantum/physics/TrackFollower.hpp>

#include <glm/geometric.hpp>
#include <quantum/geometry/RotationMinimizingFrames.hpp>
#include <algorithm>
#include <chrono>
#include <numbers>

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
    using quantum::editor::interpolateTrainPreviewPose;
    using namespace quantum::physics;
    using quantum::coaster::TopologyKind;
    using quantum::coaster::TrackKinematicState;
    using quantum::geometry::CurveFrame;

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

    void requireSameVertices(
        const std::span<const quantum::renderer::LineVertex> actual,
        const std::span<const quantum::renderer::LineVertex> expected,
        const std::string& message)
    {
        require(actual.size() == expected.size(), message + " size");
        for (std::size_t index = 0; index < actual.size(); ++index)
        {
            require(actual[index].x == expected[index].x
                    && actual[index].y == expected[index].y
                    && actual[index].z == expected[index].z
                    && actual[index].color == expected[index].color,
                message + " vertex " + std::to_string(index));
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

    [[nodiscard]] quantum::coaster::AuthoredTrack
    straightToRisingTrack()
    {
        constexpr double straightLength = 25.0;
        constexpr double risingLength = 55.0;
        quantum::coaster::AuthoredTrack track =
            quantum::coaster::createNewDocument();
        track.setLayoutMode(quantum::coaster::LayoutMode::Shuttle);
        quantum::coaster::setSectionLength(
            track.section(0), straightLength);

        quantum::coaster::AuthoredTrackSection rising =
            quantum::coaster::createRateProfileSection(risingLength);
        auto& pitch = rising.rateProfileRegion().rateProfiles.pitch
            .segments.front().transition;
        pitch.valueBegin = -0.025;
        pitch.valueEnd = -0.025;
        track.insertSectionAfter(0, rising);

        auto settings = track.physicalSettings();
        settings.initialSpeed = 30.0;
        track.setPhysicalSettings(settings);
        return track;
    }

    void requireRigidPreviewPivots(const SimulationPreview& preview)
    {
        constexpr double toleranceMeters = 1.0e-8;
        require(preview.pose() != nullptr, "preview pose must be available");
        for (const quantum::physics::TrainCarPose& trainCar
            : preview.renderPose()->cars())
        {
            const quantum::physics::CarPose& carPose = trainCar.carPose();
            const quantum::physics::CarDefinition& definition =
                preview.trainDefinition().cars[trainCar.carIndex()].car;
            for (const quantum::physics::BogiePose* bogie : {
                &carPose.frontBogie(), &carPose.rearBogie()})
            {
                const glm::dvec3 transformed = carPose.transformLocalPoint(
                    definition.bogies[bogie->definitionIndex()]
                        .referencePositionMeters);
                require(std::isfinite(glm::length(
                        transformed - bogie->worldPositionMeters()))
                        && glm::length(
                            transformed - bogie->worldPositionMeters())
                            <= toleranceMeters,
                    "preview car pivot must coincide with its bogie pose");
            }
        }
    }

    [[nodiscard]] CurveFrame frameForTangent(
        const glm::dvec3& tangent,
        const double bankRadians = 0.0)
    {
        const glm::dvec3 unit = glm::normalize(tangent);
        const CurveFrame base{
            unit,
            {0.0, 1.0, 0.0},
            glm::normalize(glm::cross(unit, glm::dvec3{0.0, 1.0, 0.0}))
        };
        return quantum::geometry::applyRoll(base, bankRadians);
    }

    [[nodiscard]] CompiledPhysicsTrack horizontalCircleTrack(
        const double radius = 25.0,
        const bool varyingBank = false)
    {
        constexpr int count = 720;
        std::vector<TrackKinematicState> samples;
        samples.reserve(count + 1);
        for (int index = 0; index <= count; ++index)
        {
            const double angle = 2.0 * std::numbers::pi
                * static_cast<double>(index) / count;
            CurveFrame frame{
                {std::cos(angle), std::sin(angle), 0.0},
                {-std::sin(angle), std::cos(angle), 0.0},
                {0.0, 0.0, 1.0}
            };
            if (varyingBank)
            {
                frame = quantum::geometry::applyRoll(
                    frame, 0.35 * std::sin(2.0 * angle));
            }
            samples.push_back({
                radius * angle,
                {
                    radius * std::sin(angle),
                    radius * (1.0 - std::cos(angle)),
                    0.0
                },
                frame,
                {
                    -std::sin(angle) / radius,
                    std::cos(angle) / radius,
                    0.0
                }
            });
        }
        return {samples, 1.0, TopologyKind::ClosedCircuit};
    }

    [[nodiscard]] CompiledPhysicsTrack verticalArcTrack(
        const bool crest,
        const bool varyingBank = false)
    {
        constexpr double radius = 24.0;
        constexpr double startAngle = -0.9;
        constexpr double endAngle = 0.9;
        constexpr int count = 600;
        std::vector<TrackKinematicState> samples;
        samples.reserve(count + 1);
        for (int index = 0; index <= count; ++index)
        {
            const double angle = startAngle
                + (endAngle - startAngle) * static_cast<double>(index) / count;
            const double verticalSign = crest ? 1.0 : -1.0;
            const glm::dvec3 tangent{
                std::cos(angle), 0.0, -verticalSign * std::sin(angle)};
            CurveFrame frame{
                tangent,
                {0.0, 1.0, 0.0},
                glm::cross(tangent, glm::dvec3{0.0, 1.0, 0.0})
            };
            if (varyingBank)
            {
                frame = quantum::geometry::applyRoll(
                    frame, 0.3 * std::sin(3.0 * angle));
            }
            samples.push_back({
                radius * (angle - startAngle),
                {
                    radius * std::sin(angle),
                    0.0,
                    verticalSign * radius * std::cos(angle)
                },
                frame,
                {
                    -std::sin(angle) / radius,
                    0.0,
                    -verticalSign * std::cos(angle) / radius
                }
            });
        }
        return {samples, 1.0, TopologyKind::OpenLinear};
    }

    [[nodiscard]] CompiledPhysicsTrack shuttleSpikeTrack()
    {
        constexpr double runInLength = 40.0;
        constexpr double transitionRadius = 15.0;
        constexpr double transitionAngle = 1.2;
        constexpr double spikeLength = 80.0;
        constexpr int transitionSampleCount = 180;

        std::vector<TrackKinematicState> samples;
        samples.reserve(transitionSampleCount + 3);
        samples.push_back({
            0.0,
            {0.0, 0.0, 0.0},
            frameForTangent({1.0, 0.0, 0.0}),
            {0.0, 0.0, 0.0}
        });
        samples.push_back({
            runInLength,
            {runInLength, 0.0, 0.0},
            frameForTangent({1.0, 0.0, 0.0}),
            {0.0, 0.0, 1.0 / transitionRadius}
        });

        for (int index = 1; index <= transitionSampleCount; ++index)
        {
            const double angle = transitionAngle
                * static_cast<double>(index) / transitionSampleCount;
            const double station = runInLength
                + transitionRadius * angle;
            const glm::dvec3 tangent{
                std::cos(angle), 0.0, std::sin(angle)};
            samples.push_back({
                station,
                {
                    runInLength + transitionRadius * std::sin(angle),
                    0.0,
                    transitionRadius * (1.0 - std::cos(angle))
                },
                frameForTangent(tangent),
                {
                    -std::sin(angle) / transitionRadius,
                    0.0,
                    std::cos(angle) / transitionRadius
                }
            });
        }

        const double transitionEndStation = runInLength
            + transitionRadius * transitionAngle;
        const glm::dvec3 spikeTangent{
            std::cos(transitionAngle), 0.0, std::sin(transitionAngle)};
        const glm::dvec3 transitionEnd{
            runInLength + transitionRadius * std::sin(transitionAngle),
            0.0,
            transitionRadius * (1.0 - std::cos(transitionAngle))
        };
        samples.push_back({
            transitionEndStation + spikeLength,
            transitionEnd + spikeLength * spikeTangent,
            frameForTangent(spikeTangent),
            {0.0, 0.0, 0.0}
        });
        return {samples, 1.0, TopologyKind::OpenLinear};
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
        requireNear(preview.interpolationAlpha(), 0.49, 1e-12,
            "accumulator before the first completed tick");
        require(preview.renderPose()->generalizedReferenceLocation() == initialLocation
                && preview.frameTelemetry().renderPoseSolveCount == 0,
            "no completed tick retains initial presentation without a solve");
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

    void requireSameState(const TrainDynamicsState& a, const TrainDynamicsState& b);
    void requireSamePose(const TrainPose& a, const TrainPose& b);

    void timingDiscontinuityPreservesFractionalTick()
    {
        SimulationPreview preview;
        SimulationPreview control;
        require(preview.rebuild(straightTrack())
            && control.rebuild(straightTrack()), "discontinuity fixture");
        preview.play();
        control.play();
        constexpr double dt = defaultFixedTimeStepSeconds;
        preview.update(2.5 * dt);
        control.update(2.5 * dt);
        const auto generation = preview.vertexGeneration();
        const auto rendered = *preview.renderPose();
        preview.beginFrameTelemetry();
        preview.update(10.0, true);
        const auto& telemetry = preview.frameTelemetry();
        require(telemetry.requestedStepCount == 0
            && telemetry.fixedStepCount == 0
            && !telemetry.maximumStepsHit
            && telemetry.consecutiveCatchUpFrameCount == 0,
            "restoration must not request or execute stale ticks");
        requireNear(telemetry.rawDeltaMilliseconds, 10000.0, 1e-9,
            "raw host interval remains observable");
        requireNear(telemetry.discardedWallTimeMilliseconds, 10000.0, 1e-9,
            "the entire interrupted wall interval is accounted for");
        requireNear(telemetry.accumulatorRemainingMilliseconds,
            0.5 * dt * 1000.0, 1e-9, "fractional tick survives restoration");
        requireSameState(*preview.dynamicsState(), *control.dynamicsState());
        requireSamePose(*preview.renderPose(), rendered);
        require(preview.vertexGeneration() == generation,
            "discarding wall time does not mutate presentation");
        preview.beginFrameTelemetry();
        preview.update(0.5 * dt);
        control.update(0.5 * dt);
        require(preview.frameTelemetry().fixedStepCount == 1,
            "the next ordinary half tick completes the retained remainder");
        requireSameState(*preview.dynamicsState(), *control.dynamicsState());
        requireSamePose(*preview.pose(), *control.pose());
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
        const auto& telemetry = preview.frameTelemetry();
        requireNear(telemetry.rawDeltaMilliseconds, 10000.0, 0.0,
            "telemetry preserves the raw incoming delta");
        requireNear(telemetry.accumulatorBeforeMilliseconds, 0.0, 0.0,
            "catch-up accumulator before input");
        requireNear(telemetry.accumulatorAfterIncomingMilliseconds,
            250.0, 1.0e-9, "catch-up accumulator cap");
        requireNear(telemetry.discardedWallTimeMilliseconds,
            9750.0, 1.0e-9, "discarded wall time telemetry");
        require(telemetry.requestedStepCount
                == SimulationPreview::maximumStepsPerFrame
                && telemetry.fixedStepCount
                    == SimulationPreview::maximumStepsPerFrame
                && telemetry.maximumStepsHit,
            "telemetry must distinguish requested, executed, and capped work");
        require(telemetry.minimumStepMilliseconds >= 0.0
                && telemetry.averageStepMilliseconds
                    >= telemetry.minimumStepMilliseconds
                && telemetry.maximumStepMilliseconds
                    >= telemetry.averageStepMilliseconds
                && telemetry.physicsMilliseconds
                    >= telemetry.maximumStepMilliseconds,
            "per-step timing must be ordered within total physics timing");
        require(telemetry.consecutiveCatchUpFrameCount == 1,
            "first catch-up frame starts the diagnostic streak");
        require(preview.vertexGeneration() == vertexGeneration + 1
                && preview.vertices().size() == 150,
            "catch-up must publish one fixed-size geometry update per frame");

        preview.beginFrameTelemetry();
        preview.update(8.0 * quantum::physics::defaultFixedTimeStepSeconds);
        require(preview.frameTelemetry().requestedStepCount == 8
                && preview.frameTelemetry().consecutiveCatchUpFrameCount == 2,
            "eight-step demand extends the documented catch-up streak");
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

    void gpuPreviewUploadAndInterpolationCacheAreSafe()
    {
        quantum::physics::gpu::GpuPhysicsContext::HeadlessHandles handles;
        try
        {
            handles = quantum::physics::gpu::GpuPhysicsContext::
                createHeadlessHandles();
        }
        catch (const std::exception& exception)
        {
            std::cout << "GPU preview production-path test skipped: "
                << exception.what() << '\n';
            return;
        }

        quantum::physics::gpu::GpuPhysicsContext gpu(std::move(handles));
        if (!gpu.gpuAvailable())
        {
            std::cout << "GPU preview production-path test skipped: "
                "compute pipeline unavailable\n";
            return;
        }

        SimulationPreview gpuPreview;
        SimulationPreview cpuPreview;
        gpuPreview.setGpuContext(&gpu);
        const auto initialTrack = straightTrack();
        require(gpuPreview.rebuild(initialTrack)
                && cpuPreview.rebuild(initialTrack),
            "GPU preview upload fixture");
        require(gpu.hasUploadedTrack() && gpu.gpuTrackReady(),
            "preview rebuild must upload the active track");

        gpuPreview.play();
        cpuPreview.play();
        constexpr double dt = defaultFixedTimeStepSeconds;
        gpuPreview.update(dt);
        cpuPreview.update(dt);
        require(gpu.lastSampleUsedGpu(),
            "preview update must execute the bogie batch on GPU");
        requireNear(gpuPreview.interpolationAlpha(), 0.0, 1e-12,
            "GPU preview stale-cache fixture alpha");
        requireSameVertices(gpuPreview.vertices(), cpuPreview.vertices(),
            "interpolated render pose must reject committed-pose GPU samples");

        auto replacementTrack = straightTrack();
        replacementTrack.setStartPose({
            {5.0, -2.0, 1.0}, replacementTrack.startPose().orientation});
        require(gpuPreview.rebuild(replacementTrack)
                && cpuPreview.rebuild(replacementTrack),
            "GPU preview replacement fixture");
        require(gpu.hasUploadedTrack() && gpu.gpuTrackReady(),
            "replacement preview track must refresh the GPU upload");
        requireSameVertices(gpuPreview.vertices(), cpuPreview.vertices(),
            "track rebuild must invalidate old cached GPU markers");
        std::cout << "GPU preview production-path test EXECUTED\n";
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
        requireNear(preview.interpolationAlpha(), 1.0, 0.0,
            "boundary intervention publishes the feasible current endpoint");
        require(preview.frameTelemetry().boundaryStopped,
            "normal boundary completion is explicitly identifiable by smoke mode");
        require(preview.renderPose()->generalizedReferenceLocation()
                == preview.pose()->generalizedReferenceLocation(),
            "boundary render pose is the committed endpoint");
        const auto endpoint = *preview.dynamicsState();
        preview.reset();
        preview.play();
        for (int frame = 0; frame < 20
            && preview.playbackState() == SimulationPreview::PlaybackState::Playing;
            ++frame)
        {
            preview.beginFrameTelemetry();
            preview.update(0.25);
        }
        requireSameState(*preview.dynamicsState(), endpoint);
        require(preview.frameTelemetry().boundaryStopped,
            "replay reaches the same normal boundary");
        preview.beginFrameTelemetry();
        require(!preview.frameTelemetry().boundaryStopped,
            "boundary marker cannot leak into an ordinary paused frame");
    }

    void transitionPlaybackKeepsEveryCommittedCarRigid()
    {
        constexpr double boundaryStationMeters = 25.0;
        SimulationPreview preview;
        require(preview.rebuild(straightToRisingTrack()),
            "straight-to-rising preview fixture");
        bool observedBefore = false;
        bool observedStraddling = false;
        bool observedAfter = false;
        preview.play();
        for (int step = 0; step < 900 && !observedAfter; ++step)
        {
            preview.update(quantum::physics::defaultFixedTimeStepSeconds);
            require(preview.isAvailable(),
                "transition playback must remain available");
            requireRigidPreviewPivots(preview);

            require(preview.renderPose()->maximumAbsoluteConnectorResidualMeters()
                    <= quantum::physics::connectorLengthToleranceMeters,
                "transition playback must preserve connector closure");

            bool anyBefore = false;
            bool anyAfter = false;
            for (const quantum::physics::TrainCarPose& trainCar
                : preview.renderPose()->cars())
            {
                const quantum::physics::CarPose& car = trainCar.carPose();
                for (const quantum::physics::BogiePose* bogie : {
                    &car.frontBogie(), &car.rearBogie()})
                {
                    anyBefore |= bogie->location().stationMeters
                        < boundaryStationMeters;
                    anyAfter |= bogie->location().stationMeters
                        >= boundaryStationMeters;
                }
            }
            observedBefore |= anyBefore && !anyAfter;
            observedStraddling |= anyBefore && anyAfter;
            observedAfter |= !anyBefore && anyAfter;
        }
        require(observedBefore && observedStraddling && observedAfter,
            "preview must advance the complete train through the transition");
    }


    void requireValidRenderPose(const TrainPose& pose, const TrainDefinition& train)
    {
        require(pose.carCount() == train.cars.size(), "render consist size");
        require(pose.maximumAbsoluteConnectorResidualMeters()
            <= connectorLengthToleranceMeters, "render connector closure");
        for (std::size_t i = 0; i < pose.carCount(); ++i)
        {
            const auto& car = pose.cars()[i].carPose();
            require(pose.cars()[i].carIndex() == i, "render car ordering");
            for (const auto* bogie : {&car.frontBogie(), &car.rearBogie()})
            {
                requireNear(glm::length(car.transformLocalPoint(
                    train.cars[i].car.bogies[bogie->definitionIndex()]
                        .referencePositionMeters) - bogie->worldPositionMeters()),
                    0.0, 1e-8, "render rigid bogie coincidence");
            }
        }
    }

    void requireSameState(const TrainDynamicsState& a, const TrainDynamicsState& b)
    {
        require(a.generalizedReferenceLocation == b.generalizedReferenceLocation
            && a.signedVelocityMetersPerSecond == b.signedVelocityMetersPerSecond
            && a.generalizedAccelerationMetersPerSecondSquared
                == b.generalizedAccelerationMetersPerSecondSquared
            && a.tick == b.tick && a.runState == b.runState,
            "interpolation must preserve every committed dynamics field");
    }

    void requireSamePose(const TrainPose& a, const TrainPose& b)
    {
        require(a.generalizedReferenceLocation() == b.generalizedReferenceLocation(),
            "deterministic render location");
        for (std::size_t i = 0; i < a.carCount(); ++i)
        {
            const auto& ac = a.cars()[i].carPose();
            const auto& bc = b.cars()[i].carPose();
            require(ac.bodyOrientation() == bc.bodyOrientation()
                && ac.transformLocalPoint({0, 0, 0}) == bc.transformLocalPoint({0, 0, 0})
                && ac.frontBogie().location() == bc.frontBogie().location()
                && ac.rearBogie().location() == bc.rearBogie().location(),
                "deterministic complete car geometry");
        }
        for (std::size_t i = 0; i < a.connectionCount(); ++i)
        {
            require(a.connections()[i].leadingEndpointWorldPositionMeters()
                    == b.connections()[i].leadingEndpointWorldPositionMeters()
                && a.connections()[i].followingEndpointWorldPositionMeters()
                    == b.connections()[i].followingEndpointWorldPositionMeters(),
                "deterministic connector geometry");
        }
    }

    void interpolationGeometryAndEndpoints()
    {
        SimulationPreview preview;
        require(preview.rebuild(straightTrack()), "interpolation train fixture");
        auto train = preview.trainDefinition();
        train.resistance = {}; // Constant-speed straight fixture has no resistance.
        const CurveFrame frame{{1,0,0}, {0,1,0}, {0,0,1}};
        const std::vector<TrackKinematicState> samples{
            {0, {0,0,0}, frame, {}}, {100, {100,0,0}, frame, {}}};
        const CompiledPhysicsTrack straight(samples, 1.0, TopologyKind::OpenLinear);
        const auto circle = horizontalCircleTrack();
        const auto vertical = verticalArcTrack(false);
        const auto spike = shuttleSpikeTrack();
        for (const auto* track : {&straight, &circle, &vertical, &spike})
        {
            for (const double distance : {0.1, -0.1})
            {
                const double station = track == &spike ? 40.0 : 25.0;
                const auto from = solveTrainPose(*track, train,
                    {primaryTrackPathId, station, TravelDirection::IncreasingStation});
                const auto to = solveTrainPose(*track, train,
                    {primaryTrackPathId, station + distance,
                        TravelDirection::IncreasingStation});
                for (const double alpha : {0.0, 1e-9, 0.5, 1.0 - 1e-9, 1.0})
                {
                    const auto rendered = interpolateTrainPreviewPose(
                        *track, train, from, to, alpha);
                    requireNear(rendered.generalizedReferenceLocation().stationMeters,
                        station + alpha * distance, 1e-12, "linear reference station");
                    requireValidRenderPose(rendered, train);
                    requireSamePose(rendered, interpolateTrainPreviewPose(
                        *track, train, from, to, alpha));
                    if (alpha == 0.0) requireSamePose(rendered, from);
                    if (alpha == 1.0) requireSamePose(rendered, to);
                }
            }
        }
        TrainDynamicsState state;
        state.generalizedReferenceLocation.stationMeters = 25.0;
        state.signedVelocityMetersPerSecond = 24.0;
        state.runState = FollowerRunState::Running;
        const auto next = stepTrain(straight, train, {}, state);
        const auto previous = solveTrainPose(straight, train, state.generalizedReferenceLocation);
        const auto half = interpolateTrainPreviewPose(straight, train,
            previous, next.telemetry.pose, 0.5);
        requireNear(half.generalizedReferenceLocation().stationMeters,
            25.0 + 0.5 * 24.0 * defaultFixedTimeStepSeconds, 1e-10,
            "constant-speed straight displacement");

        // Both seam directions use the short physical path, with unchanged facing.
        for (const double distance : {0.1, -0.1})
        {
            const double fromStation = distance > 0 ? circle.lengthMeters() - 0.05 : 0.05;
            const double toStation = distance > 0 ? 0.05 : circle.lengthMeters() - 0.05;
            const auto from = solveTrainPose(circle, train,
                {primaryTrackPathId, fromStation, TravelDirection::IncreasingStation});
            const auto to = solveTrainPose(circle, train,
                {primaryTrackPathId, toStation, TravelDirection::IncreasingStation});
            for (const double alpha : {0.0, 1e-9, 0.5, 1.0 - 1e-9, 1.0})
            {
                const auto result = interpolateTrainPreviewPose(circle, train, from, to, alpha);
                const double expected = circle.advance(
                    from.generalizedReferenceLocation(), alpha * distance).location.stationMeters;
                requireNear(std::remainder(result.generalizedReferenceLocation().stationMeters
                    - expected, circle.lengthMeters()), 0, 1e-12, "local seam motion");
                require(result.generalizedReferenceLocation().direction
                    == TravelDirection::IncreasingStation, "seam physical facing");
                requireValidRenderPose(result, train);
            }
        }
        // Complete-consist feasibility close to both legal open ends.
        for (const double station : {14.7, 98.75})
        {
            const auto from = solveTrainPose(straight, train,
                {primaryTrackPathId, station, TravelDirection::IncreasingStation});
            const auto to = solveTrainPose(straight, train,
                {primaryTrackPathId, station + 0.05, TravelDirection::IncreasingStation});
            for (const double alpha : {-1.0, 0.0, 0.5, 1.0, 2.0})
            {
                const auto result = interpolateTrainPreviewPose(straight, train, from, to, alpha);
                requireValidRenderPose(result, train);
                requireNear(result.generalizedReferenceLocation().stationMeters,
                    station + 0.05 * std::clamp(alpha, 0.0, 1.0), 1e-12,
                    "endpoint interpolation cannot extrapolate");
            }
        }
    }

    void renderCadenceDoesNotAffectPhysics()
    {
        SimulationPreview preview, control;
        require(preview.rebuild(straightToRisingTrack())
            && control.rebuild(straightToRisingTrack()), "render cadence fixtures");
        preview.play();
        control.play();
        constexpr double dt = defaultFixedTimeStepSeconds;
        double interpolationMs = 0.0, solveMs = 0.0, verticesMs = 0.0;
        for (int step = 0; step < 120; ++step)
        {
            const auto before = *preview.dynamicsState();
            preview.update(dt);
            control.update(dt);
            const auto committed = *preview.dynamicsState();
            const auto committedPose = *preview.pose();
            requireNear(preview.interpolationAlpha(), 0.0, 1e-12, "alpha after tick");
            requireNear(preview.renderPose()->generalizedReferenceLocation().stationMeters,
                before.generalizedReferenceLocation.stationMeters, 1e-12, "alpha zero is previous");
            preview.beginFrameTelemetry();
            preview.update(dt * 0.5);
            requireSameState(*preview.dynamicsState(), committed);
            requireSamePose(*preview.pose(), committedPose);
            requireNear(preview.interpolationAlpha(), 0.5, 1e-12, "half accumulator");
            require(preview.frameTelemetry().renderPoseSolveCount == 1
                && preview.frameTelemetry().renderPoseFailureCount == 0,
                "one presentation solve without physics advancement");
            requireValidRenderPose(*preview.renderPose(), preview.trainDefinition());
            interpolationMs += preview.frameTelemetry().interpolationMilliseconds;
            solveMs += preview.frameTelemetry().renderPoseSolveMilliseconds;
            verticesMs += preview.frameTelemetry().vertexPreparationMilliseconds;
            preview.update(dt * 0.5);
            control.update(dt);
            requireSameState(*preview.dynamicsState(), *control.dynamicsState());
            requireSamePose(*preview.pose(), *control.pose());
        }
        const auto before = *preview.dynamicsState();
        control.update(dt);
        control.update(dt);
        const auto penultimate = *control.dynamicsState();
        control.update(dt);
        preview.beginFrameTelemetry();
        preview.update(3.5 * dt);
        require(preview.dynamicsState()->tick == before.tick + 3,
            "three fixed steps before render");
        requireSameState(*preview.dynamicsState(), *control.dynamicsState());
        requireNear(preview.renderPose()->generalizedReferenceLocation().stationMeters,
            0.5 * (penultimate.generalizedReferenceLocation.stationMeters
                + control.dynamicsState()->generalizedReferenceLocation.stationMeters),
            1e-12, "catch-up retains last adjacent pair");
        require(preview.frameTelemetry().renderPoseSolveCount == 1,
            "catch-up solves presentation only once");
        std::cout << "Interpolation transition substeps: frames=120 solve_ms/frame="
            << solveMs / 120 << " total_ms/frame=" << interpolationMs / 120
            << " vertices_ms/frame=" << verticesMs / 120 << '\n';
    }

    void spikeRollbackInterpolation()
    {
        SimulationPreview fixture;
        require(fixture.rebuild(straightTrack()), "spike train fixture");
        const auto& train = fixture.trainDefinition();
        const auto track = shuttleSpikeTrack();
        TrainDynamicsState state;
        state.generalizedReferenceLocation.stationMeters = 25.0;
        state.signedVelocityMetersPerSecond = 24.0;
        state.runState = FollowerRunState::Running;
        auto previous = solveTrainPose(track, train, state.generalizedReferenceLocation);
        bool zero = false, backward = false, spike = false, returned = false;
        quantum::editor::SimulationPreviewFrameTelemetry telemetry;
        double totalMs = 0;
        std::size_t frames = 0, fallbacks = 0;
        for (int step = 0; step < 2400 && !returned; ++step)
        {
            const auto saved = state;
            const auto result = stepTrain(track, train, {}, state);
            const auto& current = result.telemetry.pose;
            for (const double alpha : {0.0, 1e-9, 0.5, 1.0 - 1e-9, 1.0})
            {
                const auto begin = std::chrono::steady_clock::now();
                const auto rendered = interpolateTrainPreviewPose(
                    track, train, previous, current, alpha, &telemetry);
                totalMs += std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - begin).count();
                ++frames;
                requireValidRenderPose(rendered, train);
                requireNear(rendered.generalizedReferenceLocation().stationMeters,
                    std::lerp(saved.generalizedReferenceLocation.stationMeters,
                        result.state.generalizedReferenceLocation.stationMeters, alpha),
                    1e-12, "spike local station including exact zero");
                for (std::size_t i = 0; i < rendered.carCount(); ++i)
                {
                    require(glm::dot(rendered.cars()[i].carPose().bodyOrientation(),
                        previous.cars()[i].carPose().bodyOrientation()) > 0.999,
                        "rollback cannot flip facing or change connector branch");
                }
                for (const auto& connection : rendered.connections())
                    fallbacks += connection.usedExhaustiveSearchFallback();
                requireSameState(state, saved);
            }
            if (result.state.signedVelocityMetersPerSecond == 0.0)
            {
                zero = true;
                requireSamePose(previous, current);
                const auto repeated = stepTrain(track, train, {}, saved);
                requireSameState(repeated.state, result.state);
                requireSamePose(repeated.telemetry.pose, current);
            }
            backward |= result.state.signedVelocityMetersPerSecond < 0.0;
            spike |= result.state.generalizedReferenceLocation.stationMeters > 58.0;
            returned = backward && result.state.generalizedReferenceLocation.stationMeters < 39.0;
            state = result.state;
            previous = current;
        }
        require(zero && backward && spike && returned,
            "spike must cover approach, exact zero, reversal, and return transition");
        std::cout << "Interpolation spike: frames=" << frames
            << " solves=" << telemetry.renderPoseSolveCount
            << " solve_ms/solve=" << telemetry.renderPoseSolveMilliseconds / telemetry.renderPoseSolveCount
            << " total_ms/frame=" << totalMs / frames
            << " exhaustive_fallbacks=" << fallbacks << '\n';
    }

}

int main()
{
    try
    {
        interpolationGeometryAndEndpoints();
        renderCadenceDoesNotAffectPhysics();
        spikeRollbackInterpolation();
        initializesFromAuthoredPhysicalSettings();
        playbackUsesFixedStepsAndResetIsDeterministic();
        catchUpIsBounded();
        timingDiscontinuityPreservesFractionalTick();
        rendererVerticesRespectDocumentScale();
        rebuildAndShortTrackInvalidationAreSafe();
        gpuPreviewUploadAndInterpolationCacheAreSafe();
        openTrackPausesAtItsLegalEndpoint();
        transitionPlaybackKeepsEveryCommittedCarRigid();
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
