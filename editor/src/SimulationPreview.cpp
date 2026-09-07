#include <quantum/editor/SimulationPreview.hpp>

#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>

namespace quantum::editor
{
    namespace
    {
        inline constexpr std::size_t previewCarCount = 4;
        // This fixed train first fits a straight open path at about 14.65 m.
        // The small margin keeps its initial bogies away from the endpoint;
        // Core still validates the complete consist at every candidate.
        inline constexpr double preferredInitialStationMeters = 14.75;
        inline constexpr double initialPlacementSearchStepMeters = 0.5;

        inline constexpr glm::dvec3 previewCarDimensionsMeters{
            4.0, 1.35, 1.4};
        inline constexpr double previewCarDryMassKilograms = 800.0;
        inline constexpr double previewCarLoadMassKilograms = 200.0;
        inline constexpr double previewBogieHalfSpacingMeters = 1.15;
        inline constexpr double previewConnectorLengthMeters = 0.5;

        using Color = std::array<float, 4>;
        inline constexpr std::array<Color, 2> carColors{{
            {0.20F, 0.75F, 0.95F, 1.0F},
            {0.15F, 0.55F, 0.80F, 1.0F}
        }};
        inline constexpr Color bogieColor{0.95F, 0.95F, 0.95F, 1.0F};
        inline constexpr Color connectorColor{1.0F, 0.55F, 0.12F, 1.0F};

        struct InitialPlacement
        {
            physics::TrackLocation location;
            physics::TrainPose pose;
        };

        [[nodiscard]] physics::TrainDefinition createPreviewTrainDefinition()
        {
            using namespace physics;

            CarDefinition car;
            car.dryMassKilograms = previewCarDryMassKilograms;
            car.dryCenterOfGravityMeters = {0.0, 0.0, 0.55};
            car.dryInertiaTensorBodyKgM2 =
                makeUniformBoxInertiaTensorBodyKgM2(
                    car.dryMassKilograms,
                    previewCarDimensionsMeters);
            car.bodyDimensionsMeters = previewCarDimensionsMeters;
            car.frontHitchPositionMeters = {2.0, 0.0, 0.2};
            car.rearHitchPositionMeters = {-2.0, 0.0, 0.2};
            car.bogies = {
                BogieDefinition{{previewBogieHalfSpacingMeters, 0.0, 0.0}},
                BogieDefinition{{-previewBogieHalfSpacingMeters, 0.0, 0.0}}
            };

            const CarLoadout loadout{
                previewCarLoadMassKilograms,
                {0.0, 0.0, 0.9}
            };

            TrainDefinition train;
            train.cars.reserve(previewCarCount);
            train.connections.reserve(previewCarCount - 1);
            for (std::size_t index = 0; index < previewCarCount; ++index)
            {
                train.cars.push_back({car, loadout});
                if (index != 0)
                {
                    train.connections.push_back(
                        {previewConnectorLengthMeters});
                }
            }

            // This temporary consist uses the existing aggregate resistance
            // law; no preview-specific motion or operations force is added.
            train.resistance.constantMechanicalForceNewtons = 500.0;
            train.resistance.linearResistanceCoefficientNewtonSecondsPerMeter =
                50.0;
            train.resistance.airDensityKilogramsPerCubicMeter = 1.225;
            train.resistance.dragAreaSquareMeters = 2.5;
            train.resistance.rollingResistanceCoefficient = 0.01;

            validateTrainDefinition(train);
            return train;
        }

        [[nodiscard]] std::optional<InitialPlacement> tryPlacement(
            const physics::CompiledPhysicsTrack& track,
            const physics::TrainDefinition& train,
            const double stationMeters)
        {
            physics::TrackLocation location{
                physics::primaryTrackPathId,
                stationMeters,
                physics::TravelDirection::IncreasingStation
            };
            try
            {
                return InitialPlacement{
                    location,
                    physics::solveTrainPose(track, train, location)
                };
            }
            catch (const std::exception&)
            {
                return std::nullopt;
            }
        }

        [[nodiscard]] InitialPlacement findInitialPlacement(
            const physics::CompiledPhysicsTrack& track,
            const physics::TrainDefinition& train)
        {
            if (track.topology() == coaster::TopologyKind::ClosedCircuit)
            {
                if (auto placement = tryPlacement(track, train, 0.0))
                {
                    return std::move(*placement);
                }
            }
            else
            {
                const double length = track.lengthMeters();
                for (double station = preferredInitialStationMeters;
                    station < length;
                    station += initialPlacementSearchStepMeters)
                {
                    if (auto placement = tryPlacement(track, train, station))
                    {
                        return std::move(*placement);
                    }
                }
            }

            throw std::invalid_argument(
                "The authored track has no legal increasing-station placement "
                "for the four-car preview train.");
        }

        [[nodiscard]] renderer::LineVertex vertex(
            const glm::dvec3& positionMeters,
            const double coordinateUnitsPerMeter,
            const Color& color)
        {
            const glm::dvec3 position =
                positionMeters * coordinateUnitsPerMeter;
            return {
                static_cast<float>(position.x),
                static_cast<float>(position.y),
                static_cast<float>(position.z),
                color
            };
        }

        void appendLine(
            std::vector<renderer::LineVertex>& vertices,
            const glm::dvec3& firstMeters,
            const glm::dvec3& secondMeters,
            const double coordinateUnitsPerMeter,
            const Color& color)
        {
            vertices.push_back(vertex(
                firstMeters, coordinateUnitsPerMeter, color));
            vertices.push_back(vertex(
                secondMeters, coordinateUnitsPerMeter, color));
        }

        void appendCarBox(
            std::vector<renderer::LineVertex>& vertices,
            const physics::CarPose& pose,
            const glm::dvec3& dimensionsMeters,
            const double coordinateUnitsPerMeter,
            const Color& color)
        {
            const glm::dvec3 half = 0.5 * dimensionsMeters;
            const std::array<glm::dvec3, 8> localCorners{{
                {-half.x, -half.y, -half.z},
                { half.x, -half.y, -half.z},
                { half.x,  half.y, -half.z},
                {-half.x,  half.y, -half.z},
                {-half.x, -half.y,  half.z},
                { half.x, -half.y,  half.z},
                { half.x,  half.y,  half.z},
                {-half.x,  half.y,  half.z}
            }};
            std::array<glm::dvec3, 8> worldCorners;
            std::ranges::transform(
                localCorners,
                worldCorners.begin(),
                [&pose](const glm::dvec3& corner)
                {
                    return pose.transformLocalPoint(corner);
                });

            constexpr std::array<std::array<std::size_t, 2>, 12> edges{{
                {0, 1}, {1, 2}, {2, 3}, {3, 0},
                {4, 5}, {5, 6}, {6, 7}, {7, 4},
                {0, 4}, {1, 5}, {2, 6}, {3, 7}
            }};
            for (const auto& edge : edges)
            {
                appendLine(
                    vertices,
                    worldCorners[edge[0]],
                    worldCorners[edge[1]],
                    coordinateUnitsPerMeter,
                    color);
            }
        }

        void appendBogieMarker(
            std::vector<renderer::LineVertex>& vertices,
            const physics::BogiePose& pose,
            const double coordinateUnitsPerMeter)
        {
            constexpr double halfWidthMeters = 0.4;
            constexpr double halfHeightMeters = 0.3;
            constexpr double forwardLengthMeters = 0.55;
            appendLine(vertices,
                pose.transformLocalPoint({0.0, -halfWidthMeters, 0.0}),
                pose.transformLocalPoint({0.0, halfWidthMeters, 0.0}),
                coordinateUnitsPerMeter, bogieColor);
            appendLine(vertices,
                pose.transformLocalPoint({0.0, 0.0, -halfHeightMeters}),
                pose.transformLocalPoint({0.0, 0.0, halfHeightMeters}),
                coordinateUnitsPerMeter, bogieColor);
            appendLine(vertices,
                pose.worldPositionMeters(),
                pose.transformLocalPoint({forwardLengthMeters, 0.0, 0.0}),
                coordinateUnitsPerMeter, bogieColor);
        }
    }

    physics::TrainPose interpolateTrainPreviewPose(
        const physics::CompiledPhysicsTrack& track,
        const physics::TrainDefinition& train,
        const physics::TrainPose& previous,
        const physics::TrainPose& current,
        const double alpha,
        SimulationPreviewFrameTelemetry* telemetry)
    {
        if (!std::isfinite(alpha))
        {
            throw std::invalid_argument("Preview interpolation alpha must be finite.");
        }
        const auto& from = previous.generalizedReferenceLocation();
        const auto& to = current.generalizedReferenceLocation();
        if (from.path != to.path || from.direction != to.direction)
        {
            throw std::invalid_argument(
                "Preview interpolation requires one path and physical orientation.");
        }
        if (alpha <= 0.0 || from == to)
        {
            return previous;
        }
        if (alpha >= 1.0)
        {
            return current;
        }

        double displacement = to.stationMeters - from.stationMeters;
        if (track.topology() == coaster::TopologyKind::ClosedCircuit)
        {
            // Adjacent fixed ticks describe local movement, not a lap change.
            displacement = std::remainder(displacement, track.lengthMeters());
        }
        auto location = track.advance(from, alpha * displacement).location;
        // advance records travel direction; the pose requires physical facing.
        location.direction = from.direction;
        const auto begin = std::chrono::steady_clock::now();
        if (telemetry)
        {
            ++telemetry->renderPoseSolveCount;
        }
        auto result = physics::solveTrainPose(
            track, train, location,
            telemetry ? &telemetry->solveCounters : nullptr);
        if (telemetry)
        {
            telemetry->renderPoseSolveMilliseconds +=
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - begin).count();
        }
        return result;
    }

    bool SimulationPreview::rebuild(
        const coaster::AuthoredTrack& authoredTrack) noexcept
    {
        setUnavailable({});
        try
        {
            trainDefinition_ = createPreviewTrainDefinition();
            const std::vector<coaster::TrackKinematicState> kinematics =
                coaster::integrateAuthoredTrackKinematics(
                    authoredTrack,
                    centerlineVisualizationSampleSpacing);
            compiledTrack_.emplace(
                kinematics,
                authoredTrack.physicalSettings(),
                authoredTrack.layoutMode(),
                coaster::computeTrackTopology(authoredTrack).kind);
            environment_ = physics::physicsEnvironmentFrom(
                authoredTrack.physicalSettings());
            coordinateUnitsPerMeter_ = 1.0
                / authoredTrack.physicalSettings().metersPerCoordinateUnit;

            InitialPlacement placement = findInitialPlacement(
                *compiledTrack_, trainDefinition_);
            initialPose_.emplace(std::move(placement.pose));

            physics::TrainDynamicsState initialState;
            initialState.generalizedReferenceLocation = placement.location;
            initialState.signedVelocityMetersPerSecond =
                authoredTrack.physicalSettings().initialSpeed;
            initialState.runState =
                initialState.signedVelocityMetersPerSecond == 0.0
                    ? physics::FollowerRunState::Resting
                    : physics::FollowerRunState::Running;
            initialState_.emplace(initialState);
            error_.clear();
            reset();
            return true;
        }
        catch (const std::exception& exception)
        {
            setUnavailable(
                "Simulation preview is unavailable: "
                + std::string(exception.what()));
            return false;
        }
    }

    void SimulationPreview::play() noexcept
    {
        if (!isAvailable())
        {
            return;
        }
        if (playbackState_ == PlaybackState::Stopped)
        {
            reset();
        }
        playbackState_ = PlaybackState::Playing;
    }

    void SimulationPreview::pause() noexcept
    {
        if (playbackState_ == PlaybackState::Playing)
        {
            playbackState_ = PlaybackState::Paused;
        }
    }

    void SimulationPreview::reset() noexcept
    {
        playbackState_ = PlaybackState::Stopped;
        accumulatorSeconds_ = 0.0;
        consecutiveCatchUpFrameCount_ = 0;
        renderAlpha_ = 0.0;
        if (!initialState_ || !initialPose_)
        {
            return;
        }
        dynamicsState_ = initialState_;
        previousState_ = initialState_;
        pose_ = initialPose_;
        previousPose_ = initialPose_;
        renderPose_ = initialPose_;
        rebuildVertices();
    }

    void SimulationPreview::beginFrameTelemetry() noexcept
    {
        frameTelemetry_ = {};
    }

    void SimulationPreview::update(
        const double frameDeltaSeconds,
        const bool timingDiscontinuity) noexcept
    {
        frameTelemetry_.rawDeltaMilliseconds = frameDeltaSeconds * 1000.0;
        frameTelemetry_.accumulatorBeforeMilliseconds =
            accumulatorSeconds_ * 1000.0;
        frameTelemetry_.accumulatorAfterIncomingMilliseconds =
            frameTelemetry_.accumulatorBeforeMilliseconds;
        frameTelemetry_.accumulatorRemainingMilliseconds =
            frameTelemetry_.accumulatorBeforeMilliseconds;

        if (playbackState_ != PlaybackState::Playing
            || !isAvailable()
            || !std::isfinite(frameDeltaSeconds)
            || frameDeltaSeconds <= 0.0)
        {
            consecutiveCatchUpFrameCount_ = 0;
            return;
        }

        if (timingDiscontinuity)
        {
            frameTelemetry_.discardedWallTimeMilliseconds =
                frameDeltaSeconds * 1000.0;
            consecutiveCatchUpFrameCount_ = 0;
            return;
        }

        constexpr double maximumAccumulatedSeconds =
            physics::defaultFixedTimeStepSeconds
            * static_cast<double>(maximumStepsPerFrame);
        const double uncappedAccumulatorSeconds =
            accumulatorSeconds_ + frameDeltaSeconds;
        accumulatorSeconds_ = std::min(
            uncappedAccumulatorSeconds, maximumAccumulatedSeconds);
        frameTelemetry_.accumulatorAfterIncomingMilliseconds =
            accumulatorSeconds_ * 1000.0;
        frameTelemetry_.discardedWallTimeMilliseconds = std::max(
            0.0,
            uncappedAccumulatorSeconds - maximumAccumulatedSeconds) * 1000.0;

        constexpr double accumulatorToleranceSeconds =
            physics::defaultFixedTimeStepSeconds * 1.0e-12;
        frameTelemetry_.requestedStepCount = static_cast<std::size_t>(
            (accumulatorSeconds_ + accumulatorToleranceSeconds)
                / physics::defaultFixedTimeStepSeconds);
        if (frameTelemetry_.requestedStepCount >= catchUpStepThreshold)
        {
            ++consecutiveCatchUpFrameCount_;
        }
        else
        {
            consecutiveCatchUpFrameCount_ = 0;
        }
        frameTelemetry_.consecutiveCatchUpFrameCount =
            consecutiveCatchUpFrameCount_;

        try
        {
            std::size_t stepCount = 0;
            double summedStepMilliseconds = 0.0;
            bool poseChanged = false;
            const auto physicsBegin = std::chrono::steady_clock::now();
            while (accumulatorSeconds_
                    >= physics::defaultFixedTimeStepSeconds
                        - accumulatorToleranceSeconds
                && stepCount < maximumStepsPerFrame)
            {
                const auto stepBegin = std::chrono::steady_clock::now();
                physics::TrainStepResult result = physics::stepTrain(
                    *compiledTrack_,
                    trainDefinition_,
                    environment_,
                    *dynamicsState_,
                    {},
                    {},
                    &frameTelemetry_.solveCounters);
                const double stepMilliseconds =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - stepBegin).count();
                summedStepMilliseconds += stepMilliseconds;
                frameTelemetry_.minimumStepMilliseconds = stepCount == 0
                    ? stepMilliseconds
                    : std::min(
                        frameTelemetry_.minimumStepMilliseconds,
                        stepMilliseconds);
                frameTelemetry_.maximumStepMilliseconds = std::max(
                    frameTelemetry_.maximumStepMilliseconds,
                    stepMilliseconds);
                previousState_ = dynamicsState_;
                previousPose_ = std::move(pose_);
                dynamicsState_ = result.state;
                pose_ = std::move(result.telemetry.pose);
                poseChanged = true;
                accumulatorSeconds_ = std::max(
                    0.0,
                    accumulatorSeconds_
                        - physics::defaultFixedTimeStepSeconds);
                ++stepCount;

                if (result.telemetry.boundaryIntervention)
                {
                    frameTelemetry_.boundaryStopped = true;
                    playbackState_ = PlaybackState::Paused;
                    accumulatorSeconds_ = 0.0;
                    break;
                }
            }
            const auto physicsEnd = std::chrono::steady_clock::now();
            frameTelemetry_.fixedStepCount += stepCount;
            frameTelemetry_.maximumStepsHit =
                stepCount == maximumStepsPerFrame;
            frameTelemetry_.averageStepMilliseconds = stepCount > 0
                ? summedStepMilliseconds / static_cast<double>(stepCount)
                : 0.0;
            frameTelemetry_.physicsMilliseconds +=
                std::chrono::duration<double, std::milli>(
                    physicsEnd - physicsBegin).count();
            frameTelemetry_.accumulatorRemainingMilliseconds =
                accumulatorSeconds_ * 1000.0;
            renderAlpha_ = playbackState_ == PlaybackState::Playing
                ? std::clamp(accumulatorSeconds_
                    / physics::defaultFixedTimeStepSeconds, 0.0, 1.0)
                : 1.0;
            if (poseChanged || previousState_->tick != dynamicsState_->tick)
            {
                const auto interpolationBegin = std::chrono::steady_clock::now();
                // Boundary intervention publishes Core's feasible endpoint.
                // Ordinary playback has one fixed tick of presentation latency.
                try
                {
                    renderPose_ = interpolateTrainPreviewPose(
                        *compiledTrack_, trainDefinition_, *previousPose_, *pose_,
                        interpolationAlpha(), &frameTelemetry_);
                }
                catch (const std::exception&)
                {
                    // A presentation failure cannot invalidate committed physics.
                    renderPose_ = pose_;
                    ++frameTelemetry_.renderPoseFailureCount;
                }
                frameTelemetry_.interpolationMilliseconds +=
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now()
                            - interpolationBegin).count();
                rebuildVertices();
            }
        }
        catch (const std::exception& exception)
        {
            setUnavailable(
                "Simulation preview stopped: "
                + std::string(exception.what()));
            frameTelemetry_.accumulatorRemainingMilliseconds =
                accumulatorSeconds_ * 1000.0;
        }
    }

    bool SimulationPreview::isAvailable() const noexcept
    {
        return compiledTrack_.has_value()
            && initialState_.has_value()
            && dynamicsState_.has_value()
            && initialPose_.has_value()
            && pose_.has_value()
            && error_.empty();
    }

    SimulationPreview::PlaybackState
    SimulationPreview::playbackState() const noexcept
    {
        return playbackState_;
    }

    double SimulationPreview::speedMetersPerSecond() const noexcept
    {
        return dynamicsState_
            ? std::abs(dynamicsState_->signedVelocityMetersPerSecond)
            : 0.0;
    }

    const std::string& SimulationPreview::error() const noexcept
    {
        return error_;
    }

    std::span<const renderer::LineVertex>
    SimulationPreview::vertices() const noexcept
    {
        return vertices_;
    }

    std::uint64_t SimulationPreview::vertexGeneration() const noexcept
    {
        return vertexGeneration_;
    }

    const SimulationPreviewFrameTelemetry&
    SimulationPreview::frameTelemetry() const noexcept
    {
        return frameTelemetry_;
    }

    const physics::TrainDefinition&
    SimulationPreview::trainDefinition() const noexcept
    {
        return trainDefinition_;
    }

    const physics::TrainDynamicsState*
    SimulationPreview::dynamicsState() const noexcept
    {
        return dynamicsState_ ? &*dynamicsState_ : nullptr;
    }

    const physics::TrainPose* SimulationPreview::pose() const noexcept
    {
        return pose_ ? &*pose_ : nullptr;
    }

    const physics::TrainPose* SimulationPreview::renderPose() const noexcept
    {
        return renderPose_ ? &*renderPose_ : nullptr;
    }

    double SimulationPreview::interpolationAlpha() const noexcept
    {
        return renderAlpha_;
    }

    void SimulationPreview::setUnavailable(std::string error) noexcept
    {
        compiledTrack_.reset();
        initialState_.reset();
        dynamicsState_.reset();
        previousState_.reset();
        initialPose_.reset();
        pose_.reset();
        previousPose_.reset();
        renderPose_.reset();
        renderAlpha_ = 0.0;
        vertices_.clear();
        ++vertexGeneration_;
        playbackState_ = PlaybackState::Stopped;
        accumulatorSeconds_ = 0.0;
        consecutiveCatchUpFrameCount_ = 0;
        error_ = std::move(error);
    }

    void SimulationPreview::rebuildVertices()
    {
        const auto preparationBegin = std::chrono::steady_clock::now();
        vertices_.clear();
        ++vertexGeneration_;
        if (!renderPose_ || renderPose_->carCount() != trainDefinition_.cars.size())
        {
            return;
        }

        vertices_.reserve(
            renderPose_->carCount() * 36 + renderPose_->connectionCount() * 2);
        for (std::size_t index = 0; index < renderPose_->carCount(); ++index)
        {
            const physics::CarPose& carPose =
                renderPose_->cars()[index].carPose();
            appendCarBox(
                vertices_,
                carPose,
                trainDefinition_.cars[index].car.bodyDimensionsMeters,
                coordinateUnitsPerMeter_,
                carColors[index % carColors.size()]);
            appendBogieMarker(
                vertices_, carPose.frontBogie(), coordinateUnitsPerMeter_);
            appendBogieMarker(
                vertices_, carPose.rearBogie(), coordinateUnitsPerMeter_);
        }

        for (const physics::InterCarConnectionPose& connection
            : renderPose_->connections())
        {
            appendLine(
                vertices_,
                connection.leadingEndpointWorldPositionMeters(),
                connection.followingEndpointWorldPositionMeters(),
                coordinateUnitsPerMeter_,
                connectorColor);
        }
        const auto preparationEnd = std::chrono::steady_clock::now();
        frameTelemetry_.vertexPreparationMilliseconds +=
            std::chrono::duration<double, std::milli>(
                preparationEnd - preparationBegin).count();
    }
}
