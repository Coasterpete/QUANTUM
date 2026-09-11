#include <quantum/editor/SimulationPreview.hpp>

#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/engine/Logging.hpp>

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

        void appendBogieMarkerFromGpuSample(
            std::vector<renderer::LineVertex>& vertices,
            const physics::gpu::PhysicsTrackSample& sample,
            double coordinateUnitsPerMeter)
        {
            glm::dvec3 pos{sample.position[0], sample.position[1], sample.position[2]};
            glm::dvec3 tangent{sample.tangent[0], sample.tangent[1], sample.tangent[2]};
            glm::dvec3 lateral{sample.lateral[0], sample.lateral[1], sample.lateral[2]};
            glm::dvec3 up{sample.up[0], sample.up[1], sample.up[2]};
            // GPU samples are already validated against CPU within tight tolerances
            geometry::CurveFrame frame{tangent, lateral, up};
            constexpr double halfWidthMeters = 0.4;
            constexpr double halfHeightMeters = 0.3;
            constexpr double forwardLengthMeters = 0.55;
            auto transform = [&](const glm::dvec3& local) {
                return pos + local.x * frame.tangent + local.y * frame.lateral + local.z * frame.up;
            };
            appendLine(vertices, transform({0.0, -halfWidthMeters, 0.0}), transform({0.0, halfWidthMeters, 0.0}), coordinateUnitsPerMeter, bogieColor);
            appendLine(vertices, transform({0.0, 0.0, -halfHeightMeters}), transform({0.0, 0.0, halfHeightMeters}), coordinateUnitsPerMeter, bogieColor);
            appendLine(vertices, pos, transform({forwardLengthMeters, 0.0, 0.0}), coordinateUnitsPerMeter, bogieColor);
        }

        [[nodiscard]] bool gpuQueriesMatchPose(
            const std::span<const physics::gpu::GpuTrackQuery> queries,
            const physics::TrainPose& pose) noexcept
        {
            if (queries.size() != pose.carCount() * 2)
            {
                return false;
            }

            std::size_t queryIndex = 0;
            for (const auto& car : pose.cars())
            {
                for (const auto* bogie : {
                    &car.carPose().frontBogie(),
                    &car.carPose().rearBogie()})
                {
                    const auto& query = queries[queryIndex++];
                    const auto& location = bogie->location();
                    const std::int32_t direction = location.direction
                            == physics::TravelDirection::IncreasingStation
                        ? 1
                        : -1;
                    if (query.coasterIndex != 0
                        || query.path != location.path.value
                        || query.direction != direction
                        || query.stationMeters != location.stationMeters)
                    {
                        return false;
                    }
                }
            }
            return true;
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
            const coaster::TopologyKind derivedTopology =
                coaster::computeTrackTopology(authoredTrack).kind;
            compiledTrack_.emplace(
                kinematics,
                authoredTrack.physicalSettings(),
                authoredTrack.layoutMode(),
                derivedTopology);
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

            // rebuild() is the preview's track-change boundary. Reuse the
            // kinematics that compiled the CPU track so the GPU receives the
            // same representation once per rebuild, never once per frame.
            gpuTrackReady_ = false;
            if (gpuContext_)
            {
                try
                {
                    gpuContext_->uploadTrack(
                        0,
                        kinematics,
                        authoredTrack.physicalSettings()
                            .metersPerCoordinateUnit,
                        physics::physicsTopologyForLayout(
                            authoredTrack.layoutMode(), derivedTopology),
                        authoredTrack.layoutMode());
                    gpuTrackReady_ = gpuContext_->gpuTrackReady();
                }
                catch (const std::exception& exception)
                {
                    quantum::logging::logMessagef(
                        quantum::logging::LogLevel::Warning,
                        "SIM",
                        "GPU preview track upload failed; using CPU markers: %s",
                        exception.what());
                }
            }
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
        lastGpuBogieSamples_.clear();
        lastGpuBogieQueries_.clear();
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
            // M2: batched GPU sampling for current pose's bogies (8 queries) – single production use
            // Collects all bogie TrackLocations for the committed pose before interpolation,
            // dispatches once via GpuPhysicsContext, validates against CPU.
            if (gpuContext_ && gpuTrackReady_ && pose_ && compiledTrack_ && poseChanged)
            {
                std::vector<physics::gpu::GpuTrackQuery> queries;
                queries.reserve(pose_->carCount() * 2);
                for (const auto& car : pose_->cars())
                {
                    for (const auto* bogie : {&car.carPose().frontBogie(), &car.carPose().rearBogie()})
                    {
                        const auto& loc = bogie->location();
                        physics::gpu::GpuTrackQuery q{};
                        q.coasterIndex = 0;
                        q.path = loc.path.value;
                        q.direction = (loc.direction == physics::TravelDirection::IncreasingStation ? 1 : -1);
                        q.stationMeters = loc.stationMeters;
                        queries.push_back(q);
                    }
                }
                if (!queries.empty())
                {
                    const auto gpuStart = std::chrono::steady_clock::now();
                    bool usedGpu = false;
                    std::vector<physics::gpu::PhysicsTrackSample> gpuSamples;
                    try
                    {
                        gpuSamples = gpuContext_->sampleTrackGpu(queries);
                        usedGpu = gpuContext_->lastSampleUsedGpu();
                    }
                    catch (...) { usedGpu = false; }
                    const double gpuMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - gpuStart).count();
                    if (usedGpu && gpuSamples.size() == queries.size())
                    {
                        ++gpuDispatchCount_;
                        gpuBatchedSampleCount_ += queries.size();
                        // M2 real consumption: retain GPU samples for rebuildVertices
                        lastGpuBogieSamples_ = gpuSamples;
                        lastGpuBogieQueries_ = queries;
                        auto cpuSamples = gpuContext_->sampleTrackForValidation(queries);
                        double maxPosErr = 0.0, maxStationErr = 0.0, maxCurvErr = 0.0;
                        double maxTdeg = 0.0, maxLdeg = 0.0, maxUdeg = 0.0;
                        for (size_t i = 0; i < gpuSamples.size(); ++i)
                        {
                            maxStationErr = std::max(maxStationErr, std::abs(gpuSamples[i].location.stationMeters - cpuSamples[i].location.stationMeters));
                            glm::dvec3 gp{gpuSamples[i].position[0], gpuSamples[i].position[1], gpuSamples[i].position[2]};
                            glm::dvec3 cp{cpuSamples[i].position[0], cpuSamples[i].position[1], cpuSamples[i].position[2]};
                            maxPosErr = std::max(maxPosErr, glm::length(gp - cp));
                            glm::dvec3 gcur{gpuSamples[i].curvature[0], gpuSamples[i].curvature[1], gpuSamples[i].curvature[2]};
                            glm::dvec3 ccur{cpuSamples[i].curvature[0], cpuSamples[i].curvature[1], cpuSamples[i].curvature[2]};
                            maxCurvErr = std::max(maxCurvErr, glm::length(gcur - ccur));
                            auto angleDeg = [](const glm::dvec3& a, const glm::dvec3& b) {
                                double la = glm::length(a), lb = glm::length(b);
                                if (la == 0 || lb == 0) return 0.0;
                                double c = glm::dot(a,b)/(la*lb);
                                c = std::clamp(c, -1.0, 1.0);
                                return glm::degrees(std::acos(c));
                            };
                            glm::dvec3 gt{gpuSamples[i].tangent[0], gpuSamples[i].tangent[1], gpuSamples[i].tangent[2]};
                            glm::dvec3 ct{cpuSamples[i].tangent[0], cpuSamples[i].tangent[1], cpuSamples[i].tangent[2]};
                            glm::dvec3 gl{gpuSamples[i].lateral[0], gpuSamples[i].lateral[1], gpuSamples[i].lateral[2]};
                            glm::dvec3 cl{cpuSamples[i].lateral[0], cpuSamples[i].lateral[1], cpuSamples[i].lateral[2]};
                            glm::dvec3 gu{gpuSamples[i].up[0], gpuSamples[i].up[1], gpuSamples[i].up[2]};
                            glm::dvec3 cu{cpuSamples[i].up[0], cpuSamples[i].up[1], cpuSamples[i].up[2]};
                            maxTdeg = std::max(maxTdeg, angleDeg(gt, ct));
                            maxLdeg = std::max(maxLdeg, angleDeg(gl, cl));
                            maxUdeg = std::max(maxUdeg, angleDeg(gu, cu));
                        }
                        if (maxPosErr > 1e-6 || maxStationErr > 1e-9 || maxCurvErr > 1e-9 || maxTdeg > 0.01 || maxLdeg > 0.01 || maxUdeg > 0.01)
                        {
                            quantum::logging::logMessagef(quantum::logging::LogLevel::Warning, "SIM",
                                "GPU batch mismatch pos=%.3e station=%.3e curv=%.3e t=%.4fdeg l=%.4fdeg u=%.4fdeg gpuMs=%.2f",
                                maxPosErr, maxStationErr, maxCurvErr, maxTdeg, maxLdeg, maxUdeg, gpuMs);
                        }
                    }
                    else
                    {
                        ++cpuFallbackCount_;
                        lastGpuBogieSamples_.clear();
                        lastGpuBogieQueries_.clear();
                    }
                }
                else
                {
                    lastGpuBogieSamples_.clear();
                    lastGpuBogieQueries_.clear();
                }
            }
            else
            {
                lastGpuBogieSamples_.clear();
                lastGpuBogieQueries_.clear();
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
        lastGpuBogieSamples_.clear();
        lastGpuBogieQueries_.clear();
        gpuTrackReady_ = false;
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
        // M2: GPU samples are actually consumed for bogie markers when available
        const bool useGpuBogieMarkers = gpuContext_ && gpuTrackReady_
            && lastGpuBogieSamples_.size() == renderPose_->carCount() * 2
            && gpuQueriesMatchPose(lastGpuBogieQueries_, *renderPose_);
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
            if (useGpuBogieMarkers)
            {
                appendBogieMarkerFromGpuSample(vertices_, lastGpuBogieSamples_[index * 2], coordinateUnitsPerMeter_);
                appendBogieMarkerFromGpuSample(vertices_, lastGpuBogieSamples_[index * 2 + 1], coordinateUnitsPerMeter_);
            }
            else
            {
                appendBogieMarker(
                    vertices_, carPose.frontBogie(), coordinateUnitsPerMeter_);
                appendBogieMarker(
                    vertices_, carPose.rearBogie(), coordinateUnitsPerMeter_);
            }
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

    void SimulationPreview::setGpuContext(physics::gpu::GpuPhysicsContext* gpu) noexcept
    {
        gpuContext_ = gpu;
        gpuTrackReady_ = false;
        lastGpuBogieSamples_.clear();
        lastGpuBogieQueries_.clear();
    }

    bool SimulationPreview::hasGpuContext() const noexcept
    {
        return gpuContext_ != nullptr;
    }
}
