#include <quantum/editor/SimulationPreview.hpp>

#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>

#include <algorithm>
#include <array>
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
        if (!initialState_ || !initialPose_)
        {
            return;
        }
        dynamicsState_ = initialState_;
        pose_ = initialPose_;
        rebuildVertices();
    }

    void SimulationPreview::update(const double frameDeltaSeconds) noexcept
    {
        if (playbackState_ != PlaybackState::Playing
            || !isAvailable()
            || !std::isfinite(frameDeltaSeconds)
            || frameDeltaSeconds <= 0.0)
        {
            return;
        }

        constexpr double maximumAccumulatedSeconds =
            physics::defaultFixedTimeStepSeconds
            * static_cast<double>(maximumStepsPerFrame);
        accumulatorSeconds_ = std::min(
            accumulatorSeconds_ + frameDeltaSeconds,
            maximumAccumulatedSeconds);

        try
        {
            constexpr double accumulatorToleranceSeconds =
                physics::defaultFixedTimeStepSeconds * 1.0e-12;
            std::size_t stepCount = 0;
            bool poseChanged = false;
            while (accumulatorSeconds_
                    >= physics::defaultFixedTimeStepSeconds
                        - accumulatorToleranceSeconds
                && stepCount < maximumStepsPerFrame)
            {
                physics::TrainStepResult result = physics::stepTrain(
                    *compiledTrack_,
                    trainDefinition_,
                    environment_,
                    *dynamicsState_);
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
                    playbackState_ = PlaybackState::Paused;
                    accumulatorSeconds_ = 0.0;
                    break;
                }
            }
            if (poseChanged)
            {
                rebuildVertices();
            }
        }
        catch (const std::exception& exception)
        {
            setUnavailable(
                "Simulation preview stopped: "
                + std::string(exception.what()));
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

    void SimulationPreview::setUnavailable(std::string error) noexcept
    {
        compiledTrack_.reset();
        initialState_.reset();
        dynamicsState_.reset();
        initialPose_.reset();
        pose_.reset();
        vertices_.clear();
        ++vertexGeneration_;
        playbackState_ = PlaybackState::Stopped;
        accumulatorSeconds_ = 0.0;
        error_ = std::move(error);
    }

    void SimulationPreview::rebuildVertices()
    {
        vertices_.clear();
        ++vertexGeneration_;
        if (!pose_ || pose_->carCount() != trainDefinition_.cars.size())
        {
            return;
        }

        vertices_.reserve(
            pose_->carCount() * 36 + pose_->connectionCount() * 2);
        for (std::size_t index = 0; index < pose_->carCount(); ++index)
        {
            const physics::CarPose& carPose =
                pose_->cars()[index].carPose();
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
            : pose_->connections())
        {
            appendLine(
                vertices_,
                connection.leadingEndpointWorldPositionMeters(),
                connection.followingEndpointWorldPositionMeters(),
                coordinateUnitsPerMeter_,
                connectorColor);
        }
    }
}
