#include <quantum/physics/TrackFollower.hpp>

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace quantum::physics
{
    namespace
    {
        [[nodiscard]] bool finite(const glm::dvec3& value) noexcept
        {
            return std::isfinite(value.x)
                && std::isfinite(value.y)
                && std::isfinite(value.z);
        }

        [[nodiscard]] bool validTopology(
            const coaster::TopologyKind topology) noexcept
        {
            switch (topology)
            {
            case coaster::TopologyKind::OpenLinear:
            case coaster::TopologyKind::ClosedCircuit:
                return true;
            }
            return false;
        }

        [[nodiscard]] bool validDirection(
            const TravelDirection direction) noexcept
        {
            switch (direction)
            {
            case TravelDirection::DecreasingStation:
            case TravelDirection::IncreasingStation:
                return true;
            }
            return false;
        }

        [[nodiscard]] bool validRunState(
            const FollowerRunState state) noexcept
        {
            switch (state)
            {
            case FollowerRunState::Running:
            case FollowerRunState::Resting:
            case FollowerRunState::StoppedAtStart:
            case FollowerRunState::StoppedAtEnd:
                return true;
            }
            return false;
        }

        [[nodiscard]] glm::dquat frameOrientation(
            const geometry::CurveFrame& frame)
        {
            return glm::normalize(glm::quat_cast(glm::dmat3{
                frame.tangent,
                frame.lateral,
                frame.up
            }));
        }

        [[nodiscard]] geometry::CurveFrame interpolateFrame(
            const geometry::CurveFrame& before,
            const geometry::CurveFrame& after,
            const double amount)
        {
            glm::dquat beforeOrientation = frameOrientation(before);
            glm::dquat afterOrientation = frameOrientation(after);
            if (glm::dot(beforeOrientation, afterOrientation) < 0.0)
            {
                afterOrientation = -afterOrientation;
            }

            const glm::dmat3 rotation = glm::mat3_cast(glm::normalize(
                glm::slerp(beforeOrientation, afterOrientation, amount)));
            return {rotation[0], rotation[1], rotation[2]};
        }

        struct ForceBreakdown
        {
            double gravityForceNewtons = 0.0;
            double resistanceForceNewtons = 0.0;
        };

        [[nodiscard]] ForceBreakdown evaluateForces(
            const SingleFollowerDefinition& definition,
            const PhysicsEnvironment& environment,
            const geometry::CurveFrame& frame,
            const double velocityMetersPerSecond)
        {
            const glm::dvec3 gravityWorld{
                0.0,
                0.0,
                -environment.gravityAccelerationMetersPerSecondSquared
            };
            const double gravityAcceleration =
                glm::dot(gravityWorld, frame.tangent);
            const double gravityForce =
                definition.massKilograms * gravityAcceleration;

            if (!std::isfinite(gravityForce))
            {
                throw std::invalid_argument(
                    "Follower force inputs produce a non-finite force.");
            }
            const double resistanceForce =
                evaluateBasicResistanceForceNewtons(
                    definition.resistance,
                    definition.massKilograms,
                    environment.gravityAccelerationMetersPerSecondSquared,
                    gravityForce,
                    velocityMetersPerSecond);

            return {gravityForce, resistanceForce};
        }

        [[nodiscard]] TrackBoundary boundaryForState(
            const CompiledPhysicsTrack& track,
            const TrackFollowerState& state,
            const TrackBoundary advancedBoundary) noexcept
        {
            if (advancedBoundary != TrackBoundary::None)
            {
                return advancedBoundary;
            }
            if (track.topology() == coaster::TopologyKind::OpenLinear)
            {
                if (state.runState == FollowerRunState::StoppedAtStart)
                {
                    return TrackBoundary::Start;
                }
                if (state.runState == FollowerRunState::StoppedAtEnd)
                {
                    return TrackBoundary::End;
                }
            }
            return TrackBoundary::None;
        }

        [[nodiscard]] double validatedCoordinateScale(
            const coaster::TrackPhysicalSettings& settings)
        {
            coaster::validateTrackPhysicalSettings(settings);
            return settings.metersPerCoordinateUnit;
        }
    }

    coaster::TopologyKind physicsTopologyForLayout(
        const coaster::LayoutMode layoutMode,
        const coaster::TopologyKind derivedTopology)
    {
        if (!validTopology(derivedTopology))
        {
            throw std::invalid_argument(
                "Derived physics track topology is invalid.");
        }

        switch (layoutMode)
        {
        case coaster::LayoutMode::Circuit:
            return derivedTopology;
        case coaster::LayoutMode::Shuttle:
            return coaster::TopologyKind::OpenLinear;
        }
        throw std::invalid_argument("Authored physics layout mode is invalid.");
    }

    CompiledPhysicsTrack::CompiledPhysicsTrack(
        const std::span<const coaster::TrackKinematicState> kinematics,
        const double metersPerCoordinateUnit,
        const coaster::TopologyKind topology)
        : topology_(topology)
    {
        if (!std::isfinite(metersPerCoordinateUnit)
            || metersPerCoordinateUnit <= 0.0)
        {
            throw std::invalid_argument(
                "Physics track coordinate scale must be positive and finite.");
        }
        if (!validTopology(topology))
        {
            throw std::invalid_argument("Physics track topology is invalid.");
        }
        if (kinematics.size() < 2)
        {
            throw std::invalid_argument(
                "Physics track compilation requires at least two canonical samples.");
        }

        samples_.reserve(kinematics.size());
        double previousDistance = 0.0;
        for (std::size_t index = 0; index < kinematics.size(); ++index)
        {
            const coaster::TrackKinematicState& state = kinematics[index];
            if (!std::isfinite(state.distance)
                || !finite(state.position)
                || !finite(state.frame.tangent)
                || !finite(state.frame.lateral)
                || !finite(state.frame.up)
                || !finite(state.centerlineCurvature))
            {
                throw std::invalid_argument(
                    "Physics track canonical samples must be finite.");
            }
            if ((index == 0 && state.distance != 0.0)
                || (index != 0 && state.distance <= previousDistance))
            {
                throw std::invalid_argument(
                    "Physics track canonical distances must start at zero and strictly increase.");
            }

            geometry::detail::validateCurveFrameForRotation(
                state.frame, "physics track compilation");

            Sample converted{
                state.distance * metersPerCoordinateUnit,
                state.position * metersPerCoordinateUnit,
                state.frame,
                state.centerlineCurvature / metersPerCoordinateUnit
            };
            if (!std::isfinite(converted.stationMeters)
                || !finite(converted.positionMeters)
                || !finite(converted.curvaturePerMeter)
                || (!samples_.empty()
                    && converted.stationMeters <= samples_.back().stationMeters))
            {
                throw std::invalid_argument(
                    "Physics track SI conversion must remain finite and strictly increasing.");
            }

            samples_.push_back(converted);
            previousDistance = state.distance;
        }

        lengthMeters_ = samples_.back().stationMeters;
        if (!std::isfinite(lengthMeters_) || lengthMeters_ <= 0.0)
        {
            throw std::invalid_argument(
                "Physics track length must be positive and finite.");
        }
    }

    CompiledPhysicsTrack::CompiledPhysicsTrack(
        const std::span<const coaster::TrackKinematicState> kinematics,
        const coaster::TrackPhysicalSettings& physicalSettings,
        const coaster::LayoutMode layoutMode,
        const coaster::TopologyKind derivedTopology)
        : CompiledPhysicsTrack(
            kinematics,
            validatedCoordinateScale(physicalSettings),
            physicsTopologyForLayout(layoutMode, derivedTopology))
    {
    }

    double CompiledPhysicsTrack::lengthMeters() const noexcept
    {
        return lengthMeters_;
    }

    coaster::TopologyKind CompiledPhysicsTrack::topology() const noexcept
    {
        return topology_;
    }

    void CompiledPhysicsTrack::validateLocation(
        const TrackLocation& location) const
    {
        if (location.path != primaryTrackPathId)
        {
            throw std::invalid_argument(
                "Phase 1 physics supports only the primary track path.");
        }
        if (!validDirection(location.direction))
        {
            throw std::invalid_argument("Track travel direction is invalid.");
        }
        if (!std::isfinite(location.stationMeters)
            || location.stationMeters < 0.0
            || (topology_ == coaster::TopologyKind::ClosedCircuit
                ? location.stationMeters >= lengthMeters_
                : location.stationMeters > lengthMeters_))
        {
            throw std::invalid_argument(
                "Track location station is outside the compiled path.");
        }
    }

    PhysicsTrackSample CompiledPhysicsTrack::sample(
        const TrackLocation& location) const
    {
        validateLocation(location);

        const auto upper = std::lower_bound(
            samples_.begin(),
            samples_.end(),
            location.stationMeters,
            [](const Sample& sample, const double station)
            {
                return sample.stationMeters < station;
            });

        if (upper == samples_.begin())
        {
            return {
                location,
                upper->positionMeters,
                upper->frame,
                upper->curvaturePerMeter
            };
        }
        if (upper == samples_.end())
        {
            const Sample& finalSample = samples_.back();
            return {
                location,
                finalSample.positionMeters,
                finalSample.frame,
                finalSample.curvaturePerMeter
            };
        }

        const Sample& after = *upper;
        const Sample& before = *(upper - 1);
        const double amount = std::clamp(
            (location.stationMeters - before.stationMeters)
                / (after.stationMeters - before.stationMeters),
            0.0,
            1.0);

        return {
            location,
            glm::mix(before.positionMeters, after.positionMeters, amount),
            interpolateFrame(before.frame, after.frame, amount),
            glm::mix(before.curvaturePerMeter, after.curvaturePerMeter, amount)
        };
    }

    TrackAdvanceResult CompiledPhysicsTrack::advance(
        const TrackLocation& location,
        const double signedDistanceMeters) const
    {
        validateLocation(location);
        if (!std::isfinite(signedDistanceMeters))
        {
            throw std::invalid_argument(
                "Track advancement distance must be finite.");
        }

        TrackAdvanceResult result;
        result.location = location;
        if (signedDistanceMeters > 0.0)
        {
            result.location.direction = TravelDirection::IncreasingStation;
        }
        else if (signedDistanceMeters < 0.0)
        {
            result.location.direction = TravelDirection::DecreasingStation;
        }

        if (topology_ == coaster::TopologyKind::ClosedCircuit)
        {
            result.wrapped = signedDistanceMeters > 0.0
                ? signedDistanceMeters >= lengthMeters_ - location.stationMeters
                : signedDistanceMeters < -location.stationMeters;

            const double reducedDistance =
                std::fmod(signedDistanceMeters, lengthMeters_);
            double station = std::fmod(
                location.stationMeters + reducedDistance,
                lengthMeters_);
            if (station < 0.0)
            {
                station += lengthMeters_;
            }
            if (station == lengthMeters_ || station == -0.0)
            {
                station = 0.0;
            }
            result.location.stationMeters = station;
            return result;
        }

        if (signedDistanceMeters > 0.0
            && signedDistanceMeters >= lengthMeters_ - location.stationMeters)
        {
            result.location.stationMeters = lengthMeters_;
            result.boundary = TrackBoundary::End;
        }
        else if (signedDistanceMeters < 0.0
            && signedDistanceMeters <= -location.stationMeters)
        {
            result.location.stationMeters = 0.0;
            result.boundary = TrackBoundary::Start;
        }
        else
        {
            result.location.stationMeters += signedDistanceMeters;
        }

        return result;
    }

    PhysicsEnvironment physicsEnvironmentFrom(
        const coaster::TrackPhysicalSettings& settings)
    {
        coaster::validateTrackPhysicalSettings(settings);
        return {settings.gravityAcceleration};
    }

    void validateSingleFollowerDefinition(
        const SingleFollowerDefinition& definition)
    {
        if (!std::isfinite(definition.massKilograms)
            || definition.massKilograms <= 0.0)
        {
            throw std::invalid_argument(
                "Follower mass must be positive and finite.");
        }

        validateBasicResistance(definition.resistance);
    }

    void validateBasicResistance(const BasicResistance& resistance)
    {
        if (!std::isfinite(resistance.constantMechanicalForceNewtons)
            || resistance.constantMechanicalForceNewtons < 0.0
            || !std::isfinite(
                resistance.linearResistanceCoefficientNewtonSecondsPerMeter)
            || resistance.linearResistanceCoefficientNewtonSecondsPerMeter < 0.0
            || !std::isfinite(resistance.airDensityKilogramsPerCubicMeter)
            || resistance.airDensityKilogramsPerCubicMeter < 0.0
            || !std::isfinite(resistance.dragAreaSquareMeters)
            || resistance.dragAreaSquareMeters < 0.0
            || !std::isfinite(resistance.rollingResistanceCoefficient)
            || resistance.rollingResistanceCoefficient < 0.0)
        {
            throw std::invalid_argument(
                "Follower resistance values must be finite and non-negative.");
        }
    }

    double evaluateBasicResistanceForceNewtons(
        const BasicResistance& resistance,
        const double supportedMassKilograms,
        const double gravityAccelerationMetersPerSecondSquared,
        const double impendingForceNewtons,
        const double velocityMetersPerSecond)
    {
        validateBasicResistance(resistance);
        if (!std::isfinite(supportedMassKilograms)
            || supportedMassKilograms <= 0.0
            || !std::isfinite(gravityAccelerationMetersPerSecondSquared)
            || gravityAccelerationMetersPerSecondSquared <= 0.0
            || !std::isfinite(impendingForceNewtons)
            || !std::isfinite(velocityMetersPerSecond))
        {
            throw std::invalid_argument(
                "Basic resistance evaluation inputs must be finite and physically valid.");
        }

        const double rollingForceMagnitude =
            resistance.rollingResistanceCoefficient
            * supportedMassKilograms
            * gravityAccelerationMetersPerSecondSquared;
        const double dryForceMagnitude =
            resistance.constantMechanicalForceNewtons
            + rollingForceMagnitude;
        if (!std::isfinite(rollingForceMagnitude)
            || !std::isfinite(dryForceMagnitude))
        {
            throw std::invalid_argument(
                "Basic resistance inputs produce a non-finite force.");
        }

        double resistanceForce = 0.0;
        if (std::abs(velocityMetersPerSecond)
            <= followerRestSpeedToleranceMetersPerSecond)
        {
            // Dry resistance opposes impending non-resistance motion and
            // statically balances it when the available capacity permits.
            resistanceForce = std::abs(impendingForceNewtons)
                    <= dryForceMagnitude
                ? -impendingForceNewtons
                : -std::copysign(dryForceMagnitude, impendingForceNewtons);
        }
        else
        {
            const double motionSign =
                std::copysign(1.0, velocityMetersPerSecond);
            const double linearForce =
                -resistance.linearResistanceCoefficientNewtonSecondsPerMeter
                * velocityMetersPerSecond;
            const double aerodynamicForce =
                -0.5
                * resistance.airDensityKilogramsPerCubicMeter
                * resistance.dragAreaSquareMeters
                * velocityMetersPerSecond
                * std::abs(velocityMetersPerSecond);
            resistanceForce =
                -motionSign * dryForceMagnitude
                + linearForce
                + aerodynamicForce;
        }
        if (!std::isfinite(resistanceForce))
        {
            throw std::invalid_argument(
                "Basic resistance inputs produce a non-finite force.");
        }
        return resistanceForce;
    }

    void validatePhysicsEnvironment(const PhysicsEnvironment& environment)
    {
        if (!std::isfinite(
                environment.gravityAccelerationMetersPerSecondSquared)
            || environment.gravityAccelerationMetersPerSecondSquared <= 0.0)
        {
            throw std::invalid_argument(
                "Physics gravity must be positive and finite.");
        }
    }

    void validateFixedStepSettings(const FixedStepSettings& settings)
    {
        if (!std::isfinite(settings.deltaTimeSeconds)
            || settings.deltaTimeSeconds <= 0.0)
        {
            throw std::invalid_argument(
                "Physics fixed timestep must be positive and finite.");
        }
    }

    TrackFollowerStepResult stepTrackFollower(
        const CompiledPhysicsTrack& track,
        const SingleFollowerDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrackFollowerState& currentState,
        const FixedStepSettings& step)
    {
        validateSingleFollowerDefinition(definition);
        validatePhysicsEnvironment(environment);
        validateFixedStepSettings(step);

        if (!std::isfinite(currentState.signedVelocityMetersPerSecond)
            || !std::isfinite(
                currentState.longitudinalAccelerationMetersPerSecondSquared)
            || !validRunState(currentState.runState))
        {
            throw std::invalid_argument(
                "Track follower state must be finite and valid.");
        }
        if (currentState.tick == std::numeric_limits<std::uint64_t>::max())
        {
            throw std::overflow_error("Track follower tick overflow.");
        }

        const PhysicsTrackSample currentSample =
            track.sample(currentState.location);
        double workingVelocity = currentState.signedVelocityMetersPerSecond;
        if (std::abs(workingVelocity)
            <= followerRestSpeedToleranceMetersPerSecond)
        {
            workingVelocity = 0.0;
        }

        // Reject retained outward velocity before force evaluation so an open
        // endpoint never advances incorrectly on the following step.
        if (track.topology() == coaster::TopologyKind::OpenLinear)
        {
            if ((currentState.location.stationMeters == 0.0
                    && workingVelocity < 0.0)
                || (currentState.location.stationMeters == track.lengthMeters()
                    && workingVelocity > 0.0))
            {
                workingVelocity = 0.0;
            }
        }

        const ForceBreakdown forces = evaluateForces(
            definition, environment, currentSample.frame, workingVelocity);
        const double unconstrainedForce =
            forces.gravityForceNewtons + forces.resistanceForceNewtons;
        const double unconstrainedAcceleration =
            unconstrainedForce / definition.massKilograms;
        double nextVelocity = workingVelocity
            + unconstrainedAcceleration * step.deltaTimeSeconds;
        if (!std::isfinite(unconstrainedForce)
            || !std::isfinite(unconstrainedAcceleration)
            || !std::isfinite(nextVelocity))
        {
            throw std::invalid_argument(
                "Follower step produced a non-finite velocity or acceleration.");
        }

        // A real force that persists through rest is reconsidered on the next
        // exact tick. Dry or velocity-dependent resistance therefore cannot
        // push the follower across zero or create an overshoot displacement.
        if ((workingVelocity > 0.0 && nextVelocity <= 0.0)
            || (workingVelocity < 0.0 && nextVelocity >= 0.0)
            || std::abs(nextVelocity)
                <= followerRestSpeedToleranceMetersPerSecond)
        {
            nextVelocity = 0.0;
        }

        const double signedDistance =
            nextVelocity * step.deltaTimeSeconds;
        if (!std::isfinite(signedDistance))
        {
            throw std::invalid_argument(
                "Follower step produced a non-finite advancement distance.");
        }

        TrackAdvanceResult advancement = track.advance(
            currentState.location, signedDistance);

        if (track.topology() == coaster::TopologyKind::OpenLinear
            && ((advancement.location.stationMeters == 0.0
                    && nextVelocity < 0.0)
                || (advancement.location.stationMeters == track.lengthMeters()
                    && nextVelocity > 0.0)))
        {
            nextVelocity = 0.0;
        }

        TrackFollowerState nextState;
        nextState.location = advancement.location;
        nextState.signedVelocityMetersPerSecond = nextVelocity;
        nextState.longitudinalAccelerationMetersPerSecondSquared =
            (nextVelocity - currentState.signedVelocityMetersPerSecond)
            / step.deltaTimeSeconds;
        nextState.tick = currentState.tick + 1;

        if (nextVelocity != 0.0)
        {
            nextState.runState = FollowerRunState::Running;
        }
        else if (track.topology() == coaster::TopologyKind::OpenLinear
            && nextState.location.stationMeters == 0.0)
        {
            nextState.runState = FollowerRunState::StoppedAtStart;
        }
        else if (track.topology() == coaster::TopologyKind::OpenLinear
            && nextState.location.stationMeters == track.lengthMeters())
        {
            nextState.runState = FollowerRunState::StoppedAtEnd;
        }
        else
        {
            nextState.runState = FollowerRunState::Resting;
        }

        const double totalForce = definition.massKilograms
            * nextState.longitudinalAccelerationMetersPerSecondSquared;
        const double constraintForce = totalForce - unconstrainedForce;
        const double simulationTime =
            static_cast<double>(nextState.tick) * step.deltaTimeSeconds;
        if (!std::isfinite(totalForce)
            || !std::isfinite(constraintForce)
            || !std::isfinite(simulationTime))
        {
            throw std::invalid_argument(
                "Follower telemetry is not representable as finite SI values.");
        }

        const PhysicsTrackSample committedSample =
            track.sample(nextState.location);
        const double curvatureMagnitude =
            glm::length(committedSample.curvaturePerMeter);
        const double curvatureNormalAcceleration =
            nextVelocity * nextVelocity * curvatureMagnitude;

        TrackFollowerTelemetry telemetry;
        telemetry.tick = nextState.tick;
        telemetry.simulationTimeSeconds = simulationTime;
        telemetry.location = nextState.location;
        telemetry.worldPositionMeters = committedSample.positionMeters;
        telemetry.frame = committedSample.frame;
        telemetry.signedSpeedMetersPerSecond = nextVelocity;
        telemetry.longitudinalAccelerationMetersPerSecondSquared =
            nextState.longitudinalAccelerationMetersPerSecondSquared;
        telemetry.massKilograms = definition.massKilograms;
        telemetry.gravityForceNewtons = forces.gravityForceNewtons;
        telemetry.resistanceForceNewtons = forces.resistanceForceNewtons;
        telemetry.constraintForceNewtons = constraintForce;
        telemetry.totalLongitudinalForceNewtons = totalForce;
        telemetry.gravityAccelerationMetersPerSecondSquared =
            forces.gravityForceNewtons / definition.massKilograms;
        telemetry.resistanceAccelerationMetersPerSecondSquared =
            forces.resistanceForceNewtons / definition.massKilograms;
        telemetry.totalLongitudinalAccelerationMetersPerSecondSquared =
            nextState.longitudinalAccelerationMetersPerSecondSquared;
        telemetry.curvaturePerMeter = committedSample.curvaturePerMeter;
        telemetry.curvatureMagnitudePerMeter = curvatureMagnitude;
        telemetry.curvatureNormalAccelerationMagnitudeMetersPerSecondSquared =
            curvatureNormalAcceleration;
        telemetry.runState = nextState.runState;
        telemetry.boundary = boundaryForState(
            track, nextState, advancement.boundary);
        telemetry.wrapped = advancement.wrapped;

        return {nextState, telemetry};
    }
}
