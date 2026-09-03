#include <quantum/physics/TrainPhysics.hpp>

#include <glm/common.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace quantum::physics
{
    namespace
    {
        inline constexpr std::size_t connectorSearchSampleCount = 160;
        inline constexpr std::size_t connectorRefinementIterationCount = 80;
        inline constexpr std::size_t boundaryRefinementIterationCount = 64;

        class OpenConsistBoundaryError final : public std::domain_error
        {
        public:
            using std::domain_error::domain_error;
        };

        [[nodiscard]] bool finite(const glm::dvec3& value) noexcept
        {
            return std::isfinite(value.x)
                && std::isfinite(value.y)
                && std::isfinite(value.z);
        }

        [[nodiscard]] bool finite(const glm::dquat& value) noexcept
        {
            return std::isfinite(value.w)
                && std::isfinite(value.x)
                && std::isfinite(value.y)
                && std::isfinite(value.z);
        }

        [[nodiscard]] double directionSign(const TravelDirection direction)
        {
            switch (direction)
            {
            case TravelDirection::IncreasingStation:
                return 1.0;
            case TravelDirection::DecreasingStation:
                return -1.0;
            }
            throw std::invalid_argument("Train travel direction is invalid.");
        }

        [[nodiscard]] bool validRunState(const FollowerRunState state) noexcept
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

        void requireLegalOpenCarPlacement(
            const CompiledPhysicsTrack& track,
            const CarDefinition& definition,
            const TrackLocation& referenceLocation)
        {
            if (track.topology() != coaster::TopologyKind::OpenLinear)
            {
                return;
            }

            const double sign = directionSign(referenceLocation.direction);
            for (const BogieDefinition& bogie : definition.bogies)
            {
                const double station = referenceLocation.stationMeters
                    + sign * bogie.referencePositionMeters.x;
                if (!std::isfinite(station)
                    || station < 0.0
                    || station > track.lengthMeters())
                {
                    throw OpenConsistBoundaryError(
                        "A train bogie would lie beyond an open track endpoint.");
                }
            }
        }

        [[nodiscard]] CarPose solveLegalCarPose(
            const CompiledPhysicsTrack& track,
            const TrainCarDefinition& definition,
            const TrackLocation& referenceLocation)
        {
            requireLegalOpenCarPlacement(
                track, definition.car, referenceLocation);
            return solveCarPose(
                track, definition.car, referenceLocation, definition.loadout);
        }

        struct ConnectionCandidate
        {
            double backwardOffsetMeters = 0.0;
            CarPose followingPose;
            double distanceMeters = 0.0;
            double residualMeters = 0.0;
        };

        [[nodiscard]] ConnectionCandidate connectionCandidate(
            const CompiledPhysicsTrack& track,
            const TrainCarDefinition& followingDefinition,
            const CarPose& leadingPose,
            const double connectorLengthMeters,
            const double backwardOffsetMeters)
        {
            if (!std::isfinite(backwardOffsetMeters)
                || backwardOffsetMeters < 0.0)
            {
                throw std::invalid_argument(
                    "A connector search offset must be finite and non-negative.");
            }

            const double sign = directionSign(
                leadingPose.referenceLocation().direction);
            TrackLocation followingLocation = track.advance(
                leadingPose.referenceLocation(),
                -sign * backwardOffsetMeters).location;
            followingLocation.direction =
                leadingPose.referenceLocation().direction;

            CarPose followingPose = solveLegalCarPose(
                track, followingDefinition, followingLocation);
            const double distance = glm::length(
                followingPose.frontHitchWorldPositionMeters()
                - leadingPose.rearHitchWorldPositionMeters());
            const double residual = distance - connectorLengthMeters;
            if (!std::isfinite(distance) || !std::isfinite(residual))
            {
                throw std::domain_error(
                    "Connector evaluation produced a non-finite distance.");
            }
            return {
                backwardOffsetMeters,
                std::move(followingPose),
                distance,
                residual
            };
        }

        [[nodiscard]] double carLongitudinalScale(
            const CarDefinition& definition) noexcept
        {
            double scale = std::abs(definition.bodyDimensionsMeters.x);
            scale = std::max(scale,
                std::abs(definition.frontHitchPositionMeters.x));
            scale = std::max(scale,
                std::abs(definition.rearHitchPositionMeters.x));
            for (const BogieDefinition& bogie : definition.bogies)
            {
                scale = std::max(
                    scale, std::abs(bogie.referencePositionMeters.x));
            }
            return scale;
        }

        struct SolvedFollowingCar
        {
            CarPose pose;
            std::size_t iterationCount = 0;
            double finalBracketSizeMeters = 0.0;
        };

        [[nodiscard]] SolvedFollowingCar solveFollowingCar(
            const CompiledPhysicsTrack& track,
            const TrainCarDefinition& leadingDefinition,
            const CarPose& leadingPose,
            const TrainCarDefinition& followingDefinition,
            const InterCarConnectionDefinition& connection)
        {
            const double baseSeparation =
                followingDefinition.car.frontHitchPositionMeters.x
                - leadingDefinition.car.rearHitchPositionMeters.x;
            const double expectedOffset = std::max(
                0.0, baseSeparation + connection.rigidLengthMeters);
            const double geometryScale = std::max({
                1.0,
                std::abs(baseSeparation),
                connection.rigidLengthMeters,
                carLongitudinalScale(leadingDefinition.car),
                carLongitudinalScale(followingDefinition.car)
            });
            const double searchHalfExtent = std::max(
                0.5, 1.1 * geometryScale);
            double searchBegin = std::max(
                0.0, expectedOffset - searchHalfExtent);
            double searchEnd = expectedOffset + searchHalfExtent;

            // One adjacent connection may search only within one traversal of
            // a circuit. This prevents wrapping to the same or another nearby
            // geometric branch on a different lap.
            if (track.topology() == coaster::TopologyKind::ClosedCircuit)
            {
                const double localLimit = std::nextafter(
                    track.lengthMeters(), 0.0);
                searchEnd = std::min(searchEnd, localLimit);
            }
            else
            {
                const double availableBehind =
                    leadingPose.referenceLocation().direction
                        == TravelDirection::IncreasingStation
                    ? leadingPose.referenceLocation().stationMeters
                    : track.lengthMeters()
                        - leadingPose.referenceLocation().stationMeters;
                searchEnd = std::min(searchEnd, availableBehind);
            }

            if (!std::isfinite(searchBegin)
                || !std::isfinite(searchEnd)
                || searchEnd < searchBegin)
            {
                throw std::domain_error(
                    "A physical local connector search interval could not be established.");
            }

            struct Bracket
            {
                ConnectionCandidate lower;
                ConnectionCandidate upper;
            };
            std::optional<Bracket> selectedBracket;
            std::optional<ConnectionCandidate> bestCandidate;
            std::optional<ConnectionCandidate> previous;
            double sampleSpacing = searchEnd - searchBegin;
            if (connectorSearchSampleCount != 0)
            {
                sampleSpacing /= static_cast<double>(
                    connectorSearchSampleCount);
            }

            for (std::size_t index = 0;
                index <= connectorSearchSampleCount;
                ++index)
            {
                const double amount = static_cast<double>(index)
                    / static_cast<double>(connectorSearchSampleCount);
                const double offset = index == connectorSearchSampleCount
                    ? searchEnd
                    : std::lerp(searchBegin, searchEnd, amount);
                try
                {
                    ConnectionCandidate candidate = connectionCandidate(
                        track,
                        followingDefinition,
                        leadingPose,
                        connection.rigidLengthMeters,
                        offset);
                    if (!bestCandidate
                        || std::abs(candidate.residualMeters)
                            < std::abs(bestCandidate->residualMeters)
                        || (std::abs(candidate.residualMeters)
                                == std::abs(bestCandidate->residualMeters)
                            && std::abs(offset - expectedOffset)
                                < std::abs(
                                    bestCandidate->backwardOffsetMeters
                                    - expectedOffset)))
                    {
                        bestCandidate = candidate;
                    }

                    if (previous
                        && std::signbit(previous->residualMeters)
                            != std::signbit(candidate.residualMeters))
                    {
                        Bracket bracket{*previous, candidate};
                        const double midpoint = 0.5
                            * (bracket.lower.backwardOffsetMeters
                                + bracket.upper.backwardOffsetMeters);
                        if (!selectedBracket
                            || std::abs(midpoint - expectedOffset)
                                < std::abs(
                                    0.5 * (selectedBracket->lower
                                            .backwardOffsetMeters
                                        + selectedBracket->upper
                                            .backwardOffsetMeters)
                                    - expectedOffset))
                        {
                            selectedBracket = std::move(bracket);
                        }
                    }
                    previous = std::move(candidate);
                }
                catch (const OpenConsistBoundaryError&)
                {
                    previous.reset();
                }
            }

            if (!bestCandidate)
            {
                throw OpenConsistBoundaryError(
                    "No legal following-car placement exists on the open track.");
            }

            if (selectedBracket)
            {
                ConnectionCandidate lower = selectedBracket->lower;
                ConnectionCandidate upper = selectedBracket->upper;
                std::size_t iterations = 0;
                for (; iterations < connectorRefinementIterationCount;
                    ++iterations)
                {
                    if (std::abs(lower.residualMeters)
                            <= connectorLengthToleranceMeters
                        || std::abs(upper.residualMeters)
                            <= connectorLengthToleranceMeters)
                    {
                        break;
                    }
                    const double midpoint = 0.5
                        * (lower.backwardOffsetMeters
                            + upper.backwardOffsetMeters);
                    ConnectionCandidate middle = connectionCandidate(
                        track,
                        followingDefinition,
                        leadingPose,
                        connection.rigidLengthMeters,
                        midpoint);
                    if (std::signbit(lower.residualMeters)
                        == std::signbit(middle.residualMeters))
                    {
                        lower = std::move(middle);
                    }
                    else
                    {
                        upper = std::move(middle);
                    }
                }

                const double finalBracketSize = std::abs(
                    upper.backwardOffsetMeters
                    - lower.backwardOffsetMeters);
                ConnectionCandidate solved =
                    std::abs(lower.residualMeters)
                        <= std::abs(upper.residualMeters)
                    ? std::move(lower)
                    : std::move(upper);
                if (std::abs(solved.residualMeters)
                    <= connectorLengthToleranceMeters)
                {
                    return {
                        std::move(solved.followingPose),
                        iterations,
                        finalBracketSize
                    };
                }
            }

            if (std::abs(bestCandidate->residualMeters)
                <= connectorLengthToleranceMeters)
            {
                return {
                    std::move(bestCandidate->followingPose),
                    0,
                    sampleSpacing
                };
            }

            // A zero-length connector has a tangent root and therefore need
            // not form a sign-changing bracket. Refine the best sampled local
            // minimum solely as a diagnostic fallback, then still enforce the
            // same rigid closure tolerance.
            double lowerOffset = std::max(
                searchBegin,
                bestCandidate->backwardOffsetMeters - sampleSpacing);
            double upperOffset = std::min(
                searchEnd,
                bestCandidate->backwardOffsetMeters + sampleSpacing);
            constexpr double goldenRatioConjugate = 0.6180339887498948482;
            ConnectionCandidate left = connectionCandidate(
                track,
                followingDefinition,
                leadingPose,
                connection.rigidLengthMeters,
                upperOffset - goldenRatioConjugate
                    * (upperOffset - lowerOffset));
            ConnectionCandidate right = connectionCandidate(
                track,
                followingDefinition,
                leadingPose,
                connection.rigidLengthMeters,
                lowerOffset + goldenRatioConjugate
                    * (upperOffset - lowerOffset));
            std::size_t iterations = 0;
            for (; iterations < connectorRefinementIterationCount;
                ++iterations)
            {
                if (std::abs(left.residualMeters)
                    < std::abs(right.residualMeters))
                {
                    upperOffset = right.backwardOffsetMeters;
                    right = std::move(left);
                    left = connectionCandidate(
                        track,
                        followingDefinition,
                        leadingPose,
                        connection.rigidLengthMeters,
                        upperOffset - goldenRatioConjugate
                            * (upperOffset - lowerOffset));
                }
                else
                {
                    lowerOffset = left.backwardOffsetMeters;
                    left = std::move(right);
                    right = connectionCandidate(
                        track,
                        followingDefinition,
                        leadingPose,
                        connection.rigidLengthMeters,
                        lowerOffset + goldenRatioConjugate
                            * (upperOffset - lowerOffset));
                }
            }
            ConnectionCandidate solved =
                std::abs(left.residualMeters)
                    <= std::abs(right.residualMeters)
                ? std::move(left)
                : std::move(right);
            if (std::abs(solved.residualMeters)
                <= connectorLengthToleranceMeters)
            {
                return {
                    std::move(solved.followingPose),
                    iterations,
                    upperOffset - lowerOffset
                };
            }

            if (track.topology() == coaster::TopologyKind::OpenLinear
                && searchEnd < expectedOffset + searchHalfExtent)
            {
                throw OpenConsistBoundaryError(
                    "The following car cannot close its connector before the open track endpoint.");
            }
            throw std::domain_error(
                "The rigid inter-car connector cannot close within the physical local search bounds.");
        }

        [[nodiscard]] glm::dvec3 inBodyFrame(
            const glm::dvec3& worldVector,
            const geometry::CurveFrame& frame) noexcept
        {
            return {
                glm::dot(worldVector, frame.tangent),
                glm::dot(worldVector, frame.lateral),
                glm::dot(worldVector, frame.up)
            };
        }

        [[nodiscard]] glm::dquat canonicalized(glm::dquat value)
        {
            value = glm::normalize(value);
            if (!finite(value))
            {
                throw std::domain_error(
                    "Relative car articulation orientation is non-finite.");
            }
            if (value.w < 0.0)
            {
                value = -value;
            }
            return value;
        }

        [[nodiscard]] glm::dvec3 relativeYawPitchRoll(
            const geometry::CurveFrame& leading,
            const geometry::CurveFrame& following)
        {
            const double yaw = std::atan2(
                glm::dot(following.tangent, leading.lateral),
                glm::dot(following.tangent, leading.tangent));
            const double pitch = std::asin(glm::clamp(
                glm::dot(following.tangent, leading.up), -1.0, 1.0));
            const double roll = std::atan2(
                glm::dot(following.up, leading.lateral),
                glm::dot(following.up, leading.up));
            const glm::dvec3 result{yaw, pitch, roll};
            if (!finite(result))
            {
                throw std::domain_error(
                    "Relative car articulation diagnostics are non-finite.");
            }
            return result;
        }

        [[nodiscard]] TrackLocation displacedLocation(
            const CompiledPhysicsTrack& track,
            const TrackLocation& location,
            const double distanceMeters)
        {
            if (track.topology() == coaster::TopologyKind::OpenLinear)
            {
                const double rawStation = location.stationMeters
                    + distanceMeters;
                if (rawStation < 0.0 || rawStation > track.lengthMeters())
                {
                    throw OpenConsistBoundaryError(
                        "A train finite-difference sample lies outside the open track.");
                }
            }
            TrackLocation result = track.advance(location, distanceMeters).location;
            result.direction = location.direction;
            return result;
        }

        [[nodiscard]] std::optional<TrainPose> tryDisplacedPose(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const TrackLocation& location,
            const double distanceMeters)
        {
            try
            {
                return solveTrainPose(
                    track,
                    definition,
                    displacedLocation(track, location, distanceMeters));
            }
            catch (const OpenConsistBoundaryError&)
            {
                return std::nullopt;
            }
        }

        [[nodiscard]] std::vector<glm::dvec3> cogPositions(
            const TrainPose& pose)
        {
            std::vector<glm::dvec3> result;
            result.reserve(pose.carCount());
            for (const TrainCarPose& car : pose.cars())
            {
                result.push_back(
                    car.loadedWorldCenterOfGravityMeters());
            }
            return result;
        }

        struct DerivativeSamples
        {
            std::vector<glm::dvec3> first;
            std::vector<glm::dvec3> second;
            TrainFiniteDifferenceKind kind =
                TrainFiniteDifferenceKind::Central;
        };

        [[nodiscard]] DerivativeSamples kinematicDerivatives(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const TrainPose& center)
        {
            const double epsilon = trainKinematicJacobianStepMeters;
            const TrackLocation& location =
                center.generalizedReferenceLocation();
            const std::optional<TrainPose> before = tryDisplacedPose(
                track, definition, location, -epsilon);
            const std::optional<TrainPose> after = tryDisplacedPose(
                track, definition, location, epsilon);
            const std::vector<glm::dvec3> centerPositions =
                cogPositions(center);
            DerivativeSamples result;
            result.first.resize(center.carCount());
            result.second.resize(center.carCount());

            if (before && after)
            {
                const auto beforePositions = cogPositions(*before);
                const auto afterPositions = cogPositions(*after);
                for (std::size_t index = 0;
                    index < center.carCount();
                    ++index)
                {
                    result.first[index] =
                        (afterPositions[index] - beforePositions[index])
                        / (2.0 * epsilon);
                    result.second[index] =
                        (afterPositions[index]
                            - 2.0 * centerPositions[index]
                            + beforePositions[index])
                        / (epsilon * epsilon);
                }
                return result;
            }

            if (after)
            {
                const std::optional<TrainPose> afterTwice = tryDisplacedPose(
                    track, definition, location, 2.0 * epsilon);
                if (!afterTwice)
                {
                    throw OpenConsistBoundaryError(
                        "The valid train envelope is too narrow for a forward kinematic derivative.");
                }
                const auto afterPositions = cogPositions(*after);
                const auto afterTwicePositions = cogPositions(*afterTwice);
                result.kind = TrainFiniteDifferenceKind::Forward;
                for (std::size_t index = 0;
                    index < center.carCount();
                    ++index)
                {
                    result.first[index] =
                        (-3.0 * centerPositions[index]
                            + 4.0 * afterPositions[index]
                            - afterTwicePositions[index])
                        / (2.0 * epsilon);
                    result.second[index] =
                        (centerPositions[index]
                            - 2.0 * afterPositions[index]
                            + afterTwicePositions[index])
                        / (epsilon * epsilon);
                }
                return result;
            }

            if (before)
            {
                const std::optional<TrainPose> beforeTwice = tryDisplacedPose(
                    track, definition, location, -2.0 * epsilon);
                if (!beforeTwice)
                {
                    throw OpenConsistBoundaryError(
                        "The valid train envelope is too narrow for a backward kinematic derivative.");
                }
                const auto beforePositions = cogPositions(*before);
                const auto beforeTwicePositions = cogPositions(*beforeTwice);
                result.kind = TrainFiniteDifferenceKind::Backward;
                for (std::size_t index = 0;
                    index < center.carCount();
                    ++index)
                {
                    result.first[index] =
                        (3.0 * centerPositions[index]
                            - 4.0 * beforePositions[index]
                            + beforeTwicePositions[index])
                        / (2.0 * epsilon);
                    result.second[index] =
                        (centerPositions[index]
                            - 2.0 * beforePositions[index]
                            + beforeTwicePositions[index])
                        / (epsilon * epsilon);
                }
                return result;
            }

            throw OpenConsistBoundaryError(
                "No valid neighboring train pose exists for a kinematic derivative.");
        }

        [[nodiscard]] bool legalTrainPose(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const TrackLocation& location)
        {
            try
            {
                static_cast<void>(solveTrainPose(track, definition, location));
                return true;
            }
            catch (const OpenConsistBoundaryError&)
            {
                return false;
            }
        }
    }

    void validateInterCarConnectionDefinition(
        const InterCarConnectionDefinition& definition)
    {
        if (!std::isfinite(definition.rigidLengthMeters)
            || definition.rigidLengthMeters < 0.0)
        {
            throw std::invalid_argument(
                "Inter-car connector length must be finite and non-negative.");
        }
    }

    void validateTrainDefinition(const TrainDefinition& definition)
    {
        if (definition.cars.empty())
        {
            throw std::invalid_argument(
                "A train definition must contain at least one car.");
        }
        if (definition.connections.size() + 1 != definition.cars.size())
        {
            throw std::invalid_argument(
                "A train must contain exactly one fewer connection than cars.");
        }

        double totalMass = 0.0;
        for (const TrainCarDefinition& car : definition.cars)
        {
            validateCarDefinition(car.car);
            validateCarLoadout(car.loadout);
            totalMass += totalCarMassKilograms(car.car, car.loadout);
        }
        if (!std::isfinite(totalMass) || totalMass <= 0.0)
        {
            throw std::invalid_argument(
                "Train cars must produce a positive finite loaded mass.");
        }
        for (const InterCarConnectionDefinition& connection
            : definition.connections)
        {
            validateInterCarConnectionDefinition(connection);
        }
        validateBasicResistance(definition.resistance);
    }

    TrainCarPose::TrainCarPose(
        const std::size_t carIndex,
        CarPose carPose)
        : carIndex_(carIndex),
          carPose_(std::move(carPose))
    {
    }

    std::size_t TrainCarPose::carIndex() const noexcept
    {
        return carIndex_;
    }

    const TrackLocation& TrainCarPose::referenceLocation() const noexcept
    {
        return carPose_.referenceLocation();
    }

    const CarPose& TrainCarPose::carPose() const noexcept
    {
        return carPose_;
    }

    double TrainCarPose::loadedMassKilograms() const noexcept
    {
        return carPose_.totalMassKilograms();
    }

    const glm::dvec3& TrainCarPose::loadedLocalCenterOfGravityMeters()
        const noexcept
    {
        return carPose_.localCenterOfGravityMeters();
    }

    const glm::dvec3& TrainCarPose::loadedWorldCenterOfGravityMeters()
        const noexcept
    {
        return carPose_.worldCenterOfGravityMeters();
    }

    InterCarConnectionPose::InterCarConnectionPose(
        const std::size_t connectionIndex,
        const std::size_t leadingCarIndex,
        const std::size_t followingCarIndex,
        const double authoredRigidLengthMeters,
        glm::dvec3 leadingEndpointWorldPositionMeters,
        glm::dvec3 followingEndpointWorldPositionMeters,
        const double actualEndpointDistanceMeters,
        const double signedLengthResidualMeters,
        std::optional<glm::dvec3> worldDirection,
        std::optional<glm::dvec3> directionInLeadingBody,
        std::optional<glm::dvec3> directionInFollowingBody,
        glm::dquat followingBodyRelativeOrientation,
        glm::dvec3 relativeYawPitchRollRadians,
        const std::size_t solverIterationCount,
        const double finalBracketSizeMeters)
        : connectionIndex_(connectionIndex),
          leadingCarIndex_(leadingCarIndex),
          followingCarIndex_(followingCarIndex),
          authoredRigidLengthMeters_(authoredRigidLengthMeters),
          leadingEndpointWorldPositionMeters_(
              leadingEndpointWorldPositionMeters),
          followingEndpointWorldPositionMeters_(
              followingEndpointWorldPositionMeters),
          actualEndpointDistanceMeters_(actualEndpointDistanceMeters),
          signedLengthResidualMeters_(signedLengthResidualMeters),
          worldDirection_(std::move(worldDirection)),
          directionInLeadingBody_(std::move(directionInLeadingBody)),
          directionInFollowingBody_(std::move(directionInFollowingBody)),
          followingBodyRelativeOrientation_(
              followingBodyRelativeOrientation),
          relativeYawPitchRollRadians_(relativeYawPitchRollRadians),
          solverIterationCount_(solverIterationCount),
          finalBracketSizeMeters_(finalBracketSizeMeters)
    {
    }

    std::size_t InterCarConnectionPose::connectionIndex() const noexcept
    {
        return connectionIndex_;
    }

    std::size_t InterCarConnectionPose::leadingCarIndex() const noexcept
    {
        return leadingCarIndex_;
    }

    std::size_t InterCarConnectionPose::followingCarIndex() const noexcept
    {
        return followingCarIndex_;
    }

    double InterCarConnectionPose::authoredRigidLengthMeters() const noexcept
    {
        return authoredRigidLengthMeters_;
    }

    const glm::dvec3&
    InterCarConnectionPose::leadingEndpointWorldPositionMeters() const noexcept
    {
        return leadingEndpointWorldPositionMeters_;
    }

    const glm::dvec3&
    InterCarConnectionPose::followingEndpointWorldPositionMeters() const noexcept
    {
        return followingEndpointWorldPositionMeters_;
    }

    double InterCarConnectionPose::actualEndpointDistanceMeters() const noexcept
    {
        return actualEndpointDistanceMeters_;
    }

    double InterCarConnectionPose::signedLengthResidualMeters() const noexcept
    {
        return signedLengthResidualMeters_;
    }

    double InterCarConnectionPose::absoluteLengthErrorMeters() const noexcept
    {
        return std::abs(signedLengthResidualMeters_);
    }

    const std::optional<glm::dvec3>&
    InterCarConnectionPose::worldDirection() const noexcept
    {
        return worldDirection_;
    }

    const std::optional<glm::dvec3>&
    InterCarConnectionPose::directionInLeadingBody() const noexcept
    {
        return directionInLeadingBody_;
    }

    const std::optional<glm::dvec3>&
    InterCarConnectionPose::directionInFollowingBody() const noexcept
    {
        return directionInFollowingBody_;
    }

    const glm::dquat&
    InterCarConnectionPose::followingBodyRelativeOrientation() const noexcept
    {
        return followingBodyRelativeOrientation_;
    }

    const glm::dvec3&
    InterCarConnectionPose::relativeYawPitchRollRadians() const noexcept
    {
        return relativeYawPitchRollRadians_;
    }

    std::size_t InterCarConnectionPose::solverIterationCount() const noexcept
    {
        return solverIterationCount_;
    }

    double InterCarConnectionPose::finalBracketSizeMeters() const noexcept
    {
        return finalBracketSizeMeters_;
    }

    TrainPose::TrainPose(
        TrackLocation generalizedReferenceLocation,
        std::vector<TrainCarPose> cars,
        std::vector<InterCarConnectionPose> connections,
        const double totalLoadedMassKilograms,
        glm::dvec3 aggregateWorldCenterOfGravityMeters,
        const double maximumAbsoluteConnectorResidualMeters)
        : generalizedReferenceLocation_(generalizedReferenceLocation),
          cars_(std::move(cars)),
          connections_(std::move(connections)),
          totalLoadedMassKilograms_(totalLoadedMassKilograms),
          aggregateWorldCenterOfGravityMeters_(
              aggregateWorldCenterOfGravityMeters),
          maximumAbsoluteConnectorResidualMeters_(
              maximumAbsoluteConnectorResidualMeters)
    {
    }

    const TrackLocation& TrainPose::generalizedReferenceLocation()
        const noexcept
    {
        return generalizedReferenceLocation_;
    }

    const std::vector<TrainCarPose>& TrainPose::cars() const noexcept
    {
        return cars_;
    }

    const std::vector<InterCarConnectionPose>& TrainPose::connections()
        const noexcept
    {
        return connections_;
    }

    std::size_t TrainPose::carCount() const noexcept
    {
        return cars_.size();
    }

    std::size_t TrainPose::connectionCount() const noexcept
    {
        return connections_.size();
    }

    double TrainPose::totalLoadedMassKilograms() const noexcept
    {
        return totalLoadedMassKilograms_;
    }

    const glm::dvec3& TrainPose::aggregateWorldCenterOfGravityMeters()
        const noexcept
    {
        return aggregateWorldCenterOfGravityMeters_;
    }

    double TrainPose::maximumAbsoluteConnectorResidualMeters() const noexcept
    {
        return maximumAbsoluteConnectorResidualMeters_;
    }

    TrainPose solveTrainPose(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const TrackLocation& generalizedReferenceLocation)
    {
        validateTrainDefinition(definition);
        // sample() is the authoritative public location validation seam.
        static_cast<void>(track.sample(generalizedReferenceLocation));

        std::vector<TrainCarPose> cars;
        std::vector<InterCarConnectionPose> connections;
        cars.reserve(definition.cars.size());
        connections.reserve(definition.connections.size());

        CarPose leadPose = solveLegalCarPose(
            track, definition.cars.front(), generalizedReferenceLocation);
        cars.emplace_back(0, std::move(leadPose));

        double maximumResidual = 0.0;
        for (std::size_t connectionIndex = 0;
            connectionIndex < definition.connections.size();
            ++connectionIndex)
        {
            const std::size_t leadingIndex = connectionIndex;
            const std::size_t followingIndex = connectionIndex + 1;
            const CarPose& leadingPose = cars.back().carPose();
            SolvedFollowingCar solved = solveFollowingCar(
                track,
                definition.cars[leadingIndex],
                leadingPose,
                definition.cars[followingIndex],
                definition.connections[connectionIndex]);

            const glm::dvec3 leadingEndpoint =
                leadingPose.rearHitchWorldPositionMeters();
            const glm::dvec3 followingEndpoint =
                solved.pose.frontHitchWorldPositionMeters();
            const glm::dvec3 endpointVector =
                followingEndpoint - leadingEndpoint;
            const double actualDistance = glm::length(endpointVector);
            const double residual = actualDistance
                - definition.connections[connectionIndex].rigidLengthMeters;
            const double absoluteResidual = std::abs(residual);
            if (!std::isfinite(actualDistance)
                || !std::isfinite(residual)
                || absoluteResidual > connectorLengthToleranceMeters)
            {
                throw std::domain_error(
                    "A solved rigid connector exceeds the numerical closure tolerance.");
            }

            std::optional<glm::dvec3> direction;
            std::optional<glm::dvec3> leadingBodyDirection;
            std::optional<glm::dvec3> followingBodyDirection;
            if (definition.connections[connectionIndex].rigidLengthMeters
                > 0.0)
            {
                if (actualDistance == 0.0)
                {
                    throw std::domain_error(
                        "A non-zero connector cannot define a world direction.");
                }
                direction = endpointVector / actualDistance;
                if (!finite(*direction))
                {
                    throw std::domain_error(
                        "Connector world direction is non-finite.");
                }
                leadingBodyDirection = inBodyFrame(
                    *direction, leadingPose.bodyFrame());
                followingBodyDirection = inBodyFrame(
                    *direction, solved.pose.bodyFrame());
            }

            const glm::dquat relativeOrientation = canonicalized(
                glm::conjugate(leadingPose.bodyOrientation())
                * solved.pose.bodyOrientation());
            const glm::dvec3 articulation = relativeYawPitchRoll(
                leadingPose.bodyFrame(), solved.pose.bodyFrame());
            connections.emplace_back(
                connectionIndex,
                leadingIndex,
                followingIndex,
                definition.connections[connectionIndex].rigidLengthMeters,
                leadingEndpoint,
                followingEndpoint,
                actualDistance,
                residual,
                direction,
                leadingBodyDirection,
                followingBodyDirection,
                relativeOrientation,
                articulation,
                solved.iterationCount,
                solved.finalBracketSizeMeters);
            maximumResidual = std::max(maximumResidual, absoluteResidual);
            cars.emplace_back(followingIndex, std::move(solved.pose));
        }

        double totalMass = 0.0;
        glm::dvec3 weightedCenter{0.0};
        for (const TrainCarPose& car : cars)
        {
            totalMass += car.loadedMassKilograms();
            weightedCenter += car.loadedMassKilograms()
                * car.loadedWorldCenterOfGravityMeters();
        }
        const glm::dvec3 aggregateCenter = weightedCenter / totalMass;
        if (!std::isfinite(totalMass)
            || totalMass <= 0.0
            || !finite(aggregateCenter))
        {
            throw std::domain_error(
                "The solved train mass properties are non-finite.");
        }

        return {
            generalizedReferenceLocation,
            std::move(cars),
            std::move(connections),
            totalMass,
            aggregateCenter,
            maximumResidual
        };
    }

    TrainKinematicEvaluation evaluateTrainKinematics(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrackLocation& generalizedReferenceLocation)
    {
        validatePhysicsEnvironment(environment);
        TrainPose center = solveTrainPose(
            track, definition, generalizedReferenceLocation);
        DerivativeSamples derivatives = kinematicDerivatives(
            track, definition, center);
        const glm::dvec3 gravityWorld{
            0.0,
            0.0,
            -environment.gravityAccelerationMetersPerSecondSquared
        };

        std::vector<TrainCarKinematics> cars;
        cars.reserve(center.carCount());
        double effectiveMass = 0.0;
        double effectiveMassDerivative = 0.0;
        double gravityForce = 0.0;
        for (std::size_t index = 0; index < center.carCount(); ++index)
        {
            const double mass = center.cars()[index].loadedMassKilograms();
            const glm::dvec3& jacobian = derivatives.first[index];
            const glm::dvec3& secondDerivative = derivatives.second[index];
            if (!finite(jacobian) || !finite(secondDerivative))
            {
                throw std::domain_error(
                    "Train kinematic derivatives are non-finite.");
            }
            const double carGravity = mass
                * glm::dot(gravityWorld, jacobian);
            effectiveMass += mass * glm::dot(jacobian, jacobian);
            effectiveMassDerivative += 2.0 * mass
                * glm::dot(jacobian, secondDerivative);
            gravityForce += carGravity;
            cars.push_back({index, jacobian, carGravity});
        }
        if (!std::isfinite(effectiveMass)
            || effectiveMass <= 0.0
            || !std::isfinite(effectiveMassDerivative)
            || !std::isfinite(gravityForce))
        {
            throw std::domain_error(
                "Train reduced-coordinate mechanics are non-finite or degenerate.");
        }

        return {
            std::move(center),
            std::move(cars),
            effectiveMass,
            effectiveMassDerivative,
            gravityForce,
            trainKinematicJacobianStepMeters,
            derivatives.kind
        };
    }

    TrainStepResult stepTrain(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrainDynamicsState& currentState,
        const FixedStepSettings& step)
    {
        validateTrainDefinition(definition);
        validatePhysicsEnvironment(environment);
        validateFixedStepSettings(step);
        if (!std::isfinite(currentState.signedVelocityMetersPerSecond)
            || !std::isfinite(
                currentState.generalizedAccelerationMetersPerSecondSquared)
            || !validRunState(currentState.runState))
        {
            throw std::invalid_argument(
                "Train dynamics state must be finite and valid.");
        }
        if (currentState.tick == std::numeric_limits<std::uint64_t>::max())
        {
            throw std::overflow_error("Train dynamics tick overflow.");
        }

        TrainKinematicEvaluation current = evaluateTrainKinematics(
            track,
            definition,
            environment,
            currentState.generalizedReferenceLocation);
        double workingVelocity = currentState.signedVelocityMetersPerSecond;
        if (std::abs(workingVelocity)
            <= followerRestSpeedToleranceMetersPerSecond)
        {
            workingVelocity = 0.0;
        }
        if ((currentState.runState == FollowerRunState::StoppedAtStart
                && workingVelocity < 0.0)
            || (currentState.runState == FollowerRunState::StoppedAtEnd
                && workingVelocity > 0.0))
        {
            workingVelocity = 0.0;
        }

        const double resistanceForce = evaluateBasicResistanceForceNewtons(
            definition.resistance,
            current.pose.totalLoadedMassKilograms(),
            environment.gravityAccelerationMetersPerSecondSquared,
            current.generalizedGravityForceNewtons,
            workingVelocity);
        const double massGradientForce = -0.5
            * current.effectiveGeneralizedMassDerivativeKilogramsPerMeter
            * workingVelocity * workingVelocity;
        const double unconstrainedForce =
            current.generalizedGravityForceNewtons
            + resistanceForce
            + massGradientForce;
        const double unconstrainedAcceleration = unconstrainedForce
            / current.effectiveGeneralizedMassKilograms;
        double nextVelocity = workingVelocity
            + unconstrainedAcceleration * step.deltaTimeSeconds;
        if (!std::isfinite(resistanceForce)
            || !std::isfinite(massGradientForce)
            || !std::isfinite(unconstrainedForce)
            || !std::isfinite(unconstrainedAcceleration)
            || !std::isfinite(nextVelocity))
        {
            throw std::domain_error(
                "Train step produced a non-finite force or velocity.");
        }

        if ((workingVelocity > 0.0 && nextVelocity <= 0.0)
            || (workingVelocity < 0.0 && nextVelocity >= 0.0)
            || std::abs(nextVelocity)
                <= followerRestSpeedToleranceMetersPerSecond)
        {
            nextVelocity = 0.0;
        }

        const double requestedDistance =
            nextVelocity * step.deltaTimeSeconds;
        TrackAdvanceResult advancement = track.advance(
            currentState.generalizedReferenceLocation,
            requestedDistance);
        bool boundaryIntervention = false;
        TrackBoundary consistBoundary = advancement.boundary;

        if (track.topology() == coaster::TopologyKind::OpenLinear
            && !legalTrainPose(track, definition, advancement.location))
        {
            boundaryIntervention = true;
            consistBoundary = requestedDistance < 0.0
                ? TrackBoundary::Start
                : TrackBoundary::End;
            double legalFraction = 0.0;
            double illegalFraction = 1.0;
            for (std::size_t iteration = 0;
                iteration < boundaryRefinementIterationCount;
                ++iteration)
            {
                const double midpoint = 0.5
                    * (legalFraction + illegalFraction);
                const TrackLocation candidate = track.advance(
                    currentState.generalizedReferenceLocation,
                    requestedDistance * midpoint).location;
                if (legalTrainPose(track, definition, candidate))
                {
                    legalFraction = midpoint;
                }
                else
                {
                    illegalFraction = midpoint;
                }
            }
            advancement = track.advance(
                currentState.generalizedReferenceLocation,
                requestedDistance * legalFraction);
            nextVelocity = 0.0;
        }

        TrainDynamicsState nextState;
        nextState.generalizedReferenceLocation = advancement.location;
        nextState.signedVelocityMetersPerSecond = nextVelocity;
        nextState.generalizedAccelerationMetersPerSecondSquared =
            (nextVelocity - currentState.signedVelocityMetersPerSecond)
            / step.deltaTimeSeconds;
        nextState.tick = currentState.tick + 1;
        if (nextVelocity != 0.0)
        {
            nextState.runState = FollowerRunState::Running;
        }
        else if (consistBoundary == TrackBoundary::Start)
        {
            nextState.runState = FollowerRunState::StoppedAtStart;
        }
        else if (consistBoundary == TrackBoundary::End)
        {
            nextState.runState = FollowerRunState::StoppedAtEnd;
        }
        else
        {
            nextState.runState = FollowerRunState::Resting;
        }

        const double totalForce =
            current.effectiveGeneralizedMassKilograms
            * nextState.generalizedAccelerationMetersPerSecondSquared;
        const double constraintForce = totalForce - unconstrainedForce;
        const double simulationTime = static_cast<double>(nextState.tick)
            * step.deltaTimeSeconds;
        if (!std::isfinite(totalForce)
            || !std::isfinite(constraintForce)
            || !std::isfinite(simulationTime))
        {
            throw std::domain_error(
                "Train telemetry is not representable as finite SI values.");
        }

        // Match Phase 1: pose is for the committed state while forces and the
        // per-car Jacobians are those used to integrate from the prior state.
        TrainPose committedPose = solveTrainPose(
            track, definition, nextState.generalizedReferenceLocation);
        const double committedMaximumResidual =
            committedPose.maximumAbsoluteConnectorResidualMeters();
        TrainTelemetry telemetry{
            nextState.tick,
            simulationTime,
            nextState.generalizedReferenceLocation,
            std::move(committedPose),
            std::move(current.cars),
            nextVelocity,
            nextState.generalizedAccelerationMetersPerSecondSquared,
            current.pose.totalLoadedMassKilograms(),
            current.effectiveGeneralizedMassKilograms,
            current.generalizedGravityForceNewtons,
            resistanceForce,
            massGradientForce,
            constraintForce,
            totalForce,
            current.finiteDifferenceStepMeters,
            current.finiteDifferenceKind,
            definition.cars.size(),
            definition.connections.size(),
            committedMaximumResidual,
            nextState.runState,
            consistBoundary,
            advancement.wrapped,
            boundaryIntervention
        };
        return {nextState, std::move(telemetry)};
    }
}
