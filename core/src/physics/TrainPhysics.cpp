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

        void validateExternalForceApplications(
            const std::span<const ExternalForceApplication> applications,
            const std::size_t carCount)
        {
            for (const ExternalForceApplication& application : applications)
            {
                if (application.carIndex >= carCount)
                {
                    throw std::invalid_argument(
                        "External force application car index is outside the train.");
                }
                if (!finite(application.localApplicationPointMeters))
                {
                    throw std::invalid_argument(
                        "External force application point must be finite and expressed in car-local metres.");
                }
                if (!finite(application.worldForceNewtons))
                {
                    throw std::invalid_argument(
                        "External force vector must be finite and expressed in world-space Newtons.");
                }
            }
        }

        [[nodiscard]] std::vector<glm::dvec3> applicationPointPositions(
            const TrainPose& pose,
            const std::span<const ExternalForceApplication> applications)
        {
            std::vector<glm::dvec3> result;
            result.reserve(applications.size());
            for (const ExternalForceApplication& application : applications)
            {
                const glm::dvec3 point = pose.cars()[application.carIndex]
                    .carPose().transformLocalPoint(
                        application.localApplicationPointMeters);
                if (!finite(point))
                {
                    throw std::domain_error(
                        "External force application produced a non-finite world point.");
                }
                result.push_back(point);
            }
            return result;
        }

        struct DerivativeSamples
        {
            std::vector<glm::dvec3> first;
            std::vector<glm::dvec3> second;
            std::vector<glm::dvec3> applicationPoints;
            std::vector<glm::dvec3> applicationPointFirst;
            TrainFiniteDifferenceKind kind =
                TrainFiniteDifferenceKind::Central;
        };

        [[nodiscard]] DerivativeSamples kinematicDerivatives(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const TrainPose& center,
            const std::span<const ExternalForceApplication> applications)
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
            result.applicationPoints = applicationPointPositions(
                center, applications);
            result.applicationPointFirst.resize(applications.size());

            if (before && after)
            {
                const auto beforePositions = cogPositions(*before);
                const auto afterPositions = cogPositions(*after);
                const auto beforeApplicationPoints = applicationPointPositions(
                    *before, applications);
                const auto afterApplicationPoints = applicationPointPositions(
                    *after, applications);
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
                for (std::size_t index = 0;
                    index < applications.size();
                    ++index)
                {
                    result.applicationPointFirst[index] =
                        (afterApplicationPoints[index]
                            - beforeApplicationPoints[index])
                        / (2.0 * epsilon);
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
                const auto afterApplicationPoints = applicationPointPositions(
                    *after, applications);
                const auto afterTwiceApplicationPoints =
                    applicationPointPositions(*afterTwice, applications);
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
                for (std::size_t index = 0;
                    index < applications.size();
                    ++index)
                {
                    result.applicationPointFirst[index] =
                        (-3.0 * result.applicationPoints[index]
                            + 4.0 * afterApplicationPoints[index]
                            - afterTwiceApplicationPoints[index])
                        / (2.0 * epsilon);
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
                const auto beforeApplicationPoints = applicationPointPositions(
                    *before, applications);
                const auto beforeTwiceApplicationPoints =
                    applicationPointPositions(*beforeTwice, applications);
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
                for (std::size_t index = 0;
                    index < applications.size();
                    ++index)
                {
                    result.applicationPointFirst[index] =
                        (3.0 * result.applicationPoints[index]
                            - 4.0 * beforeApplicationPoints[index]
                            + beforeTwiceApplicationPoints[index])
                        / (2.0 * epsilon);
                }
                return result;
            }

            throw OpenConsistBoundaryError(
                "No valid neighboring train pose exists for a kinematic derivative.");
        }

        struct CarLocalDerivativeSamples
        {
            glm::dvec3 centerOfGravity{0.0};
            glm::dvec3 frontHitch{0.0};
            glm::dvec3 rearHitch{0.0};
            std::vector<glm::dvec3> applicationPointDerivatives;
            TrainFiniteDifferenceKind kind =
                TrainFiniteDifferenceKind::Central;
        };

        [[nodiscard]] std::vector<glm::dvec3> localApplicationPointPositions(
            const CarPose& pose,
            const std::span<const ExternalForceApplication> applications,
            const std::span<const std::size_t> applicationIndices)
        {
            std::vector<glm::dvec3> result;
            result.reserve(applicationIndices.size());
            for (const std::size_t index : applicationIndices)
            {
                const glm::dvec3 point = pose.transformLocalPoint(
                    applications[index].localApplicationPointMeters);
                if (!finite(point))
                {
                    throw std::domain_error(
                        "External force application produced a non-finite local-coordinate sample.");
                }
                result.push_back(point);
            }
            return result;
        }

        [[nodiscard]] std::optional<CarPose> tryDisplacedCarPose(
            const CompiledPhysicsTrack& track,
            const TrainCarDefinition& definition,
            const TrackLocation& location,
            const double distanceMeters)
        {
            try
            {
                return solveLegalCarPose(
                    track,
                    definition,
                    displacedLocation(track, location, distanceMeters));
            }
            catch (const OpenConsistBoundaryError&)
            {
                return std::nullopt;
            }
        }

        [[nodiscard]] CarLocalDerivativeSamples localCarDerivatives(
            const CompiledPhysicsTrack& track,
            const TrainCarDefinition& definition,
            const CarPose& center,
            const std::span<const ExternalForceApplication> applications,
            const std::span<const std::size_t> applicationIndices)
        {
            const double epsilon = connectorLoadLocalDerivativeStepMeters;
            const TrackLocation& location = center.referenceLocation();
            const std::optional<CarPose> before = tryDisplacedCarPose(
                track, definition, location, -epsilon);
            const std::optional<CarPose> after = tryDisplacedCarPose(
                track, definition, location, epsilon);

            const auto central = [epsilon](
                const glm::dvec3& lower,
                const glm::dvec3& upper)
            {
                return (upper - lower) / (2.0 * epsilon);
            };
            const auto forward = [epsilon](
                const glm::dvec3& centerValue,
                const glm::dvec3& first,
                const glm::dvec3& second)
            {
                return (-3.0 * centerValue + 4.0 * first - second)
                    / (2.0 * epsilon);
            };
            const auto backward = [epsilon](
                const glm::dvec3& centerValue,
                const glm::dvec3& first,
                const glm::dvec3& second)
            {
                return (3.0 * centerValue - 4.0 * first + second)
                    / (2.0 * epsilon);
            };

            CarLocalDerivativeSamples result;
            const auto centerApplicationPoints =
                localApplicationPointPositions(
                    center, applications, applicationIndices);
            result.applicationPointDerivatives.resize(
                applicationIndices.size());
            if (before && after)
            {
                result.centerOfGravity = central(
                    before->worldCenterOfGravityMeters(),
                    after->worldCenterOfGravityMeters());
                result.frontHitch = central(
                    before->frontHitchWorldPositionMeters(),
                    after->frontHitchWorldPositionMeters());
                result.rearHitch = central(
                    before->rearHitchWorldPositionMeters(),
                    after->rearHitchWorldPositionMeters());
                const auto beforeApplicationPoints =
                    localApplicationPointPositions(
                        *before, applications, applicationIndices);
                const auto afterApplicationPoints =
                    localApplicationPointPositions(
                        *after, applications, applicationIndices);
                for (std::size_t index = 0;
                    index < applicationIndices.size();
                    ++index)
                {
                    result.applicationPointDerivatives[index] = central(
                        beforeApplicationPoints[index],
                        afterApplicationPoints[index]);
                }
            }
            else if (after)
            {
                const std::optional<CarPose> afterTwice = tryDisplacedCarPose(
                    track, definition, location, 2.0 * epsilon);
                if (!afterTwice)
                {
                    throw OpenConsistBoundaryError(
                        "The valid car envelope is too narrow for a forward connector-load derivative.");
                }
                result.kind = TrainFiniteDifferenceKind::Forward;
                result.centerOfGravity = forward(
                    center.worldCenterOfGravityMeters(),
                    after->worldCenterOfGravityMeters(),
                    afterTwice->worldCenterOfGravityMeters());
                result.frontHitch = forward(
                    center.frontHitchWorldPositionMeters(),
                    after->frontHitchWorldPositionMeters(),
                    afterTwice->frontHitchWorldPositionMeters());
                result.rearHitch = forward(
                    center.rearHitchWorldPositionMeters(),
                    after->rearHitchWorldPositionMeters(),
                    afterTwice->rearHitchWorldPositionMeters());
                const auto afterApplicationPoints =
                    localApplicationPointPositions(
                        *after, applications, applicationIndices);
                const auto afterTwiceApplicationPoints =
                    localApplicationPointPositions(
                        *afterTwice, applications, applicationIndices);
                for (std::size_t index = 0;
                    index < applicationIndices.size();
                    ++index)
                {
                    result.applicationPointDerivatives[index] = forward(
                        centerApplicationPoints[index],
                        afterApplicationPoints[index],
                        afterTwiceApplicationPoints[index]);
                }
            }
            else if (before)
            {
                const std::optional<CarPose> beforeTwice = tryDisplacedCarPose(
                    track, definition, location, -2.0 * epsilon);
                if (!beforeTwice)
                {
                    throw OpenConsistBoundaryError(
                        "The valid car envelope is too narrow for a backward connector-load derivative.");
                }
                result.kind = TrainFiniteDifferenceKind::Backward;
                result.centerOfGravity = backward(
                    center.worldCenterOfGravityMeters(),
                    before->worldCenterOfGravityMeters(),
                    beforeTwice->worldCenterOfGravityMeters());
                result.frontHitch = backward(
                    center.frontHitchWorldPositionMeters(),
                    before->frontHitchWorldPositionMeters(),
                    beforeTwice->frontHitchWorldPositionMeters());
                result.rearHitch = backward(
                    center.rearHitchWorldPositionMeters(),
                    before->rearHitchWorldPositionMeters(),
                    beforeTwice->rearHitchWorldPositionMeters());
                const auto beforeApplicationPoints =
                    localApplicationPointPositions(
                        *before, applications, applicationIndices);
                const auto beforeTwiceApplicationPoints =
                    localApplicationPointPositions(
                        *beforeTwice, applications, applicationIndices);
                for (std::size_t index = 0;
                    index < applicationIndices.size();
                    ++index)
                {
                    result.applicationPointDerivatives[index] = backward(
                        centerApplicationPoints[index],
                        beforeApplicationPoints[index],
                        beforeTwiceApplicationPoints[index]);
                }
            }
            else
            {
                throw OpenConsistBoundaryError(
                    "No valid neighboring car pose exists for a connector-load derivative.");
            }

            if (!finite(result.centerOfGravity)
                || !finite(result.frontHitch)
                || !finite(result.rearHitch))
            {
                throw std::domain_error(
                    "Connector-load local car derivatives are non-finite.");
            }
            for (const glm::dvec3& derivative
                : result.applicationPointDerivatives)
            {
                if (!finite(derivative))
                {
                    throw std::domain_error(
                        "Connector-load external-force derivative is non-finite.");
                }
            }
            return result;
        }

        [[nodiscard]] bool aggregateResistanceNeedsDistribution(
            const BasicResistance& resistance) noexcept
        {
            return resistance.constantMechanicalForceNewtons > 0.0
                || resistance.linearResistanceCoefficientNewtonSecondsPerMeter
                    > 0.0
                || resistance.rollingResistanceCoefficient > 0.0
                || (resistance.airDensityKilogramsPerCubicMeter > 0.0
                    && resistance.dragAreaSquareMeters > 0.0);
        }

        [[nodiscard]] bool explicitAerodynamicsConfigured(
            const TrainDefinition& definition) noexcept
        {
            return std::any_of(
                definition.cars.begin(),
                definition.cars.end(),
                [](const TrainCarDefinition& car)
                {
                    return car.car.aerodynamicDragAreaSquareMeters > 0.0;
                });
        }

        [[nodiscard]] std::vector<RigidConnectorLoad> unavailableLoads(
            const TrainPose& pose)
        {
            std::vector<RigidConnectorLoad> result;
            result.reserve(pose.connectionCount());
            for (const InterCarConnectionPose& connection
                : pose.connections())
            {
                result.emplace_back(
                    connection.connectionIndex(),
                    connection.leadingCarIndex(),
                    connection.followingCarIndex(),
                    RigidConnectorLoadClassification::Unavailable,
                    std::nullopt,
                    std::nullopt,
                    connection.worldDirection(),
                    std::nullopt,
                    std::nullopt,
                    connection.signedLengthResidualMeters());
            }
            return result;
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
        if (explicitAerodynamicsConfigured(definition)
            && definition.resistance.dragAreaSquareMeters > 0.0)
        {
            throw std::invalid_argument(
                "Per-car aerodynamic CdA cannot be combined with the aggregate BasicResistance aerodynamic coefficient.");
        }
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

    RigidConnectorLoad::RigidConnectorLoad(
        const std::size_t connectionIndex,
        const std::size_t leadingCarIndex,
        const std::size_t followingCarIndex,
        const RigidConnectorLoadClassification classification,
        std::optional<double> axialForceNewtons,
        std::optional<double> absoluteAxialLoadNewtons,
        std::optional<glm::dvec3> worldDirection,
        std::optional<glm::dvec3> worldForceOnLeadingCarNewtons,
        std::optional<glm::dvec3> worldForceOnFollowingCarNewtons,
        const double connectorClosureResidualMeters)
        : connectionIndex_(connectionIndex),
          leadingCarIndex_(leadingCarIndex),
          followingCarIndex_(followingCarIndex),
          classification_(classification),
          axialForceNewtons_(axialForceNewtons),
          absoluteAxialLoadNewtons_(absoluteAxialLoadNewtons),
          worldDirection_(std::move(worldDirection)),
          worldForceOnLeadingCarNewtons_(
              std::move(worldForceOnLeadingCarNewtons)),
          worldForceOnFollowingCarNewtons_(
              std::move(worldForceOnFollowingCarNewtons)),
          connectorClosureResidualMeters_(connectorClosureResidualMeters)
    {
    }

    std::size_t RigidConnectorLoad::connectionIndex() const noexcept
    {
        return connectionIndex_;
    }

    std::size_t RigidConnectorLoad::leadingCarIndex() const noexcept
    {
        return leadingCarIndex_;
    }

    std::size_t RigidConnectorLoad::followingCarIndex() const noexcept
    {
        return followingCarIndex_;
    }

    RigidConnectorLoadClassification
    RigidConnectorLoad::classification() const noexcept
    {
        return classification_;
    }

    const std::optional<double>& RigidConnectorLoad::axialForceNewtons()
        const noexcept
    {
        return axialForceNewtons_;
    }

    const std::optional<double>&
    RigidConnectorLoad::absoluteAxialLoadNewtons() const noexcept
    {
        return absoluteAxialLoadNewtons_;
    }

    const std::optional<glm::dvec3>& RigidConnectorLoad::worldDirection()
        const noexcept
    {
        return worldDirection_;
    }

    const std::optional<glm::dvec3>&
    RigidConnectorLoad::worldForceOnLeadingCarNewtons() const noexcept
    {
        return worldForceOnLeadingCarNewtons_;
    }

    const std::optional<glm::dvec3>&
    RigidConnectorLoad::worldForceOnFollowingCarNewtons() const noexcept
    {
        return worldForceOnFollowingCarNewtons_;
    }

    double RigidConnectorLoad::connectorClosureResidualMeters() const noexcept
    {
        return connectorClosureResidualMeters_;
    }

    RigidConnectorLoadAnalysis::RigidConnectorLoadAnalysis(
        std::vector<RigidConnectorLoad> connectorLoads,
        const RigidConnectorLoadRecoveryStatus status,
        std::optional<double> maximumAbsoluteLoadNewtons,
        std::optional<std::size_t> maximumAbsoluteLoadConnectionIndex,
        const double maximumTensionNewtons,
        std::optional<std::size_t> maximumTensionConnectionIndex,
        const double maximumCompressionMagnitudeNewtons,
        std::optional<std::size_t> maximumCompressionConnectionIndex,
        std::optional<double> balanceResidualNewtons,
        const double balanceToleranceNewtons,
        const double minimumUsedAxialProjection,
        const double localDerivativeStepMeters,
        std::vector<TrainFiniteDifferenceKind> localDerivativeKinds,
        const TrainFiniteDifferenceKind constrainedDerivativeKind)
        : connectorLoads_(std::move(connectorLoads)),
          status_(status),
          maximumAbsoluteLoadNewtons_(maximumAbsoluteLoadNewtons),
          maximumAbsoluteLoadConnectionIndex_(
              maximumAbsoluteLoadConnectionIndex),
          maximumTensionNewtons_(maximumTensionNewtons),
          maximumTensionConnectionIndex_(maximumTensionConnectionIndex),
          maximumCompressionMagnitudeNewtons_(
              maximumCompressionMagnitudeNewtons),
          maximumCompressionConnectionIndex_(
              maximumCompressionConnectionIndex),
          balanceResidualNewtons_(balanceResidualNewtons),
          balanceToleranceNewtons_(balanceToleranceNewtons),
          minimumUsedAxialProjection_(minimumUsedAxialProjection),
          localDerivativeStepMeters_(localDerivativeStepMeters),
          localDerivativeKinds_(std::move(localDerivativeKinds)),
          constrainedDerivativeKind_(constrainedDerivativeKind)
    {
    }

    const std::vector<RigidConnectorLoad>&
    RigidConnectorLoadAnalysis::connectorLoads() const noexcept
    {
        return connectorLoads_;
    }

    bool RigidConnectorLoadAnalysis::exactRecoveryAvailable() const noexcept
    {
        return status_ == RigidConnectorLoadRecoveryStatus::Available;
    }

    RigidConnectorLoadRecoveryStatus RigidConnectorLoadAnalysis::status()
        const noexcept
    {
        return status_;
    }

    const std::optional<double>&
    RigidConnectorLoadAnalysis::maximumAbsoluteLoadNewtons() const noexcept
    {
        return maximumAbsoluteLoadNewtons_;
    }

    const std::optional<std::size_t>& RigidConnectorLoadAnalysis::
        maximumAbsoluteLoadConnectionIndex() const noexcept
    {
        return maximumAbsoluteLoadConnectionIndex_;
    }

    double RigidConnectorLoadAnalysis::maximumTensionNewtons() const noexcept
    {
        return maximumTensionNewtons_;
    }

    const std::optional<std::size_t>& RigidConnectorLoadAnalysis::
        maximumTensionConnectionIndex() const noexcept
    {
        return maximumTensionConnectionIndex_;
    }

    double RigidConnectorLoadAnalysis::maximumCompressionMagnitudeNewtons()
        const noexcept
    {
        return maximumCompressionMagnitudeNewtons_;
    }

    const std::optional<std::size_t>& RigidConnectorLoadAnalysis::
        maximumCompressionConnectionIndex() const noexcept
    {
        return maximumCompressionConnectionIndex_;
    }

    const std::optional<double>&
    RigidConnectorLoadAnalysis::balanceResidualNewtons() const noexcept
    {
        return balanceResidualNewtons_;
    }

    double RigidConnectorLoadAnalysis::balanceToleranceNewtons() const noexcept
    {
        return balanceToleranceNewtons_;
    }

    double RigidConnectorLoadAnalysis::minimumUsedAxialProjection()
        const noexcept
    {
        return minimumUsedAxialProjection_;
    }

    double RigidConnectorLoadAnalysis::localDerivativeStepMeters()
        const noexcept
    {
        return localDerivativeStepMeters_;
    }

    const std::vector<TrainFiniteDifferenceKind>&
    RigidConnectorLoadAnalysis::localDerivativeKinds() const noexcept
    {
        return localDerivativeKinds_;
    }

    TrainFiniteDifferenceKind
    RigidConnectorLoadAnalysis::constrainedDerivativeKind() const noexcept
    {
        return constrainedDerivativeKind_;
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
        const TrackLocation& generalizedReferenceLocation,
        const std::span<const ExternalForceApplication> externalForces)
    {
        validatePhysicsEnvironment(environment);
        TrainPose center = solveTrainPose(
            track, definition, generalizedReferenceLocation);
        validateExternalForceApplications(externalForces, center.carCount());
        DerivativeSamples derivatives = kinematicDerivatives(
            track, definition, center, externalForces);
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
            cars.push_back({
                index,
                jacobian,
                secondDerivative,
                carGravity
            });
        }

        std::vector<ExternalForceApplicationEvaluation>
            externalForceEvaluations;
        externalForceEvaluations.reserve(externalForces.size());
        double generalizedExternalForce = 0.0;
        for (std::size_t index = 0; index < externalForces.size(); ++index)
        {
            const ExternalForceApplication& application =
                externalForces[index];
            const glm::dvec3& derivative =
                derivatives.applicationPointFirst[index];
            // Virtual work in the one train coordinate: Q = F dot dp/dq.
            const double generalizedForce = glm::dot(
                application.worldForceNewtons, derivative);
            if (!finite(derivative) || !std::isfinite(generalizedForce))
            {
                throw std::domain_error(
                    "External force generalized projection is non-finite.");
            }
            generalizedExternalForce += generalizedForce;
            externalForceEvaluations.push_back({
                application.carIndex,
                derivatives.applicationPoints[index],
                derivative,
                generalizedForce
            });
        }
        if (!std::isfinite(effectiveMass)
            || effectiveMass <= 0.0
            || !std::isfinite(effectiveMassDerivative)
            || !std::isfinite(gravityForce)
            || !std::isfinite(generalizedExternalForce))
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
            std::move(externalForceEvaluations),
            generalizedExternalForce,
            trainKinematicJacobianStepMeters,
            derivatives.kind
        };
    }

    ExplicitResistanceTelemetry generateExplicitResistanceForces(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrainDynamicsState& state,
        std::vector<ExternalForceApplication>& outputForces)
    {
        validateTrainDefinition(definition);
        validatePhysicsEnvironment(environment);
        if (!std::isfinite(state.signedVelocityMetersPerSecond))
        {
            throw std::invalid_argument(
                "Explicit resistance generation requires a finite generalized velocity.");
        }

        outputForces.clear();
        outputForces.reserve(definition.cars.size());
        for (std::size_t carIndex = 0;
            carIndex < definition.cars.size();
            ++carIndex)
        {
            const CarDefinition& car = definition.cars[carIndex].car;
            if (car.aerodynamicDragAreaSquareMeters == 0.0)
            {
                continue;
            }
            outputForces.push_back({
                carIndex,
                car.aerodynamicCenterLocalMeters,
                glm::dvec3{0.0}
            });
        }

        if (outputForces.empty())
        {
            return {};
        }

        const TrainPose center = solveTrainPose(
            track, definition, state.generalizedReferenceLocation);
        DerivativeSamples derivatives = kinematicDerivatives(
            track, definition, center, outputForces);
        double generalizedAerodynamicForce = 0.0;
        for (std::size_t index = 0; index < outputForces.size(); ++index)
        {
            ExternalForceApplication& application = outputForces[index];
            const glm::dvec3 pointVelocity =
                derivatives.applicationPointFirst[index]
                * state.signedVelocityMetersPerSecond;
            const glm::dvec3 relativeAirVelocity = pointVelocity
                - environment.windVelocityMetersPerSecond;
            const double relativeSpeedSquared = glm::dot(
                relativeAirVelocity, relativeAirVelocity);
            if (!finite(pointVelocity)
                || !finite(relativeAirVelocity)
                || !std::isfinite(relativeSpeedSquared)
                || relativeSpeedSquared < 0.0)
            {
                throw std::domain_error(
                    "Aerodynamic resistance velocity is non-finite.");
            }

            if (relativeSpeedSquared == 0.0)
            {
                application.worldForceNewtons = glm::dvec3{0.0};
            }
            else
            {
                const double relativeSpeed = std::sqrt(relativeSpeedSquared);
                const double dragScale = -0.5
                    * environment.airDensityKilogramsPerCubicMeter
                    * definition.cars[application.carIndex].car
                        .aerodynamicDragAreaSquareMeters
                    * relativeSpeed;
                application.worldForceNewtons =
                    dragScale * relativeAirVelocity;
            }

            const double generalizedForce = glm::dot(
                application.worldForceNewtons,
                derivatives.applicationPointFirst[index]);
            if (!finite(application.worldForceNewtons)
                || !std::isfinite(generalizedForce))
            {
                throw std::domain_error(
                    "Aerodynamic resistance produced a non-finite force.");
            }
            generalizedAerodynamicForce += generalizedForce;
        }
        if (!std::isfinite(generalizedAerodynamicForce))
        {
            throw std::domain_error(
                "Aerodynamic resistance generalized force is non-finite.");
        }

        return {
            outputForces.size(),
            outputForces.size(),
            generalizedAerodynamicForce,
            generalizedAerodynamicForce,
            trainKinematicJacobianStepMeters,
            derivatives.kind
        };
    }

    RigidConnectorLoadAnalysis evaluateRigidConnectorLoads(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrainDynamicsState& state,
        const std::span<const ExternalForceApplication> externalForces)
    {
        validateTrainDefinition(definition);
        validatePhysicsEnvironment(environment);
        if (!std::isfinite(state.signedVelocityMetersPerSecond)
            || !std::isfinite(
                state.generalizedAccelerationMetersPerSecondSquared)
            || !validRunState(state.runState))
        {
            throw std::invalid_argument(
                "Connector-load analysis requires a finite, valid train state.");
        }
        validateExternalForceApplications(
            externalForces, definition.cars.size());

        const TrainPose center = solveTrainPose(
            track, definition, state.generalizedReferenceLocation);
        const auto unavailable = [&center](
            const RigidConnectorLoadRecoveryStatus status,
            std::optional<double> residual = std::nullopt,
            const double tolerance = 0.0,
            const double minimumProjection = 0.0,
            std::vector<TrainFiniteDifferenceKind> localKinds = {},
            const TrainFiniteDifferenceKind constrainedKind =
                TrainFiniteDifferenceKind::Central)
        {
            return RigidConnectorLoadAnalysis{
                unavailableLoads(center),
                status,
                std::nullopt,
                std::nullopt,
                0.0,
                std::nullopt,
                0.0,
                std::nullopt,
                residual,
                tolerance,
                minimumProjection,
                connectorLoadLocalDerivativeStepMeters,
                std::move(localKinds),
                constrainedKind
            };
        };

        if (center.connectionCount() == 0)
        {
            return {
                {},
                RigidConnectorLoadRecoveryStatus::Available,
                std::nullopt,
                std::nullopt,
                0.0,
                std::nullopt,
                0.0,
                std::nullopt,
                std::nullopt,
                0.0,
                0.0,
                connectorLoadLocalDerivativeStepMeters,
                {},
                TrainFiniteDifferenceKind::Central
            };
        }

        if (aggregateResistanceNeedsDistribution(definition.resistance))
        {
            return unavailable(
                RigidConnectorLoadRecoveryStatus::
                    AggregateResistanceUnderdetermined);
        }

        for (const InterCarConnectionPose& connection
            : center.connections())
        {
            if (connection.authoredRigidLengthMeters()
                    < connectorLoadMinimumDirectionalLengthMeters
                || !connection.worldDirection()
                || !finite(*connection.worldDirection()))
            {
                return unavailable(
                    RigidConnectorLoadRecoveryStatus::UndefinedConnectorAxis);
            }
        }

        const DerivativeSamples constrained = kinematicDerivatives(
            track, definition, center, {});
        std::vector<std::vector<std::size_t>> forceIndicesByCar(
            center.carCount());
        for (std::size_t index = 0; index < externalForces.size(); ++index)
        {
            forceIndicesByCar[externalForces[index].carIndex].push_back(index);
        }
        std::vector<CarLocalDerivativeSamples> local;
        std::vector<TrainFiniteDifferenceKind> localKinds;
        local.reserve(center.carCount());
        localKinds.reserve(center.carCount());
        for (std::size_t index = 0; index < center.carCount(); ++index)
        {
            local.push_back(localCarDerivatives(
                track,
                definition.cars[index],
                center.cars()[index].carPose(),
                externalForces,
                forceIndicesByCar[index]));
            localKinds.push_back(local.back().kind);
        }

        const double velocitySquared =
            state.signedVelocityMetersPerSecond
            * state.signedVelocityMetersPerSecond;
        if (!std::isfinite(velocitySquared))
        {
            throw std::invalid_argument(
                "Connector-load speed is too large to square as a finite value.");
        }
        const glm::dvec3 gravityWorld{
            0.0,
            0.0,
            -environment.gravityAccelerationMetersPerSecondSquared
        };

        const std::size_t carCount = center.carCount();
        std::vector<double> requiredGeneralizedForce(carCount, 0.0);
        std::vector<double> knownExternalGeneralizedForce(carCount, 0.0);
        std::vector<double> previousConnectorCoefficient(carCount, 0.0);
        std::vector<double> nextConnectorCoefficient(carCount, 0.0);
        for (std::size_t index = 0; index < carCount; ++index)
        {
            for (std::size_t localIndex = 0;
                localIndex < forceIndicesByCar[index].size();
                ++localIndex)
            {
                const ExternalForceApplication& application = externalForces[
                    forceIndicesByCar[index][localIndex]];
                const double contribution = glm::dot(
                    application.worldForceNewtons,
                    local[index].applicationPointDerivatives[localIndex]);
                if (!std::isfinite(contribution))
                {
                    throw std::domain_error(
                        "Connector-load external-force projection is non-finite.");
                }
                knownExternalGeneralizedForce[index] += contribution;
            }
            const glm::dvec3 acceleration =
                constrained.first[index]
                    * state.generalizedAccelerationMetersPerSecondSquared
                + constrained.second[index] * velocitySquared;
            const double mass = center.cars()[index].loadedMassKilograms();
            // The remaining generalized force must be supplied by adjacent
            // connector axes after gravity and each known F dot dp/ds_i term.
            requiredGeneralizedForce[index] = mass * glm::dot(
                acceleration - gravityWorld,
                local[index].centerOfGravity)
                - knownExternalGeneralizedForce[index];
            if (!finite(acceleration)
                || !std::isfinite(requiredGeneralizedForce[index]))
            {
                throw std::domain_error(
                    "Connector-load car force balance is non-finite.");
            }
        }

        for (std::size_t connectionIndex = 0;
            connectionIndex < center.connectionCount();
            ++connectionIndex)
        {
            const glm::dvec3& direction =
                *center.connections()[connectionIndex].worldDirection();
            nextConnectorCoefficient[connectionIndex] = glm::dot(
                direction, local[connectionIndex].rearHitch);
            previousConnectorCoefficient[connectionIndex + 1] = -glm::dot(
                direction, local[connectionIndex + 1].frontHitch);
            if (!std::isfinite(nextConnectorCoefficient[connectionIndex])
                || !std::isfinite(
                    previousConnectorCoefficient[connectionIndex + 1]))
            {
                throw std::domain_error(
                    "Connector-load hitch projection is non-finite.");
            }
        }

        std::size_t omittedRow = 0;
        double bestMinimumProjection = -1.0;
        for (std::size_t candidate = 0; candidate < carCount; ++candidate)
        {
            double minimumProjection =
                std::numeric_limits<double>::infinity();
            for (std::size_t connectionIndex = 0;
                connectionIndex < candidate;
                ++connectionIndex)
            {
                minimumProjection = std::min(
                    minimumProjection,
                    std::abs(nextConnectorCoefficient[connectionIndex]));
            }
            for (std::size_t connectionIndex = candidate;
                connectionIndex < center.connectionCount();
                ++connectionIndex)
            {
                minimumProjection = std::min(
                    minimumProjection,
                    std::abs(previousConnectorCoefficient[
                        connectionIndex + 1]));
            }
            if (minimumProjection > bestMinimumProjection)
            {
                bestMinimumProjection = minimumProjection;
                omittedRow = candidate;
            }
        }

        if (!std::isfinite(bestMinimumProjection)
            || bestMinimumProjection < connectorLoadMinimumAxialProjection)
        {
            return unavailable(
                RigidConnectorLoadRecoveryStatus::IllConditioned,
                std::nullopt,
                0.0,
                std::max(0.0, bestMinimumProjection),
                std::move(localKinds),
                constrained.kind);
        }

        std::vector<double> axialForces(center.connectionCount(), 0.0);
        for (std::size_t carIndex = 0;
            carIndex < omittedRow;
            ++carIndex)
        {
            const double previousContribution = carIndex == 0
                ? 0.0
                : previousConnectorCoefficient[carIndex]
                    * axialForces[carIndex - 1];
            axialForces[carIndex] =
                (requiredGeneralizedForce[carIndex]
                    - previousContribution)
                / nextConnectorCoefficient[carIndex];
        }
        for (std::size_t carIndex = carCount - 1;
            carIndex > omittedRow;
            --carIndex)
        {
            const double nextContribution = carIndex + 1 == carCount
                ? 0.0
                : nextConnectorCoefficient[carIndex]
                    * axialForces[carIndex];
            axialForces[carIndex - 1] =
                (requiredGeneralizedForce[carIndex] - nextContribution)
                / previousConnectorCoefficient[carIndex];
        }

        double recoveredContribution = 0.0;
        if (omittedRow > 0)
        {
            recoveredContribution += previousConnectorCoefficient[omittedRow]
                * axialForces[omittedRow - 1];
        }
        if (omittedRow < center.connectionCount())
        {
            recoveredContribution += nextConnectorCoefficient[omittedRow]
                * axialForces[omittedRow];
        }
        const double balanceResidual =
            requiredGeneralizedForce[omittedRow] - recoveredContribution;

        double balanceScale = 1.0;
        for (const double value : requiredGeneralizedForce)
        {
            balanceScale = std::max(balanceScale, std::abs(value));
        }
        for (const double value : axialForces)
        {
            if (!std::isfinite(value))
            {
                return unavailable(
                    RigidConnectorLoadRecoveryStatus::IllConditioned,
                    std::nullopt,
                    0.0,
                    bestMinimumProjection,
                    std::move(localKinds),
                    constrained.kind);
            }
        }
        for (std::size_t carIndex = 0; carIndex < carCount; ++carIndex)
        {
            double connectorScale = 0.0;
            if (carIndex > 0)
            {
                connectorScale += std::abs(
                    previousConnectorCoefficient[carIndex]
                    * axialForces[carIndex - 1]);
            }
            if (carIndex < center.connectionCount())
            {
                connectorScale += std::abs(
                    nextConnectorCoefficient[carIndex]
                    * axialForces[carIndex]);
            }
            balanceScale = std::max(balanceScale, connectorScale);
        }
        const double balanceTolerance =
            connectorLoadBalanceAbsoluteToleranceNewtons
            + connectorLoadBalanceRelativeTolerance * balanceScale;
        if (!std::isfinite(balanceResidual)
            || !std::isfinite(balanceTolerance))
        {
            throw std::domain_error(
                "Connector-load balance diagnostics are non-finite.");
        }
        if (std::abs(balanceResidual) > balanceTolerance)
        {
            return unavailable(
                RigidConnectorLoadRecoveryStatus::InconsistentBalance,
                balanceResidual,
                balanceTolerance,
                bestMinimumProjection,
                std::move(localKinds),
                constrained.kind);
        }

        std::vector<RigidConnectorLoad> loads;
        loads.reserve(center.connectionCount());
        double maximumAbsolute = 0.0;
        std::size_t maximumAbsoluteIndex = 0;
        double maximumTension = 0.0;
        std::optional<std::size_t> maximumTensionIndex;
        double maximumCompression = 0.0;
        std::optional<std::size_t> maximumCompressionIndex;
        for (std::size_t connectionIndex = 0;
            connectionIndex < center.connectionCount();
            ++connectionIndex)
        {
            const double axialForce = axialForces[connectionIndex];
            const double absoluteLoad = std::abs(axialForce);
            RigidConnectorLoadClassification classification =
                RigidConnectorLoadClassification::NearZero;
            if (axialForce > connectorLoadClassificationToleranceNewtons)
            {
                classification = RigidConnectorLoadClassification::Tension;
            }
            else if (axialForce
                < -connectorLoadClassificationToleranceNewtons)
            {
                classification =
                    RigidConnectorLoadClassification::Compression;
            }

            const glm::dvec3& direction =
                *center.connections()[connectionIndex].worldDirection();
            const glm::dvec3 forceOnLeading = axialForce * direction;
            const glm::dvec3 forceOnFollowing = -forceOnLeading;
            if (!finite(forceOnLeading) || !finite(forceOnFollowing))
            {
                throw std::domain_error(
                    "Connector-load world force vector is non-finite.");
            }

            loads.emplace_back(
                connectionIndex,
                connectionIndex,
                connectionIndex + 1,
                classification,
                axialForce,
                absoluteLoad,
                direction,
                forceOnLeading,
                forceOnFollowing,
                center.connections()[connectionIndex]
                    .signedLengthResidualMeters());

            if (connectionIndex == 0 || absoluteLoad > maximumAbsolute)
            {
                maximumAbsolute = absoluteLoad;
                maximumAbsoluteIndex = connectionIndex;
            }
            if (axialForce > maximumTension)
            {
                maximumTension = axialForce;
                maximumTensionIndex = connectionIndex;
            }
            if (-axialForce > maximumCompression)
            {
                maximumCompression = -axialForce;
                maximumCompressionIndex = connectionIndex;
            }
        }

        return {
            std::move(loads),
            RigidConnectorLoadRecoveryStatus::Available,
            maximumAbsolute,
            maximumAbsoluteIndex,
            maximumTension,
            maximumTensionIndex,
            maximumCompression,
            maximumCompressionIndex,
            balanceResidual,
            balanceTolerance,
            bestMinimumProjection,
            connectorLoadLocalDerivativeStepMeters,
            std::move(localKinds),
            constrained.kind
        };
    }

    BogieReactionAnalysis evaluateBogieReactions(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrainDynamicsState& state,
        const std::span<const ExternalForceApplication> externalForces)
    {
        validateTrainDefinition(definition);
        validatePhysicsEnvironment(environment);
        if (!std::isfinite(state.signedVelocityMetersPerSecond)
            || !std::isfinite(
                state.generalizedAccelerationMetersPerSecondSquared)
            || !validRunState(state.runState))
        {
            throw std::invalid_argument(
                "Bogie-reaction analysis requires a finite, valid train state.");
        }
        validateExternalForceApplications(
            externalForces, definition.cars.size());

        const auto makeBogie = [](
            const std::size_t carIndex,
            const BogiePose& pose,
            const BogieRole role,
            const BogieReactionRecoveryStatus status)
        {
            return BogieReaction{
                carIndex,
                pose.definitionIndex(),
                role,
                pose.location(),
                pose.worldPositionMeters(),
                pose.trackFrame(),
                status,
                std::nullopt,
                std::nullopt,
                std::nullopt
            };
        };

        const auto unavailableCars = [&makeBogie](
            const TrainPose& pose,
            const CarTrackReactionRecoveryStatus status)
        {
            std::vector<CarTrackReaction> result;
            result.reserve(pose.carCount());
            for (const TrainCarPose& trainCar : pose.cars())
            {
                const std::size_t carIndex = trainCar.carIndex();
                const CarPose& carPose = trainCar.carPose();
                result.push_back({
                    carIndex,
                    status,
                    carPose.worldCenterOfGravityMeters(),
                    std::nullopt,
                    std::nullopt,
                    std::nullopt,
                    std::nullopt,
                    makeBogie(
                        carIndex,
                        carPose.frontBogie(),
                        BogieRole::Front,
                        BogieReactionRecoveryStatus::
                            AggregateCarReactionUnavailable),
                    makeBogie(
                        carIndex,
                        carPose.rearBogie(),
                        BogieRole::Rear,
                        BogieReactionRecoveryStatus::
                            AggregateCarReactionUnavailable),
                    std::nullopt,
                    0.0
                });
            }
            return result;
        };

        std::optional<TrainKinematicEvaluation> kinematics;
        try
        {
            kinematics.emplace(evaluateTrainKinematics(
                track,
                definition,
                environment,
                state.generalizedReferenceLocation,
                externalForces));
        }
        catch (const OpenConsistBoundaryError&)
        {
            const TrainPose pose = solveTrainPose(
                track, definition, state.generalizedReferenceLocation);
            return {
                CarTrackReactionRecoveryStatus::KinematicsUnavailable,
                RigidConnectorLoadRecoveryStatus::Available,
                unavailableCars(
                    pose,
                    CarTrackReactionRecoveryStatus::KinematicsUnavailable),
                std::nullopt,
                0.0,
                trainKinematicJacobianStepMeters,
                TrainFiniteDifferenceKind::Central
            };
        }

        const double velocitySquared =
            state.signedVelocityMetersPerSecond
            * state.signedVelocityMetersPerSecond;
        if (!std::isfinite(velocitySquared))
        {
            throw std::invalid_argument(
                "Bogie-reaction speed is too large to square as a finite value.");
        }

        const double generalizedRequiredForce =
            kinematics->effectiveGeneralizedMassKilograms
                * state.generalizedAccelerationMetersPerSecondSquared
            + 0.5
                * kinematics->effectiveGeneralizedMassDerivativeKilogramsPerMeter
                * velocitySquared;
        const double generalizedKnownForce =
            kinematics->generalizedGravityForceNewtons
            + kinematics->generalizedExternalForceNewtons;
        const double generalizedBalanceResidual =
            generalizedRequiredForce - generalizedKnownForce;
        const double generalizedBalanceScale = std::max({
            1.0,
            std::abs(generalizedRequiredForce),
            std::abs(generalizedKnownForce)
        });
        const double generalizedBalanceTolerance =
            bogieReactionBalanceAbsoluteToleranceNewtons
            + bogieReactionBalanceRelativeTolerance
                * generalizedBalanceScale;
        if (!std::isfinite(generalizedRequiredForce)
            || !std::isfinite(generalizedKnownForce)
            || !std::isfinite(generalizedBalanceResidual)
            || !std::isfinite(generalizedBalanceTolerance))
        {
            throw std::domain_error(
                "Bogie-reaction generalized balance is non-finite.");
        }

        const auto unavailable = [&kinematics, &unavailableCars,
                                  generalizedBalanceResidual,
                                  generalizedBalanceTolerance](
            const CarTrackReactionRecoveryStatus status,
            const RigidConnectorLoadRecoveryStatus connectorStatus)
        {
            return BogieReactionAnalysis{
                status,
                connectorStatus,
                unavailableCars(kinematics->pose, status),
                generalizedBalanceResidual,
                generalizedBalanceTolerance,
                kinematics->finiteDifferenceStepMeters,
                kinematics->finiteDifferenceKind
            };
        };

        // Aggregate BasicResistance has neither a per-car allocation nor a
        // world-space application point. Treat any configured force-producing
        // law conservatively, matching Phase 4 connector-load recovery.
        if (aggregateResistanceNeedsDistribution(definition.resistance))
        {
            const RigidConnectorLoadRecoveryStatus connectorStatus =
                definition.connections.empty()
                ? RigidConnectorLoadRecoveryStatus::Available
                : RigidConnectorLoadRecoveryStatus::
                    AggregateResistanceUnderdetermined;
            return unavailable(
                CarTrackReactionRecoveryStatus::
                    AggregateResistanceUnderdetermined,
                connectorStatus);
        }

        if (std::abs(generalizedBalanceResidual)
            > generalizedBalanceTolerance)
        {
            return unavailable(
                CarTrackReactionRecoveryStatus::
                    InconsistentGeneralizedBalance,
                RigidConnectorLoadRecoveryStatus::InconsistentBalance);
        }

        const RigidConnectorLoadAnalysis connectorLoads =
            evaluateRigidConnectorLoads(
                track,
                definition,
                environment,
                state,
                externalForces);
        if (!connectorLoads.exactRecoveryAvailable())
        {
            return unavailable(
                CarTrackReactionRecoveryStatus::
                    ConnectorLoadRecoveryUnavailable,
                connectorLoads.status());
        }

        std::vector<glm::dvec3> connectorForces(
            definition.cars.size(), glm::dvec3{0.0});
        for (const RigidConnectorLoad& load
            : connectorLoads.connectorLoads())
        {
            if (!load.worldForceOnLeadingCarNewtons()
                || !load.worldForceOnFollowingCarNewtons())
            {
                return unavailable(
                    CarTrackReactionRecoveryStatus::
                        ConnectorLoadRecoveryUnavailable,
                    connectorLoads.status());
            }
            connectorForces[load.leadingCarIndex()] +=
                *load.worldForceOnLeadingCarNewtons();
            connectorForces[load.followingCarIndex()] +=
                *load.worldForceOnFollowingCarNewtons();
        }

        std::vector<glm::dvec3> appliedForces(
            definition.cars.size(), glm::dvec3{0.0});
        for (const ExternalForceApplication& application : externalForces)
        {
            appliedForces[application.carIndex] +=
                application.worldForceNewtons;
        }

        const glm::dvec3 gravityAcceleration{
            0.0,
            0.0,
            -environment.gravityAccelerationMetersPerSecondSquared
        };
        std::vector<CarTrackReaction> cars;
        cars.reserve(definition.cars.size());
        for (std::size_t carIndex = 0;
            carIndex < definition.cars.size();
            ++carIndex)
        {
            const TrainCarPose& trainCar =
                kinematics->pose.cars()[carIndex];
            const CarPose& carPose = trainCar.carPose();
            const TrainCarKinematics& carKinematics =
                kinematics->cars[carIndex];
            const glm::dvec3 acceleration =
                carKinematics
                    .worldCenterOfGravityDerivativePerGeneralizedMeter
                    * state.generalizedAccelerationMetersPerSecondSquared
                + carKinematics
                    .worldCenterOfGravitySecondDerivativePerGeneralizedMeterSquared
                    * velocitySquared;
            const double mass = trainCar.loadedMassKilograms();
            const glm::dvec3 gravityForce = mass * gravityAcceleration;
            const glm::dvec3 inertialForce = mass * acceleration;
            const glm::dvec3 reaction = inertialForce
                - gravityForce
                - appliedForces[carIndex]
                - connectorForces[carIndex];
            const glm::dvec3 forceBalanceResidual = inertialForce
                - (gravityForce
                    + appliedForces[carIndex]
                    + connectorForces[carIndex]
                    + reaction);
            const double forceBalanceScale = std::max({
                1.0,
                glm::length(inertialForce),
                glm::length(gravityForce),
                glm::length(appliedForces[carIndex]),
                glm::length(connectorForces[carIndex]),
                glm::length(reaction)
            });
            const double forceBalanceTolerance =
                bogieReactionBalanceAbsoluteToleranceNewtons
                + bogieReactionBalanceRelativeTolerance
                    * forceBalanceScale;
            const geometry::CurveFrame& bodyFrame = carPose.bodyFrame();
            const glm::dvec3 bodyComponents{
                glm::dot(reaction, bodyFrame.tangent),
                glm::dot(reaction, bodyFrame.lateral),
                glm::dot(reaction, bodyFrame.up)
            };
            const double bogieSeparation = glm::length(
                carPose.frontBogie().worldPositionMeters()
                - carPose.rearBogie().worldPositionMeters());
            if (!finite(acceleration)
                || !finite(reaction)
                || !finite(forceBalanceResidual)
                || !finite(bodyComponents)
                || !std::isfinite(forceBalanceScale)
                || !std::isfinite(forceBalanceTolerance)
                || !std::isfinite(bogieSeparation))
            {
                throw std::domain_error(
                    "Bogie-reaction car balance is non-finite.");
            }

            const BogieReactionRecoveryStatus bogieStatus =
                bogieSeparation < bogieReactionMinimumSeparationMeters
                ? BogieReactionRecoveryStatus::SingularGeometry
                : BogieReactionRecoveryStatus::MissingRotationalModel;
            cars.push_back({
                carIndex,
                CarTrackReactionRecoveryStatus::Available,
                carPose.worldCenterOfGravityMeters(),
                acceleration,
                reaction,
                glm::length(reaction),
                bodyComponents,
                makeBogie(
                    carIndex,
                    carPose.frontBogie(),
                    BogieRole::Front,
                    bogieStatus),
                makeBogie(
                    carIndex,
                    carPose.rearBogie(),
                    BogieRole::Rear,
                    bogieStatus),
                forceBalanceResidual,
                forceBalanceTolerance
            });
        }

        return {
            CarTrackReactionRecoveryStatus::Available,
            connectorLoads.status(),
            std::move(cars),
            generalizedBalanceResidual,
            generalizedBalanceTolerance,
            kinematics->finiteDifferenceStepMeters,
            kinematics->finiteDifferenceKind
        };
    }

    TrainStepResult stepTrain(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrainDynamicsState& currentState,
        const FixedStepSettings& step,
        const std::span<const ExternalForceApplication> externalForces)
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
            currentState.generalizedReferenceLocation,
            externalForces);
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
            current.generalizedGravityForceNewtons
                + current.generalizedExternalForceNewtons,
            workingVelocity);
        const double massGradientForce = -0.5
            * current.effectiveGeneralizedMassDerivativeKilogramsPerMeter
            * workingVelocity * workingVelocity;
        const double unconstrainedForce =
            current.generalizedGravityForceNewtons
            + current.generalizedExternalForceNewtons
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
            current.generalizedExternalForceNewtons,
            externalForces.size(),
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
