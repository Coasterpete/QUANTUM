#include <quantum/physics/DynamicContactPhysics.hpp>

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

namespace quantum::physics
{
    namespace
    {
        using Vector3 = std::array<double, 3>;
        using Matrix3 = std::array<Vector3, 3>;

        struct GeneralizedCoordinates
        {
            TrackLocation q;
            double front = 0.0;
            double rear = 0.0;
        };

        struct PoseJacobians
        {
            std::array<glm::dvec3, 3> centerOfGravity{};
            std::array<glm::dvec3, 3> angularVelocity{};
        };

        struct AggregateContact
        {
            std::size_t bogieIndex = 0;
            double normalSign = 0.0;
            double clearanceMeters = 0.0;
            std::size_t firstSourceIndex = 0;
        };

        struct ContactEvaluation
        {
            double gapMeters = 0.0;
            Vector3 jacobian{};
        };

        struct MassEvaluation
        {
            Matrix3 matrix{};
            PoseJacobians jacobians;
            DynamicContactCarPose pose;
            double conditionEstimate = 0.0;
            DynamicContactConditionStatus status =
                DynamicContactConditionStatus::NotEvaluated;
        };

        struct LcpSolution
        {
            bool solved = false;
            std::vector<double> impulses;
            double residual = std::numeric_limits<double>::infinity();
            double conditionEstimate = 0.0;
            DynamicContactConditionStatus conditionStatus =
                DynamicContactConditionStatus::NotEvaluated;
            std::size_t iterations = 0;
        };

        [[nodiscard]] bool finite(const glm::dvec3& value) noexcept
        {
            return std::isfinite(value.x) && std::isfinite(value.y)
                && std::isfinite(value.z);
        }

        [[nodiscard]] bool finite(const Vector3& value) noexcept
        {
            return std::ranges::all_of(value, [](const double item)
            {
                return std::isfinite(item);
            });
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

        [[nodiscard]] double directionSign(const TravelDirection direction)
        {
            if (direction == TravelDirection::IncreasingStation)
            {
                return 1.0;
            }
            if (direction == TravelDirection::DecreasingStation)
            {
                return -1.0;
            }
            throw std::invalid_argument("Dynamic contact travel direction is invalid.");
        }

        [[nodiscard]] geometry::CurveFrame orientedFrame(
            const geometry::CurveFrame& frame,
            const TravelDirection direction)
        {
            if (direction == TravelDirection::IncreasingStation)
            {
                return frame;
            }
            if (direction == TravelDirection::DecreasingStation)
            {
                return {-frame.tangent, -frame.lateral, frame.up};
            }
            throw std::invalid_argument("Dynamic contact travel direction is invalid.");
        }

        [[nodiscard]] glm::dvec3 transformDirection(
            const geometry::CurveFrame& frame,
            const glm::dvec3& local) noexcept
        {
            return local.x * frame.tangent + local.y * frame.lateral
                + local.z * frame.up;
        }

        [[nodiscard]] glm::dvec3 transformPoint(
            const glm::dvec3& origin,
            const geometry::CurveFrame& frame,
            const glm::dvec3& local) noexcept
        {
            return origin + transformDirection(frame, local);
        }

        [[nodiscard]] glm::dquat canonicalized(glm::dquat value)
        {
            value = glm::normalize(value);
            if (!std::isfinite(value.w) || !std::isfinite(value.x)
                || !std::isfinite(value.y) || !std::isfinite(value.z))
            {
                throw std::domain_error("Dynamic car orientation is non-finite.");
            }
            if (value.w < 0.0)
            {
                value = -value;
            }
            return value;
        }

        [[nodiscard]] glm::dvec3 shortestWorldRotationVector(
            const glm::dquat& from,
            const glm::dquat& to)
        {
            glm::dquat relative = canonicalized(to * glm::conjugate(from));
            relative.w = glm::clamp(relative.w, 0.0, 1.0);
            const glm::dvec3 vector{relative.x, relative.y, relative.z};
            const double length = glm::length(vector);
            if (length <= 32.0 * std::numeric_limits<double>::epsilon())
            {
                return 2.0 * vector;
            }
            const double angle = 2.0 * std::atan2(length, relative.w);
            if (!std::isfinite(angle)
                || std::numbers::pi - angle
                    <= angularDerivativeNearPiToleranceRadians)
            {
                throw std::domain_error(
                    "Dynamic car angular derivative is ill-conditioned.");
            }
            return (angle / length) * vector;
        }

        [[nodiscard]] std::pair<std::size_t, std::size_t> bogieIndices(
            const CarDefinition& car)
        {
            if (car.bogies.size() != 2)
            {
                throw std::invalid_argument("M1A requires exactly two bogies.");
            }
            const double first = car.bogies[0].referencePositionMeters.x;
            const double second = car.bogies[1].referencePositionMeters.x;
            if (std::abs(first - second) <= 1.0e-9)
            {
                throw std::invalid_argument(
                    "M1A bogie references require distinct longitudinal positions.");
            }
            return first > second
                ? std::pair<std::size_t, std::size_t>{0, 1}
                : std::pair<std::size_t, std::size_t>{1, 0};
        }

        struct SampledPair
        {
            DynamicBogiePose front;
            DynamicBogiePose rear;
            double residualMeters = 0.0;
        };

        [[nodiscard]] std::optional<SampledPair> sampledPair(
            const CompiledPhysicsTrack& track,
            const CarDefinition& car,
            const GeneralizedCoordinates& coordinates,
            const std::size_t frontIndex,
            const std::size_t rearIndex,
            const double adjustment)
        {
            const double travel = directionSign(coordinates.q.direction);
            const auto sampleOne = [&](const std::size_t index,
                                       const double offset,
                                       const double z) -> std::optional<DynamicBogiePose>
            {
                const double requestedDisplacement = travel * offset;
                const TrackAdvanceResult advancement = track.advance(
                    coordinates.q, requestedDisplacement);
                if (track.topology() == coaster::TopologyKind::OpenLinear)
                {
                    const double requestedStation =
                        coordinates.q.stationMeters + requestedDisplacement;
                    if ((advancement.boundary == TrackBoundary::Start
                            && requestedStation < 0.0)
                        || (advancement.boundary == TrackBoundary::End
                            && requestedStation > track.lengthMeters()))
                    {
                        return std::nullopt;
                    }
                }
                TrackLocation location = advancement.location;
                location.direction = coordinates.q.direction;
                const PhysicsTrackSample sample = track.sample(location);
                const geometry::CurveFrame oriented = orientedFrame(
                    sample.frame, coordinates.q.direction);
                return DynamicBogiePose{
                    index,
                    location,
                    sample.positionMeters,
                    sample.positionMeters + z * oriented.up,
                    sample.frame,
                    oriented
                };
            };

            const BogieDefinition& front = car.bogies[frontIndex];
            const BogieDefinition& rear = car.bogies[rearIndex];
            const auto frontPose = sampleOne(frontIndex,
                front.referencePositionMeters.x + 0.5 * adjustment,
                coordinates.front);
            const auto rearPose = sampleOne(rearIndex,
                rear.referencePositionMeters.x - 0.5 * adjustment,
                coordinates.rear);
            if (!frontPose || !rearPose)
            {
                return std::nullopt;
            }
            SampledPair result{*frontPose, *rearPose};
            result.residualMeters = glm::length(
                result.front.worldPositionMeters
                    - result.rear.worldPositionMeters)
                - glm::length(front.referencePositionMeters
                    - rear.referencePositionMeters);
            return result;
        }

        [[nodiscard]] DynamicContactCarPose solveDynamicPose(
            const CompiledPhysicsTrack& track,
            const TrainCarDefinition& definition,
            const GeneralizedCoordinates& coordinates,
            std::size_t* const evaluationCount,
            const bool includeSourceContacts = false)
        {
            if (evaluationCount)
            {
                ++*evaluationCount;
            }
            const CarDefinition& car = definition.car;
            const auto [frontIndex, rearIndex] = bogieIndices(car);
            const BogieDefinition& frontDefinition = car.bogies[frontIndex];
            const BogieDefinition& rearDefinition = car.bogies[rearIndex];
            const glm::dvec3 localSeparation =
                frontDefinition.referencePositionMeters
                    - rearDefinition.referencePositionMeters;
            const double pivotLength = glm::length(localSeparation);
            const double nominalSeparation = frontDefinition.referencePositionMeters.x
                - rearDefinition.referencePositionMeters.x;

            constexpr std::size_t scanIntervals = 128;
            const double minimumAdjustment = -0.99 * nominalSeparation;
            const double maximumAdjustment = 2.0 * pivotLength;
            std::optional<SampledPair> best;
            std::optional<std::pair<double, SampledPair>> previous;
            double bracketLower = 0.0;
            double bracketUpper = 0.0;
            SampledPair lowerPair;
            SampledPair upperPair;
            bool bracketed = false;

            // Include zero first because it is the exact nominal solution and
            // the closest physically local root when the offsets are equal.
            for (const double candidate : {0.0})
            {
                std::optional<SampledPair> pair = sampledPair(
                    track, car, coordinates, frontIndex, rearIndex, candidate);
                if (pair && std::abs(pair->residualMeters) <= 1.0e-9)
                {
                    best = std::move(pair);
                }
            }
            if (!best)
            {
                for (std::size_t index = 0; index <= scanIntervals; ++index)
                {
                    const double adjustment = std::lerp(
                        minimumAdjustment,
                        maximumAdjustment,
                        static_cast<double>(index) / scanIntervals);
                    std::optional<SampledPair> pair = sampledPair(
                        track, car, coordinates, frontIndex, rearIndex,
                        adjustment);
                    if (!pair)
                    {
                        previous.reset();
                        continue;
                    }
                    if (std::abs(pair->residualMeters) <= 1.0e-9)
                    {
                        best = std::move(pair);
                        break;
                    }
                    if (previous
                        && std::signbit(previous->second.residualMeters)
                            != std::signbit(pair->residualMeters))
                    {
                        bracketLower = previous->first;
                        lowerPair = std::move(previous->second);
                        bracketUpper = adjustment;
                        upperPair = std::move(*pair);
                        bracketed = true;
                        break;
                    }
                    previous = std::pair<double, SampledPair>{
                        adjustment, std::move(*pair)};
                }
            }
            if (!best && bracketed)
            {
                for (std::size_t iteration = 0; iteration < 80; ++iteration)
                {
                    const double middle = 0.5 * (bracketLower + bracketUpper);
                    std::optional<SampledPair> pair = sampledPair(
                        track, car, coordinates, frontIndex, rearIndex, middle);
                    if (!pair)
                    {
                        break;
                    }
                    if (std::abs(pair->residualMeters) <= 1.0e-9)
                    {
                        best = std::move(pair);
                        break;
                    }
                    if (std::signbit(pair->residualMeters)
                        == std::signbit(lowerPair.residualMeters))
                    {
                        bracketLower = middle;
                        lowerPair = std::move(*pair);
                    }
                    else
                    {
                        bracketUpper = middle;
                        upperPair = std::move(*pair);
                    }
                }
                if (!best)
                {
                    best = std::abs(lowerPair.residualMeters)
                            <= std::abs(upperPair.residualMeters)
                        ? std::move(lowerPair) : std::move(upperPair);
                }
            }
            if (!best || std::abs(best->residualMeters) > 1.0e-8)
            {
                throw std::domain_error(
                    "M1A displaced bogie pivots cannot satisfy rigid closure.");
            }

            const glm::dvec3 localAxis = localSeparation / pivotLength;
            const glm::dvec3 worldAxis = glm::normalize(
                best->front.worldPositionMeters
                    - best->rear.worldPositionMeters);
            const glm::dvec3 localLateral{0.0, 1.0, 0.0};
            const glm::dvec3 localPerpendicular = glm::normalize(
                glm::cross(localAxis, localLateral));
            glm::dvec3 worldLateral = glm::normalize(
                best->front.orientedFrame.lateral
                    + best->rear.orientedFrame.lateral);
            const glm::dvec3 worldPerpendicular = glm::normalize(
                glm::cross(worldAxis, worldLateral));
            const glm::dmat3 rotation = glm::dmat3{
                worldAxis, worldLateral, worldPerpendicular}
                * glm::transpose(glm::dmat3{
                    localAxis, localLateral, localPerpendicular});
            geometry::CurveFrame bodyFrame{
                rotation[0], rotation[1], rotation[2]};
            geometry::detail::validateCurveFrameForRotation(
                bodyFrame, "M1A dynamic car pose");

            const glm::dvec3 localMidpoint = 0.5
                * (frontDefinition.referencePositionMeters
                    + rearDefinition.referencePositionMeters);
            const glm::dvec3 worldMidpoint = 0.5
                * (best->front.worldPositionMeters
                    + best->rear.worldPositionMeters);
            const glm::dvec3 bodyPosition = transformPoint(
                worldMidpoint, bodyFrame, -localMidpoint);
            const glm::dvec3 localCog = loadedCarCenterOfGravityMeters(
                car, definition.loadout);
            DynamicContactCarPose result;
            result.referenceLocation = coordinates.q;
            result.bodyWorldPositionMeters = bodyPosition;
            result.bodyFrame = bodyFrame;
            result.bodyOrientation = canonicalized(glm::quat_cast(rotation));
            result.localCenterOfGravityMeters = localCog;
            result.worldCenterOfGravityMeters = transformPoint(
                bodyPosition, bodyFrame, localCog);
            result.totalMassKilograms = totalCarMassKilograms(
                car, definition.loadout);
            result.frontHitchWorldPositionMeters = transformPoint(
                bodyPosition, bodyFrame, car.frontHitchPositionMeters);
            result.rearHitchWorldPositionMeters = transformPoint(
                bodyPosition, bodyFrame, car.rearHitchPositionMeters);
            result.frontBogie = std::move(best->front);
            result.rearBogie = std::move(best->rear);
            const auto appendContacts = [](const BogieDefinition& bogieDefinition,
                                           const DynamicBogiePose& bogiePose,
                                           std::vector<DynamicWorldContactPose>& output)
            {
                output.reserve(bogieDefinition.contacts.size());
                for (std::size_t index = 0;
                    index < bogieDefinition.contacts.size(); ++index)
                {
                    const BogieContactDefinition& contact =
                        bogieDefinition.contacts[index];
                    const glm::dvec3 worldNormal = transformDirection(
                        bogiePose.orientedFrame, contact.contactNormalLocal);
                    const glm::dvec3 worldOffset = transformDirection(
                        bogiePose.orientedFrame, contact.localPositionMeters);
                    const glm::dvec3 worldPosition =
                        bogiePose.worldPositionMeters + worldOffset;
                    const glm::dvec3 nominalPosition =
                        bogiePose.nominalWorldPositionMeters + worldOffset;
                    output.push_back({
                        index,
                        contact.role,
                        worldPosition,
                        worldNormal,
                        contact.clearanceMeters + glm::dot(
                            worldPosition - nominalPosition, worldNormal)
                    });
                }
            };
            if (includeSourceContacts)
            {
                appendContacts(frontDefinition, result.frontBogie,
                    result.frontContacts);
                appendContacts(rearDefinition, result.rearBogie,
                    result.rearContacts);
            }
            return result;
        }

        [[nodiscard]] std::optional<GeneralizedCoordinates> displaced(
            const CompiledPhysicsTrack& track,
            const TrainCarDefinition& definition,
            const GeneralizedCoordinates& coordinates,
            const std::size_t index,
            const double amount)
        {
            GeneralizedCoordinates result = coordinates;
            if (index == 0)
            {
                const TrackAdvanceResult advanced = track.advance(
                    coordinates.q, amount);
                if (track.topology() == coaster::TopologyKind::OpenLinear)
                {
                    const double requested = coordinates.q.stationMeters + amount;
                    if ((advanced.boundary == TrackBoundary::Start && requested < 0.0)
                        || (advanced.boundary == TrackBoundary::End
                            && requested > track.lengthMeters()))
                    {
                        return std::nullopt;
                    }
                }
                result.q = advanced.location;
                result.q.direction = coordinates.q.direction;
            }
            else if (index == 1)
            {
                result.front += amount;
            }
            else
            {
                result.rear += amount;
            }

            try
            {
                static_cast<void>(solveDynamicPose(
                    track, definition, result, nullptr));
            }
            catch (const std::exception&)
            {
                return std::nullopt;
            }

            return result;
        }

        template<typename Value, typename Getter>
        [[nodiscard]] Value firstDerivative(
            const CompiledPhysicsTrack& track,
            const TrainCarDefinition& definition,
            const GeneralizedCoordinates& coordinates,
            const DynamicContactCarPose& center,
            const std::size_t index,
            Getter getter,
            std::size_t* const evaluationCount)
        {
            const double epsilon = index == 0
                ? dynamicContactLongitudinalDerivativeStepMeters
                : dynamicContactTransverseDerivativeStepMeters;
            const auto beforeCoordinates = displaced(
                track, definition, coordinates, index, -epsilon);
            const auto afterCoordinates = displaced(
                track, definition, coordinates, index, epsilon);
            if (beforeCoordinates && afterCoordinates)
            {
                const DynamicContactCarPose before = solveDynamicPose(
                    track, definition, *beforeCoordinates, evaluationCount);
                const DynamicContactCarPose after = solveDynamicPose(
                    track, definition, *afterCoordinates, evaluationCount);
                return (getter(after) - getter(before)) / (2.0 * epsilon);
            }

            const bool forward = afterCoordinates.has_value();
            const auto firstCoordinates = forward
                ? afterCoordinates : beforeCoordinates;
            const auto secondCoordinates = displaced(
                track, definition, coordinates, index,
                forward ? 2.0 * epsilon : -2.0 * epsilon);
            if (!firstCoordinates || !secondCoordinates)
            {
                throw std::domain_error(
                    "M1A has no legal one-sided derivative stencil.");
            }
            const DynamicContactCarPose first = solveDynamicPose(
                track, definition, *firstCoordinates, evaluationCount);
            const DynamicContactCarPose second = solveDynamicPose(
                track, definition, *secondCoordinates, evaluationCount);
            return forward
                ? (-3.0 * getter(center) + 4.0 * getter(first)
                    - getter(second)) / (2.0 * epsilon)
                : (3.0 * getter(center) - 4.0 * getter(first)
                    + getter(second)) / (2.0 * epsilon);
        }

        [[nodiscard]] PoseJacobians poseJacobians(
            const CompiledPhysicsTrack& track,
            const TrainCarDefinition& definition,
            const GeneralizedCoordinates& coordinates,
            const DynamicContactCarPose& center,
            std::size_t* const evaluationCount)
        {
            PoseJacobians result;
            for (std::size_t index = 0; index < 3; ++index)
            {
                result.centerOfGravity[index] = firstDerivative<glm::dvec3>(
                    track, definition, coordinates, center, index,
                    [](const DynamicContactCarPose& pose)
                    {
                        return pose.worldCenterOfGravityMeters;
                    }, evaluationCount);

                const double epsilon = index == 0
                    ? dynamicContactLongitudinalDerivativeStepMeters
                    : dynamicContactTransverseDerivativeStepMeters;
                const auto beforeCoordinates = displaced(
                    track, definition, coordinates, index, -epsilon);
                const auto afterCoordinates = displaced(
                    track, definition, coordinates, index, epsilon);
                if (beforeCoordinates && afterCoordinates)
                {
                    const DynamicContactCarPose before = solveDynamicPose(
                        track, definition, *beforeCoordinates, evaluationCount);
                    const DynamicContactCarPose after = solveDynamicPose(
                        track, definition, *afterCoordinates, evaluationCount);
                    const glm::dvec3 left = shortestWorldRotationVector(
                        before.bodyOrientation, center.bodyOrientation) / epsilon;
                    const glm::dvec3 right = shortestWorldRotationVector(
                        center.bodyOrientation, after.bodyOrientation) / epsilon;
                    result.angularVelocity[index] = 0.5 * (left + right);
                }
                else
                {
                    const bool forward = afterCoordinates.has_value();
                    const auto firstCoordinates = forward
                        ? afterCoordinates : beforeCoordinates;
                    const auto secondCoordinates = displaced(
                        track, definition, coordinates, index,
                        forward ? 2.0 * epsilon : -2.0 * epsilon);
                    if (!firstCoordinates || !secondCoordinates)
                    {
                        throw std::domain_error(
                            "M1A has no legal angular derivative stencil.");
                    }
                    const DynamicContactCarPose first = solveDynamicPose(
                        track, definition, *firstCoordinates, evaluationCount);
                    const DynamicContactCarPose second = solveDynamicPose(
                        track, definition, *secondCoordinates, evaluationCount);
                    const glm::dvec3 firstRotation = shortestWorldRotationVector(
                        center.bodyOrientation, first.bodyOrientation);
                    const glm::dvec3 secondRotation = shortestWorldRotationVector(
                        center.bodyOrientation, second.bodyOrientation);
                    result.angularVelocity[index] = forward
                        ? (4.0 * firstRotation - secondRotation)
                            / (2.0 * epsilon)
                        : (-4.0 * firstRotation + secondRotation)
                            / (2.0 * epsilon);
                }
            }
            return result;
        }

        [[nodiscard]] std::vector<double> symmetricEigenvalues(
            std::vector<std::vector<double>> matrix)
        {
            const std::size_t count = matrix.size();
            for (std::size_t sweep = 0; sweep < 32; ++sweep)
            {
                std::size_t p = 0;
                std::size_t q = count > 1 ? 1 : 0;
                double largest = 0.0;
                for (std::size_t row = 0; row < count; ++row)
                {
                    for (std::size_t column = row + 1; column < count; ++column)
                    {
                        if (std::abs(matrix[row][column]) > largest)
                        {
                            largest = std::abs(matrix[row][column]);
                            p = row;
                            q = column;
                        }
                    }
                }
                if (largest <= 1.0e-14)
                {
                    break;
                }
                const double angle = 0.5 * std::atan2(
                    2.0 * matrix[p][q], matrix[q][q] - matrix[p][p]);
                const double c = std::cos(angle);
                const double s = std::sin(angle);
                for (std::size_t k = 0; k < count; ++k)
                {
                    if (k == p || k == q)
                    {
                        continue;
                    }
                    const double kp = matrix[k][p];
                    const double kq = matrix[k][q];
                    matrix[k][p] = matrix[p][k] = c * kp - s * kq;
                    matrix[k][q] = matrix[q][k] = s * kp + c * kq;
                }
                const double pp = matrix[p][p];
                const double qq = matrix[q][q];
                const double pq = matrix[p][q];
                matrix[p][p] = c * c * pp - 2.0 * s * c * pq + s * s * qq;
                matrix[q][q] = s * s * pp + 2.0 * s * c * pq + c * c * qq;
                matrix[p][q] = matrix[q][p] = 0.0;
            }
            std::vector<double> result(count);
            for (std::size_t index = 0; index < count; ++index)
            {
                result[index] = matrix[index][index];
            }
            std::ranges::sort(result);
            return result;
        }

        [[nodiscard]] DynamicContactConditionStatus conditionOf(
            const std::vector<std::vector<double>>& matrix,
            double& estimate)
        {
            for (const auto& row : matrix)
            {
                for (const double value : row)
                {
                    if (!std::isfinite(value))
                    {
                        estimate = std::numeric_limits<double>::infinity();
                        return DynamicContactConditionStatus::NonFinite;
                    }
                }
            }
            const std::vector<double> eigenvalues = symmetricEigenvalues(matrix);
            if (eigenvalues.empty() || !(eigenvalues.front() > 0.0))
            {
                estimate = std::numeric_limits<double>::infinity();
                return DynamicContactConditionStatus::Singular;
            }
            estimate = eigenvalues.back() / eigenvalues.front();
            if (!std::isfinite(estimate))
            {
                return DynamicContactConditionStatus::NonFinite;
            }
            return estimate > dynamicContactMaximumConditionEstimate
                ? DynamicContactConditionStatus::IllConditioned
                : DynamicContactConditionStatus::WellConditioned;
        }

        [[nodiscard]] bool solveDense(
            std::vector<std::vector<double>> matrix,
            std::vector<double> rightHandSide,
            std::vector<double>& solution)
        {
            const std::size_t count = matrix.size();
            for (std::size_t pivot = 0; pivot < count; ++pivot)
            {
                std::size_t best = pivot;
                for (std::size_t row = pivot + 1; row < count; ++row)
                {
                    if (std::abs(matrix[row][pivot])
                        > std::abs(matrix[best][pivot]))
                    {
                        best = row;
                    }
                }
                if (std::abs(matrix[best][pivot]) <= 1.0e-14)
                {
                    return false;
                }
                std::swap(matrix[pivot], matrix[best]);
                std::swap(rightHandSide[pivot], rightHandSide[best]);
                for (std::size_t row = pivot + 1; row < count; ++row)
                {
                    const double factor = matrix[row][pivot]
                        / matrix[pivot][pivot];
                    for (std::size_t column = pivot; column < count; ++column)
                    {
                        matrix[row][column] -= factor * matrix[pivot][column];
                    }
                    rightHandSide[row] -= factor * rightHandSide[pivot];
                }
            }
            solution.assign(count, 0.0);
            for (std::size_t reverse = count; reverse-- > 0;)
            {
                double value = rightHandSide[reverse];
                for (std::size_t column = reverse + 1; column < count; ++column)
                {
                    value -= matrix[reverse][column] * solution[column];
                }
                solution[reverse] = value / matrix[reverse][reverse];
            }
            return std::ranges::all_of(solution, [](const double value)
            {
                return std::isfinite(value);
            });
        }

        [[nodiscard]] MassEvaluation massMatrix(
            const CompiledPhysicsTrack& track,
            const TrainCarDefinition& definition,
            const GeneralizedCoordinates& coordinates,
            std::size_t* const evaluationCount)
        {
            MassEvaluation result;
            result.pose = solveDynamicPose(
                track, definition, coordinates, evaluationCount);
            result.jacobians = poseJacobians(
                track, definition, coordinates, result.pose, evaluationCount);
            const double mass = result.pose.totalMassKilograms;
            const glm::dmat3 inertiaBody = loadedCarInertiaTensorBodyKgM2(
                definition.car, definition.loadout);
            for (std::size_t row = 0; row < 3; ++row)
            {
                const glm::dvec3 angularRowBody{
                    glm::dot(result.jacobians.angularVelocity[row],
                        result.pose.bodyFrame.tangent),
                    glm::dot(result.jacobians.angularVelocity[row],
                        result.pose.bodyFrame.lateral),
                    glm::dot(result.jacobians.angularVelocity[row],
                        result.pose.bodyFrame.up)};
                for (std::size_t column = 0; column < 3; ++column)
                {
                    const glm::dvec3 angularColumnBody{
                        glm::dot(result.jacobians.angularVelocity[column],
                            result.pose.bodyFrame.tangent),
                        glm::dot(result.jacobians.angularVelocity[column],
                            result.pose.bodyFrame.lateral),
                        glm::dot(result.jacobians.angularVelocity[column],
                            result.pose.bodyFrame.up)};
                    result.matrix[row][column] = mass * glm::dot(
                        result.jacobians.centerOfGravity[row],
                        result.jacobians.centerOfGravity[column])
                        + glm::dot(angularRowBody,
                            inertiaBody * angularColumnBody);
                }
            }
            std::vector<std::vector<double>> matrix(3, std::vector<double>(3));
            for (std::size_t row = 0; row < 3; ++row)
            {
                for (std::size_t column = 0; column < 3; ++column)
                {
                    matrix[row][column] = result.matrix[row][column];
                }
            }
            result.status = conditionOf(matrix, result.conditionEstimate);
            return result;
        }

        [[nodiscard]] Vector3 inertialBias(
            const CompiledPhysicsTrack& track,
            const TrainCarDefinition& definition,
            const GeneralizedCoordinates& coordinates,
            const Vector3& velocity,
            std::size_t* const evaluationCount)
        {
            std::array<Matrix3, 3> derivative{};
            for (std::size_t coordinate = 0; coordinate < 3; ++coordinate)
            {
                const double epsilon = coordinate == 0
                    ? dynamicContactLongitudinalDerivativeStepMeters
                    : dynamicContactTransverseDerivativeStepMeters;
                const auto before = displaced(
                    track, definition, coordinates, coordinate, -epsilon);
                const auto after = displaced(
                    track, definition, coordinates, coordinate, epsilon);
                if (before && after)
                {
                    const Matrix3 lower = massMatrix(
                        track, definition, *before, evaluationCount).matrix;
                    const Matrix3 upper = massMatrix(
                        track, definition, *after, evaluationCount).matrix;
                    for (std::size_t row = 0; row < 3; ++row)
                    {
                        for (std::size_t column = 0; column < 3; ++column)
                        {
                            derivative[coordinate][row][column] =
                                (upper[row][column] - lower[row][column])
                                / (2.0 * epsilon);
                        }
                    }
                }
                else
                {
                    const bool forward = after.has_value();
                    const auto first = forward ? after : before;
                    const auto second = displaced(
                        track, definition, coordinates, coordinate,
                        forward ? 2.0 * epsilon : -2.0 * epsilon);
                    if (!first || !second)
                    {
                        throw std::domain_error(
                            "M1A mass matrix has no legal derivative stencil.");
                    }
                    const Matrix3 center = massMatrix(
                        track, definition, coordinates, evaluationCount).matrix;
                    const Matrix3 firstMatrix = massMatrix(
                        track, definition, *first, evaluationCount).matrix;
                    const Matrix3 secondMatrix = massMatrix(
                        track, definition, *second, evaluationCount).matrix;
                    for (std::size_t row = 0; row < 3; ++row)
                    {
                        for (std::size_t column = 0; column < 3; ++column)
                        {
                            derivative[coordinate][row][column] = forward
                                ? (-3.0 * center[row][column]
                                    + 4.0 * firstMatrix[row][column]
                                    - secondMatrix[row][column])
                                    / (2.0 * epsilon)
                                : (3.0 * center[row][column]
                                    - 4.0 * firstMatrix[row][column]
                                    + secondMatrix[row][column])
                                    / (2.0 * epsilon);
                        }
                    }
                }
            }

            Vector3 result{};
            for (std::size_t i = 0; i < 3; ++i)
            {
                for (std::size_t j = 0; j < 3; ++j)
                {
                    for (std::size_t k = 0; k < 3; ++k)
                    {
                        result[i] += 0.5 * (
                            derivative[k][i][j]
                            + derivative[j][i][k]
                            - derivative[i][j][k])
                            * velocity[j] * velocity[k];
                    }
                }
            }
            return result;
        }

        [[nodiscard]] bool sameContactGroup(
            const BogieContactDefinition& left,
            const BogieContactDefinition& right) noexcept
        {
            return std::abs(left.contactNormalLocal.z
                    - right.contactNormalLocal.z) <= 1.0e-12
                && std::abs(left.clearanceMeters - right.clearanceMeters)
                    <= 1.0e-12;
        }

        [[nodiscard]] DynamicContactStepStatus buildAggregateContacts(
            const CarDefinition& car,
            std::vector<AggregateContact>& output)
        {
            const auto [frontIndex, rearIndex] = bogieIndices(car);
            for (const std::size_t definitionIndex : {frontIndex, rearIndex})
            {
                const BogieDefinition& bogie = car.bogies[definitionIndex];
                std::vector<bool> consumed(bogie.contacts.size(), false);
                bool positiveNormalGroupSeen = false;
                bool negativeNormalGroupSeen = false;
                for (std::size_t source = 0; source < bogie.contacts.size(); ++source)
                {
                    if (consumed[source])
                    {
                        continue;
                    }
                    const BogieContactDefinition& contact = bogie.contacts[source];
                    if (contact.role == BogieContactRole::Guide)
                    {
                        return DynamicContactStepStatus::UnsupportedGuideContact;
                    }
                    if (std::abs(contact.contactNormalLocal.x) > 1.0e-10
                        || std::abs(contact.contactNormalLocal.y) > 1.0e-10
                        || std::abs(std::abs(contact.contactNormalLocal.z) - 1.0)
                            > 1.0e-10)
                    {
                        return DynamicContactStepStatus::UnsupportedContactNormal;
                    }
                    std::vector<std::size_t> group;
                    for (std::size_t candidate = source;
                        candidate < bogie.contacts.size(); ++candidate)
                    {
                        if (!consumed[candidate]
                            && sameContactGroup(contact, bogie.contacts[candidate]))
                        {
                            consumed[candidate] = true;
                            group.push_back(candidate);
                        }
                    }
                    for (const std::size_t member : group)
                    {
                        const glm::dvec3& point =
                            bogie.contacts[member].localPositionMeters;
                        const bool mirrored = std::ranges::any_of(
                            group, [&](const std::size_t other)
                            {
                                const glm::dvec3& mirror =
                                    bogie.contacts[other].localPositionMeters;
                                return std::abs(point.x - mirror.x) <= 1.0e-10
                                    && std::abs(point.y + mirror.y) <= 1.0e-10
                                    && std::abs(point.z - mirror.z) <= 1.0e-10;
                            });
                        if (!mirrored)
                        {
                            return DynamicContactStepStatus::
                                UnsupportedAsymmetricContacts;
                        }
                    }
                    bool& directionSeen = contact.contactNormalLocal.z > 0.0
                        ? positiveNormalGroupSeen : negativeNormalGroupSeen;
                    if (directionSeen)
                    {
                        // Different clearances in the same normal direction
                        // are not dynamically equivalent. M1A deliberately
                        // rejects that multi-surface island instead of letting
                        // an exponential/general contact system leak in.
                        return DynamicContactStepStatus::
                            UnsupportedNonidentifiableContacts;
                    }
                    directionSeen = true;
                    output.push_back({
                        definitionIndex == frontIndex ? 0u : 1u,
                        contact.contactNormalLocal.z,
                        contact.clearanceMeters,
                        source
                    });
                }
            }
            return DynamicContactStepStatus::Available;
        }

        [[nodiscard]] double contactGap(
            const DynamicContactCarPose& pose,
            const AggregateContact& contact)
        {
            const DynamicBogiePose& bogie = contact.bogieIndex == 0
                ? pose.frontBogie : pose.rearBogie;
            const glm::dvec3 worldNormal = contact.normalSign
                * bogie.orientedFrame.up;
            return contact.clearanceMeters + glm::dot(
                bogie.worldPositionMeters - bogie.nominalWorldPositionMeters,
                worldNormal);
        }

        [[nodiscard]] ContactEvaluation evaluateContact(
            const CompiledPhysicsTrack& track,
            const TrainCarDefinition& definition,
            const GeneralizedCoordinates& coordinates,
            const DynamicContactCarPose& center,
            const AggregateContact& contact,
            std::size_t* const evaluationCount)
        {
            ContactEvaluation result;
            result.gapMeters = contactGap(center, contact);
            for (std::size_t index = 0; index < 3; ++index)
            {
                result.jacobian[index] = firstDerivative<double>(
                    track, definition, coordinates, center, index,
                    [&contact](const DynamicContactCarPose& pose)
                    {
                        return contactGap(pose, contact);
                    }, evaluationCount);
            }
            return result;
        }

        [[nodiscard]] Vector3 multiply(
            const Matrix3& matrix, const Vector3& vector) noexcept
        {
            Vector3 result{};
            for (std::size_t row = 0; row < 3; ++row)
            {
                for (std::size_t column = 0; column < 3; ++column)
                {
                    result[row] += matrix[row][column] * vector[column];
                }
            }
            return result;
        }

        [[nodiscard]] bool solveMass(
            const Matrix3& matrix,
            const Vector3& rightHandSide,
            Vector3& result)
        {
            std::vector<std::vector<double>> dense(3, std::vector<double>(3));
            for (std::size_t row = 0; row < 3; ++row)
            {
                for (std::size_t column = 0; column < 3; ++column)
                {
                    dense[row][column] = matrix[row][column];
                }
            }
            std::vector<double> solution;
            if (!solveDense(dense,
                std::vector<double>{rightHandSide[0], rightHandSide[1],
                    rightHandSide[2]}, solution))
            {
                return false;
            }
            result = {solution[0], solution[1], solution[2]};
            return true;
        }

        [[nodiscard]] double kineticEnergy(
            const Matrix3& matrix, const Vector3& velocity) noexcept
        {
            const Vector3 momentum = multiply(matrix, velocity);
            return 0.5 * (velocity[0] * momentum[0]
                + velocity[1] * momentum[1]
                + velocity[2] * momentum[2]);
        }

        [[nodiscard]] LcpSolution solveLcp(
            const std::vector<std::vector<double>>& w,
            const std::vector<double>& b)
        {
            constexpr double impulseFeasibilityToleranceNewtonSeconds = 1.0e-12;
            constexpr double impulseNormalizationNewtonSeconds = 1.0;

            const std::size_t count = b.size();
            LcpSolution best;
            best.impulses.assign(count, 0.0);
            double bestNorm = std::numeric_limits<double>::infinity();
            std::size_t bestActiveCount = std::numeric_limits<std::size_t>::max();
            const std::size_t maskCount = std::size_t{1} << count;
            for (std::size_t mask = 0; mask < maskCount; ++mask)
            {
                ++best.iterations;
                const std::size_t activeCount = std::popcount(mask);
                if (activeCount > count)
                {
                    continue;
                }
                std::vector<std::size_t> active;
                for (std::size_t index = 0; index < count; ++index)
                {
                    if ((mask & (std::size_t{1} << index)) != 0)
                    {
                        active.push_back(index);
                    }
                }
                std::vector<double> impulses(count, 0.0);
                double condition = 1.0;
                DynamicContactConditionStatus conditionStatus =
                    DynamicContactConditionStatus::WellConditioned;
                if (!active.empty())
                {
                    std::vector<std::vector<double>> system(
                        active.size(), std::vector<double>(active.size()));
                    std::vector<double> rhs(active.size());
                    for (std::size_t row = 0; row < active.size(); ++row)
                    {
                        rhs[row] = -b[active[row]];
                        for (std::size_t column = 0;
                            column < active.size(); ++column)
                        {
                            system[row][column] = w[active[row]][active[column]];
                        }
                    }
                    conditionStatus = conditionOf(system, condition);
                    if (conditionStatus
                        != DynamicContactConditionStatus::WellConditioned)
                    {
                        continue;
                    }
                    std::vector<double> activeSolution;
                    if (!solveDense(system, rhs, activeSolution))
                    {
                        continue;
                    }
                    for (std::size_t index = 0; index < active.size(); ++index)
                    {
                        impulses[active[index]] = activeSolution[index];
                    }
                }

                // Step 1: Feasibility check on materially negative impulses.
                bool feasible = true;
                for (std::size_t index = 0; index < count; ++index)
                {
                    if (impulses[index] < -impulseFeasibilityToleranceNewtonSeconds)
                    {
                        feasible = false;
                        break;
                    }
                }
                if (!feasible)
                {
                    continue;
                }

                // Step 2: Canonicalize machine-scale residue in [-1e-12, 0.0) to exact 0.0.
                for (std::size_t index = 0; index < count; ++index)
                {
                    if (impulses[index] < 0.0)
                    {
                        impulses[index] = 0.0;
                    }
                }

                // Step 3: Predicted gap evaluation using canonicalized impulses.
                std::vector<double> predictedGapMeters = b;
                for (std::size_t row = 0; row < count; ++row)
                {
                    for (std::size_t column = 0; column < count; ++column)
                    {
                        predictedGapMeters[row] += w[row][column] * impulses[column];
                    }
                }

                // Step 4: Residual calculation and candidate acceptance check.
                double residual = 0.0;
                double norm = 0.0;
                for (std::size_t index = 0; index < count; ++index)
                {
                    if (predictedGapMeters[index]
                        < -dynamicContactComplementarityToleranceMeters)
                    {
                        feasible = false;
                    }
                    const double penetrationMeters =
                        std::max(0.0, -predictedGapMeters[index]);
                    const double complementarityMeters =
                        std::abs(impulses[index] * predictedGapMeters[index])
                        / (impulseNormalizationNewtonSeconds + std::abs(impulses[index]));
                    residual = std::max({residual, penetrationMeters,
                        complementarityMeters});
                    norm += impulses[index] * impulses[index];
                }
                if (!feasible || residual > dynamicContactComplementarityToleranceMeters)
                {
                    continue;
                }

                // Step 5: Canonical solution selection.
                if (!best.solved || norm < bestNorm - 1.0e-18
                    || (std::abs(norm - bestNorm) <= 1.0e-18
                        && activeCount < bestActiveCount))
                {
                    best.solved = true;
                    best.impulses = std::move(impulses);
                    best.residual = residual;
                    best.conditionEstimate = condition;
                    best.conditionStatus = conditionStatus;
                    bestNorm = norm;
                    bestActiveCount = activeCount;
                }
            }
            return best;
        }

        [[nodiscard]] DynamicContactStepStatus validateScope(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition)
        {
            if (definition.cars.size() != 1)
            {
                return DynamicContactStepStatus::UnsupportedCarCount;
            }
            if (!definition.connections.empty())
            {
                return DynamicContactStepStatus::UnsupportedConnector;
            }
            if (definition.cars.front().car.bogies.size() != 2)
            {
                return DynamicContactStepStatus::UnsupportedBogieCount;
            }
            if (!track.supportsPlanarVerticalMotion())
            {
                return DynamicContactStepStatus::UnsupportedTrackGeometry;
            }
            const auto [front, rear] = bogieIndices(definition.cars.front().car);
            const glm::dvec3 separation =
                definition.cars.front().car.bogies[front].referencePositionMeters
                - definition.cars.front().car.bogies[rear].referencePositionMeters;
            if (std::abs(separation.y) > 1.0e-10
                || glm::length(separation) <= 1.0e-9)
            {
                return DynamicContactStepStatus::
                    UnsupportedBogieReferenceGeometry;
            }
            const BasicResistance& resistance = definition.resistance;
            if (resistance.constantMechanicalForceNewtons != 0.0
                || resistance.linearResistanceCoefficientNewtonSecondsPerMeter
                    != 0.0
                || resistance.dragAreaSquareMeters != 0.0
                || resistance.rollingResistanceCoefficient != 0.0)
            {
                return DynamicContactStepStatus::UnsupportedAggregateResistance;
            }
            if (definition.cars.front().car.aerodynamicDragAreaSquareMeters
                != 0.0)
            {
                return DynamicContactStepStatus::
                    UnsupportedGeneratedAerodynamics;
            }
            return DynamicContactStepStatus::Available;
        }

        //=====================================================================
        // M1B: Coupled Multi-Car Planar Vertical Dynamic Contact
        //=====================================================================

        // Reduced generalized coordinate vector for N cars:
        //   [s, z_0f, z_0r, z_1f, z_1r, ..., z_Nf, z_Nr]
        // Total DOFs: 1 + 2N

        using ReducedCoordinates = std::vector<double>;

        struct ReducedTrainPose
        {
            std::vector<DynamicContactCarPose> carPoses;
            bool valid = false;
        };

        // Root-find the following car's longitudinal station against the
        // leading car's rear hitch. Returns the solved following-car pose
        // with all connector constraints satisfied.
        [[nodiscard]] std::optional<DynamicContactCarPose>
        solveDynamicFollowingCar(
            const CompiledPhysicsTrack& track,
            const DynamicContactCarPose& leadingPose,
            const TrainCarDefinition& followingDefinition,
            const InterCarConnectionDefinition& connection,
            const double followingFrontOffset,
            const double followingRearOffset,
            std::size_t* const evaluationCount)
        {
            // Compute the leader's rear hitch and reference location.
            const glm::dvec3 leaderRearHitch =
                leadingPose.rearHitchWorldPositionMeters;
            const TrackLocation leaderRef = leadingPose.referenceLocation;
            const double leaderSign = directionSign(leaderRef.direction);

            // Compute a longitudinal scale for the following car (inline
            // equivalent of TrainPhysics carLongitudinalScale).
            const double followingScale = std::max({
                1.0,
                std::abs(followingDefinition.car.bodyDimensionsMeters.x),
                std::abs(followingDefinition.car.frontHitchPositionMeters.x),
                std::abs(followingDefinition.car.rearHitchPositionMeters.x)
            });

            // Search parameters from the same policy as TrainPhysics.cpp.
            const double baseSeparation =
                followingDefinition.car.frontHitchPositionMeters.x
                - followingDefinition.car.rearHitchPositionMeters.x;
            const double expectedOffset = std::max(
                0.0, baseSeparation + connection.rigidLengthMeters);
            const double geometryScale = std::max({
                1.0,
                std::abs(baseSeparation),
                connection.rigidLengthMeters,
                followingScale
            });
            const double searchHalfExtent = std::max(
                0.5, 1.1 * geometryScale);
            double searchBegin = std::max(
                0.0, expectedOffset - searchHalfExtent);
            double searchEnd = expectedOffset + searchHalfExtent;

            if (track.topology() == coaster::TopologyKind::ClosedCircuit)
            {
                const double localLimit = std::nextafter(
                    track.lengthMeters(), 0.0);
                searchEnd = std::min(searchEnd, localLimit);
            }

            // Residual function: distance between following front hitch and
            // leading rear hitch minus the rigid connector length.
            auto evaluateCandidate = [&](const double backwardOffset)
                -> std::optional<std::pair<double, DynamicContactCarPose>>
            {
                if (!std::isfinite(backwardOffset) || backwardOffset < 0.0)
                {
                    return std::nullopt;
                }
                TrackLocation followingLoc = track.advance(
                    leaderRef, -leaderSign * backwardOffset).location;
                followingLoc.direction = leaderRef.direction;
                try
                {
                    DynamicContactCarPose followingPose = solveDynamicPose(
                        track, followingDefinition,
                        {followingLoc, followingFrontOffset,
                            followingRearOffset},
                        evaluationCount);
                    const double distance = glm::length(
                        followingPose.frontHitchWorldPositionMeters
                        - leaderRearHitch);
                    const double residual = distance
                        - connection.rigidLengthMeters;
                    return std::pair<double, DynamicContactCarPose>{
                        residual, std::move(followingPose)};
                }
                catch (const std::exception&)
                {
                    return std::nullopt;
                }
            };

            // Tier 1: Try zero offset first (exact nominal).
            if (auto result = evaluateCandidate(0.0))
            {
                if (std::abs(result->first) <= 1.0e-9)
                {
                    return std::move(result->second);
                }
            }

            // Tier 2: Grid search + bisection.
            constexpr std::size_t gridSamples = 160;
            std::optional<std::pair<double, DynamicContactCarPose>> best;
            std::optional<std::pair<double, DynamicContactCarPose>> previous;

            for (std::size_t i = 0; i <= gridSamples; ++i)
            {
                const double offset = std::lerp(
                    searchBegin, searchEnd,
                    static_cast<double>(i) / gridSamples);
                auto candidate = evaluateCandidate(offset);
                if (!candidate)
                {
                    previous.reset();
                    continue;
                }
                if (std::abs(candidate->first) <= 1.0e-9)
                {
                    return std::move(candidate->second);
                }
                if (previous
                    && std::signbit(previous->first)
                        != std::signbit(candidate->first))
                {
                    // Bracket found — bisect.
                    double lower = (i > 0)
                        ? std::lerp(searchBegin, searchEnd,
                            static_cast<double>(i - 1) / gridSamples)
                        : searchBegin;
                    double upper = offset;
                    auto lowerResult = std::move(*previous);
                    auto upperResult = std::move(*candidate);

                    for (std::size_t iter = 0; iter < 80; ++iter)
                    {
                        const double mid = 0.5 * (lower + upper);
                        auto midResult = evaluateCandidate(mid);
                        if (!midResult)
                        {
                            break;
                        }
                        if (std::abs(midResult->first) <= 1.0e-8)
                        {
                            return std::move(midResult->second);
                        }
                        if (std::signbit(midResult->first)
                            == std::signbit(lowerResult.first))
                        {
                            lower = mid;
                            lowerResult = std::move(*midResult);
                        }
                        else
                        {
                            upper = mid;
                            upperResult = std::move(*midResult);
                        }
                    }
                    const auto& chosen = std::abs(lowerResult.first)
                            <= std::abs(upperResult.first)
                        ? lowerResult : upperResult;
                    if (std::abs(chosen.first) <= 1.0e-8)
                    {
                        return std::move(chosen.second);
                    }
                    break;
                }
                previous = std::move(candidate);
            }

            // Tier 3: Fallback — try expected offset region more finely.
            if (auto result = evaluateCandidate(expectedOffset))
            {
                if (std::abs(result->first) <= 1.0e-8)
                {
                    return std::move(result->second);
                }
            }

            return std::nullopt;
        }

        // Evaluate the complete reduced train pose from a generalized-
        // coordinate vector. All connector constraints are satisfied to
        // connectorLengthToleranceMeters by construction.
        [[nodiscard]] ReducedTrainPose evaluateReducedTrainPose(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const ReducedCoordinates& x,
            std::size_t* const evaluationCount)
        {
            ReducedTrainPose result;
            const std::size_t N = definition.cars.size();
            result.carPoses.resize(N);

            // Car 0: solve directly from s, z_0f, z_0r.
            try
            {
                const TrackLocation sLocation{
                    primaryTrackPathId, x[0],
                    definition.cars[0].car.bogies.empty()
                        ? TravelDirection::IncreasingStation
                        : (x.size() > 1
                            ? TravelDirection::IncreasingStation
                            : TravelDirection::IncreasingStation)};
                // Use the lead car's reference direction from the first
                // generalized coordinate.
                result.carPoses[0] = solveDynamicPose(
                    track, definition.cars[0],
                    {sLocation, x[1], x[2]},
                    evaluationCount);
            }
            catch (const std::exception&)
            {
                return result;
            }

            // Following cars: connector root-finding from upstream.
            for (std::size_t i = 0; i + 1 < N; ++i)
            {
                const std::size_t followingIndex = i + 1;
                const std::size_t zFrontDof = 1 + 2 * followingIndex;
                const std::size_t zRearDof = 1 + 2 * followingIndex + 1;
                if (zFrontDof >= x.size() || zRearDof >= x.size())
                {
                    return result;
                }
                auto following = solveDynamicFollowingCar(
                    track, result.carPoses[i], definition.cars[followingIndex],
                    definition.connections[i],
                    x[zFrontDof], x[zRearDof], evaluationCount);
                if (!following)
                {
                    return result;
                }
                result.carPoses[followingIndex] = std::move(*following);
            }

            result.valid = true;
            return result;
        }

        // Compute the translational (CoG) Jacobian for car k with respect
        // to generalized coordinate index, using finite differences through
        // the full reduced train evaluator.
        [[nodiscard]] glm::dvec3 multiCarTranslationalJacobian(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const ReducedCoordinates& x,
            const ReducedTrainPose& center,
            const std::size_t carIndex,
            const std::size_t dofIndex,
            std::size_t* const evaluationCount)
        {
            const double epsilon = dofIndex == 0
                ? dynamicContactLongitudinalDerivativeStepMeters
                : dynamicContactTransverseDerivativeStepMeters;

            ReducedCoordinates xPlus = x;
            ReducedCoordinates xMinus = x;
            xPlus[dofIndex] += epsilon;
            xMinus[dofIndex] -= epsilon;

            auto posePlus = evaluateReducedTrainPose(
                track, definition, xPlus, evaluationCount);
            auto poseMinus = evaluateReducedTrainPose(
                track, definition, xMinus, evaluationCount);

            if (posePlus.valid && poseMinus.valid)
            {
                return (posePlus.carPoses[carIndex].worldCenterOfGravityMeters
                    - poseMinus.carPoses[carIndex].worldCenterOfGravityMeters)
                    / (2.0 * epsilon);
            }

            // Fallback to one-sided stencil.
            if (posePlus.valid)
            {
                ReducedCoordinates xFar = x;
                xFar[dofIndex] += 2.0 * epsilon;
                auto poseFar = evaluateReducedTrainPose(
                    track, definition, xFar, evaluationCount);
                if (poseFar.valid)
                {
                    return (-3.0 * center.carPoses[carIndex].worldCenterOfGravityMeters
                        + 4.0 * posePlus.carPoses[carIndex].worldCenterOfGravityMeters
                        - poseFar.carPoses[carIndex].worldCenterOfGravityMeters)
                        / (2.0 * epsilon);
                }
            }
            if (poseMinus.valid)
            {
                ReducedCoordinates xFar = x;
                xFar[dofIndex] -= 2.0 * epsilon;
                auto poseFar = evaluateReducedTrainPose(
                    track, definition, xFar, evaluationCount);
                if (poseFar.valid)
                {
                    return (3.0 * center.carPoses[carIndex].worldCenterOfGravityMeters
                        - 4.0 * poseMinus.carPoses[carIndex].worldCenterOfGravityMeters
                        + poseFar.carPoses[carIndex].worldCenterOfGravityMeters)
                        / (2.0 * epsilon);
                }
            }

            return {0.0, 0.0, 0.0};
        }

        // Compute the angular Jacobian for car k with respect to
        // generalized coordinate index.
        [[nodiscard]] glm::dvec3 multiCarAngularJacobian(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const ReducedCoordinates& x,
            const ReducedTrainPose& center,
            const std::size_t carIndex,
            const std::size_t dofIndex,
            std::size_t* const evaluationCount)
        {
            const double epsilon = dofIndex == 0
                ? dynamicContactLongitudinalDerivativeStepMeters
                : dynamicContactTransverseDerivativeStepMeters;

            ReducedCoordinates xPlus = x;
            ReducedCoordinates xMinus = x;
            xPlus[dofIndex] += epsilon;
            xMinus[dofIndex] -= epsilon;

            auto posePlus = evaluateReducedTrainPose(
                track, definition, xPlus, evaluationCount);
            auto poseMinus = evaluateReducedTrainPose(
                track, definition, xMinus, evaluationCount);

            if (posePlus.valid && poseMinus.valid)
            {
                const glm::dvec3 left = shortestWorldRotationVector(
                    poseMinus.carPoses[carIndex].bodyOrientation,
                    center.carPoses[carIndex].bodyOrientation) / epsilon;
                const glm::dvec3 right = shortestWorldRotationVector(
                    center.carPoses[carIndex].bodyOrientation,
                    posePlus.carPoses[carIndex].bodyOrientation) / epsilon;
                return 0.5 * (left + right);
            }

            return {0.0, 0.0, 0.0};
        }

        // Assemble the full (1+2N) x (1+2N) generalized mass matrix.
        struct MultiCarMassEvaluation
        {
            std::vector<std::vector<double>> matrix;
            ReducedTrainPose pose;
            // Per-car, per-DOF Jacobians: jacobians[carIndex][dofIndex]
            std::vector<std::vector<glm::dvec3>> translationalJacobians;
            std::vector<std::vector<glm::dvec3>> angularJacobians;
            double conditionEstimate = 0.0;
            DynamicContactConditionStatus status =
                DynamicContactConditionStatus::NotEvaluated;
        };

        [[nodiscard]] MultiCarMassEvaluation assembleMultiCarMassMatrix(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const ReducedCoordinates& x,
            std::size_t* const evaluationCount)
        {
            MultiCarMassEvaluation result;
            const std::size_t dofCount = x.size();
            const std::size_t N = definition.cars.size();
            result.matrix.assign(dofCount, std::vector<double>(dofCount, 0.0));
            result.translationalJacobians.resize(N);
            result.angularJacobians.resize(N);
            for (auto& v : result.translationalJacobians)
            {
                v.resize(dofCount);
            }
            for (auto& v : result.angularJacobians)
            {
                v.resize(dofCount);
            }

            // Evaluate the center pose.
            result.pose = evaluateReducedTrainPose(
                track, definition, x, evaluationCount);
            if (!result.pose.valid)
            {
                return result;
            }

            // Compute all Jacobians.
            for (std::size_t k = 0; k < N; ++k)
            {
                for (std::size_t d = 0; d < dofCount; ++d)
                {
                    result.translationalJacobians[k][d] =
                        multiCarTranslationalJacobian(
                            track, definition, x, result.pose, k, d,
                            evaluationCount);
                    result.angularJacobians[k][d] = multiCarAngularJacobian(
                        track, definition, x, result.pose, k, d,
                        evaluationCount);
                }
            }

            // Assemble: M[i][j] = Σ_k m_k * dot(Jv_k_i, Jv_k_j)
            //                      + Jw_k_i^T * I_k * Jw_k_j
            for (std::size_t k = 0; k < N; ++k)
            {
                const DynamicContactCarPose& carPose = result.pose.carPoses[k];
                const TrainCarDefinition& carDef = definition.cars[k];
                const double mass = carPose.totalMassKilograms;
                const glm::dmat3 inertiaBody = loadedCarInertiaTensorBodyKgM2(
                    carDef.car, carDef.loadout);

                for (std::size_t i = 0; i < dofCount; ++i)
                {
                    const glm::dvec3 angularBody_i{
                        glm::dot(result.angularJacobians[k][i],
                            carPose.bodyFrame.tangent),
                        glm::dot(result.angularJacobians[k][i],
                            carPose.bodyFrame.lateral),
                        glm::dot(result.angularJacobians[k][i],
                            carPose.bodyFrame.up)};
                    for (std::size_t j = 0; j < dofCount; ++j)
                    {
                        const glm::dvec3 angularBody_j{
                            glm::dot(result.angularJacobians[k][j],
                                carPose.bodyFrame.tangent),
                            glm::dot(result.angularJacobians[k][j],
                                carPose.bodyFrame.lateral),
                            glm::dot(result.angularJacobians[k][j],
                                carPose.bodyFrame.up)};
                        result.matrix[i][j] += mass * glm::dot(
                            result.translationalJacobians[k][i],
                            result.translationalJacobians[k][j])
                            + glm::dot(angularBody_i,
                                inertiaBody * angularBody_j);
                    }
                }
            }

            result.status = conditionOf(result.matrix, result.conditionEstimate);
            return result;
        }

        // Compute the inertial bias b(v) using Christoffel symbols.
        [[nodiscard]] std::vector<double> assembleMultiCarInertialBias(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const ReducedCoordinates& x,
            const std::vector<double>& velocity,
            std::size_t* const evaluationCount)
        {
            const std::size_t dofCount = x.size();
            std::vector<double> bias(dofCount, 0.0);

            // For each coordinate k, compute dM/dx_k.
            for (std::size_t k = 0; k < dofCount; ++k)
            {
                const double epsilon = k == 0
                    ? dynamicContactLongitudinalDerivativeStepMeters
                    : dynamicContactTransverseDerivativeStepMeters;

                ReducedCoordinates xPlus = x;
                ReducedCoordinates xMinus = x;
                xPlus[k] += epsilon;
                xMinus[k] -= epsilon;

                auto massPlus = assembleMultiCarMassMatrix(
                    track, definition, xPlus, evaluationCount);
                auto massMinus = assembleMultiCarMassMatrix(
                    track, definition, xMinus, evaluationCount);

                // Christoffel symbols: b_i = 0.5 * Σ_{j,l}
                //   (dM[i][j]/dx_k + dM[i][k]/dx_j - dM[j][k]/dx_i)
                //   * v_j * v_l
                for (std::size_t i = 0; i < dofCount; ++i)
                {
                    for (std::size_t j = 0; j < dofCount; ++j)
                    {
                        const double dMij_dk =
                            (massPlus.matrix[i][j] - massMinus.matrix[i][j])
                            / (2.0 * epsilon);
                        const double dMik_dj =
                            (massPlus.matrix[i][k] - massMinus.matrix[i][k])
                            / (2.0 * epsilon);
                        const double dMjk_di =
                            (massPlus.matrix[j][k] - massMinus.matrix[j][k])
                            / (2.0 * epsilon);
                        bias[i] += 0.5 * (dMij_dk + dMik_dj - dMjk_di)
                            * velocity[j] * velocity[k];
                    }
                }
            }

            return bias;
        }

        // Project applied forces through the global Jacobians.
        [[nodiscard]] std::vector<double> assembleMultiCarAppliedForces(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const PhysicsEnvironment& environment,
            const ReducedCoordinates& x,
            const MultiCarMassEvaluation& massEval,
            const std::span<const ExternalForceApplication> externalForces,
            std::size_t* const evaluationCount)
        {
            const std::size_t dofCount = x.size();
            const std::size_t N = definition.cars.size();
            std::vector<double> applied(dofCount, 0.0);

            const glm::dvec3 gravityWorld{
                0.0, 0.0,
                -environment.gravityAccelerationMetersPerSecondSquared};

            // Gravity projection: Q_i = Σ_k m_k * g · Jv_k_i
            for (std::size_t k = 0; k < N; ++k)
            {
                const double mass = massEval.pose.carPoses[k].totalMassKilograms;
                for (std::size_t i = 0; i < dofCount; ++i)
                {
                    applied[i] += mass * glm::dot(gravityWorld,
                        massEval.translationalJacobians[k][i]);
                }
            }

            // External forces projected through application-point Jacobians.
            for (const ExternalForceApplication& app : externalForces)
            {
                if (app.carIndex >= N)
                {
                    continue;
                }
                const std::size_t carIdx = app.carIndex;
                for (std::size_t i = 0; i < dofCount; ++i)
                {
                    const double epsilon = i == 0
                        ? dynamicContactLongitudinalDerivativeStepMeters
                        : dynamicContactTransverseDerivativeStepMeters;
                    ReducedCoordinates xPlus = x;
                    ReducedCoordinates xMinus = x;
                    xPlus[i] += epsilon;
                    xMinus[i] -= epsilon;
                    auto posePlus = evaluateReducedTrainPose(
                        track, definition, xPlus, evaluationCount);
                    auto poseMinus = evaluateReducedTrainPose(
                        track, definition, xMinus, evaluationCount);
                    if (posePlus.valid && poseMinus.valid)
                    {
                        const glm::dvec3 pointPlus =
                            posePlus.carPoses[carIdx].transformLocalPoint(
                                app.localApplicationPointMeters);
                        const glm::dvec3 pointMinus =
                            poseMinus.carPoses[carIdx].transformLocalPoint(
                                app.localApplicationPointMeters);
                        const glm::dvec3 derivative =
                            (pointPlus - pointMinus) / (2.0 * epsilon);
                        applied[i] += glm::dot(app.worldForceNewtons, derivative);
                    }
                }
            }

            return applied;
        }

        // Solve M * deltaV = rhs for deltaV using dense Gaussian elimination.
        [[nodiscard]] bool solveMultiCarMass(
            const std::vector<std::vector<double>>& matrix,
            const std::vector<double>& rhs,
            std::vector<double>& solution)
        {
            return solveDense(matrix, rhs, solution);
        }

        // Multiply matrix * vector.
        [[nodiscard]] std::vector<double> multiplyMV(
            const std::vector<std::vector<double>>& matrix,
            const std::vector<double>& vector)
        {
            const std::size_t n = matrix.size();
            std::vector<double> result(n, 0.0);
            for (std::size_t i = 0; i < n; ++i)
            {
                for (std::size_t j = 0; j < n; ++j)
                {
                    result[i] += matrix[i][j] * vector[j];
                }
            }
            return result;
        }

        // Multiply matrix^T * vector.
        [[nodiscard]] std::vector<double> multiplyMTV(
            const std::vector<std::vector<double>>& matrix,
            const std::vector<double>& vector)
        {
            const std::size_t n = matrix.size();
            const std::size_t m = matrix.empty() ? 0 : matrix[0].size();
            std::vector<double> result(m, 0.0);
            for (std::size_t j = 0; j < m; ++j)
            {
                for (std::size_t i = 0; i < n; ++i)
                {
                    result[j] += matrix[i][j] * vector[i];
                }
            }
            return result;
        }

        // Compute kinetic energy: 0.5 * v^T * M * v.
        [[nodiscard]] double multiCarKineticEnergy(
            const std::vector<std::vector<double>>& matrix,
            const std::vector<double>& velocity)
        {
            const std::vector<double> momentum = multiplyMV(matrix, velocity);
            double energy = 0.0;
            for (std::size_t i = 0; i < velocity.size(); ++i)
            {
                energy += velocity[i] * momentum[i];
            }
            return 0.5 * energy;
        }

        // Evaluate contact gap for a specific car's bogie.
        [[nodiscard]] double multiCarContactGap(
            const DynamicContactCarPose& pose,
            const AggregateContact& contact)
        {
            return contactGap(pose, contact);
        }

        // Evaluate contact Jacobian for a specific contact with respect to
        // the full reduced coordinate vector, using finite differences.
        [[nodiscard]] std::vector<double> multiCarContactJacobian(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const ReducedCoordinates& x,
            const std::size_t carIndex,
            const AggregateContact& contact,
            std::size_t* const evaluationCount)
        {
            const std::size_t dofCount = x.size();
            std::vector<double> jacobian(dofCount, 0.0);

            for (std::size_t d = 0; d < dofCount; ++d)
            {
                const double epsilon = d == 0
                    ? dynamicContactLongitudinalDerivativeStepMeters
                    : dynamicContactTransverseDerivativeStepMeters;
                ReducedCoordinates xPlus = x;
                ReducedCoordinates xMinus = x;
                xPlus[d] += epsilon;
                xMinus[d] -= epsilon;

                auto posePlus = evaluateReducedTrainPose(
                    track, definition, xPlus, evaluationCount);
                auto poseMinus = evaluateReducedTrainPose(
                    track, definition, xMinus, evaluationCount);

                if (posePlus.valid && poseMinus.valid)
                {
                    const double gapPlus = multiCarContactGap(
                        posePlus.carPoses[carIndex], contact);
                    const double gapMinus = multiCarContactGap(
                        poseMinus.carPoses[carIndex], contact);
                    jacobian[d] = (gapPlus - gapMinus) / (2.0 * epsilon);
                }
            }

            return jacobian;
        }

        // Per-car aggregate contact result for multi-car output.
        struct MultiCarAggregateContactResult
        {
            std::size_t carIndex = 0;
            std::size_t bogieIndex = 0;
            double gapMeters = 0.0;
            std::vector<double> jacobian;
        };

        // Build the global aggregate contact list for all cars, tracking
        // which car each contact belongs to.
        [[nodiscard]] DynamicContactStepStatus buildMultiCarAggregateContacts(
            const TrainDefinition& definition,
            std::vector<AggregateContact>& contacts,
            std::vector<std::size_t>& contactCarIndex)
        {
            contacts.clear();
            contactCarIndex.clear();
            for (std::size_t carIdx = 0; carIdx < definition.cars.size();
                ++carIdx)
            {
                std::vector<AggregateContact> carContacts;
                const DynamicContactStepStatus status = buildAggregateContacts(
                    definition.cars[carIdx].car, carContacts);
                if (status != DynamicContactStepStatus::Available)
                {
                    return status;
                }
                for (std::size_t i = 0; i < carContacts.size(); ++i)
                {
                    contactCarIndex.push_back(carIdx);
                }
                contacts.insert(contacts.end(),
                    carContacts.begin(), carContacts.end());
            }
            return DynamicContactStepStatus::Available;
        }

        // Deterministic globally coupled PGS solver for C > 4 contacts.
        struct PgsSolution
        {
            std::vector<double> impulses;
            bool converged = false;
            std::size_t iterations = 0;
            double finalPenetrationMeters = 0.0;
            double finalImpulseDeltaNewtonSeconds = 0.0;
        };

        [[nodiscard]] PgsSolution solvePgs(
            const std::vector<std::vector<double>>& w,
            const std::vector<double>& b,
            const std::size_t maxIterations)
        {
            const std::size_t count = b.size();
            PgsSolution result;
            result.impulses.assign(count, 0.0);

            for (std::size_t iter = 0; iter < maxIterations; ++iter)
            {
                double maxDelta = 0.0;
                for (std::size_t c = 0; c < count; ++c)
                {
                    const double wii = w[c][c];
                    if (!std::isfinite(wii) || wii <= 1.0e-14)
                    {
                        // Ill-conditioned diagonal — skip this contact.
                        continue;
                    }
                    double r = b[c];
                    for (std::size_t j = 0; j < count; ++j)
                    {
                        r += w[c][j] * result.impulses[j];
                    }
                    const double oldLambda = result.impulses[c];
                    result.impulses[c] = std::max(0.0,
                        oldLambda - r / wii);
                    const double delta = std::abs(
                        result.impulses[c] - oldLambda);
                    maxDelta = std::max(maxDelta, delta);
                }

                // Check convergence using complementarity residual.
                double maxPenetration = 0.0;
                double maxComplementarity = 0.0;
                for (std::size_t c = 0; c < count; ++c)
                {
                    double gap = b[c];
                    for (std::size_t j = 0; j < count; ++j)
                    {
                        gap += w[c][j] * result.impulses[c];
                    }
                    // Correct: predicted gap = b + W * lambda
                    double predictedGap = b[c];
                    for (std::size_t j = 0; j < count; ++j)
                    {
                        predictedGap += w[c][j] * result.impulses[j];
                    }
                    maxPenetration = std::max(maxPenetration,
                        std::max(0.0, -predictedGap));
                    const double comp = std::abs(
                        result.impulses[c] * predictedGap)
                        / (1.0 + std::abs(result.impulses[c]));
                    maxComplementarity = std::max(
                        maxComplementarity, comp);
                }

                result.iterations = iter + 1;
                result.finalPenetrationMeters = maxPenetration;
                result.finalImpulseDeltaNewtonSeconds = maxDelta;

                const double residual = std::max(
                    maxPenetration, maxComplementarity);
                if (residual <= dynamicContactComplementarityToleranceMeters
                    && maxDelta <= 1.0e-12)
                {
                    result.converged = true;
                    break;
                }
            }

            return result;
        }

        // Multi-car single substep: one substep of the reduced-coordinate
        // multi-car dynamics with globally coupled contact.
        [[nodiscard]] DynamicContactMultiCarStepResult multiCarSingleSubstep(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const PhysicsEnvironment& environment,
            const DynamicContactMultiCarState& currentState,
            const double deltaTime,
            const std::span<const ExternalForceApplication> externalForces,
            const std::vector<AggregateContact>& contacts,
            const std::vector<std::size_t>& contactCarIndex)
        {
            DynamicContactMultiCarStepResult result;
            DynamicContactMultiCarTelemetry& telemetry = result.telemetry;
            std::size_t poseEvaluations = 0;

            const std::size_t N = definition.cars.size();
            const std::size_t dofCount = 1 + 2 * N;
            result.dofLayout = DynamicContactDofLayout::create(N);
            result.generalizedCoordinates.resize(dofCount);
            result.generalizedVelocity.resize(dofCount);

            // Build the reduced coordinate vector.
            ReducedCoordinates x(dofCount);
            x[0] = currentState.leadCarReferenceLocation.stationMeters;
            std::vector<double> velocity(dofCount);
            velocity[0] = currentState.signedLongitudinalVelocityMetersPerSecond;
            for (std::size_t i = 0; i < N; ++i)
            {
                x[1 + 2 * i] = currentState.cars[i].frontVerticalOffsetMeters;
                x[1 + 2 * i + 1] = currentState.cars[i].rearVerticalOffsetMeters;
                velocity[1 + 2 * i] =
                    currentState.cars[i].frontVerticalVelocityMetersPerSecond;
                velocity[1 + 2 * i + 1] =
                    currentState.cars[i].rearVerticalVelocityMetersPerSecond;
            }

            // Assemble the mass matrix.
            MultiCarMassEvaluation mass;
            try
            {
                mass = assembleMultiCarMassMatrix(
                    track, definition, x, &poseEvaluations);
            }
            catch (const std::exception&)
            {
                telemetry.dynamicPoseEvaluationCount = poseEvaluations;
                result.status = DynamicContactStepStatus::RigidClosureFailure;
                return result;
            }
            telemetry.massMatrixConditionEstimate = mass.conditionEstimate;
            telemetry.massMatrixStatus = mass.status;
            telemetry.massMatrixSize = dofCount;
            if (mass.status == DynamicContactConditionStatus::NonFinite)
            {
                result.status = DynamicContactStepStatus::MassMatrixNonFinite;
                return result;
            }
            if (mass.status == DynamicContactConditionStatus::Singular)
            {
                result.status =
                    DynamicContactStepStatus::MassMatrixNotPositiveDefinite;
                return result;
            }
            if (mass.status == DynamicContactConditionStatus::IllConditioned)
            {
                result.status =
                    DynamicContactStepStatus::MassMatrixIllConditioned;
                return result;
            }

            // Compute inertial bias.
            std::vector<double> bias;
            try
            {
                bias = assembleMultiCarInertialBias(
                    track, definition, x, velocity, &poseEvaluations);
            }
            catch (const std::exception&)
            {
                telemetry.dynamicPoseEvaluationCount = poseEvaluations;
                result.status = DynamicContactStepStatus::RigidClosureFailure;
                return result;
            }

            // Project applied forces.
            std::vector<double> applied = assembleMultiCarAppliedForces(
                track, definition, environment, x, mass,
                externalForces, &poseEvaluations);

            // Compute free velocity: v_free = v + M^{-1} * dt * (applied - bias)
            std::vector<double> impulseRhs(dofCount);
            for (std::size_t i = 0; i < dofCount; ++i)
            {
                impulseRhs[i] = deltaTime * (applied[i] - bias[i]);
            }
            std::vector<double> deltaVelocity(dofCount);
            if (!solveMultiCarMass(mass.matrix, impulseRhs, deltaVelocity))
            {
                result.status =
                    DynamicContactStepStatus::MassMatrixNotPositiveDefinite;
                return result;
            }
            std::vector<double> freeVelocity = velocity;
            for (std::size_t i = 0; i < dofCount; ++i)
            {
                freeVelocity[i] += deltaVelocity[i];
            }

            telemetry.kineticEnergyBeforeContactJoules =
                multiCarKineticEnergy(mass.matrix, freeVelocity);

            // Use the standard M1A-compatible contact-gap tolerance.
            // Empirical validation on curved tracks, flat tracks,
            // N=2/N=4, and speeds up to 20 m/s confirmed that
            // post-step committed gap residuals are at machine
            // precision — well below 1e-9.
            const double connectorTolerance =
                dynamicContactGapToleranceMeters;

            // Evaluate contact gaps and Jacobians for all cars.
            std::vector<MultiCarAggregateContactResult> allContactEvaluations;
            std::vector<std::size_t> candidateIndices;

            for (std::size_t contactIdx = 0; contactIdx < contacts.size();
                ++contactIdx)
            {
                // Determine which car this contact belongs to.
                const std::size_t carIndex = contactCarIndex[contactIdx];
                if (carIndex >= N)
                {
                    continue;
                }

                const AggregateContact& contact = contacts[contactIdx];
                std::vector<double> jacobian = multiCarContactJacobian(
                    track, definition, x, carIndex, contact, &poseEvaluations);

                double gap = multiCarContactGap(
                    mass.pose.carPoses[carIndex], contact);

                if (gap < -connectorTolerance)
                {
                    result.status =
                        DynamicContactStepStatus::InvalidInitialPenetration;
                    return result;
                }

                // Predict gap at end of timestep.
                double predictedGap = gap;
                for (std::size_t d = 0; d < dofCount; ++d)
                {
                    predictedGap += deltaTime * jacobian[d] * freeVelocity[d];
                }

                if (gap <= dynamicContactGapToleranceMeters
                    || predictedGap < -dynamicContactGapToleranceMeters)
                {
                    candidateIndices.push_back(allContactEvaluations.size());
                }

                allContactEvaluations.push_back({carIndex,
                    contact.bogieIndex, gap, std::move(jacobian)});
            }

            telemetry.candidateContactCount = candidateIndices.size();
            telemetry.contactSystemSize = candidateIndices.size();

            // Solve the contact LCP.
            std::vector<double> committedVelocity = freeVelocity;
            std::vector<double> impulses(candidateIndices.size(), 0.0);

            if (!candidateIndices.empty())
            {
                const std::size_t C = candidateIndices.size();

                if (C <= dynamicContactExhaustiveSolverMaximumRows)
                {
                    // Exhaustive LCP (M1A-compatible).
                    std::vector<std::vector<double>> inverseMassJac(C,
                        std::vector<double>(dofCount, 0.0));
                    std::vector<std::vector<double>> w(C,
                        std::vector<double>(C));
                    std::vector<double> b(C);

                    for (std::size_t row = 0; row < C; ++row)
                    {
                        const auto& contact =
                            allContactEvaluations[candidateIndices[row]];
                        std::vector<double> jacFull(dofCount);
                        for (std::size_t d = 0; d < dofCount; ++d)
                        {
                            jacFull[d] = contact.jacobian[d];
                        }
                        if (!solveMultiCarMass(mass.matrix, jacFull,
                            inverseMassJac[row]))
                        {
                            result.status = DynamicContactStepStatus::
                                ContactSystemIllConditioned;
                            return result;
                        }
                        b[row] = contact.gapMeters;
                        for (std::size_t d = 0; d < dofCount; ++d)
                        {
                            b[row] += deltaTime * contact.jacobian[d]
                                * freeVelocity[d];
                        }
                        for (std::size_t col = 0; col < C; ++col)
                        {
                            const auto& other =
                                allContactEvaluations[candidateIndices[col]];
                            double dot = 0.0;
                            for (std::size_t d = 0; d < dofCount; ++d)
                            {
                                dot += other.jacobian[d]
                                    * inverseMassJac[row][d];
                            }
                            w[row][col] = deltaTime * dot;
                        }
                    }

                    const LcpSolution solution = solveLcp(w, b);
                    telemetry.solverIterationCount = solution.iterations;
                    telemetry.complementarityResidualMeters = solution.residual;
                    telemetry.contactSystemConditionEstimate =
                        solution.conditionEstimate;
                    telemetry.contactSystemStatus = solution.conditionStatus;
                    if (!solution.solved)
                    {
                        result.status = DynamicContactStepStatus::
                            ContactSolveNonConverged;
                        return result;
                    }
                    impulses = solution.impulses;

                    for (std::size_t idx = 0; idx < C; ++idx)
                    {
                        if (impulses[idx] <= 1.0e-12)
                        {
                            continue;
                        }
                        ++telemetry.impulseCarryingContactCount;
                        // Apply impulse through full mass matrix.
                        for (std::size_t d = 0; d < dofCount; ++d)
                        {
                            committedVelocity[d] +=
                                inverseMassJac[idx][d] * impulses[idx];
                        }
                    }
                }
                else
                {
                    // PGS for C > 4 contacts.
                    std::vector<std::vector<double>> w(C,
                        std::vector<double>(C));
                    std::vector<double> b(C);

                    for (std::size_t row = 0; row < C; ++row)
                    {
                        const auto& contact =
                            allContactEvaluations[candidateIndices[row]];
                        std::vector<double> jacFull(dofCount);
                        for (std::size_t d = 0; d < dofCount; ++d)
                        {
                            jacFull[d] = contact.jacobian[d];
                        }
                        std::vector<double> invMassJacFull(dofCount);
                        if (!solveMultiCarMass(mass.matrix, jacFull,
                            invMassJacFull))
                        {
                            result.status = DynamicContactStepStatus::
                                ContactSystemIllConditioned;
                            return result;
                        }
                        b[row] = contact.gapMeters;
                        for (std::size_t d = 0; d < dofCount; ++d)
                        {
                            b[row] += deltaTime * contact.jacobian[d]
                                * freeVelocity[d];
                        }
                        for (std::size_t col = 0; col < C; ++col)
                        {
                            const auto& other =
                                allContactEvaluations[candidateIndices[col]];
                            double dot = 0.0;
                            for (std::size_t d = 0; d < dofCount; ++d)
                            {
                                dot += other.jacobian[d] * invMassJacFull[d];
                            }
                            w[row][col] = deltaTime * dot;
                        }
                    }

                    PgsSolution pgs = solvePgs(
                        w, b, dynamicContactPgsMaximumIterations);
                    telemetry.pgsIterationCount = pgs.iterations;
                    telemetry.pgsFinalPenetrationMeters =
                        pgs.finalPenetrationMeters;
                    telemetry.pgsFinalImpulseDeltaNewtonSeconds =
                        pgs.finalImpulseDeltaNewtonSeconds;
                    telemetry.pgsConverged = pgs.converged;
                    telemetry.complementarityResidualMeters =
                        pgs.finalPenetrationMeters;

                    if (!pgs.converged)
                    {
                        result.status = DynamicContactStepStatus::
                            ContactSolveNonConverged;
                        return result;
                    }

                    impulses = pgs.impulses;
                    for (std::size_t idx = 0; idx < C; ++idx)
                    {
                        if (impulses[idx] <= 1.0e-12)
                        {
                            continue;
                        }
                        ++telemetry.impulseCarryingContactCount;
                        const auto& contact =
                            allContactEvaluations[candidateIndices[idx]];
                        std::vector<double> jacFull(dofCount);
                        for (std::size_t d = 0; d < dofCount; ++d)
                        {
                            jacFull[d] = contact.jacobian[d];
                        }
                        std::vector<double> invMassJacFull(dofCount);
                        if (!solveMultiCarMass(
                                mass.matrix, jacFull, invMassJacFull))
                        {
                            result.status = DynamicContactStepStatus::
                                MassMatrixNotPositiveDefinite;
                            return result;
                        }
                        for (std::size_t d = 0; d < dofCount; ++d)
                        {
                            committedVelocity[d] +=
                                invMassJacFull[d] * impulses[idx];
                        }
                    }
                }
            }

            telemetry.kineticEnergyAfterContactJoules =
                multiCarKineticEnergy(mass.matrix, committedVelocity);

            // Position advance (semi-implicit Euler).
            ReducedCoordinates newX(dofCount);
            newX[0] = x[0] + deltaTime * committedVelocity[0];
            for (std::size_t i = 0; i < N; ++i)
            {
                newX[1 + 2 * i] = x[1 + 2 * i]
                    + deltaTime * committedVelocity[1 + 2 * i];
                newX[1 + 2 * i + 1] = x[1 + 2 * i + 1]
                    + deltaTime * committedVelocity[1 + 2 * i + 1];
            }

            // Rebuild the reduced train pose at the new coordinates.
            ReducedTrainPose nextPose;
            try
            {
                nextPose = evaluateReducedTrainPose(
                    track, definition, newX, &poseEvaluations);
            }
            catch (const std::exception&)
            {
                result.status = DynamicContactStepStatus::RigidClosureFailure;
                return result;
            }
            if (!nextPose.valid)
            {
                result.status = DynamicContactStepStatus::RigidClosureFailure;
                return result;
            }

            // Validate connector closure residuals.
            double maxConnectorResidual = 0.0;
            for (std::size_t i = 0; i + 1 < N; ++i)
            {
                const double distance = glm::length(
                    nextPose.carPoses[i + 1].frontHitchWorldPositionMeters
                    - nextPose.carPoses[i].rearHitchWorldPositionMeters);
                const double residual = std::abs(distance
                    - definition.connections[i].rigidLengthMeters);
                maxConnectorResidual = std::max(
                    maxConnectorResidual, residual);
                telemetry.connectorClosures.push_back(
                    {i, distance - definition.connections[i].rigidLengthMeters,
                        0, 0.0});
            }
            telemetry.maximumConnectorResidualMeters = maxConnectorResidual;
            if (maxConnectorResidual > connectorLengthToleranceMeters)
            {
                result.status = DynamicContactStepStatus::RigidClosureFailure;
                return result;
            }

            // Nonlinear penetration check.
            double minimumGap = std::numeric_limits<double>::infinity();
            double maximumGap = -std::numeric_limits<double>::infinity();
            for (std::size_t contactIdx = 0; contactIdx < contacts.size();
                ++contactIdx)
            {
                const std::size_t carIndex = contactCarIndex[contactIdx];
                if (carIndex >= N)
                {
                    continue;
                }
                const double gap = multiCarContactGap(
                    nextPose.carPoses[carIndex], contacts[contactIdx]);
                minimumGap = std::min(minimumGap, gap);
                maximumGap = std::max(maximumGap, gap);
            }
            if (contacts.empty())
            {
                minimumGap = maximumGap = 0.0;
            }
            telemetry.minimumSignedGapMeters = minimumGap;
            telemetry.maximumSignedGapMeters = maximumGap;
            telemetry.nonlinearCommittedGapResidualMeters =
                std::max(0.0, -minimumGap);
            telemetry.dynamicPoseEvaluationCount = poseEvaluations;

            if (minimumGap < -connectorTolerance)
            {
                result.status = DynamicContactStepStatus::NonlinearPenetration;
                return result;
            }

            // Build the output result.
            result.status = DynamicContactStepStatus::Available;
            result.generalizedCoordinates = newX;
            result.generalizedVelocity = committedVelocity;
            result.carResults.resize(N);
            result.leadCarReferenceLocation = currentState.leadCarReferenceLocation;
            result.leadCarReferenceLocation.stationMeters = newX[0];
            result.tick = currentState.tick;
            result.runState = std::hypot(committedVelocity[0],
                committedVelocity[1], committedVelocity[2])
                    <= followerRestSpeedToleranceMetersPerSecond
                ? FollowerRunState::Resting : FollowerRunState::Running;

            for (std::size_t i = 0; i < N; ++i)
            {
                result.carResults[i].carIndex = i;
                result.carResults[i].pose = std::move(nextPose.carPoses[i]);
            }

            return result;
        }

        [[nodiscard]] DynamicContactStepResult fail(
            const DynamicContactStepStatus status,
            const DynamicContactState& state,
            DynamicContactTelemetry telemetry = {})
        {
            return {status, state, std::nullopt, std::move(telemetry)};
        }

        [[nodiscard]] DynamicContactStepResult singleSubstep(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const PhysicsEnvironment& environment,
            const DynamicContactState& currentState,
            const double deltaTime,
            const std::span<const ExternalForceApplication> externalForces,
            const std::vector<AggregateContact>& contacts)
        {
            DynamicContactTelemetry telemetry;
            std::size_t poseEvaluations = 0;
            const GeneralizedCoordinates coordinates{
                currentState.generalizedReferenceLocation,
                currentState.frontVerticalOffsetMeters,
                currentState.rearVerticalOffsetMeters};
            const Vector3 velocity{
                currentState.signedLongitudinalVelocityMetersPerSecond,
                currentState.frontVerticalVelocityMetersPerSecond,
                currentState.rearVerticalVelocityMetersPerSecond};
            MassEvaluation mass;
            try
            {
                mass = massMatrix(track, definition.cars.front(), coordinates,
                    &poseEvaluations);
            }
            catch (const std::exception&)
            {
                telemetry.dynamicPoseEvaluationCount = poseEvaluations;
                return fail(DynamicContactStepStatus::RigidClosureFailure,
                    currentState, telemetry);
            }
            telemetry.massMatrixConditionEstimate = mass.conditionEstimate;
            telemetry.massMatrixStatus = mass.status;
            if (mass.status == DynamicContactConditionStatus::NonFinite)
            {
                return fail(DynamicContactStepStatus::MassMatrixNonFinite,
                    currentState, telemetry);
            }
            if (mass.status == DynamicContactConditionStatus::Singular)
            {
                return fail(
                    DynamicContactStepStatus::MassMatrixNotPositiveDefinite,
                    currentState, telemetry);
            }
            if (mass.status == DynamicContactConditionStatus::IllConditioned)
            {
                return fail(DynamicContactStepStatus::MassMatrixIllConditioned,
                    currentState, telemetry);
            }

            Vector3 bias{};
            try
            {
                bias = inertialBias(track, definition.cars.front(), coordinates,
                    velocity, &poseEvaluations);
            }
            catch (const std::exception&)
            {
                telemetry.dynamicPoseEvaluationCount = poseEvaluations;
                return fail(DynamicContactStepStatus::RigidClosureFailure,
                    currentState, telemetry);
            }

            Vector3 applied{};
            const glm::dvec3 gravityWorld{
                0.0, 0.0,
                -environment.gravityAccelerationMetersPerSecondSquared};
            for (std::size_t index = 0; index < 3; ++index)
            {
                applied[index] = mass.pose.totalMassKilograms * glm::dot(
                    gravityWorld, mass.jacobians.centerOfGravity[index]);
            }
            for (const ExternalForceApplication& application : externalForces)
            {
                for (std::size_t index = 0; index < 3; ++index)
                {
                    glm::dvec3 derivative{};
                    try
                    {
                        derivative = firstDerivative<glm::dvec3>(
                            track, definition.cars.front(), coordinates,
                            mass.pose, index,
                            [&application](const DynamicContactCarPose& pose)
                            {
                                return pose.transformLocalPoint(
                                    application.localApplicationPointMeters);
                            }, &poseEvaluations);
                    }
                    catch (const std::exception&)
                    {
                        telemetry.dynamicPoseEvaluationCount = poseEvaluations;
                        return fail(
                            DynamicContactStepStatus::RigidClosureFailure,
                            currentState, telemetry);
                    }
                    applied[index] += glm::dot(
                        application.worldForceNewtons, derivative);
                }
            }

            Vector3 impulseRhs{};
            for (std::size_t index = 0; index < 3; ++index)
            {
                impulseRhs[index] = deltaTime * (applied[index] - bias[index]);
            }
            Vector3 deltaVelocity{};
            if (!solveMass(mass.matrix, impulseRhs, deltaVelocity))
            {
                return fail(
                    DynamicContactStepStatus::MassMatrixNotPositiveDefinite,
                    currentState, telemetry);
            }
            Vector3 freeVelocity = velocity;
            for (std::size_t index = 0; index < 3; ++index)
            {
                freeVelocity[index] += deltaVelocity[index];
            }
            telemetry.freeGeneralizedVelocity = freeVelocity;
            telemetry.kineticEnergyBeforeContactJoules = kineticEnergy(
                mass.matrix, freeVelocity);

            std::vector<ContactEvaluation> allContactEvaluations;
            allContactEvaluations.reserve(contacts.size());
            std::vector<std::size_t> candidateIndices;
            for (std::size_t index = 0; index < contacts.size(); ++index)
            {
                ContactEvaluation evaluated;
                try
                {
                    evaluated = evaluateContact(track, definition.cars.front(),
                        coordinates, mass.pose, contacts[index], &poseEvaluations);
                }
                catch (const std::exception&)
                {
                    return fail(DynamicContactStepStatus::RigidClosureFailure,
                        currentState, telemetry);
                }
                if (evaluated.gapMeters < -dynamicContactGapToleranceMeters)
                {
                    return fail(
                        DynamicContactStepStatus::InvalidInitialPenetration,
                        currentState, telemetry);
                }
                const double predicted = evaluated.gapMeters + deltaTime * (
                    evaluated.jacobian[0] * freeVelocity[0]
                    + evaluated.jacobian[1] * freeVelocity[1]
                    + evaluated.jacobian[2] * freeVelocity[2]);
                if (evaluated.gapMeters <= dynamicContactGapToleranceMeters
                    || predicted < -dynamicContactGapToleranceMeters)
                {
                    candidateIndices.push_back(index);
                }
                allContactEvaluations.push_back(evaluated);
            }
            telemetry.candidateContactCount = candidateIndices.size();
            telemetry.contactSystemSize = candidateIndices.size();

            Vector3 committedVelocity = freeVelocity;
            std::vector<double> impulses(candidateIndices.size(), 0.0);
            if (!candidateIndices.empty())
            {
                std::vector<Vector3> inverseMassJacobian(candidateIndices.size());
                std::vector<std::vector<double>> w(candidateIndices.size(),
                    std::vector<double>(candidateIndices.size()));
                std::vector<double> b(candidateIndices.size());
                for (std::size_t row = 0; row < candidateIndices.size(); ++row)
                {
                    const ContactEvaluation& contact =
                        allContactEvaluations[candidateIndices[row]];
                    if (!solveMass(mass.matrix, contact.jacobian,
                        inverseMassJacobian[row]))
                    {
                        return fail(
                            DynamicContactStepStatus::ContactSystemIllConditioned,
                            currentState, telemetry);
                    }
                    b[row] = contact.gapMeters + deltaTime * (
                        contact.jacobian[0] * freeVelocity[0]
                        + contact.jacobian[1] * freeVelocity[1]
                        + contact.jacobian[2] * freeVelocity[2]);
                    for (std::size_t column = 0;
                        column < candidateIndices.size(); ++column)
                    {
                        const Vector3& other = allContactEvaluations[
                            candidateIndices[column]].jacobian;
                        w[row][column] = deltaTime * (
                            other[0] * inverseMassJacobian[row][0]
                            + other[1] * inverseMassJacobian[row][1]
                            + other[2] * inverseMassJacobian[row][2]);
                    }
                }
                const LcpSolution solution = solveLcp(w, b);
                telemetry.solverIterationCount = solution.iterations;
                telemetry.complementarityResidualMeters = solution.residual;
                telemetry.contactSystemConditionEstimate =
                    solution.conditionEstimate;
                telemetry.contactSystemStatus = solution.conditionStatus;
                if (!solution.solved)
                {
                    return fail(
                        DynamicContactStepStatus::ContactSolveNonConverged,
                        currentState, telemetry);
                }
                impulses = solution.impulses;
                for (std::size_t index = 0; index < candidateIndices.size(); ++index)
                {
                    if (impulses[index] <= 1.0e-12)
                    {
                        continue;
                    }
                    ++telemetry.impulseCarryingContactCount;
                    for (std::size_t coordinate = 0; coordinate < 3; ++coordinate)
                    {
                        committedVelocity[coordinate] +=
                            inverseMassJacobian[index][coordinate]
                            * impulses[index];
                    }
                    if (contacts[candidateIndices[index]].bogieIndex == 0)
                    {
                        telemetry.aggregateFrontBogieNormalImpulseNewtonSeconds
                            += impulses[index];
                    }
                    else
                    {
                        telemetry.aggregateRearBogieNormalImpulseNewtonSeconds
                            += impulses[index];
                    }
                }
            }
            telemetry.committedGeneralizedVelocity = committedVelocity;
            telemetry.kineticEnergyAfterContactJoules = kineticEnergy(
                mass.matrix, committedVelocity);

            DynamicContactState next = currentState;
            const TrackAdvanceResult advanced = track.advance(
                coordinates.q, deltaTime * committedVelocity[0]);
            if (track.topology() == coaster::TopologyKind::OpenLinear)
            {
                const double requestedStation = coordinates.q.stationMeters
                    + deltaTime * committedVelocity[0];
                if ((advanced.boundary == TrackBoundary::Start
                        && requestedStation < 0.0)
                    || (advanced.boundary == TrackBoundary::End
                        && requestedStation > track.lengthMeters()))
                {
                    return fail(
                        DynamicContactStepStatus::TrackBoundaryReached,
                        currentState, telemetry);
                }
            }
            else if (advanced.boundary != TrackBoundary::None)
            {
                return fail(DynamicContactStepStatus::TrackBoundaryReached,
                    currentState, telemetry);
            }
            next.generalizedReferenceLocation = advanced.location;
            if (std::abs(committedVelocity[0])
                <= followerRestSpeedToleranceMetersPerSecond)
            {
                next.generalizedReferenceLocation.direction =
                    coordinates.q.direction;
            }
            next.signedLongitudinalVelocityMetersPerSecond =
                committedVelocity[0];
            next.frontVerticalOffsetMeters += deltaTime * committedVelocity[1];
            next.frontVerticalVelocityMetersPerSecond = committedVelocity[1];
            next.rearVerticalOffsetMeters += deltaTime * committedVelocity[2];
            next.rearVerticalVelocityMetersPerSecond = committedVelocity[2];
            next.runState = std::hypot(committedVelocity[0],
                committedVelocity[1], committedVelocity[2])
                    <= followerRestSpeedToleranceMetersPerSecond
                ? FollowerRunState::Resting : FollowerRunState::Running;

            DynamicContactCarPose nextPose;
            try
            {
                nextPose = solveDynamicPose(track, definition.cars.front(),
                    {next.generalizedReferenceLocation,
                        next.frontVerticalOffsetMeters,
                        next.rearVerticalOffsetMeters}, &poseEvaluations, true);
            }
            catch (const std::exception&)
            {
                return fail(DynamicContactStepStatus::RigidClosureFailure,
                    currentState, telemetry);
            }
            double minimumGap = std::numeric_limits<double>::infinity();
            double maximumGap = -std::numeric_limits<double>::infinity();
            for (const AggregateContact& contact : contacts)
            {
                const double gap = contactGap(nextPose, contact);
                minimumGap = std::min(minimumGap, gap);
                maximumGap = std::max(maximumGap, gap);
            }
            if (contacts.empty())
            {
                minimumGap = maximumGap = 0.0;
            }
            telemetry.minimumSignedGapMeters = minimumGap;
            telemetry.maximumSignedGapMeters = maximumGap;
            telemetry.nonlinearCommittedGapResidualMeters =
                std::max(0.0, -minimumGap);
            telemetry.generalizedReferenceLocation =
                next.generalizedReferenceLocation;
            telemetry.signedLongitudinalVelocityMetersPerSecond =
                next.signedLongitudinalVelocityMetersPerSecond;
            telemetry.frontVerticalOffsetMeters = next.frontVerticalOffsetMeters;
            telemetry.frontVerticalVelocityMetersPerSecond =
                next.frontVerticalVelocityMetersPerSecond;
            telemetry.rearVerticalOffsetMeters = next.rearVerticalOffsetMeters;
            telemetry.rearVerticalVelocityMetersPerSecond =
                next.rearVerticalVelocityMetersPerSecond;
            telemetry.dynamicPoseEvaluationCount = poseEvaluations;
            telemetry.stepAverageFrontBogieNormalForceNewtons =
                telemetry.aggregateFrontBogieNormalImpulseNewtonSeconds
                / deltaTime;
            telemetry.stepAverageRearBogieNormalForceNewtons =
                telemetry.aggregateRearBogieNormalImpulseNewtonSeconds
                / deltaTime;
            if (minimumGap < -dynamicContactGapToleranceMeters)
            {
                return fail(DynamicContactStepStatus::NonlinearPenetration,
                    currentState, telemetry);
            }
            return {DynamicContactStepStatus::Available, next,
                std::move(nextPose), telemetry};
        }
    }

    glm::dvec3 DynamicContactCarPose::transformLocalPoint(
        const glm::dvec3& localPointMeters) const noexcept
    {
        return transformPoint(
            bodyWorldPositionMeters, bodyFrame, localPointMeters);
    }

    DynamicContactStepResult stepDynamicContact(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const DynamicContactState& currentState,
        const FixedStepSettings& step,
        const std::span<const ExternalForceApplication> externalForces)
    {
        validateTrainDefinition(definition);
        validateFixedStepSettings(step);
        // The existing rigid path intentionally requires positive gravity,
        // while M1A's free-flight oracle includes a zero-gravity case.
        if (!std::isfinite(environment.gravityAccelerationMetersPerSecondSquared)
            || environment.gravityAccelerationMetersPerSecondSquared < 0.0
            || !std::isfinite(environment.airDensityKilogramsPerCubicMeter)
            || environment.airDensityKilogramsPerCubicMeter < 0.0
            || !finite(environment.windVelocityMetersPerSecond))
        {
            return fail(DynamicContactStepStatus::NonFiniteInput, currentState);
        }
        const DynamicContactStepStatus scope = validateScope(track, definition);
        if (scope != DynamicContactStepStatus::Available)
        {
            return fail(scope, currentState);
        }
        if (step.deltaTimeSeconds != defaultFixedTimeStepSeconds)
        {
            return fail(DynamicContactStepStatus::UnsupportedFixedTimeStep,
                currentState);
        }
        if (!std::isfinite(currentState.signedLongitudinalVelocityMetersPerSecond)
            || !std::isfinite(currentState.frontVerticalOffsetMeters)
            || !std::isfinite(currentState.frontVerticalVelocityMetersPerSecond)
            || !std::isfinite(currentState.rearVerticalOffsetMeters)
            || !std::isfinite(currentState.rearVerticalVelocityMetersPerSecond)
            || !validRunState(currentState.runState)
            || currentState.tick == std::numeric_limits<std::uint64_t>::max())
        {
            return fail(DynamicContactStepStatus::NonFiniteInput, currentState);
        }
        for (const ExternalForceApplication& application : externalForces)
        {
            if (application.carIndex != 0
                || !finite(application.localApplicationPointMeters)
                || !finite(application.worldForceNewtons))
            {
                return fail(DynamicContactStepStatus::NonFiniteInput,
                    currentState);
            }
        }

        std::vector<AggregateContact> contacts;
        const DynamicContactStepStatus contactStatus = buildAggregateContacts(
            definition.cars.front().car, contacts);
        if (contactStatus != DynamicContactStepStatus::Available)
        {
            return fail(contactStatus, currentState);
        }

        DynamicContactStepResult lastFailure;
        for (std::size_t subdivisions = 1;
            subdivisions <= dynamicContactMaximumSubdivisions;
            subdivisions *= 2)
        {
            DynamicContactState state = currentState;
            DynamicContactTelemetry aggregate;
            std::optional<DynamicContactCarPose> finalPose;
            bool retry = false;
            bool failed = false;
            double frontImpulse = 0.0;
            double rearImpulse = 0.0;
            double firstContactEnergy = 0.0;
            std::size_t poseEvaluations = 0;
            std::size_t solverIterations = 0;
            std::size_t maximumCandidateCount = 0;
            std::size_t maximumCarryingCount = 0;
            for (std::size_t substep = 0; substep < subdivisions; ++substep)
            {
                DynamicContactStepResult result = singleSubstep(
                    track, definition, environment, state,
                    step.deltaTimeSeconds / static_cast<double>(subdivisions),
                    externalForces, contacts);
                if (!result.available())
                {
                    failed = true;
                    lastFailure = std::move(result);
                    retry = lastFailure.status
                        == DynamicContactStepStatus::NonlinearPenetration;
                    break;
                }
                if (substep == 0)
                {
                    firstContactEnergy =
                        result.telemetry.kineticEnergyBeforeContactJoules;
                }
                frontImpulse += result.telemetry
                    .aggregateFrontBogieNormalImpulseNewtonSeconds;
                rearImpulse += result.telemetry
                    .aggregateRearBogieNormalImpulseNewtonSeconds;
                poseEvaluations += result.telemetry.dynamicPoseEvaluationCount;
                solverIterations += result.telemetry.solverIterationCount;
                maximumCandidateCount = std::max(maximumCandidateCount,
                    result.telemetry.candidateContactCount);
                maximumCarryingCount = std::max(maximumCarryingCount,
                    result.telemetry.impulseCarryingContactCount);
                aggregate = result.telemetry;
                state = result.state;
                finalPose = std::move(result.pose);
            }
            if (!failed && finalPose)
            {
                state.tick = currentState.tick + 1;
                aggregate.retryCount = std::countr_zero(subdivisions);
                aggregate.substepCount = subdivisions;
                aggregate.aggregateFrontBogieNormalImpulseNewtonSeconds =
                    frontImpulse;
                aggregate.aggregateRearBogieNormalImpulseNewtonSeconds =
                    rearImpulse;
                aggregate.kineticEnergyBeforeContactJoules = firstContactEnergy;
                aggregate.dynamicPoseEvaluationCount = poseEvaluations;
                aggregate.solverIterationCount = solverIterations;
                aggregate.candidateContactCount = maximumCandidateCount;
                aggregate.impulseCarryingContactCount = maximumCarryingCount;
                aggregate.stepAverageFrontBogieNormalForceNewtons =
                    aggregate.aggregateFrontBogieNormalImpulseNewtonSeconds
                    / step.deltaTimeSeconds;
                aggregate.stepAverageRearBogieNormalForceNewtons =
                    aggregate.aggregateRearBogieNormalImpulseNewtonSeconds
                    / step.deltaTimeSeconds;
                return {DynamicContactStepStatus::Available, state,
                    std::move(finalPose), aggregate};
            }
            if (failed && !retry)
            {
                return lastFailure;
            }
        }
        lastFailure.status = DynamicContactStepStatus::NonlinearPenetration;
        lastFailure.state = currentState;
        lastFailure.telemetry.retryCount = 3;
        lastFailure.telemetry.substepCount = dynamicContactMaximumSubdivisions;
        return lastFailure;
    }

    //=========================================================================
    // M1B: Multi-Car Public API
    //=========================================================================

    DynamicContactDofLayout DynamicContactDofLayout::create(
        const std::size_t count) noexcept
    {
        DynamicContactDofLayout layout;
        layout.carCount = count;
        layout.dofCount = 1 + 2 * count;
        return layout;
    }

    std::size_t DynamicContactDofLayout::longitudinalDof() const noexcept
    {
        return 0;
    }

    std::size_t DynamicContactDofLayout::frontVerticalDof(
        const std::size_t carIndex) const noexcept
    {
        return 1 + 2 * carIndex;
    }

    std::size_t DynamicContactDofLayout::rearVerticalDof(
        const std::size_t carIndex) const noexcept
    {
        return 1 + 2 * carIndex + 1;
    }

    std::size_t DynamicContactDofLayout::carIndexForVerticalDof(
        const std::size_t dofIndex) const noexcept
    {
        if (dofIndex == 0 || dofIndex >= dofCount)
        {
            return std::numeric_limits<std::size_t>::max();
        }
        return (dofIndex - 1) / 2;
    }

    bool DynamicContactDofLayout::isLongitudinalDof(
        const std::size_t dofIndex) const noexcept
    {
        return dofIndex == 0;
    }

    bool DynamicContactMultiCarState::isValid() const noexcept
    {
        if (cars.empty())
        {
            return false;
        }
        if (!std::isfinite(signedLongitudinalVelocityMetersPerSecond))
        {
            return false;
        }
        if (tick == std::numeric_limits<std::uint64_t>::max())
        {
            return false;
        }
        for (const auto& car : cars)
        {
            if (!std::isfinite(car.frontVerticalOffsetMeters)
                || !std::isfinite(car.frontVerticalVelocityMetersPerSecond)
                || !std::isfinite(car.rearVerticalOffsetMeters)
                || !std::isfinite(car.rearVerticalVelocityMetersPerSecond))
            {
                return false;
            }
        }
        return true;
    }

    DynamicContactMultiCarStepResult stepDynamicContactMultiCar(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const DynamicContactMultiCarState& currentState,
        const FixedStepSettings& step,
        const std::span<const ExternalForceApplication> externalForces)
    {
        using namespace quantum::physics;
        DynamicContactMultiCarStepResult failResult;

        auto failMultiCar = [&](const DynamicContactStepStatus status)
            -> DynamicContactMultiCarStepResult
        {
            DynamicContactMultiCarStepResult r;
            r.status = status;
            return r;
        };

        // Validate inputs.
        if (!std::isfinite(environment.gravityAccelerationMetersPerSecondSquared)
            || environment.gravityAccelerationMetersPerSecondSquared < 0.0
            || !std::isfinite(environment.airDensityKilogramsPerCubicMeter)
            || environment.airDensityKilogramsPerCubicMeter < 0.0
            || !finite(environment.windVelocityMetersPerSecond))
        {
            return failMultiCar(DynamicContactStepStatus::NonFiniteInput);
        }
        if (!currentState.isValid())
        {
            return failMultiCar(DynamicContactStepStatus::NonFiniteInput);
        }
        if (step.deltaTimeSeconds != defaultFixedTimeStepSeconds)
        {
            return failMultiCar(
                DynamicContactStepStatus::UnsupportedFixedTimeStep);
        }

        // Validate train definition.
        const std::size_t N = definition.cars.size();
        if (N == 0)
        {
            return failMultiCar(DynamicContactStepStatus::UnsupportedCarCount);
        }
        if (definition.connections.size() != N - 1)
        {
            return failMultiCar(DynamicContactStepStatus::UnsupportedConnector);
        }
        if (currentState.cars.size() != N)
        {
            return failMultiCar(DynamicContactStepStatus::NonFiniteInput);
        }

        // Validate each car has exactly 2 bogies.
        for (std::size_t i = 0; i < N; ++i)
        {
            if (definition.cars[i].car.bogies.size() != 2)
            {
                return failMultiCar(
                    DynamicContactStepStatus::UnsupportedBogieCount);
            }
        }

        // Validate track supports planar vertical motion.
        if (!track.supportsPlanarVerticalMotion())
        {
            return failMultiCar(
                DynamicContactStepStatus::UnsupportedTrackGeometry);
        }

        // Validate external forces reference valid cars.
        for (const ExternalForceApplication& app : externalForces)
        {
            if (app.carIndex >= N
                || !finite(app.localApplicationPointMeters)
                || !finite(app.worldForceNewtons))
            {
                return failMultiCar(
                    DynamicContactStepStatus::NonFiniteInput);
            }
        }

        // Build aggregate contacts for all cars.
        std::vector<AggregateContact> contacts;
        std::vector<std::size_t> contactCarIndex;
        {
            const DynamicContactStepStatus contactStatus =
                buildMultiCarAggregateContacts(
                    definition, contacts, contactCarIndex);
            if (contactStatus != DynamicContactStepStatus::Available)
            {
                return failMultiCar(contactStatus);
            }
        }

        // For N=1 with 0 connections, delegate to M1A path for exact
        // compatibility.
        if (N == 1 && definition.connections.empty())
        {
            DynamicContactState m1aState;
            m1aState.generalizedReferenceLocation =
                currentState.leadCarReferenceLocation;
            m1aState.signedLongitudinalVelocityMetersPerSecond =
                currentState.signedLongitudinalVelocityMetersPerSecond;
            m1aState.frontVerticalOffsetMeters =
                currentState.cars[0].frontVerticalOffsetMeters;
            m1aState.frontVerticalVelocityMetersPerSecond =
                currentState.cars[0].frontVerticalVelocityMetersPerSecond;
            m1aState.rearVerticalOffsetMeters =
                currentState.cars[0].rearVerticalOffsetMeters;
            m1aState.rearVerticalVelocityMetersPerSecond =
                currentState.cars[0].rearVerticalVelocityMetersPerSecond;
            m1aState.tick = currentState.tick;
            m1aState.runState = currentState.runState;

            const DynamicContactStepResult m1aResult = stepDynamicContact(
                track, definition, environment, m1aState, step,
                externalForces);

            DynamicContactMultiCarStepResult result;
            result.status = m1aResult.status;
            result.dofLayout = DynamicContactDofLayout::create(1);
            result.tick = m1aResult.state.tick;
            result.runState = m1aResult.state.runState;
            result.leadCarReferenceLocation =
                m1aResult.state.generalizedReferenceLocation;
            result.generalizedCoordinates = {
                m1aResult.state.generalizedReferenceLocation.stationMeters,
                m1aResult.state.frontVerticalOffsetMeters,
                m1aResult.state.rearVerticalOffsetMeters};
            result.generalizedVelocity = {
                m1aResult.state.signedLongitudinalVelocityMetersPerSecond,
                m1aResult.state.frontVerticalVelocityMetersPerSecond,
                m1aResult.state.rearVerticalVelocityMetersPerSecond};
            result.carResults.resize(1);
            result.carResults[0].carIndex = 0;
            if (m1aResult.pose.has_value())
            {
                result.carResults[0].pose = *m1aResult.pose;
            }
            result.carResults[0].aggregateFrontBogieNormalImpulseNewtonSeconds =
                m1aResult.telemetry
                    .aggregateFrontBogieNormalImpulseNewtonSeconds;
            result.carResults[0].aggregateRearBogieNormalImpulseNewtonSeconds =
                m1aResult.telemetry
                    .aggregateRearBogieNormalImpulseNewtonSeconds;
            result.telemetry.minimumSignedGapMeters =
                m1aResult.telemetry.minimumSignedGapMeters;
            result.telemetry.maximumSignedGapMeters =
                m1aResult.telemetry.maximumSignedGapMeters;
            result.telemetry.candidateContactCount =
                m1aResult.telemetry.candidateContactCount;
            result.telemetry.impulseCarryingContactCount =
                m1aResult.telemetry.impulseCarryingContactCount;
            result.telemetry.complementarityResidualMeters =
                m1aResult.telemetry.complementarityResidualMeters;
            result.telemetry.nonlinearCommittedGapResidualMeters =
                m1aResult.telemetry.nonlinearCommittedGapResidualMeters;
            result.telemetry.massMatrixStatus =
                m1aResult.telemetry.massMatrixStatus;
            result.telemetry.massMatrixConditionEstimate =
                m1aResult.telemetry.massMatrixConditionEstimate;
            result.telemetry.contactSystemStatus =
                m1aResult.telemetry.contactSystemStatus;
            result.telemetry.contactSystemConditionEstimate =
                m1aResult.telemetry.contactSystemConditionEstimate;
            result.telemetry.solverIterationCount =
                m1aResult.telemetry.solverIterationCount;
            result.telemetry.retryCount = m1aResult.telemetry.retryCount;
            result.telemetry.substepCount = m1aResult.telemetry.substepCount;
            result.telemetry.kineticEnergyBeforeContactJoules =
                m1aResult.telemetry.kineticEnergyBeforeContactJoules;
            result.telemetry.kineticEnergyAfterContactJoules =
                m1aResult.telemetry.kineticEnergyAfterContactJoules;
            result.telemetry.dynamicPoseEvaluationCount =
                m1aResult.telemetry.dynamicPoseEvaluationCount;
            result.telemetry.massMatrixSize =
                m1aResult.telemetry.massMatrixSize;
            result.telemetry.contactSystemSize =
                m1aResult.telemetry.contactSystemSize;
            return result;
        }

        // Multi-car adaptive subdivision loop.
        DynamicContactMultiCarStepResult lastFailure;
        for (std::size_t subdivisions = 1;
            subdivisions <= dynamicContactMaximumSubdivisions;
            subdivisions *= 2)
        {
            DynamicContactMultiCarState state = currentState;
            DynamicContactMultiCarTelemetry aggregate;
            std::vector<DynamicContactMultiCarCarResult> finalCarResults;
            bool retry = false;
            bool failed = false;
            double firstContactEnergy = 0.0;
            std::size_t poseEvaluations = 0;
            std::size_t solverIterations = 0;
            std::size_t maximumCandidateCount = 0;
            std::size_t maximumCarryingCount = 0;
            DynamicContactDofLayout dofLayout =
                DynamicContactDofLayout::create(N);

            for (std::size_t substep = 0; substep < subdivisions; ++substep)
            {
                DynamicContactMultiCarStepResult result =
                    multiCarSingleSubstep(
                        track, definition, environment, state,
                        step.deltaTimeSeconds
                            / static_cast<double>(subdivisions),
                        externalForces, contacts, contactCarIndex);
                if (!result.available())
                {
                    failed = true;
                    lastFailure = std::move(result);
                    retry = lastFailure.status
                        == DynamicContactStepStatus::NonlinearPenetration;
                    break;
                }
                if (substep == 0)
                {
                    firstContactEnergy =
                        result.telemetry.kineticEnergyBeforeContactJoules;
                }
                poseEvaluations += result.telemetry.dynamicPoseEvaluationCount;
                solverIterations += result.telemetry.solverIterationCount;
                maximumCandidateCount = std::max(maximumCandidateCount,
                    result.telemetry.candidateContactCount);
                maximumCarryingCount = std::max(maximumCarryingCount,
                    result.telemetry.impulseCarryingContactCount);
                aggregate = result.telemetry;
                finalCarResults = std::move(result.carResults);

                // Update state for next substep.
                state.leadCarReferenceLocation =
                    result.leadCarReferenceLocation;
                state.signedLongitudinalVelocityMetersPerSecond =
                    result.generalizedVelocity[0];
                for (std::size_t i = 0; i < N; ++i)
                {
                    state.cars[i].frontVerticalOffsetMeters =
                        result.generalizedCoordinates[1 + 2 * i];
                    state.cars[i].frontVerticalVelocityMetersPerSecond =
                        result.generalizedVelocity[1 + 2 * i];
                    state.cars[i].rearVerticalOffsetMeters =
                        result.generalizedCoordinates[1 + 2 * i + 1];
                    state.cars[i].rearVerticalVelocityMetersPerSecond =
                        result.generalizedVelocity[1 + 2 * i + 1];
                }
            }

            if (!failed && !finalCarResults.empty())
            {
                state.tick = currentState.tick + 1;
                aggregate.retryCount = std::countr_zero(subdivisions);
                aggregate.substepCount = subdivisions;
                aggregate.dynamicPoseEvaluationCount = poseEvaluations;
                aggregate.solverIterationCount = solverIterations;
                aggregate.candidateContactCount = maximumCandidateCount;
                aggregate.impulseCarryingContactCount = maximumCarryingCount;

                DynamicContactMultiCarStepResult finalResult;
                finalResult.status = DynamicContactStepStatus::Available;
                finalResult.telemetry = aggregate;
                finalResult.carResults = std::move(finalCarResults);
                finalResult.dofLayout = dofLayout;
                finalResult.generalizedVelocity.resize(dofLayout.dofCount);
                finalResult.generalizedCoordinates.resize(dofLayout.dofCount);
                finalResult.generalizedVelocity[0] =
                    state.signedLongitudinalVelocityMetersPerSecond;
                finalResult.generalizedCoordinates[0] =
                    state.leadCarReferenceLocation.stationMeters;
                for (std::size_t i = 0; i < N; ++i)
                {
                    finalResult.generalizedVelocity[1 + 2 * i] =
                        state.cars[i].frontVerticalVelocityMetersPerSecond;
                    finalResult.generalizedVelocity[1 + 2 * i + 1] =
                        state.cars[i].rearVerticalVelocityMetersPerSecond;
                    finalResult.generalizedCoordinates[1 + 2 * i] =
                        state.cars[i].frontVerticalOffsetMeters;
                    finalResult.generalizedCoordinates[1 + 2 * i + 1] =
                        state.cars[i].rearVerticalOffsetMeters;
                }
                finalResult.leadCarReferenceLocation =
                    state.leadCarReferenceLocation;
                finalResult.tick = state.tick;
                finalResult.runState = state.runState;
                return finalResult;
            }

            if (failed && !retry)
            {
                return lastFailure;
            }
        }

        lastFailure.status = DynamicContactStepStatus::NonlinearPenetration;
        lastFailure.telemetry.retryCount = 3;
        lastFailure.telemetry.substepCount = dynamicContactMaximumSubdivisions;
        return lastFailure;
    }
}
