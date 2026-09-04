#include <quantum/physics/CarPose.hpp>

#include <glm/geometric.hpp>
#include <glm/mat3x3.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace quantum::physics
{
    namespace
    {
        inline constexpr double minimumBogieSeparationMeters = 1.0e-9;
        inline constexpr double directionalResolution =
            128.0 * std::numeric_limits<double>::epsilon();

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

        [[nodiscard]] bool finite(const glm::dmat3& value) noexcept
        {
            for (glm::length_t column = 0; column < 3; ++column)
            {
                for (glm::length_t row = 0; row < 3; ++row)
                {
                    if (!std::isfinite(value[column][row]))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        [[nodiscard]] bool validContactRole(
            const BogieContactRole role) noexcept
        {
            switch (role)
            {
            case BogieContactRole::Running:
            case BogieContactRole::Guide:
            case BogieContactRole::Upstop:
                return true;
            }
            return false;
        }

        [[nodiscard]] double maximumAbsoluteElement(
            const glm::dmat3& value) noexcept
        {
            double result = 0.0;
            for (glm::length_t column = 0; column < 3; ++column)
            {
                for (glm::length_t row = 0; row < 3; ++row)
                {
                    result = std::max(result, std::abs(value[column][row]));
                }
            }
            return result;
        }

        [[nodiscard]] double determinant(const glm::dmat3& value) noexcept
        {
            return value[0][0]
                    * (value[1][1] * value[2][2]
                        - value[2][1] * value[1][2])
                - value[1][0]
                    * (value[0][1] * value[2][2]
                        - value[2][1] * value[0][2])
                + value[2][0]
                    * (value[0][1] * value[1][2]
                        - value[1][1] * value[0][2]);
        }

        void validateInertiaTensor(const glm::dmat3& inertia)
        {
            if (!finite(inertia))
            {
                throw std::invalid_argument(
                    "Car dry inertia tensor must contain only finite kg*m^2 values.");
            }

            const double scale = maximumAbsoluteElement(inertia);
            if (!(scale > 0.0) || !std::isfinite(scale))
            {
                throw std::invalid_argument(
                    "Car dry inertia tensor must be non-singular and positive definite.");
            }
            constexpr double relativeTolerance = 1.0e-10;
            const double symmetryTolerance = relativeTolerance * scale;
            for (glm::length_t column = 0; column < 3; ++column)
            {
                for (glm::length_t row = column + 1; row < 3; ++row)
                {
                    if (std::abs(inertia[column][row]
                            - inertia[row][column]) > symmetryTolerance)
                    {
                        throw std::invalid_argument(
                            "Car dry inertia tensor must be symmetric in the body frame.");
                    }
                }
            }

            // Scale before testing principal minors so very large/small but
            // otherwise valid SI tensors do not overflow or underflow.
            const glm::dmat3 normalized = inertia / scale;
            constexpr double positiveDefiniteTolerance = 1.0e-12;
            const double leadingTwoByTwo = normalized[0][0]
                    * normalized[1][1]
                - normalized[1][0] * normalized[0][1];
            const double normalizedDeterminant = determinant(normalized);
            if (normalized[0][0] <= positiveDefiniteTolerance
                || leadingTwoByTwo <= positiveDefiniteTolerance
                || normalizedDeterminant <= positiveDefiniteTolerance)
            {
                throw std::invalid_argument(
                    "Car dry inertia tensor must be symmetric positive definite and numerically non-singular.");
            }

            // Principal inertias of a physical mass distribution obey the
            // triangle inequalities. K=trace(I)/2*I3-I has eigenvalues
            // (Ij+Ik-Ii)/2, so positive-semidefinite principal minors of K
            // enforce those inequalities without choosing principal axes.
            const double halfTrace = 0.5 * (
                normalized[0][0]
                + normalized[1][1]
                + normalized[2][2]);
            glm::dmat3 physicality{-normalized};
            physicality[0][0] += halfTrace;
            physicality[1][1] += halfTrace;
            physicality[2][2] += halfTrace;
            const double p01 = physicality[0][0] * physicality[1][1]
                - physicality[1][0] * physicality[0][1];
            const double p02 = physicality[0][0] * physicality[2][2]
                - physicality[2][0] * physicality[0][2];
            const double p12 = physicality[1][1] * physicality[2][2]
                - physicality[2][1] * physicality[1][2];
            if (physicality[0][0] < -relativeTolerance
                || physicality[1][1] < -relativeTolerance
                || physicality[2][2] < -relativeTolerance
                || p01 < -relativeTolerance
                || p02 < -relativeTolerance
                || p12 < -relativeTolerance
                || determinant(physicality) < -relativeTolerance)
            {
                throw std::invalid_argument(
                    "Car dry inertia tensor violates physical principal-moment triangle inequalities.");
            }
        }

        [[nodiscard]] glm::dmat3 parallelAxisShift(
            const double massKilograms,
            const glm::dvec3& displacementMeters) noexcept
        {
            const double squaredDistance = glm::dot(
                displacementMeters, displacementMeters);
            glm::dmat3 result{squaredDistance};
            for (glm::length_t column = 0; column < 3; ++column)
            {
                for (glm::length_t row = 0; row < 3; ++row)
                {
                    result[column][row] -= displacementMeters[column]
                        * displacementMeters[row];
                }
            }
            return massKilograms * result;
        }

        [[nodiscard]] double magnitude(const glm::dvec3& value) noexcept
        {
            return std::hypot(value.x, value.y, value.z);
        }

        [[nodiscard]] double directionSign(
            const TravelDirection direction)
        {
            switch (direction)
            {
            case TravelDirection::IncreasingStation:
                return 1.0;
            case TravelDirection::DecreasingStation:
                return -1.0;
            }
            throw std::invalid_argument("Car travel direction is invalid.");
        }

        [[nodiscard]] glm::dvec3 normalized(
            const glm::dvec3& value,
            const char* const errorMessage)
        {
            const double length = magnitude(value);
            if (!std::isfinite(length) || length == 0.0)
            {
                throw std::domain_error(errorMessage);
            }

            const glm::dvec3 result = value / length;
            if (!finite(result))
            {
                throw std::domain_error(errorMessage);
            }
            return result;
        }

        [[nodiscard]] glm::dquat canonicalized(
            glm::dquat orientation)
        {
            orientation = glm::normalize(orientation);
            if (!finite(orientation))
            {
                throw std::domain_error(
                    "Car pose orientation could not be normalized.");
            }

            if (orientation.w < 0.0
                || (orientation.w == 0.0 && orientation.x < 0.0)
                || (orientation.w == 0.0 && orientation.x == 0.0
                    && orientation.y < 0.0)
                || (orientation.w == 0.0 && orientation.x == 0.0
                    && orientation.y == 0.0 && orientation.z < 0.0))
            {
                orientation = -orientation;
            }
            return orientation;
        }

        [[nodiscard]] glm::dquat orientationFromFrame(
            const geometry::CurveFrame& frame)
        {
            return canonicalized(glm::quat_cast(glm::dmat3{
                frame.tangent,
                frame.lateral,
                frame.up
            }));
        }

        [[nodiscard]] geometry::CurveFrame orientedTrackFrame(
            const geometry::CurveFrame& frame,
            const TravelDirection direction)
        {
            if (direction == TravelDirection::IncreasingStation)
            {
                return frame;
            }
            if (direction == TravelDirection::DecreasingStation)
            {
                // Negating tangent and lateral preserves T x L = U and keeps
                // the physical track-up/bank direction unchanged.
                return {-frame.tangent, -frame.lateral, frame.up};
            }
            throw std::invalid_argument("Car travel direction is invalid.");
        }

        [[nodiscard]] glm::dvec3 projectedPerpendicular(
            const glm::dvec3& vector,
            const glm::dvec3& unitForward) noexcept
        {
            return vector - glm::dot(vector, unitForward) * unitForward;
        }

        [[nodiscard]] bool usableDirection(
            const glm::dvec3& value) noexcept
        {
            const double length = magnitude(value);
            return std::isfinite(length) && length > directionalResolution;
        }

        [[nodiscard]] glm::dvec3 fallbackWorldUp(
            const glm::dvec3& unitForward)
        {
            const std::array<glm::dvec3, 3> axes{
                glm::dvec3{1.0, 0.0, 0.0},
                glm::dvec3{0.0, 1.0, 0.0},
                glm::dvec3{0.0, 0.0, 1.0}
            };
            const auto leastAligned = std::min_element(
                axes.begin(), axes.end(),
                [&unitForward](const glm::dvec3& left,
                               const glm::dvec3& right)
                {
                    return std::abs(glm::dot(left, unitForward))
                        < std::abs(glm::dot(right, unitForward));
                });
            return projectedPerpendicular(*leastAligned, unitForward);
        }

        [[nodiscard]] geometry::CurveFrame bodyFrameFromBogies(
            const glm::dvec3& rearPosition,
            const glm::dvec3& frontPosition,
            const geometry::CurveFrame& rearFrame,
            const geometry::CurveFrame& frontFrame)
        {
            const glm::dvec3 chord = frontPosition - rearPosition;
            const double chordLength = magnitude(chord);
            const double positionScale = std::max({
                1.0,
                magnitude(frontPosition),
                magnitude(rearPosition)
            });
            if (!std::isfinite(chordLength)
                || chordLength <= directionalResolution * positionScale)
            {
                throw std::domain_error(
                    "The sampled front and rear bogie positions cannot define a car body direction.");
            }
            const glm::dvec3 forward = chord / chordLength;

            // Average both track-up axes before projection so pitch, yaw, and
            // bank at both bogies influence the body. Ordered fallbacks retain
            // one sampled up direction before choosing a world axis.
            glm::dvec3 projectedUp = projectedPerpendicular(
                rearFrame.up + frontFrame.up, forward);
            if (!usableDirection(projectedUp))
            {
                projectedUp = projectedPerpendicular(frontFrame.up, forward);
            }
            if (!usableDirection(projectedUp))
            {
                projectedUp = projectedPerpendicular(rearFrame.up, forward);
            }
            if (!usableDirection(projectedUp))
            {
                projectedUp = fallbackWorldUp(forward);
            }

            glm::dvec3 up = normalized(
                projectedUp,
                "The bogie track frames cannot define a car body up direction.");
            const glm::dvec3 lateral = normalized(
                glm::cross(up, forward),
                "The car body lateral direction could not be constructed.");
            up = normalized(
                glm::cross(forward, lateral),
                "The car body up direction could not be reconstructed.");

            const geometry::CurveFrame frame{forward, lateral, up};
            geometry::detail::validateCurveFrameForRotation(
                frame, "car body pose construction");
            return frame;
        }

        [[nodiscard]] glm::dvec3 transformPoint(
            const glm::dvec3& origin,
            const geometry::CurveFrame& frame,
            const glm::dvec3& localPoint) noexcept
        {
            return origin
                + localPoint.x * frame.tangent
                + localPoint.y * frame.lateral
                + localPoint.z * frame.up;
        }

        struct SampledBogie
        {
            std::size_t definitionIndex = 0;
            TrackLocation location;
            glm::dvec3 positionMeters{0.0};
            geometry::CurveFrame trackFrame;
            geometry::CurveFrame orientedFrame;
        };

        [[nodiscard]] SampledBogie sampleBogie(
            const CompiledPhysicsTrack& track,
            const TrackLocation& referenceLocation,
            const BogieDefinition& bogie,
            const std::size_t definitionIndex,
            const double travelSign)
        {
            TrackLocation location = track.advance(
                referenceLocation,
                travelSign * bogie.referencePositionMeters.x).location;

            // advance() records the direction of its signed displacement. A
            // pose offset is not motion, so retain the car's travel direction.
            location.direction = referenceLocation.direction;
            const PhysicsTrackSample sample = track.sample(location);
            return {
                definitionIndex,
                location,
                sample.positionMeters,
                sample.frame,
                orientedTrackFrame(sample.frame, referenceLocation.direction)
            };
        }

        struct SolvedCarGeometry
        {
            SampledBogie front;
            SampledBogie rear;
            geometry::CurveFrame bodyFrame;
            glm::dvec3 bodyPositionMeters{0.0};
            glm::dvec3 frontHitchPositionMeters{0.0};
            glm::dvec3 rearHitchPositionMeters{0.0};
        };

        void requireFinitePoseValue(
            const glm::dvec3& value,
            const char* errorMessage);

        [[nodiscard]] SolvedCarGeometry solveCarGeometry(
            const CompiledPhysicsTrack& track,
            const CarDefinition& definition,
            const TrackLocation& referenceLocation)
        {
            if (definition.bogies.size() != 2)
            {
                throw std::invalid_argument(
                    "The Phase 2 car pose solver supports exactly two bogies.");
            }

            const BogieDefinition* const bogies = definition.bogies.data();
            const double firstX = bogies[0].referencePositionMeters.x;
            const double secondX = bogies[1].referencePositionMeters.x;
            if (std::abs(firstX - secondX) <= minimumBogieSeparationMeters)
            {
                throw std::invalid_argument(
                    "The two bogies must have distinct longitudinal reference positions.");
            }

            const std::size_t frontIndex = firstX > secondX ? 0 : 1;
            const std::size_t rearIndex = frontIndex == 0 ? 1 : 0;
            const BogieDefinition& frontDefinition = bogies[frontIndex];
            const BogieDefinition& rearDefinition = bogies[rearIndex];
            const double travelSign = directionSign(referenceLocation.direction);

            SolvedCarGeometry result;
            result.front = sampleBogie(track,
                referenceLocation, frontDefinition, frontIndex, travelSign);
            result.rear = sampleBogie(track,
                referenceLocation, rearDefinition, rearIndex, travelSign);
            result.bodyFrame = bodyFrameFromBogies(
                result.rear.positionMeters,
                result.front.positionMeters,
                result.rear.orientedFrame,
                result.front.orientedFrame);

            const glm::dvec3 localBogieMidpoint = 0.5
                * (frontDefinition.referencePositionMeters
                    + rearDefinition.referencePositionMeters);
            const glm::dvec3 worldBogieMidpoint = 0.5
                * (result.front.positionMeters + result.rear.positionMeters);
            result.bodyPositionMeters = worldBogieMidpoint
                - localBogieMidpoint.x * result.bodyFrame.tangent
                - localBogieMidpoint.y * result.bodyFrame.lateral
                - localBogieMidpoint.z * result.bodyFrame.up;
            result.frontHitchPositionMeters = transformPoint(
                result.bodyPositionMeters,
                result.bodyFrame,
                definition.frontHitchPositionMeters);
            result.rearHitchPositionMeters = transformPoint(
                result.bodyPositionMeters,
                result.bodyFrame,
                definition.rearHitchPositionMeters);
            requireFinitePoseValue(
                result.bodyPositionMeters,
                "Car body position is non-finite.");
            requireFinitePoseValue(
                result.frontHitchPositionMeters,
                "Car front hitch world position is non-finite.");
            requireFinitePoseValue(
                result.rearHitchPositionMeters,
                "Car rear hitch world position is non-finite.");
            return result;
        }

        [[nodiscard]] double relativeYaw(
            const geometry::CurveFrame& bodyFrame,
            const geometry::CurveFrame& bogieFrame) noexcept
        {
            const double forward = glm::dot(
                bogieFrame.tangent, bodyFrame.tangent);
            const double lateral = glm::dot(
                bogieFrame.tangent, bodyFrame.lateral);
            return std::atan2(lateral, forward);
        }

        void requireFinitePoseValue(
            const glm::dvec3& value,
            const char* const errorMessage)
        {
            if (!finite(value))
            {
                throw std::domain_error(errorMessage);
            }
        }

        [[nodiscard]] double totalCarMassKilogramsUnchecked(
            const CarDefinition& definition,
            const CarLoadout& loadout)
        {
            const double total = definition.dryMassKilograms
                + loadout.massKilograms;
            if (!std::isfinite(total) || total <= 0.0)
            {
                throw std::invalid_argument(
                    "Car dry and load masses must produce a positive finite total mass.");
            }
            return total;
        }

        [[nodiscard]] glm::dvec3 loadedCarCenterOfGravityMetersUnchecked(
            const CarDefinition& definition,
            const CarLoadout& loadout)
        {
            const double totalMass = totalCarMassKilogramsUnchecked(
                definition, loadout);
            if (loadout.massKilograms == 0.0)
            {
                return definition.dryCenterOfGravityMeters;
            }

            const glm::dvec3 centerOfGravity =
                (definition.dryMassKilograms
                    * definition.dryCenterOfGravityMeters
                 + loadout.massKilograms * loadout.centerOfMassMeters)
                / totalMass;
            if (!finite(centerOfGravity))
            {
                throw std::invalid_argument(
                    "Car mass properties produce a non-finite loaded center of gravity.");
            }
            return centerOfGravity;
        }
    }

    void validateBogieDefinition(const BogieDefinition& definition)
    {
        if (!finite(definition.referencePositionMeters))
        {
            throw std::invalid_argument(
                "Bogie reference position must be finite and expressed in metres.");
        }
        if (definition.contacts.size() > maximumBogieContactCount)
        {
            throw std::invalid_argument(
                "A bogie physics definition contains too many contacts.");
        }
        for (const BogieContactDefinition& contact : definition.contacts)
        {
            if (!validContactRole(contact.role))
            {
                throw std::invalid_argument(
                    "Bogie contact role is invalid.");
            }
            if (!finite(contact.localPositionMeters))
            {
                throw std::invalid_argument(
                    "Bogie contact position must be finite and expressed in bogie-local metres.");
            }
            if (!finite(contact.contactNormalLocal))
            {
                throw std::invalid_argument(
                    "Bogie contact normal must be finite.");
            }
            const double normalLength = glm::length(
                contact.contactNormalLocal);
            if (!std::isfinite(normalLength) || normalLength <= directionalResolution)
            {
                throw std::invalid_argument(
                    "Bogie contact normal must be nonzero.");
            }
            if (std::abs(normalLength - 1.0)
                > bogieContactNormalUnitTolerance)
            {
                throw std::invalid_argument(
                    "Bogie contact normal must be unit length.");
            }
            if (std::abs(contact.contactNormalLocal.x)
                > bogieContactTangentComponentTolerance)
            {
                throw std::invalid_argument(
                    "Frictionless bogie contact normals cannot contain a rolling-tangent component.");
            }
        }
    }

    void validateCarDefinition(const CarDefinition& definition)
    {
        if (!std::isfinite(definition.dryMassKilograms)
            || definition.dryMassKilograms <= 0.0)
        {
            throw std::invalid_argument(
                "Car dry mass must be positive and finite.");
        }
        if (!finite(definition.bodyDimensionsMeters)
            || definition.bodyDimensionsMeters.x <= 0.0
            || definition.bodyDimensionsMeters.y <= 0.0
            || definition.bodyDimensionsMeters.z <= 0.0)
        {
            throw std::invalid_argument(
                "Car body dimensions must be positive, finite, and expressed in metres.");
        }
        if (!finite(definition.dryCenterOfGravityMeters))
        {
            throw std::invalid_argument(
                "Car dry center of gravity must be finite.");
        }
        validateInertiaTensor(definition.dryInertiaTensorBodyKgM2);
        if (!finite(definition.frontHitchPositionMeters)
            || !finite(definition.rearHitchPositionMeters))
        {
            throw std::invalid_argument(
                "Car hitch positions must be finite.");
        }
        if (!std::isfinite(definition.aerodynamicDragAreaSquareMeters)
            || definition.aerodynamicDragAreaSquareMeters < 0.0)
        {
            throw std::invalid_argument(
                "Car aerodynamic CdA must be finite and non-negative.");
        }
        if (!finite(definition.aerodynamicCenterLocalMeters))
        {
            throw std::invalid_argument(
                "Car aerodynamic center must be finite and expressed in car-local metres.");
        }
        if (definition.bogies.empty())
        {
            throw std::invalid_argument(
                "A car definition must contain at least one bogie.");
        }
        for (const BogieDefinition& bogie : definition.bogies)
        {
            validateBogieDefinition(bogie);
        }
    }

    void validateCarLoadout(const CarLoadout& loadout)
    {
        if (!std::isfinite(loadout.massKilograms)
            || loadout.massKilograms < 0.0)
        {
            throw std::invalid_argument(
                "Car load mass must be finite and non-negative.");
        }
        if (!finite(loadout.centerOfMassMeters))
        {
            throw std::invalid_argument(
                "Car load center of mass must be finite.");
        }
    }

    double totalCarMassKilograms(
        const CarDefinition& definition,
        const CarLoadout& loadout)
    {
        validateCarDefinition(definition);
        validateCarLoadout(loadout);
        return totalCarMassKilogramsUnchecked(definition, loadout);
    }

    glm::dvec3 loadedCarCenterOfGravityMeters(
        const CarDefinition& definition,
        const CarLoadout& loadout)
    {
        validateCarDefinition(definition);
        validateCarLoadout(loadout);
        return loadedCarCenterOfGravityMetersUnchecked(definition, loadout);
    }

    glm::dmat3 makeUniformBoxInertiaTensorBodyKgM2(
        const double massKilograms,
        const glm::dvec3& dimensionsMeters)
    {
        if (!std::isfinite(massKilograms) || massKilograms <= 0.0
            || !finite(dimensionsMeters)
            || dimensionsMeters.x <= 0.0
            || dimensionsMeters.y <= 0.0
            || dimensionsMeters.z <= 0.0)
        {
            throw std::invalid_argument(
                "Uniform-box inertia inputs must be positive finite SI values.");
        }
        const double factor = massKilograms / 12.0;
        return glm::dmat3{
            factor * (dimensionsMeters.y * dimensionsMeters.y
                + dimensionsMeters.z * dimensionsMeters.z), 0.0, 0.0,
            0.0, factor * (dimensionsMeters.x * dimensionsMeters.x
                + dimensionsMeters.z * dimensionsMeters.z), 0.0,
            0.0, 0.0, factor * (
                dimensionsMeters.x * dimensionsMeters.x
                + dimensionsMeters.y * dimensionsMeters.y)
        };
    }

    glm::dmat3 loadedCarInertiaTensorBodyKgM2(
        const CarDefinition& definition,
        const CarLoadout& loadout)
    {
        validateCarDefinition(definition);
        validateCarLoadout(loadout);
        const glm::dvec3 loadedCenter = loadedCarCenterOfGravityMetersUnchecked(
            definition, loadout);
        glm::dmat3 result = definition.dryInertiaTensorBodyKgM2
            + parallelAxisShift(
                definition.dryMassKilograms,
                definition.dryCenterOfGravityMeters - loadedCenter);
        if (loadout.massKilograms > 0.0)
        {
            result += parallelAxisShift(
                loadout.massKilograms,
                loadout.centerOfMassMeters - loadedCenter);
        }
        if (!finite(result))
        {
            throw std::domain_error(
                "Loaded car inertia tensor is non-finite.");
        }
        return result;
    }

    BogiePose::BogiePose(
        const std::size_t definitionIndex,
        TrackLocation location,
        glm::dvec3 worldPositionMeters,
        geometry::CurveFrame trackFrame,
        geometry::CurveFrame orientedFrame,
        glm::dquat bodyRelativeOrientation,
        const double bodyRelativeYawRadians)
        : definitionIndex_(definitionIndex),
          location_(location),
          worldPositionMeters_(worldPositionMeters),
          trackFrame_(trackFrame),
          orientedFrame_(orientedFrame),
          bodyRelativeOrientation_(bodyRelativeOrientation),
          bodyRelativeYawRadians_(bodyRelativeYawRadians)
    {
    }

    std::size_t BogiePose::definitionIndex() const noexcept
    {
        return definitionIndex_;
    }

    const TrackLocation& BogiePose::location() const noexcept
    {
        return location_;
    }

    const glm::dvec3& BogiePose::worldPositionMeters() const noexcept
    {
        return worldPositionMeters_;
    }

    const geometry::CurveFrame& BogiePose::trackFrame() const noexcept
    {
        return trackFrame_;
    }

    const geometry::CurveFrame& BogiePose::orientedFrame() const noexcept
    {
        return orientedFrame_;
    }

    const glm::dquat& BogiePose::bodyRelativeOrientation() const noexcept
    {
        return bodyRelativeOrientation_;
    }

    double BogiePose::bodyRelativeYawRadians() const noexcept
    {
        return bodyRelativeYawRadians_;
    }

    glm::dvec3 BogiePose::transformLocalPoint(
        const glm::dvec3& localPointMeters) const noexcept
    {
        return transformPoint(
            worldPositionMeters_, orientedFrame_, localPointMeters);
    }

    glm::dvec3 BogiePose::transformLocalDirection(
        const glm::dvec3& localDirection) const noexcept
    {
        return localDirection.x * orientedFrame_.tangent
            + localDirection.y * orientedFrame_.lateral
            + localDirection.z * orientedFrame_.up;
    }

    CarPose::CarPose(
        TrackLocation referenceLocation,
        glm::dvec3 bodyWorldPositionMeters,
        geometry::CurveFrame bodyFrame,
        glm::dquat bodyOrientation,
        glm::dvec3 localCenterOfGravityMeters,
        glm::dvec3 worldCenterOfGravityMeters,
        const double totalMassKilograms,
        glm::dvec3 frontHitchWorldPositionMeters,
        glm::dvec3 rearHitchWorldPositionMeters,
        std::array<BogiePose, 2> bogies)
        : referenceLocation_(referenceLocation),
          bodyWorldPositionMeters_(bodyWorldPositionMeters),
          bodyFrame_(bodyFrame),
          bodyOrientation_(bodyOrientation),
          localCenterOfGravityMeters_(localCenterOfGravityMeters),
          worldCenterOfGravityMeters_(worldCenterOfGravityMeters),
          totalMassKilograms_(totalMassKilograms),
          frontHitchWorldPositionMeters_(frontHitchWorldPositionMeters),
          rearHitchWorldPositionMeters_(rearHitchWorldPositionMeters),
          bogies_(std::move(bogies))
    {
    }

    const TrackLocation& CarPose::referenceLocation() const noexcept
    {
        return referenceLocation_;
    }

    const glm::dvec3& CarPose::bodyWorldPositionMeters() const noexcept
    {
        return bodyWorldPositionMeters_;
    }

    const geometry::CurveFrame& CarPose::bodyFrame() const noexcept
    {
        return bodyFrame_;
    }

    const glm::dquat& CarPose::bodyOrientation() const noexcept
    {
        return bodyOrientation_;
    }

    const glm::dvec3& CarPose::localCenterOfGravityMeters() const noexcept
    {
        return localCenterOfGravityMeters_;
    }

    const glm::dvec3& CarPose::worldCenterOfGravityMeters() const noexcept
    {
        return worldCenterOfGravityMeters_;
    }

    double CarPose::totalMassKilograms() const noexcept
    {
        return totalMassKilograms_;
    }

    const glm::dvec3& CarPose::frontHitchWorldPositionMeters() const noexcept
    {
        return frontHitchWorldPositionMeters_;
    }

    const glm::dvec3& CarPose::rearHitchWorldPositionMeters() const noexcept
    {
        return rearHitchWorldPositionMeters_;
    }

    const BogiePose& CarPose::frontBogie() const noexcept
    {
        return bogies_[0];
    }

    const BogiePose& CarPose::rearBogie() const noexcept
    {
        return bogies_[1];
    }

    glm::dvec3 CarPose::transformLocalPoint(
        const glm::dvec3& localPointMeters) const noexcept
    {
        return transformPoint(
            bodyWorldPositionMeters_, bodyFrame_, localPointMeters);
    }

    CarPose detail::solveCarPoseForValidatedDefinition(
        const CompiledPhysicsTrack& track,
        const CarDefinition& definition,
        const TrackLocation& referenceLocation,
        const CarLoadout& loadout)
    {
        const SolvedCarGeometry geometry = solveCarGeometry(
            track, definition, referenceLocation);
        const glm::dquat bodyOrientation = orientationFromFrame(
            geometry.bodyFrame);

        const double totalMass = totalCarMassKilogramsUnchecked(
            definition, loadout);
        const glm::dvec3 localCenterOfGravity =
            loadedCarCenterOfGravityMetersUnchecked(definition, loadout);
        const glm::dvec3 worldCenterOfGravity = transformPoint(
            geometry.bodyPositionMeters,
            geometry.bodyFrame,
            localCenterOfGravity);
        requireFinitePoseValue(
            worldCenterOfGravity,
            "Car world center of gravity is non-finite.");

        const auto makePose = [&geometry, &bodyOrientation](
            const SampledBogie& sampled)
        {
            const glm::dquat relativeOrientation = canonicalized(
                glm::conjugate(bodyOrientation)
                    * orientationFromFrame(sampled.orientedFrame));
            const double yaw = relativeYaw(
                geometry.bodyFrame, sampled.orientedFrame);
            if (!std::isfinite(yaw))
            {
                throw std::domain_error(
                    "Bogie articulation produced a non-finite relative yaw.");
            }
            return BogiePose{
                sampled.definitionIndex,
                sampled.location,
                sampled.positionMeters,
                sampled.trackFrame,
                sampled.orientedFrame,
                relativeOrientation,
                yaw
            };
        };

        std::array<BogiePose, 2> bogiePoses{
            makePose(geometry.front),
            makePose(geometry.rear)
        };
        return CarPose{
            referenceLocation,
            geometry.bodyPositionMeters,
            geometry.bodyFrame,
            bodyOrientation,
            localCenterOfGravity,
            worldCenterOfGravity,
            totalMass,
            geometry.frontHitchPositionMeters,
            geometry.rearHitchPositionMeters,
            std::move(bogiePoses)
        };
    }

    glm::dvec3 detail::solveFrontHitchPositionForValidatedDefinition(
        const CompiledPhysicsTrack& track,
        const CarDefinition& definition,
        const TrackLocation& referenceLocation)
    {
        return solveCarGeometry(track, definition, referenceLocation)
            .frontHitchPositionMeters;
    }

    CarPose solveCarPose(
        const CompiledPhysicsTrack& track,
        const CarDefinition& definition,
        const TrackLocation& referenceLocation,
        const CarLoadout& loadout)
    {
        validateCarDefinition(definition);
        validateCarLoadout(loadout);
        return detail::solveCarPoseForValidatedDefinition(
            track, definition, referenceLocation, loadout);
    }

    glm::dmat3 worldCarInertiaTensorKgM2(
        const CarPose& pose,
        const glm::dmat3& inertiaTensorBodyKgM2)
    {
        if (!finite(inertiaTensorBodyKgM2))
        {
            throw std::invalid_argument(
                "Body inertia tensor must be finite before world transformation.");
        }
        const geometry::CurveFrame& frame = pose.bodyFrame();
        const glm::dmat3 bodyToWorld{
            frame.tangent,
            frame.lateral,
            frame.up
        };
        const glm::dmat3 result = bodyToWorld
            * inertiaTensorBodyKgM2 * glm::transpose(bodyToWorld);
        if (!finite(result))
        {
            throw std::domain_error(
                "World inertia tensor transformation is non-finite.");
        }
        return result;
    }
}
