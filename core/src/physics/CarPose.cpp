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
    }

    void validateBogieDefinition(const BogieDefinition& definition)
    {
        if (!finite(definition.referencePositionMeters))
        {
            throw std::invalid_argument(
                "Bogie reference position must be finite and expressed in metres.");
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
        if (!finite(definition.frontHitchPositionMeters)
            || !finite(definition.rearHitchPositionMeters))
        {
            throw std::invalid_argument(
                "Car hitch positions must be finite.");
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
        const double total = definition.dryMassKilograms
            + loadout.massKilograms;
        if (!std::isfinite(total) || total <= 0.0)
        {
            throw std::invalid_argument(
                "Car dry and load masses must produce a positive finite total mass.");
        }
        return total;
    }

    glm::dvec3 loadedCarCenterOfGravityMeters(
        const CarDefinition& definition,
        const CarLoadout& loadout)
    {
        const double totalMass = totalCarMassKilograms(definition, loadout);
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

    CarPose solveCarPose(
        const CompiledPhysicsTrack& track,
        const CarDefinition& definition,
        const TrackLocation& referenceLocation,
        const CarLoadout& loadout)
    {
        validateCarDefinition(definition);
        validateCarLoadout(loadout);
        if (definition.bogies.size() != 2)
        {
            throw std::invalid_argument(
                "The Phase 2 car pose solver supports exactly two bogies.");
        }

        const double firstX = definition.bogies[0].referencePositionMeters.x;
        const double secondX = definition.bogies[1].referencePositionMeters.x;
        if (std::abs(firstX - secondX) <= minimumBogieSeparationMeters)
        {
            throw std::invalid_argument(
                "The two bogies must have distinct longitudinal reference positions.");
        }

        const std::size_t frontIndex = firstX > secondX ? 0 : 1;
        const std::size_t rearIndex = frontIndex == 0 ? 1 : 0;
        const BogieDefinition& frontDefinition =
            definition.bogies[frontIndex];
        const BogieDefinition& rearDefinition =
            definition.bogies[rearIndex];
        const double travelSign = directionSign(referenceLocation.direction);

        const SampledBogie front = sampleBogie(
            track, referenceLocation, frontDefinition, frontIndex, travelSign);
        const SampledBogie rear = sampleBogie(
            track, referenceLocation, rearDefinition, rearIndex, travelSign);
        const geometry::CurveFrame bodyFrame = bodyFrameFromBogies(
            rear.positionMeters,
            front.positionMeters,
            rear.orientedFrame,
            front.orientedFrame);
        const glm::dquat bodyOrientation = orientationFromFrame(bodyFrame);

        const glm::dvec3 localBogieMidpoint = 0.5
            * (frontDefinition.referencePositionMeters
                + rearDefinition.referencePositionMeters);
        const glm::dvec3 worldBogieMidpoint = 0.5
            * (front.positionMeters + rear.positionMeters);
        const glm::dvec3 bodyPosition = worldBogieMidpoint
            - localBogieMidpoint.x * bodyFrame.tangent
            - localBogieMidpoint.y * bodyFrame.lateral
            - localBogieMidpoint.z * bodyFrame.up;
        requireFinitePoseValue(
            bodyPosition, "Car body position is non-finite.");

        const double totalMass = totalCarMassKilograms(definition, loadout);
        const glm::dvec3 localCenterOfGravity =
            loadedCarCenterOfGravityMeters(definition, loadout);
        const glm::dvec3 worldCenterOfGravity = transformPoint(
            bodyPosition, bodyFrame, localCenterOfGravity);
        const glm::dvec3 frontHitch = transformPoint(
            bodyPosition, bodyFrame, definition.frontHitchPositionMeters);
        const glm::dvec3 rearHitch = transformPoint(
            bodyPosition, bodyFrame, definition.rearHitchPositionMeters);
        requireFinitePoseValue(
            worldCenterOfGravity,
            "Car world center of gravity is non-finite.");
        requireFinitePoseValue(
            frontHitch, "Car front hitch world position is non-finite.");
        requireFinitePoseValue(
            rearHitch, "Car rear hitch world position is non-finite.");

        const auto makePose = [&bodyFrame, &bodyOrientation](
            const SampledBogie& sampled)
        {
            const glm::dquat relativeOrientation = canonicalized(
                glm::conjugate(bodyOrientation)
                    * orientationFromFrame(sampled.orientedFrame));
            const double yaw = relativeYaw(bodyFrame, sampled.orientedFrame);
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
            makePose(front),
            makePose(rear)
        };
        return CarPose{
            referenceLocation,
            bodyPosition,
            bodyFrame,
            bodyOrientation,
            localCenterOfGravity,
            worldCenterOfGravity,
            totalMass,
            frontHitch,
            rearHitch,
            std::move(bogiePoses)
        };
    }
}
