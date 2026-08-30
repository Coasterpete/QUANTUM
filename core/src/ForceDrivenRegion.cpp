#include <quantum/coaster/ForceDrivenRegion.hpp>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace quantum::coaster
{
    TrackGenerationError::TrackGenerationError(TrackGenerationFailure failure)
        : std::runtime_error(failure.message), failure_(std::move(failure))
    {
    }

    const TrackGenerationFailure& TrackGenerationError::failure() const noexcept
    {
        return failure_;
    }

    void validateForceDrivenRegion(const ForceDrivenRegion& region, const double length)
    {
        for (const ChannelProfile* channel :
            {&region.targetNormalG, &region.targetLateralG, &region.rollRate})
        {
            validateChannelProfile(*channel, length);
            for (const ProfileSegment& segment : channel->segments)
            {
                if (channel->nextSegmentId <= segment.id)
                {
                    throw std::invalid_argument(
                        "Force profile nextSegmentId must exceed every segment ID.");
                }
            }
        }
    }

    namespace
    {
        bool finite(const glm::dvec3& v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        struct Pose
        {
            // Displacement from section entry keeps error control independent
            // of world translation. There is no integrated energy variable.
            glm::dvec3 displacement;
            glm::dquat orientation;
        };

        struct Derivative
        {
            glm::dvec3 tangent;
            glm::dquat orientation;
            glm::dvec3 rates; // roll, pitch, yaw in radians / Core unit
            double distance;
            double speedSquared;
        };

        [[noreturn]] void fail(
            const TrackGenerationFailureReason reason,
            const double distance,
            const std::optional<double> speedSquared,
            const char* message)
        {
            throw TrackGenerationError({reason, std::nullopt, distance,
                std::nullopt, speedSquared, message});
        }

        glm::dquat normalized(const glm::dquat& q, const double distance)
        {
            const double magnitude = glm::length(q);
            if (!std::isfinite(magnitude) || magnitude == 0.0)
            {
                fail(TrackGenerationFailureReason::IntegrationFailure,
                    distance, std::nullopt, "Force integration produced an invalid orientation.");
            }
            return q / magnitude;
        }

        geometry::CurveFrame frameOf(const glm::dquat& q)
        {
            return {q * glm::dvec3{1, 0, 0}, q * glm::dvec3{0, 1, 0},
                q * glm::dvec3{0, 0, 1}};
        }

        Pose advance(const Pose& pose, const Derivative& derivative, const double h)
        {
            return {pose.displacement + h * derivative.tangent,
                pose.orientation + h * derivative.orientation};
        }
    }

    std::vector<TrackKinematicState> integrateForceDrivenRegion(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const ForceDrivenRegion& region,
        const double length,
        const TrackPhysicalSettings& physicalSettings,
        const glm::dvec3& wholeTrackStartPosition,
        const double integrationSpacing,
        const ForceDrivenIntegrationSettings& settings)
    {
        validateForceDrivenRegion(region, length);
        validateTrackPhysicalSettings(physicalSettings);
        if (!std::isfinite(integrationSpacing) || integrationSpacing <= 0.0
            || !std::isfinite(settings.tolerance) || settings.tolerance <= 0.0
            || settings.maximumRefinements > 50
            || !finite(startingPosition) || !finite(wholeTrackStartPosition)
            || !finite(startingFrame.tangent) || !finite(startingFrame.lateral)
            || !finite(startingFrame.up)
            || std::abs(glm::length(startingFrame.tangent) - 1.0) > 1.0e-9
            || std::abs(glm::length(startingFrame.lateral) - 1.0) > 1.0e-9
            || std::abs(glm::dot(startingFrame.tangent, startingFrame.lateral)) > 1.0e-9
            || glm::length(glm::cross(startingFrame.tangent, startingFrame.lateral)
                - startingFrame.up) > 1.0e-9)
        {
            throw std::invalid_argument("Invalid force integration settings or entry pose.");
        }

        const glm::dvec3 gravity{0.0, 0.0, -physicalSettings.gravityAcceleration};
        const auto derivative = [&](const Pose& pose, const double s) -> Derivative
        {
            const glm::dvec3 worldPosition = startingPosition + pose.displacement;
            if (!finite(pose.displacement) || !finite(worldPosition))
            {
                fail(TrackGenerationFailureReason::IntegrationFailure, s,
                    std::nullopt, "Force integration produced a nonfinite position.");
            }
            const glm::dquat q = normalized(pose.orientation, s);
            const geometry::CurveFrame frame = frameOf(q);
            const detail::TrackEnergy energy = detail::trackEnergyAtPosition(
                physicalSettings, wholeTrackStartPosition, worldPosition);
            if (energy.speedSquared < -energy.tolerance)
            {
                fail(TrackGenerationFailureReason::EnergeticallyUnreachable, s,
                    energy.speedSquared, "Force generation encountered an unreachable energy barrier.");
            }
            if (energy.speedSquared <= energy.tolerance)
            {
                fail(TrackGenerationFailureReason::InsufficientSpeed, s,
                    energy.speedSquared, "Force generation requires numerically resolved positive speed squared.");
            }

            const double normal = evaluateChannelProfile(region.targetNormalG, s);
            const double lateral = evaluateChannelProfile(region.targetLateralG, s);
            const double roll = evaluateChannelProfile(region.rollRate, s);
            const double yaw = physicalSettings.metersPerCoordinateUnit
                * (standardGravityAcceleration * lateral + glm::dot(gravity, frame.lateral))
                / energy.speedSquared;
            const double pitch = -physicalSettings.metersPerCoordinateUnit
                * (standardGravityAcceleration * normal + glm::dot(gravity, frame.up))
                / energy.speedSquared;
            const glm::dvec3 rates{roll, pitch, yaw};
            if (!finite(rates) || !std::isfinite(std::hypot(roll, pitch, yaw)))
            {
                fail(TrackGenerationFailureReason::NonfiniteDerivedRates, s,
                    energy.speedSquared, "Force-to-rate conversion produced nonfinite rates.");
            }
            // Local angular velocity (r,p,y) gives T'=yL-pU, L'=-yT+rU.
            return {frame.tangent, 0.5 * (q * glm::dquat{0.0, roll, pitch, yaw}),
                rates, s, energy.speedSquared};
        };

        const auto rk4 = [&](const Pose& pose, const double begin, const double end)
        {
            const double h = end - begin;
            const double middle = begin + 0.5 * h;
            const Derivative k1 = derivative(pose, begin);
            const Derivative k2 = derivative(advance(pose, k1, 0.5 * h), middle);
            const Derivative k3 = derivative(advance(pose, k2, 0.5 * h), middle);
            const Derivative k4 = derivative(advance(pose, k3, h), end);
            // A rotation bound prevents large aliased steps from missing an
            // interior turn/barrier even if endpoint step-doubling agrees.
            for (const Derivative* k : {&k1, &k2, &k3, &k4})
            {
                if (h * std::hypot(k->rates.x, k->rates.y, k->rates.z) > 0.25)
                {
                    fail(TrackGenerationFailureReason::IntegrationFailure,
                        k->distance, k->speedSquared, "Force integration requires angular refinement.");
                }
            }
            Pose result{
                pose.displacement + (h / 6.0)
                    * (k1.tangent + 2.0 * k2.tangent + 2.0 * k3.tangent + k4.tangent),
                normalized(pose.orientation + (h / 6.0)
                    * (k1.orientation + 2.0 * k2.orientation + 2.0 * k3.orientation
                        + k4.orientation), end)};
            static_cast<void>(derivative(result, end));
            return result;
        };

        std::size_t attempts = 0;
        const auto integrateInterval = [&](auto&& self, const Pose& pose,
            const double begin, const double end, const std::size_t depth) -> Pose
        {
            if (++attempts > 1000000)
            {
                fail(TrackGenerationFailureReason::IntegrationFailure, begin,
                    detail::trackEnergyAtPosition(physicalSettings, wholeTrackStartPosition,
                        startingPosition + pose.displacement).speedSquared,
                    "Force integration exceeded its work limit.");
            }
            const double middle = begin + 0.5 * (end - begin);
            if (middle == begin || middle == end)
            {
                fail(TrackGenerationFailureReason::IntegrationFailure, begin,
                    detail::trackEnergyAtPosition(physicalSettings, wholeTrackStartPosition,
                        startingPosition + pose.displacement).speedSquared,
                    "Force integration distance refinement is not representable.");
            }
            std::optional<TrackGenerationFailure> stageFailure;
            try
            {
                const Pose coarse = rk4(pose, begin, end);
                const Pose half = rk4(pose, begin, middle);
                const Pose fine = rk4(half, middle, end);
                const double positionError = glm::length(fine.displacement - coarse.displacement) / length;
                const double frameError = 2.0 * std::min(
                    glm::length(fine.orientation - coarse.orientation),
                    glm::length(fine.orientation + coarse.orientation));
                const double error = std::max(positionError, frameError) / 15.0;
                if (error <= settings.tolerance * ((end - begin) / length))
                {
                    return fine;
                }
            }
            catch (const TrackGenerationError& error)
            {
                // A provisional stage can overestimate height. Refine the
                // same interval before declaring a physical failure.
                stageFailure = error.failure();
            }
            if (depth == settings.maximumRefinements)
            {
                if (stageFailure)
                {
                    throw TrackGenerationError(*stageFailure);
                }
                fail(TrackGenerationFailureReason::IntegrationFailure, begin,
                    detail::trackEnergyAtPosition(physicalSettings, wholeTrackStartPosition,
                        startingPosition + pose.displacement).speedSquared,
                    "Force integration did not meet its error tolerance.");
            }
            const Pose half = self(self, pose, begin, middle, depth + 1);
            return self(self, half, middle, end, depth + 1);
        };

        // Breakpoints constrain integration intervals only. Each channel is
        // always queried in its original domain, including staggered easing.
        const double outputCount = std::ceil(length / integrationSpacing);
        if (!std::isfinite(outputCount) || outputCount > 1000000)
        {
            fail(TrackGenerationFailureReason::IntegrationFailure, 0.0,
                detail::trackEnergyAtPosition(physicalSettings, wholeTrackStartPosition,
                    startingPosition).speedSquared,
                "Force integration requires too many output samples.");
        }
        std::vector<double> boundaries{0.0, length};
        for (std::size_t i = 1; i < static_cast<std::size_t>(outputCount); ++i)
        {
            boundaries.push_back(static_cast<double>(i) * integrationSpacing);
        }
        for (const ChannelProfile* channel :
            {&region.targetNormalG, &region.targetLateralG, &region.rollRate})
        {
            for (const ProfileSegment& segment : channel->segments)
            {
                boundaries.push_back(segment.transition.domainEnd);
            }
        }
        std::sort(boundaries.begin(), boundaries.end());
        boundaries.erase(std::unique(boundaries.begin(), boundaries.end()), boundaries.end());

        Pose pose{{0.0, 0.0, 0.0}, normalized(glm::quat_cast(glm::dmat3{
            startingFrame.tangent, startingFrame.lateral, startingFrame.up}), 0.0)};
        std::vector<TrackKinematicState> states;
        states.reserve(boundaries.size());
        for (std::size_t i = 0; i < boundaries.size(); ++i)
        {
            const double s = boundaries[i];
            if (i != 0)
            {
                pose = integrateInterval(integrateInterval, pose, boundaries[i - 1], s, 0);
            }
            const Derivative d = derivative(pose, s);
            const geometry::CurveFrame frame = i == 0 ? startingFrame : frameOf(pose.orientation);
            states.push_back({s, startingPosition + pose.displacement, frame,
                d.rates.z * frame.lateral - d.rates.y * frame.up});
        }
        return states;
    }
}
