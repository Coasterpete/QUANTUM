#include <quantum/coaster/PlanarArcRegion.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace quantum::coaster
{
    namespace
    {
        // Stored lengths are produced from planarArcLength at creation and
        // rescale time, so agreement is exact up to the floating-point
        // round trip through division and multiplication.
        constexpr double lengthToleranceRelative = 1e-9;
    }

    double planarArcLength(const PlanarArcRegion& region)
    {
        return std::abs(region.sweptAngle) * region.radius;
    }

    void validatePlanarArcRegion(
        const PlanarArcRegion& region,
        const double storedLength)
    {
        if (!std::isfinite(region.radius) || !std::isfinite(region.sweptAngle)
            || !std::isfinite(region.planeTilt)
            || !std::isfinite(region.bankChange))
        {
            throw std::invalid_argument(
                "A planar arc region requires finite parameters."
            );
        }

        if (region.radius <= 0.0)
        {
            throw std::invalid_argument(
                "A planar arc region requires a positive radius."
            );
        }

        if (region.sweptAngle == 0.0)
        {
            throw std::invalid_argument(
                "A planar arc region requires a nonzero swept angle."
            );
        }

        if (!std::isfinite(storedLength))
        {
            throw std::invalid_argument(
                "A planar arc region requires a finite stored length."
            );
        }

        const double constructedLength = planarArcLength(region);
        const double tolerance =
            lengthToleranceRelative * std::max(1.0, std::abs(storedLength));

        if (std::abs(constructedLength - storedLength) > tolerance)
        {
            throw std::invalid_argument(
                "A planar arc region's stored length must equal "
                "|swept angle| * radius."
            );
        }
    }

    PlanarArcCompiledRates compilePlanarArcRates(
        const PlanarArcRegion& region,
        const double length)
    {
        validatePlanarArcRegion(region, length);

        const double sweepSign =
            region.sweptAngle < 0.0 ? -1.0 : 1.0;
        const double curvature = sweepSign / region.radius;

        const double pitchRateValue =
            -curvature * std::sin(region.planeTilt);
        const double yawRateValue = curvature * std::cos(region.planeTilt);

        return PlanarArcCompiledRates{
            math::ScalarTransition{
                .domainBegin = 0.0,
                .domainEnd = length,
                .valueBegin = pitchRateValue,
                .valueEnd = pitchRateValue,
                .transitionType = math::TransitionType::Linear
            },
            math::ScalarTransition{
                .domainBegin = 0.0,
                .domainEnd = length,
                .valueBegin = yawRateValue,
                .valueEnd = yawRateValue,
                .transitionType = math::TransitionType::Linear
            }
        };
    }

    namespace
    {
        // Rodrigues rotation of v about a unit axis; used to superpose the
        // authored bank onto solved frames without touching their tangents.
        [[nodiscard]] glm::dvec3 rotateAboutUnitAxis(
            const glm::dvec3& v,
            const glm::dvec3& unitAxis,
            const double angle)
        {
            return v * std::cos(angle)
                + glm::cross(unitAxis, v) * std::sin(angle)
                + unitAxis * glm::dot(unitAxis, v) * (1.0 - std::cos(angle));
        }

        [[nodiscard]] math::ScalarTransition createZeroRollTransition(
            const double length)
        {
            return math::ScalarTransition{
                .domainBegin = 0.0,
                .domainEnd = length,
                .valueBegin = 0.0,
                .valueEnd = 0.0,
                .transitionType = math::TransitionType::Linear
            };
        }
    }

    std::vector<RiderLocalGeometryState> integratePlanarArcRegion(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const PlanarArcRegion& region,
        const double integrationSpacing)
    {
        const double length = planarArcLength(region);
        const PlanarArcCompiledRates compiled =
            compilePlanarArcRates(region, length);

        std::vector<RiderLocalGeometryState> states =
            integrateLocalRollPitchYawRateProfiles(
                startingPosition,
                startingFrame,
                createZeroRollTransition(length),
                compiled.pitchRate,
                compiled.yawRate,
                integrationSpacing
            );

        if (region.bankChange != 0.0)
        {
            // Bank superposition: psi(d) = bankChange * d / length, applied
            // about each state's own tangent so the centerline and tangent
            // field remain exactly those of the unbanked arc.
            for (RiderLocalGeometryState& state : states)
            {
                const double bankAngle =
                    region.bankChange * state.distance / length;
                state.frame.lateral = rotateAboutUnitAxis(
                    state.frame.lateral, state.frame.tangent, bankAngle);
                state.frame.up = rotateAboutUnitAxis(
                    state.frame.up, state.frame.tangent, bankAngle);
            }
        }

        return states;
    }

    std::vector<TrackKinematicState> integratePlanarArcRegionKinematics(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const PlanarArcRegion& region,
        const double integrationSpacing)
    {
        const std::vector<RiderLocalGeometryState> geometryStates =
            integratePlanarArcRegion(
                startingPosition,
                startingFrame,
                region,
                integrationSpacing
            );

        // The arc's plane normal and signed curvature are fixed by the
        // construction's entry frame. This remains authoritative even when
        // bankChange rotates each returned lateral/up pair about its tangent.
        const glm::dvec3 planeNormal =
            std::cos(region.planeTilt) * startingFrame.up
            - std::sin(region.planeTilt) * startingFrame.lateral;
        const double signedCurvature =
            (region.sweptAngle < 0.0 ? -1.0 : 1.0) / region.radius;

        std::vector<TrackKinematicState> kinematics;
        kinematics.reserve(geometryStates.size());
        for (const RiderLocalGeometryState& state : geometryStates)
        {
            kinematics.push_back(TrackKinematicState{
                state.distance,
                state.position,
                state.frame,
                signedCurvature
                    * glm::cross(planeNormal, state.frame.tangent)
            });
        }

        return kinematics;
    }
}
