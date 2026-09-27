#include <quantum/coaster/RiderLoads.hpp>

#include <glm/geometric.hpp>

#include <cmath>
#include <stdexcept>

namespace quantum::coaster
{
    namespace
    {
        [[nodiscard]] bool finite(const glm::dvec3& value) noexcept
        {
            return std::isfinite(value.x)
                && std::isfinite(value.y)
                && std::isfinite(value.z);
        }

        void validateKinematics(
            const std::span<const TrackKinematicState> kinematics)
        {
            if (kinematics.empty())
            {
                throw std::invalid_argument(
                    "Rider-load evaluation requires canonical kinematics."
                );
            }

            double previousDistance = 0.0;
            for (std::size_t index = 0; index < kinematics.size(); ++index)
            {
                const TrackKinematicState& state = kinematics[index];
                if (!std::isfinite(state.distance)
                    || !finite(state.position)
                    || !finite(state.frame.tangent)
                    || !finite(state.frame.lateral)
                    || !finite(state.frame.up)
                    || !finite(state.centerlineCurvature))
                {
                    throw std::invalid_argument(
                        "Canonical rider-load kinematics must be finite."
                    );
                }

                if ((index == 0 && state.distance != 0.0)
                    || (index != 0 && state.distance <= previousDistance))
                {
                    throw std::invalid_argument(
                        "Canonical rider-load distances must start at zero "
                        "and strictly increase."
                    );
                }

                previousDistance = state.distance;
            }
        }
    }

    RiderLoadHistory evaluateRiderLoads(
        const std::span<const TrackKinematicState> kinematics,
        const RiderLoadEvaluationSettings& settings)
    {
        const TrackPhysicalSettings physicalSettings{
            settings.initialSpeed, settings.metersPerCoordinateUnit,
            settings.gravityAcceleration};
        validateTrackPhysicalSettings(physicalSettings);
        if (!std::isfinite(settings.riderReferenceOffsetMeters)
            || settings.riderReferenceOffsetMeters < 0.0)
        {
            throw std::invalid_argument(
                "Rider-reference offset must be finite and non-negative.");
        }

        validateKinematics(kinematics);

        RiderLoadHistory history;
        history.states.reserve(kinematics.size());

        const glm::dvec3 gravity{
            0.0, 0.0, -settings.gravityAcceleration};
        const double riderOffset = settings.riderReferenceOffsetMeters
            / settings.metersPerCoordinateUnit;
        for (const TrackKinematicState& kinematic : kinematics)
        {
            const detail::TrackEnergy energy = detail::trackEnergyAtPosition(
                physicalSettings, kinematics.front().position, kinematic.position);
            double speedSquared = energy.speedSquared;
            const double negativeTolerance = energy.tolerance;

            if (speedSquared < -negativeTolerance)
            {
                history.unreachable = RiderLoadUnreachableState{
                    kinematic.distance,
                    speedSquared
                };
                break;
            }

            if (speedSquared < 0.0)
            {
                speedSquared = 0.0;
            }

            const double constructionLongitudinalAcceleration =
                glm::dot(gravity, kinematic.frame.tangent);
            const double rollRate = kinematic.localFrameRates.x;
            const double pitchRate = kinematic.localFrameRates.y;
            const double yawRate = kinematic.localFrameRates.z;
            const double rollRateDerivative =
                kinematic.rollRateDerivative;
            const double pitchRateDerivative =
                kinematic.pitchRateDerivative;

            // H(s)=C(s)+hU(s), with s the construction-reference station.
            // The derivatives retain roll-induced lateral motion and the
            // centripetal acceleration of a rider away from the roll axis.
            const glm::dvec3 riderPositionDerivative = riderOffset == 0.0
                ? kinematic.frame.tangent
                : (1.0 + riderOffset * pitchRate)
                    * kinematic.frame.tangent
                    - riderOffset * rollRate * kinematic.frame.lateral;
            const glm::dvec3 riderPositionSecondDerivative = riderOffset == 0.0
                ? kinematic.centerlineCurvature
                : riderOffset * (pitchRateDerivative
                        + rollRate * yawRate) * kinematic.frame.tangent
                    + ((1.0 + riderOffset * pitchRate) * yawRate
                        - riderOffset * rollRateDerivative)
                        * kinematic.frame.lateral
                    - ((1.0 + riderOffset * pitchRate) * pitchRate
                        + riderOffset * rollRate * rollRate)
                        * kinematic.frame.up;
            const glm::dvec3 acceleration =
                constructionLongitudinalAcceleration
                    * riderPositionDerivative
                + speedSquared * riderPositionSecondDerivative
                    / settings.metersPerCoordinateUnit;
            const glm::dvec3 specificForce = acceleration - gravity;

            history.states.push_back(RiderLoadState{
                kinematic.distance,
                std::sqrt(speedSquared)
                    * glm::length(riderPositionDerivative),
                glm::dot(specificForce, kinematic.frame.up)
                    / standardGravityAcceleration,
                glm::dot(specificForce, kinematic.frame.lateral)
                    / standardGravityAcceleration,
                glm::dot(specificForce, kinematic.frame.tangent)
                    / standardGravityAcceleration,
                riderReferencePosition(kinematic, riderOffset)
            });
        }

        return history;
    }
}
