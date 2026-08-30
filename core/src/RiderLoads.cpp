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

        validateKinematics(kinematics);

        RiderLoadHistory history;
        history.states.reserve(kinematics.size());

        const glm::dvec3 gravity{
            0.0, 0.0, -settings.gravityAcceleration};
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

            const double longitudinalAcceleration =
                glm::dot(gravity, kinematic.frame.tangent);
            const glm::dvec3 physicalCurvature =
                kinematic.centerlineCurvature
                / settings.metersPerCoordinateUnit;
            const glm::dvec3 acceleration =
                longitudinalAcceleration * kinematic.frame.tangent
                + speedSquared * physicalCurvature;
            const glm::dvec3 specificForce = acceleration - gravity;

            history.states.push_back(RiderLoadState{
                kinematic.distance,
                std::sqrt(speedSquared),
                glm::dot(specificForce, kinematic.frame.up)
                    / standardGravityAcceleration,
                glm::dot(specificForce, kinematic.frame.lateral)
                    / standardGravityAcceleration,
                glm::dot(specificForce, kinematic.frame.tangent)
                    / standardGravityAcceleration
            });
        }

        return history;
    }
}
