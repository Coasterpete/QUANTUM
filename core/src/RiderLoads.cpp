#include <quantum/coaster/RiderLoads.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
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
        if (!std::isfinite(settings.initialSpeed)
            || settings.initialSpeed < 0.0)
        {
            throw std::invalid_argument(
                "Initial rider-load speed must be finite and non-negative."
            );
        }

        if (!std::isfinite(settings.metersPerCoordinateUnit)
            || settings.metersPerCoordinateUnit <= 0.0)
        {
            throw std::invalid_argument(
                "Rider-load physical scale must be positive and finite."
            );
        }

        if (!std::isfinite(settings.gravityAcceleration)
            || settings.gravityAcceleration <= 0.0)
        {
            throw std::invalid_argument(
                "Rider-load gravity acceleration must be positive and "
                "finite."
            );
        }

        validateKinematics(kinematics);

        RiderLoadHistory history;
        history.states.reserve(kinematics.size());

        const glm::dvec3 gravity{
            0.0, 0.0, -settings.gravityAcceleration};
        const glm::dvec3 startingPhysicalPosition =
            settings.metersPerCoordinateUnit * kinematics.front().position;
        const double initialSpeedSquared =
            settings.initialSpeed * settings.initialSpeed;

        for (const TrackKinematicState& kinematic : kinematics)
        {
            const glm::dvec3 physicalPosition =
                settings.metersPerCoordinateUnit * kinematic.position;
            double speedSquared = initialSpeedSquared
                + 2.0 * glm::dot(
                    gravity,
                    physicalPosition - startingPhysicalPosition);

            // Energy cancellation near a turning point can leave a tiny
            // negative result. Only a scale-aware multiple of machine
            // epsilon is treated as roundoff; larger negatives are a real
            // unreachable-track result and terminate the history.
            const double energyScale = std::max({
                1.0,
                initialSpeedSquared,
                2.0 * settings.gravityAcceleration
                    * std::abs(physicalPosition.z
                        - startingPhysicalPosition.z)
            });
            const double negativeTolerance =
                64.0 * std::numeric_limits<double>::epsilon() * energyScale;

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
