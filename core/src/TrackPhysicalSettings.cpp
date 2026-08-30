#include <quantum/coaster/TrackPhysicalSettings.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace quantum::coaster
{
    void validateTrackPhysicalSettings(const TrackPhysicalSettings& settings)
    {
        if (!std::isfinite(settings.initialSpeed) || settings.initialSpeed < 0.0
            || !std::isfinite(settings.initialSpeed * settings.initialSpeed))
        {
            throw std::invalid_argument(
                "Track initial speed must be finite, non-negative, and have representable squared speed.");
        }
        if (!std::isfinite(settings.metersPerCoordinateUnit)
            || settings.metersPerCoordinateUnit <= 0.0)
        {
            throw std::invalid_argument("Track physical scale must be positive and finite.");
        }
        if (!std::isfinite(settings.gravityAcceleration)
            || settings.gravityAcceleration <= 0.0)
        {
            throw std::invalid_argument("Track gravity acceleration must be positive and finite.");
        }
    }

    detail::TrackEnergy detail::trackEnergyAtPosition(
        const TrackPhysicalSettings& settings,
        const glm::dvec3& wholeTrackStartPosition,
        const glm::dvec3& position)
    {
        const double initialSpeedSquared = settings.initialSpeed * settings.initialSpeed;
        const double physicalHeightChange = settings.metersPerCoordinateUnit
            * (position.z - wholeTrackStartPosition.z);
        const double gravityWork = 2.0 * (-settings.gravityAcceleration * physicalHeightChange);
        const double energyScale = std::max({1.0, initialSpeedSquared, std::abs(gravityWork)});
        const TrackEnergy result{
            initialSpeedSquared + gravityWork,
            64.0 * std::numeric_limits<double>::epsilon() * energyScale};
        if (!std::isfinite(result.speedSquared) || !std::isfinite(result.tolerance))
        {
            throw std::invalid_argument("Track physical energy is not finite.");
        }
        return result;
    }
}
