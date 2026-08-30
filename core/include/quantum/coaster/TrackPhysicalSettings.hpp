#pragma once

#include <glm/vec3.hpp>

namespace quantum::coaster
{
    inline constexpr double standardGravityAcceleration = 9.80665;

    // Document inputs. Core distances are unit-neutral; speed and gravity
    // cross the physical boundary in m/s and m/s^2 respectively.
    struct TrackPhysicalSettings
    {
        double initialSpeed = 20.0;
        double metersPerCoordinateUnit = 1.0;
        double gravityAcceleration = standardGravityAcceleration;

        [[nodiscard]] friend bool operator==(
            const TrackPhysicalSettings&, const TrackPhysicalSettings&) = default;
    };

    void validateTrackPhysicalSettings(const TrackPhysicalSettings& settings);

    namespace detail
    {
        struct TrackEnergy
        {
            double speedSquared;
            double tolerance;
        };

        // One global, position-derived energy reference for generation and
        // evaluation. Subtract before scaling to preserve translated tracks.
        [[nodiscard]] TrackEnergy trackEnergyAtPosition(
            const TrackPhysicalSettings& settings,
            const glm::dvec3& wholeTrackStartPosition,
            const glm::dvec3& position);
    }
}
