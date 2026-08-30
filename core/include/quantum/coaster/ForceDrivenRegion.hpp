#pragma once

#include <quantum/coaster/GeometricSection.hpp>
#include <quantum/coaster/TrackGeneration.hpp>
#include <quantum/coaster/TrackPhysicalSettings.hpp>

namespace quantum::coaster
{
    struct ForceDrivenRegion
    {
        // Dimensionless rider-force targets over [0, section.length].
        ChannelProfile targetNormalG;
        ChannelProfile targetLateralG;
        // Authored roll remains radians per Core coordinate unit.
        ChannelProfile rollRate;
    };

    // Numerical controls, not authored force/physical state. Position error
    // is normalized by section length; orientation error is dimensionless.
    struct ForceDrivenIntegrationSettings
    {
        double tolerance = 1.0e-10;
        std::size_t maximumRefinements = 24;
    };

    void validateForceDrivenRegion(const ForceDrivenRegion& region, double length);

    // Profiles are evaluated at their original local coordinates at every
    // RK stage. No clipped/reconstructed easing or generated rates are stored.
    [[nodiscard]] std::vector<TrackKinematicState> integrateForceDrivenRegion(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const ForceDrivenRegion& region,
        double length,
        const TrackPhysicalSettings& physicalSettings,
        const glm::dvec3& wholeTrackStartPosition,
        double integrationSpacing,
        const ForceDrivenIntegrationSettings& settings = {});
}
