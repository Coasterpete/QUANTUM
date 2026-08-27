#pragma once

#include <quantum/coaster/RiderLocalGeometry.hpp>

#include <cstddef>
#include <vector>

namespace quantum::coaster::detail
{
    // One authored output boundary in the explicit coupled-integration walk.
    // completedSubstepCount is the exclusive end index in substepLengths.
    struct CoupledIntegrationOutputBoundary
    {
        std::size_t completedSubstepCount;
        double distance;
    };

    // Records the exact substep lengths and output boundaries selected by the
    // nominal coupled integrator. internalPanelCount preserves the global
    // spacing regime from which the per-output subdivisions were selected.
    struct CoupledIntegrationSchedule
    {
        double profileLength;
        std::size_t internalPanelCount;
        std::vector<double> substepLengths;
        std::vector<CoupledIntegrationOutputBoundary> outputBoundaries;
    };

    [[nodiscard]] CoupledIntegrationSchedule
    makeCoupledIntegrationSchedule(
        const math::ScalarTransition* rollRateTransition,
        const math::ScalarTransition& pitchRateTransition,
        const math::ScalarTransition& yawRateTransition,
        double integrationSpacing
    );

    // These detail entry points deliberately bypass specialized zero,
    // constant, and single-axis dispatch. Callers must supply already-
    // validated profiles with the same domain used to create schedule.
    [[nodiscard]] std::vector<RiderLocalGeometryState>
    integrateCoupledRateProfilesNumerically(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const math::ScalarTransition* rollRateTransition,
        const math::ScalarTransition& pitchRateTransition,
        const math::ScalarTransition& yawRateTransition,
        const CoupledIntegrationSchedule& schedule
    );

    [[nodiscard]] RiderLocalGeometryState
    integrateCoupledRateProfilesEndpoint(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const math::ScalarTransition* rollRateTransition,
        const math::ScalarTransition& pitchRateTransition,
        const math::ScalarTransition& yawRateTransition,
        const CoupledIntegrationSchedule& schedule
    );
}
