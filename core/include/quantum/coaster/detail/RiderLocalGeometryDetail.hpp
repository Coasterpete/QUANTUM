#pragma once

#include <quantum/coaster/RiderLocalGeometry.hpp>

#include <array>
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

    inline constexpr std::size_t coupledSensitivityParameterCount = 9;

    using CoupledLocalRateDerivatives =
        std::array<glm::dvec3, coupledSensitivityParameterCount>;
    using CoupledLocalRateDerivativeEvaluator = void (*)(
        double profileCoordinate,
        double profileLength,
        CoupledLocalRateDerivatives& derivatives
    ) noexcept;

    // Sensitivities use a world-space infinitesimal rotation zeta satisfying
    // dF/dtheta = [zeta]x F for F = [T L U]. The endpoint distance is
    // cumulative so successive profile spans can be chained directly.
    struct CoupledEndpointSensitivityState
    {
        RiderLocalGeometryState endpoint;
        std::array<glm::dvec3, coupledSensitivityParameterCount> dPosition{};
        std::array<glm::dvec3, coupledSensitivityParameterCount>
            rotationSensitivity{};
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

    // Applies the SO(3) left Jacobian J_l(rotationVector) without forming a
    // matrix. A cancellation-free series is used around zero.
    [[nodiscard]] glm::dvec3 applySo3LeftJacobian(
        const glm::dvec3& rotationVector,
        const glm::dvec3& vector
    ) noexcept;

    // Differentiates the same fixed-schedule Gauss-Magnus substeps used by
    // integrateCoupledRateProfilesEndpoint. The supplied evaluator maps an
    // absolute profile coordinate to local (roll, pitch, yaw) rate
    // derivatives. Specialized nominal dispatch is deliberately bypassed.
    [[nodiscard]] CoupledEndpointSensitivityState
    integrateCoupledRateProfileSensitivitiesEndpoint(
        const CoupledEndpointSensitivityState& startingState,
        const math::ScalarTransition* rollRateTransition,
        const math::ScalarTransition& pitchRateTransition,
        const math::ScalarTransition& yawRateTransition,
        const CoupledIntegrationSchedule& schedule,
        double derivativeProfileLength,
        CoupledLocalRateDerivativeEvaluator evaluateRateDerivatives
    );
}
