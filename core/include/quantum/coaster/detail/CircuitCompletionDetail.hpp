#pragma once

#include <quantum/coaster/detail/RiderLocalGeometryDetail.hpp>

#include <glm/vec3.hpp>

#include <array>
#include <cstddef>

namespace quantum::coaster::detail
{
    inline constexpr std::size_t circuitCompletionParameterCount = 9;
    inline constexpr std::size_t circuitCompletionResidualCount = 9;

    using CircuitCompletionParameterVector =
        std::array<double, circuitCompletionParameterCount>;
    using CircuitCompletionResidual =
        std::array<double, circuitCompletionResidualCount>;
    using CircuitCompletionJacobian =
        std::array<CircuitCompletionResidual,
            circuitCompletionParameterCount>;
    using CircuitCompletionOrientationResidual =
        std::array<double, 6>;

    struct CircuitCompletionRateBasis
    {
        double start;
        double midpoint;
        double end;
    };

    struct CircuitCompletionEndpoint
    {
        glm::dvec3 position;
        glm::dvec3 tangent;
        glm::dvec3 up;
    };

    // Circuit Completion always has two linear half-spans. Each member is the
    // exact Phase 1 schedule selected for its nominal half-span.
    struct CircuitCompletionIntegrationSchedule
    {
        std::array<CoupledIntegrationSchedule, 2> spans;
    };

    struct CircuitCompletionSensitivityResult
    {
        CoupledEndpointSensitivityState endpointSensitivity;
        CircuitCompletionJacobian residualJacobian;
    };

    [[nodiscard]] CircuitCompletionRateBasis
    evaluateCircuitCompletionRateBasis(double normalizedCoordinate) noexcept;

    void evaluateCircuitCompletionLocalRateDerivatives(
        double profileCoordinate,
        double profileLength,
        CoupledLocalRateDerivatives& derivatives
    ) noexcept;

    [[nodiscard]] CircuitCompletionIntegrationSchedule
    makeCircuitCompletionIntegrationSchedule(
        const CircuitCompletionParameterVector& parameters,
        double connectorLength
    );

    [[nodiscard]] CircuitCompletionEndpoint
    evaluateCircuitCompletionFullCoupledEndpoint(
        const CircuitCompletionEndpoint& startingEndpoint,
        const CircuitCompletionParameterVector& parameters,
        double connectorLength,
        const CircuitCompletionIntegrationSchedule& schedule
    );

    [[nodiscard]] CircuitCompletionEndpoint
    evaluateCircuitCompletionProductionEndpoint(
        const CircuitCompletionEndpoint& startingEndpoint,
        const CircuitCompletionParameterVector& parameters,
        double connectorLength
    );

    [[nodiscard]] CircuitCompletionSensitivityResult
    evaluateCircuitCompletionEndpointSensitivities(
        const CircuitCompletionEndpoint& startingEndpoint,
        const CircuitCompletionParameterVector& parameters,
        double connectorLength,
        const CircuitCompletionIntegrationSchedule& schedule
    );

    [[nodiscard]] CircuitCompletionResidual
    computeCircuitCompletionResidual(
        const CircuitCompletionEndpoint& actualEndpoint,
        const CircuitCompletionEndpoint& desiredEndpoint,
        double connectorLength
    );

    [[nodiscard]] inline CircuitCompletionOrientationResidual
    computeCircuitCompletionOrientationResidual(
        const glm::dvec3& actualTangent,
        const glm::dvec3& actualUp,
        const glm::dvec3& desiredTangent,
        const glm::dvec3& desiredUp)
    {
        const glm::dvec3 tangentDifference =
            actualTangent - desiredTangent;
        const glm::dvec3 upDifference = actualUp - desiredUp;

        return {
            tangentDifference.x,
            tangentDifference.y,
            tangentDifference.z,
            upDifference.x,
            upDifference.y,
            upDifference.z};
    }

    [[nodiscard]] inline CircuitCompletionParameterVector
    expandLegacyCircuitCompletionParameters(
        const std::array<double, 6>& legacyParameters)
    {
        return {
            legacyParameters[0],
            (legacyParameters[0] + legacyParameters[1]) * 0.5,
            legacyParameters[1],
            legacyParameters[2],
            (legacyParameters[2] + legacyParameters[3]) * 0.5,
            legacyParameters[3],
            legacyParameters[4],
            (legacyParameters[4] + legacyParameters[5]) * 0.5,
            legacyParameters[5]};
    }

    [[nodiscard]] inline bool shouldReplaceCircuitCompletionAttempt(
        const bool haveBestAttempt,
        const bool bestAttemptConverged,
        const double bestResidualRms,
        const bool candidateConverged,
        const double candidateResidualRms)
    {
        if (!haveBestAttempt)
        {
            return true;
        }

        if (candidateConverged != bestAttemptConverged)
        {
            return candidateConverged;
        }

        return candidateResidualRms < bestResidualRms;
    }
}
