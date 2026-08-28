#pragma once

#include <quantum/coaster/CircuitCompletion.hpp>
#include <quantum/coaster/detail/RiderLocalGeometryDetail.hpp>

#include <glm/vec3.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

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

    // Internal/test-only selector. The public completion entry point uses
    // Sensitivity; FiniteDifference remains available as a regression oracle.
    enum class CircuitCompletionJacobianStrategy : std::uint8_t
    {
        FiniteDifference,
        Sensitivity
    };

    [[nodiscard]] const char* circuitCompletionJacobianStrategyLabel(
        CircuitCompletionJacobianStrategy strategy) noexcept;

    struct CircuitCompletionLmWorkCounts
    {
        std::uint64_t jacobianConstructions = 0;
        std::uint64_t connectorIntegrations = 0;
        std::uint64_t sensitivityTraversals = 0;
        std::uint64_t trialIntegrations = 0;
        std::uint64_t dampingTrials = 0;
        std::uint64_t rejectedDampingTrials = 0;
        std::uint64_t clampedParameters = 0;
    };

    struct CircuitCompletionLmIterationDiagnostics
    {
        std::size_t seedIndex = 0;
        std::uint32_t iterationIndex = 0;
        double residualRmsBefore = 0.0;
        std::array<double, circuitCompletionParameterCount> singularValues{};
        std::size_t numericalRank = 0;
        double gradientNorm = 0.0;
        double proposedStepNorm = 0.0;
        double approximateNullSpaceStepNorm = 0.0;
        double parameterNorm = 0.0;
        double lambdaEntering = 0.0;
        std::uint32_t dampingTrials = 0;
        std::uint32_t rejectedDampingTrials = 0;
        bool stepAccepted = false;
        double acceptedResidualRms = 0.0;
        double lambdaAfterStep = 0.0;
        std::uint32_t clampedParameters = 0;
        CircuitCompletionLmWorkCounts cumulativeWork{};
    };

    struct CircuitCompletionLmSeedDiagnostics
    {
        std::size_t seedIndex = 0;
        CircuitCompletionParameterVector initialParameters{};
        CircuitCompletionParameterVector finalParameters{};
        bool converged = false;
        bool hitIterationLimit = false;
        std::uint32_t iterationCount = 0;
        double finalResidualRms = 0.0;
        double finalPositionError = 0.0;
        double finalTangentErrorDegrees = 0.0;
        double finalFrameErrorDegrees = 0.0;
        CircuitCompletionLmWorkCounts work{};
        std::vector<double> residualProgression;
        std::vector<CircuitCompletionLmIterationDiagnostics> iterations;
    };

    struct CircuitCompletionLmDiagnostics
    {
        CircuitCompletionJacobianStrategy strategy =
            CircuitCompletionJacobianStrategy::Sensitivity;
        std::vector<CircuitCompletionLmSeedDiagnostics> seeds;
        std::size_t selectedSeedIndex =
            std::numeric_limits<std::size_t>::max();
        std::uint32_t totalIterationsSpent = 0;
        CircuitCompletionParameterVector finalParameters{};
        double finalResidualRms = 0.0;
        CircuitCompletionLmWorkCounts work{};
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

    [[nodiscard]] CircuitCompletionResult
    completeCircuitCandidateWithJacobianStrategy(
        const AuthoredTrack& source,
        const CircuitCompletionSettings& settings,
        CircuitCompletionJacobianStrategy strategy,
        CircuitCompletionLmDiagnostics* diagnostics = nullptr
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
