#pragma once

#include <glm/vec3.hpp>

#include <array>

namespace quantum::coaster::detail
{
    inline constexpr std::size_t circuitCompletionParameterCount = 9;
    inline constexpr std::size_t circuitCompletionResidualCount = 9;

    using CircuitCompletionParameterVector =
        std::array<double, circuitCompletionParameterCount>;
    using CircuitCompletionOrientationResidual =
        std::array<double, 6>;

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
