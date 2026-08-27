#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/TrackTopology.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace quantum::coaster
{
    // Why a completion attempt was not made or did not succeed.
    enum class CircuitCompletionFailure : std::uint8_t
    {
        None,
        AlreadyClosed,
        ShuttleLayout,
        InvalidInput,
        UnsupportedGeometry,
        DidNotConverge,
        ValidationFailed
    };

    // Returns a human-readable label for a failure reason.
    [[nodiscard]] const char* circuitCompletionFailureLabel(
        CircuitCompletionFailure failure) noexcept;

    // User-facing settings for the completion solver.  Only a small
    // subset is exposed; solver internals stay hidden.
    struct CircuitCompletionSettings
    {
        // Preferred connector region length.  The solver may adjust
        // this slightly but will not deviate far from the request.
        double preferredConnectorLength = 40.0;

        // Diagnostic: if set, override the solver's initial parameter
        // guess.  Parameters are [pitchStart, pitchEnd, yawStart,
        // yawEnd, rollStart, rollEnd] in rad/m.
        std::optional<std::array<double, 6>> initialParamOverride;
    };

    // Outcome of a circuit completion attempt.  On success the
    // completedTrack is valid and contains the connector region.
    // On failure the failureReason and failureMessage explain why.
    struct CircuitCompletionResult
    {
        bool success = false;
        AuthoredTrack completedTrack{};
        std::size_t connectorRegionCount = 0;
        double finalPositionalGap = 0.0;
        double finalTangentErrorDegrees = 0.0;
        double finalFrameErrorDegrees = 0.0;
        double finalMaxAbsParam = 0.0;
        std::uint32_t iterationCount = 0;
        CircuitCompletionFailure failureReason =
            CircuitCompletionFailure::None;
        std::string failureMessage;
    };

    // Attempts to generate connector Region(s) that close the
    // circuit.  Does not mutate `source`.
    //
    // On success the result's completedTrack is identical to
    // `source` with one new RateProfile Region appended that closes
    // the loop within topology tolerances.
    //
    // On failure the source document is guaranteed unchanged.
    [[nodiscard]] CircuitCompletionResult
    completeCircuitCandidate(
        const AuthoredTrack& source,
        const CircuitCompletionSettings& settings = {});
}
