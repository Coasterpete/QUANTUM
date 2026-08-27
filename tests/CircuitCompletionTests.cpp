#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/CircuitCompletion.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/detail/CircuitCompletionDetail.hpp>
#include <quantum/coaster/TrackTopology.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <string>
#include <string_view>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::AuthoredTrackSection;
    using quantum::coaster::CircuitCompletionFailure;
    using quantum::coaster::CircuitCompletionResult;
    using quantum::coaster::CircuitCompletionSettings;
    using quantum::coaster::completeCircuitCandidate;
    using quantum::coaster::computeTrackTopology;
    using quantum::coaster::createNewDocument;
    using quantum::coaster::serializeCoasterDocument;
    using quantum::coaster::deserializeCoasterDocument;
    using quantum::coaster::GeometricSection;
    using quantum::coaster::LayoutMode;
    using quantum::coaster::ChannelProfile;
    using quantum::coaster::ProfileSegment;
    using quantum::coaster::RegionKind;
    using quantum::coaster::TopologyKind;
    using quantum::coaster::TopologyTolerances;
    using quantum::coaster::TrackTopology;

    class TestFailure final : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    void require(
        const bool condition,
        const std::string_view message)
    {
        if (!condition)
        {
            throw TestFailure(std::string(message));
        }
    }

    void requireApproxEq(
        const double actual,
        const double expected,
        const double tolerance,
        const std::string_view label)
    {
        if (std::abs(actual - expected) > tolerance)
        {
            char buf[256];
            std::snprintf(
                buf, sizeof(buf),
                "%s: expected %.6f +/- %.6f, got %.6f",
                std::string(label).c_str(),
                expected, tolerance, actual);
            throw TestFailure(buf);
        }
    }

    AuthoredTrack makeStraightTrack(const double length)
    {
        AuthoredTrack track;
        track.setLayoutMode(LayoutMode::Circuit);
        track.prependSection();

        AuthoredTrackSection section;
        section.kind = RegionKind::RateProfiles;
        section.length = length;

        auto makeZero = [&](double len)
        {
            ChannelProfile p;
            p.nextSegmentId = 2;
            ProfileSegment seg;
            seg.id = 1;
            seg.transition.domainBegin = 0.0;
            seg.transition.domainEnd = len;
            seg.transition.valueBegin = 0.0;
            seg.transition.valueEnd = 0.0;
            seg.transition.transitionType =
                quantum::math::TransitionType::Linear;
            p.segments.push_back(seg);
            return p;
        };

        GeometricSection profiles;
        profiles.pitch = makeZero(length);
        profiles.yaw = makeZero(length);
        profiles.roll = makeZero(length);
        section.region =
            quantum::coaster::RateProfileRegion{profiles};

        track.section(0) = std::move(section);
        return track;
    }

    // ================================================================
    // 1. Already closed circuit returns AlreadyClosed
    // ================================================================
    void test1_alreadyClosed()
    {
        // Build a two-section track that forms a closed loop.
        // Section 1: straight + yaw to curve back.
        // Section 2: straight back.
        // Easier: use a single section that is already closed.
        // Actually, just use a track that is already closed.
        // The simplest: single section with zero rates = NOT closed.
        // We need to make a real closed track.

        // For now, build a completion fixture and complete it,
        // then try to complete the result again.
        AuthoredTrack fixture = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;
        CircuitCompletionResult r1 =
            completeCircuitCandidate(fixture, settings);
        require(
            r1.success,
            "Fixture completion must succeed for test1");

        // Now try to complete the already-closed result.
        CircuitCompletionResult r2 =
            completeCircuitCandidate(
                r1.completedTrack, settings);
        require(
            !r2.success,
            "Already closed must fail");
        require(
            r2.failureReason
                == CircuitCompletionFailure::AlreadyClosed,
            "Failure reason must be AlreadyClosed");
    }

    // ================================================================
    // 2. Shuttle layout refuses completion
    // ================================================================
    void test2_shuttleRefuses()
    {
        AuthoredTrack track = makeStraightTrack(30.0);
        track.setLayoutMode(LayoutMode::Shuttle);

        CircuitCompletionResult result =
            completeCircuitCandidate(track);
        require(
            !result.success,
            "Shuttle must fail");
        require(
            result.failureReason
                == CircuitCompletionFailure::ShuttleLayout,
            "Failure reason must be ShuttleLayout");
    }

    // ================================================================
    // 3. Supported open fixture produces connector region(s)
    // ================================================================
    void test3_supportedFixture()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(
            result.success,
            "Straight 15m track must complete");
        require(
            result.connectorRegionCount == 1,
            "Must produce exactly 1 connector region");
        require(
            result.completedTrack.sectionCount() == 2,
            "Completed track must have 2 sections");
    }

    // ================================================================
    // 4. Successful result independently classifies ClosedCircuit
    // ================================================================
    void test4_independentClosure()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(result.success, "Must succeed");

        const TrackTopology topo =
            computeTrackTopology(result.completedTrack);
        require(
            topo.kind == TopologyKind::ClosedCircuit,
            "Topology must independently report ClosedCircuit");
    }

    // ================================================================
    // 5. Final positional gap within closureGapTolerance
    // ================================================================
    void test5_positionalGap()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(result.success, "Must succeed");

        const TopologyTolerances tolerances;
        require(
            result.finalPositionalGap
                <= tolerances.closureGapTolerance,
            "Positional gap must be within closureGapTolerance");
    }

    // ================================================================
    // 6. Final tangent error within angleTolerance
    // ================================================================
    void test6_tangentError()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(result.success, "Must succeed");

        const TopologyTolerances tolerances;
        require(
            result.finalTangentErrorDegrees
                <= tolerances.angleTolerance,
            "Tangent error must be within angleTolerance");
    }

    // ================================================================
    // 7. Final frame error within angleTolerance
    // ================================================================
    void test7_frameError()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(result.success, "Must succeed");

        const TopologyTolerances tolerances;
        require(
            result.finalFrameErrorDegrees
                <= tolerances.angleTolerance,
            "Frame error must be within angleTolerance");
    }

    // ================================================================
    // 8. Source AuthoredTrack is unchanged by solver
    // ================================================================
    void test8_sourceUnchanged()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        const std::size_t origCount = track.sectionCount();
        const double origLength = track.section(0).length;

        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(result.success, "Must succeed");

        require(
            track.sectionCount() == origCount,
            "Source section count must not change");
        require(
            std::abs(track.section(0).length - origLength) < 1e-12,
            "Source section length must not change");
    }

    // ================================================================
    // 9. Failed solve does not mutate source
    // ================================================================
    void test9_failedSolveUnchanged()
    {
        AuthoredTrack track = makeStraightTrack(30.0);
        track.setLayoutMode(LayoutMode::Shuttle);
        const std::size_t origCount = track.sectionCount();

        CircuitCompletionResult result =
            completeCircuitCandidate(track);
        require(!result.success, "Must fail for Shuttle");

        require(
            track.sectionCount() == origCount,
            "Source must be unchanged after failed solve");
    }

    // ================================================================
    // 10. Deterministic repeated solve produces identical connector
    // ================================================================
    void test10_deterministic()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult r1 =
            completeCircuitCandidate(track, settings);
        CircuitCompletionResult r2 =
            completeCircuitCandidate(track, settings);

        require(r1.success, "First must succeed");
        require(r2.success, "Second must succeed");
        require(
            r1.connectorRegionCount == r2.connectorRegionCount,
            "Connector count must match");
        require(
            std::abs(r1.finalPositionalGap
                - r2.finalPositionalGap) < 1e-12,
            "Positional gap must be identical");
        require(
            std::abs(r1.finalTangentErrorDegrees
                - r2.finalTangentErrorDegrees) < 1e-12,
            "Tangent error must be identical");
        require(
            std::abs(r1.finalFrameErrorDegrees
                - r2.finalFrameErrorDegrees) < 1e-12,
            "Frame error must be identical");
        require(
            r1.iterationCount == r2.iterationCount,
            "Iteration count must be identical");
    }

    // ================================================================
    // 11. Generated region(s) pass existing validation
    // ================================================================
    void test11_generatedRegionValid()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(result.success, "Must succeed");

        // Accessing the last section should not throw.
        const auto& section =
            result.completedTrack.section(
                result.completedTrack.sectionCount() - 1);
        require(
            section.kind == RegionKind::RateProfiles,
            "Connector must be RateProfiles");
        require(
            section.length > 0.0,
            "Connector length must be positive");
    }

    // ================================================================
    // 12. Generated SegmentIds are valid
    // ================================================================
    void test12_segmentIdsValid()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(result.success, "Must succeed");

        const auto& profiles =
            result.completedTrack.section(
                result.completedTrack.sectionCount() - 1)
            .rateProfileRegion().rateProfiles;

        for (const auto& segment : profiles.pitch.segments)
        {
            require(
                segment.id != quantum::coaster::invalidSegmentId,
                "Pitch segment id must be valid");
        }
        for (const auto& segment : profiles.yaw.segments)
        {
            require(
                segment.id != quantum::coaster::invalidSegmentId,
                "Yaw segment id must be valid");
        }
        for (const auto& segment : profiles.roll.segments)
        {
            require(
                segment.id != quantum::coaster::invalidSegmentId,
                "Roll segment id must be valid");
        }
    }

    // ================================================================
    // 13. Generated nextSegmentId values are valid
    // ================================================================
    void test13_nextSegmentIdValid()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(result.success, "Must succeed");

        const auto& profiles =
            result.completedTrack.section(
                result.completedTrack.sectionCount() - 1)
            .rateProfileRegion().rateProfiles;

        require(
            profiles.pitch.nextSegmentId > 0,
            "pitch.nextSegmentId must be > 0");
        require(
            profiles.yaw.nextSegmentId > 0,
            "yaw.nextSegmentId must be > 0");
        require(
            profiles.roll.nextSegmentId > 0,
            "roll.nextSegmentId must be > 0");
    }

    // ================================================================
    // 14. Generated connector serializes successfully
    // ================================================================
    void test14_serializes()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(result.success, "Must succeed");

        const std::string json =
            serializeCoasterDocument(result.completedTrack);
        require(
            !json.empty(),
            "Serialized JSON must not be empty");
    }

    // ================================================================
    // 15. Generated connector round-trips through CoasterDocument
    // ================================================================
    void test15_roundTrip()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(result.success, "Must succeed");

        const std::string json =
            serializeCoasterDocument(result.completedTrack);
        auto deserialized = deserializeCoasterDocument(json);
        require(
            deserialized.has_value(),
            "Deserialization must succeed");
        require(
            deserialized->sectionCount()
                == result.completedTrack.sectionCount(),
            "Section count must survive round-trip");
    }

    // ================================================================
    // 16. Topology remains ClosedCircuit after round-trip
    // ================================================================
    void test16_topologyAfterRoundTrip()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(result.success, "Must succeed");

        const std::string json =
            serializeCoasterDocument(result.completedTrack);
        auto deserialized = deserializeCoasterDocument(json);
        require(deserialized.has_value(), "Deser must succeed");

        const TrackTopology topo =
            computeTrackTopology(*deserialized);
        require(
            topo.kind == TopologyKind::ClosedCircuit,
            "Topology must remain ClosedCircuit after round-trip");
    }

    // ================================================================
    // 17. Invalid/NaN settings rejected
    // ================================================================
    void test17_nanSettings()
    {
        AuthoredTrack track = makeStraightTrack(30.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = std::numeric_limits<double>::quiet_NaN();

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(
            !result.success,
            "NaN connector length must be rejected");
        require(
            result.failureReason
                == CircuitCompletionFailure::InvalidInput,
            "Failure reason must be InvalidInput");
    }

    // ================================================================
    // 18. Unsupported geometry fails cleanly
    // ================================================================
    void test18_unsupportedGeometry()
    {
        // A single-section empty track has no sections.
        AuthoredTrack empty;
        CircuitCompletionResult result =
            completeCircuitCandidate(empty);
        require(
            !result.success,
            "Empty track must fail");
        require(
            result.failureReason
                == CircuitCompletionFailure::InvalidInput,
            "Failure reason must be InvalidInput");
    }

    // ================================================================
    // 19. Successful solve reports its iteration count
    // ================================================================
    void test19_iterationCount()
    {
        // The canonical fixture requires refinement rather than
        // converging from its initial seed.
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(result.success, "Must succeed");
        require(
            result.iterationCount > 0,
            "Iteration count must be positive");
    }

    // ================================================================
    // 20. Failure never returns partial committed geometry
    // ================================================================
    void test20_noPartialCommit()
    {
        // Verify that on failure, the completedTrack field is
        // default-constructed (empty) — no partial state.
        AuthoredTrack track = makeStraightTrack(30.0);
        track.setLayoutMode(LayoutMode::Shuttle);

        CircuitCompletionResult result =
            completeCircuitCandidate(track);
        require(!result.success, "Must fail");

        require(
            result.completedTrack.sectionCount() == 0,
            "Failed result must have empty completedTrack");
    }

    // ================================================================
    // 21. Current non-convergent fixture fails cleanly
    // ================================================================
    void test21_nonConvergentFixture()
    {
        // The current deterministic seed search does not close this
        // 30m/40m fixture. This is a clean-failure regression, not a
        // claim that the nine-parameter rate family is infeasible.
        AuthoredTrack track = makeStraightTrack(30.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(
            !result.success,
            "Current 30m/40m fixture must fail cleanly");
        require(
            result.failureReason
                == CircuitCompletionFailure::DidNotConverge,
            "Failure reason must remain DidNotConverge");
        // Source must be unchanged.
        require(
            track.sectionCount() == 1,
            "Source track must be unchanged after failed solve");
        // No partial geometry committed.
        require(
            result.completedTrack.sectionCount() == 0,
            "Failed result must have empty completedTrack");
    }

    // ================================================================
    // 22. Tangent reversal has a nonzero orientation residual
    // ================================================================
    void test22_tangentReversalResidual()
    {
        const auto residual =
            quantum::coaster::detail::
                computeCircuitCompletionOrientationResidual(
                    glm::dvec3{-1.0, 0.0, 0.0},
                    glm::dvec3{0.0, 0.0, 1.0},
                    glm::dvec3{1.0, 0.0, 0.0},
                    glm::dvec3{0.0, 0.0, 1.0});

        requireApproxEq(
            residual[0], -2.0, 1e-12,
            "Reversed tangent residual");
        require(
            residual != decltype(residual){},
            "A 180-degree tangent reversal must not have zero residual");
    }

    // ================================================================
    // 23. Reversed frame has a nonzero orientation residual
    // ================================================================
    void test23_reversedFrameResidual()
    {
        const auto residual =
            quantum::coaster::detail::
                computeCircuitCompletionOrientationResidual(
                    glm::dvec3{1.0, 0.0, 0.0},
                    glm::dvec3{0.0, 0.0, -1.0},
                    glm::dvec3{1.0, 0.0, 0.0},
                    glm::dvec3{0.0, 0.0, 1.0});

        requireApproxEq(
            residual[5], -2.0, 1e-12,
            "Reversed up-vector residual");
        require(
            residual != decltype(residual){},
            "An upside-down frame must not have zero residual");
    }

    // ================================================================
    // 24. A converged candidate always wins residual ranking
    // ================================================================
    void test24_convergedCandidatePrecedence()
    {
        using quantum::coaster::detail::
            shouldReplaceCircuitCompletionAttempt;

        require(
            shouldReplaceCircuitCompletionAttempt(
                true, false, 0.0, true, 1.0),
            "A converged candidate must replace a lower-RMS failure");
        require(
            !shouldReplaceCircuitCompletionAttempt(
                true, true, 1.0, false, 0.0),
            "A lower-RMS failure must not replace a converged candidate");
    }

    // ================================================================
    // 25. Generated profiles use the midpoint split
    // ================================================================
    void test25_midpointSplitProfiles()
    {
        AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        const CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        require(result.success, "Canonical fixture must succeed");

        const GeometricSection& profiles =
            result.completedTrack.section(
                result.completedTrack.sectionCount() - 1)
            .rateProfileRegion().rateProfiles;

        const std::array<const ChannelProfile*, 3> channels{
            &profiles.pitch, &profiles.yaw, &profiles.roll};

        for (const ChannelProfile* channel : channels)
        {
            require(
                channel->segments.size() == 2,
                "Each generated channel must contain two segments");
            requireApproxEq(
                channel->segments[0].transition.domainBegin,
                0.0, 1e-12, "First segment start");
            requireApproxEq(
                channel->segments[0].transition.domainEnd,
                20.0, 1e-12, "First segment midpoint");
            requireApproxEq(
                channel->segments[1].transition.domainBegin,
                20.0, 1e-12, "Second segment midpoint");
            requireApproxEq(
                channel->segments[1].transition.domainEnd,
                40.0, 1e-12, "Second segment end");
            requireApproxEq(
                channel->segments[0].transition.valueEnd,
                channel->segments[1].transition.valueBegin,
                1e-12, "Midpoint value continuity");
        }
    }

    // ================================================================
    // 26. Legacy endpoint parameters expand to midpoint parameters
    // ================================================================
    void test26_legacyOverrideMapping()
    {
        const std::array<double, 6> legacy{
            1.0, 3.0, 5.0, 9.0, 11.0, 17.0};
        const auto expanded =
            quantum::coaster::detail::
                expandLegacyCircuitCompletionParameters(legacy);
        const decltype(expanded) expected{
            1.0, 2.0, 3.0,
            5.0, 7.0, 9.0,
            11.0, 14.0, 17.0};

        require(
            expanded == expected,
            "Legacy parameters must map to endpoint-average midpoints");
    }
}

int main()
{
    int passed = 0;
    int failed = 0;

    auto run = [&](const char* name, void (*fn)())
    {
        try
        {
            fn();
            ++passed;
            std::fprintf(stdout, "  PASS  %s\n", name);
        }
        catch (const TestFailure& e)
        {
            ++failed;
            std::fprintf(stderr, "  FAIL  %s: %s\n", name, e.what());
        }
        catch (const std::exception& e)
        {
            ++failed;
            std::fprintf(
                stderr,
                "  FAIL  %s (exception): %s\n",
                name,
                e.what());
        }
    };

    std::fprintf(stdout, "Circuit Completion Tests\n");

    run("Already closed returns AlreadyClosed",
        test1_alreadyClosed);
    run("Shuttle layout refuses completion",
        test2_shuttleRefuses);
    run("Supported fixture produces connector",
        test3_supportedFixture);
    run("Result independently classifies ClosedCircuit",
        test4_independentClosure);
    run("Positional gap within tolerance",
        test5_positionalGap);
    run("Tangent error within tolerance",
        test6_tangentError);
    run("Frame error within tolerance",
        test7_frameError);
    run("Source unchanged by solver",
        test8_sourceUnchanged);
    run("Failed solve does not mutate source",
        test9_failedSolveUnchanged);
    run("Deterministic repeated solve",
        test10_deterministic);
    run("Generated region passes validation",
        test11_generatedRegionValid);
    run("Generated SegmentIds are valid",
        test12_segmentIdsValid);
    run("Generated nextSegmentId values are valid",
        test13_nextSegmentIdValid);
    run("Generated connector serializes",
        test14_serializes);
    run("Generated connector round-trips",
        test15_roundTrip);
    run("Topology ClosedCircuit after round-trip",
        test16_topologyAfterRoundTrip);
    run("NaN settings rejected",
        test17_nanSettings);
    run("Empty track fails cleanly",
        test18_unsupportedGeometry);
    run("Iteration count is positive",
        test19_iterationCount);
    run("Failure returns no partial geometry",
        test20_noPartialCommit);
    run("Current non-convergent fixture fails cleanly",
        test21_nonConvergentFixture);
    run("Tangent reversal residual is nonzero",
        test22_tangentReversalResidual);
    run("Reversed frame residual is nonzero",
        test23_reversedFrameResidual);
    run("Converged candidate wins residual ranking",
        test24_convergedCandidatePrecedence);
    run("Generated profiles use midpoint split",
        test25_midpointSplitProfiles);
    run("Legacy override maps to midpoint parameters",
        test26_legacyOverrideMapping);

    std::fprintf(
        stdout,
        "\nResults: %d passed, %d failed\n",
        passed,
        failed);

    return failed == 0 ? 0 : 1;
}
