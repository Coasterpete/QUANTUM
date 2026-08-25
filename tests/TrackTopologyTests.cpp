#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/TrackTopology.hpp>

#include <cmath>
#include <cstdio>
#include <exception>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::ChannelProfile;
    using quantum::coaster::ClosureDiagnostics;
    using quantum::coaster::createNewDocument;
    using quantum::coaster::defaultNewSectionLength;
    using quantum::coaster::ProfileSegment;
    using quantum::coaster::SegmentId;
    using quantum::coaster::TrackEndpoint;
    using quantum::coaster::TrackEndpointRole;
    using quantum::coaster::TrackTopology;
    using quantum::coaster::TopologyKind;
    using quantum::coaster::TopologyTolerances;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

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
        char buf[128]{};
        std::snprintf(
            buf,
            sizeof(buf),
            "%s: expected %.8f +/- %.8f, got %.8f",
            std::string(label).c_str(),
            expected,
            tolerance,
            actual);
        require(std::abs(actual - expected) <= tolerance, buf);
    }

    ProfileSegment makeSegment(
        SegmentId id,
        double domainBegin,
        double domainEnd,
        double valueBegin,
        double valueEnd,
        TransitionType type)
    {
        return ProfileSegment{id, ScalarTransition{domainBegin, domainEnd, valueBegin, valueEnd, type}};
    }

    void setChannelToConstant(
        ChannelProfile& channel,
        double domainEnd,
        double value,
        SegmentId id = 1)
    {
        channel.segments.clear();
        channel.segments.push_back(
            makeSegment(id, 0.0, domainEnd, value, value, TransitionType::Linear));
        channel.nextSegmentId = id + 1;
    }

    AuthoredTrack createStraightTrack()
    {
        return quantum::coaster::createNewDocument();
    }

    // Creates a closed circular track in the XZ plane using constant yaw
    // rate. With rate R=2pi and length L=1.0 the centerline traces a
    // unit-circle arc that returns to the origin with tangent aligned to
    // the start tangent (1,0,0).
    AuthoredTrack createCircularTrack()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        auto& section = track.section(0);

        constexpr double yawRate = 2.0 * 3.14159265358979323846;
        constexpr double len = 1.0;

        section.length = len;

        auto& profiles = section.rateProfileRegion().rateProfiles;
        setChannelToConstant(profiles.roll, len, 0.0);
        setChannelToConstant(profiles.pitch, len, 0.0);
        setChannelToConstant(profiles.yaw, len, yawRate);

        return track;
    }

    // Test 1: straight open track reports OpenLinear
    void test1_straightOpenTrack()
    {
        const AuthoredTrack track = createStraightTrack();
        const TrackTopology topology =
            quantum::coaster::computeTrackTopology(track);

        require(
            topology.kind == TopologyKind::OpenLinear,
            "Straight track should be OpenLinear");
    }

    // Test 2: endpoint extraction returns correct Start/End roles
    void test2_endpointRoles()
    {
        const AuthoredTrack track = createStraightTrack();
        const TrackEndpoint start =
            quantum::coaster::extractStartEndpoint(track);
        const TrackEndpoint end =
            quantum::coaster::extractEndEndpoint(track);

        require(
            start.role == TrackEndpointRole::Start,
            "Start endpoint should have Start role");
        require(
            end.role == TrackEndpointRole::End,
            "End endpoint should have End role");
    }

    // Test 3: positional gap calculation
    void test3_positionalGap()
    {
        const AuthoredTrack track = createStraightTrack();
        const TrackTopology topology =
            quantum::coaster::computeTrackTopology(track);

        requireApproxEq(
            topology.diagnostics.positionalGap,
            defaultNewSectionLength,
            0.01,
            "Positional gap");
    }

    // Test 4: tangent mismatch calculation
    void test4_tangentMismatch()
    {
        const AuthoredTrack track = createStraightTrack();
        const TrackTopology topology =
            quantum::coaster::computeTrackTopology(track);

        requireApproxEq(
            topology.diagnostics.tangentMismatchDegrees,
            0.0,
            0.001,
            "Tangent mismatch for straight track");
    }

    // Test 5: frame mismatch calculation
    void test5_frameMismatch()
    {
        const AuthoredTrack track = createStraightTrack();
        const TrackTopology topology =
            quantum::coaster::computeTrackTopology(track);

        requireApproxEq(
            topology.diagnostics.frameMismatchDegrees,
            0.0,
            0.001,
            "Frame mismatch for straight track");
    }

    // Test 6: exact matching endpoints classify as compatible
    void test6_exactEndpointsAreCompatible()
    {
        const AuthoredTrack track = createStraightTrack();

        // Generous gap tolerance that the straight track's zero angular
        // mismatches easily satisfy.
        TopologyTolerances generous;
        generous.closureGapTolerance = 100.0;

        const TrackTopology topology =
            quantum::coaster::computeTrackTopology(track, generous);

        require(
            topology.diagnostics.closureValid,
            "Straight track with generous gap tolerance should be "
            "closure-compatible");

        require(
            topology.kind == TopologyKind::ClosedCircuit,
            "Compatible endpoints should classify as ClosedCircuit");
    }

    // Test 7: position mismatch prevents closure
    void test7_positionMismatchPreventsClosure()
    {
        const AuthoredTrack track = createStraightTrack();
        const TrackTopology topology =
            quantum::coaster::computeTrackTopology(track);

        require(
            !topology.diagnostics.closureValid,
            "60-unit gap should prevent closure");
        require(
            topology.kind == TopologyKind::OpenLinear,
            "Position mismatch should yield OpenLinear");
    }

    // Test 8: tangent mismatch prevents closure
    void test8_tangentMismatchPreventsClosure()
    {
        // A small yaw rate creates a gentle curve with nonzero end tangent
        // mismatch but a huge positional gap. Even with a generous gap
        // tolerance the tangent mismatch blocks closure.
        AuthoredTrack track = quantum::coaster::createNewDocument();
        auto& section = track.section(0);

        constexpr double yawRate = 0.1;
        constexpr double len = 60.0;
        section.length = len;

        auto& profiles = section.rateProfileRegion().rateProfiles;
        setChannelToConstant(profiles.roll, len, 0.0);
        setChannelToConstant(profiles.pitch, len, 0.0);
        setChannelToConstant(profiles.yaw, len, yawRate);

        TopologyTolerances loose;
        loose.closureGapTolerance = 1000.0;

        const TrackTopology topology =
            quantum::coaster::computeTrackTopology(track, loose);

        require(
            topology.diagnostics.tangentMismatchDegrees > 0.5,
            "Yaw rate should produce tangent mismatch above tolerance");
        require(
            !topology.diagnostics.closureValid,
            "Tangent mismatch should prevent closure");
    }

    // Test 9: frame mismatch prevents closure when over tolerance
    void test9_frameMismatchPreventsClosure()
    {
        // A roll rate rotates the frame without moving the centerline.
        // After integration the up direction deviates from the start.
        AuthoredTrack track = quantum::coaster::createNewDocument();
        auto& section = track.section(0);

        constexpr double rollRate = 1.0;
        constexpr double len = 10.0;
        section.length = len;

        auto& profiles = section.rateProfileRegion().rateProfiles;
        setChannelToConstant(profiles.roll, len, rollRate);
        setChannelToConstant(profiles.pitch, len, 0.0);
        setChannelToConstant(profiles.yaw, len, 0.0);

        // Generous gap tolerance so only the frame mismatch blocks closure.
        TopologyTolerances loose;
        loose.closureGapTolerance = 100.0;

        const TrackTopology topology =
            quantum::coaster::computeTrackTopology(track, loose);

        require(
            topology.diagnostics.frameMismatchDegrees > 0.5,
            "Roll rate should produce frame mismatch above tolerance");
        require(
            !topology.diagnostics.closureValid,
            "Frame mismatch should prevent closure");
    }

    // Test 10: closed-loop fixture reports ClosedCircuit
    void test10_closedLoopFixture()
    {
        const AuthoredTrack track = createCircularTrack();
        const TrackTopology topology =
            quantum::coaster::computeTrackTopology(track);

        require(
            topology.kind == TopologyKind::ClosedCircuit,
            "Circular track should be ClosedCircuit");
        require(
            topology.diagnostics.closureValid,
            "Circular track diagnostics should indicate closure");
        require(
            topology.diagnostics.positionalGap < 0.001,
            "Circular track gap should be near zero");
        require(
            topology.diagnostics.tangentMismatchDegrees < 0.5,
            "Circular track tangent mismatch should be small");
        require(
            topology.diagnostics.frameMismatchDegrees < 0.5,
            "Circular track frame mismatch should be small");
    }

    // Test 11: topology analysis is deterministic
    void test11_deterministic()
    {
        const AuthoredTrack track = createCircularTrack();
        const TrackTopology first =
            quantum::coaster::computeTrackTopology(track);
        const TrackTopology second =
            quantum::coaster::computeTrackTopology(track);

        require(
            first.kind == second.kind,
            "Topology kind must be deterministic");
        requireApproxEq(
            first.diagnostics.positionalGap,
            second.diagnostics.positionalGap,
            0.0,
            "Positional gap determinism");
        requireApproxEq(
            first.diagnostics.tangentMismatchDegrees,
            second.diagnostics.tangentMismatchDegrees,
            0.0,
            "Tangent mismatch determinism");
        requireApproxEq(
            first.diagnostics.frameMismatchDegrees,
            second.diagnostics.frameMismatchDegrees,
            0.0,
            "Frame mismatch determinism");
    }

    // Test 12: analysis does not mutate AuthoredTrack
    void test12_doesNotMutate()
    {
        AuthoredTrack track = createStraightTrack();
        const std::size_t originalCount = track.sectionCount();
        const double originalLength =
            quantum::coaster::sectionLength(track.section(0));

        const TrackTopology topology =
            quantum::coaster::computeTrackTopology(track);

        require(
            track.sectionCount() == originalCount,
            "Section count must not change");
        requireApproxEq(
            quantum::coaster::sectionLength(track.section(0)),
            originalLength,
            0.0,
            "Section length must not change");
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

    std::fprintf(stdout, "Track Topology Tests\n");

    run("Straight open track reports OpenLinear",
        test1_straightOpenTrack);
    run("Endpoint extraction returns correct Start/End roles",
        test2_endpointRoles);
    run("Positional gap calculation", test3_positionalGap);
    run("Tangent mismatch calculation", test4_tangentMismatch);
    run("Frame mismatch calculation", test5_frameMismatch);
    run("Exact matching endpoints classify as compatible",
        test6_exactEndpointsAreCompatible);
    run("Position mismatch prevents closure",
        test7_positionMismatchPreventsClosure);
    run("Tangent mismatch prevents closure",
        test8_tangentMismatchPreventsClosure);
    run("Frame mismatch prevents closure when over tolerance",
        test9_frameMismatchPreventsClosure);
    run("Closed-loop fixture reports ClosedCircuit",
        test10_closedLoopFixture);
    run("Topology analysis is deterministic", test11_deterministic);
    run("Analysis does not mutate AuthoredTrack", test12_doesNotMutate);

    std::fprintf(
        stdout,
        "\nResults: %d passed, %d failed\n",
        passed,
        failed);

    return failed == 0 ? 0 : 1;
}
