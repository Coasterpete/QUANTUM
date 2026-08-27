#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/CircuitCompletion.hpp>
#include <quantum/coaster/GeometricSection.hpp>
#include <quantum/coaster/RiderLocalGeometry.hpp>
#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/math/ScalarTransition.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::AuthoredTrackSection;
    using quantum::coaster::CircuitCompletionResult;
    using quantum::coaster::CircuitCompletionSettings;
    using quantum::coaster::completeCircuitCandidate;
    using quantum::coaster::GeometricSection;
    using quantum::coaster::ChannelProfile;
    using quantum::coaster::ProfileSegment;
    using quantum::coaster::RegionKind;
    using quantum::coaster::LayoutMode;
    using quantum::coaster::RateProfileRegion;

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
        section.region = RateProfileRegion{profiles};
        track.section(0) = std::move(section);
        return track;
    }

    void runTest(const char* label,
                 AuthoredTrack& track,
                 std::array<double, 6> params)
    {
        using Clock = std::chrono::steady_clock;
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;
        settings.initialParamOverride = params;

        auto t0 = Clock::now();
        CircuitCompletionResult r =
            completeCircuitCandidate(track, settings);
        auto t1 = Clock::now();
        double ms = std::chrono::duration<double,
            std::milli>(t1 - t0).count();

        std::fprintf(stdout,
            "%-30s %s %4u %10.6f %8.4f %8.4f %8.4f %8.1fms\n",
            label,
            r.success ? "YES" : "NO ",
            r.iterationCount,
            r.finalPositionalGap,
            r.finalTangentErrorDegrees,
            r.finalFrameErrorDegrees,
            r.finalMaxAbsParam,
            ms);
        fflush(stdout);
    }
}

int main()
{
    AuthoredTrack track30 = makeStraightTrack(30.0);

    std::fprintf(stdout,
        "%-30s %3s %4s %10s %8s %8s %8s %10s\n",
        "config", "ok", "iter", "posGap",
        "tang", "frame", "maxP", "time");
    std::fprintf(stdout,
        "------------------------------ --- ---- "
        "---------- -------- -------- -------- ----------\n");
    fflush(stdout);

    // Seeds to test
    double seeds[] = {0.05, 0.1};
    for (double s : seeds)
    {
        char label[64];
        std::snprintf(label, sizeof(label),
            "pitch+yaw seed=%.3f", s);
        runTest(label, track30,
            {s, s, -s, -s, 0.0, 0.0});
    }

    // Best brute-force parameters from feasibility analysis
    runTest("BF best (-0.2,0.1,-0.2,0.1)",
        track30, {-0.2, 0.1, -0.2, 0.1, 0.0, 0.0});

    // Symmetric BF
    runTest("BF sym (-0.2,-0.2,-0.2,-0.2)",
        track30, {-0.2, -0.2, -0.2, -0.2, 0.0, 0.0});

    // With roll
    runTest("pitch+yaw+roll seed=0.1",
        track30, {0.1, 0.1, -0.1, -0.1, 0.05, -0.05});

    // Default heuristic
    runTest("default heuristic",
        track30, {});

    // 30m track, 60m connector
    std::fprintf(stdout, "\n--- 30m track, 60m connector ---\n");
    fflush(stdout);
    AuthoredTrack track30_60 = makeStraightTrack(30.0);
    {
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 60.0;
        using Clock = std::chrono::steady_clock;
        auto t0 = Clock::now();
        CircuitCompletionResult r =
            completeCircuitCandidate(track30_60, settings);
        auto t1 = Clock::now();
        double ms = std::chrono::duration<double,
            std::milli>(t1 - t0).count();
        std::fprintf(stdout,
            "%-30s %s %4u %10.6f %8.4f %8.4f %8.4f %8.1fms\n",
            "L=60 default",
            r.success ? "YES" : "NO ",
            r.iterationCount,
            r.finalPositionalGap,
            r.finalTangentErrorDegrees,
            r.finalFrameErrorDegrees,
            r.finalMaxAbsParam,
            ms);
        fflush(stdout);
    }

    // 20m track, 40m connector
    std::fprintf(stdout, "\n--- 20m track, 40m connector ---\n");
    fflush(stdout);
    AuthoredTrack track20 = makeStraightTrack(20.0);
    {
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;
        using Clock = std::chrono::steady_clock;
        auto t0 = Clock::now();
        CircuitCompletionResult r =
            completeCircuitCandidate(track20, settings);
        auto t1 = Clock::now();
        double ms = std::chrono::duration<double,
            std::milli>(t1 - t0).count();
        std::fprintf(stdout,
            "%-30s %s %4u %10.6f %8.4f %8.4f %8.4f %8.1fms\n",
            "20m/40m default",
            r.success ? "YES" : "NO ",
            r.iterationCount,
            r.finalPositionalGap,
            r.finalTangentErrorDegrees,
            r.finalFrameErrorDegrees,
            r.finalMaxAbsParam,
            ms);
        fflush(stdout);
    }

    // 20m track, 60m connector
    std::fprintf(stdout, "\n--- 20m track, 60m connector ---\n");
    fflush(stdout);
    AuthoredTrack track20_60 = makeStraightTrack(20.0);
    {
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 60.0;
        using Clock = std::chrono::steady_clock;
        auto t0 = Clock::now();
        CircuitCompletionResult r =
            completeCircuitCandidate(track20_60, settings);
        auto t1 = Clock::now();
        double ms = std::chrono::duration<double,
            std::milli>(t1 - t0).count();
        std::fprintf(stdout,
            "%-30s %s %4u %10.6f %8.4f %8.4f %8.4f %8.1fms\n",
            "20m/60m default",
            r.success ? "YES" : "NO ",
            r.iterationCount,
            r.finalPositionalGap,
            r.finalTangentErrorDegrees,
            r.finalFrameErrorDegrees,
            r.finalMaxAbsParam,
            ms);
        fflush(stdout);
    }

    return 0;
}
