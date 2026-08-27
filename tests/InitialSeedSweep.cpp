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
        section.region =
            RateProfileRegion{profiles};

        track.section(0) = std::move(section);
        return track;
    }
}

int main()
{
    using Clock = std::chrono::steady_clock;

    AuthoredTrack track = makeStraightTrack(30.0);

    // --- Config A: pitch + yaw seeded, roll zero ---
    // For the along-tangent case (30 m straight track,
    // 40 m connector), the gap is in -x.
    // The connector needs yaw to curve back and pitch for 3D
    // conditioning.

    const double seeds[] = {
        0.001, 0.0025, 0.005, 0.01, 0.02, 0.05, 0.1
    };

    std::fprintf(stdout,
        "%-8s %-5s %5s %10s %8s %8s %8s %8s %8s %10s\n",
        "seed", "ok", "iter", "posGap",
        "tangErr", "frameErr", "maxP", "cond",
        "reject?", "time_ms");
    std::fprintf(stdout,
        "-------- ----- ----- "
        "---------- -------- -------- "
        "-------- -------- -------- ----------\n");
    fflush(stdout);

    for (const double seed : seeds)
    {
        // Pitch seeded positively (connector pitches up
        // slightly), yaw seeded negatively (curves back
        // toward start).  Signs chosen by geometry: the
        // gap is -x along the tangent, so the connector
        // must yaw toward -y then back.
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;
        settings.initialParamOverride =
            std::array<double, 6>{
                seed, seed,
                -seed, -seed,
                0.0, 0.0};

        auto t0 = Clock::now();
        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        auto t1 = Clock::now();

        const double timeMs =
            std::chrono::duration<double,
                std::milli>(t1 - t0).count();

        std::fprintf(stdout,
            "%-8.4f %-5s %5u %10.6f %8.4f %8.4f "
            "%8.4f %8s %8s %10.1f\n",
            seed,
            result.success ? "yes" : "NO",
            result.iterationCount,
            result.finalPositionalGap,
            result.finalTangentErrorDegrees,
            result.finalFrameErrorDegrees,
            result.finalMaxAbsParam,
            "-",  // condition reported by solver stderr
            result.success ? "" : "FAIL",
            timeMs);
        fflush(stdout);
    }

    // --- Config B: pitch + yaw + roll seeded ---
    std::fprintf(stdout,
        "\n--- Config B: pitch + yaw + roll seeded ---\n");
    fflush(stdout);

    for (const double seed : seeds)
    {
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;
        settings.initialParamOverride =
            std::array<double, 6>{
                seed, seed,
                -seed, -seed,
                seed * 0.5, -seed * 0.5};

        auto t0 = Clock::now();
        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        auto t1 = Clock::now();

        const double timeMs =
            std::chrono::duration<double,
                std::milli>(t1 - t0).count();

        std::fprintf(stdout,
            "%-8.4f %-5s %5u %10.6f %8.4f %8.4f "
            "%8.4f %8s %8s %10.1f\n",
            seed,
            result.success ? "yes" : "NO",
            result.iterationCount,
            result.finalPositionalGap,
            result.finalTangentErrorDegrees,
            result.finalFrameErrorDegrees,
            result.finalMaxAbsParam,
            "-",
            result.success ? "" : "FAIL",
            timeMs);
        fflush(stdout);
    }

    // --- Config C: yaw-only (baseline) ---
    std::fprintf(stdout,
        "\n--- Config C: yaw-only ---\n");
    fflush(stdout);

    for (const double seed : seeds)
    {
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;
        settings.initialParamOverride =
            std::array<double, 6>{
                0.0, 0.0,
                -seed, -seed,
                0.0, 0.0};

        auto t0 = Clock::now();
        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        auto t1 = Clock::now();

        const double timeMs =
            std::chrono::duration<double,
                std::milli>(t1 - t0).count();

        std::fprintf(stdout,
            "%-8.4f %-5s %5u %10.6f %8.4f %8.4f "
            "%8.4f %8s %8s %10.1f\n",
            seed,
            result.success ? "yes" : "NO",
            result.iterationCount,
            result.finalPositionalGap,
            result.finalTangentErrorDegrees,
            result.finalFrameErrorDegrees,
            result.finalMaxAbsParam,
            "-",
            result.success ? "" : "FAIL",
            timeMs);
        fflush(stdout);
    }

    // --- Config D: default heuristic (no override) ---
    std::fprintf(stdout,
        "\n--- Config D: default heuristic ---\n");
    fflush(stdout);

    {
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;

        auto t0 = Clock::now();
        CircuitCompletionResult result =
            completeCircuitCandidate(track, settings);
        auto t1 = Clock::now();

        const double timeMs =
            std::chrono::duration<double,
                std::milli>(t1 - t0).count();

        std::fprintf(stdout,
            "%-8s %-5s %5u %10.6f %8.4f %8.4f "
            "%8.4f %8s %8s %10.1f\n",
            "default",
            result.success ? "yes" : "NO",
            result.iterationCount,
            result.finalPositionalGap,
            result.finalTangentErrorDegrees,
            result.finalFrameErrorDegrees,
            result.finalMaxAbsParam,
            "-",
            result.success ? "" : "FAIL",
            timeMs);
        fflush(stdout);
    }

    return 0;
}
