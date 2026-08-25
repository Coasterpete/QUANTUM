#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/CircuitCompletion.hpp>
#include <quantum/coaster/GeometricSection.hpp>
#include <quantum/coaster/RiderLocalGeometry.hpp>
#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/math/ScalarTransition.hpp>

#include <array>
#include <cstdio>

int main()
{
    using namespace quantum::coaster;

    AuthoredTrack track;
    track.setLayoutMode(LayoutMode::Circuit);
    track.prependSection();

    AuthoredTrackSection section;
    section.kind = RegionKind::RateProfiles;
    section.length = 30.0;

    auto makeZero = [](double len)
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
    profiles.pitch = makeZero(30.0);
    profiles.yaw = makeZero(30.0);
    profiles.roll = makeZero(30.0);
    section.region = RateProfileRegion{profiles};
    track.section(0) = std::move(section);

    CircuitCompletionSettings settings;
    settings.preferredConnectorLength = 40.0;

    fprintf(stdout, "=== Default heuristic (30m/40m) ===\n");
    fflush(stdout);
    CircuitCompletionResult result =
        completeCircuitCandidate(track, settings);
    fprintf(stdout, "RESULT: %s iter=%u posGap=%.6f tang=%.4f frame=%.4f\n",
        result.success ? "OK" : "FAIL",
        result.iterationCount,
        result.finalPositionalGap,
        result.finalTangentErrorDegrees,
        result.finalFrameErrorDegrees);
    fflush(stdout);

    return 0;
}
