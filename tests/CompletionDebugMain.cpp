#include <quantum/coaster/CircuitCompletion.hpp>
#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <cstdio>
#include <exception>

using namespace quantum::coaster;

static AuthoredTrack makeStraightTrack(double length)
{
    AuthoredTrack track;
    track.setLayoutMode(LayoutMode::Circuit);
    track.prependSection();

    AuthoredTrackSection section;
    section.kind = RegionKind::RateProfiles;
    section.length = length;

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
    profiles.pitch = makeZero(length);
    profiles.yaw = makeZero(length);
    profiles.roll = makeZero(length);
    section.region = RateProfileRegion{profiles};
    track.section(0) = std::move(section);
    return track;
}

int main()
{
    try
    {
        AuthoredTrack track = makeStraightTrack(30.0);

        double lengths[] = {40.0, 50.0, 60.0, 80.0, 100.0};
        for (double len : lengths)
        {
            CircuitCompletionSettings settings;
            settings.preferredConnectorLength = len;
            CircuitCompletionResult result =
                completeCircuitCandidate(track, settings);
            fprintf(stderr,
                "[L=%.0f] success=%d reason=%d gap=%.4f "
                "tang=%.4f frame=%.4f iter=%u msg='%s'\n",
                len, result.success ? 1 : 0,
                static_cast<int>(result.failureReason),
                result.finalPositionalGap,
                result.finalTangentErrorDegrees,
                result.finalFrameErrorDegrees,
                result.iterationCount,
                result.failureMessage.c_str());
            fflush(stderr);
        }
        return 0;
    }
    catch (const std::exception& e)
    {
        fprintf(stderr, "EXCEPTION: %s\n", e.what());
        return 2;
    }
}
