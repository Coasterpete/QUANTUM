#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/GeometricSection.hpp>

#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::ChannelProfile;
    using quantum::coaster::evaluateChannelProfile;
    using quantum::coaster::evaluateGeometricSection;
    using quantum::coaster::findChannelSegmentAtDistance;
    using quantum::coaster::findChannelSegmentTransition;
    using quantum::coaster::GeometricSection;
    using quantum::coaster::GeometricSectionState;
    using quantum::coaster::invalidSegmentId;
    using quantum::coaster::moveChannelSegmentBoundary;
    using quantum::coaster::ProfileBoundary;
    using quantum::coaster::ProfileSegment;
    using quantum::coaster::removeChannelSegment;
    using quantum::coaster::SegmentId;
    using quantum::coaster::setChannelSegmentValue;
    using quantum::coaster::splitChannelSegment;
    using quantum::coaster::validateChannelProfile;
    using quantum::coaster::validateGeometricSection;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

    class TestFailure final : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw TestFailure(std::string(message));
        }
    }

    template<typename ExpectedException, typename Function>
    void requireThrows(Function&& function, const std::string_view context)
    {
        try
        {
            std::forward<Function>(function)();
        }
        catch (const ExpectedException&)
        {
            return;
        }
        catch (const std::exception& exception)
        {
            throw TestFailure(
                std::string(context) + ": unexpected exception: "
                + exception.what()
            );
        }

        throw TestFailure(
            std::string(context) + ": expected exception was not thrown"
        );
    }

    struct SegmentSpec
    {
        double domainBegin;
        double domainEnd;
        double valueBegin;
        double valueEnd;
        TransitionType type;
    };

    // Builds a channel whose segments receive stable ids 1..n and whose
    // nextSegmentId continues after the last assigned id.
    [[nodiscard]] ChannelProfile makeChannel(
        const std::vector<SegmentSpec>& segments
    )
    {
        ChannelProfile profile;
        SegmentId nextId = 1;

        for (const SegmentSpec& spec : segments)
        {
            profile.segments.push_back(ProfileSegment{
                nextId,
                ScalarTransition{
                    spec.domainBegin,
                    spec.domainEnd,
                    spec.valueBegin,
                    spec.valueEnd,
                    spec.type
                }
            });
            ++nextId;
        }

        profile.nextSegmentId = nextId;
        return profile;
    }

    // Three-span reference channel over [0, 30]: rising ramp, constant
    // plateau, falling cosine ease. Interior joints are C0 at value 1.
    [[nodiscard]] ChannelProfile referenceChannel()
    {
        return makeChannel({
            {0.0, 10.0, 0.0, 1.0, TransitionType::Linear},
            {10.0, 20.0, 1.0, 1.0, TransitionType::Smoothstep},
            {20.0, 30.0, 1.0, -2.0, TransitionType::CosineEaseInOut}
        });
    }

    constexpr double referenceLength = 30.0;

    [[nodiscard]] const ProfileSegment& segmentById(
        const ChannelProfile& profile,
        const SegmentId id)
    {
        for (const ProfileSegment& segment : profile.segments)
        {
            if (segment.id == id)
            {
                return segment;
            }
        }

        throw TestFailure("segment id not found while testing");
    }

    void testFindSegmentAtDistance()
    {
        const ChannelProfile profile = referenceChannel();

        require(
            findChannelSegmentAtDistance(profile, -1.0) == invalidSegmentId,
            "distance before the chain resolves to no segment"
        );
        require(
            findChannelSegmentAtDistance(profile, 31.0) == invalidSegmentId,
            "distance past the chain resolves to no segment"
        );
        require(
            findChannelSegmentAtDistance(profile,
                std::numeric_limits<double>::quiet_NaN())
                == invalidSegmentId,
            "a non-finite distance resolves to no segment"
        );
        require(
            findChannelSegmentAtDistance(profile, 0.0) == 1,
            "section start belongs to the first segment"
        );
        require(
            findChannelSegmentAtDistance(profile, 4.5) == 1,
            "interior distance resolves inside the first span"
        );
        require(
            findChannelSegmentAtDistance(profile, 10.0) == 1,
            "an exact joint resolves to the left segment"
        );
        require(
            findChannelSegmentAtDistance(profile, 20.0) == 2,
            "the second joint also resolves left"
        );
        require(
            findChannelSegmentAtDistance(profile, 30.0) == 3,
            "section end belongs to the last segment"
        );

        ChannelProfile empty{};
        require(
            findChannelSegmentAtDistance(empty, 0.0) == invalidSegmentId,
            "an empty channel resolves to no segment"
        );
    }

    void testSplitPreservesCoverageContinuityAndIds()
    {
        ChannelProfile profile = referenceChannel();

        const SegmentId rightId = splitChannelSegment(profile, 2, 15.0);

        require(rightId == profile.nextSegmentId - 1,
            "split hands out the pending next segment id");
        require(profile.nextSegmentId == 5,
            "split consumes exactly one fresh id");
        require(profile.segments.size() == 4,
            "split inserts one additional segment");

        const ProfileSegment& left = segmentById(profile, 2);
        const ProfileSegment& right = segmentById(profile, rightId);

        require(left.transition.domainBegin == 10.0
                && left.transition.domainEnd == 15.0,
            "the left piece keeps the original begin and ends at the split");
        require(right.transition.domainBegin == 15.0
                && right.transition.domainEnd == 20.0,
            "the right piece starts at the split and keeps the original end");
        require(left.id == 2,
            "the left piece keeps the original stable id");

        // Smoothstep at the midpoint of [10, 20] evaluates to 1.0 on both
        // pieces because the plateau holds value 1 throughout.
        require(left.transition.valueEnd == 1.0
                && right.transition.valueBegin == 1.0,
            "both pieces carry the interpolated split value");
        require(left.transition.transitionType
                == TransitionType::Smoothstep
            && right.transition.transitionType
                == TransitionType::Smoothstep,
            "both pieces keep the original transition type");

        validateChannelProfile(profile, referenceLength);
        require(evaluateChannelProfile(profile, 14.999999) <= 1.0
                && evaluateChannelProfile(profile, 15.000001) >= 1.0,
            "evaluation stays continuous across the inserted joint");

        // Splitting again must keep every existing id stable.
        const SegmentId thirdId = splitChannelSegment(profile, rightId, 17.5);
        require(thirdId == 5 && profile.nextSegmentId == 6,
            "repeated splits keep handing out fresh ids");
        require(segmentById(profile, 2).transition.domainBegin == 10.0,
            "earlier pieces are untouched by later splits");
        validateChannelProfile(profile, referenceLength);

        requireThrows<std::invalid_argument>(
            [&] { (void)splitChannelSegment(profile, 2, 10.0); },
            "splitting exactly at a segment begin is refused");
        requireThrows<std::invalid_argument>(
            [&] { (void)splitChannelSegment(profile, 2, 15.0); },
            "splitting exactly at a segment end is refused");
        requireThrows<std::invalid_argument>(
            [&] { (void)splitChannelSegment(profile, 2,
                std::numeric_limits<double>::infinity()); },
            "non-finite split distances are refused");
        requireThrows<std::invalid_argument>(
            [&] { (void)splitChannelSegment(profile, 99, 12.0); },
            "unknown segment ids are refused");
    }

    void testRemoveMergesIntoNeighbour()
    {
        // Removing a middle segment merges into the previous one.
        ChannelProfile profile = referenceChannel();
        const SegmentId survivor = removeChannelSegment(profile, 2);

        require(survivor == 1,
            "removing a middle segment leaves the previous one standing");
        require(profile.segments.size() == 2,
            "removal shrinks the chain to two segments");
        const ProfileSegment& merged = segmentById(profile, survivor);
        require(merged.transition.domainBegin == 0.0
                && merged.transition.domainEnd == 20.0,
            "the survivor spans its own begin through the removed end");
        require(merged.transition.valueBegin == 0.0
                && merged.transition.valueEnd == 1.0,
            "the survivor keeps its own leading values");
        require(segmentById(profile, 3).transition.domainBegin == 20.0,
            "the following segment is untouched");
        validateChannelProfile(profile, referenceLength);

        // Removing the first segment merges the next one backwards.
        profile = referenceChannel();
        const SegmentId firstSurvivor = removeChannelSegment(profile, 1);
        require(firstSurvivor == 2,
            "removing the first segment promotes its neighbour");
        const ProfileSegment& promoted = segmentById(profile, firstSurvivor);
        require(promoted.transition.domainBegin == 0.0
                && promoted.transition.valueBegin == 0.0,
            "the promoted segment inherits the section start");
        require(promoted.transition.domainEnd == 20.0
                && promoted.transition.valueEnd == 1.0,
            "the promoted segment keeps its own end");
        validateChannelProfile(profile, referenceLength);

        // Removing the last segment merges into the previous one.
        profile = referenceChannel();
        const SegmentId lastSurvivor = removeChannelSegment(profile, 3);
        require(lastSurvivor == 2,
            "removing the last segment extends its predecessor");
        require(segmentById(profile, lastSurvivor).transition.domainEnd
                == 30.0,
            "the extended predecessor reaches the section end");
        validateChannelProfile(profile, referenceLength);

        requireThrows<std::invalid_argument>(
            [] {
                ChannelProfile single = makeChannel({
                    {0.0, 30.0, 0.0, 0.0, TransitionType::Linear}
                });
                (void)removeChannelSegment(single, 1);
            },
            "the final remaining segment cannot be removed");
        requireThrows<std::invalid_argument>(
            [&] { (void)removeChannelSegment(profile, 42); },
            "unknown segment ids cannot be removed");
    }

    void testMoveBoundaryKeepsContiguityAndPins()
    {
        ChannelProfile profile = referenceChannel();

        moveChannelSegmentBoundary(
            profile, 2, ProfileBoundary::Begin, 6.0);
        require(
            segmentById(profile, 1).transition.domainEnd == 6.0
                && segmentById(profile, 2).transition.domainBegin == 6.0,
            "moving a begin boundary drags the adjoining end with it");
        validateChannelProfile(profile, referenceLength);

        moveChannelSegmentBoundary(profile, 2, ProfileBoundary::End, 25.0);
        require(
            segmentById(profile, 2).transition.domainEnd == 25.0
                && segmentById(profile, 3).transition.domainBegin == 25.0,
            "moving an end boundary drags the adjoining begin with it");
        require(segmentById(profile, 2).transition.valueBegin == 1.0,
            "boundary moves never touch authored values");
        validateChannelProfile(profile, referenceLength);

        // Refusals run against a fresh reference chain so every check sees
        // the canonical boundaries [0,10], [10,20], [20,30].
        ChannelProfile fresh = referenceChannel();
        requireThrows<std::invalid_argument>(
            [&] { moveChannelSegmentBoundary(
                fresh, 2, ProfileBoundary::Begin, 20.0); },
            "a boundary cannot collapse the span it bounds");
        requireThrows<std::invalid_argument>(
            [&] { moveChannelSegmentBoundary(
                fresh, 3, ProfileBoundary::Begin, 9.9); },
            "crossing the previous neighbour's boundary is refused");
        requireThrows<std::invalid_argument>(
            [&] { moveChannelSegmentBoundary(
                fresh, 2, ProfileBoundary::End, 30.0); },
            "an interior joint cannot reach the section end");
        requireThrows<std::invalid_argument>(
            [&] { moveChannelSegmentBoundary(
                fresh, 1, ProfileBoundary::Begin, 1.0); },
            "the section start boundary is pinned");
        requireThrows<std::invalid_argument>(
            [&] { moveChannelSegmentBoundary(
                fresh, 3, ProfileBoundary::End, 29.0); },
            "the section end boundary is pinned");
        requireThrows<std::invalid_argument>(
            [&] { moveChannelSegmentBoundary(
                fresh, 2, ProfileBoundary::Begin,
                std::numeric_limits<double>::quiet_NaN()); },
            "non-finite boundary distances are refused");

        // A two-segment chain pins the interior joint between the fixed
        // section bounds only.
        ChannelProfile pair = makeChannel({
            {0.0, 10.0, 0.0, 1.0, TransitionType::Linear},
            {10.0, 30.0, 1.0, -1.0, TransitionType::CosineEaseInOut}
        });
        moveChannelSegmentBoundary(pair, 2, ProfileBoundary::Begin, 29.0);
        validateChannelProfile(pair, 30.0);
        requireThrows<std::invalid_argument>(
            [&] { moveChannelSegmentBoundary(
                pair, 2, ProfileBoundary::Begin, 30.0); },
            "the interior joint cannot reach the section end");
    }

    void testSetValuePropagatesAcrossSharedBoundaries()
    {
        ChannelProfile profile = referenceChannel();

        setChannelSegmentValue(
            profile, 2, ProfileBoundary::Begin, 0.75);
        require(segmentById(profile, 2).transition.valueBegin == 0.75,
            "the addressed endpoint receives the new value");
        require(segmentById(profile, 1).transition.valueEnd == 0.75,
            "the previous segment's matching endpoint follows");
        require(segmentById(profile, 2).transition.valueEnd == 1.0,
            "unrelated endpoints stay untouched");
        validateChannelProfile(profile, referenceLength);

        setChannelSegmentValue(profile, 3, ProfileBoundary::Begin, -0.5);
        require(segmentById(profile, 2).transition.valueEnd == -0.5,
            "setting a begin propagates backwards across the joint");

        setChannelSegmentValue(profile, 1, ProfileBoundary::Begin, 2.0);
        require(segmentById(profile, 1).transition.valueBegin == 2.0,
            "the section start value is set directly");

        setChannelSegmentValue(profile, 3, ProfileBoundary::End, -3.0);
        require(segmentById(profile, 3).transition.valueEnd == -3.0,
            "the section end value is set directly");

        requireThrows<std::invalid_argument>(
            [&] { setChannelSegmentValue(profile, 2,
                ProfileBoundary::End,
                std::numeric_limits<double>::infinity()); },
            "non-finite profile values are refused");
        requireThrows<std::invalid_argument>(
            [&] { setChannelSegmentValue(profile, 77,
                ProfileBoundary::End, 1.0); },
            "unknown segment ids are refused");
        validateChannelProfile(profile, referenceLength);
    }

    void testStableIdsSurviveEditingSequences()
    {
        ChannelProfile profile = makeChannel({
            {0.0, 30.0, 0.0, 1.0, TransitionType::Linear}
        });

        const SegmentId second = splitChannelSegment(profile, 1, 10.0);
        const SegmentId third = splitChannelSegment(profile, second, 20.0);
        require(profile.nextSegmentId == 4,
            "two splits consume two fresh ids");

        (void)removeChannelSegment(profile, second);
        require(profile.segments.size() == 2,
            "removing the middle piece leaves two spans");
        require(findChannelSegmentTransition(profile, second) == nullptr,
            "removed ids no longer resolve");
        require(segmentById(profile, 1).transition.domainEnd == 20.0,
            "the first piece absorbed the removed span");
        require(segmentById(profile, third).transition.domainBegin == 20.0,
            "the surviving fresh id keeps its identity");
        require(profile.nextSegmentId == 4,
            "ids are never reused after removal");

        // A subsequent split continues after the highest consumed id even
        // though some ids were freed by removals.
        const SegmentId fourth = splitChannelSegment(profile, third, 25.0);
        require(fourth == 4 && profile.nextSegmentId == 5,
            "fresh ids continue past all previously consumed ids");
        validateChannelProfile(profile, referenceLength);
    }

    void testChannelsStayIndependentInsideASection()
    {
        GeometricSection section;
        section.pitch = referenceChannel();
        section.yaw = makeChannel({
            {0.0, 30.0, 0.0, 0.0, TransitionType::Linear}
        });
        section.roll = referenceChannel();

        (void)splitChannelSegment(section.pitch, 2, 15.0);
        setChannelSegmentValue(
            section.roll, 1, ProfileBoundary::Begin, 0.25);

        validateGeometricSection(section, referenceLength);

        require(section.yaw.segments.size() == 1,
            "editing one channel never adds segments to another");
        require(section.yaw.segments.front().transition.valueBegin == 0.0
                && section.yaw.segments.front().transition.valueEnd == 0.0,
            "editing one channel never touches another channel's values");

        const GeometricSectionState state = evaluateGeometricSection(
            section,
            referenceLength,
            7.5
        );
        require(state.pitch == 0.75,
            "edited pitch evaluation reflects the split chain");
        require(state.roll == 0.8125,
            "edited roll start value feeds through evaluation");
        require(state.yaw == 0.0,
            "untouched yaw evaluation stays flat");
    }
}

int main()
{
    const std::pair<std::string_view, void(*)()> tests[] = {
        {"find segment at distance", testFindSegmentAtDistance},
        {"split preserves coverage continuity and ids",
            testSplitPreservesCoverageContinuityAndIds},
        {"remove merges into neighbour", testRemoveMergesIntoNeighbour},
        {"move boundary keeps contiguity and pins",
            testMoveBoundaryKeepsContiguityAndPins},
        {"set value propagates across shared boundaries",
            testSetValuePropagatesAcrossSharedBoundaries},
        {"stable ids survive editing sequences",
            testStableIdsSurviveEditingSequences},
        {"channels stay independent inside a section",
            testChannelsStayIndependentInsideASection}
    };

    for (const auto& [name, test] : tests)
    {
        try
        {
            test();
            std::cout << "[PASS] " << name << '\n';
        }
        catch (const std::exception& exception)
        {
            std::cerr << "[FAIL] " << name << ": "
                << exception.what() << '\n';
            return 1;
        }
    }

    std::cout << "All ChannelProfileEditing tests passed.\n";
    return 0;
}
