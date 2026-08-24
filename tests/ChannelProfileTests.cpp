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
    using quantum::coaster::GeometricSection;
    using quantum::coaster::GeometricSectionState;
    using quantum::coaster::invalidSegmentId;
    using quantum::coaster::ProfileSegment;
    using quantum::coaster::SegmentId;
    using quantum::coaster::validateChannelProfile;
    using quantum::coaster::validateGeometricSection;
    using quantum::math::evaluateScalarTransition;
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

    void testSingleSegmentProfileIsValid()
    {
        const ChannelProfile profile = makeChannel({
            {0.0, referenceLength, 0.5, -0.25, TransitionType::Smootherstep}
        });

        validateChannelProfile(profile, referenceLength);

        require(
            profile.segments.front().id == 1,
            "the first created segment receives id one"
        );
        require(
            profile.nextSegmentId == 2,
            "nextSegmentId continues after assigned ids"
        );
    }

    void testMultiSegmentProfileIsValid()
    {
        const ChannelProfile profile = referenceChannel();

        validateChannelProfile(profile, referenceLength);

        require(profile.segments.size() == 3, "reference has three segments");
        require(
            profile.segments[0].id == 1 && profile.segments[1].id == 2
                && profile.segments[2].id == 3,
            "segment ids are stable and sequential"
        );
        require(
            profile.nextSegmentId == 4,
            "nextSegmentId stays available for new segments"
        );
    }

    void testValidationRejectsMalformedProfiles()
    {
        requireThrows<std::invalid_argument>(
            [] { validateChannelProfile(ChannelProfile{}, referenceLength); },
            "empty profile"
        );

        const double invalidLengths[]{
            0.0,
            -1.0,
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity()
        };

        for (const double length : invalidLengths)
        {
            requireThrows<std::invalid_argument>(
                [length]
                { validateChannelProfile(referenceChannel(), length); },
                "invalid expected length"
            );
        }

        ChannelProfile lateStart = referenceChannel();
        lateStart.segments.front().transition.domainBegin = 0.5;
        requireThrows<std::invalid_argument>(
            [lateStart] { validateChannelProfile(lateStart, referenceLength); },
            "profile not starting at zero"
        );

        ChannelProfile shortCoverage = referenceChannel();
        shortCoverage.segments.back().transition.domainEnd = 29.0;
        requireThrows<std::invalid_argument>(
            [shortCoverage]
            { validateChannelProfile(shortCoverage, referenceLength); },
            "profile ending before the expected length"
        );

        ChannelProfile longCoverage = referenceChannel();
        longCoverage.segments.back().transition.domainEnd = 31.0;
        requireThrows<std::invalid_argument>(
            [longCoverage]
            { validateChannelProfile(longCoverage, referenceLength); },
            "profile ending past the expected length"
        );

        ChannelProfile gapped = referenceChannel();
        gapped.segments[1].transition.domainBegin =
            gapped.segments[0].transition.domainEnd + 1.0;
        requireThrows<std::invalid_argument>(
            [gapped] { validateChannelProfile(gapped, referenceLength); },
            "gap between adjacent segments"
        );

        ChannelProfile discontinuous = referenceChannel();
        discontinuous.segments[1].transition.valueBegin =
            discontinuous.segments[0].transition.valueEnd + 0.5;
        requireThrows<std::invalid_argument>(
            [discontinuous]
            { validateChannelProfile(discontinuous, referenceLength); },
            "value discontinuity between adjacent segments"
        );

        ChannelProfile zeroWidth = referenceChannel();
        zeroWidth.segments[1].transition.domainEnd =
            zeroWidth.segments[1].transition.domainBegin;
        requireThrows<std::invalid_argument>(
            [zeroWidth] { validateChannelProfile(zeroWidth, referenceLength); },
            "zero-width segment"
        );

        ChannelProfile reversed = referenceChannel();
        reversed.segments[1].transition.domainBegin =
            reversed.segments[1].transition.domainEnd + 1.0;
        requireThrows<std::invalid_argument>(
            [reversed] { validateChannelProfile(reversed, referenceLength); },
            "reversed segment domain"
        );

        ChannelProfile unsupportedType = referenceChannel();
        unsupportedType.segments[2].transition.transitionType =
            static_cast<TransitionType>(999);
        requireThrows<std::invalid_argument>(
            [unsupportedType]
            { validateChannelProfile(unsupportedType, referenceLength); },
            "unsupported transition type"
        );

        ChannelProfile unassignedId = referenceChannel();
        unassignedId.segments[2].id = invalidSegmentId;
        requireThrows<std::invalid_argument>(
            [unassignedId]
            { validateChannelProfile(unassignedId, referenceLength); },
            "unassigned segment id"
        );

        ChannelProfile duplicateIds = referenceChannel();
        duplicateIds.segments[2].id = duplicateIds.segments[0].id;
        requireThrows<std::invalid_argument>(
            [duplicateIds]
            { validateChannelProfile(duplicateIds, referenceLength); },
            "duplicate segment ids"
        );
    }

    void testValidationRejectsNonFiniteSegments()
    {
        const double nonFiniteValues[]{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()
        };

        for (const double nonFinite : nonFiniteValues)
        {
            ChannelProfile profile = referenceChannel();

            profile.segments[1].transition.domainBegin = nonFinite;
            requireThrows<std::invalid_argument>(
                [profile]
                { validateChannelProfile(profile, referenceLength); },
                "non-finite segment domain beginning"
            );

            profile = referenceChannel();
            profile.segments[1].transition.domainEnd = nonFinite;
            requireThrows<std::invalid_argument>(
                [profile]
                { validateChannelProfile(profile, referenceLength); },
                "non-finite segment domain end"
            );

            profile = referenceChannel();
            profile.segments[1].transition.valueBegin = nonFinite;
            requireThrows<std::invalid_argument>(
                [profile]
                { validateChannelProfile(profile, referenceLength); },
                "non-finite segment value beginning"
            );

            profile = referenceChannel();
            profile.segments[1].transition.valueEnd = nonFinite;
            requireThrows<std::invalid_argument>(
                [profile]
                { validateChannelProfile(profile, referenceLength); },
                "non-finite segment value end"
            );
        }
    }

    void testEvaluationMatchesPerSegmentScalars()
    {
        const ChannelProfile profile = referenceChannel();

        struct Query
        {
            double distance;
            std::size_t segmentIndex;
        };

        const Query queries[]{
            {0.0, 0},
            {2.5, 0},
            {7.5, 0},
            {10.0, 0},
            {12.5, 1},
            {17.5, 1},
            {20.0, 1},
            {22.5, 2},
            {27.5, 2},
            {30.0, 2}
        };

        for (const Query& query : queries)
        {
            const double expected = evaluateScalarTransition(
                profile.segments[query.segmentIndex].transition,
                query.distance
            );
            const double actual =
                evaluateChannelProfile(profile, query.distance);

            require(
                actual == expected,
                "channel evaluation matches the containing segment scalar"
            );
        }
    }

    void testBoundaryEvaluationIsContinuous()
    {
        const ChannelProfile profile = referenceChannel();

        constexpr double joints[] = {10.0, 20.0};

        for (const double joint : joints)
        {
            // A query landing exactly on the joint evaluates to the shared
            // authored endpoint value on either side.
            const double jointValue = evaluateChannelProfile(profile, joint);
            require(
                jointValue == profile.segments[0].transition.valueEnd,
                "joint query matches the left segment's end value"
            );
            require(
                jointValue == profile.segments[1].transition.valueBegin,
                "joint query matches the right segment's begin value"
            );

            // One-representable-step neighbors agree to within the rate
            // scale: C0 continuity guarantees the limit, not bit equality.
            const double below = std::nextafter(
                joint,
                -std::numeric_limits<double>::infinity()
            );
            const double above = std::nextafter(
                joint,
                std::numeric_limits<double>::infinity()
            );

            const double leftValue = evaluateChannelProfile(profile, below);
            const double rightValue = evaluateChannelProfile(profile, above);

            require(
                std::abs(leftValue - jointValue) <= 1e-9
                    && std::abs(rightValue - jointValue) <= 1e-9,
                "boundary queries agree across the C0 joint"
            );
        }
    }

    void testEvaluationRejectsInvalidQueries()
    {
        const ChannelProfile profile = referenceChannel();

        requireThrows<std::out_of_range>(
            [&profile]
            { static_cast<void>(evaluateChannelProfile(profile, -0.01)); },
            "query below the covered domain"
        );
        requireThrows<std::out_of_range>(
            [&profile]
            { static_cast<void>(evaluateChannelProfile(profile, 30.01)); },
            "query above the covered domain"
        );

        const double nonFiniteValues[]{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()
        };

        for (const double nonFinite : nonFiniteValues)
        {
            requireThrows<std::invalid_argument>(
                [&profile, nonFinite]
                { static_cast<void>(evaluateChannelProfile(profile, nonFinite)); },
                "non-finite query distance"
            );
        }
    }

    void testGeometricSectionWrapsThreeChannels()
    {
        GeometricSection section;
        section.pitch = makeChannel({
            {0.0, referenceLength, 0.0, 1.0, TransitionType::Linear}
        });
        section.yaw = referenceChannel();
        section.roll = makeChannel({
            {0.0, referenceLength / 2.0, -0.5, -0.5, TransitionType::Linear},
            {referenceLength / 2.0,
             referenceLength,
             -0.5,
             0.5,
             TransitionType::Smoothstep}
        });

        validateGeometricSection(section, referenceLength);

        const GeometricSectionState state = evaluateGeometricSection(
            section,
            referenceLength,
            25.0
        );

        require(
            state.pitch == evaluateChannelProfile(section.pitch, 25.0),
            "geometric-section pitch delegates to the channel evaluator"
        );
        require(
            state.yaw == evaluateChannelProfile(section.yaw, 25.0),
            "geometric-section yaw delegates to the channel evaluator"
        );
        require(
            state.roll == evaluateChannelProfile(section.roll, 25.0),
            "geometric-section roll delegates to the channel evaluator"
        );

        requireThrows<std::invalid_argument>(
            [&section]
            {
                validateGeometricSection(section, referenceLength + 1.0);
            },
            "section validation rejects a mismatched length"
        );

        section.roll.segments[1].transition.valueBegin = 42.0;
        requireThrows<std::invalid_argument>(
            [&section]
            { validateGeometricSection(section, referenceLength); },
            "section validation propagates channel defects"
        );
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;

    const std::vector<Test> tests{
        {"single-segment profile is valid", testSingleSegmentProfileIsValid},
        {"multi-segment profile is valid", testMultiSegmentProfileIsValid},
        {
            "validation rejects malformed profiles",
            testValidationRejectsMalformedProfiles
        },
        {
            "validation rejects non-finite segments",
            testValidationRejectsNonFiniteSegments
        },
        {
            "evaluation matches per-segment scalars",
            testEvaluationMatchesPerSegmentScalars
        },
        {"boundary evaluation is continuous", testBoundaryEvaluationIsContinuous},
        {
            "evaluation rejects invalid queries",
            testEvaluationRejectsInvalidQueries
        },
        {
            "geometric section wraps three channels",
            testGeometricSectionWrapsThreeChannels
        }
    };

    int failures = 0;

    for (const auto& [name, test] : tests)
    {
        try
        {
            test();
            std::cout << "[PASS] " << name << '\n';
        }
        catch (const std::exception& exception)
        {
            ++failures;
            std::cerr << "[FAIL] " << name << ": "
                      << exception.what() << '\n';
        }
        catch (...)
        {
            ++failures;
            std::cerr << "[FAIL] " << name
                      << ": unknown exception\n";
        }
    }

    if (failures != 0)
    {
        std::cerr << failures << " test group(s) failed.\n";
        return 1;
    }

    std::cout << tests.size() << " test groups passed.\n";
    return 0;
}
