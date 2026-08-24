#include <quantum/coaster/ForceSection.hpp>
#include <quantum/coaster/GeometricSection.hpp>

#include <bit>
#include <cmath>
#include <cstdint>
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
    using quantum::coaster::ForceSection;
    using quantum::coaster::GeometricSection;
    using quantum::coaster::GeometricSectionState;
    using quantum::coaster::ProfileSegment;
    using quantum::coaster::validateForceSection;
    using quantum::coaster::validateGeometricSection;
    using quantum::math::evaluateScalarTransition;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

    // Canonical section-local distance-domain length of the reference
    // geometric section below.
    constexpr double referenceSectionLength = 20.75;

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

    [[nodiscard]] constexpr ScalarTransition transition(
        const double valueBegin,
        const double valueEnd,
        const TransitionType type,
        const double domainBegin = -7.25,
        const double domainEnd = 13.5
    )
    {
        return {
            domainBegin,
            domainEnd,
            valueBegin,
            valueEnd,
            type
        };
    }

    // Single-segment channel covering [domainBegin, domainEnd] exactly.
    [[nodiscard]] ChannelProfile singleSegmentChannel(
        const double valueBegin,
        const double valueEnd,
        const TransitionType type,
        const double domainBegin = 0.0,
        const double domainEnd = referenceSectionLength
    )
    {
        ChannelProfile profile;
        profile.segments.push_back(ProfileSegment{
            profile.nextSegmentId,
            ScalarTransition{
                domainBegin,
                domainEnd,
                valueBegin,
                valueEnd,
                type
            }
        });
        ++profile.nextSegmentId;
        return profile;
    }

    [[nodiscard]] constexpr ForceSection validForceSection()
    {
        return {
            transition(-1.0, 4.0, TransitionType::Linear),
            transition(3.0, -2.0, TransitionType::Smoothstep),
            transition(0.25, 0.25, TransitionType::Smootherstep)
        };
    }

    [[nodiscard]] GeometricSection validGeometricSection()
    {
        return {
            singleSegmentChannel(-0.2, 0.4, TransitionType::Smoothstep),
            singleSegmentChannel(0.3, -0.1, TransitionType::Smootherstep),
            singleSegmentChannel(-0.5, -0.5, TransitionType::Linear)
        };
    }

    void requireTransitionRetained(
        const ScalarTransition& actual,
        const ScalarTransition& expected,
        const std::string_view context
    )
    {
        require(
            actual.domainBegin == expected.domainBegin,
            std::string(context) + " domain beginning"
        );
        require(
            actual.domainEnd == expected.domainEnd,
            std::string(context) + " domain end"
        );
        require(
            actual.valueBegin == expected.valueBegin,
            std::string(context) + " value beginning"
        );
        require(
            actual.valueEnd == expected.valueEnd,
            std::string(context) + " value end"
        );
        require(
            actual.transitionType == expected.transitionType,
            std::string(context) + " transition type"
        );
    }

    void requireState(
        const GeometricSectionState& actual,
        const GeometricSectionState& expected,
        const std::string_view context
    )
    {
        require(
            actual.pitch == expected.pitch,
            std::string(context) + " pitch"
        );
        require(
            actual.yaw == expected.yaw,
            std::string(context) + " yaw"
        );
        require(
            actual.roll == expected.roll,
            std::string(context) + " roll"
        );
    }

    void testForceSectionRepresentation()
    {
        constexpr ForceSection section = validForceSection();

        validateForceSection(section);

        requireTransitionRetained(
            section.verticalForce,
            transition(-1.0, 4.0, TransitionType::Linear),
            "vertical force"
        );
        requireTransitionRetained(
            section.lateralForce,
            transition(3.0, -2.0, TransitionType::Smoothstep),
            "lateral force"
        );
        requireTransitionRetained(
            section.roll,
            transition(0.25, 0.25, TransitionType::Smootherstep),
            "force-section roll"
        );
    }

    void testGeometricSectionRepresentation()
    {
        const GeometricSection section = validGeometricSection();

        validateGeometricSection(section, referenceSectionLength);

        requireTransitionRetained(
            section.pitch.segments.front().transition,
            transition(
                -0.2,
                0.4,
                TransitionType::Smoothstep,
                0.0,
                referenceSectionLength
            ),
            "pitch"
        );
        requireTransitionRetained(
            section.yaw.segments.front().transition,
            transition(
                0.3,
                -0.1,
                TransitionType::Smootherstep,
                0.0,
                referenceSectionLength
            ),
            "yaw"
        );
        requireTransitionRetained(
            section.roll.segments.front().transition,
            transition(
                -0.5,
                -0.5,
                TransitionType::Linear,
                0.0,
                referenceSectionLength
            ),
            "geometric-section roll"
        );

        requireThrows<std::invalid_argument>(
            [&section]
            {
                validateGeometricSection(
                    section,
                    referenceSectionLength + 1.0
                );
            },
            "section validation rejects a mismatched length"
        );
    }

    void testForceSectionSharedDomain()
    {
        validateForceSection(validForceSection());

        ForceSection mismatchedBeginning = validForceSection();
        mismatchedBeginning.lateralForce.domainBegin = std::nextafter(
            mismatchedBeginning.verticalForce.domainBegin,
            mismatchedBeginning.verticalForce.domainEnd
        );
        requireThrows<std::invalid_argument>(
            [&mismatchedBeginning]
            {
                validateForceSection(mismatchedBeginning);
            },
            "force-section smallest-representable beginning mismatch"
        );

        ForceSection mismatchedEnd = validForceSection();
        mismatchedEnd.roll.domainEnd = std::nextafter(
            mismatchedEnd.verticalForce.domainEnd,
            std::numeric_limits<double>::infinity()
        );
        requireThrows<std::invalid_argument>(
            [&mismatchedEnd]
            {
                validateForceSection(mismatchedEnd);
            },
            "force-section smallest-representable end mismatch"
        );
    }

    void testGeometricSectionSharedLength()
    {
        validateGeometricSection(validGeometricSection(), referenceSectionLength);

        // Every channel must cover [0, length] exactly, so shrinking one
        // channel's coverage by even one representable step is rejected.
        GeometricSection shortRoll = validGeometricSection();
        shortRoll.roll.segments.back().transition.domainEnd =
            std::nextafter(
                shortRoll.roll.segments.back().transition.domainEnd,
                -std::numeric_limits<double>::infinity()
            );
        requireThrows<std::invalid_argument>(
            [&shortRoll]
            {
                validateGeometricSection(shortRoll, referenceSectionLength);
            },
            "geometric-section smallest-representable coverage shortfall"
        );

        GeometricSection latePitch = validGeometricSection();
        latePitch.pitch.segments.front().transition.domainBegin =
            std::nextafter(0.0, std::numeric_limits<double>::infinity());
        requireThrows<std::invalid_argument>(
            [&latePitch]
            {
                validateGeometricSection(latePitch, referenceSectionLength);
            },
            "geometric-section channel not starting at zero"
        );
    }

    void testForceSectionInvalidChannels()
    {
        ForceSection zeroWidth = validForceSection();
        zeroWidth.verticalForce.domainEnd =
            zeroWidth.verticalForce.domainBegin;
        requireThrows<std::invalid_argument>(
            [&zeroWidth]
            {
                validateForceSection(zeroWidth);
            },
            "force-section zero-width scalar transition"
        );

        ForceSection reversed = validForceSection();
        reversed.lateralForce.domainBegin =
            reversed.lateralForce.domainEnd + 1.0;
        requireThrows<std::invalid_argument>(
            [&reversed]
            {
                validateForceSection(reversed);
            },
            "force-section reversed scalar transition"
        );

        ForceSection unsupportedType = validForceSection();
        unsupportedType.roll.transitionType =
            static_cast<TransitionType>(999);
        requireThrows<std::invalid_argument>(
            [&unsupportedType]
            {
                validateForceSection(unsupportedType);
            },
            "force-section unsupported scalar transition type"
        );
    }

    void testGeometricSectionInvalidChannels()
    {
        GeometricSection zeroWidth = validGeometricSection();
        zeroWidth.pitch.segments.front().transition.domainEnd =
            zeroWidth.pitch.segments.front().transition.domainBegin;
        requireThrows<std::invalid_argument>(
            [&zeroWidth]
            {
                validateGeometricSection(zeroWidth, referenceSectionLength);
            },
            "geometric-section zero-width scalar transition"
        );

        GeometricSection reversed = validGeometricSection();
        reversed.yaw.segments.front().transition.domainBegin =
            reversed.yaw.segments.front().transition.domainEnd + 1.0;
        requireThrows<std::invalid_argument>(
            [&reversed]
            {
                validateGeometricSection(reversed, referenceSectionLength);
            },
            "geometric-section reversed scalar transition"
        );

        GeometricSection unsupportedType = validGeometricSection();
        unsupportedType.roll.segments.front().transition.transitionType =
            static_cast<TransitionType>(999);
        requireThrows<std::invalid_argument>(
            [&unsupportedType]
            {
                validateGeometricSection(unsupportedType, referenceSectionLength);
            },
            "geometric-section unsupported scalar transition type"
        );

        // A C0 discontinuity between adjacent segments of one channel is
        // rejected at the section level too.
        GeometricSection discontinuousYaw = validGeometricSection();
        ChannelProfile& yaw = discontinuousYaw.yaw;
        yaw.segments.push_back(ProfileSegment{
            yaw.nextSegmentId,
            ScalarTransition{
                referenceSectionLength / 2.0,
                referenceSectionLength,
                yaw.segments.back().transition.valueEnd + 0.25,
                -0.1,
                TransitionType::Linear
            }
        });
        ++yaw.nextSegmentId;
        yaw.segments.front().transition.domainEnd =
            referenceSectionLength / 2.0;
        requireThrows<std::invalid_argument>(
            [&discontinuousYaw]
            {
                validateGeometricSection(
                    discontinuousYaw,
                    referenceSectionLength
                );
            },
            "geometric-section C0 discontinuity between segments"
        );
    }

    void testNonFiniteAuthoredValues()
    {
        const double nonFiniteValues[]{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()
        };

        for (const double nonFinite : nonFiniteValues)
        {
            ForceSection force = validForceSection();
            force.verticalForce.valueBegin = nonFinite;
            requireThrows<std::invalid_argument>(
                [&force]
                {
                    validateForceSection(force);
                },
                "non-finite vertical-force value"
            );

            force = validForceSection();
            force.lateralForce.valueEnd = nonFinite;
            requireThrows<std::invalid_argument>(
                [&force]
                {
                    validateForceSection(force);
                },
                "non-finite lateral-force value"
            );

            force = validForceSection();
            force.roll.domainBegin = nonFinite;
            requireThrows<std::invalid_argument>(
                [&force]
                {
                    validateForceSection(force);
                },
                "non-finite force-section roll domain"
            );

            GeometricSection geometric = validGeometricSection();
            geometric.pitch.segments.front().transition.valueBegin = nonFinite;
            requireThrows<std::invalid_argument>(
                [&geometric]
                {
                    validateGeometricSection(geometric, referenceSectionLength);
                },
                "non-finite pitch value"
            );

            geometric = validGeometricSection();
            geometric.yaw.segments.front().transition.valueEnd = nonFinite;
            requireThrows<std::invalid_argument>(
                [&geometric]
                {
                    validateGeometricSection(geometric, referenceSectionLength);
                },
                "non-finite yaw value"
            );

            geometric = validGeometricSection();
            geometric.roll.segments.front().transition.domainEnd = nonFinite;
            requireThrows<std::invalid_argument>(
                [&geometric]
                {
                    validateGeometricSection(geometric, referenceSectionLength);
                },
                "non-finite geometric-section roll domain"
            );
        }
    }

    void testGeometricSectionEvaluationReference()
    {
        const GeometricSection section{
            singleSegmentChannel(0.0, 1.0, TransitionType::Linear, 0.0, 100.0),
            singleSegmentChannel(
                -2.0,
                2.0,
                TransitionType::Smoothstep,
                0.0,
                100.0
            ),
            singleSegmentChannel(
                1.0,
                -1.0,
                TransitionType::Smootherstep,
                0.0,
                100.0
            )
        };

        requireState(
            evaluateGeometricSection(section, 100.0, 0.0),
            {0.0, -2.0, 1.0},
            "reference beginning"
        );
        requireState(
            evaluateGeometricSection(section, 100.0, 25.0),
            {0.25, -1.375, 0.79296875},
            "reference 25 percent"
        );
        requireState(
            evaluateGeometricSection(section, 100.0, 50.0),
            {0.5, 0.0, 0.0},
            "reference midpoint"
        );
        requireState(
            evaluateGeometricSection(section, 100.0, 75.0),
            {0.75, 1.375, -0.79296875},
            "reference 75 percent"
        );
        requireState(
            evaluateGeometricSection(section, 100.0, 100.0),
            {1.0, 2.0, -1.0},
            "reference end"
        );
    }

    void testGeometricSectionMultiSegmentEvaluation()
    {
        // Pitch stays single-segment; yaw chains two C0-joined segments and
        // roll is an explicit constant plateau split into two segments.
        const GeometricSection section{
            singleSegmentChannel(0.0, 1.0, TransitionType::Linear, 0.0, 100.0),
            [] {
                ChannelProfile yaw;
                yaw.segments.push_back(ProfileSegment{
                    yaw.nextSegmentId++,
                    ScalarTransition{
                        0.0,
                        40.0,
                        0.0,
                        1.0,
                        TransitionType::Smoothstep}
                });
                yaw.segments.push_back(ProfileSegment{
                    yaw.nextSegmentId++,
                    ScalarTransition{
                        40.0,
                        100.0,
                        1.0,
                        0.0,
                        TransitionType::Smootherstep}
                });
                return yaw;
            }(),
            [] {
                ChannelProfile roll;
                roll.segments.push_back(ProfileSegment{
                    roll.nextSegmentId++,
                    ScalarTransition{
                        0.0,
                        55.0,
                        -0.5,
                        -0.5,
                        TransitionType::Linear}
                });
                roll.segments.push_back(ProfileSegment{
                    roll.nextSegmentId++,
                    ScalarTransition{
                        55.0,
                        100.0,
                        -0.5,
                        -0.5,
                        TransitionType::Smoothstep}
                });
                return roll;
            }()
        };

        validateGeometricSection(section, 100.0);

        requireState(
            evaluateGeometricSection(section, 100.0, 0.0),
            {0.0, 0.0, -0.5},
            "multi-segment beginning"
        );
        requireState(
            evaluateGeometricSection(section, 100.0, 20.0),
            {0.2, 0.5, -0.5},
            "multi-segment inside first yaw segment"
        );

        // The yaw joint at 40 evaluates identically from both sides because
        // the chain is C0 there.
        const GeometricSectionState joint = evaluateGeometricSection(
            section,
            100.0,
            40.0
        );
        requireState(joint, {0.4, 1.0, -0.5}, "multi-segment yaw joint");

        requireState(
            evaluateGeometricSection(section, 100.0, 70.0),
            {0.7, 0.5, -0.5},
            "multi-segment inside second yaw segment"
        );
        requireState(
            evaluateGeometricSection(section, 100.0, 100.0),
            {1.0, 0.0, -0.5},
            "multi-segment end"
        );

        // Channel-level evaluation agrees bit-for-bit with the section-level
        // evaluation on every queried distance.
        for (const double distance : {0.0, 13.75, 40.0, 55.0, 82.5, 100.0})
        {
            const GeometricSectionState state = evaluateGeometricSection(
                section,
                100.0,
                distance
            );
            require(
                state.pitch ==
                    evaluateChannelProfile(section.pitch, distance)
                    && state.yaw ==
                    evaluateChannelProfile(section.yaw, distance)
                    && state.roll ==
                    evaluateChannelProfile(section.roll, distance),
                "section and channel evaluators agree"
            );
        }
    }

    void testGeometricSectionEvaluationRejectsInvalidQueries()
    {
        const GeometricSection section = validGeometricSection();

        requireThrows<std::out_of_range>(
            [&section]
            {
                static_cast<void>(evaluateGeometricSection(
                    section,
                    referenceSectionLength,
                    -0.01
                ));
            },
            "geometric-section query below domain"
        );
        requireThrows<std::out_of_range>(
            [&section]
            {
                static_cast<void>(evaluateGeometricSection(
                    section,
                    referenceSectionLength,
                    referenceSectionLength + 0.01
                ));
            },
            "geometric-section query above domain"
        );

        const double nonFiniteValues[]{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()
        };

        for (const double nonFinite : nonFiniteValues)
        {
            requireThrows<std::invalid_argument>(
                [&section, nonFinite]
                {
                    static_cast<void>(evaluateGeometricSection(
                        section,
                        referenceSectionLength,
                        nonFinite
                    ));
                },
                "geometric-section non-finite query"
            );
        }
    }

    void testGeometricSectionEvaluationRejectsInvalidSection()
    {
        GeometricSection mismatchedCoverage = validGeometricSection();
        mismatchedCoverage.roll.segments.back().transition.domainEnd = 14.0;

        requireThrows<std::invalid_argument>(
            [&mismatchedCoverage]
            {
                static_cast<void>(evaluateGeometricSection(
                    mismatchedCoverage,
                    referenceSectionLength,
                    2.0
                ));
            },
            "geometric-section evaluation with mismatched coverage"
        );
    }

    void testGeometricSectionEvaluationIsDeterministic()
    {
        const GeometricSection section = validGeometricSection();
        constexpr double independentValue = 2.375;
        const GeometricSectionState expected = evaluateGeometricSection(
            section,
            referenceSectionLength,
            independentValue
        );
        const std::uint64_t expectedPitchBits =
            std::bit_cast<std::uint64_t>(expected.pitch);
        const std::uint64_t expectedYawBits =
            std::bit_cast<std::uint64_t>(expected.yaw);
        const std::uint64_t expectedRollBits =
            std::bit_cast<std::uint64_t>(expected.roll);

        for (int repetition = 0; repetition < 100; ++repetition)
        {
            const GeometricSectionState actual = evaluateGeometricSection(
                section,
                referenceSectionLength,
                independentValue
            );

            require(
                std::bit_cast<std::uint64_t>(actual.pitch)
                    == expectedPitchBits,
                "geometric-section pitch evaluation changed bits"
            );
            require(
                std::bit_cast<std::uint64_t>(actual.yaw) == expectedYawBits,
                "geometric-section yaw evaluation changed bits"
            );
            require(
                std::bit_cast<std::uint64_t>(actual.roll) == expectedRollBits,
                "geometric-section roll evaluation changed bits"
            );
        }
    }

    void testDeterministicValidationAndEvaluation()
    {
        const ForceSection force = validForceSection();
        const GeometricSection geometric = validGeometricSection();
        constexpr double independentValue = 2.375;
        const std::uint64_t expectedForceBits = std::bit_cast<std::uint64_t>(
            evaluateScalarTransition(force.verticalForce, independentValue)
        );
        const std::uint64_t expectedGeometricBits =
            std::bit_cast<std::uint64_t>(
                evaluateScalarTransition(
                    geometric.yaw.segments.front().transition,
                    independentValue
                )
            );

        for (int repetition = 0; repetition < 100; ++repetition)
        {
            validateForceSection(force);
            validateGeometricSection(geometric, referenceSectionLength);

            require(
                std::bit_cast<std::uint64_t>(evaluateScalarTransition(
                    force.verticalForce,
                    independentValue
                )) == expectedForceBits,
                "force-section scalar evaluation changed bits"
            );
            require(
                std::bit_cast<std::uint64_t>(evaluateScalarTransition(
                    geometric.yaw.segments.front().transition,
                    independentValue
                )) == expectedGeometricBits,
                "geometric-section scalar evaluation changed bits"
            );
        }

        require(
            force.verticalForce.domainBegin == -7.25
                && force.verticalForce.domainEnd == 13.5
                && geometric.pitch.segments.front().transition.domainBegin
                == 0.0
                && geometric.pitch.segments.front().transition.domainEnd
                == referenceSectionLength,
            "validation changed exact authored domain endpoints"
        );
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;

    const std::vector<Test> tests{
        {"force-section representation", testForceSectionRepresentation},
        {
            "geometric-section representation",
            testGeometricSectionRepresentation
        },
        {"force-section shared domain", testForceSectionSharedDomain},
        {"geometric-section shared length", testGeometricSectionSharedLength},
        {"force-section invalid channels", testForceSectionInvalidChannels},
        {
            "geometric-section invalid channels",
            testGeometricSectionInvalidChannels
        },
        {"non-finite authored values", testNonFiniteAuthoredValues},
        {
            "geometric-section evaluation reference",
            testGeometricSectionEvaluationReference
        },
        {
            "geometric-section multi-segment evaluation",
            testGeometricSectionMultiSegmentEvaluation
        },
        {
            "geometric-section evaluation rejects invalid queries",
            testGeometricSectionEvaluationRejectsInvalidQueries
        },
        {
            "geometric-section evaluation rejects invalid section",
            testGeometricSectionEvaluationRejectsInvalidSection
        },
        {
            "geometric-section evaluation is deterministic",
            testGeometricSectionEvaluationIsDeterministic
        },
        {
            "deterministic validation and evaluation",
            testDeterministicValidationAndEvaluation
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
