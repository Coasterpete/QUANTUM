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
    using quantum::coaster::ForceSection;
    using quantum::coaster::evaluateGeometricSection;
    using quantum::coaster::GeometricSection;
    using quantum::coaster::GeometricSectionState;
    using quantum::coaster::validateForceSection;
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

    [[nodiscard]] constexpr ForceSection validForceSection()
    {
        return {
            transition(-1.0, 4.0, TransitionType::Linear),
            transition(3.0, -2.0, TransitionType::Smoothstep),
            transition(0.25, 0.25, TransitionType::Smootherstep)
        };
    }

    [[nodiscard]] constexpr GeometricSection validGeometricSection()
    {
        return {
            transition(-0.2, 0.4, TransitionType::Smoothstep),
            transition(0.3, -0.1, TransitionType::Smootherstep),
            transition(-0.5, -0.5, TransitionType::Linear)
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
        constexpr GeometricSection section = validGeometricSection();

        validateGeometricSection(section);

        requireTransitionRetained(
            section.pitch,
            transition(-0.2, 0.4, TransitionType::Smoothstep),
            "pitch"
        );
        requireTransitionRetained(
            section.yaw,
            transition(0.3, -0.1, TransitionType::Smootherstep),
            "yaw"
        );
        requireTransitionRetained(
            section.roll,
            transition(-0.5, -0.5, TransitionType::Linear),
            "geometric-section roll"
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

    void testGeometricSectionSharedDomain()
    {
        validateGeometricSection(validGeometricSection());

        GeometricSection mismatchedBeginning = validGeometricSection();
        mismatchedBeginning.yaw.domainBegin = std::nextafter(
            mismatchedBeginning.pitch.domainBegin,
            mismatchedBeginning.pitch.domainEnd
        );
        requireThrows<std::invalid_argument>(
            [&mismatchedBeginning]
            {
                validateGeometricSection(mismatchedBeginning);
            },
            "geometric-section smallest-representable beginning mismatch"
        );

        GeometricSection mismatchedEnd = validGeometricSection();
        mismatchedEnd.roll.domainEnd = std::nextafter(
            mismatchedEnd.pitch.domainEnd,
            std::numeric_limits<double>::infinity()
        );
        requireThrows<std::invalid_argument>(
            [&mismatchedEnd]
            {
                validateGeometricSection(mismatchedEnd);
            },
            "geometric-section smallest-representable end mismatch"
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
        zeroWidth.pitch.domainEnd = zeroWidth.pitch.domainBegin;
        requireThrows<std::invalid_argument>(
            [&zeroWidth]
            {
                validateGeometricSection(zeroWidth);
            },
            "geometric-section zero-width scalar transition"
        );

        GeometricSection reversed = validGeometricSection();
        reversed.yaw.domainBegin = reversed.yaw.domainEnd + 1.0;
        requireThrows<std::invalid_argument>(
            [&reversed]
            {
                validateGeometricSection(reversed);
            },
            "geometric-section reversed scalar transition"
        );

        GeometricSection unsupportedType = validGeometricSection();
        unsupportedType.roll.transitionType =
            static_cast<TransitionType>(999);
        requireThrows<std::invalid_argument>(
            [&unsupportedType]
            {
                validateGeometricSection(unsupportedType);
            },
            "geometric-section unsupported scalar transition type"
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
            geometric.pitch.valueBegin = nonFinite;
            requireThrows<std::invalid_argument>(
                [&geometric]
                {
                    validateGeometricSection(geometric);
                },
                "non-finite pitch value"
            );

            geometric = validGeometricSection();
            geometric.yaw.valueEnd = nonFinite;
            requireThrows<std::invalid_argument>(
                [&geometric]
                {
                    validateGeometricSection(geometric);
                },
                "non-finite yaw value"
            );

            geometric = validGeometricSection();
            geometric.roll.domainEnd = nonFinite;
            requireThrows<std::invalid_argument>(
                [&geometric]
                {
                    validateGeometricSection(geometric);
                },
                "non-finite geometric-section roll domain"
            );
        }
    }

    void testGeometricSectionEvaluationReference()
    {
        constexpr GeometricSection section{
            transition(0.0, 1.0, TransitionType::Linear, 0.0, 100.0),
            transition(-2.0, 2.0, TransitionType::Smoothstep, 0.0, 100.0),
            transition(1.0, -1.0, TransitionType::Smootherstep, 0.0, 100.0)
        };

        requireState(
            evaluateGeometricSection(section, 0.0),
            {0.0, -2.0, 1.0},
            "reference beginning"
        );
        requireState(
            evaluateGeometricSection(section, 25.0),
            {0.25, -1.375, 0.79296875},
            "reference 25 percent"
        );
        requireState(
            evaluateGeometricSection(section, 50.0),
            {0.5, 0.0, 0.0},
            "reference midpoint"
        );
        requireState(
            evaluateGeometricSection(section, 75.0),
            {0.75, 1.375, -0.79296875},
            "reference 75 percent"
        );
        requireState(
            evaluateGeometricSection(section, 100.0),
            {1.0, 2.0, -1.0},
            "reference end"
        );
    }

    void testGeometricSectionTranslatedDomainAndConstantChannel()
    {
        constexpr GeometricSection section{
            transition(-4.0, 4.0, TransitionType::Linear, 100.0, 140.0),
            transition(0.0, 0.0, TransitionType::Smoothstep, 100.0, 140.0),
            transition(2.0, -2.0, TransitionType::Smootherstep, 100.0, 140.0)
        };

        requireState(
            evaluateGeometricSection(section, 110.0),
            {-2.0, 0.0, 1.5859375},
            "translated-domain 25 percent"
        );
        requireState(
            evaluateGeometricSection(section, 120.0),
            {0.0, 0.0, 0.0},
            "translated-domain midpoint"
        );
        requireState(
            evaluateGeometricSection(section, 130.0),
            {2.0, 0.0, -1.5859375},
            "translated-domain 75 percent"
        );
    }

    void testGeometricSectionEvaluationRejectsInvalidQueries()
    {
        constexpr GeometricSection section{
            transition(0.0, 1.0, TransitionType::Linear, 0.0, 100.0),
            transition(-2.0, 2.0, TransitionType::Smoothstep, 0.0, 100.0),
            transition(1.0, -1.0, TransitionType::Smootherstep, 0.0, 100.0)
        };

        requireThrows<std::out_of_range>(
            [&section]
            {
                static_cast<void>(evaluateGeometricSection(section, -0.01));
            },
            "geometric-section query below domain"
        );
        requireThrows<std::out_of_range>(
            [&section]
            {
                static_cast<void>(evaluateGeometricSection(section, 100.01));
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
                    static_cast<void>(
                        evaluateGeometricSection(section, nonFinite)
                    );
                },
                "geometric-section non-finite query"
            );
        }
    }

    void testGeometricSectionEvaluationRejectsInvalidSection()
    {
        GeometricSection mismatchedDomain = validGeometricSection();
        mismatchedDomain.roll.domainEnd = 14.0;

        requireThrows<std::invalid_argument>(
            [&mismatchedDomain]
            {
                static_cast<void>(
                    evaluateGeometricSection(mismatchedDomain, 2.0)
                );
            },
            "geometric-section evaluation with mismatched domain"
        );
    }

    void testGeometricSectionEvaluationIsDeterministic()
    {
        const GeometricSection section = validGeometricSection();
        constexpr double independentValue = 2.375;
        const GeometricSectionState expected = evaluateGeometricSection(
            section,
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
                evaluateScalarTransition(geometric.yaw, independentValue)
            );

        for (int repetition = 0; repetition < 100; ++repetition)
        {
            validateForceSection(force);
            validateGeometricSection(geometric);

            require(
                std::bit_cast<std::uint64_t>(evaluateScalarTransition(
                    force.verticalForce,
                    independentValue
                )) == expectedForceBits,
                "force-section scalar evaluation changed bits"
            );
            require(
                std::bit_cast<std::uint64_t>(evaluateScalarTransition(
                    geometric.yaw,
                    independentValue
                )) == expectedGeometricBits,
                "geometric-section scalar evaluation changed bits"
            );
        }

        require(
            force.verticalForce.domainBegin == -7.25
                && force.verticalForce.domainEnd == 13.5
                && geometric.pitch.domainBegin == -7.25
                && geometric.pitch.domainEnd == 13.5,
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
        {
            "geometric-section shared domain",
            testGeometricSectionSharedDomain
        },
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
            "geometric-section translated domain and constant channel",
            testGeometricSectionTranslatedDomainAndConstantChannel
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
