#include <quantum/math/ScalarTransition.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::math::evaluateScalarTransition;
    using quantum::math::integrateScalarTransition;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

    constexpr double toleranceFactor = 64.0;

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

    void requireNear(
        const double actual,
        const double expected,
        const std::string_view context
    )
    {
        const double scale = std::max({
            1.0,
            std::abs(actual),
            std::abs(expected)
        });
        const double tolerance = toleranceFactor
            * std::numeric_limits<double>::epsilon() * scale;

        if (!std::isfinite(actual)
            || std::abs(actual - expected) > tolerance)
        {
            std::ostringstream message;
            message << std::setprecision(17)
                    << context << ": expected " << expected
                    << ", received " << actual
                    << ", tolerance " << tolerance;
            throw TestFailure(message.str());
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

    constexpr TransitionType transitionTypes[]{
        TransitionType::Linear,
        TransitionType::Smoothstep,
        TransitionType::Smootherstep,
        TransitionType::SeventhOrderSmoothstep,
        TransitionType::CosineEaseInOut,
        TransitionType::SineEaseIn,
        TransitionType::SineEaseOut,
        TransitionType::QuadraticEaseIn,
        TransitionType::QuadraticEaseOut,
        TransitionType::QuadraticEaseInOut,
        TransitionType::CubicEaseIn,
        TransitionType::CubicEaseOut,
        TransitionType::CubicEaseInOut,
        TransitionType::QuarticEaseIn,
        TransitionType::QuarticEaseOut,
        TransitionType::QuarticEaseInOut,
        TransitionType::QuinticEaseIn,
        TransitionType::QuinticEaseOut,
        TransitionType::QuinticEaseInOut
    };

    struct ExpandedTransitionReference
    {
        TransitionType type;
        double quarterValue;
        double quarterIntegral;
        double fullArea;
    };

    constexpr double pi = std::numbers::pi_v<double>;

    constexpr ExpandedTransitionReference expandedTransitionReferences[]{
        {
            TransitionType::SeventhOrderSmoothstep,
            289.0 / 4'096.0,
            523.0 / 131'072.0,
            0.5
        },
        {
            TransitionType::CosineEaseInOut,
            0.1464466094067262,
            0.01246046048036173,
            0.5
        },
        {
            TransitionType::SineEaseIn,
            0.07612046748871326,
            0.006376160398891856,
            1.0 - 2.0 / pi
        },
        {
            TransitionType::SineEaseOut,
            0.3826834323650897,
            0.04845979468517852,
            2.0 / pi
        },
        {
            TransitionType::QuadraticEaseIn,
            1.0 / 16.0,
            1.0 / 192.0,
            1.0 / 3.0
        },
        {
            TransitionType::QuadraticEaseOut,
            7.0 / 16.0,
            11.0 / 192.0,
            2.0 / 3.0
        },
        {
            TransitionType::QuadraticEaseInOut,
            1.0 / 8.0,
            1.0 / 96.0,
            0.5
        },
        {
            TransitionType::CubicEaseIn,
            1.0 / 64.0,
            1.0 / 1'024.0,
            0.25
        },
        {
            TransitionType::CubicEaseOut,
            37.0 / 64.0,
            81.0 / 1'024.0,
            0.75
        },
        {
            TransitionType::CubicEaseInOut,
            1.0 / 16.0,
            1.0 / 256.0,
            0.5
        },
        {
            TransitionType::QuarticEaseIn,
            1.0 / 256.0,
            1.0 / 5'120.0,
            0.2
        },
        {
            TransitionType::QuarticEaseOut,
            175.0 / 256.0,
            499.0 / 5'120.0,
            0.8
        },
        {
            TransitionType::QuarticEaseInOut,
            1.0 / 32.0,
            1.0 / 640.0,
            0.5
        },
        {
            TransitionType::QuinticEaseIn,
            1.0 / 1'024.0,
            1.0 / 24'576.0,
            1.0 / 6.0
        },
        {
            TransitionType::QuinticEaseOut,
            781.0 / 1'024.0,
            2'777.0 / 24'576.0,
            5.0 / 6.0
        },
        {
            TransitionType::QuinticEaseInOut,
            1.0 / 64.0,
            1.0 / 1'536.0,
            0.5
        }
    };

    void testConstructionAndMidpoint()
    {
        constexpr ScalarTransition transition{
            10.0,
            20.0,
            100.0,
            200.0,
            TransitionType::Linear
        };

        require(transition.domainBegin == 10.0, "constructed domain beginning");
        require(transition.domainEnd == 20.0, "constructed domain end");
        require(transition.valueBegin == 100.0, "constructed value beginning");
        require(transition.valueEnd == 200.0, "constructed value end");
        require(
            transition.transitionType == TransitionType::Linear,
            "constructed transition type"
        );
        require(
            evaluateScalarTransition(transition, 15.0) == 150.0,
            "linear midpoint"
        );
    }

    void testExactBoundaries()
    {
        for (const TransitionType type : transitionTypes)
        {
            const ScalarTransition transition{
                -7.25,
                13.5,
                -2.75,
                4.125,
                type
            };

            require(
                evaluateScalarTransition(transition, transition.domainBegin)
                    == transition.valueBegin,
                "beginning value must be exact"
            );
            require(
                evaluateScalarTransition(transition, transition.domainEnd)
                    == transition.valueEnd,
                "ending value must be exact"
            );
        }
    }

    void testLinearReferenceValues()
    {
        constexpr ScalarTransition transition{
            10.0,
            20.0,
            100.0,
            200.0,
            TransitionType::Linear
        };
        constexpr std::pair<double, double> references[]{
            {10.0, 100.0},
            {12.5, 125.0},
            {15.0, 150.0},
            {17.5, 175.0},
            {20.0, 200.0}
        };

        for (const auto [independentValue, expected] : references)
        {
            requireNear(
                evaluateScalarTransition(transition, independentValue),
                expected,
                "linear scalar reference"
            );
        }
    }

    void testSmoothstepReferenceValues()
    {
        constexpr ScalarTransition transition{
            10.0,
            20.0,
            100.0,
            200.0,
            TransitionType::Smoothstep
        };
        constexpr std::pair<double, double> references[]{
            {10.0, 100.0},
            {12.5, 115.625},
            {15.0, 150.0},
            {17.5, 184.375},
            {20.0, 200.0}
        };

        for (const auto [independentValue, expected] : references)
        {
            requireNear(
                evaluateScalarTransition(transition, independentValue),
                expected,
                "smoothstep scalar reference"
            );
        }
    }

    void testSmootherstepReferenceValues()
    {
        constexpr ScalarTransition transition{
            10.0,
            20.0,
            100.0,
            200.0,
            TransitionType::Smootherstep
        };
        // The quarter points use the independently reduced transition
        // fractions 53/512 and 459/512. The tenth points use 107/12500
        // and 12393/12500.
        constexpr std::pair<double, double> references[]{
            {10.0, 100.0},
            {11.0, 100.856},
            {12.5, 110.3515625},
            {15.0, 150.0},
            {17.5, 189.6484375},
            {19.0, 199.144},
            {20.0, 200.0}
        };

        for (const auto [independentValue, expected] : references)
        {
            requireNear(
                evaluateScalarTransition(transition, independentValue),
                expected,
                "smootherstep scalar reference"
            );
        }
    }

    void testExpandedTransitionCompatibility()
    {
        constexpr double domainBegin = 10.0;
        constexpr double domainEnd = 18.0;
        constexpr double domainLength = domainEnd - domainBegin;
        constexpr double valueBegin = -3.0;
        constexpr double valueEnd = 5.0;
        constexpr double valueRange = valueEnd - valueBegin;
        constexpr double quarterProgress = 0.25;
        constexpr double quarterDomainValue =
            domainBegin + quarterProgress * domainLength;

        for (const ExpandedTransitionReference& reference
            : expandedTransitionReferences)
        {
            const ScalarTransition transition{
                domainBegin,
                domainEnd,
                valueBegin,
                valueEnd,
                reference.type
            };
            const double expectedQuarterValue = valueBegin
                + valueRange * reference.quarterValue;
            const double expectedQuarterIntegral = domainLength * (
                valueBegin * quarterProgress
                + valueRange * reference.quarterIntegral
            );
            const double expectedFullIntegral = domainLength * (
                valueBegin + valueRange * reference.fullArea
            );

            requireNear(
                evaluateScalarTransition(transition, quarterDomainValue),
                expectedQuarterValue,
                "expanded scalar-transition quarter value"
            );
            requireNear(
                integrateScalarTransition(
                    transition,
                    domainBegin,
                    quarterDomainValue
                ),
                expectedQuarterIntegral,
                "expanded scalar-transition quarter integral"
            );
            requireNear(
                integrateScalarTransition(
                    transition,
                    domainBegin,
                    domainEnd
                ),
                expectedFullIntegral,
                "expanded scalar-transition full integral"
            );
        }
    }

    void testIncreasingDecreasingAndNegativeValues()
    {
        const ScalarTransition increasing{
            0.0,
            1.0,
            -5.0,
            5.0,
            TransitionType::Smoothstep
        };
        requireNear(
            evaluateScalarTransition(increasing, 0.25),
            -3.4375,
            "negative-to-positive quarter point"
        );
        requireNear(
            evaluateScalarTransition(increasing, 0.5),
            0.0,
            "negative-to-positive midpoint"
        );
        requireNear(
            evaluateScalarTransition(increasing, 0.75),
            3.4375,
            "negative-to-positive three-quarter point"
        );

        const ScalarTransition decreasing{
            0.0,
            1.0,
            10.0,
            -10.0,
            TransitionType::Smoothstep
        };
        requireNear(
            evaluateScalarTransition(decreasing, 0.25),
            6.875,
            "decreasing quarter point"
        );
        requireNear(
            evaluateScalarTransition(decreasing, 0.5),
            0.0,
            "decreasing midpoint"
        );
        requireNear(
            evaluateScalarTransition(decreasing, 0.75),
            -6.875,
            "decreasing three-quarter point"
        );

        const ScalarTransition positiveToNegative{
            2.0,
            6.0,
            4.0,
            -2.0,
            TransitionType::Linear
        };
        requireNear(
            evaluateScalarTransition(positiveToNegative, 3.0),
            2.5,
            "positive-to-negative linear value"
        );
    }

    void testConstantValues()
    {
        for (const TransitionType type : transitionTypes)
        {
            const ScalarTransition transition{
                -3.0,
                9.0,
                5.0,
                5.0,
                type
            };

            for (const double independentValue : {-3.0, 0.0, 4.25, 9.0})
            {
                require(
                    evaluateScalarTransition(transition, independentValue)
                        == 5.0,
                    "constant transition changed value"
                );
            }
        }
    }

    void testDomainTranslationAndScaling()
    {
        for (const TransitionType type : transitionTypes)
        {
            const ScalarTransition base{
                0.0,
                10.0,
                -2.0,
                4.0,
                type
            };
            const ScalarTransition translated{
                100.0,
                110.0,
                -2.0,
                4.0,
                type
            };
            const ScalarTransition unitDomain{
                0.0,
                1.0,
                -2.0,
                4.0,
                type
            };
            const ScalarTransition scaledDomain{
                0.0,
                100.0,
                -2.0,
                4.0,
                type
            };

            for (const double progress : {0.0, 0.25, 0.5, 0.75, 1.0})
            {
                const double baseValue = evaluateScalarTransition(
                    base,
                    progress * 10.0
                );
                const double translatedValue = evaluateScalarTransition(
                    translated,
                    100.0 + progress * 10.0
                );
                const double unitValue = evaluateScalarTransition(
                    unitDomain,
                    progress
                );
                const double scaledValue = evaluateScalarTransition(
                    scaledDomain,
                    progress * 100.0
                );

                requireNear(
                    translatedValue,
                    baseValue,
                    "translated domain"
                );
                requireNear(
                    scaledValue,
                    unitValue,
                    "scaled domain"
                );
            }
        }
    }

    void testAnalyticTransitionIntegrals()
    {
        constexpr double length = 20.0;
        constexpr double valueBegin = -0.2;
        constexpr double valueEnd = 0.6;
        constexpr double fullArea =
            length * (valueBegin + valueEnd) / 2.0;
        constexpr std::pair<TransitionType, double> quarterAreas[]{
            {TransitionType::Linear, -0.5},
            {TransitionType::Smoothstep, -0.78125},
            {TransitionType::Smootherstep, -0.88671875}
        };

        for (const auto [type, expectedQuarterArea] : quarterAreas)
        {
            const ScalarTransition transition{
                100.0,
                100.0 + length,
                valueBegin,
                valueEnd,
                type
            };

            requireNear(
                integrateScalarTransition(transition, 100.0, 105.0),
                expectedQuarterArea,
                "analytic quarter-domain transition area"
            );
            requireNear(
                integrateScalarTransition(transition, 100.0, 120.0),
                fullArea,
                "analytic full-domain mean area"
            );
            require(
                integrateScalarTransition(transition, 107.5, 107.5)
                    == 0.0,
                "equal integration bounds must return exact zero"
            );
        }
    }

    void testIntegralDomainTranslation()
    {
        for (const TransitionType type : transitionTypes)
        {
            const ScalarTransition base{
                0.0, 20.0, -0.2, 0.6, type
            };
            const ScalarTransition translated{
                100.0, 120.0, -0.2, 0.6, type
            };

            for (const double distance : {0.0, 2.5, 7.0, 13.25, 20.0})
            {
                requireNear(
                    integrateScalarTransition(
                        translated,
                        translated.domainBegin,
                        translated.domainBegin + distance
                    ),
                    integrateScalarTransition(
                        base,
                        base.domainBegin,
                        base.domainBegin + distance
                    ),
                    "translated transition integral"
                );
            }
        }
    }

    void testInvalidIntegralBoundsAndTransitionType()
    {
        const ScalarTransition valid{
            10.0, 20.0, 1.0, 2.0, TransitionType::Linear
        };

        requireThrows<std::invalid_argument>(
            [&valid]
            {
                static_cast<void>(integrateScalarTransition(
                    valid, 15.0, 14.0
                ));
            },
            "reversed integration bounds"
        );
        requireThrows<std::out_of_range>(
            [&valid]
            {
                static_cast<void>(integrateScalarTransition(
                    valid, 9.0, 15.0
                ));
            },
            "integration beginning below domain"
        );
        requireThrows<std::out_of_range>(
            [&valid]
            {
                static_cast<void>(integrateScalarTransition(
                    valid, 15.0, 21.0
                ));
            },
            "integration ending above domain"
        );

        ScalarTransition unsupported = valid;
        unsupported.transitionType = static_cast<TransitionType>(-1);
        requireThrows<std::invalid_argument>(
            [&unsupported]
            {
                static_cast<void>(integrateScalarTransition(
                    unsupported,
                    unsupported.domainBegin,
                    unsupported.domainEnd
                ));
            },
            "unsupported transition type"
        );
    }

    void testInvalidDomainsAndQueries()
    {
        const ScalarTransition valid{
            10.0,
            20.0,
            100.0,
            200.0,
            TransitionType::Linear
        };

        requireThrows<std::out_of_range>(
            [&valid]
            {
                static_cast<void>(evaluateScalarTransition(valid, 9.999));
            },
            "below-domain query"
        );
        requireThrows<std::out_of_range>(
            [&valid]
            {
                static_cast<void>(evaluateScalarTransition(valid, 20.001));
            },
            "above-domain query"
        );

        const ScalarTransition zeroWidth{
            10.0,
            10.0,
            100.0,
            200.0,
            TransitionType::Linear
        };
        requireThrows<std::invalid_argument>(
            [&zeroWidth]
            {
                static_cast<void>(evaluateScalarTransition(zeroWidth, 10.0));
            },
            "zero-width domain"
        );

        const ScalarTransition reversed{
            20.0,
            10.0,
            100.0,
            200.0,
            TransitionType::Linear
        };
        requireThrows<std::invalid_argument>(
            [&reversed]
            {
                static_cast<void>(evaluateScalarTransition(reversed, 15.0));
            },
            "reversed domain"
        );
    }

    void testNonFiniteValues()
    {
        const ScalarTransition valid{
            0.0,
            1.0,
            -2.0,
            4.0,
            TransitionType::Linear
        };
        const double nonFiniteValues[]{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()
        };

        for (const double nonFinite : nonFiniteValues)
        {
            ScalarTransition transition = valid;
            transition.domainBegin = nonFinite;
            requireThrows<std::invalid_argument>(
                [&transition]
                {
                    static_cast<void>(evaluateScalarTransition(transition, 0.5));
                },
                "non-finite domain beginning"
            );

            transition = valid;
            transition.domainEnd = nonFinite;
            requireThrows<std::invalid_argument>(
                [&transition]
                {
                    static_cast<void>(evaluateScalarTransition(transition, 0.5));
                },
                "non-finite domain end"
            );

            transition = valid;
            transition.valueBegin = nonFinite;
            requireThrows<std::invalid_argument>(
                [&transition]
                {
                    static_cast<void>(evaluateScalarTransition(transition, 0.5));
                },
                "non-finite beginning value"
            );

            transition = valid;
            transition.valueEnd = nonFinite;
            requireThrows<std::invalid_argument>(
                [&transition]
                {
                    static_cast<void>(evaluateScalarTransition(transition, 0.5));
                },
                "non-finite ending value"
            );

            requireThrows<std::invalid_argument>(
                [&valid, nonFinite]
                {
                    static_cast<void>(evaluateScalarTransition(valid, nonFinite));
                },
                "non-finite query"
            );
        }
    }

    void testDeterministicRepeatedEvaluation()
    {
        for (const TransitionType type : transitionTypes)
        {
            const ScalarTransition transition{
                -17.0,
                29.0,
                -2.0,
                4.0,
                type
            };
            const double expected = evaluateScalarTransition(
                transition,
                0.371
            );
            const std::uint64_t expectedBits =
                std::bit_cast<std::uint64_t>(expected);
            const double expectedIntegral = integrateScalarTransition(
                transition,
                transition.domainBegin,
                0.371
            );
            const std::uint64_t expectedIntegralBits =
                std::bit_cast<std::uint64_t>(expectedIntegral);

            for (int repetition = 0; repetition < 100; ++repetition)
            {
                const double actual = evaluateScalarTransition(
                    transition,
                    0.371
                );
                require(
                    std::bit_cast<std::uint64_t>(actual) == expectedBits,
                    "repeated scalar-transition evaluation changed bits"
                );
                const double actualIntegral = integrateScalarTransition(
                    transition,
                    transition.domainBegin,
                    0.371
                );
                require(
                    std::bit_cast<std::uint64_t>(actualIntegral)
                        == expectedIntegralBits,
                    "repeated scalar-transition integral changed bits"
                );
            }
        }
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;

    const std::vector<Test> tests{
        {"construction and midpoint", testConstructionAndMidpoint},
        {"exact boundaries", testExactBoundaries},
        {"linear reference values", testLinearReferenceValues},
        {"smoothstep reference values", testSmoothstepReferenceValues},
        {"smootherstep reference values", testSmootherstepReferenceValues},
        {
            "expanded transition compatibility",
            testExpandedTransitionCompatibility
        },
        {
            "increasing, decreasing, and negative values",
            testIncreasingDecreasingAndNegativeValues
        },
        {"constant values", testConstantValues},
        {"domain translation and scaling", testDomainTranslationAndScaling},
        {"analytic transition integrals", testAnalyticTransitionIntegrals},
        {"integral domain translation", testIntegralDomainTranslation},
        {
            "invalid integral bounds and transition type",
            testInvalidIntegralBoundsAndTransitionType
        },
        {"invalid domains and queries", testInvalidDomainsAndQueries},
        {"non-finite values", testNonFiniteValues},
        {"deterministic repeated evaluation", testDeterministicRepeatedEvaluation}
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
