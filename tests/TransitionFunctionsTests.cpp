#include <quantum/math/TransitionFunctions.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
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
    using quantum::math::evaluateTransition;
    using quantum::math::evaluateTransitionIntegral;
    using quantum::math::TransitionType;

    constexpr double referenceTolerance =
        16.0 * std::numeric_limits<double>::epsilon();
    constexpr double newSymmetryTolerance =
        64.0 * std::numeric_limits<double>::epsilon();
    constexpr double derivativeStep = 0.001;
    constexpr double firstDerivativeTolerance = 0.00001;
    constexpr double secondDerivativeTolerance = 0.0005;
    constexpr double quadratureTolerance = 0.000000000005;
    constexpr int denseIntervalCount = 10'000;
    constexpr int quadratureIntervalCount = 20'000;
    double maximumSymmetryError = 0.0;
    double maximumQuadratureError = 0.0;

    static_assert(static_cast<int>(TransitionType::Linear) == 0);
    static_assert(static_cast<int>(TransitionType::Smoothstep) == 1);
    static_assert(static_cast<int>(TransitionType::Smootherstep) == 2);

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

    void requireNearWithTolerance(
        const double actual,
        const double expected,
        const double tolerance,
        const std::string_view context
    )
    {
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

    void requireNear(
        const double actual,
        const double expected,
        const std::string_view context
    )
    {
        requireNearWithTolerance(
            actual,
            expected,
            referenceTolerance,
            context
        );
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

    struct TransitionReference
    {
        TransitionType type;
        std::string_view name;
        double quarterValue;
        double quarterIntegral;
        double fullArea;
        double beginningFirstDerivative;
        double endingFirstDerivative;
        double beginningSecondDerivative;
        double endingSecondDerivative;
        bool symmetric;
    };

    constexpr double pi = std::numbers::pi_v<double>;

    constexpr TransitionReference transitionReferences[]{
        {
            TransitionType::Linear,
            "linear",
            0.25,
            0.03125,
            0.5,
            1.0,
            1.0,
            0.0,
            0.0,
            true
        },
        {
            TransitionType::Smoothstep,
            "smoothstep",
            0.15625,
            0.013671875,
            0.5,
            0.0,
            0.0,
            6.0,
            -6.0,
            true
        },
        {
            TransitionType::Smootherstep,
            "smootherstep",
            53.0 / 512.0,
            29.0 / 4'096.0,
            0.5,
            0.0,
            0.0,
            0.0,
            0.0,
            true
        },
        {
            TransitionType::SeventhOrderSmoothstep,
            "seventh-order smoothstep",
            289.0 / 4'096.0,
            523.0 / 131'072.0,
            0.5,
            0.0,
            0.0,
            0.0,
            0.0,
            true
        },
        {
            TransitionType::CosineEaseInOut,
            "cosine ease-in-out",
            0.1464466094067262,
            0.01246046048036173,
            0.5,
            0.0,
            0.0,
            pi * pi / 2.0,
            -pi * pi / 2.0,
            true
        },
        {
            TransitionType::SineEaseIn,
            "sine ease-in",
            0.07612046748871326,
            0.006376160398891856,
            1.0 - 2.0 / pi,
            0.0,
            pi / 2.0,
            pi * pi / 4.0,
            0.0,
            false
        },
        {
            TransitionType::SineEaseOut,
            "sine ease-out",
            0.3826834323650897,
            0.04845979468517852,
            2.0 / pi,
            pi / 2.0,
            0.0,
            0.0,
            -pi * pi / 4.0,
            false
        },
        {
            TransitionType::QuadraticEaseIn,
            "quadratic ease-in",
            1.0 / 16.0,
            1.0 / 192.0,
            1.0 / 3.0,
            0.0,
            2.0,
            2.0,
            2.0,
            false
        },
        {
            TransitionType::QuadraticEaseOut,
            "quadratic ease-out",
            7.0 / 16.0,
            11.0 / 192.0,
            2.0 / 3.0,
            2.0,
            0.0,
            -2.0,
            -2.0,
            false
        },
        {
            TransitionType::QuadraticEaseInOut,
            "quadratic ease-in-out",
            1.0 / 8.0,
            1.0 / 96.0,
            0.5,
            0.0,
            0.0,
            4.0,
            -4.0,
            true
        },
        {
            TransitionType::CubicEaseIn,
            "cubic ease-in",
            1.0 / 64.0,
            1.0 / 1'024.0,
            0.25,
            0.0,
            3.0,
            0.0,
            6.0,
            false
        },
        {
            TransitionType::CubicEaseOut,
            "cubic ease-out",
            37.0 / 64.0,
            81.0 / 1'024.0,
            0.75,
            3.0,
            0.0,
            -6.0,
            0.0,
            false
        },
        {
            TransitionType::CubicEaseInOut,
            "cubic ease-in-out",
            1.0 / 16.0,
            1.0 / 256.0,
            0.5,
            0.0,
            0.0,
            0.0,
            0.0,
            true
        },
        {
            TransitionType::QuarticEaseIn,
            "quartic ease-in",
            1.0 / 256.0,
            1.0 / 5'120.0,
            0.2,
            0.0,
            4.0,
            0.0,
            12.0,
            false
        },
        {
            TransitionType::QuarticEaseOut,
            "quartic ease-out",
            175.0 / 256.0,
            499.0 / 5'120.0,
            0.8,
            4.0,
            0.0,
            -12.0,
            0.0,
            false
        },
        {
            TransitionType::QuarticEaseInOut,
            "quartic ease-in-out",
            1.0 / 32.0,
            1.0 / 640.0,
            0.5,
            0.0,
            0.0,
            0.0,
            0.0,
            true
        },
        {
            TransitionType::QuinticEaseIn,
            "quintic ease-in",
            1.0 / 1'024.0,
            1.0 / 24'576.0,
            1.0 / 6.0,
            0.0,
            5.0,
            0.0,
            20.0,
            false
        },
        {
            TransitionType::QuinticEaseOut,
            "quintic ease-out",
            781.0 / 1'024.0,
            2'777.0 / 24'576.0,
            5.0 / 6.0,
            5.0,
            0.0,
            -20.0,
            0.0,
            false
        },
        {
            TransitionType::QuinticEaseInOut,
            "quintic ease-in-out",
            1.0 / 64.0,
            1.0 / 1'536.0,
            0.5,
            0.0,
            0.0,
            0.0,
            0.0,
            true
        }
    };

    struct PowerFamily
    {
        TransitionType easeIn;
        TransitionType easeOut;
        TransitionType easeInOut;
        unsigned int order;
    };

    constexpr PowerFamily powerFamilies[]{
        {
            TransitionType::QuadraticEaseIn,
            TransitionType::QuadraticEaseOut,
            TransitionType::QuadraticEaseInOut,
            2
        },
        {
            TransitionType::CubicEaseIn,
            TransitionType::CubicEaseOut,
            TransitionType::CubicEaseInOut,
            3
        },
        {
            TransitionType::QuarticEaseIn,
            TransitionType::QuarticEaseOut,
            TransitionType::QuarticEaseInOut,
            4
        },
        {
            TransitionType::QuinticEaseIn,
            TransitionType::QuinticEaseOut,
            TransitionType::QuinticEaseInOut,
            5
        }
    };

    void testEndpointsAndFullAreas()
    {
        for (const TransitionReference& reference : transitionReferences)
        {
            const std::string context(reference.name);
            require(
                evaluateTransition(reference.type, 0.0) == 0.0,
                context + " beginning endpoint must be exact"
            );
            require(
                evaluateTransition(reference.type, 1.0) == 1.0,
                context + " ending endpoint must be exact"
            );
            require(
                evaluateTransitionIntegral(reference.type, 0.0) == 0.0,
                context + " integral beginning must be exact"
            );
            requireNear(
                evaluateTransitionIntegral(reference.type, 1.0),
                reference.fullArea,
                context + " analytic full-domain area"
            );
        }
    }

    void testExistingReferenceValues()
    {
        constexpr std::pair<double, double> linearReferences[]{
            {0.0, 0.0}, {0.25, 0.25}, {0.5, 0.5},
            {0.75, 0.75}, {1.0, 1.0}
        };
        constexpr std::pair<double, double> smoothstepReferences[]{
            {0.0, 0.0}, {0.25, 0.15625}, {0.5, 0.5},
            {0.75, 0.84375}, {1.0, 1.0}
        };
        constexpr std::pair<double, double> smootherstepReferences[]{
            {0.0, 0.0},
            {0.1, 0.00856},
            {0.25, 53.0 / 512.0},
            {0.5, 0.5},
            {0.75, 459.0 / 512.0},
            {0.9, 0.99144},
            {1.0, 1.0}
        };

        for (const auto [progress, expected] : linearReferences)
        {
            requireNear(
                evaluateTransition(TransitionType::Linear, progress),
                expected,
                "existing linear reference"
            );
        }
        for (const auto [progress, expected] : smoothstepReferences)
        {
            requireNear(
                evaluateTransition(TransitionType::Smoothstep, progress),
                expected,
                "existing smoothstep reference"
            );
        }
        for (const auto [progress, expected] : smootherstepReferences)
        {
            requireNear(
                evaluateTransition(TransitionType::Smootherstep, progress),
                expected,
                "existing smootherstep reference"
            );
        }

        constexpr std::pair<TransitionType, double> quarterIntegrals[]{
            {TransitionType::Linear, 0.03125},
            {TransitionType::Smoothstep, 0.013671875},
            {TransitionType::Smootherstep, 0.007080078125}
        };
        constexpr std::pair<TransitionType, double> midpointIntegrals[]{
            {TransitionType::Linear, 0.125},
            {TransitionType::Smoothstep, 0.09375},
            {TransitionType::Smootherstep, 0.078125}
        };

        for (const auto [type, expected] : quarterIntegrals)
        {
            requireNear(
                evaluateTransitionIntegral(type, 0.25),
                expected,
                "existing quarter-domain integral"
            );
        }
        for (const auto [type, expected] : midpointIntegrals)
        {
            requireNear(
                evaluateTransitionIntegral(type, 0.5),
                expected,
                "existing half-domain integral"
            );
        }
    }

    void testNewInteriorReferences()
    {
        for (const TransitionReference& reference : transitionReferences)
        {
            if (reference.type == TransitionType::Linear
                || reference.type == TransitionType::Smoothstep
                || reference.type == TransitionType::Smootherstep)
            {
                continue;
            }

            const std::string context(reference.name);
            requireNear(
                evaluateTransition(reference.type, 0.25),
                reference.quarterValue,
                context + " quarter-domain value"
            );
            requireNear(
                evaluateTransitionIntegral(reference.type, 0.25),
                reference.quarterIntegral,
                context + " quarter-domain integral"
            );
        }
    }

    void testComplementSymmetry()
    {
        for (const TransitionReference& reference : transitionReferences)
        {
            if (!reference.symmetric)
            {
                continue;
            }

            for (int index = 0; index <= denseIntervalCount; ++index)
            {
                const double progress =
                    static_cast<double>(index) / denseIntervalCount;
                const double complement = evaluateTransition(
                    reference.type,
                    1.0 - progress
                );
                const double expectedComplement =
                    1.0 - evaluateTransition(reference.type, progress);
                maximumSymmetryError = std::max(
                    maximumSymmetryError,
                    std::abs(complement - expectedComplement)
                );
                const bool existingPreset =
                    reference.type == TransitionType::Linear
                    || reference.type == TransitionType::Smoothstep
                    || reference.type == TransitionType::Smootherstep;
                requireNearWithTolerance(
                    complement,
                    expectedComplement,
                    existingPreset
                        ? referenceTolerance
                        : newSymmetryTolerance,
                    std::string(reference.name) + " complement symmetry"
                );
            }

            requireNear(
                reference.fullArea,
                0.5,
                std::string(reference.name) + " symmetric area"
            );
        }
    }

    void testPowerFamilyRelationships()
    {
        for (const PowerFamily& family : powerFamilies)
        {
            const double expectedEaseInArea =
                1.0 / static_cast<double>(family.order + 1);
            const double expectedEaseOutArea =
                static_cast<double>(family.order)
                / static_cast<double>(family.order + 1);

            requireNear(
                evaluateTransitionIntegral(family.easeIn, 1.0),
                expectedEaseInArea,
                "power ease-in full area"
            );
            requireNear(
                evaluateTransitionIntegral(family.easeOut, 1.0),
                expectedEaseOutArea,
                "power ease-out full area"
            );
            requireNear(
                evaluateTransitionIntegral(family.easeInOut, 1.0),
                0.5,
                "power ease-in-out full area"
            );
            requireNear(
                expectedEaseInArea + expectedEaseOutArea,
                1.0,
                "paired power areas"
            );
            require(
                expectedEaseInArea != 0.5
                    && expectedEaseOutArea != 0.5,
                "directional power profiles must remain asymmetric"
            );

            for (int index = 0; index <= denseIntervalCount; ++index)
            {
                const double progress =
                    static_cast<double>(index) / denseIntervalCount;
                requireNear(
                    evaluateTransition(family.easeOut, progress),
                    1.0 - evaluateTransition(
                        family.easeIn,
                        1.0 - progress
                    ),
                    "paired power complement"
                );
            }
        }
    }

    void testSinusoidalFamilyRelationships()
    {
        const double easeInArea = evaluateTransitionIntegral(
            TransitionType::SineEaseIn,
            1.0
        );
        const double easeOutArea = evaluateTransitionIntegral(
            TransitionType::SineEaseOut,
            1.0
        );

        requireNear(easeInArea, 1.0 - 2.0 / pi, "sine ease-in area");
        requireNear(easeOutArea, 2.0 / pi, "sine ease-out area");
        requireNear(easeInArea + easeOutArea, 1.0, "paired sine areas");
        require(
            easeInArea != 0.5 && easeOutArea != 0.5,
            "directional sinusoidal profiles must remain asymmetric"
        );

        for (int index = 0; index <= denseIntervalCount; ++index)
        {
            const double progress =
                static_cast<double>(index) / denseIntervalCount;
            requireNear(
                evaluateTransition(TransitionType::SineEaseOut, progress),
                1.0 - evaluateTransition(
                    TransitionType::SineEaseIn,
                    1.0 - progress
                ),
                "paired sine complement"
            );
        }
    }

    void testMonotonicityAndOutputRange()
    {
        for (const TransitionReference& reference : transitionReferences)
        {
            double previousValue = evaluateTransition(reference.type, 0.0);
            double previousIntegral = evaluateTransitionIntegral(
                reference.type,
                0.0
            );

            for (int index = 0; index <= denseIntervalCount; ++index)
            {
                const double progress =
                    static_cast<double>(index) / denseIntervalCount;
                const double value = evaluateTransition(
                    reference.type,
                    progress
                );
                const double integral = evaluateTransitionIntegral(
                    reference.type,
                    progress
                );
                const std::string context(reference.name);

                require(value >= previousValue, context + " value decreased");
                require(
                    value >= 0.0 && value <= 1.0,
                    context + " value left [0, 1]"
                );
                require(
                    integral >= previousIntegral,
                    context + " integral decreased"
                );
                require(
                    integral >= 0.0 && integral <= reference.fullArea,
                    context + " integral left its analytic range"
                );
                previousValue = value;
                previousIntegral = integral;
            }
        }
    }

    [[nodiscard]] double beginningFirstDerivative(const TransitionType type)
    {
        const double f0 = evaluateTransition(type, 0.0);
        const double f1 = evaluateTransition(type, derivativeStep);
        const double f2 = evaluateTransition(type, 2.0 * derivativeStep);
        const double f3 = evaluateTransition(type, 3.0 * derivativeStep);
        const double f4 = evaluateTransition(type, 4.0 * derivativeStep);
        return (-25.0 * f0 + 48.0 * f1 - 36.0 * f2
            + 16.0 * f3 - 3.0 * f4) / (12.0 * derivativeStep);
    }

    [[nodiscard]] double endingFirstDerivative(const TransitionType type)
    {
        const double f0 = evaluateTransition(type, 1.0);
        const double f1 = evaluateTransition(type, 1.0 - derivativeStep);
        const double f2 = evaluateTransition(
            type,
            1.0 - 2.0 * derivativeStep
        );
        const double f3 = evaluateTransition(
            type,
            1.0 - 3.0 * derivativeStep
        );
        const double f4 = evaluateTransition(
            type,
            1.0 - 4.0 * derivativeStep
        );
        return (25.0 * f0 - 48.0 * f1 + 36.0 * f2
            - 16.0 * f3 + 3.0 * f4) / (12.0 * derivativeStep);
    }

    [[nodiscard]] double beginningSecondDerivative(const TransitionType type)
    {
        const double f0 = evaluateTransition(type, 0.0);
        const double f1 = evaluateTransition(type, derivativeStep);
        const double f2 = evaluateTransition(type, 2.0 * derivativeStep);
        const double f3 = evaluateTransition(type, 3.0 * derivativeStep);
        const double f4 = evaluateTransition(type, 4.0 * derivativeStep);
        return (35.0 * f0 - 104.0 * f1 + 114.0 * f2
            - 56.0 * f3 + 11.0 * f4)
            / (12.0 * derivativeStep * derivativeStep);
    }

    [[nodiscard]] double endingSecondDerivative(const TransitionType type)
    {
        const double f0 = evaluateTransition(type, 1.0);
        const double f1 = evaluateTransition(type, 1.0 - derivativeStep);
        const double f2 = evaluateTransition(
            type,
            1.0 - 2.0 * derivativeStep
        );
        const double f3 = evaluateTransition(
            type,
            1.0 - 3.0 * derivativeStep
        );
        const double f4 = evaluateTransition(
            type,
            1.0 - 4.0 * derivativeStep
        );
        return (35.0 * f0 - 104.0 * f1 + 114.0 * f2
            - 56.0 * f3 + 11.0 * f4)
            / (12.0 * derivativeStep * derivativeStep);
    }

    void testEndpointDerivatives()
    {
        for (const TransitionReference& reference : transitionReferences)
        {
            const std::string context(reference.name);
            requireNearWithTolerance(
                beginningFirstDerivative(reference.type),
                reference.beginningFirstDerivative,
                firstDerivativeTolerance,
                context + " beginning first derivative"
            );
            requireNearWithTolerance(
                endingFirstDerivative(reference.type),
                reference.endingFirstDerivative,
                firstDerivativeTolerance,
                context + " ending first derivative"
            );
            requireNearWithTolerance(
                beginningSecondDerivative(reference.type),
                reference.beginningSecondDerivative,
                secondDerivativeTolerance,
                context + " beginning second derivative"
            );
            requireNearWithTolerance(
                endingSecondDerivative(reference.type),
                reference.endingSecondDerivative,
                secondDerivativeTolerance,
                context + " ending second derivative"
            );
        }
    }

    void testSeventhOrderThirdDerivativeFlatness()
    {
        constexpr double step = 0.0002;
        const auto value = [](const double progress)
        {
            return evaluateTransition(
                TransitionType::SeventhOrderSmoothstep,
                progress
            );
        };
        const double beginning = (
            -5.0 * value(0.0)
            + 18.0 * value(step)
            - 24.0 * value(2.0 * step)
            + 14.0 * value(3.0 * step)
            - 3.0 * value(4.0 * step)
        ) / (2.0 * step * step * step);
        const double ending = (
            5.0 * value(1.0)
            - 18.0 * value(1.0 - step)
            + 24.0 * value(1.0 - 2.0 * step)
            - 14.0 * value(1.0 - 3.0 * step)
            + 3.0 * value(1.0 - 4.0 * step)
        ) / (2.0 * step * step * step);

        requireNearWithTolerance(
            beginning,
            0.0,
            0.002,
            "seventh-order beginning third derivative"
        );
        requireNearWithTolerance(
            ending,
            0.0,
            0.002,
            "seventh-order ending third derivative"
        );
    }

    [[nodiscard]] double numericalIntegral(
        const TransitionType type,
        const double endingProgress
    )
    {
        const double step = endingProgress / quadratureIntervalCount;
        double weightedSum = evaluateTransition(type, 0.0)
            + evaluateTransition(type, endingProgress);

        for (int index = 1; index < quadratureIntervalCount; ++index)
        {
            const double progress = step * static_cast<double>(index);
            weightedSum += (index % 2 == 0 ? 2.0 : 4.0)
                * evaluateTransition(type, progress);
        }

        return weightedSum * step / 3.0;
    }

    void testAnalyticIntegralsAgainstQuadrature()
    {
        constexpr double endingProgress = 0.731;

        for (const TransitionReference& reference : transitionReferences)
        {
            const double analytic = evaluateTransitionIntegral(
                reference.type,
                endingProgress
            );
            const double numerical = numericalIntegral(
                reference.type,
                endingProgress
            );
            maximumQuadratureError = std::max(
                maximumQuadratureError,
                std::abs(analytic - numerical)
            );
            requireNearWithTolerance(
                analytic,
                numerical,
                quadratureTolerance,
                std::string(reference.name) + " analytic integral"
            );
        }
    }

    void testInvalidInput()
    {
        for (const TransitionReference& reference : transitionReferences)
        {
            const std::string context(reference.name);
            for (const double progress : {-0.001, 1.001})
            {
                requireThrows<std::out_of_range>(
                    [&reference, progress]
                    {
                        static_cast<void>(evaluateTransition(
                            reference.type,
                            progress
                        ));
                    },
                    context + " finite evaluation outside [0, 1]"
                );
                requireThrows<std::out_of_range>(
                    [&reference, progress]
                    {
                        static_cast<void>(evaluateTransitionIntegral(
                            reference.type,
                            progress
                        ));
                    },
                    context + " finite integral outside [0, 1]"
                );
            }

            for (const double progress : {
                     std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity()
                 })
            {
                requireThrows<std::invalid_argument>(
                    [&reference, progress]
                    {
                        static_cast<void>(evaluateTransition(
                            reference.type,
                            progress
                        ));
                    },
                    context + " non-finite evaluation"
                );
                requireThrows<std::invalid_argument>(
                    [&reference, progress]
                    {
                        static_cast<void>(evaluateTransitionIntegral(
                            reference.type,
                            progress
                        ));
                    },
                    context + " non-finite integral"
                );
            }
        }

        constexpr auto unsupported = static_cast<TransitionType>(-1);
        requireThrows<std::invalid_argument>(
            [unsupported]
            {
                static_cast<void>(evaluateTransition(unsupported, 0.5));
            },
            "unsupported transition evaluation"
        );
        requireThrows<std::invalid_argument>(
            [unsupported]
            {
                static_cast<void>(evaluateTransitionIntegral(
                    unsupported,
                    0.5
                ));
            },
            "unsupported transition integral"
        );
    }

    void testDeterministicRepeatedEvaluation()
    {
        for (const TransitionReference& reference : transitionReferences)
        {
            const double expectedValue = evaluateTransition(
                reference.type,
                0.371
            );
            const double expectedIntegral = evaluateTransitionIntegral(
                reference.type,
                0.371
            );
            const std::uint64_t expectedValueBits =
                std::bit_cast<std::uint64_t>(expectedValue);
            const std::uint64_t expectedIntegralBits =
                std::bit_cast<std::uint64_t>(expectedIntegral);

            for (int repetition = 0; repetition < 100; ++repetition)
            {
                const double actualValue = evaluateTransition(
                    reference.type,
                    0.371
                );
                const double actualIntegral = evaluateTransitionIntegral(
                    reference.type,
                    0.371
                );
                require(
                    std::bit_cast<std::uint64_t>(actualValue)
                        == expectedValueBits,
                    std::string(reference.name)
                        + " repeated evaluation changed bits"
                );
                require(
                    std::bit_cast<std::uint64_t>(actualIntegral)
                        == expectedIntegralBits,
                    std::string(reference.name)
                        + " repeated integral changed bits"
                );
            }
        }
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;

    const std::vector<Test> tests{
        {"exact endpoints and full areas", testEndpointsAndFullAreas},
        {"existing reference values", testExistingReferenceValues},
        {"new interior references", testNewInteriorReferences},
        {"complement symmetry", testComplementSymmetry},
        {"power family relationships", testPowerFamilyRelationships},
        {"sinusoidal family relationships", testSinusoidalFamilyRelationships},
        {"monotonicity and output range", testMonotonicityAndOutputRange},
        {"endpoint derivatives", testEndpointDerivatives},
        {
            "seventh-order third-derivative flatness",
            testSeventhOrderThirdDerivativeFlatness
        },
        {
            "analytic integrals against quadrature",
            testAnalyticIntegralsAgainstQuadrature
        },
        {"invalid input", testInvalidInput},
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
    std::cout << "Transition presets checked: "
              << std::size(transitionReferences) << '\n';
    std::cout << "Dense deterministic intervals per transition: "
              << denseIntervalCount << '\n';
    std::cout << std::setprecision(17)
              << "Maximum complement-symmetry error: "
              << maximumSymmetryError << '\n'
              << "Maximum analytic/quadrature error: "
              << maximumQuadratureError << '\n';
    return 0;
}
