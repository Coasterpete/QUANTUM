#include <quantum/geometry/BSplineCurve.hpp>
#include <quantum/geometry/CurveGeometry.hpp>
#include <quantum/geometry/NurbsCurve.hpp>

#include <array>
#include <cmath>
#include <exception>
#include <functional>
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
    using quantum::geometry::ArcLengthInversionOptions;
    using quantum::geometry::ArcLengthOptions;
    using quantum::geometry::BSplineCurve;
    using quantum::geometry::evaluateArcLength;
    using quantum::geometry::evaluateParameterAtArcLength;
    using quantum::geometry::NurbsCurve;
    using Point = glm::dvec3;

    // Arc length is computed numerically. This test tolerance is slightly
    // looser than the public default error budget to leave margin for
    // independent reference rounding and platform floating-point differences.
    constexpr double tolerance = 1.0e-8;

    class TestFailure final : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    class NonFiniteDerivativeCurve
    {
    public:
        [[nodiscard]] Point evaluateFirstDerivative(double) const noexcept
        {
            return Point{
                std::numeric_limits<double>::infinity(),
                0.0,
                0.0
            };
        }

        [[nodiscard]] std::pair<double, double> parameterDomain() const noexcept
        {
            return {0.0, 1.0};
        }

        [[nodiscard]] std::array<double, 2> knots() const noexcept
        {
            return {0.0, 1.0};
        }
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
        if (!std::isfinite(actual)
            || std::abs(actual - expected) > tolerance)
        {
            std::ostringstream message;
            message.precision(17);
            message << context << ": expected " << expected
                    << ", received " << actual;
            throw TestFailure(message.str());
        }
    }

    void requirePointNear(
        const Point& actual,
        const Point& expected,
        const std::string_view context
    )
    {
        requireNear(actual.x, expected.x, std::string(context) + " x");
        requireNear(actual.y, expected.y, std::string(context) + " y");
        requireNear(actual.z, expected.z, std::string(context) + " z");
    }

    template<typename Curve>
    void requireInversionResidual(
        const Curve& curve,
        const double parameterBegin,
        const double parameterEnd,
        const double recoveredParameter,
        const double targetArcLength,
        const std::string_view context,
        const ArcLengthInversionOptions& options = {}
    )
    {
        const double totalLength = evaluateArcLength(
            curve,
            parameterBegin,
            parameterEnd,
            options.arcLength
        );
        const double allowedResidual =
            options.arcLength.absoluteTolerance
            + options.arcLength.relativeTolerance * totalLength;
        const double actualLength = evaluateArcLength(
            curve,
            parameterBegin,
            recoveredParameter,
            options.arcLength
        );
        const double residual = std::abs(actualLength - targetArcLength);

        if (!std::isfinite(residual) || residual > allowedResidual)
        {
            std::ostringstream message;
            message.precision(17);
            message << context << ": distance residual " << residual
                    << " exceeded the inversion tolerance "
                    << allowedResidual;
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

    [[nodiscard]] BSplineCurve makeLinearBSpline(const double scale = 1.0)
    {
        return BSplineCurve(
            {
                scale * Point{0.0, 0.0, 0.0},
                scale * Point{2.0, 4.0, 6.0}
            },
            1,
            {0.0, 0.0, 1.0, 1.0}
        );
    }

    [[nodiscard]] NurbsCurve makeUnequalWeightLinearNurbs()
    {
        return NurbsCurve(
            {Point{0.0, 0.0, 0.0}, Point{2.0, 4.0, 6.0}},
            {1.0, 3.0},
            1,
            {0.0, 0.0, 1.0, 1.0}
        );
    }

    [[nodiscard]] NurbsCurve makeQuarterCircle(const double radius)
    {
        return NurbsCurve(
            {
                Point{radius, 0.0, 0.0},
                Point{radius, radius, 0.0},
                Point{0.0, radius, 0.0}
            },
            {1.0, std::sqrt(0.5), 1.0},
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
        );
    }

    [[nodiscard]] BSplineCurve makeNonCircularBSpline(
        const double scale = 1.0
    )
    {
        return BSplineCurve(
            {
                scale * Point{0.0, 0.0, 0.0},
                scale * Point{1.0, 0.0, 0.0},
                scale * Point{1.0, 1.0, 0.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
        );
    }

    [[nodiscard]] BSplineCurve makeRepeatedKnotBSpline()
    {
        return BSplineCurve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 2.0, 0.0},
                Point{2.0, 0.0, 0.0},
                Point{3.0, 3.0, 0.0},
                Point{5.0, 1.0, 0.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0}
        );
    }

    void testStraightBSplineReferences()
    {
        const BSplineCurve curve = makeLinearBSpline();
        const double fullLength = std::sqrt(56.0);

        requireNear(
            evaluateArcLength(curve, 0.0, 1.0),
            fullLength,
            "straight B-spline full length"
        );
        requireNear(
            evaluateArcLength(curve, 0.25, 0.75),
            0.5 * fullLength,
            "straight B-spline subinterval length"
        );
        require(
            evaluateArcLength(curve, 0.375, 0.375) == 0.0,
            "zero-width interval should return exactly zero"
        );
        require(
            evaluateArcLength(curve, 0.0, 1.0) >= 0.0,
            "arc length should be non-negative"
        );
    }

    void testParameterValidation()
    {
        const BSplineCurve curve = makeLinearBSpline();

        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(evaluateArcLength(curve, 0.75, 0.25));
            },
            "reversed interval"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(evaluateArcLength(
                    curve,
                    std::numeric_limits<double>::quiet_NaN(),
                    1.0
                ));
            },
            "NaN interval parameter"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(evaluateArcLength(
                    curve,
                    0.0,
                    std::numeric_limits<double>::infinity()
                ));
            },
            "infinite interval parameter"
        );
        requireThrows<std::out_of_range>(
            [&curve]
            {
                static_cast<void>(evaluateArcLength(curve, -0.001, 0.5));
            },
            "below-domain interval"
        );
        requireThrows<std::out_of_range>(
            [&curve]
            {
                static_cast<void>(evaluateArcLength(curve, 0.5, 1.001));
            },
            "above-domain interval"
        );
    }

    void testRationalStraightLine()
    {
        const NurbsCurve curve = makeUnequalWeightLinearNurbs();
        const double endpointDistance = std::sqrt(56.0);

        requireNear(
            evaluateArcLength(curve, 0.0, 1.0),
            endpointDistance,
            "unequal-weight rational straight-line full length"
        );

        // The rational position is P(u) = P1 * 3u / (1 + 2u).
        // Its scalar coordinate runs from 1/2 to 9/10 on [1/4, 3/4].
        requireNear(
            evaluateArcLength(curve, 0.25, 0.75),
            0.4 * endpointDistance,
            "unequal-weight rational straight-line subinterval"
        );
    }

    void testQuarterCircleReferences()
    {
        const NurbsCurve unitCircle = makeQuarterCircle(1.0);
        const NurbsCurve scaledCircle = makeQuarterCircle(10.0);
        const double unitLength = std::numbers::pi / 2.0;

        requireNear(
            evaluateArcLength(unitCircle, 0.0, 1.0),
            unitLength,
            "unit quarter-circle length"
        );

        // Symmetric control points and weights place u = 1/2 at 45 degrees.
        requireNear(
            evaluateArcLength(unitCircle, 0.0, 0.5),
            std::numbers::pi / 4.0,
            "unit quarter-circle symmetric half"
        );
        requireNear(
            evaluateArcLength(scaledCircle, 0.0, 1.0),
            5.0 * std::numbers::pi,
            "radius-10 quarter-circle length"
        );
        requireNear(
            evaluateArcLength(scaledCircle, 0.0, 1.0),
            10.0 * evaluateArcLength(unitCircle, 0.0, 1.0),
            "quarter-circle uniform scaling"
        );
    }

    void testNonCircularBSplineReference()
    {
        const BSplineCurve curve = makeNonCircularBSpline();

        // C'(u) = (2 - 2u, 2u, 0). Direct analytic integration gives
        // L = 1 + asinh(1) / sqrt(2), independently of the production rule.
        const double expectedLength =
            1.0 + std::asinh(1.0) / std::sqrt(2.0);
        requireNear(
            evaluateArcLength(curve, 0.0, 1.0),
            expectedLength,
            "non-circular quadratic B-spline length"
        );
    }

    void testRepeatedKnotAndAdditivity()
    {
        const BSplineCurve curve = makeRepeatedKnotBSpline();
        const double fullLength = evaluateArcLength(curve, 0.0, 2.0);
        const double leftLength = evaluateArcLength(curve, 0.0, 1.0);
        const double rightLength = evaluateArcLength(curve, 1.0, 2.0);

        requireNear(
            fullLength,
            leftLength + rightLength,
            "automatic repeated-knot split versus manual split"
        );

        const double firstPart = evaluateArcLength(curve, 0.2, 0.7);
        const double secondPart = evaluateArcLength(curve, 0.7, 1.6);
        requireNear(
            evaluateArcLength(curve, 0.2, 1.6),
            firstPart + secondPart,
            "general interval additivity"
        );
    }

    void testUniformScaling()
    {
        const BSplineCurve curve = makeNonCircularBSpline();
        const BSplineCurve scaledCurve = makeNonCircularBSpline(7.5);

        requireNear(
            evaluateArcLength(scaledCurve, 0.0, 1.0),
            7.5 * evaluateArcLength(curve, 0.0, 1.0),
            "non-circular B-spline uniform scaling"
        );
    }

    void testStationaryPoint()
    {
        const BSplineCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 0.0, 0.0},
                Point{0.0, 0.0, 0.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
        );

        // The curve reaches x = 1/2 at u = 1/2 and returns to zero. Its
        // speed is |2 - 4u|, including a valid zero at the stationary point.
        requireNear(
            evaluateArcLength(curve, 0.0, 1.0),
            1.0,
            "stationary-point arc length"
        );
    }

    void testStraightBSplineInversion()
    {
        const BSplineCurve curve = makeLinearBSpline();
        const double fullLength = std::sqrt(56.0);
        constexpr std::array<double, 5> fractions{
            0.0,
            0.25,
            0.5,
            0.75,
            1.0
        };

        for (const double fraction : fractions)
        {
            const double targetLength = fraction * fullLength;
            const double parameter = evaluateParameterAtArcLength(
                curve,
                0.0,
                1.0,
                targetLength
            );

            requireNear(
                parameter,
                fraction,
                "straight B-spline inverted parameter"
            );
            requirePointNear(
                curve.evaluate(parameter),
                fraction * Point{2.0, 4.0, 6.0},
                "straight B-spline inverted position"
            );
            requireInversionResidual(
                curve,
                0.0,
                1.0,
                parameter,
                targetLength,
                "straight B-spline inversion"
            );
        }

        require(
            evaluateParameterAtArcLength(curve, 0.0, 1.0, 0.0) == 0.0,
            "zero distance should return the exact interval beginning"
        );
        require(
            evaluateParameterAtArcLength(
                curve,
                0.0,
                1.0,
                fullLength
            ) == 1.0,
            "full distance should return the exact interval end"
        );
    }

    void testRationalStraightLineInversion()
    {
        const NurbsCurve curve = makeUnequalWeightLinearNurbs();
        const double fullLength = std::sqrt(56.0);
        const double targetLength = 0.5 * fullLength;
        const double parameter = evaluateParameterAtArcLength(
            curve,
            0.0,
            1.0,
            targetLength
        );

        // C(u) = P1 * 3u / (1 + 2u). Requiring the geometric midpoint
        // gives 3u / (1 + 2u) = 1/2, whose unique solution is u = 1/4.
        requireNear(
            parameter,
            0.25,
            "unequal-weight rational midpoint parameter"
        );
        requirePointNear(
            curve.evaluate(parameter),
            Point{1.0, 2.0, 3.0},
            "unequal-weight rational midpoint position"
        );
        require(
            std::abs(parameter - 0.5) > 0.1,
            "rational inversion must not use the parameter midpoint"
        );
        requireInversionResidual(
            curve,
            0.0,
            1.0,
            parameter,
            targetLength,
            "unequal-weight rational inversion"
        );
    }

    void testQuarterCircleInversion()
    {
        const NurbsCurve unitCircle = makeQuarterCircle(1.0);
        const double unitTarget = std::numbers::pi / 4.0;
        const double unitParameter = evaluateParameterAtArcLength(
            unitCircle,
            0.0,
            1.0,
            unitTarget
        );

        requireNear(
            unitParameter,
            0.5,
            "unit quarter-circle half-arc parameter"
        );
        requirePointNear(
            unitCircle.evaluate(unitParameter),
            Point{std::sqrt(0.5), std::sqrt(0.5), 0.0},
            "unit quarter-circle half-arc position"
        );
        requireInversionResidual(
            unitCircle,
            0.0,
            1.0,
            unitParameter,
            unitTarget,
            "unit quarter-circle half-arc inversion"
        );

        const NurbsCurve scaledCircle = makeQuarterCircle(10.0);
        const double scaledTarget = 2.5 * std::numbers::pi;
        const double scaledParameter = evaluateParameterAtArcLength(
            scaledCircle,
            0.0,
            1.0,
            scaledTarget
        );

        requireNear(
            scaledParameter,
            0.5,
            "radius-10 quarter-circle half-arc parameter"
        );
        requirePointNear(
            scaledCircle.evaluate(scaledParameter),
            Point{
                5.0 * std::sqrt(2.0),
                5.0 * std::sqrt(2.0),
                0.0
            },
            "radius-10 quarter-circle half-arc position"
        );
        requireInversionResidual(
            scaledCircle,
            0.0,
            1.0,
            scaledParameter,
            scaledTarget,
            "radius-10 quarter-circle half-arc inversion"
        );
    }

    void testSubintervalInversion()
    {
        const NurbsCurve curve = makeUnequalWeightLinearNurbs();
        constexpr double parameterBegin = 0.25;
        constexpr double parameterEnd = 0.75;
        const double intervalLength = evaluateArcLength(
            curve,
            parameterBegin,
            parameterEnd
        );
        const double targetLength = 0.5 * intervalLength;
        const double parameter = evaluateParameterAtArcLength(
            curve,
            parameterBegin,
            parameterEnd,
            targetLength
        );

        // The rational scalar position advances from 1/2 to 9/10 on this
        // interval. Its geometric midpoint is 7/10, which solves to u=7/16.
        requireNear(
            parameter,
            7.0 / 16.0,
            "rational straight-line subinterval parameter"
        );
        requirePointNear(
            curve.evaluate(parameter),
            0.7 * Point{2.0, 4.0, 6.0},
            "rational straight-line subinterval position"
        );
        requireInversionResidual(
            curve,
            parameterBegin,
            parameterEnd,
            parameter,
            targetLength,
            "rational straight-line subinterval inversion"
        );
    }

    void testMultiSpanAndRepeatedKnotInversion()
    {
        const BSplineCurve curve = makeRepeatedKnotBSpline();
        const double knotLength = evaluateArcLength(curve, 0.0, 1.0);
        const double knotParameter = evaluateParameterAtArcLength(
            curve,
            0.0,
            2.0,
            knotLength
        );

        require(
            knotParameter == 1.0,
            "known repeated-knot distance should recover the exact knot"
        );
        requireInversionResidual(
            curve,
            0.0,
            2.0,
            knotParameter,
            knotLength,
            "repeated-knot boundary inversion"
        );

        constexpr double expectedAfterKnot = 1.6;
        const double distanceAfterKnot = evaluateArcLength(
            curve,
            0.0,
            expectedAfterKnot
        );
        const double recoveredAfterKnot = evaluateParameterAtArcLength(
            curve,
            0.0,
            2.0,
            distanceAfterKnot
        );

        requireNear(
            recoveredAfterKnot,
            expectedAfterKnot,
            "multi-span parameter after repeated knot"
        );
        requireInversionResidual(
            curve,
            0.0,
            2.0,
            recoveredAfterKnot,
            distanceAfterKnot,
            "multi-span inversion after repeated knot"
        );
    }

    void testInversionRoundTrips()
    {
        const BSplineCurve bSpline = makeNonCircularBSpline();
        constexpr std::array<double, 3> bSplineParameters{0.2, 0.63, 0.9};

        for (const double knownParameter : bSplineParameters)
        {
            const double targetLength = evaluateArcLength(
                bSpline,
                0.0,
                knownParameter
            );
            const double recoveredParameter = evaluateParameterAtArcLength(
                bSpline,
                0.0,
                1.0,
                targetLength
            );

            requireNear(
                recoveredParameter,
                knownParameter,
                "B-spline inversion round trip"
            );
            requireInversionResidual(
                bSpline,
                0.0,
                1.0,
                recoveredParameter,
                targetLength,
                "B-spline inversion round-trip residual"
            );
        }

        const NurbsCurve nurbs = makeQuarterCircle(1.0);
        constexpr std::array<double, 3> nurbsParameters{0.15, 0.5, 0.83};

        for (const double knownParameter : nurbsParameters)
        {
            const double targetLength = evaluateArcLength(
                nurbs,
                0.0,
                knownParameter
            );
            const double recoveredParameter = evaluateParameterAtArcLength(
                nurbs,
                0.0,
                1.0,
                targetLength
            );

            requireNear(
                recoveredParameter,
                knownParameter,
                "NURBS inversion round trip"
            );
            requireInversionResidual(
                nurbs,
                0.0,
                1.0,
                recoveredParameter,
                targetLength,
                "NURBS inversion round-trip residual"
            );
        }
    }

    void testZeroLengthInversionPolicies()
    {
        const BSplineCurve curve = makeLinearBSpline();

        require(
            evaluateParameterAtArcLength(curve, 0.375, 0.375, 0.0)
                == 0.375,
            "zero-width zero-distance inversion should return its beginning"
        );
        requireThrows<std::out_of_range>(
            [&curve]
            {
                static_cast<void>(evaluateParameterAtArcLength(
                    curve,
                    0.375,
                    0.375,
                    1.0e-12
                ));
            },
            "positive target on a zero-width interval"
        );

        const BSplineCurve degenerateCurve(
            {Point{1.0, 2.0, 3.0}, Point{1.0, 2.0, 3.0}},
            1,
            {0.0, 0.0, 1.0, 1.0}
        );

        require(
            evaluateParameterAtArcLength(
                degenerateCurve,
                0.0,
                1.0,
                0.0
            ) == 0.0,
            "degenerate zero-length curve should select the interval beginning"
        );
        requireThrows<std::out_of_range>(
            [&degenerateCurve]
            {
                static_cast<void>(evaluateParameterAtArcLength(
                    degenerateCurve,
                    0.0,
                    1.0,
                    1.0e-12
                ));
            },
            "positive target on a degenerate zero-length curve"
        );
    }

    void testStationaryPointInversion()
    {
        const BSplineCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 0.0, 0.0},
                Point{0.0, 0.0, 0.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
        );
        const double targetLength = evaluateArcLength(curve, 0.0, 0.5);
        const double parameter = evaluateParameterAtArcLength(
            curve,
            0.0,
            1.0,
            targetLength
        );

        require(
            parameter == 0.5,
            "stationary-point distance should recover its exact parameter"
        );
        requireInversionResidual(
            curve,
            0.0,
            1.0,
            parameter,
            targetLength,
            "stationary-point inversion"
        );
    }

    void testInversionValidationAndEndpointTolerance()
    {
        const BSplineCurve curve = makeLinearBSpline();
        const double fullLength = evaluateArcLength(curve, 0.0, 1.0);
        const ArcLengthInversionOptions options;
        const double endpointTolerance =
            options.arcLength.absoluteTolerance
            + options.arcLength.relativeTolerance * fullLength;

        requireThrows<std::out_of_range>(
            [&curve]
            {
                static_cast<void>(evaluateParameterAtArcLength(
                    curve,
                    0.0,
                    1.0,
                    -1.0
                ));
            },
            "negative inversion target"
        );
        requireThrows<std::out_of_range>(
            [&curve, fullLength, endpointTolerance]
            {
                static_cast<void>(evaluateParameterAtArcLength(
                    curve,
                    0.0,
                    1.0,
                    fullLength + 2.0 * endpointTolerance
                ));
            },
            "above-total inversion target"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(evaluateParameterAtArcLength(
                    curve,
                    0.0,
                    1.0,
                    std::numeric_limits<double>::quiet_NaN()
                ));
            },
            "NaN inversion target"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(evaluateParameterAtArcLength(
                    curve,
                    0.0,
                    1.0,
                    std::numeric_limits<double>::infinity()
                ));
            },
            "infinite inversion target"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(evaluateParameterAtArcLength(
                    curve,
                    std::numeric_limits<double>::quiet_NaN(),
                    1.0,
                    0.0
                ));
            },
            "NaN inversion interval parameter"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(evaluateParameterAtArcLength(
                    curve,
                    0.0,
                    std::numeric_limits<double>::infinity(),
                    0.0
                ));
            },
            "infinite inversion interval parameter"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(evaluateParameterAtArcLength(
                    curve,
                    0.75,
                    0.25,
                    0.0
                ));
            },
            "reversed inversion interval"
        );

        require(
            evaluateParameterAtArcLength(
                curve,
                0.0,
                1.0,
                fullLength + 0.5 * endpointTolerance
            ) == 1.0,
            "endpoint discrepancy within the arc-length tolerance should select the end"
        );
    }

    void testBoundedInversionFailure()
    {
        const BSplineCurve curve = makeNonCircularBSpline();
        const double fullLength = evaluateArcLength(curve, 0.0, 1.0);
        const ArcLengthInversionOptions noIterations{
            .maximumIterations = 0
        };

        requireThrows<std::runtime_error>(
            [&curve, fullLength, &noIterations]
            {
                static_cast<void>(evaluateParameterAtArcLength(
                    curve,
                    0.0,
                    1.0,
                    0.3 * fullLength,
                    noIterations
                ));
            },
            "bounded inversion failure"
        );
    }

    void testDeterministicRepeatedInversion()
    {
        const NurbsCurve curve = makeQuarterCircle(1.0);
        const double targetLength = 0.37 * evaluateArcLength(
            curve,
            0.0,
            1.0
        );
        const double expected = evaluateParameterAtArcLength(
            curve,
            0.0,
            1.0,
            targetLength
        );

        for (int repetition = 0; repetition < 100; ++repetition)
        {
            require(
                evaluateParameterAtArcLength(
                    curve,
                    0.0,
                    1.0,
                    targetLength
                ) == expected,
                "repeated arc-length inversion changed its result"
            );
        }
    }

    void testDeterministicRepeatedEvaluation()
    {
        const NurbsCurve curve = makeQuarterCircle(1.0);
        const double expected = evaluateArcLength(curve, 0.125, 0.875);

        for (int repetition = 0; repetition < 100; ++repetition)
        {
            require(
                evaluateArcLength(curve, 0.125, 0.875) == expected,
                "repeated arc-length evaluation changed its result"
            );
        }
    }

    void testNumericalFailureBehavior()
    {
        const BSplineCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 0.0, 0.0},
                Point{0.0, 0.0, 0.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
        );
        const ArcLengthOptions deliberatelyUnachievable{
            .absoluteTolerance = std::numeric_limits<double>::denorm_min(),
            .relativeTolerance = 0.0,
            .maximumSubdivisionDepth = 0
        };

        requireThrows<std::runtime_error>(
            [&curve, &deliberatelyUnachievable]
            {
                static_cast<void>(evaluateArcLength(
                    curve,
                    0.0,
                    1.0,
                    deliberatelyUnachievable
                ));
            },
            "bounded integration failure"
        );
    }

    void testNonFiniteDerivativeRejection()
    {
        const NonFiniteDerivativeCurve curve;

        requireThrows<std::domain_error>(
            [&curve]
            {
                static_cast<void>(evaluateArcLength(curve, 0.0, 1.0));
            },
            "non-finite derivative magnitude"
        );
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;

    const std::vector<Test> tests{
        {"straight B-spline references", testStraightBSplineReferences},
        {"parameter validation", testParameterValidation},
        {"rational straight line", testRationalStraightLine},
        {"quarter-circle references", testQuarterCircleReferences},
        {"non-circular B-spline reference", testNonCircularBSplineReference},
        {"repeated knot and additivity", testRepeatedKnotAndAdditivity},
        {"uniform scaling", testUniformScaling},
        {"stationary point", testStationaryPoint},
        {"straight B-spline inversion", testStraightBSplineInversion},
        {"rational straight-line inversion", testRationalStraightLineInversion},
        {"quarter-circle inversion", testQuarterCircleInversion},
        {"subinterval inversion", testSubintervalInversion},
        {
            "multi-span and repeated-knot inversion",
            testMultiSpanAndRepeatedKnotInversion
        },
        {"inversion round trips", testInversionRoundTrips},
        {"zero-length inversion policies", testZeroLengthInversionPolicies},
        {"stationary-point inversion", testStationaryPointInversion},
        {
            "inversion validation and endpoint tolerance",
            testInversionValidationAndEndpointTolerance
        },
        {"bounded inversion failure", testBoundedInversionFailure},
        {
            "deterministic repeated inversion",
            testDeterministicRepeatedInversion
        },
        {"deterministic repeated evaluation", testDeterministicRepeatedEvaluation},
        {"numerical failure behavior", testNumericalFailureBehavior},
        {"non-finite derivative rejection", testNonFiniteDerivativeRejection}
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
