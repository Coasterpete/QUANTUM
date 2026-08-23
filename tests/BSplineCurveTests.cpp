#include <quantum/geometry/BSplineCurve.hpp>

#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::geometry::BSplineCurve;
    using Point = BSplineCurve::Point;

    // Expected coordinates in these tests are small, independently calculated
    // values. An absolute tolerance of 1e-12 accommodates double-precision
    // rounding without hiding a meaningful evaluation error.
    constexpr double tolerance = 1.0e-12;

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
        if (!std::isfinite(actual)
            || std::abs(actual - expected) > tolerance)
        {
            std::ostringstream message;
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

    [[nodiscard]] BSplineCurve makeLinearCurve()
    {
        return BSplineCurve(
            {
                Point{0.0, 0.0, 0.0},
                Point{2.0, 4.0, 6.0},
                Point{6.0, 4.0, 2.0}
            },
            1,
            {0.0, 0.0, 1.0, 2.0, 2.0}
        );
    }

    [[nodiscard]] BSplineCurve makeQuadraticReferenceCurve()
    {
        return BSplineCurve(
            {
                Point{0.0, 0.0, 0.0},
                Point{2.0, 4.0, 0.0},
                Point{4.0, 0.0, 2.0},
                Point{6.0, 2.0, 4.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0}
        );
    }

    void testValidDefinition()
    {
        const BSplineCurve curve = makeQuadraticReferenceCurve();
        const auto [domainStart, domainEnd] = curve.parameterDomain();

        require(curve.degree() == 2, "degree was not preserved");
        require(curve.controlPoints().size() == 4, "control points were not preserved");
        require(curve.knots().size() == 7, "knots were not preserved");
        requireNear(domainStart, 0.0, "domain start");
        requireNear(domainEnd, 2.0, "domain end");
    }

    void testInvalidDefinitions()
    {
        requireThrows<std::invalid_argument>(
            []
            {
                BSplineCurve(
                    {Point{0.0, 0.0, 0.0}},
                    -1,
                    {0.0}
                );
            },
            "negative degree"
        );

        requireThrows<std::invalid_argument>(
            []
            {
                BSplineCurve(
                    {
                        Point{0.0, 0.0, 0.0},
                        Point{1.0, 0.0, 0.0},
                        Point{2.0, 0.0, 0.0}
                    },
                    3,
                    {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
                );
            },
            "insufficient control points"
        );

        requireThrows<std::invalid_argument>(
            []
            {
                BSplineCurve(
                    {
                        Point{0.0, 0.0, 0.0},
                        Point{1.0, 0.0, 0.0},
                        Point{2.0, 0.0, 0.0}
                    },
                    2,
                    {0.0, 0.0, 0.0, 1.0, 1.0}
                );
            },
            "incorrect knot count"
        );

        requireThrows<std::invalid_argument>(
            []
            {
                BSplineCurve(
                    {
                        Point{0.0, 0.0, 0.0},
                        Point{1.0, 0.0, 0.0}
                    },
                    1,
                    {0.0, 0.0, 1.0, 0.5}
                );
            },
            "non-monotonic knot vector"
        );
    }

    void testAdditionalMalformedDefinitions()
    {
        const double infinity = std::numeric_limits<double>::infinity();

        requireThrows<std::invalid_argument>(
            [infinity]
            {
                BSplineCurve(
                    {
                        Point{0.0, 0.0, 0.0},
                        Point{infinity, 0.0, 0.0}
                    },
                    1,
                    {0.0, 0.0, 1.0, 1.0}
                );
            },
            "non-finite control point"
        );

        requireThrows<std::invalid_argument>(
            [infinity]
            {
                BSplineCurve(
                    {
                        Point{0.0, 0.0, 0.0},
                        Point{1.0, 0.0, 0.0}
                    },
                    1,
                    {0.0, 0.0, infinity, infinity}
                );
            },
            "non-finite knot"
        );

        requireThrows<std::invalid_argument>(
            []
            {
                BSplineCurve(
                    {
                        Point{0.0, 0.0, 0.0},
                        Point{1.0, 0.0, 0.0},
                        Point{2.0, 0.0, 0.0}
                    },
                    1,
                    {0.0, 0.0, 0.0, 1.0, 1.0}
                );
            },
            "excessive knot multiplicity"
        );

        requireThrows<std::invalid_argument>(
            []
            {
                BSplineCurve(
                    {
                        Point{0.0, 0.0, 0.0},
                        Point{1.0, 0.0, 0.0}
                    },
                    1,
                    {-1.0, 0.0, 0.0, 1.0}
                );
            },
            "zero-length parameter domain"
        );
    }

    void testParameterRangeValidation()
    {
        const BSplineCurve curve = makeLinearCurve();

        requireThrows<std::out_of_range>(
            [&curve] { static_cast<void>(curve.evaluate(-0.001)); },
            "parameter below domain"
        );
        requireThrows<std::out_of_range>(
            [&curve] { static_cast<void>(curve.evaluate(2.001)); },
            "parameter above domain"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(curve.evaluate(
                    std::numeric_limits<double>::quiet_NaN()
                ));
            },
            "non-finite parameter"
        );
    }

    void testFirstDerivativeParameterRangeValidation()
    {
        const BSplineCurve curve = makeLinearCurve();

        requireThrows<std::out_of_range>(
            [&curve]
            {
                static_cast<void>(curve.evaluateFirstDerivative(-0.001));
            },
            "derivative parameter below domain"
        );
        requireThrows<std::out_of_range>(
            [&curve]
            {
                static_cast<void>(curve.evaluateFirstDerivative(2.001));
            },
            "derivative parameter above domain"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(curve.evaluateFirstDerivative(
                    std::numeric_limits<double>::quiet_NaN()
                ));
            },
            "non-finite derivative parameter"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(curve.evaluateFirstDerivative(
                    std::numeric_limits<double>::infinity()
                ));
            },
            "infinite derivative parameter"
        );
    }

    void testSecondDerivativeParameterRangeValidation()
    {
        const BSplineCurve curve = makeLinearCurve();

        requireThrows<std::out_of_range>(
            [&curve]
            {
                static_cast<void>(curve.evaluateSecondDerivative(-0.001));
            },
            "second-derivative parameter below domain"
        );
        requireThrows<std::out_of_range>(
            [&curve]
            {
                static_cast<void>(curve.evaluateSecondDerivative(2.001));
            },
            "second-derivative parameter above domain"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(curve.evaluateSecondDerivative(
                    std::numeric_limits<double>::quiet_NaN()
                ));
            },
            "non-finite second-derivative parameter"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(curve.evaluateSecondDerivative(
                    std::numeric_limits<double>::infinity()
                ));
            },
            "infinite second-derivative parameter"
        );
    }

    void testDegreeOneLinearInterpolationAndClampedEndpoints()
    {
        const BSplineCurve curve = makeLinearCurve();

        requirePointNear(
            curve.evaluate(0.0),
            Point{0.0, 0.0, 0.0},
            "clamped domain beginning"
        );
        requirePointNear(
            curve.evaluate(2.0),
            Point{6.0, 4.0, 2.0},
            "clamped domain end"
        );
        requirePointNear(
            curve.evaluate(0.25),
            Point{0.5, 1.0, 1.5},
            "first linear span"
        );
        requirePointNear(
            curve.evaluate(1.5),
            Point{4.0, 4.0, 4.0},
            "second linear span"
        );
    }

    void testDegreeZeroBehavior()
    {
        const BSplineCurve curve(
            {
                Point{1.0, 2.0, 3.0},
                Point{4.0, 5.0, 6.0}
            },
            0,
            {0.0, 1.0, 2.0}
        );

        requirePointNear(
            curve.evaluate(0.5),
            Point{1.0, 2.0, 3.0},
            "degree-zero first span"
        );
        requirePointNear(
            curve.evaluate(1.0),
            Point{4.0, 5.0, 6.0},
            "degree-zero interior knot"
        );
        requirePointNear(
            curve.evaluate(2.0),
            Point{4.0, 5.0, 6.0},
            "degree-zero domain end"
        );
    }

    void testDegreeZeroFirstDerivative()
    {
        const BSplineCurve curve(
            {
                Point{1.0, 2.0, 3.0},
                Point{4.0, 5.0, 6.0}
            },
            0,
            {0.0, 1.0, 2.0}
        );
        const Point zero{0.0, 0.0, 0.0};

        requirePointNear(
            curve.evaluateFirstDerivative(0.0),
            zero,
            "degree-zero derivative at domain beginning"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(0.5),
            zero,
            "degree-zero derivative within first span"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(1.0),
            zero,
            "degree-zero derivative at interior knot"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(2.0),
            zero,
            "degree-zero derivative at domain end"
        );
    }

    void testDegreeZeroSecondDerivative()
    {
        const BSplineCurve curve(
            {
                Point{1.0, 2.0, 3.0},
                Point{4.0, 5.0, 6.0}
            },
            0,
            {0.0, 1.0, 2.0}
        );
        const Point zero{0.0, 0.0, 0.0};

        requirePointNear(
            curve.evaluateSecondDerivative(0.0),
            zero,
            "degree-zero second derivative at domain beginning"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(0.5),
            zero,
            "degree-zero second derivative within first span"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(1.0),
            zero,
            "degree-zero second derivative at interior knot"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(2.0),
            zero,
            "degree-zero second derivative at domain end"
        );
    }

    void testDegreeOneFirstDerivative()
    {
        const BSplineCurve curve = makeLinearCurve();

        // Each degree-one span has the constant derivative
        // (P_{i + 1} - P_i) / (U_{i + 2} - U_{i + 1}). The exact
        // interior knot selects the span to its right.
        requirePointNear(
            curve.evaluateFirstDerivative(0.0),
            Point{2.0, 4.0, 6.0},
            "linear derivative at domain beginning"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(0.25),
            Point{2.0, 4.0, 6.0},
            "linear derivative within first span"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(1.0),
            Point{4.0, 0.0, -4.0},
            "linear derivative at interior knot"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(2.0),
            Point{4.0, 0.0, -4.0},
            "linear derivative at domain end"
        );
    }

    void testDegreeOneSecondDerivative()
    {
        const BSplineCurve curve = makeLinearCurve();
        const Point zero{0.0, 0.0, 0.0};

        requirePointNear(
            curve.evaluateSecondDerivative(0.0),
            zero,
            "linear second derivative at domain beginning"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(0.25),
            zero,
            "linear second derivative within first span"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(1.0),
            zero,
            "linear second derivative at interior knot"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(2.0),
            zero,
            "linear second derivative at domain end"
        );
    }

    void testQuadraticReferenceValues()
    {
        const BSplineCurve curve = makeQuadraticReferenceCurve();

        requirePointNear(
            curve.evaluate(0.0),
            Point{0.0, 0.0, 0.0},
            "quadratic clamped beginning"
        );
        requirePointNear(
            curve.evaluate(2.0),
            Point{6.0, 2.0, 4.0},
            "quadratic clamped end"
        );

        // At u = 0.5 the nonzero basis values are 0.25, 0.625, and
        // 0.125. Applying those weights to P0, P1, and P2 gives this
        // independently calculated coordinate.
        requirePointNear(
            curve.evaluate(0.5),
            Point{1.75, 2.5, 0.25},
            "quadratic reference at u = 0.5"
        );

        // At the interior knot u = 1, P1 and P2 each have weight 0.5.
        requirePointNear(
            curve.evaluate(1.0),
            Point{3.0, 2.0, 1.0},
            "quadratic reference at u = 1"
        );
    }

    void testQuadraticFirstDerivativeReferenceValues()
    {
        const BSplineCurve curve = makeQuadraticReferenceCurve();

        requirePointNear(
            curve.evaluateFirstDerivative(0.0),
            Point{4.0, 8.0, 0.0},
            "quadratic derivative at clamped beginning"
        );

        // Independently differentiating the first-span basis functions gives
        // N0' = -2 + 2u, N1' = 2 - 3u, and N2' = u. Applying those values
        // to P0, P1, and P2 at u = 0.5 gives (3, 2, 1).
        requirePointNear(
            curve.evaluateFirstDerivative(0.5),
            Point{3.0, 2.0, 1.0},
            "quadratic derivative at u = 0.5"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(1.0),
            Point{2.0, -4.0, 2.0},
            "quadratic derivative at interior knot"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(2.0),
            Point{4.0, 4.0, 4.0},
            "quadratic derivative at clamped end"
        );
    }

    void testQuadraticSecondDerivativeReferenceValues()
    {
        const BSplineCurve curve = makeQuadraticReferenceCurve();

        // On [0, 1), twice differentiating the polynomial basis gives
        // N0'' = 2, N1'' = -3, and N2'' = 1. Thus
        // C'' = 2P0 - 3P1 + P2 = (-2, -12, 2).
        requirePointNear(
            curve.evaluateSecondDerivative(0.0),
            Point{-2.0, -12.0, 2.0},
            "quadratic second derivative at clamped beginning"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(0.5),
            Point{-2.0, -12.0, 2.0},
            "quadratic second derivative at u = 0.5"
        );

        // The right-hand span selected at u = 1 has constant second
        // derivative (2, 8, 2), which is also the left-hand endpoint value.
        requirePointNear(
            curve.evaluateSecondDerivative(1.0),
            Point{2.0, 8.0, 2.0},
            "quadratic second derivative at interior knot"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(2.0),
            Point{2.0, 8.0, 2.0},
            "quadratic second derivative at clamped end"
        );
    }

    void testCubicFirstDerivativeReferenceValue()
    {
        const BSplineCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 2.0, 0.0},
                Point{3.0, 3.0, 1.0},
                Point{4.0, 0.0, 2.0}
            },
            3,
            {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}
        );

        // This clamped cubic is a Bezier curve. At u = 0.25 its derivative
        // control points (3, 6, 0), (6, 3, 3), and (3, -9, 3) have quadratic
        // Bernstein weights 0.5625, 0.375, and 0.0625, respectively.
        requirePointNear(
            curve.evaluateFirstDerivative(0.25),
            Point{4.125, 3.9375, 1.3125},
            "cubic derivative at u = 0.25"
        );
    }

    void testCubicSecondDerivativeReferenceValue()
    {
        const BSplineCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 2.0, 0.0},
                Point{3.0, 3.0, 1.0},
                Point{4.0, 0.0, 2.0}
            },
            3,
            {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}
        );

        // This cubic Bezier has second-derivative control points
        // 6(P2 - 2P1 + P0) = (6, -6, 6) and
        // 6(P3 - 2P2 + P1) = (-6, -24, 0). Their linear Bernstein
        // weights at u = 0.25 are 0.75 and 0.25.
        requirePointNear(
            curve.evaluateSecondDerivative(0.25),
            Point{3.0, -10.5, 4.5},
            "cubic second derivative at u = 0.25"
        );
    }

    void testRepeatedKnotFirstDerivativeBehavior()
    {
        const BSplineCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 2.0, 0.0},
                Point{2.0, 0.0, 1.0},
                Point{4.0, 3.0, 2.0},
                Point{5.0, 3.0, 4.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0}
        );

        // The double interior knot reduces this quadratic curve to C0 there.
        // The exact knot follows evaluate() and selects the right-hand span,
        // whose first derivative control point is (4, 6, 2). The left-hand
        // derivative would be (2, -4, 2), so this also verifies the convention.
        requirePointNear(
            curve.evaluateFirstDerivative(1.0),
            Point{4.0, 6.0, 2.0},
            "derivative at repeated interior knot"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(1.5),
            Point{3.0, 3.0, 3.0},
            "derivative after repeated interior knot"
        );
    }

    void testRepeatedKnotSecondDerivativeBehavior()
    {
        const BSplineCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 2.0, 0.0},
                Point{2.0, 0.0, 1.0},
                Point{4.0, 3.0, 2.0},
                Point{5.0, 3.0, 4.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0}
        );

        // The exact double knot selects the quadratic span on [1, 2].
        // Its derivative control points are (4, 6, 2) and (2, 0, 4),
        // so its constant second derivative is (-2, -6, 2).
        requirePointNear(
            curve.evaluateSecondDerivative(1.0),
            Point{-2.0, -6.0, 2.0},
            "second derivative at repeated interior knot"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(1.5),
            Point{-2.0, -6.0, 2.0},
            "second derivative after repeated interior knot"
        );
    }

    void testUnclampedBoundaryBehavior()
    {
        const BSplineCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{2.0, 4.0, 0.0},
                Point{4.0, 0.0, 2.0},
                Point{6.0, 2.0, 4.0}
            },
            2,
            {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}
        );

        const auto [domainStart, domainEnd] = curve.parameterDomain();
        requireNear(domainStart, 2.0, "unclamped domain start");
        requireNear(domainEnd, 4.0, "unclamped domain end");
        requirePointNear(
            curve.evaluate(domainStart),
            Point{1.0, 2.0, 0.0},
            "unclamped domain beginning"
        );
        requirePointNear(
            curve.evaluate(domainEnd),
            Point{5.0, 1.0, 3.0},
            "unclamped domain end"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(domainStart),
            Point{2.0, 4.0, 0.0},
            "unclamped derivative at domain beginning"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(domainEnd),
            Point{2.0, 2.0, 2.0},
            "unclamped derivative at domain end"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(domainStart),
            Point{0.0, -8.0, 2.0},
            "unclamped second derivative at domain beginning"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(3.0),
            Point{0.0, 6.0, 0.0},
            "unclamped second derivative at interior knot"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(domainEnd),
            Point{0.0, 6.0, 0.0},
            "unclamped second derivative at domain end"
        );
    }

    void testNonFiniteSecondDerivativeIsRejected()
    {
        const double largestFinite =
            std::numeric_limits<double>::max();
        const BSplineCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{largestFinite / 2.0, 0.0, 0.0},
                Point{0.0, 0.0, 0.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
        );

        requireThrows<std::domain_error>(
            [&curve]
            {
                static_cast<void>(curve.evaluateSecondDerivative(0.5));
            },
            "non-finite B-spline second derivative"
        );
    }

    void testDeterministicRepeatedEvaluation()
    {
        const BSplineCurve curve = makeQuadraticReferenceCurve();
        const Point expected = curve.evaluate(0.625);
        const Point expectedDerivative = curve.evaluateFirstDerivative(0.625);
        const Point expectedSecondDerivative =
            curve.evaluateSecondDerivative(0.625);

        for (int repetition = 0; repetition < 100; ++repetition)
        {
            const Point actual = curve.evaluate(0.625);
            const Point actualDerivative =
                curve.evaluateFirstDerivative(0.625);
            const Point actualSecondDerivative =
                curve.evaluateSecondDerivative(0.625);
            require(
                actual.x == expected.x
                    && actual.y == expected.y
                    && actual.z == expected.z,
                "repeated evaluation changed its result"
            );
            require(
                actualDerivative.x == expectedDerivative.x
                    && actualDerivative.y == expectedDerivative.y
                    && actualDerivative.z == expectedDerivative.z,
                "repeated derivative evaluation changed its result"
            );
            require(
                actualSecondDerivative.x == expectedSecondDerivative.x
                    && actualSecondDerivative.y == expectedSecondDerivative.y
                    && actualSecondDerivative.z == expectedSecondDerivative.z,
                "repeated second-derivative evaluation changed its result"
            );
        }
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;

    const std::vector<Test> tests{
        {"valid definition", testValidDefinition},
        {"invalid definitions", testInvalidDefinitions},
        {"additional malformed definitions", testAdditionalMalformedDefinitions},
        {"parameter range validation", testParameterRangeValidation},
        {
            "first-derivative parameter range validation",
            testFirstDerivativeParameterRangeValidation
        },
        {
            "second-derivative parameter range validation",
            testSecondDerivativeParameterRangeValidation
        },
        {
            "degree-one interpolation and clamped endpoints",
            testDegreeOneLinearInterpolationAndClampedEndpoints
        },
        {"degree-zero behavior", testDegreeZeroBehavior},
        {"degree-zero first derivative", testDegreeZeroFirstDerivative},
        {"degree-zero second derivative", testDegreeZeroSecondDerivative},
        {"degree-one first derivative", testDegreeOneFirstDerivative},
        {"degree-one second derivative", testDegreeOneSecondDerivative},
        {"quadratic reference values", testQuadraticReferenceValues},
        {
            "quadratic first-derivative reference values",
            testQuadraticFirstDerivativeReferenceValues
        },
        {
            "quadratic second-derivative reference values",
            testQuadraticSecondDerivativeReferenceValues
        },
        {
            "cubic first-derivative reference value",
            testCubicFirstDerivativeReferenceValue
        },
        {
            "cubic second-derivative reference value",
            testCubicSecondDerivativeReferenceValue
        },
        {
            "repeated-knot first-derivative behavior",
            testRepeatedKnotFirstDerivativeBehavior
        },
        {
            "repeated-knot second-derivative behavior",
            testRepeatedKnotSecondDerivativeBehavior
        },
        {"unclamped boundary behavior", testUnclampedBoundaryBehavior},
        {
            "non-finite second-derivative rejection",
            testNonFiniteSecondDerivativeIsRejected
        },
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
