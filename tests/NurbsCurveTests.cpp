#include <quantum/geometry/BSplineCurve.hpp>
#include <quantum/geometry/NurbsCurve.hpp>

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
    using quantum::geometry::NurbsCurve;
    using Point = NurbsCurve::Point;

    // Expected coordinates are independently calculated small values. An
    // absolute tolerance of 1e-12 covers double-precision rounding without
    // hiding a meaningful geometric error.
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

    [[nodiscard]] NurbsCurve makeWeightedLinearCurve()
    {
        return NurbsCurve(
            {
                Point{0.0, 0.0, 0.0},
                Point{2.0, 4.0, 6.0}
            },
            {1.0, 3.0},
            1,
            {0.0, 0.0, 1.0, 1.0}
        );
    }

    void testValidDefinition()
    {
        const NurbsCurve curve = makeWeightedLinearCurve();
        const auto [domainStart, domainEnd] = curve.parameterDomain();

        require(curve.degree() == 1, "degree was not preserved");
        require(curve.controlPoints().size() == 2, "control points were not preserved");
        require(curve.weights().size() == 2, "weights were not preserved");
        require(curve.knots().size() == 4, "knots were not preserved");
        requireNear(curve.weights()[0], 1.0, "first weight");
        requireNear(curve.weights()[1], 3.0, "second weight");
        requireNear(domainStart, 0.0, "domain start");
        requireNear(domainEnd, 1.0, "domain end");
    }

    void testWeightValidation()
    {
        const double infinity = std::numeric_limits<double>::infinity();
        const double nan = std::numeric_limits<double>::quiet_NaN();

        requireThrows<std::invalid_argument>(
            []
            {
                NurbsCurve(
                    {Point{0.0, 0.0, 0.0}, Point{1.0, 0.0, 0.0}},
                    {1.0},
                    1,
                    {0.0, 0.0, 1.0, 1.0}
                );
            },
            "mismatched weight count"
        );
        requireThrows<std::invalid_argument>(
            [infinity]
            {
                NurbsCurve(
                    {Point{0.0, 0.0, 0.0}, Point{1.0, 0.0, 0.0}},
                    {1.0, infinity},
                    1,
                    {0.0, 0.0, 1.0, 1.0}
                );
            },
            "infinite weight"
        );
        requireThrows<std::invalid_argument>(
            [nan]
            {
                NurbsCurve(
                    {Point{0.0, 0.0, 0.0}, Point{1.0, 0.0, 0.0}},
                    {1.0, nan},
                    1,
                    {0.0, 0.0, 1.0, 1.0}
                );
            },
            "NaN weight"
        );
        requireThrows<std::invalid_argument>(
            []
            {
                NurbsCurve(
                    {Point{0.0, 0.0, 0.0}, Point{1.0, 0.0, 0.0}},
                    {1.0, 0.0},
                    1,
                    {0.0, 0.0, 1.0, 1.0}
                );
            },
            "zero weight"
        );
        requireThrows<std::invalid_argument>(
            []
            {
                NurbsCurve(
                    {Point{0.0, 0.0, 0.0}, Point{1.0, 0.0, 0.0}},
                    {1.0, -1.0},
                    1,
                    {0.0, 0.0, 1.0, 1.0}
                );
            },
            "negative weight"
        );
    }

    void testInvalidDegreeAndKnotDefinitions()
    {
        requireThrows<std::invalid_argument>(
            []
            {
                NurbsCurve(
                    {Point{0.0, 0.0, 0.0}},
                    {1.0},
                    -1,
                    {0.0}
                );
            },
            "negative degree"
        );
        requireThrows<std::invalid_argument>(
            []
            {
                NurbsCurve(
                    {
                        Point{0.0, 0.0, 0.0},
                        Point{1.0, 0.0, 0.0},
                        Point{2.0, 0.0, 0.0}
                    },
                    {1.0, 1.0, 1.0},
                    3,
                    {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
                );
            },
            "insufficient control points"
        );
        requireThrows<std::invalid_argument>(
            []
            {
                NurbsCurve(
                    {Point{0.0, 0.0, 0.0}, Point{1.0, 0.0, 0.0}},
                    {1.0, 1.0},
                    1,
                    {0.0, 0.0, 1.0}
                );
            },
            "incorrect knot count"
        );
        requireThrows<std::invalid_argument>(
            []
            {
                NurbsCurve(
                    {Point{0.0, 0.0, 0.0}, Point{1.0, 0.0, 0.0}},
                    {1.0, 1.0},
                    1,
                    {0.0, 0.0, 1.0, 0.5}
                );
            },
            "non-monotonic knots"
        );
        requireThrows<std::invalid_argument>(
            []
            {
                NurbsCurve(
                    {
                        Point{0.0, 0.0, 0.0},
                        Point{1.0, 0.0, 0.0},
                        Point{2.0, 0.0, 0.0}
                    },
                    {1.0, 1.0, 1.0},
                    1,
                    {0.0, 0.0, 0.0, 1.0, 1.0}
                );
            },
            "excessive knot multiplicity"
        );
        requireThrows<std::invalid_argument>(
            []
            {
                NurbsCurve(
                    {Point{0.0, 0.0, 0.0}, Point{1.0, 0.0, 0.0}},
                    {1.0, 1.0},
                    1,
                    {-1.0, 0.0, 0.0, 1.0}
                );
            },
            "zero-length parameter domain"
        );
    }

    void testNonFiniteGeometryDefinition()
    {
        const double infinity = std::numeric_limits<double>::infinity();

        requireThrows<std::invalid_argument>(
            [infinity]
            {
                NurbsCurve(
                    {Point{0.0, 0.0, 0.0}, Point{infinity, 0.0, 0.0}},
                    {1.0, 1.0},
                    1,
                    {0.0, 0.0, 1.0, 1.0}
                );
            },
            "non-finite control point"
        );
        requireThrows<std::invalid_argument>(
            [infinity]
            {
                NurbsCurve(
                    {Point{0.0, 0.0, 0.0}, Point{1.0, 0.0, 0.0}},
                    {1.0, 1.0},
                    1,
                    {0.0, 0.0, infinity, infinity}
                );
            },
            "non-finite knot"
        );
    }

    void testParameterValidation()
    {
        const NurbsCurve curve = makeWeightedLinearCurve();

        requireThrows<std::out_of_range>(
            [&curve] { static_cast<void>(curve.evaluate(-0.001)); },
            "parameter below domain"
        );
        requireThrows<std::out_of_range>(
            [&curve] { static_cast<void>(curve.evaluate(1.001)); },
            "parameter above domain"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(curve.evaluate(
                    std::numeric_limits<double>::quiet_NaN()
                ));
            },
            "NaN parameter"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(curve.evaluate(
                    std::numeric_limits<double>::infinity()
                ));
            },
            "infinite parameter"
        );
    }

    void testFirstDerivativeParameterValidation()
    {
        const NurbsCurve curve = makeWeightedLinearCurve();

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
                static_cast<void>(curve.evaluateFirstDerivative(1.001));
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
            "NaN derivative parameter"
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

    void testSecondDerivativeParameterValidation()
    {
        const NurbsCurve curve = makeWeightedLinearCurve();

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
                static_cast<void>(curve.evaluateSecondDerivative(1.001));
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
            "NaN second-derivative parameter"
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

    void testWeightedLinearReferenceValues()
    {
        const NurbsCurve curve = makeWeightedLinearCurve();

        requirePointNear(
            curve.evaluate(0.0),
            Point{0.0, 0.0, 0.0},
            "weighted linear domain beginning"
        );
        requirePointNear(
            curve.evaluate(1.0),
            Point{2.0, 4.0, 6.0},
            "weighted linear domain end"
        );

        // At u = 0.5, the rational coefficients are proportional to
        // 0.5 * 1 and 0.5 * 3, so the second point contributes 3/4.
        requirePointNear(
            curve.evaluate(0.5),
            Point{1.5, 3.0, 4.5},
            "weighted linear interior"
        );
    }

    void testWeightedLinearFirstDerivativeReferenceValues()
    {
        const NurbsCurve curve = makeWeightedLinearCurve();

        // With P0 = 0, P1 = (2, 4, 6), w0 = 1, and w1 = 3,
        // C(u) = 3u P1 / (1 + 2u), so
        // C'(u) = 3 P1 / (1 + 2u)^2.
        requirePointNear(
            curve.evaluateFirstDerivative(0.0),
            Point{6.0, 12.0, 18.0},
            "weighted linear derivative at domain beginning"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(0.5),
            Point{1.5, 3.0, 4.5},
            "weighted linear derivative in the interior"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(1.0),
            Point{2.0 / 3.0, 4.0 / 3.0, 2.0},
            "weighted linear derivative at domain end"
        );
    }

    void testWeightedLinearSecondDerivativeReferenceValues()
    {
        const NurbsCurve curve = makeWeightedLinearCurve();

        // With P0 = 0, P1 = (2, 4, 6), w0 = 1, and w1 = 3,
        // C(u) = 3u P1 / (1 + 2u), so
        // C''(u) = -12 P1 / (1 + 2u)^3. This is nonzero despite the
        // curve's degree-one straight-line locus because u is a nonlinear
        // rational parameterization.
        requirePointNear(
            curve.evaluateSecondDerivative(0.0),
            Point{-24.0, -48.0, -72.0},
            "weighted linear second derivative at domain beginning"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(0.5),
            Point{-3.0, -6.0, -9.0},
            "weighted linear second derivative in the interior"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(1.0),
            Point{-8.0 / 9.0, -16.0 / 9.0, -8.0 / 3.0},
            "weighted linear second derivative at domain end"
        );
    }

    void testDegreeZeroBehavior()
    {
        const NurbsCurve curve(
            {
                Point{1.0, 2.0, 3.0},
                Point{4.0, 5.0, 6.0}
            },
            {2.0, 7.0},
            0,
            {0.0, 1.0, 2.0}
        );

        requirePointNear(
            curve.evaluate(0.0),
            Point{1.0, 2.0, 3.0},
            "degree-zero domain beginning"
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
        const NurbsCurve curve(
            {
                Point{1.0, 2.0, 3.0},
                Point{4.0, 5.0, 6.0}
            },
            {2.0, 7.0},
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
        const NurbsCurve curve(
            {
                Point{1.0, 2.0, 3.0},
                Point{4.0, 5.0, 6.0}
            },
            {2.0, 7.0},
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

    void testEqualWeightBSplineEquivalence()
    {
        const std::vector<Point> controlPoints{
            Point{0.0, 0.0, 0.0},
            Point{2.0, 4.0, 0.0},
            Point{4.0, 0.0, 2.0},
            Point{6.0, 2.0, 4.0}
        };
        const std::vector<double> knots{
            0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0
        };
        const BSplineCurve bSpline(controlPoints, 2, knots);
        const NurbsCurve nurbs(
            controlPoints,
            {2.5, 2.5, 2.5, 2.5},
            2,
            knots
        );

        for (const double parameter : {0.0, 0.125, 0.5, 1.0, 1.375, 2.0})
        {
            requirePointNear(
                nurbs.evaluate(parameter),
                bSpline.evaluate(parameter),
                "equal-weight equivalence"
            );
            requirePointNear(
                nurbs.evaluateFirstDerivative(parameter),
                bSpline.evaluateFirstDerivative(parameter),
                "equal-weight derivative equivalence"
            );
            requirePointNear(
                nurbs.evaluateSecondDerivative(parameter),
                bSpline.evaluateSecondDerivative(parameter),
                "equal-weight second-derivative equivalence"
            );
        }
    }

    void testUnequalWeightQuadraticSecondDerivativeReference()
    {
        const NurbsCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 2.0, 0.0},
                Point{3.0, 0.0, 1.0}
            },
            {1.0, 2.0, 1.0},
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
        );

        // Directly expanding the rational quadratic Bezier polynomials gives
        // A = 4u(1-u)P1 + u^2 P2 and W = 1 + 2u - 2u^2.
        // At u = 1/2: C = (7/6, 4/3, 1/6), C' = (2, 0, 2/3),
        // A'' = (-2, -16, 2), W' = 0, and W'' = -4. Substitution into
        // C'' = (A'' - W''C - 2W'C') / W gives the value below.
        requirePointNear(
            curve.evaluateSecondDerivative(0.5),
            Point{16.0 / 9.0, -64.0 / 9.0, 16.0 / 9.0},
            "unequal-weight quadratic second derivative"
        );
    }

    void testRationalQuadraticQuarterCircle()
    {
        const double middleWeight = std::sqrt(0.5);
        const NurbsCurve curve(
            {
                Point{1.0, 0.0, 0.0},
                Point{1.0, 1.0, 0.0},
                Point{0.0, 1.0, 0.0}
            },
            {1.0, middleWeight, 1.0},
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
        );

        // This is the standard rational quadratic representation of a unit
        // quarter circle. At u = 0.5 it reaches the 45-degree point exactly.
        requirePointNear(
            curve.evaluate(0.5),
            Point{std::sqrt(0.5), std::sqrt(0.5), 0.0},
            "rational quadratic quarter-circle midpoint"
        );

        // Differentiating the rational quadratic Bezier numerator and
        // denominator gives these parameter derivatives. At u = 0.5,
        // A' = (-1, 1), W' = 0, and W = (1 + sqrt(0.5)) / 2, so each
        // nonzero derivative coordinate has magnitude 4 - 2 sqrt(2).
        requirePointNear(
            curve.evaluateFirstDerivative(0.0),
            Point{0.0, std::sqrt(2.0), 0.0},
            "quarter-circle derivative at domain beginning"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(0.5),
            Point{
                -(4.0 - 2.0 * std::sqrt(2.0)),
                4.0 - 2.0 * std::sqrt(2.0),
                0.0
            },
            "quarter-circle derivative at midpoint"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(1.0),
            Point{-std::sqrt(2.0), 0.0, 0.0},
            "quarter-circle derivative at domain end"
        );

        // Expanding the same rational polynomials at u = 1/2 gives
        // A'' = (2 - 2sqrt(2), 2 - 2sqrt(2)), W' = 0,
        // W'' = 4 - 2sqrt(2), and W = (2 + sqrt(2)) / 4.
        // Applying the twice-differentiated quotient relation yields equal
        // coordinates 32 - 24sqrt(2); this parameter derivative is not -C.
        requirePointNear(
            curve.evaluateSecondDerivative(0.5),
            Point{
                32.0 - 24.0 * std::sqrt(2.0),
                32.0 - 24.0 * std::sqrt(2.0),
                0.0
            },
            "quarter-circle second derivative at midpoint"
        );
    }

    void testRepeatedKnotFirstDerivativeBehavior()
    {
        const NurbsCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 2.0, 0.0},
                Point{2.0, 0.0, 1.0},
                Point{4.0, 3.0, 2.0},
                Point{5.0, 3.0, 4.0}
            },
            {1.0, 2.0, 3.0, 4.0, 5.0},
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0}
        );

        // The double interior knot makes the curve C0 there. Exact-knot
        // evaluation selects the right-hand span. Its homogeneous point is
        // Q2 = (6, 0, 3, 3), and its homogeneous derivative is
        // 2(Q3 - Q2) = (20, 24, 10, 2), giving the rational derivative below.
        requirePointNear(
            curve.evaluateFirstDerivative(1.0),
            Point{16.0 / 3.0, 8.0, 8.0 / 3.0},
            "rational derivative at repeated interior knot"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(1.5),
            Point{2.78125, 2.53125, 3.03125},
            "rational derivative after repeated interior knot"
        );
    }

    void testRepeatedKnotSecondDerivativeBehavior()
    {
        const NurbsCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 2.0, 0.0},
                Point{2.0, 0.0, 1.0},
                Point{4.0, 3.0, 2.0},
                Point{5.0, 3.0, 4.0}
            },
            {1.0, 2.0, 3.0, 4.0, 5.0},
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0}
        );

        // On the selected right-hand span, homogeneous Bezier control points
        // Q2, Q3, Q4 give Q'' = 2(Q4 - 2Q3 + Q2)
        // = (-2, -18, 14, 0). Combining that with the independently checked
        // rational point and first derivative gives these values.
        requirePointNear(
            curve.evaluateSecondDerivative(1.0),
            Point{-70.0 / 9.0, -50.0 / 3.0, 10.0 / 9.0},
            "rational second derivative at repeated interior knot"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(1.5),
            Point{-3.28125, -7.03125, 0.46875},
            "rational second derivative after repeated interior knot"
        );
    }

    void testUnclampedBoundaryAndInteriorKnotBehavior()
    {
        const NurbsCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{2.0, 4.0, 0.0},
                Point{4.0, 0.0, 2.0},
                Point{6.0, 2.0, 4.0}
            },
            {1.0, 1.0, 1.0, 1.0},
            2,
            {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}
        );

        requirePointNear(
            curve.evaluate(2.0),
            Point{1.0, 2.0, 0.0},
            "unclamped domain beginning"
        );
        requirePointNear(
            curve.evaluate(3.0),
            Point{3.0, 2.0, 1.0},
            "unclamped interior knot"
        );
        requirePointNear(
            curve.evaluate(4.0),
            Point{5.0, 1.0, 3.0},
            "unclamped domain end"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(2.0),
            Point{2.0, 4.0, 0.0},
            "unclamped derivative at domain beginning"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(3.0),
            Point{2.0, -4.0, 2.0},
            "unclamped derivative at interior knot"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(4.0),
            Point{2.0, 2.0, 2.0},
            "unclamped derivative at domain end"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(2.0),
            Point{0.0, -8.0, 2.0},
            "unclamped second derivative at domain beginning"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(3.0),
            Point{0.0, 6.0, 0.0},
            "unclamped second derivative at interior knot"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(4.0),
            Point{0.0, 6.0, 0.0},
            "unclamped second derivative at domain end"
        );
    }

    void testSmallEqualWeightsDoNotCreateZeroDenominator()
    {
        const double smallestWeight =
            std::numeric_limits<double>::denorm_min();
        const NurbsCurve curve(
            {Point{0.0, 0.0, 0.0}, Point{2.0, 4.0, 6.0}},
            {smallestWeight, smallestWeight},
            1,
            {0.0, 0.0, 1.0, 1.0}
        );

        // A common nonzero weight scale cancels from a rational curve. The
        // evaluator normalizes active weights before homogeneous de Boor so
        // this valid definition does not underflow its denominator to zero.
        requirePointNear(
            curve.evaluate(0.5),
            Point{1.0, 2.0, 3.0},
            "small equal weights"
        );
        requirePointNear(
            curve.evaluateFirstDerivative(0.5),
            Point{2.0, 4.0, 6.0},
            "small equal-weight derivative"
        );
        requirePointNear(
            curve.evaluateSecondDerivative(0.5),
            Point{0.0, 0.0, 0.0},
            "small equal-weight second derivative"
        );
    }

    void testInvalidDerivativeIntermediateIsRejected()
    {
        const double smallestWeight =
            std::numeric_limits<double>::denorm_min();
        const double largestFinite =
            std::numeric_limits<double>::max();
        const NurbsCurve underflowedDenominator(
            {Point{0.0, 0.0, 0.0}, Point{1.0, 0.0, 0.0}},
            {smallestWeight, largestFinite},
            1,
            {0.0, 0.0, 1.0, 1.0}
        );

        // Normalizing by the largest active weight underflows the first
        // normalized weight at u = 0. The evaluator must reject the resulting
        // zero floating-point denominator rather than divide by it.
        requireThrows<std::domain_error>(
            [&underflowedDenominator]
            {
                static_cast<void>(
                    underflowedDenominator.evaluateFirstDerivative(0.0)
                );
            },
            "underflowed derivative rational denominator"
        );
        requireThrows<std::domain_error>(
            [&underflowedDenominator]
            {
                static_cast<void>(
                    underflowedDenominator.evaluateSecondDerivative(0.0)
                );
            },
            "underflowed second-derivative rational denominator"
        );

        const NurbsCurve unrepresentableDerivative(
            {
                Point{largestFinite, 0.0, 0.0},
                Point{-largestFinite, 0.0, 0.0}
            },
            {1.0, 1.0},
            1,
            {0.0, 0.0, 1.0, 1.0}
        );

        requireThrows<std::domain_error>(
            [&unrepresentableDerivative]
            {
                static_cast<void>(
                    unrepresentableDerivative.evaluateFirstDerivative(0.5)
                );
            },
            "non-finite homogeneous derivative"
        );

        const NurbsCurve unrepresentableSecondDerivative(
            {
                Point{0.0, 0.0, 0.0},
                Point{largestFinite / 2.0, 0.0, 0.0},
                Point{0.0, 0.0, 0.0}
            },
            {1.0, 1.0, 1.0},
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
        );

        requireThrows<std::domain_error>(
            [&unrepresentableSecondDerivative]
            {
                static_cast<void>(
                    unrepresentableSecondDerivative
                        .evaluateSecondDerivative(0.5)
                );
            },
            "non-finite homogeneous second derivative"
        );
    }

    void testDeterministicRepeatedEvaluation()
    {
        const NurbsCurve curve = makeWeightedLinearCurve();
        const Point expected = curve.evaluate(0.375);
        const Point expectedDerivative =
            curve.evaluateFirstDerivative(0.375);
        const Point expectedSecondDerivative =
            curve.evaluateSecondDerivative(0.375);

        for (int repetition = 0; repetition < 100; ++repetition)
        {
            const Point actual = curve.evaluate(0.375);
            const Point actualDerivative =
                curve.evaluateFirstDerivative(0.375);
            const Point actualSecondDerivative =
                curve.evaluateSecondDerivative(0.375);
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
        {"weight validation", testWeightValidation},
        {"invalid degree and knot definitions", testInvalidDegreeAndKnotDefinitions},
        {"non-finite geometry definition", testNonFiniteGeometryDefinition},
        {"parameter validation", testParameterValidation},
        {
            "first-derivative parameter validation",
            testFirstDerivativeParameterValidation
        },
        {
            "second-derivative parameter validation",
            testSecondDerivativeParameterValidation
        },
        {"weighted linear reference values", testWeightedLinearReferenceValues},
        {
            "weighted linear first-derivative reference values",
            testWeightedLinearFirstDerivativeReferenceValues
        },
        {
            "weighted linear second-derivative reference values",
            testWeightedLinearSecondDerivativeReferenceValues
        },
        {"degree-zero behavior", testDegreeZeroBehavior},
        {"degree-zero first derivative", testDegreeZeroFirstDerivative},
        {"degree-zero second derivative", testDegreeZeroSecondDerivative},
        {"equal-weight B-spline equivalence", testEqualWeightBSplineEquivalence},
        {
            "unequal-weight quadratic second derivative",
            testUnequalWeightQuadraticSecondDerivativeReference
        },
        {"rational quadratic quarter circle", testRationalQuadraticQuarterCircle},
        {
            "repeated-knot first-derivative behavior",
            testRepeatedKnotFirstDerivativeBehavior
        },
        {
            "repeated-knot second-derivative behavior",
            testRepeatedKnotSecondDerivativeBehavior
        },
        {
            "unclamped boundary and interior-knot behavior",
            testUnclampedBoundaryAndInteriorKnotBehavior
        },
        {
            "small equal-weight denominator safety",
            testSmallEqualWeightsDoNotCreateZeroDenominator
        },
        {
            "invalid derivative intermediate rejection",
            testInvalidDerivativeIntermediateIsRejected
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
