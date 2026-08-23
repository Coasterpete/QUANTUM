#include <quantum/geometry/BSplineCurve.hpp>
#include <quantum/geometry/CurveGeometry.hpp>
#include <quantum/geometry/CurveSampling.hpp>
#include <quantum/geometry/NurbsCurve.hpp>

#include <algorithm>
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
    using quantum::geometry::evaluateCurvature;
    using quantum::geometry::evaluateRadiusOfCurvature;
    using quantum::geometry::evaluateUnitTangent;
    using quantum::geometry::NurbsCurve;
    using quantum::geometry::sampleCurveByArcLength;
    using Point = glm::dvec3;

    // These references use small, mathematically controlled curves. The
    // existing 1e-12 absolute tolerance remains sufficient for their analytic
    // double-precision derivatives and the curvature formula.
    constexpr double tolerance = 1.0e-12;
    double maximumUnitLengthError = 0.0;

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
        const std::string prefix(context);
        requireNear(actual.x, expected.x, prefix + " x");
        requireNear(actual.y, expected.y, prefix + " y");
        requireNear(actual.z, expected.z, prefix + " z");
    }

    template<typename Curve>
    [[nodiscard]] Point evaluateCheckedUnitTangent(
        const Curve& curve,
        const double parameter,
        const std::string_view context
    )
    {
        const Point tangent = evaluateUnitTangent(curve, parameter);
        require(
            std::isfinite(tangent.x)
                && std::isfinite(tangent.y)
                && std::isfinite(tangent.z),
            std::string(context) + " tangent components must be finite"
        );

        const double tangentMagnitude = std::hypot(
            tangent.x,
            tangent.y,
            tangent.z
        );
        maximumUnitLengthError = std::max(
            maximumUnitLengthError,
            std::abs(tangentMagnitude - 1.0)
        );
        requireNear(
            tangentMagnitude,
            1.0,
            std::string(context) + " tangent magnitude"
        );

        const Point derivative = curve.evaluateFirstDerivative(parameter);
        const double directionAgreement =
            tangent.x * derivative.x
            + tangent.y * derivative.y
            + tangent.z * derivative.z;
        require(
            std::isfinite(directionAgreement) && directionAgreement > 0.0,
            std::string(context)
                + " tangent must agree with the first-derivative direction"
        );

        return tangent;
    }

    void requirePositiveInfinity(
        const double actual,
        const std::string_view context
    )
    {
        if (!std::isinf(actual) || actual < 0.0)
        {
            std::ostringstream message;
            message << context << ": expected positive infinity, received "
                    << actual;
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

    [[nodiscard]] BSplineCurve makeLinearBSpline()
    {
        return BSplineCurve(
            {Point{0.0, 0.0, 0.0}, Point{2.0, 4.0, 6.0}},
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

    [[nodiscard]] BSplineCurve makeRepeatedKnotBSpline()
    {
        return BSplineCurve(
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
    }

    [[nodiscard]] BSplineCurve makeStationaryPointBSpline()
    {
        return BSplineCurve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 0.0, 0.0},
                Point{0.0, 0.0, 0.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
        );
    }

    void testLinearBSplineUnitTangent()
    {
        const BSplineCurve curve = makeLinearBSpline();
        const double inverseDerivativeMagnitude = 1.0 / std::sqrt(56.0);
        const Point expected =
            inverseDerivativeMagnitude * Point{2.0, 4.0, 6.0};

        for (const double parameter : {0.0, 0.5, 1.0})
        {
            const std::string context =
                "linear B-spline at u = " + std::to_string(parameter);
            requirePointNear(
                evaluateCheckedUnitTangent(curve, parameter, context),
                expected,
                context
            );
        }
    }

    void testUnequalWeightRationalLineUnitTangent()
    {
        const NurbsCurve curve = makeUnequalWeightLinearNurbs();
        const double inverseDirectionMagnitude = 1.0 / std::sqrt(56.0);
        const Point expected =
            inverseDirectionMagnitude * Point{2.0, 4.0, 6.0};
        double beginningSpeed = 0.0;
        double endingSpeed = 0.0;

        for (const double parameter : {0.0, 0.25, 0.5, 0.75, 1.0})
        {
            const std::string context =
                "unequal-weight rational line at u = "
                + std::to_string(parameter);
            requirePointNear(
                evaluateCheckedUnitTangent(curve, parameter, context),
                expected,
                context
            );

            const Point derivative = curve.evaluateFirstDerivative(parameter);
            const double speed = std::hypot(
                derivative.x,
                derivative.y,
                derivative.z
            );

            if (parameter == 0.0)
            {
                beginningSpeed = speed;
            }
            else if (parameter == 1.0)
            {
                endingSpeed = speed;
            }
        }

        require(
            beginningSpeed > endingSpeed,
            "unequal-weight rational line parameter speed must vary"
        );
    }

    void testQuarterCircleUnitTangentsAndScaleInvariance()
    {
        const NurbsCurve unitCircle = makeQuarterCircle(1.0);
        const NurbsCurve scaledCircle = makeQuarterCircle(10.0);
        const double inverseSqrtTwo = std::sqrt(0.5);
        const std::vector<std::pair<double, Point>> references{
            {0.0, Point{0.0, 1.0, 0.0}},
            {
                0.5,
                Point{-inverseSqrtTwo, inverseSqrtTwo, 0.0}
            },
            {1.0, Point{-1.0, 0.0, 0.0}}
        };

        for (const auto& [parameter, expected] : references)
        {
            const std::string context =
                "quarter circle at u = " + std::to_string(parameter);
            const Point unitTangent = evaluateCheckedUnitTangent(
                unitCircle,
                parameter,
                context
            );
            const Point scaledTangent = evaluateCheckedUnitTangent(
                scaledCircle,
                parameter,
                "radius-10 " + context
            );
            requirePointNear(unitTangent, expected, context);
            requirePointNear(
                scaledTangent,
                expected,
                "radius-10 " + context
            );
            requirePointNear(
                scaledTangent,
                unitTangent,
                "quarter-circle tangent scale invariance"
            );
        }
    }

    void testNonCircularBSplineUnitTangent()
    {
        const BSplineCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 0.0, 0.0},
                Point{1.0, 1.0, 0.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
        );

        // Independent differentiation of this quadratic Bezier gives
        // C'(1/2) = (1, 1, 0), whose unit direction is the reference below.
        requirePointNear(
            evaluateCheckedUnitTangent(
                curve,
                0.5,
                "non-circular quadratic B-spline"
            ),
            Point{std::sqrt(0.5), std::sqrt(0.5), 0.0},
            "non-circular quadratic B-spline tangent"
        );
    }

    void testRepeatedKnotUnitTangent()
    {
        const BSplineCurve curve = makeRepeatedKnotBSpline();

        // The controlled parameter u = 1/2 lies in the left span and has
        // C' = (2, 0, 1). The exact double knot u = 1 selects the right span,
        // whose one-sided derivative is C' = (4, 6, 2).
        requirePointNear(
            evaluateCheckedUnitTangent(
                curve,
                0.5,
                "repeated-knot left span"
            ),
            Point{2.0 / std::sqrt(5.0), 0.0, 1.0 / std::sqrt(5.0)},
            "repeated-knot left-span tangent"
        );
        requirePointNear(
            evaluateCheckedUnitTangent(
                curve,
                1.0,
                "exact repeated interior knot"
            ),
            Point{
                4.0 / std::sqrt(56.0),
                6.0 / std::sqrt(56.0),
                2.0 / std::sqrt(56.0)
            },
            "right-selected repeated-knot tangent"
        );
    }

    void testUndefinedUnitTangents()
    {
        const BSplineCurve degreeZeroCurve(
            {Point{1.0, 2.0, 3.0}},
            0,
            {0.0, 1.0}
        );
        requireThrows<std::domain_error>(
            [&degreeZeroCurve]
            {
                static_cast<void>(
                    evaluateUnitTangent(degreeZeroCurve, 0.5)
                );
            },
            "degree-zero unit tangent"
        );

        const BSplineCurve stationaryCurve = makeStationaryPointBSpline();
        requireThrows<std::domain_error>(
            [&stationaryCurve]
            {
                static_cast<void>(
                    evaluateUnitTangent(stationaryCurve, 0.5)
                );
            },
            "stationary-point unit tangent"
        );
        requirePointNear(
            evaluateCheckedUnitTangent(
                stationaryCurve,
                0.25,
                "before stationary point"
            ),
            Point{1.0, 0.0, 0.0},
            "unit tangent before stationary point"
        );
        requirePointNear(
            evaluateCheckedUnitTangent(
                stationaryCurve,
                0.75,
                "after stationary point"
            ),
            Point{-1.0, 0.0, 0.0},
            "unit tangent after stationary point"
        );
    }

    void testUnitTangentParameterValidation()
    {
        const NurbsCurve curve = makeQuarterCircle(1.0);

        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(evaluateUnitTangent(
                    curve,
                    std::numeric_limits<double>::quiet_NaN()
                ));
            },
            "NaN unit-tangent parameter"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(evaluateUnitTangent(
                    curve,
                    std::numeric_limits<double>::infinity()
                ));
            },
            "infinite unit-tangent parameter"
        );
        requireThrows<std::out_of_range>(
            [&curve]
            {
                static_cast<void>(evaluateUnitTangent(curve, -0.001));
            },
            "below-domain unit-tangent parameter"
        );
        requireThrows<std::out_of_range>(
            [&curve]
            {
                static_cast<void>(evaluateUnitTangent(curve, 1.001));
            },
            "above-domain unit-tangent parameter"
        );
    }

    void testUnitTangentNumericalPolicy()
    {
        struct FixedDerivativeCurve
        {
            Point derivative;

            [[nodiscard]] Point evaluateFirstDerivative(double) const
            {
                return derivative;
            }
        };

        const FixedDerivativeCurve smallDerivative{
            Point{std::numeric_limits<double>::denorm_min(), 0.0, 0.0}
        };
        requirePointNear(
            evaluateCheckedUnitTangent(
                smallDerivative,
                0.0,
                "small representable derivative"
            ),
            Point{1.0, 0.0, 0.0},
            "small representable derivative tangent"
        );

        const FixedDerivativeCurve nonFiniteDerivative{
            Point{std::numeric_limits<double>::infinity(), 0.0, 0.0}
        };
        requireThrows<std::domain_error>(
            [&nonFiniteDerivative]
            {
                static_cast<void>(
                    evaluateUnitTangent(nonFiniteDerivative, 0.0)
                );
            },
            "non-finite derivative unit tangent"
        );
    }

    void testCanonicalSampleUnitTangentCompatibility()
    {
        const NurbsCurve curve = makeQuarterCircle(1.0);
        const auto samples = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            0.25
        );

        require(samples.size() > 2, "canonical sample reference count");

        for (const auto& sample : samples)
        {
            static_cast<void>(evaluateCheckedUnitTangent(
                curve,
                sample.parameter,
                "canonical sample parameter"
            ));
        }
    }

    void testDeterministicRepeatedUnitTangentEvaluation()
    {
        const NurbsCurve curve = makeQuarterCircle(1.0);
        const Point expected = evaluateCheckedUnitTangent(
            curve,
            0.375,
            "deterministic tangent reference"
        );

        for (int repetition = 0; repetition < 100; ++repetition)
        {
            const Point actual = evaluateUnitTangent(curve, 0.375);
            require(
                actual.x == expected.x
                    && actual.y == expected.y
                    && actual.z == expected.z,
                "repeated unit-tangent evaluation changed its result"
            );
        }
    }

    void testLinearBSpline()
    {
        const BSplineCurve curve = makeLinearBSpline();

        for (const double parameter : {0.0, 0.5, 1.0})
        {
            requireNear(
                evaluateCurvature(curve, parameter),
                0.0,
                "linear B-spline curvature"
            );
            requirePositiveInfinity(
                evaluateRadiusOfCurvature(curve, parameter),
                "linear B-spline radius"
            );
        }
    }

    void testUnequalWeightRationalStraightLine()
    {
        const NurbsCurve curve = makeUnequalWeightLinearNurbs();

        for (const double parameter : {0.0, 0.5, 1.0})
        {
            const std::string context =
                "weighted straight line at u = "
                + std::to_string(parameter);
            const Point secondDerivative =
                curve.evaluateSecondDerivative(parameter);
            require(
                std::hypot(
                    secondDerivative.x,
                    secondDerivative.y,
                    secondDerivative.z
                ) > 0.0,
                "weighted straight-line second derivative should be nonzero"
            );
            requireNear(
                evaluateCurvature(curve, parameter),
                0.0,
                context + " curvature"
            );
            requirePositiveInfinity(
                evaluateRadiusOfCurvature(curve, parameter),
                context + " radius"
            );
        }
    }

    void testUnitQuarterCircle()
    {
        const NurbsCurve curve = makeQuarterCircle(1.0);

        for (const double parameter : {0.0, 0.25, 0.5, 0.75, 1.0})
        {
            requireNear(
                evaluateCurvature(curve, parameter),
                1.0,
                "unit quarter-circle curvature"
            );
            requireNear(
                evaluateRadiusOfCurvature(curve, parameter),
                1.0,
                "unit quarter-circle radius"
            );
        }
    }

    void testScaledQuarterCircle()
    {
        const NurbsCurve curve = makeQuarterCircle(10.0);

        for (const double parameter : {0.0, 0.5, 1.0})
        {
            requireNear(
                evaluateCurvature(curve, parameter),
                0.1,
                "radius-10 quarter-circle curvature"
            );
            requireNear(
                evaluateRadiusOfCurvature(curve, parameter),
                10.0,
                "radius-10 quarter-circle radius"
            );
        }
    }

    void testNonCircularBSplineReference()
    {
        const BSplineCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{1.0, 0.0, 0.0},
                Point{1.0, 1.0, 0.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
        );

        // At u = 1/2, C' = (1, 1, 0) and C'' = (-2, 2, 0).
        // Therefore |C' x C''| = 4 and |C'|^3 = 2 sqrt(2).
        requireNear(
            evaluateCurvature(curve, 0.5),
            std::sqrt(2.0),
            "non-circular quadratic B-spline curvature"
        );
        requireNear(
            evaluateRadiusOfCurvature(curve, 0.5),
            std::sqrt(0.5),
            "non-circular quadratic B-spline radius"
        );
    }

    void testNonCircularNurbsReference()
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

        // Independently derived values at u = 1/2 are
        // C' = (2, 0, 2/3) and C'' = (16/9, -64/9, 16/9).
        // Their cross-product magnitude is 64 sqrt(41) / 27, while
        // |C'|^3 = 80 sqrt(10) / 27.
        const double expectedCurvature =
            (4.0 / 5.0) * std::sqrt(41.0 / 10.0);
        requireNear(
            evaluateCurvature(curve, 0.5),
            expectedCurvature,
            "non-circular quadratic NURBS curvature"
        );
        requireNear(
            evaluateRadiusOfCurvature(curve, 0.5),
            1.0 / expectedCurvature,
            "non-circular quadratic NURBS radius"
        );
    }

    void testExactInteriorKnotUsesSelectedDerivatives()
    {
        const BSplineCurve curve(
            {
                Point{0.0, 0.0, 0.0},
                Point{2.0, 4.0, 0.0},
                Point{4.0, 0.0, 2.0},
                Point{6.0, 2.0, 4.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0}
        );

        // At the exact interior knot u = 1, the existing convention selects
        // the right-hand span: C' = (2, -4, 2), C'' = (2, 8, 2).
        // This gives kappa = 1 / (2 sqrt(3)).
        const double expectedCurvature = 1.0 / (2.0 * std::sqrt(3.0));
        requireNear(
            evaluateCurvature(curve, 1.0),
            expectedCurvature,
            "curvature at exact interior knot"
        );
        requireNear(
            evaluateRadiusOfCurvature(curve, 1.0),
            1.0 / expectedCurvature,
            "radius at exact interior knot"
        );
    }

    void testDegenerateFirstDerivative()
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

        // This quadratic has C'(1/2) = 0 and C''(1/2) = (-4, 0, 0).
        // Curvature is undefined there, not zero.
        requireThrows<std::domain_error>(
            [&curve]
            {
                static_cast<void>(evaluateCurvature(curve, 0.5));
            },
            "degenerate curvature"
        );
        requireThrows<std::domain_error>(
            [&curve]
            {
                static_cast<void>(evaluateRadiusOfCurvature(curve, 0.5));
            },
            "degenerate radius"
        );
    }

    void testParameterValidation()
    {
        const NurbsCurve curve = makeQuarterCircle(1.0);

        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(evaluateCurvature(
                    curve,
                    std::numeric_limits<double>::quiet_NaN()
                ));
            },
            "NaN curvature parameter"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(evaluateRadiusOfCurvature(
                    curve,
                    std::numeric_limits<double>::infinity()
                ));
            },
            "infinite radius parameter"
        );
        requireThrows<std::out_of_range>(
            [&curve]
            {
                static_cast<void>(evaluateCurvature(curve, -0.001));
            },
            "below-domain curvature parameter"
        );
        requireThrows<std::out_of_range>(
            [&curve]
            {
                static_cast<void>(evaluateRadiusOfCurvature(curve, 1.001));
            },
            "above-domain radius parameter"
        );
    }

    void testDeterministicRepeatedEvaluation()
    {
        const NurbsCurve curve = makeQuarterCircle(1.0);
        const double expectedCurvature = evaluateCurvature(curve, 0.375);
        const double expectedRadius =
            evaluateRadiusOfCurvature(curve, 0.375);

        for (int repetition = 0; repetition < 100; ++repetition)
        {
            require(
                evaluateCurvature(curve, 0.375) == expectedCurvature,
                "repeated curvature evaluation changed its result"
            );
            require(
                evaluateRadiusOfCurvature(curve, 0.375) == expectedRadius,
                "repeated radius evaluation changed its result"
            );
        }
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;

    const std::vector<Test> tests{
        {"linear B-spline unit tangent", testLinearBSplineUnitTangent},
        {
            "unequal-weight rational-line unit tangent",
            testUnequalWeightRationalLineUnitTangent
        },
        {
            "quarter-circle unit tangents and scale invariance",
            testQuarterCircleUnitTangentsAndScaleInvariance
        },
        {
            "non-circular B-spline unit tangent",
            testNonCircularBSplineUnitTangent
        },
        {"repeated-knot unit tangent", testRepeatedKnotUnitTangent},
        {"undefined unit tangents", testUndefinedUnitTangents},
        {
            "unit-tangent parameter validation",
            testUnitTangentParameterValidation
        },
        {"unit-tangent numerical policy", testUnitTangentNumericalPolicy},
        {
            "canonical-sample unit-tangent compatibility",
            testCanonicalSampleUnitTangentCompatibility
        },
        {
            "deterministic repeated unit-tangent evaluation",
            testDeterministicRepeatedUnitTangentEvaluation
        },
        {"linear B-spline", testLinearBSpline},
        {
            "unequal-weight rational straight line",
            testUnequalWeightRationalStraightLine
        },
        {"unit quarter circle", testUnitQuarterCircle},
        {"scaled quarter circle", testScaledQuarterCircle},
        {"non-circular B-spline reference", testNonCircularBSplineReference},
        {"non-circular NURBS reference", testNonCircularNurbsReference},
        {
            "exact interior-knot derivative convention",
            testExactInteriorKnotUsesSelectedDerivatives
        },
        {"degenerate first derivative", testDegenerateFirstDerivative},
        {"parameter validation", testParameterValidation},
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
    std::cout << "Maximum unit-tangent length error: "
              << maximumUnitLengthError << '\n';
    return 0;
}
