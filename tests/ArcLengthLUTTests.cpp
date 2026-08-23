#include <quantum/geometry/ArcLengthLUT.hpp>
#include <quantum/geometry/BSplineCurve.hpp>
#include <quantum/geometry/CurveGeometry.hpp>
#include <quantum/geometry/NurbsCurve.hpp>

#include <algorithm>
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
    using quantum::geometry::ArcLengthLUT;
    using quantum::geometry::ArcLengthLUTOptions;
    using quantum::geometry::BSplineCurve;
    using quantum::geometry::evaluateArcLength;
    using quantum::geometry::evaluateParameterAtArcLength;
    using quantum::geometry::NurbsCurve;
    using Point = glm::dvec3;

    constexpr double referenceTolerance = 1.0e-8;

    struct Measurements
    {
        double worstDistanceError = 0.0;
        std::size_t straightSampleCount = 0;
        std::size_t rationalLineSampleCount = 0;
        std::size_t unitCircleSampleCount = 0;
        std::size_t scaledCircleSampleCount = 0;
        std::size_t repeatedKnotSampleCount = 0;
        std::size_t stationarySampleCount = 0;
        double straightLength = 0.0;
        double rationalLineLength = 0.0;
        double rationalLineHalfParameter = 0.0;
        double unitCircleLength = 0.0;
        double scaledCircleLength = 0.0;
        double repeatedKnotLength = 0.0;
        double repeatedKnotDistance = 0.0;
    } measurements;

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
        const double tolerance,
        const std::string_view context
    )
    {
        if (!std::isfinite(actual)
            || std::abs(actual - expected) > tolerance)
        {
            std::ostringstream message;
            message.precision(17);
            message << context << ": expected " << expected
                    << " within " << tolerance
                    << ", received " << actual;
            throw TestFailure(message.str());
        }
    }

    void requirePointNear(
        const Point& actual,
        const Point& expected,
        const double tolerance,
        const std::string_view context
    )
    {
        const Point difference = actual - expected;
        const double error = std::hypot(
            difference.x,
            difference.y,
            difference.z
        );

        requireNear(error, 0.0, tolerance, context);
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
                Point{2.0, 0.0, 0.0},
                Point{3.0, 3.0, 0.0},
                Point{5.0, 1.0, 0.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0}
        );
    }

    [[nodiscard]] BSplineCurve makeZeroLengthBSpline()
    {
        return BSplineCurve(
            {
                Point{3.0, -2.0, 5.0},
                Point{3.0, -2.0, 5.0},
                Point{3.0, -2.0, 5.0}
            },
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
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

    [[nodiscard]] double lutTolerance(
        const ArcLengthLUT& lut,
        const ArcLengthLUTOptions& options
    )
    {
        return options.absoluteTolerance
            + options.relativeTolerance * lut.totalLength();
    }

    void requireOrderedSamples(
        const ArcLengthLUT& lut,
        const std::string_view context
    )
    {
        require(!lut.samples().empty(), "LUT must contain at least one sample");
        require(
            lut.samples().front().distance == 0.0,
            "first LUT distance must be exactly zero"
        );
        require(
            lut.samples().back().distance == lut.totalLength(),
            "last LUT distance must equal total length exactly"
        );

        for (std::size_t index = 1; index < lut.samples().size(); ++index)
        {
            if (!(lut.samples()[index - 1].parameter
                    < lut.samples()[index].parameter))
            {
                throw TestFailure(
                    std::string(context)
                    + ": LUT parameters are not strictly increasing"
                );
            }

            if (lut.samples()[index - 1].distance
                > lut.samples()[index].distance)
            {
                throw TestFailure(
                    std::string(context)
                    + ": LUT distances are decreasing"
                );
            }
        }
    }

    template<typename Curve>
    void requireDirectAgreement(
        const Curve& curve,
        const ArcLengthLUT& lut,
        const ArcLengthLUTOptions& options,
        const std::string_view context
    )
    {
        const double parameterBegin = lut.samples().front().parameter;
        const double parameterEnd = lut.samples().back().parameter;
        const double allowedError = lutTolerance(lut, options);

        for (int index = 0; index <= 64; ++index)
        {
            const double fraction = static_cast<double>(index) / 64.0;
            const double parameter = std::lerp(
                parameterBegin,
                parameterEnd,
                fraction
            );
            const double directDistance = evaluateArcLength(
                curve,
                parameterBegin,
                parameter,
                options.arcLength
            );
            const double lutDistance = lut.distanceAtParameter(parameter);
            const double forwardError =
                std::abs(lutDistance - directDistance);
            measurements.worstDistanceError = std::max(
                measurements.worstDistanceError,
                forwardError
            );

            if (forwardError > allowedError)
            {
                std::ostringstream message;
                message.precision(17);
                message << context << " parameter-to-distance error "
                        << forwardError << " exceeded " << allowedError
                        << " at u = " << parameter;
                throw TestFailure(message.str());
            }

            const double targetDistance = fraction * lut.totalLength();
            const double lutParameter =
                lut.parameterAtDistance(targetDistance);
            const double directRecoveredParameter =
                evaluateParameterAtArcLength(
                    curve,
                    parameterBegin,
                    parameterEnd,
                    targetDistance
                );
            const double lutResidual = std::abs(
                evaluateArcLength(
                    curve,
                    parameterBegin,
                    lutParameter,
                    options.arcLength
                ) - targetDistance
            );
            const double directResidual = std::abs(
                evaluateArcLength(
                    curve,
                    parameterBegin,
                    directRecoveredParameter,
                    options.arcLength
                ) - targetDistance
            );
            measurements.worstDistanceError = std::max(
                measurements.worstDistanceError,
                lutResidual
            );

            if (lutResidual > allowedError)
            {
                std::ostringstream message;
                message.precision(17);
                message << context << " distance-to-parameter residual "
                        << lutResidual << " exceeded " << allowedError
                        << " at s = " << targetDistance;
                throw TestFailure(message.str());
            }

            require(
                directResidual
                    <= options.arcLength.absoluteTolerance
                        + options.arcLength.relativeTolerance
                            * lut.totalLength(),
                "authoritative inversion reference missed its own tolerance"
            );
        }
    }

    template<typename Curve>
    void requireRoundTrips(
        const Curve& curve,
        const ArcLengthLUT& lut,
        const ArcLengthLUTOptions& options,
        const std::string_view context
    )
    {
        const double parameterBegin = lut.samples().front().parameter;
        const double parameterEnd = lut.samples().back().parameter;
        const double allowedError = lutTolerance(lut, options);

        for (int index = 0; index <= 32; ++index)
        {
            const double fraction = static_cast<double>(index) / 32.0;
            const double parameter = std::lerp(
                parameterBegin,
                parameterEnd,
                fraction
            );
            const double distance = lut.distanceAtParameter(parameter);
            const double recoveredParameter =
                lut.parameterAtDistance(distance);
            const double recoveredDirectDistance = evaluateArcLength(
                curve,
                parameterBegin,
                recoveredParameter,
                options.arcLength
            );
            const double originalDirectDistance = evaluateArcLength(
                curve,
                parameterBegin,
                parameter,
                options.arcLength
            );
            requireNear(
                recoveredDirectDistance,
                originalDirectDistance,
                2.0 * allowedError,
                std::string(context) + " parameter round trip"
            );

            const double targetDistance = fraction * lut.totalLength();
            const double roundTripDistance = lut.distanceAtParameter(
                lut.parameterAtDistance(targetDistance)
            );
            requireNear(
                roundTripDistance,
                targetDistance,
                16.0 * std::numeric_limits<double>::epsilon()
                    * std::max(1.0, lut.totalLength()),
                std::string(context) + " distance round trip"
            );
        }
    }

    void testStraightBSpline()
    {
        const BSplineCurve curve = makeLinearBSpline();
        const ArcLengthLUT lut = ArcLengthLUT::build(curve, 0.0, 1.0);
        const double length = std::sqrt(56.0);

        measurements.straightSampleCount = lut.samples().size();
        measurements.straightLength = lut.totalLength();
        requireOrderedSamples(lut, "straight B-spline");
        requireNear(
            lut.totalLength(),
            length,
            referenceTolerance,
            "straight B-spline total length"
        );
        require(lut.samples().size() == 2, "linear mapping should need two samples");
        require(lut.samples().front().parameter == 0.0, "exact LUT beginning");
        require(lut.samples().back().parameter == 1.0, "exact LUT end");

        for (const double fraction : {0.25, 0.5, 0.75})
        {
            requireNear(
                lut.distanceAtParameter(fraction),
                fraction * length,
                referenceTolerance,
                "straight B-spline parameter query"
            );
            requireNear(
                lut.parameterAtDistance(fraction * length),
                fraction,
                referenceTolerance,
                "straight B-spline distance query"
            );
        }

        requireDirectAgreement(curve, lut, {}, "straight B-spline");
        requireRoundTrips(curve, lut, {}, "straight B-spline");
    }

    void testUnequalWeightRationalLine()
    {
        const NurbsCurve curve = makeUnequalWeightLinearNurbs();
        const ArcLengthLUTOptions options{};
        const ArcLengthLUT lut = ArcLengthLUT::build(curve, 0.0, 1.0, options);
        const double halfLength = 0.5 * lut.totalLength();
        const double parameter = lut.parameterAtDistance(halfLength);

        measurements.rationalLineSampleCount = lut.samples().size();
        measurements.rationalLineLength = lut.totalLength();
        measurements.rationalLineHalfParameter = parameter;
        requireOrderedSamples(lut, "unequal-weight rational line");
        require(lut.samples().size() > 2, "nonlinear mapping must be refined");
        requireNear(
            parameter,
            0.25,
            lutTolerance(lut, options),
            "rational-line half-distance parameter"
        );
        requirePointNear(
            curve.evaluate(parameter),
            Point{1.0, 2.0, 3.0},
            lutTolerance(lut, options),
            "rational-line geometric midpoint"
        );

        requireDirectAgreement(curve, lut, options, "rational line");
        requireRoundTrips(curve, lut, options, "rational line");
    }

    void testQuarterCirclesAndScaling()
    {
        const ArcLengthLUTOptions options{};
        const NurbsCurve unitCurve = makeQuarterCircle(1.0);
        const NurbsCurve scaledCurve = makeQuarterCircle(10.0);
        const ArcLengthLUT unitLut =
            ArcLengthLUT::build(unitCurve, 0.0, 1.0, options);
        const ArcLengthLUT scaledLut =
            ArcLengthLUT::build(scaledCurve, 0.0, 1.0, options);

        measurements.unitCircleSampleCount = unitLut.samples().size();
        measurements.scaledCircleSampleCount = scaledLut.samples().size();
        measurements.unitCircleLength = unitLut.totalLength();
        measurements.scaledCircleLength = scaledLut.totalLength();
        requireOrderedSamples(unitLut, "unit quarter circle");
        requireOrderedSamples(scaledLut, "scaled quarter circle");
        requireNear(
            unitLut.totalLength(),
            0.5 * std::numbers::pi,
            referenceTolerance,
            "unit quarter-circle length"
        );
        requireNear(
            scaledLut.totalLength(),
            5.0 * std::numbers::pi,
            10.0 * referenceTolerance,
            "radius-10 quarter-circle length"
        );
        requirePointNear(
            unitCurve.evaluate(
                unitLut.parameterAtDistance(0.25 * std::numbers::pi)
            ),
            Point{std::sqrt(0.5), std::sqrt(0.5), 0.0},
            lutTolerance(unitLut, options),
            "unit quarter-circle half-arc point"
        );

        requireDirectAgreement(unitCurve, unitLut, options, "unit circle");
        requireDirectAgreement(
            scaledCurve,
            scaledLut,
            options,
            "scaled circle"
        );
        requireRoundTrips(unitCurve, unitLut, options, "unit circle");
        requireRoundTrips(scaledCurve, scaledLut, options, "scaled circle");

        require(
            scaledLut.samples().size() <= 2 * unitLut.samples().size(),
            "scale-relative tolerance should avoid disproportionate refinement"
        );
    }

    void testRepeatedKnotAndMultiSpan()
    {
        const BSplineCurve curve = makeRepeatedKnotBSpline();
        const ArcLengthLUTOptions options{};
        const ArcLengthLUT lut = ArcLengthLUT::build(curve, 0.0, 2.0, options);

        measurements.repeatedKnotSampleCount = lut.samples().size();
        measurements.repeatedKnotLength = lut.totalLength();
        requireOrderedSamples(lut, "repeated-knot curve");
        require(
            std::ranges::any_of(
                lut.samples(),
                [](const auto& sample)
                {
                    return sample.parameter == 1.0;
                }
            ),
            "distinct repeated interior knot must be an exact LUT sample"
        );

        const double knotDistance = evaluateArcLength(curve, 0.0, 1.0);
        measurements.repeatedKnotDistance = knotDistance;
        requireNear(
            lut.distanceAtParameter(1.0),
            knotDistance,
            referenceTolerance,
            "stored repeated-knot distance"
        );
        require(lut.distanceAtParameter(0.75) < knotDistance, "lookup before knot");
        require(lut.distanceAtParameter(1.25) > knotDistance, "lookup after knot");
        require(
            lut.parameterAtDistance(0.75 * lut.totalLength()) > 1.0,
            "distance query after repeated knot"
        );

        requireDirectAgreement(curve, lut, options, "repeated-knot curve");
        requireRoundTrips(curve, lut, options, "repeated-knot curve");
    }

    void testZeroLengthPolicies()
    {
        const BSplineCurve line = makeLinearBSpline();
        const ArcLengthLUT zeroWidth =
            ArcLengthLUT::build(line, 0.375, 0.375);

        require(zeroWidth.samples().size() == 1, "zero-width LUT sample count");
        require(zeroWidth.totalLength() == 0.0, "zero-width LUT length");
        require(
            zeroWidth.distanceAtParameter(0.375) == 0.0,
            "zero-width parameter query"
        );
        require(
            zeroWidth.parameterAtDistance(0.0) == 0.375,
            "zero-width inverse policy"
        );
        requireThrows<std::out_of_range>(
            [&zeroWidth]
            {
                static_cast<void>(zeroWidth.distanceAtParameter(0.5));
            },
            "zero-width out-of-range parameter"
        );
        requireThrows<std::out_of_range>(
            [&zeroWidth]
            {
                static_cast<void>(zeroWidth.parameterAtDistance(0.001));
            },
            "zero-width positive distance"
        );

        const BSplineCurve curve = makeZeroLengthBSpline();
        const ArcLengthLUT zeroLength =
            ArcLengthLUT::build(curve, 0.0, 1.0);

        requireOrderedSamples(zeroLength, "geometrically zero-length curve");
        require(zeroLength.totalLength() == 0.0, "geometric zero length");
        require(
            zeroLength.distanceAtParameter(0.625) == 0.0,
            "zero-length parameter query"
        );
        require(
            zeroLength.parameterAtDistance(0.0) == 0.0,
            "zero-length inverse selects interval beginning"
        );
        requireThrows<std::out_of_range>(
            [&zeroLength]
            {
                static_cast<void>(zeroLength.parameterAtDistance(1.0e-12));
            },
            "zero-length curve positive distance"
        );
    }

    void testStationaryPoint()
    {
        const BSplineCurve curve = makeStationaryPointBSpline();
        const ArcLengthLUTOptions options{};
        const ArcLengthLUT lut = ArcLengthLUT::build(curve, 0.0, 1.0, options);

        measurements.stationarySampleCount = lut.samples().size();
        requireOrderedSamples(lut, "stationary-point curve");
        require(
            curve.evaluateFirstDerivative(0.5) == Point{0.0, 0.0, 0.0},
            "test curve must contain an exact stationary point"
        );
        requireNear(
            lut.totalLength(),
            1.0,
            referenceTolerance,
            "stationary-point curve total length"
        );
        requireNear(
            lut.distanceAtParameter(0.5),
            0.5,
            lutTolerance(lut, options),
            "distance at stationary point"
        );
        requireNear(
            lut.distanceAtParameter(0.25),
            evaluateArcLength(curve, 0.0, 0.25),
            lutTolerance(lut, options),
            "distance before stationary point"
        );
        requireNear(
            lut.distanceAtParameter(0.75),
            evaluateArcLength(curve, 0.0, 0.5)
                + evaluateArcLength(curve, 0.5, 0.75),
            lutTolerance(lut, options),
            "distance after stationary point"
        );
        require(
            lut.parameterAtDistance(0.5) == 0.5,
            "stationary-point inverse should recover the exact sample"
        );
    }

    void testDeterminism()
    {
        const NurbsCurve curve = makeQuarterCircle(1.0);
        const ArcLengthLUT first = ArcLengthLUT::build(curve, 0.0, 1.0);
        const ArcLengthLUT second = ArcLengthLUT::build(curve, 0.0, 1.0);

        require(first.samples() == second.samples(), "repeated LUT construction");

        for (int index = 0; index <= 100; ++index)
        {
            const double fraction = static_cast<double>(index) / 100.0;
            require(
                first.distanceAtParameter(fraction)
                    == second.distanceAtParameter(fraction),
                "deterministic parameter query"
            );
            require(
                first.parameterAtDistance(fraction * first.totalLength())
                    == second.parameterAtDistance(
                        fraction * second.totalLength()
                    ),
                "deterministic distance query"
            );
        }
    }

    void testInvalidInputsAndWorkLimit()
    {
        const NurbsCurve curve = makeUnequalWeightLinearNurbs();
        const ArcLengthLUT lut = ArcLengthLUT::build(curve, 0.0, 1.0);
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();

        requireThrows<std::invalid_argument>(
            [&lut, nan]
            {
                static_cast<void>(lut.distanceAtParameter(nan));
            },
            "NaN parameter query"
        );
        requireThrows<std::invalid_argument>(
            [&lut, infinity]
            {
                static_cast<void>(lut.parameterAtDistance(infinity));
            },
            "infinite distance query"
        );
        requireThrows<std::out_of_range>(
            [&lut]
            {
                static_cast<void>(lut.distanceAtParameter(-0.001));
            },
            "below-range parameter query"
        );
        requireThrows<std::out_of_range>(
            [&lut]
            {
                static_cast<void>(lut.distanceAtParameter(1.001));
            },
            "above-range parameter query"
        );
        requireThrows<std::out_of_range>(
            [&lut]
            {
                static_cast<void>(lut.parameterAtDistance(-0.001));
            },
            "negative distance query"
        );
        requireThrows<std::out_of_range>(
            [&lut]
            {
                static_cast<void>(
                    lut.parameterAtDistance(lut.totalLength() + 0.001)
                );
            },
            "above-range distance query"
        );
        requireThrows<std::invalid_argument>(
            [&curve]
            {
                ArcLengthLUTOptions options{};
                options.absoluteTolerance = 0.0;
                options.relativeTolerance = 0.0;
                static_cast<void>(
                    ArcLengthLUT::build(curve, 0.0, 1.0, options)
                );
            },
            "zero LUT tolerances"
        );
        requireThrows<std::invalid_argument>(
            [&curve, nan]
            {
                ArcLengthLUTOptions options{};
                options.absoluteTolerance = nan;
                static_cast<void>(
                    ArcLengthLUT::build(curve, 0.0, 1.0, options)
                );
            },
            "NaN LUT tolerance"
        );

        requireThrows<std::runtime_error>(
            [&curve]
            {
                ArcLengthLUTOptions options{};
                options.absoluteTolerance = 1.0e-12;
                options.relativeTolerance = 0.0;
                options.maximumSubdivisionDepth = 0;
                static_cast<void>(
                    ArcLengthLUT::build(curve, 0.0, 1.0, options)
                );
            },
            "bounded LUT construction failure"
        );
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;

    const std::vector<Test> tests{
        {"straight B-spline", testStraightBSpline},
        {"unequal-weight rational line", testUnequalWeightRationalLine},
        {"quarter circles and scaling", testQuarterCirclesAndScaling},
        {"multi-span repeated-knot curve", testRepeatedKnotAndMultiSpan},
        {"zero-length policies", testZeroLengthPolicies},
        {"isolated stationary point", testStationaryPoint},
        {"deterministic construction and lookup", testDeterminism},
        {"invalid inputs and bounded work", testInvalidInputsAndWorkLimit}
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

    std::cout.precision(17);
    std::cout << "[METRIC] worst measured LUT distance error: "
              << measurements.worstDistanceError << '\n';
    std::cout << "[METRIC] lengths: straight="
              << measurements.straightLength
              << ", rational-line="
              << measurements.rationalLineLength
              << ", unit-circle="
              << measurements.unitCircleLength
              << ", radius-10-circle="
              << measurements.scaledCircleLength
              << ", repeated-knot="
              << measurements.repeatedKnotLength << '\n';
    std::cout << "[METRIC] rational-line half-distance parameter: "
              << measurements.rationalLineHalfParameter << '\n';
    std::cout << "[METRIC] repeated interior-knot distance: "
              << measurements.repeatedKnotDistance << '\n';
    std::cout << "[METRIC] sample counts: straight="
              << measurements.straightSampleCount
              << ", rational-line="
              << measurements.rationalLineSampleCount
              << ", unit-circle="
              << measurements.unitCircleSampleCount
              << ", radius-10-circle="
              << measurements.scaledCircleSampleCount
              << ", repeated-knot="
              << measurements.repeatedKnotSampleCount
              << ", stationary="
              << measurements.stationarySampleCount << '\n';
    std::cout << tests.size() << " test groups passed.\n";
    return 0;
}
