#include <quantum/geometry/BSplineCurve.hpp>
#include <quantum/geometry/CurveGeometry.hpp>
#include <quantum/geometry/CurveSampling.hpp>
#include <quantum/geometry/NurbsCurve.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
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
    using quantum::geometry::ArcLengthLUTOptions;
    using quantum::geometry::BSplineCurve;
    using quantum::geometry::CurveSample;
    using quantum::geometry::evaluateArcLength;
    using quantum::geometry::evaluateParameterAtArcLength;
    using quantum::geometry::maximumCurveSampleCount;
    using quantum::geometry::NurbsCurve;
    using quantum::geometry::sampleCurveByArcLength;
    using Point = glm::dvec3;

    constexpr double referenceTolerance = 1.0e-8;

    struct Measurements
    {
        double worstDistanceResidual = 0.0;
        double straightLength = 0.0;
        double rationalLineLength = 0.0;
        double unitCircleLength = 0.0;
        double radiusTenCircleLength = 0.0;
        double repeatedKnotLength = 0.0;
        double stationaryLength = 0.0;
        std::size_t straightSampleCount = 0;
        std::size_t rationalLineSampleCount = 0;
        std::size_t unitCircleSampleCount = 0;
        std::size_t radiusTenCircleSampleCount = 0;
        std::size_t repeatedKnotSampleCount = 0;
        std::size_t stationarySampleCount = 0;
        std::vector<double> rationalLineParameters;
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

    [[nodiscard]] BSplineCurve makeStraightBSpline()
    {
        return BSplineCurve(
            {Point{0.0, 0.0, 0.0}, Point{10.0, 0.0, 0.0}},
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

    [[nodiscard]] double samplingTolerance(
        const double totalLength,
        const ArcLengthLUTOptions& options
    )
    {
        return options.absoluteTolerance
            + options.relativeTolerance * totalLength;
    }

    template<typename Curve>
    void requireSampleInvariants(
        const Curve& curve,
        const std::vector<CurveSample>& samples,
        const double parameterBegin,
        const double parameterEnd,
        const double spacing,
        const ArcLengthLUTOptions& options,
        const std::string_view context
    )
    {
        require(!samples.empty(), "sampling must return at least one sample");
        require(samples.front().distance == 0.0, "exact beginning distance");
        require(
            samples.front().parameter == parameterBegin,
            "exact beginning parameter"
        );
        require(
            samples.front().position == curve.evaluate(parameterBegin),
            "exact beginning position"
        );

        const double totalLength = evaluateArcLength(
            curve,
            parameterBegin,
            parameterEnd,
            options.arcLength
        );

        if (totalLength == 0.0)
        {
            require(samples.size() == 1, "zero length must have one sample");
            return;
        }

        require(samples.size() >= 2, "nonzero length needs both endpoints");
        require(
            samples.back().distance == totalLength,
            "endpoint distance must equal total length exactly"
        );
        require(
            samples.back().parameter == parameterEnd,
            "exact endpoint parameter"
        );
        require(
            samples.back().position == curve.evaluate(parameterEnd),
            "exact endpoint position"
        );

        const double allowedResidual = samplingTolerance(totalLength, options);

        for (std::size_t index = 0; index < samples.size(); ++index)
        {
            const CurveSample& sample = samples[index];
            require(
                std::isfinite(sample.distance),
                std::string(context) + ": finite distance"
            );
            require(
                std::isfinite(sample.parameter),
                std::string(context) + ": finite parameter"
            );
            require(
                std::isfinite(sample.position.x)
                    && std::isfinite(sample.position.y)
                    && std::isfinite(sample.position.z),
                std::string(context) + ": finite position"
            );
            require(
                sample.distance >= 0.0 && sample.distance <= totalLength,
                std::string(context) + ": distance bounds"
            );
            require(
                sample.parameter >= parameterBegin
                    && sample.parameter <= parameterEnd,
                std::string(context) + ": parameter bounds"
            );

            if (index > 0)
            {
                require(
                    samples[index - 1].distance < sample.distance,
                    std::string(context) + ": strict distance ordering"
                );
                require(
                    samples[index - 1].parameter < sample.parameter,
                    std::string(context) + ": strict parameter ordering"
                );
            }

            if (index > 0 && index + 1 < samples.size())
            {
                require(
                    sample.distance
                        == static_cast<double>(index) * spacing,
                    std::string(context) + ": requested interior distance"
                );
            }

            const double directDistance = evaluateArcLength(
                curve,
                parameterBegin,
                sample.parameter,
                options.arcLength
            );
            const double residual = std::abs(
                directDistance - sample.distance
            );
            measurements.worstDistanceResidual = std::max(
                measurements.worstDistanceResidual,
                residual
            );

            if (residual > allowedResidual)
            {
                std::ostringstream message;
                message.precision(17);
                message << context << ": authoritative distance residual "
                        << residual << " exceeded " << allowedResidual
                        << " at sample " << index;
                throw TestFailure(message.str());
            }
        }
    }

    void testBasicRepresentation()
    {
        const CurveSample sample{
            1.25,
            0.375,
            Point{2.0, -3.0, 4.5}
        };

        require(sample.distance == 1.25, "CurveSample distance field");
        require(sample.parameter == 0.375, "CurveSample parameter field");
        require(
            sample.position == Point{2.0, -3.0, 4.5},
            "CurveSample position field"
        );
    }

    void testStraightBSpline()
    {
        const BSplineCurve curve = makeStraightBSpline();
        const ArcLengthLUTOptions options{};
        constexpr double spacing = 2.0;
        const std::vector<CurveSample> samples = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            spacing,
            options
        );

        measurements.straightLength = samples.back().distance;
        measurements.straightSampleCount = samples.size();
        require(samples.size() == 6, "straight sample count");
        requireSampleInvariants(
            curve,
            samples,
            0.0,
            1.0,
            spacing,
            options,
            "straight B-spline"
        );

        for (std::size_t index = 0; index < samples.size(); ++index)
        {
            const double distance = static_cast<double>(index) * spacing;
            requireNear(
                samples[index].parameter,
                distance / 10.0,
                referenceTolerance,
                "straight parameter"
            );
            requirePointNear(
                samples[index].position,
                Point{distance, 0.0, 0.0},
                referenceTolerance,
                "straight position"
            );
        }
    }

    void testUnequalWeightRationalLine()
    {
        const NurbsCurve curve = makeUnequalWeightLinearNurbs();
        const ArcLengthLUTOptions options{};
        const double totalLength = evaluateArcLength(curve, 0.0, 1.0);
        const double spacing = totalLength / 4.0;
        const std::vector<CurveSample> samples = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            spacing,
            options
        );

        measurements.rationalLineLength = samples.back().distance;
        measurements.rationalLineSampleCount = samples.size();
        require(samples.size() == 5, "rational-line sample count");
        requireSampleInvariants(
            curve,
            samples,
            0.0,
            1.0,
            spacing,
            options,
            "unequal-weight rational line"
        );

        for (std::size_t index = 0; index < samples.size(); ++index)
        {
            const double fraction = static_cast<double>(index) / 4.0;
            const double expectedParameter =
                fraction == 1.0
                ? 1.0
                : fraction / (3.0 - 2.0 * fraction);
            requireNear(
                samples[index].parameter,
                expectedParameter,
                2.0e-6,
                "rational-line nonlinear parameter"
            );
            requirePointNear(
                samples[index].position,
                Point{2.0, 4.0, 6.0} * fraction,
                2.0e-5,
                "rational-line equal-distance position"
            );
            measurements.rationalLineParameters.push_back(
                samples[index].parameter
            );
        }

        const double firstIncrement =
            samples[1].parameter - samples[0].parameter;
        const double secondIncrement =
            samples[2].parameter - samples[1].parameter;
        require(
            firstIncrement != secondIncrement,
            "unequal rational weights must not imply uniform parameters"
        );

        const double directParameter = evaluateParameterAtArcLength(
            curve,
            0.0,
            1.0,
            samples[2].distance
        );
        requireNear(
            samples[2].parameter,
            directParameter,
            2.0e-6,
            "rational-line LUT versus direct inversion"
        );
    }

    void testUnitQuarterCircle()
    {
        const NurbsCurve curve = makeQuarterCircle(1.0);
        const ArcLengthLUTOptions options{};
        const double totalLength = evaluateArcLength(curve, 0.0, 1.0);
        const double spacing = totalLength / 4.0;
        const std::vector<CurveSample> samples = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            spacing,
            options
        );

        measurements.unitCircleLength = samples.back().distance;
        measurements.unitCircleSampleCount = samples.size();
        require(samples.size() == 5, "unit-circle sample count");
        requireNear(
            totalLength,
            0.5 * std::numbers::pi,
            referenceTolerance,
            "unit quarter-circle length"
        );
        requireSampleInvariants(
            curve,
            samples,
            0.0,
            1.0,
            spacing,
            options,
            "unit quarter circle"
        );
        requireNear(
            samples[2].distance,
            0.25 * std::numbers::pi,
            referenceTolerance,
            "unit-circle half arc distance"
        );
        requirePointNear(
            samples[2].position,
            Point{std::sqrt(0.5), std::sqrt(0.5), 0.0},
            4.0e-6,
            "unit-circle 45-degree point"
        );

        const double directParameter = evaluateParameterAtArcLength(
            curve,
            0.0,
            1.0,
            samples[2].distance
        );
        requireNear(
            samples[2].parameter,
            directParameter,
            4.0e-6,
            "unit-circle LUT versus direct inversion"
        );
    }

    void testRadiusTenQuarterCircle()
    {
        const NurbsCurve curve = makeQuarterCircle(10.0);
        const ArcLengthLUTOptions options{};
        constexpr double spacing = 3.0;
        const std::vector<CurveSample> samples = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            spacing,
            options
        );

        measurements.radiusTenCircleLength = samples.back().distance;
        measurements.radiusTenCircleSampleCount = samples.size();
        require(samples.size() == 7, "radius-10 circle sample count");
        requireNear(
            samples.back().distance,
            5.0 * std::numbers::pi,
            10.0 * referenceTolerance,
            "radius-10 quarter-circle length"
        );
        requireSampleInvariants(
            curve,
            samples,
            0.0,
            1.0,
            spacing,
            options,
            "radius-10 quarter circle"
        );

        for (const std::size_t index : {1U, 3U, 5U})
        {
            const double angle = samples[index].distance / 10.0;
            requirePointNear(
                samples[index].position,
                Point{
                    10.0 * std::cos(angle),
                    10.0 * std::sin(angle),
                    0.0
                },
                3.0e-5,
                "radius-10 distance-to-position"
            );
        }

        require(
            samples.back().position == Point{0.0, 10.0, 0.0},
            "radius-10 exact endpoint position"
        );
    }

    void testEndpointAndPartialIntervalPolicies()
    {
        const BSplineCurve curve = makeStraightBSpline();
        const ArcLengthLUTOptions options{};

        const std::vector<CurveSample> partial = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            3.0,
            options
        );
        require(partial.size() == 5, "final partial sample count");
        const std::vector<double> expectedPartial{0.0, 3.0, 6.0, 9.0, 10.0};
        for (std::size_t index = 0; index < partial.size(); ++index)
        {
            require(
                partial[index].distance == expectedPartial[index],
                "final partial distances are not redistributed"
            );
        }
        require(
            partial.back().distance
                - partial[partial.size() - 2].distance == 1.0,
            "final partial gap"
        );

        const std::vector<CurveSample> exact = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            2.0,
            options
        );
        require(exact.size() == 6, "exact-multiple sample count");
        require(
            std::count_if(
                exact.begin(),
                exact.end(),
                [](const CurveSample& sample)
                {
                    return sample.distance == 10.0;
                }
            ) == 1,
            "exact-multiple endpoint must occur once"
        );

        const std::vector<CurveSample> larger = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            20.0,
            options
        );
        require(larger.size() == 2, "spacing larger than length");
        require(
            larger[0].distance == 0.0 && larger[1].distance == 10.0,
            "larger-spacing endpoint policy"
        );

        const std::vector<CurveSample> equal = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            10.0,
            options
        );
        require(equal.size() == 2, "spacing equal to length");
        require(
            equal[0].distance == 0.0 && equal[1].distance == 10.0,
            "equal-spacing endpoint policy"
        );
    }

    void testZeroLengthPolicies()
    {
        const BSplineCurve line = makeStraightBSpline();
        const std::vector<CurveSample> zeroWidth = sampleCurveByArcLength(
            line,
            0.375,
            0.375,
            1.0
        );

        require(zeroWidth.size() == 1, "zero-width sample count");
        require(zeroWidth.front().distance == 0.0, "zero-width distance");
        require(zeroWidth.front().parameter == 0.375, "zero-width parameter");
        require(
            zeroWidth.front().position == line.evaluate(0.375),
            "zero-width position"
        );

        const BSplineCurve zeroLengthCurve = makeZeroLengthBSpline();
        const std::vector<CurveSample> zeroLength = sampleCurveByArcLength(
            zeroLengthCurve,
            0.0,
            1.0,
            0.001
        );

        require(zeroLength.size() == 1, "geometric zero-length sample count");
        require(zeroLength.front().distance == 0.0, "zero-length distance");
        require(zeroLength.front().parameter == 0.0, "zero-length parameter");
        require(
            zeroLength.front().position == Point{3.0, -2.0, 5.0},
            "zero-length beginning position"
        );
    }

    void testRepeatedKnotAndMultiSpan()
    {
        const BSplineCurve curve = makeRepeatedKnotBSpline();
        const ArcLengthLUTOptions options{};
        constexpr double spacing = 1.0;
        const std::vector<CurveSample> samples = sampleCurveByArcLength(
            curve,
            0.0,
            2.0,
            spacing,
            options
        );

        measurements.repeatedKnotLength = samples.back().distance;
        measurements.repeatedKnotSampleCount = samples.size();
        require(samples.size() == 9, "repeated-knot sample count");
        requireSampleInvariants(
            curve,
            samples,
            0.0,
            2.0,
            spacing,
            options,
            "repeated-knot curve"
        );

        const double knotDistance = evaluateArcLength(curve, 0.0, 1.0);
        require(knotDistance > 2.0 && knotDistance < 3.0, "knot distance bracket");
        require(samples[2].parameter < 1.0, "sample before repeated knot");
        require(samples[3].parameter > 1.0, "sample after repeated knot");
        require(
            std::none_of(
                samples.begin(),
                samples.end(),
                [](const CurveSample& sample)
                {
                    return sample.parameter == 1.0;
                }
            ),
            "canonical output must not inject an unaligned knot sample"
        );
    }

    void testStationaryPoint()
    {
        const BSplineCurve curve = makeStationaryPointBSpline();
        const ArcLengthLUTOptions options{};
        constexpr double spacing = 0.25;
        const std::vector<CurveSample> samples = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            spacing,
            options
        );

        measurements.stationaryLength = samples.back().distance;
        measurements.stationarySampleCount = samples.size();
        require(samples.size() == 5, "stationary-point sample count");
        require(
            curve.evaluateFirstDerivative(0.5) == Point{0.0, 0.0, 0.0},
            "test curve has an exact stationary point"
        );
        requireSampleInvariants(
            curve,
            samples,
            0.0,
            1.0,
            spacing,
            options,
            "stationary-point curve"
        );
        require(samples[2].distance == 0.5, "stationary sample distance");
        require(samples[2].parameter == 0.5, "stationary sample parameter");
        require(
            samples[2].position == Point{0.5, 0.0, 0.0},
            "stationary sample position"
        );
    }

    void testInvalidInputsAndResourceGuard()
    {
        const BSplineCurve curve = makeStraightBSpline();
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();

        for (const double spacing : {0.0, -1.0, nan, infinity, -infinity})
        {
            requireThrows<std::invalid_argument>(
                [&curve, spacing]
                {
                    static_cast<void>(
                        sampleCurveByArcLength(
                            curve,
                            0.0,
                            1.0,
                            spacing
                        )
                    );
                },
                "invalid spacing"
            );
        }

        requireThrows<std::invalid_argument>(
            [&curve]
            {
                static_cast<void>(
                    sampleCurveByArcLength(curve, 0.75, 0.25, 1.0)
                );
            },
            "reversed parameter interval"
        );
        requireThrows<std::invalid_argument>(
            [&curve, nan]
            {
                static_cast<void>(
                    sampleCurveByArcLength(curve, nan, 1.0, 1.0)
                );
            },
            "non-finite parameter interval"
        );
        requireThrows<std::out_of_range>(
            [&curve]
            {
                static_cast<void>(
                    sampleCurveByArcLength(curve, -0.1, 1.0, 1.0)
                );
            },
            "out-of-domain parameter interval"
        );

        const double excessiveSpacing =
            10.0 / static_cast<double>(maximumCurveSampleCount);
        requireThrows<std::length_error>(
            [&curve, excessiveSpacing]
            {
                static_cast<void>(
                    sampleCurveByArcLength(
                        curve,
                        0.0,
                        1.0,
                        excessiveSpacing
                    )
                );
            },
            "excessive canonical sample request"
        );
    }

    [[nodiscard]] std::uint64_t doubleBits(const double value)
    {
        return std::bit_cast<std::uint64_t>(value);
    }

    void testDeterminism()
    {
        const NurbsCurve curve = makeQuarterCircle(1.0);
        const std::vector<CurveSample> first = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            0.2
        );
        const std::vector<CurveSample> second = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            0.2
        );

        require(first == second, "deterministic sample value equality");
        require(first.size() == second.size(), "deterministic sample count");

        for (std::size_t index = 0; index < first.size(); ++index)
        {
            require(
                doubleBits(first[index].distance)
                    == doubleBits(second[index].distance),
                "bitwise deterministic distance"
            );
            require(
                doubleBits(first[index].parameter)
                    == doubleBits(second[index].parameter),
                "bitwise deterministic parameter"
            );
            require(
                doubleBits(first[index].position.x)
                    == doubleBits(second[index].position.x)
                    && doubleBits(first[index].position.y)
                        == doubleBits(second[index].position.y)
                    && doubleBits(first[index].position.z)
                        == doubleBits(second[index].position.z),
                "bitwise deterministic position"
            );
        }
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;

    const std::vector<Test> tests{
        {"basic canonical sample representation", testBasicRepresentation},
        {"straight B-spline", testStraightBSpline},
        {"unequal-weight rational line", testUnequalWeightRationalLine},
        {"unit quarter circle", testUnitQuarterCircle},
        {"radius-10 quarter circle", testRadiusTenQuarterCircle},
        {"endpoint and final-partial policies", testEndpointAndPartialIntervalPolicies},
        {"zero-width and zero-length policies", testZeroLengthPolicies},
        {"multi-span repeated-knot curve", testRepeatedKnotAndMultiSpan},
        {"isolated stationary point", testStationaryPoint},
        {"invalid inputs and resource guard", testInvalidInputsAndResourceGuard},
        {"deterministic generation", testDeterminism}
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
    std::cout << "[METRIC] worst authoritative distance residual: "
              << measurements.worstDistanceResidual << '\n';
    std::cout << "[METRIC] lengths: straight="
              << measurements.straightLength
              << ", rational-line="
              << measurements.rationalLineLength
              << ", unit-circle="
              << measurements.unitCircleLength
              << ", radius-10-circle="
              << measurements.radiusTenCircleLength
              << ", repeated-knot="
              << measurements.repeatedKnotLength
              << ", stationary="
              << measurements.stationaryLength << '\n';
    std::cout << "[METRIC] sample counts: straight="
              << measurements.straightSampleCount
              << ", rational-line="
              << measurements.rationalLineSampleCount
              << ", unit-circle="
              << measurements.unitCircleSampleCount
              << ", radius-10-circle="
              << measurements.radiusTenCircleSampleCount
              << ", repeated-knot="
              << measurements.repeatedKnotSampleCount
              << ", stationary="
              << measurements.stationarySampleCount << '\n';
    std::cout << "[METRIC] unequal-weight rational-line parameters:";
    for (const double parameter : measurements.rationalLineParameters)
    {
        std::cout << ' ' << parameter;
    }
    std::cout << '\n';
    std::cout << tests.size() << " test groups passed.\n";
    return 0;
}
