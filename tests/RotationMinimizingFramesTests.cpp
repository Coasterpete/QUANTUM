#include <quantum/geometry/BSplineCurve.hpp>
#include <quantum/geometry/CurveGeometry.hpp>
#include <quantum/geometry/CurveSampling.hpp>
#include <quantum/geometry/NurbsCurve.hpp>
#include <quantum/geometry/RotationMinimizingFrames.hpp>

#include <algorithm>
#include <array>
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
#include <type_traits>
#include <utility>
#include <vector>

namespace
{
    using quantum::geometry::ArcLengthLUTOptions;
    using quantum::geometry::applyLocalPitch;
    using quantum::geometry::applyLocalYaw;
    using quantum::geometry::applyRoll;
    using quantum::geometry::BSplineCurve;
    using quantum::geometry::buildRotationMinimizingFrames;
    using quantum::geometry::CurveFrame;
    using quantum::geometry::CurveSample;
    using quantum::geometry::evaluateUnitTangent;
    using quantum::geometry::NurbsCurve;
    using quantum::geometry::sampleCurveByArcLength;
    using Point = glm::dvec3;

    constexpr double frameTolerance = 2.0e-12;

    struct Measurements
    {
        double maximumOrthonormalityError = 0.0;
        double maximumTangentAgreementError = 0.0;
        double maximumDensityOrientationDifference = 0.0;
        double beginningDensityOrientationDifference = 0.0;
        double middleDensityOrientationDifference = 0.0;
        double endpointDensityOrientationDifference = 0.0;
        double maximumRollOrthonormalityError = 0.0;
        double maximumRollTangentChangeError = 0.0;
        double maximumRollReferenceError = 0.0;
        double maximumRollCompositionError = 0.0;
        double maximumRollPeriodicityError = 0.0;
        double largeAngleReferenceError = 0.0;
        double maximumPlanarRelativeRollError = 0.0;
        double maximumSpatialRelativeRollError = 0.0;
        double maximumLocalRotationOrthonormalityError = 0.0;
        double maximumLocalRotationAxisChangeError = 0.0;
        double maximumLocalRotationReferenceError = 0.0;
        double maximumRolledLocalRotationReferenceError = 0.0;
        double localRotationOrderDifference = 0.0;
        std::size_t straightFrameCount = 0;
        std::size_t quarterCircleFrameCount = 0;
        std::size_t spatialFrameCount = 0;
        std::size_t repeatedKnotFrameCount = 0;
        std::size_t planarRolledFrameCount = 0;
        std::size_t spatialRolledFrameCount = 0;
        std::size_t planarLocalRotationFrameCount = 0;
        std::size_t spatialLocalRotationFrameCount = 0;
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

    [[nodiscard]] double magnitude(const Point& vector)
    {
        return std::hypot(vector.x, vector.y, vector.z);
    }

    [[nodiscard]] double pointError(
        const Point& first,
        const Point& second
    )
    {
        return magnitude(first - second);
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
        requireNear(pointError(actual, expected), 0.0, tolerance, context);
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

    [[nodiscard]] std::uint64_t doubleBits(const double value)
    {
        return std::bit_cast<std::uint64_t>(value);
    }

    [[nodiscard]] bool pointBitsEqual(
        const Point& first,
        const Point& second
    )
    {
        return doubleBits(first.x) == doubleBits(second.x)
            && doubleBits(first.y) == doubleBits(second.y)
            && doubleBits(first.z) == doubleBits(second.z);
    }

    [[nodiscard]] bool frameBitsEqual(
        const CurveFrame& first,
        const CurveFrame& second
    )
    {
        return pointBitsEqual(first.tangent, second.tangent)
            && pointBitsEqual(first.lateral, second.lateral)
            && pointBitsEqual(first.up, second.up);
    }

    void requireValidRolledFrame(
        const CurveFrame& input,
        const CurveFrame& rolled,
        const std::string_view context
    )
    {
        require(
            std::isfinite(rolled.tangent.x)
                && std::isfinite(rolled.tangent.y)
                && std::isfinite(rolled.tangent.z)
                && std::isfinite(rolled.lateral.x)
                && std::isfinite(rolled.lateral.y)
                && std::isfinite(rolled.lateral.z)
                && std::isfinite(rolled.up.x)
                && std::isfinite(rolled.up.y)
                && std::isfinite(rolled.up.z),
            std::string(context) + " rolled frame components must be finite"
        );

        const double tangentLengthError =
            std::abs(magnitude(rolled.tangent) - 1.0);
        const double lateralLengthError =
            std::abs(magnitude(rolled.lateral) - 1.0);
        const double upLengthError = std::abs(magnitude(rolled.up) - 1.0);
        const double tangentLateralError =
            std::abs(glm::dot(rolled.tangent, rolled.lateral));
        const double tangentUpError =
            std::abs(glm::dot(rolled.tangent, rolled.up));
        const double lateralUpError =
            std::abs(glm::dot(rolled.lateral, rolled.up));
        const double handednessError = pointError(
            glm::cross(rolled.tangent, rolled.lateral),
            rolled.up
        );

        measurements.maximumRollOrthonormalityError = std::max({
            measurements.maximumRollOrthonormalityError,
            tangentLengthError,
            lateralLengthError,
            upLengthError,
            tangentLateralError,
            tangentUpError,
            lateralUpError,
            handednessError
        });

        require(
            tangentLengthError <= frameTolerance
                && lateralLengthError <= frameTolerance
                && upLengthError <= frameTolerance
                && tangentLateralError <= frameTolerance
                && tangentUpError <= frameTolerance
                && lateralUpError <= frameTolerance
                && handednessError <= frameTolerance,
            std::string(context)
                + " rolled frame must be an orthonormal right-handed basis"
        );

        const double tangentChangeError =
            pointError(rolled.tangent, input.tangent);
        measurements.maximumRollTangentChangeError = std::max(
            measurements.maximumRollTangentChangeError,
            tangentChangeError
        );
        require(
            pointBitsEqual(rolled.tangent, input.tangent),
            std::string(context)
                + " roll must preserve the tangent bit-for-bit"
        );
    }

    void requireRollReference(
        const CurveFrame& actual,
        const CurveFrame& expected,
        const double tolerance,
        const std::string_view context
    )
    {
        const double referenceError = std::max({
            pointError(actual.tangent, expected.tangent),
            pointError(actual.lateral, expected.lateral),
            pointError(actual.up, expected.up)
        });
        measurements.maximumRollReferenceError = std::max(
            measurements.maximumRollReferenceError,
            referenceError
        );
        requireNear(
            referenceError,
            0.0,
            tolerance,
            std::string(context) + " analytic roll reference"
        );
    }

    void requireValidLocalRotationFrame(
        const CurveFrame& frame,
        const std::string_view context
    )
    {
        require(
            std::isfinite(frame.tangent.x)
                && std::isfinite(frame.tangent.y)
                && std::isfinite(frame.tangent.z)
                && std::isfinite(frame.lateral.x)
                && std::isfinite(frame.lateral.y)
                && std::isfinite(frame.lateral.z)
                && std::isfinite(frame.up.x)
                && std::isfinite(frame.up.y)
                && std::isfinite(frame.up.z),
            std::string(context) + " frame components must be finite"
        );

        const double tangentLengthError =
            std::abs(magnitude(frame.tangent) - 1.0);
        const double lateralLengthError =
            std::abs(magnitude(frame.lateral) - 1.0);
        const double upLengthError = std::abs(magnitude(frame.up) - 1.0);
        const double tangentLateralError =
            std::abs(glm::dot(frame.tangent, frame.lateral));
        const double tangentUpError =
            std::abs(glm::dot(frame.tangent, frame.up));
        const double lateralUpError =
            std::abs(glm::dot(frame.lateral, frame.up));
        const double handednessError = pointError(
            glm::cross(frame.tangent, frame.lateral),
            frame.up
        );

        measurements.maximumLocalRotationOrthonormalityError = std::max({
            measurements.maximumLocalRotationOrthonormalityError,
            tangentLengthError,
            lateralLengthError,
            upLengthError,
            tangentLateralError,
            tangentUpError,
            lateralUpError,
            handednessError
        });

        require(
            tangentLengthError <= frameTolerance
                && lateralLengthError <= frameTolerance
                && upLengthError <= frameTolerance
                && tangentLateralError <= frameTolerance
                && tangentUpError <= frameTolerance
                && lateralUpError <= frameTolerance
                && handednessError <= frameTolerance,
            std::string(context)
                + " frame must be an orthonormal right-handed basis"
        );
    }

    void requirePreservedLocalRotationAxis(
        const Point& actual,
        const Point& expected,
        const std::string_view context
    )
    {
        measurements.maximumLocalRotationAxisChangeError = std::max(
            measurements.maximumLocalRotationAxisChangeError,
            pointError(actual, expected)
        );
        require(
            pointBitsEqual(actual, expected),
            std::string(context)
                + " must preserve its local rotation axis bit-for-bit"
        );
    }

    void requireLocalRotationReference(
        const CurveFrame& actual,
        const CurveFrame& expected,
        const std::string_view context,
        double& maximumError
    )
    {
        const double referenceError = std::max({
            pointError(actual.tangent, expected.tangent),
            pointError(actual.lateral, expected.lateral),
            pointError(actual.up, expected.up)
        });
        maximumError = std::max(maximumError, referenceError);
        requireNear(
            referenceError,
            0.0,
            frameTolerance,
            std::string(context) + " analytic local-rotation reference"
        );
    }

    [[nodiscard]] CurveFrame controlledFrame()
    {
        return {
            Point{1.0, 0.0, 0.0},
            Point{0.0, 0.0, -1.0},
            Point{0.0, 1.0, 0.0}
        };
    }

    [[nodiscard]] CurveFrame controlledRollReference(const double angle)
    {
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);

        // For T=+X, L=-Z, and U=+Y, the right-hand rule gives
        // L' = cos(angle)L + sin(angle)U and
        // U' = cos(angle)U - sin(angle)L.
        return {
            Point{1.0, 0.0, 0.0},
            Point{0.0, sine, -cosine},
            Point{0.0, cosine, sine}
        };
    }

    [[nodiscard]] CurveFrame localPitchReference(
        const CurveFrame& frame,
        const double angle
    )
    {
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);

        // T x L = U implies L x T = -U and L x U = T.
        return {
            cosine * frame.tangent - sine * frame.up,
            frame.lateral,
            sine * frame.tangent + cosine * frame.up
        };
    }

    [[nodiscard]] CurveFrame localYawReference(
        const CurveFrame& frame,
        const double angle
    )
    {
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);

        // T x L = U implies U x T = L and U x L = -T.
        return {
            cosine * frame.tangent + sine * frame.lateral,
            cosine * frame.lateral - sine * frame.tangent,
            frame.up
        };
    }

    [[nodiscard]] double angularDifference(
        const Point& first,
        const Point& second
    )
    {
        return std::atan2(
            magnitude(glm::cross(first, second)),
            std::clamp(glm::dot(first, second), -1.0, 1.0)
        );
    }

    [[nodiscard]] ArcLengthLUTOptions samplingOptions(
        const double coordinateScale = 1.0
    )
    {
        ArcLengthLUTOptions options;
        options.absoluteTolerance = coordinateScale * 1.0e-6;
        options.relativeTolerance = 1.0e-6;
        options.arcLength.absoluteTolerance = coordinateScale * 1.0e-9;
        options.arcLength.relativeTolerance = 1.0e-9;
        return options;
    }

    [[nodiscard]] BSplineCurve makeStraightBSpline()
    {
        return BSplineCurve(
            {Point{0.0, 0.0, 0.0}, Point{10.0, 0.0, 0.0}},
            1,
            {0.0, 0.0, 1.0, 1.0}
        );
    }

    [[nodiscard]] BSplineCurve makeDiagonalBSpline()
    {
        return BSplineCurve(
            {Point{0.0, 0.0, 0.0}, Point{4.0, 4.0, 0.0}},
            1,
            {0.0, 0.0, 1.0, 1.0}
        );
    }

    [[nodiscard]] NurbsCurve makeQuarterCircle()
    {
        return NurbsCurve(
            {
                Point{1.0, 0.0, 0.0},
                Point{1.0, 1.0, 0.0},
                Point{0.0, 1.0, 0.0}
            },
            {1.0, std::sqrt(0.5), 1.0},
            2,
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}
        );
    }

    [[nodiscard]] BSplineCurve makeSpatialBSpline(
        const double scale = 1.0
    )
    {
        return BSplineCurve(
            {
                scale * Point{0.0, 0.0, 0.0},
                scale * Point{1.0, 0.0, 0.5},
                scale * Point{2.0, 1.0, 2.0},
                scale * Point{2.0, 3.0, 2.0}
            },
            3,
            {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}
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

    [[nodiscard]] BSplineCurve makeStraightRunBSpline()
    {
        return BSplineCurve(
            {
                Point{0.0, 0.0, 0.0},
                Point{0.0, 1.0, 0.0},
                Point{1.0, 1.0, 0.0},
                Point{6.0, 1.0, 0.0}
            },
            1,
            {0.0, 0.0, 1.0, 2.0, 3.0, 3.0}
        );
    }

    template<typename Curve>
    void requireValidFrame(
        const Curve& curve,
        const CurveSample& sample,
        const CurveFrame& frame,
        const std::string_view context
    )
    {
        require(
            std::isfinite(frame.tangent.x)
                && std::isfinite(frame.tangent.y)
                && std::isfinite(frame.tangent.z)
                && std::isfinite(frame.lateral.x)
                && std::isfinite(frame.lateral.y)
                && std::isfinite(frame.lateral.z)
                && std::isfinite(frame.up.x)
                && std::isfinite(frame.up.y)
                && std::isfinite(frame.up.z),
            std::string(context) + " frame components must be finite"
        );

        const double tangentLengthError =
            std::abs(magnitude(frame.tangent) - 1.0);
        const double lateralLengthError =
            std::abs(magnitude(frame.lateral) - 1.0);
        const double upLengthError = std::abs(magnitude(frame.up) - 1.0);
        const double tangentLateralError =
            std::abs(glm::dot(frame.tangent, frame.lateral));
        const double tangentUpError =
            std::abs(glm::dot(frame.tangent, frame.up));
        const double lateralUpError =
            std::abs(glm::dot(frame.lateral, frame.up));
        const double handednessError = pointError(
            glm::cross(frame.tangent, frame.lateral),
            frame.up
        );

        measurements.maximumOrthonormalityError = std::max({
            measurements.maximumOrthonormalityError,
            tangentLengthError,
            lateralLengthError,
            upLengthError,
            tangentLateralError,
            tangentUpError,
            lateralUpError,
            handednessError
        });

        require(
            tangentLengthError <= frameTolerance
                && lateralLengthError <= frameTolerance
                && upLengthError <= frameTolerance
                && tangentLateralError <= frameTolerance
                && tangentUpError <= frameTolerance
                && lateralUpError <= frameTolerance
                && handednessError <= frameTolerance,
            std::string(context)
                + " frame must be an orthonormal right-handed basis"
        );

        const Point expectedTangent =
            evaluateUnitTangent(curve, sample.parameter);
        const double tangentAgreementError =
            pointError(frame.tangent, expectedTangent);
        measurements.maximumTangentAgreementError = std::max(
            measurements.maximumTangentAgreementError,
            tangentAgreementError
        );
        requireNear(
            tangentAgreementError,
            0.0,
            0.0,
            std::string(context) + " analytic tangent agreement"
        );
    }

    template<typename Curve>
    void requireValidSequence(
        const Curve& curve,
        const std::vector<CurveSample>& samples,
        const std::vector<CurveFrame>& frames,
        const std::string_view context
    )
    {
        require(
            frames.size() == samples.size(),
            std::string(context) + " frame count must match sample count"
        );

        for (std::size_t index = 0; index < frames.size(); ++index)
        {
            requireValidFrame(
                curve,
                samples[index],
                frames[index],
                std::string(context) + " frame " + std::to_string(index)
            );
        }
    }

    void testRepresentationEmptyAndSingleSample()
    {
        static_assert(
            std::is_same_v<decltype(CurveFrame{}.tangent), glm::dvec3>
        );
        static_assert(
            std::is_same_v<decltype(CurveFrame{}.lateral), glm::dvec3>
        );
        static_assert(std::is_same_v<decltype(CurveFrame{}.up), glm::dvec3>);
        static_assert(
            std::is_same_v<decltype(CurveSample{}.distance), double>
        );
        static_assert(
            std::is_same_v<decltype(CurveSample{}.parameter), double>
        );
        static_assert(
            std::is_same_v<decltype(CurveSample{}.position), glm::dvec3>
        );

        const CurveFrame represented{
            Point{1.0, 0.0, 0.0},
            Point{0.0, 0.0, -1.0},
            Point{0.0, 1.0, 0.0}
        };
        require(
            represented.tangent == Point{1.0, 0.0, 0.0}
                && represented.lateral == Point{0.0, 0.0, -1.0}
                && represented.up == Point{0.0, 1.0, 0.0},
            "CurveFrame must expose tangent, lateral, and up"
        );

        struct RejectingCurve
        {
            [[nodiscard]] Point evaluateFirstDerivative(double) const
            {
                throw std::runtime_error("curve must not be evaluated");
            }
        };

        const std::vector<CurveFrame> emptyFrames =
            buildRotationMinimizingFrames(
                RejectingCurve{},
                std::vector<CurveSample>{},
                Point{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}
            );
        require(
            emptyFrames.empty(),
            "empty samples must return empty frames without curve evaluation"
        );

        const BSplineCurve curve = makeStraightBSpline();
        const std::vector<CurveSample> samples = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            2.0,
            samplingOptions()
        );
        const std::vector<CurveSample> singleSample{samples.front()};
        const std::vector<CurveFrame> singleFrame =
            buildRotationMinimizingFrames(
                curve,
                singleSample,
                Point{0.0, 1.0, 0.0}
            );
        requireValidSequence(curve, singleSample, singleFrame, "single sample");
        require(singleFrame.size() == 1, "single sample must produce one frame");
    }

    void testStraightBSplineAndWorldUpProjection()
    {
        const BSplineCurve straight = makeStraightBSpline();
        const std::vector<CurveSample> straightSamples =
            sampleCurveByArcLength(
                straight,
                0.0,
                1.0,
                0.125,
                samplingOptions()
            );
        const std::vector<CurveFrame> straightFrames =
            buildRotationMinimizingFrames(
                straight,
                straightSamples,
                Point{0.0, 1.0, 0.0}
            );
        requireValidSequence(
            straight,
            straightSamples,
            straightFrames,
            "straight B-spline"
        );
        measurements.straightFrameCount = straightFrames.size();

        for (const CurveFrame& frame : straightFrames)
        {
            require(
                pointBitsEqual(frame.tangent, straightFrames.front().tangent)
                    && pointBitsEqual(
                        frame.lateral,
                        straightFrames.front().lateral
                    )
                    && pointBitsEqual(frame.up, straightFrames.front().up),
                "a straight curve must accumulate no frame rotation"
            );
        }

        requirePointNear(
            straightFrames.front().tangent,
            Point{1.0, 0.0, 0.0},
            frameTolerance,
            "straight tangent"
        );
        requirePointNear(
            straightFrames.front().lateral,
            Point{0.0, 0.0, -1.0},
            frameTolerance,
            "straight lateral"
        );
        requirePointNear(
            straightFrames.front().up,
            Point{0.0, 1.0, 0.0},
            frameTolerance,
            "straight up"
        );

        const BSplineCurve diagonal = makeDiagonalBSpline();
        const std::vector<CurveSample> diagonalSamples =
            sampleCurveByArcLength(
                diagonal,
                0.0,
                1.0,
                1.0,
                samplingOptions()
            );
        const std::vector<CurveFrame> diagonalFrames =
            buildRotationMinimizingFrames(
                diagonal,
                diagonalSamples,
                Point{0.0, 1.0, 0.0}
            );
        const double inverseSqrtTwo = 1.0 / std::sqrt(2.0);
        requirePointNear(
            diagonalFrames.front().up,
            Point{-inverseSqrtTwo, inverseSqrtTwo, 0.0},
            frameTolerance,
            "+Y projection onto the initial normal plane"
        );
        requireValidSequence(
            diagonal,
            diagonalSamples,
            diagonalFrames,
            "+Y initial-reference curve"
        );
    }

    void testQuarterCircle()
    {
        const NurbsCurve curve = makeQuarterCircle();
        const std::vector<CurveSample> samples = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            std::numbers::pi / 64.0,
            samplingOptions()
        );
        const std::vector<CurveFrame> frames =
            buildRotationMinimizingFrames(
                curve,
                samples,
                Point{0.0, 0.0, 1.0}
            );
        requireValidSequence(curve, samples, frames, "unit quarter circle");
        measurements.quarterCircleFrameCount = frames.size();

        for (std::size_t index = 0; index < frames.size(); ++index)
        {
            const Point radial = curve.evaluate(samples[index].parameter);
            requirePointNear(
                frames[index].up,
                Point{0.0, 0.0, 1.0},
                2.0e-12,
                "quarter-circle transported up"
            );
            requirePointNear(
                frames[index].lateral,
                -radial,
                2.0e-10,
                "quarter-circle inward lateral"
            );
        }
    }

    void testSpatialCurve()
    {
        const BSplineCurve curve = makeSpatialBSpline();
        const std::vector<CurveSample> samples = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            0.08,
            samplingOptions()
        );
        const std::vector<CurveFrame> frames =
            buildRotationMinimizingFrames(
                curve,
                samples,
                Point{0.0, 1.0, 0.0}
            );
        requireValidSequence(curve, samples, frames, "spatial B-spline");
        measurements.spatialFrameCount = frames.size();

        for (std::size_t index = 1; index < frames.size(); ++index)
        {
            require(
                glm::dot(frames[index - 1].lateral, frames[index].lateral)
                        > 0.95
                    && glm::dot(frames[index - 1].up, frames[index].up)
                        > 0.95,
                "spatial transported axes must remain continuous"
            );
        }

        requirePointNear(
            frames.back().tangent,
            Point{0.0, 1.0, 0.0},
            frameTolerance,
            "spatial endpoint tangent"
        );
        require(
            std::abs(glm::dot(frames.back().up, Point{0.0, 1.0, 0.0}))
                    <= frameTolerance
                && glm::dot(frames[frames.size() - 2].up, frames.back().up)
                    > 0.99,
            "transport through a world-up tangent must not snap to world up"
        );
    }

    void testUniformScaleInvariance()
    {
        constexpr double scale = 8.0;
        const BSplineCurve baseCurve = makeSpatialBSpline();
        const BSplineCurve scaledCurve = makeSpatialBSpline(scale);
        const std::vector<CurveSample> baseSamples = sampleCurveByArcLength(
            baseCurve,
            0.0,
            1.0,
            0.125,
            samplingOptions()
        );
        const std::vector<CurveSample> scaledSamples = sampleCurveByArcLength(
            scaledCurve,
            0.0,
            1.0,
            scale * 0.125,
            samplingOptions(scale)
        );
        const std::vector<CurveFrame> baseFrames =
            buildRotationMinimizingFrames(
                baseCurve,
                baseSamples,
                Point{0.0, 1.0, 0.0}
            );
        const std::vector<CurveFrame> scaledFrames =
            buildRotationMinimizingFrames(
                scaledCurve,
                scaledSamples,
                Point{0.0, 1.0, 0.0}
            );
        requireValidSequence(
            baseCurve,
            baseSamples,
            baseFrames,
            "base scale-invariance sequence"
        );
        requireValidSequence(
            scaledCurve,
            scaledSamples,
            scaledFrames,
            "scaled scale-invariance sequence"
        );

        require(
            baseFrames.size() == scaledFrames.size(),
            "uniformly scaled sampling must produce corresponding frames"
        );

        for (std::size_t index = 0; index < baseFrames.size(); ++index)
        {
            requireNear(
                scaledSamples[index].parameter,
                baseSamples[index].parameter,
                2.0e-12,
                "scale-invariant sample parameter"
            );
            requirePointNear(
                scaledFrames[index].tangent,
                baseFrames[index].tangent,
                2.0e-12,
                "scale-invariant tangent"
            );
            requirePointNear(
                scaledFrames[index].lateral,
                baseFrames[index].lateral,
                2.0e-12,
                "scale-invariant lateral"
            );
            requirePointNear(
                scaledFrames[index].up,
                baseFrames[index].up,
                2.0e-12,
                "scale-invariant up"
            );
        }
    }

    void testInitialRollDependence()
    {
        const BSplineCurve curve = makeSpatialBSpline();
        const std::vector<CurveSample> samples = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            0.1,
            samplingOptions()
        );
        const std::vector<CurveFrame> worldUpFrames =
            buildRotationMinimizingFrames(
                curve,
                samples,
                Point{0.0, 1.0, 0.0}
            );
        const std::vector<CurveFrame> worldZFrames =
            buildRotationMinimizingFrames(
                curve,
                samples,
                Point{0.0, 0.0, 1.0}
            );
        requireValidSequence(
            curve,
            samples,
            worldUpFrames,
            "world-up initial roll"
        );
        requireValidSequence(
            curve,
            samples,
            worldZFrames,
            "world-Z initial roll"
        );

        for (std::size_t index = 0; index < samples.size(); ++index)
        {
            require(
                pointBitsEqual(
                    worldUpFrames[index].tangent,
                    worldZFrames[index].tangent
                ),
                "initial roll choice must not alter tangents"
            );
            require(
                std::abs(glm::dot(
                    worldUpFrames[index].up,
                    worldZFrames[index].up
                )) < 1.0e-10,
                "different initial references must preserve different roll"
            );
        }
    }

    void testStraightRunAndRepeatedKnot()
    {
        const BSplineCurve straightRun = makeStraightRunBSpline();
        const std::vector<CurveSample> straightRunSamples =
            sampleCurveByArcLength(
                straightRun,
                0.0,
                3.0,
                0.25,
                samplingOptions()
            );
        const std::vector<CurveFrame> straightRunFrames =
            buildRotationMinimizingFrames(
                straightRun,
                straightRunSamples,
                Point{0.0, 0.0, 1.0}
            );
        requireValidSequence(
            straightRun,
            straightRunSamples,
            straightRunFrames,
            "long straight run"
        );

        std::size_t runBegin = straightRunFrames.size();
        for (std::size_t index = 0; index < straightRunFrames.size(); ++index)
        {
            if (straightRunFrames[index].tangent == Point{1.0, 0.0, 0.0})
            {
                runBegin = index;
                break;
            }
        }
        require(
            runBegin + 10 < straightRunFrames.size(),
            "test curve must contain a long constant-tangent sample run"
        );

        for (std::size_t index = runBegin + 1;
             index < straightRunFrames.size();
             ++index)
        {
            require(
                pointBitsEqual(
                    straightRunFrames[index].lateral,
                    straightRunFrames[runBegin].lateral
                )
                    && pointBitsEqual(
                        straightRunFrames[index].up,
                        straightRunFrames[runBegin].up
                    ),
                "constant-tangent run must introduce no artificial twist"
            );
        }

        const BSplineCurve repeatedKnot = makeRepeatedKnotBSpline();
        const std::vector<CurveSample> repeatedKnotSamples =
            sampleCurveByArcLength(
                repeatedKnot,
                0.0,
                2.0,
                0.1,
                samplingOptions()
            );
        const std::vector<CurveFrame> repeatedKnotFrames =
            buildRotationMinimizingFrames(
                repeatedKnot,
                repeatedKnotSamples,
                Point{0.0, 0.0, 1.0}
            );
        requireValidSequence(
            repeatedKnot,
            repeatedKnotSamples,
            repeatedKnotFrames,
            "regular repeated-knot samples"
        );
        measurements.repeatedKnotFrameCount = repeatedKnotFrames.size();
    }

    void testInvalidInitialReferencesAndDegenerateTangents()
    {
        const BSplineCurve straight = makeStraightBSpline();
        const std::vector<CurveSample> straightSamples{
            CurveSample{0.0, 0.0, straight.evaluate(0.0)}
        };

        requireThrows<std::invalid_argument>(
            [&]
            {
                static_cast<void>(buildRotationMinimizingFrames(
                    straight,
                    straightSamples,
                    Point{0.0, 0.0, 0.0}
                ));
            },
            "zero initial reference"
        );
        requireThrows<std::invalid_argument>(
            [&]
            {
                static_cast<void>(buildRotationMinimizingFrames(
                    straight,
                    straightSamples,
                    Point{
                        std::numeric_limits<double>::quiet_NaN(),
                        1.0,
                        0.0
                    }
                ));
            },
            "NaN initial reference"
        );
        requireThrows<std::invalid_argument>(
            [&]
            {
                static_cast<void>(buildRotationMinimizingFrames(
                    straight,
                    straightSamples,
                    Point{
                        0.0,
                        std::numeric_limits<double>::infinity(),
                        0.0
                    }
                ));
            },
            "infinite initial reference"
        );
        requireThrows<std::domain_error>(
            [&]
            {
                static_cast<void>(buildRotationMinimizingFrames(
                    straight,
                    straightSamples,
                    Point{2.0, 0.0, 0.0}
                ));
            },
            "parallel initial reference"
        );
        requireThrows<std::domain_error>(
            [&]
            {
                static_cast<void>(buildRotationMinimizingFrames(
                    straight,
                    straightSamples,
                    Point{-2.0, 0.0, 0.0}
                ));
            },
            "antiparallel initial reference"
        );

        const std::vector<CurveFrame> nearParallelFrame =
            buildRotationMinimizingFrames(
                straight,
                straightSamples,
                Point{1.0, 1.0e-12, 0.0}
            );
        requireValidSequence(
            straight,
            straightSamples,
            nearParallelFrame,
            "resolvable near-parallel initial reference"
        );

        const BSplineCurve stationary = makeStationaryPointBSpline();
        const std::vector<CurveSample> stationarySamples =
            sampleCurveByArcLength(
                stationary,
                0.0,
                1.0,
                0.25
            );
        require(stationarySamples.size() == 5, "stationary fixture samples");
        requireThrows<std::domain_error>(
            [&]
            {
                static_cast<void>(buildRotationMinimizingFrames(
                    stationary,
                    stationarySamples,
                    Point{0.0, 1.0, 0.0}
                ));
            },
            "stationary canonical sample"
        );

        const std::vector<CurveSample> antiparallelSamples{
            stationarySamples.front(),
            stationarySamples.back()
        };
        requireThrows<std::domain_error>(
            [&]
            {
                static_cast<void>(buildRotationMinimizingFrames(
                    stationary,
                    antiparallelSamples,
                    Point{0.0, 1.0, 0.0}
                ));
            },
            "exact antiparallel tangent transition"
        );

        struct NearlyAntiparallelCurve
        {
            [[nodiscard]] Point evaluateFirstDerivative(
                const double parameter
            ) const
            {
                return parameter == 0.0
                    ? Point{1.0, 0.0, 0.0}
                    : Point{-1.0, 1.0e-9, 0.0};
            }
        };
        const std::vector<CurveSample> unresolvedSamples{
            {0.0, 0.0, Point{0.0}},
            {1.0, 1.0, Point{0.0}}
        };
        requireThrows<std::domain_error>(
            [&]
            {
                static_cast<void>(buildRotationMinimizingFrames(
                    NearlyAntiparallelCurve{},
                    unresolvedSamples,
                    Point{0.0, 1.0, 0.0}
                ));
            },
            "numerically unresolved antiparallel tangent transition"
        );
    }

    void testDeterminism()
    {
        const BSplineCurve curve = makeSpatialBSpline();
        const std::vector<CurveSample> samples = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            0.1,
            samplingOptions()
        );
        const std::vector<CurveFrame> first =
            buildRotationMinimizingFrames(
                curve,
                samples,
                Point{0.0, 1.0, 0.0}
            );
        const std::vector<CurveFrame> second =
            buildRotationMinimizingFrames(
                curve,
                samples,
                Point{0.0, 1.0, 0.0}
            );
        require(first.size() == second.size(), "deterministic frame count");

        for (std::size_t index = 0; index < first.size(); ++index)
        {
            require(
                pointBitsEqual(first[index].tangent, second[index].tangent)
                    && pointBitsEqual(
                        first[index].lateral,
                        second[index].lateral
                    )
                    && pointBitsEqual(first[index].up, second[index].up),
                "repeated RMF generation must be bitwise deterministic"
            );
        }
    }

    void testSamplingDensityAgreement()
    {
        const BSplineCurve curve = makeSpatialBSpline();
        const ArcLengthLUTOptions options = samplingOptions();
        const std::vector<CurveSample> coarseSamples = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            0.2,
            options
        );
        const std::vector<CurveSample> fineSamples = sampleCurveByArcLength(
            curve,
            0.0,
            1.0,
            0.1,
            options
        );
        const std::vector<CurveFrame> coarseFrames =
            buildRotationMinimizingFrames(
                curve,
                coarseSamples,
                Point{0.0, 1.0, 0.0}
            );
        const std::vector<CurveFrame> fineFrames =
            buildRotationMinimizingFrames(
                curve,
                fineSamples,
                Point{0.0, 1.0, 0.0}
            );
        requireValidSequence(
            curve,
            coarseSamples,
            coarseFrames,
            "coarse sampling-density sequence"
        );
        requireValidSequence(
            curve,
            fineSamples,
            fineFrames,
            "fine sampling-density sequence"
        );

        for (std::size_t coarseIndex = 0;
             coarseIndex + 1 < coarseFrames.size();
             ++coarseIndex)
        {
            const std::size_t fineIndex = 2 * coarseIndex;
            require(
                fineIndex < fineFrames.size()
                    && coarseSamples[coarseIndex].distance
                        == fineSamples[fineIndex].distance,
                "sampling-density fixtures must share interior distances"
            );
            measurements.maximumDensityOrientationDifference = std::max({
                measurements.maximumDensityOrientationDifference,
                angularDifference(
                    coarseFrames[coarseIndex].lateral,
                    fineFrames[fineIndex].lateral
                ),
                angularDifference(
                    coarseFrames[coarseIndex].up,
                    fineFrames[fineIndex].up
                )
            });
        }

        measurements.maximumDensityOrientationDifference = std::max({
            measurements.maximumDensityOrientationDifference,
            angularDifference(
                coarseFrames.back().lateral,
                fineFrames.back().lateral
            ),
            angularDifference(coarseFrames.back().up, fineFrames.back().up)
        });

        const auto correspondingOrientationDifference =
            [&](const std::size_t coarseIndex, const std::size_t fineIndex)
            {
                return std::max(
                    angularDifference(
                        coarseFrames[coarseIndex].lateral,
                        fineFrames[fineIndex].lateral
                    ),
                    angularDifference(
                        coarseFrames[coarseIndex].up,
                        fineFrames[fineIndex].up
                    )
                );
            };
        const std::size_t middleCoarseIndex =
            (coarseFrames.size() - 1) / 2;
        measurements.beginningDensityOrientationDifference =
            correspondingOrientationDifference(0, 0);
        measurements.middleDensityOrientationDifference =
            correspondingOrientationDifference(
                middleCoarseIndex,
                2 * middleCoarseIndex
            );
        measurements.endpointDensityOrientationDifference =
            correspondingOrientationDifference(
                coarseFrames.size() - 1,
                fineFrames.size() - 1
            );

        require(
            measurements.maximumDensityOrientationDifference < 0.01,
            "coarse and fine RMFs must closely agree at corresponding locations"
        );
    }

    void testRollZeroQuarterAndHalfTurns()
    {
        const CurveFrame frame = controlledFrame();
        const CurveFrame zero = applyRoll(frame, 0.0);
        requireValidRolledFrame(frame, zero, "zero roll");
        require(
            frameBitsEqual(zero, frame),
            "zero roll must preserve every frame component bit-for-bit"
        );

        const auto requireControlledTurn =
            [&](const double angle,
                const CurveFrame& expected,
                const std::string_view context)
            {
                const CurveFrame rolled = applyRoll(frame, angle);
                requireValidRolledFrame(frame, rolled, context);
                requireRollReference(
                    rolled,
                    expected,
                    frameTolerance,
                    context
                );
            };

        requireControlledTurn(
            std::numbers::pi / 2.0,
            {
                Point{1.0, 0.0, 0.0},
                Point{0.0, 1.0, 0.0},
                Point{0.0, 0.0, 1.0}
            },
            "positive quarter turn"
        );
        requireControlledTurn(
            -std::numbers::pi / 2.0,
            {
                Point{1.0, 0.0, 0.0},
                Point{0.0, -1.0, 0.0},
                Point{0.0, 0.0, -1.0}
            },
            "negative quarter turn"
        );
        requireControlledTurn(
            std::numbers::pi,
            {
                Point{1.0, 0.0, 0.0},
                Point{0.0, 0.0, 1.0},
                Point{0.0, -1.0, 0.0}
            },
            "half turn"
        );
    }

    void testRollArbitraryAnglesCompositionAndPeriodicity()
    {
        const CurveFrame frame = controlledFrame();
        constexpr std::array angles{
            std::numbers::pi / 6.0,
            std::numbers::pi / 4.0,
            -std::numbers::pi / 3.0
        };

        for (const double angle : angles)
        {
            const CurveFrame rolled = applyRoll(frame, angle);
            requireValidRolledFrame(frame, rolled, "arbitrary roll");
            requireRollReference(
                rolled,
                controlledRollReference(angle),
                frameTolerance,
                "arbitrary roll"
            );
        }

        for (const double angle : {
                 2.0 * std::numbers::pi,
                 -2.0 * std::numbers::pi
             })
        {
            const CurveFrame rolled = applyRoll(frame, angle);
            requireValidRolledFrame(frame, rolled, "full revolution");
            const double periodicityError = std::max({
                pointError(rolled.tangent, frame.tangent),
                pointError(rolled.lateral, frame.lateral),
                pointError(rolled.up, frame.up)
            });
            measurements.maximumRollPeriodicityError = std::max(
                measurements.maximumRollPeriodicityError,
                periodicityError
            );
            requireNear(
                periodicityError,
                0.0,
                frameTolerance,
                "full-revolution periodicity"
            );
        }

        constexpr double largeAngle =
            1'000'000.0 + std::numbers::pi / 7.0;
        const CurveFrame largeAngleRoll = applyRoll(frame, largeAngle);
        requireValidRolledFrame(frame, largeAngleRoll, "large finite roll");
        const CurveFrame largeAngleReference =
            controlledRollReference(largeAngle);
        measurements.largeAngleReferenceError = std::max({
            pointError(largeAngleRoll.tangent, largeAngleReference.tangent),
            pointError(largeAngleRoll.lateral, largeAngleReference.lateral),
            pointError(largeAngleRoll.up, largeAngleReference.up)
        });
        requireRollReference(
            largeAngleRoll,
            largeAngleReference,
            frameTolerance,
            "large finite roll"
        );

        constexpr double firstAngle = 0.37;
        constexpr double secondAngle = -1.11;
        const CurveFrame firstRoll = applyRoll(frame, firstAngle);
        const CurveFrame composed = applyRoll(firstRoll, secondAngle);
        const CurveFrame direct = applyRoll(
            frame,
            firstAngle + secondAngle
        );
        requireValidRolledFrame(frame, firstRoll, "first composed roll");
        requireValidRolledFrame(firstRoll, composed, "second composed roll");
        requireValidRolledFrame(frame, direct, "direct composed roll");
        measurements.maximumRollCompositionError = std::max({
            pointError(composed.tangent, direct.tangent),
            pointError(composed.lateral, direct.lateral),
            pointError(composed.up, direct.up)
        });
        requireNear(
            measurements.maximumRollCompositionError,
            0.0,
            frameTolerance,
            "roll composition"
        );

        const CurveFrame deterministicFirst = applyRoll(frame, 0.731);
        const CurveFrame deterministicSecond = applyRoll(frame, 0.731);
        requireValidRolledFrame(
            frame,
            deterministicFirst,
            "first deterministic roll"
        );
        requireValidRolledFrame(
            frame,
            deterministicSecond,
            "second deterministic roll"
        );
        require(
            frameBitsEqual(deterministicFirst, deterministicSecond),
            "repeated roll evaluation must be bitwise deterministic"
        );
    }

    void testRollRejectsInvalidInputs()
    {
        const CurveFrame frame = controlledFrame();
        for (const double angle : {
                 std::numeric_limits<double>::quiet_NaN(),
                 std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity()
             })
        {
            requireThrows<std::invalid_argument>(
                [&]
                {
                    static_cast<void>(applyRoll(frame, angle));
                },
                "non-finite roll angle"
            );
        }

        const auto requireInvalidFrame =
            [&](const CurveFrame& invalid, const std::string_view context)
            {
                requireThrows<std::invalid_argument>(
                    [&]
                    {
                        static_cast<void>(applyRoll(invalid, 0.25));
                    },
                    context
                );
            };

        CurveFrame invalid = frame;
        invalid.tangent.x = std::numeric_limits<double>::quiet_NaN();
        requireInvalidFrame(invalid, "non-finite tangent");
        invalid = frame;
        invalid.lateral.y = std::numeric_limits<double>::infinity();
        requireInvalidFrame(invalid, "non-finite lateral");
        invalid = frame;
        invalid.up.z = -std::numeric_limits<double>::infinity();
        requireInvalidFrame(invalid, "non-finite up");

        invalid = frame;
        invalid.tangent = Point{0.0};
        requireInvalidFrame(invalid, "zero tangent");
        invalid = frame;
        invalid.lateral = Point{0.0};
        requireInvalidFrame(invalid, "zero lateral");
        invalid = frame;
        invalid.up = Point{0.0};
        requireInvalidFrame(invalid, "zero up");

        invalid = frame;
        invalid.tangent *= 1.001;
        requireInvalidFrame(invalid, "non-unit tangent");
        invalid = frame;
        invalid.lateral *= 0.999;
        requireInvalidFrame(invalid, "non-unit lateral");
        invalid = frame;
        invalid.up *= 1.001;
        requireInvalidFrame(invalid, "non-unit up");

        invalid = frame;
        invalid.lateral = glm::normalize(Point{0.1, 0.0, -1.0});
        requireInvalidFrame(invalid, "tangent-lateral non-orthogonality");
        invalid = frame;
        invalid.up = glm::normalize(Point{0.1, 1.0, 0.0});
        requireInvalidFrame(invalid, "tangent-up non-orthogonality");
        invalid = frame;
        invalid.up = glm::normalize(Point{0.0, 1.0, -0.1});
        requireInvalidFrame(invalid, "lateral-up non-orthogonality");

        invalid = frame;
        invalid.up = -invalid.up;
        requireInvalidFrame(invalid, "incorrect frame handedness");
    }

    void testLocalPitchZeroQuarterHalfAndArbitraryAngles()
    {
        const CurveFrame frame = controlledFrame();
        const CurveFrame zero = applyLocalPitch(frame, 0.0);
        requireValidLocalRotationFrame(zero, "zero local pitch");
        requirePreservedLocalRotationAxis(
            zero.lateral,
            frame.lateral,
            "zero local pitch"
        );
        require(
            frameBitsEqual(zero, frame),
            "zero local pitch must preserve every frame component bit-for-bit"
        );

        const auto requireControlledPitch =
            [&](const double angle,
                const CurveFrame& expected,
                const std::string_view context)
            {
                const CurveFrame pitched = applyLocalPitch(frame, angle);
                requireValidLocalRotationFrame(pitched, context);
                requirePreservedLocalRotationAxis(
                    pitched.lateral,
                    frame.lateral,
                    context
                );
                requireLocalRotationReference(
                    pitched,
                    expected,
                    context,
                    measurements.maximumLocalRotationReferenceError
                );
            };

        // With T=+X, L=-Z, U=+Y, positive pitch about +L moves
        // forward toward -up. Negative pitch moves forward toward +up.
        requireControlledPitch(
            std::numbers::pi / 2.0,
            {
                Point{0.0, -1.0, 0.0},
                Point{0.0, 0.0, -1.0},
                Point{1.0, 0.0, 0.0}
            },
            "positive local-pitch quarter turn"
        );
        requireControlledPitch(
            -std::numbers::pi / 2.0,
            {
                Point{0.0, 1.0, 0.0},
                Point{0.0, 0.0, -1.0},
                Point{-1.0, 0.0, 0.0}
            },
            "negative local-pitch quarter turn"
        );
        requireControlledPitch(
            std::numbers::pi,
            {
                Point{-1.0, 0.0, 0.0},
                Point{0.0, 0.0, -1.0},
                Point{0.0, -1.0, 0.0}
            },
            "local-pitch half turn"
        );

        constexpr std::array angles{
            std::numbers::pi / 6.0,
            std::numbers::pi / 4.0,
            -std::numbers::pi / 3.0
        };
        for (const double angle : angles)
        {
            requireControlledPitch(
                angle,
                localPitchReference(frame, angle),
                "arbitrary local pitch"
            );
        }

        const CurveFrame deterministicFirst =
            applyLocalPitch(frame, 0.731);
        const CurveFrame deterministicSecond =
            applyLocalPitch(frame, 0.731);
        requireValidLocalRotationFrame(
            deterministicFirst,
            "first deterministic local pitch"
        );
        requireValidLocalRotationFrame(
            deterministicSecond,
            "second deterministic local pitch"
        );
        require(
            frameBitsEqual(deterministicFirst, deterministicSecond),
            "repeated local-pitch evaluation must be bitwise deterministic"
        );
    }

    void testLocalYawZeroQuarterHalfAndArbitraryAngles()
    {
        const CurveFrame frame = controlledFrame();
        const CurveFrame zero = applyLocalYaw(frame, 0.0);
        requireValidLocalRotationFrame(zero, "zero local yaw");
        requirePreservedLocalRotationAxis(
            zero.up,
            frame.up,
            "zero local yaw"
        );
        require(
            frameBitsEqual(zero, frame),
            "zero local yaw must preserve every frame component bit-for-bit"
        );

        const auto requireControlledYaw =
            [&](const double angle,
                const CurveFrame& expected,
                const std::string_view context)
            {
                const CurveFrame yawed = applyLocalYaw(frame, angle);
                requireValidLocalRotationFrame(yawed, context);
                requirePreservedLocalRotationAxis(
                    yawed.up,
                    frame.up,
                    context
                );
                requireLocalRotationReference(
                    yawed,
                    expected,
                    context,
                    measurements.maximumLocalRotationReferenceError
                );
            };

        // With T=+X, L=-Z, U=+Y, positive yaw about +U moves
        // forward toward lateral (-Z). Negative yaw moves it toward +Z.
        requireControlledYaw(
            std::numbers::pi / 2.0,
            {
                Point{0.0, 0.0, -1.0},
                Point{-1.0, 0.0, 0.0},
                Point{0.0, 1.0, 0.0}
            },
            "positive local-yaw quarter turn"
        );
        requireControlledYaw(
            -std::numbers::pi / 2.0,
            {
                Point{0.0, 0.0, 1.0},
                Point{1.0, 0.0, 0.0},
                Point{0.0, 1.0, 0.0}
            },
            "negative local-yaw quarter turn"
        );
        requireControlledYaw(
            std::numbers::pi,
            {
                Point{-1.0, 0.0, 0.0},
                Point{0.0, 0.0, 1.0},
                Point{0.0, 1.0, 0.0}
            },
            "local-yaw half turn"
        );

        constexpr std::array angles{
            std::numbers::pi / 6.0,
            std::numbers::pi / 4.0,
            -std::numbers::pi / 3.0
        };
        for (const double angle : angles)
        {
            requireControlledYaw(
                angle,
                localYawReference(frame, angle),
                "arbitrary local yaw"
            );
        }

        const CurveFrame deterministicFirst = applyLocalYaw(frame, -0.417);
        const CurveFrame deterministicSecond = applyLocalYaw(frame, -0.417);
        requireValidLocalRotationFrame(
            deterministicFirst,
            "first deterministic local yaw"
        );
        requireValidLocalRotationFrame(
            deterministicSecond,
            "second deterministic local yaw"
        );
        require(
            frameBitsEqual(deterministicFirst, deterministicSecond),
            "repeated local-yaw evaluation must be bitwise deterministic"
        );
    }

    void testLocalRotationsRejectInvalidInputs()
    {
        const CurveFrame frame = controlledFrame();

        const auto requireOperationRejectsInvalidInputs =
            [&](const auto operation, const std::string_view operationName)
            {
                for (const double angle : {
                         std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity()
                     })
                {
                    requireThrows<std::invalid_argument>(
                        [&]
                        {
                            static_cast<void>(operation(frame, angle));
                        },
                        std::string(operationName) + " non-finite angle"
                    );
                }

                const auto requireInvalidFrame =
                    [&](const CurveFrame& invalid,
                        const std::string_view reason)
                    {
                        requireThrows<std::invalid_argument>(
                            [&]
                            {
                                static_cast<void>(operation(invalid, 0.25));
                            },
                            std::string(operationName) + " "
                                + std::string(reason)
                        );
                    };

                CurveFrame invalid = frame;
                invalid.tangent.x =
                    std::numeric_limits<double>::quiet_NaN();
                requireInvalidFrame(invalid, "non-finite tangent");
                invalid = frame;
                invalid.lateral.y = std::numeric_limits<double>::infinity();
                requireInvalidFrame(invalid, "non-finite lateral");
                invalid = frame;
                invalid.up.z = -std::numeric_limits<double>::infinity();
                requireInvalidFrame(invalid, "non-finite up");

                invalid = frame;
                invalid.tangent = Point{0.0};
                requireInvalidFrame(invalid, "zero tangent");
                invalid = frame;
                invalid.lateral = Point{0.0};
                requireInvalidFrame(invalid, "zero lateral");
                invalid = frame;
                invalid.up = Point{0.0};
                requireInvalidFrame(invalid, "zero up");

                invalid = frame;
                invalid.tangent *= 1.001;
                requireInvalidFrame(invalid, "non-unit tangent");
                invalid = frame;
                invalid.lateral *= 0.999;
                requireInvalidFrame(invalid, "non-unit lateral");
                invalid = frame;
                invalid.up *= 1.001;
                requireInvalidFrame(invalid, "non-unit up");

                invalid = frame;
                invalid.lateral = glm::normalize(Point{0.1, 0.0, -1.0});
                requireInvalidFrame(
                    invalid,
                    "tangent-lateral non-orthogonality"
                );
                invalid = frame;
                invalid.up = glm::normalize(Point{0.1, 1.0, 0.0});
                requireInvalidFrame(invalid, "tangent-up non-orthogonality");
                invalid = frame;
                invalid.up = glm::normalize(Point{0.0, 1.0, -0.1});
                requireInvalidFrame(invalid, "lateral-up non-orthogonality");

                invalid = frame;
                invalid.up = -invalid.up;
                requireInvalidFrame(invalid, "incorrect handedness");
            };

        requireOperationRejectsInvalidInputs(
            [](const CurveFrame& input, const double angle)
            {
                return applyLocalPitch(input, angle);
            },
            "local pitch"
        );
        requireOperationRejectsInvalidInputs(
            [](const CurveFrame& input, const double angle)
            {
                return applyLocalYaw(input, angle);
            },
            "local yaw"
        );
    }

    void testRolledLocalAxesAndRotationOrder()
    {
        const CurveFrame frame = controlledFrame();
        const CurveFrame rolled = applyRoll(
            frame,
            std::numbers::pi / 5.0
        );
        requireValidRolledFrame(frame, rolled, "rolled local-axis input");
        require(
            pointError(rolled.lateral, frame.lateral) > 0.1
                && pointError(rolled.up, frame.up) > 0.1,
            "the rolled fixture must differ from the original world-aligned transverse axes"
        );

        const double pitchAngle = std::numbers::pi / 4.0;
        const CurveFrame pitched = applyLocalPitch(rolled, pitchAngle);
        requireValidLocalRotationFrame(pitched, "pitch after roll");
        requirePreservedLocalRotationAxis(
            pitched.lateral,
            rolled.lateral,
            "pitch after roll"
        );
        requireLocalRotationReference(
            pitched,
            localPitchReference(rolled, pitchAngle),
            "pitch after roll",
            measurements.maximumRolledLocalRotationReferenceError
        );
        require(
            pointError(pitched.lateral, frame.lateral) > 0.1,
            "pitch after roll must preserve the rolled lateral axis, not the original axis"
        );

        const double yawAngle = -std::numbers::pi / 3.0;
        const CurveFrame yawed = applyLocalYaw(rolled, yawAngle);
        requireValidLocalRotationFrame(yawed, "yaw after roll");
        requirePreservedLocalRotationAxis(
            yawed.up,
            rolled.up,
            "yaw after roll"
        );
        requireLocalRotationReference(
            yawed,
            localYawReference(rolled, yawAngle),
            "yaw after roll",
            measurements.maximumRolledLocalRotationReferenceError
        );
        require(
            pointError(yawed.up, frame.up) > 0.1,
            "yaw after roll must preserve the rolled up axis, not world up"
        );

        const CurveFrame pitchedThenYawed =
            applyLocalYaw(pitched, yawAngle);
        requireValidLocalRotationFrame(
            pitchedThenYawed,
            "sequential pitch and yaw after roll"
        );
        requirePreservedLocalRotationAxis(
            pitchedThenYawed.up,
            pitched.up,
            "sequential yaw after rolled-frame pitch"
        );
        requireLocalRotationReference(
            pitchedThenYawed,
            localYawReference(pitched, yawAngle),
            "sequential pitch and yaw after roll",
            measurements.maximumRolledLocalRotationReferenceError
        );

        constexpr double orderPitch = std::numbers::pi / 4.0;
        constexpr double orderYaw = std::numbers::pi / 6.0;
        const CurveFrame pitchThenYaw = applyLocalYaw(
            applyLocalPitch(frame, orderPitch),
            orderYaw
        );
        const CurveFrame yawThenPitch = applyLocalPitch(
            applyLocalYaw(frame, orderYaw),
            orderPitch
        );
        requireValidLocalRotationFrame(
            pitchThenYaw,
            "pitch-then-yaw order result"
        );
        requireValidLocalRotationFrame(
            yawThenPitch,
            "yaw-then-pitch order result"
        );
        measurements.localRotationOrderDifference = std::max({
            pointError(pitchThenYaw.tangent, yawThenPitch.tangent),
            pointError(pitchThenYaw.lateral, yawThenPitch.lateral),
            pointError(pitchThenYaw.up, yawThenPitch.up)
        });
        require(
            measurements.localRotationOrderDifference > 0.1,
            "nonzero local pitch and yaw must demonstrate order dependence"
        );
    }

    template<typename Curve>
    [[nodiscard]] double requireConstantRollOnRmfSequence(
        const Curve& curve,
        const std::vector<CurveSample>& samples,
        const std::vector<CurveFrame>& frames,
        const double rollRadians,
        const std::string_view context
    )
    {
        requireValidSequence(curve, samples, frames, context);
        double maximumRelativeRollError = 0.0;
        const double cosine = std::cos(rollRadians);
        const double sine = std::sin(rollRadians);

        for (std::size_t index = 0; index < frames.size(); ++index)
        {
            const CurveFrame& frame = frames[index];
            const CurveFrame rolled = applyRoll(frame, rollRadians);
            const std::string frameContext =
                std::string(context) + " rolled frame "
                + std::to_string(index);
            requireValidRolledFrame(frame, rolled, frameContext);

            const CurveFrame expected{
                frame.tangent,
                cosine * frame.lateral + sine * frame.up,
                cosine * frame.up - sine * frame.lateral
            };
            requireRollReference(
                rolled,
                expected,
                frameTolerance,
                frameContext
            );

            const double observedRoll = std::atan2(
                glm::dot(rolled.lateral, frame.up),
                glm::dot(rolled.lateral, frame.lateral)
            );
            maximumRelativeRollError = std::max(
                maximumRelativeRollError,
                std::abs(observedRoll - rollRadians)
            );
            require(
                pointError(rolled.lateral, frame.lateral) > 0.1,
                frameContext
                    + " must retain the authored roll instead of rebuilding from world up"
            );
        }

        requireNear(
            maximumRelativeRollError,
            0.0,
            frameTolerance,
            std::string(context) + " constant relative roll"
        );
        return maximumRelativeRollError;
    }

    void testRollOnPlanarAndSpatialRmfOutput()
    {
        const NurbsCurve planarCurve = makeQuarterCircle();
        const std::vector<CurveSample> planarSamples =
            sampleCurveByArcLength(
                planarCurve,
                0.0,
                1.0,
                std::numbers::pi / 32.0,
                samplingOptions()
            );
        const std::vector<CurveFrame> planarFrames =
            buildRotationMinimizingFrames(
                planarCurve,
                planarSamples,
                Point{0.0, 0.0, 1.0}
            );
        measurements.maximumPlanarRelativeRollError =
            requireConstantRollOnRmfSequence(
                planarCurve,
                planarSamples,
                planarFrames,
                std::numbers::pi / 6.0,
                "planar RMF"
            );
        measurements.planarRolledFrameCount = planarFrames.size();

        const BSplineCurve spatialCurve = makeSpatialBSpline();
        const std::vector<CurveSample> spatialSamples =
            sampleCurveByArcLength(
                spatialCurve,
                0.0,
                1.0,
                0.08,
                samplingOptions()
            );
        const std::vector<CurveFrame> spatialFrames =
            buildRotationMinimizingFrames(
                spatialCurve,
                spatialSamples,
                Point{0.0, 1.0, 0.0}
            );
        measurements.maximumSpatialRelativeRollError =
            requireConstantRollOnRmfSequence(
                spatialCurve,
                spatialSamples,
                spatialFrames,
                -std::numbers::pi / 5.0,
                "spatial RMF"
            );
        measurements.spatialRolledFrameCount = spatialFrames.size();
    }

    template<typename Curve>
    void requireLocalRotationsOnRmfSequence(
        const Curve& curve,
        const std::vector<CurveSample>& samples,
        const std::vector<CurveFrame>& frames,
        const std::string_view context
    )
    {
        requireValidSequence(curve, samples, frames, context);

        constexpr double pitchAngle = std::numbers::pi / 6.0;
        constexpr double yawAngle = -std::numbers::pi / 4.0;
        for (std::size_t index = 0; index < frames.size(); ++index)
        {
            const CurveFrame& frame = frames[index];
            const std::string frameContext =
                std::string(context) + " frame " + std::to_string(index);

            const CurveFrame pitched = applyLocalPitch(frame, pitchAngle);
            requireValidLocalRotationFrame(
                pitched,
                frameContext + " local pitch"
            );
            requirePreservedLocalRotationAxis(
                pitched.lateral,
                frame.lateral,
                frameContext + " local pitch"
            );
            requireLocalRotationReference(
                pitched,
                localPitchReference(frame, pitchAngle),
                frameContext + " local pitch",
                measurements.maximumLocalRotationReferenceError
            );

            const CurveFrame yawed = applyLocalYaw(frame, yawAngle);
            requireValidLocalRotationFrame(
                yawed,
                frameContext + " local yaw"
            );
            requirePreservedLocalRotationAxis(
                yawed.up,
                frame.up,
                frameContext + " local yaw"
            );
            requireLocalRotationReference(
                yawed,
                localYawReference(frame, yawAngle),
                frameContext + " local yaw",
                measurements.maximumLocalRotationReferenceError
            );
        }
    }

    void testLocalRotationsOnPlanarAndSpatialRmfOutput()
    {
        const NurbsCurve planarCurve = makeQuarterCircle();
        const std::vector<CurveSample> planarSamples =
            sampleCurveByArcLength(
                planarCurve,
                0.0,
                1.0,
                std::numbers::pi / 32.0,
                samplingOptions()
            );
        const std::vector<CurveFrame> planarFrames =
            buildRotationMinimizingFrames(
                planarCurve,
                planarSamples,
                Point{0.0, 0.0, 1.0}
            );
        requireLocalRotationsOnRmfSequence(
            planarCurve,
            planarSamples,
            planarFrames,
            "planar RMF local rotations"
        );
        measurements.planarLocalRotationFrameCount = planarFrames.size();

        const BSplineCurve spatialCurve = makeSpatialBSpline();
        const std::vector<CurveSample> spatialSamples =
            sampleCurveByArcLength(
                spatialCurve,
                0.0,
                1.0,
                0.08,
                samplingOptions()
            );
        const std::vector<CurveFrame> spatialFrames =
            buildRotationMinimizingFrames(
                spatialCurve,
                spatialSamples,
                Point{0.0, 1.0, 0.0}
            );
        requireLocalRotationsOnRmfSequence(
            spatialCurve,
            spatialSamples,
            spatialFrames,
            "spatial RMF local rotations"
        );
        measurements.spatialLocalRotationFrameCount = spatialFrames.size();

        require(
            !spatialFrames.empty(),
            "spatial RMF local-rotation fixture must not be empty"
        );
        const CurveFrame rolled = applyRoll(
            spatialFrames[spatialFrames.size() / 2],
            std::numbers::pi / 5.0
        );
        const CurveFrame pitched = applyLocalPitch(
            rolled,
            -std::numbers::pi / 3.0
        );
        requireValidLocalRotationFrame(
            pitched,
            "local pitch on rolled spatial RMF"
        );
        requirePreservedLocalRotationAxis(
            pitched.lateral,
            rolled.lateral,
            "local pitch on rolled spatial RMF"
        );
        requireLocalRotationReference(
            pitched,
            localPitchReference(rolled, -std::numbers::pi / 3.0),
            "local pitch on rolled spatial RMF",
            measurements.maximumRolledLocalRotationReferenceError
        );

        const CurveFrame yawed = applyLocalYaw(
            pitched,
            std::numbers::pi / 4.0
        );
        requireValidLocalRotationFrame(
            yawed,
            "local yaw after pitch on rolled spatial RMF"
        );
        requirePreservedLocalRotationAxis(
            yawed.up,
            pitched.up,
            "local yaw after pitch on rolled spatial RMF"
        );
        requireLocalRotationReference(
            yawed,
            localYawReference(pitched, std::numbers::pi / 4.0),
            "local yaw after pitch on rolled spatial RMF",
            measurements.maximumRolledLocalRotationReferenceError
        );

        const CurveFrame deterministicPitchFirst =
            applyLocalPitch(rolled, 0.371);
        const CurveFrame deterministicPitchSecond =
            applyLocalPitch(rolled, 0.371);
        requireValidLocalRotationFrame(
            deterministicPitchFirst,
            "first deterministic rolled-RMF pitch"
        );
        requireValidLocalRotationFrame(
            deterministicPitchSecond,
            "second deterministic rolled-RMF pitch"
        );
        require(
            frameBitsEqual(
                deterministicPitchFirst,
                deterministicPitchSecond
            ),
            "local pitch on real rolled RMF output must be bitwise deterministic"
        );

        const CurveFrame deterministicYawFirst =
            applyLocalYaw(rolled, -0.619);
        const CurveFrame deterministicYawSecond =
            applyLocalYaw(rolled, -0.619);
        requireValidLocalRotationFrame(
            deterministicYawFirst,
            "first deterministic rolled-RMF yaw"
        );
        requireValidLocalRotationFrame(
            deterministicYawSecond,
            "second deterministic rolled-RMF yaw"
        );
        require(
            frameBitsEqual(deterministicYawFirst, deterministicYawSecond),
            "local yaw on real rolled RMF output must be bitwise deterministic"
        );
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;

    const std::vector<Test> tests{
        {
            "frame representation, empty input, and single sample",
            testRepresentationEmptyAndSingleSample
        },
        {
            "straight B-spline and +Y initial projection",
            testStraightBSplineAndWorldUpProjection
        },
        {"horizontal rational quarter circle", testQuarterCircle},
        {"genuine 3D curve", testSpatialCurve},
        {"uniform positive scale invariance", testUniformScaleInvariance},
        {"initial roll dependence", testInitialRollDependence},
        {
            "long straight run and regular repeated knot",
            testStraightRunAndRepeatedKnot
        },
        {
            "invalid references and degenerate tangents",
            testInvalidInitialReferencesAndDegenerateTangents
        },
        {"deterministic generation", testDeterminism},
        {"sampling-density agreement", testSamplingDensityAgreement},
        {
            "roll zero, quarter turns, and half turn",
            testRollZeroQuarterAndHalfTurns
        },
        {
            "roll arbitrary angles, composition, and periodicity",
            testRollArbitraryAnglesCompositionAndPeriodicity
        },
        {"roll invalid inputs", testRollRejectsInvalidInputs},
        {"roll on planar and spatial RMFs", testRollOnPlanarAndSpatialRmfOutput},
        {
            "local pitch zero, quarter, half, and arbitrary turns",
            testLocalPitchZeroQuarterHalfAndArbitraryAngles
        },
        {
            "local yaw zero, quarter, half, and arbitrary turns",
            testLocalYawZeroQuarterHalfAndArbitraryAngles
        },
        {"local pitch and yaw invalid inputs", testLocalRotationsRejectInvalidInputs},
        {
            "rolled local axes and pitch-yaw order",
            testRolledLocalAxesAndRotationOrder
        },
        {
            "local rotations on planar and spatial RMFs",
            testLocalRotationsOnPlanarAndSpatialRmfOutput
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
            std::cerr << "[FAIL] " << name << ": unknown exception\n";
        }
    }

    if (failures != 0)
    {
        std::cerr << failures << " test group(s) failed.\n";
        return 1;
    }

    std::cout.precision(17);
    std::cout << "[METRIC] maximum orthonormality/handedness error: "
              << measurements.maximumOrthonormalityError << '\n';
    std::cout << "[METRIC] maximum analytic tangent agreement error: "
              << measurements.maximumTangentAgreementError << '\n';
    std::cout << "[METRIC] maximum coarse/fine orientation difference (rad): "
              << measurements.maximumDensityOrientationDifference << '\n';
    std::cout << "[METRIC] coarse/fine orientation difference (rad): begin="
              << measurements.beginningDensityOrientationDifference
              << ", middle="
              << measurements.middleDensityOrientationDifference
              << ", end="
              << measurements.endpointDensityOrientationDifference << '\n';
    std::cout << "[METRIC] frame counts: straight="
              << measurements.straightFrameCount
              << ", quarter-circle="
              << measurements.quarterCircleFrameCount
              << ", spatial="
              << measurements.spatialFrameCount
              << ", repeated-knot="
              << measurements.repeatedKnotFrameCount << '\n';
    std::cout << "[METRIC] roll maximum tangent-change error: "
              << measurements.maximumRollTangentChangeError << '\n';
    std::cout << "[METRIC] roll maximum orthonormality/handedness error: "
              << measurements.maximumRollOrthonormalityError << '\n';
    std::cout << "[METRIC] roll maximum analytic-reference error: "
              << measurements.maximumRollReferenceError << '\n';
    std::cout << "[METRIC] roll composition error: "
              << measurements.maximumRollCompositionError << '\n';
    std::cout << "[METRIC] roll full-revolution error: "
              << measurements.maximumRollPeriodicityError << '\n';
    std::cout << "[METRIC] roll large-finite-angle reference error: "
              << measurements.largeAngleReferenceError << '\n';
    std::cout << "[METRIC] constant relative-roll error: planar="
              << measurements.maximumPlanarRelativeRollError
              << ", spatial="
              << measurements.maximumSpatialRelativeRollError << '\n';
    std::cout << "[METRIC] rolled RMF counts: planar="
              << measurements.planarRolledFrameCount
              << ", spatial="
              << measurements.spatialRolledFrameCount << '\n';
    std::cout << "[METRIC] local-rotation maximum preserved-axis error: "
              << measurements.maximumLocalRotationAxisChangeError << '\n';
    std::cout
        << "[METRIC] local-rotation maximum orthonormality/handedness error: "
        << measurements.maximumLocalRotationOrthonormalityError << '\n';
    std::cout << "[METRIC] local-rotation maximum analytic-reference error: "
              << measurements.maximumLocalRotationReferenceError << '\n';
    std::cout << "[METRIC] rolled local-rotation reference error: "
              << measurements.maximumRolledLocalRotationReferenceError
              << '\n';
    std::cout << "[METRIC] pitch/yaw order difference: "
              << measurements.localRotationOrderDifference << '\n';
    std::cout << "[METRIC] local-rotation RMF counts: planar="
              << measurements.planarLocalRotationFrameCount
              << ", spatial="
              << measurements.spatialLocalRotationFrameCount << '\n';
    std::cout << tests.size() << " test groups passed.\n";
    return 0;
}
