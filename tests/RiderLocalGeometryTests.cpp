#include <quantum/coaster/RiderLocalGeometry.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
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
    using quantum::coaster::integrateConstantLocalPitchRate;
    using quantum::coaster::RiderLocalGeometryState;
    using quantum::geometry::applyLocalPitch;
    using quantum::geometry::applyRoll;
    using quantum::geometry::CurveFrame;

    constexpr double geometryTolerance = 2.0e-12;
    constexpr double frameTolerance = 2.0e-12;

    const CurveFrame canonicalFrame{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    struct Measurements
    {
        double maximumAnalyticPositionError = 0.0;
        double maximumAnalyticFrameError = 0.0;
        double maximumCircleRadiusError = 0.0;
        double maximumPlaneError = 0.0;
        double rolledEndpointWorldVerticalExcursion = 0.0;
        double coarsePolylineLengthError = 0.0;
        double finePolylineLengthError = 0.0;
        double coarseFineEndpointPositionDifference = 0.0;
        double coarseFineEndpointFrameDifference = 0.0;
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

    [[nodiscard]] double vectorMagnitude(const glm::dvec3& vector)
    {
        return std::hypot(vector.x, vector.y, vector.z);
    }

    [[nodiscard]] double pointError(
        const glm::dvec3& first,
        const glm::dvec3& second
    )
    {
        return vectorMagnitude(first - second);
    }

    [[nodiscard]] double frameError(
        const CurveFrame& first,
        const CurveFrame& second
    )
    {
        return std::max({
            pointError(first.tangent, second.tangent),
            pointError(first.lateral, second.lateral),
            pointError(first.up, second.up)
        });
    }

    void requireValidFrame(
        const CurveFrame& frame,
        const std::string_view context
    )
    {
        requireNear(
            vectorMagnitude(frame.tangent),
            1.0,
            frameTolerance,
            std::string(context) + " tangent magnitude"
        );
        requireNear(
            vectorMagnitude(frame.lateral),
            1.0,
            frameTolerance,
            std::string(context) + " lateral magnitude"
        );
        requireNear(
            vectorMagnitude(frame.up),
            1.0,
            frameTolerance,
            std::string(context) + " up magnitude"
        );
        requireNear(
            glm::dot(frame.tangent, frame.lateral),
            0.0,
            frameTolerance,
            std::string(context) + " tangent/lateral orthogonality"
        );
        requireNear(
            glm::dot(frame.tangent, frame.up),
            0.0,
            frameTolerance,
            std::string(context) + " tangent/up orthogonality"
        );
        requireNear(
            glm::dot(frame.lateral, frame.up),
            0.0,
            frameTolerance,
            std::string(context) + " lateral/up orthogonality"
        );
        requireNear(
            pointError(
                glm::cross(frame.tangent, frame.lateral),
                frame.up
            ),
            0.0,
            frameTolerance,
            std::string(context) + " handedness"
        );
    }

    [[nodiscard]] glm::dvec3 analyticPosition(
        const glm::dvec3& startingPosition,
        const CurveFrame& startingFrame,
        const double pitchRate,
        const double distance
    )
    {
        if (pitchRate == 0.0)
        {
            return startingPosition + distance * startingFrame.tangent;
        }

        const double angle = pitchRate * distance;
        return startingPosition
            + (std::sin(angle) / pitchRate) * startingFrame.tangent
            + ((std::cos(angle) - 1.0) / pitchRate) * startingFrame.up;
    }

    [[nodiscard]] double polylineLength(
        const std::vector<RiderLocalGeometryState>& states
    )
    {
        double length = 0.0;
        for (std::size_t index = 1; index < states.size(); ++index)
        {
            length += pointError(
                states[index].position,
                states[index - 1].position
            );
        }

        return length;
    }

    void requireAnalyticArc(
        const glm::dvec3& startingPosition,
        const CurveFrame& startingFrame,
        const double sectionLength,
        const double pitchRate,
        const double spacing,
        const std::string_view context
    )
    {
        const std::vector<RiderLocalGeometryState> states =
            integrateConstantLocalPitchRate(
                startingPosition,
                startingFrame,
                sectionLength,
                pitchRate,
                spacing
            );

        require(!states.empty(), std::string(context) + " returned no states");
        require(states.front().distance == 0.0,
                std::string(context) + " start distance must be exact");
        require(states.back().distance == sectionLength,
                std::string(context) + " end distance must be exact");

        const glm::dvec3 circleCenter =
            startingPosition - startingFrame.up / pitchRate;
        const double expectedRadius = 1.0 / std::abs(pitchRate);
        double accumulatedDistance = 0.0;

        for (std::size_t index = 0; index < states.size(); ++index)
        {
            const RiderLocalGeometryState& state = states[index];
            const glm::dvec3 expectedPosition = analyticPosition(
                startingPosition,
                startingFrame,
                pitchRate,
                state.distance
            );
            const CurveFrame expectedFrame = applyLocalPitch(
                startingFrame,
                pitchRate * state.distance
            );
            const double positionError = pointError(
                state.position,
                expectedPosition
            );
            const double orientationError = frameError(
                state.frame,
                expectedFrame
            );
            const double radiusError = std::abs(
                pointError(state.position, circleCenter) - expectedRadius
            );
            const double planeError = std::abs(glm::dot(
                state.position - startingPosition,
                startingFrame.lateral
            ));

            measurements.maximumAnalyticPositionError = std::max(
                measurements.maximumAnalyticPositionError,
                positionError
            );
            measurements.maximumAnalyticFrameError = std::max(
                measurements.maximumAnalyticFrameError,
                orientationError
            );
            measurements.maximumCircleRadiusError = std::max(
                measurements.maximumCircleRadiusError,
                radiusError
            );
            measurements.maximumPlaneError = std::max(
                measurements.maximumPlaneError,
                planeError
            );

            requireNear(
                positionError,
                0.0,
                geometryTolerance,
                std::string(context) + " analytic position"
            );
            requireNear(
                orientationError,
                0.0,
                frameTolerance,
                std::string(context) + " accumulated pitch orientation"
            );
            requireNear(
                radiusError,
                0.0,
                geometryTolerance,
                std::string(context) + " circle radius"
            );
            requireNear(
                planeError,
                0.0,
                geometryTolerance,
                std::string(context) + " curvature plane"
            );
            requireValidFrame(state.frame, context);

            if (index > 0)
            {
                const double distanceIncrement =
                    state.distance - states[index - 1].distance;
                const double spacingRoundoff =
                    16.0
                    * std::numeric_limits<double>::epsilon()
                    * std::max(sectionLength, spacing);
                require(
                    distanceIncrement > 0.0
                        && distanceIncrement <= spacing + spacingRoundoff,
                    std::string(context)
                        + " state spacing must be positive and bounded"
                );
                accumulatedDistance += distanceIncrement;
            }
        }

        requireNear(
            accumulatedDistance,
            sectionLength,
            geometryTolerance,
            std::string(context) + " traveled centerline distance"
        );
    }

    void testZeroPitchIsExactlyStraight()
    {
        const glm::dvec3 startingPosition{2.0, -3.0, 5.0};
        constexpr double sectionLength = 12.5;
        constexpr double spacing = 0.7;
        const std::vector<RiderLocalGeometryState> states =
            integrateConstantLocalPitchRate(
                startingPosition,
                canonicalFrame,
                sectionLength,
                0.0,
                spacing
            );

        require(states.back().distance == sectionLength,
                "straight result must end at the exact requested distance");

        for (const RiderLocalGeometryState& state : states)
        {
            const glm::dvec3 expected =
                startingPosition + state.distance * canonicalFrame.tangent;
            require(
                pointError(state.position, expected) == 0.0,
                "zero pitch must produce exactly straight positions"
            );
            require(
                state.frame == canonicalFrame,
                "zero pitch must preserve the starting frame exactly"
            );
        }
    }

    void testPositiveNegativeQuarterAndHalfCircles()
    {
        const glm::dvec3 startingPosition{1.0, -2.0, 3.0};
        constexpr double pitchMagnitude = 0.2;
        constexpr double radius = 1.0 / pitchMagnitude;
        const double quarterLength =
            std::numbers::pi / (2.0 * pitchMagnitude);
        const double halfLength = std::numbers::pi / pitchMagnitude;

        requireAnalyticArc(
            startingPosition,
            canonicalFrame,
            quarterLength,
            pitchMagnitude,
            quarterLength / 24.0,
            "positive quarter-circle"
        );
        requireAnalyticArc(
            startingPosition,
            canonicalFrame,
            quarterLength,
            -pitchMagnitude,
            quarterLength / 24.0,
            "negative quarter-circle"
        );
        requireAnalyticArc(
            startingPosition,
            canonicalFrame,
            halfLength,
            pitchMagnitude,
            halfLength / 48.0,
            "positive half-circle"
        );
        requireAnalyticArc(
            startingPosition,
            canonicalFrame,
            halfLength,
            -pitchMagnitude,
            halfLength / 48.0,
            "negative half-circle"
        );

        const RiderLocalGeometryState positiveQuarter =
            integrateConstantLocalPitchRate(
                startingPosition,
                canonicalFrame,
                quarterLength,
                pitchMagnitude,
                quarterLength
            ).back();
        requireNear(
            pointError(
                positiveQuarter.position,
                startingPosition + glm::dvec3{radius, 0.0, -radius}
            ),
            0.0,
            geometryTolerance,
            "positive quarter-circle endpoint"
        );
        requireNear(
            pointError(
                positiveQuarter.frame.tangent,
                glm::dvec3{0.0, 0.0, -1.0}
            ),
            0.0,
            frameTolerance,
            "positive quarter-circle tangent"
        );

        const RiderLocalGeometryState negativeQuarter =
            integrateConstantLocalPitchRate(
                startingPosition,
                canonicalFrame,
                quarterLength,
                -pitchMagnitude,
                quarterLength
            ).back();
        requireNear(
            pointError(
                negativeQuarter.position,
                startingPosition + glm::dvec3{radius, 0.0, radius}
            ),
            0.0,
            geometryTolerance,
            "negative quarter-circle endpoint"
        );
        requireNear(
            pointError(
                negativeQuarter.frame.tangent,
                glm::dvec3{0.0, 0.0, 1.0}
            ),
            0.0,
            frameTolerance,
            "negative quarter-circle tangent"
        );
    }

    void testRolledStartingFrameRotatesCurvaturePlane()
    {
        const glm::dvec3 startingPosition{0.0};
        constexpr double pitchRate = 0.1;
        constexpr double radius = 1.0 / pitchRate;
        const double quarterLength =
            std::numbers::pi / (2.0 * pitchRate);
        const CurveFrame rolledFrame = applyRoll(
            canonicalFrame,
            std::numbers::pi / 2.0
        );

        requireAnalyticArc(
            startingPosition,
            rolledFrame,
            quarterLength,
            pitchRate,
            quarterLength / 32.0,
            "rolled-frame quarter-circle"
        );

        const RiderLocalGeometryState unrolledEnd =
            integrateConstantLocalPitchRate(
                startingPosition,
                canonicalFrame,
                quarterLength,
                pitchRate,
                quarterLength / 32.0
            ).back();
        const RiderLocalGeometryState rolledEnd =
            integrateConstantLocalPitchRate(
                startingPosition,
                rolledFrame,
                quarterLength,
                pitchRate,
                quarterLength / 32.0
            ).back();

        measurements.rolledEndpointWorldVerticalExcursion =
            std::abs(rolledEnd.position.z);
        requireNear(
            pointError(
                unrolledEnd.position,
                glm::dvec3{radius, 0.0, -radius}
            ),
            0.0,
            geometryTolerance,
            "unrolled banking reference"
        );
        requireNear(
            pointError(
                rolledEnd.position,
                glm::dvec3{radius, radius, 0.0}
            ),
            0.0,
            geometryTolerance,
            "rolled curvature-plane endpoint"
        );
        requireNear(
            rolledEnd.position.z,
            0.0,
            geometryTolerance,
            "rolled curvature plane must not remain world vertical"
        );
        requireNear(
            pointError(rolledEnd.frame.tangent, glm::dvec3{0.0, 1.0, 0.0}),
            0.0,
            frameTolerance,
            "rolled curvature-plane tangent"
        );
        requireNear(
            glm::dot(canonicalFrame.lateral, rolledFrame.lateral),
            0.0,
            frameTolerance,
            "quarter-roll must rotate the two curvature-plane normals"
        );
    }

    void testResolutionStabilityAndPolylineConvergence()
    {
        const glm::dvec3 startingPosition{-4.0, 2.0, 7.0};
        constexpr double pitchRate = 0.125;
        const double sectionLength = std::numbers::pi / pitchRate;
        const std::vector<RiderLocalGeometryState> coarse =
            integrateConstantLocalPitchRate(
                startingPosition,
                canonicalFrame,
                sectionLength,
                pitchRate,
                sectionLength / 4.0
            );
        const std::vector<RiderLocalGeometryState> fine =
            integrateConstantLocalPitchRate(
                startingPosition,
                canonicalFrame,
                sectionLength,
                pitchRate,
                sectionLength / 256.0
            );
        const glm::dvec3 expectedPosition = analyticPosition(
            startingPosition,
            canonicalFrame,
            pitchRate,
            sectionLength
        );
        const CurveFrame expectedFrame = applyLocalPitch(
            canonicalFrame,
            pitchRate * sectionLength
        );

        requireNear(
            pointError(coarse.back().position, expectedPosition),
            0.0,
            geometryTolerance,
            "coarse analytic endpoint"
        );
        requireNear(
            pointError(fine.back().position, expectedPosition),
            0.0,
            geometryTolerance,
            "fine analytic endpoint"
        );
        requireNear(
            frameError(coarse.back().frame, expectedFrame),
            0.0,
            frameTolerance,
            "coarse analytic endpoint frame"
        );
        requireNear(
            frameError(fine.back().frame, expectedFrame),
            0.0,
            frameTolerance,
            "fine analytic endpoint frame"
        );

        measurements.coarseFineEndpointPositionDifference = pointError(
            coarse.back().position,
            fine.back().position
        );
        measurements.coarseFineEndpointFrameDifference = frameError(
            coarse.back().frame,
            fine.back().frame
        );
        measurements.coarsePolylineLengthError =
            sectionLength - polylineLength(coarse);
        measurements.finePolylineLengthError =
            sectionLength - polylineLength(fine);

        require(
            measurements.coarsePolylineLengthError > 0.0,
            "coarse sampled polyline must be shorter than its analytic arc"
        );
        require(
            measurements.finePolylineLengthError > 0.0
                && measurements.finePolylineLengthError
                    < measurements.coarsePolylineLengthError,
            "fine sampled polyline must converge toward the analytic arc length"
        );
        requireNear(
            measurements.coarseFineEndpointPositionDifference,
            0.0,
            geometryTolerance,
            "coarse/fine endpoint position stability"
        );
        requireNear(
            measurements.coarseFineEndpointFrameDifference,
            0.0,
            frameTolerance,
            "coarse/fine endpoint frame stability"
        );
    }

    void testValidationAndZeroLength()
    {
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        constexpr double infinity = std::numeric_limits<double>::infinity();

        for (const glm::dvec3 position : {
                 glm::dvec3{nan, 0.0, 0.0},
                 glm::dvec3{0.0, infinity, 0.0},
                 glm::dvec3{0.0, 0.0, -infinity}
             })
        {
            requireThrows<std::invalid_argument>(
                [position]
                {
                    static_cast<void>(integrateConstantLocalPitchRate(
                        position, canonicalFrame, 1.0, 0.1, 0.1
                    ));
                },
                "non-finite starting position"
            );
        }

        for (const double length : {-1.0, nan, infinity})
        {
            requireThrows<std::invalid_argument>(
                [length]
                {
                    static_cast<void>(integrateConstantLocalPitchRate(
                        glm::dvec3{0.0},
                        canonicalFrame,
                        length,
                        0.1,
                        0.1
                    ));
                },
                "invalid section length"
            );
        }

        for (const double rate : {nan, infinity, -infinity})
        {
            requireThrows<std::invalid_argument>(
                [rate]
                {
                    static_cast<void>(integrateConstantLocalPitchRate(
                        glm::dvec3{0.0},
                        canonicalFrame,
                        1.0,
                        rate,
                        0.1
                    ));
                },
                "invalid local pitch rate"
            );
        }

        for (const double spacing : {0.0, -1.0, nan, infinity})
        {
            requireThrows<std::invalid_argument>(
                [spacing]
                {
                    static_cast<void>(integrateConstantLocalPitchRate(
                        glm::dvec3{0.0},
                        canonicalFrame,
                        1.0,
                        0.1,
                        spacing
                    ));
                },
                "invalid integration spacing"
            );
        }

        const std::vector<CurveFrame> invalidFrames{
            {{infinity, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
            {{2.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
            {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}},
            {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}}
        };

        for (const CurveFrame& frame : invalidFrames)
        {
            requireThrows<std::invalid_argument>(
                [&frame]
                {
                    static_cast<void>(integrateConstantLocalPitchRate(
                        glm::dvec3{0.0},
                        frame,
                        1.0,
                        0.1,
                        0.1
                    ));
                },
                "invalid starting frame"
            );
        }

        const glm::dvec3 startingPosition{3.0, 4.0, 5.0};
        const CurveFrame rolledFrame = applyRoll(canonicalFrame, 0.37);
        const std::vector<RiderLocalGeometryState> zeroLength =
            integrateConstantLocalPitchRate(
                startingPosition,
                rolledFrame,
                0.0,
                -0.2,
                0.25
            );

        require(zeroLength.size() == 1,
                "zero length must return exactly one state");
        require(zeroLength.front().distance == 0.0,
                "zero length must return exact zero distance");
        require(pointError(zeroLength.front().position, startingPosition) == 0.0,
                "zero length must preserve starting position exactly");
        require(zeroLength.front().frame == rolledFrame,
                "zero length must preserve starting frame exactly");
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;
    const std::vector<Test> tests{
        {"zero pitch is exactly straight", testZeroPitchIsExactlyStraight},
        {
            "positive/negative quarter- and half-circles",
            testPositiveNegativeQuarterAndHalfCircles
        },
        {
            "rolled frame rotates the curvature plane",
            testRolledStartingFrameRotatesCurvaturePlane
        },
        {
            "resolution stability and polyline convergence",
            testResolutionStabilityAndPolylineConvergence
        },
        {"validation and zero length", testValidationAndZeroLength}
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

    std::cout << std::setprecision(17);
    std::cout << "[METRIC] maximum analytic position error: "
              << measurements.maximumAnalyticPositionError << '\n';
    std::cout << "[METRIC] maximum analytic frame error: "
              << measurements.maximumAnalyticFrameError << '\n';
    std::cout << "[METRIC] maximum circle-radius error: "
              << measurements.maximumCircleRadiusError << '\n';
    std::cout << "[METRIC] maximum curvature-plane error: "
              << measurements.maximumPlaneError << '\n';
    std::cout << "[METRIC] rolled endpoint world-vertical excursion: "
              << measurements.rolledEndpointWorldVerticalExcursion << '\n';
    std::cout << "[METRIC] coarse/fine endpoint position difference: "
              << measurements.coarseFineEndpointPositionDifference << '\n';
    std::cout << "[METRIC] coarse/fine endpoint frame difference: "
              << measurements.coarseFineEndpointFrameDifference << '\n';
    std::cout << "[METRIC] polyline length errors: coarse="
              << measurements.coarsePolylineLengthError
              << ", fine=" << measurements.finePolylineLengthError << '\n';
    std::cout << tests.size() << " test groups passed.\n";
    return 0;
}
