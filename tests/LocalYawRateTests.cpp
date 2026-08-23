#include <quantum/coaster/RiderLocalGeometry.hpp>

#include <glm/geometric.hpp>

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
    using quantum::coaster::integrateConstantLocalYawRate;
    using quantum::coaster::integrateLocalYawRateProfile;
    using quantum::coaster::RiderLocalGeometryState;
    using quantum::geometry::applyLocalPitch;
    using quantum::geometry::applyRoll;
    using quantum::geometry::CurveFrame;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

    constexpr double constantTolerance = 5.0e-12;
    constexpr double positionTolerance = 5.0e-10;
    constexpr double orientationTolerance = 3.0e-12;
    constexpr std::size_t referenceIntervalCount = 32'768;

    const CurveFrame canonicalFrame{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    struct Measurements
    {
        double maximumConstantPositionError = 0.0;
        double maximumConstantOrientationError = 0.0;
        double maximumCircleRadiusError = 0.0;
        double maximumConstantProfilePositionError = 0.0;
        double maximumConstantProfileOrientationError = 0.0;
        double maximumReferencePositionError = 0.0;
        double maximumReferenceOrientationError = 0.0;
        double maximumFrameInvariantError = 0.0;
        double maximumUpAxisError = 0.0;
        double symmetricAccumulatedYaw = 0.0;
        double asymmetricAccumulatedYaw = 0.0;
        double progressiveFirstAverageCurvature = 0.0;
        double progressiveLastAverageCurvature = 0.0;
        double reverseFirstAverageCurvature = 0.0;
        double reverseLastAverageCurvature = 0.0;
        double signChangingEndpointLateralDisplacement = 0.0;
        double rolledPlaneError = 0.0;
        double pitchedPlaneError = 0.0;
        double translatedPositionDifference = 0.0;
        double translatedOrientationDifference = 0.0;
        glm::dvec3 quarterCircleEndpoint{0.0};
        glm::dvec3 halfCircleEndpoint{0.0};
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
            message << context << ": expected " << std::setprecision(17)
                    << expected << ", got " << actual
                    << ", tolerance " << tolerance;
            throw TestFailure(message.str());
        }
    }

    template<typename Exception, typename Operation>
    void requireThrows(
        Operation&& operation,
        const std::string_view context
    )
    {
        try
        {
            std::invoke(std::forward<Operation>(operation));
        }
        catch (const Exception&)
        {
            return;
        }
        catch (const std::exception& exception)
        {
            throw TestFailure(
                std::string(context) + ": wrong exception type: "
                + exception.what()
            );
        }

        throw TestFailure(
            std::string(context) + ": expected exception was not thrown"
        );
    }

    [[nodiscard]] bool isFinite(const glm::dvec3& vector) noexcept
    {
        return std::isfinite(vector.x)
            && std::isfinite(vector.y)
            && std::isfinite(vector.z);
    }

    [[nodiscard]] double magnitude(const glm::dvec3& vector)
    {
        return std::hypot(vector.x, vector.y, vector.z);
    }

    [[nodiscard]] double pointError(
        const glm::dvec3& first,
        const glm::dvec3& second
    )
    {
        return magnitude(first - second);
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

    [[nodiscard]] bool bitwiseEqual(
        const double first,
        const double second
    ) noexcept
    {
        return std::bit_cast<std::uint64_t>(first)
            == std::bit_cast<std::uint64_t>(second);
    }

    [[nodiscard]] bool bitwiseEqual(
        const glm::dvec3& first,
        const glm::dvec3& second
    ) noexcept
    {
        return bitwiseEqual(first.x, second.x)
            && bitwiseEqual(first.y, second.y)
            && bitwiseEqual(first.z, second.z);
    }

    [[nodiscard]] bool bitwiseEqual(
        const CurveFrame& first,
        const CurveFrame& second
    ) noexcept
    {
        return bitwiseEqual(first.tangent, second.tangent)
            && bitwiseEqual(first.lateral, second.lateral)
            && bitwiseEqual(first.up, second.up);
    }

    [[nodiscard]] CurveFrame analyticYawFrame(
        const CurveFrame& startingFrame,
        const double yaw
    )
    {
        const double cosine = std::cos(yaw);
        const double sine = std::sin(yaw);
        return {
            cosine * startingFrame.tangent
                + sine * startingFrame.lateral,
            cosine * startingFrame.lateral
                - sine * startingFrame.tangent,
            startingFrame.up
        };
    }

    void requireValidState(
        const RiderLocalGeometryState& state,
        const glm::dvec3& expectedUp,
        const std::string_view context
    )
    {
        require(std::isfinite(state.distance),
            std::string(context) + " distance is non-finite");
        require(isFinite(state.position),
            std::string(context) + " position is non-finite");
        require(isFinite(state.frame.tangent)
                && isFinite(state.frame.lateral)
                && isFinite(state.frame.up),
            std::string(context) + " frame is non-finite");

        const double invariantError = std::max({
            std::abs(magnitude(state.frame.tangent) - 1.0),
            std::abs(magnitude(state.frame.lateral) - 1.0),
            std::abs(magnitude(state.frame.up) - 1.0),
            std::abs(glm::dot(
                state.frame.tangent,
                state.frame.lateral
            )),
            std::abs(glm::dot(state.frame.tangent, state.frame.up)),
            std::abs(glm::dot(state.frame.lateral, state.frame.up)),
            pointError(
                glm::cross(state.frame.tangent, state.frame.lateral),
                state.frame.up
            )
        });
        const double upError = pointError(state.frame.up, expectedUp);
        measurements.maximumFrameInvariantError = std::max(
            measurements.maximumFrameInvariantError,
            invariantError
        );
        measurements.maximumUpAxisError = std::max(
            measurements.maximumUpAxisError,
            upError
        );
        requireNear(
            invariantError,
            0.0,
            orientationTolerance,
            std::string(context) + " frame invariants"
        );
        requireNear(
            upError,
            0.0,
            orientationTolerance,
            std::string(context) + " unchanged up axis"
        );
        require(
            bitwiseEqual(state.frame.up, expectedUp),
            std::string(context) + " changed up-axis bits"
        );
    }

    [[nodiscard]] long double transitionIntegralReference(
        const TransitionType type,
        const long double progress
    )
    {
        const long double square = progress * progress;
        const long double cube = square * progress;

        switch (type)
        {
        case TransitionType::Linear:
            return square / 2.0L;
        case TransitionType::Smootherstep:
            return cube * progress
                * (2.5L - 3.0L * progress + square);
        case TransitionType::SeventhOrderSmoothstep:
            return progress * progress * progress * progress * progress
                * (7.0L - 14.0L * progress + 10.0L * square
                    - 2.5L * cube);
        case TransitionType::CosineEaseInOut:
            return progress / 2.0L
                - std::sin(std::numbers::pi_v<long double> * progress)
                    / (2.0L * std::numbers::pi_v<long double>);
        case TransitionType::SineEaseOut:
            return 2.0L * (
                1.0L - std::cos(
                    std::numbers::pi_v<long double> * progress / 2.0L
                )
            ) / std::numbers::pi_v<long double>;
        case TransitionType::QuadraticEaseIn:
            return cube / 3.0L;
        default:
            throw TestFailure(
                "A yaw test requested an unsupported independent transition reference."
            );
        }
    }

    [[nodiscard]] long double accumulatedYawReference(
        const ScalarTransition& profile,
        const long double traveledDistance
    )
    {
        const long double length = static_cast<long double>(
            profile.domainEnd - profile.domainBegin
        );
        const long double progress = traveledDistance / length;
        return static_cast<long double>(profile.valueBegin) * traveledDistance
            + static_cast<long double>(
                profile.valueEnd - profile.valueBegin
            ) * length * transitionIntegralReference(
                profile.transitionType,
                progress
            );
    }

    [[nodiscard]] glm::dvec3 referencePosition(
        const glm::dvec3& startingPosition,
        const CurveFrame& startingFrame,
        const ScalarTransition& profile,
        const double traveledDistance
    )
    {
        if (traveledDistance == 0.0)
        {
            return startingPosition;
        }

        // Composite Simpson integration is independent of the production
        // Gauss-Legendre quadrature.
        const long double step = static_cast<long double>(traveledDistance)
            / static_cast<long double>(referenceIntervalCount);
        long double tangentIntegral = 0.0L;
        long double lateralIntegral = 0.0L;

        for (std::size_t index = 0;
             index <= referenceIntervalCount;
             ++index)
        {
            const long double distance = step
                * static_cast<long double>(index);
            const long double yaw = accumulatedYawReference(
                profile,
                distance
            );
            const long double weight = index == 0
                    || index == referenceIntervalCount
                ? 1.0L
                : index % 2 == 0 ? 2.0L : 4.0L;
            tangentIntegral += weight * std::cos(yaw);
            lateralIntegral += weight * std::sin(yaw);
        }

        tangentIntegral *= step / 3.0L;
        lateralIntegral *= step / 3.0L;
        return startingPosition
            + static_cast<double>(tangentIntegral) * startingFrame.tangent
            + static_cast<double>(lateralIntegral) * startingFrame.lateral;
    }

    void requireMatchesVariableReference(
        const glm::dvec3& startingPosition,
        const CurveFrame& startingFrame,
        const ScalarTransition& profile,
        const std::vector<RiderLocalGeometryState>& states,
        const std::string_view context
    )
    {
        for (const RiderLocalGeometryState& state : states)
        {
            const glm::dvec3 expectedPosition = referencePosition(
                startingPosition,
                startingFrame,
                profile,
                state.distance
            );
            const double expectedYaw = static_cast<double>(
                accumulatedYawReference(profile, state.distance)
            );
            const CurveFrame expectedFrame = analyticYawFrame(
                startingFrame,
                expectedYaw
            );
            const double positionError = pointError(
                state.position,
                expectedPosition
            );
            const double orientationError = frameError(
                state.frame,
                expectedFrame
            );
            measurements.maximumReferencePositionError = std::max(
                measurements.maximumReferencePositionError,
                positionError
            );
            measurements.maximumReferenceOrientationError = std::max(
                measurements.maximumReferenceOrientationError,
                orientationError
            );
            requireNear(
                positionError,
                0.0,
                positionTolerance,
                std::string(context) + " position reference"
            );
            requireNear(
                orientationError,
                0.0,
                orientationTolerance,
                std::string(context) + " orientation reference"
            );
            requireValidState(state, startingFrame.up, context);
        }
    }

    [[nodiscard]] double localYaw(
        const CurveFrame& frame,
        const CurveFrame& startingFrame
    )
    {
        return std::atan2(
            glm::dot(frame.tangent, startingFrame.lateral),
            glm::dot(frame.tangent, startingFrame.tangent)
        );
    }

    void testConstantAnalyticGeometry()
    {
        const glm::dvec3 startingPosition{2.0, -3.0, 4.0};
        const auto straight = integrateConstantLocalYawRate(
            startingPosition,
            canonicalFrame,
            7.5,
            0.0,
            2.0
        );
        const std::vector<double> expectedDistances{0.0, 2.0, 4.0, 6.0, 7.5};
        require(straight.size() == expectedDistances.size(),
            "zero-yaw sampling count");

        for (std::size_t index = 0; index < straight.size(); ++index)
        {
            requireNear(straight[index].distance, expectedDistances[index],
                0.0, "zero-yaw traveled distance");
            requireNear(
                pointError(
                    straight[index].position,
                    startingPosition
                        + straight[index].distance * canonicalFrame.tangent
                ),
                0.0,
                0.0,
                "zero-yaw straight position"
            );
            require(bitwiseEqual(straight[index].frame, canonicalFrame),
                "zero yaw changed the frame");
            requireValidState(straight[index], canonicalFrame.up, "zero yaw");
        }

        constexpr double rate = 0.2;
        const double quarterLength = std::numbers::pi / (2.0 * rate);
        const double halfLength = std::numbers::pi / rate;

        const auto checkArc = [&](
            const double localRate,
            const double length,
            const double spacing,
            const std::string_view context
        )
        {
            const auto states = integrateConstantLocalYawRate(
                startingPosition,
                canonicalFrame,
                length,
                localRate,
                spacing
            );
            const glm::dvec3 center = startingPosition
                + canonicalFrame.lateral / localRate;

            for (const RiderLocalGeometryState& state : states)
            {
                const double yaw = localRate * state.distance;
                const glm::dvec3 expectedPosition = startingPosition
                    + (std::sin(yaw) / localRate) * canonicalFrame.tangent
                    + ((1.0 - std::cos(yaw)) / localRate)
                        * canonicalFrame.lateral;
                const CurveFrame expectedFrame = analyticYawFrame(
                    canonicalFrame,
                    yaw
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
                    magnitude(state.position - center)
                    - 1.0 / std::abs(localRate)
                );
                measurements.maximumConstantPositionError = std::max(
                    measurements.maximumConstantPositionError,
                    positionError
                );
                measurements.maximumConstantOrientationError = std::max(
                    measurements.maximumConstantOrientationError,
                    orientationError
                );
                measurements.maximumCircleRadiusError = std::max(
                    measurements.maximumCircleRadiusError,
                    radiusError
                );
                requireNear(positionError, 0.0, constantTolerance,
                    std::string(context) + " analytic position");
                requireNear(orientationError, 0.0, constantTolerance,
                    std::string(context) + " analytic frame");
                requireNear(radiusError, 0.0, constantTolerance,
                    std::string(context) + " circle radius");
                requireValidState(state, canonicalFrame.up, context);
            }

            return states.back();
        };

        const RiderLocalGeometryState positiveQuarter = checkArc(
            rate, quarterLength, 0.37, "positive quarter circle"
        );
        const RiderLocalGeometryState negativeQuarter = checkArc(
            -rate, quarterLength, 0.43, "negative quarter circle"
        );
        const RiderLocalGeometryState positiveHalf = checkArc(
            rate, halfLength, 0.61, "positive half circle"
        );
        static_cast<void>(checkArc(
            -0.137, 9.3, 0.52, "negative non-quarter circle"
        ));

        measurements.quarterCircleEndpoint = positiveQuarter.position;
        measurements.halfCircleEndpoint = positiveHalf.position;
        requireNear(
            pointError(
                positiveQuarter.position,
                startingPosition + glm::dvec3{5.0, 5.0, 0.0}
            ),
            0.0,
            constantTolerance,
            "positive quarter-circle endpoint"
        );
        requireNear(
            pointError(
                negativeQuarter.position,
                startingPosition + glm::dvec3{5.0, -5.0, 0.0}
            ),
            0.0,
            constantTolerance,
            "negative quarter-circle endpoint"
        );
        requireNear(
            pointError(
                positiveHalf.position,
                startingPosition + glm::dvec3{0.0, 10.0, 0.0}
            ),
            0.0,
            constantTolerance,
            "positive half-circle endpoint"
        );

        const auto zeroLength = integrateConstantLocalYawRate(
            startingPosition, canonicalFrame, 0.0, 0.2, 1.0
        );
        require(zeroLength.size() == 1, "zero length must return one state");
        require(bitwiseEqual(zeroLength.front().position, startingPosition)
                && bitwiseEqual(zeroLength.front().frame, canonicalFrame),
            "zero length changed the beginning state");
    }

    void testConstantProfileEquivalence()
    {
        struct ConstantCase
        {
            double domainBegin;
            double length;
            double rate;
            double spacing;
        };

        const std::vector<ConstantCase> cases{
            {0.0, 8.0, 0.0, 1.3},
            {0.0, 9.0, 0.17, 0.7},
            {100.0, 7.0, -0.21, 0.9},
            {0.0, std::numbers::pi / (2.0 * 0.2), 0.2, 0.37},
            {-20.0, 9.3, -0.137, 0.52}
        };
        const glm::dvec3 startingPosition{-1.0, 2.0, 3.0};

        for (const ConstantCase& testCase : cases)
        {
            const ScalarTransition profile{
                testCase.domainBegin,
                testCase.domainBegin + testCase.length,
                testCase.rate,
                testCase.rate,
                TransitionType::SeventhOrderSmoothstep
            };
            const auto constantStates = integrateConstantLocalYawRate(
                startingPosition,
                canonicalFrame,
                testCase.length,
                testCase.rate,
                testCase.spacing
            );
            const auto profileStates = integrateLocalYawRateProfile(
                startingPosition,
                canonicalFrame,
                profile,
                testCase.spacing
            );
            require(constantStates.size() == profileStates.size(),
                "constant profile equivalence state count");

            for (std::size_t index = 0;
                 index < constantStates.size();
                 ++index)
            {
                const double positionError = pointError(
                    constantStates[index].position,
                    profileStates[index].position
                );
                const double orientationError = frameError(
                    constantStates[index].frame,
                    profileStates[index].frame
                );
                measurements.maximumConstantProfilePositionError = std::max(
                    measurements.maximumConstantProfilePositionError,
                    positionError
                );
                measurements.maximumConstantProfileOrientationError = std::max(
                    measurements.maximumConstantProfileOrientationError,
                    orientationError
                );
                require(
                    bitwiseEqual(
                        constantStates[index].distance,
                        profileStates[index].distance
                    )
                        && bitwiseEqual(
                            constantStates[index].position,
                            profileStates[index].position
                        )
                        && bitwiseEqual(
                            constantStates[index].frame,
                            profileStates[index].frame
                        ),
                    "constant profile did not delegate bitwise-equivalently"
                );
            }
        }
    }

    void testVariableOrientationAndAreaReferences()
    {
        const ScalarTransition linear{
            0.0, 20.0, 0.02, 0.18, TransitionType::Linear
        };
        const auto linearStates = integrateLocalYawRateProfile(
            {-2.0, 3.0, 5.0}, canonicalFrame, linear, 2.5
        );
        constexpr double slope = (0.18 - 0.02) / 20.0;

        for (const RiderLocalGeometryState& state : linearStates)
        {
            const double expectedYaw = 0.02 * state.distance
                + 0.5 * slope * state.distance * state.distance;
            const double error = frameError(
                state.frame,
                analyticYawFrame(canonicalFrame, expectedYaw)
            );
            measurements.maximumReferenceOrientationError = std::max(
                measurements.maximumReferenceOrientationError,
                error
            );
            requireNear(error, 0.0, orientationTolerance,
                "linear a*s + 0.5*b*s^2 orientation");
        }

        const ScalarTransition smootherstep{
            0.0, 12.0, 0.015, 0.13, TransitionType::Smootherstep
        };
        const ScalarTransition symmetric{
            0.0, 10.0, 0.01, 0.11, TransitionType::CosineEaseInOut
        };
        const ScalarTransition asymmetric{
            0.0, 9.0, 0.02, 0.14, TransitionType::QuadraticEaseIn
        };

        const auto smootherstepStates = integrateLocalYawRateProfile(
            {1.0, -2.0, 0.5}, canonicalFrame, smootherstep, 2.0
        );
        const auto symmetricStates = integrateLocalYawRateProfile(
            {0.0, 0.0, 0.0}, canonicalFrame, symmetric, 2.0
        );
        const auto asymmetricStates = integrateLocalYawRateProfile(
            {0.0, 0.0, 0.0}, canonicalFrame, asymmetric, 1.5
        );
        requireMatchesVariableReference(
            {1.0, -2.0, 0.5},
            canonicalFrame,
            smootherstep,
            smootherstepStates,
            "smootherstep yaw"
        );
        requireMatchesVariableReference(
            {0.0, 0.0, 0.0},
            canonicalFrame,
            symmetric,
            symmetricStates,
            "symmetric cosine yaw"
        );
        requireMatchesVariableReference(
            {0.0, 0.0, 0.0},
            canonicalFrame,
            asymmetric,
            asymmetricStates,
            "asymmetric quadratic yaw"
        );

        const double expectedSymmetricYaw = 10.0
            * (0.01 + (0.11 - 0.01) * 0.5);
        const double expectedAsymmetricYaw = 9.0
            * (0.02 + (0.14 - 0.02) / 3.0);
        measurements.symmetricAccumulatedYaw = localYaw(
            symmetricStates.back().frame,
            canonicalFrame
        );
        measurements.asymmetricAccumulatedYaw = localYaw(
            asymmetricStates.back().frame,
            canonicalFrame
        );
        requireNear(
            measurements.symmetricAccumulatedYaw,
            expectedSymmetricYaw,
            orientationTolerance,
            "symmetric full-domain accumulated yaw"
        );
        requireNear(
            measurements.asymmetricAccumulatedYaw,
            expectedAsymmetricYaw,
            orientationTolerance,
            "asymmetric full-domain accumulated yaw"
        );
        require(
            std::abs(expectedAsymmetricYaw
                - 9.0 * (0.02 + 0.14) / 2.0) > 0.1,
            "asymmetric area reference accidentally equals endpoint average"
        );
    }

    [[nodiscard]] double averageYawCurvature(
        const RiderLocalGeometryState& first,
        const RiderLocalGeometryState& second,
        const CurveFrame& startingFrame
    )
    {
        return (localYaw(second.frame, startingFrame)
            - localYaw(first.frame, startingFrame))
            / (second.distance - first.distance);
    }

    void testProgressiveReverseAndSignChangingProfiles()
    {
        const ScalarTransition progressive{
            0.0, 12.0, 0.01, 0.12, TransitionType::Smootherstep
        };
        const ScalarTransition reverse{
            0.0, 12.0, 0.12, 0.01, TransitionType::SineEaseOut
        };
        const ScalarTransition signChanging{
            0.0, 12.0, -0.12, 0.12, TransitionType::Linear
        };
        const auto progressiveStates = integrateLocalYawRateProfile(
            {0.0, 0.0, 0.0}, canonicalFrame, progressive, 1.0
        );
        const auto reverseStates = integrateLocalYawRateProfile(
            {0.0, 0.0, 0.0}, canonicalFrame, reverse, 1.0
        );
        const auto signChangingStates = integrateLocalYawRateProfile(
            {0.0, 0.0, 0.0}, canonicalFrame, signChanging, 1.0
        );

        measurements.progressiveFirstAverageCurvature = averageYawCurvature(
            progressiveStates[0], progressiveStates[1], canonicalFrame
        );
        measurements.progressiveLastAverageCurvature = averageYawCurvature(
            progressiveStates[progressiveStates.size() - 2],
            progressiveStates.back(),
            canonicalFrame
        );
        measurements.reverseFirstAverageCurvature = averageYawCurvature(
            reverseStates[0], reverseStates[1], canonicalFrame
        );
        measurements.reverseLastAverageCurvature = averageYawCurvature(
            reverseStates[reverseStates.size() - 2],
            reverseStates.back(),
            canonicalFrame
        );
        require(
            measurements.progressiveFirstAverageCurvature
                < measurements.progressiveLastAverageCurvature,
            "progressive yaw curvature did not increase"
        );
        require(
            measurements.reverseFirstAverageCurvature
                > measurements.reverseLastAverageCurvature,
            "reverse yaw curvature did not decrease"
        );

        double previousAverageCurvature = -std::numeric_limits<double>::infinity();
        for (std::size_t index = 1;
             index < signChangingStates.size();
             ++index)
        {
            const double averageCurvature = averageYawCurvature(
                signChangingStates[index - 1],
                signChangingStates[index],
                canonicalFrame
            );
            require(
                averageCurvature > previousAverageCurvature,
                "sign-changing yaw curvature did not change smoothly"
            );
            previousAverageCurvature = averageCurvature;
            requireValidState(
                signChangingStates[index],
                canonicalFrame.up,
                "sign-changing yaw"
            );
        }
        require(
            averageYawCurvature(
                signChangingStates[0],
                signChangingStates[1],
                canonicalFrame
            ) < 0.0,
            "sign-changing yaw did not begin negative"
        );
        require(
            averageYawCurvature(
                signChangingStates[signChangingStates.size() - 2],
                signChangingStates.back(),
                canonicalFrame
            ) > 0.0,
            "sign-changing yaw did not end positive"
        );
        requireNear(
            frameError(signChangingStates.back().frame, canonicalFrame),
            0.0,
            orientationTolerance,
            "zero-net-yaw final orientation"
        );
        measurements.signChangingEndpointLateralDisplacement = std::abs(
            glm::dot(
                signChangingStates.back().position,
                canonicalFrame.lateral
            )
        );
        require(
            measurements.signChangingEndpointLateralDisplacement > 0.1,
            "zero-net-yaw profile incorrectly returned to its starting position"
        );

        requireMatchesVariableReference(
            {0.0, 0.0, 0.0},
            canonicalFrame,
            progressive,
            progressiveStates,
            "progressive yaw"
        );
        requireMatchesVariableReference(
            {0.0, 0.0, 0.0},
            canonicalFrame,
            reverse,
            reverseStates,
            "reverse yaw"
        );
        requireMatchesVariableReference(
            {0.0, 0.0, 0.0},
            canonicalFrame,
            signChanging,
            signChangingStates,
            "sign-changing yaw"
        );
    }

    void testTransformedStartingFrames()
    {
        const glm::dvec3 startingPosition{-3.0, 2.0, 1.0};
        const ScalarTransition rolledProfile{
            0.0, 14.0, 0.02, 0.17, TransitionType::Smootherstep
        };
        const ScalarTransition pitchedProfile{
            0.0, 11.0, -0.03, 0.14, TransitionType::CosineEaseInOut
        };
        const CurveFrame rolledFrame = applyRoll(canonicalFrame, 0.63);
        const CurveFrame pitchedFrame = applyLocalPitch(canonicalFrame, 0.47);
        const auto rolledStates = integrateLocalYawRateProfile(
            startingPosition, rolledFrame, rolledProfile, 2.0
        );
        const auto pitchedStates = integrateLocalYawRateProfile(
            startingPosition, pitchedFrame, pitchedProfile, 1.75
        );

        requireMatchesVariableReference(
            startingPosition,
            rolledFrame,
            rolledProfile,
            rolledStates,
            "rolled-frame yaw"
        );
        requireMatchesVariableReference(
            startingPosition,
            pitchedFrame,
            pitchedProfile,
            pitchedStates,
            "pitched-frame yaw"
        );

        for (const RiderLocalGeometryState& state : rolledStates)
        {
            measurements.rolledPlaneError = std::max(
                measurements.rolledPlaneError,
                std::abs(glm::dot(
                    state.position - startingPosition,
                    rolledFrame.up
                ))
            );
        }
        for (const RiderLocalGeometryState& state : pitchedStates)
        {
            measurements.pitchedPlaneError = std::max(
                measurements.pitchedPlaneError,
                std::abs(glm::dot(
                    state.position - startingPosition,
                    pitchedFrame.up
                ))
            );
        }
        requireNear(measurements.rolledPlaneError, 0.0,
            positionTolerance, "rolled rider-local yaw plane");
        requireNear(measurements.pitchedPlaneError, 0.0,
            positionTolerance, "pitched rider-local yaw plane");
        require(
            std::abs(rolledStates.back().position.z - startingPosition.z)
                > 0.1,
            "rolled yaw remained locked to a world-horizontal plane"
        );
        require(
            std::abs(pitchedStates.back().position.z - startingPosition.z)
                > 0.1,
            "pitched yaw snapped to a world-horizontal tangent plane"
        );
    }

    void testTranslatedDomainSamplingAndDeterminism()
    {
        const ScalarTransition originProfile{
            0.0, 20.0, 0.01, 0.16, TransitionType::SeventhOrderSmoothstep
        };
        const ScalarTransition translatedProfile{
            100.0, 120.0, 0.01, 0.16,
            TransitionType::SeventhOrderSmoothstep
        };
        const glm::dvec3 startingPosition{4.0, -2.0, 7.0};
        const auto originStates = integrateLocalYawRateProfile(
            startingPosition, canonicalFrame, originProfile, 2.5
        );
        const auto translatedStates = integrateLocalYawRateProfile(
            startingPosition, canonicalFrame, translatedProfile, 2.5
        );
        require(originStates.size() == translatedStates.size(),
            "translated-domain state count");
        require(originStates.front().distance == 0.0
                && translatedStates.front().distance == 0.0,
            "translated-domain traveled distance did not begin at zero");

        for (std::size_t index = 0; index < originStates.size(); ++index)
        {
            requireNear(
                translatedStates[index].distance,
                originStates[index].distance,
                0.0,
                "translated-domain traveled distance"
            );
            measurements.translatedPositionDifference = std::max(
                measurements.translatedPositionDifference,
                pointError(
                    translatedStates[index].position,
                    originStates[index].position
                )
            );
            measurements.translatedOrientationDifference = std::max(
                measurements.translatedOrientationDifference,
                frameError(
                    translatedStates[index].frame,
                    originStates[index].frame
                )
            );
        }
        requireNear(measurements.translatedPositionDifference, 0.0,
            positionTolerance, "translated-domain position equivalence");
        requireNear(measurements.translatedOrientationDifference, 0.0,
            orientationTolerance, "translated-domain frame equivalence");

        const ScalarTransition sampledProfile{
            0.0, 7.25, 0.01, 0.15, TransitionType::QuadraticEaseIn
        };
        const auto states = integrateLocalYawRateProfile(
            startingPosition, canonicalFrame, sampledProfile, 2.0
        );
        const std::vector<double> expectedDistances{0.0, 2.0, 4.0, 6.0, 7.25};
        require(states.size() == expectedDistances.size(),
            "variable yaw output sample count");
        for (std::size_t index = 0; index < states.size(); ++index)
        {
            requireNear(states[index].distance, expectedDistances[index],
                0.0, "variable yaw deterministic spacing");
        }
        require(bitwiseEqual(states.front().position, startingPosition)
                && bitwiseEqual(states.front().frame, canonicalFrame),
            "variable yaw did not preserve exact beginning state");
        require(states.back().distance == sampledProfile.domainEnd,
            "variable yaw terminal traveled distance is not exact");
        require(states[states.size() - 2].distance < states.back().distance,
            "variable yaw duplicated its endpoint");

        const auto coarse = integrateLocalYawRateProfile(
            startingPosition, canonicalFrame, sampledProfile, 20.0
        );
        require(coarse.size() == 2 && coarse.front().distance == 0.0
                && coarse.back().distance == 7.25,
            "spacing larger than profile length did not return begin/end");

        const auto repeated = integrateLocalYawRateProfile(
            startingPosition, canonicalFrame, sampledProfile, 2.0
        );
        require(states.size() == repeated.size(),
            "deterministic repeated state count");
        for (std::size_t index = 0; index < states.size(); ++index)
        {
            require(
                bitwiseEqual(states[index].distance, repeated[index].distance)
                    && bitwiseEqual(
                        states[index].position,
                        repeated[index].position
                    )
                    && bitwiseEqual(
                        states[index].frame,
                        repeated[index].frame
                    ),
                "repeated variable yaw integration changed component bits"
            );
        }

        const auto constantFirst = integrateConstantLocalYawRate(
            startingPosition, canonicalFrame, 8.3, -0.17, 0.6
        );
        const auto constantSecond = integrateConstantLocalYawRate(
            startingPosition, canonicalFrame, 8.3, -0.17, 0.6
        );
        require(constantFirst.size() == constantSecond.size(),
            "deterministic constant state count");
        for (std::size_t index = 0; index < constantFirst.size(); ++index)
        {
            require(
                bitwiseEqual(
                    constantFirst[index].distance,
                    constantSecond[index].distance
                )
                    && bitwiseEqual(
                        constantFirst[index].position,
                        constantSecond[index].position
                    )
                    && bitwiseEqual(
                        constantFirst[index].frame,
                        constantSecond[index].frame
                    ),
                "repeated constant yaw integration changed component bits"
            );
        }
    }

    void testValidation()
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();
        const ScalarTransition valid{
            0.0, 5.0, 0.01, 0.1, TransitionType::Linear
        };

        for (const glm::dvec3 position : {
                 glm::dvec3{nan, 0.0, 0.0},
                 glm::dvec3{0.0, infinity, 0.0},
                 glm::dvec3{0.0, 0.0, -infinity}
             })
        {
            requireThrows<std::invalid_argument>(
                [position]
                {
                    static_cast<void>(integrateConstantLocalYawRate(
                        position, canonicalFrame, 5.0, 0.1, 1.0
                    ));
                },
                "constant yaw non-finite starting position"
            );
            requireThrows<std::invalid_argument>(
                [position, &valid]
                {
                    static_cast<void>(integrateLocalYawRateProfile(
                        position, canonicalFrame, valid, 1.0
                    ));
                },
                "variable yaw non-finite starting position"
            );
        }

        const CurveFrame invalidFrame{
            {2.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };
        requireThrows<std::invalid_argument>(
            [&invalidFrame]
            {
                static_cast<void>(integrateConstantLocalYawRate(
                    {0.0, 0.0, 0.0}, invalidFrame, 5.0, 0.1, 1.0
                ));
            },
            "constant yaw invalid frame"
        );
        requireThrows<std::invalid_argument>(
            [&invalidFrame, &valid]
            {
                static_cast<void>(integrateLocalYawRateProfile(
                    {0.0, 0.0, 0.0}, invalidFrame, valid, 1.0
                ));
            },
            "variable yaw invalid frame"
        );

        for (const double length : {-1.0, nan, infinity})
        {
            requireThrows<std::invalid_argument>(
                [length]
                {
                    static_cast<void>(integrateConstantLocalYawRate(
                        {0.0, 0.0, 0.0}, canonicalFrame,
                        length, 0.1, 1.0
                    ));
                },
                "invalid constant yaw length"
            );
        }
        for (const double rate : {nan, infinity, -infinity})
        {
            requireThrows<std::invalid_argument>(
                [rate]
                {
                    static_cast<void>(integrateConstantLocalYawRate(
                        {0.0, 0.0, 0.0}, canonicalFrame,
                        5.0, rate, 1.0
                    ));
                },
                "invalid constant yaw rate"
            );
        }
        for (const double spacing : {0.0, -1.0, nan, infinity})
        {
            requireThrows<std::invalid_argument>(
                [spacing]
                {
                    static_cast<void>(integrateConstantLocalYawRate(
                        {0.0, 0.0, 0.0}, canonicalFrame,
                        5.0, 0.1, spacing
                    ));
                },
                "invalid constant yaw spacing"
            );
            requireThrows<std::invalid_argument>(
                [spacing, &valid]
                {
                    static_cast<void>(integrateLocalYawRateProfile(
                        {0.0, 0.0, 0.0}, canonicalFrame, valid, spacing
                    ));
                },
                "invalid variable yaw spacing"
            );
        }

        const std::vector<ScalarTransition> invalidTransitions{
            {1.0, 1.0, 0.0, 0.1, TransitionType::Linear},
            {2.0, 1.0, 0.0, 0.1, TransitionType::Linear},
            {nan, 1.0, 0.0, 0.1, TransitionType::Linear},
            {0.0, infinity, 0.0, 0.1, TransitionType::Linear},
            {0.0, 1.0, nan, 0.1, TransitionType::Linear},
            {0.0, 1.0, 0.0, infinity, TransitionType::Linear},
            {0.0, 1.0, 0.0, 0.1, static_cast<TransitionType>(-1)}
        };
        for (const ScalarTransition& transition : invalidTransitions)
        {
            requireThrows<std::invalid_argument>(
                [&transition]
                {
                    static_cast<void>(integrateLocalYawRateProfile(
                        {0.0, 0.0, 0.0},
                        canonicalFrame,
                        transition,
                        1.0
                    ));
                },
                "invalid yaw-rate transition"
            );
        }
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;
    const std::vector<Test> tests{
        {"constant analytic geometry", testConstantAnalyticGeometry},
        {"constant profile equivalence", testConstantProfileEquivalence},
        {
            "variable orientation and area references",
            testVariableOrientationAndAreaReferences
        },
        {
            "progressive, reverse, and sign-changing profiles",
            testProgressiveReverseAndSignChangingProfiles
        },
        {"transformed starting frames", testTransformedStartingFrames},
        {
            "translated domain, sampling, and determinism",
            testTranslatedDomainSamplingAndDeterminism
        },
        {"validation", testValidation}
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
    std::cout << "[METRIC] maximum constant analytic position error: "
              << measurements.maximumConstantPositionError << '\n';
    std::cout << "[METRIC] maximum constant analytic orientation error: "
              << measurements.maximumConstantOrientationError << '\n';
    std::cout << "[METRIC] maximum circle radius error: "
              << measurements.maximumCircleRadiusError << '\n';
    std::cout << "[METRIC] maximum constant-profile position error: "
              << measurements.maximumConstantProfilePositionError << '\n';
    std::cout << "[METRIC] maximum constant-profile orientation error: "
              << measurements.maximumConstantProfileOrientationError << '\n';
    std::cout << "[METRIC] maximum independent-reference position error: "
              << measurements.maximumReferencePositionError << '\n';
    std::cout << "[METRIC] maximum analytic orientation error: "
              << measurements.maximumReferenceOrientationError << '\n';
    std::cout << "[METRIC] maximum frame invariant error: "
              << measurements.maximumFrameInvariantError << '\n';
    std::cout << "[METRIC] maximum unchanged-up-axis error: "
              << measurements.maximumUpAxisError << '\n';
    std::cout << "[METRIC] accumulated yaw: symmetric="
              << measurements.symmetricAccumulatedYaw
              << ", asymmetric="
              << measurements.asymmetricAccumulatedYaw << '\n';
    std::cout << "[METRIC] progressive average curvature: first="
              << measurements.progressiveFirstAverageCurvature
              << ", last="
              << measurements.progressiveLastAverageCurvature << '\n';
    std::cout << "[METRIC] reverse average curvature: first="
              << measurements.reverseFirstAverageCurvature
              << ", last="
              << measurements.reverseLastAverageCurvature << '\n';
    std::cout << "[METRIC] sign-changing endpoint lateral displacement: "
              << measurements.signChangingEndpointLateralDisplacement << '\n';
    std::cout << "[METRIC] transformed-frame plane errors: rolled="
              << measurements.rolledPlaneError
              << ", pitched=" << measurements.pitchedPlaneError << '\n';
    std::cout << "[METRIC] translated-domain differences: position="
              << measurements.translatedPositionDifference
              << ", orientation="
              << measurements.translatedOrientationDifference << '\n';
    std::cout << "[METRIC] quarter-circle endpoint: ("
              << measurements.quarterCircleEndpoint.x << ", "
              << measurements.quarterCircleEndpoint.y << ", "
              << measurements.quarterCircleEndpoint.z << ")\n";
    std::cout << "[METRIC] half-circle endpoint: ("
              << measurements.halfCircleEndpoint.x << ", "
              << measurements.halfCircleEndpoint.y << ", "
              << measurements.halfCircleEndpoint.z << ")\n";
    std::cout << tests.size() << " yaw test groups passed.\n";
    return 0;
}
