#include <quantum/coaster/RiderLocalGeometry.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::integrateLocalPitchRateProfile;
    using quantum::coaster::integrateLocalPitchYawRateProfiles;
    using quantum::coaster::integrateLocalRollPitchYawRateProfiles;
    using quantum::coaster::integrateLocalRollRateProfile;
    using quantum::coaster::integrateLocalYawRateProfile;
    using quantum::coaster::RiderLocalGeometryState;
    using quantum::geometry::applyLocalPitch;
    using quantum::geometry::applyLocalYaw;
    using quantum::geometry::applyRoll;
    using quantum::geometry::CurveFrame;
    using quantum::math::integrateScalarTransition;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

    constexpr double positionTolerance = 5.0e-10;
    constexpr double orientationTolerance = 8.0e-11;
    constexpr std::size_t referenceIntervalCount = 131'072;

    const CurveFrame canonicalFrame{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    const glm::dvec3 startingPosition{2.5, -3.0, 1.25};

    struct Measurements
    {
        double maximumReductionError = 0.0;
        double maximumConstantPositionError = 0.0;
        double maximumConstantFrameError = 0.0;
        double maximumFixedAxisError = 0.0;
        double curvatureError = 0.0;
        double axialAdvanceError = 0.0;
        double radialError = 0.0;
        double nonplanarVolume = 0.0;
        double positiveRollAxisAdvance = 0.0;
        double negativeRollAxisAdvance = 0.0;
        double maximumVariablePositionError = 0.0;
        double maximumVariableFrameError = 0.0;
        double asymmetricTransitionIntegral = 0.0;
        double omegaDirectionDot = 1.0;
        double maximumTransformedError = 0.0;
        double translatedPositionError = 0.0;
        double translatedFrameError = 0.0;
        double coarseReferencePositionError = 0.0;
        double coarseReferenceFrameError = 0.0;
        double fineReferencePositionError = 0.0;
        double fineReferenceFrameError = 0.0;
        double coarseFinePositionDifference = 0.0;
        double coarseFineFrameDifference = 0.0;
        double maximumFrameInvariantError = 0.0;
        double naiveRollPitchYawDifference = 0.0;
        double naiveYawPitchRollDifference = 0.0;
    } measurements;

    [[noreturn]] void fail(const std::string& message)
    {
        throw std::runtime_error(message);
    }

    void require(const bool condition, const std::string& message)
    {
        if (!condition)
        {
            fail(message);
        }
    }

    void requireNear(
        const double actual,
        const double expected,
        const double tolerance,
        const std::string& context
    )
    {
        const double error = std::abs(actual - expected);
        if (!(error <= tolerance))
        {
            fail(
                context + ": error " + std::to_string(error)
                    + " exceeds " + std::to_string(tolerance)
            );
        }
    }

    template<typename Exception, typename Callable>
    void requireThrows(Callable&& callable, const std::string& context)
    {
        try
        {
            std::invoke(std::forward<Callable>(callable));
        }
        catch (const Exception&)
        {
            return;
        }
        catch (const std::exception& exception)
        {
            fail(context + ": wrong exception: " + exception.what());
        }

        fail(context + ": expected exception was not thrown");
    }

    [[nodiscard]] double vectorError(
        const glm::dvec3& first,
        const glm::dvec3& second
    )
    {
        return glm::length(first - second);
    }

    [[nodiscard]] double frameError(
        const CurveFrame& first,
        const CurveFrame& second
    )
    {
        return std::max({
            vectorError(first.tangent, second.tangent),
            vectorError(first.lateral, second.lateral),
            vectorError(first.up, second.up)
        });
    }

    [[nodiscard]] bool bitwiseEqual(
        const double first,
        const double second
    )
    {
        return std::bit_cast<std::uint64_t>(first)
            == std::bit_cast<std::uint64_t>(second);
    }

    [[nodiscard]] bool bitwiseEqual(
        const glm::dvec3& first,
        const glm::dvec3& second
    )
    {
        return bitwiseEqual(first.x, second.x)
            && bitwiseEqual(first.y, second.y)
            && bitwiseEqual(first.z, second.z);
    }

    [[nodiscard]] bool bitwiseEqual(
        const CurveFrame& first,
        const CurveFrame& second
    )
    {
        return bitwiseEqual(first.tangent, second.tangent)
            && bitwiseEqual(first.lateral, second.lateral)
            && bitwiseEqual(first.up, second.up);
    }

    void requireBitwiseEqual(
        const std::vector<RiderLocalGeometryState>& first,
        const std::vector<RiderLocalGeometryState>& second,
        const std::string& context
    )
    {
        require(first.size() == second.size(), context + " state count");
        for (std::size_t index = 0; index < first.size(); ++index)
        {
            require(
                bitwiseEqual(first[index].distance, second[index].distance)
                    && bitwiseEqual(first[index].position, second[index].position)
                    && bitwiseEqual(first[index].frame, second[index].frame),
                context + " component bits"
            );
        }
    }

    void checkState(
        const RiderLocalGeometryState& state,
        const std::string& context
    )
    {
        require(
            std::isfinite(state.position.x)
                && std::isfinite(state.position.y)
                && std::isfinite(state.position.z),
            context + " finite position"
        );
        const CurveFrame& frame = state.frame;
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
            context + " finite frame"
        );
        const double error = std::max({
            std::abs(glm::length(frame.tangent) - 1.0),
            std::abs(glm::length(frame.lateral) - 1.0),
            std::abs(glm::length(frame.up) - 1.0),
            std::abs(glm::dot(frame.tangent, frame.lateral)),
            std::abs(glm::dot(frame.tangent, frame.up)),
            std::abs(glm::dot(frame.lateral, frame.up)),
            glm::length(glm::cross(frame.tangent, frame.lateral) - frame.up)
        });
        measurements.maximumFrameInvariantError = std::max(
            measurements.maximumFrameInvariantError,
            error
        );
        requireNear(error, 0.0, 3.0e-15, context + " frame invariants");
    }

    [[nodiscard]] glm::dvec3 rotateAroundAxis(
        const glm::dvec3& vector,
        const glm::dvec3& unitAxis,
        const double angle
    )
    {
        return std::cos(angle) * vector
            + std::sin(angle) * glm::cross(unitAxis, vector)
            + (1.0 - std::cos(angle))
                * glm::dot(unitAxis, vector) * unitAxis;
    }

    [[nodiscard]] RiderLocalGeometryState analyticConstantState(
        const glm::dvec3& position,
        const CurveFrame& frame,
        const double rollRate,
        const double pitchRate,
        const double yawRate,
        const double distance
    )
    {
        const double angularRate = std::hypot(
            rollRate,
            pitchRate,
            yawRate
        );
        const double angle = angularRate * distance;
        const glm::dvec3 axis = (
            rollRate * frame.tangent
                + pitchRate * frame.lateral
                + yawRate * frame.up
        ) / angularRate;
        const glm::dvec3 turning = glm::cross(axis, frame.tangent);
        const double tangentAxisComponent = glm::dot(axis, frame.tangent);
        const glm::dvec3 integratedPosition = position
            + std::sin(angle) / angularRate * frame.tangent
            + (1.0 - std::cos(angle)) / angularRate * turning
            + (distance - std::sin(angle) / angularRate)
                * tangentAxisComponent * axis;

        return {
            distance,
            integratedPosition,
            {
                rotateAroundAxis(frame.tangent, axis, angle),
                rotateAroundAxis(frame.lateral, axis, angle),
                rotateAroundAxis(frame.up, axis, angle)
            }
        };
    }

    struct ReferenceState
    {
        glm::dvec3 position;
        CurveFrame frame;
    };

    struct ReferenceDerivative
    {
        glm::dvec3 position;
        glm::dvec3 tangent;
        glm::dvec3 lateral;
        glm::dvec3 up;
    };

    [[nodiscard]] glm::dvec3 referenceRates(
        const ScalarTransition& roll,
        const ScalarTransition& pitch,
        const ScalarTransition& yaw,
        const double traveledDistance,
        const double profileLength
    )
    {
        const double boundedDistance = std::clamp(
            traveledDistance,
            0.0,
            profileLength
        );
        const auto evaluate = [boundedDistance, profileLength](
            const ScalarTransition& transition
        )
        {
            const double domainValue = boundedDistance == profileLength
                ? transition.domainEnd
                : transition.domainBegin + boundedDistance;
            return quantum::math::evaluateScalarTransition(
                transition,
                domainValue
            );
        };
        return {evaluate(roll), evaluate(pitch), evaluate(yaw)};
    }

    [[nodiscard]] ReferenceDerivative referenceDerivative(
        const ReferenceState& state,
        const glm::dvec3& rates
    )
    {
        const double roll = rates.x;
        const double pitch = rates.y;
        const double yaw = rates.z;
        return {
            state.frame.tangent,
            yaw * state.frame.lateral - pitch * state.frame.up,
            -yaw * state.frame.tangent + roll * state.frame.up,
            pitch * state.frame.tangent - roll * state.frame.lateral
        };
    }

    [[nodiscard]] ReferenceState addReferenceDerivative(
        const ReferenceState& state,
        const ReferenceDerivative& derivative,
        const double scale
    )
    {
        return {
            state.position + scale * derivative.position,
            {
                state.frame.tangent + scale * derivative.tangent,
                state.frame.lateral + scale * derivative.lateral,
                state.frame.up + scale * derivative.up
            }
        };
    }

    [[nodiscard]] ReferenceState highAccuracyReference(
        const glm::dvec3& position,
        const CurveFrame& frame,
        const ScalarTransition& roll,
        const ScalarTransition& pitch,
        const ScalarTransition& yaw,
        const double traveledEnd
    )
    {
        if (traveledEnd == 0.0)
        {
            return {position, frame};
        }

        const double profileLength = roll.domainEnd - roll.domainBegin;
        const std::size_t intervalCount = std::max<std::size_t>(
            1,
            static_cast<std::size_t>(std::ceil(
                static_cast<double>(referenceIntervalCount)
                    * traveledEnd / profileLength
            ))
        );
        const double step = traveledEnd
            / static_cast<double>(intervalCount);
        ReferenceState state{position, frame};

        for (std::size_t index = 0; index < intervalCount; ++index)
        {
            const double distance = static_cast<double>(index) * step;
            const glm::dvec3 rates1 = referenceRates(
                roll, pitch, yaw, distance, profileLength
            );
            const ReferenceDerivative k1 = referenceDerivative(state, rates1);
            const glm::dvec3 rates2 = referenceRates(
                roll, pitch, yaw, distance + 0.5 * step, profileLength
            );
            const ReferenceDerivative k2 = referenceDerivative(
                addReferenceDerivative(state, k1, 0.5 * step),
                rates2
            );
            const ReferenceDerivative k3 = referenceDerivative(
                addReferenceDerivative(state, k2, 0.5 * step),
                rates2
            );
            const glm::dvec3 rates4 = referenceRates(
                roll, pitch, yaw, distance + step, profileLength
            );
            const ReferenceDerivative k4 = referenceDerivative(
                addReferenceDerivative(state, k3, step),
                rates4
            );

            state.position += step / 6.0 * (
                k1.position + 2.0 * k2.position
                    + 2.0 * k3.position + k4.position
            );
            state.frame.tangent += step / 6.0 * (
                k1.tangent + 2.0 * k2.tangent
                    + 2.0 * k3.tangent + k4.tangent
            );
            state.frame.lateral += step / 6.0 * (
                k1.lateral + 2.0 * k2.lateral
                    + 2.0 * k3.lateral + k4.lateral
            );
            state.frame.up += step / 6.0 * (
                k1.up + 2.0 * k2.up
                    + 2.0 * k3.up + k4.up
            );
        }

        return state;
    }

    void compareWithReference(
        const RiderLocalGeometryState& actual,
        const ReferenceState& expected,
        const std::string& context
    )
    {
        const double positionError = vectorError(
            actual.position,
            expected.position
        );
        const double orientationError = frameError(actual.frame, expected.frame);
        measurements.maximumVariablePositionError = std::max(
            measurements.maximumVariablePositionError,
            positionError
        );
        measurements.maximumVariableFrameError = std::max(
            measurements.maximumVariableFrameError,
            orientationError
        );
        requireNear(positionError, 0.0, positionTolerance, context + " position");
        requireNear(
            orientationError,
            0.0,
            orientationTolerance,
            context + " frame"
        );
    }

    void testReductionsAndTwoAxisCoupling()
    {
        constexpr double length = 7.5;
        const ScalarTransition zeroRoll{
            0.0, length, 0.0, 0.0, TransitionType::CosineEaseInOut
        };
        const ScalarTransition zeroPitch{
            0.0, length, 0.0, 0.0, TransitionType::SeventhOrderSmoothstep
        };
        const ScalarTransition zeroYaw{
            0.0, length, 0.0, 0.0, TransitionType::QuadraticEaseIn
        };
        const ScalarTransition roll{
            0.0, length, -0.08, 0.16, TransitionType::CosineEaseInOut
        };
        const ScalarTransition pitch{
            0.0, length, 0.04, 0.18, TransitionType::SeventhOrderSmoothstep
        };
        const ScalarTransition yaw{
            0.0, length, 0.15, -0.07, TransitionType::QuadraticEaseIn
        };

        requireBitwiseEqual(
            integrateLocalRollPitchYawRateProfiles(
                startingPosition, canonicalFrame,
                zeroRoll, zeroPitch, zeroYaw, 0.73
            ),
            integrateLocalPitchYawRateProfiles(
                startingPosition, canonicalFrame, zeroPitch, zeroYaw, 0.73
            ),
            "all-zero reduction"
        );
        requireBitwiseEqual(
            integrateLocalRollPitchYawRateProfiles(
                startingPosition, canonicalFrame,
                roll, zeroPitch, zeroYaw, 0.73
            ),
            integrateLocalRollRateProfile(
                startingPosition, canonicalFrame, roll, 0.73
            ),
            "roll-only reduction"
        );
        requireBitwiseEqual(
            integrateLocalRollPitchYawRateProfiles(
                startingPosition, canonicalFrame,
                zeroRoll, pitch, zeroYaw, 0.73
            ),
            integrateLocalPitchRateProfile(
                startingPosition, canonicalFrame, pitch, 0.73
            ),
            "pitch-only reduction"
        );
        requireBitwiseEqual(
            integrateLocalRollPitchYawRateProfiles(
                startingPosition, canonicalFrame,
                zeroRoll, zeroPitch, yaw, 0.73
            ),
            integrateLocalYawRateProfile(
                startingPosition, canonicalFrame, yaw, 0.73
            ),
            "yaw-only reduction"
        );
        requireBitwiseEqual(
            integrateLocalRollPitchYawRateProfiles(
                startingPosition, canonicalFrame,
                zeroRoll, pitch, yaw, 0.73
            ),
            integrateLocalPitchYawRateProfiles(
                startingPosition, canonicalFrame, pitch, yaw, 0.73
            ),
            "pitch/yaw reduction"
        );

        const auto rollPitch = integrateLocalRollPitchYawRateProfiles(
            startingPosition, canonicalFrame,
            roll, pitch, zeroYaw, 0.91
        );
        compareWithReference(
            rollPitch.back(),
            highAccuracyReference(
                startingPosition, canonicalFrame,
                roll, pitch, zeroYaw, length
            ),
            "simultaneous roll/pitch"
        );
        const auto rollYaw = integrateLocalRollPitchYawRateProfiles(
            startingPosition, canonicalFrame,
            roll, zeroPitch, yaw, 0.91
        );
        compareWithReference(
            rollYaw.back(),
            highAccuracyReference(
                startingPosition, canonicalFrame,
                roll, zeroPitch, yaw, length
            ),
            "simultaneous roll/yaw"
        );
    }

    void testConstantAnalyticHelix()
    {
        constexpr double length = 12.0;
        constexpr double rollRate = 0.09;
        constexpr double pitchRate = 0.17;
        constexpr double yawRate = -0.11;
        const ScalarTransition roll{
            0.0, length, rollRate, rollRate, TransitionType::CosineEaseInOut
        };
        const ScalarTransition pitch{
            0.0, length, pitchRate, pitchRate,
            TransitionType::SeventhOrderSmoothstep
        };
        const ScalarTransition yaw{
            0.0, length, yawRate, yawRate, TransitionType::QuadraticEaseIn
        };
        const auto states = integrateLocalRollPitchYawRateProfiles(
            startingPosition, canonicalFrame, roll, pitch, yaw, 0.25
        );
        const double angularRate = std::hypot(
            rollRate, pitchRate, yawRate
        );
        const double curvature = std::hypot(pitchRate, yawRate);
        const glm::dvec3 spatialAxis = glm::normalize(glm::dvec3{
            rollRate, pitchRate, yawRate
        });
        const glm::dvec3 turning = glm::cross(
            spatialAxis,
            canonicalFrame.tangent
        );
        const glm::dvec3 cylinderAxisOrigin =
            startingPosition + turning / angularRate;
        const double expectedRadius = curvature
            / (angularRate * angularRate);

        for (const RiderLocalGeometryState& state : states)
        {
            const RiderLocalGeometryState expected = analyticConstantState(
                startingPosition,
                canonicalFrame,
                rollRate,
                pitchRate,
                yawRate,
                state.distance
            );
            measurements.maximumConstantPositionError = std::max(
                measurements.maximumConstantPositionError,
                vectorError(state.position, expected.position)
            );
            measurements.maximumConstantFrameError = std::max(
                measurements.maximumConstantFrameError,
                frameError(state.frame, expected.frame)
            );
            const glm::dvec3 evolvedAxis =
                rollRate * state.frame.tangent
                + pitchRate * state.frame.lateral
                + yawRate * state.frame.up;
            measurements.maximumFixedAxisError = std::max(
                measurements.maximumFixedAxisError,
                vectorError(evolvedAxis / angularRate, spatialAxis)
            );
            const double axialAdvance = glm::dot(
                state.position - startingPosition,
                spatialAxis
            );
            measurements.axialAdvanceError = std::max(
                measurements.axialAdvanceError,
                std::abs(
                    axialAdvance - state.distance * rollRate / angularRate
                )
            );
            const glm::dvec3 axisPoint = cylinderAxisOrigin
                + state.distance * rollRate / angularRate * spatialAxis;
            measurements.radialError = std::max(
                measurements.radialError,
                std::abs(glm::length(state.position - axisPoint)
                    - expectedRadius)
            );
            checkState(state, "constant helix");
        }
        requireNear(measurements.maximumConstantPositionError, 0.0, 2.0e-14,
            "constant analytic position");
        requireNear(measurements.maximumConstantFrameError, 0.0, 3.0e-15,
            "constant analytic frame");
        requireNear(measurements.maximumFixedAxisError, 0.0, 2.0e-15,
            "constant spatial axis");
        requireNear(measurements.axialAdvanceError, 0.0, 3.0e-15,
            "helix axial advance");
        requireNear(measurements.radialError, 0.0, 2.0e-14,
            "helix radial behavior");

        constexpr double derivativeLength = 2.0e-5;
        const ScalarTransition shortRoll{
            0.0, derivativeLength,
            rollRate, rollRate, TransitionType::Linear
        };
        const ScalarTransition shortPitch{
            0.0, derivativeLength,
            pitchRate, pitchRate, TransitionType::Linear
        };
        const ScalarTransition shortYaw{
            0.0, derivativeLength,
            yawRate, yawRate, TransitionType::Linear
        };
        const auto derivativeStates = integrateLocalRollPitchYawRateProfiles(
            startingPosition,
            canonicalFrame,
            shortRoll,
            shortPitch,
            shortYaw,
            derivativeLength
        );
        const double measuredCurvature = glm::length(
            derivativeStates.back().frame.tangent
                - derivativeStates.front().frame.tangent
        ) / derivativeLength;
        measurements.curvatureError = std::abs(
            measuredCurvature - curvature
        );
        requireNear(measurements.curvatureError, 0.0, 2.0e-11,
            "constant helix curvature characterization");

        const glm::dvec3 first = states[12].position - states[0].position;
        const glm::dvec3 second = states[24].position - states[0].position;
        const glm::dvec3 third = states[36].position - states[0].position;
        measurements.nonplanarVolume = glm::dot(
            first,
            glm::cross(second, third)
        );
        require(std::abs(measurements.nonplanarVolume) > 0.1,
            "constant all-three centerline was unexpectedly planar");

        measurements.positiveRollAxisAdvance = glm::dot(
            states.back().position - startingPosition,
            spatialAxis
        );
        const ScalarTransition negativeRoll{
            0.0, length, -rollRate, -rollRate, TransitionType::Linear
        };
        const auto negativeStates = integrateLocalRollPitchYawRateProfiles(
            startingPosition, canonicalFrame,
            negativeRoll, pitch, yaw, length
        );
        const glm::dvec3 negativeAxis = glm::normalize(glm::dvec3{
            -rollRate, pitchRate, yawRate
        });
        measurements.negativeRollAxisAdvance = glm::dot(
            negativeStates.back().position - startingPosition,
            negativeAxis
        );
        require(measurements.positiveRollAxisAdvance > 0.0
                && measurements.negativeRollAxisAdvance < 0.0,
            "roll sign did not control signed helix axial advance");
    }

    void testVariableReferenceSignCrossingsAndTransitions()
    {
        constexpr double length = 10.0;
        const ScalarTransition roll{
            0.0, length, -0.14, 0.19, TransitionType::CosineEaseInOut
        };
        const ScalarTransition pitch{
            0.0, length, 0.05, 0.18,
            TransitionType::SeventhOrderSmoothstep
        };
        const ScalarTransition yaw{
            0.0, length, 0.16, -0.09, TransitionType::QuadraticEaseIn
        };
        const auto states = integrateLocalRollPitchYawRateProfiles(
            startingPosition, canonicalFrame, roll, pitch, yaw, 0.8
        );
        compareWithReference(
            states.back(),
            highAccuracyReference(
                startingPosition, canonicalFrame,
                roll, pitch, yaw, length
            ),
            "variable all-three independent reference"
        );
        for (const RiderLocalGeometryState& state : states)
        {
            checkState(state, "variable all-three");
        }

        measurements.asymmetricTransitionIntegral = integrateScalarTransition(
            yaw,
            yaw.domainBegin,
            yaw.domainEnd
        );
        const double midpointArea = length
            * 0.5 * (yaw.valueBegin + yaw.valueEnd);
        require(std::abs(
                measurements.asymmetricTransitionIntegral - midpointArea
            )
                > 0.1,
            "asymmetric transition unexpectedly behaved like half-area easing");

        const ScalarTransition crossingRoll{
            0.0, length, 0.08, 0.16, TransitionType::SineEaseOut
        };
        const ScalarTransition crossingPitch{
            0.0, length, 0.19, -0.17, TransitionType::Linear
        };
        const ScalarTransition activeYaw{
            0.0, length, 0.13, 0.04, TransitionType::CubicEaseIn
        };
        const auto crossingStates = integrateLocalRollPitchYawRateProfiles(
            startingPosition,
            canonicalFrame,
            crossingRoll,
            crossingPitch,
            activeYaw,
            1.1
        );
        compareWithReference(
            crossingStates.back(),
            highAccuracyReference(
                startingPosition,
                canonicalFrame,
                crossingRoll,
                crossingPitch,
                activeYaw,
                length
            ),
            "pitch sign crossing with active roll/yaw"
        );
        const glm::dvec3 beginningOmega{
            crossingRoll.valueBegin,
            crossingPitch.valueBegin,
            activeYaw.valueBegin
        };
        const glm::dvec3 endingOmega{
            crossingRoll.valueEnd,
            crossingPitch.valueEnd,
            activeYaw.valueEnd
        };
        measurements.omegaDirectionDot = glm::dot(
            glm::normalize(beginningOmega),
            glm::normalize(endingOmega)
        );
        require(measurements.omegaDirectionDot < 0.0,
            "instantaneous omega did not change direction significantly");

        constexpr std::array transitionTypes{
            TransitionType::Linear,
            TransitionType::Smoothstep,
            TransitionType::Smootherstep,
            TransitionType::SeventhOrderSmoothstep,
            TransitionType::CosineEaseInOut,
            TransitionType::SineEaseIn,
            TransitionType::SineEaseOut,
            TransitionType::QuadraticEaseIn,
            TransitionType::QuadraticEaseOut,
            TransitionType::QuadraticEaseInOut,
            TransitionType::CubicEaseIn,
            TransitionType::CubicEaseOut,
            TransitionType::CubicEaseInOut,
            TransitionType::QuarticEaseIn,
            TransitionType::QuarticEaseOut,
            TransitionType::QuarticEaseInOut,
            TransitionType::QuinticEaseIn,
            TransitionType::QuinticEaseOut,
            TransitionType::QuinticEaseInOut
        };
        for (const TransitionType transitionType : transitionTypes)
        {
            const ScalarTransition presetRoll{
                0.0, 1.0, 0.01, 0.03, transitionType
            };
            const ScalarTransition presetPitch{
                0.0, 1.0, 0.02, 0.04, TransitionType::Linear
            };
            const ScalarTransition presetYaw{
                0.0, 1.0, -0.03, 0.01, TransitionType::SineEaseOut
            };
            const auto presetStates = integrateLocalRollPitchYawRateProfiles(
                startingPosition,
                canonicalFrame,
                presetRoll,
                presetPitch,
                presetYaw,
                2.0
            );
            require(presetStates.size() == 2,
                "transition preset produced wrong sample count");
            checkState(presetStates.back(), "transition preset");
        }
    }

    [[nodiscard]] glm::dvec3 coordinatesInFrame(
        const glm::dvec3& vector,
        const CurveFrame& frame
    )
    {
        return {
            glm::dot(vector, frame.tangent),
            glm::dot(vector, frame.lateral),
            glm::dot(vector, frame.up)
        };
    }

    void compareTransformedRun(
        const std::vector<RiderLocalGeometryState>& canonical,
        const glm::dvec3& canonicalStartPosition,
        const CurveFrame& canonicalStartFrame,
        const std::vector<RiderLocalGeometryState>& transformed,
        const glm::dvec3& transformedStartPosition,
        const CurveFrame& transformedStartFrame,
        const std::string& context
    )
    {
        require(canonical.size() == transformed.size(), context + " state count");
        for (std::size_t index = 0; index < canonical.size(); ++index)
        {
            double error = vectorError(
                coordinatesInFrame(
                    canonical[index].position - canonicalStartPosition,
                    canonicalStartFrame
                ),
                coordinatesInFrame(
                    transformed[index].position - transformedStartPosition,
                    transformedStartFrame
                )
            );
            error = std::max(error, vectorError(
                coordinatesInFrame(
                    canonical[index].frame.tangent,
                    canonicalStartFrame
                ),
                coordinatesInFrame(
                    transformed[index].frame.tangent,
                    transformedStartFrame
                )
            ));
            error = std::max(error, vectorError(
                coordinatesInFrame(
                    canonical[index].frame.lateral,
                    canonicalStartFrame
                ),
                coordinatesInFrame(
                    transformed[index].frame.lateral,
                    transformedStartFrame
                )
            ));
            error = std::max(error, vectorError(
                coordinatesInFrame(
                    canonical[index].frame.up,
                    canonicalStartFrame
                ),
                coordinatesInFrame(
                    transformed[index].frame.up,
                    transformedStartFrame
                )
            ));
            measurements.maximumTransformedError = std::max(
                measurements.maximumTransformedError,
                error
            );
        }
        requireNear(measurements.maximumTransformedError, 0.0, 8.0e-12,
            context);
    }

    void testStartingFrameAndRigidTransformInvariance()
    {
        constexpr double length = 7.0;
        const ScalarTransition roll{
            0.0, length, -0.12, 0.09, TransitionType::CosineEaseInOut
        };
        const ScalarTransition pitch{
            0.0, length, 0.03, 0.17,
            TransitionType::SeventhOrderSmoothstep
        };
        const ScalarTransition yaw{
            0.0, length, 0.14, -0.06, TransitionType::QuadraticEaseIn
        };
        const auto canonical = integrateLocalRollPitchYawRateProfiles(
            startingPosition, canonicalFrame, roll, pitch, yaw, 0.7
        );
        const CurveFrame generalFrame = applyRoll(
            applyLocalYaw(
                applyLocalPitch(canonicalFrame, 0.47),
                -0.39
            ),
            0.63
        );
        const std::vector<CurveFrame> transformedFrames{
            applyRoll(canonicalFrame, 0.63),
            applyLocalPitch(canonicalFrame, 0.47),
            applyLocalYaw(canonicalFrame, -0.39),
            generalFrame
        };
        for (std::size_t index = 0; index < transformedFrames.size(); ++index)
        {
            const auto transformed = integrateLocalRollPitchYawRateProfiles(
                startingPosition,
                transformedFrames[index],
                roll,
                pitch,
                yaw,
                0.7
            );
            compareTransformedRun(
                canonical,
                startingPosition,
                canonicalFrame,
                transformed,
                startingPosition,
                transformedFrames[index],
                "starting-frame transform " + std::to_string(index)
            );
        }

        const glm::dvec3 transformedPosition{-11.0, 4.25, 8.0};
        const auto rigidlyTransformed = integrateLocalRollPitchYawRateProfiles(
            transformedPosition,
            generalFrame,
            roll,
            pitch,
            yaw,
            0.7
        );
        compareTransformedRun(
            canonical,
            startingPosition,
            canonicalFrame,
            rigidlyTransformed,
            transformedPosition,
            generalFrame,
            "rigid position/frame transform"
        );
    }

    void testDomainSamplingAccuracyAndDeterminism()
    {
        constexpr double length = 7.25;
        const ScalarTransition roll{
            0.0, length, -0.09, 0.13, TransitionType::CosineEaseInOut
        };
        const ScalarTransition pitch{
            0.0, length, 0.02, 0.17,
            TransitionType::SeventhOrderSmoothstep
        };
        const ScalarTransition yaw{
            0.0, length, -0.12, 0.08, TransitionType::QuadraticEaseIn
        };
        const ScalarTransition translatedRoll{
            100.0, 100.0 + length,
            roll.valueBegin, roll.valueEnd, roll.transitionType
        };
        const ScalarTransition translatedPitch{
            100.0, 100.0 + length,
            pitch.valueBegin, pitch.valueEnd, pitch.transitionType
        };
        const ScalarTransition translatedYaw{
            100.0, 100.0 + length,
            yaw.valueBegin, yaw.valueEnd, yaw.transitionType
        };
        const auto origin = integrateLocalRollPitchYawRateProfiles(
            startingPosition, canonicalFrame, roll, pitch, yaw, 2.0
        );
        const auto translated = integrateLocalRollPitchYawRateProfiles(
            startingPosition,
            canonicalFrame,
            translatedRoll,
            translatedPitch,
            translatedYaw,
            2.0
        );
        require(origin.size() == translated.size(),
            "translated-domain state count");
        for (std::size_t index = 0; index < origin.size(); ++index)
        {
            require(origin[index].distance == translated[index].distance,
                "translated-domain traveled distance");
            measurements.translatedPositionError = std::max(
                measurements.translatedPositionError,
                vectorError(origin[index].position, translated[index].position)
            );
            measurements.translatedFrameError = std::max(
                measurements.translatedFrameError,
                frameError(origin[index].frame, translated[index].frame)
            );
        }
        requireNear(measurements.translatedPositionError, 0.0, 3.0e-13,
            "translated-domain positions");
        requireNear(measurements.translatedFrameError, 0.0, 3.0e-13,
            "translated-domain frames");

        const std::vector<double> expectedDistances{0.0, 2.0, 4.0, 6.0, length};
        require(origin.size() == expectedDistances.size(),
            "three-axis output sample count");
        for (std::size_t index = 0; index < origin.size(); ++index)
        {
            requireNear(origin[index].distance, expectedDistances[index], 0.0,
                "deterministic output spacing");
        }
        require(bitwiseEqual(origin.front().distance, 0.0)
                && bitwiseEqual(origin.front().position, startingPosition)
                && bitwiseEqual(origin.front().frame, canonicalFrame),
            "three-axis beginning state was not exact");
        require(origin.back().distance == length,
            "terminal traveled distance was not exact");
        require(origin[origin.size() - 2].distance < origin.back().distance,
            "terminal state was duplicated");

        const ReferenceState reference = highAccuracyReference(
            startingPosition,
            canonicalFrame,
            roll,
            pitch,
            yaw,
            length
        );
        const auto coarse = integrateLocalRollPitchYawRateProfiles(
            startingPosition, canonicalFrame, roll, pitch, yaw, 20.0
        );
        require(coarse.size() == 2
                && coarse.front().distance == 0.0
                && coarse.back().distance == length,
            "spacing larger than section did not return begin/end");
        measurements.coarseReferencePositionError = vectorError(
            coarse.back().position,
            reference.position
        );
        measurements.coarseReferenceFrameError = frameError(
            coarse.back().frame,
            reference.frame
        );
        const auto fine = integrateLocalRollPitchYawRateProfiles(
            startingPosition, canonicalFrame, roll, pitch, yaw, 0.2
        );
        measurements.fineReferencePositionError = vectorError(
            fine.back().position,
            reference.position
        );
        measurements.fineReferenceFrameError = frameError(
            fine.back().frame,
            reference.frame
        );
        measurements.coarseFinePositionDifference = vectorError(
            coarse.back().position,
            fine.back().position
        );
        measurements.coarseFineFrameDifference = frameError(
            coarse.back().frame,
            fine.back().frame
        );
        requireNear(measurements.coarseReferencePositionError, 0.0,
            positionTolerance, "coarse-output reference position");
        requireNear(measurements.coarseReferenceFrameError, 0.0,
            orientationTolerance, "coarse-output reference frame");
        requireNear(measurements.fineReferencePositionError, 0.0,
            positionTolerance, "fine-output reference position");
        requireNear(measurements.fineReferenceFrameError, 0.0,
            orientationTolerance, "fine-output reference frame");
        requireNear(measurements.coarseFinePositionDifference, 0.0,
            positionTolerance, "coarse/fine position agreement");
        requireNear(measurements.coarseFineFrameDifference, 0.0,
            orientationTolerance, "coarse/fine frame agreement");

        const auto repeated = integrateLocalRollPitchYawRateProfiles(
            startingPosition, canonicalFrame, roll, pitch, yaw, 2.0
        );
        requireBitwiseEqual(origin, repeated, "three-axis determinism");
    }

    void testNaiveEulerDifference()
    {
        constexpr double length = 8.0;
        const ScalarTransition roll{
            0.0, length, -0.15, 0.18, TransitionType::CosineEaseInOut
        };
        const ScalarTransition pitch{
            0.0, length, 0.04, 0.19,
            TransitionType::SeventhOrderSmoothstep
        };
        const ScalarTransition yaw{
            0.0, length, 0.17, -0.08, TransitionType::QuadraticEaseIn
        };
        const auto actual = integrateLocalRollPitchYawRateProfiles(
            startingPosition, canonicalFrame, roll, pitch, yaw, length
        );
        const double accumulatedRoll = integrateScalarTransition(
            roll, roll.domainBegin, roll.domainEnd
        );
        const double accumulatedPitch = integrateScalarTransition(
            pitch, pitch.domainBegin, pitch.domainEnd
        );
        const double accumulatedYaw = integrateScalarTransition(
            yaw, yaw.domainBegin, yaw.domainEnd
        );
        const CurveFrame rollPitchYaw = applyLocalYaw(
            applyLocalPitch(
                applyRoll(canonicalFrame, accumulatedRoll),
                accumulatedPitch
            ),
            accumulatedYaw
        );
        const CurveFrame yawPitchRoll = applyRoll(
            applyLocalPitch(
                applyLocalYaw(canonicalFrame, accumulatedYaw),
                accumulatedPitch
            ),
            accumulatedRoll
        );
        measurements.naiveRollPitchYawDifference = frameError(
            actual.back().frame,
            rollPitchYaw
        );
        measurements.naiveYawPitchRollDifference = frameError(
            actual.back().frame,
            yawPitchRoll
        );
        require(measurements.naiveRollPitchYawDifference > 0.05,
            "simultaneous result matched naive roll/pitch/yaw ordering");
        require(measurements.naiveYawPitchRollDifference > 0.05,
            "simultaneous result matched naive yaw/pitch/roll ordering");
    }

    void testValidation()
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();
        const ScalarTransition validRoll{
            0.0, 5.0, -0.03, 0.11, TransitionType::CosineEaseInOut
        };
        const ScalarTransition validPitch{
            0.0, 5.0, 0.01, 0.1, TransitionType::SeventhOrderSmoothstep
        };
        const ScalarTransition validYaw{
            0.0, 5.0, -0.02, 0.08, TransitionType::QuadraticEaseIn
        };

        for (const glm::dvec3 position : {
                 glm::dvec3{nan, 0.0, 0.0},
                 glm::dvec3{0.0, infinity, 0.0},
                 glm::dvec3{0.0, 0.0, -infinity}
             })
        {
            requireThrows<std::invalid_argument>(
                [position, &validRoll, &validPitch, &validYaw]
                {
                    static_cast<void>(integrateLocalRollPitchYawRateProfiles(
                        position,
                        canonicalFrame,
                        validRoll,
                        validPitch,
                        validYaw,
                        1.0
                    ));
                },
                "three-axis non-finite starting position"
            );
        }

        const CurveFrame invalidFrame{
            {2.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };
        requireThrows<std::invalid_argument>(
            [&invalidFrame, &validRoll, &validPitch, &validYaw]
            {
                static_cast<void>(integrateLocalRollPitchYawRateProfiles(
                    startingPosition,
                    invalidFrame,
                    validRoll,
                    validPitch,
                    validYaw,
                    1.0
                ));
            },
            "three-axis invalid starting frame"
        );

        for (const double spacing : {0.0, -1.0, nan, infinity})
        {
            requireThrows<std::invalid_argument>(
                [spacing, &validRoll, &validPitch, &validYaw]
                {
                    static_cast<void>(integrateLocalRollPitchYawRateProfiles(
                        startingPosition,
                        canonicalFrame,
                        validRoll,
                        validPitch,
                        validYaw,
                        spacing
                    ));
                },
                "three-axis invalid integration spacing"
            );
        }

        const std::vector<ScalarTransition> invalidTransitions{
            {1.0, 1.0, 0.0, 0.1, TransitionType::Linear},
            {2.0, 1.0, 0.0, 0.1, TransitionType::Linear},
            {nan, 5.0, 0.0, 0.1, TransitionType::Linear},
            {0.0, infinity, 0.0, 0.1, TransitionType::Linear},
            {0.0, 5.0, nan, 0.1, TransitionType::Linear},
            {0.0, 5.0, 0.0, infinity, TransitionType::Linear},
            {0.0, 5.0, 0.0, 0.1, static_cast<TransitionType>(-1)}
        };
        for (const ScalarTransition& invalid : invalidTransitions)
        {
            requireThrows<std::invalid_argument>(
                [&invalid, &validPitch, &validYaw]
                {
                    static_cast<void>(integrateLocalRollPitchYawRateProfiles(
                        startingPosition, canonicalFrame,
                        invalid, validPitch, validYaw, 1.0
                    ));
                },
                "invalid roll transition"
            );
            requireThrows<std::invalid_argument>(
                [&invalid, &validRoll, &validYaw]
                {
                    static_cast<void>(integrateLocalRollPitchYawRateProfiles(
                        startingPosition, canonicalFrame,
                        validRoll, invalid, validYaw, 1.0
                    ));
                },
                "invalid pitch transition"
            );
            requireThrows<std::invalid_argument>(
                [&invalid, &validRoll, &validPitch]
                {
                    static_cast<void>(integrateLocalRollPitchYawRateProfiles(
                        startingPosition, canonicalFrame,
                        validRoll, validPitch, invalid, 1.0
                    ));
                },
                "invalid yaw transition"
            );
        }

        const ScalarTransition mismatchedBegin{
            std::nextafter(0.0, -1.0),
            5.0,
            validYaw.valueBegin,
            validYaw.valueEnd,
            validYaw.transitionType
        };
        const ScalarTransition mismatchedEnd{
            0.0,
            std::nextafter(5.0, 6.0),
            validYaw.valueBegin,
            validYaw.valueEnd,
            validYaw.transitionType
        };
        for (const ScalarTransition& mismatch : {
                 mismatchedBegin,
                 mismatchedEnd
             })
        {
            requireThrows<std::invalid_argument>(
                [&validRoll, &validPitch, &mismatch]
                {
                    static_cast<void>(integrateLocalRollPitchYawRateProfiles(
                        startingPosition,
                        canonicalFrame,
                        validRoll,
                        validPitch,
                        mismatch,
                        1.0
                    ));
                },
                "exact shared-domain validation"
            );
        }

        const ScalarTransition hugeRoll{
            0.0, 4.0,
            std::numeric_limits<double>::max() / 2.0,
            std::numeric_limits<double>::max() / 2.0,
            TransitionType::Linear
        };
        const ScalarTransition constantPitch{
            0.0, 4.0, 0.1, 0.1, TransitionType::Linear
        };
        const ScalarTransition constantYaw{
            0.0, 4.0, 0.1, 0.1, TransitionType::Linear
        };
        requireThrows<std::domain_error>(
            [&hugeRoll, &constantPitch, &constantYaw]
            {
                static_cast<void>(integrateLocalRollPitchYawRateProfiles(
                    startingPosition,
                    canonicalFrame,
                    hugeRoll,
                    constantPitch,
                    constantYaw,
                    1.0
                ));
            },
            "non-representable constant three-axis angle"
        );
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;
    const std::vector<Test> tests{
        {"reductions and two-axis coupling", testReductionsAndTwoAxisCoupling},
        {"constant analytic helix", testConstantAnalyticHelix},
        {
            "variable reference, sign crossings, and transitions",
            testVariableReferenceSignCrossingsAndTransitions
        },
        {
            "starting-frame and rigid-transform invariance",
            testStartingFrameAndRigidTransformInvariance
        },
        {
            "domain, sampling, accuracy, and determinism",
            testDomainSamplingAccuracyAndDeterminism
        },
        {"naive Euler difference", testNaiveEulerDifference},
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
        std::cerr << failures << " three-axis test group(s) failed.\n";
        return 1;
    }

    std::cout << std::setprecision(17);
    std::cout << "[METRIC] maximum reduction error: "
              << measurements.maximumReductionError << '\n';
    std::cout << "[METRIC] constant analytic errors: position="
              << measurements.maximumConstantPositionError
              << ", frame=" << measurements.maximumConstantFrameError << '\n';
    std::cout << "[METRIC] constant fixed-axis error: "
              << measurements.maximumFixedAxisError << '\n';
    std::cout << "[METRIC] helix characterization: curvature error="
              << measurements.curvatureError
              << ", axial error=" << measurements.axialAdvanceError
              << ", radial error=" << measurements.radialError
              << ", signed nonplanar volume="
              << measurements.nonplanarVolume << '\n';
    std::cout << "[METRIC] signed axial advance: positive roll="
              << measurements.positiveRollAxisAdvance
              << ", negative roll="
              << measurements.negativeRollAxisAdvance << '\n';
    std::cout << "[METRIC] variable RK4 reference errors: position="
              << measurements.maximumVariablePositionError
              << ", frame=" << measurements.maximumVariableFrameError << '\n';
    std::cout << "[METRIC] asymmetric transition integral: "
              << measurements.asymmetricTransitionIntegral << '\n';
    std::cout << "[METRIC] endpoint omega direction dot: "
              << measurements.omegaDirectionDot << '\n';
    std::cout << "[METRIC] maximum transformed-start error: "
              << measurements.maximumTransformedError << '\n';
    std::cout << "[METRIC] translated-domain errors: position="
              << measurements.translatedPositionError
              << ", frame=" << measurements.translatedFrameError << '\n';
    std::cout << "[METRIC] coarse-output reference errors: position="
              << measurements.coarseReferencePositionError
              << ", frame=" << measurements.coarseReferenceFrameError << '\n';
    std::cout << "[METRIC] fine-output reference errors: position="
              << measurements.fineReferencePositionError
              << ", frame=" << measurements.fineReferenceFrameError << '\n';
    std::cout << "[METRIC] coarse/fine differences: position="
              << measurements.coarseFinePositionDifference
              << ", frame=" << measurements.coarseFineFrameDifference << '\n';
    std::cout << "[METRIC] maximum orthonormality/handedness error: "
              << measurements.maximumFrameInvariantError << '\n';
    std::cout << "[METRIC] naive Euler frame differences: roll-pitch-yaw="
              << measurements.naiveRollPitchYawDifference
              << ", yaw-pitch-roll="
              << measurements.naiveYawPitchRollDifference << '\n';
    std::cout << tests.size()
              << " simultaneous roll/pitch/yaw test groups passed.\n";
    return 0;
}
