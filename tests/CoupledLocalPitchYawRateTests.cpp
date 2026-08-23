#include <quantum/coaster/RiderLocalGeometry.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
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
    using quantum::coaster::integrateLocalYawRateProfile;
    using quantum::coaster::RiderLocalGeometryState;
    using quantum::geometry::applyLocalPitch;
    using quantum::geometry::applyLocalYaw;
    using quantum::geometry::applyRoll;
    using quantum::geometry::CurveFrame;
    using quantum::math::integrateScalarTransition;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

    constexpr double positionTolerance = 3.0e-10;
    constexpr double orientationTolerance = 4.0e-11;
    constexpr std::size_t referenceIntervalCount = 131'072;

    const CurveFrame canonicalFrame{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    const glm::dvec3 startingPosition{2.5, -3.0, 1.25};

    struct Measurements
    {
        double maximumPitchReductionError = 0.0;
        double maximumYawReductionError = 0.0;
        double maximumConstantPositionError = 0.0;
        double maximumConstantOrientationError = 0.0;
        double maximumVariablePositionError = 0.0;
        double maximumVariableOrientationError = 0.0;
        double maximumFrameInvariantError = 0.0;
        double curvatureError = 0.0;
        double radiusError = 0.0;
        double proportionalOrientationError = 0.0;
        double proportionalPositionError = 0.0;
        double pitchThenYawDifference = 0.0;
        double yawThenPitchDifference = 0.0;
        double threeDimensionalVolume = 0.0;
        double transformedFrameError = 0.0;
        double translatedPositionError = 0.0;
        double translatedOrientationError = 0.0;
        double coarseReferencePositionError = 0.0;
        double coarseReferenceOrientationError = 0.0;
        double fineReferencePositionError = 0.0;
        double fineReferenceOrientationError = 0.0;
        double coarseFinePositionDifference = 0.0;
        double coarseFineOrientationDifference = 0.0;
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
                    && bitwiseEqual(
                        first[index].position,
                        second[index].position
                    )
                    && bitwiseEqual(first[index].frame, second[index].frame),
                context + " component bits"
            );
        }
    }

    void checkFrame(const CurveFrame& frame, const std::string& context)
    {
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
        requireNear(error, 0.0, 3.0e-15, context);
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
        const double pitchRate,
        const double yawRate,
        const double distance
    )
    {
        const double angularRate = std::hypot(pitchRate, yawRate);
        const double angle = angularRate * distance;
        const glm::dvec3 axis = (
            pitchRate * frame.lateral + yawRate * frame.up
        ) / angularRate;
        const glm::dvec3 turningDirection = glm::cross(axis, frame.tangent);
        const glm::dvec3 integratedPosition = position
            + std::sin(angle) / angularRate * frame.tangent
            + (1.0 - std::cos(angle)) / angularRate * turningDirection;

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

    [[nodiscard]] std::pair<double, double> referenceRates(
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
        const double pitchDomainValue = boundedDistance == profileLength
            ? pitch.domainEnd
            : pitch.domainBegin + boundedDistance;
        const double yawDomainValue = boundedDistance == profileLength
            ? yaw.domainEnd
            : yaw.domainBegin + boundedDistance;
        return {
            quantum::math::evaluateScalarTransition(
                pitch,
                pitchDomainValue
            ),
            quantum::math::evaluateScalarTransition(yaw, yawDomainValue)
        };
    }

    [[nodiscard]] ReferenceDerivative referenceDerivative(
        const ReferenceState& state,
        const double pitchRate,
        const double yawRate
    )
    {
        return {
            state.frame.tangent,
            yawRate * state.frame.lateral - pitchRate * state.frame.up,
            -yawRate * state.frame.tangent,
            pitchRate * state.frame.tangent
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
        const ScalarTransition& pitch,
        const ScalarTransition& yaw,
        const double traveledEnd
    )
    {
        if (traveledEnd == 0.0)
        {
            return {position, frame};
        }

        const double profileLength = pitch.domainEnd - pitch.domainBegin;
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
            const auto [p1, y1] = referenceRates(
                pitch, yaw, distance, profileLength
            );
            const ReferenceDerivative k1 = referenceDerivative(state, p1, y1);

            const auto [p2, y2] = referenceRates(
                pitch, yaw, distance + 0.5 * step, profileLength
            );
            const ReferenceDerivative k2 = referenceDerivative(
                addReferenceDerivative(state, k1, 0.5 * step), p2, y2
            );
            const ReferenceDerivative k3 = referenceDerivative(
                addReferenceDerivative(state, k2, 0.5 * step), p2, y2
            );

            const auto [p4, y4] = referenceRates(
                pitch, yaw, distance + step, profileLength
            );
            const ReferenceDerivative k4 = referenceDerivative(
                addReferenceDerivative(state, k3, step), p4, y4
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
                k1.up + 2.0 * k2.up + 2.0 * k3.up + k4.up
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
        measurements.maximumVariableOrientationError = std::max(
            measurements.maximumVariableOrientationError,
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

    void testZeroAndSingleAxisReductions()
    {
        constexpr double length = 7.5;
        const ScalarTransition zeroPitch{
            0.0, length, 0.0, 0.0, TransitionType::Smootherstep
        };
        const ScalarTransition zeroYaw{
            0.0, length, 0.0, 0.0, TransitionType::CosineEaseInOut
        };
        const auto straight = integrateLocalPitchYawRateProfiles(
            startingPosition, canonicalFrame, zeroPitch, zeroYaw, 1.1
        );
        for (const RiderLocalGeometryState& state : straight)
        {
            requireNear(
                vectorError(
                    state.position,
                    startingPosition + state.distance * canonicalFrame.tangent
                ),
                0.0,
                0.0,
                "zero-channel straight position"
            );
            require(bitwiseEqual(state.frame, canonicalFrame),
                "zero-channel straight frame changed");
        }

        const std::vector<ScalarTransition> pitchProfiles{
            {0.0, length, 0.13, 0.13, TransitionType::Linear},
            {0.0, length, -0.08, 0.17, TransitionType::Smootherstep}
        };
        for (const ScalarTransition& pitch : pitchProfiles)
        {
            const auto expected = integrateLocalPitchRateProfile(
                startingPosition, canonicalFrame, pitch, 0.73
            );
            const auto actual = integrateLocalPitchYawRateProfiles(
                startingPosition, canonicalFrame, pitch, zeroYaw, 0.73
            );
            requireBitwiseEqual(actual, expected, "pitch-only reduction");
            for (std::size_t index = 0; index < actual.size(); ++index)
            {
                measurements.maximumPitchReductionError = std::max(
                    measurements.maximumPitchReductionError,
                    frameError(actual[index].frame, expected[index].frame)
                );
            }
        }

        const std::vector<ScalarTransition> yawProfiles{
            {0.0, length, -0.11, -0.11, TransitionType::Linear},
            {0.0, length, 0.03, -0.16, TransitionType::SineEaseOut}
        };
        for (const ScalarTransition& yaw : yawProfiles)
        {
            const auto expected = integrateLocalYawRateProfile(
                startingPosition, canonicalFrame, yaw, 0.73
            );
            const auto actual = integrateLocalPitchYawRateProfiles(
                startingPosition, canonicalFrame, zeroPitch, yaw, 0.73
            );
            requireBitwiseEqual(actual, expected, "yaw-only reduction");
            for (std::size_t index = 0; index < actual.size(); ++index)
            {
                measurements.maximumYawReductionError = std::max(
                    measurements.maximumYawReductionError,
                    frameError(actual[index].frame, expected[index].frame)
                );
            }
        }
    }

    void testConstantCombinedAnalyticGeometry()
    {
        constexpr double length = 11.75;
        constexpr double pitchRate = 0.17;
        constexpr double yawRate = -0.11;
        const ScalarTransition pitch{
            0.0, length, pitchRate, pitchRate, TransitionType::QuadraticEaseIn
        };
        const ScalarTransition yaw{
            0.0, length, yawRate, yawRate, TransitionType::SineEaseOut
        };
        const auto states = integrateLocalPitchYawRateProfiles(
            startingPosition, canonicalFrame, pitch, yaw, 0.37
        );
        const double angularRate = std::hypot(pitchRate, yawRate);
        const glm::dvec3 axis = (
            pitchRate * canonicalFrame.lateral
                + yawRate * canonicalFrame.up
        ) / angularRate;
        const glm::dvec3 turning = glm::cross(axis, canonicalFrame.tangent);
        const glm::dvec3 circleCenter =
            startingPosition + turning / angularRate;

        for (const RiderLocalGeometryState& state : states)
        {
            const RiderLocalGeometryState expected = analyticConstantState(
                startingPosition,
                canonicalFrame,
                pitchRate,
                yawRate,
                state.distance
            );
            const double positionError = vectorError(
                state.position,
                expected.position
            );
            const double orientationError = frameError(
                state.frame,
                expected.frame
            );
            measurements.maximumConstantPositionError = std::max(
                measurements.maximumConstantPositionError,
                positionError
            );
            measurements.maximumConstantOrientationError = std::max(
                measurements.maximumConstantOrientationError,
                orientationError
            );
            requireNear(positionError, 0.0, 2.0e-14,
                "constant combined analytic position");
            requireNear(orientationError, 0.0, 3.0e-15,
                "constant combined analytic frame");
            checkFrame(state.frame, "constant combined frame invariants");
            measurements.radiusError = std::max(
                measurements.radiusError,
                std::abs(
                    glm::length(state.position - circleCenter)
                        - 1.0 / angularRate
                )
            );
        }
        requireNear(measurements.radiusError, 0.0, 2.0e-14,
            "constant combined circle radius");

        constexpr double derivativeSpacing = 1.0e-5;
        const ScalarTransition shortPitch{
            0.0, 2.0 * derivativeSpacing,
            pitchRate, pitchRate, TransitionType::Linear
        };
        const ScalarTransition shortYaw{
            0.0, 2.0 * derivativeSpacing,
            yawRate, yawRate, TransitionType::Linear
        };
        const auto derivativeStates = integrateLocalPitchYawRateProfiles(
            {0.0, 0.0, 0.0}, canonicalFrame,
            shortPitch, shortYaw, derivativeSpacing
        );
        const double measuredCurvature = glm::length(
            derivativeStates.back().frame.tangent
                - derivativeStates.front().frame.tangent
        ) / (2.0 * derivativeSpacing);
        measurements.curvatureError = std::abs(
            measuredCurvature - angularRate
        );
        requireNear(measurements.curvatureError, 0.0, 2.0e-11,
            "constant combined curvature magnitude");
    }

    void testProportionalVariableProfiles()
    {
        constexpr double length = 9.0;
        constexpr double ratio = -0.7;
        const ScalarTransition pitch{
            0.0, length, 0.015, 0.19, TransitionType::Smootherstep
        };
        const ScalarTransition yaw{
            0.0, length,
            ratio * pitch.valueBegin,
            ratio * pitch.valueEnd,
            TransitionType::Smootherstep
        };
        const auto states = integrateLocalPitchYawRateProfiles(
            startingPosition, canonicalFrame, pitch, yaw, 0.8
        );
        const double accumulatedPitch = integrateScalarTransition(
            pitch, pitch.domainBegin, pitch.domainEnd
        );
        const double angle = accumulatedPitch * std::sqrt(1.0 + ratio * ratio);
        const glm::dvec3 axis = (
            canonicalFrame.lateral + ratio * canonicalFrame.up
        ) / std::sqrt(1.0 + ratio * ratio);
        const CurveFrame expectedFrame{
            rotateAroundAxis(canonicalFrame.tangent, axis, angle),
            rotateAroundAxis(canonicalFrame.lateral, axis, angle),
            rotateAroundAxis(canonicalFrame.up, axis, angle)
        };
        measurements.proportionalOrientationError = frameError(
            states.back().frame,
            expectedFrame
        );
        requireNear(measurements.proportionalOrientationError, 0.0, 2.0e-12,
            "proportional-profile analytic frame");

        const ReferenceState reference = highAccuracyReference(
            startingPosition, canonicalFrame, pitch, yaw, length
        );
        measurements.proportionalPositionError = vectorError(
            states.back().position,
            reference.position
        );
        requireNear(measurements.proportionalPositionError, 0.0,
            positionTolerance, "proportional-profile position reference");
    }

    void testVariableReferencesAndThreeDimensionalGeometry()
    {
        constexpr double length = 10.0;
        const std::vector<std::pair<ScalarTransition, ScalarTransition>> cases{
            {
                {0.0, length, 0.01, 0.20, TransitionType::Smootherstep},
                {0.0, length, -0.14, 0.12, TransitionType::CosineEaseInOut}
            },
            {
                {0.0, length, -0.06, 0.18, TransitionType::QuadraticEaseIn},
                {0.0, length, 0.16, -0.03, TransitionType::SineEaseOut}
            }
        };

        for (std::size_t caseIndex = 0; caseIndex < cases.size(); ++caseIndex)
        {
            const auto& [pitch, yaw] = cases[caseIndex];
            const auto states = integrateLocalPitchYawRateProfiles(
                startingPosition, canonicalFrame, pitch, yaw, 1.0
            );
            for (const RiderLocalGeometryState& state : states)
            {
                checkFrame(state.frame, "variable frame invariants");
            }
            const ReferenceState reference = highAccuracyReference(
                startingPosition, canonicalFrame, pitch, yaw, length
            );
            compareWithReference(
                states.back(),
                reference,
                "variable transition combination " + std::to_string(caseIndex)
            );

            if (caseIndex == 0)
            {
                const glm::dvec3 first = states[3].position - states[0].position;
                const glm::dvec3 second = states[6].position - states[0].position;
                const glm::dvec3 third = states[10].position - states[0].position;
                measurements.threeDimensionalVolume = std::abs(
                    glm::dot(glm::cross(first, second), third)
                );
                require(measurements.threeDimensionalVolume > 1.0e-3,
                    "variable simultaneous geometry was not demonstrably 3D");
            }
        }
    }

    void testNoncommutativity()
    {
        constexpr double length = 8.0;
        constexpr double pitchRate = 0.16;
        constexpr double yawRate = 0.12;
        const ScalarTransition pitch{
            0.0, length, pitchRate, pitchRate, TransitionType::Linear
        };
        const ScalarTransition yaw{
            0.0, length, yawRate, yawRate, TransitionType::Linear
        };
        const CurveFrame coupled = integrateLocalPitchYawRateProfiles(
            startingPosition, canonicalFrame, pitch, yaw, length
        ).back().frame;
        const double totalPitch = pitchRate * length;
        const double totalYaw = yawRate * length;
        const CurveFrame pitchThenYaw = applyLocalYaw(
            applyLocalPitch(canonicalFrame, totalPitch),
            totalYaw
        );
        const CurveFrame yawThenPitch = applyLocalPitch(
            applyLocalYaw(canonicalFrame, totalYaw),
            totalPitch
        );
        measurements.pitchThenYawDifference = frameError(
            coupled, pitchThenYaw
        );
        measurements.yawThenPitchDifference = frameError(
            coupled, yawThenPitch
        );
        require(measurements.pitchThenYawDifference > 0.1,
            "coupled result matched pitch-then-yaw accumulated Euler angles");
        require(measurements.yawThenPitchDifference > 0.1,
            "coupled result matched yaw-then-pitch accumulated Euler angles");
    }

    void testSignCrossings()
    {
        constexpr double length = 8.0;
        const std::vector<std::pair<ScalarTransition, ScalarTransition>> cases{
            {
                {0.0, length, -0.18, 0.14, TransitionType::Linear},
                {0.0, length, 0.04, 0.13, TransitionType::Smootherstep}
            },
            {
                {0.0, length, 0.05, 0.17, TransitionType::CosineEaseInOut},
                {0.0, length, 0.16, -0.12, TransitionType::Linear}
            },
            {
                {0.0, length, -0.15, 0.13, TransitionType::QuadraticEaseIn},
                {0.0, length, 0.14, -0.17, TransitionType::SineEaseOut}
            }
        };
        for (std::size_t index = 0; index < cases.size(); ++index)
        {
            const auto& [pitch, yaw] = cases[index];
            const auto states = integrateLocalPitchYawRateProfiles(
                startingPosition, canonicalFrame, pitch, yaw, 0.9
            );
            for (const RiderLocalGeometryState& state : states)
            {
                require(std::isfinite(state.position.x)
                        && std::isfinite(state.position.y)
                        && std::isfinite(state.position.z),
                    "sign crossing produced a non-finite position");
                checkFrame(state.frame, "sign-crossing frame invariants");
            }
            compareWithReference(
                states.back(),
                highAccuracyReference(
                    startingPosition, canonicalFrame, pitch, yaw, length
                ),
                "sign-crossing reference " + std::to_string(index)
            );
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
        const std::vector<RiderLocalGeometryState>& transformed,
        const CurveFrame& transformedStart,
        const std::string& context
    )
    {
        require(canonical.size() == transformed.size(), context + " state count");
        for (std::size_t index = 0; index < canonical.size(); ++index)
        {
            double error = vectorError(
                coordinatesInFrame(
                    canonical[index].position - startingPosition,
                    canonicalFrame
                ),
                coordinatesInFrame(
                    transformed[index].position - startingPosition,
                    transformedStart
                )
            );
            error = std::max(error, vectorError(
                coordinatesInFrame(canonical[index].frame.tangent, canonicalFrame),
                coordinatesInFrame(
                    transformed[index].frame.tangent, transformedStart
                )
            ));
            error = std::max(error, vectorError(
                coordinatesInFrame(canonical[index].frame.lateral, canonicalFrame),
                coordinatesInFrame(
                    transformed[index].frame.lateral, transformedStart
                )
            ));
            error = std::max(error, vectorError(
                coordinatesInFrame(canonical[index].frame.up, canonicalFrame),
                coordinatesInFrame(transformed[index].frame.up, transformedStart)
            ));
            measurements.transformedFrameError = std::max(
                measurements.transformedFrameError,
                error
            );
        }
        requireNear(measurements.transformedFrameError, 0.0, 8.0e-12, context);
    }

    void testStartingFrameInvariance()
    {
        constexpr double length = 7.0;
        const ScalarTransition pitch{
            0.0, length, -0.03, 0.18, TransitionType::Smootherstep
        };
        const ScalarTransition yaw{
            0.0, length, 0.15, -0.04, TransitionType::CosineEaseInOut
        };
        const auto canonical = integrateLocalPitchYawRateProfiles(
            startingPosition, canonicalFrame, pitch, yaw, 0.7
        );
        const std::vector<CurveFrame> transformedStarts{
            applyRoll(canonicalFrame, 0.63),
            applyLocalPitch(canonicalFrame, 0.47),
            applyLocalYaw(canonicalFrame, -0.39)
        };
        for (std::size_t index = 0; index < transformedStarts.size(); ++index)
        {
            const auto transformed = integrateLocalPitchYawRateProfiles(
                startingPosition,
                transformedStarts[index],
                pitch,
                yaw,
                0.7
            );
            compareTransformedRun(
                canonical,
                transformed,
                transformedStarts[index],
                "starting-frame transform " + std::to_string(index)
            );
        }
    }

    void testTranslatedDomainSamplingAndDeterminism()
    {
        constexpr double length = 7.25;
        const ScalarTransition pitch{
            0.0, length, 0.01, 0.17, TransitionType::QuadraticEaseIn
        };
        const ScalarTransition yaw{
            0.0, length, -0.13, 0.09, TransitionType::SineEaseOut
        };
        const ScalarTransition translatedPitch{
            100.0, 100.0 + length,
            pitch.valueBegin, pitch.valueEnd, pitch.transitionType
        };
        const ScalarTransition translatedYaw{
            100.0, 100.0 + length,
            yaw.valueBegin, yaw.valueEnd, yaw.transitionType
        };
        const auto origin = integrateLocalPitchYawRateProfiles(
            startingPosition, canonicalFrame, pitch, yaw, 2.0
        );
        const auto translated = integrateLocalPitchYawRateProfiles(
            startingPosition,
            canonicalFrame,
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
            measurements.translatedOrientationError = std::max(
                measurements.translatedOrientationError,
                frameError(origin[index].frame, translated[index].frame)
            );
        }
        requireNear(measurements.translatedPositionError, 0.0, 2.0e-13,
            "translated-domain positions");
        requireNear(measurements.translatedOrientationError, 0.0, 2.0e-13,
            "translated-domain frames");

        const std::vector<double> expectedDistances{0.0, 2.0, 4.0, 6.0, length};
        require(origin.size() == expectedDistances.size(),
            "coupled output sample count");
        for (std::size_t index = 0; index < origin.size(); ++index)
        {
            requireNear(origin[index].distance, expectedDistances[index], 0.0,
                "coupled deterministic output spacing");
        }
        require(bitwiseEqual(origin.front().position, startingPosition)
                && bitwiseEqual(origin.front().frame, canonicalFrame),
            "coupled beginning state was not exact");
        require(origin.back().distance == length,
            "coupled terminal traveled distance was not exact");
        require(origin[origin.size() - 2].distance < origin.back().distance,
            "coupled endpoint was duplicated");

        const auto coarse = integrateLocalPitchYawRateProfiles(
            startingPosition, canonicalFrame, pitch, yaw, 20.0
        );
        require(coarse.size() == 2 && coarse.front().distance == 0.0
                && coarse.back().distance == length,
            "spacing larger than section did not return begin/end");
        const ReferenceState reference = highAccuracyReference(
            startingPosition, canonicalFrame, pitch, yaw, length
        );
        measurements.coarseReferencePositionError = vectorError(
            coarse.back().position,
            reference.position
        );
        measurements.coarseReferenceOrientationError = frameError(
            coarse.back().frame,
            reference.frame
        );
        requireNear(measurements.coarseReferencePositionError, 0.0,
            positionTolerance, "coarse-output internal position accuracy");
        requireNear(measurements.coarseReferenceOrientationError, 0.0,
            orientationTolerance, "coarse-output internal frame accuracy");

        const auto fine = integrateLocalPitchYawRateProfiles(
            startingPosition, canonicalFrame, pitch, yaw, 0.2
        );
        measurements.fineReferencePositionError = vectorError(
            fine.back().position,
            reference.position
        );
        measurements.fineReferenceOrientationError = frameError(
            fine.back().frame,
            reference.frame
        );
        measurements.coarseFinePositionDifference = vectorError(
            coarse.back().position,
            fine.back().position
        );
        measurements.coarseFineOrientationDifference = frameError(
            coarse.back().frame,
            fine.back().frame
        );
        requireNear(measurements.fineReferencePositionError, 0.0,
            positionTolerance, "fine-output internal position accuracy");
        requireNear(measurements.fineReferenceOrientationError, 0.0,
            orientationTolerance, "fine-output internal frame accuracy");
        requireNear(measurements.coarseFinePositionDifference, 0.0,
            positionTolerance, "coarse/fine output position agreement");
        requireNear(measurements.coarseFineOrientationDifference, 0.0,
            orientationTolerance, "coarse/fine output frame agreement");

        const auto repeated = integrateLocalPitchYawRateProfiles(
            startingPosition, canonicalFrame, pitch, yaw, 2.0
        );
        requireBitwiseEqual(origin, repeated, "coupled determinism");
    }

    void testValidation()
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();
        const ScalarTransition validPitch{
            0.0, 5.0, 0.01, 0.1, TransitionType::Linear
        };
        const ScalarTransition validYaw{
            0.0, 5.0, -0.02, 0.08, TransitionType::Smoothstep
        };

        for (const glm::dvec3 position : {
                 glm::dvec3{nan, 0.0, 0.0},
                 glm::dvec3{0.0, infinity, 0.0},
                 glm::dvec3{0.0, 0.0, -infinity}
             })
        {
            requireThrows<std::invalid_argument>(
                [position, &validPitch, &validYaw]
                {
                    static_cast<void>(integrateLocalPitchYawRateProfiles(
                        position,
                        canonicalFrame,
                        validPitch,
                        validYaw,
                        1.0
                    ));
                },
                "coupled non-finite starting position"
            );
        }

        const CurveFrame invalidFrame{
            {2.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };
        requireThrows<std::invalid_argument>(
            [&invalidFrame, &validPitch, &validYaw]
            {
                static_cast<void>(integrateLocalPitchYawRateProfiles(
                    startingPosition,
                    invalidFrame,
                    validPitch,
                    validYaw,
                    1.0
                ));
            },
            "coupled invalid starting frame"
        );

        for (const double spacing : {0.0, -1.0, nan, infinity})
        {
            requireThrows<std::invalid_argument>(
                [spacing, &validPitch, &validYaw]
                {
                    static_cast<void>(integrateLocalPitchYawRateProfiles(
                        startingPosition,
                        canonicalFrame,
                        validPitch,
                        validYaw,
                        spacing
                    ));
                },
                "coupled invalid integration spacing"
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
                [&invalid, &validYaw]
                {
                    static_cast<void>(integrateLocalPitchYawRateProfiles(
                        startingPosition,
                        canonicalFrame,
                        invalid,
                        validYaw,
                        1.0
                    ));
                },
                "coupled invalid pitch profile"
            );
            requireThrows<std::invalid_argument>(
                [&invalid, &validPitch]
                {
                    static_cast<void>(integrateLocalPitchYawRateProfiles(
                        startingPosition,
                        canonicalFrame,
                        validPitch,
                        invalid,
                        1.0
                    ));
                },
                "coupled invalid yaw profile"
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
                [&validPitch, &mismatch]
                {
                    static_cast<void>(integrateLocalPitchYawRateProfiles(
                        startingPosition,
                        canonicalFrame,
                        validPitch,
                        mismatch,
                        1.0
                    ));
                },
                "coupled exact shared-domain validation"
            );
        }
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;
    const std::vector<Test> tests{
        {"zero and single-axis reductions", testZeroAndSingleAxisReductions},
        {"constant combined analytic geometry", testConstantCombinedAnalyticGeometry},
        {"proportional variable profiles", testProportionalVariableProfiles},
        {
            "variable references and three-dimensional geometry",
            testVariableReferencesAndThreeDimensionalGeometry
        },
        {"noncommutativity", testNoncommutativity},
        {"sign crossings", testSignCrossings},
        {"starting-frame invariance", testStartingFrameInvariance},
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
    std::cout << "[METRIC] maximum pitch-only reduction error: "
              << measurements.maximumPitchReductionError << '\n';
    std::cout << "[METRIC] maximum yaw-only reduction error: "
              << measurements.maximumYawReductionError << '\n';
    std::cout << "[METRIC] maximum constant position error: "
              << measurements.maximumConstantPositionError << '\n';
    std::cout << "[METRIC] maximum constant orientation error: "
              << measurements.maximumConstantOrientationError << '\n';
    std::cout << "[METRIC] curvature error: "
              << measurements.curvatureError << '\n';
    std::cout << "[METRIC] radius error: "
              << measurements.radiusError << '\n';
    std::cout << "[METRIC] proportional frame error: "
              << measurements.proportionalOrientationError << '\n';
    std::cout << "[METRIC] proportional position error: "
              << measurements.proportionalPositionError << '\n';
    std::cout << "[METRIC] maximum variable reference position error: "
              << measurements.maximumVariablePositionError << '\n';
    std::cout << "[METRIC] maximum variable reference orientation error: "
              << measurements.maximumVariableOrientationError << '\n';
    std::cout << "[METRIC] sequential Euler differences: pitch-then-yaw="
              << measurements.pitchThenYawDifference
              << ", yaw-then-pitch="
              << measurements.yawThenPitchDifference << '\n';
    std::cout << "[METRIC] variable 3D scalar volume: "
              << measurements.threeDimensionalVolume << '\n';
    std::cout << "[METRIC] maximum transformed-start error: "
              << measurements.transformedFrameError << '\n';
    std::cout << "[METRIC] translated-domain errors: position="
              << measurements.translatedPositionError
              << ", orientation="
              << measurements.translatedOrientationError << '\n';
    std::cout << "[METRIC] coarse-output reference errors: position="
              << measurements.coarseReferencePositionError
              << ", orientation="
              << measurements.coarseReferenceOrientationError << '\n';
    std::cout << "[METRIC] fine-output reference errors: position="
              << measurements.fineReferencePositionError
              << ", orientation="
              << measurements.fineReferenceOrientationError << '\n';
    std::cout << "[METRIC] coarse/fine output differences: position="
              << measurements.coarseFinePositionDifference
              << ", orientation="
              << measurements.coarseFineOrientationDifference << '\n';
    std::cout << "[METRIC] maximum frame invariant error: "
              << measurements.maximumFrameInvariantError << '\n';
    std::cout << tests.size() << " coupled pitch/yaw test groups passed.\n";
    return 0;
}
