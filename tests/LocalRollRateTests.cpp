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
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::integrateConstantLocalRollRate;
    using quantum::coaster::integrateLocalRollRateProfile;
    using quantum::coaster::RiderLocalGeometryState;
    using quantum::geometry::applyLocalPitch;
    using quantum::geometry::applyLocalYaw;
    using quantum::geometry::applyRoll;
    using quantum::geometry::CurveFrame;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

    constexpr double frameTolerance = 3.0e-12;
    constexpr double positionTolerance = 3.0e-12;
    constexpr double derivativeTolerance = 2.0e-7;
    constexpr double pi = std::numbers::pi_v<double>;

    const CurveFrame canonicalFrame{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    struct Measurements
    {
        double maximumAnalyticFrameError = 0.0;
        double maximumPositionError = 0.0;
        double maximumFrameInvariantError = 0.0;
        double maximumTangentError = 0.0;
        double maximumCollinearityError = 0.0;
        double maximumCurvature = 0.0;
        double maximumConstantProfileError = 0.0;
        double maximumTranslatedPositionDifference = 0.0;
        double maximumTranslatedFrameDifference = 0.0;
        double maximumTransformedFrameError = 0.0;
        double symmetricAccumulatedRoll = 0.0;
        double asymmetricAccumulatedRoll = 0.0;
        double signChangingFinalRoll = 0.0;
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

    [[nodiscard]] double vectorError(
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

    [[nodiscard]] bool bitwiseEqual(
        const RiderLocalGeometryState& first,
        const RiderLocalGeometryState& second
    )
    {
        return bitwiseEqual(first.distance, second.distance)
            && bitwiseEqual(first.position, second.position)
            && bitwiseEqual(first.frame, second.frame);
    }

    [[nodiscard]] CurveFrame independentRolledFrame(
        const CurveFrame& frame,
        const double angle
    )
    {
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);
        return {
            frame.tangent,
            cosine * frame.lateral + sine * frame.up,
            cosine * frame.up - sine * frame.lateral
        };
    }

    void requireValidFrame(
        const CurveFrame& frame,
        const std::string_view context
    )
    {
        const std::array<double, 12> components{
            frame.tangent.x, frame.tangent.y, frame.tangent.z,
            frame.lateral.x, frame.lateral.y, frame.lateral.z,
            frame.up.x, frame.up.y, frame.up.z,
            glm::dot(frame.tangent, frame.lateral),
            glm::dot(frame.tangent, frame.up),
            glm::dot(frame.lateral, frame.up)
        };
        for (const double component : components)
        {
            require(
                std::isfinite(component),
                std::string(context) + " contains a non-finite frame component"
            );
        }

        const double invariantError = std::max({
            std::abs(vectorMagnitude(frame.tangent) - 1.0),
            std::abs(vectorMagnitude(frame.lateral) - 1.0),
            std::abs(vectorMagnitude(frame.up) - 1.0),
            std::abs(glm::dot(frame.tangent, frame.lateral)),
            std::abs(glm::dot(frame.tangent, frame.up)),
            std::abs(glm::dot(frame.lateral, frame.up)),
            vectorError(glm::cross(frame.tangent, frame.lateral), frame.up)
        });
        measurements.maximumFrameInvariantError = std::max(
            measurements.maximumFrameInvariantError,
            invariantError
        );
        requireNear(
            invariantError,
            0.0,
            frameTolerance,
            std::string(context) + " orthonormality and handedness"
        );
    }

    void requireRollState(
        const RiderLocalGeometryState& state,
        const glm::dvec3& startingPosition,
        const CurveFrame& startingFrame,
        const double expectedAngle,
        const std::string_view context
    )
    {
        const glm::dvec3 expectedPosition = startingPosition
            + state.distance * startingFrame.tangent;
        const CurveFrame expectedFrame = independentRolledFrame(
            startingFrame,
            expectedAngle
        );
        const double positionError = vectorError(
            state.position,
            expectedPosition
        );
        const double analyticFrameError = frameError(
            state.frame,
            expectedFrame
        );
        const double tangentError = vectorError(
            state.frame.tangent,
            startingFrame.tangent
        );
        measurements.maximumPositionError = std::max(
            measurements.maximumPositionError,
            positionError
        );
        measurements.maximumAnalyticFrameError = std::max(
            measurements.maximumAnalyticFrameError,
            analyticFrameError
        );
        measurements.maximumTangentError = std::max(
            measurements.maximumTangentError,
            tangentError
        );

        require(
            bitwiseEqual(state.position, expectedPosition),
            std::string(context) + " did not use the exact straight-line position"
        );
        require(
            bitwiseEqual(state.frame.tangent, startingFrame.tangent),
            std::string(context) + " changed the starting tangent bits"
        );
        requireNear(
            analyticFrameError,
            0.0,
            frameTolerance,
            std::string(context) + " analytic roll frame"
        );
        requireValidFrame(state.frame, context);
    }

    void requireStraightCenterline(
        const std::vector<RiderLocalGeometryState>& states,
        const glm::dvec3& startingPosition,
        const CurveFrame& startingFrame,
        const std::string_view context
    )
    {
        require(!states.empty(), std::string(context) + " returned no states");

        for (std::size_t index = 0; index < states.size(); ++index)
        {
            const RiderLocalGeometryState& state = states[index];
            const glm::dvec3 expectedPosition = startingPosition
                + state.distance * startingFrame.tangent;
            const double collinearityError = vectorMagnitude(glm::cross(
                state.position - startingPosition,
                startingFrame.tangent
            ));
            measurements.maximumCollinearityError = std::max(
                measurements.maximumCollinearityError,
                collinearityError
            );

            require(
                bitwiseEqual(state.position, expectedPosition),
                std::string(context) + " position is not the exact line solution"
            );
            require(
                bitwiseEqual(state.frame.tangent, startingFrame.tangent),
                std::string(context) + " tangent changed"
            );
            requireNear(
                collinearityError,
                0.0,
                positionTolerance,
                std::string(context) + " centerline collinearity"
            );

            if (index != 0)
            {
                const double interval = state.distance
                    - states[index - 1].distance;
                const double curvature = vectorError(
                    state.frame.tangent,
                    states[index - 1].frame.tangent
                ) / interval;
                measurements.maximumCurvature = std::max(
                    measurements.maximumCurvature,
                    curvature
                );
                require(
                    curvature == 0.0,
                    std::string(context) + " created centerline curvature"
                );
            }
        }
    }

    [[nodiscard]] double normalizedIntegralReference(
        const TransitionType type,
        const double progress
    )
    {
        switch (type)
        {
        case TransitionType::Linear:
            return 0.5 * progress * progress;
        case TransitionType::Smootherstep:
            return progress * progress * progress * progress
                * (2.5 - 3.0 * progress + progress * progress);
        case TransitionType::CosineEaseInOut:
            return 0.5 * progress
                - std::sin(pi * progress) / (2.0 * pi);
        case TransitionType::QuadraticEaseIn:
            return progress * progress * progress / 3.0;
        default:
            throw TestFailure("missing independent roll transition reference");
        }
    }

    [[nodiscard]] double accumulatedRollReference(
        const ScalarTransition& transition,
        const double traveledDistance
    )
    {
        const double length = transition.domainEnd
            - transition.domainBegin;
        const double progress = traveledDistance / length;
        return transition.valueBegin * traveledDistance
            + (transition.valueEnd - transition.valueBegin)
                * length
                * normalizedIntegralReference(
                    transition.transitionType,
                    progress
                );
    }

    void requireSeriesBitwiseEqual(
        const std::vector<RiderLocalGeometryState>& first,
        const std::vector<RiderLocalGeometryState>& second,
        const std::string_view context
    )
    {
        require(first.size() == second.size(), std::string(context) + " size");
        for (std::size_t index = 0; index < first.size(); ++index)
        {
            require(
                bitwiseEqual(first[index], second[index]),
                std::string(context) + " state " + std::to_string(index)
            );
        }
    }

    void testConstantAnalyticRolls()
    {
        const glm::dvec3 startingPosition{2.0, -3.0, 5.0};
        const auto zero = integrateConstantLocalRollRate(
            startingPosition,
            canonicalFrame,
            5.5,
            0.0,
            2.0
        );
        const std::array<double, 4> expectedDistances{0.0, 2.0, 4.0, 5.5};
        require(zero.size() == expectedDistances.size(), "zero-roll sampling count");
        for (std::size_t index = 0; index < zero.size(); ++index)
        {
            require(
                bitwiseEqual(zero[index].distance, expectedDistances[index]),
                "zero-roll sample distance"
            );
            require(
                bitwiseEqual(zero[index].frame, canonicalFrame),
                "zero roll changed frame bits"
            );
            requireRollState(
                zero[index], startingPosition, canonicalFrame, 0.0, "zero roll"
            );
        }
        requireStraightCenterline(
            zero, startingPosition, canonicalFrame, "zero constant roll"
        );

        struct AnalyticCase
        {
            double angle;
            glm::dvec3 expectedLateral;
            glm::dvec3 expectedUp;
            std::string_view name;
        };
        const std::array<AnalyticCase, 6> cases{
            AnalyticCase{
                0.5 * pi, {0.0, 0.0, 1.0}, {0.0, -1.0, 0.0}, "+90 degrees"
            },
            AnalyticCase{
                -0.5 * pi, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, "-90 degrees"
            },
            AnalyticCase{
                pi, {0.0, -1.0, 0.0}, {0.0, 0.0, -1.0}, "180 degrees"
            },
            AnalyticCase{
                2.0 * pi, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, "360 degrees"
            },
            AnalyticCase{
                9.0 * pi, {0.0, -1.0, 0.0}, {0.0, 0.0, -1.0},
                "multiple revolutions"
            },
            AnalyticCase{
                -1.37, {0.0, std::cos(-1.37), std::sin(-1.37)},
                {0.0, -std::sin(-1.37), std::cos(-1.37)}, "negative roll"
            }
        };

        for (const AnalyticCase& testCase : cases)
        {
            constexpr double sectionLength = 2.0;
            const auto states = integrateConstantLocalRollRate(
                startingPosition,
                canonicalFrame,
                sectionLength,
                testCase.angle / sectionLength,
                0.37
            );
            for (const RiderLocalGeometryState& state : states)
            {
                requireRollState(
                    state,
                    startingPosition,
                    canonicalFrame,
                    testCase.angle * state.distance / sectionLength,
                    testCase.name
                );
            }
            requireStraightCenterline(
                states, startingPosition, canonicalFrame, testCase.name
            );
            requireNear(
                vectorError(states.back().frame.lateral, testCase.expectedLateral),
                0.0,
                frameTolerance,
                std::string(testCase.name) + " independent lateral reference"
            );
            requireNear(
                vectorError(states.back().frame.up, testCase.expectedUp),
                0.0,
                frameTolerance,
                std::string(testCase.name) + " independent up reference"
            );
        }

        const auto zeroLength = integrateConstantLocalRollRate(
            startingPosition, canonicalFrame, 0.0, 7.0, 1.0
        );
        require(zeroLength.size() == 1, "zero-length roll state count");
        require(
            bitwiseEqual(
                zeroLength.front(),
                RiderLocalGeometryState{0.0, startingPosition, canonicalFrame}
            ),
            "zero-length roll beginning state"
        );

        const auto coarse = integrateConstantLocalRollRate(
            startingPosition, canonicalFrame, 3.25, 0.7, 10.0
        );
        require(coarse.size() == 2, "large roll spacing should return two states");
        require(
            bitwiseEqual(coarse.back().distance, 3.25),
            "large-spacing terminal distance"
        );
    }

    void testConstantProfileEquivalence()
    {
        const glm::dvec3 startingPosition{-4.0, 1.5, 8.0};
        const CurveFrame startingFrame = applyLocalPitch(
            applyLocalYaw(canonicalFrame, -0.38),
            0.27
        );
        constexpr double length = 2.75;
        const std::array<double, 4> rates{0.0, 0.31, -0.27, 4.0 * pi};

        for (const double rate : rates)
        {
            const auto constant = integrateConstantLocalRollRate(
                startingPosition, startingFrame, length, rate, 0.43
            );
            const ScalarTransition transition{
                100.0,
                100.0 + length,
                rate,
                rate,
                TransitionType::SeventhOrderSmoothstep
            };
            const auto profile = integrateLocalRollRateProfile(
                startingPosition, startingFrame, transition, 0.43
            );
            requireSeriesBitwiseEqual(
                constant, profile, "constant roll/profile equivalence"
            );

            for (std::size_t index = 0; index < constant.size(); ++index)
            {
                measurements.maximumConstantProfileError = std::max(
                    measurements.maximumConstantProfileError,
                    std::max(
                        vectorError(
                            constant[index].position,
                            profile[index].position
                        ),
                        frameError(constant[index].frame, profile[index].frame)
                    )
                );
            }
        }
    }

    void testVariableRollReferencesAndPresets()
    {
        const glm::dvec3 startingPosition{1.0, 2.0, -1.0};
        struct ProfileCase
        {
            ScalarTransition transition;
            std::string_view name;
        };
        const std::array<ProfileCase, 4> cases{
            ProfileCase{{0.0, 4.0, 0.1, 0.5, TransitionType::Linear}, "linear"},
            ProfileCase{
                {0.0, 5.0, -0.2, 0.6, TransitionType::Smootherstep},
                "smootherstep"
            },
            ProfileCase{
                {0.0, 3.0, 0.7, -0.1, TransitionType::CosineEaseInOut},
                "cosine ease-in-out"
            },
            ProfileCase{
                {0.0, 6.0, 0.1, 0.7, TransitionType::QuadraticEaseIn},
                "quadratic ease-in"
            }
        };

        for (const ProfileCase& testCase : cases)
        {
            const auto states = integrateLocalRollRateProfile(
                startingPosition,
                canonicalFrame,
                testCase.transition,
                0.71
            );
            for (const RiderLocalGeometryState& state : states)
            {
                requireRollState(
                    state,
                    startingPosition,
                    canonicalFrame,
                    accumulatedRollReference(
                        testCase.transition,
                        state.distance
                    ),
                    testCase.name
                );
            }
            requireStraightCenterline(
                states, startingPosition, canonicalFrame, testCase.name
            );

            const double terminalReference = accumulatedRollReference(
                testCase.transition,
                testCase.transition.domainEnd
                    - testCase.transition.domainBegin
            );
            const double terminalFrameError = frameError(
                states.back().frame,
                independentRolledFrame(canonicalFrame, terminalReference)
            );
            requireNear(
                terminalFrameError,
                0.0,
                frameTolerance,
                std::string(testCase.name) + " accumulated roll"
            );

            if (testCase.transition.transitionType
                == TransitionType::Smootherstep)
            {
                measurements.symmetricAccumulatedRoll = terminalReference;
                requireNear(
                    terminalReference,
                    5.0 * (-0.2 + 0.8 * 0.5),
                    frameTolerance,
                    "symmetric transition full area"
                );
            }
            if (testCase.transition.transitionType
                == TransitionType::QuadraticEaseIn)
            {
                measurements.asymmetricAccumulatedRoll = terminalReference;
                requireNear(
                    terminalReference,
                    6.0 * (0.1 + 0.6 / 3.0),
                    frameTolerance,
                    "quadratic ease-in one-third area"
                );
                require(
                    std::abs(terminalReference - 6.0 * (0.1 + 0.6 * 0.5))
                        > 0.5,
                    "asymmetric roll incorrectly used one-half area"
                );
            }
        }

        constexpr int lastTransition = static_cast<int>(
            TransitionType::QuinticEaseInOut
        );
        for (int value = 0; value <= lastTransition; ++value)
        {
            const ScalarTransition transition{
                20.0,
                23.0,
                -0.2,
                0.4,
                static_cast<TransitionType>(value)
            };
            const auto states = integrateLocalRollRateProfile(
                startingPosition, canonicalFrame, transition, 0.8
            );
            requireStraightCenterline(
                states, startingPosition, canonicalFrame, "all roll presets"
            );
            for (const RiderLocalGeometryState& state : states)
            {
                requireValidFrame(state.frame, "all roll presets");
            }
        }
    }

    void testSignChangingAndZeroNetRoll()
    {
        const glm::dvec3 startingPosition{-2.0, 7.0, 1.0};
        const ScalarTransition signChanging{
            0.0, 6.0, -0.4, 0.4, TransitionType::Linear
        };
        const auto states = integrateLocalRollRateProfile(
            startingPosition, canonicalFrame, signChanging, 0.5
        );
        for (const RiderLocalGeometryState& state : states)
        {
            requireRollState(
                state,
                startingPosition,
                canonicalFrame,
                accumulatedRollReference(signChanging, state.distance),
                "sign-changing roll"
            );
        }
        requireStraightCenterline(
            states, startingPosition, canonicalFrame, "sign-changing roll"
        );
        measurements.signChangingFinalRoll = accumulatedRollReference(
            signChanging,
            6.0
        );
        require(
            measurements.signChangingFinalRoll == 0.0,
            "sign-changing roll reference is not exactly zero"
        );
        require(
            bitwiseEqual(states.back().frame, canonicalFrame),
            "zero-net roll did not exactly restore the starting frame"
        );
    }

    void testTransformedStartingFrames()
    {
        const glm::dvec3 startingPosition{3.0, -2.0, 9.0};
        const std::array<std::pair<CurveFrame, std::string_view>, 3> frames{
            std::pair{applyLocalPitch(canonicalFrame, 0.47), "pre-pitched"},
            std::pair{applyLocalYaw(canonicalFrame, -0.62), "pre-yawed"},
            std::pair{
                applyLocalYaw(applyLocalPitch(canonicalFrame, 0.39), -0.51),
                "pre-pitched and pre-yawed"
            }
        };

        for (const auto& [startingFrame, name] : frames)
        {
            const auto constant = integrateConstantLocalRollRate(
                startingPosition, startingFrame, 4.0, 0.37, 0.6
            );
            const ScalarTransition transition{
                50.0, 54.0, -0.1, 0.6, TransitionType::CosineEaseInOut
            };
            const auto variable = integrateLocalRollRateProfile(
                startingPosition, startingFrame, transition, 0.6
            );

            for (const RiderLocalGeometryState& state : constant)
            {
                const double angle = 0.37 * state.distance;
                requireRollState(
                    state, startingPosition, startingFrame, angle, name
                );
                measurements.maximumTransformedFrameError = std::max(
                    measurements.maximumTransformedFrameError,
                    frameError(
                        state.frame,
                        independentRolledFrame(startingFrame, angle)
                    )
                );
            }
            for (const RiderLocalGeometryState& state : variable)
            {
                const double angle = accumulatedRollReference(
                    transition,
                    state.distance
                );
                requireRollState(
                    state, startingPosition, startingFrame, angle, name
                );
                measurements.maximumTransformedFrameError = std::max(
                    measurements.maximumTransformedFrameError,
                    frameError(
                        state.frame,
                        independentRolledFrame(startingFrame, angle)
                    )
                );
            }
            requireStraightCenterline(
                constant, startingPosition, startingFrame, name
            );
            requireStraightCenterline(
                variable, startingPosition, startingFrame, name
            );
        }
    }

    void testTranslatedDomainSamplingAndDeterminism()
    {
        const glm::dvec3 startingPosition{0.25, -0.5, 1.0};
        const CurveFrame startingFrame = applyLocalYaw(
            applyLocalPitch(canonicalFrame, -0.31),
            0.44
        );
        const ScalarTransition base{
            0.0, 5.5, -0.1, 0.6, TransitionType::CosineEaseInOut
        };
        const ScalarTransition translated{
            100.0, 105.5, -0.1, 0.6, TransitionType::CosineEaseInOut
        };
        const auto baseStates = integrateLocalRollRateProfile(
            startingPosition, startingFrame, base, 2.0
        );
        const auto translatedStates = integrateLocalRollRateProfile(
            startingPosition, startingFrame, translated, 2.0
        );
        const auto repeated = integrateLocalRollRateProfile(
            startingPosition, startingFrame, base, 2.0
        );
        const std::array<double, 4> expectedDistances{0.0, 2.0, 4.0, 5.5};

        require(
            baseStates.size() == expectedDistances.size(),
            "variable roll sampling count"
        );
        require(
            translatedStates.size() == baseStates.size(),
            "translated roll sampling count"
        );
        for (std::size_t index = 0; index < baseStates.size(); ++index)
        {
            require(
                bitwiseEqual(baseStates[index].distance, expectedDistances[index]),
                "variable roll sampled distance"
            );
            require(
                index == 0
                    || baseStates[index].distance
                        > baseStates[index - 1].distance,
                "variable roll duplicated an endpoint"
            );
            measurements.maximumTranslatedPositionDifference = std::max(
                measurements.maximumTranslatedPositionDifference,
                vectorError(
                    baseStates[index].position,
                    translatedStates[index].position
                )
            );
            measurements.maximumTranslatedFrameDifference = std::max(
                measurements.maximumTranslatedFrameDifference,
                frameError(
                    baseStates[index].frame,
                    translatedStates[index].frame
                )
            );
            requireNear(
                measurements.maximumTranslatedPositionDifference,
                0.0,
                positionTolerance,
                "translated roll position"
            );
            requireNear(
                measurements.maximumTranslatedFrameDifference,
                0.0,
                frameTolerance,
                "translated roll frame"
            );
        }
        require(
            bitwiseEqual(
                baseStates.front(),
                RiderLocalGeometryState{0.0, startingPosition, startingFrame}
            ),
            "variable roll exact beginning state"
        );
        require(
            bitwiseEqual(baseStates.back().distance, 5.5),
            "variable roll exact terminal distance"
        );
        requireSeriesBitwiseEqual(
            baseStates, repeated, "deterministic repeated roll integration"
        );
    }

    void testFutureSignConvention()
    {
        const CurveFrame frame = applyLocalYaw(
            applyLocalPitch(canonicalFrame, 0.31),
            -0.43
        );
        constexpr double step = 1.0e-7;

        const CurveFrame rolled = applyRoll(frame, step);
        requireNear(
            vectorError(
                (rolled.lateral - frame.lateral) / step,
                frame.up
            ),
            0.0,
            derivativeTolerance,
            "roll sign L' = U"
        );
        requireNear(
            vectorError((rolled.up - frame.up) / step, -frame.lateral),
            0.0,
            derivativeTolerance,
            "roll sign U' = -L"
        );

        const CurveFrame pitched = applyLocalPitch(frame, step);
        requireNear(
            vectorError((pitched.tangent - frame.tangent) / step, -frame.up),
            0.0,
            derivativeTolerance,
            "pitch sign T' = -U"
        );
        requireNear(
            vectorError((pitched.up - frame.up) / step, frame.tangent),
            0.0,
            derivativeTolerance,
            "pitch sign U' = T"
        );

        const CurveFrame yawed = applyLocalYaw(frame, step);
        requireNear(
            vectorError((yawed.tangent - frame.tangent) / step, frame.lateral),
            0.0,
            derivativeTolerance,
            "yaw sign T' = L"
        );
        requireNear(
            vectorError((yawed.lateral - frame.lateral) / step, -frame.tangent),
            0.0,
            derivativeTolerance,
            "yaw sign L' = -T"
        );
    }

    void testValidation()
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();
        const ScalarTransition valid{
            0.0, 2.0, -0.1, 0.4, TransitionType::Linear
        };

        for (const glm::dvec3 position : {
                 glm::dvec3{nan, 0.0, 0.0},
                 glm::dvec3{0.0, infinity, 0.0},
                 glm::dvec3{0.0, 0.0, -infinity}})
        {
            requireThrows<std::invalid_argument>(
                [position]
                {
                    static_cast<void>(integrateConstantLocalRollRate(
                        position, canonicalFrame, 2.0, 0.2, 0.5
                    ));
                },
                "constant roll non-finite starting position"
            );
            requireThrows<std::invalid_argument>(
                [position, &valid]
                {
                    static_cast<void>(integrateLocalRollRateProfile(
                        position, canonicalFrame, valid, 0.5
                    ));
                },
                "variable roll non-finite starting position"
            );
        }

        const CurveFrame invalidFrame{
            {1.0, 0.0, 0.0},
            {1.0, 0.0, 0.0},
            {0.0, 0.0, 1.0}
        };
        requireThrows<std::invalid_argument>(
            [&invalidFrame]
            {
                static_cast<void>(integrateConstantLocalRollRate(
                    {0.0, 0.0, 0.0}, invalidFrame, 2.0, 0.2, 0.5
                ));
            },
            "constant roll invalid frame"
        );
        requireThrows<std::invalid_argument>(
            [&invalidFrame, &valid]
            {
                static_cast<void>(integrateLocalRollRateProfile(
                    {0.0, 0.0, 0.0}, invalidFrame, valid, 0.5
                ));
            },
            "variable roll invalid frame"
        );

        for (const double length : {-1.0, nan, infinity})
        {
            requireThrows<std::invalid_argument>(
                [length]
                {
                    static_cast<void>(integrateConstantLocalRollRate(
                        {0.0, 0.0, 0.0}, canonicalFrame,
                        length, 0.2, 0.5
                    ));
                },
                "invalid constant roll length"
            );
        }
        for (const double rate : {nan, infinity, -infinity})
        {
            requireThrows<std::invalid_argument>(
                [rate]
                {
                    static_cast<void>(integrateConstantLocalRollRate(
                        {0.0, 0.0, 0.0}, canonicalFrame,
                        2.0, rate, 0.5
                    ));
                },
                "invalid constant roll rate"
            );
        }
        for (const double spacing : {0.0, -1.0, nan, infinity})
        {
            requireThrows<std::invalid_argument>(
                [spacing]
                {
                    static_cast<void>(integrateConstantLocalRollRate(
                        {0.0, 0.0, 0.0}, canonicalFrame,
                        2.0, 0.2, spacing
                    ));
                },
                "invalid constant roll spacing"
            );
            requireThrows<std::invalid_argument>(
                [spacing, &valid]
                {
                    static_cast<void>(integrateLocalRollRateProfile(
                        {0.0, 0.0, 0.0}, canonicalFrame, valid, spacing
                    ));
                },
                "invalid variable roll spacing"
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
                    static_cast<void>(integrateLocalRollRateProfile(
                        {0.0, 0.0, 0.0},
                        canonicalFrame,
                        transition,
                        0.5
                    ));
                },
                "invalid roll-rate transition"
            );
        }

        requireThrows<std::domain_error>(
            []
            {
                static_cast<void>(integrateConstantLocalRollRate(
                    {0.0, 0.0, 0.0},
                    canonicalFrame,
                    4.0,
                    std::numeric_limits<double>::max() / 2.0,
                    1.0
                ));
            },
            "non-representable constant roll angle"
        );
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;
    const std::vector<Test> tests{
        {"constant analytic rolls and sampling", testConstantAnalyticRolls},
        {"constant profile equivalence", testConstantProfileEquivalence},
        {
            "variable references and all presets",
            testVariableRollReferencesAndPresets
        },
        {"sign-changing and zero-net roll", testSignChangingAndZeroNetRoll},
        {"transformed starting frames", testTransformedStartingFrames},
        {
            "translated domain, sampling, and determinism",
            testTranslatedDomainSamplingAndDeterminism
        },
        {"future full-system sign convention", testFutureSignConvention},
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
        std::cerr << failures << " roll test group(s) failed.\n";
        return 1;
    }

    std::cout << std::setprecision(17);
    std::cout << "[METRIC] maximum analytic frame error: "
              << measurements.maximumAnalyticFrameError << '\n';
    std::cout << "[METRIC] maximum exact position error: "
              << measurements.maximumPositionError << '\n';
    std::cout << "[METRIC] maximum frame invariant error: "
              << measurements.maximumFrameInvariantError << '\n';
    std::cout << "[METRIC] maximum tangent-preservation error: "
              << measurements.maximumTangentError << '\n';
    std::cout << "[METRIC] maximum centerline collinearity error: "
              << measurements.maximumCollinearityError << '\n';
    std::cout << "[METRIC] maximum centerline curvature: "
              << measurements.maximumCurvature << '\n';
    std::cout << "[METRIC] constant-profile equivalence error: "
              << measurements.maximumConstantProfileError << '\n';
    std::cout << "[METRIC] symmetric accumulated roll: "
              << measurements.symmetricAccumulatedRoll << '\n';
    std::cout << "[METRIC] asymmetric accumulated roll: "
              << measurements.asymmetricAccumulatedRoll << '\n';
    std::cout << "[METRIC] sign-changing final roll: "
              << measurements.signChangingFinalRoll << '\n';
    std::cout << "[METRIC] transformed-frame error: "
              << measurements.maximumTransformedFrameError << '\n';
    std::cout << "[METRIC] translated-domain differences: position="
              << measurements.maximumTranslatedPositionDifference
              << ", frame="
              << measurements.maximumTranslatedFrameDifference << '\n';
    std::cout << tests.size() << " roll test groups passed.\n";
    return 0;
}
