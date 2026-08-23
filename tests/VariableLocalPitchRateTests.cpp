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
    using quantum::coaster::integrateLocalPitchRateProfile;
    using quantum::coaster::RiderLocalGeometryState;
    using quantum::geometry::applyLocalPitch;
    using quantum::geometry::applyRoll;
    using quantum::geometry::CurveFrame;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

    constexpr double positionTolerance = 2.0e-11;
    constexpr double orientationTolerance = 2.0e-12;
    constexpr std::size_t referenceIntervalCount = 131'072;

    const CurveFrame canonicalFrame{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    struct Measurements
    {
        double maximumConstantPositionError = 0.0;
        double maximumConstantOrientationError = 0.0;
        double maximumReferencePositionError = 0.0;
        double maximumReferenceOrientationError = 0.0;
        double maximumFrameInvariantError = 0.0;
        double progressiveFirstAverageCurvature = 0.0;
        double progressiveLastAverageCurvature = 0.0;
        double reverseFirstAverageCurvature = 0.0;
        double reverseLastAverageCurvature = 0.0;
        double signChangingEndpointExcursion = 0.0;
        double rolledPlaneError = 0.0;
        double translatedPositionDifference = 0.0;
        double translatedOrientationDifference = 0.0;
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
        const double invariantError = std::max({
            std::abs(vectorMagnitude(frame.tangent) - 1.0),
            std::abs(vectorMagnitude(frame.lateral) - 1.0),
            std::abs(vectorMagnitude(frame.up) - 1.0),
            std::abs(glm::dot(frame.tangent, frame.lateral)),
            std::abs(glm::dot(frame.tangent, frame.up)),
            std::abs(glm::dot(frame.lateral, frame.up)),
            pointError(
                glm::cross(frame.tangent, frame.lateral),
                frame.up
            )
        });
        measurements.maximumFrameInvariantError = std::max(
            measurements.maximumFrameInvariantError,
            invariantError
        );
        requireNear(
            invariantError,
            0.0,
            orientationTolerance,
            std::string(context) + " orthonormality and handedness"
        );
    }

    [[nodiscard]] long double referenceTransitionIntegral(
        const TransitionType type,
        const long double normalizedProgress
    )
    {
        switch (type)
        {
        case TransitionType::Linear:
            return 0.5L * normalizedProgress * normalizedProgress;

        case TransitionType::Smoothstep:
            return normalizedProgress
                * normalizedProgress
                * normalizedProgress
                * (1.0L - 0.5L * normalizedProgress);

        case TransitionType::Smootherstep:
            return normalizedProgress
                * normalizedProgress
                * normalizedProgress
                * normalizedProgress
                * (2.5L - 3.0L * normalizedProgress
                    + normalizedProgress * normalizedProgress);
        }

        throw TestFailure("unsupported reference transition type");
    }

    [[nodiscard]] long double referenceAccumulatedPitch(
        const ScalarTransition& profile,
        const long double traveledDistance
    )
    {
        const long double length = static_cast<long double>(
            profile.domainEnd - profile.domainBegin
        );
        const long double normalizedProgress = traveledDistance / length;
        return static_cast<long double>(profile.valueBegin) * traveledDistance
            + static_cast<long double>(
                profile.valueEnd - profile.valueBegin
            ) * length * referenceTransitionIntegral(
                profile.transitionType,
                normalizedProgress
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

        // Composite Simpson integration is deliberately independent of the
        // production Gauss-Legendre implementation.
        const long double step = static_cast<long double>(traveledDistance)
            / static_cast<long double>(referenceIntervalCount);
        long double tangentIntegral = 0.0L;
        long double upIntegral = 0.0L;

        for (std::size_t index = 0;
             index <= referenceIntervalCount;
             ++index)
        {
            const long double distance =
                static_cast<long double>(index) * step;
            const long double pitch = referenceAccumulatedPitch(
                profile,
                distance
            );
            const long double weight = index == 0
                    || index == referenceIntervalCount
                ? 1.0L
                : (index % 2 == 0 ? 2.0L : 4.0L);
            tangentIntegral += weight * std::cos(pitch);
            upIntegral -= weight * std::sin(pitch);
        }

        tangentIntegral *= step / 3.0L;
        upIntegral *= step / 3.0L;
        return startingPosition
            + static_cast<double>(tangentIntegral) * startingFrame.tangent
            + static_cast<double>(upIntegral) * startingFrame.up;
    }

    void requireMatchesReference(
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
            const double expectedPitch = static_cast<double>(
                referenceAccumulatedPitch(profile, state.distance)
            );
            const CurveFrame expectedFrame = applyLocalPitch(
                startingFrame,
                expectedPitch
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
                std::string(context) + " independent position reference"
            );
            requireNear(
                orientationError,
                0.0,
                orientationTolerance,
                std::string(context) + " analytic orientation reference"
            );
            require(
                state.frame.lateral == startingFrame.lateral,
                std::string(context) + " changed the lateral axis"
            );
            requireValidFrame(state.frame, context);
        }
    }

    template<typename Value>
    [[nodiscard]] bool bitwiseEqual(const Value first, const Value second)
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

    void testConstantProfileEquivalence()
    {
        struct ConstantCase
        {
            double length;
            double rate;
            double spacing;
        };

        const double quarterCircleLength = std::numbers::pi / (2.0 * 0.2);
        const ConstantCase cases[]{
            {7.3, 0.0, 0.8},
            {quarterCircleLength, 0.2, quarterCircleLength / 9.0},
            {quarterCircleLength, -0.2, quarterCircleLength / 9.0},
            {11.7, 0.13, 2.0},
            {11.7, -0.13, 2.0}
        };

        for (const TransitionType type : {
                 TransitionType::Linear,
                 TransitionType::Smoothstep,
                 TransitionType::Smootherstep
             })
        {
            for (const ConstantCase& testCase : cases)
            {
                const ScalarTransition profile{
                    0.0,
                    testCase.length,
                    testCase.rate,
                    testCase.rate,
                    type
                };
                const std::vector<RiderLocalGeometryState> expected =
                    integrateConstantLocalPitchRate(
                        {1.0, -2.0, 3.0},
                        canonicalFrame,
                        testCase.length,
                        testCase.rate,
                        testCase.spacing
                    );
                const std::vector<RiderLocalGeometryState> actual =
                    integrateLocalPitchRateProfile(
                        {1.0, -2.0, 3.0},
                        canonicalFrame,
                        profile,
                        testCase.spacing
                    );

                require(actual.size() == expected.size(),
                        "constant profile sample count differs");
                for (std::size_t index = 0; index < actual.size(); ++index)
                {
                    const double positionError = pointError(
                        actual[index].position,
                        expected[index].position
                    );
                    const double orientationError = frameError(
                        actual[index].frame,
                        expected[index].frame
                    );
                    measurements.maximumConstantPositionError = std::max(
                        measurements.maximumConstantPositionError,
                        positionError
                    );
                    measurements.maximumConstantOrientationError = std::max(
                        measurements.maximumConstantOrientationError,
                        orientationError
                    );
                    require(
                        bitwiseEqual(actual[index].distance,
                                     expected[index].distance)
                            && bitwiseEqual(actual[index].position,
                                            expected[index].position)
                            && bitwiseEqual(actual[index].frame,
                                            expected[index].frame),
                        "constant profile did not exactly delegate to the constant solver"
                    );
                }
            }
        }
    }

    void testAccumulatedOrientationReferences()
    {
        constexpr double length = 20.0;
        constexpr double rateBegin = -0.2;
        constexpr double rateEnd = 0.6;
        constexpr double expectedFullPitch =
            length * (rateBegin + rateEnd) / 2.0;
        constexpr std::pair<TransitionType, double> quarterPitch[]{
            {TransitionType::Linear, -0.5},
            {TransitionType::Smoothstep, -0.78125},
            {TransitionType::Smootherstep, -0.88671875}
        };
        std::vector<double> observedQuarterPitches;

        for (const auto [type, expectedQuarterPitch] : quarterPitch)
        {
            const ScalarTransition profile{
                0.0, length, rateBegin, rateEnd, type
            };
            const std::vector<RiderLocalGeometryState> states =
                integrateLocalPitchRateProfile(
                    glm::dvec3{0.0}, canonicalFrame, profile, 5.0
                );
            const CurveFrame expectedQuarterFrame = applyLocalPitch(
                canonicalFrame,
                expectedQuarterPitch
            );
            const CurveFrame expectedFullFrame = applyLocalPitch(
                canonicalFrame,
                expectedFullPitch
            );
            const double quarterError = frameError(
                states[1].frame,
                expectedQuarterFrame
            );
            const double fullError = frameError(
                states.back().frame,
                expectedFullFrame
            );
            measurements.maximumReferenceOrientationError = std::max({
                measurements.maximumReferenceOrientationError,
                quarterError,
                fullError
            });
            requireNear(
                quarterError,
                0.0,
                orientationTolerance,
                "intermediate transition accumulated pitch"
            );
            requireNear(
                fullError,
                0.0,
                orientationTolerance,
                "full-domain mean accumulated pitch"
            );
            observedQuarterPitches.push_back(expectedQuarterPitch);
        }

        require(
            observedQuarterPitches[0] != observedQuarterPitches[1]
                && observedQuarterPitches[1] != observedQuarterPitches[2],
            "transition types need distinct intermediate accumulated pitches"
        );
    }

    void testLinearOrientationAtMultipleDistances()
    {
        const ScalarTransition profile{
            10.0, 30.0, 0.02, 0.18, TransitionType::Linear
        };
        const std::vector<RiderLocalGeometryState> states =
            integrateLocalPitchRateProfile(
                {-2.0, 3.0, 5.0}, canonicalFrame, profile, 2.5
            );
        constexpr double slope = (0.18 - 0.02) / 20.0;

        for (const RiderLocalGeometryState& state : states)
        {
            const double expectedPitch = 0.02 * state.distance
                + 0.5 * slope * state.distance * state.distance;
            const double error = frameError(
                state.frame,
                applyLocalPitch(canonicalFrame, expectedPitch)
            );
            measurements.maximumReferenceOrientationError = std::max(
                measurements.maximumReferenceOrientationError,
                error
            );
            requireNear(
                error,
                0.0,
                orientationTolerance,
                "linear a*s + 0.5*b*s^2 orientation"
            );
        }
    }

    void testProgressiveCurvatureGeometry()
    {
        const glm::dvec3 startingPosition{2.0, -1.0, 4.0};
        const ScalarTransition profile{
            0.0, 12.0, 0.0, 0.24, TransitionType::Smoothstep
        };
        const std::vector<RiderLocalGeometryState> states =
            integrateLocalPitchRateProfile(
                startingPosition, canonicalFrame, profile, 2.0
            );
        requireMatchesReference(
            startingPosition,
            canonicalFrame,
            profile,
            states,
            "progressive curvature"
        );

        double previousAverage = -1.0;
        for (std::size_t index = 1; index < states.size(); ++index)
        {
            const double firstPitch = static_cast<double>(
                referenceAccumulatedPitch(
                    profile,
                    states[index - 1].distance
                )
            );
            const double secondPitch = static_cast<double>(
                referenceAccumulatedPitch(profile, states[index].distance)
            );
            const double averageCurvature =
                (secondPitch - firstPitch)
                / (states[index].distance - states[index - 1].distance);
            require(
                averageCurvature > previousAverage,
                "progressive profile average curvature did not increase"
            );
            previousAverage = averageCurvature;
            if (index == 1)
            {
                measurements.progressiveFirstAverageCurvature =
                    averageCurvature;
            }
            if (index + 1 == states.size())
            {
                measurements.progressiveLastAverageCurvature =
                    averageCurvature;
            }
        }

        require(
            measurements.progressiveFirstAverageCurvature
                < measurements.progressiveLastAverageCurvature,
            "progressive curvature did not tighten"
        );
    }

    void testReverseAndSignChangingProfiles()
    {
        const ScalarTransition reverse{
            0.0, 12.0, 0.24, 0.0, TransitionType::Smoothstep
        };
        const std::vector<RiderLocalGeometryState> reverseStates =
            integrateLocalPitchRateProfile(
                glm::dvec3{0.0}, canonicalFrame, reverse, 2.0
            );
        requireMatchesReference(
            glm::dvec3{0.0},
            canonicalFrame,
            reverse,
            reverseStates,
            "reverse profile"
        );

        double previousAverage = std::numeric_limits<double>::infinity();
        for (std::size_t index = 1; index < reverseStates.size(); ++index)
        {
            const double firstPitch = static_cast<double>(
                referenceAccumulatedPitch(
                    reverse,
                    reverseStates[index - 1].distance
                )
            );
            const double secondPitch = static_cast<double>(
                referenceAccumulatedPitch(
                    reverse,
                    reverseStates[index].distance
                )
            );
            const double averageCurvature =
                (secondPitch - firstPitch)
                / (reverseStates[index].distance
                    - reverseStates[index - 1].distance);
            require(
                averageCurvature < previousAverage,
                "reverse profile average curvature did not decrease"
            );
            previousAverage = averageCurvature;
            if (index == 1)
            {
                measurements.reverseFirstAverageCurvature = averageCurvature;
            }
            if (index + 1 == reverseStates.size())
            {
                measurements.reverseLastAverageCurvature = averageCurvature;
            }
        }

        const ScalarTransition signChanging{
            0.0, 10.0, -0.1, 0.1, TransitionType::Linear
        };
        const std::vector<RiderLocalGeometryState> signChangingStates =
            integrateLocalPitchRateProfile(
                glm::dvec3{0.0}, canonicalFrame, signChanging, 1.0
            );
        requireMatchesReference(
            glm::dvec3{0.0},
            canonicalFrame,
            signChanging,
            signChangingStates,
            "sign-changing profile"
        );
        requireNear(
            frameError(signChangingStates.back().frame, canonicalFrame),
            0.0,
            orientationTolerance,
            "sign-changing endpoint orientation"
        );
        measurements.signChangingEndpointExcursion = std::abs(
            signChangingStates.back().position.z
        );
        require(
            measurements.signChangingEndpointExcursion > 0.1,
            "sign-changing geometry unexpectedly collapsed to a straight line"
        );
    }

    void testRolledStartingFrame()
    {
        const glm::dvec3 startingPosition{-3.0, 2.0, 1.0};
        const CurveFrame rolledFrame = applyRoll(canonicalFrame, 0.63);
        const ScalarTransition profile{
            0.0, 14.0, 0.02, 0.17, TransitionType::Smootherstep
        };
        const std::vector<RiderLocalGeometryState> states =
            integrateLocalPitchRateProfile(
                startingPosition, rolledFrame, profile, 2.0
            );
        requireMatchesReference(
            startingPosition,
            rolledFrame,
            profile,
            states,
            "rolled starting frame"
        );

        for (const RiderLocalGeometryState& state : states)
        {
            const double planeError = std::abs(glm::dot(
                state.position - startingPosition,
                rolledFrame.lateral
            ));
            measurements.rolledPlaneError = std::max(
                measurements.rolledPlaneError,
                planeError
            );
            requireNear(
                planeError,
                0.0,
                positionTolerance,
                "rolled rider-local curvature plane"
            );
        }

        require(
            std::abs(states.back().position.y - startingPosition.y) > 0.1,
            "rolled profile snapped to the unrolled world-vertical plane"
        );
    }

    void testTranslatedDomainEquivalence()
    {
        const ScalarTransition base{
            0.0, 20.0, -0.03, 0.16, TransitionType::Smootherstep
        };
        const ScalarTransition translated{
            100.0, 120.0, -0.03, 0.16, TransitionType::Smootherstep
        };
        const std::vector<RiderLocalGeometryState> baseStates =
            integrateLocalPitchRateProfile(
                {1.0, 2.0, 3.0}, canonicalFrame, base, 2.0
            );
        const std::vector<RiderLocalGeometryState> translatedStates =
            integrateLocalPitchRateProfile(
                {1.0, 2.0, 3.0}, canonicalFrame, translated, 2.0
            );

        require(baseStates.size() == translatedStates.size(),
                "translated domain changed sample count");
        require(baseStates.front().distance == 0.0
                    && translatedStates.front().distance == 0.0,
                "translated domain did not begin at zero traveled distance");
        require(baseStates.back().distance == 20.0
                    && translatedStates.back().distance == 20.0,
                "translated domain did not end at profile length");

        for (std::size_t index = 0; index < baseStates.size(); ++index)
        {
            require(
                baseStates[index].distance == translatedStates[index].distance,
                "translated domain changed traveled sample distance"
            );
            measurements.translatedPositionDifference = std::max(
                measurements.translatedPositionDifference,
                pointError(
                    baseStates[index].position,
                    translatedStates[index].position
                )
            );
            measurements.translatedOrientationDifference = std::max(
                measurements.translatedOrientationDifference,
                frameError(
                    baseStates[index].frame,
                    translatedStates[index].frame
                )
            );
        }

        requireNear(
            measurements.translatedPositionDifference,
            0.0,
            positionTolerance,
            "translated-domain geometry"
        );
        requireNear(
            measurements.translatedOrientationDifference,
            0.0,
            orientationTolerance,
            "translated-domain orientation"
        );
    }

    void testOutputSamplingAndDeterminism()
    {
        const ScalarTransition profile{
            0.0, 10.0, 0.01, 0.12, TransitionType::Smoothstep
        };
        const std::vector<RiderLocalGeometryState> states =
            integrateLocalPitchRateProfile(
                {4.0, 5.0, 6.0}, canonicalFrame, profile, 3.0
            );
        constexpr double expectedDistances[]{0.0, 3.0, 6.0, 9.0, 10.0};
        require(states.size() == std::size(expectedDistances),
                "output sampling count");
        require(
            states.front().distance == 0.0
                && states.front().position == glm::dvec3{4.0, 5.0, 6.0}
                && states.front().frame == canonicalFrame,
            "beginning state must preserve the exact integration inputs"
        );
        for (std::size_t index = 0; index < states.size(); ++index)
        {
            require(
                states[index].distance == expectedDistances[index],
                "output sampling distance"
            );
            if (index > 0)
            {
                require(
                    states[index].distance > states[index - 1].distance,
                    "output sampling duplicated an endpoint"
                );
            }
        }

        const std::vector<RiderLocalGeometryState> coarse =
            integrateLocalPitchRateProfile(
                {4.0, 5.0, 6.0}, canonicalFrame, profile, 20.0
            );
        require(coarse.size() == 2
                    && coarse.front().distance == 0.0
                    && coarse.back().distance == 10.0,
                "spacing greater than length must return both endpoints");
        require(
            pointError(coarse.back().position, states.back().position)
                <= positionTolerance,
            "coarse output spacing reduced terminal position accuracy"
        );

        const std::vector<RiderLocalGeometryState> repeated =
            integrateLocalPitchRateProfile(
                {4.0, 5.0, 6.0}, canonicalFrame, profile, 3.0
            );
        require(repeated.size() == states.size(),
                "deterministic run changed sample count");
        for (std::size_t index = 0; index < states.size(); ++index)
        {
            require(
                bitwiseEqual(states[index].distance, repeated[index].distance)
                    && bitwiseEqual(states[index].position,
                                    repeated[index].position)
                    && bitwiseEqual(states[index].frame,
                                    repeated[index].frame),
                "repeated integration changed component bits"
            );
        }
    }

    void testValidation()
    {
        const ScalarTransition valid{
            0.0, 10.0, 0.0, 0.1, TransitionType::Linear
        };
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        constexpr double infinity = std::numeric_limits<double>::infinity();

        for (const glm::dvec3 position : {
                 glm::dvec3{nan, 0.0, 0.0},
                 glm::dvec3{0.0, infinity, 0.0},
                 glm::dvec3{0.0, 0.0, -infinity}
             })
        {
            requireThrows<std::invalid_argument>(
                [position, &valid]
                {
                    static_cast<void>(integrateLocalPitchRateProfile(
                        position, canonicalFrame, valid, 1.0
                    ));
                },
                "non-finite starting position"
            );
        }

        const CurveFrame invalidFrame{
            {2.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };
        requireThrows<std::invalid_argument>(
            [&valid, &invalidFrame]
            {
                static_cast<void>(integrateLocalPitchRateProfile(
                    glm::dvec3{0.0}, invalidFrame, valid, 1.0
                ));
            },
            "invalid starting frame"
        );

        for (const double spacing : {0.0, -1.0, nan, infinity})
        {
            requireThrows<std::invalid_argument>(
                [spacing, &valid]
                {
                    static_cast<void>(integrateLocalPitchRateProfile(
                        glm::dvec3{0.0}, canonicalFrame, valid, spacing
                    ));
                },
                "invalid integration spacing"
            );
        }

        std::vector<ScalarTransition> invalidTransitions;
        invalidTransitions.push_back({
            1.0, 1.0, 0.0, 0.1, TransitionType::Linear
        });
        invalidTransitions.push_back({
            2.0, 1.0, 0.0, 0.1, TransitionType::Linear
        });
        invalidTransitions.push_back({
            nan, 1.0, 0.0, 0.1, TransitionType::Linear
        });
        invalidTransitions.push_back({
            0.0, infinity, 0.0, 0.1, TransitionType::Linear
        });
        invalidTransitions.push_back({
            0.0, 1.0, nan, 0.1, TransitionType::Linear
        });
        invalidTransitions.push_back({
            0.0, 1.0, 0.0, infinity, TransitionType::Linear
        });
        invalidTransitions.push_back({
            0.0,
            1.0,
            0.0,
            0.1,
            static_cast<TransitionType>(-1)
        });

        for (const ScalarTransition& transition : invalidTransitions)
        {
            requireThrows<std::invalid_argument>(
                [&transition]
                {
                    static_cast<void>(integrateLocalPitchRateProfile(
                        glm::dvec3{0.0},
                        canonicalFrame,
                        transition,
                        1.0
                    ));
                },
                "invalid pitch-rate transition"
            );
        }
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;
    const std::vector<Test> tests{
        {"constant-profile equivalence", testConstantProfileEquivalence},
        {
            "accumulated orientation references",
            testAccumulatedOrientationReferences
        },
        {
            "linear orientation at multiple distances",
            testLinearOrientationAtMultipleDistances
        },
        {
            "progressive-curvature geometry",
            testProgressiveCurvatureGeometry
        },
        {
            "reverse and sign-changing profiles",
            testReverseAndSignChangingProfiles
        },
        {"rolled starting frame", testRolledStartingFrame},
        {"translated-domain equivalence", testTranslatedDomainEquivalence},
        {
            "output sampling and determinism",
            testOutputSamplingAndDeterminism
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
    std::cout << "[METRIC] maximum constant-equivalence position error: "
              << measurements.maximumConstantPositionError << '\n';
    std::cout << "[METRIC] maximum constant-equivalence orientation error: "
              << measurements.maximumConstantOrientationError << '\n';
    std::cout << "[METRIC] maximum independent-reference position error: "
              << measurements.maximumReferencePositionError << '\n';
    std::cout << "[METRIC] maximum analytic orientation error: "
              << measurements.maximumReferenceOrientationError << '\n';
    std::cout << "[METRIC] maximum frame invariant error: "
              << measurements.maximumFrameInvariantError << '\n';
    std::cout << "[METRIC] progressive average curvature: first="
              << measurements.progressiveFirstAverageCurvature
              << ", last="
              << measurements.progressiveLastAverageCurvature << '\n';
    std::cout << "[METRIC] reverse average curvature: first="
              << measurements.reverseFirstAverageCurvature
              << ", last=" << measurements.reverseLastAverageCurvature
              << '\n';
    std::cout << "[METRIC] sign-changing endpoint vertical excursion: "
              << measurements.signChangingEndpointExcursion << '\n';
    std::cout << "[METRIC] maximum rolled curvature-plane error: "
              << measurements.rolledPlaneError << '\n';
    std::cout << "[METRIC] translated-domain differences: position="
              << measurements.translatedPositionDifference
              << ", orientation="
              << measurements.translatedOrientationDifference << '\n';
    std::cout << tests.size() << " test groups passed.\n";
    return 0;
}
