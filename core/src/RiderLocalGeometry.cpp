#include <quantum/coaster/RiderLocalGeometry.hpp>

#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace quantum::coaster
{
    namespace
    {
        [[nodiscard]] bool isFinitePosition(
            const glm::dvec3& position
        ) noexcept
        {
            return std::isfinite(position.x)
                && std::isfinite(position.y)
                && std::isfinite(position.z);
        }

        // sin(x) / x evaluated without cancellation near zero.
        [[nodiscard]] double sinc(const double value) noexcept
        {
            if (value == 0.0)
            {
                return 1.0;
            }

            if (std::abs(value) < 1.0e-4)
            {
                const double square = value * value;
                return 1.0
                    - square / 6.0
                    + square * square / 120.0;
            }

            return std::sin(value) / value;
        }

        // (1 - cos(x)) / x evaluated without cancellation near zero.
        [[nodiscard]] double cosc(const double value) noexcept
        {
            if (value == 0.0)
            {
                return 0.0;
            }

            if (std::abs(value) < 1.0e-3)
            {
                const double square = value * value;
                return value * (
                    0.5
                    - square / 24.0
                    + square * square / 720.0
                );
            }

            return (1.0 - std::cos(value)) / value;
        }

        // 1 - sin(x) / x evaluated without cancellation near zero.
        [[nodiscard]] double oneMinusSinc(const double value) noexcept
        {
            if (std::abs(value) < 1.0e-3)
            {
                const double square = value * value;
                return square * (
                    1.0 / 6.0
                    - square / 120.0
                    + square * square / 5'040.0
                );
            }

            return 1.0 - std::sin(value) / value;
        }

        constexpr std::array<double, 8> gaussLegendreNodes{
            0.09501250983763744,
            0.28160355077925891,
            0.45801677765722739,
            0.61787624440264375,
            0.75540440835500303,
            0.86563120238783174,
            0.94457502307323258,
            0.98940093499164993
        };

        constexpr std::array<double, 8> gaussLegendreWeights{
            0.18945061045506850,
            0.18260341504492359,
            0.16915651939500254,
            0.14959598881657673,
            0.12462897125553387,
            0.095158511682492785,
            0.062253523938647893,
            0.027152459411754095
        };

        [[nodiscard]] glm::dvec3 pitchTangent(
            const geometry::CurveFrame& startingFrame,
            const double accumulatedPitch
        ) noexcept
        {
            return std::cos(accumulatedPitch) * startingFrame.tangent
                - std::sin(accumulatedPitch) * startingFrame.up;
        }

        [[nodiscard]] glm::dvec3 yawTangent(
            const geometry::CurveFrame& startingFrame,
            const double accumulatedYaw
        ) noexcept
        {
            return std::cos(accumulatedYaw) * startingFrame.tangent
                + std::sin(accumulatedYaw) * startingFrame.lateral;
        }

        using ApplyLocalRotation = geometry::CurveFrame (*)(
            const geometry::CurveFrame&,
            double
        );
        using EvaluateRotatedTangent = glm::dvec3 (*)(
            const geometry::CurveFrame&,
            double
        ) noexcept;

        struct LocalRotationOperations
        {
            ApplyLocalRotation applyRotation;
            EvaluateRotatedTangent evaluateTangent;
            const char* invalidRateMessage;
            const char* nonFiniteConstantStepAngleMessage;
            const char* nonFiniteProfileIntervalAngleMessage;
            const char* excessivePanelCountMessage;
        };

        constexpr LocalRotationOperations pitchOperations{
            geometry::applyLocalPitch,
            pitchTangent,
            "Rider-local pitch rate must be finite and expressed in radians per coordinate unit.",
            "Rider-local pitch change over one integration interval is not representable as a finite angle.",
            "Rider-local pitch change over an integration interval is not representable as a finite angle.",
            "Variable rider-local pitch integration requires too many quadrature panels."
        };

        constexpr LocalRotationOperations yawOperations{
            geometry::applyLocalYaw,
            yawTangent,
            "Rider-local yaw rate must be finite and expressed in radians per coordinate unit.",
            "Rider-local yaw change over one integration interval is not representable as a finite angle.",
            "Rider-local yaw change over an integration interval is not representable as a finite angle.",
            "Variable rider-local yaw integration requires too many quadrature panels."
        };

        [[nodiscard]] glm::dquat orientationFromFrame(
            const geometry::CurveFrame& frame
        )
        {
            const glm::dmat3 frameMatrix{
                frame.tangent,
                frame.lateral,
                frame.up
            };
            return glm::normalize(glm::quat_cast(frameMatrix));
        }

        [[nodiscard]] geometry::CurveFrame frameFromOrientation(
            const glm::dquat& orientation
        ) noexcept
        {
            return {
                orientation * glm::dvec3{1.0, 0.0, 0.0},
                orientation * glm::dvec3{0.0, 1.0, 0.0},
                orientation * glm::dvec3{0.0, 0.0, 1.0}
            };
        }

        [[nodiscard]] glm::dquat localRotationQuaternion(
            const glm::dvec3& rotationVector
        )
        {
            const double angle = std::hypot(
                rotationVector.x,
                rotationVector.y,
                rotationVector.z
            );

            if (!std::isfinite(angle))
            {
                throw std::domain_error(
                    "Coupled rider-local frame rotation is not representable as a finite angle."
                );
            }

            if (angle == 0.0)
            {
                return {1.0, 0.0, 0.0, 0.0};
            }

            const double halfAngle = 0.5 * angle;
            const double vectorScale = 0.5 * sinc(halfAngle);
            return glm::normalize(glm::dquat{
                std::cos(halfAngle),
                vectorScale * rotationVector.x,
                vectorScale * rotationVector.y,
                vectorScale * rotationVector.z
            });
        }

        [[nodiscard]] glm::dquat composeLocalRotation(
            const glm::dquat& orientation,
            const glm::dvec3& rotationVector
        )
        {
            return glm::normalize(
                orientation * localRotationQuaternion(rotationVector)
            );
        }

        [[nodiscard]] double transitionDomainValue(
            const math::ScalarTransition& transition,
            const double traveledDistance,
            const double profileLength
        ) noexcept
        {
            return traveledDistance == profileLength
                ? transition.domainEnd
                : transition.domainBegin + traveledDistance;
        }

        [[nodiscard]] glm::dvec3 coupledLocalRate(
            const math::ScalarTransition* rollRateTransition,
            const math::ScalarTransition& pitchRateTransition,
            const math::ScalarTransition& yawRateTransition,
            const double traveledDistance,
            const double profileLength
        )
        {
            const double rollRate = rollRateTransition == nullptr
                ? 0.0
                : math::evaluateScalarTransition(
                    *rollRateTransition,
                    transitionDomainValue(
                        *rollRateTransition,
                        traveledDistance,
                        profileLength
                    )
                );
            const double pitchDomainValue = transitionDomainValue(
                pitchRateTransition,
                traveledDistance,
                profileLength
            );
            const double yawDomainValue = transitionDomainValue(
                yawRateTransition,
                traveledDistance,
                profileLength
            );
            return {
                rollRate,
                math::evaluateScalarTransition(
                    pitchRateTransition,
                    pitchDomainValue
                ),
                math::evaluateScalarTransition(
                    yawRateTransition,
                    yawDomainValue
                )
            };
        }

        // Fourth-order two-node Gauss-Legendre Magnus increment for F'=F*A.
        // A is the skew matrix of local angular rate (roll, pitch, yaw).
        // A null roll profile preserves the established pitch/yaw path.
        // The positive commutator sign is the right-acting counterpart of the
        // usual left-acting Magnus formula.
        [[nodiscard]] glm::dvec3 coupledMagnusRotationVector(
            const math::ScalarTransition* rollRateTransition,
            const math::ScalarTransition& pitchRateTransition,
            const math::ScalarTransition& yawRateTransition,
            const double traveledBegin,
            const double intervalLength,
            const double profileLength
        )
        {
            constexpr double nodeOffset =
                0.5 / std::numbers::sqrt3_v<double>;
            constexpr double lowerNode = 0.5 - nodeOffset;
            constexpr double upperNode = 0.5 + nodeOffset;
            const glm::dvec3 lowerRate = coupledLocalRate(
                rollRateTransition,
                pitchRateTransition,
                yawRateTransition,
                traveledBegin + lowerNode * intervalLength,
                profileLength
            );
            const glm::dvec3 upperRate = coupledLocalRate(
                rollRateTransition,
                pitchRateTransition,
                yawRateTransition,
                traveledBegin + upperNode * intervalLength,
                profileLength
            );
            const double commutatorScale =
                std::numbers::sqrt3_v<double>
                * intervalLength * intervalLength / 12.0;
            const glm::dvec3 rotationVector =
                0.5 * intervalLength * (lowerRate + upperRate)
                + commutatorScale * glm::cross(lowerRate, upperRate);

            if (!isFinitePosition(rotationVector))
            {
                throw std::domain_error(
                    "Coupled rider-local frame change over an integration interval is not finite."
                );
            }

            return rotationVector;
        }

        struct CoupledIntegrationState
        {
            double distance;
            glm::dvec3 position;
            glm::dquat orientation;
        };

        void integrateCoupledSubstep(
            CoupledIntegrationState& state,
            const math::ScalarTransition* rollRateTransition,
            const math::ScalarTransition& pitchRateTransition,
            const math::ScalarTransition& yawRateTransition,
            const double stepLength,
            const double profileLength
        )
        {
            constexpr double nodeOffset =
                0.5 / std::numbers::sqrt3_v<double>;
            constexpr double lowerNode = 0.5 - nodeOffset;
            constexpr double upperNode = 0.5 + nodeOffset;

            const glm::dquat lowerOrientation = composeLocalRotation(
                state.orientation,
                coupledMagnusRotationVector(
                    rollRateTransition,
                    pitchRateTransition,
                    yawRateTransition,
                    state.distance,
                    lowerNode * stepLength,
                    profileLength
                )
            );
            const glm::dquat upperOrientation = composeLocalRotation(
                state.orientation,
                coupledMagnusRotationVector(
                    rollRateTransition,
                    pitchRateTransition,
                    yawRateTransition,
                    state.distance,
                    upperNode * stepLength,
                    profileLength
                )
            );
            const glm::dvec3 lowerTangent =
                lowerOrientation * glm::dvec3{1.0, 0.0, 0.0};
            const glm::dvec3 upperTangent =
                upperOrientation * glm::dvec3{1.0, 0.0, 0.0};

            state.position += 0.5 * stepLength
                * (lowerTangent + upperTangent);
            state.orientation = composeLocalRotation(
                state.orientation,
                coupledMagnusRotationVector(
                    rollRateTransition,
                    pitchRateTransition,
                    yawRateTransition,
                    state.distance,
                    stepLength,
                    profileLength
                )
            );
            state.distance += stepLength;

            if (!isFinitePosition(state.position))
            {
                throw std::domain_error(
                    "Rider-local geometry integration produced a non-finite position."
                );
            }
        }

        [[nodiscard]] std::size_t coupledInternalPanelCount(
            const math::ScalarTransition* rollRateTransition,
            const math::ScalarTransition& pitchRateTransition,
            const math::ScalarTransition& yawRateTransition,
            const double profileLength
        )
        {
            const double maximumPitchRate = std::max(
                std::abs(pitchRateTransition.valueBegin),
                std::abs(pitchRateTransition.valueEnd)
            );
            const double maximumYawRate = std::max(
                std::abs(yawRateTransition.valueBegin),
                std::abs(yawRateTransition.valueEnd)
            );
            const double maximumAngularRate = rollRateTransition == nullptr
                ? std::hypot(maximumPitchRate, maximumYawRate)
                : std::hypot(
                    std::max(
                        std::abs(rollRateTransition->valueBegin),
                        std::abs(rollRateTransition->valueEnd)
                    ),
                    maximumPitchRate,
                    maximumYawRate
                );
            const double maximumSectionAngle =
                maximumAngularRate * profileLength;

            if (!std::isfinite(maximumSectionAngle))
            {
                throw std::domain_error(
                    "Coupled rider-local angular change over the profile is not finite."
                );
            }

            constexpr double maximumPanelAngle =
                std::numbers::pi_v<double> / 128.0;
            constexpr double baselinePanelCount = 1'024.0;
            const double requiredPanelCount = std::max(
                baselinePanelCount,
                std::ceil(maximumSectionAngle / maximumPanelAngle)
            );

            if (requiredPanelCount
                > static_cast<double>(std::numeric_limits<std::size_t>::max()))
            {
                throw std::length_error(
                    "Coupled rider-local integration requires too many internal panels."
                );
            }

            return static_cast<std::size_t>(requiredPanelCount);
        }

        [[nodiscard]] std::size_t resultStepCount(
            const double profileLength,
            const double integrationSpacing,
            const std::vector<RiderLocalGeometryState>& states
        )
        {
            const double requiredStepCount = std::max(
                1.0,
                std::ceil(profileLength / integrationSpacing)
            );
            const double maximumStepCount = static_cast<double>(
                states.max_size() - 1
            );

            if (!std::isfinite(requiredStepCount)
                || requiredStepCount > maximumStepCount)
            {
                throw std::length_error(
                    "Rider-local geometry integration spacing requires too many result states."
                );
            }

            return static_cast<std::size_t>(requiredStepCount);
        }

        [[nodiscard]] std::vector<RiderLocalGeometryState>
        integrateConstantLocalPitchYawRates(
            const glm::dvec3& startingPosition,
            const geometry::CurveFrame& startingFrame,
            const double profileLength,
            const double pitchRate,
            const double yawRate,
            const double integrationSpacing
        )
        {
            const double angularRate = std::hypot(pitchRate, yawRate);
            const double terminalAngle = angularRate * profileLength;

            if (!std::isfinite(angularRate)
                || !std::isfinite(terminalAngle))
            {
                throw std::domain_error(
                    "Constant coupled rider-local angular change is not finite."
                );
            }

            std::vector<RiderLocalGeometryState> states;
            const std::size_t stepCount = resultStepCount(
                profileLength,
                integrationSpacing,
                states
            );
            states.reserve(stepCount + 1);
            states.push_back({0.0, startingPosition, startingFrame});

            const glm::dquat startingOrientation =
                orientationFromFrame(startingFrame);
            const glm::dvec3 localAxis{
                0.0,
                pitchRate / angularRate,
                yawRate / angularRate
            };
            const glm::dvec3 turningDirection =
                (yawRate * startingFrame.lateral
                    - pitchRate * startingFrame.up) / angularRate;

            for (std::size_t stepIndex = 1;
                 stepIndex <= stepCount;
                 ++stepIndex)
            {
                const double distance = stepIndex == stepCount
                    ? profileLength
                    : std::min(
                        profileLength,
                        static_cast<double>(stepIndex) * integrationSpacing
                    );
                const double angle = angularRate * distance;
                const glm::dvec3 position = startingPosition
                    + distance * sinc(angle) * startingFrame.tangent
                    + distance * cosc(angle) * turningDirection;
                const glm::dquat orientation = composeLocalRotation(
                    startingOrientation,
                    angle * localAxis
                );
                states.push_back({
                    distance,
                    position,
                    frameFromOrientation(orientation)
                });
            }

            return states;
        }

        [[nodiscard]] std::vector<RiderLocalGeometryState>
        integrateConstantLocalRollPitchYawRates(
            const glm::dvec3& startingPosition,
            const geometry::CurveFrame& startingFrame,
            const double profileLength,
            const double rollRate,
            const double pitchRate,
            const double yawRate,
            const double integrationSpacing
        )
        {
            if (rollRate == 0.0)
            {
                return integrateConstantLocalPitchYawRates(
                    startingPosition,
                    startingFrame,
                    profileLength,
                    pitchRate,
                    yawRate,
                    integrationSpacing
                );
            }

            const double angularRate = std::hypot(
                rollRate,
                pitchRate,
                yawRate
            );
            const double terminalAngle = angularRate * profileLength;

            if (!std::isfinite(angularRate)
                || !std::isfinite(terminalAngle))
            {
                throw std::domain_error(
                    "Constant simultaneous rider-local angular change is not finite."
                );
            }

            std::vector<RiderLocalGeometryState> states;
            const std::size_t stepCount = resultStepCount(
                profileLength,
                integrationSpacing,
                states
            );
            states.reserve(stepCount + 1);
            states.push_back({0.0, startingPosition, startingFrame});

            const glm::dquat startingOrientation =
                orientationFromFrame(startingFrame);
            const glm::dvec3 localAxis{
                rollRate / angularRate,
                pitchRate / angularRate,
                yawRate / angularRate
            };
            const glm::dvec3 spatialAxis =
                localAxis.x * startingFrame.tangent
                + localAxis.y * startingFrame.lateral
                + localAxis.z * startingFrame.up;
            const glm::dvec3 turningDirection = glm::cross(
                spatialAxis,
                startingFrame.tangent
            );

            for (std::size_t stepIndex = 1;
                 stepIndex <= stepCount;
                 ++stepIndex)
            {
                const double distance = stepIndex == stepCount
                    ? profileLength
                    : std::min(
                        profileLength,
                        static_cast<double>(stepIndex) * integrationSpacing
                    );
                const double angle = angularRate * distance;
                const glm::dvec3 position = startingPosition
                    + distance * sinc(angle) * startingFrame.tangent
                    + distance * cosc(angle) * turningDirection
                    + distance * oneMinusSinc(angle)
                        * localAxis.x * spatialAxis;
                const glm::dquat orientation = composeLocalRotation(
                    startingOrientation,
                    distance * glm::dvec3{
                        rollRate,
                        pitchRate,
                        yawRate
                    }
                );
                states.push_back({
                    distance,
                    position,
                    frameFromOrientation(orientation)
                });
            }

            return states;
        }

        [[nodiscard]] std::vector<RiderLocalGeometryState>
        integrateCoupledRateProfilesNumerically(
            const glm::dvec3& startingPosition,
            const geometry::CurveFrame& startingFrame,
            const math::ScalarTransition* rollRateTransition,
            const math::ScalarTransition& pitchRateTransition,
            const math::ScalarTransition& yawRateTransition,
            const double integrationSpacing
        )
        {
            const double profileLength = pitchRateTransition.domainEnd
                - pitchRateTransition.domainBegin;
            std::vector<RiderLocalGeometryState> states;
            const std::size_t stepCount = resultStepCount(
                profileLength,
                integrationSpacing,
                states
            );
            states.reserve(stepCount + 1);
            states.push_back({0.0, startingPosition, startingFrame});

            const std::size_t internalPanelCount = coupledInternalPanelCount(
                rollRateTransition,
                pitchRateTransition,
                yawRateTransition,
                profileLength
            );
            const double maximumInternalSpacing = profileLength
                / static_cast<double>(internalPanelCount);

            if (!std::isfinite(maximumInternalSpacing)
                || maximumInternalSpacing <= 0.0)
            {
                throw std::length_error(
                    "Coupled rider-local integration cannot represent its internal spacing."
                );
            }

            CoupledIntegrationState integrationState{
                0.0,
                startingPosition,
                orientationFromFrame(startingFrame)
            };

            const auto appendState = [&](const double nextDistance)
            {
                const double intervalLength =
                    nextDistance - integrationState.distance;
                const double requiredInternalStepCount = std::max(
                    1.0,
                    std::ceil(intervalLength / maximumInternalSpacing)
                );

                if (!std::isfinite(requiredInternalStepCount)
                    || requiredInternalStepCount
                        > static_cast<double>(
                            std::numeric_limits<std::size_t>::max()
                        ))
                {
                    throw std::length_error(
                        "Coupled rider-local integration requires too many internal steps."
                    );
                }

                const std::size_t internalStepCount =
                    static_cast<std::size_t>(requiredInternalStepCount);
                for (std::size_t internalStepIndex = 0;
                     internalStepIndex < internalStepCount;
                     ++internalStepIndex)
                {
                    const double remainingDistance =
                        nextDistance - integrationState.distance;
                    const double stepLength = remainingDistance
                        / static_cast<double>(
                            internalStepCount - internalStepIndex
                        );
                    integrateCoupledSubstep(
                        integrationState,
                        rollRateTransition,
                        pitchRateTransition,
                        yawRateTransition,
                        stepLength,
                        profileLength
                    );
                }

                // Preserve authored sample distances exactly even if the
                // repeated floating-point additions finish one ulp away.
                integrationState.distance = nextDistance;
                states.push_back({
                    nextDistance,
                    integrationState.position,
                    frameFromOrientation(integrationState.orientation)
                });
            };

            for (std::size_t stepIndex = 1;
                 stepIndex < stepCount;
                 ++stepIndex)
            {
                const double nextDistance =
                    static_cast<double>(stepIndex) * integrationSpacing;

                if (!(nextDistance < profileLength))
                {
                    break;
                }

                appendState(nextDistance);
            }

            appendState(profileLength);
            return states;
        }

        [[nodiscard]] glm::dvec3 integrateVariableTangent(
            const geometry::CurveFrame& startingFrame,
            const math::ScalarTransition& rateTransition,
            const double traveledBegin,
            const double traveledEnd,
            const LocalRotationOperations& operations
        )
        {
            const double intervalLength = traveledEnd - traveledBegin;
            const double maximumRate = std::max(
                std::abs(rateTransition.valueBegin),
                std::abs(rateTransition.valueEnd)
            );
            const double maximumAngleChange = maximumRate * intervalLength;

            if (!std::isfinite(maximumAngleChange))
            {
                throw std::domain_error(
                    operations.nonFiniteProfileIntervalAngleMessage
                );
            }

            // Keep each quadrature panel below a dimensionless angular span.
            // This internal refinement depends on tangent rotation, not on
            // output spacing or a coordinate-unit tolerance.
            constexpr double maximumPanelAngleChange =
                std::numbers::pi / 4.0;
            const double requiredPanelCount = std::max(
                1.0,
                std::ceil(maximumAngleChange / maximumPanelAngleChange)
            );

            if (requiredPanelCount
                > static_cast<double>(std::numeric_limits<std::size_t>::max()))
            {
                throw std::length_error(
                    operations.excessivePanelCountMessage
                );
            }

            const std::size_t panelCount =
                static_cast<std::size_t>(requiredPanelCount);
            const double panelLength =
                intervalLength / static_cast<double>(panelCount);
            glm::dvec3 displacement{0.0};

            for (std::size_t panelIndex = 0;
                 panelIndex < panelCount;
                 ++panelIndex)
            {
                const double panelBegin = traveledBegin
                    + static_cast<double>(panelIndex) * panelLength;
                const double panelEnd = panelIndex + 1 == panelCount
                    ? traveledEnd
                    : traveledBegin
                        + static_cast<double>(panelIndex + 1) * panelLength;
                const double midpoint = 0.5 * (panelBegin + panelEnd);
                const double halfLength = 0.5 * (panelEnd - panelBegin);
                glm::dvec3 weightedTangents{0.0};

                for (std::size_t nodeIndex = 0;
                     nodeIndex < gaussLegendreNodes.size();
                     ++nodeIndex)
                {
                    const double offset =
                        halfLength * gaussLegendreNodes[nodeIndex];
                    const double lowerDistance = midpoint - offset;
                    const double upperDistance = midpoint + offset;
                    const double lowerDomainValue =
                        rateTransition.domainBegin + lowerDistance;
                    const double upperDomainValue =
                        rateTransition.domainBegin + upperDistance;
                    const double lowerAngle = math::integrateScalarTransition(
                        rateTransition,
                        rateTransition.domainBegin,
                        lowerDomainValue
                    );
                    const double upperAngle = math::integrateScalarTransition(
                        rateTransition,
                        rateTransition.domainBegin,
                        upperDomainValue
                    );

                    weightedTangents += gaussLegendreWeights[nodeIndex]
                        * (operations.evaluateTangent(
                               startingFrame,
                               lowerAngle
                           )
                            + operations.evaluateTangent(
                                startingFrame,
                                upperAngle
                            ));
                }

                displacement += halfLength * weightedTangents;
            }

            if (!isFinitePosition(displacement))
            {
                throw std::domain_error(
                    "Rider-local geometry integration produced a non-finite displacement."
                );
            }

            return displacement;
        }

        [[nodiscard]] std::vector<RiderLocalGeometryState>
        integrateConstantLocalRate(
            const glm::dvec3& startingPosition,
            const geometry::CurveFrame& startingFrame,
            const double sectionLength,
            const double localRate,
            const double integrationSpacing,
            const LocalRotationOperations& operations
        )
        {
            if (!isFinitePosition(startingPosition))
            {
                throw std::invalid_argument(
                    "The rider-local geometry starting position must be finite."
                );
            }

            if (!std::isfinite(sectionLength) || sectionLength < 0.0)
            {
                throw std::invalid_argument(
                    "Rider-local geometry section length must be finite and non-negative."
                );
            }

            if (!std::isfinite(localRate))
            {
                throw std::invalid_argument(operations.invalidRateMessage);
            }

            if (!std::isfinite(integrationSpacing)
                || integrationSpacing <= 0.0)
            {
                throw std::invalid_argument(
                    "Rider-local geometry integration spacing must be positive and finite."
                );
            }

            // A zero rotation invokes the authoritative frame validation
            // without changing any starting-frame bits.
            static_cast<void>(operations.applyRotation(startingFrame, 0.0));

            std::vector<RiderLocalGeometryState> states;
            states.push_back({0.0, startingPosition, startingFrame});

            if (sectionLength == 0.0)
            {
                return states;
            }

            const double requiredStepCount = std::max(
                1.0,
                std::ceil(sectionLength / integrationSpacing)
            );
            const double maximumStepCount =
                static_cast<double>(states.max_size() - 1);

            if (!std::isfinite(requiredStepCount)
                || requiredStepCount > maximumStepCount)
            {
                throw std::length_error(
                    "Rider-local geometry integration spacing requires too many result states."
                );
            }

            const std::size_t stepCount =
                static_cast<std::size_t>(requiredStepCount);
            states.reserve(stepCount + 1);

            double currentDistance = 0.0;
            glm::dvec3 currentPosition = startingPosition;
            geometry::CurveFrame currentFrame = startingFrame;

            for (std::size_t stepIndex = 1;
                 stepIndex <= stepCount;
                 ++stepIndex)
            {
                const double nextDistance = stepIndex == stepCount
                    ? sectionLength
                    : std::min(
                        sectionLength,
                        static_cast<double>(stepIndex) * integrationSpacing
                    );
                const double stepDistance = nextDistance - currentDistance;

                if (localRate == 0.0)
                {
                    currentPosition = startingPosition
                        + nextDistance * startingFrame.tangent;
                    currentFrame = startingFrame;
                }
                else
                {
                    const double rotationAngle = localRate * stepDistance;

                    if (!std::isfinite(rotationAngle))
                    {
                        throw std::domain_error(
                            operations.nonFiniteConstantStepAngleMessage
                        );
                    }

                    const double halfRotationAngle = 0.5 * rotationAngle;
                    const geometry::CurveFrame midpointFrame =
                        operations.applyRotation(
                            currentFrame,
                            halfRotationAngle
                        );

                    // The exact circular-arc chord for a tangent rotating at
                    // constant rate is ds * sinc(theta / 2) * T(ds / 2).
                    const double chordLength =
                        stepDistance * sinc(halfRotationAngle);
                    currentPosition += chordLength * midpointFrame.tangent;
                    currentFrame = operations.applyRotation(
                        currentFrame,
                        rotationAngle
                    );

                    if (!isFinitePosition(currentPosition))
                    {
                        throw std::domain_error(
                            "Rider-local geometry integration produced a non-finite position."
                        );
                    }
                }

                currentDistance = nextDistance;
                states.push_back({
                    currentDistance,
                    currentPosition,
                    currentFrame
                });
            }

            return states;
        }

        [[nodiscard]] std::vector<RiderLocalGeometryState>
        integrateLocalRateProfile(
            const glm::dvec3& startingPosition,
            const geometry::CurveFrame& startingFrame,
            const math::ScalarTransition& rateTransition,
            const double integrationSpacing,
            const LocalRotationOperations& operations
        )
        {
            if (!isFinitePosition(startingPosition))
            {
                throw std::invalid_argument(
                    "The rider-local geometry starting position must be finite."
                );
            }

            if (!std::isfinite(integrationSpacing)
                || integrationSpacing <= 0.0)
            {
                throw std::invalid_argument(
                    "Rider-local geometry integration spacing must be positive and finite."
                );
            }

            static_cast<void>(operations.applyRotation(startingFrame, 0.0));

            // Validate the complete transition and terminal accumulated angle
            // before producing any result.
            static_cast<void>(math::integrateScalarTransition(
                rateTransition,
                rateTransition.domainBegin,
                rateTransition.domainEnd
            ));

            const double profileLength = rateTransition.domainEnd
                - rateTransition.domainBegin;

            if (rateTransition.valueBegin == rateTransition.valueEnd)
            {
                return integrateConstantLocalRate(
                    startingPosition,
                    startingFrame,
                    profileLength,
                    rateTransition.valueBegin,
                    integrationSpacing,
                    operations
                );
            }

            const double requiredStepCount = std::max(
                1.0,
                std::ceil(profileLength / integrationSpacing)
            );
            std::vector<RiderLocalGeometryState> states;
            const double maximumStepCount = static_cast<double>(
                states.max_size() - 1
            );

            if (!std::isfinite(requiredStepCount)
                || requiredStepCount > maximumStepCount)
            {
                throw std::length_error(
                    "Rider-local geometry integration spacing requires too many result states."
                );
            }

            const std::size_t stepCount =
                static_cast<std::size_t>(requiredStepCount);
            states.reserve(stepCount + 1);
            states.push_back({0.0, startingPosition, startingFrame});

            double currentDistance = 0.0;
            glm::dvec3 currentPosition = startingPosition;

            const auto appendState = [&](const double nextDistance)
            {
                currentPosition += integrateVariableTangent(
                    startingFrame,
                    rateTransition,
                    currentDistance,
                    nextDistance,
                    operations
                );
                const double domainValue = nextDistance == profileLength
                    ? rateTransition.domainEnd
                    : rateTransition.domainBegin + nextDistance;
                const double accumulatedAngle =
                    math::integrateScalarTransition(
                        rateTransition,
                        rateTransition.domainBegin,
                        domainValue
                    );
                currentDistance = nextDistance;
                states.push_back({
                    currentDistance,
                    currentPosition,
                    operations.applyRotation(
                        startingFrame,
                        accumulatedAngle
                    )
                });
            };

            for (std::size_t stepIndex = 1;
                 stepIndex < stepCount;
                 ++stepIndex)
            {
                const double nextDistance =
                    static_cast<double>(stepIndex) * integrationSpacing;

                // Leave any endpoint candidate to the single append below.
                if (!(nextDistance < profileLength))
                {
                    break;
                }

                appendState(nextDistance);
            }

            appendState(profileLength);
            return states;
        }
    }

    std::vector<RiderLocalGeometryState>
    integrateConstantLocalPitchRate(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const double sectionLength,
        const double localPitchRate,
        const double integrationSpacing
    )
    {
        return integrateConstantLocalRate(
            startingPosition,
            startingFrame,
            sectionLength,
            localPitchRate,
            integrationSpacing,
            pitchOperations
        );
    }

    std::vector<RiderLocalGeometryState>
    integrateConstantLocalYawRate(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const double sectionLength,
        const double localYawRate,
        const double integrationSpacing
    )
    {
        return integrateConstantLocalRate(
            startingPosition,
            startingFrame,
            sectionLength,
            localYawRate,
            integrationSpacing,
            yawOperations
        );
    }

    std::vector<RiderLocalGeometryState>
    integrateConstantLocalRollRate(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const double sectionLength,
        const double localRollRate,
        const double integrationSpacing
    )
    {
        if (!isFinitePosition(startingPosition))
        {
            throw std::invalid_argument(
                "The rider-local geometry starting position must be finite."
            );
        }

        if (!std::isfinite(sectionLength) || sectionLength < 0.0)
        {
            throw std::invalid_argument(
                "Rider-local geometry section length must be finite and non-negative."
            );
        }

        if (!std::isfinite(localRollRate))
        {
            throw std::invalid_argument(
                "Rider-local roll rate must be finite and expressed in radians per coordinate unit."
            );
        }

        if (!std::isfinite(integrationSpacing)
            || integrationSpacing <= 0.0)
        {
            throw std::invalid_argument(
                "Rider-local geometry integration spacing must be positive and finite."
            );
        }

        // Invoke the authoritative frame validation without changing bits.
        static_cast<void>(geometry::applyRoll(startingFrame, 0.0));

        std::vector<RiderLocalGeometryState> states;
        states.push_back({0.0, startingPosition, startingFrame});

        if (sectionLength == 0.0)
        {
            return states;
        }

        const double terminalRoll = localRollRate * sectionLength;
        if (!std::isfinite(terminalRoll))
        {
            throw std::domain_error(
                "Constant rider-local roll change is not representable as a finite angle."
            );
        }

        const std::size_t stepCount = resultStepCount(
            sectionLength,
            integrationSpacing,
            states
        );
        states.reserve(stepCount + 1);

        const auto appendState = [&](const double distance)
        {
            const glm::dvec3 position = startingPosition
                + distance * startingFrame.tangent;

            if (!isFinitePosition(position))
            {
                throw std::domain_error(
                    "Rider-local geometry integration produced a non-finite position."
                );
            }

            states.push_back({
                distance,
                position,
                localRollRate == 0.0
                    ? startingFrame
                    : geometry::applyRoll(
                        startingFrame,
                        localRollRate * distance
                    )
            });
        };

        for (std::size_t stepIndex = 1;
             stepIndex < stepCount;
             ++stepIndex)
        {
            const double distance =
                static_cast<double>(stepIndex) * integrationSpacing;

            if (!(distance < sectionLength))
            {
                break;
            }

            appendState(distance);
        }

        appendState(sectionLength);
        return states;
    }

    std::vector<RiderLocalGeometryState>
    integrateLocalPitchRateProfile(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const math::ScalarTransition& pitchRateTransition,
        const double integrationSpacing
    )
    {
        return integrateLocalRateProfile(
            startingPosition,
            startingFrame,
            pitchRateTransition,
            integrationSpacing,
            pitchOperations
        );
    }

    std::vector<RiderLocalGeometryState>
    integrateLocalYawRateProfile(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const math::ScalarTransition& yawRateTransition,
        const double integrationSpacing
    )
    {
        return integrateLocalRateProfile(
            startingPosition,
            startingFrame,
            yawRateTransition,
            integrationSpacing,
            yawOperations
        );
    }

    std::vector<RiderLocalGeometryState>
    integrateLocalRollRateProfile(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const math::ScalarTransition& rollRateTransition,
        const double integrationSpacing
    )
    {
        if (!isFinitePosition(startingPosition))
        {
            throw std::invalid_argument(
                "The rider-local geometry starting position must be finite."
            );
        }

        if (!std::isfinite(integrationSpacing)
            || integrationSpacing <= 0.0)
        {
            throw std::invalid_argument(
                "Rider-local geometry integration spacing must be positive and finite."
            );
        }

        static_cast<void>(geometry::applyRoll(startingFrame, 0.0));

        // This validates the full transition and its terminal accumulated
        // roll before any result is produced.
        static_cast<void>(math::integrateScalarTransition(
            rollRateTransition,
            rollRateTransition.domainBegin,
            rollRateTransition.domainEnd
        ));

        const double profileLength = rollRateTransition.domainEnd
            - rollRateTransition.domainBegin;

        if (rollRateTransition.valueBegin
            == rollRateTransition.valueEnd)
        {
            return integrateConstantLocalRollRate(
                startingPosition,
                startingFrame,
                profileLength,
                rollRateTransition.valueBegin,
                integrationSpacing
            );
        }

        std::vector<RiderLocalGeometryState> states;
        const std::size_t stepCount = resultStepCount(
            profileLength,
            integrationSpacing,
            states
        );
        states.reserve(stepCount + 1);
        states.push_back({0.0, startingPosition, startingFrame});

        const auto appendState = [&](const double distance)
        {
            const double domainValue = transitionDomainValue(
                rollRateTransition,
                distance,
                profileLength
            );
            const double accumulatedRoll =
                math::integrateScalarTransition(
                    rollRateTransition,
                    rollRateTransition.domainBegin,
                    domainValue
                );
            const glm::dvec3 position = startingPosition
                + distance * startingFrame.tangent;

            if (!isFinitePosition(position))
            {
                throw std::domain_error(
                    "Rider-local geometry integration produced a non-finite position."
                );
            }

            states.push_back({
                distance,
                position,
                geometry::applyRoll(startingFrame, accumulatedRoll)
            });
        };

        for (std::size_t stepIndex = 1;
             stepIndex < stepCount;
             ++stepIndex)
        {
            const double distance =
                static_cast<double>(stepIndex) * integrationSpacing;

            if (!(distance < profileLength))
            {
                break;
            }

            appendState(distance);
        }

        appendState(profileLength);
        return states;
    }

    std::vector<RiderLocalGeometryState>
    integrateLocalPitchYawRateProfiles(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const math::ScalarTransition& pitchRateTransition,
        const math::ScalarTransition& yawRateTransition,
        const double integrationSpacing
    )
    {
        if (!isFinitePosition(startingPosition))
        {
            throw std::invalid_argument(
                "The rider-local geometry starting position must be finite."
            );
        }

        if (!std::isfinite(integrationSpacing)
            || integrationSpacing <= 0.0)
        {
            throw std::invalid_argument(
                "Rider-local geometry integration spacing must be positive and finite."
            );
        }

        // Invoke the established frame validation without changing any bits.
        static_cast<void>(geometry::applyLocalPitch(startingFrame, 0.0));

        // Validate both complete transitions before applying shared-domain or
        // zero-channel reductions.
        static_cast<void>(math::integrateScalarTransition(
            pitchRateTransition,
            pitchRateTransition.domainBegin,
            pitchRateTransition.domainEnd
        ));
        static_cast<void>(math::integrateScalarTransition(
            yawRateTransition,
            yawRateTransition.domainBegin,
            yawRateTransition.domainEnd
        ));

        if (pitchRateTransition.domainBegin
                != yawRateTransition.domainBegin
            || pitchRateTransition.domainEnd
                != yawRateTransition.domainEnd)
        {
            throw std::invalid_argument(
                "Coupled rider-local pitch- and yaw-rate profiles must have exactly matching domains."
            );
        }

        const bool pitchIsZero = pitchRateTransition.valueBegin == 0.0
            && pitchRateTransition.valueEnd == 0.0;
        const bool yawIsZero = yawRateTransition.valueBegin == 0.0
            && yawRateTransition.valueEnd == 0.0;

        if (yawIsZero)
        {
            return integrateLocalPitchRateProfile(
                startingPosition,
                startingFrame,
                pitchRateTransition,
                integrationSpacing
            );
        }

        if (pitchIsZero)
        {
            return integrateLocalYawRateProfile(
                startingPosition,
                startingFrame,
                yawRateTransition,
                integrationSpacing
            );
        }

        const double profileLength = pitchRateTransition.domainEnd
            - pitchRateTransition.domainBegin;
        const bool pitchIsConstant =
            pitchRateTransition.valueBegin
            == pitchRateTransition.valueEnd;
        const bool yawIsConstant = yawRateTransition.valueBegin
            == yawRateTransition.valueEnd;

        if (pitchIsConstant && yawIsConstant)
        {
            return integrateConstantLocalPitchYawRates(
                startingPosition,
                startingFrame,
                profileLength,
                pitchRateTransition.valueBegin,
                yawRateTransition.valueBegin,
                integrationSpacing
            );
        }

        return integrateCoupledRateProfilesNumerically(
            startingPosition,
            startingFrame,
            nullptr,
            pitchRateTransition,
            yawRateTransition,
            integrationSpacing
        );
    }

    std::vector<RiderLocalGeometryState>
    integrateLocalRollPitchYawRateProfiles(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const math::ScalarTransition& rollRateTransition,
        const math::ScalarTransition& pitchRateTransition,
        const math::ScalarTransition& yawRateTransition,
        const double integrationSpacing
    )
    {
        if (!isFinitePosition(startingPosition))
        {
            throw std::invalid_argument(
                "The rider-local geometry starting position must be finite."
            );
        }

        if (!std::isfinite(integrationSpacing)
            || integrationSpacing <= 0.0)
        {
            throw std::invalid_argument(
                "Rider-local geometry integration spacing must be positive and finite."
            );
        }

        // Invoke the established frame validation without changing any bits.
        static_cast<void>(geometry::applyLocalPitch(startingFrame, 0.0));

        // Validate every complete transition before shared-domain or
        // zero-channel reductions.
        static_cast<void>(math::integrateScalarTransition(
            rollRateTransition,
            rollRateTransition.domainBegin,
            rollRateTransition.domainEnd
        ));
        static_cast<void>(math::integrateScalarTransition(
            pitchRateTransition,
            pitchRateTransition.domainBegin,
            pitchRateTransition.domainEnd
        ));
        static_cast<void>(math::integrateScalarTransition(
            yawRateTransition,
            yawRateTransition.domainBegin,
            yawRateTransition.domainEnd
        ));

        if (rollRateTransition.domainBegin
                != pitchRateTransition.domainBegin
            || rollRateTransition.domainEnd
                != pitchRateTransition.domainEnd
            || rollRateTransition.domainBegin
                != yawRateTransition.domainBegin
            || rollRateTransition.domainEnd
                != yawRateTransition.domainEnd)
        {
            throw std::invalid_argument(
                "Rider-local roll-, pitch-, and yaw-rate profiles must have exactly matching domains."
            );
        }

        const bool rollIsZero = rollRateTransition.valueBegin == 0.0
            && rollRateTransition.valueEnd == 0.0;
        const bool pitchIsZero = pitchRateTransition.valueBegin == 0.0
            && pitchRateTransition.valueEnd == 0.0;
        const bool yawIsZero = yawRateTransition.valueBegin == 0.0
            && yawRateTransition.valueEnd == 0.0;

        if (rollIsZero)
        {
            return integrateLocalPitchYawRateProfiles(
                startingPosition,
                startingFrame,
                pitchRateTransition,
                yawRateTransition,
                integrationSpacing
            );
        }

        if (pitchIsZero && yawIsZero)
        {
            return integrateLocalRollRateProfile(
                startingPosition,
                startingFrame,
                rollRateTransition,
                integrationSpacing
            );
        }

        const double profileLength = rollRateTransition.domainEnd
            - rollRateTransition.domainBegin;
        const bool rollIsConstant = rollRateTransition.valueBegin
            == rollRateTransition.valueEnd;
        const bool pitchIsConstant = pitchRateTransition.valueBegin
            == pitchRateTransition.valueEnd;
        const bool yawIsConstant = yawRateTransition.valueBegin
            == yawRateTransition.valueEnd;

        if (rollIsConstant && pitchIsConstant && yawIsConstant)
        {
            return integrateConstantLocalRollPitchYawRates(
                startingPosition,
                startingFrame,
                profileLength,
                rollRateTransition.valueBegin,
                pitchRateTransition.valueBegin,
                yawRateTransition.valueBegin,
                integrationSpacing
            );
        }

        return integrateCoupledRateProfilesNumerically(
            startingPosition,
            startingFrame,
            &rollRateTransition,
            pitchRateTransition,
            yawRateTransition,
            integrationSpacing
        );
    }
}
