#pragma once

#include <quantum/geometry/CurveGeometry.hpp>
#include <quantum/geometry/CurveSampling.hpp>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace quantum::geometry
{
    // Frames use the right-handed convention tangent x lateral = up.
    // Equivalently, lateral = up x tangent. The transverse axes describe the
    // transported orientation and are not Frenet principal-normal axes.
    struct CurveFrame
    {
        glm::dvec3 tangent{0.0};
        glm::dvec3 lateral{0.0};
        glm::dvec3 up{0.0};

        [[nodiscard]] friend bool operator==(
            const CurveFrame&,
            const CurveFrame&
        ) = default;
    };

    namespace detail
    {
        // These tests are dimensionless because they operate on normalized
        // directions. They cover only floating-point directional resolution;
        // no coordinate-unit epsilon is used.
        inline constexpr double initialFrameDirectionalResolution =
            32.0 * std::numeric_limits<double>::epsilon();
        inline constexpr double antiparallelDirectionalResolution =
            64.0 * std::numeric_limits<double>::epsilon();
        inline constexpr double rollFrameValidationTolerance =
            4096.0 * std::numeric_limits<double>::epsilon();

        [[nodiscard]] inline bool isFiniteFrameVector(
            const glm::dvec3& vector
        ) noexcept
        {
            return std::isfinite(vector.x)
                && std::isfinite(vector.y)
                && std::isfinite(vector.z);
        }

        [[nodiscard]] inline double frameVectorMagnitude(
            const glm::dvec3& vector
        ) noexcept
        {
            return std::hypot(vector.x, vector.y, vector.z);
        }

        [[nodiscard]] inline glm::dvec3 normalizeFrameVector(
            const glm::dvec3& vector,
            const char* const errorMessage
        )
        {
            const double magnitude = frameVectorMagnitude(vector);

            if (!std::isfinite(magnitude) || magnitude == 0.0)
            {
                throw std::domain_error(errorMessage);
            }

            const glm::dvec3 normalized = vector / magnitude;

            if (!isFiniteFrameVector(normalized))
            {
                throw std::domain_error(errorMessage);
            }

            return normalized;
        }

        [[nodiscard]] inline glm::dvec3 normalizeInitialReference(
            const glm::dvec3& reference
        )
        {
            if (!isFiniteFrameVector(reference))
            {
                throw std::invalid_argument(
                    "The initial frame reference-up vector must be finite."
                );
            }

            const double scale = std::max({
                std::abs(reference.x),
                std::abs(reference.y),
                std::abs(reference.z)
            });

            if (scale == 0.0)
            {
                throw std::invalid_argument(
                    "The initial frame reference-up vector must be nonzero."
                );
            }

            // Scaling first accepts any finite nonzero vector whose direct
            // Euclidean magnitude might otherwise overflow or underflow.
            return normalizeFrameVector(
                reference / scale,
                "The initial frame reference-up vector could not be normalized."
            );
        }

        [[noreturn]] inline void throwInvalidCurveFrameForRotation(
            const char* const requirement,
            const char* const operation
        )
        {
            throw std::invalid_argument(
                std::string{"A curve frame must "} + requirement
                    + " before " + operation + " is applied."
            );
        }

        inline void validateCurveFrameForRotation(
            const CurveFrame& frame,
            const char* const operation
        )
        {
            if (!isFiniteFrameVector(frame.tangent)
                || !isFiniteFrameVector(frame.lateral)
                || !isFiniteFrameVector(frame.up))
            {
                throwInvalidCurveFrameForRotation(
                    "contain only finite axes",
                    operation
                );
            }

            const double tangentMagnitude =
                frameVectorMagnitude(frame.tangent);
            const double lateralMagnitude =
                frameVectorMagnitude(frame.lateral);
            const double upMagnitude = frameVectorMagnitude(frame.up);

            if (!std::isfinite(tangentMagnitude)
                || !std::isfinite(lateralMagnitude)
                || !std::isfinite(upMagnitude)
                || tangentMagnitude == 0.0
                || lateralMagnitude == 0.0
                || upMagnitude == 0.0)
            {
                throwInvalidCurveFrameForRotation(
                    "contain finite nonzero axes",
                    operation
                );
            }

            const double tolerance = rollFrameValidationTolerance;
            if (std::abs(tangentMagnitude - 1.0) > tolerance
                || std::abs(lateralMagnitude - 1.0) > tolerance
                || std::abs(upMagnitude - 1.0) > tolerance)
            {
                throwInvalidCurveFrameForRotation(
                    "have unit-length axes",
                    operation
                );
            }

            if (std::abs(glm::dot(frame.tangent, frame.lateral)) > tolerance
                || std::abs(glm::dot(frame.tangent, frame.up)) > tolerance
                || std::abs(glm::dot(frame.lateral, frame.up)) > tolerance)
            {
                throwInvalidCurveFrameForRotation(
                    "have mutually orthogonal axes",
                    operation
                );
            }

            const double handednessError = frameVectorMagnitude(
                glm::cross(frame.tangent, frame.lateral) - frame.up
            );
            if (!std::isfinite(handednessError)
                || handednessError > tolerance)
            {
                throwInvalidCurveFrameForRotation(
                    "satisfy tangent x lateral = up",
                    operation
                );
            }
        }

        inline void validateCurveFrameForRoll(const CurveFrame& frame)
        {
            validateCurveFrameForRotation(frame, "roll");
        }

        [[nodiscard]] inline glm::dvec3 rotateAroundUnitAxis(
            const glm::dvec3& vector,
            const glm::dvec3& unitAxis,
            const double cosine,
            const double sine
        ) noexcept
        {
            // Rodrigues' formula. Input validation keeps unitAxis at
            // roundoff-scale agreement with unit length.
            return cosine * vector
                + sine * glm::cross(unitAxis, vector)
                + (1.0 - cosine) * glm::dot(unitAxis, vector) * unitAxis;
        }

        [[nodiscard]] inline CurveFrame makeInitialCurveFrame(
            const glm::dvec3& tangent,
            const glm::dvec3& initialReferenceUp
        )
        {
            const glm::dvec3 unitTangent = normalizeFrameVector(
                tangent,
                "The initial curve tangent could not be normalized."
            );
            const glm::dvec3 reference =
                normalizeInitialReference(initialReferenceUp);
            const glm::dvec3 projectedUp =
                reference - glm::dot(reference, unitTangent) * unitTangent;
            const double projectedMagnitude =
                frameVectorMagnitude(projectedUp);

            if (!std::isfinite(projectedMagnitude)
                || projectedMagnitude <= initialFrameDirectionalResolution)
            {
                throw std::domain_error(
                    "The initial frame reference-up vector is parallel, antiparallel, or numerically unresolved relative to the initial tangent."
                );
            }

            glm::dvec3 up = projectedUp / projectedMagnitude;
            const glm::dvec3 lateral = normalizeFrameVector(
                glm::cross(up, unitTangent),
                "The initial curve frame lateral axis could not be normalized."
            );
            up = normalizeFrameVector(
                glm::cross(unitTangent, lateral),
                "The initial curve frame up axis could not be normalized."
            );

            return {tangent, lateral, up};
        }

        [[nodiscard]] inline bool frameTangentsAreIdentical(
            const glm::dvec3& first,
            const glm::dvec3& second
        ) noexcept
        {
            return first.x == second.x
                && first.y == second.y
                && first.z == second.z;
        }

        [[nodiscard]] inline glm::dvec3 applyMinimalTangentRotation(
            const glm::dvec3& previousUnitTangent,
            const glm::dvec3& nextUnitTangent,
            const glm::dvec3& vector
        )
        {
            const double cosine = std::clamp(
                glm::dot(previousUnitTangent, nextUnitTangent),
                -1.0,
                1.0
            );
            const double onePlusCosine = 1.0 + cosine;

            // At (or below) this dimensionless roundoff scale a 180-degree
            // rotation axis is not uniquely recoverable from the tangents.
            if (onePlusCosine <= antiparallelDirectionalResolution)
            {
                throw std::domain_error(
                    "Rotation-minimizing frame transport encountered an antiparallel or numerically unresolved tangent transition."
                );
            }

            const glm::dvec3 tangentCross =
                glm::cross(previousUnitTangent, nextUnitTangent);
            const glm::dvec3 firstCross = glm::cross(tangentCross, vector);

            // R = I + [v]x + [v]x^2 / (1 + c), with v = T0 x T1 and
            // c = T0 . T1. Unlike axis-angle evaluation, this stable form
            // never divides by |T0 x T1| for a small angular change.
            const glm::dvec3 rotated =
                vector
                + firstCross
                + glm::cross(tangentCross, firstCross) / onePlusCosine;

            if (!isFiniteFrameVector(rotated))
            {
                throw std::domain_error(
                    "Rotation-minimizing frame transport produced a non-finite axis."
                );
            }

            return rotated;
        }

        [[nodiscard]] inline CurveFrame transportCurveFrame(
            const CurveFrame& previous,
            const glm::dvec3& nextTangent
        )
        {
            // Preserve the axes bit-for-bit on a constant-tangent run. This
            // prevents cleanup roundoff from accumulating artificial roll.
            if (frameTangentsAreIdentical(previous.tangent, nextTangent))
            {
                return {nextTangent, previous.lateral, previous.up};
            }

            const glm::dvec3 previousUnitTangent = normalizeFrameVector(
                previous.tangent,
                "The previous curve-frame tangent could not be normalized."
            );
            const glm::dvec3 nextUnitTangent = normalizeFrameVector(
                nextTangent,
                "The next curve-frame tangent could not be normalized."
            );
            const glm::dvec3 transportedLateral =
                applyMinimalTangentRotation(
                    previousUnitTangent,
                    nextUnitTangent,
                    previous.lateral
                );
            const glm::dvec3 transportedUp = applyMinimalTangentRotation(
                previousUnitTangent,
                nextUnitTangent,
                previous.up
            );

            // Reproject one transported transverse axis, then recover the
            // other from the documented handedness relation. This is the only
            // orthonormal cleanup performed after the shared minimal rotation.
            const glm::dvec3 lateralProjection =
                transportedLateral
                - glm::dot(transportedLateral, nextUnitTangent)
                    * nextUnitTangent;
            glm::dvec3 lateral = normalizeFrameVector(
                lateralProjection,
                "The transported curve-frame lateral axis could not be normalized."
            );
            glm::dvec3 up = normalizeFrameVector(
                glm::cross(nextUnitTangent, lateral),
                "The transported curve-frame up axis could not be normalized."
            );

            // The shared rotation supplies the expected transverse sign. A
            // sign correction is only a guard against floating-point cleanup
            // choosing the opposite equivalent normalized cross-product.
            if (glm::dot(up, transportedUp) < 0.0)
            {
                lateral = -lateral;
                up = -up;
            }

            return {nextTangent, lateral, up};
        }
    }

    // Applies an authored roll angle in radians to an existing orthonormal
    // curve frame. Positive roll follows the right-hand rule about the
    // positive tangent axis. Consequently, for tangent x lateral = up,
    // positive roll moves lateral toward up and up toward -lateral.
    //
    // Frame validation uses a dimensionless roundoff-scale tolerance for unit
    // directions; it is independent of curve coordinates and world units.
    [[nodiscard]] inline CurveFrame applyRoll(
        const CurveFrame& frame,
        const double rollRadians
    )
    {
        if (!std::isfinite(rollRadians))
        {
            throw std::invalid_argument(
                "A curve-frame roll angle must be finite and expressed in radians."
            );
        }

        detail::validateCurveFrameForRoll(frame);

        if (rollRadians == 0.0)
        {
            return frame;
        }

        const double tangentMagnitude =
            detail::frameVectorMagnitude(frame.tangent);
        const glm::dvec3 unitTangent = frame.tangent / tangentMagnitude;
        const double cosine = std::cos(rollRadians);
        const double sine = std::sin(rollRadians);

        return {
            frame.tangent,
            detail::rotateAroundUnitAxis(
                frame.lateral,
                unitTangent,
                cosine,
                sine
            ),
            detail::rotateAroundUnitAxis(
                frame.up,
                unitTangent,
                cosine,
                sine
            )
        };
    }

    // Applies rider-local pitch in radians about the frame's current positive
    // lateral axis. Positive pitch follows the right-hand rule: tangent moves
    // toward -up, while up moves toward tangent. The stored lateral axis is
    // returned unchanged.
    [[nodiscard]] inline CurveFrame applyLocalPitch(
        const CurveFrame& frame,
        const double pitchRadians
    )
    {
        if (!std::isfinite(pitchRadians))
        {
            throw std::invalid_argument(
                "A curve-frame local pitch angle must be finite and expressed in radians."
            );
        }

        detail::validateCurveFrameForRotation(frame, "local pitch");

        if (pitchRadians == 0.0)
        {
            return frame;
        }

        const double lateralMagnitude =
            detail::frameVectorMagnitude(frame.lateral);
        const glm::dvec3 unitLateral = frame.lateral / lateralMagnitude;
        const double cosine = std::cos(pitchRadians);
        const double sine = std::sin(pitchRadians);

        return {
            detail::rotateAroundUnitAxis(
                frame.tangent,
                unitLateral,
                cosine,
                sine
            ),
            frame.lateral,
            detail::rotateAroundUnitAxis(
                frame.up,
                unitLateral,
                cosine,
                sine
            )
        };
    }

    // Applies rider-local yaw in radians about the frame's current positive up
    // axis. Positive yaw follows the right-hand rule: tangent moves toward
    // lateral, while lateral moves toward -tangent. The stored up axis is
    // returned unchanged.
    [[nodiscard]] inline CurveFrame applyLocalYaw(
        const CurveFrame& frame,
        const double yawRadians
    )
    {
        if (!std::isfinite(yawRadians))
        {
            throw std::invalid_argument(
                "A curve-frame local yaw angle must be finite and expressed in radians."
            );
        }

        detail::validateCurveFrameForRotation(frame, "local yaw");

        if (yawRadians == 0.0)
        {
            return frame;
        }

        const double upMagnitude = detail::frameVectorMagnitude(frame.up);
        const glm::dvec3 unitUp = frame.up / upMagnitude;
        const double cosine = std::cos(yawRadians);
        const double sine = std::sin(yawRadians);

        return {
            detail::rotateAroundUnitAxis(
                frame.tangent,
                unitUp,
                cosine,
                sine
            ),
            detail::rotateAroundUnitAxis(
                frame.lateral,
                unitUp,
                cosine,
                sine
            ),
            frame.up
        };
    }

    // Builds one frame per supplied canonical sample using each sample's
    // stored parameter and the curve's analytic unit tangent. Samples are not
    // modified. Empty input returns an empty vector without evaluating either
    // the curve or the otherwise-unused initial reference-up vector.
    //
    // Exact or numerically unresolved antiparallel tangent transitions are
    // rejected because their minimal 180-degree transport axis is not unique.
    template<typename Curve>
    [[nodiscard]] std::vector<CurveFrame> buildRotationMinimizingFrames(
        const Curve& curve,
        const std::vector<CurveSample>& samples,
        const glm::dvec3& initialReferenceUp
    )
    {
        if (samples.empty())
        {
            return {};
        }

        std::vector<CurveFrame> frames;
        frames.reserve(samples.size());
        frames.push_back(detail::makeInitialCurveFrame(
            evaluateUnitTangent(curve, samples.front().parameter),
            initialReferenceUp
        ));

        for (std::size_t index = 1; index < samples.size(); ++index)
        {
            frames.push_back(detail::transportCurveFrame(
                frames.back(),
                evaluateUnitTangent(curve, samples[index].parameter)
            ));
        }

        return frames;
    }
}
