#include <quantum/editor/ViewportCamera.hpp>

#include <glm/common.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/geometric.hpp>
#include <glm/mat4x4.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    constexpr glm::dvec3 worldUp{0.0, 0.0, 1.0};
    constexpr double pi = 3.14159265358979323846;
    // Drag-orbit stays shy of the vertical poles so a live drag can never
    // flip through them; the Top/Bottom presets set the exact poles and
    // rely on the pole-safe basis below.
    constexpr double maximumPitch = 89.0 * pi / 180.0;
    constexpr double framingMargin = 1.1;
    constexpr double minimumDistanceScale = 1.0e-3;
    constexpr double maximumDistanceScale = 1.0e4;
    constexpr double boundsClipPadding = 1.25;
    constexpr double minimumNearScale = 1.0e-4;
    constexpr double minimumFieldOfView = 10.0 * pi / 180.0;
    constexpr double maximumFieldOfView = 120.0 * pi / 180.0;

    [[nodiscard]] bool isFinite(const glm::dvec3& value) noexcept
    {
        return std::isfinite(value.x)
            && std::isfinite(value.y)
            && std::isfinite(value.z);
    }

    void requireValidAspectRatio(const double aspectRatio)
    {
        if (!std::isfinite(aspectRatio) || aspectRatio <= 0.0)
        {
            throw std::invalid_argument(
                "ViewportCamera requires a finite positive aspect ratio."
            );
        }
    }
}

namespace quantum::editor
{
    ViewportCamera::ViewportCamera()
        : yaw_(std::atan2(-1.5, 1.2)),
          pitch_(std::atan2(1.0, std::hypot(1.2, -1.5))),
          verticalFieldOfView_(45.0 * pi / 180.0)
    {
    }

    void ViewportCamera::setBounds(
        const glm::dvec3& minimumPosition,
        const glm::dvec3& maximumPosition)
    {
        if (!isFinite(minimumPosition) || !isFinite(maximumPosition)
            || minimumPosition.x > maximumPosition.x
            || minimumPosition.y > maximumPosition.y
            || minimumPosition.z > maximumPosition.z)
        {
            throw std::invalid_argument(
                "ViewportCamera requires finite ordered bounds."
            );
        }

        const glm::dvec3 diagonal = maximumPosition - minimumPosition;
        const double radius = 0.5 * glm::length(diagonal);

        if (!std::isfinite(radius) || radius <= 0.0)
        {
            throw std::invalid_argument(
                "ViewportCamera requires bounds with a finite nonzero extent."
            );
        }

        const double minimumDistance = radius * minimumDistanceScale;
        const double maximumDistance = radius * maximumDistanceScale;

        if (!std::isfinite(minimumDistance)
            || !std::isfinite(maximumDistance)
            || minimumDistance <= 0.0
            || maximumDistance <= minimumDistance)
        {
            throw std::invalid_argument(
                "ViewportCamera bounds are outside its usable distance range."
            );
        }

        minimumBounds_ = minimumPosition;
        maximumBounds_ = maximumPosition;
        boundsCenter_ = 0.5 * (minimumPosition + maximumPosition);
        boundsRadius_ = radius;
        minimumDistance_ = minimumDistance;
        maximumDistance_ = maximumDistance;

        if (!hasBounds_)
        {
            distance_ = std::clamp(
                distance_,
                minimumDistance_,
                maximumDistance_
            );
        }
        hasBounds_ = true;
    }

    void ViewportCamera::frame(const double aspectRatio)
    {
        requireValidAspectRatio(aspectRatio);

        if (!hasBounds_)
        {
            throw std::logic_error(
                "ViewportCamera cannot frame before bounds are assigned."
            );
        }

        // Assigned camera bounds guarantee finite ordered nonzero bounds, so
        // the failure path of frameBounds is unreachable here.
        static_cast<void>(
            frameBounds(minimumBounds_, maximumBounds_, aspectRatio)
        );
    }

    bool ViewportCamera::frameBounds(
        const glm::dvec3& minimumPosition,
        const glm::dvec3& maximumPosition,
        const double aspectRatio)
    {
        requireValidAspectRatio(aspectRatio);

        if (!hasBounds_)
        {
            throw std::logic_error(
                "ViewportCamera cannot frame before bounds are assigned."
            );
        }

        if (!isFinite(minimumPosition) || !isFinite(maximumPosition)
            || minimumPosition.x > maximumPosition.x
            || minimumPosition.y > maximumPosition.y
            || minimumPosition.z > maximumPosition.z)
        {
            return false;
        }

        const glm::dvec3 halfExtents =
            0.5 * (maximumPosition - minimumPosition);
        if (glm::length(halfExtents) <= 0.0)
        {
            return false;
        }

        const double verticalTangent =
            std::tan(0.5 * verticalFieldOfView_);
        const double horizontalTangent = verticalTangent * aspectRatio;
        const glm::dvec3 forward = -directionFromFocus();

        // An axis-aligned box's extent along an arbitrary unit axis is the
        // dot product of its half-extents with that axis's absolute value.
        // Keeping the camera beyond the nearest depth extent also prevents
        // very thin bounds viewed end-on from placing geometry at the eye.
        const double halfWidth = glm::dot(halfExtents, glm::abs(right()));
        const double halfHeight = glm::dot(halfExtents, glm::abs(up()));
        const double halfDepth = glm::dot(halfExtents, glm::abs(forward));
        const double projectedDistance = std::max(
            halfWidth / horizontalTangent,
            halfHeight / verticalTangent
        );
        const double framedDistance = projection_
                == ViewportProjection::Perspective
            ? framingMargin * (halfDepth + projectedDistance)
            : framingMargin * std::max(halfDepth, projectedDistance);

        if (!std::isfinite(framedDistance) || framedDistance <= 0.0)
        {
            return false;
        }

        focus_ = 0.5 * (minimumPosition + maximumPosition);
        distance_ = std::clamp(
            framedDistance,
            minimumDistance_,
            maximumDistance_
        );
        return true;
    }

    bool ViewportCamera::frameSphere(
        const glm::dvec3& center,
        const double radius,
        const double aspectRatio)
    {
        requireValidAspectRatio(aspectRatio);

        if (!hasBounds_)
        {
            throw std::logic_error(
                "ViewportCamera cannot frame before bounds are assigned."
            );
        }

        if (!isFinite(center) || !std::isfinite(radius) || radius <= 0.0)
        {
            return false;
        }

        const double verticalHalfAngle = 0.5 * verticalFieldOfView_;
        const double horizontalHalfAngle = std::atan(
            std::tan(verticalHalfAngle) * aspectRatio
        );
        const double limitingHalfAngle = std::min(
            verticalHalfAngle,
            horizontalHalfAngle
        );
        const double framedDistance = framingMargin * radius
            / std::sin(limitingHalfAngle);

        if (!std::isfinite(framedDistance) || framedDistance <= 0.0)
        {
            return false;
        }

        focus_ = center;
        distance_ = std::clamp(
            framedDistance,
            minimumDistance_,
            maximumDistance_
        );
        return true;
    }

    void ViewportCamera::applyPreset(const ViewportCameraPreset preset)
    {
        switch (preset)
        {
        case ViewportCameraPreset::Perspective:
            projection_ = ViewportProjection::Perspective;
            yaw_ = std::atan2(-1.5, 1.2);
            pitch_ = std::atan2(1.0, std::hypot(1.2, -1.5));
            break;
        case ViewportCameraPreset::Isometric:
            // Classic isometric: orthographic with equal 120-degree axis
            // separation, i.e. elevation atan(1/sqrt(2)) at azimuth 45.
            projection_ = ViewportProjection::Orthographic;
            yaw_ = 0.25 * pi;
            pitch_ = std::atan(1.0 / std::sqrt(2.0));
            break;
        case ViewportCameraPreset::Top:
            yaw_ = 0.0;
            pitch_ = 0.5 * pi;
            break;
        case ViewportCameraPreset::Bottom:
            yaw_ = 0.0;
            pitch_ = -0.5 * pi;
            break;
        case ViewportCameraPreset::Left:
            // Rider-facing convention: travel starts along +X with world up
            // +Z, so the rider's left side is +Y and the Left view looks
            // from there toward -Y.
            yaw_ = 0.5 * pi;
            pitch_ = 0.0;
            break;
        case ViewportCameraPreset::Right:
            yaw_ = -0.5 * pi;
            pitch_ = 0.0;
            break;
        }
    }

    void ViewportCamera::setPose(const ViewportCameraPose& pose)
    {
        if (!isFinite(pose.focus)
            || !std::isfinite(pose.yaw)
            || !std::isfinite(pose.pitch)
            || !std::isfinite(pose.distance)
            || pose.distance <= 0.0)
        {
            throw std::invalid_argument(
                "ViewportCamera requires a finite pose with a positive "
                "distance."
            );
        }

        focus_ = pose.focus;
        yaw_ = std::remainder(pose.yaw, 2.0 * pi);
        pitch_ = pose.pitch;
        distance_ = hasBounds_
            ? std::clamp(pose.distance, minimumDistance_, maximumDistance_)
            : pose.distance;
    }

    void ViewportCamera::orbit(
        const double yawDeltaRadians,
        const double pitchDeltaRadians)
    {
        if (!std::isfinite(yawDeltaRadians)
            || !std::isfinite(pitchDeltaRadians))
        {
            throw std::invalid_argument(
                "ViewportCamera orbit deltas must be finite."
            );
        }

        yaw_ = std::remainder(yaw_ + yawDeltaRadians, 2.0 * pi);
        pitch_ = std::clamp(
            pitch_ + pitchDeltaRadians,
            -maximumPitch,
            maximumPitch
        );
    }

    void ViewportCamera::pan(
        const double horizontalPixels,
        const double verticalPixels,
        const double viewportWidth,
        const double viewportHeight)
    {
        if (!std::isfinite(horizontalPixels)
            || !std::isfinite(verticalPixels)
            || !std::isfinite(viewportWidth)
            || !std::isfinite(viewportHeight)
            || viewportWidth <= 0.0
            || viewportHeight <= 0.0)
        {
            throw std::invalid_argument(
                "ViewportCamera pan requires finite deltas and viewport dimensions."
            );
        }

        const double visibleHeightAtFocus = 2.0 * distance_
            * std::tan(0.5 * verticalFieldOfView_);
        const double unitsPerPixel = visibleHeightAtFocus / viewportHeight;
        const glm::dvec3 displacement = unitsPerPixel
            * (-horizontalPixels * right() + verticalPixels * up());

        if (!isFinite(displacement))
        {
            throw std::runtime_error(
                "ViewportCamera pan produced a non-finite displacement."
            );
        }

        focus_ += displacement;
    }

    void ViewportCamera::zoom(
        const double wheelDelta,
        const double exponentPerWheelUnit)
    {
        if (!std::isfinite(wheelDelta)
            || !std::isfinite(exponentPerWheelUnit)
            || exponentPerWheelUnit <= 0.0)
        {
            throw std::invalid_argument(
                "ViewportCamera zoom requires a finite delta and a "
                "positive exponent scale."
            );
        }

        const double exponent = std::clamp(
            -wheelDelta * exponentPerWheelUnit,
            -20.0,
            20.0
        );
        distance_ = std::clamp(
            distance_ * std::exp(exponent),
            minimumDistance_,
            maximumDistance_
        );
    }

    void ViewportCamera::setProjection(
        const ViewportProjection projection)
    {
        projection_ = projection;
    }

    void ViewportCamera::setVerticalFieldOfView(const double radians)
    {
        if (!std::isfinite(radians))
        {
            throw std::invalid_argument(
                "ViewportCamera field of view must be finite."
            );
        }

        verticalFieldOfView_ = std::clamp(
            radians,
            minimumFieldOfView,
            maximumFieldOfView
        );
    }

    glm::dvec3 ViewportCamera::focus() const noexcept
    {
        return focus_;
    }

    glm::dvec3 ViewportCamera::position() const noexcept
    {
        return focus_ + distance_ * directionFromFocus();
    }

    double ViewportCamera::distance() const noexcept
    {
        return distance_;
    }

    double ViewportCamera::yaw() const noexcept
    {
        return yaw_;
    }

    double ViewportCamera::pitch() const noexcept
    {
        return pitch_;
    }

    double ViewportCamera::verticalFieldOfView() const noexcept
    {
        return verticalFieldOfView_;
    }

    ViewportProjection ViewportCamera::projection() const noexcept
    {
        return projection_;
    }

    bool ViewportCamera::hasBounds() const noexcept
    {
        return hasBounds_;
    }

    glm::dvec3 ViewportCamera::boundsCenter() const noexcept
    {
        return boundsCenter_;
    }

    double ViewportCamera::boundsRadius() const noexcept
    {
        return boundsRadius_;
    }

    ViewportCameraClipPlanes ViewportCamera::clipPlanes() const
    {
        if (!hasBounds_)
        {
            throw std::logic_error(
                "ViewportCamera cannot compute clip planes before bounds are assigned."
            );
        }

        const glm::dvec3 cameraPosition = position();
        const glm::dvec3 forward = -directionFromFocus();
        const double boundsCenterDepth = glm::dot(
            boundsCenter_ - cameraPosition,
            forward
        );
        const double paddedRadius = boundsClipPadding * boundsRadius_;
        const double minimumNear = std::max(
            boundsRadius_ * minimumNearScale,
            std::numeric_limits<double>::min()
        );
        const double nearPlane = std::max(
            minimumNear,
            boundsCenterDepth - paddedRadius
        );
        const double distanceToBoundsCenter = glm::length(
            boundsCenter_ - cameraPosition
        );
        const double minimumClipSpan = std::max(
            boundsRadius_ * minimumDistanceScale,
            minimumNear
        );
        const double farPlane = std::max({
            nearPlane + minimumClipSpan,
            boundsCenterDepth + paddedRadius,
            distanceToBoundsCenter + paddedRadius
        });

        if (!std::isfinite(nearPlane)
            || !std::isfinite(farPlane)
            || nearPlane <= 0.0
            || farPlane <= nearPlane)
        {
            throw std::runtime_error(
                "ViewportCamera produced invalid clip planes."
            );
        }

        return {nearPlane, farPlane};
    }

    std::array<float, 16> ViewportCamera::viewProjection(
        const double aspectRatio) const
    {
        requireValidAspectRatio(aspectRatio);

        const ViewportCameraClipPlanes clipping = clipPlanes();
        glm::dmat4 projection{1.0};

        if (projection_ == ViewportProjection::Perspective)
        {
            projection = glm::perspectiveRH_ZO(
                verticalFieldOfView_,
                aspectRatio,
                clipping.nearPlane,
                clipping.farPlane
            );
        }
        else
        {
            // Tie the orthographic frustum to the orbit distance and field
            // of view so perspective/orthographic switches keep apparent
            // size, pan scale, and dolly steps consistent.
            const double halfHeightAtFocus = distance_
                * std::tan(0.5 * verticalFieldOfView_);
            projection = glm::orthoRH_ZO(
                -halfHeightAtFocus * aspectRatio,
                halfHeightAtFocus * aspectRatio,
                -halfHeightAtFocus,
                halfHeightAtFocus,
                clipping.nearPlane,
                clipping.farPlane
            );
        }
        projection[1][1] *= -1.0;

        // Hand-built right-handed look-at instead of glm::lookAtRH: the
        // basis comes from the same pole-safe helpers as pan/orbit, so the
        // exact Top/Bottom presets remain well-defined.
        const glm::dvec3 eye = position();
        const glm::dvec3 forward = -directionFromFocus();
        const glm::dvec3 rightVector = right();
        const glm::dvec3 upVector = up();

        glm::dmat4 view{1.0};
        view[0][0] = rightVector.x;
        view[0][1] = upVector.x;
        view[0][2] = -forward.x;
        view[1][0] = rightVector.y;
        view[1][1] = upVector.y;
        view[1][2] = -forward.y;
        view[2][0] = rightVector.z;
        view[2][1] = upVector.z;
        view[2][2] = -forward.z;
        view[3][0] = -glm::dot(rightVector, eye);
        view[3][1] = -glm::dot(upVector, eye);
        view[3][2] = glm::dot(forward, eye);

        const glm::dmat4 viewProjection = projection * view;
        std::array<float, 16> rendererMatrix{};

        for (std::size_t column = 0; column < 4; ++column)
        {
            for (std::size_t row = 0; row < 4; ++row)
            {
                const double component = viewProjection[column][row];
                const float rendererComponent = static_cast<float>(component);

                if (!std::isfinite(component)
                    || !std::isfinite(rendererComponent))
                {
                    throw std::runtime_error(
                        "ViewportCamera matrix is outside the renderer's finite float range."
                    );
                }

                rendererMatrix[column * 4 + row] = rendererComponent;
            }
        }

        return rendererMatrix;
    }

    ViewportRay ViewportCamera::viewportRay(
        const double normalizedX,
        const double normalizedY,
        const double aspectRatio) const
    {
        requireValidAspectRatio(aspectRatio);

        if (!std::isfinite(normalizedX)
            || !std::isfinite(normalizedY)
            || normalizedX < 0.0
            || normalizedX > 1.0
            || normalizedY < 0.0
            || normalizedY > 1.0)
        {
            throw std::invalid_argument(
                "ViewportCamera ray coordinates must be finite and inside the viewport."
            );
        }

        const double horizontal = 2.0 * normalizedX - 1.0;
        const double vertical = 1.0 - 2.0 * normalizedY;
        const double halfHeight = distance_
            * std::tan(0.5 * verticalFieldOfView_);
        const glm::dvec3 forward = -directionFromFocus();
        const glm::dvec3 rightVector = right();
        const glm::dvec3 upVector = up();

        if (projection_ == ViewportProjection::Orthographic)
        {
            return {
                position()
                    + horizontal * halfHeight * aspectRatio * rightVector
                    + vertical * halfHeight * upVector,
                forward
            };
        }

        const glm::dvec3 direction = glm::normalize(
            forward
                + horizontal * std::tan(0.5 * verticalFieldOfView_)
                    * aspectRatio * rightVector
                + vertical * std::tan(0.5 * verticalFieldOfView_)
                    * upVector
        );

        if (!isFinite(direction))
        {
            throw std::runtime_error(
                "ViewportCamera produced a non-finite picking ray."
            );
        }

        return {position(), direction};
    }

    glm::dvec3 ViewportCamera::directionFromFocus() const noexcept
    {
        const double horizontalScale = std::cos(pitch_);
        return {
            horizontalScale * std::cos(yaw_),
            horizontalScale * std::sin(yaw_),
            std::sin(pitch_)
        };
    }

    glm::dvec3 ViewportCamera::right() const noexcept
    {
        const glm::dvec3 forward = -directionFromFocus();

        // The cross product with worldUp degenerates exactly at the
        // vertical poles (Top/Bottom presets); its analytic limit there is
        // the yaw-aligned horizontal vector below, so use it to stay
        // continuous and NaN-free.
        if (std::abs(forward.z) < 1.0 - 1.0e-9)
        {
            return glm::normalize(glm::cross(forward, worldUp));
        }

        return {-std::sin(yaw_), std::cos(yaw_), 0.0};
    }

    glm::dvec3 ViewportCamera::up() const noexcept
    {
        const glm::dvec3 forward = -directionFromFocus();
        return glm::normalize(glm::cross(right(), forward));
    }
}
