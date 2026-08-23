#include <quantum/editor/ViewportCamera.hpp>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
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
    constexpr double maximumPitch = 89.0 * pi / 180.0;
    constexpr double framingMargin = 1.1;
    constexpr double minimumDistanceScale = 1.0e-3;
    constexpr double maximumDistanceScale = 1.0e4;
    constexpr double zoomExponentPerWheelUnit = 0.18;
    constexpr double boundsClipPadding = 1.25;
    constexpr double minimumNearScale = 1.0e-4;

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

        boundsCenter_ = 0.5 * (minimumPosition + maximumPosition);
        boundsRadius_ = radius;
        minimumDistance_ = boundsRadius_ * minimumDistanceScale;
        maximumDistance_ = boundsRadius_ * maximumDistanceScale;

        if (!std::isfinite(minimumDistance_)
            || !std::isfinite(maximumDistance_)
            || minimumDistance_ <= 0.0
            || maximumDistance_ <= minimumDistance_)
        {
            throw std::invalid_argument(
                "ViewportCamera bounds are outside its usable distance range."
            );
        }

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

        const double verticalHalfAngle = 0.5 * verticalFieldOfView_;
        const double horizontalHalfAngle = std::atan(
            std::tan(verticalHalfAngle) * aspectRatio
        );
        const double limitingHalfAngle = std::min(
            verticalHalfAngle,
            horizontalHalfAngle
        );
        const double framedDistance = framingMargin * boundsRadius_
            / std::sin(limitingHalfAngle);

        if (!std::isfinite(framedDistance) || framedDistance <= 0.0)
        {
            throw std::runtime_error(
                "ViewportCamera could not compute a finite framing distance."
            );
        }

        focus_ = boundsCenter_;
        distance_ = std::clamp(
            framedDistance,
            minimumDistance_,
            maximumDistance_
        );
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

    void ViewportCamera::zoom(const double wheelDelta)
    {
        if (!std::isfinite(wheelDelta))
        {
            throw std::invalid_argument(
                "ViewportCamera zoom delta must be finite."
            );
        }

        const double exponent = std::clamp(
            -wheelDelta * zoomExponentPerWheelUnit,
            -20.0,
            20.0
        );
        distance_ = std::clamp(
            distance_ * std::exp(exponent),
            minimumDistance_,
            maximumDistance_
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
        glm::dmat4 projection = glm::perspectiveRH_ZO(
            verticalFieldOfView_,
            aspectRatio,
            clipping.nearPlane,
            clipping.farPlane
        );
        projection[1][1] *= -1.0;

        const glm::dmat4 view = glm::lookAtRH(
            position(),
            focus_,
            worldUp
        );
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
        return glm::normalize(glm::cross(-directionFromFocus(), worldUp));
    }

    glm::dvec3 ViewportCamera::up() const noexcept
    {
        const glm::dvec3 forward = -directionFromFocus();
        return glm::normalize(glm::cross(right(), forward));
    }
}
