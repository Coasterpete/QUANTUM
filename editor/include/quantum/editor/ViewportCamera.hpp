#pragma once

#include <glm/vec3.hpp>

#include <array>

namespace quantum::editor
{
    struct ViewportCameraClipPlanes
    {
        double nearPlane = 0.0;
        double farPlane = 0.0;
    };

    class ViewportCamera
    {
    public:
        ViewportCamera();

        void setBounds(
            const glm::dvec3& minimumPosition,
            const glm::dvec3& maximumPosition
        );
        void frame(double aspectRatio);
        void orbit(double yawDeltaRadians, double pitchDeltaRadians);
        void pan(
            double horizontalPixels,
            double verticalPixels,
            double viewportWidth,
            double viewportHeight
        );
        void zoom(double wheelDelta);

        [[nodiscard]] glm::dvec3 focus() const noexcept;
        [[nodiscard]] glm::dvec3 position() const noexcept;
        [[nodiscard]] double distance() const noexcept;
        [[nodiscard]] double yaw() const noexcept;
        [[nodiscard]] double pitch() const noexcept;
        [[nodiscard]] double verticalFieldOfView() const noexcept;
        [[nodiscard]] glm::dvec3 boundsCenter() const noexcept;
        [[nodiscard]] double boundsRadius() const noexcept;
        [[nodiscard]] ViewportCameraClipPlanes clipPlanes() const;

        // The returned column-major matrix uses Vulkan's zero-to-one depth
        // range and Y-flipped projection. Conversion to float happens here at
        // the Editor-to-renderer boundary.
        [[nodiscard]] std::array<float, 16> viewProjection(
            double aspectRatio
        ) const;

    private:
        [[nodiscard]] glm::dvec3 directionFromFocus() const noexcept;
        [[nodiscard]] glm::dvec3 right() const noexcept;
        [[nodiscard]] glm::dvec3 up() const noexcept;

        glm::dvec3 focus_{0.0};
        glm::dvec3 boundsCenter_{0.0};
        double boundsRadius_ = 0.0;
        double yaw_ = 0.0;
        double pitch_ = 0.0;
        double distance_ = 1.0;
        double minimumDistance_ = 1.0;
        double maximumDistance_ = 1.0;
        double verticalFieldOfView_ = 0.0;
        bool hasBounds_ = false;
    };
}
