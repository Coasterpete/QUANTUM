#pragma once

#include <glm/vec3.hpp>

#include <array>

namespace quantum::editor
{
    // Wheel-dolly strength used when the caller has no explicit zoom
    // sensitivity configured; larger exponents dolly faster per wheel unit.
    inline constexpr double defaultViewportZoomExponentPerWheelUnit = 0.18;
    inline constexpr double defaultViewportMovementUnitsPerSecond = 18.0;
    inline constexpr double defaultViewportFastMovementMultiplier = 4.0;
    inline constexpr double maximumViewportNavigationDeltaSeconds = 0.1;

    struct ViewportCameraClipPlanes
    {
        double nearPlane = 0.0;
        double farPlane = 0.0;
    };

    // ImGui-derived focus/capture facts are reduced to this backend-neutral
    // value so the navigation gate can be tested without a live UI context.
    struct ViewportKeyboardNavigationState
    {
        bool viewportActive = false;
        bool keyboardCaptured = false;
        bool textInputActive = false;
        bool itemActive = false;
        bool popupOpen = false;
        bool commandModifierDown = false;
    };

    [[nodiscard]] bool acceptsViewportKeyboardNavigation(
        const ViewportKeyboardNavigationState& state) noexcept;

    // Orthographic mode derives its half-height at the focus from the same
    // distance and field of view as perspective mode, so switching between
    // them preserves apparent object size, pan speed, and zoom feel.
    enum class ViewportProjection
    {
        Perspective,
        Orthographic
    };

    // Deterministic camera orientation presets. Perspective and Isometric
    // additionally switch the projection; the axis presets keep the current
    // projection and only change yaw/pitch. All of them preserve focus and
    // distance so the user stays anchored on what they were looking at.
    enum class ViewportCameraPreset
    {
        Perspective,
        Isometric,
        Top,
        Bottom,
        Left,
        Right
    };

    // Explicit camera pose used by geometry-derived views (Track, Walking).
    // The same orbit-spherical conventions apply: position is focus plus
    // distance times directionFromFocus(yaw, pitch).
    struct ViewportCameraPose
    {
        glm::dvec3 focus{0.0};
        double yaw = 0.0;
        double pitch = 0.0;
        double distance = 1.0;
    };

    // World-space ray through one normalized viewport location. Viewport
    // coordinates use the ImGui image convention: (0, 0) is the top-left
    // corner and (1, 1) is the bottom-right corner.
    struct ViewportRay
    {
        glm::dvec3 origin{0.0};
        glm::dvec3 direction{1.0, 0.0, 0.0};
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

        // Frames axis-aligned world bounds in the current view orientation.
        // Unlike frameSphere(), this uses their projected width and height so
        // elongated track bounds are not reduced to an excessively large
        // sphere. Returns false without moving the camera for invalid or
        // zero-extent bounds.
        bool frameBounds(
            const glm::dvec3& minimumPosition,
            const glm::dvec3& maximumPosition,
            double aspectRatio
        );

        // Frames an arbitrary sphere with the same margin formula as
        // other framing operations; returns false (leaving the camera
        // unchanged) for a non-finite center or a non-positive radius, which
        // is how an invalid selection is reported to callers.
        bool frameSphere(
            const glm::dvec3& center,
            double radius,
            double aspectRatio
        );

        // Applies a canonical orientation (and projection for Perspective/
        // Isometric) while preserving focus and distance.
        void applyPreset(ViewportCameraPreset preset);

        // Direct pose assignment for geometry-derived views. Throws
        // std::invalid_argument for non-finite components or a non-positive
        // distance; distance is clamped to the bounds-derived limits.
        void setPose(const ViewportCameraPose& pose);

        void orbit(double yawDeltaRadians, double pitchDeltaRadians);
        void pan(
            double horizontalPixels,
            double verticalPixels,
            double viewportWidth,
            double viewportHeight
        );
        void zoom(
            double wheelDelta,
            double exponentPerWheelUnit =
                defaultViewportZoomExponentPerWheelUnit
        );

        // Rotates the viewing direction in place, preserving the eye
        // position and focus distance. This is distinct from orbit(), which
        // preserves the focus and moves the eye around it.
        void look(double yawDeltaRadians, double pitchDeltaRadians);

        // Translates the eye and focus together along the camera-local
        // forward, right, and up axes. Input is normalized so diagonal
        // movement is not faster. deltaSeconds and unitsPerSecond make the
        // operation deterministic and frame-rate independent.
        void moveLocal(
            double forwardInput,
            double rightInput,
            double upInput,
            double deltaSeconds,
            double unitsPerSecond = defaultViewportMovementUnitsPerSecond
        );

        void setProjection(ViewportProjection projection);
        void setVerticalFieldOfView(double radians);

        [[nodiscard]] glm::dvec3 focus() const noexcept;
        [[nodiscard]] glm::dvec3 position() const noexcept;
        [[nodiscard]] double distance() const noexcept;
        [[nodiscard]] double yaw() const noexcept;
        [[nodiscard]] double pitch() const noexcept;
        [[nodiscard]] double verticalFieldOfView() const noexcept;
        [[nodiscard]] ViewportProjection projection() const noexcept;
        [[nodiscard]] bool hasBounds() const noexcept;
        [[nodiscard]] glm::dvec3 boundsCenter() const noexcept;
        [[nodiscard]] double boundsRadius() const noexcept;
        [[nodiscard]] ViewportCameraClipPlanes clipPlanes() const;

        // The returned column-major matrix uses Vulkan's zero-to-one depth
        // range and Y-flipped projection. Conversion to float happens here at
        // the Editor-to-renderer boundary.
        [[nodiscard]] std::array<float, 16> viewProjection(
            double aspectRatio
        ) const;

        // Derives the world-space ray used by viewport picking. Perspective
        // rays originate at the eye; orthographic rays share the camera's
        // forward direction and originate at the corresponding image-plane
        // position. Throws for coordinates outside [0, 1] or an invalid
        // aspect ratio.
        [[nodiscard]] ViewportRay viewportRay(
            double normalizedX,
            double normalizedY,
            double aspectRatio
        ) const;

    private:
        [[nodiscard]] glm::dvec3 directionFromFocus() const noexcept;
        [[nodiscard]] glm::dvec3 right() const noexcept;
        [[nodiscard]] glm::dvec3 up() const noexcept;

        glm::dvec3 focus_{0.0};
        glm::dvec3 minimumBounds_{0.0};
        glm::dvec3 maximumBounds_{0.0};
        glm::dvec3 boundsCenter_{0.0};
        double boundsRadius_ = 0.0;
        double yaw_ = 0.0;
        double pitch_ = 0.0;
        double distance_ = 1.0;
        double minimumDistance_ = 1.0;
        double maximumDistance_ = 1.0;
        double verticalFieldOfView_ = 0.0;
        ViewportProjection projection_ = ViewportProjection::Perspective;
        bool hasBounds_ = false;
    };
}
