#include <quantum/editor/ViewportCamera.hpp>

#include <glm/geometric.hpp>

#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{
    using quantum::editor::ViewportCamera;

    constexpr double tolerance = 1.0e-12;
    constexpr glm::dvec3 minimumBounds{-10.0, -5.0, -2.0};
    constexpr glm::dvec3 maximumBounds{30.0, 15.0, 8.0};
    constexpr double wideAspectRatio = 16.0 / 9.0;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    void requireNear(
        const double actual,
        const double expected,
        const std::string_view context)
    {
        if (!std::isfinite(actual)
            || std::abs(actual - expected) > tolerance)
        {
            throw std::runtime_error(
                std::string(context) + ": expected "
                    + std::to_string(expected) + ", received "
                    + std::to_string(actual)
            );
        }
    }

    void requireVectorNear(
        const glm::dvec3& actual,
        const glm::dvec3& expected,
        const std::string_view context)
    {
        requireNear(glm::length(actual - expected), 0.0, context);
    }

    [[nodiscard]] ViewportCamera makeFramedCamera(
        const double aspectRatio = wideAspectRatio)
    {
        ViewportCamera camera;
        camera.setBounds(minimumBounds, maximumBounds);
        camera.frame(aspectRatio);
        return camera;
    }

    void requireFiniteMatrix(const std::array<float, 16>& matrix)
    {
        for (const float component : matrix)
        {
            require(
                std::isfinite(component),
                "view-projection matrix components must be finite"
            );
        }
    }

    void testBoundsFraming()
    {
        ViewportCamera wideCamera = makeFramedCamera();
        const glm::dvec3 expectedCenter{10.0, 5.0, 3.0};
        const double expectedRadius = 0.5 * glm::length(
            maximumBounds - minimumBounds
        );

        requireVectorNear(
            wideCamera.focus(),
            expectedCenter,
            "frame focus"
        );
        requireVectorNear(
            wideCamera.boundsCenter(),
            expectedCenter,
            "bounds center"
        );
        requireNear(
            wideCamera.boundsRadius(),
            expectedRadius,
            "bounds radius"
        );
        require(
            wideCamera.distance() > expectedRadius,
            "framing distance must exceed the bounding radius"
        );

        ViewportCamera tallCamera = makeFramedCamera(0.5);
        require(
            tallCamera.distance() > wideCamera.distance(),
            "a tall viewport must frame farther away than a wide viewport"
        );
    }

    void testOrbitPreservesDistance()
    {
        ViewportCamera camera = makeFramedCamera();
        const double distanceBefore = camera.distance();
        const glm::dvec3 focusBefore = camera.focus();
        const glm::dvec3 positionBefore = camera.position();

        camera.orbit(0.7, -0.4);

        requireNear(camera.distance(), distanceBefore, "orbit distance");
        requireVectorNear(camera.focus(), focusBefore, "orbit focus");
        requireNear(
            glm::length(camera.position() - camera.focus()),
            distanceBefore,
            "camera-to-focus distance"
        );
        require(
            glm::length(camera.position() - positionBefore) > 1.0,
            "orbit must move the camera position"
        );

        camera.orbit(0.0, 1.0e6);
        require(
            std::abs(camera.pitch()) < 0.5 * 3.14159265358979323846,
            "orbit pitch must stay away from the world-up singularity"
        );
        requireFiniteMatrix(camera.viewProjection(wideAspectRatio));
    }

    void testPanPreservesCameraOffset()
    {
        ViewportCamera camera = makeFramedCamera();
        camera.orbit(0.25, -0.2);
        const glm::dvec3 offsetBefore = camera.position() - camera.focus();
        const glm::dvec3 focusBefore = camera.focus();

        camera.pan(180.0, -90.0, 1280.0, 720.0);

        requireVectorNear(
            camera.position() - camera.focus(),
            offsetBefore,
            "pan camera-to-focus offset"
        );
        require(
            glm::length(camera.focus() - focusBefore) > 0.0,
            "pan must move the focus"
        );
    }

    void testZoomRemainsPositiveAndFinite()
    {
        ViewportCamera camera = makeFramedCamera();
        const double initialDistance = camera.distance();

        camera.zoom(4.0);
        require(
            camera.distance() < initialDistance,
            "positive wheel input must dolly inward"
        );
        camera.zoom(1.0e9);
        require(
            std::isfinite(camera.distance()) && camera.distance() > 0.0,
            "maximum zoom-in must retain a finite positive distance"
        );
        camera.zoom(-1.0e9);
        require(
            std::isfinite(camera.distance()) && camera.distance() > 0.0,
            "maximum zoom-out must retain a finite positive distance"
        );
    }

    void testFrameRecoversTransformedState()
    {
        ViewportCamera expected = makeFramedCamera();
        expected.orbit(1.2, -0.8);
        expected.frame(0.75);

        ViewportCamera camera = makeFramedCamera();
        camera.orbit(1.2, -0.8);
        camera.pan(20'000.0, -15'000.0, 800.0, 600.0);
        camera.zoom(1.0e6);

        camera.frame(0.75);

        requireVectorNear(
            camera.focus(),
            camera.boundsCenter(),
            "reframed focus"
        );
        requireNear(
            camera.distance(),
            expected.distance(),
            "reframed distance"
        );
        const auto clipping = camera.clipPlanes();
        require(
            std::isfinite(clipping.nearPlane)
                && std::isfinite(clipping.farPlane)
                && clipping.nearPlane > 0.0
                && clipping.farPlane > clipping.nearPlane,
            "reframed clip planes must be finite and ordered"
        );
        requireFiniteMatrix(camera.viewProjection(0.75));
    }

    void testUpdatedBoundsPreserveCameraUntilExplicitFrame()
    {
        ViewportCamera camera = makeFramedCamera();
        camera.orbit(0.4, -0.2);
        camera.pan(75.0, -40.0, 1280.0, 720.0);
        camera.zoom(1.5);

        const glm::dvec3 focusBefore = camera.focus();
        const glm::dvec3 positionBefore = camera.position();
        const double distanceBefore = camera.distance();
        const double yawBefore = camera.yaw();
        const double pitchBefore = camera.pitch();
        constexpr glm::dvec3 updatedMinimum{-120.0, -40.0, -25.0};
        constexpr glm::dvec3 updatedMaximum{210.0, 90.0, 65.0};

        camera.setBounds(updatedMinimum, updatedMaximum);

        requireVectorNear(camera.focus(), focusBefore, "updated-bounds focus");
        requireVectorNear(
            camera.position(),
            positionBefore,
            "updated-bounds position"
        );
        requireNear(
            camera.distance(),
            distanceBefore,
            "updated-bounds distance"
        );
        requireNear(camera.yaw(), yawBefore, "updated-bounds yaw");
        requireNear(camera.pitch(), pitchBefore, "updated-bounds pitch");
        requireFiniteMatrix(camera.viewProjection(wideAspectRatio));

        camera.frame(wideAspectRatio);
        requireVectorNear(
            camera.focus(),
            camera.boundsCenter(),
            "updated-bounds framed focus"
        );
        require(
            glm::length(camera.position() - positionBefore) > 0.0,
            "explicit frame must use the updated centerline bounds"
        );
    }

    void testMatrixGenerationIsDeterministic()
    {
        ViewportCamera camera = makeFramedCamera();
        camera.orbit(-0.3, 0.15);
        camera.pan(-45.0, 23.0, 1600.0, 900.0);
        camera.zoom(-2.0);

        const auto first = camera.viewProjection(wideAspectRatio);
        const auto second = camera.viewProjection(wideAspectRatio);

        require(first == second, "matrix generation must be deterministic");
        requireFiniteMatrix(first);
    }

    // Transforms one world point by a column-major float[16] matrix and
    // returns clip-space (x, y, z, w).
    [[nodiscard]] std::array<double, 4> transformPoint(
        const std::array<float, 16>& matrix,
        const glm::dvec3& point)
    {
        std::array<double, 4> result{};
        const double components[4] = {point.x, point.y, point.z, 1.0};
        for (std::size_t row = 0; row < 4; ++row)
        {
            double sum = 0.0;
            for (std::size_t column = 0; column < 4; ++column)
            {
                sum += static_cast<double>(matrix[column * 4 + row])
                    * components[column];
            }
            result[row] = sum;
        }
        return result;
    }

    void testElongatedTrackBoundsUseProjectedExtent()
    {
        constexpr double viewportAspectRatio = 5.0;
        constexpr glm::dvec3 trackMinimum{0.0, 0.0, 0.0};
        constexpr glm::dvec3 trackMaximum{60.0, 0.0, 0.0};

        ViewportCamera camera;
        camera.setBounds(trackMinimum, trackMaximum);
        camera.frame(viewportAspectRatio);

        requireVectorNear(camera.focus(), {30.0, 0.0, 0.0},
            "elongated bounds focus");

        const double legacySphereDistance = 1.1 * 30.0
            / std::sin(0.5 * camera.verticalFieldOfView());
        require(
            camera.distance() < 0.75 * legacySphereDistance,
            "elongated bounds must use their projected extent instead of "
            "the AABB bounding sphere"
        );

        const auto matrix = camera.viewProjection(viewportAspectRatio);
        const auto start = transformPoint(matrix, trackMinimum);
        const auto end = transformPoint(matrix, trackMaximum);
        require(start[3] > 0.0 && end[3] > 0.0,
            "framed track endpoints must remain in front of the camera");

        const double normalizedSpan = 0.5 * std::hypot(
            start[0] / start[3] - end[0] / end[3],
            start[1] / start[3] - end[1] / end[3]
        );
        require(
            normalizedSpan > 0.5,
            "a 60-unit straight track must substantially fill a wide "
            "Perspective viewport"
        );

        const double wholeTrackDistance = camera.distance();
        require(camera.frameBounds(
            {0.0, 0.0, 0.0},
            {20.0, 0.0, 0.0},
            viewportAspectRatio),
            "valid selected-section bounds must frame successfully");
        requireVectorNear(camera.focus(), {10.0, 0.0, 0.0},
            "selected-section bounds focus");
        require(camera.distance() < wholeTrackDistance,
            "selected-section bounds must frame closer than the whole track");
    }

    void testDeterministicPresets()
    {
        using quantum::editor::ViewportCameraPreset;
        using quantum::editor::ViewportProjection;

        struct PresetExpectation
        {
            ViewportCameraPreset preset;
            const char* name;
            double yaw;
            double pitch;
            ViewportProjection projection;
        };

        const double isoPitch = std::atan(1.0 / std::sqrt(2.0));
        const double defaultYaw = std::atan2(-1.5, 1.2);
        const double defaultPitch = std::atan2(
            1.0, std::hypot(1.2, -1.5));

        const PresetExpectation expectations[] = {
            {ViewportCameraPreset::Perspective, "perspective",
                defaultYaw, defaultPitch, ViewportProjection::Perspective},
            {ViewportCameraPreset::Isometric, "isometric",
                0.25 * 3.14159265358979323846, isoPitch,
                ViewportProjection::Orthographic},
            {ViewportCameraPreset::Top, "top",
                0.0, 0.5 * 3.14159265358979323846,
                ViewportProjection::Orthographic},
            {ViewportCameraPreset::Bottom, "bottom",
                0.0, -0.5 * 3.14159265358979323846,
                ViewportProjection::Orthographic},
            {ViewportCameraPreset::Left, "left",
                0.5 * 3.14159265358979323846, 0.0,
                ViewportProjection::Orthographic},
            {ViewportCameraPreset::Right, "right",
                -0.5 * 3.14159265358979323846, 0.0,
                ViewportProjection::Orthographic},
        };

        for (const PresetExpectation& expectation : expectations)
        {
            ViewportCamera camera = makeFramedCamera();
            // Start from a different projection and heavily perturbed
            // orientation to prove the preset fully determines both.
            camera.setProjection(ViewportProjection::Orthographic);
            camera.orbit(2.3, 0.9);
            camera.applyPreset(expectation.preset);

            requireNear(
                camera.yaw(),
                expectation.yaw,
                std::string(expectation.name) + " yaw"
            );
            requireNear(
                camera.pitch(),
                expectation.pitch,
                std::string(expectation.name) + " pitch"
            );
            if (expectation.preset == ViewportCameraPreset::Perspective
                || expectation.preset == ViewportCameraPreset::Isometric)
            {
                require(
                    camera.projection() == expectation.projection,
                    std::string(expectation.name)
                        + " must switch projection"
                );
            }
            requireFiniteMatrix(camera.viewProjection(wideAspectRatio));
        }

        // Axis presets keep focus and distance and place the eye on the
        // correct side of the focus.
        ViewportCamera top = makeFramedCamera();
        const glm::dvec3 focusBefore = top.focus();
        const double distanceBefore = top.distance();
        top.applyPreset(ViewportCameraPreset::Top);
        require(top.position().z > top.focus().z + distanceBefore * 0.999,
            "Top preset must look straight down from above");
        requireVectorNear(top.focus(), focusBefore, "Top keeps focus");

        ViewportCamera bottom = makeFramedCamera();
        bottom.applyPreset(ViewportCameraPreset::Bottom);
        require(bottom.position().z < bottom.focus().z - distanceBefore * 0.999,
            "Bottom preset must look straight up from below");

        ViewportCamera left = makeFramedCamera();
        left.applyPreset(ViewportCameraPreset::Left);
        require(left.position().y > left.focus().y + distanceBefore * 0.999,
            "Left preset must place the eye on the +Y side");

        ViewportCamera right = makeFramedCamera();
        right.applyPreset(ViewportCameraPreset::Right);
        require(right.position().y < right.focus().y - distanceBefore * 0.999,
            "Right preset must place the eye on the -Y side");
    }

    void testPoleSafeMatrices()
    {
        for (const quantum::editor::ViewportCameraPreset polePreset :
            {quantum::editor::ViewportCameraPreset::Top,
                quantum::editor::ViewportCameraPreset::Bottom})
        {
            ViewportCamera camera = makeFramedCamera();
            camera.applyPreset(polePreset);

            const auto clipping = camera.clipPlanes();
            require(
                clipping.nearPlane > 0.0
                    && clipping.farPlane > clipping.nearPlane,
                "pole view clip planes must stay ordered"
            );
            requireFiniteMatrix(camera.viewProjection(wideAspectRatio));

            // Panning and zooming at the exact pole must remain finite.
            camera.pan(40.0, -30.0, 1280.0, 720.0);
            camera.zoom(-3.0);
            require(std::isfinite(camera.distance())
                    && camera.distance() > 0.0,
                "pan/zoom at the pole must keep a finite distance");
            requireFiniteMatrix(camera.viewProjection(wideAspectRatio));

            // Orbiting away from the pole recovers normal behavior.
            camera.orbit(0.6, -0.2);
            require(
                std::abs(camera.pitch()) < 0.5 * 3.14159265358979323846,
                "orbit must leave the exact pole when dragged"
            );
        }
    }

    void testProjectionSwitching()
    {
        using quantum::editor::ViewportProjection;

        ViewportCamera camera = makeFramedCamera();
        require(
            camera.projection() == ViewportProjection::Perspective,
            "cameras start in perspective mode"
        );

        const glm::dvec3 focusBefore = camera.focus();
        const double distanceBefore = camera.distance();

        camera.setProjection(ViewportProjection::Orthographic);
        require(
            camera.projection() == ViewportProjection::Orthographic,
            "projection mode must switch"
        );
        requireVectorNear(camera.focus(), focusBefore,
            "ortho switch preserves focus");
        requireNear(camera.distance(), distanceBefore,
            "ortho switch preserves distance");
        requireFiniteMatrix(camera.viewProjection(wideAspectRatio));

        // Perspective divides by view-space depth (w varies), while the
        // orthographic projection keeps w constant.
        const auto orthoMatrix = camera.viewProjection(wideAspectRatio);
        camera.setProjection(ViewportProjection::Perspective);
        const auto perspectiveMatrix =
            camera.viewProjection(wideAspectRatio);
        require(perspectiveMatrix != orthoMatrix,
            "projection mode must change the matrix");

        // Probe two points at different depths along the view direction:
        // perspective divides by depth (w varies), orthographic does not
        // (w stays 1).
        const glm::dvec3 forwardDirection = glm::normalize(
            camera.focus() - camera.position()
        );
        const glm::dvec3 nearPoint = camera.position()
            + 0.5 * camera.distance() * forwardDirection;
        const glm::dvec3 farPoint = camera.position()
            + 1.5 * camera.distance() * forwardDirection;

        const auto nearClip = transformPoint(perspectiveMatrix, nearPoint);
        const auto farClip = transformPoint(perspectiveMatrix, farPoint);
        require(
            std::abs(nearClip[3] - farClip[3]) > 1.0e-3,
            "perspective w must vary with depth"
        );
        requireNear(transformPoint(orthoMatrix, nearPoint)[3], 1.0,
            "orthographic w must be constant");
        requireNear(transformPoint(orthoMatrix, farPoint)[3], 1.0,
            "orthographic w must stay constant");
        requireFiniteMatrix(orthoMatrix);
    }

    void testFocusSphereFraming()
    {
        ViewportCamera camera = makeFramedCamera();

        const glm::dvec3 center{12.0, -3.0, 4.0};
        constexpr double radius = 9.0;

        require(camera.frameSphere(center, radius, wideAspectRatio),
            "a valid sphere must frame successfully");
        requireVectorNear(camera.focus(), center, "sphere framing focus");

        const double verticalHalfAngle =
            0.5 * camera.verticalFieldOfView();
        const double horizontalHalfAngle = std::atan(
            std::tan(verticalHalfAngle) * wideAspectRatio
        );
        const double limitingHalfAngle = std::min(
            verticalHalfAngle,
            horizontalHalfAngle
        );
        require(
            camera.distance() >= 1.09 * radius
                / std::sin(limitingHalfAngle),
            "sphere framing distance must cover the sphere with margin"
        );

        // A tall viewport frames farther than a wide one for the same
        // sphere (the vertical half-angle limits first).
        ViewportCamera tall = makeFramedCamera(0.5);
        tall.frameSphere(center, radius, 0.5);
        require(
            tall.distance() > camera.distance(),
            "tall viewport frames the sphere farther away"
        );

        // Invalid selections are rejected without moving the camera.
        const double distanceBefore = camera.distance();
        const glm::dvec3 focusBefore = camera.focus();
        require(!camera.frameSphere(center, 0.0, wideAspectRatio),
            "zero radius must be rejected");
        require(!camera.frameSphere(center, -2.0, wideAspectRatio),
            "negative radius must be rejected");
        require(!camera.frameSphere(
            glm::dvec3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
            radius, wideAspectRatio),
            "non-finite centers must be rejected");
        requireVectorNear(camera.focus(), focusBefore,
            "rejected framing keeps focus");
        requireNear(camera.distance(), distanceBefore,
            "rejected framing keeps distance");
    }

    void testPoseApplication()
    {
        using quantum::editor::ViewportCameraPose;

        ViewportCamera camera = makeFramedCamera();
        const ViewportCameraPose pose{
            glm::dvec3{5.0, 6.0, 7.0},
            10.0 * 3.14159265358979323846 + 0.25,
            -0.15,
            33.0
        };
        camera.setPose(pose);

        requireVectorNear(camera.focus(), pose.focus, "pose focus");
        requireNear(camera.yaw(), 0.25, "pose yaw wraps into range");
        requireNear(camera.pitch(), pose.pitch, "pose pitch");
        requireNear(camera.distance(), pose.distance, "pose distance");

        // Out-of-limits distances clamp to the bounds-derived range.
        ViewportCameraPose extreme = pose;
        extreme.distance = 1.0e18;
        camera.setPose(extreme);
        require(
            camera.distance() < 1.0e18,
            "pose distance clamps to the usable maximum"
        );

        bool threwInvalidDistance = false;
        try
        {
            camera.setPose(ViewportCameraPose{pose.focus, 0.0, 0.0, 0.0});
        }
        catch (const std::invalid_argument&)
        {
            threwInvalidDistance = true;
        }
        require(threwInvalidDistance,
            "non-positive distances must be rejected");

        bool threwNonFinite = false;
        try
        {
            camera.setPose(ViewportCameraPose{
                glm::dvec3{std::numeric_limits<double>::infinity(),
                    0.0, 0.0}, 0.0, 0.0, 1.0});
        }
        catch (const std::invalid_argument&)
        {
            threwNonFinite = true;
        }
        require(threwNonFinite,
            "non-finite poses must be rejected");
    }

    void testConfigurableInteractionResponse()
    {
        using quantum::editor::defaultViewportZoomExponentPerWheelUnit;

        requireNear(defaultViewportZoomExponentPerWheelUnit, 0.18,
            "default zoom exponent");

        ViewportCamera slow = makeFramedCamera();
        ViewportCamera fast = makeFramedCamera();
        const double initialSlow = slow.distance();
        const double initialFast = fast.distance();

        slow.zoom(-1.0, 0.05);
        fast.zoom(-1.0, 0.40);

        requireNear(
            slow.distance(),
            initialSlow * std::exp(0.05),
            "weak exponent dollies out slowly per wheel unit"
        );
        requireNear(
            fast.distance(),
            initialFast * std::exp(0.40),
            "strong exponent dollies out faster per wheel unit"
        );

        // Field-of-view changes affect framing: a wider FOV frames the
        // same bounds from closer.
        ViewportCamera narrow = makeFramedCamera();
        const double narrowDistance = narrow.distance();
        narrow.setVerticalFieldOfView(100.0 * 3.14159265358979323846 / 180.0);
        narrow.frame(wideAspectRatio);
        require(
            narrow.distance() < narrowDistance,
            "wider field of view frames closer"
        );

        bool threwNonFiniteFov = false;
        try
        {
            narrow.setVerticalFieldOfView(
                std::numeric_limits<double>::quiet_NaN());
        }
        catch (const std::invalid_argument&)
        {
            threwNonFiniteFov = true;
        }
        require(threwNonFiniteFov, "non-finite FOV must be rejected");
    }

    void testWorldSpaceViewportRays()
    {
        using quantum::editor::ViewportCameraPose;
        using quantum::editor::ViewportProjection;

        ViewportCamera camera;
        camera.setBounds({-5.0, -5.0, -5.0}, {5.0, 5.0, 5.0});
        camera.setPose(ViewportCameraPose{
            .focus = {0.0, 0.0, 0.0},
            .yaw = 0.0,
            .pitch = 0.0,
            .distance = 10.0
        });

        const auto centerPerspective = camera.viewportRay(0.5, 0.5, 1.0);
        requireVectorNear(centerPerspective.origin, {10.0, 0.0, 0.0},
            "perspective center ray origin");
        requireVectorNear(centerPerspective.direction, {-1.0, 0.0, 0.0},
            "perspective center ray direction");

        const auto upperRight = camera.viewportRay(1.0, 0.0, 1.0);
        require(upperRight.direction.y > 0.0,
            "right viewport edge must point toward camera-right");
        require(upperRight.direction.z > 0.0,
            "top viewport edge must point toward camera-up");
        requireNear(glm::length(upperRight.direction), 1.0,
            "perspective ray direction normalization");

        camera.setProjection(ViewportProjection::Orthographic);
        const auto centerOrthographic = camera.viewportRay(0.5, 0.5, 1.0);
        const auto cornerOrthographic = camera.viewportRay(1.0, 0.0, 1.0);
        requireVectorNear(centerOrthographic.direction,
            cornerOrthographic.direction,
            "orthographic rays are parallel");
        require(glm::length(centerOrthographic.origin
                - cornerOrthographic.origin) > 1.0,
            "orthographic viewport position changes ray origin");

        bool threwOutsideViewport = false;
        try
        {
            static_cast<void>(camera.viewportRay(-0.01, 0.5, 1.0));
        }
        catch (const std::invalid_argument&)
        {
            threwOutsideViewport = true;
        }
        require(threwOutsideViewport,
            "ray generation must reject coordinates outside the viewport");
    }
}

int main()
{
    try
    {
        testBoundsFraming();
        testOrbitPreservesDistance();
        testPanPreservesCameraOffset();
        testZoomRemainsPositiveAndFinite();
        testFrameRecoversTransformedState();
        testUpdatedBoundsPreserveCameraUntilExplicitFrame();
        testMatrixGenerationIsDeterministic();
        testElongatedTrackBoundsUseProjectedExtent();
        testDeterministicPresets();
        testPoleSafeMatrices();
        testProjectionSwitching();
        testFocusSphereFraming();
        testPoseApplication();
        testConfigurableInteractionResponse();
        testWorldSpaceViewportRays();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Viewport camera test failure: " << exception.what()
                  << '\n';
        return 1;
    }

    std::cout << "Viewport camera tests passed.\n";
    return 0;
}
