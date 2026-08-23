#include <quantum/editor/ViewportCamera.hpp>

#include <glm/geometric.hpp>

#include <array>
#include <cmath>
#include <exception>
#include <iostream>
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
        ViewportCamera expected = makeFramedCamera(0.75);
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
