#pragma once
#include <quantum/editor/ViewportTrackAnchors.hpp>
#include <array>

namespace quantum::editor
{
    struct ViewportMoveHandle
    {
        glm::dvec2 begin, end, direction;
        double coordinateUnitsPerPixel;
    };
    struct ViewportTransformHandles
    {
        glm::dvec2 origin;
        std::array<ViewportMoveHandle, 3> move;
        std::array<double, 3> rotateRadii;
        double hitRadius;
    };
    // Screen-space handles use the fallback's projected world axes and
    // concentric world-axis rotation controls, including edge-on fallbacks.
    [[nodiscard]] std::optional<ViewportTransformHandles> viewportTransformHandles(
        const ViewportCamera& camera, const glm::dvec3& position,
        std::uint32_t width, std::uint32_t height, double pixelScale = 1);
    [[nodiscard]] std::optional<StartPoseTransformAxis> pickViewportTransformHandle(
        const ViewportTransformHandles& handles, StartPoseTransformMode mode,
        const glm::dvec2& pointer);
}
