#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/editor/ViewportCamera.hpp>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace quantum::editor
{
    enum class StartPoseTransformMode
    {
        Move,
        Rotate
    };

    enum class StartPoseTransformAxis
    {
        X,
        Y,
        Z
    };

    [[nodiscard]] glm::dvec3 startPoseWorldAxis(
        StartPoseTransformAxis axis
    ) noexcept;

    // Pure candidate helpers used by the viewport gizmo and model tests.
    // Rotation is pre-multiplied, so V1 rotates about world X/Y/Z while the
    // stored quaternion remains the robust authored representation.
    [[nodiscard]] coaster::AuthoredStartPose translateStartPose(
        const coaster::AuthoredStartPose& pose,
        StartPoseTransformAxis axis,
        double distance
    );

    [[nodiscard]] coaster::AuthoredStartPose rotateStartPose(
        const coaster::AuthoredStartPose& pose,
        StartPoseTransformAxis axis,
        double angleRadians
    );

    enum class ViewportTrackAnchorKind
    {
        Start,
        Interior,
        End
    };

    [[nodiscard]] bool isViewportTrackAnchorEditable(
        ViewportTrackAnchorKind kind
    ) noexcept;

    // One semantic authored-track boundary. Interior anchors are shared by
    // the regions on both sides; they are not duplicated endpoint objects.
    struct ViewportTrackAnchor
    {
        std::size_t anchorIndex = 0;
        double distance = 0.0;
        glm::dvec3 position{0.0};
        glm::dvec3 forward{1.0, 0.0, 0.0};
        glm::dvec3 lateral{0.0, 1.0, 0.0};
        glm::dvec3 up{0.0, 0.0, 1.0};
        std::optional<std::size_t> previousRegionIndex;
        std::optional<std::size_t> nextRegionIndex;
        ViewportTrackAnchorKind kind = ViewportTrackAnchorKind::Start;
    };

    // The anchor highlight is a projection of the Editor's authoritative
    // region selection, not a second independent selection system.
    struct ViewportTrackSelection
    {
        std::size_t regionIndex = 0;
        std::optional<std::size_t> anchorIndex;
    };

    [[nodiscard]] ViewportTrackSelection selectionForViewportTrackRegion(
        std::size_t regionIndex,
        std::size_t regionCount
    );

    // Right-continuous boundary convention: anchor zero selects region zero,
    // an interior anchor selects the following region, and the final anchor
    // selects the final region.
    [[nodiscard]] ViewportTrackSelection selectionForViewportTrackAnchor(
        std::size_t anchorIndex,
        std::size_t regionCount
    );

    // Extracts only exact authored-region boundary poses from the Core
    // kinematic integration path. It does not consume viewport line vertices
    // or the visualization sample grid.
    [[nodiscard]] std::vector<ViewportTrackAnchor>
    createViewportTrackAnchors(const coaster::AuthoredTrack& track);

    struct ViewportProjectedPoint
    {
        glm::dvec2 normalizedPosition{0.0};
        double depth = 0.0;
    };

    // Projects a world point into the same top-left-origin normalized image
    // coordinates used by viewportRay(). Points behind the eye or outside
    // the active depth range are not projectable.
    [[nodiscard]] std::optional<ViewportProjectedPoint>
    projectViewportPoint(
        const ViewportCamera& camera,
        const glm::dvec3& worldPosition,
        double aspectRatio
    );

    inline constexpr double viewportTrackAnchorHitRadiusPixels = 11.0;

    struct ViewportTrackAnchorPickResult
    {
        std::size_t anchorIndex = 0;
        double distancePixels = 0.0;
        double depth = 0.0;
    };

    // Chooses the closest projected marker to the pointer. Equal screen
    // distances prefer the frontmost anchor, then the lower semantic index,
    // so overlapping markers are deterministic regardless of input order.
    [[nodiscard]] std::optional<ViewportTrackAnchorPickResult>
    pickViewportTrackAnchor(
        std::span<const ViewportTrackAnchor> anchors,
        const ViewportCamera& camera,
        const glm::dvec2& normalizedPointer,
        std::uint32_t viewportPixelWidth,
        std::uint32_t viewportPixelHeight,
        double hitRadiusPixels = viewportTrackAnchorHitRadiusPixels
    );
}
