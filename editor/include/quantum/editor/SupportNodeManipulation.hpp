#pragma once

#include <quantum/editor/SupportVisualization.hpp>
#include <quantum/editor/ViewportCamera.hpp>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <cstdint>
#include <optional>

namespace quantum::editor
{
    inline constexpr double supportNodeSnapRadiusPixels = 14.0;
    inline constexpr double supportGroundSnapThresholdPixels = 10.0;

    enum class SupportMoveAxis
    {
        X,
        Y,
        Z
    };

    [[nodiscard]] glm::dvec3 supportMoveWorldAxis(
        SupportMoveAxis axis) noexcept;

    [[nodiscard]] glm::dvec3 translateSupportNode(
        const glm::dvec3& initialPosition,
        SupportMoveAxis axis,
        double distance);

    // Connect Nodes owns node clicks while awaiting its second endpoint.
    // Stable selections are re-resolved against the current visualization.
    [[nodiscard]] bool supportNodeManipulationAvailable(
        const SupportVisualization& visualization,
        const std::optional<SupportSelection>& selection,
        bool connectWaiting) noexcept;

    struct SupportNodeSnapTarget
    {
        SupportSelection selection;
        glm::dvec3 position{0.0};
        double distancePixels = 0.0;
        double depth = 0.0;
    };

    // Chooses an exact authored node position by projected screen distance,
    // depth, structure ID, then element ID. The dragged node and targets that
    // would collapse one of its incident members are excluded.
    [[nodiscard]] std::optional<SupportNodeSnapTarget>
    pickSupportNodeSnapTarget(
        const SupportVisualization& visualization,
        const ViewportCamera& camera,
        const glm::dvec2& normalizedPointer,
        std::uint32_t viewportPixelWidth,
        std::uint32_t viewportPixelHeight,
        const SupportSelection& draggedNode,
        double radiusPixels = supportNodeSnapRadiusPixels);

    enum class SupportPositionSnapKind
    {
        None,
        Ground,
        Node
    };

    struct SupportPositionSnapResult
    {
        glm::dvec3 position{0.0};
        SupportPositionSnapKind kind = SupportPositionSnapKind::None;
        std::optional<SupportSelection> nodeTarget;
    };

    // Node snapping has priority. Ground snapping is deliberate: it applies
    // only to Z-axis motion within the caller-provided world-space threshold.
    [[nodiscard]] SupportPositionSnapResult resolveSupportPositionSnap(
        const glm::dvec3& unsnappedPosition,
        SupportMoveAxis axis,
        const std::optional<SupportNodeSnapTarget>& nodeTarget,
        bool nodeSnapEnabled,
        bool groundSnapEnabled,
        double groundThresholdWorldUnits);
}
