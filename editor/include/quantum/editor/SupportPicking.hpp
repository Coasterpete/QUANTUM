#pragma once

#include <quantum/editor/SupportVisualization.hpp>
#include <quantum/editor/ViewportCamera.hpp>

#include <glm/vec2.hpp>

#include <cstdint>
#include <optional>

namespace quantum::editor
{
    inline constexpr double supportNodeHitRadiusPixels = 10.0;
    inline constexpr double supportMemberHitTolerancePixels = 8.0;

    struct SupportPickResult
    {
        SupportSelection selection;
        double distancePixels = 0.0;
        double depth = 0.0;
    };

    // Nodes have categorical priority over members. Within each category,
    // screen distance wins, then depth, structure ID, and element ID.
    [[nodiscard]] std::optional<SupportPickResult> pickSupport(
        const SupportVisualization& visualization,
        const ViewportCamera& camera,
        const glm::dvec2& normalizedPointer,
        std::uint32_t viewportPixelWidth,
        std::uint32_t viewportPixelHeight,
        double nodeRadiusPixels = supportNodeHitRadiusPixels,
        double memberTolerancePixels = supportMemberHitTolerancePixels);
}
