#pragma once

#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/editor/ViewportCamera.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>

namespace quantum::editor
{
    // Physical-pixel radius around the cursor accepted as a line hit. Eight
    // pixels keeps one-pixel engineering curves practical to select without
    // making nearby regions feel interchangeable.
    inline constexpr double viewportSelectionTolerancePixels = 8.0;

    struct ViewportPickResult
    {
        std::size_t sectionIndex = 0;
        std::uint32_t curveIndex = 0;
        std::uint32_t segmentFirstVertex = 0;
        double rayDistance = 0.0;
        double distanceToSegment = 0.0;
    };

    // Tests every visible reference-curve segment in the exact authored
    // section slices produced during visualization construction. Valid hits
    // are ordered front-to-back. Equal-depth hits prefer the closer segment,
    // then the lower section, curve, and vertex indices for deterministic
    // shared-boundary behavior.
    [[nodiscard]] std::optional<ViewportPickResult> pickViewportSection(
        const CenterlineVisualization& visualization,
        const ViewportCamera& camera,
        const ViewportRay& ray,
        std::uint32_t viewportPixelHeight,
        std::uint32_t visibleCurveMask,
        double tolerancePixels = viewportSelectionTolerancePixels
    );
}
