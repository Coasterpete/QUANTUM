#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <glm/vec3.hpp>

#include <cstdint>
#include <vector>

namespace quantum::editor
{
    // One authored section's footprint inside the generated reference
    // curves. Vertex ranges address one curve run (left rail, right rail,
    // centerline, heartline all repeat the same segment topology); bounds
    // cover that section's solved centerline samples only.
    struct CenterlineSectionSlice
    {
        std::uint32_t firstVertex = 0;
        std::uint32_t vertexCount = 0;
        glm::dvec3 startPosition{0.0};
        glm::dvec3 startTangent{1.0, 0.0, 0.0};
        glm::dvec3 minimumPosition{0.0};
        glm::dvec3 maximumPosition{0.0};
    };

    struct CenterlineVisualization
    {
        // Concatenated line-list segments for the solved track's spatial
        // reference curves: left rail, right rail, centerline, heartline.
        std::vector<renderer::LineVertex> vertices;

        // Vertex count of each of the four equal-length reference-curve
        // runs concatenated inside `vertices`, in the order above.
        std::uint32_t verticesPerCurve = 0;

        // One slice per authored section index.
        std::vector<CenterlineSectionSlice> sectionSlices;

        // Camera-fit bounds of the solved centerline only; the other curves
        // offset from it by small track-style distances.
        glm::dvec3 minimumPosition{0.0};
        glm::dvec3 maximumPosition{0.0};
    };

    // Integrates the authored track document through QuantumCore into one
    // continuous whole-track solve and derives its viewport reference curves
    // from the per-sample solved frames.
    [[nodiscard]] CenterlineVisualization createCenterlineVisualization(
        const coaster::AuthoredTrack& track
    );
}
