#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <glm/vec3.hpp>

#include <vector>

namespace quantum::editor
{
    struct CenterlineVisualization
    {
        // Concatenated line-list segments for the solved track's spatial
        // reference curves: left rail, right rail, centerline, heartline.
        std::vector<renderer::LineVertex> vertices;

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
