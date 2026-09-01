#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/TrackStyle.hpp>
#include <quantum/editor/ViewportTrackAnchors.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <glm/vec3.hpp>

#include <cstdint>
#include <utility>
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
        double startDistance = 0.0;
        double endDistance = 0.0;
        glm::dvec3 startPosition{0.0};
        glm::dvec3 endPosition{0.0};
        glm::dvec3 startTangent{1.0, 0.0, 0.0};
        glm::dvec3 minimumPosition{0.0};
        glm::dvec3 maximumPosition{0.0};
    };

    struct CenterlineVisualization
    {
        // Authoritative solved samples used to build the renderer vertices.
        // These are retained on the editor side for section metadata, tests,
        // and camera tools; renderer upload still receives only line vertices.
        std::vector<coaster::RiderLocalGeometryState> samples;

        // Concatenated line-list segments for the solved track's spatial
        // reference curves: left rail, right rail, centerline, heartline.
        std::vector<renderer::LineVertex> vertices;

        // Renderer-neutral indexed rail mesh plus contiguous repeating-
        // hardware instance batches generated from the same solved samples.
        coaster::RenderableTrack renderableTrack;

        // Vertex count of each of the four equal-length reference-curve
        // runs concatenated inside `vertices`, in the order above.
        std::uint32_t verticesPerCurve = 0;

        // One slice per authored section index.
        std::vector<CenterlineSectionSlice> sectionSlices;

        // Exact semantic region-boundary poses. These are extracted from the
        // authored kinematic path independently of the sampled curve runs.
        std::vector<ViewportTrackAnchor> anchors;

        // Camera-fit bounds of the solved centerline only; the other curves
        // offset from it by small track-style distances.
        glm::dvec3 minimumPosition{0.0};
        glm::dvec3 maximumPosition{0.0};
    };

    // Distance-domain spacing used for viewport visualization samples.
    inline constexpr double centerlineVisualizationSampleSpacing = 0.75;

    // Integrates the authored track document through QuantumCore into one
    // continuous whole-track solve and derives its viewport reference curves
    // from the per-sample solved frames.
    [[nodiscard]] CenterlineVisualization createCenterlineVisualization(
        const coaster::AuthoredTrack& track,
        const coaster::TrackStylePreset& style =
            coaster::createStandardDualRailPreset()
    );

    // Display-only bounds for Frame All/Focus. Keep the solved centerline
    // bounds above unchanged; rails/heartline matter when framing short tracks.
    [[nodiscard]] std::pair<glm::dvec3, glm::dvec3> referenceCurveBounds(
        const CenterlineVisualization& visualization,
        const CenterlineSectionSlice* slice = nullptr);

    // Tiny editor-side dirty/version wrapper for the generated visualization.
    // The caller remains responsible for marking geometry edits dirty; pure
    // selection changes intentionally do not touch this state.
    class CenterlineVisualizationCache
    {
    public:
        void markDirty() noexcept;
        // The caller invokes this only for an accepted style/preset edit;
        // presentation-mode changes deliberately do not touch this state.
        void setTrackStyle(coaster::TrackStylePreset style);
        [[nodiscard]] bool rebuildIfDirty(const coaster::AuthoredTrack& track);
        void replace(CenterlineVisualization visualization);

        [[nodiscard]] bool isDirty() const noexcept;
        [[nodiscard]] std::uint64_t generation() const noexcept;
        [[nodiscard]] const CenterlineVisualization& visualization()
            const noexcept;
        [[nodiscard]] const coaster::TrackStylePreset& trackStyle()
            const noexcept;

    private:
        CenterlineVisualization visualization_;
        coaster::TrackStylePreset trackStyle_ =
            coaster::createStandardDualRailPreset();
        std::uint64_t generation_ = 0;
        bool dirty_ = true;
    };
}
