#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/TrackStyle.hpp>
#include <quantum/editor/TrackStylePresentation.hpp>
#include <quantum/editor/ViewportTrackAnchors.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <glm/vec3.hpp>

#include <cstdint>
#include <optional>
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

        // Canonical per-region results consumed to build renderableTrack.
        // Retained for editor inspection/tests; regions never own these full
        // copies in the authored document.
        std::vector<coaster::TrackStylePreset> resolvedRegionStyles;

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

    // Staged presentation products built from CenterlineVisualization::samples.
    // The cache applies these only after the required renderer uploads succeed.
    struct TrackStylePresentationCandidate
    {
        TrackStylePresentationImpact impact;
        coaster::TrackStylePreset documentStyle;
        std::vector<coaster::TrackStylePreset> resolvedRegionStyles;
        std::optional<coaster::ContinuousTrackMesh> continuousMesh;
        std::optional<std::vector<coaster::TrackMaterial>> trackMaterials;
        std::optional<std::vector<coaster::HardwareInstanceBatch>>
            hardwareBatches;
        std::optional<std::vector<std::optional<coaster::TrackMaterial>>>
            hardwareMaterials;
        std::optional<std::vector<renderer::LineVertex>> referenceCurveVertices;
    };

    // Distance-domain spacing used for viewport visualization samples.
    inline constexpr double centerlineVisualizationSampleSpacing = 0.75;

    // Integrates the authored track document through QuantumCore into one
    // continuous whole-track solve and derives its viewport reference curves
    // from the per-sample solved frames.
    [[nodiscard]] CenterlineVisualization createCenterlineVisualization(
        const coaster::AuthoredTrack& track
    );

    [[nodiscard]] CenterlineVisualization createCenterlineVisualization(
        const coaster::AuthoredTrack& track,
        const coaster::TrackStylePreset& style
    );

    [[nodiscard]] TrackStylePresentationCandidate
    createTrackStylePresentationCandidate(
        const coaster::AuthoredTrack& track,
        const CenterlineVisualization& cachedVisualization,
        TrackStylePresentationImpact impact);

    // Display-only bounds for Frame All/Focus. Keep the solved centerline
    // bounds above unchanged; rails/heartline matter when framing short tracks.
    [[nodiscard]] std::pair<glm::dvec3, glm::dvec3> referenceCurveBounds(
        const CenterlineVisualization& visualization,
        const CenterlineSectionSlice* slice = nullptr);

    // Tiny editor-side dirty/version wrapper for the generated visualization.
    // The caller marks geometry, track-presentation geometry, or heartline
    // reference edits dirty; pure selection changes do not touch this state.
    class CenterlineVisualizationCache
    {
    public:
        void markDirty() noexcept;
        // The caller invokes this only for an accepted style/preset edit;
        // presentation-mode changes deliberately do not touch this state.
        void setTrackStyle(coaster::TrackStylePreset style);
        [[nodiscard]] bool rebuildIfDirty(const coaster::AuthoredTrack& track);
        void replace(CenterlineVisualization visualization);
        void applyTrackStylePresentation(
            TrackStylePresentationCandidate candidate);

        [[nodiscard]] bool isDirty() const noexcept;
        [[nodiscard]] std::uint64_t generation() const noexcept;
        [[nodiscard]] std::uint64_t presentationGeneration() const noexcept;
        [[nodiscard]] const CenterlineVisualization& visualization()
            const noexcept;
        [[nodiscard]] const coaster::TrackStylePreset& trackStyle()
            const noexcept;

    private:
        CenterlineVisualization visualization_;
        coaster::TrackStylePreset trackStyle_ =
            coaster::createModernSteelPreset();
        std::uint64_t generation_ = 0;
        std::uint64_t presentationGeneration_ = 0;
        bool dirty_ = true;
    };
}
