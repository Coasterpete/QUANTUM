#pragma once

#include <quantum/coaster/Supports.hpp>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include <cstdint>
#include <vector>

namespace quantum::coaster
{
    // Which immutable unit mesh a batch draws. Both meshes are uploaded once
    // and shared, so a document never allocates a mesh per timber.
    enum class SupportMemberMeshKind : std::uint8_t
    {
        // Unit cube spanning [-0.5, 0.5] on each axis, scaled per instance.
        Rectangular,
        // Unit-diameter unit-length cylinder spanning x = [-0.5, 0.5] with
        // radius 0.5 in the y/z plane, scaled per instance.
        Circular
    };

    // One row of the shared support instance stream. The transform already
    // carries the member's length, cross-section, and orientation, so the
    // renderer needs no per-member CPU mesh. Texture coordinates are derived
    // in the vertex stage from this transform, which is what keeps grain
    // physically sized on long posts and short braces alike.
    struct SupportMemberInstance
    {
        glm::mat4 transform{1.0F};

        [[nodiscard]] friend bool operator==(
            const SupportMemberInstance&,
            const SupportMemberInstance&) = default;
    };

    // Solid members for one support structure, grouped by profile shape so
    // each shape becomes one instanced draw call.
    struct SupportSolidBatch
    {
        SupportMemberMeshKind mesh = SupportMemberMeshKind::Rectangular;
        std::vector<SupportMemberInstance> instances;

        [[nodiscard]] std::size_t memberCount() const noexcept
        {
            return instances.size();
        }
    };

    // A foundation pad derived from one SupportNode marked with a Foundation.
    // The node position stays authoritative; padDimensions and padDepth are
    // the resolved footprint and thickness in Core coordinate units. The pad
    // is centred on the node and extends half its depth to either side.
    struct SupportFoundationPad
    {
        glm::dvec3 position{0.0};
        glm::dvec2 padDimensions{0.0, 0.0};
        double padDepth = 0.0;

        [[nodiscard]] friend bool operator==(
            const SupportFoundationPad&,
            const SupportFoundationPad&) = default;
    };

    // The complete solid presentation of one support structure.
    struct SupportSolidPresentation
    {
        SupportStructureId structureId = invalidSupportStructureId;
        // Resolved, never absent: a structure that never authored an
        // appearance resolves to the conservative default timber.
        SupportAppearance appearance;
        SupportFoundationAppearance foundationAppearance;
        // Rectangular members first, then circular members.
        std::vector<SupportSolidBatch> batches;
        // One pad per foundation node, in node order.
        std::vector<SupportFoundationPad> foundations;

        [[nodiscard]] std::size_t memberCount() const noexcept;
        // One instanced draw per non-empty batch.
        [[nodiscard]] std::size_t drawCallCount() const noexcept;
        [[nodiscard]] bool empty() const noexcept;
    };

    // Members at or below this length in Core coordinate units are degenerate
    // and are skipped rather than rendered. It matches the M1 generator's own
    // degenerate-endpoint guard, so the renderer never disagrees with the
    // topology it was handed.
    inline constexpr double minimumSupportMemberLength = 1e-6;

    // One pad width/depth multiple applied to the largest cross-section
    // dimension of the members meeting a foundation node. The default keeps
    // the pad visibly wider than the post it carries without claiming to size
    // a footing from structural loads.
    inline constexpr double supportFoundationPadFootprintScale = 3.0;
    // Thickness is derived from the cross-section itself rather than from the
    // pad footprint, so a footing always reads as a flat pad rather than a
    // square pillar stub as thick as it is wide.
    inline constexpr double supportFoundationPadDepthScale = 1.5;

    // The orientation rule on its own, exposed so tests and future authoring
    // tools agree with the renderer instead of duplicating it.
    struct SupportMemberFrame
    {
        glm::dvec3 origin{0.0};
        // Unit, start to end.
        glm::dvec3 axisX{1.0, 0.0, 0.0};
        // Unit, perpendicular to axisX.
        glm::dvec3 axisY{0.0, 1.0, 0.0};
        // Unit, completes the right-handed basis.
        glm::dvec3 axisZ{0.0, 0.0, 1.0};
        double length = 0.0;
        // True when axisY fell back to world +X because the member is
        // vertical.
        bool usedVerticalFallback = false;
    };

    // Members whose axis is closer to world +Z than this cosine switch to
    // the +X fallback, which keeps the perpendicular reference well
    // conditioned instead of collapsing.
    inline constexpr double supportMemberVerticalTolerance = 0.9995;

    // Local +X is the start-to-end direction. Local +Y is world +Z projected
    // off that axis, so a member's cross-section keeps a stable "up" and
    // neighbouring members in a bent agree on orientation. A vertical member
    // instead uses world +X, because world +Z is parallel to its own axis.
    // Local +Z completes a right-handed basis.
    //
    // Throws std::invalid_argument for a non-finite endpoint. A degenerate
    // pair returns length 0 with the default identity axes; callers must skip
    // such a frame.
    [[nodiscard]] SupportMemberFrame resolveSupportMemberFrame(
        const glm::dvec3& start,
        const glm::dvec3& end);

    // Builds the solid presentation for one already-resolved structure. Node
    // positions must already have active track attachments resolved; this
    // function never re-solves the track.
    //
    // Circular members reuse the same longitudinal axis, so only the
    // cross-section shape differs. Wall thickness is a M2B hollow-section
    // concern and is not applied here.
    //
    // Throws std::invalid_argument for a non-finite resolved node position or
    // a malformed member profile or appearance.
    [[nodiscard]] SupportSolidPresentation buildSupportSolidPresentation(
        const SupportStructure& structure);

    // Builds every structure's presentation in collection order. Structures
    // with no renderable members and no foundations are still returned, so a
    // caller can key results by SupportStructureId.
    [[nodiscard]] std::vector<SupportSolidPresentation>
    buildSupportSolidPresentation(const SupportCollection& collection);
}
