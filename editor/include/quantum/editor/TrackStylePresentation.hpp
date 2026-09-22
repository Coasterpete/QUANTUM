#pragma once

#include <quantum/coaster/TrackStyle.hpp>

#include <cstdint>

namespace quantum::coaster
{
    class AuthoredTrack;
}

namespace quantum::editor
{
    enum class TrackStylePresentationProduct : std::uint8_t
    {
        None = 0,
        TrackMaterials = 1u << 0,
        HardwareMaterials = 1u << 1,
        HardwareInstances = 1u << 2,
        RenderableMesh = 1u << 3,
        EngineeringRails = 1u << 4,
        FullRegeneration = 1u << 5
    };

    struct TrackStylePresentationImpact
    {
        TrackStylePresentationProduct products =
            TrackStylePresentationProduct::None;

        [[nodiscard]] bool affects(
            TrackStylePresentationProduct product) const noexcept;
        [[nodiscard]] bool empty() const noexcept;
        [[nodiscard]] bool requiresFullRegeneration() const noexcept;
        [[nodiscard]] bool invalidatesCanonicalTrack() const noexcept;
        [[nodiscard]] bool invalidatesRiderLoads() const noexcept;
        [[nodiscard]] bool invalidatesSupports() const noexcept;
        [[nodiscard]] bool invalidatesSimulationPreview() const noexcept;
    };

    [[nodiscard]] TrackStylePresentationImpact combineTrackStyleImpacts(
        TrackStylePresentationImpact first,
        TrackStylePresentationImpact second) noexcept;

    // Sparse region edits and document configuration edits share this
    // classification. Unsupported generator/topology changes fall back to
    // FullRegeneration.
    [[nodiscard]] TrackStylePresentationImpact classifyRegionTrackStyleEdit(
        const coaster::TrackStylePreset& documentStyle,
        const coaster::RegionTrackStyleOverrides& before,
        const coaster::RegionTrackStyleOverrides& after);

    [[nodiscard]] TrackStylePresentationImpact
    classifyResolvedTrackStylePresentationChange(
        const coaster::TrackStylePreset& before,
        const coaster::TrackStylePreset& after) noexcept;

    // Classifies the effective appearance of every region after a document
    // configuration edit, including changes hidden by sparse overrides.
    [[nodiscard]] TrackStylePresentationImpact classifyDocumentTrackStyleEdit(
        const coaster::AuthoredTrack& before,
        const coaster::AuthoredTrack& after);
}
