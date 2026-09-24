#include <quantum/editor/TrackStylePresentation.hpp>

#include <quantum/coaster/AuthoredTrack.hpp>

#include <glm/vector_relational.hpp>

#include <algorithm>

namespace quantum::editor
{
    namespace
    {
        [[nodiscard]] constexpr std::uint8_t bits(
            const TrackStylePresentationProduct products) noexcept
        {
            return static_cast<std::uint8_t>(products);
        }

        void add(
            TrackStylePresentationImpact& impact,
            const TrackStylePresentationProduct product) noexcept
        {
            impact.products = static_cast<TrackStylePresentationProduct>(
                bits(impact.products) | bits(product));
        }

        [[nodiscard]] bool sameMaterial(
            const coaster::TrackMaterial& first,
            const coaster::TrackMaterial& second) noexcept
        {
            return glm::all(glm::equal(first.baseColor, second.baseColor))
                && first.metallic == second.metallic
                && first.roughness == second.roughness;
        }

        [[nodiscard]] bool sameOffset(
            const coaster::RailOffset& first,
            const coaster::RailOffset& second) noexcept
        {
            return first.lateral == second.lateral
                && first.vertical == second.vertical;
        }

        [[nodiscard]] bool sameHardwareStructure(
            const coaster::RepeatingHardwareStyle& first,
            const coaster::RepeatingHardwareStyle& second) noexcept
        {
            return first.asset.path == second.asset.path
                && first.asset.placeholder == second.asset.placeholder
                && first.startOffset == second.startOffset
                && first.localPosition == second.localPosition
                && first.localRotation == second.localRotation
                && first.localScale == second.localScale
                && first.frameFollow == second.frameFollow;
        }

        [[nodiscard]] bool sameOptionalMaterial(
            const std::optional<coaster::TrackMaterial>& first,
            const std::optional<coaster::TrackMaterial>& second) noexcept
        {
            return first.has_value() == second.has_value()
                && (!first.has_value() || sameMaterial(*first, *second));
        }
    }

    bool TrackStylePresentationImpact::affects(
        const TrackStylePresentationProduct product) const noexcept
    {
        return (bits(products) & bits(product)) != 0;
    }

    bool TrackStylePresentationImpact::empty() const noexcept
    {
        return products == TrackStylePresentationProduct::None;
    }

    bool TrackStylePresentationImpact::requiresFullRegeneration() const noexcept
    {
        return affects(TrackStylePresentationProduct::FullRegeneration);
    }

    bool TrackStylePresentationImpact::invalidatesCanonicalTrack() const noexcept
    {
        return requiresFullRegeneration();
    }

    bool TrackStylePresentationImpact::invalidatesRiderLoads() const noexcept
    {
        return requiresFullRegeneration();
    }

    bool TrackStylePresentationImpact::invalidatesSupports() const noexcept
    {
        return requiresFullRegeneration();
    }

    bool TrackStylePresentationImpact::invalidatesSimulationPreview()
        const noexcept
    {
        return requiresFullRegeneration();
    }

    TrackStylePresentationImpact combineTrackStyleImpacts(
        TrackStylePresentationImpact first,
        const TrackStylePresentationImpact second) noexcept
    {
        first.products = static_cast<TrackStylePresentationProduct>(
            bits(first.products) | bits(second.products));
        return first;
    }

    TrackStylePresentationImpact classifyRegionTrackStyleEdit(
        const coaster::TrackStylePreset& documentStyle,
        const coaster::RegionTrackStyleOverrides& beforeOverrides,
        const coaster::RegionTrackStyleOverrides& afterOverrides)
    {
        const coaster::TrackStylePreset before =
            coaster::resolveTrackStyle(documentStyle, beforeOverrides);
        const coaster::TrackStylePreset after =
            coaster::resolveTrackStyle(documentStyle, afterOverrides);
        return classifyResolvedTrackStylePresentationChange(before, after);
    }

    TrackStylePresentationImpact classifyResolvedTrackStylePresentationChange(
        const coaster::TrackStylePreset& before,
        const coaster::TrackStylePreset& after) noexcept
    {
        TrackStylePresentationImpact impact;

        // A different generator family has no presentation-only path yet.
        // Preset names are metadata; all other concrete inputs are classified
        // below so configuration selection can reuse the solved centerline.
        if (before.geometryFamily != after.geometryFamily
            || before.railCount != after.railCount
            || before.railOffsets.size() != after.railOffsets.size()
            || before.railOffsets.size() != 2)
        {
            add(impact, TrackStylePresentationProduct::FullRegeneration);
            return impact;
        }

        if (before.visible != after.visible)
        {
            add(impact, TrackStylePresentationProduct::RenderableMesh);
            add(impact, TrackStylePresentationProduct::HardwareInstances);
        }
        if (before.railsVisible != after.railsVisible
            || before.railRadius != after.railRadius
            || before.railRadialSegments != after.railRadialSegments)
        {
            add(impact, TrackStylePresentationProduct::RenderableMesh);
        }

        if (!std::equal(before.railOffsets.begin(), before.railOffsets.end(),
                after.railOffsets.begin(), sameOffset))
        {
            add(impact, TrackStylePresentationProduct::RenderableMesh);
            add(impact, TrackStylePresentationProduct::EngineeringRails);
        }

        if (before.spine.enabled != after.spine.enabled
            || before.spine.type != after.spine.type
            || !sameOffset(before.spine.offset, after.spine.offset)
            || before.spine.dimensions != after.spine.dimensions
            || before.spine.radialSegments != after.spine.radialSegments)
        {
            add(impact, TrackStylePresentationProduct::RenderableMesh);
        }
        if (!sameMaterial(before.railMaterial, after.railMaterial)
            || !sameMaterial(before.spine.material, after.spine.material))
        {
            add(impact, TrackStylePresentationProduct::TrackMaterials);
        }

        if (before.repeatingHardware.size() != after.repeatingHardware.size())
        {
            add(impact, TrackStylePresentationProduct::HardwareInstances);
            return impact;
        }

        for (std::size_t index = 0;
            index < before.repeatingHardware.size(); ++index)
        {
            const auto& beforeHardware = before.repeatingHardware[index];
            const auto& afterHardware = after.repeatingHardware[index];
            if (!sameHardwareStructure(beforeHardware, afterHardware))
            {
                add(impact, TrackStylePresentationProduct::HardwareInstances);
            }
            if (beforeHardware.enabled != afterHardware.enabled
                || beforeHardware.spacing != afterHardware.spacing)
            {
                add(impact, TrackStylePresentationProduct::HardwareInstances);
            }
            if (!sameOptionalMaterial(beforeHardware.materialOverride,
                    afterHardware.materialOverride))
            {
                add(impact, TrackStylePresentationProduct::HardwareMaterials);
            }
        }

        return impact;
    }

    TrackStylePresentationImpact classifyDocumentTrackStyleEdit(
        const coaster::AuthoredTrack& before,
        const coaster::AuthoredTrack& after)
    {
        if (before.sectionCount() != after.sectionCount())
        {
            return {TrackStylePresentationProduct::FullRegeneration};
        }

        TrackStylePresentationImpact impact;
        for (std::size_t index = 0; index < before.sectionCount(); ++index)
        {
            const coaster::TrackStylePreset beforeStyle =
                coaster::resolveTrackStyle(
                    before.trackStyle(),
                    before.section(index).trackStyleOverrides);
            const coaster::TrackStylePreset afterStyle =
                coaster::resolveTrackStyle(
                    after.trackStyle(),
                    after.section(index).trackStyleOverrides);
            impact = combineTrackStyleImpacts(impact,
                classifyResolvedTrackStylePresentationChange(
                    beforeStyle, afterStyle));
        }
        return impact;
    }
}
