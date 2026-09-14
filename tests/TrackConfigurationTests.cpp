#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/TrackConfiguration.hpp>

#include <nlohmann/json.hpp>

#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
    using namespace quantum::coaster;

    void require(const bool condition, const char* const message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    void requireSameMaterial(
        const TrackMaterial& actual,
        const TrackMaterial& expected,
        const char* const message)
    {
        require(actual.baseColor == expected.baseColor, message);
    }

    void requireSameTrackStyle(
        const TrackStylePreset& actual,
        const TrackStylePreset& expected)
    {
        require(actual.name == expected.name, "preset name differs");
        require(actual.geometryFamily == expected.geometryFamily,
            "geometry family differs");
        require(actual.visible == expected.visible, "visibility differs");
        require(actual.railsVisible == expected.railsVisible,
            "rail visibility differs");
        require(actual.railCount == expected.railCount, "rail count differs");
        require(actual.railOffsets.size() == expected.railOffsets.size(),
            "rail offset count differs");
        for (std::size_t index = 0; index < actual.railOffsets.size(); ++index)
        {
            require(actual.railOffsets[index].lateral
                        == expected.railOffsets[index].lateral
                    && actual.railOffsets[index].vertical
                        == expected.railOffsets[index].vertical,
                "rail offset differs");
        }
        require(actual.railRadius == expected.railRadius,
            "rail radius differs");
        require(actual.railRadialSegments == expected.railRadialSegments,
            "rail tessellation differs");
        requireSameMaterial(actual.railMaterial, expected.railMaterial,
            "rail material differs");

        require(actual.spine.enabled == expected.spine.enabled,
            "spine enabled state differs");
        require(actual.spine.type == expected.spine.type,
            "spine type differs");
        require(actual.spine.offset.lateral == expected.spine.offset.lateral
                && actual.spine.offset.vertical
                    == expected.spine.offset.vertical,
            "spine offset differs");
        require(actual.spine.dimensions == expected.spine.dimensions,
            "spine dimensions differ");
        require(actual.spine.radialSegments == expected.spine.radialSegments,
            "spine tessellation differs");
        requireSameMaterial(actual.spine.material, expected.spine.material,
            "spine material differs");

        require(actual.repeatingHardware.size()
                == expected.repeatingHardware.size(),
            "repeating hardware count differs");
        for (std::size_t index = 0;
            index < actual.repeatingHardware.size(); ++index)
        {
            const RepeatingHardwareStyle& actualHardware =
                actual.repeatingHardware[index];
            const RepeatingHardwareStyle& expectedHardware =
                expected.repeatingHardware[index];
            require(actualHardware.enabled == expectedHardware.enabled,
                "hardware enabled state differs");
            require(actualHardware.asset == expectedHardware.asset,
                "hardware asset differs");
            require(actualHardware.spacing == expectedHardware.spacing,
                "hardware spacing differs");
            require(actualHardware.startOffset == expectedHardware.startOffset,
                "hardware phase differs");
            require(actualHardware.localPosition
                        == expectedHardware.localPosition
                    && actualHardware.localRotation
                        == expectedHardware.localRotation
                    && actualHardware.localScale
                        == expectedHardware.localScale,
                "hardware transform differs");
            require(actualHardware.frameFollow == expectedHardware.frameFollow,
                "hardware frame behavior differs");
            require(actualHardware.materialOverride.has_value()
                    == expectedHardware.materialOverride.has_value(),
                "hardware material override state differs");
            if (actualHardware.materialOverride)
            {
                requireSameMaterial(*actualHardware.materialOverride,
                    *expectedHardware.materialOverride,
                    "hardware material override differs");
            }
        }
    }

    void builtInIdentityResolvesModernSteelExactly()
    {
        const TrackConfigurationDefinition* configuration =
            findTrackConfiguration(modernSteelTrackConfigurationId);
        require(configuration != nullptr,
            "built-in Track Configuration must be found by stable ID");
        require(configuration->id == "modern-steel",
            "built-in Track Configuration stable ID changed");
        require(configuration->displayName == "Modern Steel",
            "built-in Track Configuration display name changed");
        require(configuration->id != configuration->displayName,
            "display name must remain separate from stable ID");

        const TrackStylePreset resolved =
            resolveTrackConfiguration(*configuration);
        require(configuration->id != resolved.name,
            "stable ID must remain separate from TrackStylePreset.name");
        validateTrackStyle(resolved);
        requireSameTrackStyle(resolved, createModernSteelPreset());
    }

    void unknownIdentityDoesNotFallback()
    {
        require(findTrackConfiguration("unknown") == nullptr,
            "unknown stable ID must not resolve to Modern Steel");

        bool rejected = false;
        try
        {
            static_cast<void>(resolveTrackConfiguration(
                TrackConfigurationDefinition{"unknown", "Unknown"}));
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected,
            "unknown Track Configuration definition must be rejected");
    }

    void regionOverridesRemainASeparateResolutionLayer()
    {
        const TrackConfigurationDefinition* configuration =
            findTrackConfiguration(modernSteelTrackConfigurationId);
        require(configuration != nullptr, "Modern Steel definition missing");
        const TrackStylePreset base =
            resolveTrackConfiguration(*configuration);

        RegionTrackStyleOverrides overrides;
        overrides.enabled = true;
        overrides.visible = false;
        overrides.railCenterSpacing = 1.4;
        overrides.railVerticalOffset = 0.08;
        overrides.spineEnabled = false;
        overrides.hardwareSpacing = 1.25;
        overrides.railMaterial = TrackMaterial{
            glm::vec4{0.1F, 0.2F, 0.3F, 1.0F}};

        const TrackStylePreset resolved = resolveTrackStyle(base, overrides);
        const TrackStylePreset existingPath = resolveTrackStyle(
            createModernSteelPreset(), overrides);
        requireSameTrackStyle(resolved, existingPath);
        require(!resolved.visible
                && resolved.railOffsets[0].lateral == -0.7
                && resolved.railOffsets[1].lateral == 0.7
                && resolved.railOffsets[0].vertical == 0.08
                && resolved.railOffsets[1].vertical == 0.08
                && !resolved.spine.enabled
                && resolved.repeatingHardware.front().spacing == 1.25
                && resolved.railMaterial.baseColor
                    == glm::vec4{0.1F, 0.2F, 0.3F, 1.0F},
            "region overrides must retain existing sparse resolution behavior");
    }

    void newDocumentsUseResolvedStyleWithoutPersistingIdentity()
    {
        const AuthoredTrack track = createNewDocument();
        requireSameTrackStyle(track.trackStyle(), createModernSteelPreset());

        const nlohmann::json document = nlohmann::json::parse(
            serializeCoasterDocument(track));
        require(!document.contains("trackConfigurationId"),
            "Track Configuration identity must not enter the document schema");
        require(document.contains("trackStyle")
                && document["trackStyle"].is_object(),
            "documents must continue storing the concrete Track Style");
    }
}

int main()
{
    try
    {
        builtInIdentityResolvesModernSteelExactly();
        unknownIdentityDoesNotFallback();
        regionOverridesRemainASeparateResolutionLayer();
        newDocumentsUseResolvedStyleWithoutPersistingIdentity();
    }
    catch (const std::exception& error)
    {
        std::cerr << "Track Configuration test failure: "
                  << error.what() << '\n';
        return 1;
    }

    std::cout << "Track Configuration tests passed.\n";
    return 0;
}
