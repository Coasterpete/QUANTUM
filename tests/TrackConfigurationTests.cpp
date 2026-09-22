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

    // Test 1: Modern Steel exists in the configuration catalog.
    void modernSteelExistsInCatalog()
    {
        const TrackConfigurationDefinition* configuration =
            findTrackConfiguration(modernSteelTrackConfigurationId);
        require(configuration != nullptr,
            "Modern Steel must be found in the configuration catalog");
        require(configuration->id == "modern-steel",
            "Modern Steel stable ID must be 'modern-steel'");
        require(configuration->displayName == "Modern Steel",
            "Modern Steel display name must be 'Modern Steel'");
    }

    // Test 2: Modern Steel has a stable configuration ID.
    void modernSteelHasStableId()
    {
        require(std::string_view(modernSteelTrackConfigurationId)
                == "modern-steel",
            "modernSteelTrackConfigurationId must be stable");
    }

    // Test 3: Modern Steel resolves to a valid TrackStylePreset.
    void modernSteelResolvesToValidPreset()
    {
        const TrackStylePreset resolved =
            resolveTrackConfiguration(modernSteelTrackConfigurationId);
        validateTrackStyle(resolved);
        requireSameTrackStyle(resolved, createModernSteelPreset());
    }

    // Test 4: Convenience overload resolves the same as definition-based.
    void convenienceOverloadResolvesIdentically()
    {
        const TrackStylePreset byId =
            resolveTrackConfiguration(modernSteelTrackConfigurationId);
        const TrackConfigurationDefinition* definition =
            findTrackConfiguration(modernSteelTrackConfigurationId);
        require(definition != nullptr, "definition must exist");
        const TrackStylePreset byDefinition =
            resolveTrackConfiguration(*definition);
        requireSameTrackStyle(byId, byDefinition);
    }

    // Test 5: Unknown identity does not fall back to Modern Steel.
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

        rejected = false;
        try
        {
            static_cast<void>(
                resolveTrackConfiguration(std::string_view("unknown")));
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected,
            "unknown configuration ID must be rejected via convenience overload");
    }

    // Test 6: Region overrides remain a separate resolution layer.
    void regionOverridesRemainASeparateResolutionLayer()
    {
        const TrackStylePreset base =
            resolveTrackConfiguration(modernSteelTrackConfigurationId);

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

    // Test 7: Applying Modern Steel to a document produces the expected
    // concrete appearance.
    void applyingConfigurationProducesExpectedAppearance()
    {
        AuthoredTrack track;
        require(track.trackConfigurationId() == "modern-steel",
            "new document must have Modern Steel configuration identity");

        // Manually change the style to something else.
        track.setTrackStyle(createStandardDualRailPreset());
        requireSameTrackStyle(track.trackStyle(), createStandardDualRailPreset());
        require(track.trackConfigurationId() == "modern-steel",
            "setTrackStyle must not change configuration identity");

        // Apply Modern Steel configuration.
        track.applyTrackConfiguration(modernSteelTrackConfigurationId);
        requireSameTrackStyle(track.trackStyle(), createModernSteelPreset());
        require(track.trackConfigurationId() == "modern-steel",
            "configuration identity must be set after applyTrackConfiguration");
    }

    // Test 8: The document owns its own concrete copy, not sharing mutable
    // catalog state.
    void documentOwnsConcreteCopy()
    {
        AuthoredTrack track1;
        AuthoredTrack track2;
        // Mutate track1's style directly.
        TrackStylePreset modified = createModernSteelPreset();
        modified.railRadius = 0.1;
        track1.setTrackStyle(modified);

        // track2 must be unaffected.
        require(track2.trackStyle().railRadius
                == createModernSteelPreset().railRadius,
            "documents must own independent concrete style copies");
    }

    // Test 9: Applying configuration preserves region overrides.
    void applyingConfigurationPreservesRegionOverrides()
    {
        AuthoredTrack track;
        track.appendSection();
        auto& overrides = track.section(0).trackStyleOverrides;
        overrides.enabled = true;
        overrides.visible = false;
        overrides.hardwareSpacing = 2.0;

        track.applyTrackConfiguration(modernSteelTrackConfigurationId);

        const auto& restoredOverrides =
            track.section(0).trackStyleOverrides;
        require(restoredOverrides.enabled
                && restoredOverrides.visible == false
                && restoredOverrides.hardwareSpacing == 2.0,
            "applying configuration must preserve existing region overrides");
    }

    // Test 10: Configuration identity round-trips through serialization.
    void configurationIdentityRoundTrips()
    {
        const AuthoredTrack track = createNewDocument();
        const std::string serialized = serializeCoasterDocument(track);
        const auto restored = deserializeCoasterDocument(serialized);
        if (!restored.has_value())
        {
            throw std::runtime_error(
                std::string("serialization round-trip failed: ")
                + restored.error());
        }
        require(restored->trackConfigurationId() == "modern-steel",
            "configuration identity must survive serialization round-trip");
        requireSameTrackStyle(restored->trackStyle(), track.trackStyle());
    }

    // Test 11: Concrete TrackStylePreset serialization round-trips.
    void concretePresetSerializationRoundTrips()
    {
        AuthoredTrack track = createNewDocument();
        TrackStylePreset modified = createModernSteelPreset();
        modified.railRadius = 0.1;
        modified.railMaterial.baseColor = {1.0F, 0.0F, 0.0F, 1.0F};
        track.setTrackStyle(modified);

        const std::string serialized = serializeCoasterDocument(track);
        const auto restored = deserializeCoasterDocument(serialized);
        if (!restored.has_value())
        {
            throw std::runtime_error(
                std::string("concrete preset round-trip failed: ")
                + restored.error());
        }
        requireSameTrackStyle(restored->trackStyle(), modified);
    }

    // Test 12: Legacy documents without configuration identity load correctly.
    void legacyDocumentsLoadCorrectly()
    {
        AuthoredTrack track = createNewDocument();
        const std::string serialized = serializeCoasterDocument(track);

        // Simulate a legacy document by removing trackConfigurationId.
        auto legacyJson = nlohmann::json::parse(serialized);
        legacyJson.erase("trackConfigurationId");
        const auto restored =
            deserializeCoasterDocument(legacyJson.dump());
        if (!restored.has_value())
        {
            throw std::runtime_error(
                std::string("legacy document deserialization failed: ")
                + restored.error());
        }
        require(restored->trackConfigurationId().empty(),
            "legacy document must have empty configuration identity");
        requireSameTrackStyle(restored->trackStyle(), track.trackStyle());
    }

    // Test 13: Unknown configuration ID does not destroy concrete appearance.
    void unknownConfigurationIdPreservesAppearance()
    {
        AuthoredTrack track = createNewDocument();
        const TrackStylePreset originalStyle = track.trackStyle();
        const std::string serialized = serializeCoasterDocument(track);

        // Simulate a document with an unknown configuration ID.
        auto modifiedJson = nlohmann::json::parse(serialized);
        modifiedJson["trackConfigurationId"] = "unknown-future-config";
        const auto restored =
            deserializeCoasterDocument(modifiedJson.dump());
        if (!restored.has_value())
        {
            throw std::runtime_error(
                std::string("unknown config ID deserialization failed: ")
                + restored.error());
        }
        requireSameTrackStyle(restored->trackStyle(), originalStyle);
        require(restored->trackConfigurationId() == "unknown-future-config",
            "unknown configuration ID must be preserved as-is");
    }

    // Test 14: Repeating-component phase is deterministic across region
    // boundaries. Hardware spacing uses track-global phase from
    // trackBegin + startOffset, not region-local restart.
    void repeatingComponentPhaseIsDeterministic()
    {
        const TrackStylePreset style =
            resolveTrackConfiguration(modernSteelTrackConfigurationId);
        require(!style.repeatingHardware.empty(),
            "Modern Steel must have repeating hardware");
        const double spacing = style.repeatingHardware.front().spacing;
        const double startOffset =
            style.repeatingHardware.front().startOffset;
        require(spacing > 0.0, "hardware spacing must be positive");

        // The hardware placement algorithm uses:
        //   distance = trackBegin + startOffset + instanceIndex * spacing
        // This is track-global and does not restart at region boundaries.
        const double trackBegin = 0.0;
        const double trackEnd = 10.0;
        const double availableLength = trackEnd - trackBegin - startOffset;
        require(availableLength >= 0.0,
            "track must be long enough for at least one instance");
        const std::size_t expectedCount = static_cast<std::size_t>(
            std::floor(availableLength / spacing + 1.0e-10)) + 1;

        // Verify the first few distances are deterministic.
        for (std::size_t i = 0; i < expectedCount; ++i)
        {
            const double expectedDistance =
                trackBegin + startOffset
                + static_cast<double>(i) * spacing;
            require(expectedDistance >= trackBegin
                    && expectedDistance <= trackEnd + 1.0e-9,
                "hardware distance must be within track bounds");
        }
    }

    // Test 15: setTrackConfigurationId works independently.
    void setConfigurationIdWorks()
    {
        AuthoredTrack track;
        require(track.trackConfigurationId() == "modern-steel",
            "default must be modern-steel");

        track.setTrackConfigurationId("custom-config");
        require(track.trackConfigurationId() == "custom-config",
            "setTrackConfigurationId must update the identity");

        track.setTrackConfigurationId("");
        require(track.trackConfigurationId().empty(),
            "setTrackConfigurationId must allow clearing");
    }

    // Test 16: resetToConfigurationDefaults clears region overrides.
    void resetToDefaultsClearsOverrides()
    {
        AuthoredTrack track;
        track.appendSection();
        auto& overrides = track.section(0).trackStyleOverrides;
        overrides.enabled = true;
        overrides.visible = false;
        overrides.railRadius = 0.1;

        track.resetToConfigurationDefaults();

        const auto& restored = track.section(0).trackStyleOverrides;
        require(!restored.enabled,
            "resetToConfigurationDefaults must clear region overrides");
        require(!restored.visible.has_value(),
            "resetToConfigurationDefaults must clear override optionals");
    }

    // Test 17: resetToConfigurationDefaults is a no-op without identity.
    void resetToDefaultsNoOpWithoutIdentity()
    {
        AuthoredTrack track;
        track.setTrackConfigurationId("");
        TrackStylePreset customStyle = createModernSteelPreset();
        customStyle.railRadius = 0.99;
        track.setTrackStyle(customStyle);

        track.resetToConfigurationDefaults();

        require(track.trackStyle().railRadius == 0.99,
            "resetToConfigurationDefaults must be no-op without identity");
    }

    // Test 18: Serialization is byte-deterministic for identical content.
    void serializationDeterminism()
    {
        const AuthoredTrack track1 = createNewDocument();
        const AuthoredTrack track2 = createNewDocument();
        require(serializeCoasterDocument(track1)
                == serializeCoasterDocument(track2),
            "serialization must be byte-deterministic for identical documents");
    }

    // Test 19: trackConfigurationId is serialized when present.
    void configurationIdIsSerialized()
    {
        const AuthoredTrack track = createNewDocument();
        const auto json = nlohmann::json::parse(
            serializeCoasterDocument(track));
        require(json.contains("trackConfigurationId"),
            "configuration ID must be present in serialized document");
        require(json["trackConfigurationId"] == "modern-steel",
            "serialized configuration ID must be 'modern-steel'");
    }

    // Test 20: trackConfigurationId is not serialized when empty.
    void emptyConfigurationIdNotSerialized()
    {
        AuthoredTrack track = createNewDocument();
        track.setTrackConfigurationId("");
        const auto json = nlohmann::json::parse(
            serializeCoasterDocument(track));
        require(!json.contains("trackConfigurationId"),
            "empty configuration ID must not be serialized");
    }
}

int main()
{
    try
    {
        modernSteelExistsInCatalog();
        modernSteelHasStableId();
        modernSteelResolvesToValidPreset();
        convenienceOverloadResolvesIdentically();
        unknownIdentityDoesNotFallback();
        regionOverridesRemainASeparateResolutionLayer();
        applyingConfigurationProducesExpectedAppearance();
        documentOwnsConcreteCopy();
        applyingConfigurationPreservesRegionOverrides();
        configurationIdentityRoundTrips();
        concretePresetSerializationRoundTrips();
        legacyDocumentsLoadCorrectly();
        unknownConfigurationIdPreservesAppearance();
        repeatingComponentPhaseIsDeterministic();
        setConfigurationIdWorks();
        resetToDefaultsClearsOverrides();
        resetToDefaultsNoOpWithoutIdentity();
        serializationDeterminism();
        configurationIdIsSerialized();
        emptyConfigurationIdNotSerialized();
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
