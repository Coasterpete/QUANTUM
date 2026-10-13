#pragma once

#include <quantum/coaster/TrackStyle.hpp>

#include <string_view>
#include <string>
#include <map>

namespace quantum::coaster
{
    // Stable product identity and presentation metadata for a selectable
    // track configuration. The concrete TrackStylePreset remains the input
    // consumed by geometry, region overrides, and presentation.
    struct TrackConfigurationDefinition
    {
        std::string id;
        std::string displayName;
        std::string familyId;
        std::string category;
        std::string reference;
        std::string baseId;
        bool reserved = false;
        TrackStylePreset style;
    };

    inline constexpr std::string_view modernSteelTrackConfigurationId =
        "modern-steel";

    class TrackFamilyRegistry
    {
    public:
        // Atomic registration rejects duplicate IDs, bad providers and cycles.
        void loadJson(std::string_view json);
        void registerType(TrackConfigurationDefinition definition);
        [[nodiscard]] const TrackConfigurationDefinition* find(std::string_view id) const noexcept;
        [[nodiscard]] const std::map<std::string, TrackConfigurationDefinition, std::less<>>& types() const noexcept;
    private:
        std::map<std::string, TrackConfigurationDefinition, std::less<>> types_;
    };
    [[nodiscard]] TrackFamilyRegistry& trackFamilyRegistry();

    // Returns nullptr when the stable id is unknown.
    [[nodiscard]] const TrackConfigurationDefinition* findTrackConfiguration(
        std::string_view id);

    // Produces and validates the complete base TrackStylePreset for a built-in
    // configuration. Throws std::invalid_argument for an unknown definition.
    [[nodiscard]] TrackStylePreset resolveTrackConfiguration(
        const TrackConfigurationDefinition& configuration);

    // Convenience overload: looks up the configuration by stable ID and
    // resolves its validated default TrackStylePreset. Throws
    // std::invalid_argument when the ID is unknown.
    [[nodiscard]] TrackStylePreset resolveTrackConfiguration(
        std::string_view configurationId);
}
