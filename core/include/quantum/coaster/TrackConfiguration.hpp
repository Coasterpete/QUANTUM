#pragma once

#include <quantum/coaster/TrackStyle.hpp>

#include <string_view>

namespace quantum::coaster
{
    // Stable product identity and presentation metadata for a selectable
    // track configuration. The concrete TrackStylePreset remains the input
    // consumed by geometry, region overrides, and presentation.
    struct TrackConfigurationDefinition
    {
        std::string_view id;
        std::string_view displayName;
    };

    inline constexpr std::string_view modernSteelTrackConfigurationId =
        "modern-steel";

    // Returns nullptr when the stable id is unknown.
    [[nodiscard]] const TrackConfigurationDefinition* findTrackConfiguration(
        std::string_view id) noexcept;

    // Produces and validates the complete base TrackStylePreset for a built-in
    // configuration. Throws std::invalid_argument for an unknown definition.
    [[nodiscard]] TrackStylePreset resolveTrackConfiguration(
        const TrackConfigurationDefinition& configuration);
}
