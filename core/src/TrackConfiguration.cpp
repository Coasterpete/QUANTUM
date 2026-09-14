#include <quantum/coaster/TrackConfiguration.hpp>

#include <array>
#include <span>
#include <stdexcept>
#include <string>

namespace quantum::coaster
{
    namespace
    {
        using TrackStyleFactory = TrackStylePreset (*)();

        struct BuiltInTrackConfiguration
        {
            TrackConfigurationDefinition definition;
            TrackStyleFactory createBaseStyle;
        };

        [[nodiscard]] std::span<const BuiltInTrackConfiguration>
        builtInTrackConfigurations() noexcept
        {
            static const std::array<BuiltInTrackConfiguration, 1>
                configurations = {{
                    {{modernSteelTrackConfigurationId, "Modern Steel"},
                        &createModernSteelPreset}
                }};
            return configurations;
        }
    }

    const TrackConfigurationDefinition* findTrackConfiguration(
        const std::string_view id) noexcept
    {
        for (const BuiltInTrackConfiguration& configuration :
            builtInTrackConfigurations())
        {
            if (configuration.definition.id == id)
            {
                return &configuration.definition;
            }
        }
        return nullptr;
    }

    TrackStylePreset resolveTrackConfiguration(
        const TrackConfigurationDefinition& configuration)
    {
        for (const BuiltInTrackConfiguration& builtIn :
            builtInTrackConfigurations())
        {
            if (builtIn.definition.id == configuration.id)
            {
                TrackStylePreset style = builtIn.createBaseStyle();
                validateTrackStyle(style);
                return style;
            }
        }

        throw std::invalid_argument(
            "Unknown track configuration '"
            + std::string(configuration.id) + "'.");
    }
}
