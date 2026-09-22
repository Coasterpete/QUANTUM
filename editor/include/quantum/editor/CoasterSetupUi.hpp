#pragma once

#include <quantum/coaster/CoasterSetup.hpp>
#include <quantum/coaster/TrackPhysicalSettings.hpp>
#include <quantum/editor/EditorStyle.hpp>

#include <optional>
#include <string>

namespace quantum::coaster
{
    class AuthoredTrack;
}

namespace quantum::editor
{
    inline constexpr char coasterSetupWindowName[] =
        "Coaster Setup###COASTER SETUP";

    struct CoasterSetupWindowEdits
    {
        std::optional<coaster::CoasterSetup> coasterSetup;
        std::optional<coaster::TrackPhysicalSettings> physicalSettings;
        std::optional<std::string> trackConfigurationId;
        bool resetTrackConfiguration = false;
    };

    // Draws the self-contained Coaster Setup panel and returns one complete
    // candidate for each changed document setting. The caller owns validation,
    // document history, and dirty state; this module owns no document data.
    [[nodiscard]] CoasterSetupWindowEdits
    drawCoasterSetupWindow(
        const coaster::AuthoredTrack* authoredTrack,
        bool* open,
        const EditorFonts& fonts);
}
