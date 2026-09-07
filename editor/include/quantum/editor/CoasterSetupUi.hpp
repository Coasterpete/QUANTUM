#pragma once

#include <quantum/coaster/CoasterSetup.hpp>
#include <quantum/editor/EditorStyle.hpp>

#include <optional>

namespace quantum::coaster
{
    class AuthoredTrack;
}

namespace quantum::editor
{
    inline constexpr char coasterSetupWindowName[] =
        "Coaster Setup###COASTER SETUP";

    // Draws the self-contained Coaster Setup panel and returns one complete
    // candidate edit. The caller owns validation, document history, and dirty
    // state; this module owns no document data.
    [[nodiscard]] std::optional<coaster::CoasterSetup>
    drawCoasterSetupWindow(
        const coaster::AuthoredTrack* authoredTrack,
        bool* open,
        const EditorFonts& fonts);
}
