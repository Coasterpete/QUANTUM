#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>

#include <optional>

namespace quantum::coaster
{
    // Generates one complete structure on the authored distance domain.
    // A replacement ID must identify a previously generated wooden run.
    // The document is unchanged if generation or validation fails.
    [[nodiscard]] SupportStructureId generateWoodenSupportRun(
        AuthoredTrack& track,
        const WoodenSupportRunRecipe& recipe,
        std::optional<SupportStructureId> replaceStructureId = std::nullopt);
}
