#pragma once

#include <quantum/math/ScalarTransition.hpp>

namespace quantum::coaster
{
    // Initial authored force-section channels. Their shared independent-
    // variable domain intentionally has no physical interpretation yet.
    struct ForceSection
    {
        math::ScalarTransition verticalForce;
        math::ScalarTransition lateralForce;
        math::ScalarTransition roll;
    };

    // Throws std::invalid_argument when a channel is malformed or when the
    // authored channel domains do not match exactly.
    void validateForceSection(const ForceSection& section);
}
