#pragma once

#include <string>
#include <string_view>

namespace quantum::editor
{
    struct CameraNavigationSettings
    {
        float orbitSensitivity = 1.0F;
        float panSensitivity = 1.0F;
        float zoomSensitivity = 0.65F;
        float movementUnitsPerSecond = 18.0F;
        float fastMovementMultiplier = 4.0F;
        bool zoomToCursor = true;
        bool operator==(const CameraNavigationSettings&) const = default;
    };

    struct TransitionEditorInputSettings
    {
        float normalDragGain = 0.6F;
        float fineDragGain = 0.1F;
        bool snapEnabled = false;
        double snapIncrement = 0.005; // radians per coordinate unit
        double forceSnapIncrement = 0.05; // G
        double accelerationSnapIncrement = 0.1; // m/s^2
        bool distanceSnapEnabled = false;
        double distanceSnapIncrement = 5.0;
        bool operator==(const TransitionEditorInputSettings&) const = default;
    };

    struct InteractionPreferences
    {
        CameraNavigationSettings camera;
        TransitionEditorInputSettings transitions;
        bool operator==(const InteractionPreferences&) const = default;
    };

    // Versioned, editor-only preferences. Missing fields retain defaults;
    // malformed/out-of-range data is rejected before replacing live settings.
    [[nodiscard]] std::string serializeInteractionPreferences(const InteractionPreferences& preferences);
    [[nodiscard]] InteractionPreferences deserializeInteractionPreferences(std::string_view json);
}
