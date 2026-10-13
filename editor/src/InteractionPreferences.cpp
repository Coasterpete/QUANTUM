#include <quantum/editor/InteractionPreferences.hpp>
#include <nlohmann/json.hpp>
#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace quantum::editor
{
    std::string serializeInteractionPreferences(const InteractionPreferences& p)
    {
        return nlohmann::json{
            {"version", 1},
            {"camera", {{"orbit", p.camera.orbitSensitivity}, {"pan", p.camera.panSensitivity},
                {"zoom", p.camera.zoomSensitivity}, {"speed", p.camera.movementUnitsPerSecond},
                {"fast", p.camera.fastMovementMultiplier}, {"cursor", p.camera.zoomToCursor}}},
            {"transitions", {{"drag", p.transitions.normalDragGain}, {"fine", p.transitions.fineDragGain},
                {"snap", p.transitions.snapEnabled}, {"increment", p.transitions.snapIncrement},
                {"forceIncrement", p.transitions.forceSnapIncrement}, {"accelerationIncrement", p.transitions.accelerationSnapIncrement},
                {"distanceSnap", p.transitions.distanceSnapEnabled}, {"distanceIncrement", p.transitions.distanceSnapIncrement}}}
        }.dump();
    }

    InteractionPreferences deserializeInteractionPreferences(std::string_view text)
    {
        const auto json = nlohmann::json::parse(text);
        if (!json.is_object() || json.value("version", 1) != 1)
            throw std::invalid_argument("Unsupported interaction preferences version.");
        InteractionPreferences p;
        const auto number = [](const nlohmann::json& object, const char* key, auto& value,
            double minimum, double maximum)
        {
            const double parsed = object.value(key, static_cast<double>(value));
            if (!std::isfinite(parsed) || parsed < minimum * (1.0 - 1e-7) || parsed > maximum * (1.0 + 1e-7))
                throw std::invalid_argument(std::string("Invalid interaction preference: ") + key);
            value = static_cast<std::remove_reference_t<decltype(value)>>(parsed);
        };
        if (json.contains("camera"))
        {
            const auto& c = json.at("camera");
            if (!c.is_object()) throw std::invalid_argument("Camera preferences must be an object.");
            number(c, "orbit", p.camera.orbitSensitivity, .05, 4);
            number(c, "pan", p.camera.panSensitivity, .05, 4);
            number(c, "zoom", p.camera.zoomSensitivity, .05, 4);
            number(c, "speed", p.camera.movementUnitsPerSecond, 1, 200);
            number(c, "fast", p.camera.fastMovementMultiplier, 1, 10);
            p.camera.zoomToCursor = c.value("cursor", p.camera.zoomToCursor);
        }
        if (json.contains("transitions"))
        {
            const auto& t = json.at("transitions");
            if (!t.is_object()) throw std::invalid_argument("Transition preferences must be an object.");
            number(t, "drag", p.transitions.normalDragGain, .01, 4);
            number(t, "fine", p.transitions.fineDragGain, .01, 4);
            number(t, "increment", p.transitions.snapIncrement, 1e-12, 1e6);
            number(t, "forceIncrement", p.transitions.forceSnapIncrement, 1e-12, 1e6);
            number(t, "accelerationIncrement", p.transitions.accelerationSnapIncrement, 1e-12, 1e6);
            number(t, "distanceIncrement", p.transitions.distanceSnapIncrement, 1e-12, 1e6);
            p.transitions.snapEnabled = t.value("snap", p.transitions.snapEnabled);
            p.transitions.distanceSnapEnabled = t.value("distanceSnap", p.transitions.distanceSnapEnabled);
        }
        return p;
    }
}
