#pragma once

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <cstdint>
#include <string>
#include <string_view>

namespace quantum::coaster
{
    enum class SupportMaterialKind : std::uint8_t
    {
        Timber, PaintedSteel, GalvanizedSteel, Concrete
    };

    // Authored SI repeats: x along the member, y across the surface. Geometry
    // remains in Core units; presentation converts at the document boundary.
    struct SupportSurface
    {
        SupportMaterialKind material = SupportMaterialKind::Timber;
        glm::vec3 baseColorTint{0.78F, 0.64F, 0.47F};
        float roughness = 0.85F;
        float metallic = 0.0F;
        float normalStrength = 1.0F;
        glm::vec2 repeatMeters{1.0F, 1390.0F / 2240.0F};
        bool operator==(const SupportSurface&) const = default;
    };

    [[nodiscard]] SupportSurface defaultSupportSurface(SupportMaterialKind material);
    void validateSupportSurface(const SupportSurface& surface);
    [[nodiscard]] const char* supportMaterialName(SupportMaterialKind material);
    [[nodiscard]] SupportMaterialKind supportMaterialFromName(std::string_view name);
    [[nodiscard]] std::string serializeSupportSurface(const SupportSurface& surface);
    [[nodiscard]] SupportSurface deserializeSupportSurface(std::string_view json);
}
