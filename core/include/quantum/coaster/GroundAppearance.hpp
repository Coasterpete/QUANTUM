#pragma once

#include <glm/vec2.hpp>
#include <glm/vec4.hpp>

#include <string>
#include <string_view>

namespace quantum::coaster
{
    // Authored presentation data. Empty map identifiers use the neutral map.
    struct GroundAppearance
    {
        bool enabled = true;
        float elevation = 0.0F;
        float sizeX = 1200.0F;
        float sizeY = 1200.0F;
        glm::vec4 baseColor{0.30F, 0.32F, 0.26F, 1.0F};
        float metallic = 0.0F;
        float roughness = 0.85F;
        glm::vec2 uvTiling{48.0F, 48.0F};
        std::string albedoTexture;
        std::string normalTexture;
        std::string roughnessTexture;
    };

    // Shared by new documents and explicit Reset Ground. The struct's defaults
    // above remain the compatibility fallback for older documents.
    [[nodiscard]] GroundAppearance newDocumentGroundAppearance();

    [[nodiscard]] std::string normalizeGroundTextureAssetIdentifier(
        std::string_view identifier);
    void validateGroundAppearance(const GroundAppearance& appearance);
}
