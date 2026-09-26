#pragma once

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace quantum::renderer
{
    // Ground Surface M0 is a single flat, axis-aligned quad centered on the
    // world XY origin. It is a scene presentation element: nothing here is
    // coaster geometry, physics, or authored document state, and the renderer
    // owns every GPU resource it needs.
    //
    // The texture identifiers are package-relative logical strings
    // ("assets://ground/<file>") so a saved or shared value can never contain
    // a machine-specific absolute path. An empty identifier means "use the
    // built-in fallback", which is the neutral 1x1 map that leaves baseColor,
    // metallic, and roughness as the only appearance inputs.
    struct GroundSurfaceSettings
    {
        bool enabled = true;
        float elevation = 0.0F;
        float sizeX = 1200.0F;
        float sizeY = 1200.0F;
        glm::vec4 baseColor{0.30F, 0.32F, 0.26F, 1.0F};
        float metallic = 0.0F;
        float roughness = 0.85F;
        // Repeats of the map across the full width/height of the quad.
        glm::vec2 uvTiling{48.0F, 48.0F};
        std::string albedoTexture;
        std::string normalTexture;
        std::string roughnessTexture;
    };

    // No normal attribute: the M0 surface is horizontal by definition and its
    // shading normal comes from the normal map, whose 1x1 built-in fallback
    // decodes to straight up.
    struct GroundSurfaceVertex
    {
        glm::vec3 position{0.0F};
        glm::vec2 uv{0.0F};
    };

    struct GroundSurfaceMesh
    {
        std::vector<GroundSurfaceVertex> vertices;
        std::vector<std::uint32_t> indices;
    };

    // Throws std::invalid_argument for a non-finite component, a non-positive
    // extent, an out-of-range factor, or a malformed texture identifier. The
    // renderer calls this before touching any GPU resource so a rejected value
    // can never half-apply.
    void validateGroundSurfaceSettings(const GroundSurfaceSettings& settings);

    // CPU geometry only; the renderer retains ownership of the GPU buffer.
    // The quad's UVs already carry the tiling, so the mesh is regenerated
    // whenever elevation, extent, or tiling changes.
    [[nodiscard]] GroundSurfaceMesh createGroundSurfaceMesh(
        const GroundSurfaceSettings& settings);

    [[nodiscard]] std::string normalizeGroundTextureAssetIdentifier(
        std::string_view identifier);

    // Resolves a normalized identifier against the runtime asset root, the
    // same way package-relative track hardware assets are resolved.
    [[nodiscard]] std::filesystem::path groundTextureAssetPath(
        std::string_view identifier,
        const std::filesystem::path& runtimeRoot);

    enum class GroundTextureLoadState
    {
        Loaded,
        MissingAsset,
        UnsupportedImage,
        InvalidImage,
        LoadFailed
    };

    [[nodiscard]] const char* groundTextureLoadStateName(
        GroundTextureLoadState state) noexcept;

    // Maps a loader message onto a UI-facing state, mirroring how static mesh
    // assets classify their GLB failures.
    [[nodiscard]] GroundTextureLoadState classifyGroundTextureFailure(
        std::string_view message) noexcept;

    struct GroundTextureLoadStatus
    {
        std::string requestedIdentifier;
        GroundTextureLoadState state = GroundTextureLoadState::Loaded;
        bool usingFallback = false;
        std::string detail;
    };

    // Linear byte samples with a CPU box-filtered mip chain. srgb records how
    // the stored samples are encoded, so the renderer can upload an _SRGB
    // image for albedo and an _UNORM image for normal/roughness data, and so
    // mip reduction happens in linear light for encoded sources.
    struct GroundTextureImage
    {
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        std::uint32_t mipLevels = 1;
        bool srgb = false;
        std::vector<std::uint8_t> rgba;
    };

    [[nodiscard]] std::uint32_t groundTextureMipLevelCount(
        std::uint32_t width,
        std::uint32_t height) noexcept;

    // Decodes a ground map through SDL's PNG loader. srgbEncoded selects how
    // the stored samples are interpreted, because the file itself does not
    // record it: albedo is conventionally sRGB, while normal and roughness
    // data is linear. Throws std::runtime_error for a missing file, an
    // unreadable or malformed image, or an unsupported pixel layout; the
    // caller decides whether that is reported or falls back to the built-in
    // map.
    [[nodiscard]] GroundTextureImage loadGroundTextureImage(
        const std::filesystem::path& path,
        bool srgbEncoded);
}
