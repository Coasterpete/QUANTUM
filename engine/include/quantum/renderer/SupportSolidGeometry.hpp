#pragma once

#include <quantum/coaster/SupportSolidGeometry.hpp>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace quantum::renderer
{
    // Shared unit meshes for the solid support pass. Both span
    // [-0.5, 0.5] on every axis; the instance transform supplies length and
    // cross-section. They are immutable and uploaded once.
    struct SupportSolidVertex
    {
        glm::vec3 position{0.0F};
        glm::vec3 normal{0.0F, 0.0F, 1.0F};
    };

    struct SupportSolidMesh
    {
        std::vector<SupportSolidVertex> vertices;
        std::vector<std::uint32_t> triangleIndices;
    };

    // Faceted normals on the box keep each face's tangent basis well defined,
    // which the grain-aligned UV projection depends on.
    [[nodiscard]] SupportSolidMesh createSupportSolidBoxMesh();

    // A round post has no flat face to project across, so its V coordinate
    // wraps the circumference. radialSegments sets the tessellation; 16 is
    // enough that a support post never reads as faceted at editor zoom.
    inline constexpr std::uint32_t supportSolidCylinderRadialSegments = 16;

    [[nodiscard]] SupportSolidMesh createSupportSolidCylinderMesh(
        std::uint32_t radialSegments = supportSolidCylinderRadialSegments);

    [[nodiscard]] SupportSolidMesh createSupportSolidMesh(
        coaster::SupportMemberMeshKind mesh);

    // Resolved package-relative identifiers for the bundled timber maps.
    // These are the M2A inputs: a neutral grayscale base color, a tangent
    // normal map, and a roughness detail map. The authored appearance decides
    // the timber's actual color.
    inline constexpr std::string_view timberAlbedoAsset =
        "assets://materials/timber/pine/pinewood_basecolor_neutral.png";
    inline constexpr std::string_view timberNormalAsset =
        "assets://materials/timber/pine/pinewood_normal.png";
    inline constexpr std::string_view timberRoughnessAsset =
        "assets://materials/timber/pine/pinewood_roughness.png";

    // Rejects a malformed package-relative timber identifier. M2A ships the
    // pine set, but the grammar keeps a saved value machine-independent and
    // gives future materials a place to come from.
    [[nodiscard]] std::string normalizeTimberTextureAssetIdentifier(
        std::string_view identifier);

    // Resolves a normalized identifier against the runtime asset root, the
    // same way ground and track hardware assets resolve.
    [[nodiscard]] std::filesystem::path timberTextureAssetPath(
        std::string_view identifier,
        const std::filesystem::path& runtimeRoot);
}
