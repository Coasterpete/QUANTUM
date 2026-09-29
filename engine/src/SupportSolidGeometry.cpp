#include <quantum/renderer/SupportSolidGeometry.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <initializer_list>
#include <ranges>
#include <stdexcept>
#include <string>

namespace quantum::renderer
{
    namespace
    {
        using coaster::SupportMemberMeshKind;

        constexpr std::string_view timberAssetScheme = "assets://";
        // M2A ships the pine set only. The height and edge maps the same
        // author produced are deliberately not loaded here: displacing
        // geometry or feeding edge maps into arbitrary inputs would change
        // structural silhouettes for no defined benefit.
        constexpr std::string_view timberAssetRoot = "materials/timber/";

        // The six box faces. (u, v, normal) is a right-handed triple on every
        // face, so walking the corners in order (-u-v, +u-v, +u+v, -u+v)
        // traverses each face counter-clockwise seen from outside and the
        // emitted triangles are wound outward.
        struct BoxFace
        {
            glm::vec3 normal;
            glm::vec3 u;
            glm::vec3 v;
        };

        constexpr std::array<BoxFace, 6> boxFaces{{
            {{0.0F, 0.0F, 1.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}},
            {{0.0F, 0.0F, -1.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, -1.0F, 0.0F}},
            {{0.0F, 1.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, -1.0F}},
            {{0.0F, -1.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}},
            {{1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}},
            {{-1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 1.0F, 0.0F}},
        }};
    }

    SupportSolidMesh createSupportSolidBoxMesh()
    {
        SupportSolidMesh mesh;
        mesh.vertices.reserve(boxFaces.size() * 4);
        mesh.triangleIndices.reserve(boxFaces.size() * 6);

        for (const BoxFace& face : boxFaces)
        {
            const auto base = static_cast<std::uint32_t>(mesh.vertices.size());
            // Each corner is the face centre offset by half the face, so the
            // shared corner position of two adjacent faces matches exactly and
            // the closed solid has no seam.
            constexpr std::array<glm::vec2, 4> corners{{
                {-0.5F, -0.5F}, {0.5F, -0.5F}, {0.5F, 0.5F}, {-0.5F, 0.5F}}};
            for (const glm::vec2 corner : corners)
            {
                mesh.vertices.push_back({
                    face.normal * 0.5F + face.u * corner.x + face.v * corner.y,
                    face.normal});
            }
            mesh.triangleIndices.insert(mesh.triangleIndices.end(),
                {base, base + 1, base + 2, base, base + 2, base + 3});
        }
        return mesh;
    }

    SupportSolidMesh createSupportSolidCylinderMesh(
        const std::uint32_t radialSegments)
    {
        if (radialSegments < 3)
        {
            throw std::invalid_argument(
                "A support cylinder needs at least three radial segments.");
        }

        SupportSolidMesh mesh;
        // Two rings for the side wall plus one centre vertex per cap.
        mesh.vertices.reserve(radialSegments * 4 + 2);
        mesh.triangleIndices.reserve(radialSegments * 12);

        const auto ringPoint = [radialSegments](const std::uint32_t segment)
        {
            const float angle = 6.28318530718F
                * static_cast<float>(segment)
                / static_cast<float>(radialSegments);
            return glm::vec3{0.0F, std::cos(angle) * 0.5F,
                std::sin(angle) * 0.5F};
        };

        // The side wall shares vertices between adjacent segments. The seam is
        // continuous because each vertex is placed from its own angle.
        const auto wallBase = static_cast<std::uint32_t>(mesh.vertices.size());
        for (std::uint32_t segment = 0; segment <= radialSegments; ++segment)
        {
            const glm::vec3 point = ringPoint(segment);
            const glm::vec3 normal{0.0F, point.y * 2.0F, point.z * 2.0F};
            mesh.vertices.push_back({glm::vec3{0.5F, point.y, point.z}, normal});
            mesh.vertices.push_back({glm::vec3{-0.5F, point.y, point.z}, normal});
        }
        for (std::uint32_t segment = 0; segment < radialSegments; ++segment)
        {
            const std::uint32_t near = wallBase + segment * 2;
            const std::uint32_t far = near + 2;
            mesh.triangleIndices.insert(mesh.triangleIndices.end(),
                {near, far, far + 1, near, far + 1, near + 1});
        }

        // Caps get their own centre vertices so their normals stay flat.
        for (const float end : {0.5F, -0.5F})
        {
            const std::uint32_t centre =
                static_cast<std::uint32_t>(mesh.vertices.size());
            // The cap normal is the axis direction, not the cap's position, so
            // it must be the sign of the end rather than its half-unit offset.
            const glm::vec3 capNormal{end > 0.0F ? 1.0F : -1.0F, 0.0F, 0.0F};
            mesh.vertices.push_back({{end, 0.0F, 0.0F}, capNormal});
            const auto ringBase = static_cast<std::uint32_t>(
                mesh.vertices.size());
            for (std::uint32_t segment = 0; segment < radialSegments; ++segment)
            {
                const glm::vec3 point = ringPoint(segment);
                mesh.vertices.push_back({{end, point.y, point.z}, capNormal});
            }
            for (std::uint32_t segment = 0; segment < radialSegments; ++segment)
            {
                const std::uint32_t current = ringBase + segment;
                const std::uint32_t next =
                    ringBase + (segment + 1) % radialSegments;
                // Reversed winding on the -X cap keeps both caps facing out.
                if (end > 0.0F)
                {
                    mesh.triangleIndices.insert(mesh.triangleIndices.end(),
                        {centre, current, next});
                }
                else
                {
                    mesh.triangleIndices.insert(mesh.triangleIndices.end(),
                        {centre, next, current});
                }
            }
        }
        return mesh;
    }

    SupportSolidMesh createSupportSolidMesh(const SupportMemberMeshKind mesh)
    {
        switch (mesh)
        {
        case SupportMemberMeshKind::Rectangular:
            return createSupportSolidBoxMesh();
        case SupportMemberMeshKind::Circular:
            return createSupportSolidCylinderMesh();
        }
        throw std::invalid_argument(
            "Unknown support member mesh kind.");
    }

    std::string normalizeTimberTextureAssetIdentifier(
        const std::string_view identifier)
    {
        std::string normalized{identifier};
        std::ranges::replace(normalized, '\\', '/');
        if (!normalized.starts_with(timberAssetScheme))
            throw std::invalid_argument(
                "Timber texture must use assets://materials/timber/: "
                + normalized);
        if (normalized.find(':', timberAssetScheme.size())
                != std::string::npos
            || normalized.find('\0') != std::string::npos)
            throw std::invalid_argument(
                "Timber texture contains a machine-specific or invalid path: "
                + normalized);
        const std::filesystem::path relative = std::filesystem::path(
            normalized.substr(timberAssetScheme.size())).lexically_normal();
        for (const auto& part : relative)
            if (part == "..")
                throw std::invalid_argument(
                    "Timber texture cannot escape package root: "
                    + normalized);
        if (relative.is_absolute() || relative.has_root_name()
            || relative.has_root_directory() || relative.empty()
            || relative.begin() == relative.end())
        {
            throw std::invalid_argument(
                "Timber texture must live below assets://materials/timber/: "
                + normalized);
        }
        const std::string generic = relative.generic_string();
        if (!generic.starts_with(timberAssetRoot))
            throw std::invalid_argument(
                "Timber texture must live below assets://materials/timber/: "
                + normalized);
        std::string suffix = relative.extension().string();
        std::ranges::transform(suffix, suffix.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (suffix != ".png")
            throw std::invalid_argument(
                "Timber texture must be PNG: " + normalized);
        return "assets://" + generic;
    }

    std::filesystem::path timberTextureAssetPath(
        const std::string_view identifier,
        const std::filesystem::path& runtimeRoot)
    {
        const std::string normalized =
            normalizeTimberTextureAssetIdentifier(identifier);
        return runtimeRoot / "assets"
            / std::filesystem::path(
                normalized.substr(timberAssetScheme.size()));
    }
}
