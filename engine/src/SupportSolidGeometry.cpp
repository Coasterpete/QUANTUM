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
        const std::uint32_t radialSegments, const bool hollow)
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

        // Duplicate the cylindrical seam, including its UV. Inner and outer
        // walls share a unit radius; the vertex shader applies the instance's
        // validated inner diameter ratio only to vertices marked inner.
        for (unsigned wall = 0; wall < (hollow ? 2u : 1u); ++wall)
        {
            const auto base = static_cast<std::uint32_t>(mesh.vertices.size());
            for (std::uint32_t segment = 0; segment <= radialSegments; ++segment)
            {
                const auto p = ringPoint(segment);
                const glm::vec3 n{0, p.y * (wall ? -2.0F : 2.0F), p.z * (wall ? -2.0F : 2.0F)};
                const float v = static_cast<float>(segment) / radialSegments;
                mesh.vertices.push_back({{.5F,p.y,p.z}, n, {.5F,v}, static_cast<float>(wall)});
                mesh.vertices.push_back({{-.5F,p.y,p.z}, n, {-.5F,v}, static_cast<float>(wall)});
            }
            for (std::uint32_t segment = 0; segment < radialSegments; ++segment)
            {
                const auto a = base + segment*2, b = a+2;
                if (wall) mesh.triangleIndices.insert(mesh.triangleIndices.end(), {a,b,b+1,a,b+1,a+1});
                else mesh.triangleIndices.insert(mesh.triangleIndices.end(), {a,b+1,b,a,a+1,b+1});
            }
        }
        for (const float end : {.5F, -.5F})
        {
            const glm::vec3 n{end > 0 ? 1.0F : -1.0F, 0, 0};
            const auto base = static_cast<std::uint32_t>(mesh.vertices.size());
            if (hollow)
            {
                for (std::uint32_t segment = 0; segment <= radialSegments; ++segment)
                {
                    const auto p = ringPoint(segment);
                    mesh.vertices.push_back({{end,p.y,p.z},n,{0,0},0});
                    mesh.vertices.push_back({{end,p.y,p.z},n,{0,0},1});
                }
                for (std::uint32_t segment = 0; segment < radialSegments; ++segment)
                {
                    const auto a=base+segment*2, b=a+2;
                    if (end > 0) mesh.triangleIndices.insert(mesh.triangleIndices.end(), {a,b,a+1,b,b+1,a+1});
                    else mesh.triangleIndices.insert(mesh.triangleIndices.end(), {a,a+1,b,b,a+1,b+1});
                }
            }
            else
            {
                mesh.vertices.push_back({{end,0,0},n});
                for (std::uint32_t segment = 0; segment < radialSegments; ++segment)
                {
                    const auto p = ringPoint(segment);
                    mesh.vertices.push_back({{end,p.y,p.z},n});
                }
                for (std::uint32_t segment = 0; segment < radialSegments; ++segment)
                {
                    const auto a=base+1+segment, b=base+1+(segment+1)%radialSegments;
                    if (end > 0) mesh.triangleIndices.insert(mesh.triangleIndices.end(), {base,a,b});
                    else mesh.triangleIndices.insert(mesh.triangleIndices.end(), {base,b,a});
                }
            }
        }
        return mesh;
    }

    std::vector<coaster::SupportMemberInstance> createSupportFoundationInstances(
        const coaster::SupportFoundationPad& pad)
    {
        std::vector<coaster::SupportMemberInstance> result;
        const auto emit = [&](glm::dvec3 position, glm::dvec3 size)
        {
            coaster::SupportMemberInstance instance;
            if (pad.kind == coaster::SupportFoundationKind::Pier)
            {
                instance.transform[0] = {0,0,static_cast<float>(size.z),0};
                instance.transform[1] = {static_cast<float>(size.x),0,0,0};
                instance.transform[2] = {0,static_cast<float>(size.y),0,0};
                instance.section.y = 1;
            }
            else
            {
                instance.transform[0] = {static_cast<float>(size.x),0,0,0};
                instance.transform[1] = {0,static_cast<float>(size.y),0,0};
                instance.transform[2] = {0,0,static_cast<float>(size.z),0};
            }
            instance.transform[3] = {glm::vec3{position},1};
            result.push_back(instance);
        };
        const glm::dvec3 size{pad.padDimensions,pad.padDepth};
        if (pad.kind == coaster::SupportFoundationKind::Pedestal)
        {
            emit(pad.position-glm::dvec3{0,0,pad.padDepth*.275},{size.x,size.y,size.z*.45});
            emit(pad.position+glm::dvec3{0,0,pad.padDepth*.225},{size.x*.6,size.y*.6,size.z*.55});
        }
        else emit(pad.position,size);
        return result;
    }

    SupportSolidMesh createSupportSolidMesh(const SupportMemberMeshKind mesh)
    {
        switch (mesh)
        {
        case SupportMemberMeshKind::Rectangular:
            return createSupportSolidBoxMesh();
        case SupportMemberMeshKind::Circular:
            return createSupportSolidCylinderMesh();
        case SupportMemberMeshKind::CircularTube:
            return createSupportSolidCylinderMesh(supportSolidCylinderRadialSegments, true);
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
