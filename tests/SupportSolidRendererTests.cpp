#include <quantum/renderer/GroundSurface.hpp>
#include <quantum/renderer/SupportSolidGeometry.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#if defined(_MSC_VER) && defined(_DEBUG)
#include <crtdbg.h>
#endif

namespace
{
    using namespace quantum::renderer;
    using SupportMemberMeshKind = quantum::coaster::SupportMemberMeshKind;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    constexpr double tolerance = 1e-5;

    [[nodiscard]] bool near(const float a, const float b)
    {
        return std::abs(a - b) <= tolerance;
    }

    // glm::vec3 is not a range, so component iteration is explicit.
    void requireFinite(const glm::vec3& value)
    {
        for (const float component : {value.x, value.y, value.z})
        {
            require(std::isfinite(component),
                "Every vector component must be finite.");
        }
    }

    void boxMeshIsAClosedUnitCube()
    {
        const SupportSolidMesh mesh = createSupportSolidBoxMesh();
        require(mesh.vertices.size() == 24,
            "The unit box must have four vertices per face so each face has a "
            "single flat normal.");
        require(mesh.triangleIndices.size() == 36,
            "The unit box must emit two triangles per face.");
        require(mesh.triangleIndices.size() % 3 == 0,
            "The unit box index stream must be whole triangles.");

        for (const SupportSolidVertex& vertex : mesh.vertices)
        {
            requireFinite(vertex.position);
            for (const float component :
                {vertex.position.x, vertex.position.y, vertex.position.z})
            {
                require(std::abs(component) <= 0.5F + tolerance,
                    "Every unit box vertex must lie inside the unit cube.");
            }
            require(near(glm::length(vertex.normal), 1.0F),
                "Every unit box normal must be a unit vector.");
            // Faceted normals are what make each face's tangent basis well
            // defined, so a normal must be axis-aligned.
            const glm::vec3 absolute(
                std::abs(vertex.normal.x), std::abs(vertex.normal.y),
                std::abs(vertex.normal.z));
            const int aligned = (near(absolute.x, 1.0F) ? 1 : 0)
                + (near(absolute.y, 1.0F) ? 1 : 0)
                + (near(absolute.z, 1.0F) ? 1 : 0);
            require(aligned == 1,
                "Every unit box normal must lie on exactly one axis, so the "
                "grain-aligned UV projection has a well-defined face plane.");
        }

        // Each face must point in six distinct directions, and every index
        // must address a real vertex.
        std::vector<glm::vec3> distinct;
        for (const SupportSolidVertex& vertex : mesh.vertices)
        {
            if (std::ranges::none_of(distinct,
                [&vertex](const glm::vec3& seen)
                {
                    return near(glm::length(seen - vertex.normal), 0.0F);
                }))
            {
                distinct.push_back(vertex.normal);
            }
        }
        require(distinct.size() == 6,
            "The unit box must cover all six face directions.");

        for (const std::uint32_t index : mesh.triangleIndices)
        {
            require(index < mesh.vertices.size(),
                "Every unit box index must address a real vertex.");
        }
    }

    void boxFaceWindingIsOutward()
    {
        // A reversed face would be back-face culled away, so a member would
        // render as a hollow shell. Culling is disabled at runtime, but the
        // geometry should still be wound correctly.
        const SupportSolidMesh mesh = createSupportSolidBoxMesh();
        for (std::size_t face = 0; face < 6; ++face)
        {
            const auto base = static_cast<std::uint32_t>(face * 4);
            const glm::vec3 a = mesh.vertices[base].position;
            const glm::vec3 b = mesh.vertices[base + 1].position;
            const glm::vec3 c = mesh.vertices[base + 2].position;
            const glm::vec3 outward = mesh.vertices[base].normal;
            const glm::vec3 geometric =
                glm::normalize(glm::cross(b - a, c - a));
            require(glm::dot(geometric, outward) > 0.0F,
                "Every unit box face must be wound counter-clockwise when "
                "viewed from outside.");
        }
    }

    void boxIsClosedWithMatchingSharedCorners()
    {
        // Adjacent faces must agree on their shared corner positions,
        // otherwise the closed solid would show a seam.
        const SupportSolidMesh mesh = createSupportSolidBoxMesh();
        std::size_t sharedPairs = 0;
        for (std::size_t first = 0; first < mesh.vertices.size(); ++first)
        {
            for (std::size_t second = first + 1;
                second < mesh.vertices.size(); ++second)
            {
                const bool coincident = near(
                    glm::length(
                        mesh.vertices[first].position
                        - mesh.vertices[second].position),
                    0.0F);
                const bool distinctNormal = !near(
                    glm::length(
                        mesh.vertices[first].normal
                        - mesh.vertices[second].normal),
                    0.0F);
                sharedPairs += coincident && distinctNormal ? 1u : 0u;
            }
        }
        // A cube's 8 corners are each shared by exactly 3 faces, and each of
        // the 6 faces owns 4 corners, so 24 coincident opposite-normal pairs
        // must exist.
        require(sharedPairs == 24,
            "Unit box corners must be shared exactly between adjacent faces so "
            "the solid has no seam.");
    }

    void cylinderMeshIsClosedAndCorrectlySized()
    {
        const SupportSolidMesh mesh =
            createSupportSolidCylinderMesh(supportSolidCylinderRadialSegments);
        require(!mesh.vertices.empty() && !mesh.triangleIndices.empty(),
            "The unit cylinder mesh must not be empty.");
        require(mesh.triangleIndices.size() % 3 == 0,
            "The unit cylinder index stream must be whole triangles.");

        for (const SupportSolidVertex& vertex : mesh.vertices)
        {
            requireFinite(vertex.position);
            require(std::abs(vertex.position.x) <= 0.5F + tolerance,
                "A unit cylinder must span the unit length on its axis.");
            const float radius = std::hypot(vertex.position.y,
                vertex.position.z);
            require(radius <= 0.5F + tolerance,
                "A unit cylinder must have unit diameter.");
            const float normalLength = glm::length(vertex.normal);
            require(std::isfinite(normalLength) && near(normalLength, 1.0F),
                "Every unit cylinder normal must be a finite unit vector: "
                "length " + std::to_string(normalLength));
        }

        // The side wall shares its vertices around the ring, so it must have
        // far fewer vertices than a naive per-quad mesh.
        const std::size_t sideWallVertices =
            static_cast<std::size_t>(supportSolidCylinderRadialSegments + 1)
            * 2;
        require(sideWallVertices < mesh.vertices.size(),
            "The unit cylinder side wall must share vertices around its ring "
            "rather than duplicating them per quad.");
    }

    void cylinderRejectsTooFewSegments()
    {
        bool rejected = false;
        try
        {
            static_cast<void>(createSupportSolidCylinderMesh(2));
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected,
            "A cylinder with fewer than three segments has no closed "
            "cross-section and must be rejected.");
    }

    void meshSelectionMatchesProfileShape()
    {
        require(createSupportSolidMesh(SupportMemberMeshKind::Rectangular)
                    .vertices.size()
                == createSupportSolidBoxMesh().vertices.size(),
            "The rectangular mesh kind must select the unit box.");
        require(createSupportSolidMesh(SupportMemberMeshKind::Circular)
                    .vertices.size()
                == createSupportSolidCylinderMesh().vertices.size(),
            "The circular mesh kind must select the unit cylinder.");
    }

    void unitMeshesAreDeterministicAndImmutable()
    {
        // The meshes are uploaded once and shared by every instance, so they
        // must be byte-identical across calls.
        const SupportSolidMesh firstBox = createSupportSolidBoxMesh();
        const SupportSolidMesh secondBox = createSupportSolidBoxMesh();
        require(firstBox.vertices.size() == secondBox.vertices.size()
                && firstBox.triangleIndices == secondBox.triangleIndices,
            "Repeated unit mesh generation must produce identical indices.");
        for (std::size_t index = 0; index < firstBox.vertices.size(); ++index)
        {
            require(firstBox.vertices[index].position
                    == secondBox.vertices[index].position
                && firstBox.vertices[index].normal
                    == secondBox.vertices[index].normal,
                "Repeated unit mesh generation must produce identical "
                "vertices.");
        }
    }

    void timberIdentifiersAreCanonical()
    {
        const std::string normalized = normalizeTimberTextureAssetIdentifier(
            "assets://materials/timber/pine/pinewood_normal.png");
        require(normalized
                == "assets://materials/timber/pine/pinewood_normal.png",
            "A canonical timber identifier must round-trip unchanged.");

        require(normalizeTimberTextureAssetIdentifier(
                    "assets://materials\\timber\\pine\\pinewood_normal.png")
                == normalized,
            "Backslash separators must normalize to forward slashes.");

        const std::array<std::string_view, 7> rejected{
            "materials/timber/pine/pinewood_normal.png",
            "assets://ground/pinewood_normal.png",
            "assets://materials/timber/../ground/pinewood_normal.png",
            "assets://materials/timber/pine/pinewood_normal.jpg",
            "assets://materials/timber/pine/pinewood_normal",
            "assets://C:/timber/pinewood_normal.png",
            "assets://",
        };
        for (const std::string_view candidate : rejected)
        {
            bool refused = false;
            try
            {
                static_cast<void>(
                    normalizeTimberTextureAssetIdentifier(candidate));
            }
            catch (const std::invalid_argument&)
            {
                refused = true;
            }
            require(refused,
                "A timber identifier must be rejected when it is not a "
                "package-relative PNG below assets://materials/timber/.");
        }
    }

    // Turns a package-relative identifier into a path under the source tree,
    // which is where the bundled timber set lives in the repository.
    [[nodiscard]] std::filesystem::path bundledSourcePath(
        const std::filesystem::path& sourceRoot,
        const std::string_view identifier)
    {
        constexpr std::string_view scheme = "assets://";
        return sourceRoot / "assets"
            / std::filesystem::path(
                std::string(identifier).substr(scheme.size()));
    }

    void timberAssetPathsResolveBeneathTheRuntimeRoot(
        const std::filesystem::path& sourceRoot)
    {
        const std::filesystem::path root{"C:/runtime"};
        const std::filesystem::path resolved = timberTextureAssetPath(
            timberNormalAsset, root);
        require(resolved == root / "assets" / "materials" / "timber" / "pine"
                / "pinewood_normal.png",
            "A timber asset must resolve beneath the runtime asset root with "
            "no machine-specific path in the identifier.");
        require(resolved.is_absolute(),
            "A resolved timber asset path must be absolute so it never depends "
            "on the process working directory.");

        // Every bundled slot M2A samples must exist in the repository.
        for (const std::string_view identifier : {timberAlbedoAsset,
                 timberNormalAsset, timberRoughnessAsset})
        {
            require(std::filesystem::is_regular_file(
                        bundledSourcePath(sourceRoot, identifier)),
                "Every bundled timber slot M2A samples must exist in the "
                "repository.");
        }
    }

    void bundledTimberBaseColorIsNeutralGrayscale(
        const std::filesystem::path& sourceRoot)
    {
        // The neutral detail contract is what makes the authored tint the
        // timber's dominant color. If a future asset baked a hue in here, the
        // tint would stop being authoritative.
        const GroundTextureImage image = loadGroundTextureImage(
            bundledSourcePath(sourceRoot, timberAlbedoAsset), true);
        require(image.width > 0 && image.height > 0 && image.mipLevels > 0,
            "The bundled timber base color must decode with a mip chain.");

        // Sampling the smallest mip is enough to catch a baked hue and keeps
        // this check fast. A level's extent is its own shifted extent, not the
        // full texture extent, so both the offset walk and the sample loop use
        // per-level dimensions.
        const std::uint32_t level = image.mipLevels - 1;
        std::size_t offset = 0;
        for (std::uint32_t index = 0; index < level; ++index)
        {
            offset += 4u * static_cast<std::size_t>(
                std::max(1u, image.width >> index))
                * std::max(1u, image.height >> index);
        }
        const std::uint32_t levelWidth = std::max(1u, image.width >> level);
        const std::uint32_t levelHeight = std::max(1u, image.height >> level);
        require(offset + 4u * static_cast<std::size_t>(levelWidth)
                * levelHeight <= image.rgba.size(),
            "The sampled mip level must lie inside the decoded image data.");
        for (std::uint32_t y = 0; y < levelHeight; ++y)
        {
            for (std::uint32_t x = 0; x < levelWidth; ++x)
            {
                const std::size_t texel = offset
                    + 4u * (static_cast<std::size_t>(y) * levelWidth + x);
                require(image.rgba[texel] == image.rgba[texel + 1]
                        && image.rgba[texel + 1] == image.rgba[texel + 2],
                    "The bundled timber base color must be grayscale so the "
                    "authored tint supplies the timber's color.");
            }
        }
    }
}

int main(const int argc, char* argv[])
{
#if defined(_MSC_VER) && defined(_DEBUG)
    // Keep STL/CRT assertions visible in CTest output without opening a
    // modal dialog that stalls the test process.
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
#endif
    try
    {
        require(argc == 2, "pass the QUANTUM source root");
        const std::filesystem::path sourceRoot{argv[1]};
        boxMeshIsAClosedUnitCube();
        boxFaceWindingIsOutward();
        boxIsClosedWithMatchingSharedCorners();
        cylinderMeshIsClosedAndCorrectlySized();
        cylinderRejectsTooFewSegments();
        meshSelectionMatchesProfileShape();
        unitMeshesAreDeterministicAndImmutable();
        timberIdentifiersAreCanonical();
        timberAssetPathsResolveBeneathTheRuntimeRoot(sourceRoot);
        bundledTimberBaseColorIsNeutralGrayscale(sourceRoot);
    }
    catch (const std::exception& error)
    {
        std::cerr << "Support solid renderer test failure: "
            << error.what() << '\n';
        return 1;
    }

    std::cout << "Support solid renderer tests passed.\n";
    return 0;
}
