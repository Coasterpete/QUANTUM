#include <quantum/renderer/GroundSurface.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
    using namespace quantum::renderer;

    // Mirrors the loader's internal cap so the test asserts the documented
    // limit rather than repeating an unchecked number.
    constexpr std::uint32_t maximumTestMipLevels = 14;

    void require(const bool condition, const char* const message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    void requireNear(
        const float actual,
        const float expected,
        const float tolerance,
        const char* const message)
    {
        require(std::abs(actual - expected) <= tolerance, message);
    }

    template<typename Callable>
    [[nodiscard]] std::string messageFrom(Callable&& callable)
    {
        try
        {
            callable();
        }
        catch (const std::exception& exception)
        {
            return exception.what();
        }
        return {};
    }

    void defaultSettingsValidate()
    {
        const GroundSurfaceSettings defaults;
        validateGroundSurfaceSettings(defaults);
        require(defaults.enabled, "the built-in ground surface starts enabled");
        require(defaults.roughness > 0.5F,
            "the fallback ground is a matte dielectric");
        require(defaults.metallic == 0.0F,
            "the fallback ground is not a metal");
    }

    void invalidSettingsAreRejectedBeforeAnyGpuWork()
    {
        const auto rejects = [](const GroundSurfaceSettings& settings,
            const char* const expectedFragment)
        {
            const std::string message = messageFrom(
                [&] { validateGroundSurfaceSettings(settings); });
            require(!message.empty(), "an invalid setting is rejected");
            require(message.find(expectedFragment) != std::string::npos,
                "the rejection names the offending property");
        };

        GroundSurfaceSettings settings;
        settings.sizeX = 0.0F;
        rejects(settings, "size X");

        settings = {};
        settings.sizeY = std::numeric_limits<float>::quiet_NaN();
        rejects(settings, "size Y");

        settings = {};
        settings.elevation = 1.0e9F;
        rejects(settings, "elevation");

        settings = {};
        settings.baseColor = glm::vec4{1.0F, 0.0F, 0.0F,
            std::numeric_limits<float>::infinity()};
        rejects(settings, "base color");

        settings = {};
        settings.roughness = 1.5F;
        rejects(settings, "roughness");

        settings = {};
        settings.metallic = -0.1F;
        rejects(settings, "metallic");

        settings = {};
        settings.uvTiling = glm::vec2{0.0F, 8.0F};
        rejects(settings, "X tiling");

        settings = {};
        settings.uvTiling = glm::vec2{8.0F, 1.0e6F};
        rejects(settings, "Y tiling");
    }

    void identifiersStayInsideTheGroundPackage()
    {
        require(normalizeGroundTextureAssetIdentifier(
            "assets://ground/grass.png") == "assets://ground/grass.png",
            "a canonical ground identifier is preserved");
        require(normalizeGroundTextureAssetIdentifier(
            "assets://ground\\sub\\grass.png")
                == "assets://ground/sub/grass.png",
            "backslashes normalize to forward slashes");
        require(normalizeGroundTextureAssetIdentifier(
            "assets://ground/sub/../grass.png")
                == "assets://ground/grass.png",
            "a redundant parent segment normalizes away");
        require(normalizeGroundTextureAssetIdentifier(
            "assets://ground/Grass.PNG") == "assets://ground/Grass.PNG",
            "the extension check is case-insensitive");

        require(!messageFrom([] {
            static_cast<void>(normalizeGroundTextureAssetIdentifier(""));
        }).empty(), "an empty identifier is rejected");
        require(!messageFrom([] {
            static_cast<void>(normalizeGroundTextureAssetIdentifier(
                "C:/textures/grass.png"));
        }).empty(), "an absolute path is rejected");
        require(!messageFrom([] {
            static_cast<void>(normalizeGroundTextureAssetIdentifier(
                "assets://track/crosstie.png"));
        }).empty(), "another package root is rejected");
        require(!messageFrom([] {
            static_cast<void>(normalizeGroundTextureAssetIdentifier(
                "assets://ground/../../secret.png"));
        }).empty(), "an escaping identifier is rejected");
        require(!messageFrom([] {
            static_cast<void>(normalizeGroundTextureAssetIdentifier(
                "assets://ground/grass.tga"));
        }).empty(), "an unsupported extension is rejected");
        require(!messageFrom([] {
            static_cast<void>(normalizeGroundTextureAssetIdentifier(
                "assets://ground"));
        }).empty(), "a bare package root is rejected");
    }

    void packageRelativePathsResolveBeneathTheRuntimeRoot()
    {
        const std::filesystem::path resolved = groundTextureAssetPath(
            "assets://ground/sub/grass.png", "C:/runtime");
        require(resolved == std::filesystem::path("C:/runtime")
                / "assets" / "ground" / "sub" / "grass.png",
            "an identifier resolves below the runtime assets folder");
    }

    void meshIsAFlatQuadAtTheRequestedExtentAndTiling()
    {
        GroundSurfaceSettings settings;
        settings.elevation = 12.5F;
        settings.sizeX = 400.0F;
        settings.sizeY = 400.0F;
        settings.uvTiling = glm::vec2{8.0F, 8.0F};

        const GroundSurfaceMesh mesh = createGroundSurfaceMesh(settings);
        require(mesh.vertices.size() == 4,
            "the surface is four shared corners");
        require(mesh.indices.size() == 6, "the surface is two triangles");
        for (const std::uint32_t index : mesh.indices)
        {
            require(index < mesh.vertices.size(),
                "every index addresses a real corner");
        }
        // Each triangle must wind counter-clockwise seen from +Z, matching the
        // track pipeline's front-face convention.
        for (std::size_t triangle = 0; triangle < 2; ++triangle)
        {
            const auto& a = mesh.vertices.at(
                mesh.indices.at(triangle * 3));
            const auto& b = mesh.vertices.at(
                mesh.indices.at(triangle * 3 + 1));
            const auto& c = mesh.vertices.at(
                mesh.indices.at(triangle * 3 + 2));
            const double area =
                (b.position.x - a.position.x) * (c.position.y - a.position.y)
                - (b.position.y - a.position.y) * (c.position.x - a.position.x);
            require(area > 0.0, "a ground triangle faces upward");
        }

        const float half = 200.0F;
        for (const GroundSurfaceVertex& vertex : mesh.vertices)
        {
            requireNear(vertex.position.z, 12.5F, 1.0e-4F,
                "every corner sits at the requested elevation");
            require(std::abs(std::abs(vertex.position.x) - half) < 1.0e-3F
                || std::abs(std::abs(vertex.position.y) - half) < 1.0e-3F,
                "corners sit on the extent boundary");
            require(vertex.uv.x >= 0.0F && vertex.uv.x <= 8.0F
                && vertex.uv.y >= 0.0F && vertex.uv.y <= 8.0F,
                "UVs stay within the tiling count");
        }

        // Tiling is baked into the corner UVs, so a different tiling produces
        // different data rather than a draw-time uniform.
        settings.uvTiling = glm::vec2{2.0F, 2.0F};
        const GroundSurfaceMesh retiled = createGroundSurfaceMesh(settings);
        requireNear(retiled.vertices.at(1).uv.x, 2.0F, 1.0e-4F,
            "the tiling reaches the CPU mesh");
        require(retiled.indices == mesh.indices,
            "retiling does not change the topology");
    }

    void resizingKeepsTheSurfaceCenteredOnTheOrigin()
    {
        const auto extentOf = [](const GroundSurfaceMesh& mesh)
        {
            glm::vec2 minimum{
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max()};
            glm::vec2 maximum{
                std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::lowest()};
            for (const GroundSurfaceVertex& vertex : mesh.vertices)
            {
                minimum.x = std::min(minimum.x, vertex.position.x);
                minimum.y = std::min(minimum.y, vertex.position.y);
                maximum.x = std::max(maximum.x, vertex.position.x);
                maximum.y = std::max(maximum.y, vertex.position.y);
            }
            return std::pair{minimum, maximum};
        };

        GroundSurfaceSettings settings;
        settings.sizeX = 100.0F;
        settings.sizeY = 100.0F;
        const GroundSurfaceMesh small = createGroundSurfaceMesh(settings);
        settings.sizeX = 900.0F;
        settings.sizeY = 900.0F;
        const GroundSurfaceMesh large = createGroundSurfaceMesh(settings);

        for (const GroundSurfaceMesh& mesh : {small, large})
        {
            const auto [minimum, maximum] = extentOf(mesh);
            requireNear(minimum.x, -maximum.x, 1.0e-3F,
                "the surface is centered on the origin in X");
            requireNear(minimum.y, -maximum.y, 1.0e-3F,
                "the surface is centered on the origin in Y");
        }
        const auto [smallMinimum, smallMaximum] = extentOf(small);
        const auto [largeMinimum, largeMaximum] = extentOf(large);
        requireNear(smallMaximum.x - smallMinimum.x, 100.0F, 1.0e-2F,
            "the small surface spans the requested width");
        requireNear(largeMaximum.x - largeMinimum.x, 900.0F, 1.0e-2F,
            "resizing changes the span without moving the center");
    }

    void mipLevelCountsFollowTheSmallerExtent()
    {
        require(groundTextureMipLevelCount(0, 0) == 0,
            "a degenerate extent has no mip levels");
        require(groundTextureMipLevelCount(1, 1) == 1,
            "a 1x1 texture has one level");
        require(groundTextureMipLevelCount(4, 4) == 3,
            "a 4x4 chain reaches 1x1 through 2x2");
        require(groundTextureMipLevelCount(8, 1) == 4,
            "a non-square chain follows the larger extent");
        require(groundTextureMipLevelCount(256, 256) == 9,
            "a 256x256 chain has nine levels");
        require(groundTextureMipLevelCount(16384, 16384)
                == maximumTestMipLevels,
            "the chain length is capped for the largest supported extent");
    }

    void bundledTestTexturesDecodeIntoLinearMipChains(
        const std::filesystem::path& sourceRoot)
    {
        const std::filesystem::path albedoPath = sourceRoot / "assets"
            / "ground" / "test-ground-albedo.png";
        require(std::filesystem::is_regular_file(albedoPath),
            "the documented ground test albedo is present");

        const GroundTextureImage image = loadGroundTextureImage(
            albedoPath, true);
        require(image.width == 256 && image.height == 256,
            "the test albedo decodes at its authored size");
        require(image.srgb, "an albedo map is recorded as sRGB encoded");
        require(image.mipLevels == 9, "the mip chain is complete");

        std::size_t expected = 0;
        for (std::uint32_t level = 0; level < image.mipLevels; ++level)
        {
            expected += static_cast<std::size_t>(
                std::max(1u, image.width >> level))
                * std::max(1u, image.height >> level) * 4u;
        }
        require(expected == image.rgba.size(),
            "the packed mip chain matches the level sizes");

        // The generator's two-texel red identity border must survive as a
        // decoded sRGB sample, proving the PNG payload was read.
        const std::size_t red = (0u * 256u + 1u) * 4u;
        require(image.rgba.at(red) == 214 && image.rgba.at(red + 1) == 32
            && image.rgba.at(red + 2) == 32,
            "the albedo border decodes to its authored color");

        // Averaging four identical texels must reproduce that texel, so the
        // first mip of the border region is stable.
        const std::size_t firstMip = 4u * 256u * 256u;
        require(image.rgba.at(firstMip) == 214
            && image.rgba.at(firstMip + 1) == 32
            && image.rgba.at(firstMip + 2) == 32,
            "the box filter preserves a uniform region");

        const GroundTextureImage normal = loadGroundTextureImage(
            sourceRoot / "assets" / "ground" / "test-ground-normal.png",
            false);
        require(!normal.srgb,
            "normal data is recorded as linear");
        require(normal.width == 256 && normal.height == 256,
            "the test normal map decodes");
        // Straight up must be encoded as the blue-dominant normal.
        bool sawFlatNormal = false;
        for (std::size_t index = 0; index + 3 < normal.rgba.size(); index += 4)
        {
            if (normal.rgba.at(index) >= 126 && normal.rgba.at(index) <= 130
                && normal.rgba.at(index + 1) >= 126
                && normal.rgba.at(index + 1) <= 130
                && normal.rgba.at(index + 2) >= 253)
            {
                sawFlatNormal = true;
                break;
            }
        }
        require(sawFlatNormal,
            "the test normal map contains flat-up samples");
    }

    void unreadableTexturesReportTheirFailureReason(
        const std::filesystem::path& sourceRoot)
    {
        const std::filesystem::path missing = sourceRoot / "assets"
            / "ground" / "does-not-exist.png";
        const std::string missingMessage = messageFrom(
            [&] { static_cast<void>(loadGroundTextureImage(missing, true)); });
        require(
            classifyGroundTextureFailure(missingMessage)
                == GroundTextureLoadState::MissingAsset,
            "a missing file is classified as a missing asset");

        const std::filesystem::path notAnImage = sourceRoot / "assets"
            / "ground" / "README.md";
        const std::string imageMessage = messageFrom([&]
        {
            static_cast<void>(loadGroundTextureImage(notAnImage, true));
        });
        require(
            classifyGroundTextureFailure(imageMessage)
                == GroundTextureLoadState::UnsupportedImage,
            "a non-PNG file is classified as an unsupported image");

        for (const GroundTextureLoadState state : {
            GroundTextureLoadState::Loaded,
            GroundTextureLoadState::MissingAsset,
            GroundTextureLoadState::UnsupportedImage,
            GroundTextureLoadState::InvalidImage,
            GroundTextureLoadState::LoadFailed})
        {
            const std::string name = groundTextureLoadStateName(state);
            require(!name.empty() && name != "Load failed" || state
                    == GroundTextureLoadState::LoadFailed,
                "every state has a display name");
        }
    }
}

int main(int argc, char* argv[])
{
    try
    {
        require(argc == 2, "pass the QUANTUM source root");
        const std::filesystem::path sourceRoot = argv[1];
        defaultSettingsValidate();
        invalidSettingsAreRejectedBeforeAnyGpuWork();
        identifiersStayInsideTheGroundPackage();
        packageRelativePathsResolveBeneathTheRuntimeRoot();
        meshIsAFlatQuadAtTheRequestedExtentAndTiling();
        resizingKeepsTheSurfaceCenteredOnTheOrigin();
        mipLevelCountsFollowTheSmallerExtent();
        bundledTestTexturesDecodeIntoLinearMipChains(sourceRoot);
        unreadableTexturesReportTheirFailureReason(sourceRoot);
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Ground surface test failure: "
            << exception.what() << '\n';
        return 1;
    }

    std::cout << "Ground surface tests passed.\n";
    return 0;
}
