#include <quantum/renderer/GroundSurface.hpp>

#include <SDL3/SDL.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace
{
    // Identifiers follow the same grammar as package-relative track hardware
    // assets, and additionally pin the "ground" package root so a value can
    // never point at an unrelated asset category.
    constexpr std::string_view assetsScheme = "assets://";

    // A quad this small or large stops being a usable ground surface: the
    // lower bound keeps the texture's world density meaningful and the upper
    // bound keeps it inside the viewport far plane with a finite depth
    // precision, so a slider typo cannot produce an unusable scene.
    constexpr float minimumGroundExtent = 1.0F;
    constexpr float maximumGroundExtent = 100'000.0F;
    constexpr float maximumGroundTiling = 4'096.0F;
    constexpr float maximumGroundElevation = 1'000'000.0F;

    constexpr std::uint32_t maximumMipLevels = 14;
    constexpr std::uint32_t maximumTextureExtent = 16'384;

    float srgbChannelToLinear(const std::uint8_t encoded) noexcept
    {
        const float channel = static_cast<float>(encoded) / 255.0F;
        return channel <= 0.04045F
            ? channel / 12.92F
            : std::pow((channel + 0.055F) / 1.055F, 2.4F);
    }

    std::uint8_t linearChannelToSrgb(const float linear) noexcept
    {
        const float clamped = std::clamp(linear, 0.0F, 1.0F);
        const float channel = clamped <= 0.0031308F
            ? clamped * 12.92F
            : 1.055F * std::pow(clamped, 1.0F / 2.4F) - 0.055F;
        return static_cast<std::uint8_t>(
            std::lround(std::clamp(channel, 0.0F, 1.0F) * 255.0F));
    }

    // Byte offset of one mip level inside a tightly packed RGBA chain.
    [[nodiscard]] std::size_t mipLevelOffset(
        const std::uint32_t width,
        const std::uint32_t height,
        const std::uint32_t mipLevel) noexcept
    {
        std::size_t offset = 0;
        for (std::uint32_t level = 0; level < mipLevel; ++level)
        {
            offset += 4u * static_cast<std::size_t>(
                std::max(1u, width >> level))
                * std::max(1u, height >> level);
        }
        return offset;
    }

    // Box reduction of one level into the next. Odd source extents average the
    // samples that actually exist instead of clamping to a duplicate edge, so
    // a non-power-of-two texture keeps a uniform average brightness.
    void reduceMipLevel(
        const std::uint8_t* const source,
        const std::uint32_t sourceWidth,
        const std::uint32_t sourceHeight,
        std::uint8_t* const destination,
        const std::uint32_t destinationWidth,
        const std::uint32_t destinationHeight,
        const bool srgb)
    {
        for (std::uint32_t y = 0; y < destinationHeight; ++y)
        {
            for (std::uint32_t x = 0; x < destinationWidth; ++x)
            {
                std::array<float, 4> accumulated{};
                std::uint32_t samples = 0;
                for (std::uint32_t offsetY = 0; offsetY < 2; ++offsetY)
                {
                    const std::uint32_t sourceY = y * 2 + offsetY;
                    if (sourceY >= sourceHeight)
                        continue;
                    for (std::uint32_t offsetX = 0; offsetX < 2; ++offsetX)
                    {
                        const std::uint32_t sourceX = x * 2 + offsetX;
                        if (sourceX >= sourceWidth)
                            continue;
                        const std::size_t sourceIndex = 4u * (
                            static_cast<std::size_t>(sourceY) * sourceWidth
                            + sourceX);
                        for (std::size_t channel = 0; channel < 4; ++channel)
                        {
                            accumulated[channel] += srgb
                                ? srgbChannelToLinear(
                                    source[sourceIndex + channel])
                                : static_cast<float>(
                                    source[sourceIndex + channel]) / 255.0F;
                        }
                        ++samples;
                    }
                }

                const std::size_t destinationIndex = 4u * (
                    static_cast<std::size_t>(y) * destinationWidth + x);
                for (std::size_t channel = 0; channel < 4; ++channel)
                {
                    const float average = accumulated[channel]
                        / static_cast<float>(samples);
                    destination[destinationIndex + channel] = srgb
                        ? linearChannelToSrgb(average)
                        : static_cast<std::uint8_t>(std::lround(
                            std::clamp(average, 0.0F, 1.0F) * 255.0F));
                }
            }
        }
    }

    bool finiteVector(const glm::vec4& value) noexcept
    {
        return std::isfinite(value.x) && std::isfinite(value.y)
            && std::isfinite(value.z) && std::isfinite(value.w);
    }
}

namespace quantum::renderer
{
    std::string normalizeGroundTextureAssetIdentifier(
        const std::string_view identifier)
    {
        return coaster::normalizeGroundTextureAssetIdentifier(identifier);
    }

    std::filesystem::path groundTextureAssetPath(
        const std::string_view identifier,
        const std::filesystem::path& runtimeRoot)
    {
        const std::string normalized =
            normalizeGroundTextureAssetIdentifier(identifier);
        return runtimeRoot / "assets"
            / std::filesystem::path(normalized.substr(assetsScheme.size()));
    }

    const char* groundTextureLoadStateName(
        const GroundTextureLoadState state) noexcept
    {
        switch (state)
        {
        case GroundTextureLoadState::Loaded: return "Loaded";
        case GroundTextureLoadState::MissingAsset: return "Missing asset";
        case GroundTextureLoadState::UnsupportedImage: return "Unsupported image";
        case GroundTextureLoadState::InvalidImage: return "Invalid image";
        case GroundTextureLoadState::LoadFailed: return "Load failed";
        }
        return "Load failed";
    }

    GroundTextureLoadState classifyGroundTextureFailure(
        const std::string_view message) noexcept
    {
        if (message.find("file not found") != std::string_view::npos)
        {
            return GroundTextureLoadState::MissingAsset;
        }
        if (message.find("not a readable PNG") != std::string_view::npos
            || message.find("could not be converted")
                != std::string_view::npos
            || message.find("could not be locked") != std::string_view::npos)
        {
            return GroundTextureLoadState::UnsupportedImage;
        }
        if (message.find("unsupported size") != std::string_view::npos
            || message.find("mip upload size")
                != std::string_view::npos)
        {
            return GroundTextureLoadState::InvalidImage;
        }
        return GroundTextureLoadState::LoadFailed;
    }

    std::uint32_t groundTextureMipLevelCount(
        const std::uint32_t width,
        const std::uint32_t height) noexcept
    {
        if (width == 0 || height == 0)
        {
            return 0;
        }
        const std::uint32_t largest = std::max(width, height);
        std::uint32_t levels = 1;
        // One more level exists while the last included one is still larger
        // than a single texel, so the chain always ends at 1x1.
        while (levels < maximumMipLevels
            && (largest >> (levels - 1)) > 1u)
        {
            ++levels;
        }
        return levels;
    }

    void validateGroundSurfaceSettings(const GroundSurfaceSettings& settings)
    {
        const auto requireFinite = [](const float value, const char* name)
        {
            if (!std::isfinite(value))
            {
                throw std::invalid_argument(
                    std::string("Ground surface ") + name
                    + " must be finite.");
            }
        };
        const auto requireExtent = [requireFinite](
            const float value, const char* name)
        {
            requireFinite(value, name);
            if (value < minimumGroundExtent || value > maximumGroundExtent)
            {
                throw std::invalid_argument(
                    std::string("Ground surface ") + name
                    + " must be within [1, 100000] coordinate units.");
            }
        };

        requireExtent(settings.sizeX, "size X");
        requireExtent(settings.sizeY, "size Y");
        requireFinite(settings.elevation, "elevation");
        if (std::abs(settings.elevation) > maximumGroundElevation)
        {
            throw std::invalid_argument(
                "Ground surface elevation must be within +/-1000000 units.");
        }
        if (!finiteVector(settings.baseColor))
        {
            throw std::invalid_argument(
                "Ground surface base color must be finite.");
        }
        const auto requireFactor = [requireFinite](
            const float value, const char* name)
        {
            requireFinite(value, name);
            if (value < 0.0F || value > 1.0F)
            {
                throw std::invalid_argument(
                    std::string("Ground surface ") + name
                    + " must be within [0, 1].");
            }
        };
        requireFactor(settings.metallic, "metallic");
        requireFactor(settings.roughness, "roughness");
        const auto requireTiling = [requireFinite](
            const float value, const char* name)
        {
            requireFinite(value, name);
            if (value <= 0.0F || value > maximumGroundTiling)
            {
                throw std::invalid_argument(
                    std::string("Ground surface ") + name
                    + " tiling must be within (0, 4096].");
            }
        };
        requireTiling(settings.uvTiling.x, "X");
        requireTiling(settings.uvTiling.y, "Y");

        for (const std::string* const identifier : {
            &settings.albedoTexture,
            &settings.normalTexture,
            &settings.roughnessTexture})
        {
            if (identifier->empty())
            {
                continue;
            }
            // Throws with the same message the editor's picker uses, so a
            // rejected value never reaches the renderer as a partial update.
            static_cast<void>(
                normalizeGroundTextureAssetIdentifier(*identifier));
        }
    }

    GroundSurfaceMesh createGroundSurfaceMesh(
        const GroundSurfaceSettings& settings)
    {
        validateGroundSurfaceSettings(settings);

        // Four corners and two triangles are the whole M0 surface. Tiling is
        // baked into the UVs so the fragment shader needs no extra uniform and
        // resizing never leaves a stale repeat count on the GPU.
        const float halfX = settings.sizeX * 0.5F;
        const float halfY = settings.sizeY * 0.5F;
        const float elevation = settings.elevation;
        const glm::vec2 tiling{settings.uvTiling};

        GroundSurfaceMesh mesh;
        mesh.vertices = {
            {{-halfX, -halfY, elevation}, {0.0F, 0.0F}},
            {{ halfX, -halfY, elevation}, {tiling.x, 0.0F}},
            {{ halfX,  halfY, elevation}, {tiling.x, tiling.y}},
            {{-halfX,  halfY, elevation}, {0.0F, tiling.y}}
        };
        // Counter-clockwise seen from +Z, matching the track pipeline's
        // front-face convention even though the ground is not culled.
        mesh.indices = {0, 1, 2, 0, 2, 3};
        return mesh;
    }

    GroundTextureImage loadGroundTextureImage(
        const std::filesystem::path& path,
        const bool srgbEncoded)
    {
        std::error_code error;
        if (!std::filesystem::is_regular_file(path, error))
        {
            throw std::runtime_error(
                "Ground texture file not found: " + path.string());
        }

        const auto utf8Path = path.u8string();
        SDL_Surface* const decoded = SDL_LoadPNG(
            reinterpret_cast<const char*>(utf8Path.c_str()));
        if (decoded == nullptr)
        {
            throw std::runtime_error(
                "Ground texture is not a readable PNG: " + path.string()
                + " (" + SDL_GetError() + ")");
        }
        const std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)>
            decodedOwner{decoded, SDL_DestroySurface};

        if (decoded->w <= 0 || decoded->h <= 0
            || static_cast<std::uint32_t>(decoded->w) > maximumTextureExtent
            || static_cast<std::uint32_t>(decoded->h) > maximumTextureExtent)
        {
            throw std::runtime_error(
                "Ground texture has an unsupported size: " + path.string());
        }

        // One normalized layout keeps the copy below independent of whichever
        // channel layout the PNG decoder chose.
        SDL_Surface* const converted = SDL_ConvertSurface(
            decoded, SDL_PIXELFORMAT_RGBA32);
        if (converted == nullptr)
        {
            throw std::runtime_error(
                "Ground texture could not be converted to RGBA: "
                + path.string() + " (" + SDL_GetError() + ")");
        }
        const std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)>
            convertedOwner{converted, SDL_DestroySurface};

        const std::uint32_t width =
            static_cast<std::uint32_t>(converted->w);
        const std::uint32_t height =
            static_cast<std::uint32_t>(converted->h);
        if (SDL_MUSTLOCK(converted) && !SDL_LockSurface(converted))
        {
            throw std::runtime_error(
                "Ground texture could not be locked: " + path.string()
                + " (" + SDL_GetError() + ")");
        }

        GroundTextureImage image;
        try
        {
            image.width = width;
            image.height = height;
            image.mipLevels = groundTextureMipLevelCount(width, height);
            image.srgb = srgbEncoded;
            image.rgba.assign(
                mipLevelOffset(width, height, image.mipLevels), 0);
            std::memcpy(image.rgba.data(), converted->pixels,
                static_cast<std::size_t>(width) * height * 4);
        }
        catch (...)
        {
            if (SDL_MUSTLOCK(converted))
            {
                SDL_UnlockSurface(converted);
            }
            throw;
        }
        if (SDL_MUSTLOCK(converted))
        {
            SDL_UnlockSurface(converted);
        }

        for (std::uint32_t level = 1; level < image.mipLevels; ++level)
        {
            reduceMipLevel(
                image.rgba.data()
                    + mipLevelOffset(width, height, level - 1),
                std::max(1u, width >> (level - 1)),
                std::max(1u, height >> (level - 1)),
                image.rgba.data() + mipLevelOffset(width, height, level),
                std::max(1u, width >> level),
                std::max(1u, height >> level),
                srgbEncoded);
        }
        return image;
    }
}
