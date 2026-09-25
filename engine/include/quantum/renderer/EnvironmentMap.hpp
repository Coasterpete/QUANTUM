#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

namespace quantum::renderer
{
    // Linear RGBA32F pixels. Cubemap faces use Vulkan's +X,-X,+Y,-Y,+Z,-Z order.
    struct EnvironmentPixels
    {
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        std::uint32_t layers = 1;
        std::uint32_t mipLevels = 1;
        std::vector<float> rgba;
    };

    struct ProcessedEnvironment
    {
        EnvironmentPixels sky;
        EnvironmentPixels irradiance;
        EnvironmentPixels specular;
        EnvironmentPixels brdf;
    };

    [[nodiscard]] ProcessedEnvironment preprocessEnvironment(
        const std::filesystem::path& hdrPath);
}
