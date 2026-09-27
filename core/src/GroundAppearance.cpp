#include <quantum/coaster/GroundAppearance.hpp>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <initializer_list>
#include <limits>
#include <ranges>
#include <stdexcept>

namespace quantum::coaster
{
    std::string normalizeGroundTextureAssetIdentifier(
        const std::string_view identifier)
    {
        std::string normalized{identifier};
        std::ranges::replace(normalized, '\\', '/');
        if (!normalized.starts_with("assets://"))
            throw std::invalid_argument("Ground texture must use assets://ground/: " + normalized);
        if (normalized.find(':', 9) != std::string::npos
            || normalized.find('\0') != std::string::npos)
            throw std::invalid_argument("Ground texture contains a machine-specific or invalid path: " + normalized);
        const std::filesystem::path relative = std::filesystem::path(
            normalized.substr(9)).lexically_normal();
        if (relative.is_absolute() || relative.has_root_name()
            || relative.has_root_directory() || relative.empty()
            || relative.begin() == relative.end()
            || relative.begin()->generic_string() != "ground")
            throw std::invalid_argument("Ground texture must live below assets://ground/: " + normalized);
        for (const auto& part : relative)
            if (part == "..")
                throw std::invalid_argument("Ground texture cannot escape package root: " + normalized);
        std::string suffix = relative.extension().string();
        std::ranges::transform(suffix, suffix.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (suffix != ".png")
            throw std::invalid_argument("Ground texture must be PNG: " + normalized);
        return "assets://" + relative.generic_string();
    }

    void validateGroundAppearance(const GroundAppearance& ground)
    {
        const auto range = [](float value, float minimum, float maximum,
            const char* name)
        {
            if (!std::isfinite(value) || value < minimum || value > maximum)
                throw std::invalid_argument(std::string("Ground ") + name + " is out of range");
        };
        range(ground.elevation, -1000000.0F, 1000000.0F, "elevation");
        range(ground.sizeX, 1.0F, 100000.0F, "size X");
        range(ground.sizeY, 1.0F, 100000.0F, "size Y");
        for (float component : {ground.baseColor.x, ground.baseColor.y,
            ground.baseColor.z, ground.baseColor.w})
            if (!std::isfinite(component))
                throw std::invalid_argument("Ground base color must be finite");
        range(ground.metallic, 0.0F, 1.0F, "metallic");
        range(ground.roughness, 0.0F, 1.0F, "roughness");
        range(ground.uvTiling.x, std::numeric_limits<float>::min(), 4096.0F, "UV X");
        range(ground.uvTiling.y, std::numeric_limits<float>::min(), 4096.0F, "UV Y");
        for (const std::string* id : {&ground.albedoTexture,
            &ground.normalTexture, &ground.roughnessTexture})
            if (!id->empty())
                static_cast<void>(normalizeGroundTextureAssetIdentifier(*id));
    }
}
