#include <quantum/coaster/StaticMeshAsset.hpp>

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>

namespace quantum::coaster
{
    std::string normalizeStaticMeshAssetIdentifier(
        const std::string_view identifier,
        const std::string_view packageRoot)
    {
        constexpr std::string_view assetsScheme = "assets://";

        if (identifier.empty())
        {
            throw std::invalid_argument(
                "A static mesh asset identifier cannot be empty.");
        }
        if (packageRoot.empty())
        {
            throw std::invalid_argument(
                "A static mesh asset identifier requires a package root.");
        }

        std::string normalized{identifier};
        std::ranges::replace(normalized, '\\', '/');
        if (!normalized.starts_with(assetsScheme))
        {
            throw std::invalid_argument(
                "A static mesh asset identifier must use the assets:// scheme.");
        }

        const std::filesystem::path relative = std::filesystem::path(
            normalized.substr(assetsScheme.size())).lexically_normal();
        if (relative.empty() || relative.is_absolute()
            || relative.has_root_name() || relative.has_root_directory())
        {
            throw std::invalid_argument(
                "A static mesh asset identifier must be package-relative.");
        }
        for (const std::filesystem::path& part : relative)
        {
            if (part == "..")
            {
                throw std::invalid_argument(
                    "A static mesh asset identifier cannot escape the asset root.");
            }
        }

        const std::string rootPrefix = std::string(packageRoot) + "/";
        const std::string relativePath = relative.generic_string();
        if (!relativePath.starts_with(rootPrefix))
        {
            throw std::invalid_argument(
                "Static mesh GLBs must be below assets://"
                + std::string(packageRoot) + "/.");
        }
        if (relative.extension() != ".glb")
        {
            throw std::invalid_argument(
                "Static mesh assets must use the .glb extension.");
        }
        return std::string(assetsScheme) + relativePath;
    }
}