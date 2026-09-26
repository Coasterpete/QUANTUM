#include <quantum/renderer/EnvironmentAssets.hpp>

#include <algorithm>
#include <stdexcept>

namespace quantum::renderer
{
    std::vector<EnvironmentAsset> bundledEnvironmentAssets()
    {
        // The bundled skies are CC0. ambientCG no longer publishes Radiance
        // .hdr files, so its OpenEXR panorama is converted once by
        // tools/convert_ambientcg_hdri_to_radiance.py; the runtime decoder is
        // unchanged. See docs/ground-surface-m0.md.
        return {
            {"assets://environment/rooitou_park_1k.hdr",
                "Rooitou Park (CC0)",
                "Rooitou Park by Greg Zaal - Poly Haven, CC0"},
            {"assets://environment/dayskyhdri027b_1k.hdr",
                "Day Sky HDRI 027 B (CC0)",
                "Day Sky HDRI 027 B - ambientCG, CC0"},
        };
    }

    const std::vector<EnvironmentAsset>& environmentAssetRegistry()
    {
        // A function-local static so the registry is built once and outlives
        // every lookup. Returning a pointer into a by-value vector instead
        // would dangle as soon as the temporary died.
        static const std::vector<EnvironmentAsset> registry =
            bundledEnvironmentAssets();
        return registry;
    }

    const EnvironmentAsset* findBundledEnvironmentAsset(
        const std::string_view identifier) noexcept
    {
        if (identifier.empty())
        {
            return nullptr;
        }
        for (const EnvironmentAsset& asset : environmentAssetRegistry())
        {
            if (asset.identifier == identifier)
            {
                return &asset;
            }
        }
        return nullptr;
    }

    std::string validateEnvironmentAssetIdentifier(
        const std::string_view identifier)
    {
        if (identifier.empty())
        {
            return {};
        }
        if (const EnvironmentAsset* asset =
                findBundledEnvironmentAsset(identifier))
        {
            return asset->identifier;
        }
        throw std::invalid_argument(
            "Unknown bundled HDR environment identifier: " + std::string(identifier));
    }
}
