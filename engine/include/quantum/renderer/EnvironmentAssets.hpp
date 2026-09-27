#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace quantum::renderer
{    // Renderer-neutral identity of a bundled HDR sky. The identifier is
    // package-relative ("assets://environment/<file>.hdr") so it never carries
    // a machine-specific path, and the empty string means "no environment",
    // which keeps the renderer-neutral constant ambient term.
    struct EnvironmentAsset
    {
        std::string identifier;
        std::string displayName;
        std::string credit;
    };

    // The bundled environments, in the order the Editor presents them. Adding a
    // sky is a data change: extend this list, stage the file, and it becomes
    // selectable without touching the HDR decode or the IBL shader.
    [[nodiscard]] std::vector<EnvironmentAsset> bundledEnvironmentAssets();

    // The same data, owned by a function-local static so lookups can hand out
    // a pointer that stays valid. Do not build one per call and return a
    // pointer into it.
    [[nodiscard]] const std::vector<EnvironmentAsset>&
        environmentAssetRegistry();

    // Looks up a bundled environment by identifier. Returns nullopt for an
    // unknown identifier, including the empty "no environment" selection.
    [[nodiscard]] const EnvironmentAsset* findBundledEnvironmentAsset(
        std::string_view identifier) noexcept;

    // A stable identity for the environment used when nothing is selected or
    // the selected asset cannot be loaded.
    inline constexpr std::string_view fallbackEnvironmentIdentifier =
        "builtin://environment/constant-ambient";

    // The Editor's neutral new-session sky. Keep this independent of registry
    // ordering so presentation defaults do not change when assets are added or
    // rearranged.
    inline constexpr std::string_view defaultEnvironmentAssetIdentifier =
        "assets://environment/dayskyhdri027b_1k.hdr";

    // Throws std::invalid_argument for an identifier that is not a bundled
    // environment, so an unsupported value can never reach the renderer.
    [[nodiscard]] std::string validateEnvironmentAssetIdentifier(
        std::string_view identifier);
}
