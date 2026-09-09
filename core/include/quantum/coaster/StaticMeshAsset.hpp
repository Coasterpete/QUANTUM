#pragma once

#include <string>
#include <string_view>

namespace quantum::coaster
{
    // Renderer-neutral logical identifier for one reusable static mesh. The
    // path is a canonical package-relative logical identity; renderer cache
    // entries, GPU handles, and system file paths never enter Core authored
    // state.
    struct StaticMeshAssetReference
    {
        std::string path;
        // Diagnostic assets are replaceable development stand-ins, not
        // production-authored hardware.
        bool placeholder = false;

        [[nodiscard]] friend bool operator==(
            const StaticMeshAssetReference&,
            const StaticMeshAssetReference&) = default;
    };

    // Returns a canonical package-relative identifier for a logical static
    // mesh below assets://<packageRoot>/. File-backed assets must use the
    // .glb extension. Back/forward separators and redundant "." components
    // are normalized; absolute paths, unknown schemes, and surviving ".."
    // components are rejected.
    //
    // packageRoot is the required first path component immediately below the
    // asset root, for example "track" or "support". Track and support
    // identifiers therefore normalize through one helper without inventing a
    // parallel asset-reference system.
    [[nodiscard]] std::string normalizeStaticMeshAssetIdentifier(
        std::string_view identifier,
        std::string_view packageRoot);
}