#pragma once

#include <filesystem>
#include <expected>
#include <optional>
#include <string>

struct SDL_Window;

namespace quantum::editor
{
    // Prompts the user to select a file for opening.
    // Returns the selected path, or nullopt when the user cancels.
    [[nodiscard]] std::optional<std::filesystem::path>
    openFileDialog(SDL_Window* window);

    // Prompts the user to choose a destination path for saving.
    // Returns the selected path, or nullopt when the user cancels.
    [[nodiscard]] std::optional<std::filesystem::path>
    saveFileDialog(SDL_Window* window);

    [[nodiscard]] std::optional<std::filesystem::path>
    openTrackHardwareFileDialog(SDL_Window* window);

    // Ground Surface M0 decodes PNG only, matching the accepted ground
    // identifier grammar.
    [[nodiscard]] std::optional<std::filesystem::path>
    openGroundTextureFileDialog(SDL_Window* window);

    // Converts a transient picker result below <runtimeRoot>/assets to
    // its package-relative authored identity. Absolute paths never escape
    // this boundary into the document.
    [[nodiscard]] std::expected<std::string, std::string>
    trackHardwareAssetIdFromPath(
        const std::filesystem::path& selectedPath,
        const std::filesystem::path& runtimeRoot);

    // Same conversion for a ground map, additionally pinned to the
    // assets://ground/ package root.
    [[nodiscard]] std::expected<std::string, std::string>
    groundTextureAssetIdFromPath(
        const std::filesystem::path& selectedPath,
        const std::filesystem::path& runtimeRoot);
}
