#pragma once

#include <filesystem>
#include <optional>

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
}
