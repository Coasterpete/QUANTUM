#pragma once

#include <cstdint>
#include <optional>

namespace quantum::editor
{
    // Simulator is a dedicated mode, separate from the selected editor workspace.
    enum class WorkspaceMode : std::uint8_t
    {
        Editor,
        Simulator
    };

    enum class EditorWorkspace : std::uint8_t
    {
        Track
    };

    inline constexpr EditorWorkspace defaultEditorWorkspace =
        EditorWorkspace::Track;

    // Routing reads only UI values; it does not change the selected workspace
    // or participate in document editing/history.
    [[nodiscard]] constexpr std::optional<EditorWorkspace>
    editorWorkspaceComposition(const WorkspaceMode mode,
        const EditorWorkspace selectedWorkspace) noexcept
    {
        if (mode == WorkspaceMode::Simulator)
            return std::nullopt;
        return selectedWorkspace;
    }
}
