#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>

#include <filesystem>
#include <string>

namespace quantum::editor
{
    // Result of a user-initiated file operation attempt.
    enum class FileOperationResult
    {
        Success,
        Cancelled,
        Failed
    };

    // Confirmation dialog outcome when the document has unsaved changes.
    enum class UnsavedChangesResponse
    {
        Save,
        DontSave,
        Cancel
    };

    // Document workflow metadata: file path and dirty flag.
    // Owns no document data; the Application layer owns the track and
    // calls these methods to keep the metadata consistent.
    class DocumentState
    {
    public:
        DocumentState() = default;

        // Resets state for a new untitled document.
        void newDocument();

        // Resets state for a document loaded from path.
        void setOpenDocument(const std::filesystem::path& path);

        // Updates the current path after Save As.
        void setCurrentPath(const std::filesystem::path& path);

        // Clears the path (e.g. after New).
        void clearPath();

        void markDirty();
        void clearDirty();

        [[nodiscard]] bool isDirty() const noexcept;
        [[nodiscard]] const std::filesystem::path& currentPath() const noexcept;

        // True when the document has been saved at least once.
        [[nodiscard]] bool hasPath() const noexcept;

        // Filename for display (e.g. "MyCoaster.quantum"), or
        // "Untitled" when no path is set.
        [[nodiscard]] std::string displayName() const;

        // Full window title: "QUANTUM — <display> [*]"
        [[nodiscard]] std::string windowTitle() const;

    private:
        std::filesystem::path currentPath_;
        bool dirty_ = false;
    };
}
