#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace quantum::editor
{
    inline constexpr std::size_t defaultDocumentHistoryCapacity = 128;

    // Bounded history of accepted authoritative document states. Each entry
    // has a revision identity so dirty state can follow the history cursor
    // exactly instead of treating every Undo/Redo as a new modification.
    class DocumentHistory
    {
    public:
        explicit DocumentHistory(
            std::size_t capacity = defaultDocumentHistoryCapacity);

        // New/Open establish one clean baseline and discard every state from
        // the previous document.
        void reset(const coaster::AuthoredTrack& track);

        // Records an accepted edit. Continuous commits replace the current
        // gesture's latest state while retaining its original pre-gesture
        // Undo state.
        void record(
            const coaster::AuthoredTrack& track,
            bool continuous = false
        );
        void endContinuousEdit() noexcept;

        [[nodiscard]] std::optional<coaster::AuthoredTrack> undo();
        [[nodiscard]] std::optional<coaster::AuthoredTrack> redo();

        // Save/Save As identify the exact current revision as clean.
        void markSaved() noexcept;

        [[nodiscard]] bool canUndo() const noexcept;
        [[nodiscard]] bool canRedo() const noexcept;
        [[nodiscard]] bool isDirty() const noexcept;
        [[nodiscard]] std::size_t size() const noexcept;

    private:
        struct Entry
        {
            coaster::AuthoredTrack track;
            std::uint64_t revision = 0;
        };

        void append(const coaster::AuthoredTrack& track);

        std::vector<Entry> entries_;
        std::size_t cursor_ = 0;
        std::size_t capacity_ = defaultDocumentHistoryCapacity;
        std::uint64_t nextRevision_ = 1;
        std::optional<std::uint64_t> savedRevision_;
        bool continuousEditActive_ = false;
    };
}
