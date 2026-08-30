#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/RiderLoads.hpp>

#include <cstddef>
#include <optional>
#include <utility>

namespace quantum::editor
{
    // Owns the candidate document and defers editor-visible transaction
    // effects until the candidate has become the committed document.
    class AuthoredTrackEditTransaction
    {
    public:
        explicit AuthoredTrackEditTransaction(
            const coaster::AuthoredTrack& committedTrack)
            : candidateTrack_(committedTrack)
        {
        }

        [[nodiscard]] coaster::AuthoredTrack& candidate() noexcept
        {
            return candidateTrack_;
        }

        void requestSectionLengthBufferSync() noexcept
        {
            sectionLengthBufferSyncRequested_ = true;
        }

        void requestRegionBufferSync() noexcept
        {
            regionBufferSyncRequested_ = true;
        }

        void requestProfileValueBufferSync() noexcept
        {
            profileValueBufferSyncRequested_ = true;
        }

        [[nodiscard]] bool sectionLengthBufferSyncRequested() const noexcept
        {
            return sectionLengthBufferSyncRequested_;
        }

        [[nodiscard]] bool regionBufferSyncRequested() const noexcept
        {
            return regionBufferSyncRequested_;
        }

        [[nodiscard]] bool profileValueBufferSyncRequested() const noexcept
        {
            return profileValueBufferSyncRequested_;
        }

        void stageSelectionAfterCommit(const std::size_t sectionIndex) noexcept
        {
            stagedSelection_ = sectionIndex;
        }

        [[nodiscard]] std::optional<std::size_t>
        selectionAfterCommit() const noexcept
        {
            return committed_ ? stagedSelection_ : std::nullopt;
        }

        void commit(coaster::AuthoredTrack& committedTrack)
        {
            committedTrack = std::move(candidateTrack_);
            committed_ = true;
        }

        // Called after candidate generation/evaluation and before any GPU or
        // editor publication. Legacy unreachable-track acceptance is unchanged.
        void requireAcceptableRiderLoads(const coaster::RiderLoadHistory& history) const
        {
            if (coaster::hasForceDrivenRegions(candidateTrack_)
                && (!history.completed() || history.states.empty()))
            {
                throw std::invalid_argument(
                    "Force-driven candidates require a completed rider-load evaluation.");
            }
        }

        [[nodiscard]] bool committed() const noexcept
        {
            return committed_;
        }

    private:
        coaster::AuthoredTrack candidateTrack_;
        std::optional<std::size_t> stagedSelection_;
        bool sectionLengthBufferSyncRequested_ = false;
        bool regionBufferSyncRequested_ = false;
        bool profileValueBufferSyncRequested_ = false;
        bool committed_ = false;
    };
}
