#include <quantum/editor/DocumentHistory.hpp>

#include <stdexcept>

namespace quantum::editor
{
    DocumentHistory::DocumentHistory(const std::size_t capacity)
        : capacity_(capacity)
    {
        if (capacity_ < 2)
        {
            throw std::invalid_argument(
                "Document history capacity must hold at least two states."
            );
        }
    }

    void DocumentHistory::reset(const coaster::AuthoredTrack& track)
    {
        entries_.clear();
        cursor_ = 0;
        continuousEditActive_ = false;
        entries_.push_back(Entry{track, nextRevision_++});
        savedRevision_ = entries_.front().revision;
    }

    void DocumentHistory::record(
        const coaster::AuthoredTrack& track,
        const bool continuous)
    {
        if (entries_.empty())
        {
            throw std::logic_error(
                "Document history must be reset before recording edits."
            );
        }

        if (continuous && continuousEditActive_)
        {
            entries_[cursor_] = Entry{track, nextRevision_++};
            return;
        }

        if (!continuous)
        {
            continuousEditActive_ = false;
        }

        append(track);
        continuousEditActive_ = continuous;
    }

    void DocumentHistory::endContinuousEdit() noexcept
    {
        continuousEditActive_ = false;
    }

    std::optional<coaster::AuthoredTrack> DocumentHistory::undo()
    {
        endContinuousEdit();
        if (!canUndo())
        {
            return std::nullopt;
        }

        --cursor_;
        return entries_[cursor_].track;
    }

    std::optional<coaster::AuthoredTrack> DocumentHistory::redo()
    {
        endContinuousEdit();
        if (!canRedo())
        {
            return std::nullopt;
        }

        ++cursor_;
        return entries_[cursor_].track;
    }

    void DocumentHistory::markSaved() noexcept
    {
        endContinuousEdit();
        if (!entries_.empty())
        {
            savedRevision_ = entries_[cursor_].revision;
        }
    }

    bool DocumentHistory::canUndo() const noexcept
    {
        return !entries_.empty() && cursor_ > 0;
    }

    bool DocumentHistory::canRedo() const noexcept
    {
        return !entries_.empty() && cursor_ + 1 < entries_.size();
    }

    bool DocumentHistory::isDirty() const noexcept
    {
        return entries_.empty() || !savedRevision_.has_value()
            || entries_[cursor_].revision != *savedRevision_;
    }

    std::size_t DocumentHistory::size() const noexcept
    {
        return entries_.size();
    }

    void DocumentHistory::append(const coaster::AuthoredTrack& track)
    {
        entries_.erase(entries_.begin() + static_cast<std::ptrdiff_t>(cursor_ + 1),
            entries_.end());
        entries_.push_back(Entry{track, nextRevision_++});
        cursor_ = entries_.size() - 1;

        if (entries_.size() > capacity_)
        {
            const std::size_t excess = entries_.size() - capacity_;
            entries_.erase(entries_.begin(),
                entries_.begin() + static_cast<std::ptrdiff_t>(excess));
            cursor_ -= excess;
        }
    }
}
