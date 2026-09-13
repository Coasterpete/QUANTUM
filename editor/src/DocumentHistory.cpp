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
        lastRestoreImpact_.reset();
        entries_.push_back(Entry{track, nextRevision_++, std::nullopt});
        savedRevision_ = entries_.front().revision;
    }

    void DocumentHistory::record(
        const coaster::AuthoredTrack& track,
        const bool continuous,
        std::optional<TrackStylePresentationImpact> impact)
    {
        if (entries_.empty())
        {
            throw std::logic_error(
                "Document history must be reset before recording edits."
            );
        }

        if (continuous && continuousEditActive_)
        {
            if (entries_[cursor_].trackStylePresentationImpactFromPrevious
                && impact)
            {
                impact = combineTrackStyleImpacts(
                    *entries_[cursor_]
                        .trackStylePresentationImpactFromPrevious,
                    *impact);
            }
            else
            {
                impact.reset();
            }
            entries_[cursor_] = Entry{
                track, nextRevision_++, std::move(impact)};
            return;
        }

        if (!continuous)
        {
            continuousEditActive_ = false;
        }

        append(track, std::move(impact));
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

        lastRestoreImpact_ =
            entries_[cursor_].trackStylePresentationImpactFromPrevious;
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
        lastRestoreImpact_ =
            entries_[cursor_].trackStylePresentationImpactFromPrevious;
        return entries_[cursor_].track;
    }

    std::optional<TrackStylePresentationImpact>
    DocumentHistory::lastRestoreTrackStylePresentationImpact() const noexcept
    {
        return lastRestoreImpact_;
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

    void DocumentHistory::append(
        const coaster::AuthoredTrack& track,
        std::optional<TrackStylePresentationImpact> impact)
    {
        entries_.erase(entries_.begin() + static_cast<std::ptrdiff_t>(cursor_ + 1),
            entries_.end());
        entries_.push_back(Entry{
            track, nextRevision_++, std::move(impact)});
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
