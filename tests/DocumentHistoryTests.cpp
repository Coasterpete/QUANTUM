#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/editor/AuthoredTrackEditTransaction.hpp>
#include <quantum/editor/DocumentHistory.hpp>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::editor::AuthoredTrackEditTransaction;
    using quantum::editor::DocumentHistory;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    [[nodiscard]] std::string snapshot(const AuthoredTrack& track)
    {
        return quantum::coaster::serializeCoasterDocument(track);
    }

    [[nodiscard]] AuthoredTrack requireState(
        std::optional<AuthoredTrack> state,
        const std::string_view message)
    {
        require(state.has_value(), message);
        return std::move(*state);
    }

    void singleEditUndoRedoIsExact()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        DocumentHistory history;
        history.reset(track);
        const std::string baseline = snapshot(track);

        track.appendSection();
        history.record(track);
        const std::string edited = snapshot(track);

        require(history.canUndo() && !history.canRedo(),
            "one edit enables Undo only");
        require(snapshot(requireState(history.undo(), "Undo state missing"))
                == baseline,
            "Undo must restore the exact baseline document");
        require(snapshot(requireState(history.redo(), "Redo state missing"))
                == edited,
            "Redo must restore the exact edited document");
    }

    void sequentialEditsAndBranching()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        DocumentHistory history;
        history.reset(track);

        track.appendSection();
        history.record(track);
        const std::string first = snapshot(track);
        track.appendSection();
        history.record(track);

        track = requireState(history.undo(), "first sequential Undo missing");
        require(snapshot(track) == first,
            "sequential Undo must restore the preceding edit");

        track.setLayoutMode(quantum::coaster::LayoutMode::Shuttle);
        history.record(track);
        require(!history.canRedo(),
            "a new accepted edit after Undo must clear Redo");
    }

    void representativeStructuralAndProfileEditsUndo()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        DocumentHistory history;
        history.reset(track);
        const std::string baseline = snapshot(track);

        track.appendSection();
        history.record(track);
        track.removeSection(1);
        history.record(track);
        require(snapshot(requireState(history.undo(), "remove Undo missing"))
                != baseline,
            "Undoing remove must restore the added region");
        require(snapshot(requireState(history.undo(), "add Undo missing"))
                == baseline,
            "Undoing add must restore the original region list");

        history.reset(track);
        auto& pitch = track.section(0).rateProfileRegion().rateProfiles.pitch;
        const auto segmentId = pitch.segments.front().id;
        quantum::coaster::setChannelSegmentValue(
            pitch,
            segmentId,
            quantum::coaster::ProfileBoundary::End,
            0.125
        );
        history.record(track);
        require(snapshot(requireState(history.undo(), "profile Undo missing"))
                == baseline,
            "profile-value Undo must restore the exact authored profile");

        history.reset(track);
        const std::string beforeSplit = snapshot(track);
        static_cast<void>(quantum::coaster::splitChannelSegment(
            pitch, segmentId, 30.0));
        history.record(track);
        require(snapshot(requireState(history.undo(), "split Undo missing"))
                == beforeSplit,
            "marker split Undo must restore the unsplit profile");
    }

    void rejectedTransactionDoesNotEnterHistory()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        DocumentHistory history;
        history.reset(track);

        AuthoredTrackEditTransaction transaction{track};
        bool rejected = false;
        try
        {
            quantum::coaster::setSectionLength(
                transaction.candidate().section(0), -1.0);
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }

        require(rejected && !transaction.committed(),
            "invalid transaction must be rejected before commit");
        require(history.size() == 1 && !history.canUndo(),
            "a rejected edit must not create a history entry");
    }

    void dirtyStateTracksSavedRevision()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        DocumentHistory history;
        history.reset(track);
        require(!history.isDirty(), "new baseline must be clean");

        track.appendSection();
        history.record(track);
        require(history.isDirty(), "an edit after Save must be dirty");
        static_cast<void>(history.undo());
        require(!history.isDirty(),
            "Undo to the original saved revision must be clean");
        static_cast<void>(history.redo());
        require(history.isDirty(),
            "Redo away from the original saved revision must be dirty");

        history.markSaved();
        require(!history.isDirty(), "saved revision must be clean");

        static_cast<void>(history.undo());
        require(history.isDirty(), "Undo away from saved revision must be dirty");
        static_cast<void>(history.redo());
        require(!history.isDirty(), "Redo to exact saved revision must be clean");

        static_cast<void>(history.undo());
        track = quantum::coaster::createNewDocument();
        track.setLayoutMode(quantum::coaster::LayoutMode::Shuttle);
        history.record(track);
        require(history.isDirty() && !history.canRedo(),
            "branching away from saved Redo must remain dirty");
    }

    void newOpenResetAndContinuousCoalescing()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        DocumentHistory history;
        history.reset(track);

        track.appendSection();
        history.record(track);
        AuthoredTrack opened = quantum::coaster::createDefaultAuthoredTrack();
        history.reset(opened);
        require(!history.canUndo() && !history.canRedo() && !history.isDirty(),
            "New/Open reset must establish an isolated clean baseline");

        const std::string openedSnapshot = snapshot(opened);
        auto& pitch = opened.section(0).rateProfileRegion().rateProfiles.pitch;
        const auto segmentId = pitch.segments.front().id;
        for (const double value : {0.01, 0.02, 0.03, 0.04})
        {
            quantum::coaster::setChannelSegmentValue(
                pitch,
                segmentId,
                quantum::coaster::ProfileBoundary::End,
                value
            );
            history.record(opened, true);
        }
        history.endContinuousEdit();

        require(history.size() == 2,
            "one continuous gesture must create one Undo entry");
        require(snapshot(requireState(history.undo(), "drag Undo missing"))
                == openedSnapshot,
            "coalesced drag Undo must restore its pre-gesture state");
    }
}

int main()
{
    try
    {
        singleEditUndoRedoIsExact();
        sequentialEditsAndBranching();
        representativeStructuralAndProfileEditsUndo();
        rejectedTransactionDoesNotEnterHistory();
        dirtyStateTracksSavedRevision();
        newOpenResetAndContinuousCoalescing();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Document history test failure: " << exception.what()
                  << '\n';
        return 1;
    }

    std::cout << "Document history tests passed.\n";
    return 0;
}
