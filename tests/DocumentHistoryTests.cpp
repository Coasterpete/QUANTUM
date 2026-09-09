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
#include <utility>

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

    void trackHardwareEditsUndoAndRedo()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        DocumentHistory history;
        history.reset(track);

        auto setHardware = [&](const auto& edit, const bool continuous = false)
        {
            auto style = track.trackStyle();
            edit(style.repeatingHardware.front());
            track.setTrackStyle(style);
            history.record(track, continuous);
        };

        setHardware([](auto& hardware) {
            hardware.asset.path =
                "assets://track/standard-dual-rail/crosstie.glb";
            hardware.asset.placeholder = false;
        });
        setHardware([](auto& hardware) { hardware.spacing = 2.25; });
        setHardware([](auto& hardware) {
            hardware.localPosition = {0.1, -0.2, 0.3};
        });
        setHardware([](auto& hardware) {
            hardware.localRotation = {0.2, 0.3, 0.4};
        });
        setHardware([](auto& hardware) {
            hardware.localScale = {1.1, 1.2, 1.3};
        });

        AuthoredTrack state = requireState(history.undo(), "scale Undo missing");
        require(state.trackStyle().repeatingHardware.front().localScale
                == glm::dvec3{1.0},
            "hardware scale Undo must restore the prior value");
        state = requireState(history.undo(), "rotation Undo missing");
        require(state.trackStyle().repeatingHardware.front().localRotation
                == glm::dvec3{0.0},
            "hardware rotation Undo must restore the prior value");
        state = requireState(history.undo(), "position Undo missing");
        require(state.trackStyle().repeatingHardware.front().localPosition
                == glm::dvec3{0.0, 0.0, -0.11},
            "hardware position Undo must restore the preset value");
        state = requireState(history.undo(), "spacing Undo missing");
        require(state.trackStyle().repeatingHardware.front().spacing == 1.5,
            "hardware spacing Undo must restore the preset value");
        state = requireState(history.undo(), "asset Undo missing");
        require(state.trackStyle().repeatingHardware.front().asset.path
                == "assets://track/test-crosstie-placeholder.glb",
            "hardware asset Undo must restore the preset logical ID");
        state = requireState(history.redo(), "asset Redo missing");
        require(state.trackStyle().repeatingHardware.front().asset.path
                == "assets://track/standard-dual-rail/crosstie.glb",
            "hardware asset Redo must restore the authored logical ID");

        history.reset(state);
        track = state;
        for (const double spacing : {1.7, 1.9, 2.1})
        {
            setHardware([spacing](auto& hardware) {
                hardware.spacing = spacing;
            }, true);
        }
        history.endContinuousEdit();
        require(history.size() == 2,
            "one hardware drag must coalesce into one history entry");
    }

    void supportNodeEditsUseWholeDocumentHistory()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        quantum::coaster::SupportCollection supports;
        const auto structureId =
            quantum::coaster::allocateSupportStructureId(supports);
        supports.structures.push_back({structureId, "History"});
        auto& structure = supports.structures.back();
        const auto nodeId =
            quantum::coaster::allocateSupportElementId(structure);
        structure.nodes.push_back({nodeId, {1.0, 2.0, 3.0}});
        track.setSupports(supports);

        DocumentHistory history;
        history.reset(track);
        track.setSupportNodePosition(structureId, nodeId, {4.0, 5.0, 6.0});
        history.record(track);

        const AuthoredTrack undone = requireState(
            history.undo(), "support-node Undo missing");
        require(undone.supports().structures[0].nodes[0].position
                == glm::dvec3{1.0, 2.0, 3.0},
            "support-node Undo must restore the complete document snapshot");
        const AuthoredTrack redone = requireState(
            history.redo(), "support-node Redo missing");
        require(redone.supports().structures[0].nodes[0].position
                == glm::dvec3{4.0, 5.0, 6.0},
            "support-node Redo must restore the accepted edit");
    }

    void supportGraphAuthoringUndoRedoIsExact()
    {
        using quantum::coaster::SupportCollection;
        using quantum::coaster::SupportStructureId;

        AuthoredTrack track = quantum::coaster::createNewDocument();
        DocumentHistory history;
        history.reset(track);

        auto publish = [&](auto&& mutation)
        {
            AuthoredTrackEditTransaction transaction{track};
            std::forward<decltype(mutation)>(mutation)(transaction);
            transaction.commit(track);
            history.record(track);
        };

        SupportStructureId structureId = 0;
        publish([&](AuthoredTrackEditTransaction& transaction) {
            structureId = transaction.candidate().createSupportStructure(
                "Undo frame");
        });
        const std::string structureOnly = snapshot(track);

        const auto applyNodesAndMember = [&](
            AuthoredTrackEditTransaction& transaction)
        {
            const auto left = transaction.candidate().createSupportNode(
                structureId, {-2.0, 0.0, 0.0});
            const auto right = transaction.candidate().createSupportNode(
                structureId, {2.0, 0.0, 0.0});
            transaction.candidate().createSupportMember(
                structureId, left, right);
        };
        publish(applyNodesAndMember);
        const std::string connected = snapshot(track);

        const auto removeMember = [&](AuthoredTrackEditTransaction& transaction)
        {
            const auto& structure = transaction.candidate()
                .supports().structures.front();
            transaction.candidate().removeSupportMember(
                structureId, structure.members.front().id);
        };
        publish(removeMember);

        track = requireState(history.undo(), "member-delete Undo missing");
        require(snapshot(track) == connected,
            "Undo of member deletion must restore the connected pair");
        track = requireState(history.undo(), "member-create Undo missing");
        require(snapshot(track) == structureOnly,
            "Undo of member creation must keep the empty structure");
        track = requireState(history.undo(), "structure Undo missing");
        require(snapshot(track) == snapshot(
                quantum::coaster::createNewDocument()),
            "Undo of structure creation must restore the empty document");
        track = requireState(history.redo(), "structure Redo missing");
        track = requireState(history.redo(), "member Redo missing");
        require(snapshot(track) == connected,
            "Redo must restore the graph exactly as authored");
    }

    void supportAnchorMetadataUsesWholeDocumentHistory()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        const auto structureId = track.createSupportStructure("Anchors");
        const auto attachedId = track.createSupportNode(
            structureId, {1.0, 2.0, 3.0});
        const auto foundationId = track.createSupportNode(
            structureId, {4.0, 5.0, 6.0});

        DocumentHistory history;
        history.reset(track);
        {
            AuthoredTrackEditTransaction transaction{track};
            transaction.candidate().setSupportTrackAttachment(
                structureId, attachedId, {12.0, -1.0, 2.0});
            transaction.commit(track);
            history.record(track);
        }
        require(!requireState(history.undo(), "attachment Undo missing")
                    .supports().structures[0].nodes[0]
                    .trackAttachment.has_value(),
            "attachment Undo must restore the unanchored node");
        track = requireState(history.redo(), "attachment Redo missing");
        require(track.supports().structures[0].nodes[0].trackAttachment
                    == quantum::coaster::TrackAttachment{12.0, -1.0, 2.0},
            "attachment Redo must restore exact authored metadata");

        history.reset(track);
        {
            AuthoredTrackEditTransaction transaction{track};
            transaction.candidate().setSupportFoundation(
                structureId, foundationId);
            transaction.commit(track);
            history.record(track);
        }
        require(!requireState(history.undo(), "foundation Undo missing")
                    .supports().structures[0].nodes[1]
                    .foundation.has_value(),
            "foundation Undo must restore the unanchored node");
        track = requireState(history.redo(), "foundation Redo missing");
        require(track.supports().structures[0].nodes[1]
                    .foundation.has_value(),
            "foundation Redo must restore exact authored metadata");
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
        trackHardwareEditsUndoAndRedo();
        supportNodeEditsUseWholeDocumentHistory();
        supportGraphAuthoringUndoRedoIsExact();
        supportAnchorMetadataUsesWholeDocumentHistory();
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
