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

    void physicalSettingsEditUsesTransactionAndHistory()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        DocumentHistory history;
        history.reset(track);

        AuthoredTrackEditTransaction transaction{track};
        auto settings = transaction.candidate().physicalSettings();
        settings.initialSpeed = 12.5;
        transaction.candidate().setPhysicalSettings(settings);
        transaction.commit(track);
        history.record(track);

        require(transaction.committed()
                && track.physicalSettings().initialSpeed == 12.5,
            "initial speed edit must commit the authoritative Core setting");
        track = requireState(history.undo(), "physical-settings Undo missing");
        require(track.physicalSettings().initialSpeed == 20.0,
            "physical-settings Undo must restore the document default");
        track = requireState(history.redo(), "physical-settings Redo missing");
        require(track.physicalSettings().initialSpeed == 12.5,
            "physical-settings Redo must restore the authored value");
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
                == glm::dvec3{0.0, 0.0, -0.12},
            "hardware position Undo must restore the preset value");
        state = requireState(history.undo(), "spacing Undo missing");
        require(state.trackStyle().repeatingHardware.front().spacing == 0.75,
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

    void supportMemberEndConnectionsUndoRedoIsExact()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        track.setLayoutMode(quantum::coaster::LayoutMode::Shuttle);
        const auto structureId = track.createSupportStructure("Connected");
        const auto foundationId = track.createSupportNode(
            structureId, {0.0, 0.0, 0.0});
        const auto attachedId = track.createSupportNode(
            structureId, {1.0, 2.0, 3.0});
        const auto memberId = track.createSupportMember(
            structureId, foundationId, attachedId);
        track.setSupportFoundation(structureId, foundationId);
        track.setSupportTrackAttachment(
            structureId, attachedId, {12.0, -1.0, 2.0});

        DocumentHistory history;
        history.reset(track);
        const std::string unconnected = snapshot(track);

        const quantum::coaster::SupportMemberEndConnection connection{
            quantum::coaster::SupportMemberEndTreatment::Footing,
            std::nullopt,
            quantum::coaster::SupportMemberEndPlacement{
                {0.0, 0.1, 0.2}, {1.0, 0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}}};
        {
            AuthoredTrackEditTransaction transaction{track};
            transaction.candidate().setSupportMemberEndConnection(
                structureId, memberId,
                quantum::coaster::SupportMemberEnd::Start, connection);
            transaction.commit(track);
            history.record(track);
        }
        const std::string connected = snapshot(track);
        require(track.supports().structures[0].members[0]
                    .startConnection == connection,
            "the published connection must carry exact normalized metadata");

        AuthoredTrack undone = requireState(
            history.undo(), "connection Undo missing");
        require(snapshot(undone) == unconnected,
            "connection Undo must restore the exact unconnected document");
        AuthoredTrack redone = requireState(
            history.redo(), "connection Redo missing");
        require(snapshot(redone) == connected,
            "connection Redo must restore the exact published document");

        history.reset(redone);
        {
            AuthoredTrackEditTransaction transaction{redone};
            transaction.candidate().clearSupportMemberEndConnection(
                structureId, memberId,
                quantum::coaster::SupportMemberEnd::Start);
            transaction.commit(redone);
            history.record(redone);
        }
        undone = requireState(history.undo(), "connection-clear Undo missing");
        require(snapshot(undone) == connected,
            "clearing a connection must undo back to the connected document");
        redone = requireState(history.redo(), "connection-clear Redo missing");
        require(snapshot(redone) == unconnected,
            "clearing a connection must redo to the unconnected document");
    }

    void regionStyleOverridesUseWholeDocumentHistory()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        DocumentHistory history;
        history.reset(track);
        const std::string inherited = snapshot(track);

        {
            AuthoredTrackEditTransaction transaction{track};
            const auto before = transaction.candidate().section(0)
                .trackStyleOverrides;
            transaction.candidate().section(0)
                .trackStyleOverrides.enabled = true;
            const auto impact = quantum::editor::classifyRegionTrackStyleEdit(
                transaction.candidate().trackStyle(), before,
                transaction.candidate().section(0).trackStyleOverrides);
            transaction.commit(track);
            history.record(track, false, impact);
        }
        const std::string enabledOnly = snapshot(track);
        track = requireState(history.undo(), "region-style enable Undo missing");
        require(history.lastRestoreTrackStylePresentationImpact().has_value()
                && history.lastRestoreTrackStylePresentationImpact()->empty(),
            "style-only Undo retains its no-op presentation classification");
        require(snapshot(track) == inherited,
            "enabling local override behavior must be undoable");
        track = requireState(history.redo(), "region-style enable Redo missing");
        require(snapshot(track) == enabledOnly
                && track.section(0).trackStyleOverrides.enabled,
            "enabling local override behavior must be redoable");

        {
            AuthoredTrackEditTransaction transaction{track};
            const auto before = transaction.candidate().section(0)
                .trackStyleOverrides;
            auto& overrides = transaction.candidate().section(0)
                .trackStyleOverrides;
            overrides.hardwareSpacing = 1.6;
            const auto impact = quantum::editor::classifyRegionTrackStyleEdit(
                transaction.candidate().trackStyle(), before, overrides);
            transaction.commit(track);
            history.record(track, false, impact);
        }
        const std::string overridden = snapshot(track);
        require(track.section(0).trackStyleOverrides.hardwareSpacing == 1.6,
            "accepted region override must enter document state");

        track = requireState(history.undo(), "region-style Undo missing");
        require(history.lastRestoreTrackStylePresentationImpact().has_value()
                && history.lastRestoreTrackStylePresentationImpact()->affects(
                    quantum::editor::TrackStylePresentationProduct::
                        HardwareInstances)
                && !history.lastRestoreTrackStylePresentationImpact()->affects(
                    quantum::editor::TrackStylePresentationProduct::
                        RenderableMesh),
            "hardware-spacing Undo restores the hardware-only invalidation");
        require(snapshot(track) == enabledOnly
                && track.section(0).trackStyleOverrides.enabled
                && !track.section(0).trackStyleOverrides.hardwareSpacing,
            "Undo must remove an individual property override exactly");
        track = requireState(history.redo(), "region-style Redo missing");
        require(history.lastRestoreTrackStylePresentationImpact().has_value()
                && history.lastRestoreTrackStylePresentationImpact()->affects(
                    quantum::editor::TrackStylePresentationProduct::
                        HardwareInstances),
            "hardware-spacing Redo reuses the hardware-only invalidation");
        require(snapshot(track) == overridden
                && quantum::coaster::resolveTrackStyle(
                    track.trackStyle(),
                    track.section(0).trackStyleOverrides)
                    .repeatingHardware.front().spacing == 1.6,
            "Redo must restore region data and resolved presentation");

        {
            AuthoredTrackEditTransaction transaction{track};
            transaction.candidate().section(0).trackStyleOverrides = {};
            transaction.commit(track);
            history.record(track);
        }
        track = requireState(history.undo(), "region-style clear Undo missing");
        require(snapshot(track) == overridden,
            "clearing all overrides must be undoable");
    }

    void trackConfigurationSelectionAndResetUndoExactly()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        track.setTrackStyle(quantum::coaster::createStandardDualRailPreset());
        track.setTrackConfigurationId("");
        auto& overrides = track.section(0).trackStyleOverrides;
        overrides.enabled = true;
        overrides.hardwareSpacing = 2.0;
        DocumentHistory history;
        history.reset(track);
        const std::string legacy = snapshot(track);

        AuthoredTrack selected = track;
        selected.applyTrackConfiguration("modern-steel");
        const auto selectionImpact = quantum::editor::
            classifyDocumentTrackStyleEdit(track, selected);
        history.record(selected, false, selectionImpact);
        const std::string selectedSnapshot = snapshot(selected);
        require(selected.section(0).trackStyleOverrides.hardwareSpacing == 2.0,
            "selection preserves regional override state");

        AuthoredTrack reset = selected;
        reset.resetToConfigurationDefaults();
        const auto resetImpact = quantum::editor::
            classifyDocumentTrackStyleEdit(selected, reset);
        history.record(reset, false, resetImpact);
        require(!reset.section(0).trackStyleOverrides.enabled,
            "explicit reset clears region override state");

        track = requireState(history.undo(), "configuration reset Undo missing");
        require(snapshot(track) == selectedSnapshot
                && history.lastRestoreTrackStylePresentationImpact()
                    .has_value(),
            "reset Undo restores selected style and regional overrides");
        track = requireState(history.undo(),
            "configuration selection Undo missing");
        require(snapshot(track) == legacy
                && history.lastRestoreTrackStylePresentationImpact()
                    .has_value(),
            "selection Undo restores the legacy document exactly");
        track = requireState(history.redo(),
            "configuration selection Redo missing");
        require(snapshot(track) == selectedSnapshot,
            "selection Redo restores the concrete snapshot");
    }
    void trackDeviceEditsUseWholeDocumentHistory()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        DocumentHistory history;
        history.reset(track);
        const std::string baseline = snapshot(track);
        quantum::coaster::TrackDevice launch;
        launch.name = "Launch";
        launch.startStationMeters = 0.0;
        launch.endStationMeters = 15.0;
        AuthoredTrackEditTransaction add{track};
        const auto id = add.candidate().addTrackDevice(launch);
        add.commit(track);
        history.record(track);
        const std::string added = snapshot(track);
        require(added != baseline, "device creation changes document");

        AuthoredTrackEditTransaction rejected{track};
        auto invalid = rejected.candidate().trackDevices().devices.front();
        invalid.endStationMeters = 1000.0;
        bool threw = false;
        try { rejected.candidate().updateTrackDevice(invalid); }
        catch (const std::invalid_argument&) { threw = true; }
        require(threw && snapshot(track) == added,
            "invalid device candidate leaves committed state intact");

        require(snapshot(requireState(history.undo(), "device Undo missing"))
                == baseline, "Undo removes authored device");
        require(snapshot(requireState(history.redo(), "device Redo missing"))
                == added, "Redo restores authored device");

        AuthoredTrackEditTransaction remove{track};
        remove.candidate().removeTrackDevice(id);
        remove.commit(track);
        history.record(track);
        require(snapshot(requireState(history.undo(), "delete Undo missing"))
                == added, "Undo restores deleted device and stable ID");
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
        physicalSettingsEditUsesTransactionAndHistory();
        dirtyStateTracksSavedRevision();
        newOpenResetAndContinuousCoalescing();
        trackHardwareEditsUndoAndRedo();
        supportNodeEditsUseWholeDocumentHistory();
        supportGraphAuthoringUndoRedoIsExact();
        supportAnchorMetadataUsesWholeDocumentHistory();
        supportMemberEndConnectionsUndoRedoIsExact();
        regionStyleOverridesUseWholeDocumentHistory();
        trackConfigurationSelectionAndResetUndoExactly();
        trackDeviceEditsUseWholeDocumentHistory();
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
