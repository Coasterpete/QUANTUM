#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/editor/AuthoredTrackEditTransaction.hpp>
#include <quantum/editor/DocumentHistory.hpp>
#include <quantum/editor/WorkspaceComposition.hpp>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace
{
    using quantum::editor::EditorWorkspace;
    using quantum::editor::WorkspaceMode;
    using quantum::editor::editorWorkspaceComposition;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
            throw std::runtime_error(std::string(message));
    }

    void defaultAndSimulatorRouting()
    {
        auto selectedWorkspace = quantum::editor::defaultEditorWorkspace;
        require(selectedWorkspace == EditorWorkspace::Track,
            "Track must be the default editor workspace");
        require(editorWorkspaceComposition(WorkspaceMode::Editor,
                selectedWorkspace) == EditorWorkspace::Track,
            "Editor mode must submit the Track composition");

        for (const auto workspace : {EditorWorkspace::Track, EditorWorkspace::Train,
            EditorWorkspace::Track, EditorWorkspace::Train})
        {
            selectedWorkspace = workspace;
            require(editorWorkspaceComposition(WorkspaceMode::Editor,
                    selectedWorkspace) == workspace,
                "Editor must submit the selected Track or Train composition");
            require(!editorWorkspaceComposition(WorkspaceMode::Simulator,
                    selectedWorkspace).has_value(),
                "Simulator must not submit an editor workspace");
            require(selectedWorkspace == workspace,
                "Simulator routing must retain the selected editor workspace");
            require(editorWorkspaceComposition(WorkspaceMode::Editor,
                    selectedWorkspace) == workspace,
                "Returning to Editor must restore the selected composition");
        }
    }

    void routingPreservesDocumentAndHistory()
    {
        auto track = quantum::coaster::createNewDocument();
        quantum::editor::DocumentHistory history;
        history.reset(track);
        track.appendSection();
        history.record(track);

        quantum::editor::AuthoredTrackEditTransaction edit{track};
        quantum::coaster::setSectionLength(edit.candidate().section(1), 75.0);
        edit.commit(track);
        history.record(track);
        const std::string redoDocument =
            quantum::coaster::serializeCoasterDocument(track);
        auto undo = history.undo();
        require(undo.has_value(), "Length edit must have an Undo state");
        track = std::move(*undo);

        const std::string before = quantum::coaster::serializeCoasterDocument(track);
        const auto historySize = history.size();
        const bool dirty = history.isDirty();
        require(dirty && history.canUndo() && history.canRedo(),
            "Fixture must retain a dirty document with Undo and Redo");

        auto selectedWorkspace = quantum::editor::defaultEditorWorkspace;
        for (const auto workspace : {EditorWorkspace::Track, EditorWorkspace::Train,
            EditorWorkspace::Track})
        {
            selectedWorkspace = workspace;
            for (const auto mode : {WorkspaceMode::Editor, WorkspaceMode::Simulator,
                WorkspaceMode::Editor})
            {
                const auto composition = editorWorkspaceComposition(mode, selectedWorkspace);
                require(composition.has_value() == (mode == WorkspaceMode::Editor),
                    "Only Editor mode submits the selected editor composition");
                require(quantum::coaster::serializeCoasterDocument(track) == before,
                    "Workspace activation/routing must not alter authored content");
                require(history.size() == historySize && history.isDirty() == dirty
                        && history.canUndo() && history.canRedo(),
                    "Workspace activation/routing must preserve history and dirty state");
            }
        }

        auto redo = history.redo();
        require(redo.has_value()
                && quantum::coaster::serializeCoasterDocument(*redo) == redoDocument,
            "The exact accepted length edit must remain available after routing");
    }

    void carCountUsesSetupTransactionAndHistory()
    {
        auto track = quantum::coaster::createNewDocument();
        quantum::editor::DocumentHistory history;
        history.reset(track);
        const auto original = track.coasterSetup();
        const auto before = quantum::coaster::serializeCoasterDocument(track);

        // Train queues this same complete setup candidate. Application's
        // existing setup handler accepts it through this transaction boundary.
        auto requestedSetup = original;
        requestedSetup.carsPerTrain = original.carsPerTrain + 1;
        quantum::editor::AuthoredTrackEditTransaction edit{track};
        edit.candidate().setCoasterSetup(requestedSetup);
        require(track.coasterSetup() == original && !history.isDirty(),
            "A setup candidate must not mutate the committed document/history");
        edit.commit(track);
        history.record(track);
        require(track.coasterSetup() == requestedSetup && history.isDirty(),
            "Accepted car count must reach the existing setup and history");
        const auto after = quantum::coaster::serializeCoasterDocument(track);
        const auto loaded = quantum::coaster::deserializeCoasterDocument(after);
        require(loaded && loaded->coasterSetup() == requestedSetup,
            "Accepted car count must use the existing document format");
        auto undo = history.undo();
        require(undo && quantum::coaster::serializeCoasterDocument(*undo) == before,
            "Undo must restore the exact setup document");
        track = std::move(*undo);
        require(!history.isDirty(), "Undo to the new document must restore clean state");
        auto redo = history.redo();
        require(redo && quantum::coaster::serializeCoasterDocument(*redo) == after,
            "Redo must restore the exact accepted car-count edit");
    }
}

int main()
{
    try
    {
        defaultAndSimulatorRouting();
        routingPreservesDocumentAndHistory();
        carCountUsesSetupTransactionAndHistory();
        std::cout << "Workspace composition tests passed.\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Workspace composition test failed: " << error.what() << '\n';
        return 1;
    }
}
