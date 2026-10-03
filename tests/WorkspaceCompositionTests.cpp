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

        for (int cycle = 0; cycle < 2; ++cycle)
        {
            require(!editorWorkspaceComposition(WorkspaceMode::Simulator,
                    selectedWorkspace).has_value(),
                "Simulator must not submit an editor workspace");
            require(selectedWorkspace == EditorWorkspace::Track,
                "Simulator routing must retain the selected editor workspace");
            require(editorWorkspaceComposition(WorkspaceMode::Editor,
                    selectedWorkspace) == EditorWorkspace::Track,
                "Returning to Editor must restore the Track composition");
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
        for (const auto mode : {WorkspaceMode::Editor, WorkspaceMode::Simulator,
            WorkspaceMode::Editor, WorkspaceMode::Simulator, WorkspaceMode::Editor})
        {
            selectedWorkspace = EditorWorkspace::Track;
            const auto composition = editorWorkspaceComposition(mode, selectedWorkspace);
            require(composition.has_value() == (mode == WorkspaceMode::Editor),
                "Only Editor mode submits the selected editor composition");
            require(quantum::coaster::serializeCoasterDocument(track) == before,
                "Workspace activation/routing must not alter authored content");
            require(history.size() == historySize && history.isDirty() == dirty
                    && history.canUndo() && history.canRedo(),
                "Workspace activation/routing must preserve history and dirty state");
        }

        auto redo = history.redo();
        require(redo.has_value()
                && quantum::coaster::serializeCoasterDocument(*redo) == redoDocument,
            "The exact accepted length edit must remain available after routing");
    }
}

int main()
{
    try
    {
        defaultAndSimulatorRouting();
        routingPreservesDocumentAndHistory();
        std::cout << "Workspace composition tests passed.\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Workspace composition test failed: " << error.what() << '\n';
        return 1;
    }
}
