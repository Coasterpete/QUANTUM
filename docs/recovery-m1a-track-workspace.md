# Recovery M1A: explicit Track composition

Implemented on `recovery/m1a-track-workspace`, starting from clean current
`main` at `1c429b03834c776d1d975109c48f705aa81dc983` (2026-10-03).
The original maintenance checkout and its uncommitted work were not changed.

## What to inspect

- `editor/include/quantum/editor/WorkspaceComposition.hpp`: the existing
  Editor/Simulator mode enum, the one Track editor workspace, its default,
  and the pure `editorWorkspaceComposition()` routing function.
- `EditorUi.hpp`: `editorWorkspace_` is a small UI value alongside
  `workspaceMode_`. Neither value owns a document or simulation.
- `EditorUi.cpp`: `showMainMenuBar()` exposes `Workspace: Track` with a checked
  Track entry. Selecting it only assigns the workspace enum.
- `EditorUi.cpp`: `beginFrame()` explicitly selects the editor composition.
  The existing large pane implementations remain in place.
- `tests/WorkspaceCompositionTests.cpp`: default/routing coverage and an
  authored/history fixture with an undone length edit and available Redo.

Before M1A, `beginFrame()` branched directly on Simulator and then submitted
the editor panes inline. After M1A it reads the mode and selected workspace
through the pure routing function: Simulator has no editor composition and
still calls the unchanged `drawSimulator()`. Editor frames submit the menu,
toolbar, dockspace, shared Coaster Setup, then the explicitly identified Track
composition. The supporting preferences/settings/telemetry calls now follow
Coaster Setup; the rest of the pane order and implementations are unchanged.
Both paths still finish with `ImGui::Render()`.

The frame's composition is chosen before its menu handles input. Enter
Simulator therefore still finishes the current editor frame and takes effect
on the next one. Simulator changes only the existing mode, leaving Track
selected. Return to Editor keeps its existing pause request and camera gesture
cleanup. Workspace selection never calls `resetTransientState()`.

Track includes the existing Track Workspace pane, viewport, Transition Editor,
Geometry Editor, Force Diagnostics, Supports, Track Devices, Viewport Settings,
Performance Telemetry, and applicable Transition Editor Input preferences.
Coaster Setup remains shared editor UI. All ImGui window names, dock IDs,
default docking, Reset Workspace Layout behavior, first-launch Coaster Setup
docking correction, and `imgui-layout-v2.ini` handling remain unchanged.

The C++ concepts are scoped enums for distinct choices, a default-initialized
value member, and `std::optional` for absence of an editor composition.
Routing takes only enum values, so it cannot mutate authored/history state.
Application's committed document, transaction/history/publication paths,
derived geometry, renderer resources, SimulationPreview, and selection
representations retain their existing ownership and lifetimes. No workspace
classes, additional workspaces, layout files, or dependencies were introduced.

## Debug build and automated verification

Commands below ran from this worktree. Configuration reused already installed
vcpkg packages without installing or changing dependencies:

```powershell
$env:VCPKG_ROOT = 'C:\DEV1\vcpkg'
cmake --preset windows-msvc-debug -DVCPKG_INSTALLED_DIR=C:/DEV1/QUANTUM/build/vcpkg_installed -DVCPKG_MANIFEST_INSTALL=OFF
cmake --build --preset windows-msvc-debug --parallel 8 --target QUANTUM QuantumEditorWorkspaceCompositionTests QuantumEditorDocumentHistoryTests QuantumEditorAuthoredTrackEditTransactionTests QuantumEditorDocumentTests QuantumEditorSimulationPreviewTests QuantumEditorPreviewSmokeTests
ctest --test-dir build -C Debug --output-on-failure -R '^QuantumEditor\.(WorkspaceComposition|DocumentHistory|AuthoredTrackEditTransaction|DocumentState|SimulationPreview|PreviewSmoke)$'
& .\build\editor\Debug\QUANTUM.exe --dev-preview-smoke smoke-tests/preview-transition.quantum --mode-cycle --repeat --resize-window --duration 12 --output build/verification/mode-cycle --log-level debug *> build/verification/mode-cycle.log
git diff --check
```

Configuration and all seven requested build targets succeeded (exit 0).
The build emitted an existing C4834 warning in DocumentHistoryTests.cpp:488;
that unrelated test was not modified. All six selected CTest suites passed
(14.62 seconds total). `git diff --check` passed.

The native Debug smoke run exited 0, rendered 1,194 frames, completed all eight
mode-cycle actions at frames 30/60/90/120/150/180/210/240, and exercised resizing.
Its JSON reports `playback_completed_normally: true`,
`preview_or_physics_failure: false`, and an empty `failure_message`.
Khronos validation was active; no VUID or error was found in the smoke log.
Local reports are in `build/verification/mode-cycle.{json,txt,log}`.

## Live Windows editor verification

The real Debug executable was launched normally, without the smoke harness,
with `--log-level debug`. Input and screenshots used the computer-use skill.
Starting state was a fresh untitled document with one 60-unit Profile Region,
stopped playback, and the existing saved dock layout.

1. Confirmed `Workspace: Track` was visible and its Track entry checked;
   selecting Track retained the same document/UI.
2. Appended a Profile Region, changed its length from 60 to 61 with the numeric
   step button, then used Ctrl+Z once to restore 60 and leave Redo available.
3. Selected the first Region and resized the left dock pane from 303 to 371
   pixels, changing the viewport's left boundary. Closed Force Diagnostics.
4. Entered Simulator, exercised playback, and returned to Track. Both Regions
   still showed length 60, the first remained selected, the title retained its
   dirty asterisk, and Edit showed enabled Undo and Redo. The resized pane
   remained in place and Force Diagnostics remained closed.
5. Opened Append Region's pending type chooser, entered Simulator, and returned.
   The chooser remained active and Cancel responded normally. No new Region was
   created and no obvious input capture remained stuck.
6. Because the short track reached its endpoint during the first playback
   check, made a separate longer-track check by setting the first Region to
   1,000. Play showed Playing; clicking Pause showed Paused at 15.68 m/s.
   Resumed (Playing at 14.98 m/s), then clicked Return to Editor while still
   playing. The Track viewport showed Paused at a nonzero speed, separately
   verifying pause-on-return rather than endpoint stopping.
7. Undid that supplemental length change, restoring the two 60-unit Regions.

The live checks verify visible document/selection/dirty/history availability
and layout continuity. Exact serialized document/history invariants are covered
by the focused automated test; the live run did not inspect process memory.
The active interaction tested was Region creation, not every support-picking,
node-connection, or graph-drag gesture. First-launch docking and Reset Workspace
Layout were preserved by source inspection, not separately exercised live.
Release, per-workspace layouts, and other workspaces were not tested or built.
The pre-test layout was copied to `build/verification/layout-before-live.ini`;
the live session retains the test pane resize and closed diagnostics.

M1A stops here. Train, Supports, and Operations workspace implementation remains
deferred.
