# Recovery M1B: Train workspace

Started from a clean `recovery/m1b-train-workspace` checkout matching freshly
fetched `origin/main` at `92ddb08` on 2026-10-03. Scope ends at M1B.

## Small code paths to inspect

1. `WorkspaceComposition.hpp`: `EditorWorkspace::Train` joins Track. The default
   remains Track; the pure routing function still returns no editor composition
   in Simulator and does not mutate any state.
2. `EditorUi.cpp`: `showMainMenuBar()` assigns only the selected workspace enum.
   `beginFrame()` selects a composition before menu input and shares the existing
   viewport submission. Simulator return changes only mode and preserves the
   selected workspace. Workspace switching never calls `resetTransientState()`.
3. `EditorUiTrain.cpp`: `drawTrainWorkspace()` reads committed Coaster Setup and
   queues a complete candidate through `pendingCoasterSetupEdit_`. It draws the
   read-only physical inspector and requests existing playback controls.
4. `CoasterSetupUi.cpp`: `drawCarsPerTrainInput()` is the same bounded input used
   by both surfaces. There is no retained Train car-count setting. The existing
   `Application::runImpl()` setup handler validates a transaction candidate,
   commits to its canonical AuthoredTrack, records DocumentHistory, and updates
   dirty state. Count changes trigger the existing preview rebuild condition;
   Undo/Redo reaches the same rebuild path.
5. `SimulationPreview.cpp`: `trainInspection()` copies the first repeated physical
   car/loadout, count, actual Core mass results, optional connector length, and
   resistance from an available preview. Application publishes this snapshot
   after rebuilding, rather than allocating/copying it every frame. Unavailable
   status clears the UI snapshot. Failed rebuilds may retain internal train inputs,
   but the getter returns no inspection for them.

## Composition, ownership, and C++ concepts

Track retains its existing panes, window identities, toolbar, edits, selections,
and layout. Train has Configuration/Consist on the left, diagnostic Preview in
the centre, and Physical Definition on the right. Shared Coaster Setup remains
available in either workspace. Train uses the same viewport texture, renderer,
camera, train boxes, bogie markers, connector lines, playback, and error reporting.
Track picking and gizmos are gated to Track; stored Region/support/device
selections are not reset when Train is shown.

Before M1B, Editor routing submitted only Track. Now it submits Track or Train,
then the same viewport image/camera code; only Track continues to its Region,
support/device, and diagnostic authoring panes. The setup flow remains:
UI candidate -> Application transaction -> committed document -> history ->
preview rebuild -> inspection and diagnostic vertices. No owner is transferred.
Application still owns the canonical document, history, preview, and existing
physics integration lifetime. Renderer resources retain their existing owners.

Understand now: scoped enums represent distinct choices; `std::optional` means
no valid inspection/connector; copies isolate presentation from mutable physics;
`const` references read the private snapshot; `std::move` transfers the transient
candidate/snapshot into its existing consumer. The snapshot is derived information,
not an authored train. Numerical solvers and Vulkan lifetime details are unchanged.

Only cars per train is newly exposed as an authored Train control. Existing
serialized style/restraint metadata is still available in shared Coaster Setup
and does not currently choose a different physical car. Mass, loadout, body,
bogie/hitch geometry, connectors, and resistance are clearly labelled backend
defaults not saved in the document. Core's heterogeneous-car, inertia, contact,
and other capabilities do not imply persisted authoring controls. No such editors,
new file format, physics model, dependencies, or controller hierarchy were added.

## Layout policy

Two distinct docked compositions need independent pane placement. Track's
`QuantumEditorDockSpace` and existing window IDs are preserved. Train adds
`QuantumTrainDockSpace` and its own window IDs; shared Coaster Setup has a
presentation ID per workspace but one implementation and global open flag.
Both layouts live in the existing `imgui-layout-v2.ini`, owned by ImGui under
the SDL preference directory. No custom layout storage or layout classes exist.

Existing inactive dockspace nodes receive `ImGuiDockNodeFlags_KeepAliveOnly`,
including during Simulator, as required by the installed ImGui 1.92.8 docking API.
An unvisited workspace's hidden node is not created early, so first activation
can build its default layout. Default construction and Reset Workspace Layout
affect only the composition drawn that frame. Camera and playback remain shared.
Train layout construction selects Configuration; ordinary switching retains the
user's selected dock tab. Track-only pane toggles/preferences remain in Track's
menu, so Train does not offer controls for panes absent from its composition.
The workspace choice defaults to Track on application startup; only pane layouts
are persisted. Shared Coaster Setup's open flag is shared, not per workspace.

## Verification

Commands ran from this worktree, using already installed dependencies without
installing or changing any packages:

```powershell
$env:VCPKG_ROOT = 'C:\DEV1\vcpkg'
cmake --preset windows-msvc-debug -DVCPKG_INSTALLED_DIR=C:/DEV1/QUANTUM/build/vcpkg_installed -DVCPKG_MANIFEST_INSTALL=OFF
cmake --build --preset windows-msvc-debug --parallel 8 --target QUANTUM QuantumEditorWorkspaceCompositionTests QuantumEditorDocumentHistoryTests QuantumEditorAuthoredTrackEditTransactionTests QuantumEditorDocumentTests QuantumEditorCoasterSetupTests QuantumCoreCoasterDocumentTests QuantumEditorSimulationPreviewTests QuantumCoreTrainConfigurationTests QuantumCoreTrainPhysicsTests QuantumEditorPreviewSmokeTests
ctest --test-dir build -C Debug --output-on-failure -R '^(QuantumEditor\.(WorkspaceComposition|DocumentHistory|AuthoredTrackEditTransaction|DocumentState|CoasterSetup|SimulationPreview|PreviewSmoke)|QuantumCore\.(CoasterDocument|TrainConfiguration|TrainPhysics))$'
& .\build\editor\Debug\QUANTUM.exe --dev-preview-smoke smoke-tests/preview-transition.quantum --mode-cycle --repeat --resize-window --duration 12 --output build/verification/mode-cycle --log-level debug *> build/verification/mode-cycle.log
cmake --build --preset windows-msvc-debug --parallel 8 --target QUANTUM
git diff --check
```

Configuration succeeded. The first build completed QUANTUM but found an error in
the new workspace test: deserialization returns `std::expected`, not a bare track.
The test now checks success and reads `loaded->coasterSetup()`. Repeating the
full build succeeded for all eleven targets (exit 0). The existing C4834 warning
at `DocumentHistoryTests.cpp:488` was left unchanged. All ten selected suites
passed (21.65 seconds). The final QUANTUM rebuilds after menu/tab-focus adjustment
also succeeded (exit 0). `git diff --check` passed.

Focused tests cover Track default, Train selection, Track/Train/Track routing,
both Simulator return routes, unchanged exact serialized content/history/dirty
state, accepted setup transactions and car-count serialization/Undo/Redo,
inspection count/physical inputs/mass results/bogies/connectors/resistance,
snapshot independence, fresh/failed unavailable preview, one-car connector
absence, and recovery after failure. Existing document/setup/train/preview
regressions also passed; no UI mock framework was added.

The native Debug mode-cycle smoke exited 0, rendered 1,194 frames, executed all
eight scheduled actions at frames 30/60/90/120/150/180/210/240, and exercised
window/viewport resizing. Its report has `playback_completed_normally: true`,
`preview_or_physics_failure: false`, and an empty failure message. Khronos
validation was enabled and no VUID or validation error was found. This harness
cycles Track/Simulator; Train/Simulator was exercised live. Reports are local
at `build/verification/mode-cycle.{json,txt,log}`.

### Live native editor

Two normal launches of the real Debug executable used `--log-level debug`,
with no smoke harness. Computer-use input targeted the native SDL window.
The initial saved layout was backed up to
`build/verification/layout-before-live.ini`.

1. Clicked New. Track was selected with a clean Untitled title and one 60-unit
   Profile Region. Switching to Train kept the title clean and submitted
   Configuration/Coaster Setup, Preview, and Physical Definition.
2. Used Train Configuration's increment button: four cars became five. The title
   became dirty; both resolved count displays became five, aggregate mass changed
   from 4,000 to 5,000 kg, and five diagnostic boxes/bogie markers/connectors were
   visible. Per-car dry/load/total masses remained 800/200/1,000 kg.
3. Ctrl+Z restored four cars, 4,000 kg, four boxes, and a clean title. Ctrl+Y
   restored five cars, 5,000 kg, five boxes, and dirty state.
4. Returned to Track, appended a second Profile Region, selected Region 1, and
   temporarily set its length to 1,000 for playback checks. After Track -> Train
   -> Simulator -> Train -> Track, Region 1 remained selected, the lengths
   remained 1,000 and 60, and the dirty title and accepted five-car edit remained.
5. Entered Simulator from Train. Play showed Playing; Pause showed Paused;
   Return to Editor restored Train with the same inspection/count and paused
   preview. The captured speed was already zero; pausing or returning while
   moving was not established by this run. No preview error was displayed.
6. Entered Simulator from Track and returned; Track and its Region selection,
   lengths, dirty state, and prior panes were restored.
7. Resized Train's left dock from approximately 367 to 445 pixels. Its size
   survived switching and another Simulator cycle. Track retained its existing
   371-pixel left dock and bottom/right panes. Reset Workspace Layout in Train
   restored Train's default widths; returning to Track left its layout intact.
8. Undid all three test edits back to a clean document, switched to Track, and
   closed normally. No scratch document was saved. Relaunch confirmed Track
   default and both saved layouts. After the final rebuild, Train's menu omitted
   Track-only pane toggles/preferences; Reset selected Configuration as intended,
   and Open Coaster Setup selected the shared setup tab. Closed this clean session.

Both live logs had Khronos validation active and no VUID/validation error found:
`build/verification/live-stderr.log` and `live-final-stderr.log`. The original
Track windows retained their dock IDs, positions, and sizes. ImGui changed the
saved order of the reopened Force Diagnostics tab from 0 to 1. After closing
both test sessions, the entire pre-test ini was restored and its hash verified
against the backup, preserving the user's prior preferences exactly. The tested
two-workspace ini is retained at `build/verification/layout-after-live.ini`;
Train's default layout will be created on next activation. This restoration is
test cleanup, not a layout persistence mechanism in the application.
Security software temporarily scanned the newly built binaries;
its controls/settings were not automated or changed.

Limits: Release/Linux were not built. Live support/device selection gestures,
Track's Reset command, one-car/unavailable Train UI, and save/reopen were not
separately exercised. Selection retention is supported by the untouched selection
owners and the Track-only input gates; live Region retention was verified.
One-car/unavailable publication and serialization are covered by automated tests.
Independent layout persistence/reset and shared setup access were verified live.
No new physical authoring, Train resource format, or later recovery milestone
was started.

## Files changed

- `editor/include/quantum/editor/WorkspaceComposition.hpp`: adds Train enum value.
- `editor/include/quantum/editor/EditorUi.hpp`: declares Train pane methods,
  inspection publication/storage, and Train setup docking flag.
- `editor/src/EditorUi.cpp`: workspace menu/routing, independent docking, shared
  viewport composition, Track-only input/menu gates, unavailable snapshot clearing.
- `editor/src/EditorUiTrain.cpp` (new): Train configuration, inspector, playback
  controls, compact viewport toolbar, and snapshot setter.
- `editor/include/quantum/editor/CoasterSetupUi.hpp` and `editor/src/CoasterSetupUi.cpp`:
  shared cars-per-train input and presentation window-name parameter.
- `editor/include/quantum/editor/TrainPreviewInspection.hpp` (new): derived value.
- `editor/include/quantum/editor/SimulationPreview.hpp` and `editor/src/SimulationPreview.cpp`:
  safe inspection getter from available accepted preview inputs.
- `engine/src/Application.cpp`: publishes inspection after existing rebuild.
- `editor/CMakeLists.txt`: includes the new editor source/header in QUANTUM.
- `tests/WorkspaceCompositionTests.cpp` and `tests/SimulationPreviewTests.cpp`:
  focused routing/history/inspection coverage.
- `docs/architecture.md` and this note: current composition and learning/verification record.
