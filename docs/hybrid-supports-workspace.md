# Hybrid Supports workspace — local handoff

## Maintainer summary

The existing hybrid workbench now has a Supports workspace beside Route. It
provides a structure browser, structure-scoped node/member lists, and a grouped
Inspector around the same hosted native Vulkan viewport. Timeline remains
available below the workspace. Existing Transition Editor, direct profile
editing, Track Devices, simulation, file commands and canonical icon remain.

Avalonia owns forms and copied display rows. C++ still owns the document graph,
IDs, validation, track evaluation, member placement, presentation, history,
serialization, simulation and every Vulkan resource. There is no C# support
serializer, editable graph, track-frame solver, picking algorithm or Undo stack.

Create/rename/delete structures; create/select/delete nodes; edit free/foundation
XYZ; attach/detach nodes; edit cumulative station and offsets; connect two nodes;
delete members; edit circular/rectangular profiles, dimensions and wall thickness;
edit either end's treatment and connector asset; clear existing local placement.
The nine native treatment choices are preserved. Focused edits preserve unedited
generated recipes, appearance, roles, orientations, directed references,
mounting and connector placement.

The browser and viewport use stable native structure/element IDs. Selecting a
node/member updates the Inspector and native highlight. Deleting a selected
element returns selection to its surviving structure. New/Open clear selection;
workspace switching preserves it. Undo restores data without guessing a new
element selection. Native node/member picking is implemented; numeric movement
is available. The move gizmo remains in ImGui pending its gesture migration.

## What you can do now

The staged Release apphost is:

```powershell
& ./editor/managed/Quantum.Editor/bin/Release/net10.0/Quantum.Editor.exe
```

Open a document, switch Route → Supports, create a structure and nodes, attach
one node, connect a member, change its section and end treatments, select geometry
in the viewport, Undo/Redo, then Save and reload. Switch back to Route and continue
Timeline, Track Device and simulation work. The fallback executable remains
available at `build/editor/Release/QUANTUM.exe`.

## Supports workspace

The left browser has a structure selector with counts, virtualized node/member
lists and creation/deletion commands. Stable IDs remain visible even when names
repeat. The right Inspector groups Structure, Node position, Track attachment,
Foundation, Connect nodes, Profile/dimensions and End connection. The existing
pane and Timeline splitters remain usable; Inspector contents scroll.

## Node workflow

Add a node to the selected structure. Without a selected node it starts at the
origin; with one selected it starts one metre farther in X from its resolved
position. The new node is free. Select it, edit XYZ and Apply position. A foundation
marker keeps authored XYZ authoritative and does not project to terrain. Core
refuses deletion of a node still referenced by members; delete the members first.

Attached-node XYZ shows the stored fallback position read-only; resolved world
position is displayed separately. Change its attachment controls to move it.
Clearing the attachment returns to that fallback XYZ, matching native semantics.

## Member workflow

Choose two nodes in Connect nodes, then Create member. Endpoints must be distinct,
belong to the structure and have no existing member across that unordered pair.
Select the member in the browser or viewport, choose Circular or Rectangular,
enter dimensions/wall thickness, and Apply. Circular uses one diameter; rectangular
uses width and height. Zero wall is solid; positive wall must be less than half
the smallest dimension. Core remains the final gate and reports validation errors.

Choose Start end or End end, then one of None, Mitered cut, End cap, Plate, Flange,
Splice, Saddle, Clamp, Base or Footing and Apply treatment. The two ends are
independent. Saddle/Clamp require the endpoint's Track attachment; Base/Footing
require Foundation. Set a logical `assets://support/... .glb` connector identifier
after choosing a treatment; empty input clears the asset. Existing local placement
can be cleared explicitly. Treatment and asset changes retain other end metadata.

## Track attachments

Station is cumulative over the ordered whole-track path, in displayed metres.
Enter station, lateral and vertical offsets, then Attach / apply offsets. The ABI
converts to the document's Core units. Native validation preserves the existing
finite shuttle/circuit station domains, and native track evaluation resolves
`trackPosition + lateralOffset * L + verticalOffset * U`. C# receives its result.

Track and Foundation remain mutually exclusive. Clear a foundation before
attaching, or detach before marking a foundation. Remove dependent end treatments
before removing their required anchor. No attachment is silently clamped or
detached. Viewport track-location picking remains in fallback; the hybrid workflow
uses precise station/offset controls.

## Viewport interaction

In Supports, left-click selects native nodes or solid members using the existing
`pickSupport` priorities, physical placement and tolerances. Native node markers,
foundation outlines, selected-anchor labels, member emphasis, structure-wide
emphasis and connection-end rings are drawn over the native image. Connection
rings indicate authored end metadata, not detailed connector meshes.

Right-drag orbit, middle-drag pan, wheel, WASD and F remain. Frame selection fits
the current support object. Support-only edits keep the camera; support Undo/Redo
also retains framing. File replacement frames the new document. Track Start
migration now preserves Route camera framing through edits and history as well;
use Frame track / layout or Frame Track Start explicitly.

Picking intents carry the captured document and selection revisions. Stale intents
are dropped, and clicks during changed-scene publication are ignored. CPU session
access remains serialized; frame acquisition remains nonblocking. The existing
SDL wrapper, borrowed child lifetime, Vulkan presenter, renderer resource
retirement and shutdown ordering remain intact.

Direct node dragging, constrained axes and snapping remain in the fallback's
ImGui gesture/gizmo implementation. No temporary replacement gizmo was created.

## Undo / Redo

`ApplicationService::editSupport` copies the native document into an
`AuthoredTrackEditTransaction`, applies Core mutation/validation, builds support
presentation against cached native track samples, stages history, optionally
publishes, then commits. Rejected validation or publication leaves document and
history intact. Accepted form edits create one native history state; no-op edits
create none. Support-only edits retain the solved track and running simulation.

History restoration remains the shared full-document path. Tests compare exact
native serialization before/after Undo/Redo for each support operation. Stable
element selection is kept while the object exists, otherwise it falls back to
the parent structure or clears if that structure was deleted.

## Save / Reload

Native file I/O and format version 1 are unchanged. Supports continue to serialize
in the existing optional collection and optional node/end metadata. Resolved
positions, copied snapshots, selection, hover and workspace mode are not saved.
Save/Reload tests cover the coherent structure → nodes → attachment → member →
profile → end treatment workflow, including connector assets and non-unit scale.

## Regressions and automated correctness

Windows Debug and Release built `QuantumNative`, the ImGui `QUANTUM` fallback,
application tests and the managed app/wrapper/tests. Managed builds reported zero
warnings/errors. No dependencies, document schema, physics formulas, fixed-step
policy, renderer assets or canonical icon were changed.

The following commands passed:

```powershell
# 22 Debug tests: application, existing Supports, generator and Hybrid regressions
ctest --test-dir build -C Debug -R 'Support|Hybrid|Wooden|QuantumApplication' --output-on-failure -j 4

# 12 Debug tests: focused existing-product regressions
ctest --test-dir build -C Debug -R 'QuantumCore\.(TrackDevices|TrainPhysics|TransitionFunctions|ScalarTransition|ChannelProfileEditing|CoasterDocument)$|QuantumEditor\.(SimulationPreview|TransitionEditorModel|TrainVisualConsist|Icons|DocumentHistory)$|QuantumEngine\.StaticMeshAssets$' --output-on-failure -j 4

# Final modified native suites, after scale/metadata coverage and viewport changes
ctest --test-dir build -C Debug -R '^QuantumApplication\.(SupportBoundary|SupportsWorkspace|NativeBoundary)$' --output-on-failure

# All five application tests in Release
ctest --test-dir build -C Release -R '^QuantumApplication\.' --output-on-failure -j 3

# Managed Debug and Release wrapper/controller/model checks, with matching DLL staged
& ./editor/managed/Quantum.Managed.Tests/bin/Debug/net10.0/Quantum.Managed.Tests.exe "$PWD/smoke-tests/modern-steel-validation.quantum"
& ./editor/managed/Quantum.Managed.Tests/bin/Release/net10.0/Quantum.Managed.Tests.exe "$PWD/smoke-tests/modern-steel-validation.quantum"
```

New native suites: `QuantumApplication.SupportsWorkspace` and
`QuantumApplication.SupportBoundary`. They cover CRUD; native history;
publication failure; no-ops; all nine treatments; invalid/stale mutations;
selection-only snapshot stability; capacity/enum/version checks; presentation
revisions; SI/Core conversion; save/reload; generated metadata preservation; and
a 120-node/117-member set. Existing SupportVisualization tests exercise the
reused picking algorithm. The new service test also picks an attached node.

Managed results: 126 existing Timeline checks and 1,078 Supports checks passed
(the Supports count includes 1,000 repeated cache assertions). Supports checks
exercise the real boundary, controller and view model, simulate synchronous
ItemsSource selection clearing, preserve numeric drafts during telemetry updates,
and populate 72 nodes/69 members across three structures. Measured Release
cached support snapshots averaged about 0.0008 ms per call on this machine;
this is a cache sanity measurement, not a full interaction benchmark.

## Codex bounded smoke

Exactly one Release live smoke was run with `--verify-supports-smoke`, under a
45-second process limit. It exited successfully. Its internal sequence took
about 1.79 seconds and the real NVIDIA RTX 4070 Vulkan host presented 91 frames.
It completed support creation, numeric editing, foundation/attachment, member
profile/end treatments, Undo/Redo, save/reload, synthetic native node/member
picking with browser/Inspector synchronization, and subsequent Timeline,
Track Device/custom profile and simulation operations.

Local report: `build/hybrid-supports-smoke.json`; native logs are beside it.
No physical mouse gestures, screenshot campaign, DPI sweep, Linux GUI check,
repeated live run or visual approval was performed. No screenshots were published.

## Maintainer visual approval — pending

Manually judge structure/list usability and scrolling with a representative
support document; node/member click selection and highlight legibility; Inspector
hierarchy and draft input; numeric movement; attachment offset/station workflow;
both end-treatment forms; Undo/Redo; and Save/Reload. Check that Route, Timeline
direct dragging, all transition choices, Launch/Brake, Constant/Custom profiles
and simulation still feel correct after switching workspaces.

## Known limitations

- The native move gizmo, constrained dragging and snapping remain in fallback.
- Track-location picking, generated-run configuration/regeneration and support
  appearance/material controls remain in fallback. Existing generated data is
  displayed and preserved; it is never regenerated just by opening a document.
- Roles, orientation references and mounting are preserved and described, but
  do not have new hybrid editing forms in this task.
- Detailed connection meshes and splice pairing remain the native model's
  existing later work; end rings are selection aids.
- Structure labels and asset display strings are bounded copies. Long connector
  identifiers are marked truncated and read-only in the hybrid form; focused
  treatment edits still preserve the full native identifier. Structure rename
  applies the entered text explicitly.
- ABI 4 requires rebuilding/staging matching managed and native binaries. This
  does not change `.quantum` compatibility.
- No Linux build/runtime, physical pointer feel, DPI behavior or comprehensive
  whole-project test suite was verified. The focused suites above are the evidence.

## Next major phase

After maintainer acceptance, migrate Train Configuration → Preview → Summary
as one coherent workspace using native train definitions, geometry and physics.
Do not begin physics-engine migration or new coaster mathematics as part of it.
This handoff does not start that work.

## Optional code reading path

1. `editor/include/quantum/editor/ApplicationService.hpp`: `SupportEdit` is the
   typed intent; it owns values, never document/GPU pointers.
2. `editor/src/ApplicationService.Supports.cpp`: `editSupport`, candidate copies,
   `std::optional` anchors/end metadata, validation, publication and commit order.
3. `editor/native/QuantumNative.h`: fixed-width support rows and commands (current ABI 4).
4. `editor/native/QuantumNative.cpp`: `refreshSupports`, `qn_support_command`,
   `qn_support_snapshot`, `qn_select_support` and stale viewport intent handling.
5. `editor/native/NativeViewport.cpp`: `captureSupports`, `pickSupportAt`,
   `drawSupportOverlay` and `frameSupportSelection`; Vulkan ownership is unchanged.
6. `editor/managed/Quantum.Managed/Session.Supports.cs`: immutable display records,
   copied-buffer mapping, cached arrays and command translation.
7. `editor/managed/Quantum.Editor/EditorViewModel.Supports.cs`: command forms,
   stable selection and guarded ItemsSource refresh.
8. `editor/managed/Quantum.Editor/SupportsBrowser.axaml` and `SupportInspector.axaml`:
   pane grouping; these own presentation, not a coaster document.
9. `tests/SupportsWorkspaceTests.cpp` and `editor/native/QuantumSupportBoundaryTests.cpp`:
   exact native history, compatibility and failure protection.
10. `editor/managed/Quantum.Managed.Tests/SupportChecks.cs`: controller/model
    workflow and selection refresh protection.

The C++ concepts to understand now are value-copy candidate transactions,
scoped enums, `std::optional`, stable IDs and explicit owner-thread access.
Existing member placement mathematics and Vulkan retirement details can wait;
they were reused rather than redesigned.

## Local handoff

Branch: `codex/hybrid-supports-workspace`, based on the existing
`7b85f47` Timeline/Track Device implementation. Local commit IDs are reported
in the chat handoff and available with `git log -2 --oneline`. Nothing was pushed,
published or submitted as a PR. Suggested future PR title:
**Migrate support authoring into the hybrid QUANTUM workspace**.

Changed-file manifest:

```text
docs/architecture.md
docs/hybrid-editor-foundation.md
docs/hybrid-supports-workspace.md
editor/CMakeLists.txt
editor/include/quantum/editor/ApplicationService.hpp
editor/src/ApplicationService.Supports.cpp
editor/native/CMakeLists.txt
editor/native/QuantumNative.h
editor/native/QuantumNative.cpp
editor/native/NativeViewport.hpp
editor/native/NativeViewport.cpp
editor/native/QuantumNativeTests.cpp
editor/native/QuantumSupportBoundaryTests.cpp
editor/managed/Quantum.Managed/Session.cs
editor/managed/Quantum.Managed/Native.Supports.cs
editor/managed/Quantum.Managed/Session.Supports.cs
editor/managed/Quantum.Editor/EditorApplication.axaml.cs
editor/managed/Quantum.Editor/EditorViewModel.cs
editor/managed/Quantum.Editor/EditorViewModel.Supports.cs
editor/managed/Quantum.Editor/MainWindow.axaml
editor/managed/Quantum.Editor/MainWindow.axaml.cs
editor/managed/Quantum.Editor/SupportsBrowser.axaml
editor/managed/Quantum.Editor/SupportsBrowser.axaml.cs
editor/managed/Quantum.Editor/SupportInspector.axaml
editor/managed/Quantum.Editor/SupportInspector.axaml.cs
editor/managed/Quantum.Editor/RuntimeChecks.cs
editor/managed/Quantum.Editor/RuntimeChecks.Supports.cs
editor/managed/Quantum.Managed.Tests/Program.cs
editor/managed/Quantum.Managed.Tests/SupportChecks.cs
tests/SupportsWorkspaceTests.cpp
```
