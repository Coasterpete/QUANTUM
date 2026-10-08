# Hybrid Timeline and Track Device authoring — local handoff

This upgrade continues the existing Avalonia/native foundation. It adds direct
profile authoring and the missing Track Device lifecycle and interval controls.
The ImGui fallback remains available. Work is local on
`codex/hybrid-timeline-track-devices`; nothing was pushed, published or submitted
as a PR. Maintainer visual approval is pending.

## Maintainer summary

- Select an analytical segment by clicking its sampled curve or a handle. The
  graph, segment list and numeric form share its native owner/channel/segment ID.
- Drag a circular handle vertically to edit a Begin/End value. Drag an interior
  square below the row horizontally to move a shared distance boundary.
- Crossed gray outer squares are pinned. They select but cannot start a distance
  drag; the cursor and tooltip explain the restriction. Move Begin/End buttons
  are disabled for those endpoints. Their value circles remain editable.
- A native gesture owns an analytical candidate, not an edited document. Pointer
  moves draw copied native candidate samples. Release fully prepares and commits
  one document-history action. Escape, capture loss, selection change, invalid
  candidates and stale revisions discard the candidate. A click without movement
  creates no history action. Cancel also preserves an existing Redo branch.
- Avalonia can create Launch/Brake devices, select/delete them, apply enable
  state, start/end stations and force limits, and switch Constant/Custom modes.
  Device labels include IDs and kind so repeated names remain distinguishable.
- Custom profiles use the same handles, type catalog, split/remove commands,
  numeric controls, validation and history as Region profiles.
- Analytical formulas, unit conversion, profile continuity/clamping/snapping,
  geometry, device forces, 1/240-second simulation, history and `.quantum` I/O
  remain native. C# owns screen projection and temporary interaction state.
- Full Supports, Train Configuration/Preview/Summary workspaces and advanced
  Simulator/Math/device-force diagnostics remain in the fallback editor.

## What you can do now

Launch the staged apphost:

```powershell
& ./editor/managed/Quantum.Editor/bin/Release/net10.0/Quantum.Editor.exe
```

Open a document, choose a Region, select Roll/Pitch/Yaw in the graph, drag values
and interior boundaries, apply any of the 19 transition choices, then Undo/Redo.
Switch Timeline to Track Devices, create a Launch or Brake, set its interval and
force limit, enable it, choose Custom, edit its profile, run the native train,
and Save/Reload the authored document.

## Timeline interaction

Circular value handles and square distance handles are deliberately separate.
There is one semantic value marker per shared joint, owned by the left segment's
End, matching the existing native editor. Clicking a curve can select either
neighbor directly. Selected curves are thicker and their distance span is shaded;
hover and selected-handle feedback use the same stable native identity.

Wheel zooms horizontally around the pointer. Middle-button drag pans. Frame
profile, F or Home fits the whole distance domain. Y fits native sample values,
including zero, and freezes during a drag so the ruler does not move under the
pointer; it fits again after release. The Timeline starts taller and retains its
existing splitter. Numeric editing retains Apply buttons for precise input.

Snapping is optional and initially off. Defaults are 0.5 metres for distance and
0.1 in the displayed value unit. The user can change both increments. Native
`proposeMarkerValueDrag` and `proposeBoundaryDistanceDrag` perform rounding and
boundary clamping, including their existing interior-spacing margin. There is no
managed continuity, inversion or nearest-boundary rule. Value units remain deg/m,
G for force-driven targets, and native SI m/s² for device magnitude.

## Track Device workflow

Add Launch / Add Brake uses the native fallback's defaults and ID allocator.
Select the device, enter Start/End (metres), Force limit (newtons), and Enabled,
then Apply interval / force / enabled. This is a single validated history action.
Delete is reversible through Undo. Station validation preserves open and wrapped
circuit intervals.

Selected devices have native orange interval lines and endpoint crosses above
the solved centerline; disabled devices use gray. Wrapped intervals display both
parts. These are display approximations from native solved samples, using the
existing line upload path. No new renderer or Vulkan ownership layer was added.

Placement is numeric in this upgrade. There are no device-station drag handles
in the viewport or full-track Timeline. Profile distance handles manipulate
**device-local acceleration boundaries**, not the device's global track station.

## Custom acceleration workflow

Choose Custom and Apply mode to seed a native constant analytical profile. An
existing Custom profile is retained when Apply mode is used while already Custom.
Edit its circles, interior squares, numeric Begin/End, type, split or remove.
The positive acceleration field is the Constant value / new Custom seed; it does
not overwrite existing Custom segment values. Custom endpoints may be zero;
negative magnitudes are rejected by native validation.

When the device interval changes, native code stretches the final Custom segment
to the new device length, retaining interior distances, values, types and IDs,
matching the fallback editor. Shrinking past the final segment's begin is rejected.
Constant mode removes the custom data; Undo restores it exactly.

## Undo / Redo behavior and ownership

Previously Apply prepared and committed each command directly. The new path is:

```text
pointer down -> native token + native original/candidate ChannelProfile
pointer moves -> latest position -> native proposal/validation -> native samples
pointer up -> existing full prepare/publication/commit -> one history entry
```

The controller serializes all calls. C# keeps one in-flight preview request plus
the latest pending pointer position, preventing an event backlog. Tokens prevent
late callbacks from canceling or committing a replacement gesture. Revision
checks prevent editing a different authored state. No provisional document state
or Undo entry is created during preview, so cancellation needs no history rollback.
Full preparation can reject release even after the profile-only preview passed;
the UI then restores the accepted snapshot and reports the error.

C++ value semantics provide independent profile copies. `std::optional` stores
the active native gesture and releases its vectors on reset/destruction. The
existing `AuthoredTrackEditTransaction` owns the full candidate on commit and
moves it into the authoritative document only after preparation succeeds. The
renderer retains its existing ownership and copies the native display lines.

ABI is now 2 so an older native DLL is rejected before the managed editor starts.
Snapshot struct versions/layout and `.quantum` format version remain unchanged.
Both Debug and Release staged apphosts contain the matching DLL and assets.

## Automated correctness

Windows verification on 2026-10-07:

| Check | Exact result |
| --- | --- |
| Complete native Debug build, including fallback QUANTUM | Passed |
| Native Release build: QuantumNativeTests, QuantumTimelineAuthoringTests, QuantumApplicationServiceTests, QUANTUM | Passed |
| Debug regression `ctest --test-dir build -C Debug --output-on-failure --parallel 6 -E 'PerfBenchmark'` | 106 passed, 0 failed; 1 pre-existing disabled test; 522.12 s |
| Final Debug `ctest --test-dir build -C Debug -R QuantumApplication --output-on-failure -V` | 3/3 passed; 21.44 s |
| Final Release `ctest --test-dir build -C Release -R QuantumApplication --output-on-failure -V` | 3/3 passed; 3.53 s |
| Native TimelineAuthoring service assertions | 441 passed in both configurations |
| Managed wrapper + Timeline/controller tests, Debug | Passed; 126 Timeline/controller assertions |
| Managed wrapper + Timeline/controller tests, Release | Passed; 126 Timeline/controller assertions |
| Managed builds | Debug/Release: 0 warnings, 0 errors |
| Whitespace validation | `git diff --check` passed |

The unrelated `QuantumCore.TrainPhysicsPerfBenchmark` was excluded. The existing
`QuantumEngine.GpuTrainPoseResidency` test was already disabled and remains so.
No assertions were weakened and no tests were disabled by this change.
The focused native and managed checks were rerun after the final selection
polish and naming of unchanged viewport presentation dimensions.

Coverage includes the shared graph selection path, numeric synchronization,
endpoint edits, native unit conversion, boundary continuity/IDs/clamping,
pinned rejection/disabled buttons, one gesture/one history entry, no-move,
Cancel/Redo preservation, stale revision/token rejection, invalid candidate
preservation, failed publication preservation, type changes, split/remove,
Launch/Brake create/delete, enable/force/interval edits, Constant/Custom history,
custom value/boundary drags, wrapped selection presentation, and combined
Region/device Save/New/Reload. The service test also observes occupied bogies and
positive device force with a Custom command above the retained Constant seed,
proving the native simulation consumes the authored custom profile.

Reproduce managed verification with the existing helper after the native build:

```powershell
./editor/managed/Build.ps1 -UiOnly -Configuration Debug -Test
./editor/managed/Build.ps1 -UiOnly -Configuration Release -Test
```

Local evidence is in `build/timeline-regression-debug-ctest.log`,
`build/timeline-focused-debug-final.log`, `build/timeline-focused-release-final.log`,
`build/timeline-managed-debug-tests-final.log`, and
`build/timeline-managed-release-tests-final.log` (ignored build outputs).

## Codex bounded smoke

One live Windows Debug run used `--verify-authoring-smoke`. It passed 8 checks
in 1068.5349 ms and presented 25 embedded Vulkan frames on the RTX 4070. It checked
startup, actual Timeline bindings/layout, a synthetic value gesture, native
Undo/Redo commands, create/select device, Custom graph samples, and continued
viewport presentation with selected-device state. Shutdown completed successfully.

It did not physically drag nodes, inspect hover/cursor feel, judge curves or
interval visuals, sweep DPI, use Linux GUI interaction, or produce screenshots.
The smoke preceded final small selection/label polish and naming of unchanged
viewport dimensions; the final builds and headless checks passed after that
polish. No second live run was made.
Report: `build/timeline-bounded-smoke.json`.

## Maintainer visual approval — pending

Please exercise the full workflow with real mouse input. Check circles versus
squares, hover/cursors, selection in graph and list, smoothness, cancellation,
Undo/Redo, curve readability, minimum-window and splitter sizing, optional snap
increments, Custom mode/profile editing, numeric interval placement, selected
interval/endpoint presentation, and running the train through the device. Save,
close and reload your test document. Approval of these visual/input details is
explicitly yours.

## Performance notes

Release measurements from the tests, not a claim about every document:

- 200-Region document: 100 native candidate-only updates = 0.0043 ms; full
  preparation/commit = 70.1668 ms. Preview measurement excludes interop/sampling.
- Managed/controller burst of 100 pointer candidates, including coalescing,
  native samples and full commit on the small test document = 2.318 ms.
- Debug's 200-Region full commit took 1189.14 ms; its 100 candidate-only updates
  took 0.057 ms. These measurements distinguish preview cost from release cost.

The change does not regenerate the whole track/support presentation on each
pointer event. Full validation/regeneration and simulation reset remain at
release. Large or support-heavy commits may take longer; the UI keeps the last
native viewport frame while the worker holds CPU state. No general performance
rewrite, new caching framework or physics benchmark was undertaken.

## Regressions and known limitations

All 19 transition types, native profile mathematics and tolerances, .quantum
compatibility, fixed 1/240-second simulation, coaster/train/device physics,
supports, GLB/Vulkan rendering, canonical icon, file workflow and native history
are retained. No core mathematical or serializer source was modified.

Visual approval and physical pointer/capture behavior remain unverified by a
human. Linux builds/input were not rerun for this upgrade. Native Wayland/macOS,
floating docking and full diagnostics are still outside the existing foundation.
Curves and viewport interval lines are display samples, not editable sampled
geometry. Full regeneration happens on release, so track/viewport geometry is
accepted-document geometry while a profile preview is active.

## Next major phase

After maintainer visual approval, migrate Supports as the next substantial
workspace, preserving the current native support authoring and 3D interaction.
Do not start full Train migration as part of this upgrade.

## Optional code reading path

1. `editor/managed/Quantum.Editor/ProfilePlot.cs`: SelectAt and pointer handlers;
   screen projection, semantic circles/squares and pointer capture.
2. `editor/managed/Quantum.Editor/EditorViewModel.ProfileDrag.cs`: begin, latest
   preview pump, cancel and commit; temporary managed state only.
3. `editor/managed/Quantum.Editor/TimelinePane.axaml`: mode switch, shared profile
   controls and device fields.
4. `editor/managed/Quantum.Managed/Session.cs`: typed wrappers and copied samples.
5. `editor/native/QuantumNative.cpp`: drag/device ABI calls and revision/token
   checks; C++ exceptions stay within the boundary.
6. `editor/src/ApplicationService.cpp`: beginProfileDrag, previewProfileDrag,
   commitProfileDrag, createDevice and setDeviceProperties.
7. `editor/src/TrackDevicePresentation.cpp`: native station clipping and markers.
8. `tests/TimelineAuthoringTests.cpp`: authored state/history/physics assertions.
9. `editor/managed/Quantum.Managed.Tests/TimelineChecks.cs`: selection and complete
   controller-driven authoring workflow.

## Local handoff

Branch: `codex/hybrid-timeline-track-devices`. The final chat handoff records the
local commit ID. Suggested future PR title: **Restore direct Timeline editing and
complete hybrid Track Device authoring**. Do not push or open a PR before visual
approval. No screenshot publication is authorized.

Changed-file manifest (28 files):

```text
docs/hybrid-editor-foundation.md
docs/hybrid-timeline-track-devices.md
editor/CMakeLists.txt
editor/include/quantum/editor/ApplicationService.hpp
editor/include/quantum/editor/TrackDevicePresentation.hpp
editor/managed/Quantum.Editor/EditorApplication.axaml.cs
editor/managed/Quantum.Editor/EditorViewModel.cs
editor/managed/Quantum.Editor/EditorViewModel.ProfileDrag.cs
editor/managed/Quantum.Editor/MainWindow.axaml
editor/managed/Quantum.Editor/MainWindow.axaml.cs
editor/managed/Quantum.Editor/ProfilePlot.cs
editor/managed/Quantum.Editor/RuntimeChecks.cs
editor/managed/Quantum.Editor/TimelinePane.axaml
editor/managed/Quantum.Editor/TimelinePane.axaml.cs
editor/managed/Quantum.Managed.Tests/Program.cs
editor/managed/Quantum.Managed.Tests/Quantum.Managed.Tests.csproj
editor/managed/Quantum.Managed.Tests/TimelineChecks.cs
editor/managed/Quantum.Managed/Native.cs
editor/managed/Quantum.Managed/Session.cs
editor/native/NativeViewport.cpp
editor/native/NativeViewport.hpp
editor/native/QuantumNative.cpp
editor/native/QuantumNative.h
editor/native/QuantumNativeTests.cpp
editor/src/ApplicationService.cpp
editor/src/TrackDevicePresentation.cpp
tests/ApplicationServiceTests.cpp
tests/TimelineAuthoringTests.cpp
```
