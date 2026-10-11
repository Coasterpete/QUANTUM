# Native authoring-to-simulation pass 1

This pass extends the existing SDL3/ImGui Workbench, ApplicationService and Vulkan renderer.
It does not change the document format, numerical solvers or 1/240-second physics step.

## Working workflows

- Startup, selection and history refresh share `regionEditorValues`. Profile rates
  use the document's coordinate scale; Force-Based targets use G; Circular Arcs
  seed scalar parameters without requesting nonexistent profiles. Arc-first
  documents can initialize the native editor.
- Route selection IDs remain stable during length/type edits. The Route's More
  menu exposes all three construction types. Exact unbanked Arc-to-Profile
  conversion remains separate from explicitly replacing construction with a
  blank Profile. Force and banked Arc geometry are never silently approximated.
  Resets retain length and appearance; Undo restores the previous construction.
- Profile/Arc conversions now preserve per-Region appearance overrides. Arc
  numeric typing commits on field deactivation, avoiding partially typed edits.
  Existing transition graphs, boundary handles, snapping, Track Start gizmos,
  generated viewport geometry and document history remain the authoring path.
- Train Configuration has a native ordered-car draft editor: add, duplicate,
  remove and reorder cars; name/role; dry/load mass and centres; dimensions;
  bogie pivots and existing contact geometry; hitches/connectors; symmetric
  inertia; aerodynamic properties/resistance; GLB identity and shell alignment.
  Apply sends the complete proposal to `ApplicationService::setTrainConfiguration`.
  A valid Apply stops/resets actual physics and updates visuals without rebuilding
  track/support geometry. Invalid proposals retain the accepted document and
  show the error. Revert discards the draft. Each Apply is one history action.
  The Train viewport frames the accepted consist on entry/Apply, offers Frame
  train/car, and selects cars by their physical envelopes. Existing engineering
  overlays and the tested train picking/framing helpers serve this native view.
- Track Devices creates launches, brakes, transport, chain/cable lifts, stations
  and block sensors in the selected Region. It edits actual intervals, force
  limits, acceleration and supported custom profiles, direction/speed targets,
  holding, anti-rollback, cable availability/return timing, and station alignment,
  stopping/dispatch tolerances and downstream block references. Merely inspecting
  legacy launch/brake devices does not change their historical semantics.
- Accepted device updates retain graph drag state; continuous acceleration-profile
  drags coalesce in document history. Revert is available after a rejected edit.
- Ride Operations displays real occupancy, mechanism states, commands/applied
  force, speed limiting and interlock reasons. Runtime enable/availability and
  station dispatch commands address the preview, independently of authored data.
  Reset restores authored settings. The Simulator has an operations popup and
  returning to editing pauses playback.
- Vulkan-specific ImGui initialization, shutdown, viewport descriptor registration,
  retirement and command recording live in `EditorUiVulkan.cpp`. Ownership,
  synchronization, HDR/IBL and renderer behavior are preserved.
- Release startup exposed stack exhaustion during `vkCreateInstance`/driver
  initialization. The application frame held the large UI telemetry history.
  The UI now has a local RAII heap owner, retaining its destruction order and
  leaving the Windows thread stack available for nested API calls. Validation
  and the existing Debug stack reserve remain enabled.

## Verification

`QuantumApplication.NativeAuthoringWorkflow` covers repeated type-aware buffer
initialization at multiple coordinate scales, appearance-preserving conversions,
mixed-region endpoint continuity, Profile/Force transitions, Arc resize, train
physics/visual publication, rejected train edits, Undo/Redo, real launch speed and
force/occupancy, fixed stepping, runtime commands, Reset, and file save/reopen.
Existing Train2, hardware, transition, document/history and physics suites provide
the broader numerical and operations regression coverage.

The developer smoke `--native-authoring-workflow` emits normal native UI requests
for Arc/Force/Profile edits, train Apply, launch creation/configuration, Undo/Redo,
connected Force/Arc creation, simulation and return. Final assertions verify
accepted document values, runtime stepping and unchanged authored data during
simulation. This is programmatic runtime verification, not manual pointer/keyboard
usability acceptance. It requires the generated Arc/Force/Profile fixture.

```powershell
cmake --preset windows-msvc-debug
cmake --build build --config Debug --parallel 2
cmake --build build --config Release --parallel 2
ctest --test-dir build -C Debug --output-on-failure
ctest --test-dir build -C Release --output-on-failure
build/editor/Release/QuantumNativeAuthoringWorkflowTests.exe --write-fixture build/pass1-mixed-regions.quantum
build/editor/Release/QUANTUM.exe --dev-preview-smoke build/pass1-mixed-regions.quantum --native-authoring-workflow --resize-window --duration 15 --frame-trace --output build/pass1-native-runtime
```

Native smoke loads a fixture into memory and does not overwrite it. Back up and
restore `%APPDATA%/QUANTUM/Editor/imgui-layout-v2.ini` when running developer smoke
if the current user layout must remain byte-identical. Build diagnostics for this
execution live under `build/pass1-*`; the final delivery records their results.

## Manual Windows acceptance

1. Open `build/pass1-mixed-regions.quantum` in `build/editor/Release/QUANTUM.exe`.
   Select each Region: Arc parameters, Force-Based G targets, then Profile rates.
   Change an Arc radius, a graph endpoint/transition, split/move a graph boundary,
   and use Shift precision/value/distance snapping in Transition Editor Input.
   Confirm geometry, numeric values and selection follow the accepted changes.
2. Append/prepend/insert each type through Route; duplicate/reorder/remove Regions.
   Use More → Convert selected region. Confirm reset wording, length/appearance
   retention, contextual editor changes and Undo/Redo. Try a rejected invalid edit;
   inspect the reason and confirm accepted geometry remains intact.
3. Switch Workspace → Train. Duplicate/reorder a car, change its dry/load mass,
   bogie/hitch coordinates, connector length and visual offset/rotation. Apply;
   compare the accepted summary/preview, click a car and use Frame car/train.
   Set negative mass and Apply to verify
   rejection, then correct/Revert. Undo/Redo an accepted Apply.
4. In Track Devices, create/edit a launch, holding brake and chain/cable lift.
   Set a speed target, force cap and acceleration; configure anti-rollback or cable
   return time. For supported kinds, split/drag a custom acceleration profile,
   resize its interval and Undo the entire gesture. Invalid endpoints must report
   rejection. Create a station/block and configure downstream-block dispatch.
5. Save As to a new document; close/reopen it. Confirm Region construction/style,
   complete ordered-car configuration and all supported device parameters remain.
6. Enter Simulator; Play/Pause/Reset and inspect Ride operations. Check real occupied
   bogies/force, lift state, target limiting and station dispatch/interlock feedback.
   Return to Editor, edit, then re-enter. Resize in both modes; check Vulkan logs.
7. Repeat important interactions in Debug with validation enabled. Manual large-scene,
   long-session and pointer/keyboard/DPI acceptance remains the maintainer's check.

## Limits and next objective

- Transfer routing and multi-train behavior remain experimental; existing documents
  retain their configurations. This pass does not expose a production transfer workflow.
- Existing station access-gate references are retained/displayed, but this pass does
  not provide native access-gate relationship authoring or infrastructure forms.
- Train roles are metadata. Body dimensions do not automatically recalculate the
  authoritative inertia tensor. Existing bogie/contact topology is edited, not expanded.
- Visual references use the existing constrained static GLB importer. Missing assets
  continue to use physical diagnostic overlays; no new importer/material system exists.
- Separate scene viewports and broader direct curve-control-point manipulation remain
  future work. Existing native manipulation tools are retained.
- The intermittent frame-fence slowdown remains unresolved. No scheduling/presentation
  or physics changes in this pass establish a fix or a general performance guarantee.

Next substantial objective: audit pass 2's supports/content workflow. Extend the
native support assembly and parametric tools while preserving manual overrides,
then improve the constrained GLB/material import path with compatible fixtures.
Entry points are `ApplicationService.Supports.cpp`, the existing Supports workspace
and `StaticMeshAssets.cpp`; retain the current renderer. Native access-gate references
and a longer single-train dispatch/lift/launch/brake/arrival acceptance loop remain
the next operations work in `EditorUiOperations.cpp` and `RideOperations`.

### Runtime observations

Release CTest passed 126 enabled tests (127 registered; the pre-existing
`QuantumEngine.GpuTrainPoseResidency` remains disabled). The 13-action native
workflow passed in both Release and Debug, including four swapchain recreations;
Debug also enabled actual GPU sampling validation. No Vulkan validation error
was reported in these runs. The audit's Arc-first fixture passed a Debug eight-action
mode/resize smoke. UI screenshots were inspected after dock layout settlement.
The user's layout was restored with matching SHA-256 after each smoke.

The final Release capture/resize run had a 5.25-second frame interval around
swapchain recreation (frame 61), despite a 0.0013 ms recorded fence wait on that
frame. This is an observed outlier, not an established diagnosis or a fence fix.
The comparable Debug workflow ran about 99 average FPS, with 69 ms maximum frame
time; neither run establishes a large-scene or universal responsiveness guarantee.
See `build/pass1-native-*-final.*`, `build/pass1-arc-first-debug.*` and
`build/pass1-*-ctest.log`. `build/pass1-stack-diagnostic.log` records the pre-fix
Release stack-overflow debugger trace. Manual mouse/keyboard, DPI, gate relationship,
long-session and representative-scene performance acceptance remains outstanding.

## Preservation

Branch: `codex/native-authoring-simulation-pass1`, based on
`27978860a4b32b96be783acf7f500fefdbf960bc`. Pre-existing policy, audio, material assets,
campaign document and audio-test changes are retained. Agent-owned changes are
committed separately, including only the new Application.cpp hunks. The baseline
Application.cpp and layout snapshots remain under the ignored `build/pass1-baseline`.
