# Hybrid Track Start / world placement — local handoff

## Maintainer summary

Track Start now has a dedicated Track Layout section in the Route Inspector and
a Route browser command. XYZ edits place the entire authored coaster in world
space; increase Z to raise it. Pitch/Yaw/Roll controls edit a native quaternion
through C++ conversions. They do not edit Region profiles.

Select the blue diamond in the native viewport or Select Track Start in the
Inspector. Move exposes red X, green Y and blue Z handles; Rotate exposes three
labeled world-axis rings. Hover/drag emphasizes the active control. Release
validates and regenerates the native document, creating one Undo state. Escape,
focus loss, selection/workspace changes and viewport shutdown cancel a preview.

Attached support nodes follow native track station/frame evaluation. Free and
foundation nodes retain authored world XYZ. Devices retain their stations,
Constant/Custom mode and profiles. Timeline data remains unchanged. Successful
placement commits rebuild/reset the native train preview. Existing .quantum
serialization stores the same native position/quaternion.

The local implementation is pending viewport acceptance: the single hosted
smoke failed to observe synthetic marker selection in the Inspector. It did not
reach Move/Rotate, and was not repeated. Automated correctness and physical
interaction approval are separate. The
maintainer must approve marker visibility, selection, axis colors, Move/Rotate,
Z raising, hit targets, drag smoothness, camera behavior, Inspector UX and the
resulting support/device presentation.

## Audit of existing semantics

- `AuthoredStartPose` stores Core-coordinate XYZ and a normalized,
  sign-canonical quaternion. The right-handed rider axes are T=+X, L=+Y, U=+Z;
  T cross L = U. The ABI converts position to SI metres.
- Native generation begins at this pose and integrates downstream Regions.
  Region profiles remain local authored data. Positive native pitch turns T
  toward -U; yaw turns T toward +L; roll turns L toward +U.
- Fallback Move uses world X/Y/Z. Fallback Rotate pre-multiplies a world-axis
  quaternion. The hybrid calls the same pure translation/rotation helpers.
- The fallback previously submitted continuous regenerated edits during a drag.
  The hybrid uses staged native pose proposals, then the existing shared
  candidate/preparation/history path once on release.
- Support visualization resolves only nodes carrying TrackAttachment metadata:
  track position + lateral offset * L + vertical offset * U. Stored fallback
  positions remain unchanged. Foundation metadata keeps XYZ authoritative;
  there is no terrain projection or automatic relocation of foundation nodes.
- Device authoring uses station intervals and native acceleration profiles.
  Selected-device interval lines come from current native track geometry.
- Train rebuilding recompiles the relocated track and resets its placement and
  playback. Fixed 1/240-second simulation, train physics and Jolt are unchanged.
- Native authoring permits unreachable rate-profile layouts, but force-driven
  candidates require completed rider-load evaluation. Placement preserves this
  policy rather than adding new physical reachability restrictions.
- Track Start serialization already exists in format version 1, with default
  pose handling for historical documents. No schema or identifier changes.

The production camera correction is deliberate: the hybrid Route previously
automatically framed every geometry edit. Edits and Undo/Redo now preserve the
view; New/Open still frame the replacement document, and explicit Frame commands
remain available. Native support framing and camera controls remain available.

## What you can do now

Launch `editor/managed/Quantum.Editor/bin/Release/net10.0/Quantum.Editor.exe`.
Open a real document, select Track Start, raise Z, Undo/Redo, manipulate Move and
Rotate, run the train, and Save/Reload. The matching Release native DLL and assets
must be staged beside the apphost. ImGui remains at `build/editor/Release/QUANTUM.exe`.

## Track Start Inspector

Track Layout is separate from the selected Region form and remains visible in
Route. Apply position submits one XYZ command in metres. Apply orientation
submits Pitch/Yaw/Roll in degrees. Native conversion uses
Z(yaw) * Y(pitch) * X(roll); the quaternion remains authoritative. At +/-90-degree
pitch, the display chooses the equivalent yaw with roll zero. Equivalent angle
representations can change after refresh. Unchanged decimal values create no
history entry. Positive pitch points the starting forward direction downward,
matching native QUANTUM conventions.

## Move / Rotate workflow

Select Track Start, choose Move or Rotate, then drag a labeled control. Move
constrains displacement to its world axis. The Z handle raises/lowers the track.
Rotate uses the fallback's concentric screen rings for world X/Y/Z rotation;
these are not geometric rings lying in the three world planes.

The native child receives SDL input and coalesces down/move/up into a cumulative
proposal and a non-reused gesture ID. Avalonia keeps only temporary gesture
ordering, one in-flight request and the latest pointer value. A token/revision
guards the native staged candidate. Even down/move/up in one presented frame
becomes begin -> preview -> one commit. The target/mode/axis command seam and
pure native projected handles can be reused when another concrete authored
object migrates. Support-node gizmos were not migrated here.

During a drag, a gold centerline ghost shows the proposed placement while the
committed geometry remains as reference. It uses at most 512 segments from
cached native samples and a native rigid pose transform. It does not rebuild
track solids, support solids or train simulation on raw pointer motion. Release
runs full native validation/regeneration. The ghost is a display approximation:
force-driven Regions can solve differently after rotation because gravity stays
in world space. Failed validation preserves the committed document.

## Support behavior

Attached nodes follow regenerated track/frame evaluation with unchanged
station/lateral/vertical metadata. Free and foundation nodes remain in world
space; a member between fixed and attached nodes therefore changes its placement.
Authored fallback XYZ, member profiles, treatments, generated recipes and
connector metadata are preserved. Track Start is a named diamond; support nodes
retain their existing node/foundation markers. Route anchor clicks do not clear
the stored Support selection. Workspace changes cancel gestures but preserve
Track Start and Support identities where the objects still exist.

## Track Device behavior

Position/orientation edits preserve native IDs, station intervals, enable/force
settings, Constant/Custom mode, custom segments, transitions and values.
Selected intervals are republished against relocated geometry. Native simulation
continues evaluating devices against train bogie stations. No managed device math.

## Timeline / profile preservation

Placement leaves all authored Region data, Roll/Pitch/Yaw profiles, transition
choices, boundaries and segment IDs unchanged. Direct Region and custom device
profile drags remain on their existing native analytical path. The UI admits one
authoring interaction at a time; the native boundary also rejects competing
Timeline/transform gesture begins.

## Simulation

Placement commit regenerates track, support presentation and native train
preview. Play/Pause/Reset operate on the relocated compiled track. Previews
retain committed simulation state during a drag; a successful commit resets it.
No Train workspace or physics migration was started.

## Undo / Redo

Each accepted numeric command is one action; each meaningful Move/Rotate drag
is one action. Proposals never enter DocumentHistory. Canceled/no-op/out-and-back
gestures create no entry. Stale revisions cannot overwrite newer edits. Invalid
pose proposals preserve the last accepted preview, while invalid full solves
preserve the committed document. Sequential Undo/Redo restores exact stored
placement states and the rest of the authored document.

## Save / Reload

Existing native file I/O writes authored XYZ/quaternion in format version 1.
Selection, mode, hover, ghost and gesture tokens are transient. There is no C#
serializer or separate managed world transform. ABI 4 requires matching native
and managed binaries; the document format is unchanged.

## Regressions / automated correctness

Builds: Release QuantumNative, QUANTUM fallback and Quantum.Managed.Tests (which
also builds Quantum.Editor and Quantum.Managed).

Exact focused native test command:

```powershell
ctest --test-dir build -C Release -R 'QuantumApplication\.|QuantumEditor\.(AuthoredStartPose|ViewportTrackAnchors|SimulationPreview|TransitionEditorModel|DocumentHistory|SupportVisualization|SupportNodeManipulation)$|QuantumCore\.(TrackDevices|CoasterDocument|TransitionFunctions)$' --output-on-failure
```

The application tests are Service, TimelineAuthoring, SupportsWorkspace,
TrackPlacement, NativeBoundary, SupportBoundary and PlacementBoundary. The
remaining tests in the expression cover fallback anchors/manipulation,
simulation, transition math, native serialization and history.

Exact managed test command after staging QuantumNative.dll in its output:

```powershell
./editor/managed/Quantum.Managed.Tests/bin/Release/net10.0/Quantum.Managed.Tests.exe C:/Dev8/QUANTUM/smoke-tests/modern-steel-validation.quantum
```

The managed executable runs its original wrapper checks, TimelineChecks,
SupportChecks and PlacementChecks. PlacementChecks includes native selection,
numeric history, coalesced/single-frame gestures, cancel/no-op/stale input,
workspace/selection cancellation, support/device/profile preservation,
simulation, save/reload and direct profile dragging after relocation.

TrackPlacement tests the real-document coherent workflow, exact authored JSON
preservation except startPose, attached/free/foundation response, relocated
device lines and train vertices, device force occupancy, sequential history and
save/reload. PlacementBoundary checks SI conversion with a non-unit Core scale,
snapshot layout, tokens, invalid/stale requests and transient revisions.

Measured locally: 1,000 native pose proposals took approximately 1.2 ms total
with zero geometry solves, unchanged history and unchanged train generation.
This is CPU pose proposal timing, not a physical-pointer smoothness measurement.

## Codex bounded smoke

One `--verify-placement-smoke` launch was run and exited with failure. It rendered
47 frames and passed native Open, numeric Z raising, Undo/Redo, selection commands,
the canonical icon/Timeline checks and Frame Track Start. The next check, synthetic
native marker picking -> Inspector selection, failed. Move/Rotate checks were
not reached. The run lasted approximately 994 ms inside the smoke, and the
process exited within its 20-second external bound. It was not repeated.

The actual result is recorded in `build/track-placement-smoke.json`. A subsequent
headless test establishes that Frame Track Start centers a projectable native
marker on the real document; it does not resolve the hosted selection failure.
There
is no screenshot campaign, physical mouse approval, DPI campaign or Linux GUI run.

## Maintainer visual approval — pending

Approve marker visibility, selection feedback, Inspector numeric UX, all three
Move handles (especially Z), three Rotate controls, handle hit areas, axis colors,
hover/active feedback, drag smoothness, camera preservation and explicit framing.
Check attached vs free/foundation support presentation and device interval lines
on your real projects. Automated/native synthetic checks do not grant this approval.

## Known limitations

- Hosted synthetic marker-to-Inspector selection failed in the single bounded
  smoke; its cause is unresolved. Direct mouse Move/Rotate, marker selection and
  drag smoothness are not runtime-approved. Do not treat this handoff as accepted
  production viewport parity until that integration is resolved and approved.
- Drag preview is a native rigid centerline ghost; full rail/support/train
  presentation updates on release. Force-driven geometry can differ after solve.
- Euler display is ambiguous at vertical pitch; storage remains a quaternion.
- Rotation rings retain fallback screen-ring semantics; delta is wrapped to one
  revolution. Multi-turn rotation gestures and snapping were not introduced.
- Only Track Start currently uses the authored-transform seam. Support-node
  direct manipulation remains in ImGui; no Train UI migration was included.
- Windows Release was verified. No Debug/runtime platform or DPI campaign.
- The existing DocumentHistory test has an unrelated C4834 warning; it was not
  changed or suppressed.

## Next major phase

After maintainer acceptance: Train Configuration -> Preview -> Summary.
Do not begin it automatically.

## Optional code reading path

1. `core/include/quantum/coaster/AuthoredTrack.hpp`: AuthoredStartPose / setStartPose.
2. `editor/src/ViewportTrackAnchors.cpp`: translation, rotation and Inspector angles.
3. `editor/src/ApplicationService.Placement.cpp`: staged pose ownership and commit.
4. `editor/native/QuantumNative.h`: placement state and authored-transform ABI.
5. `editor/native/NativeViewport.Placement.cpp`: marker, ghost and pointer proposals.
6. `editor/src/ViewportTransformHandles.cpp`: projected native handle geometry/picking.
7. `editor/managed/Quantum.Editor/EditorViewModel.Placement.cs`: temporary gesture ordering.
8. `editor/managed/Quantum.Editor/TrackLayoutInspector.axaml`: Layout form.
9. `tests/TrackPlacementTests.cpp`: cross-system regression and gesture invariants.

Understand now: value-copy candidates, std::optional gesture lifetime, token and
revision checks, SI/Core unit conversion, quaternion composition, and commit
after successful preparation. Renderer retirement and numerical integration
internals are useful background that can wait. No new Vulkan resource owner was
introduced; the renderer's existing line stream and overlay resources are reused.

## Local handoff

Branch: `codex/hybrid-track-start-placement`, based on `4177b5a` with all prior
hybrid work preserved. Local feature commit and exact changed-file manifest are
reported in the final handoff. Nothing was pushed, published or opened as a PR.

Suggested future PR title: **Restore Track Start placement and viewport transforms
in the hybrid editor**.

## Changed-file manifest

```text
docs/hybrid-editor-foundation.md
docs/hybrid-supports-workspace.md
docs/hybrid-track-start-placement.md
editor/CMakeLists.txt
editor/include/quantum/editor/ApplicationService.hpp
editor/include/quantum/editor/ViewportTrackAnchors.hpp
editor/include/quantum/editor/ViewportTransformHandles.hpp
editor/managed/Quantum.Editor/EditorApplication.axaml.cs
editor/managed/Quantum.Editor/EditorViewModel.Placement.cs
editor/managed/Quantum.Editor/EditorViewModel.ProfileDrag.cs
editor/managed/Quantum.Editor/EditorViewModel.Supports.cs
editor/managed/Quantum.Editor/EditorViewModel.cs
editor/managed/Quantum.Editor/MainWindow.axaml
editor/managed/Quantum.Editor/MainWindow.axaml.cs
editor/managed/Quantum.Editor/RuntimeChecks.Placement.cs
editor/managed/Quantum.Editor/TrackLayoutInspector.axaml
editor/managed/Quantum.Editor/TrackLayoutInspector.axaml.cs
editor/managed/Quantum.Editor/ViewportHost.cs
editor/managed/Quantum.Managed.Tests/PlacementChecks.cs
editor/managed/Quantum.Managed.Tests/Program.cs
editor/managed/Quantum.Managed/Native.Placement.cs
editor/managed/Quantum.Managed/Session.Placement.cs
editor/managed/Quantum.Managed/Session.cs
editor/native/CMakeLists.txt
editor/native/NativeViewport.Placement.cpp
editor/native/NativeViewport.cpp
editor/native/NativeViewport.hpp
editor/native/QuantumNative.cpp
editor/native/QuantumNative.h
editor/native/QuantumNativeTests.cpp
editor/native/QuantumPlacementBoundaryTests.cpp
editor/src/ApplicationService.Placement.cpp
editor/src/ApplicationService.cpp
editor/src/ViewportTrackAnchors.cpp
editor/src/ViewportTransformHandles.cpp
tests/TrackPlacementTests.cpp
```
