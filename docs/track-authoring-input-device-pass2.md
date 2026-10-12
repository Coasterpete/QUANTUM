# Track authoring, input and device reliability — pass 2

Date: 2026-10-12. Branch: `codex/track-authoring-input-device-pass2`.
Starting HEAD: `f912e849629b00c093beb040e3c7f167ac101e12`.

## Evidence and confirmed problems

The requested `QUANTUM_Fsv7OpWa1l.mp4` was not present in the repository. No
video frames were reviewed. The maintainer's observations guided source inspection
and controlled reproductions; mouse feel and usability remain human acceptance work.
The preceding `66c2a92` implementation and `f912e84` verification were inspected.

Confirmed source/interaction defects:

- The transition-drag runtime smoke hardcoded Region 5 and could report success
  despite rejected edits. It now chooses an actual Profile Region/stable segment
  and requires all 30 authored edits to commit, failing explicitly otherwise.

- Rate/force numeric endpoints and Region length emitted edits during typing.
  Those edits could rebuild geometry before a number was complete. They now
  commit on Enter or leaving the field; deliberate numeric values are not snapped.
- Incrementally snapping a rolling drag anchor lost sub-grid movement every
  frame. Fine snapping could stick and depend on frame partitioning. The anchor
  now accumulates unsnapped values/distances; only published proposals are snapped.
- Absolute mouse-position subtraction used a 64-pixel-per-frame rejection rule
  to mask coordinate-origin changes. It also discarded legitimate fast motion.
  Workbench camera and profile gestures now consume accumulated SDL relative
  motion, without multiplying mouse displacement by frame time.
- Camera and graph interaction preferences were transient; pan had no separate
  sensitivity. Preferences now share ImGui's existing settings lifecycle.
- Resizing a custom device interval changed only its final segment. Shrinking
  before an interior split inverted the tail and rejected a valid range edit.
  Explicit interval resizes now proportionally retime all custom segments,
  preserving normalized shape, amplitude, transition types, allocator and IDs.

There is no evidence here that an unattractive hill implies a numerical defect.
A sustained signed local pitch rate accumulates rotation; long transitions and
large rates can legitimately produce steep or inverted shapes. No authored
rate/G limits, automatic rounding, or geometry beautification were introduced.

## openFVeiD implementation comparison

Reference: [H27CK/openFVeiD](https://github.com/H27CK/openFVeiD), inspected at
`74c02bb02bc9ab176edf4071663ba0738bb72226`. The repository is GPL-3.0.
It was studied as a workflow reference; no external source, artwork or dependency
was imported into QUANTUM.

- `core/secforced.cpp`, `secgeometric.cpp`, `secgeometricriderlocal.cpp` and
  `track.cpp` combine scalar channels with integrated orientation, speed and force
  results. Rider-local channels and world orientation are distinct concepts.
  QUANTUM retains its existing double precision rider-local and force-driven solvers.
- `core/function.cpp` and `subfunction.cpp` implement duration, amplitude,
  translating adjoining values, dynamic end locks, center/tension time warps,
  polynomial/sinusoidal/freeform/custom expressions, and time/distance domains.
  QUANTUM already has 19 analytic transition presets, exact integrals, stable
  segment IDs, C0 value propagation, split/remove and pinned outer boundaries.
  This pass exposes useful existing capabilities instead of replacing them.
- `ui/transitionview.cpp` groups length/amplitude/type controls and target
  operations. QUANTUM adds a compact Transition tools popup inside its existing
  dot-grid editor, with explicit begin/end selection and engineering units.
- `ui/targetsolver.cpp` adjusts duration/amplitude with a secant iteration,
  repeatedly reintegrates the section and rolls back failed solves. QUANTUM's new
  tool solves a narrower, well-defined problem exactly using its own analytic
  integral: change a transition end rate to reach its channel rotation integral.
  It displays the requested target, adjusted parameter, committed residual and
  convergence status. It is **not a general endpoint-orientation or speed/G solver**.
- `ui/graphview.cpp`, `measurementgraphprocessor.cpp` and
  `resultinggraphprocessor.cpp` separate editable channels, evaluated graphs and
  rider-offset analysis, and provide playhead/navigation helpers. QUANTUM retains
  Force Diagnostics and identifies editable force targets separately from loads.
- A material discrepancy with the reference README was found:
  [`core/section.cpp::chaseCurves`](https://github.com/H27CK/openFVeiD/blob/74c02bb02bc9ab176edf4071663ba0738bb72226/core/section.cpp#L540)
  aligns non-active channel lengths by extending holds or shrinking/truncating
  trailing segments. `graphview.cpp` describes that behavior. This inspected
  implementation does not solve a fixed 3D endpoint. QUANTUM already enforces a
  common channel domain and does not need a second length-alignment subsystem.
- `smoothhandler.cpp` configures smoothing intervals/iterations. Automatic
  smoothing was not introduced because preserving intentional aggressive and
  unusual elements is part of this assignment.

## Working workflows

### Camera and input

RMB look, Alt+RMB orbit, MMB pan and wheel zoom remain. Default rotation changes
from 0.005 to 0.002 radians per pixel at 100% UI scale. Orbit/look displacement
is divided by logical UI scale; perspective/orthographic pan uses visible
world span per viewport pixel. Orbit, pan and zoom have separate settings.
Shift slows mouse rotation, pan and zoom to 10%; the established Shift fly-speed
multiplier remains. F frames the selected Region; Shift+F frames the whole track.

Cursor zoom preserves the point under the cursor on the camera focus plane in
both projections. It does not raycast actual scene depth. It can be disabled.
Mouse navigation is acquired only over the viewport, suppresses acquisition
through active items/popups, ignores motion accumulated before acquisition,
and releases on button-up, Escape or focus loss. ImGui's existing SDL capture
remains responsible for platform capture; no competing capture system was added.

Navigation and transition gains/snaps persist in the existing editor layout ini
under `[QuantumInteraction][Preferences]`. Versioned JSON fields have compatible
defaults, validated limits, and diagnostics on malformed settings. Preferences
are editor state, outside the coaster document and its Undo/Redo. Unchanged
frames compare a small value snapshot; JSON encoding occurs during ini writes.

### Track authoring

- Begin/End buttons select the semantic endpoint without arming a drag. Rate
  step buttons use 0.01 deg/m (fast 0.1); force targets use 0.05 G (fast 0.5 G).
- Transition tools edits amplitude and interior segment length numerically.
  Moving an interior end resizes its adjoining segment; final ends remain tied
  to Region length in the Inspector. Non-SI coordinate scale is explicitly noted.
- Rotation target works on rate-profile pitch/yaw/roll and force-driven roll.
  It adjusts the selected transition's end rate through the normal authored
  transaction/regeneration/history path. Its integral is a local channel
  quantity, not a world Euler angle. A simultaneous 3D rotation is noncommutative.
  Residual tolerance is `1e-10 * max(1, abs(targetRadians))`; effectively zero
  end-value sensitivity (`<= 1e-12` domain units) is refused.
- The graph shows transition/Region integrals and the evaluated Region endpoint
  pitch/yaw/bank from the exact Core boundary frame. Curvature diagnostics divide
  authored rates by coordinate scale before presenting inverse metres.
- For compatible adjoining profile types, selecting the first segment exposes
  the preceding Region's end-value difference and an explicit Match previous
  Region value command. It changes the selected start value only. This matches
  channel values, not derivative continuity or a positional constraint.
- Angular-rate, G-force and device-acceleration snapping have separate increments.
  Shift fine dragging accumulates small motion, and unchanged snapped boundaries
  do not issue redundant distance edits. Numeric distance edits create discrete
  history entries; held pointer gestures remain coalesced.

Connected positions and rider frames continue to come from Core chaining. The
shared viewport boundary cannot be freely moved as a constrained point; its
message now describes inherited continuity and the supported Region-edit path.
No fake constraint or endpoint lock was added.

### Devices

Track Devices displays the selected device's actual RideOperations snapshot,
saved SI range, preview playback state, signed speed, occupied bogies, mechanism
state/interlock reason, commanded speed/acceleration, last fixed-step force,
force cap, target-speed limiting and runtime-vs-saved enable difference.
Play/Pause/Reset use the existing preview commands. Paused/stopped snapshots are
explicitly identified; displaying a saved enabled checkbox alone proves no force.

CableLift is a target-speed mechanism with catch-car availability,
Engaging/Pulling/Releasing/Returning states. Its custom profile limits drive
acceleration, rather than guaranteeing that acceleration at every speed. Gravity
compensation and the maximum-force cap still apply. Reverse entry does not acquire
the forward-only catch car. An engaged chain anti-rollback remains a mechanical
holding behavior even if the motor is unavailable; that existing invariant was
preserved rather than misclassified as an enable/disable defect.

Controlled native test: three straight connected 100 m Regions; one loaded car;
CableLift at 27–150 m; 2–3 m/s² linear custom profile; 10 m/s cable speed;
1,000,000 N force cap; identical 6 m/s starting runs. Enabled peak speed was
9.99479 m/s, exit speed 9.98437 m/s, and peak force 2292.34 N. Disabled peak speed
was its initial 6 m/s; it had stopped by the enabled run's exit. Entry, exit,
cross-Region operation, toggling, reverse-entry restriction and persistence passed.
An initial 2 m/s setup coasted to rest before entering the device, demonstrating
why configuration/reachability must be separated from a force-path defect.

No device-force or train-integrator replacement was necessary. Device-only edits
continue to avoid canonical track recompilation. C++23, Vulkan/VMA, SDL3, ImGui,
authored IDs, schema, exact arc-length routines and 1/240 s physics are retained.

## Source map and preservation

- `editor/src/EditorUi.cpp`, `EditorUiOperations.cpp`, `EditorUi.hpp`: integrated
  interactions, precision tools, live feedback and ini lifecycle.
- `ViewportCamera.*`: tested rotation response and focus-plane cursor zoom.
- `TransitionEditorModel.*`: unsnapped accumulation and analytic rotation target.
- `InteractionPreferences.*`: backend-neutral validated preference codec.
- `core/.../TrackDevices.*`, `editor/src/ApplicationService.cpp`: one shared custom
  profile retiming policy for explicit interval edits.
- `engine/src/Application.cpp`: distance-edit history distinguishes pointer
  gestures from discrete numeric edits.
- `editor/CMakeLists.txt`, `TrackAuthoringInteractionTests.cpp`,
  `RideOperationsTests.cpp`, `TimelineAuthoringTests.cpp`: integrated regressions.

Pre-existing policy, audio, assets, campaign document and audio tests are retained
and excluded from agent commits. The original dirty diff/status/HEAD are recorded
under `%TEMP%/quantum-authoring-pass2-baseline`; the original branch remains intact.

## Verification

Windows/MSVC verification completed on the active checkout, including its
pre-existing audio work. Intel Core i7-14700KF, NVIDIA GeForce RTX 4070 (driver
32.0.16.1088); Windows SDK 10.0.26100.0, Vulkan headers 1.4.357. No assertions,
tests or validation were disabled by this pass.

Commands:

```powershell
cmake --preset windows-msvc-debug
cmake --build build --config Release --parallel 2
cmake --build build --config Debug --parallel 2
ctest --test-dir build -C Release --output-on-failure
ctest --test-dir build -C Debug --output-on-failure --parallel 2
```

- Full Debug and Release builds passed. Final editor delivery builds also passed
  after the smoke-harness correction and smaller numeric step buttons.
- Release: all 127 enabled tests passed, 238.91 seconds.
- Debug: all 127 enabled tests passed, 797.64 seconds (parallel 2).
- 128 tests are registered. The pre-existing
  `QuantumEngine.GpuTrainPoseResidency` remains disabled in both configurations.
- The new `QuantumApplication.TrackAuthoringInteraction` covers DPI/partitioned
  orbit, projection-independent cursor zoom, preference defaults/round trips/
  rejection, fine snapping, all 19 rotation-target functions, real authoring
  regeneration, shared values, history and persistence, and custom-profile resizing.
- Existing geometry, train, devices, native boundaries, resistance, operations,
  supports, persistence and GPU suites ran. This is not real audio listening or
  human mouse/keyboard acceptance.
- Release programmatic runtime: the mixed 320 m/three-Region fixture completed
  all 13 native authoring actions, mode cycling and four swapchain recreations.
  1485 frames/360 fixed steps, 98.943 average FPS and 60.092 ms maximum frame
  interval in this 15.011-second diagnostic run. This is not a performance gain
  claim or representative large-scene benchmark.
- Corrected Debug programmatic runtime: the 60 m/one-Profile fixture committed
  all 30 scripted end-rate edits and handled four swapchain recreations with
  Vulkan validation and GPU sampling validation requested. The stopped-preview
  12.002-second run recorded 1196 frames and no physics steps; geometry was
  regenerated through the real authored/Vulkan path. Measured edit pre-simulation
  times were approximately 6.5�7 ms near the end of the scripted gesture.
- An earlier Debug mode-cycle run completed 226 physics steps but exposed the
  old hardcoded transition smoke bug. Its original PASS report is **not** evidence
  that those transition edits succeeded. The corrected harness now fails on a
  rejected edit or fewer than 30 committed edits.
- A no-Profile fixture was intentionally rejected by the corrected Debug smoke:
  result FAILED, exit code 2, `Transition drag smoke requires a Rate/Profile Region.`
  This verifies failure propagation instead of silently reporting success.
- An actual Release ImGui ini test loaded and rewrote nondefault orbit/pan/zoom,
  cursor-zoom, fine-gain and force-snap preferences with matching values.
- Each runtime check restored the original user layout with matching SHA-256.
  Captures were inspected for application layout/rendering; they are not evidence
  of human precision dragging or a clicked target-tools popup.
- No Vulkan validation error was reported. Debug retained the Vulkan loader
  warning that `VK_LAYER_OBS_HOOK` declares API 1.3 while QUANTUM requests 1.4.
  That external layer warning was not suppressed or treated as a fence diagnosis.

Reports, captures, generated negative fixture, ini backups and copied build/CTest
logs are under `build/pass2-verification` (ignored by Git). They retain the initial
failure evidence as well as final results. Implementation commit: `453f170`.
The verification/handoff commit follows it; `git log -2` identifies both.

The TimelineAuthoring regression originally required the old tail-only interval
resize and rejected shrinking before a split. Its contract was deliberately
updated to assert normalized timing, valid shrinking, exact Undo/Redo and atomic
rejection of an empty interval. This is a behavior change with stronger coverage,
not suppressed test failure or relaxed numerical tolerances.

## Manual Windows acceptance

1. Launch `build/editor/Release/QUANTUM.exe`. Over a selected Region, exercise
   RMB look, Alt+RMB orbit, MMB pan, wheel, Shift precision, F and Shift+F.
   Repeat in perspective and orthographic views, near small track features.
2. Change orbit/pan/zoom and graph gains/snaps, close normally, relaunch and
   verify persistence. Check high DPI and moving between differently scaled
   monitors; these physical input scenarios require human acceptance.
3. Select pitch/yaw/roll Begin/End; type a multi-digit number and press Enter.
   Ensure the intermediate text does not reshape the track. Check small steps.
4. Split a transition. Use Transition tools to edit length and amplitude.
   Confirm the shared boundary follows, the dot-grid graph and viewport update,
   and each numeric edit and complete drag has the expected Undo/Redo behavior.
5. Set a modest rotation target, solve its end rate and inspect residual and
   evaluated endpoint orientation. Change another axis and observe that a local
   integral is not a guarantee of world orientation or fixed endpoint position.
6. Select the first segment of a compatible following Region. Match its previous
   Region value; verify explicit value continuity, connected geometry and Undo.
   Shared point movement should still explain the unimplemented constraint.
7. In a Force-Based Region, edit normal/lateral target amplitude, length and G
   snapping, and compare Force Diagnostics with the editable target display.
8. On a straightforward track, configure a cable/launch/brake within a reachable
   SI range. Play in Track Devices and watch occupancy, mechanism state, actual
   speed and last-step force. Compare enabled/disabled runs after Reset. A stopped
   preview and a train that never reaches the range should explain no activation.
9. Split a custom acceleration profile and shrink its range before the old split.
   Verify normalized timing, retained amplitudes and exact Undo/Redo. Test cable
   availability and return time using Ride Operations.
10. Save, reopen, switch Editor/Simulator repeatedly, and repeat an authored edit,
    preview, pause/reset and history cycle. Verify old document/style/train/support
    configuration survives. Record any frame stalls with existing diagnostics.

## Remaining work and next substantial objective

Full endpoint-preserving constrained authoring is the next substantial objective.
It needs explicit target storage/lifecycle, parameter eligibility and bounded
positive segment lengths, a reusable residual/Jacobian interface over
`generateAuthoredTrackKinematics`, safeguarded nonlinear iteration and rejection
of failed force-driven candidates, angle unwrapping, neighboring-Region continuity
policy, cancellation/performance control, and transactional preview/history.
Reuse `CircuitCompletion`'s existing numerical infrastructure where it fits, but
do not transplant a circuit closure solve into a local editing gesture unchanged.

General endpoint pitch/yaw/bank, speed and evaluated-G targets; center/tension
warps; arbitrary expressions/freeform functions; time-domain editing; a playhead
with boundary alignment; horizontal graph pan/zoom; and free movement of shared
anchors are not delivered. The existing 19 presets and vertical graph framing
remain. Mouse capture across real OS edges/focus transitions and usability need
maintainer testing. The intermittent Vulkan frame-fence slowdown is unresolved;
this pass provides no general FPS guarantee or fence-fix claim.
