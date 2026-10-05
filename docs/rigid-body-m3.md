# Rigid Body M3: visible mechanical simulation proof

M3 makes Jolt state observable in QUANTUM's Simulator with temporary wire-box
colliders. This is an engineering visualization proof, with no authored assets,
serialization, new workspace, dependency, or train/Jolt coupling.

[M4](rigid-body-m4.md) retains this mechanism and lifecycle while making a GLB
the primary arm visual. The wire arm remains available as debug bounds/fallback;
the observations below describe the original M3 presentation.

## Observe it

Launch the Windows Debug editor (`build/editor/Debug/QUANTUM.exe`), enter Simulator,
and enable **Rigid-body mechanical proof (development)**. Enabling frames the
mechanism; **Frame proof** restores that view after camera navigation.
The framing helper temporarily bounds clipping to the complete arm rotation sweep,
so authored track bounds far from the proof do not hide it. Use existing
**Play/Pause** controls. Playback requires an available train preview and follows
its accepted ticks, including boundary stop and discarded host interruptions.

**Reset** stops/resets the train and preserves the mechanism pose. **Reset proof**
recreates the mechanism at its initial pose/tick zero; if playback is running,
accepted ticks immediately continue in the fresh world. Disable the checkbox or
return to Editor to detach/destroy the proof, remove its presentation entries,
and restore the camera that preceded framing. The default is disabled.

## Topology, units, and ownership

`RigidBodyMechanismProof` is an Editor-owned, focused demonstration owner:

- Static support: center `(0,-11.35,6)` m, half extents `(0.4,0.3,0.4)` m.
- Driven arm: center `(2,-12,6)` m, half extents `(2,0.22,0.22)` m, mass 4 kg.
- Unlimited world hinge: anchor `(0,-12,6)` m, axis `(0,1,0)`.
- Velocity motor: +1.5 rad/s, maximum torque 200 N m.

The gray support sits behind the orange arm's X/Z plane so their colliders permit
full rotation. It is a fixed mounting reference; the hinge connects the arm to
static world. Gravity remains `(0,0,-9.80665)` m/s². No optional secondary body
or slider is included. All geometry is explicitly temporary diagnostic geometry.

Application owns the proof with `unique_ptr`, declared before `SimulationPreview`.
The proof owns its isolated `RigidBodyWorld`; that world owns Jolt bodies and
constraints. Creation failure unwinds the world through RAII. The preview only
borrows the world pointer. Disable/reset detaches that pointer before destruction;
world teardown releases constraints before bodies. Recreating a whole proof world
also prevents repeated resets from exhausting M0–M2's non-reused handle slots.

`SimulationPreview::update()` is unchanged: accepted train tick -> committed train
pose -> attached `RigidBodyWorld::stepFixed()` at 1/240 s. No second accumulator,
clock, timestep, thread, or independent mechanism playback is introduced.

## State bridge and renderer

After preview update, `snapshot()` copies `bodyState()` position and quaternion,
plus the same half extents used to create each collider, into two value snapshots.
No Jolt types, handles, or borrowed objects enter presentation. Rendering samples
the current fixed-tick state directly, without authored motion or interpolation.

`appendRigidBodyProofVertices()` uses fixed centered corners in [-1,+1], scales
them componentwise by half extents, rotates with the GLM quaternion, then translates:
`worldCorner = positionMeters + orientation * (corner * halfExtentsMeters)`.
Thus full dimensions are exactly twice half extents. GLM quaternion ordering is
`(w,x,y,z)`; the existing physics boundary already converts Jolt's ordering. There
is no axis swap: right-handed X/Y horizontal, Z up. Conversion to float happens
only when writing renderer-facing `LineVertex`. Physics meters are used directly
as world coordinates and are independent of the authored track's document scale.

Application appends 48 proof vertices to the existing train diagnostic vertices.
Without a proof it publishes the original train span directly. Publication changes
when the train generation changes, the proof tick changes, or lifecycle changes.
An empty/removed proof publishes only the train stream, clearing every frame slot.

`VulkanContext::updateTrainPreviewVertices()` retains a CPU copy and marks the
existing per-frame buffers dirty. `updateTrainPreviewFrameBuffer()` updates a
slot after its fence has completed and reuses its allocation while capacity fits.
The renderer remains the sole owner of GPU buffers, pipelines, and synchronization.
No renderer API, shader, mesh asset, material system, or Vulkan ownership changes.
Fixed edge topology stays in compile-time arrays; moving world-space line endpoints
are the only extra geometry upload. No GPU allocation is created every physics tick.

## Verification

Windows x64 MSVC Debug verification on 2026-10-04:

- Configure: `cmake --preset windows-msvc-debug` passed using the existing local
  `VCPKG_ROOT`, static triplet, and dependency baseline.
- Full build: `cmake --build --preset windows-msvc-debug --parallel 2` passed,
  including Core, Engine, editor, shaders, and test executables. Final incremental
  full build passed in 11.02 s. Existing `[[nodiscard]]` warnings remain in
  untouched tests; the new proof and presentation test sources emitted no warnings.
- Standalone M3 presentation executable: all three scenarios passed. Over 2400
  ticks / 10 s, maximum rendered pivot drift was **0.0000707708 m** (about
  0.071 mm, below the 2 mm regression bound); final angular speed was 1.5 rad/s.
  Every snapshot pose equaled its actual world query exactly. Corner conversion
  uses a 2e-6 m float-rounding check; axis preservation uses 1e-5 vector tolerance.
  Coverage also includes full dimensions, quaternion rotation, train-stream
  append behavior, pause/resume/interruption, safe removal, and 140 fresh-world
  resets beyond a single world's constraint-creation capacity.
- Full enabled suite: `ctest --test-dir build -C Debug --output-on-failure
  --parallel 2` passed **98/98 enabled tests**, zero failures, in **842.36 s**.
  There are 99 registered tests; the preexisting disabled
  `QuantumEngine.GpuTrainPoseResidency` did not run. Coverage includes M0 (4.61 s),
  M1 (4.82 s), M2 (28.62 s), M3 (2.24 s), SimulationPreview (12.74 s), train/multi-car,
  dynamic contact, devices, renderer, serialization, and workspace tests.
  This full run preceded the final UI-only framing/clipping adjustment.
- After that adjustment and the final build, focused M3 presentation,
  SimulationPreview, and ViewportCamera tests passed **3/3**, zero failures,
  in **11.81 s** (2.14 s, 11.80 s, and 0.32 s respectively).
- Existing mode-cycle application smoke on `smoke-tests/preview-transition.quantum`,
  `--duration 6 --repeat --simulator --mode-cycle`: exit 0, **PASSED**, 6.001 s,
  599 rendered frames, 223 fixed ticks, eight scripted actions through Simulator
  entry, Play, Pause, resume, Reset, and Editor return. No preview/physics failure.
  This smoke leaves the proof disabled and covers the existing application path.
- Whitespace validation: `git diff --check` passed; the four new files also
  passed a trailing-whitespace check.

Live observation used the real `build/editor/Debug/QUANTUM.exe` through Windows
computer-use screenshots and controls. A temporary unsaved default track was
extended from 60 to 600 m to avoid an early open-end playback stop. Observed:

- Enable immediately framed the visible gray support and horizontal orange arm,
  with tick 0 and zero initial angular velocity.
- Play changed the orange box orientation/center through multiple sampled poses
  in the expected X/Z plane while the gray box stayed fixed. The readout reported
  +1.500 rad/s. The arm's end remained at the support/pivot with no visible detach.
- Pause held **tick 4738** and the same viewport pose across later observations.
  Stored angular velocity remained +1.500 rad/s, as expected for a paused world
  that is not stepped. Play resumed to **tick 5074** and a different arm pose.
- Train Reset restored train placement and stopped playback while retaining the
  mechanism at **tick 8154**. Reset proof then restored tick 0, zero speed, and
  the original horizontal box without an error or stale presentation.
- Disable removed the proof boxes while retaining the train diagnostic stream
  and restored the preceding camera. Return to Editor also removed an enabled
  proof. Track editing and Train's four-car preview/configuration view displayed
  normally after the Simulator checks.
- The final Debug binary also displayed the complete proof after framing with
  an ignored temporary track fixture translated to `(10000,10000,0)` m. This
  verifies that clipping is independent of distant authored bounds. That run
  completed normally: exit 0, 90.001 s, 8985 frames, 549 accepted train ticks,
  and no reported preview/physics failure. It was a manual framing check,
  not a scripted proof-control regression or an overhead benchmark.

Exact pose equality and scale conversion are established by automated state
tests; viewport observations establish visibility, axis/pivot behavior, and UI
lifecycle. These are sampled live images, not pixel comparisons or a measured
continuous-motion/frame-pacing study. No interpolation was added because no
presentation defect requiring it was identified in these observations; subtle
judder at other display cadences remains unquantified.

Host antivirus scans temporarily obscured the viewport and delayed launches;
no antivirus controls or settings were changed. The preexisting OBS Vulkan layer
API-version warning also appeared. Logs are `rigid-body-m3-configure.log`,
`rigid-body-m3-build.log`, `rigid-body-m3-final-build.log`,
`rigid-body-m3-presentation-proof.log`, `rigid-body-m3-all-tests.log`,
`rigid-body-m3-final-focused-tests.log`, `rigid-body-m3-live.log`,
`rigid-body-m3-final-live.log`, and `rigid-body-m3-smoke.log` (ignored generated files).
Smoke reports are `build/rigid-body-m3-mode-cycle.json` and `.txt`;
the framing-check reports are `build/rigid-body-m3-far-track-check.json` and `.txt`.

The disabled-proof mode-cycle smoke measured 99.845 average FPS and 0.264 ms
average GPU execution while other regressions were running. These are host/run
observations, **not** a mechanism overhead benchmark. M3 adds 48 line vertices
and small transform queries, but its incremental performance cost was not measured.

## Limits and reading path

The mechanism shares train-preview availability and playback; it stops at an open
track boundary. Wire boxes have no solid shading or production art. There is no
train collision, articulation, slider display, authoring, serialization, inspector,
or operation sequencing. Release, Linux, sanitizers, older CPUs, large mechanisms,
and cross-platform determinism are not established by this milestone.

Inspect these small sections in order:

1. `RigidBodyMechanismProof.hpp`: snapshot values and world ownership.
2. `RigidBodyMechanismProof` constructor: boxes, world hinge, and motor.
3. `snapshot()` / `armAngularSpeedRadiansPerSecond()`: actual physics queries.
4. `appendRigidBodyProofVertices()`: half extents, quaternion, and wire edges.
5. `EditorUiSimulationTelemetry.cpp`: controls, readout, camera save/restore.
6. `Application::runImpl()`: proof-control handling and diagnostic publication.
7. `SimulationPreview::update()`: unchanged accepted-tick insertion.
8. `RigidBodyPresentationTests.cpp`: conversion, pivot trajectory, and lifecycle.

The central C++ concepts are value snapshots, `std::array` for fixed topology,
`unique_ptr` for optional ownership, a non-owning pointer with explicit detachment,
RAII on failure, and reverse declaration-order destruction. Jolt solver internals
and Vulkan pipeline implementation can wait.
