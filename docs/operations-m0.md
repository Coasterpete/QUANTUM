# Operations M0: authored launch and brake

## Scope and ownership

`AuthoredTrack` owns a renderer-neutral `TrackDeviceCollection`. Each launch or
brake has a stable, never-reused ID, name, authored enabled default, whole-track
start/end stations in SI metres, positive acceleration command, and maximum
force in newtons. Devices are independent of geometry regions. JSON stores the
collection and its next ID; older documents without `trackDevices` load with an
empty collection. `AuthoredTrackEditTransaction` and `DocumentHistory` carry the
same value-owned collection as the rest of the document.

Simulator copies the definitions from the committed document. It creates
transient `TrackDeviceRuntimeState` entries from each authored enabled default.
The current runtime control has only an `armed` bit; there is no dispatch logic
or interlock yet. Device-only edits refresh these values without recompiling
the canonical track or uploading new rail geometry. Playback reset retains the
authored defaults and clears the previous force telemetry.

The viewport draws colored station spans as editor overlays, separate from the
rail mesh and region selection. Launch is orange; brake is blue. The spans are
projected from solved centerline samples and offset perpendicular to the track
in screen space, so they remain visible in both perspective and overhead views.
Device visual hardware, animation, and audio do not exist in M0.

## Station intervals

Intervals are **[start, end)**. On an open track, `0 <= start < end <= length`;
the physical endpoint at `length` belongs to a device ending there. On a
circuit, both endpoints are in `[0, length)` and `start > end` wraps through
station zero. Equal endpoints are invalid, including a would-be full-loop
device. No endpoint is clamped. Stations are expressed in metres after applying
the document's `metersPerCoordinateUnit` scale.

A Circuit document may still have open physical topology while its geometry is
incomplete. Simulator validates device intervals against its compiled track;
wrapped intervals require an actual closed circuit. M0 does not treat a visual
near-closure as a usable seam.

## Force calculation

At each 1/240-second fixed step, Simulator evaluates the current solved pose.
Each bogie whose station is inside a device occupies half of that car's loaded
mass for the device command. For each active device, requested force is

`targetAcceleration * sum(loadedCarMass / 2 for occupied bogies)`.

The sum is capped by that device's maximum force. The capped force is divided
among occupied bogies in proportion to their requested force. Applications
act at the car-local bogie reference points, in the world-space increasing
station tangent. Launch force points forward. Brake force opposes the sign of
the actual train velocity and is zero at rest. A brake cannot initiate reverse
travel from rest. Multiple devices contribute independent applications.

These forces enter the existing per-car `ExternalForceApplication` input to
`stepTrain`. The solver projects them by virtual work into the generalized
train coordinate. Gravity, track geometry, effective generalized mass,
connector constraints, and the train's existing basic resistance still
determine realized acceleration. The device command is therefore not a promise
of net acceleration. M0 adds no resistance force, so basic resistance is
counted once. The force list is fixed for a single step from its starting pose;
the next step reevaluates occupancy and velocity. This preserves deterministic
integration and the existing connector-load path.

Simulator displays actual speed, realized train acceleration, occupied bogie
count, acceleration command, and the signed applied device force. A device's
force limit is on its total train application, not on each car.

## Editor workflow

Open the **Track Devices** tab beside Supports. Add Launch or Add Brake creates
a selected device with a valid initial range. Select a row to edit its name,
enabled default, stations, acceleration, or force limit; Delete Device removes
it. Invalid edits are rejected and leave the committed document unchanged.
Save/Open and Undo/Redo use the normal document workflow. Return to Editor and
Simulator Play/Pause/Reset retain their existing behavior.

## Extension audit

- **Operations M1 acceleration profiles:** Replace the constant acceleration
  command evaluation with a distance-domain profile using QUANTUM's scalar
  transition concepts. Keep authored profiles in the device definition and
  evaluate at each occupied bogie's station. Force limiting and train force
  application can stay in the same path.
- **Chain and cable lifts:** Add distinct authored physical parameters and a
  lift-specific runtime controller for engagement, rollback restraint, and
  speed behavior. Their engagement is not equivalent to the M0 range test;
  their forces can still enter `ExternalForceApplication` where appropriate.
- **Drive tires:** Add tire placement/contact and slip or speed-control
  behavior. The current half-car mass per occupied bogie is a simple M0 force
  allocation, not a tire-contact model.
- **Block brakes:** Add train detection, block occupancy, reservation, and
  interlock state before issuing a brake command. M0 braking responds only to
  speed and bogie presence; it has no block safety semantics.
- **Stations, switches, transfer tables, and moving track:** Detection,
  dispatch, passenger state, movement, and track topology need separate
  authored and runtime interfaces. A moving track is not necessarily a station
  interval or a force source. M0 does not define a topology mutation contract.

The model deliberately names only the two implemented device kinds. Future
families should add the data and behavior they actually require, while keeping
authored definitions, runtime controls, detection/interlocks, force application,
presentation, and topology changes separate.

## Windows verification

The authored [UI fixture](../smoke-tests/operations-m0-ui-launch-brake.quantum)
was created with Add Launch and Add Brake in the Windows Editor, saved, and
reopened. Both devices and their configured ranges persisted. In the reopened
document, changing launch acceleration from 3.000 to 2.500 m/s² marked the
document dirty; Undo restored 3.000 and Redo restored 2.500. Deleting the
launch removed its row and orange overlay; Undo restored both. A further Undo
returned to the saved 3.000 m/s² value. Editor → Simulator → Editor and
Play/Reset transitions worked in the same run.

The [telemetry fixture](../smoke-tests/operations-m0-telemetry.quantum) uses
the same four-car straight track with an 8.00 m/s initial speed, launch at
0–30 m, and brake at 35–55 m. Actual Simulator captures from the Debug Windows
application showed:

| Playback point | Speed | Occupied bogies | Command | Applied device force | Net train acceleration |
| --- | ---: | ---: | ---: | ---: | ---: |
| Reset | 8.00 m/s | — | — | 0 N | 0 m/s² |
| Launch active | 8.53 m/s | 8 | 3.00 m/s² | +10,000 N | +2.143 m/s² |
| Brake active, earlier capture | 10.54 m/s | 6 | 4.00 m/s² | −10,000 N | −2.898 m/s² |
| Brake active, later capture | 7.71 m/s | 6 | 4.00 m/s² | −10,000 N | −2.842 m/s² |

The two brake captures were taken at about 3 and 4 seconds in separate reset
replays. They show speed decreasing while braking. The launch capture shows
speed rising above the 8.00 m/s reset speed. Commanded acceleration differs
from net acceleration because the existing train solver includes resistance,
geometry, effective mass, and the force cap.

Captures: [Editor spans and controls](operations-m0-editor.png),
[launch telemetry](operations-m0-simulator-launch.png),
[brake telemetry](operations-m0-simulator-brake.png), and
[later brake speed](operations-m0-simulator-brake-late.png).

The Debug preview smoke run with this fixture, Simulator mode cycling, window
resizes, and GPU preview validation passed: 594 rendered frames, 223 fixed
steps, 15 viewport resizes, and four swapchain recreations. Debug builds enable
`VK_LAYER_KHRONOS_validation` when available; the captured run logged no
missing-layer warning, VUID, or Vulkan validation error. It did log an OBS Hook
loader warning about that layer's older Vulkan API version. No renderer, RHI,
HDRI/IBL, PBR, MSAA, or GPU resource lifetime code changed in M0.

## Build and test status

The full Windows Debug build succeeded. `ctest --test-dir build -C Debug
--output-on-failure --parallel 8` finished with 79 passed and the existing
`QuantumEngine.GpuTrainPoseResidency` test disabled (80 registered tests).

The first Windows Release build was blocked at
`QuantumCoreSupportSerializationTests.exe` with `LNK1104` because Avast
quarantined the generated executable as `Win64:Evo-gen [Trj]`. Its quarantine
index also listed attempts using another build directory, output directory,
and executable names. The test's Release object compiled, and direct file
creation at the quarantined path returned `Access is denied`. No test source
or CMake target change was needed. After the developer added an Avast
exception for generated executables under `build/tests/Release/`, the affected
target linked at its normal output path and the full Windows Release build
succeeded.

`ctest --test-dir build -C Release --output-on-failure --parallel 8` completed
with 79 passed and the existing `QuantumEngine.GpuTrainPoseResidency` test
disabled (80 registered tests). The previously missing
`QuantumCore.SupportSerialization` executable ran and passed in 0.03 seconds.
