# Operations M1: custom acceleration profiles

## Scope and ownership

`TrackDevice` gains an optional `std::optional<ChannelProfile> accelerationProfile`
reusing the scalar channel profile already owned by geometric sections and force
driven regions. The profile replaces the constant
`targetAccelerationMetersPerSecondSquared` for that device; when it is absent the
device behaves exactly as in M0. Profile values are commanded acceleration
magnitudes in m/s^2 and must be non-negative. A brake still applies its force
opposing travel, so a brake profile shapes deceleration rather than direction.

`TrackDeviceCollection` stays value-owned and flows through the existing
`AuthoredTrackEditTransaction` and `DocumentHistory` paths unchanged. JSON writes
`accelerationProfile` only when present, so documents saved without one are
byte-identical to M0 output and older documents still load with an empty profile.

## Device-local distance domain

A profile is authored over the device's own distance domain `[0, deviceLength]`,
not over absolute track stations. `trackDeviceLengthMeters` is the single
authority for that length: a circuit device with `start > end` wraps through
station zero, so its length is `trackLength - start + end` rather than
`end - start`. `validateTrackDevices`, `evaluateTrackDeviceForces`, and the editor
profile graph all call that one function, so the domain a profile is validated
against is the domain it is evaluated against.

Validation requires the chain to cover the device length exactly, all values to be
non-negative, and `nextSegmentId` to exceed every authored segment ID so a later
split cannot reuse one. The editor draws the graph from the same length, so
authoring a profile on a wrapping device produces a committable document instead of
a validation error.

## Force calculation

The M0 force path is unchanged. At each 1/240-second fixed step, each bogie whose
station is inside the device occupies half of that car's loaded mass. The only
change is where the acceleration comes from: the profile evaluated at that bogie's
device-local distance instead of one authored constant. Requested force, the whole
device force cap, proportional distribution among occupied bogies, direction, and
the `ExternalForceApplication` path are all as in M0.

`TrackDeviceForceTelemetry::commandedAccelerationMetersPerSecondSquared` now reports
the **mean** command across occupied bogies rather than a device-wide constant. For
a constant command the mean is exactly the authored value, so M0 output is
unchanged. For a profile it is the value that explains the applied force, since
requested force is proportional to the sum of the per-bogie commands. An earlier
revision reported the maximum instead, which pinned the readout to the profile peak
and made a nonconstant device indistinguishable from a constant one at that peak.

## Editor workflow

The Track Devices panel gains an **Acceleration** section with a Mode selector:
**Constant** keeps the M0 single numeric field, and **Custom Profile** shows the
scalar profile graph. The graph reuses the Transition Editor's interaction model:
click a handle to select it, drag vertically to change its value with C0
propagation to the neighbouring segment, drag horizontally to move a shared
boundary, right-click to split or remove, and **Add Segment** to split the last
segment at its midpoint. The numeric field mirrors the selected endpoint and
re-seeds whenever the selection moves, so it cannot disagree with the drawn curve.

Resizing the device's start or end stations stretches the outermost profile
boundaries, keeping the chain gap-free over the new device length. Switching back
to Constant clears the profile. The constant acceleration field is left exactly as
the user last typed it: a present profile fully replaces it during force
evaluation, so mirroring the profile peak would only rewrite authored data the
moment the profile is switched off again.

## Extension audit

- **Runtime dispatch and interlocks:** unchanged from M0. The `armed` bit is still
  the only runtime control, and a brake still cannot initiate reverse travel from
  rest. A profile shapes the command once a device is armed; it does not add
  detection, block reservation, or engagement logic.
- **Chain and cable lifts, drive tires, block brakes, stations and switches:** all
  still open and unchanged. A profile is a per-device command shape, which is
  useful to each of them but is not the runtime controller any of them needs.
- **Per-bogie telemetry detail:** the telemetry exposes the mean command. A future
  diagnostic that needs the leading or trailing bogie command, or the profile value
  at a named station, needs its own accessor rather than reshaping this field.

## Windows verification

The [validation fixture](../smoke-tests/operations-m1-acceleration-profiles.quantum)
is the M0 telemetry track: a 60 m circuit, four cars, 8.00 m/s initial speed, with
a launch at 0–30 m and a brake at 35–55 m. Both devices carry nonconstant
profiles. The launch is ramp 0 → 4 m/s^2 over [0, 10], sustain 4 over [10, 20],
ramp 4 → 0 over [20, 30]. The brake ramps 5 → 2.5 over [0, 10] and 2.5 → 5 over
[10, 20] in its own 20 m device-local domain.

The fixture was opened in the Windows Editor. Both devices loaded with Mode set to
**Custom Profile** and drew their authored shapes. The launch's selected endpoint
read `4.000 m/s^2`, matching the first segment's authored end value and the drawn
ramp–sustain–ramp curve. Capture (cropped to the panel):
[Editor profile editor](operations-m1-editor-profile.png).

Simulator playback of the same document shows the command varying with position
rather than holding a constant:

| Playback point | Speed | Occupied bogies | Command | Applied device force | Net train acceleration |
| --- | ---: | ---: | ---: | ---: | ---: |
| Launch, earlier capture | 24.0 mph / 10.74 m/s | 8 | 3.09 m/s^2 | +10,000 N | +2.099 m/s^2 |
| Launch, later capture | 25.8 mph / 11.53 m/s | 7 | 2.42 m/s^2 | +8,487 N | +1.704 m/s^2 |

Neither command equals the fixture's constant field of 3.00 m/s^2, and the drop
from a capped 10,000 N to 8,487 N tracks the falling ramp rather than the force
limit. Captures (cropped to the telemetry strip): [command 3.09](operations-m1-simulator-command-3.png)
and [command 2.42](operations-m1-simulator-command-2.png).

The Release preview smoke run with this fixture in Simulator mode passed: 10.104 s,
2,122 fixed steps, 8 viewport resizes, no swapchain recreations. The Debug run
completed all eight Editor → Simulator → Editor mode-cycle actions with window
resizes and `VK_LAYER_KHRONOS_validation` enabled: 585 rendered frames, 33 viewport
resizes, 4 swapchain recreations, and no VUID or Vulkan validation error logged.
Both Debug runs on this machine were CPU-starved by unrelated background software
(about 4 FPS average against roughly 100 FPS on an idle run), so the mode cycle
needed a 150-second window to render its 245 scheduled frames; the failure it
replaced was a wall-clock timeout, not a defect. The only console warning was the
known OBS Hook loader message about its older Vulkan API version, matching M0. No
renderer, RHI, HDRI/IBL, PBR, MSAA, or GPU resource lifetime code changed in M1.

## Build and test status

The full Windows Debug build succeeded, as did the full Release build. Both needed
one incremental re-run each: vcpkg's `z-applocal` post-build step intermittently
returns `MSB3073` with exit code 32 when its target binary is momentarily locked.
The affected targets then linked normally at their usual output paths with no
source or CMake change.

`ctest --test-dir build -C Debug --output-on-failure --parallel 4` finished with
79 passed and the existing `QuantumEngine.GpuTrainPoseResidency` test disabled
(80 registered tests). `ctest --test-dir build -C Release --output-on-failure
--parallel 4` finished the same way. `QuantumCore.TrackDevices` covers the
constant-command path, three profile shapes, brake force direction, profile
serialization round-trip, a legacy document without a profile, the wrap-aware
device length, and the profile integrity rules.
