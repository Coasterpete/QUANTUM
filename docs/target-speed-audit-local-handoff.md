# MAINTAINER SUMMARY

Target Speed is an optional **directed propulsion limit**, with a completion state
while occupied. It is neither an instantaneous velocity setter nor a guaranteed
exit speed. The current accepted code already consumes it end to end. No ignored
target or disconnected controller defect was found. This task preserves the
controller and clarifies telemetry, labels and automated regression coverage.

The controlled one-car native bench starts at 5 m/s, uses a 70 m launch at stations
30–100 on a 300 m straight track, the same spatial Linear Custom profile from
2 to 3 m/s², and a 100,000 N whole-device cap. It starts 10 m inside the entry
boundary in either direction, leaving clearance beyond both device boundaries.

| Target | Forward exit speed | Reverse exit speed magnitude |
| --- | ---: | ---: |
| Unset | 13.4285 m/s | 12.6524 m/s |
| 10 m/s | 9.98437 m/s | 9.93258 m/s |
| 25 m/s | 13.4285 m/s | 12.6524 m/s |

Unreached 25 m/s and unset have identical trajectories in each direction. Reverse
motion encounters the same spatial profile in the opposite order. The highest
directed speed for either 10 m/s case is 9.99479 m/s. Existing resistance and
partial occupancy explain the small difference from the target.

Manual acceptance: keep Initial Speed = 5 m/s, launch placement, train, force cap
and Custom profile unchanged. Apply hardware configuration separately for blank,
10 and 25 m/s targets, Reset, then Play each case. Compare the accepted target,
limiter flag, propulsion and signed applied force. Ten should constrain propulsion;
25 need not be reached. A profile that cannot reach 10 also needs a longer interval
or stronger command for the 10-versus-unset comparison to be observable.

# ROOT CAUSE

There is no demonstrated target-controller physics defect in the starting HEAD
`5002d53`. The reported 15–16 m/s exit under a 25 m/s target alone does not identify
a bug; distance, profile, mass, resistance and force availability determine the
exit velocity. That exact maintainer document was not supplied or reproduced.

The presentation was ambiguous: generic telemetry displayed `Command 0 m/s` for
an unset launch target and combined the profile/limited command into one
acceleration value. There was no explicit target-limiting flag. The authored
field also did not explain that Apply hardware configuration commits its draft.
The earlier accepted draft-preservation fix remains intact.

## Complete value path

| Stage | Smallest useful code section | Responsibility |
| --- | --- | --- |
| Avalonia field | `HardwareInspector.axaml`, launch panel | `HardwareTarget` text binding; blank is permitted |
| View model | `EditorViewModel.Hardware.cs`, `ApplyHardware` | Parses positive target magnitude/blank and direction into `LaunchSettings`; submits through `SessionController` |
| Managed wrapper | `Session.Hardware.cs`, `SetHardwareConfiguration`; `Native.Hardware.cs` | Serializes camelCase JSON; calls revision-checked `qn_hardware_configuration` |
| Native ABI | `QuantumNative.cpp`, `qn_hardware_configuration` | Parses configuration for the existing stable device ID; calls application service |
| Authored transaction | `ApplicationService.cpp`, `setHardwareConfiguration` | Validates/commits the device configuration through existing history/publication |
| Authored value | `RideHardware.hpp`, `LaunchConfiguration` | `std::optional<double> targetSpeedMetersPerSecond`, direction; document-owned value |
| Saved value | `RideHardwareJson.cpp`, Launch `write`/`read` | Numeric/null target and direction in existing schema-2 hardware configuration |
| Preview | `SimulationPreview.cpp`, fixed-step `update` | Observes signed speed/bogies, steps operations, evaluates forces, records telemetry, calls `stepTrain` |
| Operations | `RideOperations.cpp`, Launch branch of `step` | Reads directed target, sets signed commanded speed; releases propulsion at target and reports Complete |
| Force limiter | `TrackDeviceForces.cpp`, `evaluateTrackDeviceForces` | Samples each occupied bogie's profile, limits acceleration by directed target error, weights mass, caps force |
| Solver | `TrainPhysics.cpp`, `stepTrain` | Projects external forces and applies gravity/resistance/generalized mass; integrates actual signed velocity |

There is no disconnected link in this path. Target does not enter the initial
velocity state or overwrite the solver's integrated velocity.

# TARGET SPEED SEMANTICS

With authored direction `d = +1` or `-1`, compare `d * signedSpeed` with the positive
target magnitude. Blank means `std::nullopt`; zero/negative targets are rejected
by existing validation. Below target, the launch can propel against an opposite
signed motion; it does not use an incorrect raw signed comparison.

For each occupied bogie, request magnitude `a` becomes:

```text
aLimited = min(a, max(0, target - d * signedSpeed) / fixedTimeStep)
signedAcceleration = d * aLimited
requestedForce = 0.5 * loadedCarMass * signedAcceleration
whole-device force scale = min(1, forceCap / sum(abs(requestedForce)))
```

The fixed timestep remains 1/240 s. No solver tolerances, geometric conventions,
mathematical model, fixed-step policy or serialized identifiers change.

# LSM BEHAVIOR

At/above target, Operations disarms propulsion and reports Complete while enabled
and occupied. It does not brake. Resistance may later reduce speed below target,
at which point Launching resumes; completion is not latched. Disabled or empty
hardware supplies no propulsion. Partial bogie occupancy retains the same mass
weighting and whole-device force cap. A downhill test confirms gravity can
accelerate an already overspeed train while launch applied force remains zero.

# CUSTOM PROFILE INTERACTION

Ordering remains spatial per-bogie Constant/Custom evaluation -> directed target
limiter -> loaded-car mass weighting -> common whole-device force cap -> native
train solver. The limiter constrains the sampled request without editing the
authored profile or substituting a speed servo. A zero Custom command remains
zero even below target. Constant commands use the same limiter. Reverse does not
mirror/rewrite the spatial profile; it encounters authored station order backward.

# OTHER HARDWARE CHECK

Transport, Chain and Cable use the existing shared `servo` in RideOperations,
which clamps signed speed error divided by fixed timestep to an acceleration
limit. They do not reuse the launch's one-sided propulsion limiter and can slow
an overspeed train. Tests change only commanded speed from 2 to 6 m/s for each
family and check physical response. Reverse Transport also approaches -2 m/s.
Chain/Cable expose forward speeds, rather than an authored reverse policy.
Cable Custom supplies the servo's acceleration bound at the train-reference
station. Lift cycles, availability, anti-rollback and gravity feed-forward stay
unchanged; existing scenario tests still cover them.

# TELEMETRY

Two transient native telemetry values are added: sampled Launch profile
acceleration before target limiting, and target-speed-limited status. Operations
records them; copied JSON snapshots expose `profileAcceleration` and
`targetSpeedLimited`; managed records default missing values to zero/false.
Fixed-layout ABI structures and saved documents are unchanged.

The launch panel displays accepted target or **unset**, direction, signed actual
speed, state, bogie/car occupancy, profile request, limited propulsion request,
signed applied force and active/inactive limiting. Target is read from accepted
configuration, so an unapplied draft cannot masquerade as runtime configuration.
Acceleration values are magnitudes, not net measured train acceleration; force
includes direction and force saturation. Sampling/limiting occurs per occupied
bogie and the displayed acceleration is its arithmetic mean.

The force samples describe the start of the last physics tick. After Reset,
profile/force telemetry is zero until a playback tick samples it; operating state
and target comparison already reflect the reset pose. No extra UI panel or
framework is added. The Target speed label is retained with a concise explanation
of propulsion limiting, no velocity assignment/braking, blank and Apply semantics.

# INITIAL SPEED REGRESSION

New native preview and managed seam tests author Initial Speed = 5 and 20 m/s
with the same 10 m/s launch target. Rebuild/start and Reset restore the authored
initial velocity and tick zero; the hardware target remains 10. Playback above
target reports Complete, positive profile request, zero propulsion/force and
actual speed above target. The two concepts remain separate.

# TEST RESULTS

Release full native build passed. Focused verification passed:

- `QuantumCore.TargetSpeed`: 26,825 checks; forward/reverse, unset/10/25,
  below-current-speed, Constant/Custom/zero, force caps, loaded mass, partial
  occupancy, gravity and Transport/Chain/Cable comparisons.
- `QuantumCore.RideOperations`: 152,759 checks; existing four scenarios unchanged.
- Track/hardware group: 4/4 (`QuantumCore.TrackDevices`,
  `QuantumCore.TrackHardwarePlacement`, `QuantumApplication.RideHardwareAuthoring`
  with 150 checks, `QuantumApplication.HardwareBoundary` with 43 checks).
- Managed Release builds: zero warnings/errors. Full managed suite passed:
  Hardware 143, Timeline/controller 126, Supports 1,078, Placement 92 checks,
  plus existing wrapper checks. Managed launch exits were unset = 21.019632,
  10 = 9.922892, 25 = 21.019632 m/s with the same 4 m/s² Custom profile.
- Simulation regressions: 5/5 (`QuantumEditor.SimulationPreview`,
  `QuantumCore.TrainPhysics`, `QuantumCore.TrackFollowerPhysics`,
  `QuantumCore.ExternalForcePhysics`, `QuantumCore.ResistancePhysics`).

Exact commands from the repository root (shell redirection to the named logs is
omitted here for readability):

```powershell
cmake --build build --config Release --parallel 1
cmake --build build --config Release --target QuantumCoreRideOperationsTests --parallel 1
ctest --test-dir build -C Release -R '^QuantumCore.TargetSpeed$' --output-on-failure -V
ctest --test-dir build -C Release -R '^QuantumCore.RideOperations$' --output-on-failure -V
ctest --test-dir build -C Release -R '^(QuantumCore\.(TrackDevices|TrackHardwarePlacement)|QuantumApplication\.(RideHardwareAuthoring|HardwareBoundary))$' --output-on-failure -V
dotnet build editor/managed/Quantum.Managed.Tests/Quantum.Managed.Tests.csproj -c Release --nologo
& editor/managed/Build.ps1 -Configuration Release -UiOnly -Test
ctest --test-dir build -C Release -R '^(QuantumEditor\.SimulationPreview|QuantumCore\.(TrainPhysics|TrackFollowerPhysics|ExternalForcePhysics|ResistancePhysics))$' --output-on-failure
ctest --test-dir build -C Release --parallel 4 --output-on-failure
git diff --check
```

Logs are under ignored `build/`: `target-speed-build.log`,
`target-speed-test-build.log`, `target-speed-focused.log`,
`target-speed-operations.log`, `target-speed-hardware.log`,
`target-speed-managed-build.log`, `target-speed-managed.log`,
`target-speed-simulation.log`, and `target-speed-full-ctest.log`.
Full enabled native suite: **115/115 passed**, 134.52 s. The pre-existing
`QuantumEngine.GpuTrainPoseResidency` test was disabled and did not run.
`git diff --check` passed. No existing assertion was weakened or test disabled.

## Single bounded smoke

One existing `--verify-velocity-smoke` was launched after automated verification.
It exited 0; its report records success, 0.3896828 s, 13 checks and five embedded
Vulkan viewport frames. It verified the real Transport speed TextBox binding,
draft preservation through response Apply, committed native speed = 6 m/s and
physical solver response = 5.9987007269 m/s. No retries, dragging or extra manual
UI campaign followed. It did not manually exercise the Launch field or inspect
the new text layout; those remain maintainer acceptance items.

Invocation used the built `Quantum.Editor.exe --verify-velocity-smoke
C:\Dev8\QUANTUM\build\target-speed-velocity-smoke.json`, through PowerShell
`Start-Process -WindowStyle Hidden` with a 20-second process timeout. The ignored
report is `build/target-speed-velocity-smoke.json`. `physicalInputVerified` is
false and maintainer approval is pending. This is programmatic smoke evidence,
separate from human physical/visual validation. Debug/non-Windows builds were
not run for this task.

# KNOWN LIMITATIONS

The bench is deterministic automated physical evidence, not validation of the
maintainer's exact ride or its visuals. Launch target constrains this hardware's
propulsion; gravity, curvature and independently authored overlapping equipment
still reach the solver. It is not a global hard velocity clamp. Force saturation,
resistance and partial occupancy can prevent reaching a target. Existing single
preview train, bogie-sampled occupancy and simplified lift mechanics remain.
No extended computer control is used; final physical/visual acceptance belongs
to the maintainer. Supports and their future materials/UV overhaul are untouched.

# LOCAL HANDOFF

Branch: `codex/operations-ride-hardware-2`. Starting HEAD: `5002d53`.
No reset, push, PR, dependency or next milestone. Accepted local work and the
pre-existing untracked concrete/fiberglass/grout/metal material directories are
preserved. This document is included in the implementation/audit commit; its SHA
is recorded in the final chat handoff and can be read with `git log -1 --oneline`.
Final working tree: no tracked changes; only the four pre-existing untracked
material directories listed above. Nothing was pushed and no PR was opened.

Changed-file manifest (15 files):

- `core/include/quantum/physics/RideOperations.hpp`
- `core/include/quantum/physics/TrackDeviceForces.hpp`
- `core/src/physics/RideOperations.cpp`
- `core/src/physics/TrackDeviceForces.cpp`
- `docs/ride-hardware-operations.md`
- `docs/target-speed-audit-local-handoff.md`
- `editor/managed/Quantum.Editor/EditorViewModel.Hardware.cs`
- `editor/managed/Quantum.Editor/HardwareInspector.axaml`
- `editor/managed/Quantum.Managed.Tests/HardwareChecks.cs`
- `editor/managed/Quantum.Managed/Session.Hardware.cs`
- `editor/native/QuantumHardwareBoundaryTests.cpp`
- `editor/native/QuantumNative.cpp`
- `tests/CMakeLists.txt`
- `tests/RideOperationsTests.cpp`
- `tests/SimulationPreviewTests.cpp`

Ownership is unchanged: the document owns configuration, SimulationPreview owns
Operations and solver state, Operations owns transient value telemetry, and the
ABI copies JSON to managed code. No new pointer/resource lifetime exists.

Read now: `RideOperations::step`'s Launch branch,
`evaluateTrackDeviceForces`, and `targetSpeedComparisons` in the existing native
test file. Useful C++ concepts are `std::optional` (unset), `std::get_if` (typed
configuration), value-owned state and read-only `std::span` inputs. Solver
generalized-mass internals and strict JSON helper templates can wait.
