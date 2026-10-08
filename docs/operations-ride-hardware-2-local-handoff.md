# MAINTAINER SUMMARY

Ride Hardware now means authored equipment with native occupancy, operating state,
interlocks, commands and telemetry. Launch, Brake, Transport, Chain Lift, Cable
Lift, Station and Block have functional train/operations behavior. Transfer has a
native lateral mechanism and lock state machine with persistent storage and
maintenance destinations. Access hardware uses one reusable swing/slide model
for gates, doors and barriers.

The native simulation reads solved bogie stations, evaluates all occupancy first,
then evaluates hardware and referenced interlocks. Train-affecting equipment
supplies force-limited external forces to the existing specialized solver at
1/240 second. It never assigns final train velocity from the UI.

The transfer proof does not attach the train to the moving table or route it onto
storage geometry. Catwalks, platforms, guardrails, stairs and landings have saved,
validated authoring hooks, without procedural generation or a dedicated editor.
Optional access GLB paths are appearance references; current presentation uses
engineering outlines.

Automated results establish the native behavior and authoring/ABI workflows below.
The sole bounded GUI smoke failed its initial Vulkan presentation check before
exercising hardware controls. Maintainer visual and operational approval remains
pending. Please manually judge launch/brake/rollback feel, lift engagement,
station dispatch, gate protection, Timeline usability and selected mechanism
presentation.

# WHAT YOU CAN DO NOW

Use the existing Timeline's Ride Hardware catalog to create all nine families,
edit intervals/enabled/force settings and the appropriate family configuration,
inspect native telemetry, and save/reload or Undo/Redo authored edits. Runtime
commands can change availability, request dispatch, move a transfer and operate
access hardware without changing the document.

Simulation playback advances gate/table motion. A command while paused requests
motion but does not fabricate elapsed simulation time. Infrastructure is currently
available through native/managed authored APIs rather than an Avalonia form.

# RIDE HARDWARE MODEL

`TrackDevice` retains stable identity, name, interval, enabled state, acceleration
and force-limit fields. `HardwareConfiguration` is a `std::variant`: it stores
one family's configuration, rather than every family's fields in one structure.
`std::monostate` preserves historical Launch/Brake configuration semantics.

`SimulationPreview` owns `RideOperations` by value. Operations owns an authored
configuration snapshot and separate transient runtime entries; it owns no Vulkan,
managed UI or general-physics backend resources. Documents/history contain
configuration values, not runtime state or implementation pointers.

Before: interval/profile -> occupied-bogie forces -> train solver.
After: bogie observations -> occupancy/sensors -> operating state/interlocks ->
force command -> existing train solver -> telemetry. `ApplicationService` remains
the authored transaction/history boundary; Avalonia remains an authoring/observing
client.

Useful C++ concepts here are value ownership, `std::variant`/`std::get_if` for typed
state, `std::span` for temporary read-only views, scoped phase enums, `const`
references and validation before committing a candidate. No new polymorphic
hierarchy or one-class-per-gate design is required.

# OCCUPANCY / SENSORS

Observations include explicit train IDs, signed speed and front/rear bogie stations
with car/bogie identities. Runtime occupancy distinguishes partial/full occupancy,
counts cars and bogies, and reports train entry/exit and clear/occupied states.
Disabled equipment still senses. Existing half-open and wrapped circuit interval
conventions remain intact. Blocks accept multiple train identities; ambiguous
multi-train force zones interlock. This is bogie sampling, not swept body detection.

# LSM LAUNCH

Existing Constant/Custom acceleration reaches occupied bogies with the established
mass weighting and whole-device force cap. Typed configuration adds directed
forward/reverse launch and optional target speed/completion. Custom profiles keep
their per-bogie local-distance evaluation and existing transition functions.

# BRAKING

Brakes oppose signed motion, including negative-speed rollback. Configuration adds
trim speed and optional holding. Holding requests zero speed through forces;
finite force and resistance determine the actual response. This is ride-level
braking, without separate electromagnetic or friction material simulation.

# DRIVE TIRES / TRANSPORT

Transport uses signed speed error to request bounded acceleration toward authored
speed and direction. It can propel or slow a train, subject to its response and
force limits. Gravity feed-forward uses the existing Z-up world and track tangent;
the coaster solver retains gravity, resistance, mass and geometry authority.

# CHAIN LIFT

Occupancy drives Available -> Engaging -> Pulling -> Releasing. An engaged lift
drives toward chain speed. The available/disabled state suppresses powered pull;
configured, engaged anti-rollback remains a force-limited mechanical holding
concept. Real inclined-track dynamics are exercised in the native scenario test.
Individual links/pickup geometry are not simulated.

# CABLE LIFT

Cable has its own availability, engagement, pull, release and timed catch-return
cycle. An unavailable catch prevents pull. Optional Custom acceleration limits
the speed controller using train-reference distance in the interval. It is a
catch-car operations model, without detailed cable tension or return geometry.

# STATION

Station forces approach, align, stop and hold the train at an authored reference
station. Dispatch requests wait for full single-train occupancy, alignment,
stopped tolerance, enabled station, clear/enabled downstream block and closed
enabled gates; locking is required by default. Departure protects linked gates
against opening/unlocking and recognizes its own train entering the next block.
Another train revokes departure permission. No passenger/restraint simulation is
included; overlapping force equipment still sums independently.

# BLOCK LOGIC

Blocks expose clear/occupied and entry/exit identities and provide live station
permissives. All occupancy precedes reference evaluation, avoiding document-order
dependence. Preview still runs one train; scheduling, atomic reservations and
multi-train collision management are future work. No safety certification is
claimed.

# TRANSFER / STORAGE

Transfer configurations author stable alignment IDs, names, offsets, MainTrack /
Storage / Maintenance kind and home selection. A move requires an empty table or
one fully occupied stopped train, plus closed locked gates. Native phases are
Locked -> Unlocking -> Moving -> Locking -> Locked; loss of permissives freezes
motion and exposes Interlocked. The mechanism actually moves to the requested
lateral position and locks. Repeated/invalid/unsecured moves are rejected.

Alternate geometry, train attachment, body inertia/collision and storage traversal
are not implemented. The train and GLB remain on the primary path. QUANTUM-owned
target/position/alignment values form the current general-mechanics seam; this
work introduces neither PhysX migration nor Jolt-specific public types.

# ACCESS HARDWARE / CATWALK INFRASTRUCTURE

One typed Access model covers station, queue/loading, maintenance, catwalk,
transfer, storage and barrier roles. It supports swing/slide, signed direction,
angle/travel, duration, offsets/yaw, enabled state, open/closed, locked/unlocked,
explicit/operational interlocks and runtime position. Appearance can be authored
independently, including a future mesh/chain-link GLB. Closing remains possible
under an interlock; only a closed mechanism can lock.

Infrastructure records persist station ranges, left/right side, lateral/vertical
offsets, width, solid/grating deck, railing configuration and functional access
gate references. Authored ranges retain existing track conventions and SI units.
They are hooks for later catwalk/platform/stair presentation, not generated assets.

# TIMELINE / AVALONIA UI

The existing Timeline, profile dragging and Transition Editor remain. Launch,
Brake and Cable expose profiles; other families show appropriate configuration
and state content. Forms route native configuration and runtime commands, including
station links, transfer destination authoring and gate operations. ABI 5 adds
bounded copied JSON snapshots and revision-checked functions without changing old
struct layouts. Native and managed binaries must be deployed together.

The ImGui fallback retains established Launch/Brake authoring and preserves loaded
new configurations; complete new-family forms are in Avalonia.

# VIEWPORT PRESENTATION

Selected interval/end markers remain. Added display-only line geometry shows
station stop position, transfer destinations/table motion and swing/slide gate
motion from native state. Geometry uses solved track frames and the established
SI-to-coordinate scale. Renderer resource ownership, publication, GLB trains and
Vulkan lifetime policy remain unchanged. Automated vertex tests establish motion;
live appearance is not approved by the failed bounded smoke.

# TELEMETRY

Snapshots expose state/interlock reason, enabled/available state, train identities,
car/bogie counts and events, signed direction, commanded/actual speed, acceleration,
applied force, station readiness/dispatch, transfer alignment/position and gate
position/lock/interlock. Telemetry corresponds to the start of the last train
physics tick. Future Lua can bind these QUANTUM values and stable-ID commands;
Lua itself is not implemented.

# RESET

Reset restores authored enabled and gate defaults, clears dispatch/protection,
homes/locks transfer, restores chain/cable availability and recomputes occupancy.
Fallback device-only updates retain the current train position/playback while
reinitializing hardware against that pose. Ordinary preview rebuilds retain the
existing reset policy.

# UNDO / REDO

Native whole-document history covers all hardware creation/deletion, placement,
enabled/force/profile changes, typed settings, station references, transfer
destinations, gate mechanics and infrastructure. Runtime commands/transitions
create no authored revisions or history entries. Referenced objects cannot be
silently deleted. Transfer and infrastructure edit APIs preserve monotonic ID
allocation, including after destination removal; Undo restores complete snapshots.

# SERIALIZATION

The outer document remains formatVersion 1 following its existing optional-field
convention. The extended collection explicitly uses `trackDevices.schemaVersion`
2. Historical Launch/Brake fields, IDs, Constant/Custom data and behavior remain
readable. Unknown schema/configuration fields and invalid references are rejected.
A missing transfer alignment counter derives from existing IDs without renaming.
Runtime state is not saved. Older strict binaries cannot read newly saved schema-2
collections; retain historical originals if using older executables.

# INTEGRATED SCENARIOS

- A: native station -> tires -> chain -> coast -> trim -> arrival station,
  plus a separate real uphill chain case; engagement/release, arrival tolerances,
  downstream/gate permissives and anti-rollback assertions.
- B: holding/release -> Custom launch -> braking -> station; force caps,
  disabled/insufficient launch, physical rollback braking and reverse propulsion.
- C: unavailable catch -> available -> engagement -> native pull -> release ->
  timed return, with force-limited physical train response.
- D: physically stop/secure train -> verify gates -> unlock -> lateral movement
  to storage -> align/lock; invalid/unsafe commands and Reset. The test explicitly
  asserts that alternate-path traversal has not occurred.

These deterministic tests use a one-car train and short compiled track benches
with the production `stepTrain` solver and 1/240 s steps. A/B's integrated bench
has a mild downhill grade; the separate chain case tests an actual uphill grade.
They are controlled integration tests, not a fully authored multi-train ride.

# PERFORMANCE

The final test run measured one observed train with 10/50/100 occupied blocks
at 0.0014966 / 0.0072659 / 0.014643 ms per operations tick. The test
runs 1,000 evaluations per count. It excludes train solving, rendering, interop and
mixed-family/reference-heavy workloads. There is no before/after optimization
claim. Allocation/copy costs remain measurable future work if representative
rides justify it; no threading/job/caching subsystem was added.

# REGRESSIONS / AUTOMATED CORRECTNESS

Final full native Release build passed. All 114 enabled CTest tests passed
(195.61 s); `QuantumEngine.GpuTrainPoseResidency` was already disabled and did not
run. The new tests passed: RideOperations 152,759 assertions (many are per-tick
force-cap checks), RideHardwareAuthoring 150 checks, HardwareBoundary 42 checks.
All four integrated scenarios passed. `git diff --check` passed.

Final managed Release builds passed with zero warnings/errors. The full managed
suite passed: Timeline/controller 126 checks, Hardware 79, Supports 1,078,
Placement 92, and existing wrapper checks.

Existing custom profiles, force conventions, geometry/solver tolerances, Supports,
support picking, Track Start/global placement, Route/Inspector, train configuration,
GLB rendering, icon and fallback were preserved. Broad regression coverage provides
automated evidence; no Linux GUI or manual visual campaign was performed.

# CODEX BOUNDED SMOKE

Exactly one `--verify-hardware-smoke` run was attempted. Its report records
`success: false`, elapsed 0.4356477 s and no completed checks. It failed the first
assertion, "Embedded Vulkan presents frames", before testing new controls. The
process exited; no retry, screenshots or extended computer control followed.
The startup failure's root cause was not established. Physical input was not
verified. The ignored local report is `build/operations-hardware-smoke.json`.

# MAINTAINER APPROVAL

Pending. Manually judge station/gate workflow, launch/brake/rollback feel, lift
behavior, hardware-state clarity, Timeline usefulness and viewport/transfer
presentation. The bounded startup presentation failure also needs assessment.

# KNOWN LIMITATIONS

Transfer is kinematic and cannot carry/route the train. Access motion is kinematic;
GLB loading/final art and infrastructure generation/editing are deferred. Cable and
chain are ride-level models. Preview remains single-train without atomic block
reservations. Sensors sample bogies rather than swept bodies. Overlapping authored
force zones require deliberate configuration. Force saturation, partial occupancy
and resistance can cause target-speed droop. ABI 5 requires matching binaries.
New saved documents cannot be read by older strict schema readers. GPU residency
and live visual behavior are not fully verified.

# NEXT MAJOR PHASE

After maintainer acceptance, the next integrated milestone should be transfer
route/attachment semantics: define alternate compiled paths, carry the secured
train with the table, and permit traversal only after a destination is aligned
and locked. That needs an explicit topology/ownership decision before choosing
mechanics integration. It has not been started here.

# OPTIONAL CODE READING PATH

READ NOW:

1. `core/include/quantum/coaster/RideHardware.hpp`: typed authored values.
2. `core/include/quantum/coaster/TrackDevices.hpp`: retained identity/interval/profile.
3. `core/src/physics/RideOperations.cpp`: `step` occupancy, Station/Transfer branches;
   `command` gate/dispatch/move checks.
4. `core/src/physics/TrackDeviceForces.cpp`: bogie forces and whole-device cap.
5. `editor/src/SimulationPreview.cpp`: Reset and operations -> forces -> train step.

OPTIONAL:

6. `editor/managed/Quantum.Editor/EditorViewModel.Hardware.cs`: native command routing.
7. `tests/RideOperationsTests.cpp`: physical integration scenarios/adverse conditions.
8. `tests/RideHardwareAuthoringTests.cpp`: exact history/save equality and ID reuse.

DEEP INTERNAL:

9. `core/src/RideHardwareJson.cpp`: strict typed JSON parsing; helper templates can wait.
10. `editor/native/QuantumNative.cpp`: bounded copied snapshots/revision checks.

# LOCAL HANDOFF

- Branch: `codex/operations-ride-hardware-2`.
- Starting accepted HEAD: `52e9e0e481d0c250e179efdcf2b987e0c04792fc` on
  `codex/hybrid-track-start-placement`.
- Implementation commit: `f8766cd63706fe5cb8e24fd3d92a0631d73a1d6c`.
- This handoff is a separate documentation commit; its SHA is recorded in the
  final chat response.
- Suggested future PR title: **Add functional ride hardware operations and access interlocks**.
- No push, PR, dependency, screenshot publication, Supports overhaul, Train workspace
  migration, Lua implementation or general-physics backend migration.
- Pre-existing untracked concrete/fiberglass/grout/metal material directories were
  preserved and excluded from the implementation commits.

Exact final verification commands:

```powershell
cmake --build build --config Release --parallel 1
ctest --test-dir build -C Release --parallel 4 --output-on-failure
dotnet build editor/managed/Quantum.Managed.Tests/Quantum.Managed.Tests.csproj -c Release --nologo
& editor/managed/Build.ps1 -Configuration Release -UiOnly -Test
git diff --check
```

Debug builds of QuantumCore, QuantumApplication, QuantumNative, QUANTUM and the
original device/service test targets passed earlier; the existing device/service
CTest pair passed 2/2. Final validation is Release. The first parallel full native
build hit an existing shared asset-staging file-lock failure; serial full builds
passed. One follow-up command named a nonexistent test target; it was corrected.
No compiler errors were suppressed, assertions weakened or tests disabled.

The new native tests are `QuantumCore.RideOperations`,
`QuantumApplication.RideHardwareAuthoring` and `QuantumApplication.HardwareBoundary`.
The full CTest command also covers existing application, Supports, placement,
Timeline, history, serialization, geometry, train/external-force, renderer and GPU
tests. Local final logs are `build/operations-final-build.log`,
`build/operations-final-ctest.log`, `build/operations-final-managed.log` and
`build/Testing/Temporary/LastTest.log`.

Changed-file manifest:

- `core/CMakeLists.txt`
- `core/include/quantum/coaster/RideHardware.hpp`
- `core/include/quantum/coaster/RideHardwareSerialization.hpp`
- `core/include/quantum/coaster/TrackDevices.hpp`
- `core/include/quantum/physics/RideOperations.hpp`
- `core/include/quantum/physics/TrackDeviceForces.hpp`
- `core/src/AuthoredTrack.cpp`
- `core/src/CoasterDocument.cpp`
- `core/src/RideHardware.cpp`
- `core/src/RideHardwareJson.cpp`
- `core/src/TrackDevices.cpp`
- `core/src/physics/RideOperations.cpp`
- `core/src/physics/TrackDeviceForces.cpp`
- `docs/architecture.md`
- `docs/hybrid-editor-foundation.md`
- `docs/operations-ride-hardware-2-local-handoff.md`
- `docs/ride-hardware-operations.md`
- `editor/CMakeLists.txt`
- `editor/include/quantum/editor/ApplicationService.hpp`
- `editor/include/quantum/editor/SimulationPreview.hpp`
- `editor/include/quantum/editor/TrackDevicePresentation.hpp`
- `editor/managed/Quantum.Editor/EditorApplication.axaml.cs`
- `editor/managed/Quantum.Editor/EditorViewModel.Hardware.cs`
- `editor/managed/Quantum.Editor/EditorViewModel.cs`
- `editor/managed/Quantum.Editor/HardwareInspector.axaml`
- `editor/managed/Quantum.Editor/HardwareInspector.axaml.cs`
- `editor/managed/Quantum.Editor/RuntimeChecks.Hardware.cs`
- `editor/managed/Quantum.Editor/TimelinePane.axaml`
- `editor/managed/Quantum.Managed.Tests/HardwareChecks.cs`
- `editor/managed/Quantum.Managed.Tests/Program.cs`
- `editor/managed/Quantum.Managed/Native.Hardware.cs`
- `editor/managed/Quantum.Managed/Session.Hardware.cs`
- `editor/managed/Quantum.Managed/Session.cs`
- `editor/native/CMakeLists.txt`
- `editor/native/NativeViewport.cpp`
- `editor/native/NativeViewport.hpp`
- `editor/native/QuantumHardwareBoundaryTests.cpp`
- `editor/native/QuantumNative.cpp`
- `editor/native/QuantumNative.h`
- `editor/native/QuantumNativeTests.cpp`
- `editor/src/ApplicationService.cpp`
- `editor/src/EditorUi.cpp`
- `editor/src/SimulationPreview.cpp`
- `editor/src/TrackDevicePresentation.cpp`
- `tests/CMakeLists.txt`
- `tests/RideHardwareAuthoringTests.cpp`
- `tests/RideOperationsTests.cpp`

For the production architecture rather than this verification handoff, read
[Ride Hardware and Operations](ride-hardware-operations.md).
