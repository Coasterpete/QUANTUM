# Ride Hardware and Operations

Ride hardware is authored simulation equipment. The document owns configuration;
the native simulation owns occupancy, operating state, interlocks and commands.
The Avalonia workbench authors and observes this equipment. The existing
specialized train solver remains authoritative at 1/240 second.

## Responsibilities and ownership

`AuthoredTrack::trackDevices()` retains its stable-ID collection and existing
Launch/Brake fields. `TrackDevice` adds a `HardwareConfiguration` variant. A
variant holds one family's configuration at a time; it does not hold all possible
hardware parameters. `TrackInfrastructure` describes access structures in the
same authored collection, with a separate infrastructure ID allocator.

`SimulationPreview` owns a `RideOperations` value, solved train pose, compiled
physics track and transient force controls. Operations keeps a configuration
snapshot for validating commands and a vector of runtime entries, indexed in
accepted document order. No runtime entry owns document, Vulkan, Avalonia, Jolt
or PhysX resources. Rebuilding or resetting the preview restores authored defaults.
The fallback's device-only update preserves its existing train position/playback
and rebuilds hardware state against the current pose.

`ApplicationService` validates and commits authored changes through the existing
whole-document transaction/history path. Runtime commands address stable hardware
IDs and never create document revisions or Undo entries. ABI 5 exposes copied,
revision-checked QUANTUM values; managed code contains no train physics.

## Fixed-step data flow

```text
Authored hardware + solved train bogies + signed speed
    -> occupancy for every hardware object
    -> sensors / operating state / referenced interlocks
    -> acceleration or speed-controller command
    -> occupied-bogie, force-limited external force applications
    -> stepTrain (1/240 second)
    -> new train pose and hardware telemetry
```

All occupancy is recomputed before any station or transfer evaluates references,
so behavior does not depend on the order of blocks/gates in the document.
Telemetry describes the command and pose at the start of the last physics tick.
Runtime commands recompute current readiness without advancing simulation time.
Mechanical motion advances during native playback; paused/stopped commands can
request motion, but do not invent elapsed time.

## Occupancy and sensors

`TrainObservation` contains an explicit train ID, reference station, signed speed,
and every front/rear bogie with car and bogie identity. `observeTrain` derives it
from the existing solved pose. The production preview supplies its current train
as ID 1; the occupancy API accepts multiple observations and never assumes a
single train ID globally.

Occupancy records occupied bogies, distinct cars per train, train IDs and complete
occupancy when all that train's observed bogies are inside the interval. Entry and
exit events compare train-ID sets across consecutive evaluations. Disabled
hardware still senses occupancy. Occupancy models bogie contact stations, not a
swept train-body volume or continuously swept sensor between ticks.

The existing station conventions remain authoritative: half-open intervals;
closed-circuit `start > end` wraps through zero; open tracks include the physical
terminal endpoint when the device ends there. Invalid/empty intervals, duplicate
IDs, dangling gate/block references, and malformed configuration are rejected.
Multiple trains in one force/mechanism zone interlock that zone rather than sharing
one ambiguous force command. Blocks can report multiple trains independently.

## Functional families

| Family | Authored configuration | Native behavior |
| --- | --- | --- |
| Launch | Existing Constant/Custom acceleration and whole-device force cap; optional target speed and forward/reverse policy | Bogie-weighted powered force; stops commanding after directed target speed; reports completion while occupied |
| Brake | Existing Constant/Custom deceleration; trim speed and holding option | Opposes signed motion, including rollback; trims toward the target; optional zero-speed holding controller |
| Transport | Positive speed magnitude and forward/reverse policy | Drives toward signed commanded speed, subject to acceleration and force limits |
| Chain lift | Chain speed, anti-rollback | Available -> Engaging -> Pulling -> Releasing; engaged, force-limited anti-rollback remains available when drive is unavailable |
| Cable lift | Pull speed, optional acceleration profile, initial catch availability, return duration | Available -> Engaging -> Pulling -> Releasing -> Returning -> Available; unavailable catch blocks drive |
| Station | Stop reference station, alignment/stopped tolerances, transport speed, block and gate references | Positions/stops/holds a train, queues dispatch requests, evaluates permissives, transports a permitted departure |
| Block | Interval and enabled availability | Clear/occupied, train IDs and entry/exit sensing; can deny station dispatch |
| Transfer | Alignment IDs/names/offsets/destination kinds, home ID, movement speed, stopped tolerance, gate references | Secures train using native forces; verifies gates; unlocks, moves, aligns and locks; exposes real lateral mechanism position |
| Access | Role, swing/slide, signed travel direction, angle/distance, duration, offsets/yaw, initial states, optional mesh reference | Closed/Opening/Open/Closing, lock and explicit/operational interlocks; normalized opening position telemetry |

The force path preserves per-bogie mass weighting (`0.5 * loaded car mass *
commanded acceleration`) and applies one common scale to respect the whole-device
sum-of-force-magnitudes cap. Legacy Launch/Brake custom profiles continue to be
evaluated at each occupied bogie's local distance, including circuit wrapping.
Zero profile commands produce finite zero forces.

Launch Target Speed is an optional positive speed magnitude in the authored
direction. It limits propulsion, not the train's final velocity. For direction
`d` (+1 forward, -1 reverse), each occupied bogie's Constant/Custom acceleration
`a` is constrained to `min(a, max(0, target - d * signedSpeed) / (1/240))`.
The directed command then becomes `0.5 * loadedCarMass * acceleration` at each
occupied bogie; the whole-device force cap applies afterward. Operations releases
propulsion and reports Complete at or above the directed target, without braking.
It can resume Launching if resistance or an incline slows the train below target.
A blank target removes the propulsion limit; a zero target is rejected. A short
launch may exit below target. Gravity or other devices can carry a train above
target; this is not a global speed ceiling or a guaranteed exit speed.

Speed controllers compute a bounded acceleration request from signed speed error.
Station approach also limits the requested speed by braking distance to the stop
reference. These are ride-level controllers, not final-velocity assignments.
Controller forces include feed-forward compensation using the existing Z-up
world gravity and tangent Z component. The specialized solver still applies mass,
gravity, resistance, track geometry and force limits. Resistance and partial
occupancy can cause speed droop; insufficient force can prevent a target being
reached. Default alignment tolerance is 0.15 m; stopped tolerance is 0.05 m/s.
No geometry solver, frame convention, train integration tolerance or fixed-step
policy is replaced.

Cable Custom acceleration currently limits the speed controller using the train's
reference distance within the cable interval. It is a ride-level catch-car cycle,
not cable tension, individual links, catch geometry or a detailed return trajectory.

## Stations and gate interlocks

A station permits dispatch when enabled, with exactly one fully occupied train,
its reference station aligned, signed speed within the stopped tolerance, its
linked downstream block clear/enabled, and all referenced gates enabled and
closed. Locked gates are required by default and may be configured separately.
An explicitly interlocked gate is unavailable for dispatch.

A request waits until those conditions hold; it does not bypass them. During
requested departure the station protects its gates against unlock/open commands.
If departure readiness is lost it commands a stop and reports the reason. The
station recognizes its own departing train in the downstream block, while another
train continues to deny departure. Multiple-train intrusion revokes permission
and drive while preserving gate protection.

Block ID zero and an empty gate list deliberately describe an unlinked station;
the form states that choice. There is no passenger/restraint simulation or
safety-certified controls claim. Station force control is self-contained. Adjacent
or overlapping drive/brake devices remain independently authored and their force
applications sum in the solver; the author must configure overlaps deliberately.

## Transfer, storage and mechanics boundary

Alignment records provide meaningful MainTrack, Storage and Maintenance
destinations, persistent IDs, names and lateral offsets. Move requests require a
locked mechanism and either an empty section or a single completely occupied,
stopped train, plus enabled/closed/locked access gates. Invalid destinations and
requests while already moving/unlocked are rejected without changing the target.

The native phases are Locked -> Unlocking -> Moving -> Locking -> Locked. Movement
integrates the authored speed toward the destination offset; the final position
snaps only within a 1e-9 m alignment tolerance. Loss of enable, train security or
gate readiness freezes the mechanism in Interlocked and protects its gates.
The selected native viewport outline follows actual runtime lateral position.

This milestone is a kinematic mechanism/state proof. It does not move the train
onto alternate track geometry, attach the train body to the moving table, simulate
rigid-body inertia/collision, or provide storage traversal. The train and its GLB
remain on the primary compiled path. A locked storage alignment is observable
alignment readiness, not proof that a train can traverse an unimplemented route.

The current mechanics boundary is QUANTUM-owned target/alignment/position/state
values. No backend objects enter the domain, document, Timeline, managed ABI or
future scripting seam. A future backend may consume these commands and return
position/alignment observations; that integration requires a separate concrete
milestone. This work performs no PhysX migration or Jolt-specific operations API.

## Access hardware and infrastructure

One `AccessConfiguration` serves station, queue/loading, maintenance, catwalk,
transfer, storage and barrier roles. Appearance does not determine operation:
mesh/chain-link appearance, a hinged door and a sliding storage door need no
separate hard-coded runtime classes. Unlock/open are rejected while operationally
protected or explicitly interlocked. Locking is allowed only when fully closed;
closing remains possible under an interlock. Disabled mechanisms do not move.
Initial open plus locked is rejected as contradictory authored state.

Swing presentation uses signed open angle around the track-frame up axis. Slide
presentation uses signed travel distance in the closed yaw direction. Runtime
position is normalized [0,1]; authored offsets, yaw and appearance survive save,
reload and history. Optional GLB paths are authored appearance hooks; this
milestone draws explicit engineering outlines and does not load final gate meshes.

`TrackInfrastructure` persists Catwalk, Platform, Guardrail, Stairs and Landing
records, station ranges, left/right side, lateral/vertical offsets, width,
solid/grating deck style, inner/outer railings and height, gate references and an
optional mesh path. Gate objects retain independent functional identities.
Infrastructure has native transaction and managed API hooks. There is no
procedural deck/rail/stair generator or dedicated infrastructure editor yet;
authored stair records do not imply generated risers/treads. Units are SI metres;
left is negative track-frame lateral, right is positive.

## Authoring, presentation and telemetry

The existing Timeline remains intact and gains a Ride Hardware catalog and
hardware-specific inspector. Launch/Brake/Cable retain Constant/Custom profile
editing. Transport, Chain, Station and Transfer expose response/acceleration limits
and speed parameters. Station, Block, Transfer and Access show state/configuration
content without an acceleration graph. Transfer destinations have their own
editable name, offset, kind, home selection and move target.

Velocity edits remain pending until Apply hardware configuration. Applying
placement/force or Constant/Custom/response settings preserves the pending speed
and target fields for the same selected hardware after successful publication.
Rejected edits still restore accepted form values, and selection/Undo/Redo refresh
from authored configuration. These buttons do not silently commit a velocity edit.

Selected hardware retains interval/end markers. Station adds its stop marker;
Transfer adds alignment markers and a moving table outline; Access adds a moving
engineering gate outline. These are display-only native line vertices, generated
from solved centerline frames and the existing SI-to-coordinate scale. No Vulkan
ownership, renderer lifetime, GLB train renderer or selection architecture changes.

Snapshots expose state/reason, enabled availability, bogie/car counts, train IDs,
entry/exit, signed motion direction, actual/commanded speed, acceleration, force,
station readiness/dispatch, transfer current/target alignment and position, and gate
position/lock/interlock. Managed forms display copied native telemetry and route
revision-checked commands; they do not decide whether dispatch is permitted.
The ImGui fallback retains its established Launch/Brake authoring and can load and
preserve the added hardware configurations. The complete new family forms live in
Avalonia.

Launch telemetry displays the accepted target (including unset), signed actual
speed, state/occupancy, the mean sampled profile acceleration before limiting,
the mean propulsion acceleration after limiting, signed force after the device
cap, and whether target limiting is active. The acceleration values are request
magnitudes, not measured net train acceleration. Profile sampling remains spatial
in authored station order, so reverse travel encounters a varying profile in the
opposite order. Limiting is active when at least one occupied bogie's positive
request is reduced or released by the target. These are transient copied JSON
fields; no saved document or fixed-layout ABI structure changes. Force telemetry
is sampled on playback ticks; immediately after Reset, profile/force values are
zero until the next tick.

## Reset, persistence and history

Reset restores authored enabled defaults, clears pending dispatch, recomputes
occupancy, returns cable catch availability/home cycle, restores chain availability,
homes and locks the transfer, and restores gate initial open/lock/interlock state.
No previous run's motion requests or gate protection survive Reset.

The outer `.quantum` format follows the existing formatVersion 1 optional-field
convention. `trackDevices.schemaVersion = 2` explicitly versions the extended
hardware collection. Old Launch/Brake documents without the nested version/config
load with legacy semantics. Legacy acceleration/profile fields and IDs are retained.
The new fields are configuration, not renamed/reinterpreted old data. Unknown
versions/fields, non-integral or out-of-range IDs, inconsistent variant kinds and
invalid references are rejected. New typed configuration/infrastructure requires
schema 2. Older binaries with strict readers cannot read newly saved schema-2
collections; historical documents remain readable by this build.

Transient runtime state is never serialized. Native whole-document history covers
creation/deletion, placement, enable, force, custom profile, typed configuration,
station links, transfer destinations, gate mechanics and infrastructure. Referenced
gates/blocks cannot be silently deleted; remove the authored references first.
Candidate validation occurs before preparation/publication so rejected edits retain
the committed document. Stable hardware, infrastructure and transfer-alignment allocators do not reuse
deleted IDs and reject exhaustion. A missing historical alignment counter is
derived from the existing IDs without changing them.

## Future scripting seam and current limits

Future Lua can bind stable-ID operations such as enable/availability, dispatch
request, transfer move and gate open/close/lock commands and read the same native
sensor/state snapshots. Additional concrete runtime parameter commands can be
added here without exposing implementation pointers or replacing the document
model. Lua itself and a general programmable PLC are not implemented.

The current preview still simulates one train. Multi-train identity and occupancy
are foundations, not completed scheduling/reservation/collision management. Block
permission is evaluated live; there is no multi-station atomic block reservation.
All moving access/transfer hardware is kinematic at this milestone. Continuous
sensor sweeps, detailed catch mechanics, final gate assets, procedural catwalks and
alternate-path train traversal remain separate milestones.

## Reading path

READ NOW:

1. `core/include/quantum/coaster/RideHardware.hpp`: small typed configurations and infrastructure values.
2. `core/include/quantum/coaster/TrackDevices.hpp`: retained authored identity/interval/profile contract.
3. `core/src/physics/RideOperations.cpp`: `step` occupancy pass and Station/Transfer/Access branches; `command` interlocks.
4. `core/src/physics/TrackDeviceForces.cpp`: occupied-bogie commands, Z-up compensation and force scaling.
5. `editor/src/SimulationPreview.cpp`: Reset and the short operations -> forces -> `stepTrain` call path.

OPTIONAL:

6. `editor/managed/Quantum.Editor/EditorViewModel.Hardware.cs`: forms route native configuration/runtime commands.
7. `tests/RideOperationsTests.cpp`: four integrated scenarios and adverse-condition assertions.
8. `tests/RideHardwareAuthoringTests.cpp`: history/persistence and runtime exclusion.

DEEP INTERNAL:

9. `core/src/RideHardwareJson.cpp`: strict typed configuration serialization. Its small type-aware helpers can wait until the domain/runtime path is understood.
10. `editor/native/QuantumNative.cpp`: bounded UTF-8 ABI snapshots and revision checks; no C++ pointers cross the boundary.
