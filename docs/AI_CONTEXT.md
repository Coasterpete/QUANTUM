# QUANTUM — AI Project Context

This file is a concise continuity snapshot for coding agents.

It does not replace:
- `AGENTS.md` — agent working rules
- `docs/architecture.md` — authoritative technical architecture
- source code and tests — authoritative implementation state

Read `AGENTS.md` first.

If this file conflicts with the current source tree, tests, or
`docs/architecture.md`, inspect the repository and report the discrepancy
rather than blindly following this file.

---

## Project Identity

QUANTUM is a native C++ roller coaster design, simulation, and
visualization application.

The long-term goal is a unified modern coaster-design environment with
high-fidelity track authoring, vehicle simulation, operations, rendering,
terrain/scenery, and extensibility.

It is not a park-management game.

The project is human-directed. AI agents assist with implementation,
testing, debugging, documentation, and analysis. The human developer
retains architectural and product direction.

---

## Current Development Focus

Current major focus:

Native track-constrained roller-coaster vehicle physics.

The physics system is being developed incrementally in numbered phases.

Do not skip ahead into later systems unless explicitly requested.

---

## Completed Physics Milestones

### Phase 1 — Track follower

Implemented:
- compiled physics track
- track location
- deterministic fixed-timestep longitudinal integration
- gravity projection
- aggregate resistance foundation
- circuit/open/shuttle behavior
- basic telemetry

### Phase 2 — Car and bogie posing

Implemented:
- authored car geometry
- loaded COG
- explicit bogies and hitches
- deterministic two-bogie car posing
- travel-oriented bogie frames
- body-relative bogie diagnostics

Current pose solver supports exactly two bogies even though storage is
not architecturally restricted to two.

### Phase 3 — Heterogeneous rigid train dynamics

Implemented:
- heterogeneous train definitions
- rigid inter-car spacing
- one generalized train coordinate
- distributed translational reduced dynamics
- effective generalized mass
- generalized mass derivative
- rigid train pose solving

### Phase 4 — Connector axial load recovery

Implemented:
- signed rigid connector axial loads
- tension/compression diagnostics
- actual connector world-force vectors
- consistency residuals

Connectors remain rigid and axial only.

### Phase 5 — Explicit external force applications

Implemented:
- per-car world forces
- authored local application points
- virtual-work projection
- connector-recovery compatibility

### Phase 6 — Explicit per-car aerodynamics

Implemented:
- per-car effective drag area
- aerodynamic application center
- world wind velocity
- relative-airflow drag
- drag through ordinary ExternalForceApplication path

Aggregate rolling/mechanical resistance remains provisional.

### Phase 7 — Aggregate track reaction recovery

Implemented:
- exact aggregate track-constraint reaction per car
- body-frame projections
- force-balance diagnostics

No front/rear split was fabricated at this stage.

### Phase 8 — Rotational train dynamics

Implemented:
- full symmetric body-frame inertia tensors
- loaded inertia using parallel-axis handling
- angular velocity
- angular acceleration
- rotational kinetic energy
- rotational generalized effective mass
- rotational contribution to train dynamics

### Phase 9 — Front/rear bogie reaction recovery

Implemented:
- conditioned front/rear bogie aggregate resultants
- rigid-body force and moment balance
- ideal reaction planes normal to bogie travel direction
- deterministic rank-revealing solve
- force/moment residuals
- singular/ill-conditioned/unavailable statuses

These are aggregate bogie reactions, not individual wheel loads.

### Phase 10 — Bogie contact geometry and wrench feasibility

Implemented:
- `BogieContactDefinition`
- `WorldBogieContact`
- Running / Guide / Upstop contact roles
- bogie-local contact locations and normals
- frictionless tangent-free contact directions
- force-span feasibility
- full zero-couple wrench feasibility
- rank / conditioning / residual diagnostics
- generic geometry compatible with conventional, SingleRail-like,
  planar, and hybrid arrangements

Phase 10 deliberately does NOT solve contact engagement or final loads.

### Phase 11 — Unilateral rigid-contact allocation

Implemented:
- deterministic Lawson-Hanson active/passive-set NNLS
- full force-and-moment wrench allocation with `lambda >= 0`
- separate Phase 10 span and Phase 11 cone/allocation statuses
- SI force and moment reconstruction and residual validation
- stable deterministic representative allocations
- conservative Unique / NonUnique / Undetermined classification
- separate mathematical nonnegativity and reporting-active tolerances
- Running / Guide / Upstop representative totals by authored role

Phase 11 representative coefficients are not claimed to be true individual
wheel loads when rigid contact makes the allocation nonunique or undetermined.

---

## Current Verified Baseline

Current physics baseline:

Phase 11 complete.

At the completion of Phase 11:

- focused Phase 1–11 regression: 9/9 passed
- complete Debug build: passed
- full CTest: 61/61 passed
- `git diff --check`: passed

Do not treat these counts as permanently authoritative after subsequent
changes. Run the current test suite.

---

## Recommended Next Milestone

Pause additional invisible contact-physics work and expose the existing
simulation in a small application preview:

- visible multi-car train using current car and bogie poses
- Play / Pause / Reset
- train speed and basic physics telemetry
- optional aggregate reaction/contact-allocation debugging

Keep this preview incremental. Do not add suspension, friction, detailed wheel
load sharing, operations, or a new rendering/track-family dependency as part of
the preview milestone.

---

## Important Physics Boundaries

### Track-constrained model

Ordinary coaster motion currently uses a track-constrained generalized
coordinate.

Do not replace this with a general-purpose 6DOF rigid-body collision
simulation without explicit architectural approval.

### Rendering independence

Core physics must remain independent of:
- Vulkan
- rendered rail meshes
- TrackStylePreset
- shader/material systems
- GLB visual topology

Physics-level contact definitions are abstract mechanical geometry.

### Track-family independence

Do not assume:
- exactly two physical rails
- round tubular rails
- one manufacturer
- one conventional wheel arrangement

Future systems must be able to represent:
- tubular steel
- truss-spine
- box-spine
- SingleRail
- wooden
- hybrid/formed-steel
- suspended/inverted arrangements

through generic physics abstractions rather than separate hardcoded
solvers.

---

## Vehicle Physics Conventions

Unless the current source says otherwise:

Car/body local axes:

    +X = forward
    +Y = lateral
    +Z = up

Use SI units internally.

Examples:

    distance     metres
    mass         kilograms
    force        newtons
    moment       newton-metres
    inertia      kg*m^2
    velocity     m/s
    acceleration m/s^2
    angular rate rad/s

Do not silently change conventions.

---

## Contact Terminology

Phase 9:
- front bogie aggregate reaction
- rear bogie aggregate reaction

Phase 10:
- Running contact
- Guide contact
- Upstop contact

Do not confuse:
- a negative local-up bogie reaction
with
- an upstop wheel load

Actual unilateral engagement begins in Phase 11.

Individual physical wheel-load recovery remains later work.

---

## Major Deferred Systems

Physics:
- suspension/compliance
- detailed individual-wheel load sharing
- contact gaps/preload
- friction/slip
- steering
- derailment
- refined rolling/bearing mechanics

Operations:
- brakes
- launches
- lifts
- stations
- block systems
- switches
- transfer tracks
- storage/maintenance tracks

Vehicle authoring:
- seats
- restraints
- rider occupancy
- detailed vehicle editor

Rendering:
- expanded track geometry families
- mature shader/material system
- per-train material instances
- optional per-car appearance overrides

Simulation UI:
- visible train playback
- Play/Pause/Reset
- physics telemetry
- force/contact debug overlays

---

## Agent Session Startup

For substantial work:

1. Read `AGENTS.md`.
2. Read this file.
3. Read the relevant sections of `docs/architecture.md`.
4. Run `git status`.
5. Inspect the implementation relevant to the requested task.
6. Treat source and tests as authoritative over stale documentation.
7. Audit before changing architecture or physics.
8. Implement only the requested milestone.
9. Build affected targets.
10. Run focused regressions.
11. Run full CTest when practical.
12. Run `git diff --check`.
13. Report exactly what changed and what remains deferred.

Unless explicitly requested:

    DO NOT COMMIT
    DO NOT PUSH
