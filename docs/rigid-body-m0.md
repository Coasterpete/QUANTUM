# Rigid Body M0: additive physics world

## Purpose and library choice

`RigidBodyWorld` adds general rigid-body physics beside the existing
track-constrained coaster dynamics. Trains, bogies, rail contact, connectors,
launches, brakes, inertia and solver tolerances remain authoritative in Core.
There is no coupling between the two domains in M0.

[Rigid Body M1](rigid-body-m1.md) extends this same world with hinges,
velocity motors and individual removal; the M0 stepping and coaster boundary
described here remain unchanged.

Library research was checked on 2026-10-03 against upstream documentation and
the repository's existing vcpkg baseline:

| QUANTUM concern | Jolt | Bullet |
| --- | --- | --- |
| Maintenance | Current release 5.6.0; maintained upstream with recent releases | Maintained upstream; current vcpkg package remains 3.25#3 |
| C++ / Windows | C++17 library, MSVC and Windows support; usable from C++23 | Established C++ API, Windows support; usable from C++23 |
| CMake / static linking | Imported `Jolt::Jolt`; port follows triplet library/CRT linkage | CMake port provides Bullet dynamics, collision and math libraries; static supported |
| Fixed ticks | Explicit `PhysicsSystem::Update(dt, collisionSteps, ...)`; supplied single-thread job runner | `stepSimulation(dt, 0)` accepts caller-owned fixed ticks without its accumulator |
| Collision / machinery | Rigid bodies, collision shapes, joints/constraints and motors | Rigid bodies, collision shapes, joints/constraints and robotics features |
| API / ownership | One system per world plus allocator, jobs and collision filters; process registration required | World, dispatcher, broadphase, solver, shapes and bodies; no Jolt-style registration |
| Performance | Designed for multicore operation; M0 uses one thread | Sequential and multithread options; M0 has no comparative benchmark |
| Isolation | Library types can stay in one implementation file | Library types can likewise stay in one implementation file |
| License | MIT; commercial distribution allowed with required copyright/license notice | zlib; commercial distribution allowed subject to its notice and source conditions |

Jolt is selected for its current maintenance, direct update API, supplied
single-thread job runner and mechanical constraint support. Bullet is viable
and has simpler process initialization, but offers no concrete integration
advantage for this milestone. This is an integration choice, not a claim that
Jolt is numerically superior or faster in QUANTUM.

Sources: [Jolt upstream](https://github.com/jrouwe/JoltPhysics),
[Jolt 5.6 architecture](https://github.com/jrouwe/JoltPhysics/blob/v5.6.0/Docs/Architecture.md),
[Jolt single-thread runner](https://github.com/jrouwe/JoltPhysics/blob/v5.6.0/Jolt/Core/JobSystemSingleThreaded.h),
[Bullet upstream](https://github.com/bulletphysics/bullet3),
[Bullet stepping implementation](https://github.com/bulletphysics/bullet3/blob/master/src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp),
[Jolt package](https://vcpkg.io/en/package/joltphysics.html),
[Bullet package](https://vcpkg.io/en/package/bullet3.html),
[MIT license](https://github.com/jrouwe/JoltPhysics/blob/v5.6.0/LICENSE),
[zlib license](https://github.com/bulletphysics/bullet3/blob/master/LICENSE.txt).
Preserve the Jolt copyright/license notice in a paid binary distribution;
vcpkg installs it under `share/joltphysics/copyright`.

## Dependency boundary

`vcpkg.json` adds only `joltphysics`. The unchanged builtin baseline
`0ac8df3b98e3afcd8bf075fa74a6bd2c32613345` selects **5.6.0#1**.
The root CMake file finds Jolt only for engine builds, and `QuantumEngine`
links `Jolt::Jolt` privately. Core-only builds retain their existing GLM/JSON
dependencies and do not require Jolt through CMake. There is no vendored tree,
new target layer, or change to the existing build presets.

The Windows preset uses `x64-windows-static` and the static MSVC CRT. The port
disables Jolt GPU backends, extra samples/tests, profiler and debug renderer.
Its default build uses single precision, SSE4/AVX/AVX2/FMA and has
`CROSS_PLATFORM_DETERMINISTIC=OFF`. Deployment CPU requirements follow that
package configuration; M0 does not broaden the project's supported hardware.

## Ownership and cleanup

M0 constructed one empty `RigidBodyWorld` immediately before `SimulationPreview`.
[M3](rigid-body-m3.md) now creates that world only inside the deliberately enabled
Editor proof owner, declared before the preview. Reverse C++ scope destruction
destroys the preview first, then the owner/world. `SimulationPreview::setRigidBodyWorld()`
stores an optional non-owning pointer. Standalone preview tests need no world.

World construction initializes Jolt and creates its implementation through
`std::unique_ptr<Impl>`. That implementation owns collision filter tables, a
10 MiB temporary allocator, a single-thread job runner, `PhysicsSystem` and
the list of created body IDs (plus constraint references since M1). The system
borrows its filter tables; declaration order keeps them alive until the system is
destroyed. Bodies own reference-
counted shapes through Jolt. Destruction first removes and releases all
constraints (hinges and M2 sliders), which borrow their connected bodies. It then
removes each body from the system and destroys it, before releasing the system
and its support objects.

Jolt requires process-wide allocator/type/collision registration and a factory.
A private function-local `JoltRuntime` initializes these once, allowing several
worlds to coexist. Registration lasts until process exit, after the locally
owned worlds have died. No world destructor unregisters types needed by another
world. A foreign preexisting Jolt factory is rejected rather than overwritten.
This integration owns Jolt registration; other code must not independently
initialize or unregister it. Keep worlds locally owned, not global objects.

`RigidBodyWorld` cannot be copied or moved. Handles contain their owning world's
address and an index; querying a foreign/invalid handle throws. Handles are
valid only until body removal or world destruction. M0 initially had no per-body
removal; M1 adds it and removes all attached constraints first. Neither milestone
reuses handle indices. Removed slots remain invalid until world destruction.
All creation, queries, stepping and destruction belong on the owning thread.

## Existing fixed-step path and insertion

The application calls `SimulationPreview::update(frameDeltaSeconds,
timingDiscontinuity)` on its main/event/render thread. The preview owns the
accumulator and uses Core's `defaultFixedTimeStepSeconds = 1.0 / 240.0`.
It caps catch-up at 60 ticks per frame, discards known host interruptions,
and only steps while playing with an available train preview.

Before M0, each accepted tick evaluated device forces, called `stepTrain()`,
committed train state/pose and consumed one accumulator interval. M0 adds
`rigidBodyWorld_->stepFixed()` after that commit, before testing the existing
boundary-stop condition. Thus the final train boundary tick also advances the
general world. Train inputs, output, numerical ordering and presentation
interpolation are unchanged. Existing physics timing covers the adjacent world
in its total elapsed span; per-step train timing still measures the train work.

`stepFixed()` calls `PhysicsSystem::Update(float(1.0 / 240.0), 1, ...)`: exactly
one collision step per host tick. There is no second accumulator, no worker
thread and no render delta passed to Jolt. An update error is reported as an
exception with Jolt's error bits; an attached preview uses its existing stop/error
path. An errored update is not rolled back; discard the affected world rather
than assuming recovery from exhausted contact buffers.

Preview pause, unavailability, boundary stop and timing discontinuities also
stop general-world progress. Train reset/rebuild preserves the independently
owned world's bodies and tick counter; playback resumes them from their current
state. Document edits cannot reset, rescale or serialize these bodies. This is
an M0 cadence integration, not independent scenery playback. M0 proof objects
were created only in tests; M3 adds an opt-in visible mechanism using this cadence.

## Coordinates and units

QUANTUM uses right-handed world coordinates: X/Y horizontal, Z up. Core's
coaster gravity vector is `(0, 0, -g)` and its local frame satisfies
`tangent x lateral = up`. The world uses that same axis ordering and the existing
standard gravity **9.80665 m/s^2**. Box positions and half extents are meters;
mass is kilograms; returned linear velocity is meters per second; M1's angular
velocity is radians per second and motor torque is newton meters; time is seconds.
No document scale is applied to the general world.

Public inputs/outputs use `glm::dvec3` and `glm::dquat`, matching existing
physical boundaries. Conversion to Jolt's single-precision types happens only
in `RigidBodyWorld.cpp`; the public double type does not imply double-precision
simulation. GLM quaternion construction is `(w,x,y,z)`, Jolt is `(x,y,z,w)`;
conversion names each component and normalizes the input. There is no axis
swap, handedness change or document quaternion-format change. Finite/range,
positive extents, mass and quaternion checks precede library body creation.

## Proof and verification

`RigidBodyWorldTests.cpp` creates a static box floor with half extents
`(10,10,0.5)` centered at `(0,0,-0.5)`, placing its top at Z=0. A dynamic
1 kg box with half extents `(0.5,0.5,0.5)` starts at Z=5. Both use Jolt's default
friction (0.2), restitution (0), damping (0.05) and sleeping policy. A zero
convex radius makes collider dimensions exact. Collision filtering enables
static/dynamic and dynamic/dynamic pairs; static/static is disabled.

The test runs 2400 ticks (10 s), checks floor support throughout with 2 cm
contact tolerance, and requires final center height near 0.5 m, speed below
0.01 m/s and sleeping. It compares two complete trajectories within 1e-6 for
positions, velocities and quaternion dot product, plus matching activation.
Other checks cover empty/populated cleanup, concurrent worlds, negative-Z
gravity, static transforms, quaternion conversion and invalid input.

`SimulationPreviewTests.cpp` compares an attached-world preview with a
train-only control using the existing exact state/pose helpers. It covers
fractional frames, bounded catch-up, pause, timing discontinuity, invalid delta,
reset/rebuild isolation, detachment, a boundary-stop tick and 30/144 Hz frame
cadences producing identical body states after 240 accepted ticks.

This requires repeatability on the same tested executable/configuration.
There is **no claim of bit-identical cross-platform determinism**. Fixed tick
duration alone is insufficient to make that claim, and this port does not
enable Jolt's optional cross-platform configuration.

The proof is physics-state verification. No viewport body representation,
authoring controls, or manual visual/live rigid-body proof is added.

## M0 limits and reading path

M0 introduced static/dynamic boxes, initial transforms, state queries and stepping.
M1 adds hinge constraints, velocity motors and removal. It retains bounded
capacities of 1024 body creations per world lifetime, 1024 body pairs and
1024 contacts, default discrete collision, and a 10 MiB scratch arena. Extreme scales, high-speed
tunnelling, capacity exhaustion, large scenes and deployment on older CPUs
are not covered by the proof. This is not an arbitrary-body serialization,
collider pipeline, ECS, general joints framework, train collision system or
flat-ride editor.
Future domain interaction requires a separately authorized milestone.

Useful reading order:

1. `RigidBodyWorld.hpp`: settings, state, handle and ownership contract.
2. `RigidBodyWorld.cpp`: `initializeJolt()` / `JoltRuntime` for process lifetime.
3. `RigidBodyWorld::Impl` for world resources and cleanup order.
4. `createBox()` / `bodyState()` for shape ownership and GLM conversion.
5. `stepFixed()` for the sole library update call.
6. `SimulationPreview::update()` for the tick insertion; `Application::runImpl()`
   for adjacent ownership and pointer attachment.
7. `RigidBodyWorldTests.cpp` and `adjacentRigidBodyWorldSharesAcceptedTicks()`
   for proof and regression expectations.

RAII, `unique_ptr`, incomplete implementation types, non-owning pointers and
reverse member/scope destruction are the key C++ concepts here. Jolt's internal
constraint algorithms and multithread scheduling can wait.

## Verification on 2026-10-03

- `cmake --preset windows-msvc-debug`: passed with local `VCPKG_ROOT` set.
- `cmake --build --preset windows-msvc-debug --parallel 2`: full Debug build
  passed, including Core, Engine, editor, tests and shaders.
- `ctest --test-dir build -C Debug --output-on-failure --parallel 2`: all
  **95 enabled tests passed**, 744.21 s. Of 96 registered tests,
  `QuantumEngine.GpuTrainPoseResidency` was already disabled at HEAD and remains
  unverified. Dynamic-contact, multi-car/train, device, preview, document/history,
  renderer and enabled GPU validation tests passed.
- Falling-box proof: after 2400 ticks, center Z=0.499999 m, speed=0, asleep in
  both trajectory runs.
- Existing application smoke on `smoke-tests/preview-transition.quantum`,
  `--duration 3 --repeat --simulator`: exit 0, 3.003 s, 300 frames, 721 fixed
  ticks, no preview/physics failure. An OBS Vulkan layer API-version warning
  was present; no rigid body was visually inspected. Reports are in
  `build/rigid-body-m0-smoke.json` and `.txt`.
- Separate Windows Core-only configuration, without a vcpkg toolchain and
  with engine/editor/testing OFF: passed using installed GLM/JSON package
  configs, without finding Jolt. This was a configuration check, not a second
  Core build.
- `git diff --check`: passed.

Release, Linux, sanitizers, older CPUs, large scenes and high-speed collision
remain unverified. Build output includes `[[nodiscard]]` warnings in untouched
tests. Full build and test logs are `rigid-body-build.log` and
`rigid-body-all-tests.log` in the worktree (ignored generated files).
