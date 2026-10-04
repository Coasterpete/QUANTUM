# Rigid Body M1: constraints and mechanical proof

## Scope and boundary

M1 adds unlimited hinge/revolute constraints, torque-limited angular velocity
motors and individual removal to the existing Engine `RigidBodyWorld`.
Public settings/state use QUANTUM handles and GLM; Jolt remains private to
`RigidBodyWorld.cpp`. `QuantumCore` remains independent of Jolt. Trains,
bogies, rail contact, connectors, inertia and device forces continue through
the specialized coaster solver without changes or coupling.

The dependency remains Jolt 5.6.0#1 at the existing vcpkg baseline. Implementation
was checked against installed headers and the matching vcpkg source, especially
`HingeConstraint`, `MotorSettings`, `BodyInterface::CreateConstraint`,
`ActivateConstraint` and `ConstraintManager` reference ownership.

## Creating and driving a mechanism

```cpp
quantum::physics::RigidBodyWorld world;
quantum::physics::RigidBodyBoxSettings box;
box.halfExtentsMeters = {2.0, 0.1, 0.1};
box.positionMeters = {2.0, 0.0, 6.0};
box.massKilograms = 4.0;
const auto arm = world.createBox(box);

quantum::physics::RigidBodyHingeSettings joint;
joint.body = arm;
joint.anchorPositionMeters = {0.0, 0.0, 6.0};
joint.axis = {0.0, 1.0, 0.0};
// Leave connectedBody absent to attach to world, or supply another body handle.
const auto hinge = world.createHinge(joint);
world.setHingeMotor(hinge, {true, 1.5, 200.0}); // rad/s, N m
world.stepFixed(); // exactly one existing 1/240 s tick
const auto state = world.bodyState(arm);

world.removeConstraint(hinge); // arm keeps its current motion and becomes free
world.removeBody(arm);
```

The anchor and axis are world-space values at creation, in right-handed
X/Y-horizontal, Z-up coordinates. Jolt records body-local attachment points
and axes from those initial transforms. The hinge removes three relative
translations and two relative rotations, leaving rotation around the hinge
axis. A private perpendicular reference defines the initial zero angle; no
angle limits or public reference-frame settings are needed for this milestone.
See the [Jolt 5.6 hinge API](https://github.com/jrouwe/JoltPhysics/blob/v5.6.0/Jolt/Physics/Constraints/HingeConstraint.h).

`body` and a supplied `connectedBody` must belong to this world, be live and
distinct, with at least one dynamic body. An absent `connectedBody` selects
Jolt's fixed world body; an explicitly supplied invalid handle is rejected.
Anchor/axis inputs must be finite and fit single precision, and the axis must
have a finite nonzero length. The direction is normalized internally.
Invalid handles/settings throw `std::invalid_argument` before creating state.

`setHingeMotor` selects Jolt's normal velocity mode or Off, sets the target
angular velocity and symmetric maximum torque, then wakes connected bodies.
Positive velocity rotates `body` relative to `connectedBody` by the right-hand
rule about the hinge axis; if connected to world, it is absolute rotation.
Targets must be finite and fit single precision. Torque must be finite,
representable and nonnegative; zero torque applies no drive. A zero target
with nonzero torque actively brakes. Disabling drive preserves motion subject
to gravity and the existing damping; it does not lock or stop the body.
Velocity mode needs no spring/PID tuning. See
[Jolt motor settings](https://github.com/jrouwe/JoltPhysics/blob/v5.6.0/Jolt/Physics/Constraints/MotorSettings.h).

`bodyState` now also returns world-space angular velocity in radians/second.
M0 box mass/inertia, gravity, damping, collision filters and sleeping remain
unchanged. Connected bodies still collide normally: a hinge does not silently
disable their collision pair. Mechanism geometry must allow the intended motion.

## Ownership, removal and handles

`Impl` stores a `JPH::Ref<HingeConstraint>` and both body IDs for each hinge.
Jolt's system retains its own reference while the constraint is registered;
the hinge borrows its bodies. Creation holds a local reference before adding
the hinge to the system, so an allocation failure during registration releases
it. Reserved entry storage prevents a later vector allocation at publication.

`removeConstraint` validates the handle, wakes its bodies, removes the system's
reference, then clears the retained reference. The hinge is destroyed before
the next step and cannot affect that step. Velocities/transforms are preserved.
Waking matters when removing the support of a sleeping pendulum.

`removeBody` validates the handle and removes **all** hinges attached on either
side before removing and destroying the Jolt body. Surviving connected bodies
are awakened. Unrelated bodies and hinges remain live. World destruction releases
every remaining hinge before removing/destroying remaining bodies, then lets
the existing system/resources destruct. Process registration remains alive
for other worlds, as in M0.

Body and constraint handles carry a world pointer and slot index; they own
nothing. Validation checks owner, bounds and whether the entry is live.
Removal leaves a tombstone, and creation always appends. Indices are never
reused, so a stale handle cannot alias a replacement and no generation counter
is needed. Double removal, queries of removed bodies and motor operations on
removed hinges throw. Handles must not be used after their world is destroyed,
including against a later world allocated at the same address.

This simple policy bounds **total successful creations per world lifetime**:
1024 bodies and 128 hinges. Removal releases Jolt resources but does not reclaim
handle slots; exceeding either limit throws `std::runtime_error`. It is intended
for this small mechanical milestone, not indefinite scene editing. All operations
run on the owning simulation thread outside an update.

The useful C++ concepts are `std::optional` (world versus explicit body),
non-owning value handles, reference-counted lifetime, `unique_ptr<Impl>` hiding
library types, short-circuit validation and destruction order. Jolt's internal
solver mathematics and worker scheduling can wait.

## Fixed-step path

Before and after M1: accepted `SimulationPreview` playback tick -> commit train
state/pose -> `RigidBodyWorld::stepFixed()` -> Jolt `Update(float(1/240), 1, ...)`.
That existing update also solves hinges and motors. There is no new accumulator,
clock, variable step, preview ownership change or reset/persistence coupling.
Pause, unavailable preview, host interruption and boundary stop retain M0
cadence behavior. The application's world remains empty; proof objects exist
only in automated tests.

## Mechanical proofs and tolerances

`QuantumEngine.RigidBodyConstraints` contains five focused scenarios:

- A 4 m, 4 kg box arm, hinged at one end around world Y under gravity, runs
  30 s at +1.5 rad/s with a 200 N m limit, then 5 s at -1.5 rad/s. Every tick
  checks finite state, pivot drift, axis alignment and unwanted angular motion.
  Orientation accumulation proves repeated forward turns; velocity verifies
  reversal. Removing the hinge releases the arm under gravity.
- A centered world-Z hinge has no gravitational torque about its axis. Zero
  torque cannot start it; enabling drive wakes it; disabling ignores a new
  reverse target while default damping slows its motion; enabling zero target
  brakes it.
- The unpowered end-hinged arm swings under gravity and settles/sleeps over
  240 s with unchanged default damping. Removing its hinge wakes it to fall.
- Two dynamic boxes separated along Z retain a shared moving pivot while
  falling; their motor reaches 1 rad/s relative rotation. Removing the parent
  removes the hinge and leaves a usable child.
- Validation/lifetime covers invalid/foreign/stale handles, self/static-only
  hinges, invalid anchor/axis/motor inputs, both removal paths, multiple
  attached hinges, unrelated constraint survival, sleeping support removal,
  replacement handles, populated teardown and simultaneous worlds.

Pivot tolerance is 0.02 m; normalized axis error is below 1e-4; off-axis
angular speed is below 0.01 rad/s; target error is below 0.05 rad/s after a
1 s startup/reversal allowance. Quaternion length is within 1e-5 of unity.
These allow iterative single-precision constraint error without allowing
translational escape, axis drift or the wrong drive direction. Passive peak
speed in the last 5 s must be below 0.05 rad/s and 5% of its first-5-s peak;
final center height must be within 0.02 m of the hanging position and asleep.
Settling uses 240 s because M0's damping had not met that criterion at 120 s.

## Limits and reading path

M1 supplies unlimited hinges and velocity motors only. No hinge limits,
position drive, operation profiles, additional shapes, general constraint
framework, renderer/UI, persistence, ride system or train/Jolt interaction is
added. Extreme scales/speeds, capacity exhaustion, contact-heavy mechanisms,
long production sessions and cross-platform determinism are not proven.

Read these small sections in order:

1. `RigidBodyWorld.hpp`: handles, hinge/motor settings and removal contract.
2. `RigidBodyWorld::createHinge`: validation, world/local frame setup, publication.
3. `setHingeMotor` and `bodyState`: motor units, direction and observable response.
4. `Impl::requireBody` / `requireHinge`: owner, bounds and tombstone checks.
5. `removeBody`, `removeConstraint`, `Impl::removeHinge` and `Impl::~Impl`:
   lifetime/dependency order and waking supported bodies.
6. `motorizedRotatingArm`, `motorDisableTorqueLimitAndWake` and
   `passivePendulumAndSleepingRelease` in `RigidBodyConstraintTests.cpp`.
7. `twoBodyHinge`, `validationAndLifetime`, existing M0 tests and
   `SimulationPreview::update`: lifetime proof and preserved fixed-step boundary.

## Verification on 2026-10-04

- `cmake --preset windows-msvc-debug`: passed with local `VCPKG_ROOT` set.
- `cmake --build --preset windows-msvc-debug --parallel 2`: full Windows x64
  MSVC Debug build passed in 125.95 s, including Core, Engine, editor, shaders
  and all test executables. Existing `[[nodiscard]]` warnings remain in
  untouched tests; the new source produced no warnings in its focused build.
- `ctest --test-dir build -C Debug --output-on-failure --parallel 2`:
  **96 enabled tests passed, zero failures**, 696.17 s; 97 registered tests.
  `QuantumEngine.GpuTrainPoseResidency` remains disabled as at the M0 base.
  Dynamic contact (602.17 s), train/multi-car, device forces, preview,
  document/history and enabled GPU validation regressions passed.
- In that full run: M1 constraints 5.99 s, M0 world 28.44 s, preview 11.04 s.
  The earlier focused rigid-body run passed 2/2 tests in 6.19 s; standalone
  preview passed 1/1 in 11.98 s. Concurrent suite timings include host contention.
- `git diff --check`: passed, with separate whitespace checks for the two
  new files. Changes remain scoped to the world API/implementation, constraint
  tests and their CMake registration, M0/M1 documentation and one architecture
  cross-reference.

Full-run mechanical measurements (all lengths in meters, speeds in rad/s):

| Proof | Observed result |
| --- | --- |
| Driven end-hinged arm, 35 s | Maximum pivot drift 0.0000707092; maximum axis error 2.89787e-16; maximum post-warmup speed error 3.57628e-6; forward rotation 44.9168 rad |
| Motor control | Driven speed 1.5; disabled speed after 5 s 1.16819; actively braked speed 0 |
| Passive arm, 240 s | Maximum pivot drift 0.000215438; early peak speed 2.64301; last-5-s peak 0; final center Z 4.00317; asleep |
| Two dynamic bodies, 2 s | Maximum attachment separation 0.00000190735; relative target 1 rad/s satisfied within 0.05 rad/s |
| Preserved M0 falling box, 10 s | Final center Z 0.499999; speed 0; asleep in both repeatability runs |

Logs: `rigid-body-m1-configure.log`, `rigid-body-m1-build.log`,
`rigid-body-m1-mechanical-tests.log`, `rigid-body-m1-preview-tests.log`,
`rigid-body-m1-all-tests.log` and `build/Testing/Temporary/LastTest.log`
(ignored generated files). Verification is automated physics state; no manual
visual mechanism validation or separate application smoke was performed.
Release, Linux, sanitizers/leak instrumentation, older CPUs, multi-DPI and
large/contact-heavy mechanisms remain unverified. No cross-platform
determinism claim is made.
