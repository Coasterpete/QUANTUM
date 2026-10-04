# Rigid Body M2: linear constraints and carriage proof

M2 adds prismatic/slider constraints to Engine's isolated `RigidBodyWorld`.
Jolt remains the general mechanical domain; Core's specialized train, bogie,
contact, connector, inertia and device solvers remain authoritative and unchanged.
No rendering, authored mechanisms, serialization or new dependency is added.

## Creating, driving and observing a slider

```cpp
quantum::physics::RigidBodyWorld world;
quantum::physics::RigidBodyBoxSettings box;
box.positionMeters = {0.0, 0.0, 2.0};
box.massKilograms = 10.0;
const auto carriage = world.createBox(box);

quantum::physics::RigidBodySliderSettings guide;
guide.body = carriage;
guide.anchorPositionMeters = box.positionMeters;
guide.axis = {0.0, 0.0, 1.0};
guide.minimumTranslationMeters = -0.25;
guide.maximumTranslationMeters = 4.75;
const auto slider = world.createSlider(guide);
world.setSliderMotor(slider, {true, 1.5, 200.0}); // m/s, N
world.stepFixed();
const auto state = world.sliderState(slider);
// state.translationMeters and state.linearVelocityMetersPerSecond
world.removeConstraint(slider);
world.removeBody(carriage);
```

A slider blocks two relative translations and all three relative rotations,
leaving translation along its normalized axis. The shared anchor is world space
at creation: Jolt converts it to each body's local attachment point, defining
the creation pose as **zero translation**, even when the anchor is offset from
the box center. Limits describe displacement from that pose, not absolute
world height. In the example the box center travels from Z=1.75 to Z=6.75 m.

An absent `connectedBody` attaches to Jolt's static world. An explicit connection
requires two live, distinct bodies in this world and at least one dynamic body.
The axis is world space at creation and then follows `connectedBody`'s rotation;
the initial relative orientation is preserved. Prefer the heavier body as
`connectedBody`, following Jolt's rotation-solver guidance. Connected colliders
still collide normally; arrange geometry to permit the intended motion.

The optional minimum/maximum default independently to unbounded travel. Jolt's
hard limits require `minimum <= 0 <= maximum`; M2 rejects reversed, zero-width
and underflowed ranges. All supplied positions, directions, bounds and motor
values must be finite and fit single precision. A nonzero axis is normalized
using a double-precision `hypot` before conversion. These are normal Jolt
constraint limits, with no custom stops, springs or friction tuning.

The motor uses Jolt's velocity mode and symmetric force limits. Positive speed
moves `body` along the axis relative to `connectedBody`/world. Zero force applies
no drive. Disabling the motor ignores its target and permits motion under
gravity and existing damping. An enabled zero target brakes/holds only within
the force budget; it is neither a position target nor a positional lock.
Drive changes wake connected dynamic bodies. No spring/PID tuning is needed.

`sliderState` recomputes translation from current attachment transforms.
Axial velocity projects the relative velocities of both bodies evaluated at
body's attachment point onto the current reference axis. This includes angular
motion of the attachment and rotating reference frame; subtracting center-of-
mass speeds alone would be incorrect for a rotating two-body assembly.

Units remain meters, seconds, kilograms, Newtons and m/s; angular body state is
rad/s. Coordinates remain right handed, X/Y horizontal, Z up, with gravity
`(0,0,-9.80665)` m/s². GLM doubles at the API do not imply double-precision Jolt.
The pinned dependency remains **Jolt 5.6.0#1**. Checked upstream and matching
local vcpkg source on 2026-10-04:
[slider API](https://github.com/jrouwe/JoltPhysics/blob/v5.6.0/Jolt/Physics/Constraints/SliderConstraint.h),
[slider implementation](https://github.com/jrouwe/JoltPhysics/blob/v5.6.0/Jolt/Physics/Constraints/SliderConstraint.cpp),
[motor settings](https://github.com/jrouwe/JoltPhysics/blob/v5.6.0/Jolt/Physics/Constraints/MotorSettings.h).

## Handles, lifetime and fixed ticks

The public `RigidBodyConstraintHandle` is unchanged. A single private vector
now stores Jolt's existing `TwoBodyConstraint` reference plus two body IDs for
both hinges and sliders. This shares removal/teardown without new classes or
a constraint registry. Specific APIs validate subtype before casting; wrong-
type, invalid, foreign or removed handles throw `std::invalid_argument`.

Removal wakes bodies, unregisters the constraint, releases its retained
reference and leaves a tombstone. Body removal first removes **all** attached
hinges and sliders on either side; unrelated constraints survive. Destruction
releases every constraint before destroying bodies. Creation always appends:
stale handles cannot alias replacements in the same world. Handles own nothing
and must not outlive their world. Capacity remains 1024 total body creations
and 128 total constraint creations (hinges and sliders combined) per world
lifetime; removal does not reclaim slots. All calls belong on the owning thread
outside stepping. Jolt types stay private behind `unique_ptr<Impl>`.

Before and after M2: accepted preview tick -> train state/pose commit ->
`RigidBodyWorld::stepFixed()` -> `Update(float(1/240), 1, ...)`. The same update
now solves slider constraints. No clock, accumulator, thread, solver tolerance,
train ordering or preview ownership changed. General physics still advances
only with accepted preview playback ticks; the application's world stays empty.

## Mechanical proofs

`QuantumEngine.RigidBodySliders` covers:

- A 10 kg rotated box with a centered anchor and vertical guide, travel
  [-0.25,4.75] m, ±1.5 m/s drive and 200 N force. Four 24 s cycles push
  against each stop, followed by 2 s upward travel, 2 s zero-speed holding
  and 3 s passive fall: **24,720 fixed ticks / 103 s per trajectory**.
  Two complete runs compare body and slider state at every tick within 1e-6.
- A 2 kg horizontal X actuator: zero force, sleeping/waking, a 2 N first-tick
  acceleration check against F/m, disabled coasting, signed 1 m/s motion and
  ±2 m limits. A separate unbounded 10 kg vertical mass with 50 N drive proves
  insufficient force cannot hold against gravity.
- Two dynamic boxes (4 kg reference, 1 kg child), separated along X to avoid
  collision, with an X slider and ±1 m limits. Over 4 s both bodies fall while
  the motor drives ±1 m/s relative motion with collinear attachments.
- Invalid settings, independent optional bounds, safely normalizable axis
  magnitudes, foreign/stale/wrong-type handles, mixed hinge/slider body removal,
  sleeping support removal, explicit release and populated teardown.

The carriage checks every tick against 0.02 m off-axis/limit tolerance,
1e-4 rad angular drift, 0.01 m/s off-axis speed and 0.01 rad/s angular speed.
Motor error must be below 0.05 m/s after 1 s warmup while away from stops;
4,336 samples per trajectory meet that condition. Holding checks drift and
speed below 0.01 m and 0.01 m/s after 0.5 s braking. These are iterative-solver
tolerances, not exact geometric guarantees. X and two-body guides use the same
0.02 m / 1e-4 rad bounds, with speed checks after 1 s / 0.5 s respectively.

Windows x64 MSVC Debug measurements (meters, seconds, radians as applicable):

| Proof | Measured worst case / response |
| --- | --- |
| Carriage, each 103 s run | Off-axis position 0 m; angular drift 0 rad; off-axis speed 0 m/s; angular speed 0 rad/s; lower overshoot **0.01715540886 m**; upper overshoot **0.003154277802 m**; post-warmup target error 0 m/s |
| Carriage zero-speed holding | Post-braking position drift 0 m; speed 0 m/s |
| Carriage disabled drive | Speed at 0.5 s **-4.842039585 m/s**, then settles at lower stop |
| Same-configuration repeatability, 24,720 samples | Maximum position, linear velocity, translation and axial velocity differences all 0 at printed precision; orientation, angular velocity and activation match within asserted tolerances |
| Horizontal actuator | Off-axis/angular drift 0; maximum limit overshoot **0.001618146896 m**; target error 0 m/s; 2 N first-tick speed **0.004166666884 m/s**; driven speed 1 m/s; disabled speed after 0.5 s **0.9753089547 m/s** |
| Insufficient vertical drive | 10 kg mass with 50 N zero-speed motor: **-2.372770309 m/s** after 0.5 s |
| Two-body guide, 4 s | Off-axis/angular drift 0; target error 0 m/s; maximum limit overshoot **0.001498699188 m** |

Zeros above are observations for these simple configurations and the tested
executable, not universal exactness or cross-platform determinism claims.

## Small reading path and limits

1. `RigidBodyWorld.hpp`: slider settings/state and unchanged handle contract.
2. `createSlider`: validation, zero reference, normalized axis and publication.
3. `setSliderMotor`: velocity mode, force units and waking.
4. `sliderState`: current translation and moving-frame point velocity.
5. `Impl::requireConstraint`, `requireHinge`, `requireSlider`: safe type checks.
6. `removeBody`, `removeConstraint`, `Impl::~Impl`: dependency/lifetime order.
7. `carriageTrajectory` / `carriageAndRepeatability` in `RigidBodySliderTests.cpp`.
8. `horizontalActuatorForceDisableAndWake`, `twoBodySlider`, `validationAndLifetime`.

Useful C++ concepts: optional values, non-owning value handles, reference counting,
base/derived pointers with checked casts, reserved vector storage and RAII.
Jolt's internal Jacobians and solver scheduling can wait. This milestone adds
creation-time limits and velocity drive only; no adjustable limits, position
motor, mechanism authoring or train/general-body coupling is supplied.

The passing proofs use centered/collinear load paths. During proof development,
an eccentric carriage anchor produced 0.001637 rad angular drift at its first
stop impact, exceeding the intended 1e-4 rad proof criterion; an eccentric
two-body stop likewise exceeded it. No solver settings were changed to mask
that behavior. Eccentric impact stability and rotating-reference accuracy are
not established by the final proofs. Jolt still supports offset attachments;
these results do not certify arbitrary machinery or impact loads.

## Verification on 2026-10-04

- `cmake --preset windows-msvc-debug`: passed, using local `VCPKG_ROOT` and
  the existing static triplet/dependency baseline.
- `cmake --build --preset windows-msvc-debug --parallel 2`: full Windows x64
  MSVC Debug build passed (Core, Engine, editor, shaders and test executables),
  141.38 s for the initial full build; final incremental full build 10.59 s.
  Existing `[[nodiscard]]` warnings remain in untouched tests; the new world
  and slider test source produced no warnings in the focused build.
- Final standalone slider executable: passed all four scenarios, including
  two complete carriage trajectories.
- `ctest --test-dir build -C Debug --output-on-failure --parallel 2`:
  **97 enabled tests passed, zero failures**, 806.60 s; **98 registered tests**.
  The one previously disabled test, `QuantumEngine.GpuTrainPoseResidency`,
  remains disabled/unverified. M0 world, M1 hinge/motor, M2 slider, preview,
  train/multi-car, dynamic-contact, devices, document/history and all enabled
  GPU/renderer regressions passed. Dynamic-contact took 708.07 s; circuit-
  completion shadow took 419.11 s. M2 sliders took 4.10 s, M1 constraints
  28.85 s, M0 world 0.92 s and preview 12.43 s (concurrent host contention).
  Full-run proof output matches the measured table above.
- `git diff --check`: passed; new files also checked for trailing whitespace.

Logs are `rigid-body-m2-configure.log`, `rigid-body-m2-focused-build.log`,
`rigid-body-m2-build.log`, `rigid-body-m2-final-build.log`,
`rigid-body-m2-slider-build.log`, `rigid-body-m2-slider-proof.log`,
`rigid-body-m2-all-tests.log` and `build/Testing/Temporary/LastTest.log`
(ignored generated files). The earlier `rigid-body-m2-mechanical-tests.log`
records the initial eccentric-proof failure, not the final passing suite.

Verification is automated physics state. No separate application smoke or
manual visual rigid-body validation was performed. Release, Linux, sanitizers,
leak instrumentation, different/older CPUs, huge/contact-heavy mechanisms,
high-speed continuous collision, visual correctness and cross-platform bit
determinism remain unverified. Rendering and independent mechanical preview
ownership remain outside M2.
