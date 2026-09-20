# Long-playback performance audit

Date: 2026-09-20

## Conclusion

The severe playback slowdown is reproduced, but it is not caused initially by
the fixed-step accumulator trying to catch up. The first cause is a
geometry-dependent increase in exact train-pose solver work when a four-car
train enters a tightly curved three-dimensional region. Catch-up begins only
after that work approaches and then exceeds the 4.167 ms budget for one 240 Hz
physics step; it subsequently multiplies the already-expensive work per
rendered frame.

The dominant code path is a nested pair of root solves:

1. Every rigid connector refinement candidate solves the following car's
   exact front-hitch position.
2. Solving that car position solves its rigid bogie placement.
3. On curved track, every rigid bogie placement refines the two track stations
   until their world-space chord equals the authored pivot separation.

At the worst measured interval, the nested path performs approximately 302
connector candidate evaluations, 318 rigid-bogie solves, 5,321 bogie
refinement iterations, and 11,916 track samples **per fixed physics step**.
The exhaustive 161-sample connector fallback was never entered.

## Reproduction and controls

The audit used the existing preview-smoke telemetry with diagnostic-only
five-second aggregation of the existing frame, physics, solver-work, and
solver-timing counters. No physics rate, tolerance, iteration limit, or solver
behavior was changed.

The controlled open track contains 300 m of straight track, a 200 m
smootherstep transition to constant pitch, roll, and yaw rates of 0.15, 0.20,
and 0.25 radians per metre, then a long constant-rate region. Initial speed is
20 m/s. This makes one run contain both a straight control and progressively
more demanding curved geometry.

The cleanest reproduction is the MSVC Debug build with GPU preview sampling
disabled. Disabling that sampling removes the validation/readback path and its
warning output without changing the physics result.

| Wall-time interval (s) | FPS | Steps/frame | Fixed-step CPU (ms) | Physics/frame (ms) |
|---:|---:|---:|---:|---:|
| 0.0-5.0 | 76.260 | 3.157 | 0.402 | 1.275 |
| 5.0-10.0 | 77.621 | 3.090 | 0.385 | 1.194 |
| 10.0-15.0 | 75.297 | 3.188 | 0.383 | 1.225 |
| 15.0-20.0 | 74.674 | 3.213 | 0.548 | 1.765 |
| 20.0-25.1 | 14.013 | 17.127 | 3.486 | 59.733 |
| 25.1-30.7 | 3.236 | 38.333 | 8.952 | 343.233 |
| 30.7-36.6 | 1.018 | 60.000 | 16.421 | 985.397 |

This reproduces the reported 80-90 FPS to 10-20 FPS transition (76 to 14 FPS
with the added diagnostic timing). Continuing farther into the test curvature
drives an even more severe collapse. The 60-step cap is reached only after the
fixed-step cost has already risen by more than an order of magnitude.

### Solver work across the same run

All values below are per fixed step.

| Interval (s) | Train poses | Connector candidates | Connector refinements | Rigid-bogie solves | Bogie refinements | Track samples |
|---:|---:|---:|---:|---:|---:|---:|
| 0.0-5.0 | 4.32 | 28.92 | 2.69 | 46.19 | 0.00 | 96.70 |
| 20.0-25.1 | 4.06 | 273.10 | 248.75 | 289.33 | 273.14 | 1,668.52 |
| 25.1-30.7 | 4.02 | 278.91 | 246.98 | 294.98 | 2,363.57 | 5,911.07 |
| 30.7-36.6 | 4.00 | 301.77 | 245.77 | 317.77 | 5,320.70 | 11,916.48 |

In the 30.7-36.6 s interval:

- Four full train poses are solved per physics step.
- A pose has three connectors, so there are twelve connector solves per step.
- Each connector averages 20.48 midpoint refinement iterations and 25.15
  total exact candidates.
- Each rigid-bogie solve averages 16.74 refinement iterations.
- The 317.77 rigid-bogie solves equal the 301.77 candidate geometry solves
  plus 16 final car geometry solves (four cars in each of four poses).
- Aggregate `connectorFallbackUses` is zero. The exhaustive search is not the
  source of the measured growth.

The stage timers are nested rather than additive. For the worst interval, a
16.421 ms fixed step contains 15.475 ms in connector candidate evaluation,
15.104 ms in rigid-bogie solves, and 11.955 ms in track sampling. Track sampling
therefore consumes about 73% of the complete step, reached through the nested
connector/bogie path.

### Controls that rule out time-dependent accumulation

- A 300-second Release run on the existing modern-steel fixture remained
  stable: 100.108 FPS and 0.683 ms/step in the first interval versus 98.743 FPS
  and 0.688 ms/step in the last interval. Solver work per step remained flat.
- A 180-second Release run on a continuous 6,000 m straight track remained
  stable: 99.891 FPS and 0.031 ms/step initially versus 99.117 FPS and
  0.029 ms/step finally. Rigid-bogie refinement remained zero.
- The same straight-to-tight 3D fixture in Release peaked at 1.182 ms per step
  and remained near 97 FPS. This confirms that compiler optimization is enough
  to keep this particular fixture within budget, although the underlying
  redundant solver work is still present (about 3,879 bogie refinements and
  9,101 track samples per step in its most expensive interval).
- Disabling GPU preview sampling leaves the Debug collapse essentially
  unchanged. Event pumping, fence waits, GPU execution, resize handling, and
  warning output are not the root cause.

## Exact code path

1. `SimulationPreview::update` consumes the fixed-step accumulator and calls
   `physics::stepTrain` for every 1/240 s step in
   `editor/src/SimulationPreview.cpp:485-498`. The 60-step safety cap is in
   `editor/include/quantum/editor/SimulationPreview.hpp:105`.
2. `stepTrain` evaluates current kinematics in
   `core/src/physics/TrainPhysics.cpp:4830-4837`.
3. `evaluateTrainKinematicsForValidatedDefinition` solves the center pose at
   `core/src/physics/TrainPhysics.cpp:2700-2704`.
4. `kinematicDerivatives` solves displaced poses at -0.01 m and +0.01 m for the
   central finite difference in `core/src/physics/TrainPhysics.cpp:1522-1535`.
5. `stepTrain` also solves the committed next pose in
   `core/src/physics/TrainPhysics.cpp:4906-4910`. Away from a boundary, that is
   four complete train poses per fixed step: center, before, after, and next.
6. `solveTrainPoseForValidatedDefinition` solves the lead car and then each of
   the three following cars in `core/src/physics/TrainPhysics.cpp:2546-2595`.
7. `solveFollowingCar` establishes the local connector bracket around the
   expected offset in `core/src/physics/TrainPhysics.cpp:856-925`, then refines
   it with midpoint bisection in `core/src/physics/TrainPhysics.cpp:926-990`.
   The local fast path is in lines 993-1105; the unused exhaustive fallback is
   in lines 1107-1164.
8. Every `connectionCandidate` computes the following car's exact front hitch
   through `solveFrontHitchPositionForValidatedDefinition` in
   `core/src/physics/TrainPhysics.cpp:786-829` and
   `core/src/physics/CarPose.cpp:1301-1319`.
9. That front-hitch solve calls `solveCarBodyGeometry`, which calls
   `solveBogieStations` in `core/src/physics/CarPose.cpp:711-754`.
10. `solveBogieStations` samples both bogies for each residual evaluation at
    `core/src/physics/CarPose.cpp:519-561`, brackets an added station separation
    at lines 594-623, and performs up to 64 safeguarded secant/bisection
    iterations at lines 634-675. Its closure tolerance is 1e-10 m at line 20.

## Why curvature causes the explosion

On a straight centerline, the distance between the two nominal bogie stations
equals the authored rigid pivot separation, so `solveBogieStations` returns
after its first residual evaluation at `CarPose.cpp:564-570`.

On a curve with curvature magnitude `k` and pivot separation `L`, a constant
curvature approximation gives the nominal world-space chord as
`2 sin(k L / 2) / k`, which is smaller than `L`. For small `k L`, the initial
residual is approximately `-k^2 L^3 / 24`. The solver must therefore increase
the station separation and refine it to the unchanged 1e-10 m closure
tolerance. Higher and three-dimensional curvature increases the correction and
the number of bogie iterations.

That cost is then multiplied by the connector solver. The connector's selected
grid cell is refined only by midpoint bisection to the 1e-8 m connector
tolerance. In the curved region it needs about 20.5 refinements for each of
twelve connector solves per physics step. Every midpoint evaluation starts a
new exact car-body solve, and every one of those starts a new rigid-bogie root
solve from zero added separation. No result from the adjacent connector
candidate, finite-difference pose, or previous 240 Hz step is used as a hint.

The measured work growth is therefore multiplicative:

`4 train poses x 3 connectors x about 25 connector candidates x about 17 bogie refinements x 2 bogie track samples`.

The exact counts differ because initial/bracketing/final samples are included,
but they predict the observed order of magnitude.

## Catch-up is an amplifier, not the initiating cause

One 240 Hz step has a 4.167 ms real-time budget. The fixed-step cost rises from
0.4 ms to 3.5 ms while the train enters the transition, with solver counters
rising at the same time. Only then does accumulated wall time request 17 steps
per frame. Once a step costs 9-16 ms, the simulation cannot execute 240 such
steps per wall-clock second. Each slow frame adds more accumulator debt and
requests more expensive steps on the next frame, eventually reaching the
60-step cap.

Reducing the rate, lowering the iteration limits, loosening either tolerance,
or discarding additional time would conceal this feedback but would not fix the
measured cause.

## Smallest recommended fix

Replace the midpoint-only update inside `solveFollowingCar::refineBracket`
(`TrainPhysics.cpp:926-965`) with a safeguarded secant step, falling back to the
midpoint when the secant is non-finite or too close to a bracket edge. The
existing rigid-bogie solver already uses this exact pattern at
`CarPose.cpp:642-655`.

This is the smallest high-leverage change because it attacks the outer
refinement that repeatedly invokes the expensive inner solver. It does not
change the 240 Hz rate, the selected local root bracket, the 1e-8 m connector
closure tolerance, the 1e-10 m bogie closure tolerance, or the exhaustive
fallback. For the locally smooth connector residual, interpolation should
remove most of the approximately 20 midpoint candidates per connector while
retaining bisection's convergence guarantee.

Before accepting that change, verify on straight, transition, tight 2D, tight
3D, open-boundary, and closed-circuit cases that:

- final connector and bogie residuals remain within their existing tolerances;
- the same local root is selected;
- pose/kinematic results remain within the solver's existing numerical
  tolerance;
- connector candidate, rigid-bogie refinement, and track-sample counts fall;
- Debug fixed-step time stays below the 4.167 ms budget on the reproducer.

If safeguarded interpolation is insufficient, the next accuracy-preserving
step should be a continuation hint for the connector offset and bogie station
adjustment, validated against the same residuals with the current search as the
fallback. That is a broader API/state change and should not be the first patch.

For day-to-day playback, a RelWithDebInfo editor build is also an immediate
accuracy-neutral mitigation; the Release control stayed within budget. It is
not the recommended solver fix because it leaves the multiplicative work and
its scaling with more demanding geometry intact.
