# GPU Physics M3: rigid-bogie and connector architecture study

Date: 2026-09-10

Baseline: `main` at `3199b52` (PR #40)

Configuration: MSVC Debug, Windows, NVIDIA GeForce RTX 4070, Vulkan fp64 enabled

## 1. Current profile

The deterministic benchmark uses a four-car train, three rigid connectors, a
fixed 240 Hz step, and 1,000 independent repetitions from the same legal state.
Headline step time is measured without diagnostics. A second pass collects
inclusive stage timings and work counts.

| Case | `stepTrain` ms |
| --- | ---: |
| Straight | 0.3554 |
| Circle R=50 m | 4.2019 |
| Circle R=25 m | 5.8416 |
| Circle R=15 m | 5.7358 |
| Vertical valley R=24 m | 5.8749 |
| Vertical crest R=24 m | 5.8893 |

R=25 is the representative curved workload. Its instrumented pass measured:

| Inclusive stage | ms/step | Share of instrumented `solveTrainPose` |
| --- | ---: | ---: |
| `solveTrainPose` | 6.0485 | 100% |
| `solveCarGeometry` path | 5.6873 | 94% |
| connector candidate evaluation | 5.5612 | 92% |
| rigid-bogie station solve | 4.1875 | 69% |
| compiled-track sampling | 3.2829 | 54% |

These rows overlap: a connector candidate calls car geometry, which calls the
rigid-bogie solve, which samples the track. Subtracting sampling from the
inclusive pose time leaves about 2.77 ms/step in surrounding solver and pose
math. Sampling is therefore important, but it is not an independent flat batch
in the current CPU call graph.

The synchronous M0-M2 GPU sampler has an approximately 57-68 microsecond fixed
dispatch/wait/readback floor for batches through 64 queries on this machine.
It loses to its CPU reference through 64 queries and wins at 256 and above:

| Queries | GPU us/dispatch | CPU us/batch | GPU ns/query |
| ---: | ---: | ---: | ---: |
| 1 | 57.26 | 9.77 | 57,255 |
| 2 | 60.02 | 10.37 | 30,009 |
| 6 | 56.59 | 13.13 | 9,433 |
| 24 | 59.05 | 25.46 | 2,460 |
| 64 | 67.63 | 52.56 | 1,057 |
| 256 | 91.61 | 180.76 | 358 |
| 1,024 | 101.59 | 698.58 | 99 |

This confirms that pairs of samples, or even the six samples used by a typical
rigid solve, are not useful synchronous GPU batches.

## 2. Exact call graph

For the representative interior step:

```text
stepTrain
  evaluateTrainKinematicsForValidatedDefinition
    solveTrainPose(center)
    kinematicDerivatives
      solveTrainPose(center - 0.01 m)
      solveTrainPose(center + 0.01 m)
  CPU force/integration math
  solveTrainPose(committed location)

solveTrainPose
  validate generalized location with CompiledPhysicsTrack::sample
  solveLegalCarPose(lead)
    solveCarGeometry
      solveBogieStations
        repeated sampleAt(adjustment)
          sample front station
          sample rear station
  for each following car, in lead-to-rear order
    solveFollowingCar
      repeated connectionCandidate(offset)
        followingCarLocation
        solveFrontHitchPositionForValidatedDefinition
          solveCarBodyGeometry
            solveBogieStations
      solveLegalCarPose at accepted offset
    construct connector diagnostics and append following pose
  aggregate train mass and center of gravity
```

Cars are dependency ordered: connector `i` requires the accepted pose of car
`i`. The center, before, and after train-pose hypotheses are independent once
their three generalized stations are known. The committed pose is not known
until the CPU has used the current kinematics to integrate the step.

## 3. Production rigid-bogie algorithm

Inputs known at entry are the immutable compiled track, car reference location,
two authored bogie reference positions, their definition indices, travel sign,
topology, track length, and optional diagnostics.

1. Order the two bogies by authored local X. Reject anything other than two
   bogies, non-finite definitions, or longitudinal separation at or below
   `1e-9` m.
2. Let nominal station separation be `front.x - rear.x`; let the rigid target
   be the full 3D distance between the two authored pivot positions.
3. `sampleAt(a)` advances the front by `front.x + a/2` and rear by
   `rear.x - a/2`, with direction applied to both. This is the required
   symmetric expansion.
4. Residual is the sampled world-space pivot distance minus the authored pivot
   distance. Tolerance is `max(1e-10 m, 512*epsilon*scale)`, where scale includes
   1 m, target separation, and both sampled position magnitudes.
5. Evaluate `a=0`. Return on tolerance; reject if the nominal residual is
   positive because the nominal stations already exceed the rigid target.
6. Limit added separation to twice the authored pivot distance. On a closed
   circuit, additionally limit it to
   `0.5*trackLength - nominalStationSeparation`, preventing selection of the
   corresponding root on another half-lap. Reject a non-positive interval.
7. Set the initial upper adjustment to the lesser of the limit and
   `max(2*(-lowerResidual), 1e-8*max(1,targetDistance))`. Double it while the
   residual remains below negative tolerance, retaining the previous endpoint.
8. Return an endpoint on tolerance. Reject if the maximum expansion still has
   negative residual.
9. Refine for at most 64 iterations. Use a secant fraction only in `[0.01,0.99]`
   and when finite; otherwise bisect. Replace the same-sign endpoint. Return
   immediately on tolerance.
10. After the budget, return the lower-error endpoint only if it satisfies the
    same tolerance; otherwise fail deterministically.

`CompiledPhysicsTrack::advance` supplies open-track clamp or circuit wrap.
Solver-local interval hints preserve exact interpolation semantics and fall
back to `lower_bound` when the next/previous interval is not sufficient.
All production calculations are double precision.

Failure cases are explicit: invalid two-bogie geometry, nominal overextension,
no positive local interval, no feasible root within the expansion limit,
non-convergence, non-finite pose/frame math, or final rigid-pivot verification
outside the same tolerance.

## 4. Connector algorithm and work counts

For each following car, `solveFollowingCar` computes an expected offset from
the hitch geometry and authored connector length. It searches a bounded local
interval, capped below one lap for circuits and by available track behind the
leading car for open tracks.

The fast path evaluates the two grid endpoints surrounding the expected offset
on a fixed 160-cell grid. It may expand through at most 16 adjacent cells,
choosing the next cell by midpoint distance from the expected offset. A found
sign-changing bracket is refined by bisection for at most 80 iterations and
must meet the `1e-8` m connector tolerance.

If the local path fails, the fallback evaluates all 161 grid points, chooses
the sign-changing interval whose midpoint is nearest the expected offset, and
uses the same refinement. A zero-length tangent root instead uses an 80-step
golden-section local minimum search. Open-boundary failures remain distinct
from geometric non-closure. Normal benchmark cases did not use the exhaustive
fallback.

R=25 work per simulation step:

| Counter | Count |
| --- | ---: |
| `solveTrainPose` calls | 4 |
| connector solves | 12 |
| `solveCarGeometry` / rigid-bogie calls | 290 |
| connector candidate evaluations | 274 |
| connector refinement iterations | 250 |
| rigid-bogie refinement iterations | 301 |
| track samples | 1,766 |
| interval-hint misses | 580 |
| rigid bracket expansions | 0 |
| connector exhaustive fallbacks | 0 |

A rigid-bogie call used 6 / 6.08 / 28 track samples and 1 / 1.04 / 12
refinement iterations (minimum/mean/maximum). The mean connector solve used
20 / 22.83 / 24 candidate evaluations and 18 / 20.83 / 22 refinement
iterations (minimum/mean/maximum). Four samples per
step are the top-level generalized-location validations; essentially every
other sample is inside rigid-bogie work.

## 5. Parallelism map and realistic batches

| Work | Class | Realistic width in the representative step |
| --- | --- | ---: |
| Center/before/after train-pose hypotheses | A: parallel across jobs | 3 |
| Committed train pose | A, but only after integration | 1 |
| Multiple trains | A | currently 1; scales with active trains |
| Cars within one train pose | C: sequential | 4 in a chain |
| Two bogie samples for one residual | B: parallel candidates, but too small alone | 2 |
| Complete rigid-bogie solves for known connector offsets | A | typically about 3 hypotheses x 20-25 candidates at an active connector layer |
| Initial connector bracket endpoints | B | 2 per connector |
| Up-to-16 local expansion candidates | partly B | can be speculated/batched, but CPU algorithm chooses order sequentially |
| Exhaustive fallback grid | B | 161, but not exercised in normal cases |
| Rigid root iterations | C | typically 1, maximum observed 12 |
| Connector bracket refinement | C | mean 20.83 iterations |
| Frame assembly, residual comparison, validation | D: trivial CPU work | scalar/small-vector work |

The 290 rigid jobs per step are not all known at once. A practical current-step
batch is approximately 60-75 complete rigid jobs across the three finite-
difference pose hypotheses at one connector layer. Larger batches require
multiple trains, fallback grids, queued offline evaluations, or intentional
speculation. Calling a two-query batch useful GPU parallelism would be
misleading.

## 6. Architecture comparison

### A. CPU root solver plus GPU track sampling

- Upload: 24-byte query per station; track remains resident.
- Return: current 192-byte sample per query.
- Dispatch/sync: one round for each newly requested sample set; a rigid
  candidate needs at least three dependent two-sample rounds in the typical
  curved case.
- Jobs/dispatch: 2 without speculative cross-job batching; roughly 120-150 at
  a connector layer only after restructuring the CPU scheduler.
- Complexity: medium scheduler rewrite.
- Numerical risk: low if the validated sampler remains authoritative.
- Fallback: existing CPU sampler.
- Likely speedup: none in the direct design. Measured batches through 64 lose
  to CPU and the fixed synchronous floor is about 57 microseconds.
- Slower because: repeated submit/fence/readback, 192-byte results, divergent
  root progress, and lost interval-hint locality.

### B. CPU root solver plus GPU residual evaluation

- Upload: rigid job constants plus candidate adjustments.
- Return: residual and optionally two stations.
- Dispatch/sync: one per root iteration wave.
- Jobs/dispatch: usually 60-75 at a connector layer; falls as jobs converge.
- Complexity: medium-high.
- Numerical risk: medium because CPU/GPU branching and interpolation must
  agree at every iteration.
- Fallback: CPU residual function.
- Likely speedup: unlikely for one train.
- Slower because: the CPU still serializes every safeguard decision and waits
  after each typical 1-iteration/max-12 solve.

### C. GPU executes one complete rigid-bogie root solve per call

- Upload: one reference location and two authored pivots.
- Return: status, stations, final samples, residual, and diagnostics.
- Dispatch/sync: up to 290 of each per representative step if called directly.
- Jobs/dispatch: 1.
- Complexity: medium.
- Numerical risk: medium; the exact expansion and safeguarded secant logic must
  be ported in fp64.
- Fallback: call the unchanged CPU solver.
- Likely speedup: none.
- Slower because: dispatch latency overwhelms a job that normally performs six
  samples and one refinement.

### D. GPU processes many complete rigid-bogie jobs in parallel

- Upload: a compact array of immutable job inputs; track data remains resident.
- Return: compact status/residual/stations for candidates, and final bogie
  samples only for selected placements.
- Dispatch/sync: one per queued job batch; no synchronization inside a root.
- Jobs/dispatch: prototype 32-1,024; production target at least 64, preferably
  128-256 through multi-hypothesis or multi-train queues.
- Complexity: medium-high, but isolated from production in the first milestone.
- Numerical risk: medium; each lane preserves the complete fp64 CPU algorithm.
- Fallback: deterministic per-job CPU replay, or whole-batch CPU execution when
  fp64/pipeline support is absent.
- Likely speedup: plausible only once a batch crosses the fixed-cost threshold;
  not yet claimed.
- Slower because: batches below roughly 64 jobs, consumer-GPU fp64 throughput,
  divergence up to 12 refinements, buffer traffic, and queue construction.

### E. GPU connector candidate batching

- Upload: leading rear-hitch state, following-car definition, and offset array.
- Return: residual per offset, with a selected candidate solved again for its
  complete pose unless retained on GPU.
- Dispatch/sync: at least one local-search wave plus about 21 dependent
  bisection waves per connector if CPU retains control.
- Jobs/dispatch: about 60-75 across three pose hypotheses on the fast path; 483
  for three exhaustive grids.
- Complexity: high.
- Numerical risk: medium-high because each candidate contains a complete rigid
  solve and connector tie-breaking must match.
- Fallback: existing `solveFollowingCar`.
- Likely speedup: possible for offline/exhaustive work, unlikely with CPU-owned
  refinement.
- Slower because: refinement synchronization recreates the prohibited loop;
  evaluating all 161 points would do substantially more work than the measured
  fast path.

### F. GPU-resident complete train-pose solve

- Upload: 3 initial generalized stations, then 1 committed station; train and
  track definitions remain resident.
- Return: complete poses/diagnostics needed by CPU kinematics.
- Dispatch/sync: ideally one for center/before/after and one after integration.
- Jobs/dispatch: 3 then 1 train jobs, with a workgroup cooperatively evaluating
  candidates while cars remain sequential.
- Complexity: very high; it ports rigid solving, connector search, car/body
  frames, tie-breaking, failures, and output assembly together.
- Numerical risk: high.
- Fallback: entire unchanged CPU train-pose path.
- Likely speedup: potentially high only after the primitive job throughput is
  proven and/or several trains are active.
- Slower because: only 1-3 workgroups for one train, heavy fp64, divergence, and
  a large correctness surface.

## 7. Recommendation and proposed job contract

The next implementation direction is **batched complete rigid-bogie GPU jobs**
(architecture D), as an isolated benchmark/validation primitive. It is the
smallest design that attacks the measured 69% inclusive rigid cost while
keeping bracket expansion and safeguarded root refinement on one processor.
It does not require a fence between root iterations and it has a clean CPU
oracle/fallback.

The conceptual input is:

```cpp
struct GpuRigidBogieJob
{
    std::uint32_t coasterIndex;
    std::uint32_t path;
    std::int32_t direction;
    std::uint32_t jobId;
    double referenceStationMeters;
    double frontReferencePositionMeters[4];
    double rearReferencePositionMeters[4];
};
```

Track samples and topology records stay resident. The conceptual result is:

```cpp
enum class GpuRigidBogieStatus : std::uint32_t
{
    Solved,
    NominalOverextended,
    NoLocalInterval,
    NoFeasibleRoot,
    DidNotConverge,
    NonFinite
};

struct GpuRigidBogieResult
{
    std::uint32_t jobId;
    GpuRigidBogieStatus status;
    std::uint32_t refinementIterations;
    std::uint32_t bracketExpansions;
    double frontStationMeters;
    double rearStationMeters;
    double finalResidualMeters;
    // Final position/frame payload is separate so residual-only connector
    // batches do not read back two full track samples per candidate.
};
```

The ABI sizes and exact split buffers should be finalized with shader layout
assertions during the prototype, not guessed into production now.

## 8. CPU/GPU data flow

1. Upload immutable track/topology and retain the validated CPU
   `CompiledPhysicsTrack` as oracle and fallback.
2. Build a queue of independent complete rigid jobs. Do not submit a batch
   merely because two front/rear samples are available.
3. Upload one contiguous job array and dispatch one invocation per job.
4. Each invocation performs station normalization, symmetric expansion,
   safeguarded secant/bisection, tolerance checks, and failure classification
   entirely on GPU.
5. Read back compact results once. Request or retain full final samples only
   for jobs whose poses will be consumed.
6. In validation mode, replay every job on CPU and compare status, stations,
   residual, iteration/expansion counts, positions, and frames. In production,
   replay failed or unsupported batches on CPU deterministically.

Production integration should wait until the prototype proves a crossover at
realistic 32/64/128/256-job batches. If it cannot, stop the migration rather
than inserting GPU calls into `TrainPhysics`.

## 9. Numerical requirements

- Shader `float64` is mandatory; absence selects the CPU path.
- Preserve full authored 3D pivot distance, not only longitudinal spacing.
- Preserve the exact symmetric `+a/2` and `-a/2` station adjustment.
- Preserve open clamping, circuit wrapping, and the nearest-half-lap cap.
- Preserve initial upper-bound formula, doubling behavior, and maximum added
  separation.
- Preserve scale-aware tolerance and the absolute `1e-10` m floor.
- Preserve the `[0.01,0.99]` secant safeguard and bisection fallback.
- Preserve the 64-iteration budget and endpoint selection/tie behavior.
- Return explicit deterministic status rather than NaN or silent clamping.
- Validation must cover seam crossing, reverse travel, open endpoints,
  nominal overextension, infeasible intervals, maximum iteration behavior,
  and the observed max-12/max-15 refinement cases.

## 10. Expected performance and next milestone

No rigid-bogie GPU speedup is claimed in M3 because no rigid solver shader was
implemented. The measured sampler establishes the governing constraint: fixed
synchronous cost dominates small batches, while large batches amortize it.
Complete jobs improve arithmetic intensity and reduce result traffic, but fp64
and divergent roots can still erase the gain.

The next milestone is an isolated `GpuRigidBogieJob` throughput benchmark:

1. Port only the production rigid-bogie station algorithm and its track sample
   dependency to a compute shader.
2. Generate deterministic CPU jobs from the existing circle/vertical/seam/open
   fixtures at batch sizes 1, 32, 64, 128, 256, and 1,024.
3. Compare status, final stations, residual, final frames, iteration counts,
   and expansion counts against CPU.
4. Measure end-to-end submit-through-readback throughput and GPU timestamps.
5. Establish a batch crossover before considering connector or `TrainPhysics`
   integration.

Production `TrainPhysics` integration, a GPU connector solver, and a complete
GPU train-pose solver are intentionally outside this milestone.

**M3 ARCHITECTURE READY — GPU COMPLETE RIGID-BOGIE JOBS RECOMMENDED**
