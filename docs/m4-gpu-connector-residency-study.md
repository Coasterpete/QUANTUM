# GPU Physics M4: connector residency and dispatch granularity study

Date: 2026-09-11

Baseline: `feature/gpu-rigid-bogie-prototype` at `f41f35a`

Study branch: `feature/gpu-connector-residency-study`

Configuration: MSVC Release, Vulkan 1.3 compute, NVIDIA GeForce RTX 4070,
driver 595.79, shader fp64 enabled

Production `TrainPhysics` is unchanged. This study adds measurement facilities
to the isolated rigid-bogie prototype only.

## 1. Baseline rigid performance

The original 64-thread shader was rebuilt and run before the study changes.
Times are averages of 100 dispatches through 128 jobs and 25 dispatches above
128. GPU time is end to end, including packing/upload, submission, fence wait,
invalidate, and host copy.

| Jobs | CPU us | Upload us | Submit us | Fence wait us | Readback us | GPU total us | GPU us/job | Speedup |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 32 | 30.29 | 0.68 | 11.90 | 53.53 | 0.39 | 66.50 | 2.078 | 0.455x |
| 60 | 53.35 | 0.90 | 12.12 | 57.08 | 0.57 | 70.68 | 1.178 | 0.755x |
| 63 | 56.29 | 0.94 | 12.04 | 54.06 | 0.59 | 67.63 | 1.073 | 0.832x |
| 64 | 57.68 | 0.94 | 11.73 | 54.72 | 0.61 | 68.01 | 1.063 | 0.848x |
| 65 | 59.81 | 0.97 | 13.07 | 91.77 | 0.62 | 106.43 | 1.637 | 0.562x |
| 75 | 70.27 | 1.02 | 13.38 | 88.12 | 0.64 | 103.16 | 1.375 | 0.681x |
| 96 | 88.97 | 1.30 | 24.65 | 89.72 | 0.68 | 116.34 | 1.212 | 0.765x |
| 128 | 122.50 | 1.57 | 11.93 | 94.37 | 0.85 | 108.72 | 0.849 | 1.127x |
| 256 | 243.60 | 2.63 | 12.13 | 92.74 | 1.20 | 108.70 | 0.425 | 2.241x |

The crossover remains near 128 jobs. The production-relevant 60--75 range is
not profitable.

## 2. Workgroup-size sweep and the 64-to-65 cliff

The shader now uses specialization constant 0 for `local_size_x`. Four Vulkan
pipelines exercise 32, 64, 128, and 256 without shader duplication. The device
reports subgroup size 32, maximum X workgroup size 1,024, maximum 1,024
invocations per workgroup, 64 valid timestamp bits, and a 1 ns timestamp
period. All four variants are supported.

The following is one complete Release sweep. Host timings contain occasional
driver jitter, so small differences between local sizes should not be treated
as tuning wins.

| Local size | Jobs | CPU us | GPU total us | GPU timestamp us | Speedup |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 32 | 32 | 28.45 | 68.02 | 18 | 0.418x |
| 32 | 60 | 53.39 | 68.67 | 18 | 0.777x |
| 32 | 63 | 57.11 | 71.34 | 18 | 0.800x |
| 32 | 64 | 56.84 | 74.22 | 18 | 0.766x |
| 32 | 65 | 59.38 | 105.45 | 53 | 0.563x |
| 32 | 75 | 69.69 | 108.60 | 54 | 0.642x |
| 32 | 96 | 89.46 | 121.46 | 54 | 0.737x |
| 32 | 128 | 119.36 | 108.43 | 55 | 1.101x |
| 32 | 256 | 243.36 | 109.04 | 55 | 2.232x |
| 64 | 32 | 28.78 | 69.06 | 18 | 0.417x |
| 64 | 60 | 56.79 | 69.19 | 18 | 0.821x |
| 64 | 63 | 61.67 | 71.01 | 19 | 0.868x |
| 64 | 64 | 57.20 | 71.97 | 19 | 0.795x |
| 64 | 65 | 60.65 | 106.20 | 54 | 0.571x |
| 64 | 75 | 68.61 | 119.32 | 54 | 0.575x |
| 64 | 96 | 90.72 | 106.09 | 55 | 0.855x |
| 64 | 128 | 121.17 | 114.47 | 58 | 1.058x |
| 64 | 256 | 238.46 | 123.79 | 57 | 1.926x |
| 128 | 32 | 28.00 | 76.37 | 18 | 0.367x |
| 128 | 60 | 52.76 | 72.10 | 19 | 0.732x |
| 128 | 63 | 57.68 | 69.91 | 19 | 0.825x |
| 128 | 64 | 59.20 | 81.70 | 19 | 0.725x |
| 128 | 65 | 62.39 | 108.69 | 56 | 0.574x |
| 128 | 75 | 70.73 | 110.14 | 57 | 0.642x |
| 128 | 96 | 89.13 | 116.29 | 60 | 0.766x |
| 128 | 128 | 122.23 | 117.38 | 65 | 1.041x |
| 128 | 256 | 253.72 | 128.09 | 65 | 1.981x |
| 256 | 32 | 30.89 | 67.80 | 18 | 0.456x |
| 256 | 60 | 54.83 | 83.25 | 18 | 0.659x |
| 256 | 63 | 56.14 | 76.64 | 19 | 0.733x |
| 256 | 64 | 59.52 | 70.64 | 19 | 0.843x |
| 256 | 65 | 60.90 | 113.15 | 55 | 0.538x |
| 256 | 75 | 71.89 | 113.56 | 56 | 0.633x |
| 256 | 96 | 90.24 | 110.86 | 59 | 0.814x |
| 256 | 128 | 119.93 | 121.74 | 64 | 0.985x |
| 256 | 256 | 246.01 | 131.34 | 76 | 1.873x |

The cliff follows job content, not workgroup count. Job 64 (the 65th job) is at
station 26.744 m, increasing travel, and takes 10 rigid refinements while the
preceding ordinary jobs take at most one. Its divergent lane raises timestamped
shader duration from about 18--19 us to 53--56 us for every local size. In
particular, sizes 128 and 256 still show the cliff even though all 65 useful
invocations fit in one workgroup. The 32-thread variant is generally best at
larger batches, consistent with the native subgroup size, but does not make
60--75 jobs profitable. There is no evidence supporting a permanent local-size
change.

Portable SPIR-V tools validate the shader but do not expose NVIDIA register
allocation or achieved occupancy. Static SPIR-V contains 85 `OpFunction`
records, 203 `OpVariable` records, and substantial fp64/sample-search work;
these counts are not a substitute for hardware register metrics. The measured
divergence is the actionable occupancy implication. Consumer RTX fp64
throughput and binary track-sample searches remain limiting factors.

## 3. Connector call graph and state

```text
solveFollowingCar
  baseSeparation = following.frontHitch.x - leading.rearHitch.x
  expectedOffset = max(0, baseSeparation + connectorLength)
  geometryScale and searchHalfExtent = max(0.5, 1.1 * geometryScale)
  bound search interval to < one circuit lap or available open track behind
  divide the interval into the fixed 160-cell grid
  choose the cell surrounding expectedOffset
  connectionCandidate(lower), connectionCandidate(upper)
    followingCarLocation(offset), preserving travel direction
    requireLegalOpenCarPlacement
    solveFrontHitchPositionForValidatedDefinition
      solveCarBodyGeometry
        solveBogieStations (complete rigid-bogie root solve)
      construct body frame and transform authored front hitch
    residual = distance(frontHitch, leadingRearHitch) - connectorLength
  if needed, expand through at most 16 adjacent grid cells
    choose the next lower/upper cell by midpoint distance to expectedOffset
    lower wins exact distance ties
  on first nearest local sign bracket, bisect for at most 80 iterations
    stop when either endpoint is within 1e-8 m
    select lower on equal absolute endpoint residual
  if the local path fails, evaluate all 161 grid points
    reset adjacency across illegal open-track candidates
    retain best residual, breaking ties by distance to expectedOffset
    retain the sign bracket whose midpoint is nearest expectedOffset
  refine the selected exhaustive bracket with the same bisection
  for a zero-length tangent root, run the 80-step golden-section fallback
  solveLegalCarPose again at the accepted offset
  return the final pose, iteration count, bracket width, and fallback class
```

One candidate needs the resident compiled-track samples/topology/length, the
leading car reference location and rear-hitch world position, the following
car's bogie pivots/body and front-hitch definitions, loadout only where final
pose mass data is requested, connector length, offset, direction, and open
placement limits. Its temporary state is the complete rigid solve bracket,
sampled pivot positions/stations, body frame, front-hitch world position,
distance, residual, status, and rigid iteration/expansion counters.

One complete connector solve additionally needs both car definitions, the full
accepted leading pose, expected-offset/search-bound scalars, grid indices,
lower/upper/best/previous candidate records, nearest-bracket tie state,
fallback classification, connector iteration count, and final accepted car
pose. Only the accepted following pose is needed to start the next connector.

## 4. GPU-residency options

| Design | GPU work unit | Dispatch/synchronization per `solveTrainPose` | Transfer | Realistic width | Equivalence risk | Complexity | Fallback | Likely benefit |
| --- | --- | --- | --- | ---: | --- | --- | --- | --- |
| Candidate evaluation | One offset, including complete rigid solve and connector residual | Initial/local waves plus about 21 dependent refinement waves | Candidate inputs up; residual/status down each wave | 3 hypotheses per currently known offset; 60--75 only with speculation | Medium | High CPU/GPU scheduler | Replay candidate/connector on CPU | Negative; preserves the round-trip loop |
| Complete connector solve | One connector performs grid search, nested rigid solves, refinement, and returns accepted pose | One per car layer; three layers for a four-car pose | Leading pose/definition up; one compact accepted pose down | 3 for center/before/after, then 1 committed | High | High | Whole connector on CPU | Better arithmetic intensity, but six exposed submits per normal step if CPU consumes every layer |
| Cooperative connector | One workgroup per connector; lanes evaluate initial/adjacent/exhaustive candidates | Same dependency-layer count as complete connector | Same compact final result | 3 then 1 workgroups | High; ordering/ties need explicit reductions | Very high | Whole connector on CPU | Helps fallback/grid work; ordinary bisection remains serial and fp64 divergent |
| Persistent connector queue | Resident kernel consumes connector jobs and publishes completions | Host polling/signaling replaces submits, but car layers still depend | Queue records and compact results | 3 then 1; more with multiple trains | High; Vulkan memory protocol | Disable queue and use CPU | May reduce submit cost, not the observed long-lane execution cost; unjustified scheduler complexity now |
| Complete train-pose residency | One invocation/workgroup owns one dependency-ordered train pose; candidate and rigid temporaries stay resident | One batch for center/before/after, one for committed pose | Stations/immutable definitions up; final poses/kinematic inputs down | 3, then 1 | Highest | Very high, isolated prototype first | Whole unchanged CPU `solveTrainPose` | Only option reducing normal step to two host synchronization points |

Cars within a train are not parallel. Connector `i+1` cannot start until the
accepted following pose from connector `i` exists. Multiple trains increase
width, but the current realistic model is one four-car train.

## 5. Feasibility and result residency

The current Release R=25 benchmark confirms the prior deterministic counts:

| Quantity | Per four-car step | Per connector |
| --- | ---: | ---: |
| `solveTrainPose` calls | 4 | n/a |
| Connector solves | 12 | 1 |
| Connector candidates | 274 | 22.83 average (20/24 min/max) |
| Connector refinements | 250 | 20.83 average (18/22 min/max) |
| Complete rigid solves | 290 | 23.83 average after allocating four lead-car solves |
| Rigid refinements | 301 | approximately 24.75 |
| Track samples | 1,766 | approximately 145 after top-level and lead allocation |

The extra rigid solve per connector is the final accepted `solveLegalCarPose`.
An ordinary complete connector job therefore has enough arithmetic to be more
than a trivial dispatch, and almost all candidate/rigid state can remain in
registers or workgroup-local memory. It can retain only the current bracket,
best candidate, and accepted pose instead of materializing every candidate.

At the existing rigid ABI, flattening 290 jobs would move 25,520 upload bytes
and 11,600 readback bytes per step, even before the CPU/GPU control traffic.
Keeping those results resident eliminates almost all of that traffic. Per
connector it avoids about 2,097 upload bytes and 953 readback bytes on average;
only a final compact pose/status/counter record must cross back. More important
than the bytes is eliminating approximately 23 internal rigid-result
synchronization opportunities per connector.

Complete connector residency is necessary but not a sufficient boundary. If
the CPU reads each following car before launching the next connector, a normal
step still needs three dependency-layer waits for center/before/after and three
more for the committed pose. At the measured roughly 67--73 us host-visible
floor, six waits alone are about 0.40--0.44 ms, already slower than the full
0.237 ms Release CPU step. The accepted pose must feed the next connector on
the GPU; that is complete train-pose residency.

## 6. Async overlap

The normal interior step is:

```text
center pose -> before pose -> after pose -> kinematic derivatives
-> force/resistance/effective-mass math -> integration -> committed station
-> committed pose -> angular/telemetry assembly
```

Center, before, and after are independent once their stations are known and
should be one GPU batch, not three serial submissions. Their results are all
required before derivative, force, and integration work can begin. The
committed station does not exist until that CPU work finishes, so committed
pose work cannot overlap it. After committed submission, angular-kinematic and
some telemetry assembly can run because they use the current evaluation, but
this is a small tail and cannot hide six connector-layer waits.

No async scheduler was implemented. Timestamp measurements bound the useful
case: ordinary <=64-job shader execution is only 18--19 us while exposed
end-to-end latency is commonly 67--73 us; the majority is queue/fence latency,
and the production CPU has no independent 50-us block between connector
refinement decisions. Double buffering helps throughput across independent
trains or offline jobs, not the latency of one dependency-ordered train.

## 7. GPU execution versus host latency

For ordinary <=64-job batches, the timestamped dispatch is about 18--19 us.
Packing/upload is about 0.6--0.9 us, command recording plus `vkQueueSubmit` is
usually 11--14 us, fence wait is about 54--60 us, and invalidate/copy is below
1 us. These host categories overlap conceptually: fence wait includes driver
scheduling plus remaining GPU execution and must not be called shader time.

At 65 jobs the divergent ten-refinement job raises actual execution by about
35--37 us, and fence wait rises by a similar amount. Readback bandwidth is not
the limiting cost. Queue/submission latency and long-lane shader divergence are.

## 8. Realistic production model at 240 Hz

The current Release R=25 four-car CPU step is 0.2367 ms, about 5.68% of one
4.1667 ms frame budget at 240 Hz. It performs four train-pose evaluations and
12 connector solves. The four poses are not simultaneously known: center,
before, and after are known together; committed is known only after CPU
integration. Within each pose, three connectors are strictly ordered.

The 290 rigid solves per step are a theoretical aggregate, not a legal
simultaneous batch. Treating them as one approximately 110-us GPU dispatch
would be dishonest. Candidate speculation can expose roughly 60--75 rigid jobs
around a connector layer, but measured batches in that range take about
69--109 us versus about 53--70 us on CPU and therefore lose. A complete
connector shader reduces internal synchronization, but returning each accepted
car creates six host waits per step and also loses on the measured latency
floor. Only a complete train-pose prototype can test a two-dispatch model:
three pose jobs first, one committed pose job later. No performance claim is
made for that unimplemented shader.

## 9. Decision

Workgroup tuning does not fix the realistic batch range. The current call graph
has insufficient independent CPU work to hide rigid-pipeline latency. CPU-owned
connector refinement recreates the synchronization problem, and stopping at a
host-visible complete connector still synchronizes once per dependent car.

The next isolated milestone should study complete GPU train-pose residency.
It should preserve the unchanged CPU pose solver as oracle and fallback, keep
the accepted following pose resident between connectors, batch the three
finite-difference hypotheses, and return only the complete pose/kinematic data
needed by CPU force and integration. Production integration must wait for
numerical equivalence and a measured win for the real 3-then-1 workload.

No connector shader was implemented in M4. A representative connector port
requires the full fixed-grid ordering, open/circuit legality, exhaustive and
tangent-root fallbacks, final pose reconstruction, and deterministic failure
categories. More importantly, a host-visible connector prototype would measure
the wrong residency boundary. Implementing a partial substitute would not
answer the production question.

**M4 READY -- TRAIN-POSE GPU RESIDENCY STUDY REQUIRED**
