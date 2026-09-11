# GPU Physics M5: complete train-pose residency study

Date: 2026-09-11

Baseline: `feature/gpu-connector-residency-study` at `44b090a`

Study branch: `feature/gpu-train-pose-residency-study`

Production `stepTrain`, `solveTrainPose`, `solveFollowingCar`, and `CarPose`
behavior are unchanged. The implementation in this study is isolated behind
`GpuPhysicsContext::uploadTrainDefinition` and `solveTrainPosesGpu`.

## 1. Exact CPU train-pose call graph

The public `solveTrainPose` first calls `validateTrainDefinition`. Validation
requires at least one car, exactly `cars - 1` connectors, finite and valid car
definitions/loadouts, positive finite aggregate mass, finite non-negative
connector lengths, valid resistance data, and no simultaneous aggregate and
per-car aerodynamic model.

`solveTrainPoseForValidatedDefinition` then performs the following exact path:

```text
track.sample(generalizedReferenceLocation)
  validate primary path, direction, and station legality
solveLegalCarPose(lead)
  requireLegalOpenCarPlacement using both nominal bogie stations
  solveCarPoseForValidatedDefinition
    solveCarGeometry
      solveCarBodyGeometry
        require exactly two bogies and distinct longitudinal pivots
        select front by greater authored X (bogie 1 wins only when greater)
        solveBogieStations
          sample nominal symmetric station pair
          reject nominal positive residual
          cap circuit search before half a lap
          exponentially expand the upper adjustment
          64-step safeguarded secant/bisection refinement
          choose upper on an equal final absolute residual
        reconstruct the least-twist body frame from both oriented bogie frames
        preserve both authored pivot transforms within scale-aware tolerance
      transform both hitches
    aggregate loaded mass and local/world COG
    construct body and bogie orientations/articulation
for connector i in lead-to-rear order
  solveFollowingCar(accepted car i -> car i+1)
    expectedOffset = max(0, following front hitch X
                            - leading rear hitch X + connector length)
    establish geometry-scaled local search bounds
    cap circuit offsets below one lap or open offsets to available track
    divide the exact interval into 160 cells
    evaluate the cell containing expectedOffset
    expand at most 16 adjacent cells in midpoint-distance order
      lower cell wins an exact distance tie
    on the first nearest local sign bracket, refine for at most 80 bisections
      stop when either endpoint is within 1e-8 m
      lower endpoint wins an equal final absolute residual
    if local search fails, evaluate all 161 grid points
      illegal open candidates reset sign-bracket adjacency
      best residual ties use distance to expectedOffset; earlier wins exact tie
      bracket midpoint ties retain the earlier/lower bracket
    refine the selected exhaustive bracket with the same bisection
    accept an already-tolerant best grid point when available
    otherwise perform the 80-step golden-section tangent-root fallback
      equal left/right residual chooses the right branch and then left result
    solveLegalCarPose again at the accepted offset
  validate connector closure and construct all connector diagnostics
  retain the accepted following pose as the next leading pose
sum car masses and mass-weighted world COGs
validate positive finite train mass and finite aggregate COG
```

Open-track boundary failures use the private `OpenConsistBoundaryError`, which
`trySolveTrainPose` alone converts to an unavailable pose for step boundary
refinement. Other invalid, rigid-root, connector-root, pose, and mass failures
remain deterministic exceptions. Circuit advancement uses modulo stations,
while the rigid solve is restricted before half a lap and a connector before a
complete lap so a geometrically repeated root cannot be selected.

## 2. GPU train/pose ABI

The proposed std430-safe ABI is explicit in
`GpuPhysicsContext.hpp` and guarded by size/offset assertions:

| Record | Size | Lifetime |
| --- | ---: | --- |
| `GpuTrainPoseJob` | 32 bytes | Per dispatch |
| `GpuResidentTrainDefinition` | 16 bytes | Resident |
| `GpuResidentCarDefinition` | 224 bytes | Resident |
| `GpuResidentConnectionDefinition` | 16 bytes | Resident |
| `GpuTrainCarPose` | 672 bytes | Result |
| `GpuTrainConnectionPose` | 288 bytes | Result |
| `GpuTrainPoseResult` | 7,520 bytes | Per job result |

The result contains the accepted car reference/front/rear stations, body frame
and quaternion, body/bogie/hitch/COG positions, bogie track frames and relative
orientations, complete connector pose diagnostics, aggregate mass/COG, failure
indices, rigid/connector work counters, and fallback classification. The fixed
prototype capacity is eight cars and seven connectors. It is not a production
train-definition redesign.

## 3. Resident immutable data

The resident buffers contain only the train record, two authored bogie pivots,
body dimensions used by search scaling, front/rear hitches, precomputed loaded
mass and local COG per car, and rigid connector lengths. Track samples and
topology remain in the already validated persistent GPU track buffers. Per-job
uploads contain only IDs/indices, path/direction, and reference station.

## 4. CPU algorithm to GPU mapping

`train_pose.comp` assigns one complete job to one invocation. It ports the
validated rigid primitive, quaternion-slerped track frames, body reconstruction,
fixed local/exhaustive connector searches, bisection and golden-section
fallbacks, open/circuit bounds, deterministic comparisons, final accepted car
solve, diagnostics, and aggregation. Accepted cars remain in invocation-local
state and are written directly to the result SSBO before the next connector;
there is no host synchronization inside a rigid solve, candidate sequence,
refinement, or car dependency chain.

The shader does not materialize a 7,520-byte result in private memory. It keeps
only the current leading/following car and candidate/bracket temporaries and
streams accepted records to storage.

## 5. CPU/GPU correctness

Correctness is not proven. Portable `glslangValidator` compilation and
`spirv-val --target-env vulkan1.3` succeed, but the target NVIDIA driver does
not return from `vkCreateComputePipelines` for the full shader within repeated
bounded attempts. The longest Debug attempt exceeded four minutes of wall time
and 176 CPU seconds. The same behavior occurs with
`VK_PIPELINE_CREATE_DISABLE_OPTIMIZATION_BIT`, so no GPU result can be compared
against the unchanged CPU oracle.

Consequently, status mismatches and maximum station, position, orientation,
connector residual, COG, counter, and fallback differences are unavailable and
must not be represented as zero.

## 6. Edge and failure cases

The focused test contains deterministic straight, circle, banked/3D, valley,
crest, circuit-seam, reverse-travel, zero-length tangent-root, open-boundary,
connector-failure, and non-finite job cases. Runtime comparison is blocked at
pipeline creation. The unchanged CPU CarPose and TrainPhysics suites pass.

## 7. One-job Release performance

Unavailable: optimized pipeline creation does not complete in the bounded run.

## 8. Three-job Release performance

Unavailable for the same reason. The harness dispatches center, center - 1 cm,
and center + 1 cm as one three-invocation submission when executable.

## 9. Timestamped GPU execution versus host latency

Unavailable because the shader never reaches execution. Job packing, upload,
command recording/submit, fence wait, readback, and timestamp-query fields are
implemented in the isolated dispatch path, but reporting zero would be
misleading.

## 10. Actual 3-then-1 step model

The harness preserves the legal production boundary: a three-pose batch,
measured CPU non-pose step work, and a separate committed-pose job. It does not
combine the committed pose with the first batch. No model is published because
neither required GPU boundary can execute.

## 11. Projected or measured step speedup

The unchanged Release performance benchmark measures the representative
four-car R=25 step at **0.2386 ms** (1,000 independent samples). GPU speedup is
unavailable. Dividing this CPU baseline by an absent GPU measurement would not
be honest.

## 12. Divergence analysis

The result ABI records per-pose totals/minimum/maximum and per-connector
candidate, connector-refinement, and nested rigid-refinement counts. Runtime
distributions cannot be collected. Static inspection still shows the expected
nested divergence: up to 161 exhaustive candidates, 80 connector refinements,
and 64 rigid refinements per candidate, with dependent connectors serialized.

## 13. Shader and resource complexity

The portable SPIR-V is 131,652 bytes, contains 40 `OpFunction` records and 717
`OpVariable` records, and validates successfully. Pipeline creation consumes a
CPU core continuously but does not return within the bounded attempts on the
RTX 4070 / driver 595.79 test machine. This is a practical compiler/driver
failure, not merely a theoretical concern about occupancy. The 7,520-byte
result stride and fixed eight-car prototype capacity are additional production
design costs that cannot be justified without executable correctness and
timing evidence.

## 14. Production integration recommendation

**D. architecture is too complex/risky for current benefit**

Do not integrate the prototype into production. A future revisit would first
need a demonstrably compilable representation of the exact connector algorithm
on the target driver, without weakening search/fallback semantics. Splitting
the solve across dispatches or returning cars to the CPU would recreate the
synchronization boundary already rejected by M4.

## 15. Build and test results

- Debug and Release shader/C++ targets build.
- `glslangValidator` and `spirv-val` pass for `train_pose.comp`.
- Debug and Release CarPose tests pass.
- Debug and Release TrainPhysics tests pass.
- Debug and Release GPU track-sampling validation passes.
- Debug and Release GPU batched sampling passes.
- Debug and Release GPU rigid-bogie prototype passes.
- The unchanged Release train-physics benchmark completes; R=25 is 0.2386 ms.
- The GPU train-pose validation executable is built, but its CTest entry is
  disabled because pipeline creation blocks before dispatch on the target
  driver.

## 16. Files changed

- `engine/shaders/physics/train_pose.comp`
- `engine/include/quantum/physics/gpu/GpuPhysicsContext.hpp`
- `engine/src/physics/gpu/GpuPhysicsContext.cpp`
- `engine/CMakeLists.txt`
- `tests/GpuTrainPoseResidencyTests.cpp`
- `tests/CMakeLists.txt`
- `docs/m5-gpu-train-pose-residency-study.md`

## 17. Final study status

The intended study changes remain confined to the files listed above on
`feature/gpu-train-pose-residency-study`. Production solver files are unchanged.

**M5 NOT READY — CORRECTNESS OR ARCHITECTURE BLOCKERS REMAIN**
