# Multi-car propulsion and diagnostics — pass 3

Date: 2026-10-12. Branch: `codex/multi-car-physics-diagnostics-pass3`.
Starting HEAD: `900d0952dfaaa7282620a12b6924bec10c1fc24d`.
Implementation checkpoint: `577d2f8` (24 files; only pass-owned changes staged).
The existing dirty audio code, Application audio hooks, audio tests, policy file,
campaign document and material assets were retained and excluded from this pass's commits.

## Findings and evidence

`QUANTUM_4isQ3oN3hP.mp4` was not found in the checkout or the user's Videos,
Downloads, Desktop, Documents or Codex attachments. No video was inspected.
Without the original document and edited vehicle parameters, the reported
0.03–0.05 m/s outcome cannot be uniquely attributed to one cause. This pass
confirmed and fixed a production placement/connector defect under the reported
train count, track length, device range, force cap and acceleration command.

The default eight-car train **fits** on 60 m: its supported straight bogie span
is 33.8 m and body envelope is 35.5 m. At the old initialization q=31.75 m,
the car reference stations were 31.75, 27.25, 22.75, 18.25, 13.75, 9.25,
4.75 and 1.25 m. The last spacing was incorrectly 3.5 m instead of 4.5 m.
Unsigned hitch distance admitted an overlapping root when the trailing endpoint
excluded the intended separated root. At tick 98, q=32.6831 m, the finite-
difference effective mass jumped from 8,000 to **2,408,000 kg**, with mass
derivative approximately **-980,000,000 kg/m**. This is numerical branch switching,
not physical train inertia or upstop/contact drag. In this controlled reproduction
it caused excess acceleration: speed at 240 ticks was 28.1521 m/s.

The corrected initialization is q=32.75 m; all seven spacings are 4.5 m.
The same 240-tick run reaches **11.0473 m/s**, with ordinary effective mass.
The force path already budgets its cap once across occupied bogies; no force
duplication was found. The initial request is 94,352 N, capped to 90,000 N,
projected into physics as 90,000 N. Gravity/resistance and the resolved coordinate
inertia determine acceleration. With default resistance, initial acceleration is
11.0894 m/s², rather than the requested 11.794 m/s².

The reported speed range also overlaps the first tick's expected speed:
11.794 / 240 is approximately 0.0491 m/s. This alone does not identify a
stall. The new committed tick counter allows the maintainer to distinguish
early playback from physics time that has stopped advancing, including host
frame stalls. No sustained render-loop freeze was reproduced in these checks.

| Default train | Loaded mass | Straight bogie span | Initial device supply | Speed after 240 ticks |
|---|---:|---:|---:|---:|
| 4 cars | 4,000 kg | 15.8 m | 47,176 N | 11.4825 m/s |
| 8 cars | 8,000 kg | 33.8 m | 90,000 N | 11.0473 m/s |
| 12 cars | 12,000 kg | 51.8 m | 90,000 N | 7.34274 m/s |

These rows use an acceleration command with a maximum force, not identical
actual force across all trains. The independent low-force matrix verifies the
mass-dependent response under the same capped total force.

Additional confirmed defects addressed:

- Connector search bounds covered only the following reference, allowing illegal
  bogie queries during refinement near a valid endpoint placement. Bounds now
  cover both bogies, including their asymmetric authored offsets.
- A valid short one-car layout could be rejected because placement search began
  at 14.75 m. Search now includes the full domain and a dimension-derived seed.
- A lead reference could clamp at an endpoint while inset asymmetric bogies
  remained legal, retaining outgoing speed. It now stops and reports intervention;
  inward force can restart it.
- Valid longer authored trains could be rejected by Apply because the existing
  shorter train happened to fit. Apply now retains the accepted configuration,
  history and document while making incompatible playback unavailable.
- The generic Simulator status now receives signed reference speed, so rollback
  agrees with the engineering view. The preview's public speed-magnitude accessor
  remains compatible with existing consumers.

## Integration and numerical contracts

The CPU reduced-coordinate rigid train remains authoritative. No independent
car velocities, spring/slack couplers, contact feedback, point-mass substitute,
acceleration override or GPU dynamics were introduced. Fixed stepping remains
1/240 s, in double precision. Circuit connector root selection is unchanged.
Connector geometric closure remains 1e-8 m. The new straight analytical tests
allow 2e-6 m/s per tick, 0.002 kg effective-mass error and 0.02 N force-projection
error; these bounds accommodate finite differences, not a different physical law.
The old perpendicular-connector test caught a temporary refinement regression;
the residual was corrected and its original IllConditioned assertion retained.

Engineering dynamics is optional in the existing Simulator. It displays actual
committed values and the last successful fixed-step force sample. It separates
controller request, capped tangent supply, integrator Q, and resolved net force.
Resistance reports dry force separately from bearing/rolling capacities, linear
resistance and aggregate aerodynamic drag. Constraint correction includes zero-
speed and endpoint interventions; it is not an inferred individual wheel load.
Rolling resistance retains the existing `mass * gravity` supported-load
approximation; contact-normal-dependent rolling resistance is not yet integrated.
Mass is kg; forces N; reference speed m/s/mph; acceleration m/s²; stations m.
Net reference acceleration is not relabeled as rider G-force.

Train Apply stops/resets and rebuilds mass, order, connectors, bogies, occupancy,
visual instances and diagnostics. Revert draft discards draft edits. Device fields
still commit separately on deactivation; they are not the Train Apply workflow.
Unavailable playback retains authored data and reports available distance versus
the solved straight reference dimensions. Curved-track feasibility still comes
from the actual rigid solver, rather than treating straight span as exact on curves.

Key ownership: `core/src/physics/TrainPhysics.cpp` owns connector roots, bounds,
endpoint integration and actual external-force projections; `TrackDeviceForces`
and `RideOperations` own request/cap/runtime telemetry. `TrainConfiguration`
computes straight reference dimensions with the existing solver.
`editor/src/SimulationPreview.cpp` captures committed diagnostics and associates
device applications with step output; `ApplicationService.cpp` owns Apply/history.
`EditorUiSimulationTelemetry.cpp` draws the engineering popup; `EditorUi.cpp`
places it, and `EditorUiOperations.cpp` clarifies device-force labels.
`PreviewSmoke` reports actual final dynamics. `MultiCarPropulsionTests.cpp` and
`Train2Tests.cpp` own the new numerical and lifecycle regression coverage.

## Verification

- Supported `windows-msvc-debug` preset configured successfully.
- Release full build passed. Full enabled Release CTest: **128 passed**, one
  pre-existing disabled `QuantumEngine.GpuTrainPoseResidency`, 230.83 s.
- Final Debug full build passed. Full enabled Debug CTest: **128 passed**, the
  same one pre-existing disabled test, 603.13 s (parallel 4).
- The first full runs in both configurations found one ConnectorLoadPhysics
  tangent-root classification regression. It was corrected before the final
  full runs; assertions, test registration and coverage were retained.
- `QuantumEditor.MultiCarPropulsion`: **532 controlled scenarios / 416,052 checks**.
  The cross-product covers 1/2/4/8/12 cars; 60/200/500 m flat, 10° uphill,
  60° uphill and connected two-Region tracks; disabled/2 kN/90 kN/1 MN caps;
  rest/5 m/s starts; ordinary/doubled masses. Further cases vary body/hitch spacing,
  bogie spacing and 0/0.5/2 m rigid connectors, sweep the former branch transition,
  test reset determinism, reference/end-of-consist boundaries and reverse recovery.
  All five counts also run Launch/CableLift/ChainLift/Transport/Brake against
  disabled comparisons, including real custom profiles for Launch and CableLift.
- Train2 lifecycle tests cover active four-to-eight Apply, 16-bogie occupancy,
  visual count, reset, exact Undo/Redo, serialization/reopen and oversize placement
  acceptance/undo recovery. Invalid authored physics continues to be rejected.
- Actual Release SDL3/Vulkan/ImGui runs passed for 4/8/12 cars, eight-car 60 m,
  and eight-car 10° uphill, with one-second programmatic playback. Eight-car flat
  finished at tick 242, 11.138892 m/s, 90,000 N device Q, -2,027.616 N resistance,
  10.996548 m/s² resolved acceleration, approximately zero constraint correction
  and zero connector residual. Uphill finished at tick 243, approximately 9.47 m/s,
  with gravity Q approximately -13,623.26 N. The slight tick-count differences
  reflect wall-clock frame scheduling; fixed-tick comparisons use the matrix.
- The real native invalid 16-car/60 m fixture refused playback with an explicit
  69.800 m bogie-span versus 60.000 m track diagnosis. Its expected smoke exit is 2.
- The engineering popup was captured and visually checked; a first capture found
  bottom clipping, corrected by centering and sizing to the available viewport.
  Native checks restored the user's layout with matching SHA-256.
- The final native Debug eight-car run also passed, with GPU sampling validation
  requested: 217 committed ticks, 9.992825 m/s, 90 kN device Q, 11.008327 m/s²
  resolved acceleration and approximately zero constraint correction. Its capture
  was visually checked. The Debug Vulkan loader reported that the installed
  `VK_LAYER_OBS_HOOK` advertises API 1.3 while the application requests 1.4.
  This external-layer compatibility warning was retained; no validation error
  or physics failure was recorded. This run is not evidence that the intermittent
  frame-fence issue is fixed.

Logs, matrix output, runtime JSON/text, layout backups, captures and generated
acceptance documents are under `build/pass3-verification` and `build/pass3-*.log`.
These are programmatic Windows runs, not human mouse/keyboard acceptance or
audio listening. Existing audio tests ran as part of the suite; their changes
were pre-existing user work. No frame-fence performance fix or FPS gain is claimed.

Commands used for the final Windows builds and suites:

```powershell
cmake --preset windows-msvc-debug
cmake --build build --config Release --parallel 2
ctest --test-dir build -C Release --output-on-failure --parallel 2
cmake --build build --config Debug --parallel 2
ctest --test-dir build -C Debug --output-on-failure --parallel 4
```

To regenerate the ready-to-open acceptance fixtures:

```powershell
.\build\tests\Release\QuantumEditorMultiCarPropulsionTests.exe --write-fixtures build/pass3-verification
```

## Exact Windows acceptance

1. From `C:\Dev8\QUANTUM`, run `.\build\editor\Release\QUANTUM.exe`.
   For a ready fixture, **File > Open** `build/pass3-verification/launch-4.quantum`.
   To author it yourself, **File > New**, select the initial Profile Region and
   set **Inspector > Dimensions > Authored length [m]** to 200, then Enter.
   Keep pitch/yaw/roll channels at zero. In **View > Coaster Setup**, set
   **Initial speed (m/s)** to 0. Default coordinate scale is one metre per unit.
2. **Workspace: Track > Train** selects the Train workspace. Verify four ordered
   cars in **Train Configuration**, with 800 kg dry mass + 200 kg load per car,
   4 m bodies, bogie X offsets ±1.15 m and 0.5 m rear connectors. These are the
   default fixture values. Draft changes require **Apply train**.
3. Return to Track. In **Track Devices**, **Add device... > Launch** if authoring
   from New. Select it, use **Enabled**, **Start (m)** 0, **End (m)** 45,
   **Mode** Constant, **Acceleration (m/s^2)** 11.794 and **Max force (N)** 90000.
   Enable **Limit to target speed**, set **Target speed (m/s)** 100, and
   **Travel direction** Forward. Leave each numeric field to commit it.
4. **Enter Simulator**, press **Reset**, then **Play**. Open **Engineering dynamics...**.
   Four cars should accelerate forward; the initial request/supply is 47,176 N,
   and initial acceleration approximately 11.571 m/s² with default resistance.
5. **Return to Editor**, enter the Train workspace. Choose Middle in **New car role**
   and press **Add car** four times, then **Apply train**. Alternatively select a
   middle car and **Duplicate car** four times to retain identical physical values.
   Draft typing alone must leave the accepted preview unchanged; **Revert draft**
   must discard it without changing devices. Apply must stop playback and reset tick/forces.
6. **Enter Simulator > Reset > Play**. Engineering dynamics should show 8 cars,
   16 bogies, 8,000 kg and 33.8 m straight bogie span. Request is 94,352 N;
   capped supply and physics Q are 90,000 N while all bogies are in range.
   **Force limit reached** is expected. At 240 physics ticks, speed is about
   11.047 m/s; frame scheduling can show a nearby value when pausing manually.
7. For equal-total-force mass comparisons, use a 2,000 N cap on both trains after
   Reset. The heavier train accelerates less. A displayed supply is distinct from
   net force: gravity/resistance and constraint correction explain the resolved acceleration.
8. With a 60 m track and eight default cars, initialization remains valid and the
   train moves until its leading bogie reaches the endpoint, where playback pauses.
   An endpoint restriction is different from weak propulsion or a static resistance hold.
9. On that 60 m layout, configure 16 default cars and Apply (or open
   `invalid-16-short.quantum`). Playback must become unavailable and report
   69.8 m supported straight bogie span versus 60 m track. Undo recovers the
   eight-car train. The authored oversized train remains saveable.
10. Open `launch-8-uphill.quantum` for the exact 10° constant-slope fixture. Reset
    and Play; device Q remains 90 kN while gravity opposes it by about 13.623 kN,
    producing less acceleration than the flat fixture. Try a low cap: rest/rollback
    can be physically legitimate, and signed speed/endpoints explain the result.
11. Use **File > Save As** for your edited test document, then **File > Open** it.
    Confirm the ordered car count, loads, connectors, device interval/cap/target,
    stopped preview and newly computed placement. Repeat Enter Simulator/Return to Editor.
12. Optionally switch Launch/CableLift to **Custom Profile**, use **Add Segment**,
    vary the acceleration values, Reset and compare supply/motion. Existing cable
    availability and return-cycle controls remain in **Ride operations...**.

## Remaining work

The exact inaccessible recording's near-immobility is still a bounded finding,
not a claimed video reproduction. Human acceptance of the changed count and
Apply/Revert controls remains necessary. The model has rigid connectors rather
than compliance/slack, and aggregate resistance prevents a general per-car axial
load interpretation. The intermittent Vulkan frame-fence slowdown remains separate.
Circuit root selection and train self-collision/compliant connector dynamics
were not extended in this open-track correction.

Recommended next substantial physics objective: author and integrate distributed
per-car resistance, then expose supported connector/bogie load recovery alongside
contact diagnostics, with conservation and curved/circuit branch-continuity tests.
This should extend the existing constrained model rather than replace its integrator.
