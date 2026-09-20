# Performance and frame-pacing audit — 2026-09-19

## Verdict

QUANTUM meets the requested 60 FPS average on the tested machine, but it does
not meet the consistency requirement. Release runs average 98–99 FPS on a
99 Hz display while 1% lows fall to 29–37 FPS and individual frames reach
60–87 ms.

There are two measured, independent hitch sources:

1. In an idle or camera-moving viewport, the main thread is usually paced by
   the two-slot Vulkan frame fence under FIFO presentation. The average GPU
   command-buffer span is only 0.5–0.7 ms, but the CPU can wait 79–85 ms for a
   frame slot. A temporary one-frame-in-flight A/B build preserved average FPS
   and substantially reduced the idle tail, so the current two-frame FIFO
   pacing policy is the highest-impact renderer lead.
2. Continuous track edits synchronously regenerate the document and drain
   in-flight users before rewriting retained buffers. A warmed transition drag
   costs 9.85 ms before simulation on every changed input frame; the first two
   frames cost 22.28 and 19.33 ms. The warmed cost is led by a 5.15 ms fence
   drain, followed by 1.57 ms centerline generation, 1.51 ms preview rebuild,
   and 0.69 ms rider-load evaluation.

Playback has a separate attribution issue. Its reported 8.88 ms “physics”
average includes one synchronous eight-query GPU preview-sampling and CPU
validation pass after the fixed steps. Removing only that audit path reduces
the physics bucket to 1.63 ms, but the same approximately 7.2 ms reappears in
the draw fence and total pacing is not materially better. It is mostly where
the display/queue wait is paid, not an overall frame-rate root cause on this
machine. Actual fixed-step CPU time averages 0.65–0.67 ms per step.

The exact component below the Vulkan fence—presentation scheduling, driver/GPU
scheduling, or CPU descheduling—cannot be separated with the available
permissions. WPR GPU tracing failed with `0xc5585011` because this account lacks
the system-performance profiling policy. The app-side evidence is sufficient
to rank the code-level boundaries, but not to name a driver or Windows
component as the final lower-level cause.

## Machine and build

- Commit: `2a47ccf47d4d88785f3a6b006e0044a45bf4d692`
- Build: MSVC Release, Vulkan validation disabled
- GPU: NVIDIA GeForce RTX 4070
- Driver: 595.79 (`32.0.15.9579` through WMI)
- Display: 3440×1440 at 99 Hz
- Present mode: `VK_PRESENT_MODE_FIFO_KHR`
- Production frames in flight: 2
- Fixture: `smoke-tests/modern-steel-validation.quantum`, six authored regions,
  Modern Steel presentation, four-car preview
- Window and presentation behavior were unchanged for production-baseline runs.

The audit added CPU aggregation, exact slowest-1% FPS, and two Vulkan timestamp
queries per frame. GPU timing is the elapsed device timestamp span from top of
pipe to bottom of pipe for the submission whose slot fence just completed. It
can include GPU preemption or scheduling delay and must not be read as pure
shader/raster work.

## Reproduction procedure

Build:

```powershell
cmake --build build --config Release --target QUANTUM --parallel 4
```

Idle viewport, three 20-second runs:

```powershell
build/editor/Release/QUANTUM.exe `
  --dev-preview-smoke smoke-tests/modern-steel-validation.quantum `
  --stopped-preview --duration 20 --spike-frame-ms 16.667 `
  --output build/diagnostics/performance-audit-20260919/idle-gpu-r1
```

Repeat as `r2` and `r3`. Playback uses the same command without
`--stopped-preview` and with `--repeat`. `--repeat` restarts only a reported
open-track boundary; physics and interpolation failures still fail the run.

Deterministic camera and edit workloads:

```powershell
# Camera movement
build/editor/Release/QUANTUM.exe `
  --dev-preview-smoke smoke-tests/modern-steel-validation.quantum `
  --stopped-preview --camera-orbit --duration 15 --spike-frame-ms 8 `
  --output build/diagnostics/performance-audit-20260919/camera-orbit

# Thirty consecutive changed transition values, frames 60–89
build/editor/Release/QUANTUM.exe `
  --dev-preview-smoke smoke-tests/modern-steel-validation.quantum `
  --stopped-preview --transition-drag --duration 1.3 --spike-frame-ms 1 `
  --output build/diagnostics/performance-audit-20260919/transition-drag-breakdown

# Presentation-only edits
build/editor/Release/QUANTUM.exe `
  --dev-preview-smoke smoke-tests/modern-steel-validation.quantum `
  --stopped-preview --duration 8 --region-style-edit hardware-spacing `
  --output build/diagnostics/performance-audit-20260919/edit-hardware-spacing
```

The other style values are `rail-material` and `rail-spacing`. The GPU-preview
A/B adds `--disable-gpu-preview-sampling` to playback. OBS isolation sets
`DISABLE_VULKAN_OBS_CAPTURE=1` only in the child process. Loader-debug logs
verify that OBS is inserted in normal runs and absent in disabled runs.

## Baseline measurements

The table reports the median metric across three independent 20-second runs.
“Worst” is the range of each run's worst frame. A 1% low is 1,000 divided by
the mean frame time of the slowest 1% of retained frames.

| Workload | Avg FPS | 1% low FPS | p95 ms | p99 ms | Worst ms | Frames >16.67 ms | Main-thread avg ms |
|---|---:|---:|---:|---:|---:|---:|---:|
| Idle, stopped preview | 98.64 | 31.17 | 19.06 | 20.28 | 78.99–84.52 | 349 | 10.14 |
| Playback, boundary repeat | 98.59 | 35.60 | 10.87 | 17.02 | 60.19–86.37 | 20 | 10.14 |
| Playback, GPU preview sampling disabled | 99.02 | 33.88 | 17.50 | 20.03 | 48.33–65.12 | 168 | 10.09 |
| Idle, OBS Vulkan layer disabled | 98.95 | 32.62 | 18.99 | 19.47 | 40.04–87.13 | 327 | 10.11 |
| Idle, temporary one-frame-in-flight A/B | 98.80 | 36.08 | 11.01 | 17.16 | 50.40–60.03 | 22 | 10.12 |

The camera-orbit run measured 98.31 FPS average, 33.98 FPS 1% low,
18.76/19.90 ms p95/p99, and a 54.20 ms worst frame. Pre-simulation CPU averaged
1.41 ms, GPU command span averaged 0.61 ms, and there were no regenerations or
uploads. It is statistically consistent with the idle presentation/fence path.

### CPU/GPU and synchronization attribution

| Workload | Physics avg / median max ms | Fence wait avg / median max ms | GPU span avg / median max ms | Event-pump median max ms |
|---|---:|---:|---:|---:|
| Idle | 0 / 0 | 8.57 / 78.86 | 0.65 / 25.90 | 1.32 |
| Playback | 8.88 / 64.94 | 0.005 / 8.72 | 0.56 / 22.70 | 1.30 |
| Playback, GPU preview sampling disabled | 1.63 / 15.01 | 7.23 / 54.48 | 0.57 / 40.08 | 1.25 |

The enabled/disabled playback comparison shows that the synchronous preview
dispatch moves the queue/display wait from `drawFrame()` into
`SimulationPreview::update()`. It does not remove it. Rendering and fixed-step
physics are not saturating their average budgets.

No retained production-baseline spike carried resize, swapchain recreation,
track mutation, hardware reload, dialog, focus, minimize, or restore tags.
Acquire and present CPU calls remained short. Event pumping was not material in
these runs; its median per-run maximum was about 1.3 ms.

## Editing and regeneration measurements

Thirty consecutive changed transition values exercised the same generic
mutation path used by a real handle drag. The first two frames include a deeper
in-flight drain; “warmed” excludes those two frames.

| Full-edit phase | All-frame avg ms | Max ms | Warmed avg ms | Warmed max ms |
|---|---:|---:|---:|---:|
| Pre-simulation total | 10.58 | 22.28 | 9.85 | 10.10 |
| Curve upload call | 5.86 | 17.23 | 5.15 | 5.29 |
| Fence time inside uploads | 5.86 | 17.23 | 5.15 | 5.29 |
| Centerline/presentation generation | 1.58 | 1.74 | 1.57 | 1.68 |
| Simulation Preview rebuild | 1.51 | 1.66 | 1.51 | 1.66 |
| Rider-load evaluation | 0.69 | 0.74 | 0.69 | 0.71 |
| Renderable mesh upload | 0.06 | 0.08 | 0.06 | 0.08 |
| Supports generation/upload | <0.01 | <0.01 | <0.01 | <0.01 |

The exact path is:

```text
EditorUi transition edit
  -> Application generic candidate mutation
  -> createCenterlineVisualization
  -> evaluateRiderLoadDiagnostics
  -> createSupportVisualization
  -> VulkanContext::updateTrackCurveVertices
       -> waitForFrameCompletion (both slot fences)
  -> updateRenderableTrack / updateSupportVertices
  -> SimulationPreview::rebuild
```

The newer presentation-only invalidation path correctly avoids unrelated Core
work, but buffer-owning changes still drain in-flight submissions:

| Edit | Pre-simulation ms | Rebuild ms | Upload/drain | Fence ms | Unrelated rider/support/simulation work |
|---|---:|---:|---:|---:|---:|
| Rail material | 1.00 | 0.01 | material publication only | 0 | 0 |
| Hardware spacing | 21.19 | 0.03 | hardware upload 20.05 ms | 20.03 | 0 |
| Rail center spacing | 20.82 | 0.41 | curves 19.28 ms + mesh 0.06 ms | 19.28 | 0 |

Thus the presentation-impact classifier is working, but mutable-buffer
retirement remains synchronous and can hitch continuous style controls.

## Ranked findings

### 1. FIFO frame-slot pacing is the largest steady-state tail source

Impact: highest for idle and camera interaction.

With two frames in flight, idle CPU pacing is bursty: normal GPU work is well
below one millisecond, yet `vkWaitForFences` reaches 79–85 ms. The temporary
one-frame build preserved average throughput and changed the median p95 from
19.06 to 11.01 ms, p99 from 20.28 to 17.16 ms, frames above 16.67 ms from 349
to 22 per 20 seconds, and worst-frame range from 79–85 to 50–60 ms. Playback
changed little, so this is an idle/camera pacing improvement, not a solver fix.

The remaining 50–60 ms outliers and GPU timestamp spans show that one frame in
flight is not a complete cure. Elevated WPR/ETW is still required to distinguish
display scheduling, driver queue scheduling, and CPU descheduling.

### 2. Continuous geometry edits synchronously drain and rebuild every frame

Impact: highest interaction-specific hitch; deterministic 9.85 ms warmed cost,
22.28 ms first-frame cost.

The curve-buffer drain is the largest phase. Full centerline generation,
rider-load evaluation, preview rebuild, and history/publication are repeated for
each changed drag value. The code is correct but leaves little budget at 99 Hz
and can miss 60 Hz when the drain catches a long presentation stall.

### 3. Buffer-owning presentation edits still block on full frame completion

Impact: 19–21 ms one-shot hitch for hardware spacing and rail spacing. Material
color publication is correctly cheap at 0.01 ms.

The specialized invalidation path removed unrelated rider/support/simulation
work, but hardware and curve/mesh buffer updates retain the same synchronous
retirement boundary.

### 4. Playback telemetry conflates fixed-step work with queue pacing

Impact: diagnosis and subsystem balance; not the measured total-frame root
cause on this display-limited workload.

Only about 1.6 ms/frame is fixed-step work in the A/B comparison. The remaining
roughly 7.2 ms in the normal “physics” bucket is the synchronous GPU preview
sample/validation round trip. Disabling it moves that wait to the renderer and
does not improve total pacing. Per-frame vectors and CPU validation are real
work, but no total-frame benefit was measured from removing this path alone.

### 5. OBS, UI event pumping, logging, and ordinary uploads are not supported as root causes

OBS-disabled runs still reached an 87.13 ms worst frame and had a 32.62 FPS
median 1% low; run-to-run variance overlaps the normal runs. Event pumping
peaked near 1.3 ms. Steady-state preview uploads are near zero, and mesh/support
uploads during the measured edit were below 0.1 ms. Normal info logging is
silent during continuous drags except at commit; audit-only logs were emitted
after the measured pre-simulation interval.

## Smallest high-impact fixes to evaluate next

1. Add a user/editor frame-pacing mode or targeted experiment that uses one
   frame in flight under FIFO, then repeat the three-run idle/camera/playback
   matrix. This has the strongest measured idle-tail improvement and is a
   one-line policy change, but it must be validated for input latency, resize,
   readback, and all per-slot resources before adoption.
2. Stop draining both frame fences for every continuous curve/style mutation.
   Use the existing per-frame ownership pattern already used by the train
   preview: publish into a slot-owned or retired spare buffer, and reclaim only
   after its owning fence signals. Preserve explicit Vulkan ownership.
3. Coalesce drag mutations to at most one published revision per rendered frame
   and avoid recomputing products that do not affect the live interaction.
   Keep the final authored value/history semantics unchanged.
4. Split fixed-step CPU time from GPU preview-sampling/validation time in the
   permanent UI telemetry. Treat the eight-query validation path as diagnostics;
   only remove or make it asynchronous after a non-display-limited A/B proves a
   total-frame benefit.
5. Capture an elevated WPR GPU/CPU scheduling trace across the reproducible
   approximately 13-second stall cluster before changing present mode, adding
   frames in flight, or blaming the NVIDIA/Windows presentation stack.

No optimization or renderer policy change was applied by this audit.

## Evidence and validation

All JSON, text, loader, and application logs are under:

`build/diagnostics/performance-audit-20260919/`

Important files include the three `idle-gpu-r*` and `playback-gpu-r*` reports,
`camera-orbit`, `transition-drag-breakdown`, the three
`playback-no-gpu-sampling-r*` reports, the three `idle-obs-disabled-r*`
reports, and the temporary `idle-one-frame-r*` reports.

The final source configuration was restored to two frames in flight. Release
`QUANTUM` built successfully, the deterministic smoke runs completed, and
`QuantumEditor.PreviewSmoke` passed. `git diff --check` was also run after the
audit changes.
