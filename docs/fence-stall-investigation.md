# Frame-slot fence investigation — 2026-09-06

Scope: continuation from the already-tested minimize/restore timing correction.
No physics, interpolation, catch-up policy, VSync or frames-in-flight changes
were made in this continuation. The earlier 479.1099 ms wait remains historical
CPU-side evidence; it did not contain submission or GPU execution timestamps.

## Final diagnosis

The retained evidence does **not** confirm a presentation-semaphore reuse defect
in the ordinary frame loop. QUANTUM already allocates one render-finished
semaphore per swapchain image and selects it with the acquired image index. That
is the synchronization pattern recommended by the Vulkan Guide: reacquiring an
image and waiting on the acquire semaphore before rendering establishes that the
previous presentation of that image, including its wait on the corresponding
render-finished semaphore, has completed. The current submit performs that
acquire wait before it signals the same image's render-finished semaphore.

The measured fence stall also starts before the current `vkAcquireNextImageKHR`,
before the current render-finished semaphore is selected, and before the current
submission. Current-frame presentation-semaphore reuse therefore cannot cause
that already-in-progress wait. The instrumented results confirm delayed
completion of a previously submitted frame-slot fence; they do not identify
whether the lower-level delay was GPU execution, a driver/display dependency, or
OS scheduling. The historical 479.1099 ms instance remains unattributed at that
lower level.

There is a separate specification-level lifetime limitation during swapchain
replacement and shutdown: `vkDeviceWaitIdle` alone does not formally prove that
an unextended `vkQueuePresentKHR` operation has released all presentation
resources. QUANTUM currently relies on that customary fallback before destroying
the old per-image semaphores and swapchain. This is not the cause of the retained
uninterrupted-playback stalls: none of their records contains a swapchain
recreation, and no old semaphore was destroyed or recycled in their causal
windows. `VK_EXT_swapchain_maintenance1` present fences are the direct mechanism
for making that retirement proof explicit.

## Method and added observations

The existing repeating 60-second smoke fixture was run serially in matched
normal/OBS-disabled Debug and Release environments, twice per configuration.
Compilation, CTest and Computer Use interaction were kept out of these matched
windows. Each pair used the same executable, fixture, duration and loader logging.
The supplied fixture is a short straight profile, not a curved-track stress test.

CPU telemetry now retains the selected frame slot, status returned by
`vkGetFenceStatus` immediately before waiting, exact steady-clock wait timestamps,
and the submission previously attached to that slot. It also records reset,
command-recording, queue-submit and present timestamps/results, image index,
swapchain generation and whether submission/presentation actually occurred.
The render-finished semaphore is identified by its image index and swapchain
generation, matching its ownership in VulkanContext; no raw handles are exported.
The immediately preceding rendered frame is retained too. In the normal two-slot
cycle the fence belongs to **N-2**, not **N-1**.

These records are plain non-owning values. They add no GPU resources or queries.
There is one `vkGetFenceStatus` observation per draw, existing bounded spike
storage, and two per-slot CPU history records. Smoke serialization happens at
finish. Fence-only stalls now qualify for retention immediately, even though
their own incoming delta may be normal and their effect appears on the next
frame. The TXT spike list identifies the fence duration and originating draw.

## OBS layer isolation

The installed manifest `C:/ProgramData/obs-studio-hook/obs-vulkan64.json` specifies
`DISABLE_VULKAN_OBS_CAPTURE`. Normal logs explicitly show OBS inserted into both
the instance and device call chains. OBS-disabled logs omit it from both chains;
merely discovering the manifest is not evidence that a layer is active.
Debug also loads Khronos validation; Release does not. NVIDIA Optimus and
Presentation layers remain active in both environments.

The comparison runner sets `DISABLE_VULKAN_OBS_CAPTURE=1` only in its process
and inherited child environment. No registry, manifests, drivers, startup files
or system/user environment settings were modified. The manifest's switch agrees
with the [OBS source manifest](https://github.com/obsproject/obs-studio/blob/master/plugins/win-capture/graphics-hook/obs-vulkan64.json).
The loader's implicit-layer rules are documented by
[Khronos](https://github.com/KhronosGroup/Vulkan-Loader/blob/main/docs/LoaderLayerInterface.md).

An initial runner mistakenly restored a missing variable as an empty but present
variable through PowerShell/.NET. Loader-chain verification exposed this before
interpretation. That completed run is preserved as `fence-debug-no-obs-exploratory`
and excluded from matched statistics; an in-progress duplicate was stopped.
The corrected runner removes the variable for a normal run and verifies actual
layer activation from each log. The exploratory OBS-disabled run nevertheless
captured a 63.5504 ms wait, so OBS is not necessary for all fence stalls.

## Synchronization audit

The active slot is `frameIndex % 2`. Its fence is waited before buffer reuse and
before resetting that fence. Acquire-out-of-date returns before the reset.
After reset, command-buffer errors or submit failures throw and terminate the
run; there is no silent return that leaves a reset fence for another iteration.
Successful presentation advances the slot index. Recreation drains the device
before replacing swapchain resources. Render-finished semaphores are indexed by
swapchain image, not by CPU frame slot.

The retained records are checked for originating-slot agreement, a successful
originating submit/reset, successful present, and the expected two-draw distance.
There is no evidence in these records of a missed queue submission or waiting
on the wrong frame-slot fence. This does not prove a particular GPU workload,
external layer or OS scheduler was responsible for the elapsed wait.

The presentation semaphore ownership is also correct for normal rendering.
`imageAvailableSemaphores_`, command buffers and frame fences are indexed by CPU
frame slot. `renderFinishedSemaphores_` is sized to the swapchain image count and
indexed by the acquired image. This matches the Vulkan Guide's safe-reuse model;
a frame-slot fence alone would not be sufficient proof that a presentation wait
had consumed a frame-indexed semaphore. No matched log contains
`VUID-vkQueueSubmit-pSignalSemaphores-00067` or another validation complaint
about reuse.

## Results: root cause and limits

**The exact lower-level cause of the historical ~479 ms fence stall remains
unattributed.**
It was not reproduced at that magnitude in the eight matched runs. The new
records captured smaller stalls of the same observable form, but do not prove
that they share its lower-level cause. A CPU wait inside Vulkan is not a GPU
execution-time measurement.

Across the matched runs, 22 retained waits exceeded 33.3 ms. Every one had
`VK_NOT_READY` (1) before the wait, a successful reset and submit for the same
slot two draws earlier, and successful presentation. There were zero slot,
submission-result or draw-distance mismatches. The maximum originating
reset-to-submit gap was 0.1689 ms, originating `vkQueueSubmit` call duration
0.0391 ms, and originating present duration 0.0358 ms. No recreation, mutation,
readback, dialog, focus or minimize tags accompanied those waited frames.
This rules out a withheld CPU submission or wrong-slot wait **in these records**.
No validation errors or VUID messages were found in the matched logs.

A fast acquire call does not establish when its image-available semaphore became
ready. A fast present call does not establish display completion. Fence elapsed
time can include queued dependencies, presentation backpressure, driver/GPU
scheduling and late CPU rescheduling. The records do not separate those from
long GPU work. ImGui command recording was included in the measured reset-to-
submit interval and was short; GPU execution of ImGui is not separately measured.

OBS is active in normal runs and absent in disabled runs. Disabling it did not
consistently improve the tails: Release pair 1 improved, but pair 2 got worse
with OBS disabled. The 49.3343 ms disabled Release wait and the exploratory
63.5504 ms disabled Debug wait show that OBS is not necessary for every stall.
Its API-version warning is not proof that it caused the historical 479 ms wait.
Release stalls also establish that Khronos validation is not necessary.

An OS trace was attempted after the matched runs with `wpr -start GPU -filemode`.
It failed with **0xc5585011**, “Failed to enable the policy to profile system
performance.” The process was not elevated; `wpr -status` confirmed that no
recording was active afterward. The attempt did not produce an ETL file or
change system configuration. Consequently there is no WPR evidence for GPU queue
execution, compositor/display scheduling, driver DPC/ISR activity, or whether the
application thread was descheduled while inside `vkWaitForFences`. The app-side
steady-clock brackets cannot separate those cases. No GPU timestamp-query
infrastructure was added. An elevated GPU/CPU scheduling trace during a
reproduced large wait is the next useful attribution step; it is not routine
manual app reopening.

**No renderer behavior fix was justified or applied.** There is no before/after
engine-fix claim. The environment comparison below is an experiment, not a cure.
Further renderer optimization, changing FIFO or adding a frame in flight is not
justified by these measurements. The earlier event-pump stalls and suspension
fix were intentionally outside this continuation's scope.

The smallest safe next implementation concerning presentation resources is a
separate, narrowly scoped swapchain-retirement change: query and enable
`VK_EXT_swapchain_maintenance1` when available, attach a fence to each present,
and retire old swapchain semaphores only after the relevant present fence signals.
If support is required on devices without that extension, the fallback needs a
small deferred-retirement list tied to later reacquisition/presentation proof.
Neither change should be represented as a fix for the uninterrupted fence stalls
without a reproduction that includes recreation. For those stalls, obtain the
elevated WPR trace first; add GPU timestamp queries only if that trace still
cannot distinguish GPU work from driver/display or CPU scheduling delay.

## Exact captured sequence

For `fence-release-normal-1`, draw/frame 1277 is the worst instrumented matched
wait (61.2034 ms). Times below are milliseconds on the same steady-clock epoch.

- **Origin N-2, draw 1275:** slot 0 first uses
  `imageAvailableSemaphores_[0]`, then acquires swapchain image 1. Its submission
  waits on that acquire semaphore, executes command buffer 0, signals
  `renderFinishedSemaphores_[1]`, and signals frame fence 0. Presentation then
  waits on `renderFinishedSemaphores_[1]`. Fence reset began
  235098822.2209 and ended 235098822.2220 (success). Command recording took
  0.0703 ms. Submit began 235098822.3003 and returned success at 235098822.3296.
  Present began 235098822.3296 and returned success at 235098822.3648.
- **N-1, draw 1276:** slot 1 uses `imageAvailableSemaphores_[1]`, acquires image
  0, and submits command buffer 1 behind that acquire wait. The submission
  signals `renderFinishedSemaphores_[0]` and frame fence 1; presentation waits on
  `renderFinishedSemaphores_[0]`. Fence reset 235098832.4025–235098832.4035;
  submit 235098832.4680–235098832.4941, success; present
  235098832.4941–235098832.5218, success. Semaphore index 0, generation 1.
  Its incoming delta was 41.8227 ms, physics 0.2072 ms, and its own fence wait
  was 9.5816 ms. Acquire took 0.0067 ms.
- **N, draw 1277:** selects slot 0 and checks frame fence 0, which belongs to
  draw 1275's submission. Its status is `VK_NOT_READY` before waiting
  (status call 0.0013 ms). Wait began 235098832.7912 and ended
  235098893.9946, approximately 61.2034 ms. The originating submit had already
  returned approximately 10.4616 ms before the wait began. After the wait,
  only after this wait completes does acquire take 0.0074 ms and return image 0.
  The resulting submission waits on `imageAvailableSemaphores_[0]` before
  signaling the reacquired image's `renderFinishedSemaphores_[0]`; submit and
  present succeed. This acquire dependency is the proof that draw 1276's present
  wait has finished consuming semaphore 0 before it is signaled again.
  The frame's incoming delta was only 10.1544 ms, requesting/executing 3/3 ticks.
- **N+1, frame 1278:** incoming delta 61.6303 ms, 15 executed ticks and only
  0.3137 ms physics CPU. The physics work did not cause the preceding wait.

Every retained large wait has the equivalent originating/N-1/N/next-frame
records in the JSON files and the complete
`build/diagnostics/fence-causal-waits.csv`. The CSV includes
all 22 retained >33.3 ms waits, not just this representative maximum.

## Matched 60-second smoke measurements

Every run below completed successfully. All timestamps include startup frames
and boundary replay. The reports retain the worst 32 spike records per run;
the summary maxima cover all frames. No GPU busy-time conclusion follows from
these CPU measurements.

| Run | FPS | p95 ms | p99 ms | Worst frame ms | Worst fence ms | Largest raw ms | Requested/executed max | Catch-up streak |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| debug-no-obs-1 | 98.49 | 12.41 | 20.72 | 30.04 | 24.21 | 30.04 | 7/7 | 0 |
| debug-no-obs-2 | 98.54 | 12.12 | 21.45 | 30.21 | 24.39 | 30.22 | 7/7 | 0 |
| debug-normal-1 | 98.42 | 12.30 | 20.73 | 40.09 | 34.46 | 40.09 | 9/9 | 1 |
| debug-normal-2 | 98.25 | 12.24 | 21.49 | 30.47 | 24.57 | 30.47 | 7/7 | 0 |
| release-no-obs-1 | 98.51 | 12.26 | 20.43 | 39.97 | 39.44 | 39.98 | 10/10 | 1 |
| release-no-obs-2 | 98.43 | 20.29 | 29.78 | 49.79 | 49.33 | 49.79 | 12/12 | 1 |
| release-normal-1 | 98.32 | 20.34 | 28.76 | 61.63 | 61.20 | 61.63 | 15/15 | 2 |
| release-normal-2 | 98.67 | 12.02 | 19.79 | 40.12 | 39.51 | 40.12 | 10/10 | 1 |

| Run | Max acquire ms | Max present ms | Max physics/frame ms | Max individual step ms |
|---|---:|---:|---:|---:|
| debug-no-obs-1 | 0.251 | 0.279 | 6.145 | 4.031 |
| debug-no-obs-2 | 0.179 | 0.220 | 6.141 | 4.554 |
| debug-normal-1 | 0.104 | 0.230 | 7.782 | 4.927 |
| debug-normal-2 | 0.274 | 0.315 | 6.038 | 4.083 |
| release-no-obs-1 | 0.047 | 0.234 | 0.925 | 0.856 |
| release-no-obs-2 | 0.269 | 0.928 | 1.112 | 1.011 |
| release-normal-1 | 0.030 | 0.166 | 0.542 | 0.311 |
| release-normal-2 | 0.331 | 0.915 | 0.898 | 0.772 |

The two-run matched averages give the remaining direct comparison. In Debug,
normal versus OBS-disabled was 98.34 versus 98.51 FPS, 12.27 versus 12.27 ms p95,
21.11 versus 21.09 ms p99, 35.28 versus 30.13 ms worst frame, and 29.51 versus
24.30 ms worst fence. In Release it was 98.50 versus 98.47 FPS, 16.18 versus
16.27 ms p95, 24.28 versus 25.10 ms p99, 50.88 versus 44.88 ms worst frame, and
50.36 versus 44.39 ms worst fence. Those are only two samples per cell and the
individual tails vary in both directions. They support no OBS effect beyond
ordinary run-to-run variation; critically, stalls remain with OBS absent and
with validation absent.

Raw reports are `build/diagnostics/fence-{debug|release}-{normal|no-obs}-{1|2}.json`
and `.txt`, with loader-chain evidence in each matching `.log`.
The `build/diagnostics/fence-comparison.csv` contains the
unrounded values; the local runner and analysis scripts are retained in the same
directory. Outputs live under ignored `build/diagnostics` and are not committed.

## Native verification and tests

Computer Use's direct launch timed out, exposed a transient native window and
then lost it. The remaining launch process was cleaned up. The existing harness
then launched a separate 30-second Release playback without requiring user
intervention. Computer Use displayed the playing train and successfully opened
View; the subsequent telemetry-menu click occurred after the timed smoke had
exited, so no telemetry-panel screenshot is claimed for this continuation.

That separate native run passed: 99.626 FPS, p95 10.807 ms, p99 12.835 ms,
worst frame 40.347 ms, worst fence 39.4646 ms, largest raw 40.349 ms and
10 maximum executed ticks. It was excluded from the matched unattended runs
because Computer Use interaction can affect timing. See
`build/diagnostics/fence-native-release.txt`.

Final Debug and Release affected-target builds passed. The focused
`QuantumEditor.PreviewSmoke` test passed in both configurations after the final
report-format change. It checks that a fence-only stall is retained even with
a normal incoming delta and zero catch-up, preserves the actual N-2 origin,
and serializes the causal fence/semaphore identity. Existing bounded-retention
and aggregation tests remain passing.

The earlier full CTest results remain **63/63 Debug and 63/63 Release**. As
requested, those already-passed full suites were not rerun in this continuation.
There was no behavioral synchronization fix requiring renewed full-suite claims.
`git diff --check` passed. Nothing was committed.

## Files changed in this continuation

- `engine/include/quantum/renderer/FrameSynchronizationTelemetry.hpp`: small
  non-owning shared CPU trace records, no Vulkan resource ownership.
- `engine/include/quantum/renderer/VulkanContext.hpp`: embed trace and retain
  two per-slot submission records plus a draw-attempt ID.
- `engine/src/VulkanContext.cpp`: observe fence status and bracket existing
  reset/record/submit/present calls; synchronization order remains unchanged.
- `editor/include/quantum/editor/FramePerformanceTelemetry.hpp`: carry the trace
  with the existing rendered-frame sample.
- `engine/src/Application.cpp`: copy the renderer's trace into that sample.
- `editor/src/PreviewSmoke.cpp`: serialize current/origin/previous records;
  retain fence-only spikes and identify them in TXT output.
- `tests/CMakeLists.txt`: expose the lightweight engine header to smoke tests.
- `tests/PreviewSmokeTests.cpp`: focused fence-cause reporting regression.
- `docs/fence-stall-investigation.md`: this report.

All preceding working-tree changes, including the physics and suspension work,
were preserved. No physics, interpolation, tolerance, timestep or catch-up
behavior was edited in this continuation.
