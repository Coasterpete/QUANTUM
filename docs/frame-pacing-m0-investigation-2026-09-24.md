# Frame-pacing M0 investigation — 2026-09-24

## Result

The Rendering M1 ~4.2 FPS state did not recur in fresh Windows Release runs.
Its observed blocking point was the one-frame-slot fence, but the available
records cannot identify the lower-level reason that the preceding submission
completed late. No Vulkan synchronization or presentation behavior change is
justified. This milestone adds diagnostic measurements and an actual Simulator
smoke workload; it does **not** claim a renderer fix or consistent 60 FPS yet.

The four final 20-second runs averaged 99.97–99.99 FPS on the tested 99 Hz
display. Their 1% lows exceeded 60 FPS, but an earlier matched 20-second
matrix on the same branch had 39–46 FPS 1% lows. Occasional 20–60 ms frames
therefore remain significant and variable. The GPU scheduling component of
those spikes remains unproven.

## Reproduction and measurements

All new runs used the current `feature/frame-pacing-m0` checkout based on
`bae21da54fb890bfb1b61e5b896b4c0309c05bad`, a newly built MSVC Release
executable, the `modern-steel-validation.quantum` fixture, a 1600 × 900
window on an NVIDIA GeForce RTX 4070 (driver `32.0.15.9579`), FIFO
presentation, one frame in flight, deterministic camera orbit
after frame 60, and `DISABLE_VULKAN_OBS_CAPTURE=1` in the child process.
Loader-chain logging confirmed that OBS was discovered but not inserted; the
NVIDIA presentation and Optimus layers remained active. The final executable
SHA-256 was `A546C6502E2AE944F639906599FF3FDFC898A106E23CF4899B7D7A389E3E3BB8`.
The four scenarios ran serially, with no build or test running concurrently.
The eight-second runs used a fresh build of the branch before the diagnostic
edits; the first long matrix used a fresh build with the Simulator and trace
options; the final matrix used a fresh build after reclamation timing was
added. None of those edits changed rendering or synchronization behavior.

Rendering M1's four historical eight-second Editor runs reported 4.19–4.35
average FPS from only 33–35 frames, 1.26–1.28 FPS 1% lows, 266–269 ms p95,
780–792 ms p99, and 0.49–0.59 ms average GPU timestamp spans. Their largest
frame-slot fence waits were 779–787 ms; many subsequent waits were roughly
250–267 ms. The recorded originating submission was draw N-1 on the correct
slot, with successful reset, submit, and present. Neither MSAA mode avoided
the stall. These are historical measurements, not a before/after comparison
for a code fix.

The fresh eight-second reproduction was repeated twice per Editor mode.
Each run rendered about 800 frames at 99.9–100.0 average FPS. The 1% lows
were 45.8–53.6 FPS, p95 10.4–11.2 ms, p99 13.7–15.8 ms, and the worst frame
30–60 ms. Neither run reproduced a 250 ms fence wait.

The first and final long matrices used the same 20-second workloads. The
`--simulator` smoke option enters the real Simulator workspace. Playback used
`--repeat` to restart at a reported open-track boundary, keeping physics
active; Editor playback was stopped. Both used the same camera-orbit path.

| Workspace / samples | First 20 s: FPS / 1% low / p99 ms / worst ms | Final 20 s: FPS / 1% low / p95 / p99 / worst ms | Final GPU avg ms | Final fixed steps |
| --- | ---: | ---: | ---: | ---: |
| Editor 1× | 99.06 / 46.41 / 17.92 / 50.00 | 99.99 / 75.28 / 10.25 / 10.48 / 40.19 | 0.162 | 0 |
| Editor 4× | 99.07 / 44.75 / 19.30 / 32.64 | 99.98 / 75.53 / 10.20 / 10.38 / 40.09 | 0.180 | 0 |
| Simulator 1× | 98.95 / 42.61 / 19.34 / 39.92 | 99.97 / 73.23 / 10.27 / 10.53 / 60.35 | 0.087 | 4,796 |
| Simulator 4× | 98.94 / 39.20 / 14.68 / 59.64 | 99.98 / 74.79 / 10.28 / 10.56 / 29.59 | 0.140 | 4,796 |

The final runs recorded 2,000 frames each. Their fence waits had 8.72–9.08
ms medians, 9.15–9.66 ms p99, and 29.29–59.56 ms maxima. Acquire-call p99
was 0.011–0.015 ms (maximum 0.10 ms); present-call p99 was 0.092–0.143 ms
(maximum 0.45 ms). Command-recording p99 was 0.12–0.15 ms. No deferred
buffers were pending in these four workloads. GPU timestamp maxima were
0.11–0.24 ms in the final matrix; an earlier Simulator run had a short
cluster of elevated GPU spans up to 34.4 ms. Simulator CPU physics averaged
0.71–0.74 ms per frame in the final runs, with zero interpolation failures.

The final matrix's largest frames mostly occurred during the first 8–11
frames; the earlier matrix also had mid-run spikes. A fast acquire or present
API call does not prove that an image-available semaphore was already signaled
or that display presentation was complete. The GPU timestamp span covers
recorded command execution, not every possible queue, presentation, driver,
or host scheduling delay.

## Synchronization audit and diagnostic changes

`VulkanContext` still owns one command buffer, image-available semaphore,
timestamp pair, dynamic preview buffer, and signaled-at-creation fence per
frame slot. `drawFrame()` waits for the previous submission's fence before
reusing or reclaiming slot-owned resources. It resets the fence only after a
successful acquire and immediately before recording and submission. A failed
acquire cannot strand a reset fence. Render-finished semaphores remain indexed
by swapchain image, not frame slot. Swapchain and viewport recreation still
drain in-flight work before destroying shared resources. The audit found no
wrong-slot wait, missing submission, or release-before-fence defect in the
retained records. Across the final 8,000-frame matrix there were zero
preceding-submission ID, slot, reset, submit, or present mismatches. Three
fence waits exceeded 33.333 ms; the fence was `VK_NOT_READY` before most
waits, as expected under FIFO pacing.

The smoke `--frame-trace` now includes frame IDs, the actual preceding
submission, fence status, record/submit/present CPU times, acquire time, GPU
timestamp span, event pump and preparation time, and deferred buffer counts.
Fence wait and deferred buffer reclamation are timed separately. The new
`--simulator` option makes the four-way benchmark exercise the Simulator
viewport. No frame count, present mode, physics timestep, resource ownership,
or rendering operation was changed.

An experiment with the Codex window activated over a running Release benchmark
still rendered 2,000 frames in 20 seconds at 99.97 FPS. It did not recreate
the historical stall. An attempted WPR GPU trace failed with policy error
`0xc5585011` (system-performance profiling unavailable); `wpr -status`
confirmed that no recording remained active. Without a reproduced large wait
and an OS GPU/CPU scheduling trace, assigning it to presentation, a driver,
or CPU descheduling would be speculation.

## Rendering, simulation, and resource regressions

Fresh 1600 × 900 selected-region captures for both sample counts were
byte-identical (SHA-256) to the Rendering M1 reference PNGs. They retain the
ModernSteel PBR rails and highlights, imported crossties, and green selected
region. The 4× path and single-sample fallback both rendered. In the live
Windows application, the fixture loaded in Editor, viewport zoom worked,
the 4× checkbox switched off and on, Simulator rendered track and train,
and play, pause, reset, and repeated Editor ↔ Simulator transitions completed.
The deterministic smoke path exercised camera orbit; a manual right-button
orbit was not exercised.

A Release stopped-preview rail-spacing edit completed while the frame trace
showed four deferred buffers (304,576 bytes) reclaimed after the slot fence;
reclamation took about 0.01 ms. A Debug Simulator smoke run with the Khronos
validation layer confirmed in both instance and device chains completed
1,912 fixed steps, nine viewport target resizes, and four swapchain
recreations. Its log contained no VUID or validation error. Debug performance
numbers are not comparable to the Release matrix.

## Build and tests

- Complete MSVC Release build: passed.
- Complete MSVC Debug build: passed.
- Release CTest: 78/78 enabled tests passed (`-j 2`, 50.19 seconds).
- Debug CTest: 78/78 enabled tests passed (`-j 2`, 448.34 seconds).
- The existing `QuantumEngine.GpuTrainPoseResidency` CTest remains disabled;
  it was not re-enabled or counted as a pass.

An earlier eight-way Release CTest attempt stopped making progress on this
Windows session; the two tests at that point passed individually, and the
complete two-way Release run passed. A first Debug attempt used a 180-second
per-test timeout; the CPU-heavy `QuantumCore.DynamicContactPhysics` test
timed out under that imposed limit. The complete Debug rerun with a
600-second limit passed, including that test in 448.33 seconds.

Raw smoke JSON, text, loader logs, and captures are in
`build/diagnostics/frame-pacing-m0-20260924/`. These build artifacts are not
tracked. The lower-level cause of the historical 4 FPS state and the
remaining tail spikes is unresolved; no 60 FPS consistency claim or
before/after renderer-fix claim is warranted.
