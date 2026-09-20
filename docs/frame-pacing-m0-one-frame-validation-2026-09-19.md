# Frame-pacing M0: one-frame-in-flight validation — 2026-09-19

## Verdict

The evidence supports keeping one frame in flight as QUANTUM's production
FIFO pacing policy on the tested machine.

Average throughput did not regress meaningfully: the matched idle median moved
from 98.64 to 98.42 FPS (-0.22%) and playback moved from 98.59 to 98.50 FPS
(-0.09%). Idle pacing improved substantially: median p95 fell from 19.06 to
11.41 ms, p99 fell from 20.28 to 18.02 ms, and frames above 16.667 ms fell from
349 to 22 per 20 seconds. The idle worst-frame range fell from 78.99–84.52 ms
to 44.25–60.10 ms.

The mode passed camera, playback, retained-buffer style edits, continuous
transition edits, authored supports, repeated window resize, and swapchain
recreation. A Debug run with `VK_LAYER_KHRONOS_validation` active completed four
swapchain recreations without a VUID or validation-error message.

One frame in flight does not eliminate every outlier. Isolated 44–65 ms frames
remain, and the continuous-edit buffer-retirement cost remains intentionally
unchanged in this milestone.

## Implementation and ownership review

`VulkanContext` now intentionally has one CPU frame slot. The shared policy
sizes every slot-owned resource:

- one primary command buffer;
- one image-available semaphore;
- one submission fence, created signaled;
- one timestamp-query pair and submitted flag;
- one dynamic train-preview buffer;
- one retained submission-history record.

`drawFrame()` waits for the slot fence before reusing any slot-owned command,
query, synchronization, or preview resource. The fence is reset only after a
successful image acquisition and immediately before command recording and
submission. An acquire-out-of-date return therefore cannot strand a reset
fence. With one slot, the waited submission is normally draw N-1.

Render-finished semaphores remain one per swapchain image and are selected by
the acquired image index. They were not collapsed to the frame-slot count.
This preserves the existing presentation-wait ownership rule. FIFO present mode
is unchanged.

Full-retirement paths still wait for every frame fence. With one slot this
means the sole submitted frame must complete before shared retained buffers,
viewport attachments, descriptors, or the synchronous readback buffer are
rewritten or destroyed. Swapchain recreation still waits for device idle,
replaces per-image views and render-finished semaphores, and lets the editor
reinitialize its ImGui Vulkan backend when the generation changes.

Startup logging and spike JSON now record the active frame-count policy. The
smoke harness also records viewport resize and swapchain recreation counts.

## Matched audit matrix

All rows use the same Release build, machine, Modern Steel fixture, FIFO
presentation, and three independent runs used by the preceding performance
audit. Values are medians except the worst-frame range.

| Workload | Configuration | Avg FPS | 1% low FPS | p95 ms | p99 ms | Worst ms | Frames >16.667 ms |
|---|---|---:|---:|---:|---:|---:|---:|
| Idle | Two-frame audit baseline | 98.64 | 31.17 | 19.06 | 20.28 | 78.99–84.52 | 349 |
| Idle | One frame, production path | 98.42 | 35.96 | 11.41 | 18.02 | 44.25–60.10 | 22 |
| Playback | Two-frame audit baseline | 98.59 | 35.60 | 10.87 | 17.02 | 60.19–86.37 | 20 |
| Playback | One frame, production path | 98.50 | 35.68 | 11.69 | 16.87 | 50.08–65.38 | 20 |
| Camera orbit | Two-frame audit run | 98.31 | 33.98 | 18.76 | 19.90 | 54.20 | — |
| Camera orbit | One frame, three-run median | 99.19 | 42.47 | 11.19 | 14.32 | 59.47–60.08 | 9 |

Idle median frame-slot wait was 8.71 ms while the median GPU timestamp span was
0.59 ms. One frame in flight changes where FIFO backpressure is admitted; it
does not make the display-paced wait or the remaining lower-level outliers
disappear. No claim is made about a Windows or NVIDIA scheduling cause.

## Extended validation

The existing `modern-steel-validation.quantum` fixture provided six authored
regions, Modern Steel dual-rail geometry, dense 0.75 m repeating hardware,
spine geometry, region/style overrides, and a four-car preview.

| Scenario | Result |
|---|---|
| Playback plus camera orbit | 98.85 FPS, 13.78 ms p99, completed normally |
| Continuous transition edit plus camera orbit | 99.81 FPS, 12.50 ms p99; all 30 changed edit frames completed |
| Repeated rail-material, rail-spacing, and hardware-spacing edits | All completed; buffer-owning paths retained their required fence drains |
| Playback + camera + hardware-spacing edit + repeated resize | 99.41 FPS over 796 frames and 1,921 physics steps; four swapchain recreations; completed normally |
| Authored supports + Modern Steel + playback + camera | 98.35 FPS, 18.56 ms p99; completed normally |
| Debug validation + supports + playback + camera + resize | 99.31 FPS; 10 viewport resize/retirement observations and four swapchain recreations; completed normally |

The support case uses the deliberately small
`smoke-tests/support-frame-pacing-validation.quantum` fixture: an 80 m Modern
Steel shuttle with dense crossties and four authored two-member support bents.
No elaborate benchmark scene was added.

Continuous geometry editing intentionally rebuilds and stops the simulation
preview under the current editor behavior, so it was tested with a stopped
preview. The combined simulation/editing workload used the presentation-only
hardware-spacing edit, which leaves playback active while still exercising a
retained GPU-buffer update.

## Verification

- Release and Debug `QUANTUM` builds passed.
- Release and Debug `QuantumEditor.PreviewSmoke` tests passed.
- All 78 enabled Release CTest tests passed. The disabled
  `QuantumEngine.GpuTrainPoseResidency` study remained disabled.
- Debug Vulkan validation was confirmed active. The resize log contains no
  VUID or validation-error message.
- `git diff --check` passed.
- Reports and logs are under
  `build/diagnostics/frame-pacing-m0-20260919/`.

## Remaining risks

- Residual 44–65 ms outliers remain and still require an elevated ETW/WPR trace
  for lower-level attribution if they become the next priority.
- Input latency was exercised through deterministic camera movement and normal
  editor updates, but no external high-speed input-latency apparatus was used.
- The known continuous-edit buffer-retirement cost remains approximately the
  same and was not addressed, as required by this milestone.
- The existing swapchain-retirement limitation described in
  `docs/fence-stall-investigation.md` remains: device-idle retirement is used
  rather than present fences from `VK_EXT_swapchain_maintenance1`. No validation
  failure or recreation defect was observed in this milestone.
