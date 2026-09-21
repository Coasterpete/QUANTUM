# Frame-pacing M1: deferred retained-buffer updates — 2026-09-21

## Verdict

Retained curve, track-mesh, support, and hardware-instance publications no
longer drain the submitted frame merely to replace their buffers. On the
Modern Steel fixture, warmed transition-drag fence time fell from 5.15 ms to
0.000 ms and curve publication fell to 0.0049 ms. Fresh one-frame-policy
baselines on this branch put rail-spacing and hardware-spacing drains at 9.45
and 9.09 ms; after M1 their publication calls took 0.008/0.048 ms for the
curve/mesh pair and 0.014 ms for hardware, with 0.000 ms attributed to a
publication fence wait.

The one-frame-in-flight and FIFO-present policies are unchanged. Release and
Debug validation covered continuous editing, camera movement, live playback,
repeated publication, resize/swapchain recreation, Modern Steel, and the
support-bearing fixture. No Vulkan VUID or validation-error message occurred.

## Resource lifetime design

Every buffer-owning edit builds and flushes a fresh candidate allocation before
changing renderer-visible state. A successful publication moves each displaced
active `VkBuffer`/`VmaAllocation` pair into the deferred list owned by
`currentFrameSlot()`. With QUANTUM's intentional one-frame policy, that slot's
most recent submission is the final possible user of the displaced buffer.

`drawFrame()` waits the slot fence before resetting or recording that slot.
Immediately after the successful wait, it destroys every deferred VMA buffer
owned by the slot. Full-frame drains also reclaim all slot lists, swapchain
recreation reclaims them after `vkDeviceWaitIdle`, and shutdown reclaims them
after its device-idle wait. Empty publications retire the previous allocation
and publish a zero draw count. Allocation or deferred-list reservation failure
leaves the current active handles unchanged.

The track mesh publishes its vertex, triangle-index, and edge-index candidates
as one group only after all three allocations succeed. Hardware instances,
track curves, and supports use the same candidate-then-publish rule. Static
hardware mesh reload retains its existing synchronous correctness boundary
because it invalidates shared cached meshes rather than an ordinary authored
buffer version. The in-place viewport-aid rewrite also retains its existing
drain. Material-only updates still only replace future command-buffer push
constant data and allocate or defer no GPU buffers.

## Measurements

The historical transition baseline is the matched warmed result from the
September 19 performance audit. The style rows use fresh before/after Release
runs from this branch and the same fixture and smoke commands.

| Edit workload | Before | After |
|---|---:|---:|
| Transition pre-simulation, warmed average | 9.85 ms | 4.697 ms |
| Transition curve publication, warmed average | 5.15 ms | 0.0049 ms |
| Transition fence time inside publications | 5.15 ms | 0.000 ms |
| Rail-spacing pre-simulation edit frame | 12.353 ms | 1.456 ms |
| Rail-spacing curve + mesh publication | 9.450 + 0.079 ms | 0.008 + 0.048 ms |
| Rail-spacing publication fence time | 9.451 ms | 0.000 ms |
| Hardware-spacing pre-simulation edit frame | 11.384 ms | 1.057 ms |
| Hardware publication | 9.112 ms | 0.014 ms |
| Hardware publication fence time | 9.085 ms | 0.000 ms |
| Rail-material publication | already non-blocking | 0.001 ms, no deferred buffer |

The complete 1.3-second transition runs retained average throughput while
improving the tail in this sample: 100.66 to 101.46 average FPS, 10.54 to
10.20 ms p95, 20.23 to 10.66 ms p99, and two to one frames over 16.667 ms.
These short runs demonstrate that throughput did not regress; they are not a
claim that M1 removes the independent FIFO/display scheduling outliers.

## Deferred-resource bounds

Each of the 30 consecutive transition updates displaced five allocations: one
curve allocation, three track-mesh allocations, and one hardware-instance
allocation. The observed maximum pending set remained five buffers / 310,200
bytes, and 150 buffers were reclaimed across the run. The rail-spacing edit
peaked at four buffers / 304,576 bytes and reclaimed all four. Hardware spacing
peaked at one buffer / 5,624 bytes and reclaimed it. Rail-material editing
remained at zero deferred buffers. There was no accumulating pending count or
observed growth across repeated updates.

## Extended validation

| Scenario | Result |
|---|---|
| Transition drag + camera orbit + repeated resize | 99.46 FPS, 10.54/32.22 ms p95/p99, 10 viewport resizes, four swapchain recreations |
| Live playback + camera + hardware-spacing edit + resize | 99.39 FPS, 12.97/31.07 ms p95/p99, four swapchain recreations; hardware publication fence 0.000 ms |
| Modern Steel transition drag | Final authored value accepted at frame 89; all 30 changed frames published |
| Support-bearing playback + camera + resize | 99.29 FPS, 11.13/27.24 ms p95/p99, four swapchain recreations |
| Debug validation, Modern Steel edit + resize | Completed with deferred-buffer max 5 / 310,200 bytes and no VUID or validation error |
| Debug validation, support-bearing fixture + resize | Completed with no VUID or validation error |

The only Vulkan diagnostic in the Debug logs was the existing loader warning
that the OBS hook advertises Vulkan 1.3 while the application requests 1.4.

## Files and paths changed

- `engine/include/quantum/renderer/VulkanContext.hpp`: deferred-buffer ownership
  state and per-frame retirement telemetry.
- `engine/src/VulkanContext.cpp`: candidate publication, frame-slot retirement,
  fence-completion reclamation, and shutdown/swapchain cleanup.
- `engine/src/Application.cpp`: forwards retirement telemetry to the smoke
  collector.
- `editor/include/quantum/editor/FramePerformanceTelemetry.hpp`,
  `editor/include/quantum/editor/PreviewSmoke.hpp`, and
  `editor/src/PreviewSmoke.cpp`: bounded-resource counters in JSON/TXT reports.
- `tests/PreviewSmokeTests.cpp`: aggregation and report-format coverage for the
  deferred counters.
- `docs/architecture.md`: support-buffer lifetime description updated to match
  deferred publication.

Raw reports and logs are intentionally untracked under
`build/diagnostics/frame-pacing-m1-20260921/`.

## Build and test results

- Release `ALL_BUILD`: passed. An initial parallel build encountered a transient
  vcpkg applocal DLL-copy file lock; the serial retry completed cleanly.
- Release CTest: all 78 enabled tests passed; the existing
  `QuantumEngine.GpuTrainPoseResidency` study remained disabled.
- Debug `QUANTUM`: passed.
- Debug Vulkan validation: active in both fixture runs; no VUID or validation
  error found.
- `git diff --check`: passed before final review.

## Remaining hitch sources

The warmed transition edit still spends about 4.70 ms before simulation,
primarily in centerline/presentation generation, simulation-preview rebuild,
and rider-load evaluation. Live playback still includes the separately known
synchronous GPU preview-sampling/validation round trip. Resize and swapchain
recreation intentionally retain device/fence idle correctness boundaries, and
ordinary FIFO/display scheduling outliers remain outside M1's scope.
