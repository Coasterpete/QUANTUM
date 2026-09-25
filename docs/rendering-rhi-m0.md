# Rendering RHI M0 — Vulkan vertical slice

## Boundary audit

`QuantumCore` owns authored documents, coaster mathematics, material values,
generated rail/spine meshes, and hardware instance descriptions. It has no
graphics API dependency. `QuantumEngine` owns renderer asset loading and the
Vulkan implementation. `VulkanContext` still owns the SDL Vulkan surface,
device, swapchain, viewport targets, VMA buffers, static GLB mesh cache,
shaders/pipelines, command recording, deferred retirement, timestamp queries,
and frame synchronization. `Application` owns lifetime and uploads the Core
products. `EditorUi` owns editor interactions and the Dear ImGui integration;
`SimulationPreview` owns its playback state and produces dynamic line vertices.
Editor and Simulator use one renderer and one viewport target.

`Renderer.hpp` is the first backend-neutral contract for these actual
workloads. The `Application` issues renderer operations through `Renderer&`.
The Editor's camera, sunlight, exposure, viewport aids, track presentation,
hardware status, resize, and MSAA capability use that contract. The Vulkan
ImGui integration still needs native swapchain and image handles, so it remains
an explicit backend-specific edge of M0. The editor installs a Vulkan command
callback once and removes it on shutdown; `Renderer::drawFrame()` itself has no
Vulkan parameter. The resize retirement callback releases ImGui's descriptor
after in-flight work completes and before the old image is destroyed.

No GPU resource handles were introduced at this stage: the existing Core mesh
and instance descriptions and renderer-owned uploads already express the
current ownership boundary. `VulkanContext` remains the sole Vulkan renderer.
Its one-frame-in-flight configuration, VMA allocations, deferred buffer
retirement, GPU timestamps, and shader paths were retained. The interface
uses coarse calls; it adds no per-draw or per-vertex virtual dispatch.

## Validation

The full MSVC Debug and Release builds passed. CTest passed all 78 enabled
tests in each configuration; `QuantumEngine.GpuTrainPoseResidency` was already
disabled. After adding the mode-cycle smoke option, seven focused Debug tests
and the full Release suite passed again. The Windows Debug application, with
Vulkan validation available,
completed a six-second Simulator smoke run with camera orbit, a rail material
edit, window and viewport resizing, and GPU preview validation: 570 frames,
1,414 simulation steps, four swapchain recreations, nine viewport resizes,
and no preview/physics failure. A separate five-second Debug `--mode-cycle`
run used the normal queued playback controls to enter Simulator, play, pause,
resume, reset, return to Editor, and repeat a mode transition. It completed
all eight scripted actions, 495 rendered frames, 218 simulation steps, four
swapchain recreations, and 15 viewport resizes without failure. Its
`--frame-trace` report contains 495 frame records.
The fresh 1600 x 900 Modern Steel 1x and 4x capture PNGs have SHA-256 hashes
identical to `docs/rendering-m1/off/modern-steel.png` and
`docs/rendering-m1/on/modern-steel.png`, respectively. This checks the
existing PBR/lighting/geometry output on those two deterministic views.

Release smoke workloads used `modern-steel-validation.quantum`, camera orbit,
an eight-second duration, and `DISABLE_VULKAN_OBS_CAPTURE=1`. The Editor runs
used stopped playback; Simulator runs used active playback with repeat. Runs
were serial, with no build or test running during measurement. The baseline
binary was the pre-change `feature/frame-pacing-m0` Release executable
(`A546C650...6599B7D7A389E3E3BB8`); the after binary was
`AF252F0A...BCE9A2555FAEB52`. Each run rendered 783–800 frames.

| Workspace | FPS before/after | 1% low FPS before/after | p95 ms before/after | p99 ms before/after | GPU ms before/after | Draw CPU ms before/after | Fence wait ms before/after |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Editor 1x | 99.92 / 97.83 | 50.74 / 29.24 | 11.86 / 10.32 | 13.44 / 23.08 | 0.258 / 0.348 | 8.899 / 8.991 | 8.757 / 8.804 |
| Editor 4x | 99.97 / 99.89 | 50.97 / 50.10 | 10.67 / 10.96 | 13.65 / 13.82 | 0.283 / 0.260 | 9.085 / 8.902 | 8.962 / 8.741 |
| Simulator 1x | 99.96 / 99.93 | 55.50 / 49.59 | 10.65 / 10.99 | 13.20 / 13.83 | 0.139 / 0.103 | 9.217 / 9.140 | 9.096 / 9.000 |
| Simulator 4x | 98.89 / 99.93 | 38.53 / 49.45 | 10.28 / 10.58 | 17.76 / 14.07 | 0.153 / 0.137 | 9.434 / 9.188 | 9.317 / 9.046 |

Draw CPU includes the frame-slot fence wait and presentation work. Subtracting
the average wait gives 0.12–0.14 ms before and 0.14–0.19 ms after, but that
remainder is **not** an isolated measurement of RHI dispatch cost. The 1% low
and p99 numbers vary, especially the first after Editor 1x run. An earlier
post-RHI eight-second matrix on the same Vulkan rendering path measured
57.76 FPS 1% low for Editor 1x and 31.56 for Simulator 4x, illustrating that
these tail results fluctuate by workload run. These short runs do not prove
performance neutrality or consistent 60 FPS frame pacing. The unchanged
Vulkan frame trace and longer frame-pacing investigation remain the basis for
diagnosing occasional stalls.

## Remaining M0 boundary limits

The native Dear ImGui renderer, viewport texture registration, screenshot
capture setup, and experimental GPU physics context still use `VulkanContext`
directly. A future Metal-compatible backend needs its own UI integration,
platform window/surface path, shader compilation/translation, and capability
handling. No macOS build or second backend is implied here. Rendering M2 can
add environment textures and passes when their actual resource and lifetime
contracts are defined; this milestone reserves no speculative cubemap or
shadow API. Automated smoke runs exercised both workspaces, their repeated
transitions, and queued play/pause/resume/reset controls. The corresponding
physical mouse clicks and keyboard gestures were not separately exercised.
