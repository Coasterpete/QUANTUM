# Runtime stabilization M2: GPU validation gating — 2026-09-21

## Verdict

The synchronous GPU track-sampling dispatch and CPU reference validation that
ran on every playback frame has been gated behind an explicit opt-in flag.
Production playback no longer enters this path unless validation is requested.

This closes the last identified measurement conflation: the frame-pacing audit
reported that the "physics" telemetry bucket included the GPU
sampling/validation round trip (~7.2 ms), not just fixed-step CPU work (~1.6
ms). The gating removes this without changing physics results.

## What changed

### SimulationPreview GPU validation gating

The GPU track-sampling path in `SimulationPreview::update()` is now gated
behind a new `gpuValidationEnabled_` bool (default `false`). When disabled
(the production default), the frame loop skips the synchronous
`GpuPhysicsContext::sampleTrackGpu` dispatch, the CPU reference comparison,
and all associated telemetry accumulation.

Files changed:

- `editor/include/quantum/editor/SimulationPreview.hpp`: added
  `gpuValidationEnabled_` member, `setGpuValidationEnabled()`, and
  `gpuValidationEnabled()` accessors.
- `editor/src/SimulationPreview.cpp`: added the `gpuValidationEnabled_`
  precondition to the GPU sampling block (line 574); added setter/getter
  implementations.

### Application GPU-context setup

Previously, the Application withheld the GPU context from SimulationPreview
when `--disable-gpu-preview-sampling` was passed. The context is now always
attached when available, and validation is separately controlled by the new
flag. This preserves GPU context availability for future non-validation uses.

- `engine/src/Application.cpp`: unconditional GPU context attachment;
  `setGpuValidationEnabled(true)` when `previewSmokeOptions->enableGpuValidation`.

### PreviewSmoke CLI

- `editor/include/quantum/editor/PreviewSmoke.hpp`: added `enableGpuValidation`
  bool to `PreviewSmokeOptions`.
- `editor/src/PreviewSmoke.cpp`: added `--enable-gpu-validation` argument
  parsing.

### Test coverage

- `tests/PreviewSmokeTests.cpp`: added `--enable-gpu-validation` to the
  CLI round-trip assertion.
- `tests/SimulationPreviewTests.cpp`: the GPU preview test now explicitly
  enables validation via `setGpuValidationEnabled(true)` so the guarded path
  is still exercised.

## Design rationale

The GPU sampling path shares `GpuPhysicsContext::graphicsQueue()` with the
Vulkan renderer. The synchronous dispatch submits compute work and blocks on a
fence, which stalls the CPU while the GPU processes both the sampling compute
and the renderer's pending presentation work. In production playback this
provides no functional benefit — it exists solely for diagnostic validation of
the GPU track-sampling implementation.

Making it opt-in rather than removing it entirely preserves the validation
infrastructure for development-time correctness audits.

## Build and test results

- Release build: passed (all targets).
- Release CTest: 78/78 enabled tests passed. The disabled
  `QuantumEngine.GpuTrainPoseResidency` study remains disabled.
- Debug build: passed.

## M2 startup regression (human validation)

### Symptom

Normal interactive editor startup crashed with a missing/unreadable SPIR-V
shader error immediately after the M2 commit.

### Root cause

Two issues in `GpuPhysicsContext`:

1. `createTrainPosePipeline()` threw `std::runtime_error` when the
   `train_pose.comp.spv` shader was not found, rather than degrading
   gracefully. This exception could propagate through `uploadTrainDefinition`
   into paths that were not guarded by the constructor's catch block.

2. `createTrainPosePipeline()` was missing the `SDL_GetBasePath()` candidate
   path that `createComputePipeline` already used, creating an inconsistency
   in shader resolution between the two pipeline-creation paths. This meant
   the train-pose pipeline could fail to locate a shader that was correctly
   placed next to the executable by the CMake POST_BUILD copy step.

Additionally, `readSpirvFile` exceptions from `createComputePipeline` could
propagate through the lambda into the constructor's catch block without
attempting the remaining candidate paths, causing partial initialization and
fragile cleanup of Vulkan resources.

### Fix

- `createTrainPosePipeline()` now returns gracefully (with a log message and
  CPU fallback) instead of throwing when the shader is not found, the file is
  unreadable, or `vkCreateComputePipelines` fails. This matches the
  `createComputePipeline` pattern.

- `createTrainPosePipeline()` now includes the `SDL_GetBasePath() / "shaders"`
  candidate path, consistent with `createComputePipeline`.

- `createComputePipeline` now wraps `readSpirvFile` in a try/catch so that a
  corrupt or unreadable candidate does not abort the search; it logs and
  continues to the next candidate.

- `uploadTrainDefinition()` now checks `trainPosePipelineReady_` after
  `createTrainPosePipeline()` and returns early if the pipeline could not be
  created, instead of proceeding to GPU buffer operations.

### Validation

- Release build: editor startup succeeds, smoke tests pass.
- Debug build: editor startup succeeds, smoke tests pass.
- Added three new regression tests to `SimulationPreviewTests`:
  - `startupGracefullyDegradesWithoutGpuContext`: verifies CPU-only preview
    initialization, playback, and vertex generation.
  - `startupGpuContextConstructionNeverThrows`: verifies the GpuPhysicsContext
    constructor never throws regardless of GPU availability.
  - `startupGpuValidationOptInDoesNotBlockStartup`: verifies validation
    opt-in does not affect normal startup or production playback.

## Remaining work

- A Release workload matrix (idle, playback, camera orbit, transition drag)
  should be re-run to measure the isolated impact on telemetry accuracy and
  any remaining frame-time outliers.
- The long-playback tight-3D-curve solver work explosion (documented in
  `docs/long-playback-performance-audit-2026-09-20.md`) remains as the next
  physics accuracy-preserving optimization target, but was already mitigated by
  the rigid-bogie continuation hints (commit `7b8c177`) and the connector
  secant refinement (already present in `TrainPhysics.cpp:954-964`).
