# Train Visual M0: GLB car shells from specialized coaster poses

This records the completed M0 milestone. [Train Visual M1](train-visual-m1.md)
now replaces the runtime repeated prototype with an ordered per-car visual
consist, reusing this GLB as the middle shell. M0's transform, interpolation,
lifecycle and repeated-geometry regression checks remain in the test suite.

M0 makes a single repeated placeholder car shell the normal train body visual.
Each solved car has its own world matrix. It adds presentation only: QUANTUM's
specialized track-constrained solver remains authoritative for the train.
There are no Jolt train bodies or bogies, no new interpolation, no solver changes,
and no serialized visual configuration. Jolt continues to govern only the
separate opt-in mechanical proof.

## Pose and publication flow

```text
existing 1/240 s specialized step -> committed TrainPose
existing preview interpolation -> renderPose_
  -> diagnostic body bounds, bogie markers, connector endpoints
  -> TrainVisualPrototype + one StaticMeshInstance per car, lead to rear
Application: train instances + active M5 mechanism instances
  -> one updateDynamicMeshInstances(collection)
  -> shared static geometry + independent frame-slot transform payloads
```

`SimulationPreview::rebuildVertices()` updates the mesh values from `renderPose_`
before preparing optional diagnostic lines. Both streams share
`vertexGeneration()`, including changes between accepted physics ticks. Play
uses the existing adjacent-tick station interpolation and exact rigid-car solve;
Pause freezes that presentation; Reset restores the initial pose; rebuild
replaces the consist; an unavailable preview clears both streams. Rendering
does not request another solve. Signed backward velocity does not change facing;
an explicitly decreasing-station physical pose already contains the appropriate
body quaternion.

`CarPose` owns its solved body position, quaternion and physical frame. Its body
origin is distinct from its loaded COG, bogie references and hitches.
`TrainVisualPrototype` is an Editor value with a logical asset identifier and
local asset matrix. Core physics contains no mesh identifiers. The renderer
receives only copied identifiers/matrices and never sees `TrainPose` or bogies.

## Authoring contract

- Logical identifier: `assets://train/placeholder-car-shell.glb`.
- Runtime/source GLB: `assets/train/placeholder-car-shell.glb`.
- Reproducible source: `tools/export_train_placeholder_car_glb.py`.
- Meters; local +X forward, +Y lateral, +Z up; origin at physical body origin.
- One mesh/node, three untextured PBR-factor primitives, indexed triangles and
  normals. No hierarchy, armature, animation, textures, seats, passengers,
  wheels, bogie models or production manufacturer artwork.
- Open teal tub, dark compartment floor, raised rear wall and tapered +X nose
  with an orange centerline marker. This is explicitly temporary artwork.
- Bounds match the current default preview body: X +/-2 m, Y +/-0.675 m,
  Z +/-0.7 m. The default dry COG is `(0,0,0.55)` and is deliberately **not**
  the asset origin. The loaded COG likewise does not define the visual pivot.

Run in a fresh Blender background session from the repository root:

```text
blender --background --factory-startup --python tools/export_train_placeholder_car_glb.py
```

The script opens/saves no `.blend` file and touches no icon assets. Blender's
ordinary +Y-up glTF export and the existing loader rotation
`(glTF.x,-glTF.z,glTF.y)` remain the only asset-coordinate conversion.
CMake stages the GLB beside the editor and renderer regression executable and
installs it through the existing Runtime component.

## Transform composition and C++ reading notes

```text
bodyWorld = translate(car.bodyWorldPositionMeters())
          * mat4_cast(car.bodyOrientation())
visualWorld = scale(coordinateUnitsPerMeter)
            * bodyWorld * prototype.localAssetTransform
```

GLM uses column vectors. A local asset point first receives the explicit local
adjustment, then the complete physical body rotation/translation, then conversion
from SI meters to the document's rendering units. Composition uses doubles;
the final renderer matrix uses floats. The M0 asset adjustment is identity.
The uniform document-unit conversion applies to translation and geometry together.
There is no automatic scaling from `bodyDimensionsMeters`: production art should
be authored for its matching vehicle definition.

`updateTrainMeshInstances()` resizes a vector to the render-pose car count and
assigns each car's identifier/matrix in lead-to-rear order. An unchanged count
reuses the vector and string storage. `meshInstances()` returns a read-only
`std::span`: a borrowed view into preview-owned values, valid only until the next
preview update/reset/rebuild. Application copies the values before changing the
preview or publishing. Its combined vector owns its copies; appending two M5
values cannot replace the four car values. No renderer value owns a physics
object or a GPU allocation.

Successful identifiers resolve through the existing `StaticMeshAssetCache`
to `shared_ptr<const StaticMeshAsset>`. The const asset is immutable and shared.
`StaticMeshGpuHandleCache` / `uploadStaticMeshOnce()` retain one uploaded geometry
handle for that identity. Four cars therefore share one CPU mesh and one GPU
mesh while the existing parallel transform vector contains four matrices.
The draw loop uses each collection index as `firstInstance` through the existing
hardware shader. Motion updates matrices rather than geometry.

Vulkan continues to own GPU allocations. Each frame slot writes/grows its
existing instance buffer only after that slot's fence completes. No train code
creates buffers, adds device waits, drains fences, or changes frames in flight.
Shutdown frees the existing slot buffers/mesh geometry before allocator/device
destruction.

## Diagnostics and asset failure

Simulator's **Show train physics diagnostics** checkbox is off by default.
It shows the existing car-body wire bounds, bogie markers and orange connector
lines; it changes neither poses nor physical state. The preview itself retains
its historical diagnostics-on default for diagnostic/test callers; Application
selects visibility from the UI or asset failure.

Missing/invalid shell identifiers log an explicit ASSET error and produce a
renderer load status requesting fallback. Application then forces the full
train diagnostics on, even if the checkbox is unchecked. Simulation remains
usable, and healthy mechanical meshes still draw. The generic renderer retains
failed identifiers in a small failure-status map for its lifetime, so repeated
cars and collection shrink/disable/re-enable reuse the outcome without repeated
file reads/logs. This is a status cache, not a second geometry/GPU cache.
Successful assets continue using the existing caches. Repairing a failed train
asset currently requires restarting the renderer/application; there is no new
train asset hot-reload control in M0.

## Tests and verification

`QuantumEditor.TrainMeshPresentation` covers body origin versus COG, quaternion
axes, local transform order, document scale, independent ordered cars, retained
storage, all imported vertices against the physical point transform, straight,
yawed, pitched, rolled and reverse-facing tracks, interpolated banked curves,
backward signed speed without facing reversal, an empty pose, existing preview
interpolation between ticks, diagnostics visibility, Play/Pause/Reset, changed
car count, unavailable/recovered preview and train/mechanism value composition.
Position/vertex comparison tolerance is 1e-5 in the tested world units, allowing
the existing double-to-float renderer conversion. Physics tolerances are unchanged.
Repeated CPU cache loads share one immutable pointer; a GPU-cache callback runs
once. That callback check is separate from actual Vulkan upload observation.

`QuantumEditor.TrainVisualRenderer` uses a hidden SDL/Vulkan window and the actual
preview and M5 mechanism. Twenty cycles alternate four shells, six combined
instances, two mechanism instances, empty collection and six instances again.
It exercises buffer growth/shrink, motion, pause, reset, unavailable preview,
missing/invalid repeated shells, diagnostic fallback and healthy siblings.
Log assertions require one train upload, two mechanism uploads, one error per
failed identity and zero Vulkan error logs. It verifies the current one-frame
policy, not a different frames-in-flight configuration or leak instrumentation.

Windows x64 MSVC Debug verification on 2026-10-05:

- Configure: `cmake --preset windows-msvc-debug` passed with the existing
  `VCPKG_ROOT`, preset, triplet and dependency baseline.
- Full build: `cmake --build --preset windows-msvc-debug --parallel 2` passed;
  a final full incremental build passed after correcting the new rollback test
  fixture. Existing `[[nodiscard]]` warnings in untouched tests remain.
- Initial focused CTest selection: 17/18 passed in 77.04 s. The one failure was
  a new test attempting a negative authored initial speed, which the existing
  document validation correctly rejects. The fixture now uses a valid uphill
  start that rolls back through normal preview dynamics. Its final CTest rerun
  passed 1/1 in 0.75 s. No existing assertion or behavior was weakened.
- Transform/alignment test: **504 vertices / 696 indices**, all imported
  vertices aligned against the physical point-transform oracle in the tested
  poses; one immutable CPU cache entry and one GPU-cache upload callback.
- Actual Vulkan regression: **154 frames**, **20 collection cycles**, **one
  train-shell upload plus two mechanical uploads**, **two expected asset error
  logs** (one missing identity and one invalid identity) and **zero Vulkan error
  logs**. Four failed shells reuse each failure across disable/re-enable, and
  the train continues stepping with diagnostics while the mechanism draws.
- Runtime install: `cmake --install build --config Debug --prefix
  build/runtime-check --component Runtime` passed. Source, staged editor,
  staged Vulkan test and installed GLBs have identical SHA-256 hashes:
  `DCA566EA0F91EF1F99C5A69603B690B062C871F31112AF075DD1E7D5E53EAF0B`.
  GLB size is **20252 bytes**, one mesh/node, three material primitives and no
  animation, skins or textures.
- Application mode-cycle smoke: `smoke-tests/preview-transition.quantum`,
  `--duration 6 --repeat --simulator --mode-cycle`: **PASSED**, exit 0,
  **6.008 s**, **602 frames**, **218 accepted fixed ticks**, eight scripted
  enter/play/pause/resume/reset/return actions. The application logged exactly
  one shell upload. This smoke leaves the mechanism disabled; simultaneous
  train/mechanism GPU submission is covered by the separate Vulkan test.
  No new missing-asset or Vulkan error occurred; the existing OBS hook
  API-version warning was present.
- `git diff --check`: passed.
- Full enabled CTest suite: **103/103 passed**, zero failures, **590.21 s** on
  the final build. This includes SimulationPreview/interpolation, train/multi-car,
  car poses, Track Devices, static/dynamic meshes, M4/M5, both new M0 tests,
  viewport presentation and Track/Train workspace regressions. The existing
  `QuantumEngine.GpuTrainPoseResidency` test remains disabled (104 registered,
  103 enabled); no test was disabled or weakened by this milestone.

No manual UI clicking or visual artwork approval was performed. The short
application smoke verifies startup, publication and scripted controls; it is
not a claim that a human inspected four shells, their appearance, the checkbox
or the combined mechanism scene. **Final visual approval remains with the
maintainer.** Release, Linux, other GPUs, sanitizers/leak instrumentation and
other frames-in-flight policies were not verified.

Re-run the full suite with:

```powershell
ctest --test-dir build -C Debug --output-on-failure --parallel 2
```

Ignored verification evidence is in `build/train-visual-m0-configure.log`,
`train-visual-m0-final-build.log`, `train-visual-m0-focused-tests.log`,
`train-visual-m0-transform-ctest.log`, `train-visual-m0-transform-test.log`,
`train-visual-m0-vulkan-test.log`, `train-visual-m0-all-tests.log`,
`train-visual-m0-install.log`, `train-visual-m0-asset-hashes.json`,
`train-visual-m0-asset-metadata.json`, `train-visual-m0-smoke.log` and
`train-visual-m0-mode-cycle.json` / `.txt`.

## Small reading path

1. `TrainMeshPresentation.hpp`: the repeated prototype's identifier/local matrix.
2. `updateTrainMeshInstances()`: full physical transform, units and vector order.
3. `SimulationPreview::rebuildVertices()`: one render pose drives both streams.
4. `SimulationPreview::setPhysicsDiagnosticsVisible()` / `setUnavailable()`:
   visibility, generation and clearing without changing dynamics.
5. Application's preview publication block: assign train values, append M5
   values, publish once, then select diagnostic fallback.
6. `StaticMeshAssetCache::load()` / `StaticMeshGpuHandleCache::getOrUpload()`:
   immutable identity reuse and renderer ownership.
7. `VulkanContext::updateDynamicMeshInstances()` / `updateDynamicMeshFrameBuffer()`:
   retained failures and the existing fence-safe transform buffer.
8. `export_train_placeholder_car_glb.py`: body-origin authoring and forward nose.
9. `TrainMeshPresentationTests.cpp` / `TrainVisualRendererTests.cpp`: mathematical
   alignment, presentation lifecycle and actual GPU coexistence evidence.

Understand values versus borrowed spans, iteration, matrix order and shared
immutable asset identity now. GLB accessor encoding, shader/PBR mathematics,
Vulkan allocation internals and solver mathematics can wait.

## Limits and next milestone

This supports one known default-car shell repeated across the current consist.
It is not an authored visual identity or final train model. M1 can replace the
repeated prototype with per-car prototypes while retaining the same instance
type and publication path. It must separately address heterogeneous authored
vehicle art/configuration. M0 adds none of those systems. Final visual approval
belongs to the maintainer; automated alignment does not approve artwork quality.

## PR handoff

Suggested title: **Train Visual M0: render GLB car shells from coaster render poses**

Source branch: `rendering/train-visual-m0`.
Base branch: `main` at `32fa9be` (the M5 merge).

Ready-to-paste description:

```markdown
Render one repeated placeholder GLB car shell per solved train car from the
existing interpolated render pose. Each car has an independent body-origin
transform; repeated shells share the existing immutable mesh and GPU caches.
Application combines train and active M5 mechanism instances before publishing
one renderer-neutral collection, so neither physics domain erases the other.

Add an optional train physics overlay and automatic diagnostic fallback for
missing/invalid assets. Retain failed dynamic asset identities across collection
changes to avoid repeated I/O and logging. Specialized train physics, Track
Devices, interpolation, document formats and fence-owned buffers are preserved;
Jolt is not involved in train transforms.

The Blender-authored placeholder is a 4 x 1.35 x 1.4 m open tub with a raised
rear wall and asymmetric forward nose, one mesh/node and untextured PBR factors.
It is temporary art, not a production train. Runtime staging/install is included.

Validation: Windows/MSVC Debug configure and full build; full enabled CTest
suite 103/103 passed (590.21 s). The M0 Vulkan test ran 154 frames with one
train-shell upload shared by four cars, two mechanism uploads and zero Vulkan
errors. Pose/vertex alignment, render-pose interpolation, reverse facing and
rollback, lifecycle, missing/invalid fallback and simultaneous train/M5 draws
passed. Six-second mode-cycle smoke, Runtime install/hash check and
git diff --check passed. One pre-existing experimental GPU test is disabled.
See docs/train-visual-m0.md for verification details and limits.
Final visual approval remains with the maintainer.
```

Exact changed/added files:

```text
assets/train/placeholder-car-shell.glb
docs/architecture.md
docs/train-visual-m0.md
editor/CMakeLists.txt
editor/include/quantum/editor/EditorUi.hpp
editor/include/quantum/editor/SimulationPreview.hpp
editor/include/quantum/editor/TrainMeshPresentation.hpp
editor/src/EditorUi.cpp
editor/src/SimulationPreview.cpp
editor/src/TrainMeshPresentation.cpp
engine/include/quantum/renderer/VulkanContext.hpp
engine/src/Application.cpp
engine/src/VulkanContext.cpp
tests/CMakeLists.txt
tests/TrainMeshPresentationTests.cpp
tests/TrainVisualRendererTests.cpp
tools/export_train_placeholder_car_glb.py
```
