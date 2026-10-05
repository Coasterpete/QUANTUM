# Train Visual M1: ordered per-car visual prototypes

M1 renders an ordered heterogeneous **visual** consist from the existing
specialized coaster render pose. It removes M0's assumption that every car
uses the same visual prototype. The four-car development preview uses a blue
lead shell, two copies of the teal M0 shell, and a red rear shell.

These are temporary geometry/orientation proofs, not production artwork.
A lead visual is not a physical zero car; a rear visual is not a physical
trailer. **Final artwork approval remains with the maintainer.**

## Physical and visual consists

The physical consist remains `TrainDefinition`: ordered `TrainCarDefinition`
values with masses, bogies, hitches, inertia and loadout. The current preview
still resolves the same repeated backend car definition through
`createDefaultTrainConfiguration()` / `resolveTrainConfiguration()`.

The visual consist is an Editor value:

```cpp
struct TrainVisualConsist
{
    std::vector<TrainVisualPrototype> cars;
};
```

Each prototype owns an asset identifier and a double-precision local matrix.
Entry i describes render-pose car i. The collection accepts arbitrary ordered
identities and adjustments; lead/middle/rear are temporary default choices,
not fundamental renderer slots or Core roles. No rendering fields were added
to `TrainDefinition`, `TrainCarDefinition`, `CarPose` or `TrainPose`.

## Temporary policy and count contract

`makeDefaultTrainVisualConsist(N)` is isolated in `TrainMeshPresentation.cpp`:

| Car count | Ordered default |
|---|---|
| 0 | Empty collection (helper/test case; no physical preview) |
| 1 | Lead |
| 2 | Lead, rear |
| 3+ | Lead, middle repeated N-2 times, rear |

`SimulationPreview::rebuild()` regenerates these values from the resolved
physical count, before Reset prepares presentation. Count changes replace the
whole visual list, with no stale entries. Unavailability clears both the visual
list and instances; recovery regenerates the current count. Play/Pause/Reset
only change poses and playback through the existing paths.

`updateTrainMeshInstances()` requires prototype count == render-pose car count.
A mismatch throws `std::invalid_argument` with both counts before resizing or
writing output. Previous output values remain intact. There is no truncation,
implicit repeated-last entry or arbitrary shell substitution. Preview rebuild's
existing exception handling reports failure and clears presentation if it
cannot establish the invariant.

## Assets and origins

| Entry | Identifier | Source GLB |
|---|---|---|
| Lead | `assets://train/placeholder-lead-car-shell.glb` | `assets/train/placeholder-lead-car-shell.glb` |
| Middle | `assets://train/placeholder-car-shell.glb` | Existing M0 GLB, unchanged |
| Rear | `assets://train/placeholder-rear-car-shell.glb` | `assets/train/placeholder-rear-car-shell.glb` |

All three are authored in meters, +X forward, +Y lateral, +Z up, about the
matching physical body origin (not loaded COG). Each has one mesh/node and
three untextured PBR-factor primitives, with no hierarchy, armature, animation,
skins or textures. The blue lead has a tall narrowing forward nose; the red
rear has a raised transverse -X tail cap and orange tail marker. The existing
teal tub remains the repeated middle; its M0 front marker remains useful for
checking orientation. None models a manufacturer's train design.

The existing Blender exporter now takes a variant argument. It opens/saves no
`.blend` file and writes only the selected GLB:

```text
blender --background --factory-startup --python tools/export_train_placeholder_car_glb.py -- --variant lead
blender --background --factory-startup --python tools/export_train_placeholder_car_glb.py -- --variant rear
```

Omitting `--variant` retains the M0 middle geometry export. M1 does not
regenerate the existing middle binary. Blender's ordinary glTF +Y-up export
and the existing loader rotation `(glTF.x,-glTF.z,glTF.y)` remain the coordinate
conversion. Each prototype still has its own explicit local adjustment for
future art origins; the three default adjustments are identity.

CMake extends the existing editor asset target and Runtime install rules to
all three files, and stages both additions beside the Vulkan regression.
There is no second deployment mechanism.

## Transform and publication flow

```text
existing 1/240 s physical step -> committed TrainPose
existing adjacent-tick interpolation -> renderPose_
visualConsist.cars[i] + renderPose_.cars()[i]
  -> asset identifier + world matrix per car
Application copies train values, appends active M5 arm/carrier values
  -> one updateDynamicMeshInstances(collection)
  -> shared immutable geometry + independent frame-slot matrices
```

For each car, the unchanged M0 composition is:

```text
bodyWorld = translate(car.bodyWorldPositionMeters())
          * mat4_cast(car.bodyOrientation())
visualWorld = documentScale * bodyWorld * prototype[i].localAssetTransform
```

GLM uses column vectors: local adjustment acts first, then body rotation and
translation, then document-unit conversion. Composition uses doubles; renderer
values use floats. Orientation comes only from the interpolated physical pose,
including pitch, roll, reverse facing and signed backward velocity. No tangent,
velocity or bogie reconstruction, new interpolation, Jolt train bodies or solver
changes were introduced. Diagnostic bounds/bogies/connectors consume that same
render pose. Physics tolerances and mathematical conventions are unchanged.

`SimulationPreview` owns the vector of prototype values and output instances.
The helper borrows a `span<const TrainVisualPrototype>` only for its call.
`meshInstances()` still returns a borrowed read-only span valid until the next
preview update/reset/rebuild. Application owns copied values in its combined
vector. These values own neither physics objects nor GPU allocations.

## Cache reuse, coexistence and failure

The existing CPU cache publishes `shared_ptr<const StaticMeshAsset>` by logical
identity. The GPU handle cache uploads that immutable geometry once per identity.
Four default cars therefore need **three** train uploads: lead once, middle
once shared by cars 1 and 2, rear once. Motion updates matrices. Unique lead/rear
identities stay cached through count changes, disabling and re-enabling, Reset,
and unavailable/recovered previews. Vulkan allocation ownership, fence-safe
per-frame buffers, destruction order and the one-frame policy are unchanged.

Application still appends M5 mechanism values after all train values and publishes
once. The renderer knows only identifiers/matrices, with no car role or index
semantics. It retains each failed identity independently and skips only its
unavailable mesh; healthy train and mechanism siblings remain drawable at their
original transform indices.

Application now checks every published train identity. If any fails, it forces
the full train physics overlay on, even when **Show train physics diagnostics**
is unchecked. All cars remain simulated; a failed shell does not remove a car
or its diagnostics. The global overlay is the retained M0 fallback policy.
Without a failure, the checkbox continues to control bounds, bogies and
connectors independently of shell choice.

The existing renderer failure map prevents repeated file I/O/logging for the
same bad identity, including a middle used by several cars. Outcomes persist
across collection changes for renderer lifetime. Repairing a failed file requires
restarting the renderer/application; M1 adds no hot reload.

## Small reading path and C++ concepts

1. `TrainMeshPresentation.hpp`: prototype values and the ordered vector contract.
2. `makeDefaultTrainVisualConsist()`: temporary role assignment via resize/front/back.
3. `updateTrainMeshInstances()`: borrowed span, count validation, index correspondence,
   and per-car matrix multiplication.
4. `SimulationPreview::rebuild()` / `setUnavailable()`: owned visual lifetime/count changes.
5. `SimulationPreview::rebuildVertices()`: the existing render pose drives both streams.
6. Application's train publication/fallback block: copied values, appended M5 values,
   one publication, and any-identity fallback.
7. `StaticMeshAssetCache::load()` / `StaticMeshGpuHandleCache::getOrUpload()`:
   shared immutable identity and one upload (unchanged implementation).
8. `TrainVisualConsistTests.cpp`: mapping/local-transform/count/lifecycle examples.
9. `TrainVisualRendererTests.cpp`: actual Vulkan cache reuse, mixed failures and coexistence.
10. `export_train_placeholder_car_glb.py`: matching origins and simple visual differences.

Understand value ownership, borrowed spans, validation and matrix order now.
Immutable identity explains cache reuse: two entries with the same identifier
share geometry while retaining independent transforms. GLB encoding, Vulkan
allocation details and solver mathematics can wait.

## Limits and later authored visuals

The visual list is a runtime default, not authored train semantics. `.quantum`
schema, physical configuration, Track Devices, stepping, rollback and boundaries
remain unchanged. There is no train-style editor, file chooser, train catalog,
seats, wheels, restraints, riders, skeletal animation or production artwork.
The optional read-only UI summary was omitted to keep this milestone focused.

Future authored visual inputs can replace the default generator while preserving
the index contract and renderer values. Future physically distinct zero/trailer
cars require separately authored physical definitions; those resolved poses can
map to corresponding visual entries without teaching the renderer their roles.
M1 does not implement that next milestone.

## Verification

Windows x64 MSVC Debug checks on 2026-10-05:

- `cmake --preset windows-msvc-debug`: passed with the existing preset,
  `VCPKG_ROOT`, triplet and baseline. CMake reported the existing-cache
  unused `CMAKE_TOOLCHAIN_FILE` CLI-variable warning; dependency resolution
  succeeded without changing build configuration conventions.
- `cmake --build --preset windows-msvc-debug --parallel 2`: full build passed;
  no compiler warnings/errors appeared in this build log.
- Focused CTest selection: **3/3 passed, 10.22 s**: existing
  `QuantumEditor.TrainMeshPresentation`, new `QuantumEditor.TrainVisualConsist`,
  and extended `QuantumEditor.TrainVisualRenderer`.
- M0 checks retain the **504 vertices / 696 indices** repeated-middle oracle,
  one CPU entry and one upload callback, straight/yawed/pitched/rolled/reverse
  alignment, curved interpolated poses, signed backward speed, diagnostic
  visibility, unavailable controls and Play/Pause/Reset.
- M1 checks cover generated counts **0/1/2/4/8**, arbitrary per-car identities,
  independent local adjustments, every imported shell vertex on pitched/rolled
  and reverse-facing poses, and rejection of shorter/longer lists without
  changing output. Position/vertex tolerance remains **1e-5 world units**,
  accommodating double-to-float renderer conversion. No physics tolerance changed.
- Lifecycle checks rebuild **4 -> 2 -> 6 -> 1**, retaining order and existing
  interpolation/play/pause/reset behavior, then fail/recover a preview.
- CPU/GPU-cache callback proof: **three immutable CPU entries and three upload
  callbacks**, one each for lead/shared middle/rear across count and pose changes.
- Actual hidden SDL/Vulkan regression: **209 frames**, **20 collection cycles**,
  **three train uploads plus two mechanism uploads**, **four retained identity
  failures**, and **zero Vulkan error logs**. It first retains M0's four repeated
  middle instances / one upload proof, then uses heterogeneous train defaults.
  Four failures are M0's missing/invalid identities and M1's missing lead/rear;
  the repeated invalid middle intentionally reuses the earlier invalid identity.
  Each logs once through repeated frames, disable/re-enable and changing siblings.
  Healthy train and M5 siblings remain loaded/drawable with unchanged transform
  indices; specialized physics continues ticking. Count changes with M5 active
  and unavailable/recovered preview also pass.
- Full enabled CTest suite: **104/104 passed, zero failures, 665.61 s** using
  `ctest --test-dir build -C Debug --output-on-failure --parallel 2`. This includes
  SimulationPreview/interpolation, train/multi-car physics, Track Devices,
  StaticMeshAssets, dynamic mesh/Vulkan, M4/M5, viewport presentation and
  Track/Train workspace composition/configuration regressions. There are 105
  registered tests; the existing experimental `QuantumEngine.GpuTrainPoseResidency`
  remains disabled. No existing test/assertion was disabled or weakened.
- `cmake --install build --config Debug --prefix build/runtime-check --component
  Runtime`: passed. Source, staged editor, staged Vulkan-test and installed Runtime
  copies have matching SHA-256 hashes for every asset:

| Asset | Bytes | SHA-256 (all four locations) |
|---|---:|---|
| Lead | 20248 | `32F9849E4CB45A26A8AF237C80A6AD03C8110EB5E985E57F78ED4D1390167980` |
| Middle (unchanged M0) | 20252 | `DCA566EA0F91EF1F99C5A69603B690B062C871F31112AF075DD1E7D5E53EAF0B` |
| Rear | 21056 | `8E93E13BC2035EDE9362130D13F7723FCAF2B4D0EFD3E639B2B8FE2C6A4515F3` |

Binary metadata checks confirm one mesh/node, three primitives each, and no
animations, skins, textures or images. Node transforms are identity; decoded
QUANTUM-space bounds match X +/-2 m, Y +/-0.675 m, Z +/-0.7 m within 1e-6 m.
Lead/middle each contain 504 vertices / 696 indices; rear contains 528 / 732.
Hash locations and metadata are retained
in `build/train-visual-m1-asset-checks.json`.

Application mode-cycle smoke with `smoke-tests/preview-transition.quantum`,
`--duration 6 --repeat --simulator --mode-cycle`: **PASSED**, exit 0,
**6.008 s**, **601 frames**, **218 accepted fixed ticks**. All eight scripted
enter/play/pause/resume/reset/return actions completed. Application logged
exactly one upload per train identity, no new asset/Vulkan error, and zero
interpolation solve failures. The existing OBS hook API-version warning was
present. The process was launched once with a 45-second bound and exited normally.
The smoke leaves M5 disabled; simultaneous train/M5 publication is covered by
the actual Vulkan regression above.

`git diff --check` passed. No manual UI clicking, screenshot inspection or
artwork approval was performed. The smoke checks startup/publication/scripted
controls; the renderer harness injects failures and the preview tests check
diagnostic visibility. A human has not inspected the checkbox, individual
shell appearance or mixed mechanism scene. **Final artwork approval remains
with the maintainer.** Release, Linux, other GPUs, sanitizers/leak instrumentation
and other frames-in-flight policies were not verified.

Ignored verification evidence:

```text
build/train-visual-m1-configure.log
build/train-visual-m1-build.log
build/train-visual-m1-focused-tests.log
build/train-visual-m1-focused-details.log
build/train-visual-m1-all-tests.log
build/train-visual-m1-install.log
build/train-visual-m1-asset-checks.json
build/train-visual-m1-smoke-stdout.log
build/train-visual-m1-smoke-stderr.log
build/train-visual-m1-mode-cycle.json
build/train-visual-m1-mode-cycle.txt
```

## PR handoff

Suggested title: **Train Visual M1: render ordered heterogeneous car shells**

Source branch: `rendering/train-visual-m1`.
Base branch: `main`; work starts at merged M0 commit `40e7332` in the normal
`C:\Dev8\QUANTUM` checkout. No worktree or milestone checkout was created.

Exact changed/added file manifest (16 files):

```text
assets/train/placeholder-lead-car-shell.glb
assets/train/placeholder-rear-car-shell.glb
docs/architecture.md
docs/train-visual-m0.md
docs/train-visual-m1.md
editor/CMakeLists.txt
editor/include/quantum/editor/SimulationPreview.hpp
editor/include/quantum/editor/TrainMeshPresentation.hpp
editor/src/SimulationPreview.cpp
editor/src/TrainMeshPresentation.cpp
engine/src/Application.cpp
tests/CMakeLists.txt
tests/TrainMeshPresentationTests.cpp
tests/TrainVisualConsistTests.cpp
tests/TrainVisualRendererTests.cpp
tools/export_train_placeholder_car_glb.py
```

The existing `assets/train/placeholder-car-shell.glb` is reused unchanged and
is therefore absent from the changed-file manifest. No new dependency, authored
visual schema, physical zero/trailer definition or train-style UI is included.

Ready-to-paste description:

```markdown
Replace the preview's single repeated visual prototype with an ordered
Editor-owned per-car visual consist. Entry i supplies the asset identifier and
local adjustment for interpolated render-pose car i; mismatched counts reject
before changing output. Rebuild regenerates exactly N entries for N physical
cars. Temporary defaults are lead for one, lead/rear for two, and
lead/middle(s)/rear for three or more.

Add blue lead and red rear placeholder GLBs through the existing Blender/CMake
asset path; reuse the unchanged teal M0 GLB as the middle. Four cars share three
immutable train meshes, with the middle uploaded once for both middle cars.
Application still combines train and active M5 mechanism values before one
renderer publication. Any failed train identity forces the existing full
physics overlay; healthy train/mechanism siblings remain drawable, physics
continues, and retained failures avoid repeated I/O/logging.

Physical definitions, masses/bogies/hitches/inertia, Track Devices, fixed
1/240 s stepping, interpolation, rollback/boundary behavior, Vulkan ownership
and .quantum schema remain unchanged. Visual lead/rear shells do not create
physical zero/trailer cars; authored visuals and production artwork remain later work.

Validation: Windows/MSVC Debug configure and full build passed. Focused visual
tests 3/3 passed (10.22 s); full enabled CTest suite 104/104 passed (665.61 s).
The actual Vulkan regression ran 209 frames/20 collection cycles with three
train plus two M5 uploads, four retained failed identities and zero Vulkan
errors. Per-car local transforms, count mismatch, reverse/pitched/rolled
alignment, interpolation, 4->2->6->1 rebuilds, Play/Pause/Reset, recovery and
mixed failure/coexistence checks pass. Six-second mode-cycle smoke passed
(601 frames, 218 fixed ticks); Runtime install/four-location asset hashes and
git diff --check passed. One pre-existing experimental GPU test stays disabled.

See docs/train-visual-m1.md for the exact manifest, hashes and reading path.
No manual artwork approval was performed; final visual approval remains with
the maintainer. Release/Linux/other GPUs and leak instrumentation are unverified.
```
