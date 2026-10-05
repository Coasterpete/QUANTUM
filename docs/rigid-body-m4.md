# Rigid Body M4: renderable mechanical asset binding

M4 replaces the M3 arm's primary wire-box appearance with one static GLB mesh
whose world transform comes from `RigidBodyWorld::bodyState()`. This is an
engineering/CGI visualization proof. M3's two bodies, world hinge, motor,
gravity, collider dimensions and accepted 1/240-second preview cadence are
unchanged. There is no independent animation or train/Jolt coupling.

## Asset and authoring contract

The logical asset is `assets://mechanical/rotating-arm-placeholder.glb`, staged
next to the editor under `assets/mechanical/` and included in Runtime installs.
`tools/export_mechanical_arm_glb.py` reproduces it in a fresh background Blender
session; no existing `.blend` file is opened or saved. The GLB is the runtime
asset. The export script is a reproducible placeholder recipe, not production art.

The placeholder has a beveled rectangular shaft, a distinct pivot collar and a
tip. All parts are joined into **one mesh/node**, with indexed triangles,
normals, applied transforms, no hierarchy, no animation and no textures.
Its untextured orange PBR factors use the existing static-mesh material path.
The loader's supported subset is unchanged.

Author in meters with QUANTUM local +X along the arm, +Y along the hinge axis,
and +Z up. Set the origin at the mechanical pivot end. This asset extends from
X=0 to X=4, with Y/Z bounds of ±0.22 m; the narrower shaft sits inside the
unchanged 4 × 0.44 × 0.44 m box collider. The collar is a visual pivot marker,
not an imported collider or a separate joint.

Export with Blender's glTF +Y Up option. The existing loader converts
`(glTF.x, -glTF.z, glTF.y)` once into QUANTUM's right-handed +Z-up convention.
Neither the binding nor shader performs another axis conversion.

## Binding and transform flow

`RigidBodyMeshBinding` lives in the Editor proof header. Its three values are
a non-owning body handle, a logical asset identifier and a double-precision
local asset matrix. It does not own a mesh or a body. `RigidBodyMechanismProof`
owns the binding alongside its existing isolated world.

`RigidBodyMeshBinding::instance(world)` queries `world.bodyState(body)` and
creates a renderer-facing value:

```text
bodyWorldTransform = translate(positionMeters) * mat4_cast(orientation)
worldVisualTransform = bodyWorldTransform * localAssetTransform
```

GLM matrices multiply column vectors, so the local adjustment acts first.
The quaternion already uses GLM `(w,x,y,z)` ordering from the physics boundary.
Composition happens in double precision; only the final renderer matrix is
converted to float. The local matrix can express translation, orientation and
scale; this proof uses identity orientation/scale and translation `(-2,0,0)` m.

The body's origin/center is initially `(2,-12,6)` m. The world hinge is
`(0,-12,6)` m. The asset origin is its pivot end. Translating the asset by −2 m
in body-local X maps the asset's X=0 origin to the hinge and its X=2 center to
the body center. The offset rotates with the body. **Do not move the physics
body to the visual pivot**: that would change the physical attachment/inertia
relationship. A future production mesh must honor this contract or provide an
intentional local adjustment.

After the preview update, Application publishes `armMeshInstance()` whenever
the proof tick/lifecycle changes. Train diagnostics continue through their
existing stream; the gray support retains its M3 wire representation. The
**Show physics/debug bounds** checkbox adds the orange arm collider for comparison.
GLB rendering is independent of the track's wire/shaded presentation setting.

## Ownership, caching and synchronization

Physics still owns bodies, constraints and motor state. Presentation owns the
body/asset relationship. The renderer receives only a copied `StaticMeshInstance`
containing a string and world matrix; no Jolt types or borrowed body pointers
cross that boundary.

`VulkanContext::updateDynamicMeshInstance()` reuses `StaticMeshAssetCache::load()`
and `uploadStaticMeshOnce()`. The existing normalized-identity CPU/GPU caches
share immutable geometry. Its vertices, triangle/edge indices and submesh
materials remain in the existing GPU mesh table until renderer shutdown.
No additional importer, mesh cache, upload system, shader or material system
is introduced.

One moving instance is supported. Each in-flight frame slot owns one persistent
`HardwareInstance` buffer using the existing `hardware.vert` layout. The CPU
pose marks the slots dirty. `drawFrame()` writes a slot only after its fence has
completed, then draws the cached mesh with the existing hardware shaded pipeline.
After each slot's first allocation, motion and proof resets only overwrite its
transform. Removed instances are not drawn; their tiny buffers remain reusable
until shutdown. Shutdown waits for work and destroys them before the VMA allocator
and device, following existing renderer lifetime rules.

Reset detaches the preview before destroying the old proof, creates a fresh
world and binding, then reattaches. Cached geometry is reused. Disable or return
to Editor detaches/destroys the proof, publishes an empty dynamic instance,
preserves train diagnostics and restores the preceding camera. The binding
dies before its world through reverse member destruction. Renderer values can
outlive that binding because they contain no borrowed physics references.

## Failure behavior

A missing/invalid GLB retains its requested identifier, load-state classification
and detailed error. An `ASSET` error is logged and the UI reports that physics
bounds are being used. The dynamic solid draw is suppressed and the existing
orange wire collider remains visible regardless of the checkbox. The proof
world and controls stay usable; fallback never overwrites an authored identifier.
The failed load result is retained during pose updates to avoid repeating I/O
and log output every tick. Disable/re-enable permits another load attempt.
GPU upload failures remain errors rather than being mislabeled as asset failures.

## Verification

Windows x64 MSVC Debug checks on 2026-10-04:

- `cmake --preset windows-msvc-debug`: passed with local `VCPKG_ROOT` pointing
  to the existing checkout. No preset, triplet or dependency baseline changed.
- `cmake --build --preset windows-msvc-debug --parallel 2`: full build passed,
  including Core, Engine, editor, shaders and test executables. Existing
  `[[nodiscard]]` warnings remain in untouched tests; the M4 sources emitted
  no compiler warnings.
- Focused run: `ctest --test-dir build -C Debug --output-on-failure --parallel 2
  -R 'RigidBody|DynamicMeshInstance|StaticMeshAssets|Renderer|SimulationPreview|ViewportPresentation|ViewportCamera'`:
  **11/11 passed**, zero failures, **23.23 s**. This includes M0–M3, both M4
  tests, static assets, support renderer, SimulationPreview and viewport checks.
- The binding regression checks all 288 imported vertices against actual
  `bodyState()` over 2400 ticks / 10 s within 3e-6 m float tolerance. Maximum
  pivot drift was **0.0000707831 m** (0.071 mm), below the 2 mm M3 bound.
  Local scale/rotation/offset composition, foreign/removed handle rejection,
  pause/resume, 140 fresh-world binding resets and safe detachment passed.
- The Vulkan regression uses the real loader/upload/draw path: 20 instance
  enable/remove cycles and 166 frames, **one immutable mesh upload**, explicit
  missing/invalid GLB fallback and no logged Vulkan errors. Fixed test poses
  isolate buffer publication; actual Jolt movement is covered by the binding
  test and live editor observations. This is lifetime/cache regression coverage,
  not memory-leak instrumentation or a performance benchmark.
- After correcting only the renderer test's printed frame count, its affected
  target rebuilt successfully and the final standalone CTest run passed **1/1**
  in **6.64 s**. No production code changed after the full Debug build/live checks.
- `cmake --install build --config Debug --prefix
  build/rigid-body-m4-runtime-check --component Runtime`: passed. Source, staged
  Debug and installed mechanical GLBs had identical SHA-256 hashes. The asset
  is **11264 bytes**, with 288 vertices and 396 triangle indices.
- Application mode-cycle smoke: `--dev-preview-smoke
  smoke-tests/preview-transition.quantum --duration 6 --repeat --simulator
  --mode-cycle --output build/rigid-body-m4-mode-cycle`: **PASSED**, exit 0,
  **6.004 s**, **600 frames**, **223 accepted fixed ticks**, eight scripted
  actions through Simulator entry, play, pause, resume, reset and Editor return.
  This smoke leaves the proof disabled and verifies the existing application path.
- Full suite: `ctest --test-dir build -C Debug --output-on-failure --parallel 2`:
  **100/100 enabled tests passed**, zero failures, exit 0, **804.33 s**.
  The existing `QuantumEngine.GpuTrainPoseResidency` test remains disabled
  (101 tests registered); no test was disabled or weakened for M4.

Live observation used the actual Windows Debug editor through computer-use
screenshots/controls. A new unsaved default track was extended to 6000 m to
avoid an early open-end stop; train resistance still applied normally.
Observed:

- Enable framed the solid orange GLB arm with visible collar and tip, gray
  support, tick 0 and zero initial angular speed. Dimensions and +X starting
  orientation matched the intended contract.
- Debug bounds enclosed the mesh: collar at the pivot end, narrower shaft
  inside the box. During sampled motion they continued to enclose it.
- Play produced distinct poses at ticks **326** and **2124**, with +1.500 rad/s
  and a fixed gray support. The mesh rotated in the expected X/Z plane around
  its pivot end without a visible separation. The production pose path is
  exclusively `bodyState()`; the GLB has no animation tracks.
- Pause held **tick 3699** and the same viewport mesh/bounds pose across later
  observations. Resume advanced to **tick 5391** and a different pose.
- Train Reset stopped playback and preserved the mechanism at **tick 6773**.
  Reset proof restored **tick 0**, zero speed and the initial horizontal mesh.
  Another Reset proof remained safe at that initial pose.
- Disable removed the mesh/support/bounds and restored the preceding track
  camera while retaining train diagnostics. Re-enable recreated the initial
  proof; returning to Editor with it enabled removed it and restored the camera.
- The Track workspace and Train's four-car configuration/preview displayed
  normally after these checks. The live renderer log shows **one** mechanical
  mesh upload across movement, repeated reset, disable and re-enable.

These are sampled live observations, not screenshot comparisons or continuous
frame-pacing measurements. Missing/invalid asset behavior was verified by the
automated renderer regression, not a manual missing-file editor run. Antivirus
scans and test console windows occasionally covered the editor; no security
controls/settings were changed. The preexisting OBS Vulkan layer API-version
warning appeared, with no mechanism-related Vulkan error in the live log.

`git diff --check` and a separate trailing-whitespace check of new text files
passed.

Logs (ignored generated files): `rigid-body-m4-configure.log`,
`rigid-body-m4-build.log`, `rigid-body-m4-final-build.log`,
`rigid-body-m4-focused-tests.log`, `rigid-body-m4-binding-proof.log`,
`rigid-body-m4-renderer-final-build.log`, `rigid-body-m4-renderer-final-test.log`,
`rigid-body-m4-install-check.log`,
`rigid-body-m4-all-tests.log`, `rigid-body-m4-live.log`,
`rigid-body-m4-smoke.log` and `rigid-body-m4-asset-export.log`.
Smoke reports are `build/rigid-body-m4-mode-cycle.json` and `.txt`.

## Limits and reading path

The arm is an explicitly temporary placeholder. Its collider is still the M3
box, not generated from GLB geometry. There is one moving mesh input, no scene
graph, authored mechanism document, hierarchy import, mesh animation, flat-ride
workspace, operations, seats, passengers or gameplay. Playback still depends
on train-preview availability and stops at an open track end. The rendered
pose is the current accepted tick without interpolation.

Release, Linux, sanitizers/leak instrumentation, other GPUs/older CPUs, large
mechanisms and cross-platform determinism were not verified. Production artwork,
arbitrary mesh collider generation and independent mechanical playback remain
outside M4. The live editor remains open with the temporary unsaved test track
in Train workspace and the proof disabled.

Inspect these focused sections:

1. `RigidBodyMechanismProof.hpp`: `RigidBodyMeshBinding` and owner fields.
2. `RigidBodyMechanismProof.cpp`: constructor, `instance()` and `armMeshInstance()`.
3. `export_mechanical_arm_glb.py`: authoring dimensions, origin and export options.
4. `StaticMeshAssets.cpp`: `load()` and `primitiveMaterial()` (unchanged).
5. `VulkanContext.cpp`: `updateDynamicMeshInstance()`, `updateDynamicMeshFrameBuffer()`
   and the dynamic-mesh draw block in `recordDrawCommands()`.
6. `Application.cpp`: the proof-control and preview publication blocks.
7. `EditorUiSimulationTelemetry.cpp`: proof controls, camera and debug-bounds checkbox.
8. `RigidBodyMeshBindingTests.cpp` / `DynamicMeshInstanceTests.cpp`: pose, asset and lifetime regressions.

Understand value structs, a non-owning handle, `std::optional`, matrix composition,
reverse destruction and per-frame fence ownership now. GLB accessor details,
PBR shader mathematics and Jolt's constraint solver internals can wait.
