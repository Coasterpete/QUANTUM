# Rigid Body M5: articulated mechanical assembly

M5 extends the opt-in Simulator engineering proof with a passive hanging carrier.
Three bodies, two hinges and two independent GLBs demonstrate a mechanically
connected assembly. This is temporary engineering/CGI geometry, with no authored
ride object, seats, passengers, sequencing, new workspace or train mesh work.

## Assembly and coordinates

Coordinates remain right handed, X/Y horizontal and Z up. Dimensions are meters,
mass is kilograms, angular velocity is radians/second and torque is N m.

| Part | Initial center | Half extents | Mass |
| --- | --- | --- | --- |
| Fixed support | `(0,-11.35,6)` | `(0.4,0.3,0.4)` | 0 (static) |
| Driven arm | `(2,-12,6)` | `(2,0.22,0.22)` | 4 |
| Passive carrier | `(4,-12,5)` | `(0.65,0.28,1)` | 1 |

The existing unlimited world hinge connects the arm to static world at
`(0,-12,6)`, about +Y. Its existing +1.5 rad/s, 200 N m velocity motor remains.
The second unlimited hinge has `body = carrier` and `connectedBody = arm`,
with world-space anchor `(4,-12,6)` and axis +Y at creation. Jolt converts the
common anchor into the two bodies' local attachment points. Its motor stays Off.
It constrains attachment translation and two relative rotations, leaving the
carrier free to rotate about Y under gravity, moving-pivot acceleration and inertia.

The support is a fixed visual mounting reference; the primary hinge connects to
world rather than the support body. There is no floor in this isolated world.

`RigidBodyWorldSettings::dynamicBodyCollisions` defaults to true, preserving
M0-M4 collision behavior in all ordinary worlds. This isolated proof opts out:
its only dynamic bodies are the two connected parts, whose coarse box colliders
overlap around the joint. The existing Jolt object-layer table simply omits
dynamic/dynamic collision in this world. Static/dynamic collision remains enabled.
Both centers of mass lie in the hinge plane at Y=-12. No global filter, pair-filter
framework or solver setting is changed. If unrelated dynamic objects are added
to this world later, this policy must be reconsidered: it excludes all dynamic
pairs in the opted-out world, not an arbitrary collection of named hinge pairs.

## Assets, origins and center of mass

The existing `assets://mechanical/rotating-arm-placeholder.glb` is unchanged.
The new `assets://mechanical/hanging-carrier-placeholder.glb` is a blue rectangular
carrier, upper bracket, hinge axle and asymmetric front rib. It has one mesh/node,
indexed triangles, normals and untextured PBR factors. It has no animation or
hierarchy. `tools/export_mechanical_gondola_glb.py` reproduces it in a fresh
background Blender session without opening/saving any authored `.blend` file.
Only GLBs are staged beside the executable and installed by the Runtime component.

Re-export with Blender 4.5 from the repository root:

```text
blender --background --factory-startup --python tools/export_mechanical_gondola_glb.py
```

Both assets use meters, +Z-up authoring, Blender's glTF +Y-up export and the
unchanged loader conversion `(glTF.x,-glTF.z,glTF.y)`. Their origins are visual
hinges, whereas the box body origins/centers of mass remain at their box centers.

| Binding | Local visual translation | Local physical attachments |
| --- | --- | --- |
| Arm | `(-2,0,0)` | world pivot `(-2,0,0)`; distal hinge `(2,0,0)` |
| Carrier | `(0,0,1)` | upper hinge `(0,0,1)` |

The carrier mesh, bracket and axle fit its box collider. **Show physics/debug
bounds** displays gray support, orange
arm box and cyan carrier box. The GLB origin `(0,0,0)` maps to the upper hinge,
and its collider-local origin `(0,0,-1)` maps to the physics center. A convenient
asset pivot does not move the center of mass or redefine the box inertia.

## Independent pose publication

The before/after control flow differs only in the number of bindings/instances:

```text
accepted preview tick
  -> specialized coaster train state/pose commit (unchanged)
  -> RigidBodyWorld::stepFixed() (unchanged 1/240 s)
  -> arm and carrier bodyState() independently
  -> iterate two bindings -> two StaticMeshInstance values
  -> renderer collection -> cached meshes at their own world transforms
```

`RigidBodyMeshBinding::instance()` remains the M4 implementation:

```text
bodyWorld = translate(body.position) * mat4_cast(body.orientation)
visualWorld = bodyWorld * binding.localAssetTransform
```

GLM uses column vectors, so the local adjustment acts first, then body rotation
and translation. Composition uses doubles; the final renderer matrix uses floats.
The carrier never uses the arm's visual matrix. The hinge supplies the mechanical
relationship, and the renderer displays the resulting poses without physics handles.

The proof uses `std::array` for its known three bodies and two bindings.
`meshBindings()` exposes a read-only `std::span`, borrowing the proof's storage;
`meshInstances()` returns an owning array of copied strings/matrices. Application
publishes it when tick/lifecycle changes and retains the existing train diagnostics.
Asset failure requests only the affected body's wire bounds; a healthy sibling
still renders. The checkbox displays both colliders on request.

## Renderer collection and caching

`Renderer::updateDynamicMeshInstances(span)` replaces the active collection.
The M4 `updateDynamicMeshInstance(optional)` convenience call forwards one or zero
instances, retaining existing callers and coverage. Asset status can be queried
by requested identifier; the no-argument query retains the first-instance behavior.

Vulkan stores a small vector of mesh handles/load outcomes and a parallel vector
of existing `HardwareInstance` payloads. Matching indices connect each geometry
handle to its transform. Pose updates reuse vector capacity and retain the current
slot's load result, including failures. Successful assets use the existing
normalized-identity CPU cache and `uploadStaticMeshOnce()` GPU cache. Two instances
of the same GLB share geometry; two different GLBs each upload once.

Each frame slot owns one mapped instance buffer and its byte capacity. Only after
that slot's fence completes does the renderer overwrite its transforms or grow
the buffer. Growth creates the replacement before releasing the old allocation;
other frames own other buffers. Shrink/disable retains capacity for reuse. Each
mesh draw uses its collection index as Vulkan `firstInstance`, selecting that
part's transform through the unchanged hardware shader. A failed mesh is skipped
without shifting later transform indices. There is no new importer, shader,
material system, GPU mesh ownership model or rendering architecture.

## Ownership, reset and teardown

Application's `unique_ptr<RigidBodyMechanismProof>` owns the proof. The proof owns
its isolated world; the world owns bodies/hinges/motor state. Bindings borrow body
handles and own no physics/GPU resource. They are declared after the world and
therefore die before it in reverse member destruction. Renderer values can outlive
the proof because they contain only copied strings/matrices.

Reset detaches SimulationPreview's borrowed pointer, destroys the old proof,
creates a fresh world with both bodies/bindings, then reattaches. This avoids
exhausting the existing non-reused handle slots. Disable/Editor return detaches
and destroys the proof, publishes an empty instance collection, preserves train
diagnostics and restores the preceding camera. **Frame proof** accounts for the
carrier's additional sweep; the existing camera save/restore mechanism remains.
Train Reset still stops/resets the train without resetting the separate proof.

Unchanged `removeBody()` removes every attached constraint before destroying the
body. Removing the arm thus removes both hinges; removing the carrier removes
only its hinge. Remaining bodies remain usable, and removed handles are rejected.
World destruction releases constraints before bodies. Renderer shutdown completes
GPU work and destroys frame-slot buffers/geometry before the allocator/device.

## Verification

Windows x64 MSVC Debug checks on 2026-10-05:

- `cmake --preset windows-msvc-debug`: passed using the existing vcpkg checkout,
  preset, triplet and dependency baseline.
- `cmake --build --preset windows-msvc-debug --parallel 2`: full build passed,
  including Core, Engine, editor, shaders and all test executables. Existing
  `[[nodiscard]]` warnings remain in untouched tests.
- Focused CTest expression
  `RigidBody|DynamicMeshInstance|StaticMeshAssets|Renderer|SimulationPreview|ViewportPresentation|ViewportCamera`:
  **12/12 passed**, zero failures, **30.60 s** on the final build, including M0-M5.
- The 30 s mechanical proof compares two complete trajectories within 1e-6.
  Maximum primary hinge drift: **0.000410777 m (0.411 mm)**. Maximum secondary
  attachment separation: **0.00127137 m (1.271 mm)**. Both remain below the
  unchanged 2 mm visible attachment bound without solver tuning.
- Unwrapped carrier/arm relative-motion range: **27.4026 rad**. Carrier world-angle
  range: **21.7917 rad**. Peak angular-speed difference: **7.56586 rad/s**.
  The ranges unwrap per-tick angle increments; they include multiple revolutions,
  not a bounded pendulum amplitude. Primary axis error: **4.68557e-10**.
- The mechanism/caching/lifecycle scenarios passed: 140 fresh-world resets,
  20 alternating body removals, independent per-vertex transforms, removed-handle
  rejection, pause/resume and safe preview detachment. Default dynamic/dynamic
  collision still separates overlapping boxes; the opt-out excludes that contact
  and preserves contact with a static floor.
- Runtime install to `build/rigid-body-m5-runtime-check` passed. Source, staged
  editor, staged test and installed carrier GLBs have matching SHA-256 hashes.
  The new asset is **14220 bytes**, **372 vertices / 528 triangle indices**.
- Mode-cycle smoke on `smoke-tests/preview-transition.quantum`, `--duration 6
  --repeat --simulator --mode-cycle`: **PASSED**, exit 0, **6.008 s**, **600 frames**,
  **223 accepted fixed ticks**, eight scripted actions on the final build. This smoke keeps the
  mechanism disabled and verifies the existing application mode/preview path.

- Full enabled CTest suite: **101/101 passed**, zero failures, **639.76 s**.
  The existing `QuantumEngine.GpuTrainPoseResidency` test remains disabled.
  This run preceded the final empty-identifier failure-cache guard; the full
  Debug build and all 12 affected tests were rerun after that guard.
- Actual Vulkan dynamic-instance test: **355 frames**, 20 single-instance and
  20 collection cycles; **two immutable mechanical uploads total**. Missing,
  invalid and empty identities each logged their failure once across repeated
  pose updates; there were zero Vulkan error log messages. Buffer growth,
  shrink, disable/re-enable and shared-arm geometry were exercised. The current
  renderer policy uses one frame in flight; this does not claim manual testing
  of a different frames-in-flight policy or leak instrumentation.
- Windows Debug live checks confirmed fixed support, driven arm, independently
  rotating carrier, mesh/bounds alignment and coherent attachments; Pause froze
  both meshes and Resume continued both. Two Reset proof clicks restored the
  initial assembly at tick zero. Disable removed it and restored the preceding
  camera; re-enable, Frame proof and Editor return worked. The Track inspector
  and Train workspace remained available; ordinary train Reset/Play displayed
  all four cars and advancing preview motion. The temporary unsaved test track
  was discarded and the editor closed normally. Test consoles/antivirus scans
  interrupted the first UI pass; checks resumed after the suite completed.
  No antivirus settings were changed. This was a bounded visual check, not a
  final assessment of placeholder artwork quality.
- The live application's log recorded **one arm upload and one carrier upload**
  across motion, pause/resume, repeated reset and disable/re-enable. No Vulkan
  error was logged; the existing OBS hook API-version warning was present.

Commands use the existing Windows preset and per-process `VCPKG_ROOT` pointing
to the already installed checkout; no project dependency configuration changed:

```powershell
cmake --preset windows-msvc-debug
cmake --build --preset windows-msvc-debug --parallel 2
ctest --test-dir build -C Debug --output-on-failure --parallel 2 -R 'RigidBody|DynamicMeshInstance|StaticMeshAssets|Renderer|SimulationPreview|ViewportPresentation|ViewportCamera'
ctest --test-dir build -C Debug --output-on-failure --parallel 2
cmake --install build --config Debug --prefix build/rigid-body-m5-runtime-check --component Runtime
build/editor/Debug/QUANTUM.exe --dev-preview-smoke smoke-tests/preview-transition.quantum --duration 6 --repeat --simulator --mode-cycle --output build/rigid-body-m5-mode-cycle-final
git diff --check
```

Local verification logs (ignored build evidence, not production assets):
`rigid-body-m5-configure.log`, `rigid-body-m5-release-ready-build.log`,
`rigid-body-m5-release-ready-focused-tests.log`, `rigid-body-m5-all-tests.log`,
`rigid-body-m5-mechanical-proof.log`, `rigid-body-m5-release-ready-install.log`,
`rigid-body-m5-release-ready-smoke.log` and `rigid-body-m5-live-errors.log`.

The new `QuantumEditor.RigidBodyArticulatedMechanism` regression compares two
7200-tick / 30-second trajectories, checks every imported vertex against its own
body state, measures both attachments and relative/world carrier motion, checks
debug colors, exercises preview pause/resume, 140 fresh-world resets and 20
alternating arm/carrier removals. Same-executable repeatability tolerance is
1e-6; float-rendered vertex tolerance is 3e-6 m. The attachment bound is the
existing M3/M4 2 mm visible pivot bound, now applied to both hinges.

The expanded Vulkan regression retains M4 scenarios and adds 20 collection
enable/remove cycles with 1/2/3/2 transforms, shared-arm geometry and a failed
first slot beside a healthy second slot. It exercises actual upload/draw/fence
paths and checks log upload/error counts. Empty asset identifiers also retain
their explicit failure across pose updates. Cache callbacks in physics tests
exercise identity reuse; they are not themselves measurements of Vulkan uploads.

## Reading path and limits

Inspect these small sections in order:

1. `RigidBodyMechanismProof.hpp`: fixed arrays, binding and copied pose values.
2. Proof constructor: carrier creation, local offset, two-body hinge with no motor.
3. `RigidBodyMeshBinding::instance()` / `meshInstances()`: independent state queries.
4. `export_mechanical_gondola_glb.py`: hinge origin, carrier center and collider envelope.
5. `Renderer.hpp`: collection input with no physics ownership.
6. Vulkan `updateDynamicMeshInstances()` / `updateDynamicMeshFrameBuffer()`:
   cached handles, transforms, capacity and fence ownership.
7. Vulkan dynamic draw loop in `recordDrawCommands()`: `firstInstance` indexing.
8. Application proof-control/publication blocks: pointer detachment and iteration.
9. `EditorUiSimulationTelemetry.cpp`: bounds/framing and unchanged controls.
10. `RigidBodyArticulatedMechanismTests.cpp` / `DynamicMeshInstanceTests.cpp`:
    physical independence, attachments, isolated collision policy, lifecycle and
    real GPU cache coverage. The world settings and layer-table conditional in
    `RigidBodyWorld` are also small enough to inspect directly.

Understand values versus borrowed spans/handles, array/vector iteration, local
matrix composition, RAII and destruction order now. GLB accessor details, shading
math and Jolt's solver internals can wait.

The assets/colliders are placeholders. Playback still depends on an available
train preview and stops at open track ends. Poses represent the current accepted
tick without interpolation. This tiny mechanism does not establish arbitrary
assembly collision policy or large-scene performance. Release, Linux, other
GPUs/older CPUs, sanitizers/leak instrumentation and cross-platform determinism
remain outside this verification. Train Visual M0 is left for a separate milestone.
