# Numbered track families — integrated pass 4

Date: 2026-10-12. Branch: `codex/track-families-pass4`.
Starting HEAD: `57d83ff967a95466c7d2f7f8a6a663e3c6a15f91`.

This is a substantial working checkpoint, **not completion of every requested
pass-4 capability**. The architecture and blockouts are integrated into the native
application. All geometry remains temporary; final owner-authored assets do not
exist. Texture sampling, specialized junctions and additional asset management
remain unfinished.

## Initial findings and preservation

The active local checkout was the pass-3 branch and commit named in the request.
The existing `TrackStylePreset`, profile sweeps, renderer-neutral `RenderableTrack`,
static CPU/GPU mesh caches, Vulkan instancing, presentation invalidation and
`DocumentHistory` were reused. The old configuration catalog contained only
`modern-steel`; `DualRailTubular` required exactly two running rails. The GLB
loader accepted one mesh and one untransformed node but already supported multiple
primitives and metallic/roughness factors. Timber and steel support generation
already used the authored support graph.

The pre-existing policy changes, audio implementation/hooks/tests, campaign
notes and concrete/fiberglass/grout/metal asset folders were preserved. No physics,
framework, audio or scripting migration was undertaken. Agent commits exclude
those pre-existing changes, including the audio hunks in `Application.cpp`.

## Catalog and reference assignments

Serialized IDs are explicit lowercase names. Display names and references are
metadata. No arithmetic on the three-digit number selects geometry.

| Type | Assigned reference | Working temporary geometry |
|---|---|---|
| Truss-100 | Intamin Intimidator 305 / Project 305 | Four structural chords, two running rails, chord connectors and bay bracing |
| Truss-120 | Intamin Expedition GeForce | Deeper three-chord triangular lattice |
| Truss-200 | Mack Hyper / DC Rivals | Wider multi-member triangular truss |
| Truss-210 | Mack Voltron Nevera | Shared Mack blockout foundation; independent asset identity |
| Truss-300 | Intamin Maverick | Compact three-chord tubular construction |
| Truss-320 | Intamin Batman Gotham City Escape | Larger backbone profile and plate-like connection beams; independent components |
| Box-100 | Classic B&M inverted: Raptor / Montu | Box above running rails with connection members |
| Box-110 | Newer B&M inverted: Banshee | Shared inverted blockout; independently replaceable assets |
| Box-200 | B&M floorless: Kraken / Medusa | Box below running rails with brackets and ties |
| Box-210 | Classic B&M sit-down: Kumba | Shared above-track train construction foundation |
| Box-220 | First-generation B&M stand-up: Mantis | Shared box foundation; no train conversion |
| Box-230 | B&M Surf Coaster: Pipeline | Shared box foundation; independent asset identity |
| Hybrid-100 | I-Box and Topper | Steel web/flanges or steel cap over timber layers, with aligned running strips |
| Wood-100 | Traditional American / PTC-era layered construction | Individual timber layers, side ledgers, running strips and close ties |
| Wood-200 | Intamin prefabricated: El Toro / Balder | Deeper laminated bearing assemblies, side surfaces and different tie spacing |
| Wood-300 | Reserved, unassigned | Excluded from selection; no fabricated construction |
| Monorail-100 | RMC Raptor | One central box rail, running/guide surfaces and attachment plates |
| Monorail-200 | Intamin Hot Racer | Wider running deck, narrower deeper central beam, side guides and saddles |

Banshee and Pipeline are assigned to the requested IDs. Types sharing a
foundation are intentionally **not advertised as uniquely reconstructed track
sections**. Shared blockouts are independently replaceable. All dimensions are
editable development approximations, not measured manufacturer dimensions.

Reference evidence includes manufacturer/park pages: [Project 305 construction](https://www.intamin.com/project/intimidator-305/),
[Expedition GeForce](https://www.intamin.com/project/expedition-geforce/),
[Mack Hyper](https://www.mack-rides.com/products/hyper-coaster-mack-rides),
[Voltron](https://www.mack-rides.com/references/voltron-nevera),
[Maverick](https://www.sixflags.com/cedarpoint/attractions/maverick),
[Gotham City Escape](https://www.intamin.com/project/gothamcityescape/),
[B&M inverted](https://www.bolliger-mabillard.com/inverted-coaster),
[B&M Surf](https://www.bolliger-mabillard.com/surf-coaster),
[RMC Raptor](https://rockymtnconstruction.com/steel-coasters/), and
[Intamin Hot Racer](https://www.intamin.com/product/hot-racer/).
Official Project 305, Expedition GeForce, Gotham, Raptor and Hot Racer photographs
were inspected in the browser. Gotham's manufacturer explicitly identifies its
new backbone design; this prompted a distinct backbone/plate-member definition.
The B&M CDN image request was blocked by browser policy. No exact Banshee/Pipeline
cross-section difference was established, and no fictional difference was added.
The wooden layer definitions remain broad construction approximations requiring
owner review against the eventual Blender models.

## Permanent data and reusable generation

`assets/track/catalog/numbered-families.json` is versioned content embedded into
Core through CMake. Editing it automatically reconfigures the build. The registry
has string family/category identities, explicit type IDs, references, base
relationships and reserved status. Additional JSON definitions are discovered
under the runtime `assets/track/catalog/` directory in filename order. Registration
is atomic per file; duplicate IDs, missing bases, cycles and unsupported providers
are rejected with diagnostics. New families using existing providers require data,
not another central type dispatch branch. There is no catalog-size cap.

A component declares its slot ID, required/optional identity, material slot,
provider, placement, offsets, dimensions, spacing/phase, visibility, GLB reference,
asset scale/tile length and material policy. Providers cover swept tubes/boxes,
round point-to-point members and rectangular connection beams. Placement covers
continuous, repeated and boundary junction components. Dimensional edits are per component: gauge/offset changes require matching member
endpoint and tie-width edits. There is no symbolic constraint solver. Required means the slot
belongs to the definition; it does not force the component to be visible.

Actual mesh generation follows components, including one-running-surface monorails.
`railCount` is only a compatibility/reference-curve input for component styles.
The Engineering overlay retains its existing two diagnostic polylines; a monorail
uses its central reference for both. It does not generate two visible rails.

Already solved rider-local frames supply placement, banking and inversions.
Definitions use metres and convert through `metersPerCoordinateUnit` before
presentation. Core remains independent of SDL/Vulkan/ImGui. The Engine supplies a
thread-local CPU mesh resolver; cached asset data and GPU handles do not enter
saved documents.

Procedural repeated members are batched into contiguous submeshes by component
and material. Custom repeated GLBs use the existing GPU instancing pipeline.
Continuous custom GLBs tile/deform along the solved frame stream and clip triangles
at exact interval endpoints. Local-X tile bounds are validated; an incompatible
re-export falls back without preventing the saved document from opening. The mesh needs
suitable longitudinal subdivisions; this is not a general high-fidelity deforming
modeler. Explicit junction components occur at route/construction starts, rather
than arbitrary mathematical Region splits.

## Asset, material and native editing workflow

Coaster Setup retains the current Workbench and offers construction category,
numbered type, reference, optional variant, component assignments and materials.
Fields fit narrow docked panels. A persistent draft allows several edits before
**Apply appearance changes** creates one accepted document/history change.
**Discard appearance draft**, **Return to blockout**, and **Reload assigned GLBs**
provide reversible editing and Blender iteration. Interval overrides expose the
same component/material editor.

Partial replacement is supported. A spine can use a GLB while rails/braces remain
procedural. Missing, malformed or unsupported assets retain their reference and
use the appropriate procedural component. Diagnostics appear in the component
inspector. A restored file can be reloaded without changing its saved reference.

The existing GLB importer now traverses the active scene, multiple meshes/nodes,
parent-child matrix/TRS transforms and multiple primitives. It transforms normals
with the inverse transpose, reverses mirrored winding, retains UVs/material factors
and reports bounds. Invalid/cyclic/singular hierarchies reject clearly. Transparent,
emissive and extension-based materials are explicitly unsupported. Existing
assets remain supported. No new parser dependency was introduced: the checked
accessor/buffer code was extended for the required static subset.
[cgltf](https://github.com/jkuhlmann/cgltf) was reviewed as an alternative: its
single-header C API, MIT license and scene/material/image metadata would reduce
future parser maintenance. It does not itself implement QUANTUM's image decoding,
texture GPU resources or track material bindings. Replacing the checked static
loader was deferred; no third-party source or new package was imported.

User color, metallic and roughness factors can be assigned per component or interval.
GLB primitive factors are retained when **Use imported GLB materials** is enabled;
disabling it applies the component override. Editor and Simulator consume the same
mesh/material presentation. **Texture sampling and normal maps are not implemented
for track components**. Textured GLBs reject with an explicit diagnostic rather
than silently losing their materials. UV retention is not a claim of texture rendering.

Paths use portable package-relative `assets://track/...` references. Source assets
live under `assets/track/`; builds copy them beside the executable even when no C++
relink occurs, and examples/catalog content are included in installation. Automatic
project-folder asset discovery, file browsing/import and packaging remain unfinished.
See [the Blender workflow](track-component-blender-workflow.md) for exact placement
and deployment instructions.

## Intervals, persistence and history

Hybrid-100 supports I-Box/Topper changes inside one mathematical Region. Stable
64-bit interval IDs, allocator, absolute metre stations, variant references,
component/material snapshots and optional overrides are serialized. Running strips
keep the same offsets across both variants. Boundaries are inserted into the
presentation samples; repeated-member phase is route-global, and braces do not
cross a construction boundary. Out-of-range portions are clipped after shaping;
the authored absolute range remains saved and can return when the track is lengthened.

An additive `trackStyle.construction` payload has independent schema version 1.
Legacy `DualRailTubular`, unassigned documents and `modern-steel` keep their existing
meaning. No manufacturer style is silently substituted. Unknown numbered IDs keep
the saved component snapshots and GLB references. Malformed/future component schemas
reject explicitly rather than discarding fields. Older executables cannot understand
new component-style documents; retain originals when testing across app versions.

Legacy Region dimension/material overrides survive a topology switch and are
explicitly identified as inactive for component styles; Region visibility still
works. Switching back restores the old behavior. Numbered components are edited
through their own inspector. A slot edit regenerates the combined visual mesh; unchanged GLB CPU/GPU assets
remain cached. Fine-grained per-component mesh uploads are not implemented, and
the reload button invalidates assigned GLBs collectively. Appearance edits and reloads use presentation-only
invalidation and avoid recomputing centerline, rider loads or train physics.
Undo/Redo follows the existing `DocumentHistory` publication path.

## Support integration and limits

Family/variant definitions expose track-frame X/Y/Z support attachments. New steel
frames and timber runs add these to their existing authored recipes, with metre/Core
unit conversion. Forward attachment offsets advance the station through the existing
attachment model; lateral/up offsets use that station's rider frame. Manual support graph data is not regenerated by appearance edits.
Generated steel profiles that were manually changed retain their values during
frame regeneration. Existing nodes/members/material editing tools remain in use.

Recipe offsets are additional to the family attachment. For a new numbered family,
set the generator's extra vertical offset to zero to meet the bearing interface;
legacy default offsets remain compatible with legacy documents.

Timber regeneration still replaces generated topology, preserving appearance but
not arbitrary edits/deletions inside that generated run. A full generated/manual/
suppressed-member provenance model is not implemented. Dedicated hybrid junction
hardware, precision endcaps, manufacturer joints and structural certification are
also absent. Abrupt but aligned blockout structural seams are a development limit.
Cross-family intervals are deliberately unavailable without a valid junction contract. At a construction boundary coincident with
a repeat station, a tie/brace may be omitted rather than overlap both variants;
dedicated junction content must complete that detail. Uniform component geometry
is generated across mathematical Region splits, preserving global tie/bracing phase.

## Verification and evidence

Windows MSVC Debug and Release full builds succeeded. Both complete suites passed
**129 enabled CTest tests**; `QuantumEngine.GpuTrainPoseResidency` was already disabled
and remains disabled. After the final presentation/persistence changes, the relevant
**13 focused tests passed in each configuration**, including native train rendering,
centerline presentation, documents/history, assets, supports and the new family test.
The latest Release family run passed approximately **2.84 million per-vertex/triangle
and contract assertions**. This is one CTest case with extensive checks, not millions
of separate user workflows. Transient implementation failures (including the sweep
winding, missing reference labels and an overly strict tessellation comparison) were
corrected; no final enabled test failure is outstanding.

Build evidence: `build/pass4-release-verified-build.log` and
`build/pass4-debug-verified-build.log`. Full-suite evidence:
`build/pass4-release-tests.log`, `build/pass4-debug-tests.log`.
Final targeted evidence: `build/pass4-final-release-tests.log`,
`build/pass4-final-debug-tests.log`; geometry/generation results:
`build/pass4-family-release-results.log`. These local logs are ignored build artifacts.

Native Editor captures cover all 17 assigned types, both hybrid constructions,
custom components, timber supports, mixed mathematical Regions, a banked truss,
an inverted monorail, an inverted custom component and a mixed construction boundary.
Release Simulator runs exercised both monorails, Hybrid-100 and custom components;
a Debug Monorail-200 Simulator smoke also succeeded without reported Vulkan validation
errors. GPU rendering is programmatic evidence, not human acceptance. Native captures
are under `docs/images/track-families-pass4/`.

Hardware observed: Intel i7-14700KF, NVIDIA RTX 4070, Windows, MSVC Release, Vulkan
FIFO presentation. A 2 km Truss-100 CPU mesh generation produced **972,920 vertices
and 14 component draws** in about **39 ms** in the latest run. The same Debug
instrumented test observed about 697 ms earlier. This is generation timing, not
interactive frame rate; there is no directly comparable previous-family baseline
and no claimed performance gain.

The 215 m stopped Editor smoke averaged **99.93 FPS**. An isolated stopped
Monorail-200 Simulator run averaged approximately **100 FPS**, with **1.35 ms** mean
GPU execution and **9.74 ms** frame-fence wait under FIFO pacing. Earlier overlapping
Vulkan test/smoke processes averaged roughly **4–5 FPS**, **1.8–2.1 ms** GPU execution,
and **187–227 ms** main-thread time dominated by frame-slot fence waits. The isolated
run did not sustain that slowdown. Concurrency/visibility may matter; causation was
not established. The intermittent frame-fence problem remains unresolved and the
60 FPS goal is not universally proven. Memory usage was not profiled. The new
`QuantumApplication.TrackFamilies` test exercises all assigned types, extra-family
registration, inheritance failure atomicity, parameter rejection, finite geometry,
unit normals, outward winding, inverted frames, intervals, history, persistence,
unknown catalog IDs, GLB hierarchy/UVs, partial replacements, fallback/recovery,
family-aware support offsets and preservation of changed generated profiles.
The canonical track samples and persisted vehicle/physics fields are compared
before/after visual edits. The fixed 1/240 s solver was not changed.

The test exposed inward triangle winding in the pre-existing common sweep builder.
Its indices were corrected to agree with outward normals. Geometry checks use
1e-6 m frame-center tolerance and 2e-5 normal-length tolerance; interval boundaries
are inserted at authored stations rather than rounded to the solve grid. Presentation
vertices remain the renderer's float representation; Core calculations remain double.

Native capture manifests use the existing SDL3/Vulkan/ImGui runner with saved-layout
writing disabled. Runtime smoke uses the real application, not a disconnected demo.
The user's saved layout was backed up for the interactive smoke and restored.
Programmatic captures are not human manual acceptance. The first native Workbench
view was observed through the computer-use skill; subsequent close-ups were inspected
from saved native captures. No claim is made that every Workbench edit was manually
operated through injected mouse/keyboard input.

## Exact manual acceptance

1. Open `tests/fixtures/track-families/documents/truss-100.quantum`; inspect the
   straight, banked curve and inverted section. Compare Truss-120/200/300/320.
2. In Coaster Setup, select Box Spine and Box-110, then Box-230. Confirm the Banshee
   and Pipeline references, unchanged train configuration, and working Undo/Redo.
3. Open `hybrid-100.quantum`: I-Box -> Topper (70–115 m) -> I-Box. In Construction
   intervals, move a boundary inside a Region, apply, Undo/Redo, save to a new file
   and reopen. Edit an interval's component material and verify only its range changes.
4. Compare `wood-100.quantum`, `wood-200.quantum`, and `wood-supports.quantum`.
   Generate a new support run with extra attachment offset zero; inspect actual
   bearings and preserve manual support edits during style selection.
5. Open both monorail documents. Confirm one central beam, banking/inversions and
   the distinct Raptor/Hot Racer definitions. Enter Simulator; the existing neutral
   train is retained, not converted to specialized monorail rolling stock.
6. Open `custom-components.quantum`. Inspect the spine and repeated tie replacements.
   Return only one slot to blockout, Undo, re-export its GLB with the documented
   origin/units, click Reload assigned GLBs, then save/reopen.
7. Rename an owned test GLB temporarily outside the app. Reload, confirm procedural
   fallback and the retained reference, restore the file and reload again.
8. Resize the viewport and cycle Editor/Simulator; check style/material parity and
   actionable diagnostics. Use `mixed-regions.quantum` and `long-layout.quantum`
   for Region boundaries and the 2 km case.

## Next integrated pass

Prioritize real track PBR texture/normal-map sampling and material batching,
project-relative asset browsing/deployment, precise owner-reviewed component profiles
and transition junctions, and generated-support edit/suppression provenance.
Keep specialized train compatibility and the intermittent frame-fence slowdown
explicitly separate; this pass does not resolve them.

## Source files and checkpoint ownership

- `core/CMakeLists.txt`
- `core/include/quantum/coaster/TrackComponentDiagnostics.hpp`
- `core/include/quantum/coaster/TrackConfiguration.hpp`
- `core/include/quantum/coaster/TrackStyle.hpp`
- `core/src/CoasterDocument.cpp`
- `core/src/SupportFrameGenerator.cpp`
- `core/src/TrackCatalogData.hpp.in`
- `core/src/TrackComponentsJson.cpp`
- `core/src/TrackConfiguration.cpp`
- `core/src/TrackStyle.cpp`
- `core/src/WoodenSupportGenerator.cpp`
- `editor/CMakeLists.txt`
- `editor/include/quantum/editor/CoasterSetupUi.hpp`
- `editor/include/quantum/editor/EditorUi.hpp`
- `editor/include/quantum/editor/TrackStylePresentation.hpp`
- `editor/src/CenterlineVisualization.cpp`
- `editor/src/CoasterSetupUi.cpp`
- `editor/src/EditorUi.cpp`
- `editor/src/ReadmeCaptureRunner.cpp`
- `editor/src/TrackStylePresentation.cpp`
- `engine/include/quantum/renderer/StaticMeshAssets.hpp`
- `engine/src/Application.cpp`
- `engine/src/StaticMeshAssets.cpp`
- `tests/TrackFamilyTests.cpp`
- `tools/track/create_component_fixtures.py`
- `assets/track/catalog/numbered-families.json`
- `assets/track/components/example-spine.glb`
- `assets/track/components/example-tie.glb`
- `docs/architecture.md`
- `docs/static-mesh-assets.md`
- `docs/track-component-blender-workflow.md`
- `docs/track-families-pass4.md`

Also added: the owned `tests/fixtures/track-families/` documents/hierarchy fixture and
`docs/images/track-families-pass4/` native captures. Only pass-owned changes are staged.
The audio hooks in `Application.cpp` are intentionally left in the working tree.
