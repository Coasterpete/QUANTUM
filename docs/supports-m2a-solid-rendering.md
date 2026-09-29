# Supports M2A — Solid Timber Rendering

Supports M2A renders the M1 wooden support topology as solid, physically based
timber members. The generator's topology remains authoritative; this milestone
only changes how that topology is presented.

> **This is visual structural representation, not structural engineering.**
> Nothing here performs load analysis, stress or buckling checks, code
> compliance, footing design, or automatic timber sizing. Footing pads are a
> presentation proportion chosen so a footing reads as a footing, nothing more.

## Architecture

```
AuthoredTrack (Core, unchanged M1 topology)
  └─ SupportCollection / SupportStructure / SupportMember / SupportNode
       ├─ SupportAppearance            (new: tint, roughness, normal, scale)
       └─ SupportFoundationAppearance  (new: concrete color, pad size)

buildSupportSolidPresentation()   Core, renderer-neutral
  └─ SupportSolidPresentation { appearance, batches[], foundations[] }
       └─ one SupportSolidBatch per profile shape
            └─ SupportMemberInstance { mat4 transform }

Renderer::updateSupportSolidPresentation()
  └─ VulkanContext
       ├─ one shared instance stream (contiguous, VMA host-visible)
       ├─ one shared unit box mesh + one shared unit cylinder mesh
       ├─ one timber descriptor set (albedo / normal / roughness)
       └─ one instanced draw call per (structure × profile shape)
```

Everything new lives behind the existing seams. No M1 structure was replaced,
and the existing line renderer is untouched.

### Why a new Core module

`SupportSolidGeometry.hpp` sits beside `Supports.hpp` in Core rather than in
the engine, because a member's transform is a pure function of its endpoint
positions and its `SupportMemberProfile`. That makes the geometry testable
without a GPU, and it keeps the renderer responsible only for GPU resources —
the same split `TrackStyle.cpp` already uses for track and hardware geometry.

### What was deliberately *not* added

**No `SupportMemberRole` enum.** §6 of the brief permits deferring it, and
deferring is the right call here. M2A renders every member identically, so a
persisted role field would be schema churn with no consumer: it would need
textual serialization, backward-compatible defaults, and history/equality
coverage, all to store a value nothing reads. Inferring roles from geometric
angle instead was also rejected, because angle is not a reliable proxy for
structural role — a shallow brace and a shallow ledger are geometrically
similar but structurally different.

The generator/render seam is structured so roles can be added later: members
already carry an explicit `SupportMemberProfile`, and the renderer already
groups by profile shape through `SupportSolidBatch`. Adding a role later means
adding a field and a batch grouping, not restructuring the pipeline.

## Unit-mesh and instance strategy

The renderer uploads **two immutable unit meshes**, once, at startup:

| Mesh | Extent | Used for |
| --- | --- | --- |
| Unit box | `[-0.5, 0.5]`³ | rectangular timber, foundation pads |
| Unit cylinder | `x ∈ [-0.5, 0.5]`, radius 0.5 | circular timber (16 radial segments) |

A member's instance transform's basis columns are `axis × extent`, so the
transform carries length, cross-section, and orientation together. The vertex
shader recovers the member's object-space size from the column lengths, which
means **no per-member CPU mesh is ever built** and the instance payload stays a
single `mat4`.

- Rectangular members of one structure: **one** `vkCmdDrawIndexed`.
- Circular members of one structure: **one** more.
- Foundation pads: **one** more, reusing the box mesh.
- A whole generated family (rectangular only, as M1 produces): **one** draw call.

Instances from every structure live in one contiguous host-visible VMA buffer.
A draw batch selects a mesh plus an instance range; no batch owns buffers.

## Member transform strategy

`memberTransform()` places the unit box centred between the two nodes:

```
transform[0] = axisX * length
transform[1] = axisY * width
transform[2] = axisZ * depth
transform[3] = midpoint(start, end)
```

Because the box spans exactly `[start, end]`, a member terminates at its
authored nodes with no floating ends and no gap at a shared node. Members
interpenetrate freely at a node, which §19 of the brief explicitly accepts.

The basis is orthonormal apart from its per-axis extents, so the vertex shader
removes those extents to get a pure rotation. That keeps normals correct for a
post whose length is far larger than its cross-section, which an inverse-
transpose alone would not.

### Rectangular orientation rule

Roll is **not** authored, so it is derived deterministically:

1. Local **+X** is start-to-end, normalized.
2. Local **+Y** is world **+Z** projected off that axis (Gram-Schmidt).
3. Local **+Z** completes a right-handed basis.

If a member's axis is within `supportMemberVerticalTolerance` (cosine
0.9995, about 2° ) of world +Z, world +Z is parallel to its own axis and the
reference collapses, so local +Y falls back to world **+X** instead.

Why this rule:

- **Deterministic.** Same endpoints always give the same frame.
- **No random roll between neighbours.** Two near-parallel posts in the same
  bent agree on which way their cross-section faces, because both project the
  same world reference. A `lookAt` basis would flip unpredictably here.
- **Continuous.** A member rotating gradually through orientations does not
  jump. A test asserts the cross-section axes stay within 0.1° of each other
  across a small axis change.
- **Keeps grain usable.** The cross-section keeps a stable "up" for
  non-vertical members, which is what makes the projection below coherent.

Verified against vertical posts, horizontal ledgers, shallow diagonals, steep
diagonals, and near-vertical diagonals. The rule is exposed as
`resolveSupportMemberFrame()` so tests and future authoring tools agree with
the renderer instead of duplicating it.

### Profile interpretation

| Shape | Width | Depth | Length |
| --- | --- | --- | --- |
| Rectangular | `outerDimensions.x` | `outerDimensions.y` | node-to-node distance |
| Circular | `outerDimensions.x` (diameter) | same | node-to-node distance |

`validateSupportMemberProfile` already requires a circular profile to have
equal outer dimensions, so the diameter applies to both cross-section axes.

`wallThickness` is a M2B hollow-section concern and is **not** applied in M2A;
the unit meshes are solid and only the outer envelope is rendered.

Degenerate members (length at or below `minimumSupportMemberLength`, matching
the M1 generator's own guard) are **skipped** rather than rendered. The
generator already rejects them, so reaching one means externally written or
hand-edited data; skipping keeps a NaN basis out of the instance buffer
without failing the whole presentation.

## Timber texture set

The M2A material samples three maps, all from the user-authored pine set at
`assets/materials/timber/pine/`:

| Map | Encoding | sRGB upload | Used |
| --- | --- | --- | --- |
| `pinewood_basecolor_neutral.png` | neutral grayscale albedo | yes | yes |
| `pinewood_normal.png` | tangent-space normal | no | yes |
| `pinewood_roughness.png` | roughness detail | no | yes |
| `pinewood_ao.png` | ambient occlusion | — | **no** |
| `pinewood_height.png` | height source | — | **no** |
| `pinewood_edge.png` | edge source | — | **no** |

The files on disk are named `pinewood_height.png` and `pinewood_edge.png`
rather than the `_source` names in the brief. They are user files and are not
renamed; they are unused either way.

### Neutral, tintable base color

`pinewood_basecolor_neutral.png` is **fully grayscale** — verified: every
sampled texel has R = G = B, mean 176/255. A regression test asserts this, so
a future asset that bakes a hue into the map fails CI rather than silently
making the tint non-authoritative.

The shader therefore computes:

```glsl
vec3 base = srgbToLinear(draw.baseColor.rgb) * texture(albedoMap, surfaceUv).rgb;
```

final base color = authored sRGB tint × neutral wood detail

The tint is a document value applied in the shader, so it is non-destructive,
participates in Undo/Redo, survives save/load, and never modifies the PNG. The
tint is **independent of `TimberSupportFamily`**: Hybrid Timber Lattice does
not imply brown, and Modern Twister Timber does not imply tan. Geometry and
appearance are separate concepts, and the documentation screenshots show the
same Hybrid structure in two different tints to make that concrete.

Default tint is a conservative natural pine, `sRGB(0.78, 0.64, 0.47)`.

### Normal-map convention — verified, not assumed

Materialize/GIMP can export either OpenGL (+Y up) or DirectX (+Y down) normal
maps, and getting this wrong inverts every bump. Rather than assume, the
convention was measured from the authored data: correlating the map's green
channel against the numerical vertical gradient of its own height source
gives

```
corr(G − 128, dh/dv) = −0.415   over 47,816 samples
```

Negative means green increases as height decreases — the **OpenGL / +Y-up**
convention. That is the same handedness `ground.frag` already assumed for its
world-aligned basis, so the sample maps directly onto +B with **no inversion**.
A DirectX-encoded map would require negating `normalSample.y`. The result is
recorded in the shader comment.

### Roughness — why the map is remapped

QUANTUM's `ground.frag` treats a roughness map as a **straight multiplier** on
the authored scalar. The supplied pine map cannot be used that way:

- its median is ≈ 0.31 (measured), and
- ≈ 13% of texels are near-black.

A straight multiply drives those texels to near-mirror roughness, which reads
as wet, varnished wood rather than untreated timber. M2A therefore treats the
map as a **detail signal remapped into a band around 1.0** and multiplied by
the authored roughness:

```glsl
float detailBand = mix(0.55, 1.45, roughnessMap.r);
float roughness  = clamp(0.85 * authoredMultiplier * detailBand, 0.045, 1.0);
```

This keeps the grain-correlated variation that makes the surface read as
timber, keeps the material matte, and keeps the authored Roughness Multiplier
control meaningful. Metallic is a hard zero — untreated timber is a dielectric
with F0 = 0.04.

## Physical texture scale

§11 of the brief calls out the failure mode where one copy of the wood texture
is stretched from one endpoint of a member to the other, producing enormous
knots and meter-wide grain. M2A avoids it structurally rather than by tuning:

- The unit mesh has **no UV attribute at all**.
- The vertex shader computes the member's object-space position in **Core
  units** (`inPosition * extent`, where `extent` is recovered from the instance
  transform), then derives UVs from that.
- UVs are divided by the authored `textureScale`, which is expressed in Core
  units per repeat.

So a 12-unit post and a 1-unit brace show the **same grain size**; the long
member simply contains more repeats. Default `textureScale = 1.0` Core unit per
repeat, adjustable from 0.1 to 8.0 in the Support Appearance panel.

## Wood grain direction

Grain runs along the member's longitudinal axis. The shader projects per face:

- **Side faces** (normal on a cross-section axis): U from object **+X** (the
  length), V from whichever cross-section axis lies in the face.
- **End caps** (normal on ±X): projected across the cross-section instead,
  which reads as end grain without a dedicated end-grain texture.

Because the tangent basis is recovered per face from the instance transform,
this stays coherent on all six box faces and on the cylinder's side wall.
Exact end-grain texturing is not implemented, as §12 permits.

Known limitation: the side-wall projection is per-face rather than a single
projection across all faces, so the seam between two adjacent faces on the
same member shows a UV discontinuity. At authored scale this is not visible in
any of the documentation screenshots.

## Support appearance data model

Appearance lives on the **structure**, in two optional fields:

```cpp
struct SupportStructure {
    ...
    std::optional<SupportAppearance> appearance;
    std::optional<SupportFoundationAppearance> foundationAppearance;
};
```

They are optional so a document written before M2A loads unchanged and resolves
to the conservative default — the same pattern M1 used for
`generatedWoodenRun`. Persistence:

- Serialized only when authored, so an untouched M1 document keeps its byte
  layout.
- Textual and structural field names; no numeric codes.
- Partial blocks load deterministically: every scalar falls back to its default.
- `SupportStructure`'s defaulted `operator==` covers the new fields, so
  document equality — which is what Undo/Redo compares — works unchanged.
- Regeneration carries both fields forward, because regeneration replaces
  topology, not material.

Undo/Redo participates through the existing `AuthoredTrackEditTransaction` +
`DocumentHistory` path, including drag coalescing for continuous slider edits.

## Foundation rendering

Each `SupportNode` carrying a `Foundation` produces one concrete pad. The
existing node position stays authoritative.

- **Shape:** world-aligned box, centred on the foundation node.
- **Footprint:** `3.0 ×` the largest cross-section dimension of the members
  that node carries.
- **Thickness:** `1.5 ×` that same cross-section dimension.

Thickness deliberately comes from the **carried cross-section**, not from the
pad footprint. Scaling thickness by the footprint made a footing as thick as it
is wide, which rendered as a square pillar stub; that was caught in the
viewport and corrected.

Pads are centred rather than hung entirely below their node, because M1 places
foundation nodes exactly on the foundation elevation plane — a pad below its
node would be buried under the ground surface and never readable.

Presentation is neutral PBR-ish concrete: neutral grey tint, high roughness,
zero metallic. An authored `SupportFoundationAppearance` can override color,
roughness, and pad dimensions; a zero dimension means "derive from the carried
cross-section", which is the default.

No excavation, no terrain following, no footing sizing from loads.

## Debug lines and solid display

Two independent checkboxes in the Support Appearance panel:

| Solid | Lines | Use |
| --- | --- | --- |
| ON | OFF | presentation |
| ON | ON | technical inspection over solid timber |
| OFF | ON | legacy / debugging |

They are **viewport presentation, not document state**, so they deliberately
bypass the transaction and the Undo history. `VulkanContext::setSupportDisplay()`
gates the two draw blocks independently.

The line overlay's buffer and publication path are preserved. When solids are
also visible, its centerlines use a depth-free line pipeline so they remain
readable through the timber; lines-only mode retains the usual depth-tested
pipeline.

## Performance observations

Measured in a successful four-second Release `--dev-preview-smoke
--support-performance --frame-trace` run at 1920×1080 with 4× MSAA.
The display flags change at frames 60, 160, and 280. Dense fixture:
273 members + 34 footings (307 solid instances).

| Display window | Frames | Renderer draw CPU (avg) | p95 |
| --- | --- | --- | --- |
| Solid timber only | 100 | 8.343 ms | 9.027 ms |
| Solid timber and lines | 120 | 8.351 ms | 9.120 ms |
| Lines only | 123 | 8.499 ms | 12.138 ms |

The playing-preview run averaged 100.46 FPS (9.954 ms/frame), with a
52.65 FPS one-percent low and two frames above 16.667 ms. The three draw-CPU
windows are close enough that this run does not isolate a reliable per-support
cost. Track, terrain, train, presentation, and frame pacing are all included.

Deliberately avoided:

- no per-timber CPU mesh,
- no per-timber draw call,
- no per-frame geometry recreation,
- no GPU resource churn on steady state.

Geometry is rebuilt and re-uploaded **only when support publication changes**
(every accepted support edit, document load, undo/redo, or regeneration) —
exactly the existing retained-upload pattern the track mesh and track hardware
already use.

I did **not** verify a tens-of-thousands-of-members structure; the dense
fixture above is 273 members, and §22's "thousands to tens of thousands"
regime is untested.

## Documentation screenshots

All images are real captures from the Windows Vulkan editor. The nine feature
views use the `--capture-screenshots` manifest in
`build/m2a-shots/codex-verify.json`; the two technical views use the same close-up
scenario with different display flags.

### Four structural families

| Family | Image |
| --- | --- |
| Traditional Timber Bent | ![Traditional timber bent](images/supports-m2a/supports-traditional.png) |
| Modern Twister Timber | ![Modern twister timber](images/supports-m2a/supports-twister.png) |
| Prefabricated Steel-Core Timber | ![Prefabricated steel-core timber](images/supports-m2a/supports-prefabricated.png) |
| Hybrid Timber Lattice | ![Hybrid timber lattice](images/supports-m2a/supports-hybrid.png) |

### Timber detail, height, and footings

| View | Image |
| --- | --- |
| Close-up (grain, miter-free joints, member cross-sections) | ![Timber close-up](images/supports-m2a/supports-closeup.png) |
| Tall structure (story framing, no random roll) | ![Tall structure](images/supports-m2a/supports-tall.png) |
| Foundations (one concrete pad per foundation node) | ![Foundations](images/supports-m2a/supports-foundations.png) |

The foundation capture uses a lower track profile so the pads are large enough
to read; the capture camera frames the centerline plus support nodes, so a tall
coaster would show its footings only as a thin line at the frame edge.

### Technical overlay

| Display | Image |
| --- | --- |
| Solid timber and debug lines | ![Solid timber with technical centerlines](images/supports-m2a/supports-overlay.png) |
| Debug lines only | ![Technical support centerlines](images/supports-m2a/supports-lines-only.png) |

### Tint is independent of family

Same Hybrid Timber Lattice structure, same framing, two authored tints:

| Tint | Image |
| --- | --- |
| Natural pine `sRGB(0.78, 0.64, 0.47)` | ![Pine tint](images/supports-m2a/supports-tint-pine.png) |
| Weathered gray `sRGB(0.50, 0.51, 0.50)` | ![Weathered tint](images/supports-m2a/supports-tint-weathered.png) |

## Resource lifetime

Follows the project's existing rules rather than inventing a new scheme.

- **Unit meshes:** uploaded once, immutable, retained for the context's
  lifetime. An MSAA switch rebuilds the pipelines but keeps the meshes.
- **Timber textures:** uploaded once at startup into a dedicated descriptor
  set, published candidate-then-drain, never re-created.
- **Instance stream:** rebuilt only on support publication change, using the
  existing `createHostVisibleBuffer` + `reserveDeferredBufferRetirements` +
  `deferBufferRetirement` pattern, so a rejected publication leaves the
  currently displayed presentation untouched.
- **Failure cleanup:** `createSupportSolidResources()` is all-or-nothing and
  calls `destroySupportSolidResources()` on any throw; the texture upload
  releases only what it created.
- **Device limit:** the shared solid push-constant block is 144 bytes, above
  Vulkan's guaranteed 128-byte minimum, so pipeline creation reads
  `maxPushConstantsSize` and fails with a named reason rather than creating an
  invalid layout.
- **Shutdown order:** solid-support textures, meshes, and instance stream are
  released before `destroyEnvironmentResources()` (its pipeline layouts
  reference the environment descriptor layout) and before
  `vmaDestroyAllocator`, since they hold VMA allocations.
- **Document switching:** loading another document republishes through the same
  path; stale structures are not retained.

Verified by the verification pass: generate, regenerate, delete a structure,
open another document, and shutdown all complete without validation errors or
GPU leaks in the log.

## Known limitations

1. **No end grain.** End caps use a cross-section projection, not a dedicated
   end-grain treatment.
2. **UV seam between adjacent side faces** of the same member (§12 permits a
   shared projection limitation; not visible at authored scale).
3. **No AO.** `pinewood_ao.png` ships unused because QUANTUM's current PBR
   path has no AO input. Adding one purely for this milestone would mean
   redesigning the shared material system, which §15 forbids. No AO is baked
   into the base color.
4. **No height displacement or edge wear.** `pinewood_height.png` and
   `pinewood_edge.png` are reserved for future parallax, micro-displacement,
   weathering, and edge-wear work. Structural silhouettes stay clean.
5. **Hollow sections render as solid.** `wallThickness` is ignored; it is a
   M2B concern.
6. **Circular-member seams** are not hidden by UVs; a round post shows a
   longitudinal texture seam.
7. **No member roles** — see the architecture section for why.
8. **No carpentry.** No mortise-and-tenon, lap joints, bolts, straps, plates,
   or saddles. Members overlap at shared nodes, which §19 accepts.
9. **Footing pads are sub-pixel** when the camera frames a whole coaster. This
   is a framing consequence, not a rendering fault: the pads draw, and the
   foundation capture frames a low track so they read clearly.
10. **Scale not stress-tested** at tens of thousands of members (§22).
11. **Foundation pads are a presentation proportion**, not an engineering
    result.

## Out of scope for M2A

Terrain, terraforming, tiled world maps, Gaea/Blender import, ground layers,
biomes, sky replacement, dynamic sky, weather, structural load analysis, FEA,
automatic timber sizing, real-world joints, bolts, steel connector plates,
catwalks, stairs, handrails, maintenance platforms, steel support generators,
new support families, a PCG graph editor, scenery collision avoidance, and
station exclusion are all explicitly out of scope. No sky, sky shader, HDRI
loading, IBL filtering, irradiance generation, reflection environment, exposure,
or sky asset was modified — verified by inspecting the diff for
`engine/shaders/sky.*`, `EnvironmentMap.cpp`, `EnvironmentVulkan.cpp`, and
`EnvironmentAssets.cpp`, all of which are untouched.

## Tests

| Suite | Coverage |
| --- | --- |
| `QuantumCore.SupportSolidGeometry` | rectangular transform vs endpoints and profile, length, width/depth, vertical/horizontal/diagonal orientation, right-handedness, determinism, no random roll, continuity, degenerate skipping, circular profiles, draw-call count, foundations, default appearance, family independence, regeneration preserving appearance, save/load round trip, old-document defaults, validation rejection, all four M1 families, curved/banked stability |
| `QuantumEngine.SupportSolidRenderer` | unit box closedness, per-face winding, shared corners, unit cylinder sizing and tessellation, mesh determinism, timber identifier grammar, asset path resolution, bundled base color is grayscale |

All existing M1 support-generator tests are preserved unchanged; no M1 topology
expectation was loosened.

## M1 topology impact

One change to `generateWoodenSupportRun`: on regeneration it now carries
`appearance` and `foundationAppearance` forward from the replaced structure.
This is required by §9 ("appearance survives regeneration where expected") and
is not a topology change — node positions, member counts, bent spacing, story
generation, foundations, track attachments, diagonal selection, and
longitudinal ties are all byte-identical. A test asserts regeneration preserves
node count, member count, and every node position exactly.

No other M1 behavior was touched.
