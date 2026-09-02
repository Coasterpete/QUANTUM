# Static mesh assets

This milestone supports small Blender-authored repeating track-hardware meshes.
It is intentionally not a scene importer, asset browser, or material system.

## Asset identity and runtime paths

Track styles retain renderer-neutral logical strings. Two schemes are accepted:

- `assets://track/...` resolves below `assets/` next to the executable, using
  the same SDL executable-base convention as shaders, fonts, and editor icons.
- `builtin://diagnostic/track-hardware-placeholder` resolves to the retained
  in-code diagnostic cube.

Separators and `.` components are normalized before caching. Absolute paths,
unknown schemes, and paths containing a surviving `..` component are rejected.
The default test hardware is
`assets://track/test-crosstie-placeholder.glb`.

`StaticMeshAssetCache` owns immutable shared CPU assets. One normalized logical
identity is parsed once. `VulkanContext` maps that identity to one opaque GPU
mesh handle, uploads one vertex buffer and two index buffers (triangles and
explicit wireframe edges), and destroys every allocation before destroying its
VMA allocator and Vulkan device. Hardware batches continue to share one
contiguous instance-transform buffer.

If a file-backed hardware asset cannot load, the renderer logs an `ASSET` error
containing the logical identifier and reason, then uses the named builtin
diagnostic mesh for that batch. The substitution is therefore visible and is
not stored in authored track data.

The Editor's compact **Track Hardware** section keeps the requested logical ID
visible and reports `Loaded`, `Missing asset`, `Invalid GLB`, `Unsupported GLB`,
or `Load failed`. Fallback use is shown separately so a failed import never
looks like a successful authored asset and never overwrites the user's ID.

On Windows, **Choose GLB...** uses the existing native dialog seam. A selected
file must be inside the running build/package's `assets/track` directory and is
converted to `assets://track/...` before it enters the document. The manual
**Reload Hardware** action invalidates only that logical asset's CPU/GPU cache
entry, recreates the shared mesh, and preserves all instance-placement values.
There is no filesystem watcher.

## Loader choice and dependency impact

No general-purpose importer was present in the project or vcpkg manifest.
Assimp is a planned/approved library for a later broader model-import milestone,
but its scene model and additional formats are unnecessary here. This milestone
uses a focused in-tree GLB 2.0 container/accessor reader with the project's
existing `nlohmann-json` dependency.

`nlohmann-json` is MIT-licensed and header-only. It already participates in the
static vcpkg build, adds no DLL, and creates no new portable-package runtime
dependency. Only the exported `.glb` is deployed.

## Supported GLB subset

The loader accepts:

- binary glTF 2.0 (`.glb`) with one JSON and one embedded binary chunk;
- exactly one mesh;
- one or more triangle-list mesh primitives;
- float `VEC3` positions;
- float `VEC3` normals;
- explicit scalar indices using unsigned 8-, 16-, or 32-bit components;
- tightly packed or valid interleaved accessors;
- multiple primitives merged into one GPU mesh while retaining CPU submesh
  index ranges;
- glTF material references, UVs, and other unused vertex attributes, which are
  ignored in favor of QUANTUM's existing hardware color/shading path.

Normals are required. Missing, non-finite, or degenerate normals are rejected;
valid normals are normalized deterministically after axis conversion. Positions
must be finite, all indices must be in bounds, and every primitive index count
must be divisible by three.

The loader rejects external buffers, `.gltf`, sparse accessors, non-triangle
primitive modes, non-indexed primitives, multiple meshes, node transforms or
hierarchy, node-based mesh instancing, animations, skins, cameras, morph
targets, and malformed buffer/accessor ranges. Lights, scene behavior, PBR,
textures, samplers, and material graphs are not imported.

## Blender export contract

Author repeating track hardware with these conventions:

1. Use metric units with unit scale `1.0`; one Blender unit is one meter.
2. Model in QUANTUM track-local axes: local `+X` is track-forward, local `+Y`
   follows the track frame's positive lateral axis, and local `+Z` is up.
3. Put the object origin at the intended attachment/placement origin. A
   crosstie will normally be centered laterally and longitudinally, with its
   origin at the center used by the style's `localPosition` adjustment.
4. Apply location, rotation, and scale before export. The exported mesh node
   must not carry a transform.
5. Apply geometry-affecting modifiers. Triangulate deliberately with an applied
   modifier when stable topology matters; Blender export-time triangulation is
   also accepted because QUANTUM requires triangle primitives.
6. Export one selected mesh as glTF Binary (`.glb`) with normals and Blender's
   `+Y Up` glTF option. Do not export cameras, lights, animation, armatures, or
   helper-object hierarchies.

Blender's glTF exporter rotates Blender's `+Z`-up coordinates into glTF's
`+Y`-up convention. The loader performs one inverse proper rotation:

```text
QUANTUM position/normal = (glTF.x, -glTF.z, glTF.y)
```

This preserves meters, vector length, handedness, and triangle winding. No
axis swaps or sign changes occur in shaders or hardware-placement code. The
existing track-frame instance transform supplies straight-track orientation,
pitch, yaw, and banking/roll consistently after this one conversion.

The included `test-crosstie-placeholder.glb` is a Blender-exported path fixture,
not final production artwork. `tools/export_test_crosstie_glb.py` reproduces it;
the portable package installs only the `.glb`, never `.blend` or `.blend1`
files.
