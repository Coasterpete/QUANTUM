# Blender → QUANTUM track components

This contract applies to numbered families and their temporary blockouts.
The owner can replace one slot at a time; track and train definitions stay independent.

1. Select a numbered type in **Coaster Setup → Track configuration**. Expand
   **Components, dimensions and materials** and identify the slot and placement.
   Required/optional slot identity does not prevent hiding a component.
2. Author metric geometry with **1 Blender unit = 1 metre**. Use **+X forward**,
   **+Y lateral**, **+Z up**. The local track origin is the construction centerline,
   not the heartline or car origin. For slot-local assets put the origin at the slot
   anchor; QUANTUM applies its saved offset exactly once.
3. A **continuous** tile spans local X **0 to tile length**, inclusive. End cross
   sections must match neighboring copies. Add longitudinal subdivisions suitable
   for curvature/banking; a long rigid cube is insufficient. Development examples
   use 1 m tiles. Triangles clip at interval/route ends and follow sampled frames;
   QUANTUM does not automatically remesh a coarse asset.
4. A **repeated rigid** component uses local X forward and an anchor-centered
   origin. Slot spacing and phase are independent of mesh bounds. A **junction**
   component anchors at a construction boundary; specialized shapes still need
   appropriate owner-authored definitions.
5. Static node hierarchy, TRS and affine matrix transforms are supported, including
   nonuniform/mirrored scale. Export only intended mesh objects, with normals and
   UV0. Cameras, animation, skins, morph targets and unsupported extensions reject.
6. Export triangle geometry as **glTF Binary (.glb)** with normals. Blender's normal
   +Y-up glTF export is correct: QUANTUM converts glTF `(x,y,z)` to `(x,-z,y)` after
   composing scene-node transforms. Do not apply an extra manual axis conversion.
7. Export into source `assets/track/components/your-file.glb`, then build QUANTUM
   to stage it; or export directly under the executable's `assets/track/components/`
   directory. Assign **assets://track/components/your-file.glb**. Absolute filesystem
   paths do not belong in documents. Carry this relative directory with a project
   when moving machines and deploy it beside that machine's executable. Automatic
   project-folder discovery and an asset browser are not implemented yet.
8. Set **Tile length** for continuous assets, **Asset scale** (normally 1/1/1), and
   the slot offset. Click **Apply appearance changes**. Inspect a straight, banked
   curve, inversion and interval boundary. X bounds outside `[0, tile length]` use
   procedural fallback with a diagnostic and retain the asset reference.
9. **Use imported GLB materials** retains primitive base-color/metallic/roughness
   factors. Disable it to apply slot factors. Multiple primitives preserve material
   divisions. **Track textures and normal maps are not rendered yet**: a textured
   asset reports unsupported and uses blockout fallback. Retained UVs are metadata.
10. Re-export and click **Reload assigned GLBs**. This invalidates the CPU cache and
    GPU asset through the existing safe reload path. A missing file can recover
    without changing its reference. **Return to blockout** clears only that slot.
11. Save/reopen the `.quantum` document and test Undo/Redo. Interval component and
    material overrides are independent of base appearance and are also saved.

`assets/track/components/example-spine.glb` demonstrates a +X 0–1 m tile.
`example-tie.glb` demonstrates an anchor-centered repeated component. Both are
QUANTUM-owned procedural test boxes, not final manufacturer meshes. Regenerate with
`python tools/track/create_component_fixtures.py`. The hierarchy fixture under
`tests/fixtures/track-families/` exercises parent translation, nonuniform scale,
mirrored nodes, materials and UVs. No obscure Blender object names are required.
