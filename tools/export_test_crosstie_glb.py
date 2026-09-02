"""Export QUANTUM's small Blender-authored static-mesh test fixture."""

import os

import bpy


repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
output_path = os.path.join(
    repo_root, "assets", "track", "test-crosstie-placeholder.glb"
)
os.makedirs(os.path.dirname(output_path), exist_ok=True)

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)

# Test/placeholder hardware only: a meter-authored, lightly beveled crosstie.
bpy.ops.mesh.primitive_cube_add(location=(0.0, 0.0, 0.0))
crosstie = bpy.context.object
crosstie.name = "QUANTUM_Test_Crosstie_Placeholder"
crosstie.dimensions = (0.18, 1.40, 0.12)
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

bevel = crosstie.modifiers.new(name="Test edge bevel", type="BEVEL")
bevel.width = 0.015
bevel.segments = 1
bpy.context.view_layer.objects.active = crosstie
bpy.ops.object.modifier_apply(modifier=bevel.name)

triangulate = crosstie.modifiers.new(name="Export triangulation", type="TRIANGULATE")
bpy.ops.object.modifier_apply(modifier=triangulate.name)

crosstie.select_set(True)
bpy.context.view_layer.objects.active = crosstie
bpy.ops.export_scene.gltf(
    filepath=output_path,
    export_format="GLB",
    use_selection=True,
    export_apply=True,
    export_yup=True,
)

print(f"Exported {output_path}")
