"""Blender fixture export: one static mesh, meters, pivot-end origin."""

import os

import bpy


repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
output_path = os.path.join(
    repo_root, "assets", "mechanical", "rotating-arm-placeholder.glb"
)
os.makedirs(os.path.dirname(output_path), exist_ok=True)

# Run in a fresh background Blender session; no authored .blend is opened/saved.
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.context.scene.unit_settings.system = "METRIC"
bpy.context.scene.unit_settings.scale_length = 1.0

# The arm extends from X=0 at the hinge to X=4. A distinct pivot collar
# remains inside the unchanged 4 x 0.44 x 0.44 m box collider envelope.
parts = []
for name, center, dimensions in (
    ("Arm", (2.0, 0.0, 0.0), (4.0, 0.30, 0.30)),
    ("Pivot collar", (0.22, 0.0, 0.0), (0.44, 0.44, 0.44)),
    ("Tip", (3.78, 0.0, 0.0), (0.44, 0.38, 0.38)),
):
    bpy.ops.mesh.primitive_cube_add(location=center)
    part = bpy.context.object
    part.name = name
    part.dimensions = dimensions
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bevel = part.modifiers.new(name="Placeholder bevel", type="BEVEL")
    bevel.width = 0.025
    bevel.segments = 1
    bpy.ops.object.modifier_apply(modifier=bevel.name)
    parts.append(part)

bpy.ops.object.select_all(action="DESELECT")
for part in parts:
    part.select_set(True)
bpy.context.view_layer.objects.active = parts[0]
bpy.ops.object.join()
arm = bpy.context.object
arm.name = "QUANTUM_Rotating_Arm_Placeholder"
triangulate = arm.modifiers.new(name="Export triangulation", type="TRIANGULATE")
bpy.ops.object.modifier_apply(modifier=triangulate.name)

material = bpy.data.materials.new(name="Engineering orange")
material.use_nodes = True
shader = material.node_tree.nodes.get("Principled BSDF")
shader.inputs["Base Color"].default_value = (0.65, 0.16, 0.015, 1.0)
shader.inputs["Metallic"].default_value = 0.6
shader.inputs["Roughness"].default_value = 0.4
arm.data.materials.append(material)

bpy.ops.export_scene.gltf(
    filepath=output_path,
    export_format="GLB",
    use_selection=True,
    export_apply=True,
    export_yup=True,
    export_animations=False,
    export_cameras=False,
    export_lights=False,
)
print(f"Exported {output_path}")
