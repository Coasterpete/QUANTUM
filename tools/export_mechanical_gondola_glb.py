"""Blender M5 placeholder: one static carrier mesh, meters, upper-hinge origin."""

import os

import bpy


repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
output_path = os.path.join(
    repo_root, "assets", "mechanical", "hanging-carrier-placeholder.glb"
)
os.makedirs(os.path.dirname(output_path), exist_ok=True)

# Fresh background session only; no authored .blend is opened or saved.
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.context.scene.unit_settings.system = "METRIC"
bpy.context.scene.unit_settings.scale_length = 1.0

material = bpy.data.materials.new(name="Engineering blue")
material.use_nodes = True
shader = material.node_tree.nodes.get("Principled BSDF")
shader.inputs["Base Color"].default_value = (0.015, 0.32, 0.62, 1.0)
shader.inputs["Metallic"].default_value = 0.6
shader.inputs["Roughness"].default_value = 0.4

# Origin (0,0,0) is the hinge. The collider/COM is (0,0,-1).
# The bracket/axle and carrier fit inside the 1.3 x 0.56 x 2 m collider.
parts = []
for name, center, dimensions in (
    ("Carrier", (0.0, 0.0, -1.4), (1.30, 0.56, 1.20)),
    ("Upper bracket", (0.0, 0.0, -0.4), (0.28, 0.32, 0.80)),
    ("Hinge axle", (0.0, 0.0, -0.06), (0.24, 0.44, 0.12)),
    ("Asymmetric front rib", (0.45, -0.265, -1.35), (0.15, 0.03, 0.95)),
):
    bpy.ops.mesh.primitive_cube_add(location=center)
    part = bpy.context.object
    part.name = name
    part.dimensions = dimensions
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bevel = part.modifiers.new(name="Placeholder bevel", type="BEVEL")
    bevel.width = 0.015
    bevel.segments = 1
    bpy.ops.object.modifier_apply(modifier=bevel.name)
    part.data.materials.append(material)
    parts.append(part)

bpy.ops.object.select_all(action="DESELECT")
for part in parts:
    part.select_set(True)
bpy.context.view_layer.objects.active = parts[0]
bpy.ops.object.join()
carrier = bpy.context.object
carrier.name = "QUANTUM_Hanging_Carrier_Placeholder"
triangulate = carrier.modifiers.new(name="Export triangulation", type="TRIANGULATE")
bpy.ops.object.modifier_apply(modifier=triangulate.name)

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
