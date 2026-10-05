"""Train visual proofs: open tubs, physical body origin, meters, +X forward."""

import argparse
import os
import sys

import bpy


repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
parser = argparse.ArgumentParser()
parser.add_argument("--variant", choices=("middle", "lead", "rear"), default="middle")
options = parser.parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
variant = options.variant
filename = "placeholder-car-shell.glb" if variant == "middle" else f"placeholder-{variant}-car-shell.glb"
output_path = os.path.join(repo_root, "assets", "train", filename)
os.makedirs(os.path.dirname(output_path), exist_ok=True)

# Use a fresh background session; never open/save an authored .blend file.
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.context.scene.unit_settings.system = "METRIC"
bpy.context.scene.unit_settings.scale_length = 1.0


def material(name, color, metallic, roughness):
    result = bpy.data.materials.new(name=name)
    result.use_nodes = True
    shader = result.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = color
    shader.inputs["Metallic"].default_value = metallic
    shader.inputs["Roughness"].default_value = roughness
    return result


shell_colors = {
    "middle": (0.02, 0.42, 0.48, 1),
    "lead": (0.08, 0.22, 0.72, 1),
    "rear": (0.55, 0.07, 0.18, 1),
}
shell_material = material(f"Placeholder {variant} shell", shell_colors[variant], 0.25, 0.38)
floor_material = material("Dark compartment", (0.035, 0.055, 0.07, 1), 0.1, 0.6)
marker_material = material("Orientation marker", (1.0, 0.52, 0.05, 1), 0.2, 0.4)

# Default preview body bounds: X +/-2, Y +/-0.675, Z +/-0.7.
# Origin is the physical body origin, not its loaded COG at positive Z.
# The raised rear wall and tapered +X nose make facing easy to recognize.
parts = []
for name, center, dimensions, finish in (
    ("Compartment floor", (-0.35, 0, -0.62), (3.3, 1.35, 0.16), floor_material),
    ("Left tub wall", (-0.3, 0.605, -0.08), (3.4, 0.14, 0.94), shell_material),
    ("Right tub wall", (-0.3, -0.605, -0.08), (3.4, 0.14, 0.94), shell_material),
    ("Raised rear wall", (-1.92, 0, 0.02), (0.16, 1.35, 1.36), shell_material),
    ("Orientation marker",
     (-1.87, 0, 0.65) if variant == "rear" else (1.28, 0, 0.58 if variant == "lead" else 0.25),
     (0.12, 1.0, 0.08) if variant == "rear" else (0.36, 0.12, 0.08), marker_material),
):
    bpy.ops.mesh.primitive_cube_add(location=center)
    part = bpy.context.object
    part.name = name
    part.dimensions = dimensions
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bevel = part.modifiers.new(name="Placeholder edge bevel", type="BEVEL")
    bevel.width = 0.025
    bevel.segments = 1
    bpy.ops.object.modifier_apply(modifier=bevel.name)
    part.data.materials.append(finish)
    parts.append(part)

nose_top = (0.7, 0.25) if variant == "lead" else ((0.1, 0.1) if variant == "rear" else (0.29, -0.15))
nose_half_width = 0.28 if variant == "lead" else 0.48
nose_mesh = bpy.data.meshes.new("Front treatment")
nose_mesh.from_pydata(
    [(1.1, -0.605, -0.54), (2, -nose_half_width, -0.54), (2, nose_half_width, -0.54),
     (1.1, 0.605, -0.54), (1.1, -0.605, nose_top[0]), (2, -nose_half_width, nose_top[1]),
     (2, nose_half_width, nose_top[1]), (1.1, 0.605, nose_top[0])],
    [],
    [(3, 2, 1, 0), (0, 1, 5, 4), (1, 2, 6, 5),
     (2, 3, 7, 6), (3, 0, 4, 7), (4, 5, 6, 7)],
)
nose_mesh.update()
nose = bpy.data.objects.new("Asymmetric forward nose", nose_mesh)
bpy.context.collection.objects.link(nose)
nose.data.materials.append(shell_material)
parts.append(nose)

if variant == "rear":
    bpy.ops.mesh.primitive_cube_add(location=(-1.7, 0, 0.48))
    tail = bpy.context.object
    tail.name = "Raised transverse tail cap"
    tail.dimensions = (0.6, 1.2, 0.44)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    tail.data.materials.append(shell_material)
    parts.append(tail)

bpy.ops.object.select_all(action="DESELECT")
for part in parts:
    part.select_set(True)
bpy.context.view_layer.objects.active = parts[0]
bpy.ops.object.join()
car = bpy.context.object
car.name = f"QUANTUM_Placeholder_{variant.capitalize()}_Car_Shell"
triangulate = car.modifiers.new(name="Export triangulation", type="TRIANGULATE")
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
