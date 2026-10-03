import bpy
from pathlib import Path

output_dir = Path(__file__).resolve().parent
output_dir.mkdir(parents=True, exist_ok=True)

output_file = output_dir / "quantum_icon_master.png"

scene = bpy.context.scene

# Preserve the source Q geometry/camera. A simple silver material avoids
# the source scene's machine-local metal textures and survives Blender versions.
def linear_channel(channel):
    encoded = channel / 255.0
    return encoded / 12.92 if encoded <= 0.04045 else ((encoded + 0.055) / 1.055) ** 2.4


material = bpy.data.materials.get("rails.001")
backdrop = bpy.data.objects.get("Plane")
if material is None or backdrop is None:
    raise RuntimeError("QUANTUM icon source is missing its rails material or backdrop")

silver = tuple(linear_channel(channel) for channel in (222, 230, 240)) + (1.0,)
blue = tuple(linear_channel(channel) for channel in (7, 67, 196)) + (1.0,)
material.diffuse_color = silver
material.use_nodes = True
material.node_tree.nodes.clear()
surface = material.node_tree.nodes.new("ShaderNodeBsdfPrincipled")
output = material.node_tree.nodes.new("ShaderNodeOutputMaterial")
material.node_tree.links.new(surface.outputs["BSDF"], output.inputs["Surface"])
surface.inputs["Base Color"].default_value = silver
surface.inputs["Metallic"].default_value = 0.5
surface.inputs["Roughness"].default_value = 0.32
surface.inputs["Coat Weight"].default_value = 0.15
surface.inputs["Emission Color"].default_value = silver
surface.inputs["Emission Strength"].default_value = 0.65

# Separate camera background from lighting so the blue field does not tint Q.
backdrop.hide_render = True
scene.world.use_nodes = True
nodes = scene.world.node_tree.nodes
nodes.clear()
camera_background = nodes.new("ShaderNodeBackground")
camera_background.inputs["Color"].default_value = blue
lighting_background = nodes.new("ShaderNodeBackground")
lighting_background.inputs["Color"].default_value = (0.35, 0.35, 0.35, 1.0)
light_path = nodes.new("ShaderNodeLightPath")
mix = nodes.new("ShaderNodeMixShader")
world_output = nodes.new("ShaderNodeOutputWorld")
links = scene.world.node_tree.links
links.new(light_path.outputs["Is Camera Ray"], mix.inputs[0])
links.new(lighting_background.outputs["Background"], mix.inputs[1])
links.new(camera_background.outputs["Background"], mix.inputs[2])
links.new(mix.outputs["Shader"], world_output.inputs["Surface"])
scene.view_settings.view_transform = "Standard"
scene.view_settings.look = "None"
scene.view_settings.exposure = 0.0
scene.view_settings.gamma = 1.0

# High-resolution master render.
scene.render.resolution_x = 1024
scene.render.resolution_y = 1024
scene.render.resolution_percentage = 100

# Opaque blue field, including every small PNG and ICO entry.
scene.render.film_transparent = False

# PNG with alpha.
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_mode = "RGBA"
scene.render.image_settings.color_depth = "8"

scene.render.filepath = str(output_file)

print(f"Rendering QUANTUM icon to: {output_file}")

bpy.ops.render.render(write_still=True)

print("QUANTUM master icon render complete.")
