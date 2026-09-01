import bpy
from pathlib import Path

# Repository root:
# C:\DEV1\QUANTUM
repo_root = Path(r"C:\DEV1\QUANTUM")

output_dir = repo_root / "assets" / "icons" / "quantum"
output_dir.mkdir(parents=True, exist_ok=True)

output_file = output_dir / "quantum_icon_master.png"

scene = bpy.context.scene

# High-resolution master render.
scene.render.resolution_x = 1024
scene.render.resolution_y = 1024
scene.render.resolution_percentage = 100

# Transparent background.
scene.render.film_transparent = True

# PNG with alpha.
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_mode = "RGBA"
scene.render.image_settings.color_depth = "8"

scene.render.filepath = str(output_file)

print(f"Rendering QUANTUM icon to: {output_file}")

bpy.ops.render.render(write_still=True)

print("QUANTUM master icon render complete.")