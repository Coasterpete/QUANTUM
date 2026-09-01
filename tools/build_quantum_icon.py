from pathlib import Path
from PIL import Image

repo_root = Path(r"C:\DEV1\QUANTUM")

icon_dir = repo_root / "assets" / "icons" / "quantum"
generated_dir = icon_dir / "generated"

master_path = icon_dir / "quantum_icon_master.png"
ico_path = icon_dir / "quantum.ico"

generated_dir.mkdir(parents=True, exist_ok=True)

sizes = [
    256,
    128,
    64,
    48,
    32,
    24,
    16,
]

with Image.open(master_path) as source:
    source = source.convert("RGBA")

    for size in sizes:
        image = source.resize(
            (size, size),
            Image.Resampling.LANCZOS
        )

        destination = generated_dir / f"quantum_{size}.png"
        image.save(destination)

        print(f"Generated {destination}")

    source.save(
        ico_path,
        format="ICO",
        sizes=[(size, size) for size in sizes],
    )

print(f"Generated Windows icon: {ico_path}")