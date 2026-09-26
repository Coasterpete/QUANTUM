"""Generate QUANTUM's Ground Surface M0 test textures.

These are deliberately synthetic diagnostic patterns, not art. They exist so
the ground-texture import path can be demonstrated and regression-checked
without shipping a real-world surface material. Regenerate with:

    python tools/make_ground_test_textures.py

The script only uses the Python standard library (zlib for PNG deflate), so it
runs without Blender or any image package.
"""

import math
import os
import struct
import zlib

TEXTURE_SIZE = 256

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OUTPUT_DIRECTORY = os.path.join(REPO_ROOT, "assets", "ground")


def write_png(path, width, height, rows):
    """Write 8-bit RGB rows (each row a bytes of length width * 3)."""
    raw = bytearray()
    for row in rows:
        raw.append(0)  # PNG filter type 0 (None) for every scanline.
        raw.extend(row)

    def chunk(tag, payload):
        return (
            struct.pack(">I", len(payload))
            + tag
            + payload
            + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
        )

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    with open(path, "wb") as handle:
        handle.write(b"\x89PNG\r\n\x1a\n")
        handle.write(chunk(b"IHDR", header))
        handle.write(chunk(b"IDAT", zlib.compress(bytes(raw), 9)))
        handle.write(chunk(b"IEND", b""))


def checker(x, y, period):
    return ((x // period) + (y // period)) % 2


def albedo_rows(size):
    """Four tinted quadrants with a fine checker and a red identity border.

    The quadrant colours make UV tiling and rotation immediately readable in a
    screenshot, the checker makes minification visible, and the border makes a
    stretched or wrongly wrapped map obvious.
    """
    quadrant_colours = (
        (108, 132, 84),    # grass
        (146, 116, 86),    # compacted earth
        (96, 104, 112),    # gravel
        (72, 88, 104),     # shaded gravel
    )
    rows = []
    for y in range(size):
        row = bytearray()
        for x in range(size):
            if x < 2 or y < 2 or x >= size - 2 or y >= size - 2:
                row.extend((214, 32, 32))
                continue
            base = quadrant_colours[
                (0 if y < size // 2 else 2) + (0 if x < size // 2 else 1)
            ]
            # A 16-texel checker plus a gentle diagonal ramp keeps the pattern
            # legible when the surface is viewed at a grazing angle.
            shade = 0.78 + 0.22 * checker(x, y, 16)
            ramp = 0.85 + 0.30 * (x + y) / (2.0 * size)
            row.extend(
                int(max(0, min(255, channel * shade * ramp)))
                for channel in base
            )
        rows.append(bytes(row))
    return rows


def height_field(size):
    """A low-amplitude bumpy pattern used only to derive the test maps."""
    field = []
    for y in range(size):
        row = []
        for x in range(size):
            ridge = 0.5 + 0.5 * math.sin(x * math.pi / 8.0) * math.sin(
                y * math.pi / 8.0
            )
            step = 1.0 if checker(x, y, 16) else 0.0
            row.append(0.75 * ridge + 0.25 * step)
        field.append(row)
    return field


def normal_rows(field, size):
    """Tangent-space normal map (OpenGL +Y) sampled with the same wrapping as
    the albedo pattern, so both maps line up under UV tiling."""
    encode = lambda value: int(max(0, min(255, round(value * 255.0))))
    rows = []
    for y in range(size):
        row = bytearray()
        for x in range(size):
            left = field[y][(x - 1) % size]
            right = field[y][(x + 1) % size]
            down = field[(y - 1) % size][x]
            up = field[(y + 1) % size][x]
            nx = (left - right) * 2.0
            ny = (down - up) * 2.0
            nz = 1.0
            length = math.sqrt(nx * nx + ny * ny + nz * nz)
            row.extend(
                (
                    encode(nx / length * 0.5 + 0.5),
                    encode(ny / length * 0.5 + 0.5),
                    encode(nz / length * 0.5 + 0.5),
                )
            )
        rows.append(bytes(row))
    return rows


def roughness_rows(field, size):
    """A roughness multiplier map: the 16-texel checker alternates a matte and
    a slightly glossier patch so roughness-map support is visible."""
    encode = lambda value: int(max(0, min(255, round(value * 255.0))))
    rows = []
    for y in range(size):
        row = bytearray()
        for x in range(size):
            value = 0.55 + 0.40 * checker(x, y, 16) + 0.10 * field[y][x]
            level = encode(min(1.0, value))
            row.extend((level, level, level))
        rows.append(bytes(row))
    return rows


def main():
    os.makedirs(OUTPUT_DIRECTORY, exist_ok=True)
    size = TEXTURE_SIZE
    field = height_field(size)
    write_png(
        os.path.join(OUTPUT_DIRECTORY, "test-ground-albedo.png"),
        size,
        size,
        albedo_rows(size),
    )
    write_png(
        os.path.join(OUTPUT_DIRECTORY, "test-ground-normal.png"),
        size,
        size,
        normal_rows(field, size),
    )
    write_png(
        os.path.join(OUTPUT_DIRECTORY, "test-ground-roughness.png"),
        size,
        size,
        roughness_rows(field, size),
    )
    print("Wrote synthetic Ground Surface M0 test textures to", OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
