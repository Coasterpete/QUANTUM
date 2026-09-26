"""Convert an ambientCG OpenEXR sky panorama to Radiance RGBE (.hdr).

Why this exists
---------------
QUANTUM's HDR environment pipeline decodes Radiance RGBE RLE (``FORMAT=32-bit_
rle_rgbe``, ``-Y +X``). ambientCG currently publishes its sky panoramas as
OpenEXR/USD plus an 8-bit tonemapped JPEG, and no longer ships a Radiance
``.hdr``. Rather than add an EXR decoder to the runtime, this developer-only
script converts the EXR once and the resulting ``.hdr`` is committed next to
the existing bundled environment, exactly as the Poly Haven asset was. The
runtime loader is untouched.

This is an offline asset-preparation step in the same spirit as
``tools/export_test_crosstie_glb.py`` (which needs Blender). It is NOT a build
dependency: nothing in CMake or vcpkg.json refers to it.

Usage
-----
    python -m pip install numpy OpenEXR
    python tools/convert_ambientcg_hdri_to_radiance.py <input.exr> <output.hdr>

Row order
---------
The script refuses to convert unless the brightest texel is in the upper half
of the image, which is the physically meaningful case for a sky panorama: it
proves the first stored row is the zenith, matching Radiance's ``-Y +X``
convention where scanline 0 is v=0 (straight up). If a future source image
fails that check the script fails loudly instead of writing an upside-down sky.

Precision
---------
RGBE keeps an 8-bit mantissa per channel with a shared exponent, so the
conversion is lossy for extreme highlights. That is the format the pipeline
already consumes, and QUANTUM authors its own directional sun separately
(``Renderer::setSunlight``), so a saturated sun disc in the panorama is not the
scene's key light.
"""

import struct
import sys

import numpy as np
import OpenEXR


def read_exr_rgb(path):
    with OpenEXR.File(path) as handle:
        rgba = np.asarray(handle.channels()["RGBA"].pixels).astype(np.float32)
    if rgba.ndim != 3 or rgba.shape[2] < 3:
        raise SystemExit(f"{path}: expected an RGB(A) EXR, got shape {rgba.shape}")
    return np.ascontiguousarray(rgba[:, :, :3])


def assert_zenith_first(rgb, path):
    """Fail unless the first stored row is the zenith."""
    luma = 0.2126 * rgb[:, :, 0] + 0.7152 * rgb[:, :, 1] + 0.0722 * rgb[:, :, 2]
    height = luma.shape[0]
    row, column = np.unravel_index(int(np.argmax(luma)), luma.shape)
    if row >= height // 2:
        raise SystemExit(
            f"{path}: the brightest texel is at row {row} of {height}, which is "
            "below the horizon for a zenith-first panorama. Refusing to write a "
            "possibly upside-down sky; flip the image rows first."
        )
    elevation = 90.0 - (row / float(height)) * 180.0
    print(
        f"  brightest texel row={row}/{height} col={column} "
        f"peak={float(luma[row, column]):.1f} -> sun elevation {elevation:.1f} deg"
    )


def encode_rgbe_row(rgb_row):
    """Convert one scanline of linear float RGB to RGBE bytes."""
    out = np.zeros((rgb_row.shape[0], 4), dtype=np.uint8)
    channel_max = rgb_row.max(axis=1)
    significant = channel_max > 1e-32
    if not np.any(significant):
        return out
    values = rgb_row[significant]
    mantissa, exponent = np.frexp(values.max(axis=1))
    # RGBE stores a shared exponent and an 8-bit mantissa per channel.
    scale = np.ldexp(mantissa, 9) / values.max(axis=1)
    encoded = np.clip(np.floor(values * scale[:, None] + 0.5), 0, 255)
    out[significant, 0:3] = encoded.astype(np.uint8)
    out[significant, 3] = (exponent + 128).astype(np.uint8)
    return out


def encode_channel_scanline(scanline):
    """Radiance new-RLE for one component of one scanline.

    Runs of at least four identical bytes become run codes; everything else is
    emitted as literal blocks. Counts stay inside 4..127 for runs and 1..128
    for literals, which is the only range QUANTUM's loader accepts.
    """
    data = bytes(scanline)
    width = len(data)
    out = bytearray()
    x = 0
    while x < width:
        run_end = x
        while run_end < width and data[run_end] == data[x]:
            run_end += 1
        run_length = run_end - x
        if run_length >= 4:
            remaining = run_length
            while remaining > 0:
                count = min(remaining, 127)
                out.append(count + 128)
                out.append(data[x])
                x += count
                remaining -= count
            continue
        literal_start = x
        x += 1
        while x < width and (x - literal_start) < 128:
            # Stop the literal as soon as a run of four begins so the run can
            # be encoded on the next iteration.
            if x + 3 < width and data[x] == data[x + 1] == data[x + 2] == data[x + 3]:
                break
            x += 1
        literal = data[literal_start:x]
        out.append(len(literal))
        out.extend(literal)
    return bytes(out)


def write_radiance(path, rgb, comment):
    height, width = rgb.shape[:2]
    if width < 8:
        raise SystemExit(f"{path}: Radiance needs a width of at least 8, got {width}")
    header = (
        "#?RADIANCE\n"
        "# QUANTUM bundled sky environment\n"
        f"# {comment}\n"
        "# Converted from OpenEXR by tools/convert_ambientcg_hdri_to_radiance.py\n"
        "FORMAT=32-bit_rle_rgbe\n"
        "\n"
        f"-Y {height} +X {width}\n"
    ).encode("ascii")

    with open(path, "wb") as handle:
        handle.write(header)
        for y in range(height):
            rgbe = encode_rgbe_row(rgb[y, :, :])
            handle.write(bytes((2, 2, (width >> 8) & 0xFF, width & 0xFF)))
            for channel in range(4):
                handle.write(encode_channel_scanline(rgbe[:, channel]))


def main(argv):
    if len(argv) != 3:
        raise SystemExit("usage: convert_ambientcg_hdri_to_radiance.py <in.exr> <out.hdr>")
    source, destination = argv[1], argv[2]
    print(f"Reading {source}")
    rgb = read_exr_rgb(source)
    print(f"  shape={rgb.shape}")
    assert_zenith_first(rgb, source)
    print(f"Writing {destination}")
    write_radiance(destination, rgb, f"source: {source}")
    print("Done. Commit the generated .hdr next to the other bundled environment.")


if __name__ == "__main__":
    main(sys.argv)
