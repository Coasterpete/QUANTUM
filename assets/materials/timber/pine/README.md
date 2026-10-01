# Timber assets

Solid Supports M2A loads package-relative PNG maps from this folder. An authored
identifier such as `assets://materials/timber/pine/pinewood_normal.png` resolves
to `assets/materials/timber/pine/pinewood_normal.png` beneath the runtime asset
root; an identifier outside this folder is rejected.

These maps were authored from a real wood source using Materialize and GIMP.
They are user-authored material assets, not generated diagnostics.

| File | Slot | Encoding | Used in M2A |
| --- | --- | --- | --- |
| `pinewood_basecolor_neutral.png` | Albedo | sRGB | Yes |
| `pinewood_normal.png` | Normal | Linear (OpenGL +Y) | Yes |
| `pinewood_roughness.png` | Roughness | Linear | Yes |
| `pinewood_ao.png` | Ambient occlusion | Linear | No — reserved |
| `pinewood_height.png` | Height source | Linear | No — reserved |
| `pinewood_edge.png` | Edge source | Linear | No — reserved |

## Neutral base color

`pinewood_basecolor_neutral.png` is intentionally **grayscale**. Every texel has
R = G = B. It carries grain, knots, and local brightness variation only; it is
not intended to force every timber support to be tan pine. The rendered color
comes from the authored **Timber Color** tint in the Support Appearance panel,
applied in the shader as:

```
final base color = sRGB-decoded authored tint × neutral wood detail
```

The tint is a document value, so it is non-destructive, survives save/load and
Undo/Redo, and is completely independent of the structural family. Hybrid
Timber Lattice does not imply brown and Modern Twister Timber does not imply
tan; geometry and appearance are separate concepts.

## Why the roughness map is remapped

The supplied roughness map is a real-world surface signal, not a multiplier map
tuned for this renderer. Its median sits near 0.31 and roughly 13% of texels are
near-black. Feeding it to the ground shader's straight-multiply convention
(`roughness = authored × map`) would drive those texels to near-mirror gloss and
read as wet, varnished wood.

M2A therefore treats it as a **detail signal remapped into a band around 1.0**
(`mix(0.55, 1.45, map)`) and then multiplied by the authored roughness. The
grain-correlated variation is preserved, timber stays matte, and the authored
Roughness Multiplier control remains meaningful.

## Reserved maps

`pinewood_ao.png`, `pinewood_height.png`, and `pinewood_edge.png` ship with the
set but are **not loaded by M2A**. They are kept for future authoring work:

- parallax and micro-displacement from the height map;
- dirt, stain, and weathering masks;
- edge wear.

Solid support geometry keeps a clean structural silhouette in M2A; physically
displacing a beam from a height map would trade silhouette accuracy for detail
that reads poorly at coaster scale. Ambient occlusion is left unused because
QUANTUM's current PBR path has no AO input, and adding one purely for this
milestone would mean redesigning the shared material system.

Supply a different timber set by dropping PNG files into this folder. M2A reads
the three required slots at startup and falls back to built-in neutral 1×1 maps
for any slot that cannot be read, logging the reason once rather than failing.
