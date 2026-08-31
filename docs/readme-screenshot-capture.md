# README screenshot capture

This developer-only workflow renders the real QUANTUM editor and saves its
complete client area as PNG. It does not author or edit track content.
Normal startup without capture arguments is unchanged.

## Run

Build from the repository root using the existing configured Debug preset:

```powershell
cmake --build --preset windows-msvc-debug --target QUANTUM
if ($LASTEXITCODE -ne 0) { throw 'Debug build failed' }

& .\build\editor\Debug\QUANTUM.exe --capture-screenshots .\docs\readme-captures.json --log-level warning
if ($LASTEXITCODE -ne 0) { throw 'Screenshot capture failed; see the error output' }
```

**First supply a document.** `readme-captures.json` is an intentionally incomplete
template: replace `USER-AUTHORED.quantum` and choose the appropriate region indices.
There is no tracked, approved screenshot document in this milestone. The command
fails clearly if the fixture is missing; it never falls back to generated content.
You can keep a local manifest under `build/` and pass its path instead.

Document and output paths resolve relative to the manifest, not the executable or
working directory. Absolute paths and paths containing spaces are also accepted.
The command returns **0** after all images are saved, or **1** on invalid arguments,
invalid documents, cancellation, Vulkan/SDL failure, or a failed write. Success
prints one line per image. `--log-level warning` keeps startup chatter quiet without
hiding warnings/errors. Unknown capture flags/fields and duplicate scenario names
are errors. Capture flags cannot be combined with an interactive editing session.

The sample manifest writes previews to `build/readme-captures/`:

| File | Supplied content / presentation |
| --- | --- |
| `editor-overview.png` | Representative document; Regions, Perspective viewport and profile/geometry controls |
| `transition-editor.png` | A Rate Profiles region with at least one varying authored profile; larger Transition Editor and dot grid |
| `geometry-regions.png` | A Geometry region, preferably Circular Arc; Regions left and Geometry Editor right |
| `track-start-gizmo.png` | Region index 0; semantic anchors visible, Track Start selected, Move or Rotate tool |
| `force-diagnostics.png` | A region with at least two real evaluated load samples; Normal G, Lateral G, Longitudinal G and Vehicle Speed plots |

Diagnostics use the same Core evaluation and read-only display as the normal
editor. No force-target authoring is added or implied. If a supplied track becomes
energetically unreachable, the existing diagnostics message remains visible.

Public screenshots must use user-authored or explicitly approved documents.
The built-in developer demo may be saved through File > Save for **local smoke
validation only**; it is not an approved public screenshot fixture. No demo track
generation is included in the capture workflow.

## Manifest fields

The JSON template contains only presentation choices needed by these five shots:

| Field | Meaning |
| --- | --- |
| `width`, `height` | Optional client pixels; defaults 1600×900, accepted ranges 960–3840 and 720–2160 |
| `settle_frames` | Optional consecutive stable-size frames before readback; default 16, range 8–600 |
| `output_directory` | Required output directory, created if missing |
| `overwrite` | Optional, defaults false; explicitly true allows replacing existing screenshot files |
| `scenarios` | One to five distinct entries using the names above |
| scenario `document` | Required existing document, loaded by the production deserializer and solve/load acceptance paths |
| scenario `region` | Required **zero-based** region index (UI region 3 is index 2) |
| scenario `framing` | Optional `all` (default) or `selected`, using existing Frame All / Focus camera behavior |
| scenario `tool` | Optional `move` (default) or `rotate`, only for `track-start-gizmo` |

Each entry requires `name`, `document`, and `region`. Each scenario may use a
different supplied document. Camera orientation is the existing Perspective
preset at 45 degrees; projection, grid/curve visibility, anchor state and panel
layout are fixed by the preset, not a general UI scripting language. Output names
are fixed to the scenario names. Existing directories/symlinks and paths aliasing
the manifest or a supplied document cannot be overwritten as screenshot files.
`overwrite: true` in the example is deliberate so approved inputs can be rerun.

## Determinism and settings safety

Each scenario creates a fresh SDL window, renderer, and EditorUi, then destroys
them in the normal dependency order. The same bundled Overpass fonts and Phase 3
style are used at scale 1.0. The borderless, nonresizable capture window requests
the configured pixel dimensions; a mismatching drawable or readback size is an
error, not an implicit image resize. The driver must support the requested size.

The editor builds a fresh in-memory dock layout. Geometry controls occupy the
right column, while the Transition Editor and Force Diagnostics receive more
space in their respective presets. Force Diagnostics is selected explicitly;
settings popups remain closed. Mouse/keyboard events are not forwarded, ImGui
input is cleared, and its frame delta is fixed. Only native quit/close events are
honored. No desktop cursor, title bar, desktop pixels, or debug text is composited.

Capture sets `ImGuiIO::IniFilename` and `LogFilename` to null **before the first
frame** and never calls the normal preference-path setup. It neither reads nor
writes the user's docking file. There is no backup/restore window in the capture
implementation: normal settings are untouched even on exceptions. It does not
invoke Save, dialogs, recent-document updates, or authored edit transactions.
The existing transaction's read-only load-acceptance check is reused on load.
The interactive demo and transition-snap environment overrides do not affect
capture content or presentation.

Framing is applied with the actual docked viewport aspect. Settling restarts if
viewport size or swapchain generation changes, with 240 extra frames allowed for
startup/recreation. After the requested stable frames, the normal editor draw
callback renders the complete composition and the renderer reads it back.

## Readback and PNG writing

`VulkanContext::initialize(..., true)` opts into swapchain transfer-source usage.
Without this opt-in the normal swapchain usage is unchanged, and no readback
buffer is allocated. Surfaces lacking transfer-source support or an RGBA8/BGRA8
format fail with a diagnostic.
Capture also disables swapchain clipping so obscured pixels remain defined,
as required by the [Vulkan swapchain specification](https://docs.vulkan.org/refpages/latest/refpages/source/VkSwapchainCreateInfoKHR.html).

An optional `FrameImage*` on `drawFrame` requests a synchronous copy after the
ImGui render callback, before presentation. Barriers transition the image from
color attachment to transfer source and then back to present; a buffer barrier,
frame-fence wait and VMA invalidation make the copy available to the CPU.
VulkanContext owns the lazily allocated, mapped VMA staging buffer and destroys
it before its allocator, after GPU completion. Skipped/out-of-date captures
return an empty image and are retried. Pixels are top-to-bottom RGBA8, with BGRA
swizzling when needed and opaque alpha. No offscreen-viewport-only shortcut is used.

The already installed SDL3 3.4 PNG writer (`SDL_CreateSurfaceFrom` + `SDL_SavePNG`)
saves those pixels. No image library, custom PNG encoder, screenshot automation,
shader change, or new dependency is required.

## Verification and limits

Focused tests:

```powershell
cmake --build --preset windows-msvc-debug --target QuantumEditorReadmeCaptureTests
ctest --test-dir build -C Debug -R 'QuantumEditor.ReadmeCapture' --output-on-failure
git diff --check
```

Tests cover argument parsing, invalid/duplicate scenarios, field types/ranges,
relative paths, presentation options, region/content validation, overwrite
opt-in, document/path collision protection (including hard links), and authored
data preservation. Native smoke validation covers layout isolation, rendering,
PNG dimensions, full editor content, and repeated-run hashes without brittle
pixel-baseline tests in CTest.

This is a native Vulkan workflow, not headless CI. Repeatability is intended for
the same build, document, manifest, fonts, and GPU/display environment; identical
pixels across drivers/platforms are not promised. Smaller accepted window sizes
may require scrolling within existing panels; 1600×900 is the validated baseline.
Focus can clip unselected regions by design. Very long region lists may also need
a more suitable supplied document or larger window for public composition.
PNG saves are sequential, not an atomic five-file transaction: after a runtime
failure, earlier images may exist and the command returns failure. A failed PNG
write may leave a partial output file; rerun after fixing the cause.

Core math, physics, serialization semantics, normal authored-edit workflows,
camera semantics, shaders, and `Application::run` are unchanged. Focused tests
plus native renderer validation are sufficient for this opt-in presentation seam;
the expensive full numerical suite is not part of this milestone. Release and
cross-driver validation remain optional follow-up checks.

## Later README refresh

Keep draft images under `build/`. After approving authored fixtures and reviewing
the five images, place final assets in the existing `docs/images/` convention and
update README references/captions in a separate milestone. Do not create a second
documentation screenshot directory. This change replaces no existing README image
and makes no claim that unfinished controls are implemented.
