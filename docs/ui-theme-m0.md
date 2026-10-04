# UI Theme M0

## Starting state and scope

The original palette/typography pass started clean on `ui/theme-m0`. A fresh
`git fetch origin` resolved
`origin/main` to `89e879b`, containing Recovery M0/M1A/M1B merges #82/#83/#84.
That commit is an ancestor of this branch. The branch already contained
`424e1f5` (Update QUANTUM icon source); its Blender source was preserved.
No reset, clean, discard, unrelated reformat, dependency addition, or later
milestone was performed.

This changes editor presentation, display sizing, startup environment and
new-document ground defaults, and the existing application-icon pipeline.
Document ownership, transactions/history, serialization, Track/Train/Simulator
routing, selections, renderer ownership, preview/physics/support generation,
window IDs, and docking persistence semantics are unchanged.

## Audit and palette

`EditorUi::initialize()` creates the ImGui context, calls `applyQuantumStyle()`,
resolves the executable directory through SDL, loads fonts/icons, then starts
the existing SDL/Vulkan backends. `EditorStyle.hpp` already held named palette
values and shared font sizes; `EditorStyle.cpp` assigned the global ImGui colors.
M0 extends those two files rather than introducing a theme engine.

Previously the three loaded faces were Overpass Regular 14 px, Overpass
SemiBold 15 px, and Overpass Mono Regular 14 px. Red Hat Mono was bundled but
not deployed/loaded. Its actual static OS/2 weight metadata is Light 300,
Regular 400, Medium 500, SemiBold 600, and Bold 700, with matching italics.
Upright and italic variable fonts are also present; M0 uses static faces only.

The globally assigned colors cover text/disabled text, windows/children/popups,
borders, frames, title/menu bars, scrollbars, checkbox/slider marks, buttons,
headers, separators, resize grips, text cursors, active/inactive/dimmed tabs,
docking, default plots, tables, links, text selection, tree lines, drag/drop,
unsaved/navigation indicators, and modal/window dimming. The same assignments
continue to consume the centralized palette.

All values below are authored **sRGB** hex colors. The existing `fromSrgb()`
conversion supplies linear values to ImGui for the sRGB swapchain; that
mathematical convention was preserved.

| Concept | Before | M0 |
| --- | --- | --- |
| Background | `#1A1A1A` | `#0D1218` |
| Menu | raised-panel color | `#111820` |
| Command toolbar | panel color | `#121A22` |
| Panel | `#222222` | `#151C24` |
| Raised panel / popup | `#292929` | `#18232E` |
| Control frame | `#303030` | `#1C2A36` |
| Hovered frame | `#3D3D3D` | `#233D51` |
| Active frame / selected tab | `#484848` | `#2C4D65` |
| Border | `#565656` | `#2B3948` |
| Strong table border | border color | `#34495C` |
| Separator | `#4A4A4A` | `#2B3948` |
| Primary text | `#ECECEC` | `#C7D7E5` |
| Secondary text | `#B8B8B8` | `#9EB3C7` |
| Muted / disabled text | `#989898` | `#7F94A8` |
| Heading text | primary text | `#AFC4D8` |
| Accent | `#A8D62A` | `#4DADEB` |
| Hover accent | `#B8E044` | `#67D2EB` |
| Active accent | `#91BE1F` | `#3690D1` |
| Muted accent | `#3F501E` | `#1B3C52` |
| Selection | `#34421C` | `#20384C` |
| Hovered selection | `#415322` | `#29475D` |
| Active selection | `#4D6326` | `#31566E` |

Success `#74C98B`, warning `#EBB95B`, error `#FF928C`, and the three existing
destructive fills are unchanged. Black plot canvases, plot dots/reference
lines, channel colors, viewport anchors/selection/axes, and engineering data
retain their existing colors. The green Normal-G channel and green axis are
data, not general accent colors. Green hardware/ground load status means
success. The former seven green accent/selection entries were UI identity.

The remaining local orange/blue packed colors in `EditorUi::updateViewportCamera()`
identify force vectors, and the black alpha outline keeps them legible over
the rendered scene. Curve alpha literals describe overlay emphasis. Those
data/drawing choices stay local; they do not select button, tab, or menu colors.
No blanket green-literal replacement was performed.

## Typography, spacing, and control flow

The three atlas-owned faces are now:

- Overpass Regular, 15 logical px: labels, body text, menus, tabs, and ordinary
  buttons. Active tabs/buttons retain regular text with blue state emphasis.
- Red Hat Mono SemiBold (weight 600), 16 px: existing pane/section headings,
  settings/telemetry sections, Commands, and SIMULATOR. No synthetic bold.
- Red Hat Mono Regular (weight 400), 15 px: existing technical-value/plot uses.

Body and technical sizes increase 7.1%; headings increase 6.7%. `constexpr`
size constants remain shared rather than repeating literal sizes at call sites.
Window padding changes from 8 to 9 px; vertical item spacing from 5 to 6 px.
Frame/cell padding and viewport picking base dimensions retain their values.
The housekeeping continuation below adjusts the fresh Train composition.
Command-area height accounts for the actual heading size.

`editorHeading()` briefly pushes the heading font/color, draws its existing
separator label, then pops both. Commands and SIMULATOR use the same borrowed
heading font. Settings reserve the longest label's width; the support-family
and heartline fields reserve their own label widths. Support descriptions/help
and ground paths wrap. Ground load status gets its own line so long paths
cannot push semantic status offscreen. These are local presentation changes;
the same input callbacks and authored-edit candidates reach their same owners.

The before/after flow is identical: startup -> named colors/font atlas -> each
workspace's existing draw functions -> existing renderer. Only presentation
values and temporary ImGui drawing state change. CMake stages/installs the three
active faces, their licenses, and provenance under executable-relative paths.

Understand now: `inline const` shares named color values; `constexpr` expresses
fixed sizes; `const` references borrow read-only data; `ImFont*` here is a
non-owning pointer. The ImGui context/atlas owns the fonts and destroys them at
shutdown. Existing startup exceptions still report missing/unreadable fonts
and reach the existing cleanup. Balanced Push/Pop restores surrounding drawing
state. The palette/typography changes add no resource owner or lifetime relationship.
Font-atlas rasterization and Vulkan backend details can wait.

## Housekeeping continuation and Legacy reference

This continuation began with intentional dirty Theme M0 files. Copies were
saved locally before editing. `git fetch origin` confirmed `origin/main` at
`89e879b27d42c914eaeef4b20648208dfaa09766` (Recovery M0/M1A/M1B merges
#82/#83/#84), an ancestor of `ui/theme-m0`. HEAD remains `424e1f5` (updated
Blender source). The `.blend` is unchanged, SHA-256
`AB4F62E0F36F0BA9BD1A9B79A07C5BEA758F05AD38B09867190BA11AA1D4BA4C`.

The actual Legacy MainWindow and Train Configuration/Preview/Summary XAML,
M160/M162/M163/M164/M165 notes, and Track/Train/engineering-plot screenshots
were inspected in the [Legacy repository](https://github.com/Coasterpete/QuantumCoasterWorksLib).
The recovered patterns are a compact workspace command hierarchy,
Configuration on the left, a primary Preview, a bottom Summary, understated
labels, aligned technical values/units, and semantic state colors. Avalonia
classes and ownership patterns were not ported.

## Existing icon pipeline

`assets/icons/quantum/quantum_icon_master.png` is the maintainer-supplied
canonical application-icon artwork. Ordinary icon generation reads this file
without modifying it: `tools/build_quantum_icon.py` derives seven LANCZOS PNGs
(256/128/64/48/32/24/16) and `quantum.ico`, preserving the supplied colors and
alpha. Paths resolve from the script's location, independently of the working
directory. `editor/platform/windows/QUANTUM.rc` embeds the ICO as group 101;
the derived PNGs are companion assets, not separately loaded by the application.

The `.blend` and `.blend1` are historical/source assets. Ordinary derivation
does not open, rewrite, or rerender them. `render_quantum_icon.py` remains a
separate tool for intentional future source-art rendering; it writes to the
master path and must not be run as part of canonical icon derivation. The
earlier Theme M0 render and its verification results below are historical;
they do not define the canonical artwork.

## Neutral environment and ground

`ViewportSettings` starts with an empty environment identifier. Vulkan clears
the viewport to linear blue-gray `(0.055, 0.075, 0.10)` and retains the current
sun and existing constant ambient shader branch. The existing environment
cache owns tiny neutral cube/BRDF images, providing valid declared samplers
without HDR preprocessing at startup. Descriptor publication still waits for
frame completion; cache cleanup remains the sole image owner. No new sky
shader, atmosphere, reflection system, or owner was introduced.

Viewport Settings labels the empty option Neutral. HDR-only rotation,
intensity, and sky controls are dimmed there. Selecting a bundled HDR keeps
existing decoding, irradiance, reflection, BRDF, caching, and exposure behavior.
DaySky's stable identifier remains the explicit HDR enable/capture reference.

`newDocumentGroundAppearance()` is shared by `AuthoredTrack::createNewDocument`
and explicit Reset Ground: base color `(0.44, 0.47, 0.46, 1)`, metallic 0,
roughness 0.92, a 1200-unit quad, and 120 UV repeats (10 units per repeat).
No diagnostic texture is assigned. The existing ground renderer/material path
is unchanged. Historical struct defaults remain the missing-field fallback;
existing authored ground values and the document format are preserved.
Reset still issues the existing pending ground edit and history transaction.

## Workspace commands and Train hierarchy

`EditorUiCommands.cpp` implements one member draw function, using small local
lambdas to compose ordinary ImGui groups. A narrow window can scroll the strip
horizontally. Menus and shortcuts retain their existing behavior.

- Shared Document: New/Open/Save and Undo/Redo set the existing pending fields.
- Track Create: Add Region opens the existing typed chooser and reveals it once
  in a short dock. Geometry/Transitions focuses the selected editor or opens
  input settings. Devices/Supports focuses existing panes. Analyze opens Forces
  or enters the existing Simulator. View uses the existing camera menu.
- Train Consist/Setup focuses Configuration or shared Coaster Setup. Preview
  issues existing Play/Pause/Reset requests and Frame All. Analyze focuses
  Summary or enters Simulator. View uses the same camera actions.

The fresh/reset Train dock composition is Configuration-left (27% width),
Preview-primary, and Summary-bottom (27% height). Existing saved arrangements
are honored. The visible Summary uses `###Train Physical Definition`, retaining
its persisted ImGui identity. Shared Coaster Setup still has one implementation,
one open flag, and the existing per-workspace window identities.

Configuration marks the document car count AUTHORED / editable, accepted-preview
values RESOLVED / read only, and success/unavailable states green/amber. Summary
places essential derived values first; physical and resistance backend defaults
are read-only collapsible sections. Tables align values and units. Long repeated
explanations became short labels/tooltips. Playback is grouped in the strip;
Preview has one compact status/speed header. Diagnostic car boxes, bogies,
connectors, camera, texture, and SimulationPreview remain shared.

Before/after authored flow is unchanged: widget -> pending candidate/request ->
Application -> AuthoredTrackEditTransaction/DocumentHistory -> accepted rebuild
-> preview/renderer publication. Lambdas organize drawing locally; const
references borrow inspection data, and optional inspection values represent
preview availability. No new command bus, Train document, or resource owner.

## Display scale and native rendering

`editorLogicalUiScale` computes `(override > 0 ? override : SDL display scale) /
pixel density`. Windows uses pixel window coordinates (density 1), so 150%
content scale means logical UI factor 1.5. A platform with density 2 and display
scale 2 uses logical factor 1 while rendering at twice the pixel density.
The SDL backend continues to publish `DisplayFramebufferScale`.

`applyEditorUiScale` reapplies the base style before `ScaleAllSizes`, preventing
cumulative spacing changes. FontScaleDpi is set once; ImGui's independent
automatic font-only scaling is disabled. ImGui 1.92 rasterizes requested sizes
and framebuffer density through its existing atlas/backend texture updates.
The context owns fonts; EditorFonts borrows pointers. There is no atlas clear,
new font subsystem, or continuously rebuilt Vulkan resource.

Auto is the default. SDL display/content-scale and pixel-size notifications
refresh scale; Preferences offers session-only Auto/100/125/150/175/200%.
The override changes presentation only. Viewport extent remains nearest-pixel
rounding of logical content size times DisplayFramebufferScale. Swapchains
still use SDL_GetWindowSizeInPixels, preserving native resolution, MSAA,
resize/synchronization, texture ownership, and picking. Startup window size is
90% of primary-display usable width and 88% of height, with SDL high-density
support; it remains resizable and windowed.

## Small code paths to inspect

1. `editor/src/EditorUiCommands.cpp`: existing requests behind each command group.
2. `editor/src/EditorUiTrain.cpp`: authored/resolved/backend presentation tables.
3. `editor/src/EditorStyle.cpp`: `editorLogicalUiScale`, `applyEditorUiScale`, and
   `contentPixelDimension`; floats describe UI size, integer extents describe pixels.
4. `editor/src/EditorUi.cpp`: default Train dock builder and beginFrame scale
   refresh; existing window IDs and pending edits remain authoritative.
5. `engine/src/EnvironmentVulkan.cpp`: `createEnvironmentResources` and
   `setEnvironment`; cache ownership and descriptor publication.
6. `core/src/GroundAppearance.cpp`: new-document/reset material factory by value;
   historical struct defaults remain separate compatibility behavior.
7. `engine/src/Application.cpp`: adaptive SDL window creation and existing
   SDL_GetWindowSizeInPixels swapchain path.
8. `assets/icons/quantum/render_quantum_icon.py`: in-memory material/background
   override; existing resize script and Windows resource complete the pipeline.

Understand these data/ownership boundaries now. Advanced Vulkan sampling,
font rasterizer internals, and unchanged coaster numerical algorithms can wait.

## Verification commands

The existing installed dependencies were reused; no package was installed.
Commands ran from this worktree unless noted:

```powershell
$env:VCPKG_ROOT = 'C:\DEV1\vcpkg'
cmake --preset windows-msvc-debug -DVCPKG_INSTALLED_DIR=C:/DEV1/QUANTUM/build/vcpkg_installed -DVCPKG_MANIFEST_INSTALL=OFF
cmake --build --preset windows-msvc-debug --parallel 8 --target QUANTUM QuantumEditorViewportPresentationTests QuantumEditorIconsTests QuantumEditorWorkspaceCompositionTests QuantumEditorDocumentHistoryTests QuantumEditorAuthoredTrackEditTransactionTests QuantumEditorDocumentTests QuantumEditorCoasterSetupTests QuantumEditorSimulationPreviewTests QuantumCoreCoasterDocumentTests QuantumCoreTrainConfigurationTests QuantumCoreTrainPhysicsTests QuantumEditorPreviewSmokeTests QuantumEngineEnvironmentAssetTests QuantumEngineGroundSurfaceTests
cmake --build --preset windows-msvc-debug --parallel 8 --target QUANTUM
ctest --test-dir build -C Debug --output-on-failure -R '^(QuantumEditor\.(ViewportPresentation|Icons|WorkspaceComposition|DocumentHistory|AuthoredTrackEditTransaction|DocumentState|CoasterSetup|SimulationPreview|PreviewSmoke)|QuantumCore\.(CoasterDocument|TrainConfiguration|TrainPhysics)|QuantumEngine\.(EnvironmentAssets|GroundSurface))$'
& .\build\editor\Debug\QUANTUM.exe --dev-preview-smoke smoke-tests/preview-transition.quantum --mode-cycle --repeat --resize-window --duration 12 --output build/verification/housekeeping-mode-cycle --log-level debug
git diff --check
```

Ordinary icon derivation uses Python with Pillow, without a Blender render:

```powershell
python tools/build_quantum_icon.py
```

Local logs, icon/resource checks, original-file snapshots, and test-layout
snapshots are under `build/verification/`. Final results and limitations follow.

## Verification results and limitations

Debug configuration and all fifteen selected build targets succeeded. Final
QUANTUM rebuild after the live corrections succeeded. Existing C4834 at
`DocumentHistoryTests.cpp:488` remains; it was not suppressed or fixed as
unrelated cleanup. The final fourteen selected CTest suites passed in 116.59 s.
Coverage includes presentation/fonts/icons/workspace composition, documents
and history, authored transactions, Coaster Setup, Train configuration/physics,
SimulationPreview, smoke options, environment assets, and ground surfaces.

New focused assertions cover automatic/manual scale math (including density 2),
no cumulative/double scale, native pixel rounding, neutral startup selection,
new-document ground values, and unchanged legacy missing-ground values. The
first history run exposed Reset Ground still using historical struct defaults;
sharing the new-document/reset factory fixed that mismatch, retaining the
exact snapshot and undo/redo assertions.

The final native Debug smoke exited 0: PASSED, 1,190 rendered frames, 227 fixed
steps, all eight scheduled mode actions, 15 viewport resizes and 4 swapchain
recreations. Playback completed normally; preview/physics failure was false
and the failure message empty. Khronos validation was active. No VUID or
validation error was found in the smoke or either live-session log. This is
functional verification, not a performance improvement claim.

Live Windows inspection used the computer-use skill and the real editor on
the current 3440x1440 display at OS scale 100%/density 1. Automatic startup was
approximately 3096x1228 client pixels. Session overrides 100/150/200% were
exercised: fonts/icons/heading weights remained crisp, controls and popups were
usable, and no obvious double scale or stretched viewport was seen. Track and
Train composition, Train first-visit Configuration focus, single preview
status, read-only backend sections, and existing semantic/data colors were
inspected. A Track length edit (60 -> 61) was undone; Train cars (4 -> 5), strip
Undo, Ctrl+Y Redo, and Ctrl+Z Undo restored the clean document. Train playback,
Track/Train entry to Simulator and return to each workspace, and Simulator
Play/Pause were exercised. Add Region reveals its existing typed chooser;
Cancel leaves the document clean. Optional DaySky loaded successfully and
switching back restored the neutral background/ground.

The master and tiny icons were visually inspected. Automated checks confirmed
opaque alpha, blue field/silver highlights, dimensions, exact LANCZOS PNGs,
seven ICO sizes, and EXE group 101/all seven RT_ICON payloads matching the ICO
byte for byte. The committed Blender source hash remains unchanged.
All verification sessions are closed; no scratch document was saved. The
original docking file was restored byte for byte, SHA-256
`14753D3E3E93240BA22165FA7BA382CFC9EC8EB4D56FE4EFC0951D0727A005B3`.
Test-session layouts remain locally for inspection. `git diff --check` passed.

Release/Linux/macOS, physical high-DPI monitors, and moving between monitors
with different OS scales were not tested live. Automated math covers those
scale/density distinctions, not their hardware behavior. Existing very narrow
saved docks can require resizing; vertical Summary/backend content can require
scrolling at larger scales. Stock combo selection text can elide long labels;
its popup exposes the full labels. No universal no-clipping claim is made.
Blender 5.2 source editing remains unverified; the installed 5.1 renderer does
not rewrite the source. No later workspace, Train authoring, terrain, or
atmosphere milestone was started.

## PR handoff

Suggested title: **UI Theme M0: refine workspace hierarchy, neutral defaults, and DPI**

Source: `ui/theme-m0`. Base: `main`. Existing user work and the icon-source
commit are preserved. Changes remain in the worktree; no commit, push, or
pull request was requested/performed.

Ready-to-paste description:

> Complete Theme M0 presentation housekeeping while retaining its navy/cyan
> palette and font hierarchy. Adapt Legacy's grouped workspace commands and
> Configuration-left / Preview-primary / Summary-bottom Train composition;
> distinguish authored values, accepted-preview readouts, and read-only backend
> defaults. Buttons reuse current actions and history paths.
>
> Use a neutral startup sky/ambient and matte new-document ground, preserving
> optional HDR/IBL and legacy document defaults. Add automatic SDL display
> scaling with session overrides and adaptive window sizing; retain native
> Vulkan pixel extents, MSAA, docking identities, resource ownership, and physics.
> Regenerate the existing truss-Q icon as opaque blue with white/silver geometry,
> preserving the Blender source and portable pipeline.
>
> Verified: Debug editor plus fourteen test targets built; fourteen suites
> passed; real Windows Track/Train/Simulator, edits/history/playback, optional
> HDR, and 100/150/200% UI scales checked; resize/mode-cycle smoke passed with
> validation active; EXE icon resources match; original docking file restored;
> git diff --check passed. Exact commands/results: docs/ui-theme-m0.md.
>
> Limits: Release/non-Windows and physical multi-monitor DPI transitions remain
> unverified. Narrow docks and large-scale summaries may need resizing/scrolling.
> Blender 5.1 renders the 5.2 source without rewriting it. No ownership, document
> format, numerical, or later-milestone redesign is included.
