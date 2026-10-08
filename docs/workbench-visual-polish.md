# QUANTUM Workbench Visual Polish

## MAINTAINER SUMMARY

This local milestone centralizes the managed dark workbench's visual language,
embeds the existing Overpass Regular/Semibold and Overpass Mono Regular assets,
and adds a coordinate-based dotted Timeline grid. Numeric fields and technical
readouts use mono typography; menus, names, labels and buttons remain proportional.
Browser, Inspector, input, dock and runtime surfaces have distinct roles. Selected
rows have an accent edge; keyboard input focus has a two-pixel outline. Authored
controls retain their existing Apply boundaries, while runtime values remain active,
read-only information on a blue-tinted surface.

Windows Debug builds and automated regressions passed. One live smoke passed
23 checks and closed cleanly. **Maintainer aesthetic acceptance is pending.** The
one saved Track image exposes Timeline label crowding at the default dock height:
three channel rulers are compressed, and the bottom range readout can overlap the
distance ticks. This is an unresolved presentation limitation, not a successful
visual acceptance claim. No post-smoke tuning or second live run was performed.

No native source, simulation, geometry, renderer, document format, stable ID,
physics timestep, material asset or Operations behavior changed.

## AUDIT

- `EditorApplication.axaml` previously contained seven brushes, two family URIs,
  a small set of styles, hard-coded pending/button/selection colors, and a
  0.45 disabled-button opacity.
- `MainWindow.axaml` added its own viewport/error colors and dock-item styles.
  Helper/title sizes varied directly throughout views, including 11-pixel labels.
- The project already contained legitimate Overpass Mono and Semibold assets,
  but Avalonia embedded only Overpass Regular and Red Hat Mono Regular.
- `ProfilePlot` used hard-coded channel/canvas/selection brushes, default system
  typography, a zero-axis line, and no drafting grid.
- Fluent templates own several state colors internally. Outer-control setters
  alone do not cover selected, hovered, focused and disabled template parts.
  The scoped state selectors were checked against the
  [official Avalonia 12.1.3 control templates](https://github.com/AvaloniaUI/Avalonia/tree/12.1.3/src/Avalonia.Themes.Fluent/Controls).
- Existing progressive disclosure, bindings, stable section keys, native samples,
  pointer operations and preference lifetime supplied the boundaries for this work.

## DESIGN TOKENS

`Theme/QuantumTokens.axaml` is the single managed color/family/type-scale source.
It contains background and surface levels; borders; primary, secondary, muted and
disabled text; accent and selection states; input/focus roles; runtime, warning,
error, info and success roles; and grid, graph and handle roles. Views choose
semantic resources or semantic classes. The token dictionary also supplies small
control-padding and corner-radius resources. `QuantumControls.axaml` applies these
roles to the existing Fluent templates. No universal canvas or theme framework was
introduced. This milestone remains dark-only; the semantic boundary permits a
later palette replacement without changing individual view colors.

## TYPOGRAPHY

| Role | Logical pixels | Family / emphasis |
|---|---:|---|
| Product title | 21 | Overpass Semibold |
| Inspector / workspace titles | 18 | Overpass |
| Normal controls / section headers | 14 | Overpass / Semibold |
| Technical values | 13 | Overpass Mono Regular |
| Helpers, graph rulers / diagnostics | 12 | Overpass / Overpass Mono |

`QuantumUIFont`, `QuantumUIFontMedium`, `QuantumUIFontBold` and `QuantumMonoFont`
centralize family lookup. Avalonia keeps family and weight separate; the heading
weight token is Semibold, backed by the real embedded face. No additional Bold
asset or unrelated unused weight was added. Numeric TextBoxes use the `numeric`
class; technical TextBlocks use `technical` or `runtime`. Paths, names and filters
retain proportional text. Coordinates, dimensions, speed, force, mass, profile
values, snap increments, graph values, IDs and telemetry use mono where displayed.

## CUSTOM FONT ASSETS

Reused, unmodified assets already in `editor/assets/fonts/Overpass/`:

- `overpass-regular.otf`
- `overpass-semibold.otf`
- `overpass-mono-regular.otf`
- `LICENSE.md` (existing upstream license, unchanged)

The provenance is Red Hat's official
[Overpass v3.0.5 release](https://github.com/RedHatOfficial/Overpass/tree/v3.0.5),
as documented in the existing README with SHA-256 hashes. No font/license file was
added or downloaded. The README gained the managed-workbench policy. The managed
project embeds all three active faces and copies the existing license and README
to `licenses/Overpass/` in its output. The native ImGui font configuration remains
unchanged.

`QuantumFonts.Initialize` checks packaged assets and resolved family/weight before
constructing the main window. Missing resources replace the affected token with
a proportional/mono fallback, emit trace/stderr diagnostics and populate
`EditorApplication.FontDiagnostics`. Dynamic font resources let controls use the
replacement. The automated missing-family test exercises recovery; the real live
app reported zero fallbacks. Glyph resources belong to Avalonia's font manager;
the validator does not dispose borrowed glyph handles.

## CONTRAST

Measured from opaque semantic token pairs using sRGB relative luminance:

| Foreground / background | Ratio |
|---|---:|
| Primary / application | 16.86:1 |
| Secondary / application | 11.66:1 |
| Secondary / Inspector | 9.46:1 |
| Input text / input | 16.04:1 |
| Selected text / selection | 7.00:1 |
| Selected text / selected hover | 6.71:1 |
| Secondary text / selected hover | 4.64:1 |
| Error / error surface | 9.71:1 |
| Runtime / runtime surface | 9.17:1 |
| Primary action text / accent | 10.22:1 |
| Primary action text / pressed accent | 7.16:1 |
| Disabled text / surface | 6.04:1 |

These are regression floors, not accessibility certification or a measurement of
every rendered pixel. Selection hover was darkened after its initial secondary
text combination fell below the target. Disabled buttons now use deliberate
foreground/background values instead of fading the entire button to 0.45 opacity.

## SURFACE HIERARCHY

The application is darkest; browsers use Surface0; Inspectors and bottom-dock
chrome use Surface1; nested section headers and secondary buttons use Surface2;
hovered/open sections use raised surfaces. Inputs are recessed with visible
borders. The Timeline canvas is dark blue, and Operations Runtime has a separate
surface. Borders remain modest. Splitters use seven logical pixels, with stronger
hover/drag/focus indication. ScrollViewer auto-hide is disabled so long Inspector
scrollbars remain discoverable. Dock sizing subtracts its measured header height,
preserving the existing saved content-height meaning as header typography changes.

## INPUTS

TextBoxes and ComboBoxes have explicit idle borders, hover/focus states and
readable disabled states. Focused inputs use two-pixel outlines; read-only inputs
use the runtime surface. Existing validation/error pseudo-states get error borders.
Native command rejection still appears in the existing global error banner, now
with an explicit ERROR label/icon. No new per-field validation or commit mechanism
was invented. Names and paths remain ordinary proportional fields.

## SELECTION / FOCUS

Route, consist, Supports, hardware and infrastructure rows share selected-surface
styles with a three-pixel accent edge. Hover and selected-hover are distinct.
Bottom dock pages use a bottom indicator and stronger selected text. The active
workspace ComboBox has emphasized text and accent chrome. Keyboard focus uses
outlines for inputs, buttons, lists, tabs, Expander headers and splitters. Existing
desktop navigation, Ctrl+T, Escape and the one-way dock-selection binding remain.

## BUTTON HIERARCHY

Primary Apply/Add/Generate/Play actions use an accent surface with dark text.
Secondary actions retain subdued raised surfaces. Remove/Delete actions use
destructive text/borders, with their action names supplying a non-color cue.
Disabled states remain legible. No command or confirmation behavior changed.

## AUTHORING VS RUNTIME

Authored hardware/Train/Support fields stay editable and staged, with existing
Modified/Apply/Revert boundaries and native history. Operations Runtime keeps its
READ ONLY heading, technical mono values and distinct blue surface. It contains
the existing runtime command buttons and no editable TextBoxes. Runtime text is
active information, not dimmed disabled text. Train preview status also uses the
runtime typography/color role.

## TIMELINE GRID

`TechnicalGrid.CalculateAxis` receives the visible coordinate range and pixel
extent. It selects 1/2/5-based steps with a target minimum spacing of 24 logical
pixels, anchored at coordinate zero. Every fifth tick is major. Each axis is
capped at 128 ticks, bounding a row to 16,384 points even for oversized canvases.
Nonfinite, zero-span and invalid-size inputs safely produce no ticks.

`ProfilePlot.DrawGrid` combines those ticks directly with its existing X/Y
projection; it does not allocate a stored point cloud. Dots therefore move with
pan and change scale with zoom. The existing frozen Y range during dragging keeps
the ruler stable. Major intersections use larger/brighter dots; the zero reference
remains a line. The bottom row supplies a shared distance ruler in metres.

The grid receives no snapping metadata and never changes authored values. Snap
continues through the existing view-model/native path. The UI explicitly labels
the adaptive display grid and separate snap increments. Profile mathematics,
clamping, tolerances and native preview evaluation are unchanged.

## TIMELINE HANDLES / CURVES

Curve colors are semantic channel roles and lines are stronger. Selection is
under the grid. Value handles remain circles and distance boundaries squares;
pinned boundaries have an explicit cross. Hover/selection gain halos, with the
selected halo larger. Circle/square visibility increased while the existing
eight-pixel hit radius, drag threshold, operations and event flow stayed intact.
Handle readouts use current native preview samples and explicit units.

Geometry layout stores a color token key; only rendering resolves brushes. This
preserves headless hit-testing without starting an application theme. The grid
uses immutable C# record structs for small calculation results, with ordinary
loops for drawing; it introduces no C++ mathematical or ownership change.

**Observed limitation:** at the default 280-pixel dock height, three channel plots
have insufficient vertical space for comfortable endpoint/ruler labels. The
bottom range readout also shares space with distance ticks. The saved image is an
honest record; this should be addressed or accepted in a focused presentation
follow-up before declaring aesthetic acceptance.

## VIEWPORT CHROME

Presentation choice, frame actions, labels and overlays popups consume managed
theme roles. Popup/flyout/tooltip surfaces and fonts are explicit. The existing
canonical icon and QUANTUM name remain. No Vulkan rendering, shader, material,
viewport ownership, synchronization or native shortcut behavior changed.

## STATUS / TELEMETRY

The simulation bar now shows playback state, time, speed and station in mono
typography alongside Play/Pause/Reset. Deep telemetry remains in its existing
read-only dock/tooltip locations. This string change reads the same snapshot and
does not change preview progression or physics.

## ACCESSIBILITY-RELATED CHECKS

The managed visual check resolves packaged font resources and all four family
tokens; exercises a missing-family fallback; checks representative numeric controls
and absence of view-local family/color literals; and verifies 12 contrast pairs.
Offscreen Fluent controls verify actual numeric typography, selection edge,
selected-hover surface and focus outline. Error text, action names, runtime labels,
selection edges and crossed pinned handles supplement color. No pixel-perfect
font test, screen-reader audit or standards certification is claimed.

## REGRESSIONS

Windows MSVC Debug `QuantumNative` build passed. Managed Editor/Managed/Tests
Debug build passed with **zero warnings and zero errors**. Final exact commands:

```powershell
cmake --build build --config Debug --target QuantumNative --parallel 2
& editor/managed/Build.ps1 -Configuration Debug -UiOnly
dotnet build editor/managed/Quantum.Managed.Tests/Quantum.Managed.Tests.csproj -c Debug --nologo
& editor/managed/Quantum.Managed.Tests/bin/Debug/net10.0/Quantum.Managed.Tests.exe --verify-visual C:\Dev8\QUANTUM
& editor/managed/Quantum.Managed.Tests/bin/Debug/net10.0/Quantum.Managed.Tests.exe --verify-workbench-binding
& editor/managed/Quantum.Managed.Tests/bin/Debug/net10.0/Quantum.Managed.Tests.exe C:\Dev8\QUANTUM\smoke-tests\modern-steel-validation.quantum
ctest --test-dir build -C Debug -R 'QuantumApplication\.(Train2|Supports2|TimelineAuthoring|RideHardwareAuthoring|SupportsWorkspace|TrackPlacement|NativeBoundary|HardwareBoundary|TrainBoundary|SupportBoundary|PlacementBoundary)$' --output-on-failure --parallel 2
```

Results: **1,723 visual resource/contrast/grid assertions**, **3 real dock-binding
checks**, and the full managed suite passed. That suite includes Timeline 126,
hardware 143, Supports 1,078, Supports 2 116, placement 92, Train 70, workbench 77,
plus wrapper/controller/serialization checks. Focused native regressions passed
**11/11**. Grid assertions cover zoom steps, pan origin, major/minor classification,
signed zero anchors, degenerate/nonfinite input, bounded point counts, finite
extreme scales (1e-300 through 1e300) and repeatability. No tests were weakened or
native assertions changed.

## CLEAN SHUTDOWN

The single live smoke used the staged Debug application, isolated report-side
layout settings and `supports-2-structural-styles.quantum`. Command arguments:

```text
--verify-workbench-smoke C:\Dev8\QUANTUM\smoke-tests\supports-2-structural-styles.quantum C:\Dev8\QUANTUM\build\visual-workbench-smoke.json
```

It passed **23 checks**, took **2,558.9 ms**, presented **148 Vulkan frames**, and
exited **0**. Track/Timeline, Train, Supports, Operations, selection, keyboard focus,
mono fields, expand/collapse and clean window/controller/viewport shutdown were
covered. Track's viewport was 856×327 at 1420×900. No resize or DPI sweep was made.
One Avalonia-side image was saved and inspected; it excludes the separately hosted
native child viewport. Vulkan presentation was verified by the frame counter.

Shutdown order stays: confirm edits → save preferences → destroy hosted viewport
→ stop/join controller → dispose native session → close managed window. No new
Vulkan resources or ownership relationships were introduced.

## KNOWN LIMITATIONS

- Timeline label crowding described above remains unresolved under the mandated
  stop after one smoke. Automated smoke success does not equal visual acceptance.
- No physical pointer/keyboard campaign, minimum-size live test, DPI sweep,
  Linux GUI run, Release build or full physics test campaign was performed.
- Contrast tests cover semantic opaque pairs, not every Fluent subcontrol/state.
  Native validation remains authoritative and global error presentation remains.
- The font recovery test simulates a missing family; it does not corrupt fonts.
- The maintainer-reported acceleration behavior was not independently reproduced
  by this milestone. Existing acceleration/mode/commit and Operations checks pass;
  simulation semantics remain untouched. If still observed, investigate it in a
  focused Operations follow-up.

## MAINTAINER VISUAL TEST

1. Launch the staged Debug app. Check proportional labels, mono values, comfortable
   sizes, panel separation, selected rows/tabs and keyboard focus in Track.
2. Inspect the Timeline at the normal and taller dock heights. Judge the dot grid,
   zoom/pan, curve/handle contrast, pinned crosses and ruler/readout crowding.
   Display dots must not imply a new snap increment.
3. Visit Train, Supports and Operations. Expand/collapse sections; check primary,
   secondary and destructive actions; compare authored fields with Runtime values.
4. Confirm workspace/dock preferences, Reset Workspace Layout, Ctrl+T, dirty/pending
   state and clean closing feel unchanged. Final aesthetic acceptance is yours.

## NEXT PHASE

After maintainer acceptance, recommend **Material System 2.0 — anti-tiling, macro
variation, deterministic per-object variation and improved material presentation**.
It was not started. Address/accept the noted Timeline presentation limitation
before that acceptance. Keep any unresolved acceleration issue as a separate
Operations investigation.

## OPTIONAL CODE READING PATH

| Priority | Smallest useful location | What to understand |
|---|---|---|
| READ NOW | `Theme/QuantumTokens.axaml` | Semantic palettes and type scale |
| READ NOW | `Theme/QuantumControls.axaml`: numeric, selected and focus selectors | Template state styling |
| READ NOW | `QuantumFonts.cs`: Initialize/Check | Font validation and fallback |
| READ NOW | `TechnicalGrid.cs`: CalculateAxis/Tick | Coordinate intervals and bounds |
| READ NOW | `ProfilePlot.cs`: Render/DrawGrid | Grid below native sample curves |
| OPTIONAL | `MainWindow.Layout.cs`: DockHeaderHeight/ApplyLayout | Presentation sizing without preference migration |
| OPTIONAL | `Quantum.Managed.Tests/VisualChecks.cs` | Resource, contrast and grid protection |
| DEEP INTERNAL | `RuntimeChecks.Workbench.cs`: RunWorkbenchSmoke | The bounded live harness and shutdown report |

Understand XAML resource lookup, state selectors and graph projection now. Native
interop and Vulkan synchronization remain useful background; solver internals can
wait. No advanced C++ concept was added by this managed presentation milestone.

## LOCAL HANDOFF

Branch: `codex/workbench-visual-polish`, based on the current local UX-consolidation
HEAD. Final local commit is reported in the chat / `git log -1 --oneline`. No push
or PR. The maintainer's untracked `assets/materials/{concrete,fiberglass,grout,metal}`
directories remain untouched and untracked. No font binary/license was added;
only existing assets were wired into the managed output.

Local verification artifacts (build outputs are not committed):

- `build/visual-native-build.log`
- `build/visual-managed-build.log`, `build/visual-staging.log`
- `build/visual-theme-tests.log`, `build/visual-dock-binding-tests.log`
- `build/visual-managed-tests.log`, `build/visual-native-regressions.log`
- `build/visual-workbench-smoke.json`, `.json.png`, `.json.layout.json`
- `build/visual-smoke-stdout.log`, `build/visual-smoke-stderr.log`

Suggested PR title: **Polish workbench typography, contrast and technical Timeline grid**.

### Changed-file manifest

```text
M editor/assets/fonts/Overpass/README.md
M editor/managed/Quantum.Editor/EditorApplication.axaml
M editor/managed/Quantum.Editor/EditorApplication.axaml.cs
M editor/managed/Quantum.Editor/EditorViewModel.Workbench.cs
M editor/managed/Quantum.Editor/HardwareInspector.axaml
M editor/managed/Quantum.Editor/HardwareRuntimePane.axaml
M editor/managed/Quantum.Editor/InfrastructureInspector.axaml
M editor/managed/Quantum.Editor/MainWindow.Layout.cs
M editor/managed/Quantum.Editor/MainWindow.axaml
M editor/managed/Quantum.Editor/OperationsBrowser.axaml
M editor/managed/Quantum.Editor/ProfilePlot.cs
M editor/managed/Quantum.Editor/Quantum.Editor.csproj
A editor/managed/Quantum.Editor/QuantumFonts.cs
M editor/managed/Quantum.Editor/RuntimeChecks.Workbench.cs
M editor/managed/Quantum.Editor/SupportInspector.axaml
M editor/managed/Quantum.Editor/SupportToolsPane.axaml
M editor/managed/Quantum.Editor/SupportsBrowser.axaml
A editor/managed/Quantum.Editor/TechnicalGrid.cs
A editor/managed/Quantum.Editor/Theme/QuantumControls.axaml
A editor/managed/Quantum.Editor/Theme/QuantumTokens.axaml
M editor/managed/Quantum.Editor/TimelinePane.axaml
M editor/managed/Quantum.Editor/TrackLayoutInspector.axaml
M editor/managed/Quantum.Editor/TrainBrowser.axaml
M editor/managed/Quantum.Editor/TrainDiagnosticsPane.axaml
M editor/managed/Quantum.Editor/TrainInspector.axaml
M editor/managed/Quantum.Managed.Tests/Program.cs
A editor/managed/Quantum.Managed.Tests/VisualChecks.cs
A docs/workbench-visual-polish.md
```
