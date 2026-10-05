# Workbench Presentation M1 — Recover Legacy Depth

## Problem and references

Work started clean on `ui/workbench-presentation-m1` at merged M4 `ca5f739`,
also the checked `origin/main` base. The continuation preserves the preceding
M1 changes in this worktree.

Theme M0 established the navy/blue-gray/silver palette, Overpass and Red Hat
Mono, command groups, display scaling, and Train's Configuration → Preview →
Summary composition. Track still mixed navigation and editing in one sidebar;
the opposite sidebar opened on Supports. Small generic headings, uniform
button chrome, sparse Route entries, and stacked readouts obscured pane roles.

The maintainer's previous application,
[Legacy QUANTUM](https://github.com/Coasterpete/QuantumCoasterWorksLib), was
studied read only at `a4ad466`. References:

- `Services/Workspaces/WorkspaceComposition.cs`: separate Route, Viewport,
  Inspector, Math Plots and Diagnostics; task-specific Train composition.
- `WorkspaceProfileCatalog.cs` and `Services/Docking/DockPaneRegistry.cs`:
  workspace/pane roles and stable identity, without copying their registries.
- `MainWindow.axaml` / `.axaml.cs`: menu, commands, work surface and status hierarchy.
- `Controls/RoutePaneControl.axaml` / `.axaml.cs`: ordered entries, kind and
  summary, restrained surface/border treatment and selection emphasis.
- Inspector, Math Plots and Diagnostics controls: focused fields, technical
  labels, station context and subdued diagnostic information.
- Train Configuration, Preview and Summary controls: authored input versus
  accepted output and a dominant preview.
- [Track screenshot](https://github.com/Coasterpete/QuantumCoasterWorksLib/blob/a4ad466/docs/images/editor/track-workspace.png)
  and [Train screenshot](https://github.com/Coasterpete/QuantumCoasterWorksLib/blob/a4ad466/docs/images/editor/train-workspace.png).

Recovered principles are pane purpose, ordered information, compact context,
different weights for editing and diagnostics, aligned technical readouts, and
deliberate proportions. Legacy's Avalonia controls, docking ownership/registry,
graph-node model, connector arrows, schematic renderer and obsolete editable
train parameters were deliberately not ported. Current C++ contracts remain
authoritative.

## Track

- Route is navigation and structural authoring. Each entry has a technical
  order/kind heading, length and cumulative authored station range. Selected
  entries have an accent border/edge; metadata is secondary. Text wraps at
  narrow widths without shrinking fonts. The header reports route length/count.
- Region Inspector receives the existing length and region-style controls.
  Authored length is distinct from derived station, height-change and net-angle
  readouts. Readouts align labels with monospaced values; narrow panes stack
  them. Placement is open initially, with the existing tree state thereafter.
- Region creation, conversion, reorder, removal, connectivity and hardware
  actions remain accessible in Route. Hardware is a collapsed detail group
  initially rather than competing with the route outline.
- Transition/force-target headings identify the selected region, whole-track
  station range and local distance domain. The ruler states metres explicitly.
  Existing channel colors, analytical curves, ranges, markers, hit testing and
  edit behavior are preserved. The canvas uses M0's existing raised-panel color
  instead of pure black, including the shared diagnostic/geometry canvases.
- Active profile controls share compact rows when space permits and stack at
  narrow widths. The graph's minimum height now scales with the UI, preventing
  the scaled ruler from consuming its unscaled minimum at 200%.
- Force Diagnostics has its own heading, region context and read-only status.

Default/reset layout: Route targets 260 logical px (14–22% of width), Inspector
330 px (18–26%), leaving approximately 52–68% of the upper width for the viewport.
Analysis uses 34% of dockspace height, increased from 27% so plot controls and
graph can coexist. Diagnostics and other technical details remain bottom tabs.
Capture scenarios keep their existing dedicated height fractions.

## Train and shared chrome

Configuration, Preview and Summary share compact pane bands with title,
optional context and status. Configuration labels authored consist edits and
accepted preview readouts separately. Summary remains entirely read only;
its value tables cap their width at 680 scaled logical px so wide docks do not
separate labels from values. Backend details use compact tree disclosures.
Preview keeps the existing camera, playback and diagnostic train geometry.

Train's bottom proportion remains 27%. Configuration targets 320 logical px,
bounded to 20–30% of width, leaving Preview 70–80% of the upper width. The car
count field reserves space for its label; Preview's view button wraps as needed.

The command strip starts with Track/Train identity and has labeled groups,
restrained dividers and consistent compact buttons. Document, Route or
Configuration, editing, preview/analysis and view actions reuse their existing
requests or pane focus. Horizontal scrolling preserves commands on small windows.

Only two shared helpers were added: `editorPaneHeading` and
`editorSectionHeading`. They borrow atlas-owned fonts, temporarily push styling,
draw immediately and restore it. There is no new UI owner, registry or framework.
Pane separation uses more space than control rows; section headings use space
and type rather than repeated full-width separator lines or heavy cards.

## Flow, ownership and persistence

`EditorUi::beginFrame` still submits menu/commands/dockspace before panes.
`showTrackWorkspace` borrows the committed track and returns `TrackWorkspaceEdit`;
moving fields to Inspector does not change Application acceptance/history.
Selection still uses the existing shared region index. `RegionStations` comes
from the existing summary calculation; Route uses one running sum in order,
not a separate geometry solver or a per-row scan of the whole track.
Transition Editor borrows a station summary for the duration of its draw call.

Route retains `###TRACK WORKSPACE`; Summary retains `###Train Physical Definition`.
Existing pane/dockspace identities, ImGui ini persistence and inactive
`KeepAliveOnly` behavior remain. A new Inspector gets a first-use placement in
the existing right dock when available; its saved placement takes precedence.
Default/reset construction selects the Route/Inspector tabs once, including
after the existing Coaster Setup docking correction. Startup does not rebuild
a persisted layout or repeatedly force selected tabs.

No physics, 1/240 s timing, renderer conventions/world rendering, GLB loader,
train simulation, track derivation, schema, authored contracts, history,
support generation, devices, icon assets, dependencies or CMake targets changed.
Simulator was not redesigned.

## Visual observations and limits

Before/earlier-after Windows Debug observations were recorded in
`build/verification/workbench-m1/` (ignored local evidence). Both used the new
Untitled document with one 60 m Profile region and four preview cars. The
before screenshot has selected-region fields and hardware competing with
Route, Supports taking the inspector position, and generic technical headings.
Earlier-after screenshots show the separated Inspector, stronger pane bands,
grouped commands and the primary viewport. Train reads more clearly as input,
preview and accepted output; the accepted four-car mass remained 4000 kg.

The final Route metadata, aligned placement readouts and plot context/canvas
changes were implemented in the source-based continuation. Those changes are
not shown in the earlier-after captures and have not received a new live visual
inspection. The maintainer explicitly requested source/build/test work instead
of extended computer control. **Final visual approval is pending maintainer review.**

| Required comparison | Evidence / conclusion |
| --- | --- |
| Stronger Track hierarchy? | Earlier live pane separation plus final source changes establish distinct navigation, editing and analysis roles. Final visual judgment pending. |
| Viewport primary? | Yes in earlier live desktop/reset observations; defaults bound sidebars and preserve the largest central surface. |
| Train Configuration → Preview → Summary? | Yes in earlier live observations; Preview dominates, Summary is accepted read-only output. |
| Intentional command surface? | Earlier captures show identity, labeled groups and dividers instead of one undifferentiated row. |
| Raw Inspector control dump? | Reduced in Region Inspector; final aligned readouts await visual review. Coaster Setup and Supports retain denser older forms. |
| Less pane competition? | Yes in earlier observations: structural actions stay left, selected edits right, diagnostics tabbed below. |
| Continuation of Legacy, rather than downgrade? | Concrete Legacy organization recovered; this qualitative acceptance belongs to the maintainer and is not established by compilation. |

Earlier manual checks: region length 60 → 61 and Undo; append/select a second
Profile and Undo; Train cars 4 → 5 with mass 5000 kg and Undo; Play/Pause/Reset;
camera/zoom; workspace switching; layout reset and persistence across relaunch;
menus and command buttons. Simulator's M4 proof was loaded, rotated, paused and
reset. Undock/redock gestures were attempted but not conclusively verified.

Earlier DPI/window observations used session overrides of 100%, 150% and 200%, a maximized
3440×1392 window, desktop windows around 1844×1020 / 3098×1256, and a smaller
1446×900 window. Pane bands wrapped; saved widths were retained; reset adapted
to scale. At 200% small windows, commands required horizontal scrolling and
Summary required vertical scrolling. The final scaled graph minimum was built
and tested after the captured 200% plot limitation; it was not visually rechecked.
Final Route/title wrapping and narrow Inspector fallback are source verified,
not newly claimed live DPI results.

## Verification

Windows MSVC Debug, using the existing preset and already installed M4 vcpkg
dependencies; no packages were added. All targets built successfully, followed
by a final QUANTUM rebuild after the Route wrapping adjustment. Logs:
`build-m1-review.log`, `build-m1-final.log` in the local evidence directory.

The earlier full CTest run passed 97 enabled tests in 618.56 s; three executables
were initially missing following a disk-space build failure. After rebuilding
them, all three passed in 6.02 s. Thus all 100 enabled tests passed across those
runs (`tests-all.log`, `tests-recovered.log`), not in one uninterrupted full run.
The existing disabled `QuantumEngine.GpuTrainPoseResidency` remains disabled.
The heading test checks font/style/DPI restoration at 1.0/1.5/2.0, without pixel
diffs. Existing suites cover viewport sizing/scaling, workspace/document/history
contracts, Track transitions/diagnostics, Train configuration/preview and M4.
The final focused run passed 15/15 in 24.41 s
(`tests-m1-final-focused.log`), covering all the relevant contracts listed above.
A duplicate full rerun passed its first 17 tests and was deliberately stopped
during the unchanged long DynamicContactPhysics test; it is not claimed as a
second completed full run (`tests-m1-review.log`). No test was weakened or disabled.

Final native Debug mode-cycle/resize smoke passed: 12.006 s, 1199 frames,
225 fixed steps, 15 viewport resizes and four swapchain recreations
(`mode-cycle-final.txt` / `.json`, exit 0). This is a
runtime regression check, not visual approval or a Train workspace interaction
test. `git diff --check` passes.

## Maintainer reading path

Start with `EditorStyle.cpp::editorPaneHeading` / `editorSectionHeading`, then
`EditorUi.cpp::showTrackWorkspace`, `showTransitionEditor`, the two default
layout builders, and `EditorUi::beginFrame`. Continue with
`EditorUiCommands.cpp::drawWorkspaceCommandArea`,
`EditorUiTrain.cpp::drawTrainWorkspace` / `drawTrainViewportToolbar`, and
`ViewportPresentationTests.cpp::fontsAndOverlay`.

The useful C++ concepts are borrowed `const` references and spans, stack-local
formatting buffers, a local lambda for repeated readout rows, balanced ImGui
font/style/table calls, and separating visible labels from persistent IDs.
Vulkan lifetime, document ownership, numerical algorithms and retained UI
patterns are background that need no changes for this milestone.
