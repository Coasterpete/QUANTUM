# MAINTAINER SUMMARY

The Avalonia workbench now concentrates on the selected workspace and object.
Track keeps Route, Region/Track Start, Design View and Timeline. Train keeps
Consist, Preview and common car settings. Supports selects a structure, node,
member or infrastructure Inspector. Operations has its own hardware browser,
authored Inspector and separate Profile/Runtime pages.

Train and Supports start with a collapsed bottom dock. Track and Operations
start with Timeline/Profile open. Every workspace retains a Timeline tab;
Ctrl+T opens it, including from the native viewport. Advanced sections remain
available through ordinary Expanders and remember expansion choices.

No engineering feature family was removed. Track mathematics, heterogeneous
train configuration, the specialized solver, 1/240-second fixed stepping,
hardware/operations semantics, Supports 2, materials/physical UV mathematics,
infrastructure, native history, document serialization, GLB assets, Vulkan
rendering algorithms and the ImGui fallback remain intact. Engineering overlays
are in the Train viewport's compact Overlays flyout and default off.

**Implementation is ready for maintainer acceptance; UX approval is pending.**
The one final live smoke failed at the Train transition because the tab strip
wrote its previous Timeline selection into the new workspace. That binding is
fixed and covered by real Avalonia binding tests without opening a window.
The live smoke was not repeated under the one-run limit. Its remaining steps,
including the smaller window size, are unverified live.

# UI DENSITY / APPLY AUDIT

Inspected the existing views, view-model partials, managed wrapper/controller,
native viewport input/lifetime path, CMake/presets, prior handoffs and tests.

| Area | Observed before | Consolidation |
|---|---|---|
| Permanent composition | 320-pixel bottom row, simulation telemetry, Route/Supports/Train panels | Per-workspace dock defaults and dimensions; compact footer with detailed telemetry tooltip |
| Track | Track Start, Region geometry, preview and Train summary appeared together | Placement Inspector only while Track Start is selected; Region Inspector returns on Route selection; geometry/orientation/setup disclosure |
| Train | Physical/Visual/Defaults tabs, full physical array, verbose explanations, overlays and combined summary/diagnostics | Common identity/role/asset first; named advanced sections; viewport flyout; separate Summary/Diagnostics pages |
| Supports | Structure/style/connect controls remained beside node/member fields; infrastructure nested in the same scroll | Object context chooses Inspector; Infrastructure browser page; frame generation in secondary dock |
| Operations | Hardware browser, placement, mode, authored configuration, commands and telemetry inside Timeline | Dedicated browser/Inspector; Profile and read-only Runtime separated; lightweight filter |
| Help | Long paragraphs repeatedly consumed Inspector height | Relevant tooltips and short hints; retained mathematical/asset explanations |
| Commands | Equal button emphasis and separate Apply transactions were hard to distinguish | Primary Add/Apply/Generate actions; explicit placement, configuration, profile and destination commit boundaries |
| Drafts | Successful placement/acceleration Apply preserved speed/target only | Preserve all configuration text and unfinished inputs in other groups; visible pending indicators and Revert |

Add/remove/reorder, prototypes and frame generation remain immediate native
actions. Car/defaults, support edits, infrastructure, placement, hardware
configuration and profile mode retain their existing validated Apply actions.
Profile endpoint/type/split/boundary actions and direct analytical dragging
retain their native transaction semantics. No native-history rewrite.

# WORKSPACE ARCHITECTURE

`MainWindow` still owns the controller and one hosted native viewport. The
view-model owns `WorkbenchLayout`, which contains presentation preferences.
Existing views/commands remain the authoring path. New small views have concrete
responsibilities: hardware browser, runtime commands and support creation tools.
No manager, registry, new service layer or dependency was introduced.

Before: fixed composition → large forms → native proposals/history.
After: workspace preferences + selection → relevant view/section → the same
form cells and native proposals/history. Showing/collapsing a section changes
layout preferences, without issuing an authored document mutation.

Understand now: one object selection versus presentation context; property
notifications; accepted values versus staged text; a successful native revision
versus a UI preference. These changes mostly demonstrate C# concepts. The only
C++ implementation change extends the established unsigned shortcut bit mask:
SDL Ctrl+T → intent 1024 → managed OpenTimelineCommand. Native-child keys bypass
Avalonia routed input, so both paths are needed. Ownership and RAII remain the
existing native session/viewport lifetime; numerical internals can wait.

# TRACK

Route and the selected Region remain primary. Select Track Start from Route or
the viewport to show world placement; select a Region to restore its Inspector.
Geometry, placement orientation and preview setup are collapsible. Timeline
keeps all segment operations, transition types, snapping and direct dragging.
Its hardware source offers an Open Operations action.

# TRAIN

The left pane exposes Consist and Add Car. Generic starting points have clearer
labels: Standard/Lead/Rear passenger, Trailer, Zero car and Custom. These map to
existing native role presets. Resize and initial speed are under Train setup.

The right pane starts with name, role and shell asset. Mass/COG, Body, Bogies,
Hitches, Visual Asset/Alignment, Advanced and Train Defaults are secondary.
The grouped arrays reference the same `TrainField` objects used by the proposal;
no physical field was removed or recomputed. Validate and explicit uniform-box
inertia remain in Advanced. Shell transformations still affect visuals only.

# SUPPORTS

Structures and Infrastructure are separate browser pages. Structures still
contain the node/member lists and their native add/remove actions. Structure
selection exposes Basics, Style/Appearance and Connect Nodes. A node exposes
position, attachment and foundation; a member exposes identity, Section,
Material/UV and Connections. Infrastructure exposes its own name/type/interval
and secondary geometry/appearance/access settings. Gate references navigate to
Operations. The secondary Infrastructure dock contains frame-generation tools.

# OPERATIONS

Hardware choices keep all existing families and native defaults. The filter
matches names, families and intervals; filtering out an item retains selection.
The Inspector shows authored configuration, with placement and profile/response
groups. Stations retain common stop placement; tolerances/interlocks,
destinations and detailed gate geometry are secondary. Profile editing no longer
duplicates the hardware Inspector.

# INSPECTOR GROUPING

`InspectorSection` is a normal Avalonia Expander with a stable settings key and
a default expansion policy. It reuses the Expander theme, forwards normal
keyboard behavior and restores saved choices on DataContext changes/reset.
Identity/common controls are open; exact geometry, COG, inertia, visual
alignment, materials/UVs, IDs and numerical details are secondary. Structure
and node/member context remains separate from infrastructure selection.

# BOTTOM DOCK

| Workspace | Pages | Initial state |
|---|---|---|
| Track | Timeline | Open |
| Train | Summary, Diagnostics, Timeline | Collapsed; Summary selected |
| Supports | Infrastructure, Diagnostics, Timeline | Collapsed; Infrastructure selected |
| Operations | Profile, Runtime, Diagnostics, Timeline | Open; Profile selected |

Click a tab or use its normal keyboard selection, resize with the horizontal
splitter, or collapse/reopen with the header action. Selecting a tab reopens it.
Workspace changes restore that workspace's last preference. The tab strip reads
selection one way; explicit activation writes it. Binding refreshes cannot
silently reopen a collapsed workspace. No viewport reparenting is involved.

# PROGRESSIVE DISCLOSURE

Common tasks need no understanding of every physical coordinate. Expanded
sections keep the full controls. Collapsing preserves pending text and stable
selection. Advanced overlays use a native-compatible Avalonia popup, matching
the existing popup configuration. Hidden presentation never recompiles track
geometry, restarts physics or recreates expensive authoring snapshots.

# PRESETS / DEFAULTS

Existing editable native presets remain authoritative. Train starting points,
all six support families and all hardware-family defaults remain accessible.
No manufacturer branding, new physical preset family or alternate asset
pipeline was introduced. Placeholder shells remain identified as placeholders.

# AUTHORING VS RUNTIME

Authored fields are editable and grouped with their Apply actions. Runtime
state uses read-only TextBlocks on a separate dock page, with dispatch,
transfer, access and availability commands. Reset restores authored initial
runtime state. No transient simulation state is added to document serialization.

# APPLY / DIRTY STATE

Car and train defaults have separate modified indicators and Revert actions.
Hardware exposes pending configuration/placement/profile state even when those
groups are collapsed. Each Apply commits its existing native transaction and
resets preview. Successful Apply preserves text in the other hardware groups,
including unfinished or invalid inputs, without validating those other groups.
Revert all hardware fields restores the accepted snapshot without a history edit.

Transfer destination Apply retains its existing combined destination/configuration
commit and is labelled accordingly. Rejected hardware edits retain the existing
accepted-form restoration policy and show an error. Rejected Train proposals
retain the pending form and native revision. Document revision/object selection
changes still reseed forms according to the existing authoring publication path;
this milestone does not add draft storage across document replacement.

# LAYOUT PERSISTENCE

`%LOCALAPPDATA%/QUANTUM/workbench-layout.json` stores workspace, per-workspace
browser/Inspector widths, dock height, selected page, collapsed state and
section choices. It is versioned, bounded on read and replaced via a temporary
file at confirmed clean close. Missing/corrupt files recover to defaults with
a readable error. It contains no document identities, authored values or
simulation state. Smoke checks use an isolated report-side settings file.

# RESET LAYOUT

View → Reset Workspace Layout restores the current workspace's sizes, dock
page/default visibility and section policy. Other workspace preferences and
native selection/history remain untouched. Window minimum remains 1020×650;
splitter constraints reserve 300 pixels of viewport width and a 260-pixel main
workspace row. ScrollViews contain long editors. The smaller size is reasoned
from these constraints but was not reached by the final live smoke.

# KEYBOARD / FOCUS

Existing document shortcuts and Escape cancellation remain. Ctrl+T opens
Timeline through both Avalonia and SDL child input. Tab from the viewport now
targets the visible browser in Train/Operations; Track avoids focusing a hidden
length field while Track Start is selected. Section headers and splitters remain
ordinary focusable desktop controls. Physical keyboard/pointer input was not
tested; no accessibility certification or DPI sweep was performed.

# REGRESSIONS

Windows MSVC Debug baseline: full configured native build succeeded. Final
affected native target `QuantumNative` built successfully after shortcut routing.
Final managed Editor/Managed/Tests build: zero warnings/errors. Existing native
ignored-nodiscard warnings in the baseline were retained.

Commands from repository root:

```powershell
cmake --build build --config Debug --parallel 2
cmake --build build --config Debug --target QuantumNative --parallel 2
dotnet build editor/managed/Quantum.Managed.Tests/Quantum.Managed.Tests.csproj -c Debug --nologo
ctest --test-dir build -C Debug -R 'QuantumApplication\.(Train2|Supports2|TimelineAuthoring|RideHardwareAuthoring|SupportsWorkspace|TrackPlacement|NativeBoundary|HardwareBoundary|TrainBoundary|SupportBoundary|PlacementBoundary)$' --output-on-failure --parallel 2
ctest --test-dir build -C Debug -R 'QuantumApplication\.(NativeBoundary|TrainBoundary|HardwareBoundary|SupportBoundary|PlacementBoundary)$' --output-on-failure --parallel 2
& editor/managed/Quantum.Managed.Tests/bin/Debug/net10.0/Quantum.Managed.Tests.exe --verify-workbench-binding
& editor/managed/Quantum.Managed.Tests/bin/Debug/net10.0/Quantum.Managed.Tests.exe C:\Dev8\QUANTUM\smoke-tests\modern-steel-validation.quantum
```

Focused native regressions: 11/11; final native boundaries after shortcut change:
5/5. Managed suite includes existing Timeline, hardware, Supports, Supports 2,
Train 2, placement and wrapper/ABI coverage, plus **77 workbench checks** and three
separate real Avalonia dock-binding checks. Exact final workbench count is in
`build/ux-managed-tests.log`. Tests cover expansion/reset, workspace defaults,
dock/tab persistence, malformed settings, bounded sizes, context, authored/runtime
separation, stable selection/filtering, conservation of form cells, pending
state and scoped Apply/Revert behavior.

No full physics test campaign, Release build, Linux GUI testing, screenshots or
DPI sweep was repeated. Native numerical/geometry/rendering algorithms were not
changed. Automated results do not constitute maintainer UX approval.

# CLEAN SHUTDOWN

Baseline: existing Supports 2 smoke passed in 1.806 s, 27 Vulkan frames, process
exit 0. The earlier shutdown abort did not reproduce, so no lifetime fix was made.
Final single smoke: 0.732 s, 20 frames; Track at 1420×900 had an 860×340 viewport.
It failed the Train default-dock check, then closed viewport/controller/window
cleanly and exited 1. This is an acceptance failure, not a shutdown abort.
The saved smoke settings showed Train selecting Timeline and opening its dock;
one-way tab binding plus explicit activation fixes that cause. No second live
run was made. Reports are `build/ux-baseline-smoke.json` and
`build/ux-final-smoke.json`; the final report remains an honest failed record.

The close order remains: confirm edits → save layout → destroy hosted viewport
→ stop/join controller → dispose native session → close managed window.
Preferences introduce no Vulkan ownership or synchronization changes.

# KNOWN LIMITATIONS

- Full four-workspace live flow and 1020×650 live layout remain unverified after
  the bounded smoke failure. The corrected tab binding has automated coverage.
- Inspector views remain constructed/bound while collapsed; no lazy view/cache
  system was added. No native per-frame cost or solver policy was changed.
- Supports Diagnostics is the existing structure summary plus preview telemetry;
  Operations Diagnostics uses existing hardware telemetry. No new structural
  analysis/diagnostic solver was invented.
- Window bounds, overlay checkbox choices and every per-list scroll position
  are not persisted. Pane sizes, dock/page state and Inspector sections are.
- Layout saves on confirmed close; a crash can lose recent preference changes.
- Existing staging/rejection/publication policies above still apply. Support
  actions retain individual Apply boundaries; no universal form transaction
  layer was added.
- Production assets, physical input, DPI behavior and visual approval remain
  the maintainer's responsibility. Existing numerical/asset limitations persist.

# MAINTAINER UX TEST

1. Launch the staged Debug app. Visit Track; select Track Start, then a Region.
   Check the Inspector returns to geometry and Timeline stays reachable.
2. Visit Train. See Consist immediately and only common car settings. Expand
   Bogies, type a value, collapse, inspect Modified, Revert/Apply, then open
   Summary and Diagnostics. Try the viewport Overlays flyout.
3. Visit Supports using `smoke-tests/supports-2-structural-styles.quantum`.
   Select structure → Style, node → attachment, member → Section/Material.
   Select a catwalk/platform in Infrastructure and confirm context changes.
   Open the dock for frame generation.
4. Visit Operations; add/select LSM. Check authored target/direction, placement,
   profile mode, visible pending groups and independent Apply/Revert. Open Profile,
   then Runtime; collapse/reopen. Runtime fields must look read-only.
5. Resize splitters, switch away/back, close/relaunch and confirm preferences.
   Test Ctrl+T and Tab while native viewport has focus. Try Reset Workspace Layout.
   Check 1420×900 and 1020×650 for scroll/primary actions/useful viewport.

Judge whether Train is less intimidating, Supports is less cluttered, Operations
is understandable, advanced controls are findable and state persistence is natural.

# NEXT PHASE

After maintainer acceptance, recommend **Material System 2.0 — anti-tiling,
macro variation and deterministic per-object material variation**, unless the
acceptance exposes a more urgent usability defect. That phase was not started.

# OPTIONAL CODE READING PATH

| Priority | Location / smallest useful section | Understand |
|---|---|---|
| READ NOW | `editor/managed/Quantum.Editor/WorkbenchLayout.cs`: Defaults, SelectWorkspace, Load/Save | UI state lives outside coaster documents |
| READ NOW | `editor/managed/Quantum.Editor/MainWindow.axaml`: Workspace and BottomDock grids | Contextual composition around the existing viewport |
| READ NOW | `editor/managed/Quantum.Editor/InspectorSection.cs`: Restore and PropertyChanged | An ordinary Expander remembers one preference |
| READ NOW | `editor/managed/Quantum.Editor/EditorViewModel.Workbench.cs`: context, pending properties, commands | Presentation choices versus native document mutations |
| READ NOW | `editor/managed/Quantum.Editor/EditorViewModel.Train.cs`: SeedCar / PhysicalFields | Groups share the exact same form-cell objects |
| OPTIONAL | `editor/managed/Quantum.Editor/EditorViewModel.HardwareDraft.cs` | Text snapshots preserve other groups across publication |
| OPTIONAL | `editor/managed/Quantum.Editor/MainWindow.Layout.cs`: ApplyLayout / CaptureLayout | Desired size versus arranged size; explicit tab activation |
| OPTIONAL | `editor/managed/Quantum.Managed.Tests/WorkbenchChecks.cs` | Deterministic acceptance and real binding regression |
| OPTIONAL | `editor/native/NativeViewport.cpp`: SDL_EVENT_KEY_DOWN | Unsigned intent flags bridge child-window shortcuts |
| DEEP INTERNAL | `editor/managed/Quantum.Managed/SessionController.cs`: Dispose | Existing worker join/session lifetime; solver details can wait |

# LOCAL HANDOFF

Branch: `codex/workbench-ux-consolidation`. Starting HEAD:
`bf22974ca8109c7493bfc7f45d80aaa9e54b908d` on
`codex/train-2-configuration-preview`. Final local commit is reported in the chat
and available via `git log -1 --oneline`. No push or PR.

The maintainer's untracked material directories
`assets/materials/{concrete,fiberglass,grout,metal}/` remain untouched/untracked.
No source assets, fixtures, native serialization or build configuration changed.
The single C++ change is native Ctrl+T input forwarding; the rest is managed
presentation, tests and this focused documentation.

Build/test logs: `build/ux-baseline-native-build.log`,
`build/ux-baseline-managed-build.log`, `build/ux-final-native-build.log`,
`build/ux-managed-build.log`, `build/ux-native-regressions.log`,
`build/ux-final-boundary-tests.log`, `build/ux-managed-tests.log`,
`build/ux-dock-binding-tests.log`. Smoke exit codes and limitations are above.

Suggested PR title: **Consolidate workbench UX with contextual workspaces,
progressive Inspectors and persistent dock layouts**.

## Changed-file manifest

The manifest below is relative to the starting HEAD; generated build logs and
the maintainer's material assets are excluded.

35 changed or added files.

```text
M	docs/architecture.md
A	docs/workbench-ux-consolidation.md
M	editor/managed/Quantum.Editor/EditorApplication.axaml
M	editor/managed/Quantum.Editor/EditorApplication.axaml.cs
M	editor/managed/Quantum.Editor/EditorViewModel.Hardware.cs
A	editor/managed/Quantum.Editor/EditorViewModel.HardwareDraft.cs
M	editor/managed/Quantum.Editor/EditorViewModel.Infrastructure.cs
M	editor/managed/Quantum.Editor/EditorViewModel.Supports.cs
M	editor/managed/Quantum.Editor/EditorViewModel.Train.cs
A	editor/managed/Quantum.Editor/EditorViewModel.Workbench.cs
M	editor/managed/Quantum.Editor/EditorViewModel.cs
M	editor/managed/Quantum.Editor/HardwareInspector.axaml
A	editor/managed/Quantum.Editor/HardwareRuntimePane.axaml
A	editor/managed/Quantum.Editor/HardwareRuntimePane.axaml.cs
M	editor/managed/Quantum.Editor/InfrastructureInspector.axaml
A	editor/managed/Quantum.Editor/InspectorSection.cs
A	editor/managed/Quantum.Editor/MainWindow.Layout.cs
M	editor/managed/Quantum.Editor/MainWindow.axaml
M	editor/managed/Quantum.Editor/MainWindow.axaml.cs
A	editor/managed/Quantum.Editor/OperationsBrowser.axaml
A	editor/managed/Quantum.Editor/OperationsBrowser.axaml.cs
A	editor/managed/Quantum.Editor/RuntimeChecks.Workbench.cs
M	editor/managed/Quantum.Editor/SupportInspector.axaml
A	editor/managed/Quantum.Editor/SupportToolsPane.axaml
A	editor/managed/Quantum.Editor/SupportToolsPane.axaml.cs
M	editor/managed/Quantum.Editor/SupportsBrowser.axaml
M	editor/managed/Quantum.Editor/TimelinePane.axaml
M	editor/managed/Quantum.Editor/TrackLayoutInspector.axaml
M	editor/managed/Quantum.Editor/TrainBrowser.axaml
M	editor/managed/Quantum.Editor/TrainDiagnosticsPane.axaml
M	editor/managed/Quantum.Editor/TrainInspector.axaml
A	editor/managed/Quantum.Editor/WorkbenchLayout.cs
M	editor/managed/Quantum.Managed.Tests/Program.cs
A	editor/managed/Quantum.Managed.Tests/WorkbenchChecks.cs
M	editor/native/NativeViewport.cpp
```
