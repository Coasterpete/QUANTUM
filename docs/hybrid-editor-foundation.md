# Hybrid editor foundation

QUANTUM has an opt-in Avalonia desktop frontend beside the existing C++/ImGui
editor. The ImGui editor remains the reference and fallback until engineering
workspace parity and human visual acceptance are approved.

Current QUANTUM is the feature baseline. Legacy is a read-only UX reference for
desktop workflow, pane hierarchy and forms; it does not define the feature scope.
Newer authoring, simulation and diagnostics must retain proper homes in the
hybrid workbench.

## Ownership

```text
Avalonia window, panes, selection, file pickers, dirty prompts
    -> EditorViewModel / SessionController
    -> Quantum.Managed (LibraryImport, copied snapshots, deterministic disposal)
    -> QuantumNative C ABI
    -> ApplicationService
    -> existing document / history / geometry / simulation
```

`ApplicationService` owns one authored document, native history, file state,
generated centerline/support presentation, and simulation preview. Both
frontends use its document preparation, geometry transaction commit, history,
file operations, and preview controls. Specialized ImGui workspaces retain their
existing edit intents and optimized GPU publication, then finish through the
service's accepted-edit/history path. Their migration is still pending.

Physics, the 1/240-second fixed step, Track Devices, geometry, validation,
serialization, GLB loading, supports, Vulkan resources, and numerical tolerances
remain native. C# has no document serializer or independent history.

The controller worker serializes file operations, document commands, simulation
updates, and snapshot production. An immutable copied snapshot is published to
the UI. Route arrays are rebuilt only when the native document revision changes;
simulation-only updates reuse them. Selection is UI state, identified by Region
index plus the snapshot revision. Stale edits return a conflict rather than
editing a different document. This does not introduce persistent Region IDs or
change the saved schema.

The revision-cached authoring snapshot also copies Region profile segments and
Track Devices. The native transition catalog is the existing 19-choice preset
table. Each segment includes 33 native analytic display samples. C# only projects
these samples onto graph coordinates; they are a display approximation, never
editable geometry or a second profile model. Angular rates are converted using
the native document coordinate scale; force targets remain G and device commands
remain SI acceleration magnitude over the device-local metre domain.

The UI thread owns the platform child and all viewport calls. Avalonia positions
and destroys that child. Native SDL wraps its HWND or X11 XID without owning the
platform handle, and the existing `VulkanContext` renders directly into it.
ImGui is used inside this path only to present the native viewport texture.
Pixels never travel through managed arrays. A single visible viewport is
supported per process because SDL events and the ImGui presenter are process
resources; multiple headless sessions are supported.

Close ordering is explicit: confirm unsaved changes, stop the viewport timer,
destroy native viewport resources, join the controller worker, destroy the
native session, and finally release Avalonia's child/window. GPU resources are
retired using the renderer's existing synchronization. Finalizers are a headless
safety fallback, not the application's shutdown mechanism.

## Interop contract

`editor/native/QuantumNative.h` defines ABI 2 for 64-bit processes with natural
8-byte alignment. It contains fixed-width values, caller-owned buffers, UTF-8
paths, non-reused numeric session tokens, explicit results, and bounded errors.
No C++ objects, Vulkan handles, or exceptions cross it. Snapshot size/version
and managed layout checks reject incompatible runtimes.

Document replacement prepares and validates a candidate before committing it.
Rejected edits and failed reads preserve the current document/history. Save
serializes natively, writes a unique sibling temporary file, and replaces the
destination before marking that exact history revision saved. This is not a
claim of crash-durable storage on every filesystem.

Viewport frames use nonblocking CPU-state acquisition. While an edit/load owns
the session mutex, the UI retains the last image. GPU waits occur after the CPU
lock is released. Pending native-child shortcuts survive a busy controller and
are delivered as managed command intents on a later frame.

## Build on Windows

Prerequisites are the existing MSVC/CMake/vcpkg toolchain, Vulkan driver, .NET 10
SDK, and the pinned Avalonia 12.1.3 packages. No new native dependency is added.
Set `VCPKG_ROOT` to the existing bootstrapped vcpkg checkout.

```powershell
# Native + managed; stages DLL, shaders and assets beside the apphost.
./editor/managed/Build.ps1 -Configuration Debug -Test -Run
./editor/managed/Build.ps1 -Configuration Release -Test

# UI-only change, using the already-built matching native runtime.
./editor/managed/Build.ps1 -UiOnly -Configuration Debug -Run

# Native service/boundary checks.
cmake --build build --config Debug --target QuantumNativeTests QuantumApplicationServiceTests
ctest --test-dir build -C Debug -R QuantumApplication --output-on-failure

# Optional native window/lifetime/resize validation, on the UI thread.
./build/editor/native/Debug/QuantumNativeTests.exe smoke-tests/modern-steel-validation.quantum --vulkan
```

Close the running app before rebuilding its staged DLLs. The native option
`QUANTUM_ENABLE_HYBRID_EDITOR` defaults OFF. The Windows helper enables it using
the existing CMake preset and also builds the fallback `QUANTUM` target.

## Build on Linux/X11

Use the existing full Linux preset and vcpkg checkout. The helper accepts an
already-configured build directory; it does not install toolchains.

```bash
cmake --preset linux-clang-full-debug -DQUANTUM_ENABLE_HYBRID_EDITOR=ON
bash editor/managed/Build-Linux.sh "$PWD/build/linux-clang-full-debug" Debug
ctest --test-dir build/linux-clang-full-debug -R QuantumApplication --output-on-failure
build/linux-clang-full-debug/managed/bin/Quantum.Managed.Tests/debug/Quantum.Managed.Tests "$PWD/smoke-tests/modern-steel-validation.quantum"
build/linux-clang-full-debug/managed/bin/Quantum.Editor/debug/Quantum.Editor
```

Launch the staged **apphost executable**. Launching the DLL through `dotnet`
changes SDL's executable base path and can make native assets resolve against
the runtime directory. A non-system .NET installation may require `DOTNET_ROOT`;
the host must supply the normal .NET native runtime dependencies and Vulkan
loader/driver. X11 is the supported hosting path in this phase. WSLg can verify
functionality without establishing native Linux GPU performance. Native Wayland
and macOS/MoltenVK integration are not implemented.

## Current user workflow and layout

New/Open creates or loads a real native document. Route lists actual authored
Regions. Selecting one displays its length, station range, endpoints and net
rotation. Length and initial speed are editable through validated native
commands. Undo/Redo uses native history. Play/Pause/Reset controls the native
train; the footer reports real tick/time, speed, station and playback state.
Save/Save As and close/replacement prompts use Avalonia UX with native I/O.

Route, Design View and Inspector have stable identities and resizable splitters.
Below them, a full-width resizable Timeline contains the distance-domain
Transition Editor, separately from simulation/status. It displays real authored
profiles and accepts endpoint/type edits, split/remove and interior-boundary
moves. Core keeps outer boundaries pinned and shared endpoints contiguous.
Rate and force-driven Regions use the existing native channel mapping; planar
arcs do not pretend to have scalar profiles. Track Devices mode shows authored
device state, Constant/Custom acceleration controls and native custom profiles.
Changing to Constant removes the custom profile through native history; Undo can
restore it. Device creation/deletion, interval/force/enable controls and direct
profile editing are now available in Avalonia. See
[Timeline and Track Device authoring](hybrid-timeline-track-devices.md) for the
gesture contract, native preview/commit behavior and validation handoff.

This phase deliberately uses a fixed pane arrangement. Floating docking and
layout persistence are not implemented. Native-window popups are configured
instead of managed overlays; menus, display-mode choices, tooltips and dialogs
use separate windows. Arbitrary managed overlays or drag adorners over the
native viewport are not supported by this design.

Viewport controls are right-drag orbit, middle-drag pan, wheel zoom, WASD move,
and F to frame the track. Product shortcuts from the child are forwarded to the
same commands as menus. Tab leaves the viewport for Inspector; Shift+Tab moves
to Route. Display modes are shaded, wireframe, shaded with wire, and reference
curves. Camera/display state is transient UI presentation, not authored data.
The currently bundled GLB shells and test hardware retain their existing
placeholder status; this change does not introduce final production assets.

The main window and Windows apphost use the canonical
`assets/icons/quantum/quantum.ico`. Its master and existing derivatives are
unchanged. Linux uses that resource as its window icon; a desktop-entry package
can reference the existing generated PNGs when packaging is implemented.

## Homes for the remaining current workflows

| Workflow | Hybrid responsibility and migration status |
| --- | --- |
| Timeline / Transition Editor | Native profiles, graph selection, value/boundary dragging, optional snapping, pan/zoom and numeric editing are available; full diagnostics parity remains. |
| Track Devices | Native create/delete/select, interval/force/enable, Constant/Custom, direct custom-profile editing and selected-interval viewport lines are available; live device-force diagnostics remain. |
| Train | Preserve Configuration → native Preview → Summary/Diagnostics as a dedicated workspace; the current footer summary is not the final Train UI. |
| Supports | Native document/support generation is retained; migrate authoring forms and existing 3D picking/manipulation in a dedicated workspace. |
| Simulator / Math / newer diagnostics | Extend managed workspace composition around the existing native viewport, snapshots and diagnostics; do not replace the native simulation with UI calculations. |
| Machinery and Lua ride logic | Future operations/ride sequencing belongs alongside Timeline; machinery physics remains native. Runtime/scripting APIs require their own concrete milestone and are not implemented here. |

These homes extend the current Grid/UserControl composition and concrete native
service. No docking registry, generic event bus, machinery framework or scripting
runtime is introduced to reserve future features.

## Verification and migration limits

`--verify-runtime <fixture> <report.json>` runs the actual live commands and
embedded host, including edit/history/file flow, simulation, resize,
minimize/restore, host recreation, dirty-choice branches and shutdown. Its file
picker/confirmation delegates are deterministic, so it does not verify physical
dialog input, camera gestures, popup airspace, or monitor DPI. Those require
separate visual and input checks. Automated correctness, bounded visual smoke,
and maintainer visual approval must be reported separately.

Full Train, Supports and Simulator/Math/diagnostics remain in the fallback
frontend. Direct Timeline and Track Device authoring is implemented locally;
physical interaction and visual approval remain with the maintainer. Supports
is the next major workspace after that approval. The current fixed pane layout
and the renderer, simulation and document ownership contracts remain intact.
