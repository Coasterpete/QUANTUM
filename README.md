# QUANTUM

**A native roller-coaster design and simulation application with authored geometry, rider-local track construction, force-aware design, multi-car train simulation, and an interactive 3D editor.**

> [!IMPORTANT]
> ## QUANTUM is in active development
>
> QUANTUM is **not a finished coaster simulator**. The application is a functional connected editor and simulation prototype. Authored multi-Region track construction, force diagnostics, multi-car train simulation with rigid-bogie physics, and a Modern Steel track visualization are all operational today.
>
> The UI, project format, rendering pipeline, and workflows may continue to change substantially before an initial release.

## At a Glance

QUANTUM is a native Windows desktop application for designing and simulating roller coasters. Unlike a generic spline editor, QUANTUM builds track from ordered authored **Regions** whose geometry and rider-local orientation are evaluated continuously over distance.

**What can be done today:**

- Author connected multi-Region tracks using Profile (roll/pitch/yaw rates), Circular Arc, or Force-Based geometry
- Edit transition profiles in a distance-domain Transition Editor
- Evaluate rider loads (normal G, lateral G, longitudinal G, speed) through Force Diagnostics
- Configure physical settings including authored initial speed
- Run fixed-step multi-car train simulation with rigid-bogie geometry and rollback behavior
- Visualize track with the Modern Steel style including configurable rail, spine, and crosstie hardware
- Author support structures (nodes, members, end connections) with track attachments
- Use document history (undo/redo) for all authored edits

**How early is it?**

QUANTUM is early-development software. The editor is functional but undergoing visual and UX modernization. There is no production installer, no stable binary release, and many planned systems remain unimplemented. The Vulkan renderer currently uses reference geometry visualization rather than production rail meshes.

**Recommended platform:** Windows with MSVC, Release build.

---

# Current Development Status

QUANTUM has progressed from isolated geometry experiments into a connected interactive authoring and simulation application. Recent work has focused on:

1. **Connected authored-track editing** — multiple Regions form one canonical track and regenerate through QuantumCore.
2. **Force-aware geometry and diagnostics** — Force-Based construction and universal rider-load evaluation share the same track pipeline.
3. **Multi-car train simulation** — a four-car train preview with rigid-bogie kinematics, signed velocity, and rollback behavior.
4. **Track presentation** — the Modern Steel configuration provides configurable rail, spine, and crosstie hardware instancing.
5. **Support authoring** — persistent support structures with nodes, members, track attachments, and end connections.
6. **Performance stabilization** — frame-pacing work including fixed-step simulation, deferred retained buffers, and GPU validation gating.

The current editor uses:

- neutral charcoal application chrome;
- a near-black 3D viewport;
- Overpass for ordinary UI text;
- Overpass Mono for technical and numeric values;
- chartreuse interaction and selection emphasis;
- cyan/teal coaster and technical viewport geometry;
- magenta/purple transition data where channel semantics call for it;
- amber technical hover and warning emphasis;
- red primarily for destructive and error states;
- a distance-domain Transition Editor with a dot grid;
- a compact viewport toolbar;
- semantic track anchors and an editable authored Track Start.

---

# What Works Today?

## Editor and documents

- Native Windows desktop application
- SDL3 application/window layer
- Vulkan renderer with offscreen viewport rendering
- Dear ImGui docking interface
- New / Open / Save / Save As document workflow
- Authored-track document state and serialization foundation
- Transaction-backed authored edits
- Ordered multi-Region track editing
- Selection preservation through accepted structural edits
- Document history with undo/redo (128-entry capacity, continuous-edit coalescing)

## Track construction

QUANTUM currently supports multiple geometry-construction approaches inside the same authored track:

### Profile

Profile-based authoring uses distance-domain roll, pitch, and yaw rates to construct rider-local geometry. The Transition Editor provides direct profile editing over Region distance with 19 transition-function presets.

### Circular Arc

Circular Arc Regions provide direct constant-curvature planar geometry using parameters such as radius and arc angle while remaining part of the same connected track chain.

### Force-Based

Force-Based Regions generate track from authored target normal/lateral G behavior together with authored roll-rate behavior. These share the same canonical track state and physical settings as the rest of the authored track.

### Track topology

QUANTUM explicitly distinguishes layout topology:

- **Circuit** layouts can be evaluated for endpoint position gap, tangent mismatch, rider-frame mismatch, and closed-circuit validity. An experimental Circuit Completion solver can generate a connecting Profile Region.
- **Shuttle** layouts are not required to return to the starting pose.

## Rider-load diagnostics

QUANTUM includes construction-independent rider-load evaluation for authored tracks. The Force Diagnostics workflow visualizes actual speed, normal G, lateral G, and longitudinal G over the selected region's distance domain.

## Physical settings

The document owns canonical physical settings: initial speed (m/s), physical scale (metres per coordinate unit), and gravity magnitude. These are editable in the editor and shared by track generation, rider-load evaluation, and simulation preview.

## Simulation preview

A fixed-step multi-car train simulation preview is available for diagnostic playback. The preview builds a four-car train with rigid-bogie geometry, supports signed velocity through zero (rollback), and renders wireframe car boxes, bogie markers, and connector lines in the 3D viewport. The simulation uses a 1/240-second fixed timestep with a bounded catch-up accumulator and interpolation.

## Multi-car rigid-bogie physics

The native physics foundation covers deterministic track-follower, single-car, and rigid multi-car train behavior through Phase 11. Implemented capabilities include:

- rigid bogie pose solving on compiled track geometry
- inter-car rigid connector kinematic constraints
- aggregate resistance (rolling, linear, aerodynamic)
- per-car explicit aerodynamic drag
- rigid connector axial-load recovery
- car-body rotational kinetic energy and inertia
- conditioned front/rear aggregate bogie reactions (Phase 9)
- rigid bogie contact geometry and wrench feasibility (Phase 10)
- unilateral rigid-contact allocation via NNLS (Phase 11)
- dynamic wheel/rail contact motion (M1A vertical single-car)

Signed velocity, stall, and rollback behavior are fully supported. The physics model is track-constrained reduced-coordinate — it is not a complete coaster operations simulation.

## Track visualization and hardware

The current viewport visualizes track through reference geometry: left rail, right rail, centerline, and heartline. The Modern Steel track configuration provides configurable rail, spine, and crosstie hardware with instance-buffer-based GPU rendering.

Track-style properties (rail visibility/radius, spine profile, hardware spacing/color, overall visibility) can be overridden per-region through the document's `TrackStylePreset`. Material-only changes update CPU draw data; mesh and hardware changes go through synchronized Vulkan buffer updates.

## Support structures

QUANTUM includes a Support Workspace for authoring support structures. The persistent support model consists of:

- Support structures containing nodes and members
- Node editing (position, track attachment, foundation marker)
- Member authoring (start/end node, profile: circular or rectangular, wall thickness)
- Member end connections with 9 treatment types (MiteredCut, EndCap, Plate, Flange, Splice, Saddle, Clamp, Base, Footing)
- Track attachments with cumulative station, lateral offset, and vertical offset
- Selection of structures, nodes, and members with viewport visualization
- All support edits participate in document undo/redo

## Viewport

Current viewport modes: Perspective, Isometric, Top, Bottom, Left, Right, Track, Walking.

Current viewport interaction includes: orientation-aware Frame All, Focus on selected geometry, orbit and pan, deterministic authored-Region picking, distinct hover and selection presentation, semantic track-boundary anchors, Move/Rotate tools for the authored Track Start, DPI-aware presentation, and ground/reference grid.

The authored Track Start stores canonical world position and orientation. Moving or rotating it regenerates the downstream track from authored state.

## Mathematical foundation

QuantumCore includes reusable numerical and geometric infrastructure: 3D B-splines, NURBS, analytic first and second derivatives, curvature and radius helpers, adaptive arc-length integration, arc-length inversion, arc-length lookup tables, distance-domain curve sampling, rotation-minimizing frames, rider-local roll/pitch/yaw integration, and whole-track kinematic evaluation.

---

# Known Limitations

- **Debug editor startup:** The current Windows Debug editor executable may terminate during startup while Release launches and operates correctly. This is a known GPU/Vulkan initialization reliability issue under investigation. Developers encountering Debug startup failure should use Release for normal interactive testing.
- **Frame pacing:** Idle viewport frame pacing is currently constrained by the two-slot Vulkan FIFO presentation fence. Average FPS is high but 1% lows can drop significantly during transitions and stalls. This is an active area of optimization.
- **No production renderer:** The viewport uses reference geometry visualization (line-list rails, centerline, heartline) rather than final shaded track meshes. Production rail/spine/cross-tie meshes, materials, and lighting are future work.
- **Experimental circuit completion:** The circuit completion solver is experimental and may not converge on all layouts.
- **No individual wheel loads:** The physics model provides representative rigid-contact allocations, not true individual wheel loads. Contact-resolved rolling, guide, and upstop forces are aggregate representatives.

---

# Editor Architecture

QUANTUM keeps authored numerical state separate from presentation and rendering. A simplified flow is:

```text
QuantumCore
     ↓
CoasterDocument / AuthoredTrack
     ↓
Editor interaction state
     ↓
Visualization data
     ↓
Vulkan renderer
```

The Editor is the composition root. QuantumCore does not depend on the renderer, and the Vulkan engine does not own coaster-authoring truth.

See [`docs/architecture.md`](docs/architecture.md) for the current subsystem boundaries, mathematical conventions, force-driven construction details, topology behavior, and editor transaction model.

---

# Technology

Current primary technologies include:

- C++ (C++23)
- CMake
- Vulkan + Vulkan Memory Allocator (VMA)
- SDL3
- Dear ImGui
- GLM
- nlohmann/json
- vcpkg

QUANTUM currently targets Windows development first. Linux Core-only builds are available via Clang and GCC presets.

---

# Building QUANTUM

QUANTUM is under active development and does not yet provide a fully automated clean-machine bootstrap.

A current Windows development environment requires at least:

- Visual Studio / MSVC with C++ desktop-development components;
- CMake;
- Git;
- vcpkg;
- a Vulkan SDK/toolchain suitable for shader compilation;
- the `VCPKG_ROOT` environment variable pointing at the local vcpkg installation.

## Release build (recommended for normal use)

Release is the recommended configuration for normal editor use and testing.

```powershell
git clone https://github.com/Coasterpete/QUANTUM.git
cd QUANTUM

$env:VCPKG_ROOT = "C:\path\to\vcpkg"

cmake --preset windows-msvc-debug
cmake --build build --config Release --parallel 2
```

The Release editor executable is produced under:

```text
build/editor/Release/QUANTUM.exe
```

## Debug build (for development and diagnostics)

Debug is used for development, diagnostics, and validation. The current Windows Debug editor startup path has a known GPU/Vulkan initialization reliability issue under investigation. Developers encountering Debug startup failure should use Release for normal interactive testing.

```powershell
cmake --build build --config Debug --parallel 2
```

The Debug editor executable is produced under:

```text
build/editor/Debug/QUANTUM.exe
```

The exact local Vulkan SDK and vcpkg locations depend on the developer machine.

---

# Running Tests

After configuring and building:

```powershell
ctest --test-dir build -C Release --output-on-failure
```

Release testing is a supported normal workflow. Debug testing is also valid:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

The test suite covers Core geometry/math, authored-track behavior, topology, force-driven generation, rider loads, document state, transactions, viewport behavior, selection, anchors, typography/presentation, Circuit Completion, train physics, car pose, simulation preview, support authoring, and track hardware.

Some Circuit Completion numerical-validation tests are intentionally expensive and can dominate the total runtime of the complete suite.

---

# Development Direction

Near-term development is expected to continue along several complementary tracks.

## Authoring

- Editable force-target profiles and endpoint-constrained force solving
- Interior/shared-anchor constraints and terminal pose constraints
- Expanded support authoring (gizmo movement, node snapping, procedural generation, final member meshes)

## Physics and simulation

- GPU-resident train simulation (Vulkan compute)
- Multi-car dynamic contact (M1B)
- Connector compliance, slack, and springs
- Per-car resistance and operational devices (brakes, launches, lifts)

## Visual / product

- Human-authored track visual assets and shaded track rendering
- Recolorable material controls and non-repeating material variation
- Lighting, shadows, terrain, and environment presentation
- Production rail/spine/cross-tie meshes and manufacturer-specific track styles

The exact order may evolve as architecture and usability testing continue.

---

# AI-Assisted Development

QUANTUM is a **human-directed** project developed with substantial use of modern AI-assisted programming tools.

AI coding agents may assist with work such as:

- implementation;
- testing;
- debugging;
- code review;
- documentation;
- repetitive integration work.

The project's product direction, architecture decisions, UX direction, artistic direction, visual assets/models, sounds, acceptance criteria, and final testing decisions remain human-directed.

AI assistance is treated as an implementation tool rather than a substitute for project ownership or design intent.

---

# Disclaimer

QUANTUM is experimental software under active development.

It is not a certified engineering, structural-analysis, ride-safety, or regulatory tool. Numerical and simulation output should not be treated as professional engineering approval or safety certification.

---

# License / Distribution

Licensing and public distribution policy are still evolving with the project.

Before using QUANTUM source or assets outside the repository, review the repository's current license and asset notices.
