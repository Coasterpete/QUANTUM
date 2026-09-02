# QUANTUM

**An early-stage native roller-coaster design and simulation project focused on authored geometry, rider-local track construction, force-aware design, and an interactive 3D editor.**

> [!IMPORTANT]
> ## QUANTUM is in active early development
>
> QUANTUM is **not a finished coaster simulator**. Core authoring, geometry, diagnostics, document, and viewport foundations are functional, while major systems such as final track rendering, trains, supports, terrain, and production ride simulation remain future work.
>
> The UI, project format, rendering pipeline, and workflows may continue to change substantially before an initial release.

## What is QUANTUM?

QUANTUM is an independent roller-coaster design and simulation application written primarily in modern C++.

The project is being built around coaster-specific authoring concepts rather than treating an entire ride as one generic editable spline. Track is assembled from ordered authored **Regions** whose geometry and rider-local orientation are evaluated continuously over distance.

The long-term goal is a complete environment for:

- designing coaster layouts;
- authoring and refining track geometry;
- shaping roll, pitch, and yaw behavior over distance;
- designing around rider loads;
- visualizing track orientation and banking;
- adding final track styles, supports, terrain, trains, and ride systems;
- and eventually presenting complete simulated rides.

Detailed implementation notes and mathematical conventions live in [`docs/architecture.md`](docs/architecture.md).

---

# Current Development Status

QUANTUM has progressed from isolated geometry experiments into a connected interactive authoring application.

Recent work has focused on three areas:

1. **Connected authored-track editing** — multiple Regions form one canonical track and regenerate through QuantumCore.
2. **Force-aware geometry and diagnostics** — Force-Based construction and universal rider-load evaluation now share the same track pipeline.
3. **Editor modernization** — the interface has received multiple presentation passes covering terminology, hierarchy, typography, viewport framing, selection, anchors, and camera behavior.

The current editor uses:

- neutral charcoal application chrome;
- a near-black 3D viewport;
- Overpass for ordinary UI text;
- Overpass Mono for technical and numeric values;
- cyan/teal selection emphasis;
- amber hover and warning emphasis;
- red primarily for destructive and error states;
- a distance-domain Transition Editor with a dot grid;
- a compact viewport toolbar;
- semantic track anchors and an editable authored Track Start.

The current viewport remains an engineering/reference visualization rather than the final shaded coaster-track renderer.

---

# What Works Today?

## Editor and documents

- Native Windows desktop application
- SDL3 application/window layer
- Vulkan renderer
- Dear ImGui docking interface
- New / Open / Save / Save As document workflow
- Authored-track document state and serialization foundation
- Transaction-backed authored edits
- Ordered multi-Region track editing
- Selection preservation through accepted structural edits

## Viewport

QUANTUM currently includes these viewport modes:

- Perspective
- Isometric
- Top
- Bottom
- Left
- Right
- Track
- Walking

Current viewport interaction includes:

- orientation-aware **Frame All**;
- **Focus** on selected geometry;
- orbit and pan;
- deterministic authored-Region picking;
- distinct hover and selection presentation;
- semantic track-boundary anchors;
- Move / Rotate tools for the authored Track Start;
- DPI-aware anchor and gizmo presentation;
- ground/reference grid and track reference curves.

The authored Track Start stores canonical world position and orientation. Moving or rotating it regenerates the downstream track from authored state; it is not a renderer-only visual offset.

Interior and final semantic anchors remain read-only until constrained neighboring-Region and terminal-pose solving exists.

## Regions and geometry

QUANTUM currently supports multiple geometry-construction approaches inside the same authored track.

### Profile

Profile-based authoring uses distance-domain roll, pitch, and yaw rates to construct rider-local geometry.

Current channels include:

- Roll Rate
- Pitch Rate
- Yaw Rate

The Transition Editor provides direct profile editing over Region distance and includes multiple transition-function families.

### Circular Arc

Circular Arc Regions provide direct constant-curvature planar geometry using parameters such as radius and arc angle while remaining part of the same connected AuthoredTrack chain.

### Force-Based

Force-Based Regions generate track from authored target normal/lateral G behavior together with authored roll-rate behavior.

Force-Based generation uses the same canonical track state and physical settings as the rest of the authored track rather than introducing a separate incompatible force-only track representation.

## Rider-load diagnostics

QUANTUM includes construction-independent rider-load evaluation for authored tracks.

The current Force Diagnostics workflow can visualize actual:

- speed;
- normal G;
- lateral G;
- longitudinal G.

Load evaluation is derived from the canonical generated track and rider frame.

**Editable force-target authoring in the public editor is not yet complete.**

## Track topology

QUANTUM explicitly distinguishes layout topology.

### Circuit

Circuit layouts can be evaluated for:

- endpoint position gap;
- tangent mismatch;
- rider-frame mismatch;
- closed-circuit validity.

An experimental Circuit Completion solver can generate a connecting Profile Region and attempt to satisfy the closure constraints.

### Shuttle

Shuttle layouts are not required to return to the starting pose and therefore do not use ordinary Circuit closure semantics.

## Mathematical foundation

QuantumCore includes reusable numerical and geometric infrastructure including:

- 3D B-splines;
- NURBS;
- analytic first and second derivatives;
- curvature and radius helpers;
- adaptive arc-length integration;
- arc-length inversion;
- arc-length lookup tables;
- distance-domain curve sampling;
- rotation-minimizing frames;
- rider-local roll/pitch/yaw integration;
- whole-track kinematic evaluation;
- sensitivity-based Circuit Completion Jacobian support.

QuantumCore remains independent of Vulkan and Dear ImGui.

---

# Current Track Visualization

The current viewport intentionally visualizes track through reference geometry rather than pretending that final coaster-track assets already exist.

Reference elements include:

```text
Left Rail
Centerline
Right Rail
Heartline
```

The first static-mesh seam also loads the explicitly labeled Blender-authored
test crosstie fixture once and reuses it through the existing hardware
instancing path. Its limited authoring/import contract is documented in
[`docs/static-mesh-assets.md`](docs/static-mesh-assets.md); production track
artwork and materials remain future work.

Rail offsets follow the solved rider-local lateral direction, while the heartline follows rider-local up. This makes banking and frame orientation readable before final rail/spine/cross-tie meshes are implemented.

The data path remains conceptually:

```text
AuthoredTrack
      ↓
QuantumCore geometry / kinematics
      ↓
Editor visualization representation
      ↓
Vulkan rendering
      ↓
3D viewport
```

Final human-authored rail assemblies, material systems, shaded track rendering, and manufacturer/style-specific visual assets remain future work.

---

# What Does NOT Work Yet?

QUANTUM is still early-stage software. Major incomplete or planned areas include the following.

## Final track rendering

- Production rail/spine/cross-tie meshes
- Manufacturer-specific visual track styles
- Human-authored production material library
- PBR material workflow
- Production lighting and shadows
- Final shaded track renderer

## Supports

The **Supports** workspace represents future functionality. Automatic support generation, foundations, connectors, structural analysis, and production support workflows are not yet implemented.

## Terrain and environment

- Terrain authoring and sculpting
- Terrain grading
- Roads and paths
- Foliage
- Scenery and buildings
- Water
- Environment-lighting workflow
- Environmental effects

## Trains and ride systems

- Train/vehicle rendering
- Complete train dynamics
- Wheel/bogie simulation
- Multi-train operation
- Block systems
- Stations and dispatch logic
- Lift hills
- LSM/LIM launches
- Brakes and trims
- Transfer/switch tracks

## Advanced authoring still planned

- Editable force-target profile workflow
- Shared interior-anchor inverse solving
- Terminal/end-pose constraint solving
- Direct manipulation of constrained interior boundaries
- Complete ride-envelope/clearance workflow

## Distribution

- Production installer
- Stable public binary releases
- Stable project-format compatibility guarantees
- Maintained native Linux build
- Maintained macOS build

---

# Editor Architecture

QUANTUM keeps authored numerical state separate from presentation and rendering.

A simplified flow is:

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

- C++
- CMake
- Vulkan
- Vulkan Memory Allocator
- SDL3
- Dear ImGui
- GLM
- nlohmann/json
- vcpkg

QUANTUM currently targets Windows development first.

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

Example PowerShell workflow:

```powershell
git clone https://github.com/Coasterpete/QUANTUM.git
cd QUANTUM

$env:VCPKG_ROOT = "C:\path\to\vcpkg"

cmake --preset windows-msvc-debug
cmake --build build --config Debug --parallel 2
```

The Debug editor executable is normally produced under the configured build tree, for example:

```text
build/editor/Debug/QUANTUM.exe
```

The exact local Vulkan SDK and vcpkg locations depend on the developer machine.

---

# Running Tests

After configuring and building:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

The test suite covers Core geometry/math, authored-track behavior, topology, force-driven generation, rider loads, document state, transactions, viewport behavior, selection, anchors, typography/presentation, and Circuit Completion.

Some Circuit Completion numerical-validation tests are intentionally expensive and can dominate the total runtime of the complete suite.

---

# Development Direction

Near-term development is expected to continue along two complementary tracks.

## Authoring

- Force target vs. actual workflows
- Editable force-target profiles
- Interior/shared-anchor constraints
- Terminal pose constraints
- Support-generation foundations

## Visual / product

- Human-authored track visual assets
- Recolorable material controls
- Smoothness/roughness and other material parameters
- Non-repeating material variation for large surfaces
- Shaded track rendering
- Lighting, shadows, terrain, and environment presentation

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
