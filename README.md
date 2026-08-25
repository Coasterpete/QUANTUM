# QUANTUM CoasterWorks

QUANTUM is an in-development native C++ roller-coaster design, simulation,
and visualization project. It is being built incrementally around a
renderer-independent, double-precision geometry core and a separate desktop
Editor based on SDL3, Vulkan, and Dear ImGui.

The project is not yet a complete coaster editor or ride simulator. Most of
the verified progress is currently in `QuantumCore`; the Editor remains an
early rendering and docking shell.

## Current status

### Implemented and verified

`QuantumCore` currently provides:

- double-precision B-spline and NURBS evaluation;
- analytic first and second derivatives;
- analytic unit tangents, curvature, and radius of curvature;
- arc-length evaluation and arc-length parameter inversion;
- an adaptive `ArcLengthLUT` and canonical distance-based curve sampling;
- rotation-minimizing frames and rider-local pitch, yaw, and roll frame
  transforms;
- `ScalarTransition`, normalized transition functions, analytic transition
  integrals, and 19 built-in transition presets;
- experimental rider-local centerline integration for separate constant and
  transition-profile pitch, yaw, and roll rates; and
- simultaneous pitch/yaw rate integration over a shared distance domain.

The Core mathematical behavior is covered by automated CTest targets intended
for both Debug and Release configurations.

### Editor today

The `QUANTUM` Editor executable currently provides:

- a resizable SDL3 window and application loop;
- Vulkan device, swapchain, dynamic-rendering, command, and synchronization
  setup;
- VMA-backed test geometry rendering;
- a Dear ImGui dockspace with layout persistence; and
- three temporary docking-verification panels over the existing cyan Vulkan
  test triangle and dark background.

The panels are placeholders. The Editor does not yet display Core-generated
coaster centerlines, edit transition profiles, author track sections, generate
track meshes, or run a ride simulation.

### In progress and planned

The next stages include connecting the mathematical Core to authored coaster
sections and to Editor visualization. Major areas that are not implemented yet
include:

- simultaneous rider-local pitch/yaw/roll integration;
- connecting `GeometricSection` data to geometry solving and section chaining;
- force-based section solving;
- velocity, acceleration, and G-force systems;
- launches, brakes, and lift systems;
- heartline and track offsets;
- track visualization and meshing;
- interactive transition-profile editing;
- Core-to-Editor centerline visualization; and
- train and ride simulation.

These are development directions, not finished features or promised release
dates.

## Architecture

The current targets have deliberately separate responsibilities:

```text
QuantumCore (GLM)
    renderer-independent mathematics and geometry

QuantumEngine (SDL3 + Vulkan + VMA)
    Vulkan renderer resources and frame submission
        |
        v
QUANTUM Editor (SDL3 + Dear ImGui)
    application lifetime and early docking UI
```

`QuantumCore` does not depend on Vulkan, SDL3, Dear ImGui, or the Editor. The
Editor is also not yet linked to `QuantumCore`; that connection belongs to a
future visualization milestone.

See [Current Architecture and Mathematics](docs/architecture.md) for the
target boundaries, mathematical conventions, transition families, and current
rider-local integration scope.

## Current technology

- C++23 and CMake
- GLM for Core mathematics
- SDL3 for windowing and platform integration
- Vulkan for rendering
- Vulkan Memory Allocator (VMA) for renderer-managed GPU allocations
- Dear ImGui for the early Editor UI

Windows is the current development platform. Other platform support remains a
future possibility.

## Building on Windows

Prerequisites:

- Visual Studio 2022 or later with the Desktop development with C++ workload
  (the debug preset uses the Visual Studio generator)
- CMake 3.25 or newer
- The [LunarG Vulkan SDK](https://vulkan.lunarg.com/sdk/home)
- A bootstrapped checkout of [vcpkg](https://github.com/microsoft/vcpkg) with
  the `VCPKG_ROOT` environment variable pointing at it

The repository ships a `vcpkg.json` manifest, so configuring the project
installs the required dependencies (SDL3 with Vulkan support, GLM, Vulkan
Memory Allocator, and Dear ImGui with docking and SDL3/Vulkan bindings) into
the build directory automatically. Configure and build with:

```sh
cmake --preset windows-msvc-debug
cmake --build --preset windows-msvc-debug
```

For a Release configuration, pass `--config Release` to `cmake --build`.
Run the automated tests with:

```sh
ctest --test-dir build -C Debug
```

The editor executable is written to `build/editor/<config>/QUANTUM.exe`.
