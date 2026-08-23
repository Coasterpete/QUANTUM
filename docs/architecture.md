# Current Architecture and Mathematics

This document describes the QUANTUM repository as it exists now. It separates
implemented behavior from planned coaster-design, physics, and Editor systems.

## Target boundaries

The repository currently builds three primary targets plus Core tests.

### `QuantumCore`

`QuantumCore` is a static library containing the current curve mathematics,
transition profiles, authored-section data structures, and experimental
rider-local geometry integration. Its only third-party target dependency is
GLM. It is independent of Vulkan, SDL3, VMA, Dear ImGui, and Editor code.

### `QuantumEngine`

`QuantumEngine` is the Vulkan renderer library. `VulkanContext` owns the
current Vulkan instance, presentation surface, selected device and queues,
VMA allocator, swapchain, viewport line pipeline and static geometry buffer,
offscreen viewport color/depth attachments, command resources, and
synchronization resources. SDL3 supplies the native window and Vulkan surface
integration.

`QuantumEngine` does not currently link to `QuantumCore`.

### `QUANTUM` Editor

The `QUANTUM` executable contains the application lifetime and Editor UI. It
links `QuantumCore`, the renderer, SDL3, and Dear ImGui. Editor-owned fixture
data uses the current Core rider-local geometry integration to generate the
centerline passed to the renderer.

```text
QuantumCore ---------------------- mathematics and coaster geometry
    |
    +-- GLM
    |
    +---------------------------------------+
                                            v
QUANTUM Editor <---------------- QuantumEngine
    |                                  |
    +-- SDL3                           +-- SDL3
    +-- Dear ImGui                     +-- Vulkan
                                       +-- VMA

QuantumCore does not depend on either consumer.
```

Within Core, authored transition profiles feed the experimental rider-local
geometry operations directly:

```text
TransitionFunctions
        |
        v
ScalarTransition
        |
        v
RiderLocalGeometry
```

The diagram is a description of current dependency flow, not a commitment to
additional architectural layers.

## Core curve foundation

### Curves and derivatives

`quantum::geometry::BSplineCurve` and `quantum::geometry::NurbsCurve` operate
on three-dimensional `glm::dvec3` control points. Both support double-precision
curve evaluation and analytic first and second derivatives over their inclusive
parameter domains.

The generic curve-geometry operations currently provide:

- `evaluateUnitTangent` from the analytic first derivative;
- `evaluateCurvature` from analytic first and second derivatives; and
- `evaluateRadiusOfCurvature`, including infinite radius for zero curvature.

Curvature uses inverse-coordinate units and radius uses the same units as the
curve coordinates. Core does not impose metres or another physical coordinate
unit.

### Arc length and sampling

`evaluateArcLength` integrates the magnitude of the analytic first derivative
over a curve interval. `evaluateParameterAtArcLength` inverts cumulative
distance to a curve parameter.

`ArcLengthLUT` is an adaptive, piecewise-linear acceleration structure for
parameter/distance queries. Direct arc-length evaluation remains the
authoritative mathematical path used to construct it. `sampleCurveByArcLength`
uses one LUT to produce the canonical sequence of distance, parameter, and
position samples at a requested geometric spacing.

### Frames and local rotations

Rotation-minimizing frames are built from canonical curve samples and analytic
unit tangents. The rider-local frame is right-handed:

```text
T = tangent / forward
L = lateral
U = up

T × L = U
```

Core also provides frame transforms for roll about `T`, local pitch about `L`,
and local yaw about `U`. Positive angles follow the current right-hand-rule
conventions implemented by `applyRoll`, `applyLocalPitch`, and `applyLocalYaw`.

## Transition profiles

`quantum::math::ScalarTransition` maps an authored scalar value from
`valueBegin` to `valueEnd` over an inclusive independent-variable domain.
`evaluateScalarTransition` evaluates the profile and
`integrateScalarTransition` evaluates its analytic area over a subinterval.

The underlying normalized functions and their analytic integrals are exposed
through `evaluateTransition` and `evaluateTransitionIntegral`. The 19 current
`TransitionType` presets are:

| Family | Presets |
| --- | --- |
| Linear | `Linear` |
| Smooth polynomial | `Smoothstep`, `Smootherstep`, `SeventhOrderSmoothstep` |
| Sinusoidal | `CosineEaseInOut`, `SineEaseIn`, `SineEaseOut` |
| Quadratic power | `QuadraticEaseIn`, `QuadraticEaseOut`, `QuadraticEaseInOut` |
| Cubic power | `CubicEaseIn`, `CubicEaseOut`, `CubicEaseInOut` |
| Quartic power | `QuarticEaseIn`, `QuarticEaseOut`, `QuarticEaseInOut` |
| Quintic power | `QuinticEaseIn`, `QuinticEaseOut`, `QuinticEaseInOut` |

`GeometricSection` and `ForceSection` currently group three
`ScalarTransition` channels and validate that their domains match. A
`GeometricSection` can also evaluate its authored pitch, yaw, and roll channel
values. Those values do not yet have a connected geometry-solver
interpretation, and force-section solving is not implemented.

## Rider-local geometry

Rider-local geometry operations advance a centerline position and the frame
`(T, L, U)` through a distance-domain section. Pitch, yaw, and roll inputs to
these operations are orientation rates measured in:

```text
radians per coordinate unit
```

They are rates with respect to traveled distance, not absolute Euler angles
and not time-domain angular velocities.

### Implemented integrations

The current public operations provide:

- constant local pitch-rate integration;
- `ScalarTransition`-authored variable local pitch-rate integration;
- constant local yaw-rate integration;
- `ScalarTransition`-authored variable local yaw-rate integration;
- simultaneous `ScalarTransition`-authored pitch/yaw rate integration;
- constant local roll-rate integration; and
- `ScalarTransition`-authored variable local roll-rate integration.

The separate roll-rate integrations are implemented and tested. With zero
pitch and yaw, roll changes the lateral/up orientation while leaving the
centerline tangent and straight-line position unchanged.

The simultaneous pitch/yaw solver evolves one orientation rather than
combining independently accumulated Euler angles. Variable-profile frame
evolution is performed on SO(3) with a fourth-order, two-node Gauss-Legendre
Magnus increment and quaternion rotation composition. Position is integrated
from the evolving tangent with matching high-order quadrature and private
refinement independent of the requested output sample spacing.

### Full rate-system convention

The verified sign convention reserved for future simultaneous pitch/yaw/roll
coupling is:

```text
T' =  yL - pU
L' = -yT + rU
U' =  pT - rL

omega = (r, p, y)
```

Here `p`, `y`, and `r` are local pitch, yaw, and roll rates with respect to
distance. This convention is already covered by the independent frame/rate
behavior and the coupled pitch/yaw solver, but the complete simultaneous
pitch/yaw/roll solver is not implemented.

## Editor status

The current Editor opens a resizable SDL3 Vulkan window with an intentional,
persistent Dear ImGui workspace. A main menu and provisional COMMAND toolbar
sit above the dockspace. The default docked shell contains `TRACK WORKSPACE`,
the texture-backed `3D Viewport`, `Support Workspace`, and `Transition Editor`.

The 3D Viewport displays a 241-sample centerline generated through
`QuantumCore`, a finite XY ground grid at Z = 0, and positive world X/Y/Z axes.
An Editor-owned orbit camera supplies the final view-projection matrix to the
renderer and supports orbit, pan, wheel dolly, and centerline framing. Vulkan
renders the static line geometry into VMA-owned offscreen color and depth
attachments; Dear ImGui displays the sampled color image in the central
workspace. Viewport resize recreates the attachments without regenerating or
re-uploading the static geometry.

The surrounding workspaces currently communicate intended workflow only:

- Track Workspace property controls and its Section List are placeholders;
- Support Workspace prefab and generation tools are placeholders;
- actual section authoring and section-list behavior are not implemented;
- support, foundation, and rail-connector generation are not implemented; and
- transition-profile editing is not implemented.

The Transition Editor currently demonstrates its timeline-based
authored-profile structure with one shared temporary domain `[0, 100]` and
three aligned, read-only `ScalarTransition` rows: Linear, Smoothstep, and
Smootherstep. The ruler, vertical grid, and all three sampled profiles use one
Editor-owned domain-to-pixel mapping. These mathematical examples are not a
fixed channel set or the final authored-section model, and timeline navigation
and editing remain future work.

## Planned systems

The following are current roadmap areas, not implemented capabilities or fixed
architectural commitments:

- complete simultaneous rider-local pitch/yaw/roll integration;
- connection of authored `GeometricSection` data to geometry solving;
- section chaining and force-based section solving;
- velocity, acceleration, and G-force evaluation;
- launches, brakes, and lift systems;
- heartline and track offsets;
- track visualization and meshing;
- interactive transition-profile editing; and
- train and ride simulation.

No compatibility with another coaster-design application's file formats,
source code, or architecture is claimed.

## Verification

`tests/` contains CTest-registered mathematical tests for the curve,
arc-length, sampling, frame, transition, authored-section, and rider-local
geometry behavior described above. The tests are designed to run in both
Debug and Release configurations. Exact historical assertion or test-group
counts are intentionally omitted because they change as the Core develops.
