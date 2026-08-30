# Current Architecture and Mathematics

This document describes the QUANTUM repository as it exists now. It separates
implemented behavior from planned coaster-design, physics, and Editor systems.

## Target boundaries

The repository currently builds three primary targets plus Core tests.

### `QuantumCore`

`QuantumCore` is a static library containing the current curve mathematics,
analytic transition profiles, authored-track document structures and editing
operations, document serialization, topology operations, and rider-local
geometry integration. Its third-party target dependencies are GLM and
nlohmann JSON. It is independent of Vulkan, SDL3, VMA, Dear ImGui, and Editor
code.

### `QuantumEngine`

`QuantumEngine` is the Vulkan renderer library. `VulkanContext` owns the
current Vulkan instance, presentation surface, selected device and queues,
VMA allocator, swapchain, viewport line pipeline, viewport-aid and dynamic
track-curve buffers, offscreen viewport color/depth attachments, command
resources, and synchronization resources. SDL3 supplies the native window and
Vulkan surface integration.

`QuantumEngine` does not currently link to `QuantumCore`.

### `QUANTUM` Editor

The `QUANTUM` executable contains the application lifetime and Editor UI. It
links `QuantumCore`, the renderer, SDL3, and Dear ImGui. `Application` owns the
committed `AuthoredTrack` and coordinates document commands, Core generation,
renderer uploads, and editor-visible state. `EditorUi` owns interaction state
such as the selected region, selected profile segments, drag state, and numeric
edit buffers; it does not own the authored document.

```text
QuantumCore                         QuantumEngine
  |-- GLM                              |-- Vulkan
  +-- nlohmann JSON                    |-- VMA
                                      +-- SDL3 platform integration
        \                              /
         +-------- QUANTUM Editor ----+
                    |-- Application orchestration
                    |-- Dear ImGui editor state
                    +-- SDL3 application/window lifetime
```

`QuantumCore` and `QuantumEngine` do not depend on each other. The Editor is
the composition root that translates Core-generated, double-precision solved
states into renderer line vertices.

The current authored-to-rendered data flow is:

```text
AuthoredTrack
  |-- RateProfileRegion -- analytic ChannelProfiles --+
  |                                                   |
  +-- GeometryRegion ---- PlanarArc compilation ------+
                                                      v
                                  continuous QuantumCore solve
                                                      |
                                                      v
                            CenterlineVisualization + section slices
                                                      |
                                                      v
                                      Vulkan track-curve vertex buffer
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

## Authored track document

### Ordered model and local domains

`quantum::coaster::AuthoredTrack` is the ordered authored coaster document used
by the Editor. It owns the layout intent (`Circuit` or `Shuttle`) and an ordered
sequence of `AuthoredTrackSection` values. The UI calls these ordered values
regions; the Core type retains the established section name. A region's place
in the track is its vector position, and its length contributes to cumulative
whole-track stationing.

Each `AuthoredTrackSection` owns one positive finite length and one authored
region payload. That stored length defines the canonical section-local distance
domain `[0, length]`; the selected region's editor and the Core solver use the
same domain. Structural operations append, prepend, insert, duplicate, remove,
and reorder regions by value. Duplicated rate profiles share no mutable state,
and removing the final region is refused. A freshly created Editor document
contains one neutral 60-unit Rate/Profile region. Direct default construction
can produce an empty `AuthoredTrack` for assembly and tests, but whole-track
generation rejects an empty track.

The current JSON document format serializes the layout mode and every authored
region. Deserialization constructs a new document and accepts it only after
Core validation; it does not partially mutate an existing document on failure.

### Implemented region kinds

The authored region variant currently has two kinds:

- `RateProfileRegion` owns the Roll, Pitch, and Yaw angular-rate profiles used
  directly by the coupled rider-local solver.
- `GeometryRegion` currently contains one designer-facing construction,
  `PlanarArcRegion`, authored by radius, swept angle, plane tilt, and bank
  change.

For a Planar Arc, `|sweptAngle| * radius` must equal the stored section length.
A radius edit preserves stored length and turn direction by changing the sweep;
a sweep edit defines a new stored length; plane tilt and bank edits preserve
length. Core compiles the arc's centerline-driving curvature into constant
Pitch and Yaw rate transitions and sends those through the shared rider-local
solver. Bank is then applied about each solved tangent so it changes rider-frame
orientation without moving the authored planar centerline.

Conversions between the two region kinds preserve the authored length.
Converting a Planar Arc to Rate/Profile is exact for an unbanked arc and is
rejected for a banked arc rather than silently changing its geometry. A mixed
track may contain both kinds in any order.

`integrateAuthoredTrack` validates every region and chains their solves. Each
region starts from the previous region's final position and frame, cumulative
distance increases across the whole track, and a shared boundary sample appears
once. Consequently, an edit to an earlier region can change every downstream
region's world-space pose without changing their authored local data.

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

`GeometricSection` groups the Roll, Pitch, and Yaw `ChannelProfile`s of one
Rate/Profile region. Each channel is an ordered chain of stable-id
`ProfileSegment`s, each embedding one analytic `ScalarTransition`. All three
channels cover the same section-local domain `[0, length]` exactly. A channel
has no gaps or overlaps, adjacent pieces share an identical endpoint value for
C0 continuity, and slope discontinuities at semantic boundaries remain
authored curvature/rotation-rate steps. Segment ids remain stable across edits
and are not reused after removal.

Core owns the invariant-preserving profile operations. Splitting creates two
analytic pieces at the evaluated split value; removing merges a piece into a
neighbor and refuses to remove the final piece; moving an interior boundary
updates both adjoining domains; and editing a shared endpoint updates both
adjoining values. The outer boundaries stay pinned to `0` and `length`.
Resizing a Rate/Profile region proportionally rescales every segment boundary
while preserving endpoint values, transition types, and ids.

The obsolete standalone `ForceSection` placeholder has been removed. Rider
loads are universal diagnostics over canonical track kinematics rather than an
authored region kind; force-target authoring is not implemented yet.

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
- constant local roll-rate integration;
- `ScalarTransition`-authored variable local roll-rate integration; and
- simultaneous `ScalarTransition`-authored roll/pitch/yaw rate integration,
  including a breakpoint-aware variant over multi-segment authored
  `ChannelProfile`s.

The separate roll-rate integrations are implemented and tested. With zero
pitch and yaw, roll changes the lateral/up orientation while leaving the
centerline tangent and straight-line position unchanged.

The simultaneous pitch/yaw solver evolves one orientation rather than
combining independently accumulated Euler angles. Variable-profile frame
evolution is performed on SO(3) with a fourth-order, two-node Gauss-Legendre
Magnus increment and quaternion rotation composition. Position is integrated
from the evolving tangent with matching high-order quadrature and private
refinement independent of the requested output sample spacing. The full
roll/pitch/yaw entry point uses the same coupled approach with three-channel
rate vectors.

For multi-segment `ChannelProfile` inputs, the union of all segment boundaries
splits the section into spans inside which every channel behaves as a single
constant-type transition; each span is integrated by the identical
single-transition solver and spans chain through their shared endpoint states.
One-segment channels reproduce the single-transition output exactly, and C0
continuity keeps frames continuous across curvature-step breakpoints. The
returned sample grid restarts at every profile breakpoint.

### Full rate-system convention

The verified sign convention for the implemented simultaneous pitch/yaw/roll
coupling is:

```text
T' =  yL - pU
L' = -yT + rU
U' =  pT - rL

omega = (r, p, y)
```

Here `p`, `y`, and `r` are local pitch, yaw, and roll rates with respect to
distance. This convention is implemented by the coupled roll/pitch/yaw
solver and exercised by the authored-track centerline generation, which
chains every section's authored channels through that solver.

## Universal track kinematics and rider loads

`TrackKinematicState` is the construction-independent generated state used by
physics and diagnostics. It adds world-space centerline curvature `dT/ds` to
the cumulative coordinate-unit distance, position, and rider frame. Rate/Profile
regions produce curvature from `yL - pU`; Planar Arc regions use their analytic
fixed-plane circular curvature. Authored bank rotates the rider frame without
changing the Planar Arc curvature vector.

`integrateAuthoredTrackKinematics` chains these states over the whole authored
track. An internal shared section boundary is represented once and belongs to
the following section for curvature; the final endpoint belongs to the final
section. Position and frame stay continuous even when curvature jumps.

`evaluateRiderLoads` consumes only canonical kinematics. Core coordinates stay
unit-neutral, while `RiderLoadEvaluationSettings::metersPerCoordinateUnit`
provides the explicit conversion to SI. The first speed model is point-mass,
gravity-only energy propagation from one initial speed at track distance zero.
Loads are mass-independent specific force projected onto `(U, L, T)` and
reported in standard G. Materially negative speed squared terminates the
history with an explicit unreachable state; launches, losses, brakes, lifts,
and train-length effects are not part of this milestone.

The Editor evaluates that Core pipeline once for the committed whole authored
track, then `RiderLoadDiagnosticsModel` maps the returned cumulative distances
onto the selected section by subtracting its authored prefix length. The exact
Core sample at a shared boundary is retained in both adjacent section views,
so its right-continuous load value is never averaged or recomputed. The
read-only Force Diagnostics window presents Normal G, Lateral G,
Longitudinal G, and Vehicle Speed over the same section-local distance domain
for Rate/Profile and Geometry regions. If evaluation becomes energetically
unreachable, only valid earlier samples are retained and the stop location is
reported explicitly.

Until document-level simulation settings exist, the diagnostic path uses one
visible editor-owned development configuration: 20 m/s initial speed,
1 metre per Core coordinate unit, 0.75-unit kinematic sample spacing, and
standard gravity. These values are diagnostic configuration, not authored
force targets.

## Editor authored-track workflow

The current Editor opens a resizable SDL3 Vulkan window with an intentional,
persistent Dear ImGui workspace. A main menu and provisional COMMAND toolbar
sit above the dockspace. The default docked shell contains `TRACK WORKSPACE`,
the texture-backed `3D Viewport`, `Support Workspace`, and the detailed editor
appropriate to the selected region.

### Shared region selection

`EditorUi` owns one selected region index shared by the authored-track views.
The Section List and viewport picking both update it through the same selection
path. A Rate/Profile selection makes the Transition Editor operate on that
region; a Geometry selection presents the Geometry Editor for its Planar Arc
parameters. The same selection chooses the section slice highlighted across
all visible viewport reference curves.

Structural commits explicitly transform or replace the selected index so the
selection follows the intended region after insertion, duplication, removal,
or movement. Changing selection resets region-specific transient graph state
and refreshes numeric edit buffers from the selected committed region. A pure
selection change does not regenerate track geometry.

The Track Workspace Section List is therefore an active authored-document view,
not a placeholder. It displays the ordered region kinds and lengths, exposes
typed Rate/Profile and Planar Arc creation and conversion, and issues structural
and length-edit intent. Core remains responsible for accepting the resulting
document mutations.

### Transition Editor

The Transition Editor is a distance-domain authoring surface for the selected
Rate/Profile region. It is not a generic stack of unrelated example graphs.
Roll, Pitch, and Yaw are the three actual authored channels, aligned on one
shared horizontal domain `[0, selectedSection.length]` and one ruler. Each
channel has an independent vertical presentation range, while all X positions
refer to the same section-local distance.

The graph samples analytic transitions only for drawing and hit testing.
Editable markers correspond to semantic authored boundaries: an N-segment
channel exposes N+1 markers, with an interior marker representing the shared
boundary rather than a sampled curve point. The active channel's selected
segment or endpoint can be edited numerically or by dragging. The current
authoring path supports endpoint values, all 19 transition presets, segment
split and removal, and horizontal dragging of interior boundaries. The outer
region boundaries remain pinned. Optional value and distance snapping is
applied to user proposals; selecting or loading data never quantizes the
document.

Core stores rates in radians per coordinate unit; the current Editor presents
angular rates in degrees per metre. Pitch and Yaw selections show signed
curvature and radius diagnostics, their vector magnitude provides the local
resultant centerline curvature/radius, and Roll shows integrated region
rotation. Net Roll/Pitch/Yaw rotation summaries use the analytic integrals of
the authored segments. These diagnostics are derived views and do not mutate
the document.

### Geometry Editor

The Geometry Editor operates on the selected Planar Arc region and presents
radius, swept angle, plane tilt, and bank change. It emits editor intent in
designer-facing units, converts displayed degrees to Core radians, and relies
on Core for the length policies, validation, and solve described above. It is
an alternative detailed editor for a Geometry region, not a second view of a
Rate/Profile graph.

### Multi-region viewport visualization and picking

`createCenterlineVisualization` asks Core to solve the entire authored track at
the current 0.75-unit viewport sample spacing and retains the resulting
cumulative-distance, position, and frame states. The sample count therefore
depends on the authored lengths and profile breakpoints rather than being a
fixed fixture count. It derives four continuous line-list reference curves from
those states: left rail, right rail, centerline, and heartline. The rail gauge
and heartline offset are temporary visualization constants; these lines are not
final rail meshes or authored track-style geometry.

The visualization also derives one `CenterlineSectionSlice` per authored region
from cumulative section boundaries. Each slice records its range inside every
curve run plus bounds and entry/exit metadata. The renderer receives float line
vertices and the per-curve run size; the Editor retains the double-precision
solved states and slices for picking, camera tools, summaries, and selection.

Viewport picking tests the visible reference-curve segments within those exact
section slices. Hits are resolved front-to-back with deterministic tie breaks,
and empty space preserves the current selection. The renderer highlights the
selected slice by drawing that range again over every visible curve; selection
does not rewrite the geometry buffer. Accepted authored geometry or structure
edits regenerate the whole continuous visualization and dynamically update the
Vulkan track-curve buffer. An Editor-owned orbit camera supplies the
view-projection matrix and supports navigation and region/whole-track framing.
Viewport resize only recreates the offscreen color/depth target.

### Authored edit transactions

Authored document edits use the following acceptance and publication invariant:

```text
editor intent
    -> candidate AuthoredTrack
    -> Core mutation / validation
    -> solved/generated geometry
    -> GPU update
    -> committed document
    -> synchronized editor-visible state
```

`AuthoredTrackEditTransaction` owns a provisional candidate copied from the
committed document. The application applies editor commands only to that
candidate, then asks Core to validate it as part of generating the continuous
visualization. The candidate document does not become authoritative merely
because a local mutation succeeded.

The transaction also stages editor-visible effects that are consequences of
the edit, including the selection computed for a structural change and requests
to resynchronize numeric buffers. Staged selection is publishable only after
`commit` moves the accepted candidate into the application-owned document. On
rejection during mutation, Core solve, visualization generation, or GPU upload,
the committed document and selection remain unchanged; provisional viewport
bounds/slices are restored, and affected section-length, profile-value, and
Planar Arc buffers are refreshed from committed document state.

Editor-visible effects derived from an authored edit become authoritative only
after successful commit. This keeps the document, generated geometry, GPU
contents, selection, and numeric controls synchronized to one accepted state.

The transaction owns publication policy, not coaster mathematics. Core mutation
and validation functions remain authoritative for section domains, profile
continuity and ids, Planar Arc constraints, and generated geometry. Renderer
acceptance is part of the commit gate because the Editor must not publish a
document whose visible GPU representation failed to update.

## Planned systems

The following are current roadmap areas, not implemented capabilities or fixed
architectural commitments:

- force-based section solving;
- velocity, acceleration, and G-force evaluation;
- launches, brakes, and lift systems;
- authored track-style, rail, heartline-offset, and final rail meshing systems;
- direct deformation or control-point editing in the 3D viewport;
- supports, foundations, and rail connectors (the Support Workspace remains an
  unfinished disabled shell); and
- train and ride simulation.

No compatibility with another coaster-design application's file formats,
source code, or architecture is claimed.

## Verification

`tests/` contains CTest-registered coverage for the curve and rider-local
mathematics, authored document structure and persistence, channel-profile
editing, mixed-region authoring and generation, transaction rejection,
Transition Editor model semantics, centerline visualization, viewport picking,
and selection mapping described above. The tests are designed to run in both
Debug and Release configurations. Exact historical assertion or test-group
counts are intentionally omitted because they change as the Core develops.
