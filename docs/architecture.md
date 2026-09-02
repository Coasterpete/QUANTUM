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
VMA allocator, swapchain, viewport and track pipelines, renderer-side static
mesh cache, GPU mesh allocations, viewport-aid and dynamic track/instance
buffers, offscreen viewport color/depth attachments, command resources, and
synchronization resources. SDL3 supplies the native window and Vulkan surface
integration. It consumes renderer-neutral generated geometry and asset
references from `QuantumCore`; Core does not depend on the renderer.

The deliberately small Blender/GLB contract and asset lifetime boundary are
documented in [`static-mesh-assets.md`](static-mesh-assets.md).

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
             \                         |-- nlohmann JSON (GLB metadata)
              +----------------------->+-- QuantumCore renderable types
                                      +-- SDL3 platform integration
                                       /
                    QUANTUM Editor ---+
                    |-- Application orchestration
                    |-- Dear ImGui editor state
                    +-- SDL3 application/window lifetime
```

`QuantumEngine` consumes renderer-neutral renderable types from `QuantumCore`;
the dependency never points back into the renderer. The Editor remains the
composition root that coordinates authored state, Core generation, renderer
uploads, and UI state.

The current authored-to-rendered data flow is:

```text
AuthoredTrack
  |-- AuthoredStartPose -- world position + rider orientation
  |-- TrackPhysicalSettings -- speed, physical scale, gravity
  |-- RateProfileRegion -- analytic ChannelProfiles --+
  |                                                   |
  +-- GeometryRegion ---- PlanarArc compilation ------+
                     +-- ForceDriven coupled solve --+
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
by the Editor. It owns the layout intent (`Circuit` or `Shuttle`), one global
`AuthoredStartPose`, `TrackPhysicalSettings`, one `TrackStylePreset`, and an
ordered sequence of `AuthoredTrackSection` values. The track style includes the
logical repeating-hardware asset identity and its spacing, phase, and local
transform; renderer cache entries and GPU handles are not document state.
The start pose stores a world position and normalized quaternion that rotates
the canonical local `(T, L, U)` axes into the initial rider frame. The UI calls
the ordered section values regions; the Core type retains the established
section name. A region's place in the track is its vector position, and its
length contributes to cumulative whole-track stationing.

Each `AuthoredTrackSection` owns one positive finite length and one authored
region payload. That stored length defines the canonical section-local distance
domain `[0, length]`; the selected region's editor and the Core solver use the
same domain. Structural operations append, prepend, insert, duplicate, remove,
and reorder regions by value. Duplicated rate profiles share no mutable state,
and removing the final region is refused. A freshly created Editor document
contains one neutral 60-unit Rate/Profile region. Direct default construction
can produce an empty `AuthoredTrack` for assembly and tests, but whole-track
generation rejects an empty track.

The current JSON document format serializes the layout mode, authored start
pose, physical settings, track style, and every authored region. Track-hardware
assets remain package-relative logical IDs such as `assets://track/...`.
Documents that predate the `startPose` field load with the original
origin/identity pose, and documents that predate `trackStyle` load the standard
dual-rail preset. Deserialization constructs a new document and accepts it only
after Core validation; it does not partially mutate an existing document on
failure.

### Implemented region kinds

The authored region variant currently has two kinds:

- `RateProfileRegion` owns the Roll, Pitch, and Yaw angular-rate profiles used
  directly by the coupled rider-local solver.
- `GeometryRegion::construction` is a variant of `PlanarArcRegion` (radius,
  swept angle, plane tilt, and bank change) and `ForceDrivenRegion` (Normal G,
  Lateral G, and roll-rate profiles). There is no separate Force region kind.

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

`integrateAuthoredTrack` validates every region and chains their solves. The
document-owned start pose is the single canonical initial world position and
frame; each region thereafter starts from the previous region's final position
and frame. Cumulative distance increases across the whole track, and a shared
boundary sample appears once. Changing the start pose regenerates the complete
track without rewriting region-local construction data. Translation preserves
relative-height energy. Rotation relative to world gravity can change the
shape of force-driven geometry itself; it is not necessarily a rigid rotation.
An earlier region edit can change every downstream region's world-space pose.

### Force-driven region foundation

`TrackPhysicalSettings` is the document's canonical physical input: initial
speed in m/s, positive metres per Core coordinate unit (`mu`), and positive
gravity magnitude in m/s². All must be finite; speed is nonnegative and its
square must be representable. Defaults are 20 m/s, unit scale, and standard
gravity (9.80665 m/s²), preserving previous editor diagnostics for legacy files.
`RiderLoadEvaluationSettings` is constructed from these inputs for evaluation;
it is not a second authored settings object. Sampling/refinement controls are
numerical caller settings and are not persisted.

`ForceDrivenRegion` owns three `ChannelProfile`s: dimensionless `targetNormalG`
and `targetLateralG`, plus `rollRate` in radians per Core coordinate unit.
Each covers `[0, section.length]`. Length edits rescale every segment boundary
while preserving values, transitions, IDs, allocator state, and ordering.
`createForceDrivenSection` provides the model construction path (+1 normal G,
zero lateral G and roll). Conversion to Planar Arc replaces the construction
while preserving length; conversion to state-independent rate profiles is
rejected because it cannot preserve the state-dependent solve.

At every integration stage, using the provisional world position `P` and
right-handed rider frame `(T,L,U)`, Core evaluates:

```text
g = (0, 0, -gravityAcceleration), g0 = standardGravityAcceleration
w = initialSpeed² + 2 dot(g, mu * (P - wholeTrackStartPosition))
yawRate   =  mu * (g0 * targetLateralG(s) + dot(g,L)) / w
pitchRate = -mu * (g0 * targetNormalG(s)  + dot(g,U)) / w
rollRate  = authored rollRate(s)
P' = T; T' = yawRate L - pitchRate U
L' = -yawRate T + rollRate U; U' = pitchRate T - rollRate L
```

The focused forward integrator advances position and a normalized quaternion
together with fourth-order Runge–Kutta and step-doubling error control. It
compares normalized position and orientation errors, limits stage angular
steps, and fails if refinement or work limits are exhausted. The default error
tolerance is `1e-10` with at most 24 bisections per output/breakpoint interval.
Profiles are evaluated at their original local coordinates at all stages;
staggered breakpoints never reconstruct or re-ease another channel's nonlinear
transition. Existing Rate/Profile clipping behavior is unchanged.

Energy always references the whole-track start, including after Rate/Profile
or Planar Arc lead-ins. No region resets speed, freezes entry speed, or
integrates a separate drifting energy state. Generation and rider-load
evaluation share physical validation and the scale-aware energy tolerance
`64 * epsilon * max(1, initialSpeed², abs(gravityWork))`. Energy below the
negative tolerance is unreachable; force generation rejects zero and positive
energy unresolved within that tolerance without a minimum-speed clamp.
Internal stages and accepted endpoints are checked. Near a singular barrier,
refinement can fail before energy becomes negative; no continuation is emitted.

`generateAuthoredTrackKinematics` returns either a complete canonical track or
`TrackGenerationFailure`: invalid input, energetically unreachable,
insufficient speed, nonfinite derived rates, or integration/refinement failure.
Failures carry section index, local/cumulative distance, and speed squared
where available. Existing throwing entry points propagate `TrackGenerationError`
for force solve failures. This is separate from `RiderLoadUnreachableState`.

Targets are authored intent; `RiderLoadHistory` is evaluated truth. Generated
rates, geometry, speed samples, and loads are never stored in the construction
or document. Force output uses the same `TrackKinematicState` and right-owned
boundary curvature as other constructions. The unchanged universal evaluator
then computes actual speed and Normal/Lateral/Longitudinal G. Longitudinal
targets are deferred until propulsion, braking, and losses are modeled; the
current gravity-only point mass has zero longitudinal specific force apart
from numerical error.

Format version 1 is extended additively with root `physicalSettings`. A
`kind: "Geometry"` section requires exactly one `planarArc` or `forceDriven`
payload. Force channels reuse the existing ordered segments, IDs,
`nextSegmentId`, and transition JSON shape. Missing physical settings retain
the legacy defaults; malformed settings or ambiguous construction payloads
are rejected. Older application builds with strict parsers cannot read the
new fields; existing documents remain readable by this build.

Editable force-target UI and endpoint/anchor inverse solving remain deferred.
The Geometry Editor safely identifies force regions as read-only. A future
endpoint-constrained solver can vary authored targets/length/settings and call
the result-based forward generator to measure endpoint residuals; no inverse
solver or constraints are introduced here.

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
authored region kind; force targets are a Geometry construction's authored intent.

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

Diagnostics use the document's physical settings and display them read-only.
The editor retains its 0.75-unit kinematic sample spacing as output configuration.

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
region; a Geometry selection presents Planar Arc parameters or read-only Force
Driven identification. The same selection chooses the section slice highlighted across
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
and empty space preserves the current selection. The Editor overlays the
selected slice with thicker rails and square end caps (falling back to a visible
reference curve when rails are hidden), and gives hover a separate emphasis.
These image-clipped ImGui overlays do not rewrite the geometry buffer or add a
Vulkan rendering pass. Accepted authored geometry or structure
edits regenerate the whole continuous visualization and dynamically update the
Vulkan track-curve buffer. An Editor-owned orbit camera supplies the
view-projection matrix and supports navigation and region/whole-track framing.
Viewport resize only recreates the offscreen color/depth target.

Perspective starts at a 38-degree elevation with the existing 45-degree FOV.
Only initialization or an explicit Perspective preset chooses that elevation;
orbit, geometry updates, Frame All and Focus retain user orientation. Presets
that choose a projection also synchronize the authoritative viewport setting.
Axis presets keep the user's projection. Grid generation remains renderer-owned;
its CPU vertex builder is independently tested for translated X/Y centers.
The solved centerline bounds retain their original meaning. Separate display
bounds include the generated rail and heartline offsets for Frame All, Focus
and clipping, so those offsets cannot escape the frame on very short tracks.
They refresh after visualization changes without moving the user camera.

The Editor uses three static Overpass faces from the official v3.0.5 release:
Regular at 14 logical pixels, SemiBold headings at 15, and Mono Regular technical
values at 14. CMake deploys these with their license and provenance. ImGui's atlas
owns the fonts; EditorFonts only borrows handles until context shutdown. DPI
scales fonts and viewport drawing/picking dimensions together. Missing bundled
fonts retain an explicit startup error.

### Semantic viewport region anchors

The Editor also derives one semantic boundary anchor for every authored-track
boundary, so an N-region track exposes N+1 anchors. Anchor zero is the exact
initial authored-track pose, every interior anchor is the single shared pose
between the regions on both sides, and the final anchor is the exact terminal
pose. These anchors come from Core's chained kinematic integration endpoints;
they are independent of the 0.75-unit visualization sample grid and are not
spline control points or generated centerline samples.

Anchor selection is projected through the existing authoritative region
selection. Anchor zero selects region zero, an interior anchor selects the
following region, and the final anchor selects the final region. This is the
same right-continuous convention used at kinematic section boundaries. Section
List and reference-curve selections highlight the entry anchor of the selected
region, replacing any stale anchor-specific highlight. Screen-space picking
uses a fixed marker radius, gives an anchor hit priority over a track-curve hit,
and resolves overlaps by pointer distance, depth, then semantic anchor index.

The selected anchor displays its complete right-handed rider frame. Anchor zero
is editable through the compact viewport Move/Rotate gizmo: Move applies world
X/Y/Z translation and Rotate pre-multiplies a world-axis quaternion rotation.
The quaternion is normalized and sign-canonicalized before it enters the
document, so the rider frame remains orthonormal and satisfies `T x L = U`.
Dragging proposes a new global start pose; it never moves generated vertices or
rewrites region-local construction values.

Shared interior anchors remain read-only because moving one requires a future
constrained inverse solve across its neighboring regions. The final anchor also
remains read-only until a terminal pose constraint exists. Neither kind exposes
active manipulation handles.

### Authored edit transactions

Authored document edits use the following acceptance and publication invariant:

```text
editor intent
    -> candidate AuthoredTrack
    -> Core mutation / validation
    -> solved/generated geometry
    -> universal rider-load evaluation and acceptance
    -> GPU update
    -> committed document
    -> synchronized editor-visible state
```

`AuthoredTrackEditTransaction` owns a provisional candidate copied from the
committed document. The application applies editor commands only to that
candidate, then asks Core to validate it as part of generating the continuous
visualization. The candidate document does not become authoritative merely
because a local mutation succeeded.

Candidates containing force-driven regions must also complete universal load
evaluation before GPU upload. Failed generation or incomplete evaluation rejects
them without changing committed diagnostics, geometry, selection, or buffers.
Rate/Profile and Planar Arc-only candidates retain legacy unreachable-load
acceptance. Opening documents also prepares generation and load acceptance
before replacing the current document or uploading its candidate vertices.

The transaction also stages editor-visible effects that are consequences of
the edit, including the selection computed for a structural change and requests
to resynchronize numeric buffers. Staged selection is publishable only after
`commit` moves the accepted candidate into the application-owned document. On
rejection during mutation, Core solve, visualization generation, or GPU upload,
the committed document and selection remain unchanged; provisional viewport
bounds/slices are restored, and affected section-length, profile-value, and
Planar Arc buffers are refreshed from committed document state.

Start-pose dragging uses the same transaction on every changed drag frame for
live preview. Each accepted candidate regenerates semantic anchors, reference
curves, centerline camera bounds, and Force Diagnostics through the canonical
Core path before commit. A rejected candidate cancels the gizmo and returns it
to the committed Anchor 0 pose; continuous frames remain Trace-level and one
Info summary is emitted when a successful drag finishes.

Track-hardware asset and placement controls also edit the candidate
`AuthoredTrack`. Numeric drags publish live candidates and coalesce into one
history entry per gesture. Because history snapshots the complete authored
track, asset, spacing, phase, local position, local rotation, and local scale
changes participate in the same Undo/Redo and saved-baseline semantics as
region edits.

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

- editable force-target profiles and endpoint-constrained force solving;
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
