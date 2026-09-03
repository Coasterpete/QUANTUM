# Current Architecture and Mathematics

This document describes the QUANTUM repository as it exists now. It separates
implemented behavior from planned coaster-design, physics, and Editor systems.

## Target boundaries

The repository currently builds three primary targets plus Core tests.

### `QuantumCore`

`QuantumCore` is a static library containing the current curve mathematics,
analytic transition profiles, authored-track document structures and editing
operations, document serialization, topology operations, rider-local geometry
integration, and native track-constrained physics. Its third-party target
dependencies are GLM and nlohmann JSON. It is independent of Vulkan, SDL3,
VMA, Dear ImGui, and Editor code.

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
history with an explicit unreachable state. Launches, losses, brakes, lifts,
and train-length effects are not part of this diagnostic evaluator; the native
track-constrained train dynamics are described below.

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

## Native physics foundation

The implemented native physics foundation covers deterministic track-follower,
single-car, and rigid multi-car train behavior through Phase 11. It remains a
track-constrained reduced-coordinate model for ordinary coaster motion, not a
complete coaster operations simulation. Physics consumes the canonical
`TrackKinematicState` data through an immutable SI `CompiledPhysicsTrack`; it
does not depend on rendered `TrackGeometryFamily` or `TrackStylePreset` rail,
mesh, tube, or hardware details.

### Phase 1: track follower

`TrackLocation` identifies the current path, station, and travel direction.
`stepTrackFollower` advances one longitudinal follower deterministically with a
fixed timestep, gravity, and the shared aggregate resistance law. Circuit paths
wrap in either direction; open paths, including authored shuttle layouts, clamp
at their endpoints and report the intervention. Each step returns the committed
state and physics telemetry for motion, force contributions, curvature, run
state, and boundary behavior.

### Phase 2: authored car and bogie geometry

`CarDefinition` separates reusable dry mass and dry center of gravity from the
scenario-specific mass and center of mass in `CarLoadout`. It also authors body
dimensions, explicit front and rear hitch positions, and `BogieDefinition`
reference positions. The current deterministic `solveCarPose` path accepts
exactly two bogies, places them on the compiled track, and derives the rigid car
body pose, loaded center of gravity, hitch positions, and geometric bogie
articulation. These poses are kinematic results; wheel/rail contact and bogie
load reactions are not inferred from rendered track geometry.

### Phase 3: rigid heterogeneous train dynamics

`TrainDefinition` is an ordered lead-to-rear consist of heterogeneous
`CarDefinition` and `CarLoadout` values. Adjacent cars are joined by explicit
`InterCarConnectionDefinition` fixed lengths. `solveTrainPose` uses the Phase 2
reference location of the lead car as one generalized longitudinal train
coordinate and solves each following car sequentially so its hitch-to-hitch
distance satisfies the authored rigid connector length. Circuit seam crossing
and reverse travel preserve consist order, while an open track admits motion
only while the complete consist envelope remains on the track.

Individual cars do not have independently integrated longitudinal degrees of
freedom. Phase 3 introduced the inter-car connectors as kinematic constraints;
Phase 4 recovers their supported axial loads as described below. Phase 3
computes each loaded car's distributed world-space center of gravity and its
derivative with respect to the generalized coordinate. Those derivatives
produce the distributed generalized gravity force, effective generalized mass,
and its coordinate derivative used by the deterministic fixed-step train
integrator. Phase 3 originally included only translational car-center-of-gravity
kinetic energy; Phase 8 adds car-body rotational energy without adding another
degree of freedom. Bogie rotational inertia remains deferred.

Train steps publish train-level telemetry together with immutable/read-only
train, car, bogie, and connection pose diagnostics, including connector-length
residuals and solver information. Boundary interventions, circuit wrapping,
aggregate resistance, distributed gravity, effective mass, and generalized
motion are observable without making the physics layer depend on the Editor or
renderer.

### Phase 4: rigid connector axial-load recovery

`evaluateRigidConnectorLoads` recovers signed draft/buff force for each
non-zero-length rigid inter-car connector as derived telemetry; the recovered
internal forces are not fed back into the Phase 3 integrator. Positive force is
tension/draft and negative force is compression/buff. World force vectors use
the solved connector direction from the leading rear hitch to the following
front hitch and are equal and opposite on the adjacent cars.

The recovery uses each car's Phase 2 reference station as an auxiliary local
coordinate, without adding independently integrated car state. Deterministic
local derivatives of loaded COG, body orientation, and authored hitch positions
are combined with the constrained train acceleration, including generalized
acceleration and velocity-squared geometric terms. Phase 8 adds each car's exact
one-coordinate rotational generalized-inertia demand to this auxiliary local
balance. The resulting equations are solved as a structured linear chain. One
redundant car equation is retained as
a global balance residual, and undefined connector axes, ill-conditioned hitch
projections, non-finite values, or an excessive residual make the exact result
unavailable.

Gravity is currently the only known per-car external force. The shared
`BasicResistance` law is authored only for the aggregate train, so a multi-car
definition with force-producing aggregate resistance cannot have exact
per-connector loads recovered without inventing a per-car allocation; the API
reports that case as explicitly underdetermined. Zero-length connectors remain
supported kinematically but have no unique world-space axial direction and are
therefore make the coupled chain recovery unavailable in this milestone.

These results are axial loads consistent with the current reduced model. They
include the Phase 8 car-body rotational contribution to generalized demand, but
do not claim connector torque or a full structural load state. Connector
compliance and slack, connector moments, bogie rotational dynamics, suspension,
the front/rear reaction split, wheel reaction loads, and operational device
forces remain deferred. A
complete rigid-body or structural connector-load interpretation must not be
inferred from this telemetry.

### Phase 5: explicit per-car external force applications

`ExternalForceApplication` is a source-agnostic runtime value that identifies a
target car, an application point in that car's physical local coordinates
(+X forward, +Y lateral, +Z up, in metres), and an arbitrary world-space force
vector in Newtons. Runtime applications are supplied as a read-only contiguous
collection; they are not owned by `TrainDefinition` and do not add brake,
launch, lift, transport, or other device-specific behavior.

Train dynamics transforms each application point through the solved `CarPose`
and projects its force by virtual work, `Q = F dot dp/dq`, where `q` is the
lead-car generalized reference station. The legal central or one-sided
full-consist finite-difference poses used for train kinematics are shared by all
applications in an evaluation. Contributions are summed with distributed
gravity and the existing reduced-coordinate mass-gradient term, and telemetry
reports the total generalized external force and application count.

Rigid connector-load recovery consumes the same application collection. Each
car's already sampled auxiliary local-coordinate poses are reused to evaluate
`F dot dp/ds_i` for every application on that car. That known per-car force is
included directly in the existing translational balance equations; no second
connector-only force representation exists.

Aggregate `BasicResistance` remains a separate compatibility-preserving train
law and retains its static holding behavior. Explicit applications are additive,
so a caller representing resistance explicitly must disable the aggregate law
to avoid supplying the same physical effect twice. Because aggregate resistance
still has no authored per-car application, exact multi-car connector recovery
continues to report `AggregateResistanceUnderdetermined` whenever that law can
produce force.

The stored application point preserves a future seam for body moments, but
Phase 5 does not introduce an external-torque API, connector moments,
compliance, independent car motion, or bogie/wheel contact loads. Phase 8 later
adds deterministic body angular motion and inertia to the same one-coordinate
model; off-origin force applications continue to enter through their existing
point-motion virtual work. Force profiles and operational brake, launch, lift,
tire, station, and block systems remain deferred.

### Phase 6: explicit per-car aerodynamic resistance

`CarDefinition` may author an effective per-car aerodynamic drag area, `CdA`,
and a car-local aerodynamic center in the same +X-forward, +Y-lateral, +Z-up
physical coordinate system used by Phase 5 force applications. Different cars,
including a non-passenger lead vehicle, may use different values. A zero `CdA`
disables explicit drag for that car and causes the producer to omit its force
application. The aerodynamic center is not assumed to be the loaded center of
gravity. It remains a force application point only: no aerodynamic torque or
rotational drag coefficient is inferred from it.

`generateExplicitResistanceForces` writes into caller-owned reusable contiguous
storage. It solves one nominal train pose and reuses the Phase 5 legal central or
one-sided full-consist finite-difference poses for all configured aerodynamic
centers. For generalized train coordinate `q` and speed `qdot`, each center's
world velocity is

```text
v_point = (dp_center / dq) qdot
```

The uniform Phase 6 atmospheric state is `PhysicsEnvironment` air density and
world-space wind velocity. With relative airflow

```text
v_relative = v_point - wind_world
```

the generated world force is

```text
F_drag = -0.5 rho CdA |v_relative| v_relative
```

Zero relative speed produces an exact zero vector without normalization. In
still air the force therefore performs non-positive work at its application
point. Reverse motion, curved-track point motion, circuit seams, and open-track
one-sided derivatives use the existing kinematic policies rather than scalar
speed sign branches or wrapped-station differentiation.

Each result is an ordinary `ExternalForceApplication`. The caller supplies that
same generated collection to `stepTrain` and `evaluateRigidConnectorLoads`, so
train acceleration and connector axial-load recovery use the existing Phase 5
virtual-work paths without resistance-specific connector equations. Producer
telemetry reports generated/aerodynamic application counts and the total
generalized explicit aerodynamic contribution; train telemetry continues to
report aggregate legacy resistance separately from all explicit external-force
contributions.

Per-car aerodynamic `CdA` and the aggregate `BasicResistance` aerodynamic
coefficient are mutually exclusive in one `TrainDefinition`; validation rejects
the overlap instead of silently double-counting it. Existing definitions with
zero per-car `CdA` retain the Phase 1 aggregate behavior. The air-density field
inside `BasicResistance` remains scoped to that legacy aggregate aerodynamic
term for source and behavior compatibility, while explicit per-car drag uses
the environment density and optional steady world-space wind.

The aggregate constant mechanical/bearing and linear terms remain compatibility
laws because their physical per-car ownership and application locations are not
currently defined. Aggregate static holding also remains unchanged because the
contact supplying that holding force is unknown. Rolling resistance remains the
explicitly provisional `Crr * total-loaded-mass * gravity` supported-load
approximation in `BasicResistance`; it has not been promoted to a per-car
wheel/rail model. Any force-producing aggregate component therefore continues
to make exact multi-car connector-load recovery
`AggregateResistanceUnderdetermined`. Per-bogie and contact-resolved supported
load, running/guide/upstop wheel reactions, bearing allocation, and replacement
of the provisional rolling law remain deferred until later bogie/contact
physics.

### Phase 7: aggregate bogie-reaction audit and car-level recovery

`evaluateBogieReactions` performs inverse translational balance at an already
defined `TrainDynamicsState`. The audit found that the current model uniquely
determines only the sum of the front and rear track-constraint reactions on
each car. For car `i`, the implemented free-vector resultant is

```text
R_aggregate,i = R_front,i + R_rear,i
              = m_i a_COG,i
                - (m_i g + F_external,i + F_connector,i)
```

Gravity is the uniform world vector `(0, 0, -g)`. Explicit Phase 5 forces,
including generated Phase 6 aerodynamic drag, enter as their actual world
vectors on their authored target cars. Recovered Phase 4 connector forces enter
as the actual equal/opposite world vectors at the adjacent cars; no scalar
generalized connector value replaces them. Force application points remain
available to the existing virtual-work and connector-recovery paths, but they
do not affect a purely translational resultant balance through a fabricated
moment model.

The COG acceleration uses the Phase 3 full-consist derivatives now exposed in
`TrainCarKinematics`:

```text
a_COG,i = (dr_i/dq) qdd + (d2r_i/dq2) qdot^2
```

The velocity-squared configuration term is retained on crests, valleys, loops,
banked curves, in reverse motion, across circuit seams, and at legal one-sided
open-track derivative locations. The complete consist is sampled once per
finite-difference displacement, not independently for each bogie.

`BogieReactionAnalysis` reports one `CarTrackReaction` per car. An available
car result contains its world COG acceleration, aggregate world reaction,
magnitude, car-body `(tangent/forward, lateral, up)` components, and a world
force-balance residual. The aggregate is a free-vector resultant for
translational balance; the API does not invent an application point for it.
The local components are coordinate projections only. In particular, their
lateral and up values are not guide-wheel, running-wheel, or upstop-wheel
loads.

Each car result also contains named front and rear `BogieReaction` metadata,
reusing Phase 2's validated role ordering, authored definition index,
`TrackLocation`, world position, and canonical track frame. In Phase 7 their
force, magnitude, and track-frame component optionals remain empty. A normal
two-bogie arrangement reports `MomentBalanceNotImplemented`; less than one
micrometre of world bogie separation reports `SingularGeometry`. The latter is
a reaction-conditioning threshold, distinct from the smaller Phase 2
pose-validity threshold.

The split is unavailable because two arbitrary bogie resultants contain six
unknown force components while translational equilibrium supplies only three
equations. Restricting both resultants to their ideal track-normal planes still
leaves four scalar unknowns and generally one unresolved scalar after force
balance. A dynamic moment equation,

```text
sum(tau_COG) = I alpha + omega cross (I omega)
```

requires rotational kinematics, an authored body inertia tensor, and explicit
constraint-direction assumptions. Phase 8 now supplies the first two pieces,
but does not implement this moment-balance/contact solve. `bodyDimensions +
mass` is not used to fabricate an authoritative inertia tensor, and no
quasi-static, zero-inertia, or 50/50 split is applied. Consequently there is
still no front/rear moment-balance residual to report.

Before publishing an aggregate reaction, the analysis checks the reduced
generalized equation

```text
residual_Q = M_eff qdd + 0.5 M_eff' qdot^2
             - (Q_gravity + Q_external)
```

against `1 mN + 0.01%` of the generalized force scale. With aggregate
resistance disabled, this is the ideal-constraint consistency check that keeps
the ordinary track reaction from becoming hidden propulsion or braking. Each
world force balance uses the same tolerance policy. Invalid authored/runtime
inputs retain the established validation exceptions. Otherwise the explicit
aggregate availability states are `Available`,
`AggregateResistanceUnderdetermined`, `ConnectorLoadRecoveryUnavailable`,
`InconsistentGeneralizedBalance`, and `KinematicsUnavailable`; the underlying
Phase 4 connector status is retained.

Any configured force-producing `BasicResistance` law makes aggregate recovery
conservatively unavailable because neither its per-car allocation nor its
world application is authored. This applies to a single car as well: a scalar
generalized resistance is not silently turned into a world force. Explicit
aerodynamic drag needs no special branch because it is already an
`ExternalForceApplication`. Rolling resistance remains the unchanged
provisional aggregate approximation and does not consume Phase 7 telemetry.

No wheel/rail cross-section or `TrackGeometryFamily` participates. Phase 7
adds no running/guide/upstop split, individual-wheel distribution, contact
engagement, suspension, steering, connector compliance, operational device,
seat/restraint, or rider-body model.

### Phase 8: car-body rotational foundation

`CarDefinition::dryInertiaTensorBodyKgM2` is a full symmetric inertia tensor
about the dry COG in the physical car frame: +X forward, +Y lateral, and +Z up.
It is explicit authored SI data in kg*m^2. `bodyDimensionsMeters` never silently
defines authoritative inertia; `makeUniformBoxInertiaTensorBodyKgM2` is only a
named convenience approximation. Validation rejects non-finite or materially
asymmetric tensors, non-positive-definite or numerically singular tensors, and
principal moments that violate the physical triangle inequalities.

`CarLoadout` still authors only aggregate mass and a point COM. Phase 8 therefore
treats that load as having zero intrinsic inertia, rather than inventing rider
shape. `loadedCarInertiaTensorBodyKgM2` shifts both the dry body and point load
to the already computed loaded COG with

```text
I_shift = I_local + m (|d|^2 I3 - d d^T)
```

where every displacement is in the body frame. For a solved `CarPose`, the
world tensor is `I_world = R I_body R^T`, with `R` mapping body vectors to world
space.

The existing shared legal train samples at `q-h`, `q`, and `q+h`, with
`h = 0.01 m`, now also supply body orientations. Each adjacent orientation pair
is converted to a shortest-path world rotation vector from the relative
quaternion `Q_to conjugate(Q_from)`. Relative signs are canonicalized before the
quaternion logarithm, so `Q` and `-Q` cannot create a circuit-seam spike. A
relative rotation within `1e-6 rad` of pi is rejected as ill-conditioned. Open
track boundaries reuse the existing legal forward or backward three-pose
samples; interval angular rates are extrapolated to the endpoint instead of
reading wrapped station differences or Euler angles.

The resulting world angular-rate Jacobian and derivative define

```text
omega_i = J_omega,i qdot
alpha_i = J_omega,i qdd + J_omega,i' qdot^2
```

`TrainCarKinematics` exposes both world/body Jacobians, loaded body inertia, and
per-car translational and rotational effective-mass contributions.
`evaluateTrainAngularKinematics` converts an existing evaluation plus `qdot`
and `qdd` into contiguous `CarAngularKinematics` results with world/body angular
velocity and acceleration, rotational kinetic energy, and derivative
diagnostics without resolving another pose. Train step telemetry includes the
per-car results and train rotational totals.

For each car,

```text
T_rot,i = 0.5 omega_body,i^T I_body,i omega_body,i
M_rot,i = J_body,i^T I_body,i J_body,i
M_rot,i' = 2 J_body,i^T I_body,i J_body,i'
```

and the reduced dynamics use

```text
M_eff = M_trans + sum(M_rot,i)
M_eff qdd = Q_gravity + Q_external + Q_resistance
             - 0.5 M_eff' qdot^2
```

Thus total mechanical energy is translational plus rotational kinetic energy
plus gravitational potential energy. COG motion is not counted again in the
rotational term. A straight, unbanked constant-orientation track has exactly
zero `J_omega`, rotational energy, and rotational effective mass, reducing to
the Phase 7 behavior.

Phase 4 axial connector recovery includes the corresponding local-coordinate
rotational generalized-inertia demand so its redundant balance remains
consistent with the updated train acceleration. It still publishes only axial
forces; connector moments and compliance remain absent. Phase 7 aggregate track
reaction recovery remains a translational resultant evaluated with the updated
actual `qdd`. Separate front/rear bogie reactions remain unavailable with
`MomentBalanceNotImplemented`: Phase 8 does not add moment balance, contact-role
directions, running/guide/upstop loads, wheel loads, suspension, steering,
restraints/riders, operational devices, or rendering/track-family dependencies.

### Phase 9: conditioned front/rear aggregate bogie reactions

`evaluateBogieReactions` now conditionally splits each available Phase 7 car
resultant into one aggregate front-bogie and one aggregate rear-bogie resultant.
The raw problem contains six force-component unknowns. Although force and moment
balance provide six scalar rows, the separated two-point system has rank five:
after force balance, equal-and-opposite forces parallel to the line between the
two application points produce neither net force nor net moment. Coincident
points are still more degenerate, so arbitrary three-dimensional resultants are
not uniquely recoverable.

The implemented ideal-constraint model removes that nullspace by requiring each
bogie reaction to lie in the plane normal to its local admissible rolling
direction. This is the virtual-work condition `R_bogie dot t_bogie = 0` for a
bogie reference point constrained to move along the track. The two basis vectors
are the existing travel-oriented `BogiePose::orientedFrame()` lateral and up
axes; no separate frame is reconstructed. Reverse travel changes basis signs but
not the physical plane or the authored front/rear roles. The four unknowns are
therefore front lateral/up and rear lateral/up aggregate constraint components,
not running-, guide-, upstop-, left/right-, or individual-wheel loads.

For each car, the physical equations about its loaded COG are

```text
R_front + R_rear + F_known = m a_COG

r_front cross R_front + r_rear cross R_rear + tau_known
    = I_world alpha + omega cross (I_world omega)
```

`F_known` contains gravity, recovered connector forces, and every explicit
`ExternalForceApplication`. Gravity acts through the loaded COG and therefore
adds no moment. A leading car receives its connector force at its actual rear
hitch; a following car receives the equal-and-opposite force at its actual front
hitch. Each explicit force uses the world application point already produced by
the shared Phase 5 transform/kinematic evaluation. Phase 6 aerodynamic drag is
therefore handled by that ordinary path and its aerodynamic-center offset
contributes a moment without a resistance-specific branch. The right-hand
rotational demand uses the Phase 8 world inertia, angular acceleration, angular
velocity, and full gyroscopic term.

The reaction application points are the current Phase 2 sampled front and rear
bogie world reference positions. They are ideal aggregate constraint points;
they are not wheel-patch centroids, axle contact patches, or authored
running-wheel contacts. Aggregate `BasicResistance` still has neither a per-car
world force nor an application point, so any force-producing aggregate law keeps
reaction recovery unavailable rather than inventing a distribution.

The four minimal coordinates are solved against all three force and all three
moment rows as a 6-by-4 least-squares system. Moment rows are multiplied by the
inverse of `max(1 m, |r_front|, |r_rear|, bogie separation)` so metre-valued
lever arms do not dominate force rows numerically; diagnostics report that row
scale, while public residuals remain in N and N*m. A deterministic
column-pivoted, twice-orthogonalized modified Gram-Schmidt QR supplies the
solution and rank. The reported compact condition estimate is the ratio of the
largest to smallest accepted QR diagonal pivot. Rank uses a relative `1e-10`
pivot tolerance, condition estimates above `1e5` are rejected, and separations
below `1e-6 m` retain the explicit `SingularGeometry` result.

Least squares is not used to conceal incompatible mechanics. A finite candidate
must close both

```text
F_residual = R_front + R_rear + F_known - m a_COG

tau_residual = r_front cross R_front + r_rear cross R_rear
               + tau_known
               - (I_world alpha + omega cross (I_world omega))
```

The force tolerance retains Phase 7's `1 mN + 0.01%` scale-aware policy. The
moment tolerance independently uses `1 mN*m + 0.01%` of the largest inertial,
known, or recovered reaction-moment scale. Excessive force or moment residuals,
missing bases/rotational state, deficient rank, poor conditioning, and non-finite
systems have distinct per-bogie statuses, and invalid candidates are not
published as authoritative forces. When available, the two forces sum to the
existing Phase 7 aggregate within its force tolerance and each tangent
projection is approximately zero. World resultants, magnitudes, canonical
track-frame tangent/lateral/up components, body-frame components, application
points, rank, pivot-ratio condition estimate, row scale, and force/moment
residual diagnostics remain immutable result data.

The point-resultant model intentionally remains unavailable when a valid reduced
state demands a couple the two ideal application points cannot supply, such as
some roll-moment configurations. Phase 9 does not add contact engagement,
wheel-role allocation, individual wheel loads, suspension, steering, connector
moments or compliance, external pure torques, rolling-resistance redistribution,
restraints/riders, operational devices, or track-family/rendering dependencies.

### Phase 10: rigid bogie contact geometry and wrench feasibility

`BogieDefinition::contacts` now owns abstract physics contact geometry for its
rigid wheel/bogie assembly. This data is independent of rail mesh vertices,
`TrackStylePreset`, `TrackGeometryFamily`, visual wheel assets, and materials.
The same representation can describe tubular, central-spine/SingleRail-like,
planar, formed-steel, wood, and hybrid layouts without a family-specific solver
or geometric closest-point search against rendered rails.

Each `BogieContactDefinition` contains a descriptive `BogieContactRole`
(`Running`, `Guide`, or `Upstop`), a contact point in metres, and a unit contact
normal. Both vectors use the existing travel-oriented Phase 2 bogie frame whose
origin is the sampled bogie reference point and whose axes are +X forward and
along the admissible rolling tangent, +Y lateral, and +Z up. A positive scalar
`lambda_i` means the rail applies

```text
F_i = lambda_i n_i
```

to the bogie. The role never selects the normal or implies engagement; the
authored normal is authoritative. Validation requires finite points, finite
nonzero unit normals, valid roles, a bounded contact count, and negligible
local-X normal components. The latter enforces Phase 10's frictionless,
tangent-free normal-force assumption rather than silently introducing traction,
drive, or braking force. Directions are validated once as authored data and are
not repeatedly normalized during evaluation.

World contact points and normals are transformed only through the Phase 2
travel-oriented bogie frame. For contact arm
`r_i = p_contact,i - p_bogie_reference`, one scalar contact contribution has the
world-space wrench basis

```text
w_i = [ n_i ; r_i cross n_i ]
```

Phase 9 supplies an aggregate resultant at that same bogie reference point, not
a couple. The required Phase 10 wrench is therefore exactly
`[R_bogie ; 0]`. `analyzeBogieContactFeasibility` separately tests the 3-by-N
force system and the 6-by-N wrench system. This distinction detects geometry
such as one offset contact that can reproduce a force but necessarily produces
an unbalanced moment, while a symmetric pair can reproduce the centered force
with zero net moment.

The Phase 9 column-pivoted, twice-orthogonalized modified Gram-Schmidt QR is
shared with these small systems. It reports numerical rank and a compact ratio
of largest to smallest accepted pivot. Redundant contact columns and deficient
column rank are allowed when the required force or wrench still lies in the
span; those cases are feasible but their representative coefficient vectors are
explicitly nonunique. Condition estimates above `1e5` report
`IllConditioned`, and unstable representative coefficients are withheld.

For the wrench solve, moment rows are scaled by the inverse of
`max(1 m, maximum contact radius from the bogie reference)`. Published force and
moment residuals remain in N and N*m. Independent tolerances use `1 mN + 0.01%`
of their respective force and recovered-moment scales. The optional coefficient
vectors are least-squares diagnostics only: they are not authoritative wheel
loads, contact loads, or engagement decisions.

Rigid wheel/rail normal contact is mechanically unilateral, but Phase 10 tests
only unconstrained linear-span representability. A negative diagnostic
coefficient is never interpreted as reverse engagement. An empty authored
contact set reports `NoContacts`; it never falls back to the Phase 9 ideal
reaction plane as fabricated contact geometry. Phase 11 extends this analysis
without changing any of these Phase 10 status or diagnostic semantics.

### Phase 11: unilateral rigid-contact allocation

For a Phase 10 `Available` wrench system, Phase 11 solves

```text
minimize ||A lambda - W_required||^2
subject to lambda >= 0
```

using a deterministic Lawson-Hanson active/passive-set NNLS method. `A` is the
same full scaled 6-by-N wrench matrix used by Phase 10, including force and
contact-arm moment rows; no force-only coefficient vector is reused. At each
outer step, the zero-set column with the largest positive dual value enters the
passive set, with source-contact index breaking numerical ties. The passive
least-squares system is solved with the existing rank-revealing QR. If a
candidate crosses the nonnegative boundary, the solver steps to the first
blocking coefficient, returns every zero coefficient to the zero set, and
re-solves the reduced passive problem before selecting another entering column.
It does not solve once and clamp negative coefficients.

With residual `r = W_required - A lambda`, the implementation uses
`w = A^T r`, the negative gradient of the half-squared residual objective. The
zero-set KKT condition is therefore `w_i <= 0`, within the solver's numerical
tolerance. Coefficients stay in source-contact order independently of the QR
pivot order. The nonnegative cleanup tolerance is
`1e-12 N + 1.5e-14` times the larger of 1 N and the maximum absolute required
force component; it is only a machine-scale boundary tolerance.

NNLS convergence alone does not establish physical feasibility. Every final
coefficient contributes to an unscaled world-force and world-moment
reconstruction. The result is `Available` only when those SI residuals close
within the existing Phase 10 force and moment tolerances and the passive
representative is not ill-conditioned. A converged NNLS best fit that misses
either physical tolerance reports `UnilaterallyInfeasible` and publishes no
representative contact coefficients, reconstructed wrench, or role totals.
`NonConverged`, `IllConditioned`, and `Unavailable` remain distinguishable.
An unconstrained force or wrench span failure implies unilateral infeasibility;
other non-`Available` Phase 10 results leave allocation unavailable, with the
Phase 10 status carrying the reason.

`BogieContactFeasibilityResult::status` remains exclusively the Phase 10
linear-span status. `BogieContactAllocation::status` independently reports the
Phase 11 cone/allocation result. Thus a Phase 10 `Available` result can correctly
have a Phase 11 `UnilaterallyInfeasible` allocation. Phase 10 ranks, condition
estimates, residuals, and signed diagnostic coefficient vectors are preserved.

Available allocations expose one deterministic nonnegative mathematical
representative. Full column rank proves `Unique`. A positive passive-set
nullspace or an inactive contact wrench already in the positive-column span
proves `NonUnique`; remaining rank-deficient boundary cases report
`Undetermined` rather than overclaiming uniqueness. In a nonunique or
undetermined rigid system, per-contact coefficients are representative
allocations, not true individual wheel loads.

Contact `reportingActive` telemetry uses the separate threshold
`1e-6 N + 1e-6 * max(1 N, ||R_required||)`. This threshold never changes the
mathematical coefficients, reconstruction, or role totals. Running, Guide, and
Upstop totals sum every representative scalar coefficient by authored role;
the authored normals, not hardcoded role directions, determine world force.
Front and rear bogies are allocated independently and retain their Phase 9
identity and load split.

Phase 11 remains a frictionless rigid-contact feasibility/allocation model. It
does not add gaps, preload, hysteresis, suspension or wheel/rail deformation,
friction/slip, wheel rotational dynamics, steering, derailment, connector
compliance, operational devices, rendering, or track-family dependencies.

## Planned systems

The following are current roadmap areas, not implemented capabilities or fixed
architectural commitments:

- editable force-target profiles and endpoint-constrained force solving;
- expanded authored track-style geometry families, configurable rail/heartline geometry, and final rail meshing systems;
- direct deformation or control-point editing in the 3D viewport;
- supports, foundations, and track/support attachment hardware (the Support Workspace remains an
  unfinished disabled shell);
- connector compliance, slack, springs, damping, and train whip;
- suspension/compliance, gaps/preload, friction/slip, and physically resolved
  individual-wheel load sharing beyond the rigid representative allocation;
- operational lifts, transport tires, brakes, launches, and stations;
- storage, blocks, occupancy/reservations, dispatch, switches, transfer tables,
  and topology-aware operations;
- multiple simultaneously simulated trains;
- persistence and Editor UI for train definitions; and
- train and connector rendering.

No compatibility with another coaster-design application's file formats,
source code, or architecture is claimed.

## Verification

`tests/` contains CTest-registered coverage for the curve and rider-local
mathematics, native track-follower, car-pose, and train physics, authored
document structure and persistence, channel-profile editing, mixed-region
authoring and generation, transaction rejection, Transition Editor model
semantics, centerline visualization, viewport picking, and selection mapping
described above. The tests are designed to run in both Debug and Release
configurations. Exact historical assertion or test-group counts are
intentionally omitted because they change as the Core develops.
