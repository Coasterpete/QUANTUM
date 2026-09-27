# Geometry Authoring M0 — Editable Force-Driven Sections

Turns the existing Core force-driven regions into a usable FVD-inspired
authoring workflow. Before this milestone the Geometry Editor showed a
Force-Based region as read-only. Now a user can create one, edit all three of
its authored target curves, and see the track geometry regenerate.

This milestone adds authoring surface and Editor plumbing. It does **not** add
a force solver, change the integrator, change Core's force semantics, or
implement functional heartline.

## Workflow

### Creating a Force-Based region

`Append Region...`, `Prepend Region...`, and `Insert After Selected...` all
open the same type-choice strip, which now offers three types:

| Type | Authored content |
| --- | --- |
| **Profile** | Three rider-local angular-rate profiles |
| **Circular Arc** | Radius, swept angle, plane tilt, bank change |
| **Force-Based** | Normal G, Lateral G, and Roll Rate targets |

A new Force-Based region is valid and complete the moment it exists. Core's
`convertSectionToForceDriven` gives it a constant 1.0 G normal target and zero
lateral and roll rates, each covering the region's whole authored length. The
new region is selected automatically, so its editor is already open.

A constant 1.0 G normal target on a level start is free riding: it is exactly
the specific force gravity already applies, so the region generates straight.
That is what makes it a safe creation default.

### Editing the three targets

Selecting a Force-Based region opens the **Geometry Editor** with the same
profile graph the Transition Editor uses, over the region's three authored
channels. Every established interaction is shared:

- channel selection and authored endpoint selection,
- vertical dragging of values and horizontal dragging of interior segment
  boundaries (with the same axis lock, Shift fine-drag, and snapping settings),
- numeric editing of the selected endpoint,
- every existing transition shape,
- split / remove via the row context menu,
- `Fit Y`, `Y In`, `Y Out`, and the `Details...` read-outs.

Rows map to the rider-local axes, which is the same mapping Core's integrator
uses:

| Row | Authored channel | Units |
| --- | --- | --- |
| **Roll Rate** | `ForceDrivenRegion::rollRate` | deg/m |
| **Normal G** | `ForceDrivenRegion::targetNormalG` | G (dimensionless) |
| **Lateral G** | `ForceDrivenRegion::targetLateralG` | G (dimensionless) |

Core authors the roll rate in **radians per Core coordinate unit**. Because
degrees-per-meter is a per-meter quantity, the row divides by the document's
`metersPerCoordinateUnit` before presenting it. The two G rows are
dimensionless and are shown unchanged.

### What the region does to the track

Editing any channel regenerates the region through the existing force-driven
integrator, using the document's physical settings and the region entry pose.
The integrator stays speed dependent: a constant lateral target produces
curvature `g0 * G * mu / v^2`, so the same target gives a different shape at a
different entry speed.

### Infeasible targets

If a requested target cannot be generated, the edit is **rejected** and the
Editor reports the real reason and location in the Track Workspace, for example:

```
energetically unreachable: Force generation encountered an unreachable energy
barrier. (region 2) at 20.9 into the region (track station 65.9);
speed squared 2.26e-03 m^2/s^2
```

The last valid committed region and its geometry are kept. The Editor never
silently clamps a target, substitutes a spline for a failed solve, or replaces
a solver failure with a generic message. `trackGenerationFailureReasonToString`
in Core supplies the reason label, so the five `TrackGenerationFailureReason`
values are reported distinctly.

## Target forces versus actual forces

The Geometry Editor is for **authoring targets**. **Force Diagnostics** is for
**inspecting loads**. They are deliberately different surfaces.

A target curve is not the same thing as the resulting dynamic train load:

- the target is a point-mass authoring intent evaluated against the document's
  gravity and energy;
- the evaluated load additionally reflects the solved vehicle pose, and for a
  real train will include bogie/wheel behaviour that this point-mass model does
  not represent.

The force surface states this under the graph and offers **Show Force
Diagnostics**. The `Details...` popup also places the authored target next to
the nearest evaluated sample for the selected endpoint, reporting the
difference, so the two can be compared at one location without implying they
are the same quantity.

## Transactions

Every edit goes through the established candidate-validation, publication, and
document-history pipeline. The Editor queues an intent; Application applies it
to an `AuthoredTrackEditTransaction` candidate; Core validates the profiles,
regenerates the geometry, and evaluates rider loads; only then is the candidate
committed and recorded in history.

Consequences that hold for force-driven regions:

- Save/load round-trips byte identically.
- Undo/Redo work per gesture, including continuous drags.
- Duplication produces an independent copy; reordering preserves content.
- Region length changes rescale all three channels and keep them covered.
- A rejected profile, a rejected length, an infeasible target, and a candidate
  without a completed rider-load evaluation all leave the committed document,
  the geometry cache, and the editor buffers untouched.
- Dragging a preview handle never mutates committed authored state.

## Architecture notes

### One row model for both authoring models

`TransitionEditorModel` previously named its three graph rows `RateChannel`,
which stopped being true once a row could hold a force target. It is now
`ProfileChannel`, named for the rider-local axis each row controls, and
`sectionRateChannel` is now `sectionProfileChannel`, which resolves a row to
the right authored `ChannelProfile` for either authoring model and refuses a
planar arc (which owns scalar parameters, not curves).

Presentation and units moved out of the row identity and into
`ProfileRowStyle`, with `rateProfileRowStyles` and `forceDrivenRowStyles`
factories. A style carries the row label, display unit, value-field label,
authored-to-display factors, the flat-profile fit magnitude, and which derived
read-out the row supports. This is a mechanical rename plus one added
descriptor; the graph, drag, hit-testing, and numeric code is shared unchanged.

### Failure reporting

`Application` now catches `TrackGenerationError` separately and formats the
failure through `describeTrackGenerationFailure`, which keeps the reason, the
region index, the in-region distance, the whole-track station, and the speed
squared. `EditorUi::setGeometryEditError` carries that message to the Track
Workspace, which shows it for every selection until an edit is accepted or the
selection changes.

### Prepared for functional heartline

Nothing in this milestone assumes a force-driven region has exactly three
channels forever:

- the graph is driven by a row table, so a fourth authored channel is a Core
  field plus a row, not a new widget;
- `sectionProfileChannel` is the single routing point, so a rider-reference
  channel is added in one switch;
- the integrator still receives authored targets and the entry pose; a
  heartline offset belongs in that Core call, not in the Editor.

No heartline behaviour is implemented here, and no Editor code anticipates a
specific future field.

## Verification

```
cmake --build --preset windows-msvc-debug --target QUANTUM
cmake --build --preset windows-msvc-debug --config Release
ctest --test-dir build -C Debug  --output-on-failure
ctest --test-dir build -C Release --output-on-failure
```

- Full Debug CTest: **82/82 enabled tests passed** (one pre-existing GPU
  residency test remains disabled).
- Full Release CTest: **82/82 enabled tests passed**.
- `QuantumEditor.ForceDrivenAuthoring` adds 14 focused cases: creation,
  row routing, display units, view-only immutability, profile editing,
  length changes, regeneration, infeasible rejection, rejected transactions,
  serialization, undo/redo, duplication/reordering, region-kind switching,
  and the capture fixture.

### Real application

`--dev-preview-smoke ... --force-driven-authoring` drives the whole authoring
workflow through the real editor command pipeline in the real application: it
injects the same intents the Geometry Editor emits and asserts each committed
outcome. Result: **PASSED**, including the infeasible target being rejected
with the committed region unchanged, and Undo and Redo both restoring
revisions.

```
[EDIT] appended region=3 kind=forceDriven length=60.000000 normalGEnd=1.000000
[EDIT] section=1 channel=1 endpoint=2 value=1.400000 segment=1
[EDIT] section=1 channel=2 endpoint=2 value=0.350000 segment=1
[EDIT] section=1 channel=0 endpoint=2 value=0.040000 segment=1
[EDIT] section=1 channel=1 segment=1 split distance=15.000000 newSegment=3
WARNING: [EDIT] Authored edit was rejected: Force integration did not meet its
    error tolerance.
[SMOKE] force-driven authoring: infeasible target rejected, committed region
    unchanged
[EDIT] Undo restored the previous document revision
[EDIT] Redo restored the next document revision
```

Note the run needs `--stopped-preview`: a structural edit rebuilds the
simulation preview, which stops continuous playback, and the smoke harness
treats a playback stop as a failure. That is pre-existing harness behaviour, not
a force-driven issue.

Editor/Simulator transitions, a graph drag on a force-driven document, and the
Simulator running a force-driven track were also run with
`--mode-cycle --transition-drag --enable-gpu-validation`: all PASSED, with no
Vulkan validation messages.

### Screenshots

Captured from the real Editor at 1600x900 through the readme capture runner.

| Image | Shows |
| --- | --- |
| `force-driven-profiles.png` | A newly created Force-Based region: the three targets at their creation defaults covering the whole region |
| `force-driven-authored.png` | The same region with authored multi-segment eased targets, and the regenerated track |
| `profile-region-transition-editor.png` | The Profile region's Transition Editor in the same document |
| `circular-arc-region-geometry-editor.png` | The Circular Arc region's Geometry Editor in the same document |

The last two show that switching region types does not disturb the other two
editors.

## Limitations

- **Conversion back to Profile is not offered.** Core rejects
  force-driven → rate-profile, and a generated region has no authored geometry
  to recover. A Force-Based region can still be converted to a Circular Arc,
  which preserves its length. Converting a Profile or Circular Arc region to
  Force-Based is available and keeps the region length and track style.
- **A target is a point-mass intent.** There is no authored rider-reference
  channel yet, and evaluated loads are not a final dynamic train load.
- **No inverse solving.** Endpoint and anchor picking do not solve for the
  target that would place a marker at a requested position; that is deferred.
- **Error text is editor prose over Core data.** The reason and location come
  from Core verbatim, but the sentence around them is not localized.
- **The roll-rate display now honours the coordinate scale** on both rate and
  force rows. With the default `metersPerCoordinateUnit = 1.0` nothing changes;
  documents with another scale now show a correct deg/m value where they
  previously showed radians per coordinate unit mislabelled.
- **Save/load and Undo/Redo are verified by tests and by the scripted
  application run, not by a scripted on-disk save.** The scripted run exercises
  the history path directly; driving the platform save dialog is not part of it.

## Follow-ups

- Functional heartline (M1): an authored rider-reference channel feeding the
  integrator.
- Inverse solving from viewport picks to authored targets.
- Per-segment transition-type and distance numeric entry beyond the context
  menu, if the workflow proves it is needed.
