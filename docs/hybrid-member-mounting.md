# Hybrid member mounting and structural layering

This note records the mounting implementation within the completed, frozen
Hybrid M2A work. Its focused Debug results below are historical pass results;
final integration validation is recorded in [the final audit](hybrid-m2a-final-validation.md).

This change applies mounting intent to newly generated Hybrid members only.
The existing topology, stories, roles, orientation enums and directed
`orientationReference` remain authoritative. Loading a saved member without
mounting metadata keeps its historical centered placement exactly. Regeneration
deliberately generates new mounting intent from the current Hybrid recipe.

## Persisted schema

Each existing `SupportMemberEndConnection` gains optional `mounting` of type
`SupportMemberMounting`. Connector `localPlacement` keeps its original meaning.
No connection identity, node type, host registry or persisted transform is added.

```json
"startConnection": {
  "treatment": "MiteredCut",
  "mounting": {
    "mode": "Face",
    "face": "PositiveZ",
    "layer": "Direct",
    "separation": 0.0,
    "coverage": "OutsideSupport",
    "overhang": 0.075
  }
}
```

| Field | Values | Default when omitted inside a mounting object |
| --- | --- | --- |
| `mode` | `Face`, `TerminalSeat` | `Face` |
| `face` | `PositiveY`, `NegativeY`, `PositiveZ`, `NegativeZ`, `PositiveX`, `NegativeX` | `PositiveZ` |
| `layer` | `Direct`, `OutsideLedger` | `Direct` |
| `separation` | Finite nonnegative distance in Core units | `0` |
| `coverage` | `Node`, `OutsideSupport` | `Node` |
| `overhang` | Finite nonnegative axial distance in Core units | `0` |

The whole mounting object is optional. Its absence means centered behavior;
an empty present object means direct mounting on positive host Z. Face mode
uses Y/Z side faces; terminal seats use X end faces and require a terminal post
end pointing outward. A seated member cannot request an outside-ledger layer.
Unknown textual values, numeric enum values, incompatible mode/face pairs and
invalid distances are rejected. Defaulted equality includes the entire block,
so document history records it through the existing transaction/history path.

## Placement resolution

`resolveSupportMemberPlacements` in Core returns derived physical endpoints and
straight-member frames in member order. It does not modify the support graph.

1. The existing track-attachment resolver updates the presentation copy's
   logical positions before Core placement resolution.
2. Resolve incident PrimaryPost chains from connectivity, orienting each chain
   away from its foundation. An unanchored manual chain uses its lowest node
   ID as a deterministic root. At a story junction, prefer the segment toward
   the root, including the upright lower post at the banked shoulder transition.
   Missing/ambiguous hosts fail explicitly.
3. Host X follows the directed chain; Y follows the post's existing directed
   cross-section reference; Z is X cross Y. Select the named signed host axis.
   This sign is independent of mounted endpoint order, supporting endpoint
   order, and container order. No mounting displacement uses a world axis.
4. For a direct face contact, offset each logical endpoint along its own normal
   by supporting-section half-thickness + mounted-section half-thickness +
   separation. Rectangular half-thickness in normal `n` is
   `0.5 * (width * abs(dot(Y,n)) + depth * abs(dot(Z,n)))`; circular sections
   use the radial projection. Thus brace thickness can use member Y while
   ledger thickness uses member Z. A terminal seat starts at the post end plane
   and adds only the cap half-section and separation.
5. Different endpoint normals can alter the physical member axis. Recompute
   the mounted section's projections against the resulting straight-member
   frame until endpoint changes are at most `1e-11` Core units (maximum 64
   iterations; failure to converge is reported). Direct contacts resolve first.
6. For `OutsideLedger`, use the outer envelope of incident directly mounted
   ledgers on the same signed face, including their resolved displacement and
   actual section. Then add the brace half-section and explicit separation.
   In an aligned bent this is post half-thickness + full ledger thickness +
   brace half-thickness + gap. It is not an integer layer multiplier.
7. Apply axial extension separately. `OutsideSupport` covers the supporting
   side plane most aligned with the final member axis: half the corresponding
   supporting dimension divided by the absolute axis/face-normal dot product.
   Add authored overhang, extending the start backward and the end forward.
8. Build the final frame and pass it with the section to `memberTransform`,
   which still only converts resolved geometry into an instance matrix.

New Hybrid ledgers and caps use positive host Z, `OutsideSupport` coverage and
`0.15 * recipe.memberSize` procedural overhang. Transverse braces default to repeating on that
same signed face through every story, using `OutsideLedger` plus
`0.02 * recipe.memberSize` clearance. These ratios are visual approximations,
not reference measurements or engineering dimensions.

The subsequent [local bracing pass](hybrid-local-bracing.md) adds explicit
transverse face/direction overrides and ConnectedTowers longitudinal panel
choices. `OutsideLedger` now clears same-face longitudinal ties as well as
ledgers. Negative-face braces do not inherit clearance from a positive-face
ledger; each endpoint resolves only the selected face's actual envelope.

Longitudinal ties (and the existing SimpleBent run braces) select negative host
Y for the first post lane and positive host Y for the second. **Both endpoints
resolve independently** against their incident post frames and sections. A
curved bay therefore uses two different normals rather than translating its
whole tie from the start frame. Lower tie elevations and the existing staggered
logical junctions remain unchanged. PrimaryPost and foundations stay centered.
Current generated Hybrid caps are face-mounted; no generated cap uses a seat.

## Editor and renderer

Solid rendering, member picking, selection highlights and framing bounds
consume resolved physical placements. Rectangular/circular member picking
tests the oriented solid, retaining the existing pixel tolerance around the
physical centerline. Bounds include oriented sections, axial overhang and
foundation pad extents. Node handles and the technical line stream keep logical
positions and connectivity.

The renderer's instanced mesh, materials, sections, texture scale and resource
ownership are unchanged. Physical placements are transient Core/editor data;
the document owns only mounting intent. Section edits recompute distances;
there are no baked world offsets. Unchanged regeneration preserves generated
mounting exactly, while changed recipes re-author procedural distances.

## Verification and captures

Focused Debug targets for this implementation pass included `QuantumEditor.SupportMounting` and
existing Hybrid accuracy, solid geometry, model, connection, serialization,
role, orientation/reference, generator, visualization, authoring, history and
capture-manifest suites. This earlier pass did not run full CTest or Release.

The Windows MSVC Debug editor and all affected focused targets built
successfully. The focused run passed **14/14 suites** in 37.65 seconds; the
capture-manifest suite was rerun after adding capture-only node visibility and
passed in 6.13 seconds. Logs are in `build/hybrid-mounting/focused-debug-tests.log`
and `build/hybrid-mounting/capture-tests.log`.

Export the capture documents with:

```powershell
build/tests/Debug/QuantumCoreHybridTimberAccuracyTests.exe `
  build/hybrid-mounting/simple.quantum `
  build/hybrid-mounting/towers.quantum `
  build/hybrid-mounting/staggered.quantum
docs/export-hybrid-mounting-tower.ps1
build/editor/Debug/QUANTUM.exe --capture-screenshots docs/hybrid-mounting-captures.json
```

The capture manifest specifies reproducible poses using the ordinary editor
camera, including orthographic front/back views. It does not override material
appearance. Front/back views use a copy of the first generated tower, preserving
its nodes, split post segments, ledgers, braces and intent verbatim; full
assemblies remain in the perspective, staggered-bay and junction views. Capture
node handles are hidden so they do not cover the mounting surfaces. Interactive
node handles and technical lines retain their original positions and behavior.

- Front tower: [supports-tall.png](images/hybrid-mounting/supports-tall.png)
- Back tower: [supports-foundations.png](images/hybrid-mounting/supports-foundations.png)
- Perspective towers: [supports-hybrid.png](images/hybrid-mounting/supports-hybrid.png)
- Staggered tie close-up: [supports-closeup.png](images/hybrid-mounting/supports-closeup.png)
- Post/ledger/brace/tie junction: [supports-tint-pine.png](images/hybrid-mounting/supports-tint-pine.png)
- Same staggered view with logical overlay: [supports-tint-weathered.png](images/hybrid-mounting/supports-tint-weathered.png)

The last two filenames reuse existing capture preset names; neither changes
the document's timber tint. All captures use the existing material and lighting.

Direct comparison with the three supplied reference images confirms continuous
upright lower posts, ledgers crossing post faces with small outside-face
overhangs, single diagonals repeating on a consistent face layer, distinct
post/ledger/brace/tie placement, open bays and unchanged staggered elevations.
The selected face appears mirrored when viewed from its reverse side, as it
should; reversing member endpoint order does not change the chosen face.
The current tower/section proportions remain those of the existing generator,
not a dimensional reconstruction of the reference model.

## Files changed in this implementation

Existing working-tree topology/orientation changes were retained. This task
modified the following sources (and added the six PNGs linked above):

- `core/include/quantum/coaster/Supports.hpp`
- `core/include/quantum/coaster/SupportSolidGeometry.hpp`
- `core/src/Supports.cpp`
- `core/src/CoasterDocument.cpp`
- `core/src/SupportSolidGeometry.cpp`
- `core/src/WoodenSupportGenerator.cpp`
- `editor/include/quantum/editor/SupportVisualization.hpp`
- `editor/src/SupportVisualization.cpp`
- `editor/src/SupportPicking.cpp`
- `editor/src/EditorUi.cpp`
- `editor/include/quantum/editor/ReadmeCapture.hpp`
- `editor/src/ReadmeCapture.cpp`
- `tests/SupportMountingTests.cpp` (new)
- `tests/CMakeLists.txt`
- `tests/ReadmeCaptureTests.cpp`
- `docs/supports-m2a-solid-rendering.md`
- `docs/hybrid-member-mounting.md` (new)
- `docs/hybrid-mounting-captures.json` (new)
- `docs/export-hybrid-mounting-tower.ps1` (new)

No other support family was implemented. PR #80 was not updated or merged.

## Remaining approximations

The supplied references establish face layering, ledger coverage, repeated
diagonal direction, upright posts and open bays. They do not measure exact
clearances or specify every hidden layer order. The selected brace-outside-ledger
order, sections, small gaps and overhangs remain procedural visual choices.
Members remain straight rectangular/circular solids with square ends. Skewed
connections are section-envelope placement approximations; they do not model
cut contact surfaces, miters, notches, carpentry, fasteners or steel hardware.
At an unshifted receiving junction, the perpendicular ledger's small axial
overhang can intersect the edge of the directly post-side-mounted tie. This is
an explicit remaining square-solid corner approximation (0.075 Core units of
overhang in the capture fixture), visible in the receiving-junction close-up.
The brace/ledger face stack is separated; this perpendicular corner contact is
not the former coplanar centerline overlap. Eliminating it while keeping direct
post-side contact would require a cut/joint detail or a different explicit
mounting-layer policy; neither is inferred from the references here.
No structural analysis or other support-family implementation is included.
