# Hybrid panel-local bracing

This note records the local-bracing implementation within frozen Hybrid M2A.
Its focused Debug results are historical; see [final integration validation](hybrid-m2a-final-validation.md).

This pass extends the existing working tree on
`feature/supports-m2a-solid-timber-rendering`. It preserves SimpleBent,
ConnectedTowers, local stories, staggered post junctions, upright lower posts,
member roles/roll references, mounting, picking/bounds, persistence and history.
PR #80 was not updated or merged. No other support family was changed.

The primary baseline is the completed
[64-photo Lightning Rod audit](<C:/Users/coast/.codex/visualizations/2026/09/30/01a0f45d-988d-7b70-944b-be7563f81ea4/lightning-rod-reference-audit.md>),
including its photo-by-photo qualifications and separation of original-era
and 2024 imagery. The three supplied Blender/reference images and existing
[mounting captures](hybrid-member-mounting.md) are the secondary comparison.

## Additive recipe data

`WoodenSupportRunRecipe` gains two optional-on-disk vectors. Neither adds a
support-node kind, member identity, persisted transform, graph framework or
resource owner. The document owns the recipe choices and the existing member
mounting descriptors; physical endpoints remain derived Core/editor data.

| Field | Local address | Choice |
| --- | --- | --- |
| `hybridLongitudinalPanels` | `bayIndex`, `panelIndex`, `laneIndex` | `HybridLongitudinalBracing::{Open, SingleDiagonal}` |
| `hybridTransversePanels` | `towerIndex`, `panelIndex` | Existing `SupportMemberMountingFace::{PositiveZ, NegativeZ}` and `HybridDiagonalDirection::{LowerFirstToUpperLast, LowerLastToUpperFirst}` |

Indices are zero-based. Towers and bays follow increasing authored station;
panels are ordered bottom-up. A transverse panel begins at its actual lower
ledger/story row and ends at the next tower-local story row. A longitudinal
panel is the interval between two consecutive actual local tie elevations in
one connecting bay. `laneIndex` selects the first or last primary-post lane,
not a global coordinate or a viewing direction.

In the directed post frame, X follows the foundation-rooted post chain, Y
follows the generated/authored across-bent reference, and Z completes that
basis. Positive/NegativeZ therefore name the two bent faces. First/last in
the diagonal direction enum name that same directed bent's lane order. A
reverse-side view mirrors the projected diagonal; it does not change the
stored direction or the selected face.

Omitted ConnectedTowers run panels resolve to **Open**. Omitted transverse
choices preserve the existing repeated single diagonal on PositiveZ, from the
lower first lane to the upper last lane. An explicit Open entry is persisted
and compared just like SingleDiagonal. Duplicate local addresses, other bent
faces, unknown/numeric enum values, malformed indices, extra fields, and local
choices on another family are rejected before publication.

Example of selecting one panel while its neighboring bays remain open:

```json
"hybridLongitudinalPanels": [
  {"bayIndex": 0, "panelIndex": 1, "laneIndex": 0, "bracing": "Open"},
  {"bayIndex": 1, "panelIndex": 1, "laneIndex": 0, "bracing": "Open"},
  {"bayIndex": 2, "panelIndex": 1, "laneIndex": 0, "bracing": "SingleDiagonal"},
  {"bayIndex": 3, "panelIndex": 1, "laneIndex": 0, "bracing": "Open"}
],
"hybridTransversePanels": [
  {"towerIndex": 0, "panelIndex": 2, "face": "NegativeZ",
   "direction": "LowerLastToUpperFirst"}
]
```

The generator's two local choice lookups are isolated from tower assembly and
mounting. Future support-region/PCG decisions can supply these records. There
is no new Automatic bracing selector: the demo authors two panels in bay 2,
lane 0, explicitly. The old `longitudinalBracing` boolean remains the legacy
SimpleBent/other-family control; ConnectedTowers uses its explicit choices
regardless of that boolean. The existing Automatic **archetype** selection
and SimpleBent's existing periodic run-brace approximation are preserved,
not promoted to photographic facts or applied to ConnectedTowers.

These are procedural recipe addresses, not stable geometric region IDs.
Unchanged recipes regenerate exactly. Changing spacing, station range or story
layout can change which geometry occupies an index. Choices for absent panels
remain dormant and persist; generation never redirects them to another panel
or makes replacement story rows. Shortening and restoring the run exercises
that behavior. A future authored-region mapping and panel-editing UI remain
separate work; the current authoring seam is the recipe/API/document data.

## Brace geometry and physical mounting

A selected run brace connects the lower tie corner on the earlier tower to
the upper tie corner on the next tower, on the selected lane. Both elevations
come from the existing bay-local connection list after actual-elevation
clipping and deduplication. The upper interface and tower caps do not become
extra braced panels. No foundation-to-tie fill, unmatched-story bridge, fake
ledger, extra post point, fanned row or opposite-side duplicate is generated.

Run braces receive `Brace`, `RunDiagonal`, the existing solid Hybrid brace
profile (`0.55 * memberSize` by `0.45 * memberSize`), and a normalized reference
from the cross product of the actual local post-side panel edge and diagonal.
Transverse directions choose the appropriate lower/upper corners while the
directed across-bent row and diagonal continue to supply their roll reference.
Each selected transverse panel still generates exactly one `BentDiagonal`.

Both run-brace endpoints use the existing lane-side host faces (NegativeY on
the first lane, PositiveY on the last). Transverse endpoints use their selected
PositiveZ or NegativeZ bent face. Both carry `OutsideLedger` plus the existing
`0.02 * memberSize` separation. The only resolver integration change is that
the incident horizontal envelope now includes `LongitudinalTie` as well as
`LedgerCap`. Each endpoint clears the actual directly mounted horizontal
surface on **its own selected face**, then its own projected half-section and
gap. A NegativeZ brace does not inherit the PositiveZ ledger thickness.

The existing foundation-directed post-chain resolver preserves face signs when
mounted/post endpoint order or node/member container order is reversed. The
new tests also rotate the whole structural geometry and its directed
references: resolved placements rotate with it. Curved/banked, rising/falling
fixtures verify independent endpoint contacts and actual panel boundaries.
Ledger face placement, axial coverage/overhang, upright primary posts and
logical nodes remain unchanged. Rendering, picking, bounds and logical overlay
continue through the existing physical/logical presentation paths.

## Focused Debug verification

The Windows MSVC Debug editor and all affected focused targets built. The
filtered Debug run passed **15/15 suites in 7.96 seconds**:

`ReadmeCapture`, `SupportModel`, `SupportConnection`, `SupportSerialization`,
`SupportVisualization`, `SupportAuthoring`, `DocumentHistory`,
`WoodenSupportGenerator`, `SupportMemberRole`, `HybridTimberAccuracy`,
`SupportOrientationReference`, `SupportOrientationFrame`, `SupportMounting`,
`HybridLocalBracing`, and `SupportSolidGeometry`.

The new suite has six groups: local longitudinal choices; transverse variants;
physical clearance/stable faces; curved/banked/changing stories; exact
save/load/regeneration/history including dormant choices and legacy defaults;
and atomic rejection of malformed choices. The final suite rerun also checks
that the opposite face has no phantom ledger layer. The existing mounting
suite supplies section-edit, overhang, picking, bounds and logical-overlay
regression coverage. Old documents omit both new arrays and retain their
persisted geometry on load. No full CTest or Release run was performed.

Logs: `build/hybrid-local-bracing/focused-debug-tests.log`,
`build/hybrid-local-bracing/local-final-tests.log`, and
`build/hybrid-local-bracing/captures.log`, and
`build/hybrid-local-bracing/captures-retry.log`. After the camera correction,
five captures completed; the last hit a transient editor dockspace/window-size
error. Its isolated retry exited successfully and replaced the final negative
face close-up. All six final files were visually inspected. No editor patch was
required for that retry. Existing MSVC ignored-nodiscard
warnings in older test files were left unchanged. The capture log includes
the pre-existing OBS Vulkan-layer API-version warning; no renderer workaround
or warning suppression was added.

Reproduce the fixtures and captures:

```powershell
build/tests/Debug/QuantumEditorHybridLocalBracingTests.exe build/hybrid-local-bracing
build/editor/Debug/QUANTUM.exe --capture-screenshots docs/hybrid-local-bracing-captures.json
```

## Windows captures and reference comparison

All six images use the ordinary Windows editor and existing materials. The
manifest supplies reproducible cameras, hides node handles and logical lines,
and uses the previous mounting capture's lighting. Preset filenames containing
`tint` do not change timber tint. The isolated tower views copy the first
generated tower verbatim, including split posts and mounting metadata, while
keeping the source recipe for UI display. They are explicitly capture crops;
the overview and run junction use the complete generated assembly.

| Capture | What to inspect |
| --- | --- |
| [Open region](images/hybrid-local-bracing/supports-hybrid.png) | Five framed towers with four open run bays; separate raised/staggered horizontal rows. |
| [Local braced region](images/hybrid-local-bracing/supports-tall.png) | Same overview with two explicit panels in only the third bay: open / open / braced / open. The opposite run side stays open. |
| [Positive-face tower view](images/hybrid-local-bracing/supports-foundations.png) | Single diagonal per panel; positive-face lower panels, selected opposite directions and upper negative-face variants. |
| [Opposite-face tower view](images/hybrid-local-bracing/supports-tint-pine.png) | Reverse-side oblique view of the same tower, showing the face change without generating a rear duplicate. |
| [Run junction close-up](images/hybrid-local-bracing/supports-closeup.png) | Run brace outside the tie, transverse brace outside its ledger, post contact and ledger end overhang. |
| [Negative-face junction close-up](images/hybrid-local-bracing/supports-tint-weathered.png) | The selected negative-face brace at the third/fourth-panel junction and ledger on the other post face. |

The directly supported result is **local coexistence of open and braced
run-side regions**, independently ending framing, upright inner posts and
members in distinct face layers. Audit photos 71922, 68307, 72979 and 72990
support that organization. Local transverse direction/plane variation is
supported by the audit, including 71909/71912; the audit does not trace a
manufacturer-wide front/back pairing schedule. Photo 68313 and the mounting
references support depth-separated horizontal/diagonal contacts and projecting
ends. Their geometry justifies retaining physical placement rather than
centering the new members on logical graph lines.

The supplied perspective reference still supports repeated single transverse
diagonals and sparse connecting bays. The orthographic reference still matches
locally stepped tie elevations without forced transverse ledgers. The supplied
tower reference matches upright posts, face-mounted projecting ledgers and a
repeated single diagonal. Comparing the existing mounting screenshots with the
new junction views retains those contact/overhang decisions and adds local
run bracing and negative-face variants. Projected crossings in an overview
may belong to different structural planes; none authorizes an automatic X.

**QUANTUM approximations:** the exact demo panel selections, brace direction
schedule, outer-side placement, OutsideLedger order, section ratios, gaps,
overhangs, eight-unit stories, quarter-story shoulder/lower-ledger proportions,
existing 0.35-story stagger and Automatic archetype threshold. The photographs
do not measure those values or establish a ride/manufacturer bracing schedule.
Negative-face braces may have no same-face ledger in this baseline; the
resolver correctly places them against that post face with the selected gap.

The references additionally show inclined outer supports, varied foundation
elevations/pedestals, more layered/possibly paired horizontals, longer diagonal
spans and detailed steel/track interfaces. At this pass, those were not represented.
The subsequent [outer-support pass](hybrid-outer-support.md) adds explicitly
selected inclined primary lines; it does not add terrain adaptation or a raker
system. Paired brace/ledger topology, terrain adaptation, exact carpentry,
miters/notches, bolts/plates and structural engineering remain unimplemented.
The previously documented square-solid perpendicular ledger-overhang/tie
corner intersection remains; this pass clears brace/horizontal layers without
inventing joint cuts or moving logical geometry.

## Exact files changed in this pass

- `core/include/quantum/coaster/Supports.hpp`
- `core/src/Supports.cpp`
- `core/src/CoasterDocument.cpp`
- `core/src/WoodenSupportGenerator.cpp`
- `core/src/SupportSolidGeometry.cpp`
- `tests/CMakeLists.txt`
- `tests/HybridLocalBracingTests.cpp` (new)
- `docs/hybrid-member-mounting.md` (small follow-up clarification)
- `docs/hybrid-local-bracing.md` (new)
- `docs/hybrid-local-bracing-captures.json` (new)
- The six PNGs linked above under `docs/images/hybrid-local-bracing/` (new).

All other pre-existing working-tree files match the initial local snapshots
byte-for-byte. No Vulkan,
material/PBR, sky, environment, terrain, track geometry or train physics
implementation was changed. Work stops at local Hybrid bracing, focused Debug
verification and captures; PR #80 remains untouched.
