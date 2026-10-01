# Hybrid local outer inclined support lines

This note records the last topology capability before Hybrid M2A was frozen.
Its focused Debug results below are historical; see [final integration validation](hybrid-m2a-final-validation.md).

This is the final local topology pass before freezing Hybrid M2A. It builds on
the existing uncommitted Hybrid mounting and panel-local bracing work. The
upright two-post core, its foundations, local stories, stepped run ties and
banked upper interface are preserved. No PR was changed.

## Reference basis

The baseline is the supplied
[complete Lightning Rod photographic audit](<C:/Users/coast/.codex/visualizations/2026/09/30/01a0f45d-988d-7b70-944b-be7563f81ea4/lightning-rod-reference-audit.md>)
and [RCDB gallery](https://rcdb.com/13369.htm#p=0). All 64 original-resolution
photo copies beside the audit were reopened for visual inspection in this pass,
including original construction and 2024 images. RCDB's live ride page was
also reopened; its text-only Pictures response did not expose the photographs,
so visual inspection used the audit's identified source-photo copies.

- [71911](https://rcdb.com/13369.htm#p=71911),
  [71919](https://rcdb.com/13369.htm#p=71919),
  [71921](https://rcdb.com/13369.htm#p=71921), and
  [72988](https://rcdb.com/13369.htm#p=72988) expose upright inner posts with
  additional inclined outer lines and separately placed visible feet.
- [71896](https://rcdb.com/13369.htm#p=71896),
  [72979](https://rcdb.com/13369.htm#p=72979), and
  [72990](https://rcdb.com/13369.htm#p=72990) show tall upright interior framing
  within a wider inclined envelope. Additional wide/tall references inspected
  include 72986, 72982, 71912, 71916, 71928, 68311 and 68317.
- [68313](https://rcdb.com/13369.htm#p=68313) and 68320 show substantial inclined
  lines meeting local horizontal framing at repeated elevations, with distinct
  member layers and projecting ends. Construction tips and unfinished bays
  were not interpreted as permanent requirements.

All three supplied reference images were reopened: the perspective multi-tower
view supports keeping independently framed inner towers and open run bays;
the orthographic view supports stepped local ties; the front/back tower view
supports upright posts and face-mounted ledgers. Existing
[mounting](hybrid-member-mounting.md) and
[local-bracing](hybrid-local-bracing.md) captures were inspected before editing.
Lightning Rod supplements those sparse local archetypes; it does not establish
a universal post count, slope, width multiplier, height threshold or bracing
schedule. Photographic inclination alone does not establish an engineering
classification; these new lines are explicitly authored as primary supports
as required by this pass.

## Minimal procedural selector

`WoodenSupportRunRecipe::hybridOuterSupports` is a sparse vector of
`HybridOuterSupportChoice`. An example selecting only the third tower:

```json
"hybridOuterSupports": [
  {"towerIndex": 2, "sides": "Both", "foundationOutset": 10.0, "topOutset": 1.0}
]
```

`sides` accepts `None`, `Left`, `Right`, or `Both`. Omitted towers resolve to
None. Left/right are the negative/positive horizontal lateral directions used
by the existing inner lanes, relative to increasing station, independent of
the camera or bank. The selector works with SimpleBent and ConnectedTowers;
Automatic archetype selection does not choose outer supports.

Both outsets are explicit Core-unit distances from the corresponding inner
post: one at the foundation, one at the lower-frame shoulder. They do not
scale with width or height. The top remains outboard, preserving a separate
primary chain and an unambiguous mounting host. Validation requires finite
positive top outset and a larger foundation outset to make an inclined line;
the 1e-6 tolerance is the existing geometric degeneracy convention. Default
outsets of 4 and 1 units are QUANTUM visual approximations, not measurements.

Each selected side gets one independently anchored straight PrimaryPost chain.
It uses the existing full square solid primary profile, BentPost orientation
and geometry-derived directed orientationReference. Its nodes are real points
on that outer structural line. Transverse LedgerCap extensions connect it to
the inner post at the existing raised lower ledger, story ledgers and shoulder.
Staggered run-tie junctions and banked attachments do not create extra outer
connection rows. Each extension uses the existing signed host-face mounting,
outside-support coverage and overhang; physical contacts remain derived by
the shared placement resolver.

The generator appends the outer topology after completing the inner assembly,
retaining all existing inner element IDs, geometry, member descriptors and
physical placement. Inner post foundations never move. Primary chains remain
separate and connect through horizontal members rather than ambiguous primary
branches. The document owns nodes, members and recipe data through its
existing support collection; no new resource owner, graph layer or renderer
framework is introduced.

The optional JSON array uses string enum names and strict existing validation.
Duplicate tower indices, unknown choices/fields, malformed indices/distances
and use on another family are rejected atomically. Missing fields in older
documents default to an empty selector without changing persisted geometry.
Unavailable tower indices remain dormant when the range changes, like the
existing local panel choices. These are procedural station-order addresses,
not stable geometric region identities. Authoring uses the recipe/API/document
seam; a separate tower-selection UI is outside this pass.

## Captures and comparison

The manifest uses the existing Windows editor capture path and the earlier
Hybrid lighting/material settings. Preset filenames containing `tint` do not
change timber color. All fixtures contain complete generated runs, without
cropping out neighboring members or editing geometry for the screenshots.

| Capture | Local topology and reference comparison |
| --- | --- |
| [Low, no outer lines](images/hybrid-outer-support/supports-hybrid.png) | Five unchanged upright inner pairs. A sparse local unit remains available, consistent with the supplied reference organization. |
| [Low, one outer line](images/hybrid-outer-support/supports-foundations.png) | Only tower 2 adds a left inclined line and its own pad. The other four towers retain their two feet. Compare the independent inner/outer lines in 71911, 71919 and 71921; flat fixture terrain and exact dimensions are approximations. |
| [Tall before](images/hybrid-outer-support/supports-tint-pine.png) | Complete connected-tower baseline, preserving stepped ties and open run bays from the supplied perspective/orthographic references and earlier captures. |
| [Tall, both outer lines](images/hybrid-outer-support/supports-tall.png) | Only tower 2 gains two outer primary lines; its upright core stays in place. Compare the upright core inside the inclined envelope in 71896, 72979 and 72990. This exercises the capability without reconstructing their whole fan-like assemblies. |
| [Outer framing contact](images/hybrid-outer-support/supports-closeup.png) | Existing-level ledger extension between the upright core and inclined line, with independent host contacts and primary member thickness. Compare local horizontal contacts/layers in 68313. |
| [Transverse outer/core view](images/hybrid-outer-support/supports-tint-weathered.png) | Head-on orthographic inspection of the full run: both selected outer lines and their separate pads around the unchanged upright core. Other towers overlap through depth, as in 71896; projected intersections are not additional graph joints. |

The low pair and tall pair use matching cameras for before/after comparison.
No universal four/six-post bent or mandatory outer-support run is encoded.
The chosen tower, exact base/top outsets, ledger schedule, square primary
section, face choice and overhang are procedural approximations. The source
photos show richer local arrangements and terrain-dependent foundation heights;
this fixture does not claim a measured Lightning Rod reconstruction.

## Verification and scope

The Windows MSVC Debug editor and all 16 focused test targets built
successfully. The filtered Debug CTest run passed **16/16 suites in 30.09
seconds**: ReadmeCapture, SupportModel, SupportConnection,
SupportSerialization, SupportVisualization, SupportAuthoring, DocumentHistory,
WoodenSupportGenerator, SupportMemberRole, HybridTimberAccuracy,
SupportOrientationReference, SupportOrientationFrame, SupportMounting,
HybridLocalBracing, HybridOuterSupport and SupportSolidGeometry. No full CTest
or Release run was performed. Existing ignored-nodiscard warnings in older
tests were left unchanged. No new compilation warning was reported for this
pass's code.

Build/test logs are under `build/hybrid-outer-support/`: `build-debug.log`,
`test-rebuild.log`, `focused-build.log` and `focused-debug-tests.log`.
Capture logs are `captures.log` and `capture-front.log`. The ordinary capture
path includes the existing OBS Vulkan-layer API-version warning; no validation
or warning was disabled. All six final captures completed successfully and
were visually inspected. The transverse camera was widened after the first
inspection to include the neighboring tower tops and feet; its final capture
is recorded separately in `capture-front.log`.

Reproduce the fixtures and captures:

```powershell
build/tests/Debug/QuantumEditorHybridOuterSupportTests.exe build/hybrid-outer-support
build/editor/Debug/QUANTUM.exe --capture-screenshots docs/hybrid-outer-support-captures.json
```

The new suite covers independent footing placement and all four side choices
on low/tall structures with both existing archetypes; unchanged upright inner
posts and physical placements; full primary sections and deliberate existing
ledger connections; deterministic generation and dormant selections; exact
save/load/regeneration and Undo/Redo; curved, banked, rising/falling geometry;
endpoint/container reversal and full spatial rotation; and atomic rejection.
The fixture exporter also verifies save/load through actual document files.

Files changed in this pass: `core/include/quantum/coaster/Supports.hpp`,
`core/src/Supports.cpp`, `core/src/CoasterDocument.cpp`,
`core/src/WoodenSupportGenerator.cpp`, `tests/CMakeLists.txt`, and the new
`tests/HybridOuterSupportTests.cpp`, this note, capture manifest and six PNGs.
No changes to the physical-placement resolver, renderer, editor UI or history
implementation are needed. Other pre-existing working-tree edits are preserved.

Work stops at this capability. Joints, paired ledgers, terrain adaptation,
additional support families, other framing schedules, detailed track interfaces
and PR updates remain outside the pass. The existing square-solid ledger/tie
corner overlap remains a separate joint-detail limitation. Hybrid M2A topology
is frozen after this verification; no further topology milestone is included.
