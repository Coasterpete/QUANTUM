# Frozen Hybrid M2A integration validation

Validated on Windows on 2026-09-30, on
`feature/supports-m2a-solid-timber-rendering`. Local HEAD and PR #80 HEAD were
both `4f375d7fe5a9c4985e2709fe76caec1edcdeeb8f` when the audit began.
The completed implementation was already in the working tree.
PR #80 was not updated or merged. No new topology, bracing, mounting, joint,
terrain, track-interface or visual capability was added during this cleanup.

## Working-tree audit

The audit covered the full tracked diff from PR #80 HEAD, all 36 non-ignored
untracked files, support documentation, seven added test sources, four Hybrid
capture manifests, the tower-crop exporter and all 21 Hybrid capture assets.
The starting tracked diff contained 19 modified files. Ignored build, dependency,
distribution and old smoke artifacts were identified separately; they are not PR
content. Validation outputs were written under ignored `build/hybrid-m2a-final/`.

The untracked work is legitimate completed M2A work: role/orientation/frame,
timber accuracy, mounting, local-bracing and outer-support tests; three focused
implementation notes; four capture manifests; the read-only-source crop exporter;
and the topology/mounting/local-bracing/outer-support PNGs. Nothing was deleted.
The full pending diff intentionally includes shared role sections and
actual-elevation story correspondence in other families. Those completed shared
changes predate this cleanup; their dedicated accuracy work remains deferred.

No accidental feature scope expansion, competing subsystem, new dependency,
unnecessary resource owner or test-only geometry-generation hook was found.
Capture poses and hidden node markers are opt-in presentation data guarded by
the existing capture scenario; normal startup and authored geometry are unaffected.
Fixture generation/cropping remains in tests and the documentation script.

## Exact cleanup file inventory

These are changes made by this integration pass, relative to the initial dirty
tree, rather than the entire pending PR diff:

| File | Cleanup |
| --- | --- |
| `README.md` | Stop listing implemented support authoring/generation/solids as future work. |
| `core/include/quantum/coaster/Supports.hpp` | Correct PrimaryPost and upper longitudinal-tie comments. |
| `core/src/WoodenSupportGenerator.cpp` | Remove redundant non-Hybrid profile caches/switch; correct historical comments. |
| `docs/architecture.md` | Update only support model, mounting, renderer, picking, authoring and roadmap descriptions. |
| `docs/supports-m2a-solid-rendering.md` | Reconcile completed semantics, topology, orientation, mounting and historical evidence. |
| `docs/wooden-support-generator-m0.md` | Mark the original milestone description as historical/superseded. |
| `docs/wooden-support-generator-m1.md` | Mark old line-only/shared-section/story claims as historical/superseded. |
| `docs/hybrid-member-mounting.md` | Distinguish focused historical validation from final integration validation. |
| `docs/hybrid-local-bracing.md` | Qualify pass-local scope and the subsequently completed outer supports. |
| `docs/hybrid-outer-support.md` | Identify the final pre-freeze capability and historical focused results. |
| `docs/readme-screenshot-capture.md` | Correct scenario count, optional camera/marker controls and historical validation scope. |
| `tests/HybridTimberAccuracyTests.cpp` | Correct stale M1/shared-section comment and count-check diagnostic; assertions unchanged. |
| `tests/SupportOrientationFrameTests.cpp` | Correct obsolete inner-post batter comment; assertions unchanged. |
| `docs/hybrid-m2a-final-validation.md` | Add this audit and validation record. |

Runtime documents, logs, JUnit/discovery reports and two new banked inspection
captures are disposable validation artifacts under `build/hybrid-m2a-final/`.
Existing completed captures and manifests were preserved byte-for-byte.

## Documentation and architecture conclusions

The current [M2A description](supports-m2a-solid-rendering.md) now documents
`SupportMemberRole`, `SupportMemberOrientation`, directed `orientationReference`,
actual-elevation correspondence, SimpleBent/ConnectedTowers, sparse local choices,
member mounting and physical placement, and optional inclined outer primary lines.
Claims of absent roles, unauthored orientation, absent mounting, unchanged M1
topology and line-only presentation are corrected or clearly historical.
Initial renderer timings/captures are retained as historical evidence, not a
benchmark or visual acceptance record for completed Hybrid generation.

Separation remains explicit:

- The document owns the logical graph, stable IDs, recipe, roles, orientation,
  mounting intent and appearance. Loading preserves saved graph geometry.
- Generation assigns topology and member semantics; explicit regeneration
  recomputes geometry/references and preserves structure identity/appearance.
- Core derives physical endpoints, signed post frames and layer/coverage
  placement. It does not move logical nodes or serialize matrices.
- Editor picking, highlights and bounds share that physical resolver. Node
  handles and technical centerlines remain logical; renderer instances remain
  presentation data with the established Vulkan buffer ownership/lifetime.
- Existing whole-document transactions, equality, persistence and history
  cover the additive intent. No separate history or persistence layer was added.

The only executable cleanup calls the existing shared `timberProfileForRole`
directly for non-Hybrid members instead of caching five profiles and repeating
its dispatch. Hybrid proportions and Unspecified rejection remain unchanged.
No dead persisted field or unnecessary schema removal was identified.

## Full build and test matrix

Configuration: existing CMake multi-configuration build, Visual Studio 18 2026,
MSVC x64, `BUILD_TESTING=ON`, Engine and Editor enabled. No build configuration,
test disablement or warning suppression was changed.

```powershell
cmake --build build --config Debug --parallel 8
ctest --test-dir build -C Debug --output-on-failure --parallel 8
cmake --build build --config Release --parallel 8
ctest --test-dir build -C Release --output-on-failure --parallel 8
```

| Configuration | Full build | Discovered | Passed | Failed | Disabled | Other skipped | CTest wall time |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Debug | Passed, exit 0 | 94 | 93 | 0 | 1 | 0 | 677.47 s |
| Release | Passed, exit 0 | 94 | 93 | 0 | 1 | 0 | 39.54 s |

Both full CTest runs were unfiltered. `QuantumEngine.GpuTrainPoseResidency`
retains its existing intentional `DISABLED TRUE` setting; it is the sole
non-running test. All 17 support-labeled suites ran, including all seven new
role/Hybrid/orientation/mounting/local-bracing/outer-support suites.
The reports include discovered test JSON and complete JUnit XML.

After the final comment and test-diagnostic wording edits, both full build
commands passed again, and HybridTimberAccuracy/SupportOrientationFrame passed
again in both configurations (2/2 each). No assertions or expected behavior
changed after the full matrix.

The initial full builds each reported 37 existing MSVC C4834 warnings for ignored
`[[nodiscard]]` results in older SupportModel, SupportSerialization,
SupportConnection, SupportAuthoring, DocumentHistory and ForceDrivenAuthoring
tests. No new production warning was reported. The final incremental full-target
builds emitted no warnings. Windows antivirus scans of fresh Debug test binaries
were observed during the Debug run; wall times are validation timings, not a
performance comparison.

## Normal Windows runtime smoke

The ordinary Debug editor was started without capture/smoke arguments. Native
UI actions exercised Hybrid generation, changed-recipe regeneration, Save,
Undo/Redo, deletion, a new-document switch, Open, a second document switch and
normal window close. The application exited with code 0.

- Generation saved one structure with 88 nodes and 173 members.
- Regeneration with maximum story height changed from 14 to 4 saved 172 nodes
  and 308 members, using the existing automatic connected-tower capability.
- Undo and Redo each matched the corresponding entire serialized document
  exactly, after saving through the normal UI.
- Deletion removed the structure and its viewport content; the saved structure
  count became zero. Undo restored the exact pre-deletion document.
- New cleared the displayed structure. Opening the saved generated document
  restored it, and a normal re-save matched the full pre-load document exactly.
- Opening a disposable copy of the existing curved/banked fixture replaced the
  active document. Its unchanged recipe was regenerated and saved for inspection.
- Clean shutdown completed without an application or Vulkan validation error.

The established `--dev-preview-smoke` path additionally ran the saved banked
fixture with `--duration 4 --stopped-preview --camera-orbit` in Debug and Release.
Both reports say PASSED, both processes exited 0, and neither reported a
preview/physics failure. Debug rendered 399 frames; Release rendered 400.
These stopped-preview runs do not claim simulator playback coverage.

Debug runtime and capture logs contain only the existing loader warning that
`VK_LAYER_OBS_HOOK` advertises Vulkan 1.3 while the app requests 1.4. No validation
error, synchronization/lifetime diagnostic or VUID error was observed.

## Final visual inspection

Representative completed views were inspected for:

- SimpleBent: `images/hybrid-outer-support/supports-hybrid.png`.
- ConnectedTowers and inclined outer primary chains/independent pads:
  `images/hybrid-outer-support/supports-tall.png` and `supports-closeup.png`.
- Local longitudinal bracing with neighboring bays open:
  `images/hybrid-local-bracing/supports-tall.png` and `supports-closeup.png`.
- Front/back face mounting and staggered ties:
  `images/hybrid-mounting/supports-foundations.png` and `supports-closeup.png`.
- Banked upper interface: two fresh native captures of the normally regenerated
  existing banked fixture, in `build/hybrid-m2a-final/captures/`.

The existing capture harness completed with exit 0. The new banked views show
upright lower framing and separate upper links/caps following the banked track,
without an obvious frame-roll regression. Other inspected views retain distinct
member layers, staggered tie elevations, selected local braces and outer lines.
No visual correction or new topology was needed.

## Remaining limitations and PR status

Sections, shoulder clearance, raised ledgers, Automatic archetype threshold,
SimpleBent periodic bracing, ConnectedTowers' 0.35-story staggering, mounting
gaps/overhangs and outer outsets remain procedural visual approximations.
Local choices address recipe indices, not stable authored regions, and have no
dedicated panel/tower-selection UI. Foundations use the authored flat elevation;
there is no terrain adaptation. Square-ended solids do not model cuts, notches,
fasteners, joints or steel track-interface hardware. The documented receiving
ledger/tie corner overlap remains a known square-solid approximation.
There is no structural analysis, dimensional manufacturer reconstruction,
other-family accuracy completion, new performance benchmark or cross-driver
pixel guarantee.

The completed work is ready to commit and update PR #80, including the existing
untracked tests/docs/capture assets and this cleanup. This pass made no commit,
push, PR edit or merge. Hybrid M2A remains frozen; work stops here.
