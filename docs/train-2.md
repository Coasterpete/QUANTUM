# Train 2.0 — authoring and local handoff

## MAINTAINER SUMMARY

The document now owns an ordered physical consist. Each car owns its editable physical definition, load, role, stable ID, rear connection, GLB identifier and visual alignment. The Train workspace edits those records through the native document/history boundary. The simulation and GLB presentation resolve the same ordered records.

Cars can differ in mass, COG, dimensions, inertia, bogie positions, hitches and load. Lead/Middle/Rear are editable passenger presets; Zero and Trailer supply different generic physical defaults. A role label does not constrain physics. Exactly two separated bogies per car remain required by the existing solver.

The existing Vulkan viewport shows physical bounds, bogies, hitches, connectors, loaded COG and body axes. Native summaries calculate masses, spacing and straight assembled body-envelope length. Ride Hardware retains its existing states and force laws, with stable authored car IDs added to occupancy observations.

Final visual/input acceptance belongs to the maintainer. In particular, inspect shell-to-track alignment, selection, ordering, camera controls and hardware occupancy using the acceptance fixture. Placeholder art is an engineering reference, not a production asset.

## Audit: the previous data path

| Area | Previous ownership/behavior | Train 2 change |
|---|---|---|
| Configuration | `TrainConfiguration` held one repeated physical car/load, car count, one repeated connection and aggregate resistance. | Retains those values as defaults and adds ordered authored records. |
| Document | `CoasterSetup.carsPerTrain` was saved; full train physical data was not. | `AuthoredTrack` owns and serializes the complete configuration. |
| Simulation | `SimulationPreview::rebuild` recreated the default repeated car, changing only count from setup. | Resolves the document's ordered records. This was the identical-car assumption that needed removal. |
| Native mechanics | `TrainDefinition.cars` and specialized car/connector solvers already accepted distinct physical definitions and loads. | No solver replacement or change to the mathematical model. |
| Body/load | Dry mass, dry COG, explicit dry inertia, dimensions, load mass and load COM were native data. | Every authored car exposes that existing data. |
| Bogies/hitches | Native car-local geometry; rigid pose solver required two distinct longitudinal bogies. | Placement is editable per car; the solver constraint is explicit. |
| Connections | Connection index i linked rear hitch i to front hitch i+1. | Leading car's stored rear connection supplies that relationship. |
| Visuals | `TrainVisualConsist` could use different ordered shells; preview generated lead/middle/rear placeholder selection from count. | Each physical car owns its shell/alignment; reorder moves both together. |
| GLB loading | Renderer asset normalization, runtime `assets` directory, cached static meshes and established glTF conversion. | Reuses the pipeline and caches; no new loader or source format. |
| Reset/history | Preview rebuilt derived runtime state; document history owned authored edits. | Train edits are document transactions; Reset resolves the current configuration. |
| Hardware | Observations contained train ID and current car/bogie indices. Forces already used each occupied car's loaded mass. | Adds stable car ID metadata, preserving indexing, force distribution and state machines. |
| Avalonia/ImGui | Managed editor lacked complete ordered train authoring; ImGui showed repeated-car preview inspection. | Dedicated managed workspace; ImGui still supports count edits and now inspects each resolved physical car. |

Before: setup count → transient repeated default train → solver; separate generated visual sequence.

After: document ordered records → native validation/history → physical `TrainDefinition` and matching `TrainVisualConsist` → existing solver/render pose → hardware, viewport and native summary.

## TRAIN DATA MODEL

`AuthoredTrainCar` is a value record. Its `TrainCarId` belongs to the train document and moves with the record. `TrainConfiguration` stores `trainId`, name, next ID, ordered cars, creation defaults and aggregate resistance. Runtime `TrainCarDefinition.authoredCarId` carries the identity into observations; it does not own another authored document.

The existing physical data declarations moved into `physics/TrainDefinition.hpp` because putting the configuration into `AuthoredTrack` otherwise created an include cycle through `TrainPhysics → CarPose → TrackTopology → AuthoredTrack`. Pose classes and numerical algorithms stay in their existing files. This is a dependency boundary for data, not a new ownership layer.

## TRAIN WORKSPACE

Choose Train in the workspace selector. Left: count, initial speed, prototype choice and ordered consist. Right: selected physical car, visual shell/alignment and creation defaults. Below the shared Vulkan viewport: Preview/engineering overlays and Summary/diagnostics.

Numeric fields are staged. **Validate proposal** calculates native diagnostics without a document edit. **Apply car** commits the staged car fields as one native action. **Apply train name/defaults**, **Resize**, and **Apply prototype** are separate explicit actions. Selecting another car or changing document revision reseeds the form; ordinary heartbeat snapshots preserve staged fields.

Only one train is currently simulated. The workspace says so; no unsupported multiple-train count is presented as working simulation semantics. Initial speed uses the existing native setting and unit convention.

## ORDERED CONSIST

Add appends a chosen prototype; Insert puts it before the selected car. Duplicate copies all configuration with a new ID. Remove retains at least one car. Up/Down move the complete record. The current supported range remains 1–32 cars.

Reorder changes physical order, visual order, neighbor connection relationships, summary, serialized order and reconstructed preview. A car's rear-connection value follows that car; its new next neighbor uses that value. The last car keeps its stored value for later edits but has no outgoing connection in the resolved train.

## CAR ROLES / PROTOTYPES

Lead, Middle, Rear and Custom start from the established passenger-car defaults. Lead/Rear choose their existing placeholder shell variants. Zero starts at 350 kg dry, 2 m body length, no passenger load and two separated bogies. Trailer starts at 550 kg dry, 3 m body length and two separated bogies. Other dimensions, bogie spacing, hitches, COG and box-approximated initial inertia differ accordingly.

Changing the **role label** preserves explicitly authored values. **Apply prototype** deliberately replaces those values with the chosen defaults while keeping the selected car ID. Every resulting value remains editable. No manufacturer identity or new production art was introduced.

## PHYSICAL CAR PARAMETERS

Per-car fields expose dry/load mass, dry/load COG, XYZ dimensions, full symmetric 3×3 dry inertia, two bogie XYZ positions, front/rear hitch XYZ, outgoing rigid connection length, car CdA and aerodynamic center. Contact records are preserved and serialized; this milestone does not add a contact editor.

Dry inertia is about dry COG in the physical body frame. Changing mass/dimensions does **not** silently recalculate it. **Apply with uniform-box dry inertia** is an explicit native approximation that commits the staged car and computed inertia together. Native load mechanics retain the point-mass model and parallel-axis combination about loaded COG.

## BOGIES

Bogies belong to their stable physical car. Their full local XYZ positions remain authoritative. The existing rigid body solver requires exactly two bogies with longitudinal separation greater than its existing 1e-6 m minimum. Body placement derives from solved bogie pivots, so vertical offsets change the physical body/rail relationship rather than just moving a marker.

Bogie identity is train ID + car ID + definition-local bogie index. Current sequence indices remain available for the existing force/occupancy paths. A one-bogie, shared-bogie or more-than-two-bogie design is unsupported and rejected; Zero/Trailer presets do not claim that capability.

## HITCHES / CONNECTORS

The existing rigid connector representation is retained. Neighboring cars join through the leading car's rear hitch and following car's front hitch, using the leading car's nonnegative rigid length. Zero length is supported by the native representation and is not automatically rejected. Connector closure retains the native 1e-8 m tolerance.

Orange crosses show solved physical hitches; lines join those points. There is no independent visual hitch model. Native proposal validation also attempts the actual track preview when the current preview is available, rejecting a proposal that cannot replace that working preview.

## VISUAL GLB ASSIGNMENT

The three available placeholder shells are selectable; the identifier field also accepts custom `assets://...glb` paths through the existing assets pipeline. Zero and Trailer currently reuse suitable placeholder art. A missing file produces a warning and engineering geometry remains available. A GLB load failure also enables the existing diagnostic fallback; malformed files can still fail only when the renderer loads them.

GLB geometry never supplies mass, COG, bogies, hitches, body envelope or inertia automatically.

## VISUAL / PHYSICAL ORIGIN CONVENTION

Physical body-local coordinates are meters, +X forward, +Y lateral, +Z up. The body origin is the physical reference frame and is distinct from COG. Reference bounds are centered about that origin. Author source shell geometry around the corresponding reference using the established GLB/glTF asset conversion.

The explicit **visual alignment only** translation/rotation maps imported shell coordinates into that body frame. XYZ degree rotations compose `Rz × Ry × Rx`. The renderer transform is coordinate-unit conversion × solved body-world transform × local alignment. No scaling override was added. Alignment changes neither physical pose nor physical points.

The fixture uses an explicit shared +0.7 m Z alignment for the existing temporary shells. It is stored as ordinary authoring data, not a filename-specific correction. The old placeholder dimensions do not fit every heterogeneous body exactly; inspect engineering bounds as the physical reference.

## PREVIEW / OVERLAYS

Train mode selects physical oriented envelopes, with the nearest ray intersection winning. List selection and viewport selection synchronize through native stable IDs. Other modes retain track/support/placement selection behavior. Selection/overlay/camera state is transient and excluded from history/serialization.

Yellow bounds highlight the selected car even when the general bounds overlay is off. White marks bogies, orange marks hitches/connections, magenta marks loaded COG and its line to the body origin, and RGB axes show the body frame. World positions come from the same interpolated render pose as the shells.

Frame train, selected car, front car or rear car uses transformed physical corners and the existing camera. Framing preserves yaw/pitch. Numeric edits do not automatically reframe. In Train mode F frames the train. Bounds are engineering envelopes, not detailed collision meshes.

## SUMMARY / DIAGNOSTICS

Native code calculates counts, dry/loaded masses, per-car loaded COG, reference X, gap to next, total assembled body-envelope span, front/rear extents and loaded aggregate COG. The straight reference assembly uses the **existing solver** on a synthetic straight rail; it does not assume equal spacing, symmetric bogies or coincident body/rail origins.

Length = maximum transformed body-envelope X − minimum transformed body-envelope X across all cars. Extents and aggregate COG are relative to car 0's **body origin** on that straight assembly. Gap is the leading body's rear X minus the following body's front X. The last car's gap is unused. Negative neighboring gaps beyond 1e-6 m produce overlap warnings, including the measured amount; no automatic correction occurs. These longitudinal checks are diagnostics, not 3D collision simulation.

ERROR covers invalid mass/dimensions/load/inertia, identity, connector/alignment and unsupported bogie arrangements. WARNING covers suspicious COG/bogie placement, overlapping envelopes, missing visuals or inability to assemble a straight reference. A straight-reference failure can still describe a track-specific unusual geometry, so it is distinct from actual-track preview failure. The summary exposes whether the straight assembly is available.

## OPERATIONS INTEGRATION

Occupancy uses solved bogie stations, preserving partial occupancy and hardware train-entry/exit behavior. Stable IDs add association without changing the existing state machines. Device acceleration forces were already proportional to each occupied car's loaded mass per bogie; they remain so, with the existing aggregate force cap.

Station alignment compares the configured stop station with the train's generalized reference station, which is the reference `TrackLocation` used to solve **car 0**. Bogie stations seed from that reference plus their local X and are refined to preserve rigid spacing. It is not automatically the train's front envelope, hitch or aggregate COG. Reordering deliberately changes which physical car occupies index 0 and therefore the reference-associated car.

Tests cover Zero-car partial occupancy, stable identities, unequal-mass forces/caps for Launch, Brake, Transport, Chain Lift and Cable Lift, lift engagement/advancement, and a station-hold/dispatch → transport → launch → brake → arrival-station scenario. The integrated scenario reaches station 165 m at tick 8829, peak 10 m/s, using the unchanged 1/240 s step.

## RESET / SIMULATION

Apply validates a candidate document, records one history state, commits it and stops/rebuilds preview at tick zero. No physical topology is hot-swapped inside a step. Reset reconstructs the current authored train. Play/Pause/Reset and runtime inspection do not add history actions.

ApplicationService owns the document/history and SimulationPreview owns derived CPU definitions, pose and operations. Candidate preview data is temporary. Mesh instances own identifiers/matrices, while poses are borrowed during publication. Vulkan resource ownership remains in the renderer and existing viewport lifetime; no new resource framework or GPU ownership was introduced.

## UNDO / REDO

Native tests check exact serialized restoration for add, remove, reorder, duplicate, role, mass, COG, bogie, hitch, GLB, alignment, load mass and load COM edits. Managed controls route gestures through that same native history. Transient Train selection/overlays preserve document revision.

## SERIALIZATION / MIGRATION

The root `.quantum` `formatVersion` remains 1. The additive `train` object has `schemaVersion: 2`. It persists ordered IDs/names/roles, every physical/load field, full inertia, bogie/contact records, hitches, outgoing connections, assets, alignment, defaults and resistance. Strict train fields/schema/type/finite-value validation are retained; count disagreement with setup is rejected.

Old documents without `train` expand their saved cars-per-train count into the equivalent repeated physical defaults and former lead/middle/rear visual policy. Tests save/reload that migration and compare poses within 1e-8 m and speed within 1e-10 m/s. The old repeated configuration API remains usable for existing callers. A new document always owns ordered records. Older binaries that reject unknown root fields are not promised forward compatibility with new saves.

## PERFORMANCE

Keystrokes stage text only. Apply rebuilds the train preview, retaining centerline generation and track/support presentation instead of rebuilding those unrelated products. Native authored summary/configuration and managed parsed records are cached by revision. Runtime selection/availability is refreshed independently.

One measured atomic Apply including native validation/rebuild and snapshot took 87.126 ms in the managed test run while the broader suite was active. This is a responsiveness sample, not a comparative benchmark or optimization claim. The existing solver remains responsible for preview cost.

## REGRESSIONS

Final verification commands/results are recorded below. The TrainVisualConsist count fixture now explicitly creates a default ordered consist for each count; its original shell/order assertions remain intact. A count edit on an authored train correctly preserves retained car records rather than regenerating roles and visuals.

No Supports geometry, styles, assets, gate/catwalk meshes, Transfer mechanics, general machinery physics, fixed-step policy or solver tolerances were redesigned. Existing untracked material directories remain untouched and outside the commit.

## KNOWN LIMITATIONS

- One simulated train; 1–32 cars; exactly two separated bogies per car. Shared/single bogies and specialized articulated physics remain future work.
- Generic editable presets and temporary shells, not production manufacturer-specific vehicles. No shell import wizard, automatic asset-derived physics or visual scaling.
- Straight longitudinal overlap/length diagnostics are not collision simulation or curved-track clearance certification.
- Contact records persist but have no managed contact editor in this milestone. ImGui remains a fallback, with count authoring and per-car inspection rather than full managed workspace parity.
- Staged fields are reseeded after a document revision or car selection change. Runtime asset warnings are revision-cached; asset-file replacement without a document edit still follows the existing renderer reload behavior.
- Windows Debug is verified. Release, Linux, DPI behavior, physical pointer input and final visual acceptance were not verified.

## MAINTAINER VISUAL TEST

Open `smoke-tests/train-2-acceptance.quantum` in the hybrid editor and choose Train. It contains Lead/Middle/Trailer/Rear, different masses/dimensions/COGs/bogie heights, three existing shell variants, 300 m of straight shuttle track, departure/arrival stations, transport, launch and brake hardware. Its initial speed is zero.

1. Frame train; compare the list order and per-car identity with yellow viewport selection. Try list and physical-envelope viewport picks and front/rear/selected framing.
2. Toggle engineering layers; inspect shell height against physical bounds, bogies, hitches, loaded COG and axes. The fixture's temporary art is explicitly aligned, not polished.
3. Reorder or duplicate a car, Apply a physical/load edit and then Undo/Redo. Check the shell and physical definition move together. Try missing GLB and overlap warnings without expecting automatic repair.
4. Play to approach departure stop 25 m. Use the existing Ride Hardware station Dispatch command, then resume/observe traversal and arrival stop 165 m. Inspect partial car/bogie occupancy.
5. Edit while running and verify the documented stop/reset behavior. Reset, save and reopen; confirm current consist and alignment survive.

No maintainer approval is claimed by automated tests.

## NEXT MAJOR PHASE

After maintainer acceptance, recommend the **Simulator / Operations control workspace** so dispatch, device state, occupancy and simulation controls are easier to inspect together. It was not started by this task.

## OPTIONAL CODE READING PATH

| Priority | Location | Smallest useful concept |
|---|---|---|
| READ NOW | `core/include/quantum/coaster/TrainConfiguration.hpp` | `AuthoredTrainCar`, configuration and summary value records. |
| READ NOW | `core/src/TrainConfiguration.cpp` | `resolveTrainConfiguration`, reorder and summary: one ordered record maps to one physical car. |
| READ NOW | `editor/src/ApplicationService.cpp` | `setTrainConfiguration`: validate candidate → history → commit → reset. |
| READ NOW | `editor/managed/Quantum.Editor/EditorViewModel.Train.cs` | `CarProposal`, `ApplyTrainProposal`, `PublishTrain`: staged form routing, not managed physics. |
| OPTIONAL | `core/src/TrainConfigurationJson.cpp` | Strict schema and value serialization. |
| OPTIONAL | `editor/src/SimulationPreview.cpp` | Rebuild's authored resolution and engineering publication from render pose. |
| OPTIONAL | `editor/native/NativeViewport.Train.cpp` | Stable-ID picking/framing in the existing viewport. |
| DEEP INTERNAL | `core/src/physics/CarPose.cpp` | `solveCarBodyGeometry`: rigid two-bogie pivot placement; numerical details can wait. |
| DEEP INTERNAL | `core/src/physics/RideOperations.cpp` | `observeTrain` and station reference/occupancy; state-machine details can wait. |

Understand now: vector order versus stable identity; value copies and `std::move` when reordering; const borrowed poses; candidate transactions and exception-safe commit. Useful background: include dependencies and units at the renderer boundary. Advanced sensitivity/contact/closure mathematics can wait; this change does not require reading the entire numerical solver.

## LOCAL HANDOFF

Branch: `codex/train-2-configuration-preview`, starting from local `0dcacf7482bce0f16afc0ab39be29108e59d1f07` on `codex/supports-2-structural-styles`. Final local commit ID is reported in the task handoff. No push, PR or screenshot publication.

Acceptance fixture: `smoke-tests/train-2-acceptance.quantum`. Deterministic generator:

```powershell
& build/editor/Debug/QuantumTrain2Tests.exe --write-fixture smoke-tests/train-2-acceptance.quantum
```

Bounded live smoke: **incomplete, not passed**. It presented 9 Vulkan frames, opened the fixture and bound the heterogeneous Train workspace, then failed the first framing command because the C boundary still rejected actions above 13. The range is fixed to include Train actions 14–18, with native boundary and geometry regressions. Following the one-smoke limit, no second live run was performed. Physical pointer input and maintainer visual approval remain pending. Report: `build/train-2-smoke.json`.

Suggested future PR title: **Add native ordered Train authoring, physical preview and diagnostics**.

### Verification record

```powershell
cmake --preset windows-msvc-debug -DQUANTUM_ENABLE_HYBRID_EDITOR=ON
cmake --build --preset windows-msvc-debug --parallel 2
dotnet build editor/managed/Quantum.Managed.Tests/Quantum.Managed.Tests.csproj -c Debug --nologo
ctest --test-dir build -C Debug --output-on-failure --parallel 2
ctest --test-dir build -C Debug -R 'Train2|TrainBoundary|TrainConfiguration|TrainVisualConsist|TrainMeshPresentation|SimulationPreview|CoasterDocument|CoasterSetup|DocumentHistory' --output-on-failure --parallel 2
& build/editor/Debug/QuantumTrain2Tests.exe
& editor/managed/Quantum.Managed.Tests/bin/Debug/net10.0/Quantum.Managed.Tests.exe smoke-tests/modern-steel-validation.quantum
```

Configure and final complete Windows MSVC Debug build passed. Managed build passed with zero warnings/errors. Full CTest passed **118/118 enabled tests** in 902.98 s; the existing `QuantumEngine.GpuTrainPoseResidency` test remains disabled. After the final build, the focused nine-test command above passed **9/9** in 28.92 s. Final standalone Train scenarios passed **81,098 checks**, including picking/framing. Managed suite passed, including **70 Train checks** plus existing hardware/supports/placement coverage. An initial concurrent build encountered Windows locks on two running numerical test executables; the final complete build succeeded after CTest exited. Existing unrelated nodiscard warnings were retained.

Logs: `build/train-2-configure.log`, `build/train-2-complete-build.log`, `build/train-2-final-managed-build.log`, `build/train-2-all-tests.log`, `build/train-2-final-focused-tests.log`, `build/train-2-scenarios.log`, `build/train-2-final-managed-tests.log`. Acceptance fixture regeneration matched SHA256 `CD8C597C4B8E5A83937E12C94C99BAFACE951F0AA46F25785C25784B32F0F333` byte-for-byte.

Runtime test prerequisites are unchanged: stage `QuantumNative.dll`, the configured Vulkan loader, existing `assets` and `shaders` beside the managed executables. No machine-specific paths were added to project files. Logs live under `build/train-2-*.log`.

### Changed-file manifest

51 changed or added files:

```text
core/CMakeLists.txt
core/include/quantum/coaster/AuthoredTrack.hpp
core/include/quantum/coaster/TrainConfiguration.hpp
core/include/quantum/physics/CarPose.hpp
core/include/quantum/physics/RideOperations.hpp
core/include/quantum/physics/TrackFollower.hpp
core/include/quantum/physics/TrainDefinition.hpp
core/include/quantum/physics/TrainPhysics.hpp
core/src/AuthoredTrack.cpp
core/src/CoasterDocument.cpp
core/src/TrainConfiguration.cpp
core/src/TrainConfigurationJson.cpp
core/src/physics/RideOperations.cpp
docs/architecture.md
docs/train-2.md
editor/CMakeLists.txt
editor/include/quantum/editor/ApplicationService.hpp
editor/include/quantum/editor/EditorUi.hpp
editor/include/quantum/editor/SimulationPreview.hpp
editor/include/quantum/editor/TrainMeshPresentation.hpp
editor/include/quantum/editor/TrainPreviewInspection.hpp
editor/managed/Quantum.Editor/EditorApplication.axaml.cs
editor/managed/Quantum.Editor/EditorViewModel.Supports.cs
editor/managed/Quantum.Editor/EditorViewModel.Train.cs
editor/managed/Quantum.Editor/EditorViewModel.cs
editor/managed/Quantum.Editor/MainWindow.axaml
editor/managed/Quantum.Editor/RuntimeChecks.Train.cs
editor/managed/Quantum.Editor/TrainBrowser.axaml
editor/managed/Quantum.Editor/TrainBrowser.axaml.cs
editor/managed/Quantum.Editor/TrainDiagnosticsPane.axaml
editor/managed/Quantum.Editor/TrainDiagnosticsPane.axaml.cs
editor/managed/Quantum.Editor/TrainInspector.axaml
editor/managed/Quantum.Editor/TrainInspector.axaml.cs
editor/managed/Quantum.Managed.Tests/Program.cs
editor/managed/Quantum.Managed.Tests/TrainChecks.cs
editor/managed/Quantum.Managed/Session.Train.cs
editor/managed/Quantum.Managed/Session.cs
editor/native/CMakeLists.txt
editor/native/NativeViewport.Train.cpp
editor/native/NativeViewport.cpp
editor/native/NativeViewport.hpp
editor/native/QuantumNative.cpp
editor/native/QuantumNative.h
editor/native/QuantumTrainBoundaryTests.cpp
editor/src/ApplicationService.cpp
editor/src/EditorUiTrain.cpp
editor/src/SimulationPreview.cpp
editor/src/TrainMeshPresentation.cpp
smoke-tests/train-2-acceptance.quantum
tests/Train2Tests.cpp
tests/TrainVisualConsistTests.cpp
```
