# Supports 2 local handoff

Branch: `codex/supports-2-structural-styles`.

Started from local `codex/operations-ride-hardware-2`, HEAD
`04241a4a8f0a97a09d45b3a67e37734632dd5711`. No reset, push, PR, or screenshot
publication. The implementation commit is the local HEAD reported in the chat;
`git log -1 --oneline` retrieves it. The maintainer's previously untracked
`assets/materials/{concrete,fiberglass,grout,metal}/` directories remain untracked
and outside this patch.

The [technical audit](supports-2-structural-styles.md) explains the old UV
pipeline, new precedence rules, data flow, ownership, C++ reading path, and limits.

## Builds and automated verification

Commands below ran from `C:\Dev8\QUANTUM`, Windows/MSVC Debug.

```powershell
cmake --build build --config Debug --parallel 4
dotnet build editor/managed/Quantum.Managed.Tests/Quantum.Managed.Tests.csproj -c Debug --no-restore --nologo
build/editor/Debug/QuantumSupports2Tests.exe
ctest --test-dir build -C Debug -R 'Support|Hybrid|Wooden|Operations|TargetSpeed|Hardware|TrainPhysics|TrackPlacement|NativeBoundary|NativeSession' -E 'PerfBenchmark' --output-on-failure
editor/managed/Quantum.Managed.Tests/bin/Debug/net10.0/Quantum.Managed.Tests.exe C:\Dev8\QUANTUM\smoke-tests\modern-steel-validation.quantum
```

Native build succeeded, including QuantumCore, QuantumEngine, QuantumApplication,
QUANTUM, QuantumNative, shaders, and test targets. Managed build succeeded with
zero warnings and errors; it includes Quantum.Managed, Quantum.Editor, and the
managed tests. Native legacy test compilation retains existing ignored-nodiscard
warnings; no warnings or validation were disabled.

The new native suite passed **337 checks**. The focused CTest run passed **28/28**.
The managed wrapper executable passed, including Timeline **126**, Hardware
**143**, existing Supports **1078**, Supports 2 **116**, and Placement **92** checks.
The wrapper's lifecycle, cached snapshots, playback, and launch target comparison
checks also ran. A real QuantumNative.dll was staged beside both managed binaries.

The smoke-discovered shader correction also passed its compiled SPIR-V contract
test in QuantumEngine.SupportSolidRenderer. Offline validation used:

```powershell
& 'C:/VulkanSDK/1.4.350.0/Bin/spirv-val.exe' --target-env vulkan1.3 build/shaders/support.frag.spv
& 'C:/VulkanSDK/1.4.350.0/Bin/spirv-dis.exe' build/shaders/support.frag.spv -o build/support-fragment-final.spvasm
```

Validation succeeded; disassembly declares only `Shader` capability and uses
`OpKill`, with no DemoteToHelperInvocation capability. The SDK path above records
this machine's verification command; project CMake resolves tools normally.
`git diff --check` passed.

Exact CTest names:

- `QuantumApplication.Supports2` — passed.
- `QuantumApplication.RideHardwareAuthoring` — passed.
- `QuantumApplication.SupportsWorkspace` — passed.
- `QuantumApplication.TrackPlacement` — passed.
- `QuantumApplication.HardwareBoundary` — passed.
- `QuantumApplication.NativeBoundary` — passed.
- `QuantumApplication.SupportBoundary` — passed.
- `QuantumCore.TrainPhysics` — passed.
- `QuantumCore.TrackHardwarePlacement` — passed.
- `QuantumEngine.SupportSolidRenderer` — passed.
- `QuantumCore.SupportModel` — passed.
- `QuantumCore.SupportAttachment` — passed.
- `QuantumCore.SupportConnection` — passed.
- `QuantumCore.SupportSerialization` — passed.
- `QuantumEditor.SupportVisualization` — passed.
- `QuantumEditor.SupportNodeManipulation` — passed.
- `QuantumEditor.SupportAuthoring` — passed.
- `QuantumCore.WoodenSupportGenerator` — passed.
- `QuantumCore.SupportMemberRole` — passed.
- `QuantumCore.HybridTimberAccuracy` — passed.
- `QuantumCore.SupportOrientationReference` — passed.
- `QuantumCore.SupportOrientationFrame` — passed.
- `QuantumEditor.SupportMounting` — passed.
- `QuantumEditor.HybridLocalBracing` — passed.
- `QuantumEditor.HybridOuterSupport` — passed.
- `QuantumCore.SupportSolidGeometry` — passed.
- `QuantumCore.RideOperations` — passed.
- `QuantumCore.TargetSpeed` — passed.

## Modest performance check

Single representative Debug CPU runs, no claimed optimization or GPU benchmark:

| Case | CPU presentation generation | Parts / instance bytes | Expanded-equivalent vertices / indices |
|---|---:|---:|---:|
| 100 solid rectangular members | 0.2705 ms | 100 / 8,000 | 2,400 / 3,600 |
| 1,000 solid rectangular members | 2.6839 ms | 1,000 / 80,000 | 24,000 / 36,000 |
| 50 m track-following catwalk | 0.7272 ms | 370 / 29,600 | 8,880 / 13,320 |

The member cases use one draw batch and one shared 24-vertex/36-index box, not
thousands of separately uploaded meshes. HSS uses four boxes per member; I/H
uses three. The shared 16-segment tube has 136 vertices/384 indices. The catwalk
has separate deck and railing materials. GPU frame impact and gate-heavy scene
uploads were not benchmarked; gate fraction changes currently republish the
combined instance stream. Static infrastructure is cached by document revision.

## One bounded live smoke

Ran exactly once, with a 45-second outer process limit:

```powershell
Quantum.Editor.exe --verify-supports-2-smoke C:\Dev8\QUANTUM\smoke-tests\supports-2-structural-styles.quantum C:\Dev8\QUANTUM\build\supports-2-live-smoke.json
```

Functional checks completed in **1.644 seconds**, with **26 Vulkan frames** on an
NVIDIA RTX 4070: opening the five-family document, framing timber, material edit,
Undo/Redo, simulation, and Access open/close telemetry all succeeded.
The process subsequently exited **-1073740791 (0xC0000409)**. The log contained
VUID-VkShaderModuleCreateInfo-pCode-08740 for the new discard/demote shader
capability. The shader was corrected and verified offline. The harness now awaits
MainWindow's asynchronous resource cleanup before requesting desktop shutdown.
The native shutdown abort's actual cause is unconfirmed. **This is not a clean
live-smoke pass.** No second live run was performed.

Evidence remains in ignored local build files: `supports-2-final-build.log`,
`supports-2-managed-build.log`, `supports-2-tests.log`, `supports-2-regressions.log`,
`supports-2-managed-tests.log`, `supports-2-live-smoke.json`, and
`supports-2-smoke-stderr.log`. The corrected shaders and native DLL are staged in
the Avalonia Debug output. No screenshots were taken or published.

## Maintainer visual test and remaining limits

Launch `editor/managed/Quantum.Editor/bin/Debug/net10.0/Quantum.Editor.exe` and
open `smoke-tests/supports-2-structural-styles.quantum`. Use the Avalonia editor;
the historical ImGui editor has no matching new infrastructure controls/publication.

1. In Supports, inspect each family and frame members with the existing Frame
   action. Check long timber grain density/orientation, caps, HSS bores, I/H
   silhouettes, steel/concrete shading, and foundations. Compare a short member
   and a long member with the same repeat distances.
2. Inspect the curved, graded, banking catwalk, its grating and outer guardrail,
   world platform, stairs, and landing. Try deck/rail spacing controls and a
   foundation's Pedestal/Pier modes.
3. Pick nodes/members and infrastructure. Support picks retain priority; picking
   is conservative around holes, treatments, and hollow/I sections. Foundation
   editing is through the node. DPI/physical-pointer testing was not performed.
4. Inspect the referenced Access gate, select Play, and use existing Open/Close.
   Verify its mesh perimeter and leaf motion against telemetry. Try solid infill
   and Slide mode. Interlocks and locks remain Operations-owned.
5. Exercise Undo/Redo, save/reload, and Track Start relocation. Attached objects
   should move; world foundations/platform/stairs should remain fixed. Check
   clean application shutdown explicitly because the single smoke aborted.

Visual quality and clean shutdown remain pending. No Release, Linux GUI, DPI,
pointer, stress-render, or GPU timing validation is claimed. Connection geometry
is procedural engineering approximation; GLB connector loading, paired splices,
bolt detail, tapered members, automatic rail openings, stair routing, and
manufacturer-specific libraries are outside this increment. Regeneration owns
the recipe's four node positions/footings and preserves member overrides plus
added manual objects. New documents with new fields require this reader;
older strict readers will reject them.

After maintainer approval, recommend a **broader procedural support-generation
pass**. No next phase has been started.

Suggested future PR title: **Supports 2: structural families, physical materials,
and native ride infrastructure**. Resolve runtime shutdown/visual acceptance
before presenting this as approved.

## Changed-file manifest

55 files; A = added, M = modified.

- M `core/CMakeLists.txt`
- M `core/include/quantum/coaster/RideHardware.hpp`
- M `core/include/quantum/coaster/SupportSolidGeometry.hpp`
- A `core/include/quantum/coaster/SupportSurface.hpp`
- M `core/include/quantum/coaster/Supports.hpp`
- M `core/include/quantum/coaster/WoodenSupportGenerator.hpp`
- M `core/src/CoasterDocument.cpp`
- M `core/src/RideHardware.cpp`
- M `core/src/RideHardwareJson.cpp`
- A `core/src/SupportFrameGenerator.cpp`
- M `core/src/SupportSolidGeometry.cpp`
- A `core/src/SupportStyles.cpp`
- M `core/src/Supports.cpp`
- M `core/src/WoodenSupportGenerator.cpp`
- A `docs/supports-2-local-handoff.md`
- A `docs/supports-2-structural-styles.md`
- M `editor/CMakeLists.txt`
- M `editor/include/quantum/editor/ApplicationService.hpp`
- A `editor/include/quantum/editor/InfrastructurePresentation.hpp`
- M `editor/managed/Quantum.Editor/EditorApplication.axaml.cs`
- M `editor/managed/Quantum.Editor/EditorViewModel.Hardware.cs`
- A `editor/managed/Quantum.Editor/EditorViewModel.Infrastructure.cs`
- A `editor/managed/Quantum.Editor/EditorViewModel.SupportAppearance.cs`
- M `editor/managed/Quantum.Editor/EditorViewModel.Supports.cs`
- M `editor/managed/Quantum.Editor/EditorViewModel.cs`
- M `editor/managed/Quantum.Editor/HardwareInspector.axaml`
- A `editor/managed/Quantum.Editor/InfrastructureInspector.axaml`
- A `editor/managed/Quantum.Editor/InfrastructureInspector.axaml.cs`
- A `editor/managed/Quantum.Editor/RuntimeChecks.Supports2.cs`
- M `editor/managed/Quantum.Editor/SupportInspector.axaml`
- M `editor/managed/Quantum.Managed.Tests/Program.cs`
- A `editor/managed/Quantum.Managed.Tests/Supports2Checks.cs`
- M `editor/managed/Quantum.Managed/Session.Hardware.cs`
- A `editor/managed/Quantum.Managed/Session.SupportAppearance.cs`
- M `editor/managed/Quantum.Managed/Session.Supports.cs`
- M `editor/native/NativeViewport.cpp`
- M `editor/native/NativeViewport.hpp`
- M `editor/native/QuantumNative.cpp`
- M `editor/native/QuantumNative.h`
- M `editor/src/ApplicationService.Supports.cpp`
- A `editor/src/InfrastructurePresentation.cpp`
- M `editor/src/SupportPicking.cpp`
- M `editor/src/SupportVisualization.cpp`
- M `engine/CMakeLists.txt`
- M `engine/include/quantum/renderer/SupportSolidGeometry.hpp`
- M `engine/include/quantum/renderer/VulkanContext.hpp`
- M `engine/shaders/support.frag`
- M `engine/shaders/support.vert`
- M `engine/src/SupportSolidGeometry.cpp`
- M `engine/src/SupportSolidVulkan.cpp`
- M `engine/src/VulkanContext.cpp`
- A `smoke-tests/supports-2-structural-styles.quantum`
- M `tests/CMakeLists.txt`
- M `tests/SupportSolidRendererTests.cpp`
- A `tests/Supports2Tests.cpp`
