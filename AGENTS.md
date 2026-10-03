# QUANTUM — Agent Instructions

## Project

QUANTUM is a native C++ roller coaster design, simulation, and visualization application.

The project is being developed incrementally.

Preserve the existing architecture, behavior, mathematical conventions, file formats, and established project patterns unless a task explicitly requires changing them.

QUANTUM is not a throwaway prototype.

Code must remain maintainable by a human developer who is actively learning C++, graphics programming, simulation, and the architecture of the engine.

The human maintainer remains the architect.

---

# Core Rule: Avoid AI Slop

Prefer the smallest correct implementation that fits the current architecture.

Do NOT:

- invent unnecessary abstractions
- create speculative systems for hypothetical future requirements
- introduce managers, factories, registries, service locators, or dependency-injection systems without a concrete need
- create interfaces when there is currently only one implementation
- wrap every Vulkan object in a separate class merely because it is possible
- create giant utility libraries
- duplicate functionality that already exists
- rewrite working code simply to make it look more sophisticated
- perform unrelated cleanup during a focused task
- rename existing concepts without a concrete reason
- introduce dependencies without explicit permission
- silently redesign public APIs
- turn a small task into a large refactor
- redesign working systems merely because another architecture is fashionable
- generate large amounts of code that the human maintainer cannot reasonably understand

Simple and understandable is preferred over clever.

The goal is not maximum code generation.

The goal is a clean, understandable, maintainable QUANTUM codebase.

---

# Mandatory Human-Learning Workflow

The human maintainer is actively learning C++ and the QUANTUM architecture.

AI assistance MUST increase the maintainer's understanding of the project rather than merely producing finished files.

These requirements apply to every non-trivial implementation, refactor, bug fix, optimization, or architectural change unless the maintainer explicitly overrides them for that specific task.

## Before Changing Code

Before making a substantial change, explain:

1. What subsystem is involved.
2. Which files are relevant.
3. Which classes, structs, functions, or data structures are important.
4. What those pieces currently do.
5. The relevant control flow or data flow.
6. Important ownership or lifetime relationships.
7. What specifically needs to change.
8. Why the change belongs where it is being made.
9. Important C++ concepts involved in the change.

Do not require the maintainer to read an entire multi-thousand-line source file to understand a task.

Point to the smallest useful functions, classes, or source sections.

Distinguish between:

- concepts that must be understood now
- concepts that are useful background
- advanced details that can wait

Do not overwhelm the maintainer with unrelated language or architecture concepts.

## During Implementation

Prefer one understandable change at a time.

Do NOT combine:

- feature implementation
- architectural redesign
- unrelated cleanup
- speculative future work
- large-scale modernization

into one patch unless explicitly requested.

For cleanup work, prefer behavior-preserving changes before ownership or architecture redesign.

When a large class contains several cohesive implementation areas, moving existing member-function implementations into focused `.cpp` files may be preferable to inventing new classes.

Do not split code merely to reduce line counts.

Large cohesive numerical or mathematical implementations may legitimately remain large.

## After Changing Code

A non-trivial task MUST NOT end with only statements such as:

- "Done."
- "Implemented."
- "Tests pass."
- "Feature complete."
- "Changes applied."

After every substantial change, report:

1. What changed.
2. What stayed the same.
3. Why the change was made.
4. How the affected code works.
5. Important C++ concepts demonstrated by the change.
6. The smallest files or functions the maintainer should inspect.
7. Relevant before/after control flow or data flow.
8. Ownership or lifetime implications where relevant.
9. What was built.
10. What tests were run.
11. What was not verified.
12. Remaining risks, assumptions, or limitations.

The agent may recommend one logical next step.

Do NOT automatically perform that next step.

---

# Teaching Mode

When the maintainer asks to:

- learn
- study
- understand
- inspect
- review
- walk through
- explain

existing code, switch to Teaching Mode.

In Teaching Mode:

- Do not modify repository files unless explicitly requested.
- Teach one small concept or code path at a time.
- Prefer real QUANTUM code over unrelated toy examples when practical.
- Explain unfamiliar syntax in plain language.
- Favor concrete examples before abstract explanations.
- Use recognition before demanding free-form recall.
- Do not turn explanations into quizzes unless the maintainer requests one.
- Typographical mistakes by the maintainer are not conceptual failures.
- Do not dump complete replacement implementations when the purpose is learning.
- Do not immediately solve every exercise for the maintainer.
- Explain how code connects to the surrounding QUANTUM system.
- Point out what can safely be ignored until later.

The purpose of Teaching Mode is understanding, not task throughput.

---

# Incremental Development

QUANTUM is built one working milestone at a time.

When implementing a requested feature:

1. Inspect the existing implementation first.
2. Read relevant headers.
3. Read relevant source files.
4. Inspect relevant CMake configuration.
5. Inspect relevant documentation.
6. Inspect nearby tests.
7. Determine the smallest change required.
8. Explain the intended change.
9. Implement only that change.
10. Build the affected target.
11. Fix errors caused by the change.
12. Run relevant tests.
13. Explain the resulting implementation.
14. Report exactly what changed.

Do not implement future milestones unless explicitly requested.

For example, if the task is:

> Create the Vulkan surface.

Do not also implement:

- physical-device selection
- logical-device creation
- queue-family management
- swapchains
- command buffers
- synchronization
- render passes
- shaders
- rendering architecture

unless those items are specifically requested.

---

# Read Before Modifying

Existing working code is evidence.

Do not assume architecture from filenames alone.

Before modifying a subsystem, inspect enough existing code to understand, where relevant:

- ownership
- lifetime
- inputs
- outputs
- callers
- consumers
- data flow
- invalidation
- publication
- serialization
- error handling
- tests
- naming conventions
- established API patterns

Do not replace an existing pattern merely because another implementation would also work.

If existing code contains a genuine defect, distinguish that defect from optional style improvements.

---

# Architecture

Keep responsibilities separated.

Current intended high-level structure includes:

- `Application` controls application lifetime and the main loop.
- SDL3 handles the native window and platform events.
- Renderer-related Vulkan code belongs under the renderer subsystem.
- `VulkanContext` owns fundamental Vulkan context resources appropriate to its current stage.
- Editor-specific behavior belongs in the editor application rather than the reusable engine.
- Core coaster-domain code should remain independent from editor presentation where practical.

Do not turn `Application` into a god class.

Do not put arbitrary Vulkan implementation code into `Application.cpp` when it belongs in the renderer.

Do not turn editor UI implementation files into dumping grounds for unrelated responsibilities.

Do not create new architectural layers merely to prepare for possible future features.

If an architectural change appears necessary:

1. explain the current limitation
2. explain the proposed responsibility boundary
3. explain why a smaller change is insufficient
4. identify compatibility risks
5. obtain explicit direction before performing a broad redesign

---

# Large Files and Refactoring

File size alone is not evidence of bad architecture.

Before splitting a large file, identify a real responsibility boundary.

Good reasons to split a file include:

- several clearly independent UI areas
- unrelated renderer responsibilities
- independent implementation groups belonging to the same public class
- significantly improved navigation without ownership changes

Bad reasons include:

- arbitrary line-count targets
- making metrics look cleaner
- creating one class per concept regardless of need
- introducing abstractions solely to move code elsewhere

When possible, prefer moving cohesive existing implementations into responsibility-specific source files before redesigning ownership.

Do not create new classes solely to make another file shorter.

---

# Current Technical Direction

Primary project technologies include:

- Modern C++ / C++23
- CMake
- SDL3 for windowing and platform integration
- Vulkan for rendering
- GLM for mathematics
- Vulkan Memory Allocator (VMA) where appropriate
- Dear ImGui for editor UI

Libraries that are approved or planned for responsibilities where they are actually needed may include:

- EnTT
- Assimp
- OpenAL Soft

The presence or approval of a library does NOT mean it must be used everywhere.

Before using an approved dependency, verify that the current subsystem actually uses it or that the requested feature requires it.

Do not introduce competing frameworks without explicit permission.

Do not migrate the project to:

- Unreal Engine
- Unity
- GLFW
- DirectX
- OpenGL
- another rendering architecture
- another application framework

without explicit instruction.

---

# C++ Style

Prefer readable modern C++.

Use:

- clear types
- explicit ownership
- RAII
- `const` where meaningful
- `[[nodiscard]]` where ignoring a result would reasonably be a mistake
- scoped namespaces
- small focused functions where practical
- descriptive names
- standard library facilities where they improve clarity
- value semantics where appropriate

Avoid:

- clever template metaprogramming without necessity
- unnecessary inheritance
- deep inheritance hierarchies
- macros where normal C++ works
- premature generic programming
- excessive forwarding wrappers
- unexplained magic constants
- unnecessary singletons
- unnecessary global state
- advanced syntax merely to appear modern
- abstraction for abstraction's sake

Do not use advanced C++ merely to demonstrate advanced C++.

A straightforward implementation is preferred.

When advanced C++ is genuinely useful, explain why it improves the implementation.

---

# Vulkan Rules

Vulkan resource lifetime and ownership must remain explicit.

When adding or changing a Vulkan resource:

- identify who owns it
- identify its lifetime
- initialize handles appropriately, commonly to `VK_NULL_HANDLE`
- destroy owned resources
- destroy resources in correct dependency order
- avoid use-after-destroy lifetime problems
- preserve failure cleanup
- preserve exception cleanup
- do not duplicate ownership
- respect synchronization requirements
- respect existing frames-in-flight assumptions

Prefer RAII where it makes ownership clearer.

Do not create an elaborate custom Vulkan RAII framework without explicit approval.

Use SDL3 Vulkan integration APIs where SDL owns the platform integration responsibility.

Use VMA where allocator-backed Vulkan resources require it rather than inventing a parallel memory-allocation framework.

Do not hide important Vulkan behavior behind excessive abstraction.

The human maintainer should still be able to determine:

- which Vulkan operations occur
- who owns each resource
- when resources are created
- when resources are destroyed
- how synchronization works

---

# Rendering Architecture

Do not assume that changing the rendering abstraction will automatically improve visual quality.

Visual quality and rendering architecture are separate concerns.

Do not perform a full rendering-hardware-interface rewrite merely to:

- improve lighting
- improve materials
- add shadows
- improve ground rendering
- improve sky rendering
- add post-processing
- improve presentation quality

Prefer focused improvements to the existing renderer when they solve the actual problem.

If renderer decomposition is needed, split by concrete Vulkan responsibility before proposing a universal RHI.

---

# Error Handling

Do not ignore failures from SDL, Vulkan, file loading, serialization, asset loading, or other important operations.

Errors should contain enough information to identify the failed operation.

When an SDL API provides useful information through `SDL_GetError()`, preserve it where appropriate.

When debugging compiler errors, identify the root syntax, type, ownership, or API problem instead of applying random edits to downstream errors.

One malformed declaration, missing delimiter, or incorrect type can create many secondary compiler errors.

Fix the earliest root cause first.

Do not broaden exception handling merely to silence crashes without understanding what exception types are actually possible.

---

# Editing Rules

Keep patches tightly scoped.

Do not modify unrelated files.

Do not reformat entire files for a small change.

Do not change whitespace across large sections unnecessarily.

Do not rewrite existing comments unless:

- they are incorrect
- the implementation they describe changed
- clarification is directly relevant to the task

Preserve project naming conventions.

Before creating a new file, determine whether the functionality naturally belongs in an existing subsystem.

Before creating a new class, explain what unique responsibility requires that class.

Before renaming a public concept, determine:

- serialization impact
- test impact
- documentation impact
- asset impact
- compatibility impact

---

# Dependencies

Do not add a new third-party dependency unless explicitly requested.

Before implementing functionality already provided by an approved dependency, determine whether that dependency is appropriate for the current subsystem.

Approved or planned libraries may include:

- SDL3
- Vulkan
- GLM
- VMA
- EnTT
- Assimp
- OpenAL Soft
- Dear ImGui

Their presence does not mean they must be used everywhere.

Do not add a dependency simply because it could make an implementation shorter.

Do not replace existing focused code with a general-purpose dependency without a demonstrated need.

---

# CMake and Build System

Treat the existing CMake configuration as authoritative.

Before changing build configuration:

- inspect root `CMakeLists.txt`
- inspect relevant nested CMake files
- inspect `CMakePresets.json` if present
- preserve target boundaries
- preserve dependency-resolution conventions
- preserve supported build configurations

Do not reorganize the entire build system for a localized task.

Do not hard-code machine-specific absolute paths into project files.

Do not assume dependency installation locations when existing CMake configuration already resolves them.

When adding a new source file, verify that it is included in the appropriate target.

---

# Documentation

`AGENTS.md` defines working rules.

It is not intended to duplicate the entire architecture.

For deeper project information, inspect relevant material under:

- `docs/`
- `docs/architecture.md`
- source comments
- header comments
- tests
- serialization fixtures where relevant

If implementation behavior materially changes documented architecture, point out the documentation mismatch.

Do not automatically rewrite large documentation files unless requested or clearly required by the implementation change.

Documentation should reflect reality.

Do not alter documentation merely to make an implementation appear compliant.

---

# Roller Coaster Domain Code

QUANTUM's coaster geometry and physics are core project functionality.

Do not casually replace existing mathematical models.

When working with:

- track centerlines
- B-Splines
- NURBS
- heartline calculations
- banking
- roll
- curvature
- radius
- arc length
- rotation-minimizing frames
- speed
- acceleration
- G-forces
- force-vector design
- track sections
- train geometry
- bogie placement
- contact forces
- reaction forces

preserve the project's existing mathematical conventions unless a task explicitly changes them.

Do not invent formulas.

Do not silently substitute a simpler mathematical model because it is easier to implement.

Do not replace a numerically robust method with a simpler method merely to shorten the code.

When changing numerical or geometry code:

1. explain the mathematical meaning of the change
2. identify the invariant being preserved or changed
3. identify relevant tolerances
4. identify regression tests
5. report numerical consequences where measurable

---

# Numerical and Physics Refactoring

Do NOT refactor complicated numerical code merely because it is large or difficult to read.

Exercise particular caution around:

- rider-frame integration
- circuit-completion mathematics
- train closure or root solving
- bogie placement
- contact/reaction solvers
- dynamic-contact physics
- sensitivity calculations
- numerical fallbacks
- convergence logic
- integration policy

Such code requires:

- a concrete reason for modification
- understanding of current invariants
- focused regression protection
- preservation of numerical behavior unless change is intentional

A mathematically cohesive 3,000-line implementation may be safer than an artificially fragmented design.

Do not split mathematical algorithms across arbitrary files simply to satisfy style preferences.

---

# Track Systems and Presentation

Track configuration, visual style, geometry, and authored-document identity are related but distinct concepts.

Do not silently make a temporary visual preset into a permanent universal coaster identity.

Do not rename stable configuration identifiers casually.

When changing track styles or configuration behavior, consider:

- existing documents
- serialization compatibility
- missing-style behavior
- test fixtures
- default new-document behavior
- authored overrides
- generated geometry
- presentation invalidation

Production visual assets should follow established project asset conventions.

Do not substitute placeholder geometry for final production assets without making the placeholder status explicit.

---

# Asset Workflow

Preserve the project's established asset pipeline.

Do not silently introduce a competing source-asset workflow.

When dealing with 3D model assets, preserve the project's current GLB/glTF-oriented workflow unless explicitly directed otherwise.

Legacy or external formats may be used as import/export/intermediate formats when required by a target tool, but should not silently replace the established editable/source asset direction.

Do not embed large generated assets into source code unless there is a concrete technical reason.

---

# Serialization and Compatibility

Saved QUANTUM documents are user data.

Treat serialization compatibility as a real contract.

When modifying serialized data:

- inspect current format version handling
- inspect existing defaults
- inspect missing-field behavior
- inspect unknown identifier behavior
- inspect compatibility tests
- preserve historical documents when practical

Do not silently migrate identifiers merely because naming conventions changed.

Do not rewrite existing documents unless migration is explicitly required.

If a breaking serialization change is unavoidable, identify it clearly before implementation.

---

# Performance

Do not prematurely optimize.

Correctness and architectural clarity come first.

However:

- avoid obviously unnecessary per-frame allocations
- avoid unnecessary copies of large data
- avoid knowingly poor algorithms in performance-critical paths
- preserve known frame-lifetime assumptions
- avoid rebuilding expensive resources without cause

Do not introduce complicated:

- caching
- threading
- SIMD
- GPU compute
- job systems
- asynchronous pipelines

until measurements or requirements justify them.

Measure before performing major optimization work.

When optimizing, report:

- the measured bottleneck
- the measurement method
- before results
- after results
- tradeoffs introduced

Do not claim an optimization based solely on intuition.

---

# Comments

Comments should primarily explain:

- why something exists
- ownership/lifetime constraints
- unusual API requirements
- non-obvious mathematics
- numerical assumptions
- compatibility requirements
- important architectural decisions

Do not comment obvious syntax.

Bad:

```cpp
// Set running to true
bool running = true;
```

Useful:

```cpp
// Vulkan resources must be destroyed before the SDL window that owns
// the presentation surface is destroyed.
```

Do not generate excessive comments that merely restate each line of code.

---

# Tests and Verification

After modifying code:

1. Build the affected target.
2. Fix compilation errors caused by the change.
3. Run relevant tests when they exist.
4. Run broader tests when the risk justifies them.
5. Clearly state what was and was not tested.

Do not claim something was tested if it was not actually tested.

Do not claim runtime behavior was verified if only static inspection was performed.

Do not alter tests merely to make failing behavior pass unless expected behavior has legitimately changed.

Do not weaken assertions merely to accommodate a regression.

Do not disable warnings, validation, or tests just to hide an error.

If a test cannot be run, state why.

Manual runtime validation must be identified separately from automated testing.

---

# Bug Fixes

When fixing a defect:

1. identify the observed or reported behavior
2. identify the actual root cause
3. distinguish the defect from unrelated technical debt
4. make the smallest reliable fix
5. add or update regression coverage where practical
6. explain why the fix addresses the cause rather than only the symptom

Do not rewrite an entire subsystem merely because a localized bug exposed it.

Do not silently fix unrelated defects encountered during a focused task.

Report unrelated findings separately.

---

# Cleanup and Refactoring

Cleanup must have a concrete purpose.

Good cleanup goals include:

- making an existing responsibility easier to locate
- eliminating demonstrated duplication
- fixing unsafe lifetime behavior
- clarifying an ownership boundary
- reducing accidental coupling
- correcting stale invalidation or publication logic
- making a subsystem more understandable without changing behavior

Bad cleanup goals include:

- "modernize everything"
- "make it enterprise"
- "make it scalable"
- "use more patterns"
- "reduce every file below N lines"
- "rewrite because AI would design it differently"

For behavior-preserving refactors:

- preserve observable behavior
- preserve serialization
- preserve stable identifiers
- preserve mathematical conventions
- preserve solver tolerances
- preserve ownership assumptions unless ownership is the explicit subject
- preserve renderer publication behavior
- preserve fixed-step simulation behavior
- preserve relevant tests and fixtures

If one of these cannot be preserved, explain the reason before changing it.

---

# Working With Existing Code

Existing working code is evidence.

Prefer extending established project patterns over inventing a new style.

Before implementing something new, inspect nearby code for:

- naming
- ownership
- error handling
- file organization
- API style
- testing style
- serialization patterns
- invalidation behavior

Do not assume existing code is wrong merely because another implementation would also work.

If existing code contains a genuine defect, distinguish that defect from optional style improvements.

---

# Agent Communication

Communication is part of the implementation.

Before substantial work, explain the relevant subsystem and intended scope.

During long or complex work, report meaningful findings rather than only announcing activity.

After completing a task, summarize:

- files changed
- what was implemented
- what was intentionally preserved
- how the changed code works
- important ownership/lifetime decisions
- important C++ concepts involved
- build result
- test result
- manual validation status
- anything intentionally left for a later milestone

If the requested implementation would require a significant architectural decision, identify that decision clearly.

Do not bury major architectural changes inside an implementation patch.

Do not present speculative interpretation as verified behavior.

---

# Scope Discipline

If asked to implement X, implement X.

Do not implement:

> X + everything that might eventually depend on X.

Do not attempt to "finish QUANTUM."

Do not automatically implement the next roadmap item.

The project should advance through small, reviewable, working increments.

When uncertain between:

- a small concrete implementation
- a broad flexible abstraction

choose the small concrete implementation unless current requirements already demonstrate the need for the abstraction.

---

# Human Understandability

The human developer remains the architect.

Generated code must be understandable enough that the developer can eventually maintain it without depending permanently on an AI agent.

AI-generated code that only another AI can reasonably understand is not an acceptable result.

Do not intentionally produce code whose complexity exceeds the requirements of the feature.

Do not hide important behavior behind abstractions solely to make implementation faster.

When code becomes difficult to understand, improve navigability before automatically adding another abstraction layer.

The long-term objective of AI assistance is:

**Make QUANTUM more capable and maintainable while making its human maintainer progressively more capable of understanding, debugging, modifying, and extending the project independently.**

The goal is not maximum AI throughput.

The goal is a clean, understandable, human-owned QUANTUM codebase.