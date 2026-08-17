# QUANTUM CoasterWorks — Agent Instructions

## Project

QUANTUM CoasterWorks is a native C++ roller coaster design, simulation, and visualization application.

The project is being developed incrementally. Preserve the existing architecture and make small, understandable changes.

QUANTUM is not a throwaway prototype. Code should be maintainable by a human developer who is actively learning and working on the engine.

## Current Technical Direction

Primary stack:

* Modern C++ / C++23
* CMake
* SDL3 for windowing and platform integration
* Vulkan for rendering
* GLM for mathematics
* Vulkan Memory Allocator (VMA) for Vulkan memory management where appropriate
* EnTT for ECS where an ECS is actually needed
* Assimp for model importing
* OpenAL Soft for audio
* Dear ImGui may be used for editor UI

Do not replace these technologies or introduce competing frameworks unless explicitly requested.

Do not migrate the project to Unreal Engine, Unity, GLFW, DirectX, OpenGL, or another engine/rendering architecture without explicit instruction.

---

# Core Rule: Avoid AI Slop

Prefer the smallest correct implementation that fits the current architecture.

Do NOT:

* invent unnecessary abstractions
* create speculative systems for hypothetical future requirements
* introduce managers, factories, registries, service locators, or dependency-injection systems without a concrete need
* create interfaces when there is currently only one implementation
* wrap every Vulkan object in a separate class merely because it is possible
* create giant utility libraries
* duplicate functionality that already exists
* rewrite working code simply to make it look more sophisticated
* perform unrelated cleanup during a focused task
* rename existing concepts without a concrete reason
* introduce dependencies without explicit permission
* silently redesign public APIs
* turn a small task into a large refactor

Simple and understandable is preferred over clever.

---

# Incremental Development

QUANTUM is built one working milestone at a time.

When implementing a requested feature:

1. Inspect the existing implementation first.
2. Read relevant headers, source files, CMake configuration, and documentation.
3. Determine the smallest change required.
4. Implement only that change.
5. Build the affected target.
6. Fix errors caused by the change.
7. Report exactly what changed.

Do not implement future milestones unless explicitly requested.

For example, if the task is:

> Create the Vulkan surface.

Do not also implement:

* physical-device selection
* logical-device creation
* queue-family management
* swapchains
* command buffers
* synchronization
* render passes
* shaders
* rendering architecture

unless those items are specifically requested.

---

# Architecture

Keep responsibilities separated.

Current intended high-level structure:

* `Application` controls application lifetime and the main loop.
* SDL3 handles the native window and platform events.
* Renderer-related Vulkan code belongs under the renderer subsystem.
* `VulkanContext` owns fundamental Vulkan context resources appropriate to its current stage.
* Editor-specific behavior belongs in the editor application rather than the reusable engine.

Do not turn `Application` into a god class.

Do not put arbitrary Vulkan implementation code into `Application.cpp` when it belongs in the renderer.

Do not create new architectural layers merely to prepare for possible future features.

If an architectural change appears necessary, explain why before implementing a large redesign.

---

# Vulkan Rules

Vulkan resource lifetime and ownership must be explicit.

When adding a Vulkan resource:

* identify who owns it
* initialize handles to `VK_NULL_HANDLE`
* destroy owned resources
* destroy resources in the correct dependency order
* avoid use-after-destroy lifetime problems
* preserve exception/failure cleanup
* do not duplicate ownership

Prefer RAII where it makes ownership clearer, but do not build an elaborate custom RAII framework without explicit approval.

Use SDL3 Vulkan integration APIs where SDL owns the platform integration responsibility.

Use VMA for Vulkan memory allocation once resources requiring allocator-backed memory are introduced, rather than creating an unnecessary parallel allocation framework.

Do not hide important Vulkan behavior behind excessive abstraction.

The developer should still be able to understand which Vulkan operations occur.

---

# C++ Style

Prefer readable modern C++.

Use:

* clear types
* explicit ownership
* RAII
* `const` where meaningful
* `[[nodiscard]]` where ignoring a result would reasonably be a mistake
* scoped namespaces
* small focused functions
* descriptive names

Avoid:

* clever template metaprogramming without necessity
* unnecessary inheritance
* deep inheritance hierarchies
* macros where normal C++ works
* premature generic programming
* excessive forwarding wrappers
* unexplained magic constants
* unnecessary singletons
* unnecessary global state

Do not use advanced C++ merely to demonstrate advanced C++.

A straightforward implementation is preferred.

---

# Error Handling

Do not ignore failures from SDL or Vulkan operations.

Errors should contain enough information to identify the failed operation.

When an SDL API provides useful information through `SDL_GetError()`, preserve it where appropriate.

When debugging compiler errors, identify the root syntax/type/API problem instead of applying random edits to downstream errors.

One malformed declaration or bracket can generate many secondary errors. Fix the earliest root cause first.

---

# Editing Rules

Keep patches tightly scoped.

Do not modify unrelated files.

Do not reformat entire files for a small change.

Do not change whitespace across large sections unnecessarily.

Do not rewrite existing comments unless they are incorrect or affected by the task.

Preserve the project's naming conventions.

Before creating a new file, determine whether the functionality naturally belongs in an existing class or subsystem.

Before creating a new class, explain what unique responsibility requires that class.

---

# Dependencies

Do not add a new third-party dependency unless explicitly requested.

Before implementing functionality already provided by an approved dependency, check whether that dependency should be used instead.

Approved/planned project libraries include:

* SDL3
* Vulkan
* GLM
* VMA
* EnTT
* Assimp
* OpenAL Soft
* Dear ImGui

Their presence does not mean they must be used everywhere.

Use a dependency only where its responsibility actually applies.

---

# CMake and Build System

Treat the existing CMake configuration as authoritative.

Before changing build configuration:

* inspect `CMakeLists.txt`
* inspect relevant nested CMake files
* inspect `CMakePresets.json` if present
* preserve existing target boundaries

Do not reorganize the entire build system for a localized task.

Do not hard-code machine-specific absolute paths into project files.

Do not assume a dependency installation location when the existing CMake configuration already resolves it.

---

# Documentation

`AGENTS.md` defines working rules, not the entire architecture.

For deeper project information, inspect relevant files under:

* `docs/`
* `docs/architecture.md`
* existing source/header comments

Do not duplicate large architecture documents into `AGENTS.md`.

If implementation behavior materially changes documented architecture, point out the documentation mismatch.

Do not automatically rewrite architecture documentation unless requested or clearly required by the change.

---

# Roller Coaster Domain Code

QUANTUM's coaster geometry and physics are core project functionality.

Do not casually replace existing mathematical models.

When working with:

* track centerlines
* B-Splines
* NURBS
* heartline calculations
* banking
* roll
* curvature
* radius
* arc length
* rotation-minimizing frames
* speed
* acceleration
* G-forces
* track sections

preserve the project's existing mathematical conventions.

Do not invent formulas.

Do not silently substitute a simpler mathematical model because it is easier to implement.

When changing numerical or geometry code, explain the mathematical meaning of the change.

---

# Performance

Do not prematurely optimize.

Correctness and architectural clarity come first.

However:

* avoid obviously unnecessary per-frame allocations
* avoid unnecessary copies of large data
* avoid knowingly poor algorithms in performance-critical rendering or geometry paths

Do not introduce complicated caching, threading, SIMD, GPU compute, or job systems until measurements or requirements justify them.

Measure before performing major optimization work.

---

# Comments

Comments should explain:

* why something exists
* ownership/lifetime constraints
* unusual API requirements
* non-obvious mathematics
* important architectural decisions

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

---

# Tests and Verification

After modifying code:

1. Build the affected target.
2. Report compilation errors if they cannot be resolved within task scope.
3. Run relevant tests when they exist.
4. Do not claim something was tested if it was not actually tested.

Do not alter tests merely to make failing behavior pass unless the expected behavior has legitimately changed.

Do not disable warnings or validation just to hide an error.

---

# Working With Existing Code

Existing working code is evidence.

Prefer extending established project patterns over inventing a new style.

Before implementing something new, inspect nearby code for:

* naming
* ownership
* error handling
* file organization
* API style

Do not assume existing code is wrong merely because another implementation would also work.

If existing code contains a genuine defect, distinguish that defect from optional style improvements.

---

# Agent Communication

After completing a task, summarize:

* files changed
* what was implemented
* important ownership/lifetime decisions
* build/test result
* anything intentionally left for a later milestone

If the requested implementation would require a significant architectural decision, identify that decision clearly.

Do not bury major architectural changes inside an implementation patch.

---

# Scope Discipline

If asked to implement X, implement X.

Do not implement:

> X + everything that might eventually depend on X.

Do not attempt to "finish QUANTUM."

The project should advance through small, reviewable, working increments.

When uncertain between:

* a small concrete implementation
* a broad flexible abstraction

choose the small concrete implementation unless current requirements already demonstrate the need for the abstraction.

---

# Human Understandability

The human developer remains the architect.

Generated code must be understandable enough that the developer can maintain it without depending permanently on an AI agent.

Do not intentionally produce code whose complexity exceeds the requirements of the feature.

The goal is not maximum code generation.

The goal is a clean, understandable QUANTUM CoasterWorks codebase.
