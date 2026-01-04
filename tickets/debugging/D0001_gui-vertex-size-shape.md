# DEBUG TICKET

---

## Issue Summary

**Title:** GUI Objects are Smaller than expected and do not render with the correct shape

**Severity:** 🔴 Critical 

**Type:** [ ] Crash | [x] Wrong Output | [ ] Memory Issue | [ ] Performance | [ ] Build Error | [ ] Other

---

## Problem Description

### What is happening?
When executing the msd-exe application, the user has the ability to add pyramids and cubes at randomized locations. At the default camera angle, these shapes should take up an appreciable percentage of the screen and render as randomly colored 3D objects. Instead, the objects show up as extremely small (<1% of the screen size) and are not rendering properly. The vertices appear to be at incorrect locations (e.g. instead of rendering a pyramid, there appear to be multiple flat triangular surfaces that are not correctly closed as a convex shape)

```
danielnewman@Daniels-MacBook-Pro ~ % /Users/danielnewman/Documents/GitHub/MSD-CPP/build/Debug/debug/msd_exe ; exit;
Using GPU device driver: metal
2026-01-03 15:44:08.000 msd_exe[2818:3880891] Successfully loaded vertex shader: Position3DColorTransform.vert
2026-01-03 15:44:08.000 msd_exe[2818:3880891] Successfully loaded fragment shader: SolidColor.frag
2026-01-03 15:44:08.001 msd_exe[2818:3880891] Registered geometry 'pyramid': index=0, baseVertex=0, vertexCount=18
2026-01-03 15:44:08.001 msd_exe[2818:3880891] Registered geometry 'cube': index=1, baseVertex=18, vertexCount=36
2026-01-03 15:44:08.001 msd_exe[2818:3880891] Registered 2 geometry types, total vertices: 54
2026-01-03 15:44:08.001 msd_exe[2818:3880891] Successfully created graphics pipeline with PositionOnly shader policy
2026-01-03 15:44:08.001 msd_exe[2818:3880891] Total vertex count: 54, Geometry types registered: 2, Vertex size: 36 bytes, Instance size: 32 bytes
2026-01-03 15:44:08.001 msd_exe[2818:3880891]   Geometry 'cube': baseVertex=18, vertexCount=36
2026-01-03 15:44:08.001 msd_exe[2818:3880891]   Geometry 'pyramid': baseVertex=0, vertexCount=18
2026-01-03 15:44:08.024 msd_exe[2818:3880891] Camera at (0.00, 0.00, 5.00)
2026-01-03 15:44:08.024 msd_exe[2818:3880891] Camera at (0.00, 0.00, 5.00)
2026-01-03 15:44:08.024 msd_exe[2818:3880891] Camera at (0.00, 0.00, 5.00)
2026-01-03 15:44:08.024 msd_exe[2818:3880891] Camera at (0.00, 0.00, 5.00)
2026-01-03 15:44:08.024 msd_exe[2818:3880891] DEBUG updateObjects: 0 objects -> 0 instances
2026-01-03 15:44:08.042 msd_exe[2818:3880891] Camera at (0.00, 0.00, 5.00)
2026-01-03 15:44:08.042 msd_exe[2818:3880891] Camera at (0.00, 0.00, 5.00)
2026-01-03 15:44:08.042 msd_exe[2818:3880891] Camera at (0.00, 0.00, 5.00)
2026-01-03 15:44:08.042 msd_exe[2818:3880891] DEBUG updateObjects: 0 objects -> 0 instances
...
2026-01-03 15:44:12.520 msd_exe[2818:3880891] DEBUG updateObjects: 1 objects -> 1 instances
2026-01-03 15:44:12.528 msd_exe[2818:3880891] DEBUG updateObjects: 1 objects -> 1 instances
2026-01-03 15:44:12.543 msd_exe[2818:3880891] Spawned pyramid at (-1.59, -3.06, 4.08) with orientation (137.12, 30.48, 60.78) and color (0.03, 0.01, 0.53). Total objects: 2
2026-01-03 15:44:12.543 msd_exe[2818:3880891] Camera at (0.00, 0.00, 5.00)
2026-01-03 15:44:12.543 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.544 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.553 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.561 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.570 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.578 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.586 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.595 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.603 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.612 msd_exe[2818:3880891] Camera at (0.00, 0.00, 5.00)
2026-01-03 15:44:12.612 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.620 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.628 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.636 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.645 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.653 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.662 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.670 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.678 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.687 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.695 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.703 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.712 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.720 msd_exe[2818:3880891] DEBUG updateObjects: 2 objects -> 2 instances
2026-01-03 15:44:12.730 msd_exe[2818:3880891] Spawned pyramid at (-4.43, 3.11, -3.00) with orientation (-71.53, 102.15, -113.70) and color (0.96, 0.37, 0.29). Total objects: 3
2026-01-03 15:44:12.730 msd_exe[2818:3880891] Camera at (0.00, 0.00, 5.00)
```

### What should happen instead?
When the user creates a shape with the default camera angle, pyramids and/or cubes should be rendered as convex 3D shapes and occupy an easily noticeable percenteage of the screen.

### When does it occur?
- [x] Always reproducible
- [ ] Intermittent (describe frequency: _______)
- [ ] Only under specific conditions (describe: _______)

---

## Reproduction Steps


**Minimal reproduction command:**
With current testing mechanisms, this is only reproducible with a human in the loop

---

## Suspect Code

List the files, classes, and/or functions you believe may be involved:

| File | Class/Function | Why Suspected |
|------|----------------|---------------|
| `msd/msd-gui/SDLGPUManager.hpp` | `SDLGPUManager` | The main SDL orchestrator |
| `msd/msd-gui/ShaderPolicy.hpp` | `PositionOnlyShaderPolicy` | This object contains the shader definitions that are relevant |
| `msd/msd-sim/Environment/ReferenceFrame.hpp` | `ReferenceFrame` | The matrix math here may be incompatible with the sdl rendering matrix math formatting |
| | | |

**Call chain (if known):**
```
```

---

## Build Environment

**Compiler:**
- [X] GCC (version: _____)
- [ ] Clang (version: _____)
- [ ] MSVC (version: _____)

**Build configuration:**
- [X] Debug
- [X] Release
- [X] RelWithDebInfo

**Build system:**
- [X] CMake (version: _____)
- [ ] Make
- [ ] Bazel
- [ ] Other: _____

**OS/Platform:**
- [ ] Linux (distro/version: _____)
- [X] macOS (version: _____)
- [ ] Windows (version: _____)

**Relevant CMake/compile flags:**
None

---

## Dependencies

List relevant dependencies and their versions:

| Dependency | Version | Relevant? |
|------------|---------|-----------|
| | | |

---

## What I've Already Tried


---

## Test Coverage

**Existing tests for suspect code:**
- [X] Yes → Location: `msd/msd-gui/test/**`
- [ ] No
- [ ] Unknown

**Do existing tests pass?**
- [X] Yes
- [ ] No → Which ones fail? _____
- [ ] Haven't run them

---

## Additional Context

### Sanitizer output (if available)
N/A

### Valgrind output (if available)
N/A

### Recent changes
An overhaul of the backend object management occurred recently. The basic functionality of rendering random pyramids worked properly in commit: 7ccd40e31fc781946e9b1fa6dd170e871bc79248

### Related issues
N/A

---

## Initial Hypotheses
N/A
---

## Constraints

- [ ] Cannot modify certain files (list: _____)
- [ ] Must maintain ABI compatibility
- [ ] Performance-critical code
- [ ] Other constraints: _____

---

**Ticket created:** [DATE]
**Created by:** [YOUR NAME/HANDLE]