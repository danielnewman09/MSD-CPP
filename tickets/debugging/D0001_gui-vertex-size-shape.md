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
| `src/example.cpp` | `MyClass::method()` | [reason] |
| `include/example.h` | `MyClass` | [reason] |
| | | |

**Call chain (if known):**
```
main() 
  → ComponentA::init()
    → ComponentB::process()
      → [crash/issue occurs here]
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

- [ ] [Describe what you tried and what happened]
- [ ] [Another thing you tried]

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
[Did this work before? What changed? Link to relevant commits if known.]

### Related issues
[Links to related bug reports, similar issues, etc.]

---

## Initial Hypotheses

If you have any theories about what might be wrong:

1. **Hypothesis:** [Your theory]
   **Reasoning:** [Why you think this]

2. **Hypothesis:** [Another theory]
   **Reasoning:** [Why you think this]

---

## Constraints

- [ ] Cannot modify certain files (list: _____)
- [ ] Must maintain ABI compatibility
- [ ] Performance-critical code
- [ ] Other constraints: _____

---

**Ticket created:** [DATE]
**Created by:** [YOUR NAME/HANDLE]