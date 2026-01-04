# DEBUG TICKET

> Copy this file to your project root as `DEBUG_TICKET.md` and fill in all sections before starting a debug session.

---

## Issue Summary

**Title:** [Brief descriptive title]

**Severity:** 🔴 Critical | 🟠 High | 🟡 Medium | 🟢 Low

**Type:** [ ] Crash | [ ] Wrong Output | [ ] Memory Issue | [ ] Performance | [ ] Build Error | [ ] Other

---

## Problem Description

### What is happening?
[Describe the symptoms. Include exact error messages, stack traces, or unexpected behavior.]

```
[Paste error output, stack trace, or relevant logs here]
```

### What should happen instead?
[Describe the expected/correct behavior.]

### When does it occur?
- [ ] Always reproducible
- [ ] Intermittent (describe frequency: _______)
- [ ] Only under specific conditions (describe: _______)

---

## Reproduction Steps

1. [First step]
2. [Second step]
3. [Third step]
4. [Observe: describe what happens]

**Minimal reproduction command:**
```bash
# Command to reproduce the issue
```

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
- [ ] GCC (version: _____)
- [ ] Clang (version: _____)
- [ ] MSVC (version: _____)

**Build configuration:**
- [ ] Debug
- [ ] Release
- [ ] RelWithDebInfo

**Build system:**
- [ ] CMake (version: _____)
- [ ] Make
- [ ] Bazel
- [ ] Other: _____

**OS/Platform:**
- [ ] Linux (distro/version: _____)
- [ ] macOS (version: _____)
- [ ] Windows (version: _____)

**Relevant CMake/compile flags:**
```cmake
# Paste relevant CMake configuration or compile flags
```

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
- [ ] Yes → Location: `tests/_____`
- [ ] No
- [ ] Unknown

**Do existing tests pass?**
- [ ] Yes
- [ ] No → Which ones fail? _____
- [ ] Haven't run them

---

## Additional Context

### Sanitizer output (if available)
```
[AddressSanitizer/UBSan/TSan output]
```

### Valgrind output (if available)
```
[Valgrind output]
```

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