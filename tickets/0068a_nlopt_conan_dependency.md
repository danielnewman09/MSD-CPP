# Ticket 0068a: Add NLopt Conan Dependency

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Review
**Type**: Infrastructure
**Priority**: High
**Created**: 2026-02-16
**Generate Tutorial**: No
**Parent Ticket**: [0068_nlopt_friction_cone_solver](0068_nlopt_friction_cone_solver.md)
**Depends On**: None

---

## Overview

Add NLopt as a Conan dependency and wire it into the msd-sim CMake build. This is the foundation for the NLopt friction cone solver — without it, nothing else in 0068 can build.

NLopt is already listed in `conanfile.py` (`nlopt/2.10.0`). This ticket completes the CMake integration: `find_package`, `target_link_libraries`, and build verification.

---

## Requirements

### R1: CMake Integration

Update `msd/msd-sim/CMakeLists.txt`:
- `find_package(NLopt REQUIRED)` (or `nlopt` — verify Conan's package name)
- `target_link_libraries(msd_sim PRIVATE nlopt::nlopt)`

### R2: Build Verification

- `conan install . --build=missing -s build_type=Debug`
- `cmake --preset conan-debug`
- `cmake --build --preset debug-sim-only` compiles without errors
- A minimal NLopt include in any msd-sim source (e.g., a test) resolves correctly

### R3: Verify NLopt Header Availability

Create a minimal compilation test (can be a temporary test case or just verify that `#include <nlopt.hpp>` compiles in a new test file).

---

## Acceptance Criteria

1. `cmake --build --preset debug-sim-only` succeeds with NLopt linked
2. `#include <nlopt.hpp>` resolves in msd-sim sources
3. No existing tests regress

---

## Files of Interest

| File | Relevance |
|------|-----------|
| `conanfile.py` | Already has `nlopt/2.10.0` — verify correct |
| `msd/msd-sim/CMakeLists.txt` | Add find_package + link |

---

## Notes

- NLopt is MIT-licensed (compatible with project)
- Conan package name may be `nlopt` or `NLopt` — check generator output
- NLopt should be PRIVATE dependency (not exposed in msd-sim public headers)

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-02-16 22:17
- **Completed**: 2026-02-16 22:18
- **Branch**: 0068-nlopt-friction-cone-solver
- **PR**: #71 (draft)
- **Artifacts**:
  - `msd/msd-sim/test/Physics/Constraints/NLoptDependencyTest.cpp` (verification test)
  - `docs/designs/0068_nlopt_friction_cone_solver/iteration-log.md`
  - `docs/designs/0068_nlopt_friction_cone_solver/implementation-notes-0068a.md`
- **Commit**: f382ff4
- **Notes**:
  - NLopt dependency was already present in conanfile.py and msd-sim CMakeLists.txt
  - Added NLopt::nlopt to test target link libraries (required for direct header access in tests)
  - Created minimal verification test with 2 test cases (header inclusion, optimizer creation)
  - All acceptance criteria met: build succeeds, headers accessible, no regressions (727/734 baseline maintained)
  - Test will be removed after 0068b implements NLoptFrictionSolver with comprehensive tests
