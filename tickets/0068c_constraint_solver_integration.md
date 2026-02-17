# Ticket 0068c: Integrate NLoptFrictionSolver with ConstraintSolver

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Ready for Implementation
**Type**: Feature
**Priority**: High
**Created**: 2026-02-16
**Generate Tutorial**: No
**Parent Ticket**: [0068_nlopt_friction_cone_solver](0068_nlopt_friction_cone_solver.md)
**Depends On**: [0068b](0068b_nlopt_friction_solver_class.md)

---

## Overview

Replace `FrictionConeSolver` with `NLoptFrictionSolver` inside `ConstraintSolver::solveWithFriction()`. This is the integration point — after this ticket, all friction cone solving goes through NLopt. Also remove the old `FrictionConeSolver` and `ConeProjection` files.

---

## Requirements

### R1: Replace Solver Member

In `ConstraintSolver.hpp`:
- Remove `#include "FrictionConeSolver.hpp"`
- Add `#include "NLoptFrictionSolver.hpp"`
- Replace `FrictionConeSolver frictionConeSolver_` with `NLoptFrictionSolver nloptSolver_`

### R2: Update solveWithFriction()

In `ConstraintSolver.cpp`:
- Replace solver invocation:
  ```cpp
  auto nloptResult = nloptSolver_.solve(A, b, spec.frictionCoefficients, lambda0);
  ```
- Map `NLoptFrictionSolver::SolveResult` → `ActiveSetResult`:
  ```cpp
  ActiveSetResult result;
  result.lambda = nloptResult.lambda;
  result.converged = nloptResult.converged;
  result.iterations = nloptResult.iterations;
  result.active_set_size = 0;  // Not applicable
  ```

### R3: Remove Old Solver Files

- Delete `FrictionConeSolver.hpp` / `FrictionConeSolver.cpp`
- Delete `ConeProjection.hpp` / `ConeProjection.cpp`
- Remove from `CMakeLists.txt` if listed in `target_sources()`
- Remove any includes of these files from test code

### R4: Build + Regression Check

- `cmake --build --preset debug-sim-only` compiles cleanly
- Run full test suite: `./build/Debug/debug/msd_sim_test`
- Baseline: 734/741 or better (no regressions from solver swap)

---

## Acceptance Criteria

1. `ConstraintSolver` uses `NLoptFrictionSolver` exclusively
2. `FrictionConeSolver.{hpp,cpp}` and `ConeProjection.{hpp,cpp}` deleted
3. All existing physics tests pass at baseline or better
4. No compilation warnings from the integration

---

## Files to Modify

| File | Change |
|------|--------|
| `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Replace member + include |
| `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Update solveWithFriction() |
| `msd/msd-sim/CMakeLists.txt` | Remove old files, add new files |

## Files to Delete

| File | Reason |
|------|--------|
| `msd/msd-sim/src/Physics/Constraints/FrictionConeSolver.hpp` | Replaced by NLoptFrictionSolver |
| `msd/msd-sim/src/Physics/Constraints/FrictionConeSolver.cpp` | Replaced by NLoptFrictionSolver |
| `msd/msd-sim/src/Physics/Constraints/ConeProjection.hpp` | Only used by FrictionConeSolver |
| `msd/msd-sim/src/Physics/Constraints/ConeProjection.cpp` | Only used by FrictionConeSolver |

---

## Notes

- The `ActiveSetResult` struct may need a minor update if field names differ — check at integration time
- Warm-start lambda0 comes from `ContactCache` — the existing warm-start flow is unchanged
- ECOS path in ConstraintSolver is NOT removed in this ticket (separate follow-up)
