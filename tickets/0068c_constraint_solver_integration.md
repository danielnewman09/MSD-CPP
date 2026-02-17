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

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-02-16 22:36 (workflow orchestrator → cpp-implementer agent)
- **Completed**: 2026-02-16 22:46 (implementation code complete, 1 regression to resolve)
- **Branch**: 0068-nlopt-friction-cone-solver
- **PR**: #71 (draft, in progress)
- **Artifacts**:
  - `docs/designs/0068_nlopt_friction_cone_solver/implementation-notes-0068c.md`
  - `docs/designs/0068_nlopt_friction_cone_solver/iteration-log.md` (updated)
- **Changes**:
  - Modified: ConstraintSolver.{hpp,cpp}, 2× CMakeLists.txt, iteration-log.md
  - Deleted: FrictionConeSolver.{hpp,cpp}, ConeProjection.{hpp,cpp}, 3× test files
- **Build**: ✅ PASS (3 warnings — unused variables in Replay test, non-blocking)
- **Tests**: ⚠️ 687/693 passing (6 failures)
  - Baseline: 820/827 (7 failures)
  - Impact: -133 tests (removed unit tests for old solver), -1 failure (net)
  - NEW regression: FrictionConeSolverTest.SlidingCubeOnFloor_FrictionSaturatesAtConeLimit (friction deceleration 6.065 vs expected 4.905 m/s², +23.7% oversaturation)
- **Notes**: All requirements (R1-R4) implemented per design. Integration successful but NLopt produces different friction forces than old solver in sliding contact test. Investigation required to determine if this is a correctness issue or acceptable solver variation. The oversaturation suggests NLopt may be converging to a suboptimal point or the cone constraint formulation needs adjustment. Recommend addressing regression before advancing to Quality Gate phase.
