# Iteration Log — 0071f_solver_workspace_reuse

> **Purpose**: Track every build-test cycle during implementation or investigation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/designs/0071f_solver_workspace_reuse/iteration-log.md`
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0071f_solver_workspace_reuse
**Branch**: 0071f-solver-workspace-reuse
**Baseline**: 718/718 tests passing

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 1 — 2026-02-21 00:00
**Commit**: (pending — pre-commit state ae8051c)
**Hypothesis**: Promote per-solve local variables in all three solvers (PGS, ConstraintSolver friction path, PositionCorrector) to member workspaces using clear/resize-only semantics, eliminating O(islands) heap allocations per frame.
**Changes**:
- `msd/msd-sim/src/Physics/Constraints/ProjectedGaussSeidel.hpp`: Added 5 member workspace variables: `rows_`, `muPerContact_`, `diag_`, `b_`, `lambda_`
- `msd/msd-sim/src/Physics/Constraints/ProjectedGaussSeidel.cpp`: Updated `solve()` to use member workspaces: `rows_.clear()` + `reserve()`, `diag_.resize()`, `b_.resize()`, `lambda_.resize()` + `setZero()`, `vRes_.resize()` + `setZero()`
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`: Added 2 member workspace variables: `flat_` (FlattenedConstraints), `flatEffectiveMass_` (Eigen::MatrixXd); added 2 new non-static methods `populateFlatConstraints_()` and `assembleFlatEffectiveMassInPlace_()`
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`: Implemented `populateFlatConstraints_()` and `assembleFlatEffectiveMassInPlace_()`; updated friction path in `solve()` to use `flat_` and `flatEffectiveMass_` throughout
- `msd/msd-sim/src/Physics/Constraints/PositionCorrector.hpp`: Added 2 member workspace variables: `bodyMInv_` and `activeIndices_`
- `msd/msd-sim/src/Physics/Constraints/PositionCorrector.cpp`: Updated `correctPositions()` to use `bodyMInv_` (resize) and `activeIndices_` (clear + reserve) instead of local variables
**Build Result**: PASS
**Test Result**: 718/718 — no regressions
**Impact vs Previous**: +0 passes, 0 regressions, net 0 (correctness unchanged)
**Assessment**: All workspace promotions compile cleanly and pass all tests. No stale-data bugs — each member is fully re-initialized at the start of each `solve()` / `correctPositions()` call (clear/resize/setZero). Ready to commit.
