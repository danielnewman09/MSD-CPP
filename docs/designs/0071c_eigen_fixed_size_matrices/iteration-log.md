# Iteration Log — 0071c_eigen_fixed_size_matrices

> **Purpose**: Track every build-test cycle during implementation or investigation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/designs/0071c_eigen_fixed_size_matrices/iteration-log.md`
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0071c_eigen_fixed_size_matrices
**Branch**: 0071c-eigen-fixed-size-matrices
**Baseline**: 718/718 tests passing at start

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 1 — 2026-02-21 00:00
**Commit**: (see below)
**Hypothesis**: Convert the no-friction path's `assembleJacobians()` return type and all callers from `vector<Eigen::MatrixXd>` to `vector<JacobianRow>` (alias for `Matrix<double,1,12>`). This eliminates per-constraint heap allocations and enables compile-time block extractions in `assembleEffectiveMass`, `assembleRHS`, and `extractBodyForces`.
**Changes**:
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`: Added `JacobianRow` type alias (`Matrix<double,1,12>`); updated `assembleJacobians`, `assembleEffectiveMass`, `assembleRHS`, `extractBodyForces` signatures to use `vector<JacobianRow>`; added ticket reference.
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`: Updated `assembleJacobians` to return `vector<JacobianRow>` (copies row 0 from virtual `jacobian()` return at the boundary); updated `assembleEffectiveMass` to use `leftCols<6>()` / `rightCols<6>()`; updated `assembleRHS` to use `jacobians[i].dot(v)` (compile-time scalar product); updated `extractBodyForces` to use `leftCols<6>()` / `rightCols<6>()`; added ticket references.
**Build Result**: PASS
**Test Result**: 718/718 — no regressions, matches baseline
**Impact vs Previous**: +0 passes, 0 regressions, net change: none (correctness unchanged, allocation pattern improved)
**Assessment**: Successful. All conversions in the no-friction path complete. The virtual boundary in `assembleJacobians` converts `MatrixXd` row 0 to fixed-size `Matrix<double,1,12>` — from that point forward the no-friction path is allocation-free for Jacobian rows (matching the existing flat/friction path). `FlattenedConstraints::jacobianRows` was already `vector<Matrix<double,1,12>>` from a prior ticket.

