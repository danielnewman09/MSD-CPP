# Implementation Notes — 0071f: Solver Workspace Reuse

## Summary

Promoted per-solve local variables in three physics solvers to reusable member
workspaces. Each solver now allocates heap memory at most once (on first call or when
the problem size grows beyond previous peak), then reuses those buffers in subsequent
calls using clear/resize-only semantics.

---

## Files Modified

### `msd/msd-sim/src/Physics/Constraints/ProjectedGaussSeidel.hpp`
- Added 5 member workspace variables (see below)
- Added `// Ticket: 0071f_solver_workspace_reuse` header comment

### `msd/msd-sim/src/Physics/Constraints/ProjectedGaussSeidel.cpp`
- Updated `solve()` to use member workspaces:
  - `rows_.clear()` + `rows_.reserve()` instead of local `std::vector<FlatRow> rows`
  - `muPerContact_.clear()` instead of local `std::vector<double> muPerContact`
  - `diag_.resize(numRows)` instead of local `Eigen::VectorXd diag(numRows)`
  - `b_.resize(numRows)` instead of local `Eigen::VectorXd b(numRows)`
  - `lambda_.resize(numRows)` + `lambda_.setZero()` instead of `Eigen::VectorXd::Zero(numRows)`
  - `vRes_.resize(...)` + `vRes_.setZero()` instead of `Eigen::VectorXd::Zero(...)`

### `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`
- Added 2 member workspace variables: `flat_` and `flatEffectiveMass_`
- Added 2 new private methods: `populateFlatConstraints_()` and
  `assembleFlatEffectiveMassInPlace_()`
- Added `// Ticket: 0071f_solver_workspace_reuse` header comment

### `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`
- Added `// Ticket: 0071f_solver_workspace_reuse` header comment
- Implemented `populateFlatConstraints_()`: populates `flat_` in-place using
  `clear()` + `reserve()` + `push_back()`, preserving vector capacities across frames
- Implemented `assembleFlatEffectiveMassInPlace_()`: populates `flatEffectiveMass_`
  in-place using `resize()` + `setZero()`, preserving matrix buffer across frames
- Updated friction path in `solve()` to call these new in-place methods and use
  `flat_`/`flatEffectiveMass_` throughout (replacing local `flat` and `a`)

### `msd/msd-sim/src/Physics/Constraints/PositionCorrector.hpp`
- Added 2 member workspace variables: `bodyMInv_` and `activeIndices_`
- Added `// Ticket: 0071f_solver_workspace_reuse` header comment

### `msd/msd-sim/src/Physics/Constraints/PositionCorrector.cpp`
- Updated `correctPositions()` to use `bodyMInv_` (`resize()`) and `activeIndices_`
  (`clear()` + `reserve()` + `push_back()`) instead of stack-local variables

---

## New Member Variables

### ProjectedGaussSeidel (5 new members)

| Member | Type | Purpose |
|--------|------|---------|
| `rows_` | `std::vector<FlatRow>` | Flattened per-row constraint data |
| `muPerContact_` | `std::vector<double>` | Friction coefficients per contact |
| `diag_` | `Eigen::VectorXd` | Diagonal effective-mass elements A_ii |
| `b_` | `Eigen::VectorXd` | RHS vector |
| `lambda_` | `Eigen::VectorXd` | Constraint multiplier vector |

### ConstraintSolver (2 new members + 2 existing were named `flat_`, `flatEffectiveMass_`)

| Member | Type | Purpose |
|--------|------|---------|
| `flat_` | `FlattenedConstraints` | Reused flat constraint representation |
| `flatEffectiveMass_` | `Eigen::MatrixXd` | Reused (3C × 3C) effective mass matrix |

### PositionCorrector (2 new members)

| Member | Type | Purpose |
|--------|------|---------|
| `bodyMInv_` | `std::vector<Eigen::Matrix<double,6,6>>` | Per-body 6×6 inverse mass matrix |
| `activeIndices_` | `std::vector<int>` | ASM active constraint set |

---

## Design Adherence

| Requirement | Status |
|-------------|--------|
| PGS 5+ local allocations eliminated | Done — all promoted to members |
| FlattenedConstraints stored as member | Done — `flat_` member; `populateFlatConstraints_()` fills in-place |
| flatEffectiveMass stored as member | Done — `flatEffectiveMass_` member; `assembleFlatEffectiveMassInPlace_()` resizes in-place |
| PositionCorrector bodyMInv promoted | Done — `bodyMInv_` member |
| PositionCorrector activeIndices promoted | Done — `activeIndices_` member |
| No stale-data bugs | Verified — each member is fully re-initialized at the top of each solve call |
| Thread safety maintained | Verified — all solvers remain single-threaded |
| All existing tests pass | 718/718 |

---

## Allocation Savings Estimate

| Solver | Before | After | Savings at 50 islands |
|--------|--------|-------|----------------------|
| PGS per island | 5-6 allocs | 0 allocs | ~250-300 allocs/frame |
| ConstraintSolver friction path | 5 allocs (FlattenedConstraints vectors) + 1 large MatrixXd | 0 allocs | ~5-6 allocs/frame |
| PositionCorrector per call | 2 allocs | 0 allocs | ~2 allocs/frame |

---

## API Impact

- `flattenConstraints()` static method: **unchanged** — still available for external use
- `assembleFlatEffectiveMass()` static method: **unchanged** — still available
- New `populateFlatConstraints_()` and `assembleFlatEffectiveMassInPlace_()`: private,
  not part of public API
- `ConstraintSolver`, `ProjectedGaussSeidel`, `PositionCorrector` are no longer
  trivially copyable (member workspaces contain heap-allocated vectors/matrices),
  but all three already had `= default` Rule-of-Five implementations which handle
  this correctly via value semantics (copy/move of the members).

---

## Deviations from Ticket

None. All four changes described in the ticket were implemented as specified.

---

## Known Limitations / Future Considerations

- The `bodyMInv_` workspace in `PositionCorrector` is still `O(numBodies)` per call.
  This is intentional — the ticket only targets the local allocation, not the
  computational cost of filling the matrix.
- Ticket 0071g (constraint pool allocation) handles the next layer of allocation
  churn (constraint object creation).
