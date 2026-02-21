# Ticket 0071f: Solver Workspace Reuse

## Status
- [x] Draft
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Implementation
- [x] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Implementation Complete
**Type**: Performance
**Priority**: Medium
**Created**: 2026-02-21
**Generate Tutorial**: No
**Parent Ticket**: [0071_collision_pipeline_profiling](0071_collision_pipeline_profiling.md)
**Related Tickets**: [0071e_trivial_allocation_elimination](0071e_trivial_allocation_elimination.md) (prerequisite trivial fixes), [0071g_constraint_pool_allocation](0071g_constraint_pool_allocation.md) (constraint object pooling)

---

## Summary

Convert per-solve local allocations in `ProjectedGaussSeidel`, `ConstraintSolver`, and `PositionCorrector` to reusable member workspaces. These solvers currently allocate fresh `Eigen::VectorXd`, `std::vector<FlatRow>`, and other workspace containers on every call. By promoting them to member variables with resize-only semantics, we eliminate O(islands) allocations per frame.

---

## Problem

### ProjectedGaussSeidel::solve() — 5+ allocations per island

Every call to `solve()` creates local variables:
- `std::vector<FlatRow> rows` (~line 56) — reserved at 3x constraint count
- `std::vector<double> muPerContact` (~line 60) — friction coefficients
- `Eigen::VectorXd diag` (~line 111) — diagonal effective mass
- `Eigen::VectorXd b` (~line 137) — RHS vector
- `Eigen::VectorXd lambda` (~line 172) — multipliers (zero or warm-start)

Member `vRes_` is already a workspace but is resized per solve (~line 189).

With island decomposition (ticket 0073), PGS is called per-island. With 50 small islands, this produces **250+ allocations per frame** just from PGS workspace creation.

### ConstraintSolver::flattenConstraints() — returned struct with 5 vectors

`flattenConstraints()` returns a `FlattenedConstraints` struct by value, allocating 5 vectors each call. This is called once per friction-path solve.

### ConstraintSolver::assembleFlatEffectiveMass() — per-solve MatrixXd

The friction-path effective mass matrix `a` (~line 756) is a local `MatrixXd::Zero(numRows, numRows)`. For 100 contacts with friction (300 rows), this is a 720KB allocation.

### PositionCorrector — mixed workspace pattern

Some workspaces are already member variables (ticket 0053f): `jacobians_`, `bPos_`, `effectiveMass_`, `lambdaPos_`. But `bodyMInv` (~line 106) and `activeIndices` (~line 172) are still local allocations per call.

---

## Changes

### 1. Promote PGS local variables to member workspaces

Move `rows`, `muPerContact`, `diag`, `b`, `lambda` to `ProjectedGaussSeidel` member variables. On each `solve()` call, clear/resize them instead of reconstructing.

**Pattern**:
```cpp
// Before (local allocation per call):
std::vector<FlatRow> rows;
rows.reserve(constraints.size() * 3);

// After (member workspace, resize-only):
rows_.clear();
rows_.reserve(constraints.size() * 3);
```

`Eigen::VectorXd` members use `.conservativeResize()` or just `.resize()` (which does not reallocate if size is unchanged or smaller with Eigen's default behavior — verify).

**Savings**: 5+ allocations per island eliminated. With 50 islands: ~250 allocations saved.

### 2. Store FlattenedConstraints as ConstraintSolver member

Instead of returning `FlattenedConstraints` by value from `flattenConstraints()`, store it as a member. Clear and reuse across frames.

**Savings**: 5 vector allocations per friction solve.

### 3. Store friction-path effective mass as ConstraintSolver member

Promote `a` (MatrixXd) in `assembleFlatEffectiveMass()` to a member `flatEffectiveMass_`. Resize per solve (no realloc if size unchanged).

**Savings**: 1 large MatrixXd allocation per friction solve (720KB for 300 rows).

### 4. Promote remaining PositionCorrector locals to members

Move `bodyMInv` vector and `activeIndices` vector to member variables following the existing ticket 0053f pattern.

**Savings**: 2 allocations per position correction call.

---

## Design Considerations

- **Thread safety**: All solvers are single-threaded (called from WorldModel::updatePhysics). Member workspaces are safe.
- **Memory lifetime**: Workspaces persist for the solver's lifetime. For PGS (owned by CollisionPipeline), this is the WorldModel lifetime — acceptable since peak memory usage is bounded by peak collision count.
- **Eigen resize semantics**: `Eigen::VectorXd::resize(n)` does NOT reallocate if `n <= current capacity` when using `conservativeResize`. Verify behavior for `resize()` — Eigen may reallocate even when shrinking. Use `conservativeResize()` or guard with size check.

---

## Estimated Effort

**Medium** — 2-4 hours. Requires moving local variables to class members, adjusting initialization, and verifying no stale-data bugs from workspace reuse. Each solver touched independently.

---

## Success Criteria

- All existing physics tests pass (no regressions)
- No new member variables leak stale data across frames (verify with StackCollapse scenario)
- Profile shows reduced allocator samples in StackCollapse/16
- PGS per-island allocation count drops from 5+ to ~0 (verify with instrumentation or profiling)

---

## Profiling Artifacts

- StackCollapse/16 profile: `profile_results/profile_stackcollapse16.trace`
- Benchmark: `msd/msd-sim/bench/MultiBodyCollisionBench.cpp`

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-21
- **Notes**:
  - Identified from allocation churn investigation of StackCollapse/16 profiling data
  - Follows pattern established by ticket 0053f (PositionCorrector workspace reuse)
  - PGS workspace is the highest-value target due to O(islands) scaling
  - Should be implemented after 0071e trivial fixes

### Implementation Phase
- **Started**: 2026-02-21
- **Completed**: 2026-02-21
- **Branch**: `0071f-solver-workspace-reuse` (branched from `0071e-trivial-allocation-elimination`)
- **PR**: N/A (pending push)
- **Artifacts**:
  - `docs/designs/0071f_solver_workspace_reuse/iteration-log.md`
  - `docs/designs/0071f_solver_workspace_reuse/implementation-notes.md`
  - `msd/msd-sim/src/Physics/Constraints/ProjectedGaussSeidel.hpp` — 5 new member workspaces
  - `msd/msd-sim/src/Physics/Constraints/ProjectedGaussSeidel.cpp` — solve() uses members
  - `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` — 2 new members + 2 new private methods
  - `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` — in-place populate + friction path uses members
  - `msd/msd-sim/src/Physics/Constraints/PositionCorrector.hpp` — 2 new member workspaces
  - `msd/msd-sim/src/Physics/Constraints/PositionCorrector.cpp` — correctPositions() uses members
- **Notes**:
  - All 4 changes from ticket implemented in single iteration (Iteration 1, commit 80e7902)
  - 718/718 tests pass — no regressions
  - Static `flattenConstraints()` and `assembleFlatEffectiveMass()` methods preserved unchanged for API compatibility
  - New private `populateFlatConstraints_()` and `assembleFlatEffectiveMassInPlace_()` fill member workspaces in-place
