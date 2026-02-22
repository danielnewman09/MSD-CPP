# Ticket 0071e: Trivial Allocation Elimination

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
**Related Tickets**: [0071f_solver_workspace_reuse](0071f_solver_workspace_reuse.md), [0071g_constraint_pool_allocation](0071g_constraint_pool_allocation.md)

---

## Summary

Eliminate low-hanging-fruit heap allocations in the collision pipeline that require minimal code changes and no architectural redesign. These are pre-reserve calls, duplicate elimination, and small vector optimizations identified by profiling StackCollapse/16 (allocation functions totaling ~2.6% of CPU).

---

## Problem

Profiling StackCollapse/16 (6,188 samples) shows allocator functions as the top individual hotspots:
- `_xzm_free`: 52 samples (0.8%)
- `_xzm_xzone_malloc_tiny`: 42 samples (0.7%)
- `Eigen::aligned_free`: 10 samples (0.2%)
- `_free`: 11 samples (0.2%)
- `malloc_type_malloc`: 11 samples (0.2%)

Several of these allocations are trivially avoidable with pre-reserve calls and duplicate elimination.

---

## Changes

### 1. Eliminate duplicate `extractContactPoints()` call in warm-start cache loop

In `CollisionPipeline::solveConstraintsWithWarmStart()`, `extractContactPoints()` is called twice per collision pair per frame — once for the warm-start query (~line 524) and again for the cache update (~line 624). The second call produces identical data.

**Fix**: Cache the result from the first call and reuse it.

**Savings**: 1 vector allocation per collision pair per frame.

### 2. Pre-reserve `allConstraints_` in `createConstraints()`

`CollisionPipeline::createConstraints()` pushes `unique_ptr<ContactConstraint>` and `unique_ptr<FrictionConstraint>` into `allConstraints_` without reserving capacity. Each reallocation copies all existing `unique_ptr`s.

**Fix**: Add `allConstraints_.reserve(collisions_.size() * 2)` before the loop (worst case: 1 contact + 1 friction per collision pair).

**Savings**: Eliminates vector reallocation growth during constraint insertion.

### 3. Pre-reserve `FlattenedConstraints` vectors in `flattenConstraints()`

`ConstraintSolver::flattenConstraints()` builds 5 vectors (jacobianRows, bodyAIndices, bodyBIndices, rowTypes, restitutions) via `push_back()` without reserving. The total row count is computable before the loop.

**Fix**: Compute total rows from constraint dimensions, then `.reserve()` all 5 vectors before the loop.

**Savings**: Eliminates reallocation during flattening. 5 fewer reallocations per friction solve.

### 4. Pre-reserve `collisions_` in `detectCollisions()`

`collisions_` is cleared each frame and repopulated via `push_back()`. No capacity estimate is set.

**Fix**: Add `collisions_.reserve(previousSize)` or a heuristic estimate after `.clear()` (clear preserves capacity, so this may already be a non-issue after frame 1 — verify).

**Savings**: Eliminates reallocation on first frame or when collision count exceeds previous peak.

---

## Estimated Effort

**Trivial** — Each change is 1-5 lines. No interface changes, no new classes, no behavioral changes. Total: ~30 minutes implementation + testing.

---

## Success Criteria

- All existing physics tests pass (no regressions)
- Profile shows reduced `_xzm_xzone_malloc_tiny` samples in StackCollapse/16
- No new allocations introduced

---

## Profiling Artifacts

- StackCollapse/16 profile: `profile_results/profile_stackcollapse16.trace`
- ClusterDrop/32 profile: `profile_results/profile_clusterdrop32.trace`
- Benchmark: `msd/msd-sim/bench/MultiBodyCollisionBench.cpp`

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-21
- **Notes**:
  - Identified from allocation churn investigation of StackCollapse/16 profiling data
  - All changes are additive (reserve calls) or subtractive (remove duplicate call)
  - No design phase needed — changes are mechanical

### Implementation Phase
- **Started**: 2026-02-21
- **Completed**: 2026-02-21
- **Branch**: `0071e-trivial-allocation-elimination`
- **PR**: #91 (draft)
- **Artifacts**:
  - `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp` — added `pairContactPoints_` member
  - `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp` — all four allocation fixes
  - `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` — pre-reserve in `flattenConstraints()`
- **Notes**:
  - Change 1: Pre-computed `extractContactPoints()` once per pair into `pairContactPoints_`
    member (indexed by `collisions_` index). Both warm-start query and cache-update loops
    now reference pre-computed data. `pairContactPoints_` cleared in `clearEphemeralState()`.
  - Change 2: Added `allConstraints_.reserve(collisions_.size() * 2 * (anyFriction ? 2 : 1))`
    and `pairRanges_.reserve(collisions_.size())` in `createConstraints()`.
  - Change 3: Added pre-pass in `flattenConstraints()` computing `totalRows`, then reserved
    all 5 `FlattenedConstraints` vectors before the constraint iteration loop.
  - Change 4: Added first-frame `collisions_.reserve()` estimate in `execute()`;
    after frame 1 `clear()` preserves capacity (no-op).
  - All 718 existing tests pass with no regressions.
