# Ticket 0071g: Constraint Object Pool Allocation

## Status
- [x] Draft
- [x] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Implementation
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Design Complete — Awaiting Review
**Type**: Performance
**Priority**: Low
**Created**: 2026-02-21
**Generate Tutorial**: No
**Parent Ticket**: [0071_collision_pipeline_profiling](0071_collision_pipeline_profiling.md)
**Related Tickets**: [0071e_trivial_allocation_elimination](0071e_trivial_allocation_elimination.md), [0071f_solver_workspace_reuse](0071f_solver_workspace_reuse.md) (prerequisite workspace fixes)

---

## Summary

Replace per-frame `std::make_unique<ContactConstraint>` and `std::make_unique<FrictionConstraint>` allocations in `CollisionPipeline::createConstraints()` with a reusable object pool. Currently, every frame destroys all constraints and allocates new ones from scratch — with 100 collisions and friction enabled, this is **200+ heap allocations per frame** that produce identical-sized objects.

Additionally, optimize the warm-start cache loop in `solveConstraintsWithWarmStart()` to reduce per-pair temporary vector allocations.

---

## Problem

### Constraint Object Churn

`CollisionPipeline::createConstraints()` (~lines 291-316):
- Calls `std::make_unique<ContactConstraint>(...)` per contact point
- Calls `std::make_unique<FrictionConstraint>(...)` per contact point (when friction enabled)
- Pushes both into `allConstraints_` vector
- At frame end, `allConstraints_.clear()` destroys all `unique_ptr`s, calling `delete` on each

For 100 collision contacts with friction: **200 `new` + 200 `delete` per frame**.

`ContactConstraint` and `FrictionConstraint` are fixed-size objects (~200-400 bytes each). Their size does not vary at runtime. This is an ideal candidate for pool allocation.

### Warm-Start Cache Vector Churn

`solveConstraintsWithWarmStart()` (~lines 504-630):
- Per collision pair: allocates `islandLambda` VectorXd (~line 505)
- Per collision pair: allocates `solvedLambdas` vector<double> (~line 610)
- These scale as O(collision pairs) per frame

---

## Approach Options

### Option A: Fixed-Capacity Vector with Placement

Replace `vector<unique_ptr<Constraint>>` with `vector<variant<ContactConstraint, FrictionConstraint>>`:
- Value semantics, no heap allocation per constraint
- Contiguous memory, cache-friendly iteration
- **Trade-off**: Closed type set (same concern as 0071d variant option). Future joint constraints would need variant expansion.

### Option B: Simple Free-List Pool

Maintain separate pools for `ContactConstraint` and `FrictionConstraint`:
```cpp
class ConstraintPool {
  std::vector<ContactConstraint> contactPool_;
  std::vector<FrictionConstraint> frictionPool_;
  size_t contactCount_{0};
  size_t frictionCount_{0};
public:
  ContactConstraint& allocateContact(...);
  FrictionConstraint& allocateFriction(...);
  void clearAll();  // Reset counts, no deallocation
};
```
- Open type set — new constraint types get their own pool
- Zero allocation after first frame (pool grows to high-water mark)
- **Trade-off**: Changes ownership model from `unique_ptr` to pool references. Constraint pointers/references valid only until `clearAll()`.

### Option C: PMR (Polymorphic Memory Resource) Allocator

Use `std::pmr::monotonic_buffer_resource` as backing for constraint allocation:
- Transparent to existing `unique_ptr` interface (with custom deleter)
- Zero fragmentation (monotonic allocation, bulk deallocation)
- **Trade-off**: C++17 PMR support, slight complexity in allocator plumbing.

### Option D: Warm-Start Cache Optimization Only

Focus solely on reducing per-pair vector allocations in the cache loop:
- Pre-allocate reusable `solvedLambdas` vector outside the loop
- Batch cache queries to avoid per-pair VectorXd creation
- **Trade-off**: Does not address constraint object churn, but lower risk.

---

## Design Considerations

- **Constraint extensibility**: The `Constraint` interface supports future joint types (hinges, ball-socket). Any pool design must accommodate new types without major refactoring. Option B (typed pools) is most extensible.
- **Solver interface**: `ConstraintSolver::solve()` and `ProjectedGaussSeidel::solve()` accept `vector<Constraint*>`. Pool must produce stable pointers for the frame duration.
- **CollisionPipeline ownership**: Currently `allConstraints_` owns via `unique_ptr`. Pool changes ownership semantics — constraints are "borrowed" from the pool rather than exclusively owned.
- **Thread safety**: Single-threaded physics loop. No thread-safety concerns.

---

## Estimated Effort

**Large** — 1-2 days. Requires design review (ownership model change), implementation of pool mechanism, modification of `createConstraints()` and `allConstraints_` interface, and verification that constraint pointer lifetime assumptions hold throughout the solver pipeline.

---

## Success Criteria

- All existing physics tests pass (no regressions)
- Zero per-frame `new`/`delete` calls for constraint objects after first frame
- Profile shows measurable reduction in `_xzm_free` and `_xzm_xzone_malloc_tiny` samples
- Constraint extensibility preserved (new types addable without pool redesign)
- Warm-start cache loop allocations reduced

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
  - Constraint object churn is the single largest source of per-frame allocations (~200 new/delete per frame)
  - Should be implemented after 0071e (trivial) and 0071f (workspace reuse) to establish allocation baseline
  - Design review needed to select pool approach (A/B/C/D)
  - Option B (typed pools) is recommended given extensibility requirements from planned joint constraints

### Design Phase
- **Started**: 2026-02-21
- **Completed**: 2026-02-21
- **Branch**: `0071g-constraint-pool-allocation` (branched from `0071f-solver-workspace-reuse`)
- **PR**: N/A (pending creation after commit)
- **Artifacts**:
  - `docs/designs/0071g_constraint_pool_allocation/design.md`
  - `docs/designs/0071g_constraint_pool_allocation/0071g_constraint_pool_allocation.puml`
- **Notes**:
  - Selected **Option B** (Typed Free-List Pool) — open type set, zero allocations after first frame, preserves `vector<Constraint*>` interface throughout
  - Rejected Option A (variant) — closed type set incompatible with planned joint constraints
  - Rejected Option C (PMR) — over-engineered for homogeneous fixed-size objects; viable future direction if joint constraints require heterogeneous sizes
  - Included Option D work (warm-start vector elimination) as part of Option B implementation
  - Key design decisions:
    - `ConstraintPool` is a new class in `Physics/Constraints/` with typed backing vectors
    - `allConstraints_` changes from `vector<unique_ptr<Constraint>>` to `vector<Constraint*>` (non-owning view)
    - Factory bypass: inline `createFromCollision()` logic directly into `createConstraints()` (single call site)
    - `solvedLambdas_` and `islandConstraintSet_` promoted to member workspaces
    - Pointer stability guaranteed by pre-`resize()` before allocation batch within each frame
