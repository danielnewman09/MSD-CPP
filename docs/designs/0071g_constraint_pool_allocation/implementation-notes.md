# Implementation Notes — 0071g Constraint Object Pool Allocation

**Ticket**: 0071g_constraint_pool_allocation
**Branch**: 0071g-constraint-pool-allocation
**Date**: 2026-02-22
**Status**: Implementation Complete

---

## Summary

Replaced 200+ per-frame heap allocations in `CollisionPipeline::createConstraints()` with a typed
free-list pool (`ConstraintPool`) that achieves zero heap allocation in steady state. The pool uses
`reserve() + emplace_back() + clear()` semantics: after frame 1, the backing vectors have grown to
their high-water mark and `reset()` calls `clear()` which destructs constraint objects but preserves
capacity, so subsequent `emplace_back()` calls reuse the allocated memory.

Two local workspace variables (`solvedLambdas` and `islandConstraintSet`) were promoted to member
workspaces on `CollisionPipeline` to eliminate their per-frame heap allocation overhead.
`ContactConstraintFactory` was inlined into `createConstraints()` since it had a single call site,
reducing indirection.

---

## Files Created

| File | Purpose | LOC |
|------|---------|-----|
| `msd/msd-sim/src/Physics/Constraints/ConstraintPool.hpp` | Typed pool class declaration with full API documentation | 147 |
| `msd/msd-sim/src/Physics/Constraints/ConstraintPool.cpp` | Pool implementation (allocate, reset, reserve, count) | 84 |
| `msd/msd-sim/test/Physics/Constraints/ConstraintPoolTest.cpp` | Unit tests for ConstraintPool (15 tests) | 270 |
| `docs/designs/0071g_constraint_pool_allocation/iteration-log.md` | Implementation iteration tracking log | — |

---

## Files Modified

| File | Description of Changes |
|------|------------------------|
| `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp` | Added `ConstraintPool.hpp` include; changed `allConstraints_` from `vector<unique_ptr<Constraint>>` to `vector<Constraint*>`; added `constraintPool_` member (before `allConstraints_` for correct destruction order); promoted `solvedLambdas_` and `islandConstraintSet_` from local variables to member workspaces |
| `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp` | `createConstraints()`: pre-reserve + pool allocation with inlined factory logic; `clearEphemeralState()`: added `constraintPool_.reset()`; `buildSolverView()`, `buildContactView()`, `solveConstraintsWithWarmStart()`, `propagateSolvedLambdas()`, `findPairIndexForConstraint()`: removed all `.get()` calls; member workspace reuse for `solvedLambdas_` and `islandConstraintSet_` |
| `msd/msd-sim/src/Physics/Constraints/CMakeLists.txt` | Added `ConstraintPool.cpp` to target sources |
| `msd/msd-sim/test/Physics/Constraints/CMakeLists.txt` | Added `ConstraintPoolTest.cpp` to test sources |

---

## Design Adherence Matrix

| Design Requirement | Status | Notes |
|-------------------|--------|-------|
| Option B: Typed free-list pool with two backing vectors | Implemented | `contactStorage_` and `frictionStorage_` in `ConstraintPool` |
| `reserve() + emplace_back() + clear()` semantics (Design Review Note N1) | Implemented | Required because `ContactConstraint`/`FrictionConstraint` have no default constructors |
| `reset()` clears counts but preserves capacity | Implemented | `clear()` on both vectors |
| `allConstraints_` changed to non-owning `vector<Constraint*>` | Implemented | All `.get()` calls removed throughout |
| `constraintPool_` declared before `allConstraints_` | Implemented | Ensures pool outlives pointer view during destruction |
| Pre-reserve before allocation batch (pointer stability) | Implemented | `reserveContacts(collisions_.size() * 4)` in `createConstraints()` |
| Promote `solvedLambdas` and `islandConstraintSet` to member workspaces | Implemented | `solvedLambdas_` and `islandConstraintSet_` on `CollisionPipeline` |
| Inline `ContactConstraintFactory::createFromCollision()` | Implemented | Loop body inlined into `createConstraints()`; free functions preserved |
| Rule of Zero for `ConstraintPool` | Implemented | Compiler-generated copy/move (backing vectors own memory) |
| Ticket and design file comments in all new files | Implemented | All new files have `// Ticket:` and `// Design:` comments |

---

## Prototype Application Notes

The design review identified that `ContactConstraint` and `FrictionConstraint` have no default
constructors, making the design's proposed `resize()` approach non-compilable. This was caught
before writing any code and the implementation used `reserve() + emplace_back() + clear()` from
the start.

The design's pointer stability analysis (Phase 2 of frame lifecycle: allocation batch complete
before any pointer is stored) was critical. The implementation enforces this by pre-reserving
capacity to `collisions_.size() * 4` (worst case: 4 contact points per collision pair) before
the allocation loop. This was the root cause of the segfault in Iteration 1 (see iteration log).

---

## Deviations from Design

None. The implementation follows the design exactly with the `reserve() + emplace_back() + clear()`
correction specified in the design review.

---

## Test Coverage Summary

**New tests**: 15 in `ConstraintPoolTest`

| Test | Behavior Verified |
|------|-------------------|
| `InitialState_CountsAreZero` | Pool starts empty |
| `AllocateContact_ReturnsNonNullPointer` | Allocation returns valid pointer |
| `AllocateContact_IncreasesContactCount` | Count increments per allocation |
| `AllocateFriction_ReturnNonNullPointer` | Friction allocation returns valid pointer |
| `AllocateFriction_IncreasesFrictionCount` | Friction count increments per allocation |
| `AllocateContact_ConstraintHasCorrectNormal` | Constraint data integrity (normal vector) |
| `AllocateContact_ConstraintHasPenetrationDepth` | Constraint data integrity (penetration depth) |
| `AllocateContact_ConstraintHasBodyIndices` | Constraint data integrity (body indices) |
| `Reset_CountsBecomeZero` | Reset clears all counts |
| `Reset_ThenReallocate_ProducesValidConstraints` | Steady-state reuse correctness |
| `PointerStability_WithReserve_PointersRemainValid` | Reserve prevents reallocation mid-batch |
| `MultipleResetCycles_NoHeapGrowthAfterHighWaterMark` | Capacity preserved across multiple frames |
| `AllocateManyContacts_CountsCorrect` | Bulk allocation (200 contacts + 200 friction) |
| `ReserveContacts_DoesNotAllocateConstraints` | Reserve changes capacity, not count |
| `ReserveFriction_DoesNotAllocateConstraints` | Reserve changes capacity, not count |

**Existing tests**: 717/717 pass (excluding pre-existing hanging test
`EngineIntegrationTest.Engine_OverlappingObjects_VelocitiesChange` which was confirmed to hang
on the baseline binary before this implementation).

**Total after implementation**: 732/732 pass.

---

## Known Limitations

- **No thread safety**: `ConstraintPool` is not thread-safe. This matches the single-threaded
  physics loop assumption documented in the class header.
- **Pre-existing hanging test**: `EngineIntegrationTest.Engine_OverlappingObjects_VelocitiesChange`
  hangs independent of this ticket's changes (confirmed via `git stash` + rebuild).

---

## Future Considerations

- The `ContactConstraintFactory` free functions (`computeRelativeNormalVelocity`,
  `combineRestitution`, `kRestVelocityThreshold`) remain in their original file even though
  `createFromCollision()` is now inlined. These could be moved to an anonymous namespace in
  `CollisionPipeline.cpp` in a future cleanup ticket, or the factory header could be deleted
  if no other consumers exist.
- Extensibility: New constraint types (joints, limits) can add a new typed backing vector and
  `allocate*()` method to `ConstraintPool` without modifying existing code paths.
