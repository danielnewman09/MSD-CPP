# Iteration Log — 0071g_constraint_pool_allocation

> **Purpose**: Track every build-test cycle during implementation or investigation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/designs/0071g_constraint_pool_allocation/iteration-log.md`
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0071g_constraint_pool_allocation
**Branch**: 0071g-constraint-pool-allocation
**Baseline**: 717/717 tests passing at start (excluding pre-existing hang: EngineIntegrationTest.Engine_OverlappingObjects_VelocitiesChange)

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 1 — 2026-02-22 03:00
**Commit**: (pre-commit — initial implementation)
**Hypothesis**: Replace per-frame `make_unique<ContactConstraint>` and `make_unique<FrictionConstraint>` calls with a typed pool using `reserve() + emplace_back() + clear()` to eliminate heap allocation in steady state.
**Changes**:
- `msd/msd-sim/src/Physics/Constraints/ConstraintPool.hpp`: Created new `ConstraintPool` class with `contactStorage_` and `frictionStorage_` backing vectors, `allocate*()`, `reset()`, `reserve*()`, and `count*()` methods.
- `msd/msd-sim/src/Physics/Constraints/ConstraintPool.cpp`: Implemented `ConstraintPool` methods using `emplace_back()` for allocation and `clear()` for reset (preserves capacity).
- `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp`: Added `constraintPool_` member, changed `allConstraints_` from `vector<unique_ptr<Constraint>>` to `vector<Constraint*>`, promoted `solvedLambdas_` and `islandConstraintSet_` from local variables to member workspaces.
- `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp`: Rewrote `createConstraints()` with pool allocation and inlined factory logic; fixed all `.get()` call sites in `buildSolverView()`, `buildContactView()`, `solveConstraintsWithWarmStart()`, `propagateSolvedLambdas()`, `findPairIndexForConstraint()`; added `constraintPool_.reset()` in `clearEphemeralState()`.
- `msd/msd-sim/src/Physics/Constraints/CMakeLists.txt`: Added `ConstraintPool.cpp`.
**Build Result**: FAIL (segfault on `ObjectRestsOnfloor` test — exit code 139)
**Test Result**: Crash before tests complete
**Impact vs Previous**: Regression — segfault
**Assessment**: Pointer stability violation. `emplace_back()` into `contactStorage_` reallocation invalidates all previously returned `Constraint*` pointers stored in `allConstraints_`. Fix: pre-`reserve()` before the allocation batch.

### Iteration 2 — 2026-02-22 03:05
**Commit**: (pre-commit — pointer stability fix)
**Hypothesis**: Pre-reserving `constraintPool_.reserveContacts(collisions_.size() * 4)` and `reserveFriction(...)` before the allocation loop guarantees no reallocation occurs during the batch, preserving pointer stability.
**Changes**:
- `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp`: Added `reserveContacts()` and `reserveFriction()` calls at the top of `createConstraints()` before any `allocate*()` calls. Used `collisions_.size() * 4` (max 4 contact points per collision pair per `CollisionResult` design).
**Build Result**: PASS
**Test Result**: 717/717 — all tests pass (excluding pre-existing hang)
**Impact vs Previous**: +717 passes (crash resolved), net +717
**Assessment**: Pointer stability fix is correct. Capacity is pre-reserved to worst case before any pointers are stored.

### Iteration 3 — 2026-02-22 03:10
**Commit**: (current — unit tests added)
**Hypothesis**: Add unit tests for `ConstraintPool` covering allocation, counting, reset, pointer stability with reserve, multiple reset cycles, and reserve-without-allocation behavior.
**Changes**:
- `msd/msd-sim/test/Physics/Constraints/ConstraintPoolTest.cpp`: Created 15 unit tests.
- `msd/msd-sim/test/Physics/Constraints/CMakeLists.txt`: Added `ConstraintPoolTest.cpp` to test sources.
**Build Result**: PASS
**Test Result**: 732/732 — 15 new ConstraintPoolTest tests all pass, zero regressions
**Impact vs Previous**: +15 passes (new tests), net +15
**Assessment**: Implementation complete. All tests pass. Pool correctly resets counts while preserving capacity.

