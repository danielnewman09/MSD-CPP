# Design: Constraint Object Pool Allocation

**Ticket**: [0071g_constraint_pool_allocation](../../../tickets/0071g_constraint_pool_allocation.md)
**Status**: Design Complete
**Date**: 2026-02-21

---

## Problem Statement

`CollisionPipeline::createConstraints()` allocates 200+ heap objects per frame via
`std::make_unique<ContactConstraint>(...)` and `std::make_unique<FrictionConstraint>(...)`.
At frame end, `clearEphemeralState()` calls `allConstraints_.clear()`, which destroys all
`unique_ptr`s and triggers 200+ `delete` calls. For 100 contacts with friction this is
**200 `new` + 200 `delete` per frame** for fixed-size objects whose size never varies.

Additionally, `solveConstraintsWithWarmStart()` allocates a `std::vector<double> solvedLambdas`
per collision pair in the cache-update loop (O(collision pairs) allocations per frame).

**Context**: Tickets 0071e and 0071f have already eliminated trivial allocations and promoted
solver workspace locals to members. Constraint object churn is the remaining dominant allocation
source, visible as `_xzm_free` and `_xzm_xzone_malloc_tiny` samples in the Instruments profiler.

---

## Option Analysis

The ticket lists four approaches (A–D). This section evaluates each against the project's
specific requirements.

### Constraint Size Survey

| Type | Approx. Size | Count per Frame (100 contacts, friction) |
|------|-------------|------------------------------------------|
| `ContactConstraint` | ~200 bytes | 100 |
| `FrictionConstraint` | ~230 bytes | 100 |

Both are polymorphic (virtual destructor) and fixed-size. Both implement `Rule of Five` with
default copy/move.

### Option A: `vector<variant<ContactConstraint, FrictionConstraint>>`

Replace `vector<unique_ptr<Constraint>>` with a variant vector for value semantics and contiguous
layout.

**Verdict: Rejected.** The constraint hierarchy is open: planned joint types (hinge, ball-socket)
would require expanding the variant. Every visitor site (the solver, position corrector, data
recorder, island builder, sliding-state propagation) would need `std::visit`. The variant size is
`max(sizeof(CC), sizeof(FC), ...) + discriminant` — bloated when joint constraints are added.
`dynamic_cast` is already used at 6 sites in `CollisionPipeline` and `ConstraintSolver`; replacing
those with `std::visit` is more invasive than pool allocation. Option A violates the Open/Closed
Principle for this codebase.

### Option B: Typed Free-List Pool (SELECTED)

Maintain a `ConstraintPool` member inside `CollisionPipeline` with separate contiguous storage
for each concrete type:

```cpp
class ConstraintPool {
  std::vector<ContactConstraint>  contactStorage_;
  std::vector<FrictionConstraint> frictionStorage_;
  size_t contactCount_{0};
  size_t frictionCount_{0};
public:
  ContactConstraint&  allocateContact(/* args */);
  FrictionConstraint& allocateFriction(/* args */);
  void reset();  // Set counts to 0, no deallocation
};
```

The caller stores `Constraint*` pointers (stable for the frame lifetime) in `allConstraints_`.

**Verdict: Selected.** See "Selected Design" below.

### Option C: PMR Monotonic Buffer

Use `std::pmr::monotonic_buffer_resource` with a `std::pmr::polymorphic_allocator`. Objects are
allocated with standard `new` syntax via a custom allocator backed by a per-frame buffer. Dealloc
is a single bulk reset.

**Verdict: Viable but over-engineered for this case.** PMR is designed for heterogeneous
allocations (many different sizes). `ContactConstraint` and `FrictionConstraint` are homogeneous
fixed-size objects — typed pools are simpler and avoid PMR's plumbing (custom deleters, allocator
plumbing through `make_unique` equivalent). The typed pool approach achieves zero fragmentation
through contiguous storage without introducing new allocator types. PMR would also prevent future
move of `allConstraints_` to a pool-aware design. Option C remains a viable future direction if
joint constraints require heterogeneous sizes that make per-type pools impractical.

### Option D: Warm-Start Cache Optimization Only

Address only the `solvedLambdas` allocation in the cache-update loop; defer constraint object
pooling.

**Verdict: Too conservative.** The `solvedLambdas` allocation is a subset of total churn.
With Option D alone, the dominant 200 `new`/`delete` per frame remain. The ticket explicitly
targets both sources. Option D work is included as part of the chosen Option B implementation.

---

## Selected Design: Option B — Typed Free-List Pool

### Core Principle

After the first frame, `ConstraintPool` has grown its storage vectors to the high-water mark.
Subsequent `reset()` calls set counters to zero without releasing memory. Pointers obtained
from `allocateContact()` and `allocateFriction()` are stable until the next `reset()`, which
happens at the start of `execute()` via `clearEphemeralState()` (same point where
`allConstraints_.clear()` currently runs).

### Ownership Model Change

| Aspect | Before (0071f) | After (0071g) |
|--------|---------------|---------------|
| Constraint owner | `vector<unique_ptr<Constraint>>` | `ConstraintPool` |
| Pointer stability | Guaranteed by `unique_ptr` | Guaranteed by `vector` stability (no realloc after first frame) |
| Destruction | Individual `delete` per frame | Single `reset()` (no dealloc after first frame) |
| `allConstraints_` type | `vector<unique_ptr<Constraint>>` | `vector<Constraint*>` (non-owning view) |
| `buildSolverView()` | Extracts `.get()` from `unique_ptr`s | Returns `allConstraints_` directly or copy |
| `buildContactView()` | Filters by `dynamic_cast` | Unchanged — still filters `allConstraints_` by `dynamic_cast` |

The `vector<Constraint*>` interface that `ConstraintSolver::solve()` and
`ProjectedGaussSeidel::solve()` accept is **unchanged**. Those functions receive raw pointer
spans today (via `buildSolverView`) and will continue to do so.

### Pointer Stability Guarantee

The pool's backing `std::vector<ContactConstraint>` and `std::vector<FrictionConstraint>` can
reallocate when the high-water mark is exceeded (i.e., on the first frame and on any frame
with more contacts than seen before). After `reserve()` grows the vector, pointers into it
are invalidated during the `push_back` that triggers reallocation.

**Solution**: `ConstraintPool::allocateContact()` and `allocateFriction()` detect potential
reallocation and pre-`reserve()` before construction. The contract is:
- Callers must obtain pointers **after** construction, not store them across `allocate` calls.
- `allConstraints_` is rebuilt each frame after all pool allocations complete.

This matches the existing per-frame rebuild pattern: `allConstraints_` is already cleared and
repopulated every frame in `createConstraints()`.

An alternative is to use `std::deque` (stable pointers even on growth). However, deque has poor
cache locality and higher overhead per element. Given that all constraints for a frame are
allocated in a batch at the start of `createConstraints()`, pre-`reserve()` before any push
is the right approach: reserve once to the high-water mark + new contacts, then push_back without
triggering reallocation.

### `allConstraints_` Type Change

`allConstraints_` changes from `vector<unique_ptr<Constraint>>` to `vector<Constraint*>`.
This is a non-owning view into the pool's storage. Raw pointer semantics are safe here because:
1. The pool outlives `allConstraints_` (both are members of `CollisionPipeline`).
2. `allConstraints_` is cleared at the start of every frame before the pool is reset.
3. No external code stores `Constraint*` pointers beyond frame scope.

`buildSolverView()` and `buildContactView()` currently extract `.get()` from `unique_ptr`s.
After this change, `buildSolverView()` can return `allConstraints_` directly (it is already
`vector<Constraint*>`). `buildContactView()` continues to filter by `dynamic_cast`.

### `solvedLambdas` Vector Elimination

In `solveConstraintsWithWarmStart()`, the cache-update loop allocates
`std::vector<double> solvedLambdas` per pair. This is a small, short-lived allocation
(~8 doubles = 64 bytes typical).

**Fix**: Promote `solvedLambdas_` to a `CollisionPipeline` member workspace, using the same
pattern established by ticket 0071f: `solvedLambdas_.clear(); solvedLambdas_.reserve(count);`
at the start of each cache loop iteration. The vector is declared as a member alongside
`pairContactPoints_`.

### `islandConstraintSet` Allocation

The warm-start loop allocates `std::unordered_set<const Constraint*> islandConstraintSet`
per island. This is O(island constraints) per island per frame.

**Fix**: Promote `islandConstraintSet_` to a member workspace of type
`std::unordered_set<const Constraint*>`. Clear and repopulate per island using
`.clear()` + insert. This follows the 0071f pattern.

---

## Architecture Changes

### PlantUML Diagram

See `./0071g_constraint_pool_allocation.puml`

### New Component: `ConstraintPool`

**Header**: `msd/msd-sim/src/Physics/Constraints/ConstraintPool.hpp`
**Source**: `msd/msd-sim/src/Physics/Constraints/ConstraintPool.cpp`
**Location**: Physics/Constraints/ — alongside `ContactConstraint.hpp` and `FrictionConstraint.hpp`

```cpp
/**
 * @brief Per-frame typed free-list pool for constraint objects.
 *
 * Maintains two contiguous backing stores — one for ContactConstraint,
 * one for FrictionConstraint. After the first frame the backing vectors
 * have grown to their high-water mark; subsequent reset() calls return
 * counts to zero without deallocating, achieving zero heap allocation
 * for constraint objects in steady state.
 *
 * Pointer stability: Pointers returned by allocate*() are valid until
 * the next reset() call. Callers must not store pointers across frames.
 *
 * Extensibility: New constraint types (joints, limits) add a new
 * typed backing vector and a corresponding allocate*() method.
 * No changes are needed to existing allocation paths.
 *
 * Thread safety: Not thread-safe (single-threaded physics loop assumed).
 *
 * @ticket 0071g_constraint_pool_allocation
 */
class ConstraintPool {
public:
  ConstraintPool() = default;

  /**
   * @brief Construct a ContactConstraint in-pool and return a pointer to it.
   *
   * Grows the backing vector on first call or when count exceeds current
   * capacity. After the first frame no allocation occurs.
   *
   * @return Pointer to the constructed ContactConstraint.
   *         Valid until next reset().
   */
  ContactConstraint* allocateContact(
      size_t bodyAIndex, size_t bodyBIndex,
      const Coordinate& normal,
      const Coordinate& contactPointA, const Coordinate& contactPointB,
      double penetrationDepth,
      const Coordinate& comA, const Coordinate& comB,
      double restitution, double preImpactRelVelNormal);

  /**
   * @brief Construct a FrictionConstraint in-pool and return a pointer.
   */
  FrictionConstraint* allocateFriction(
      size_t bodyAIndex, size_t bodyBIndex,
      const Coordinate& normal,
      const Coordinate& contactPointA, const Coordinate& contactPointB,
      const Coordinate& comA, const Coordinate& comB,
      double frictionCoefficient);

  /**
   * @brief Reset all counts to zero. No memory is released.
   *
   * All previously returned pointers become invalid after this call.
   */
  void reset();

  /** @return Number of active ContactConstraints in the pool. */
  size_t contactCount() const;

  /** @return Number of active FrictionConstraints in the pool. */
  size_t frictionCount() const;

  // Rule of Zero: compiler-generated copy/move are correct
  // (backing vectors own their memory; counts are trivially copyable)

private:
  std::vector<ContactConstraint>  contactStorage_;
  std::vector<FrictionConstraint> frictionStorage_;
  size_t contactCount_{0};
  size_t frictionCount_{0};
};
```

### Modified Components

#### `CollisionPipeline`

**Header changes** (`CollisionPipeline.hpp`):
1. Add `#include "msd-sim/src/Physics/Constraints/ConstraintPool.hpp"`
2. Change `allConstraints_` from `vector<unique_ptr<Constraint>>` to `vector<Constraint*>`
3. Add `ConstraintPool constraintPool_` member (new owning member, declared before
   `allConstraints_` so pool outlives the view)
4. Add `std::vector<double> solvedLambdas_` member workspace (alongside `pairContactPoints_`)
5. Add `std::unordered_set<const Constraint*> islandConstraintSet_` member workspace
6. Remove `#include <memory>` if `unique_ptr` is no longer needed elsewhere (check)

**Source changes** (`CollisionPipeline.cpp`):
1. `clearEphemeralState()`: Replace `allConstraints_.clear()` with
   `allConstraints_.clear(); constraintPool_.reset();`
2. `createConstraints()`: Replace `std::make_unique<ContactConstraint>(...)` and
   `std::move(contactConstraints[ci])` with pool allocation pattern:
   ```cpp
   // Before:
   auto cc = std::make_unique<ContactConstraint>(...);
   allConstraints_.push_back(std::move(cc));

   // After:
   ContactConstraint* cc = constraintPool_.allocateContact(...);
   allConstraints_.push_back(cc);
   ```
3. `createConstraints()`: Replace `std::make_unique<FrictionConstraint>(...)` similarly.
4. `buildSolverView()`: Simplify — `allConstraints_` is already `vector<Constraint*>`,
   so the method returns `allConstraints_` directly (both interleaved and non-interleaved
   cases are the same since the storage pattern is identical).
5. `solveConstraintsWithWarmStart()`: Replace `std::vector<double> solvedLambdas` local
   variable with `solvedLambdas_` member (clear + reserve per iteration).
6. `solveConstraintsWithWarmStart()`: Replace `std::unordered_set<const Constraint*>
   islandConstraintSet` local variable with `islandConstraintSet_` member (clear + insert).
7. `solverData_.numConstraints = allConstraints_.size()` — unchanged.
8. All `dynamic_cast<ContactConstraint*>(c.get())` become
   `dynamic_cast<ContactConstraint*>(c)` (raw pointer, no `.get()`).

#### `ContactConstraintFactory`

The factory currently returns `vector<unique_ptr<ContactConstraint>>`. After this change,
`createConstraints()` in `CollisionPipeline` will call pool allocation directly instead of
going through the factory.

**Two approaches**:

**Approach 1 (Simpler)**: Keep factory unchanged. `createConstraints()` calls the factory,
then moves each constraint into the pool via placement construction. This avoids modifying
the factory's public API.
- **Drawback**: Creates and immediately destroys a `unique_ptr` per constraint — defeats the purpose.

**Approach 2 (Correct)**: Inline the factory logic directly into `createConstraints()`,
replacing the factory call with direct pool allocation using the same parameter computation
(pre-impact velocity, lever arms). The factory functions `computeRelativeNormalVelocity()` and
`combineRestitution()` remain as free functions — only `createFromCollision()` is bypassed.
- **Benefit**: Zero intermediate allocations. The factory's `createFromCollision()` is only
  called from `CollisionPipeline::createConstraints()` (one call site), so this change has
  no API breadth impact.

**Selected**: Approach 2. Inline the loop from `createFromCollision()` directly into
`createConstraints()`. The factory utility functions are preserved.

---

## Interface Preservation

| Interface | Preserved? | Notes |
|-----------|-----------|-------|
| `ConstraintSolver::solve(vector<Constraint*>, ...)` | Yes | Unchanged |
| `ProjectedGaussSeidel::solve(vector<Constraint*>, ...)` | Yes | Unchanged |
| `PositionCorrector::correctPositions(vector<Constraint*>, ...)` | Yes | Unchanged |
| `ConstraintIslandBuilder::buildIslands(vector<Constraint*>, ...)` | Yes | Unchanged |
| `CollisionPipeline::execute()` | Yes | Unchanged |
| `CollisionPipeline::recordConstraints()` | Yes | Unchanged — iterates `allConstraints_` |
| `ContactConstraintFactory::createFromCollision()` | Kept, but unused by pipeline | Free functions remain |

---

## Pointer Stability Analysis

The backing vectors in `ConstraintPool` can reallocate when `push_back` exceeds capacity.
To guarantee pointer stability within a frame:

1. `ConstraintPool::allocateContact()` reserves additional capacity before the first push:
   ```cpp
   if (contactCount_ == contactStorage_.size()) {
     // Growth: double capacity (amortized O(1))
     contactStorage_.reserve(std::max(contactStorage_.size() * 2, size_t{1}));
   }
   // Capacity guaranteed — push_back will not reallocate
   contactStorage_.push_back(ContactConstraint{...});
   return &contactStorage_[contactCount_++];
   ```
   Wait — this is wrong. After `reserve()`, `contactStorage_.push_back()` does not
   reallocate, but `contactStorage_.size()` increases. The pool needs to track which
   slots are "active" (count) vs "allocated" (storage size).

   **Correct implementation**: Use `resize()` to pre-grow the backing vector to capacity
   and use placement assignment:
   ```cpp
   if (contactCount_ >= contactStorage_.size()) {
     contactStorage_.resize(contactStorage_.size() + kGrowthChunk);
   }
   contactStorage_[contactCount_] = ContactConstraint{args...};
   return &contactStorage_[contactCount_++];
   ```
   This allows reuse of existing slots by assignment (ContactConstraint has default operator=).
   No reallocation occurs after the vector reaches high-water mark because `resize()` only
   grows (never shrinks). On reset, contactCount_ is set to 0; the storage objects remain
   valid in memory.

   `kGrowthChunk` is set to 32 (typical contact count per scene). In practice, the vector
   reaches high-water mark within a few frames and never reallocates again.

   Pointers `&contactStorage_[i]` are stable as long as `contactStorage_` does not reallocate.
   After `resize()` the vector's buffer is stable for indices `[0, contactStorage_.size())`.
   The pool guarantees: it will never call `push_back` or `resize` on the backing vector
   after returning a pointer from `allocate*()` within the same frame (reset to frame start,
   all allocations happen in `createConstraints()`, then pointers are used in subsequent phases).

   The frame lifecycle is:
   1. `clearEphemeralState()` → `constraintPool_.reset()` (count=0, storage intact)
   2. `createConstraints()` → all `allocate*()` calls (storage grows if needed)
   3. Solver, position correction, data recording → read-only pointer usage
   4. Next frame → goto 1

   There is no interleaving of grow and use within a frame.

---

## Warm-Start Vector Elimination

### `solvedLambdas_` Workspace

**Before**: Inside the cache-update loop (lines 646-658 of `CollisionPipeline.cpp`):
```cpp
std::vector<double> solvedLambdas;
solvedLambdas.reserve(range.count * lambdasPerContact);
for (...) { solvedLambdas.push_back(...); }
contactCache_.update(..., solvedLambdas, ...);
```

**After**: Member `std::vector<double> solvedLambdas_` in `CollisionPipeline`:
```cpp
// In cache-update loop:
solvedLambdas_.clear();
solvedLambdas_.reserve(range.count * lambdasPerContact);
for (...) { solvedLambdas_.push_back(...); }
contactCache_.update(..., solvedLambdas_, ...);
```

After the first frame, `solvedLambdas_` has sufficient capacity and `clear()` + `reserve()`
are both O(1) no-ops for the steady-state size.

### `islandConstraintSet_` Workspace

**Before**: Inside the island loop (~line 543 of `CollisionPipeline.cpp`):
```cpp
std::unordered_set<const Constraint*> islandConstraintSet(
  islandConstraints.begin(), islandConstraints.end());
```

**After**: Member `std::unordered_set<const Constraint*> islandConstraintSet_`:
```cpp
islandConstraintSet_.clear();
islandConstraintSet_.insert(islandConstraints.begin(), islandConstraints.end());
```

`unordered_set::clear()` resets the count but keeps allocated buckets, so rehashing is
avoided on subsequent frames when the island size is similar.

---

## Success Criteria

| Criterion | How Verified |
|-----------|-------------|
| All 718+ existing physics tests pass | `cmake --build --preset debug-sim-only --target msd_sim_test` |
| Zero per-frame heap allocs for constraint objects after frame 1 | Review: no `make_unique` or `new` in `createConstraints()` |
| `solvedLambdas_` and `islandConstraintSet_` are reused members | Code review: local variable declarations removed from loops |
| `vector<Constraint*>` interface preserved for solver/corrector | Unchanged solver headers verified |
| New constraint types addable without pool redesign | Design: new type → add backing vector + `allocate*()` method |

---

## Risks and Mitigations

| Risk | Likelihood | Mitigation |
|------|-----------|------------|
| Pointer invalidation during `createConstraints()` if pool grows mid-allocation | Medium | Pool pre-reserves capacity before any `allocate*()` call via high-water-mark `resize()` pattern |
| `allConstraints_` contains stale raw pointers after `reset()` | Low | `clearEphemeralState()` clears both `allConstraints_` and calls `constraintPool_.reset()` atomically |
| `dynamic_cast` sites miss `.get()` removal | Low | Compiler error: `dynamic_cast` from `unique_ptr` is ill-formed |
| Factory bypass misses some parameter computation | Medium | Unit test `ContactConstraintFactoryTest` verifies constraint construction — post-change tests validate pool-allocated constraints behave identically |

---

## Alternative Considered: Keep Factory, Add Pool Overload

A second variant of Option B would add a `ConstraintPool*` parameter to
`contact_constraint_factory::createFromCollision()`. The factory would allocate into the pool
instead of via `make_unique`.

**Rejected** because:
1. It couples the stateless factory to the owning pool.
2. The factory's `vector<unique_ptr<ContactConstraint>>` return type would need to change to
   `vector<ContactConstraint*>`, which propagates the change outward.
3. There is only one call site (`CollisionPipeline::createConstraints()`), so inlining
   is straightforward and avoids interface pollution.

---

## Open Questions (Resolved)

**Q: Should `ConstraintPool` be a separate class or inline in `CollisionPipeline`?**
A: Separate class. Encapsulation keeps `CollisionPipeline` from becoming a monolith.
   Future joint constraints add a pool member without touching `CollisionPipeline` logic.

**Q: Should `allConstraints_` remain `vector<unique_ptr<Constraint>>` with a custom no-op deleter?**
A: No. A custom no-op deleter adds cognitive overhead and is a code smell when pool ownership
   is clearly established. `vector<Constraint*>` directly communicates "non-owning view".

**Q: Does `contactCache_.update()` store a copy or reference to `solvedLambdas`?**
A: `ContactCache::update()` takes `const std::vector<double>&` and stores a copy internally.
   Reusing `solvedLambdas_` as a workspace is safe.
