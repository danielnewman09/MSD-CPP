# Ticket 0040d: Contact Persistence and Warm-Starting

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Quality Gate Passed
**Assignee**: TBD
**Created**: 2026-02-07
**Generate Tutorial**: No
**Parent Ticket**: [0040_collision_stabilization_phase2](0040_collision_stabilization_phase2.md)
**Dependencies**: [0040b](0040b_split_impulse_position_correction.md)
**Type**: Implementation

---

## Overview

Constraints are completely destroyed and recreated each frame in `WorldModel::updateCollisions()`. All λ (impulse) values start from zero every frame. The solver must reconverge from scratch, causing frame-to-frame jitter and requiring many iterations for stable resting contacts.

This ticket adds a contact cache that preserves solved λ values across frames, enabling warm-starting the solver with previous solutions as initial guesses.

---

## Problem

### Current Behavior

Each frame:
1. `updateCollisions()` clears all constraints
2. Collision detection produces new `CollisionResult`s
3. `ContactConstraintFactory` creates new constraints with λ = 0
4. Solver iterates from zero to find the correct impulses
5. For resting contacts, the correct λ is approximately `m·g·dt` every frame
6. The solver must "rediscover" this value each frame

This causes:
- **Jitter**: Small frame-to-frame variations in solved λ produce oscillating forces
- **Slow convergence**: Starting from zero requires more iterations to reach the correct partition in the Active Set Method
- **Energy artifacts**: Reconvergence errors compound over hundreds of frames

### Desired Behavior

A `ContactCache` maps body pairs to their previous frame's solved contact data (λ values, contact normals, contact points). When contacts persist between frames, the solver starts from the previous λ, requiring only small corrections. When contacts change significantly (normal rotation >15°), the cache is invalidated.

---

## Requirements

### R1: ContactCache Data Structure

Create a `ContactCache` that maps body pairs to previous frame contact state:

```cpp
struct CachedContact {
  uint32_t bodyA_id;
  uint32_t bodyB_id;
  Vector3D normal;              // Previous frame contact normal
  std::vector<float> lambdas;   // Previous frame solved λ values
  std::vector<Coordinate> points; // Previous frame contact points
  uint32_t age{0};              // Frames since last refresh
};
```

Key: `(min(bodyA_id, bodyB_id), max(bodyA_id, bodyB_id))` — symmetric for body order independence.

### R2: Cache Update Protocol

Each frame:

1. **Before solve**: For each new contact, look up the body pair in the cache
2. **Match contacts**: Match new contact points to cached points by proximity (nearest-neighbor within threshold)
3. **Warm-start**: If match found and normal hasn't changed >15°, use cached λ as initial guess
4. **Invalidate**: If normal changed >15° or no cached entry exists, start from λ = 0
5. **After solve**: Store the solved λ values back into the cache
6. **Expire**: Remove cache entries older than N frames (e.g., 10) without refresh

### R3: Accept Initial λ in Solver

Modify `ConstraintSolver` to accept an optional initial λ vector:

```cpp
void solve(const std::vector<Constraint*>& constraints,
           double dt,
           const std::vector<float>& initialLambda = {});
```

The Active Set Method should use `initialLambda` as the starting point instead of zero.

### R4: Normal Similarity Threshold

Use dot product to compare normals:

```cpp
bool normalsSimilar(const Vector3D& a, const Vector3D& b) {
  return a.dot(b) > cos(15° * π/180);  // ~0.966
}
```

If normals diverge beyond 15°, the previous λ is likely wrong and would cause the solver to diverge.

### R5: Contact Point Matching

Match new contact points to cached points using nearest-neighbor:

```cpp
float matchThreshold = 0.02f;  // 2cm matching radius
```

Unmatched new contacts start with λ = 0. Unmatched cached contacts are discarded.

---

## Affected Tests

This ticket improves convergence quality across all scenarios. It does not target specific failing tests but reduces jitter and improves stability for:

| Test Category | Improvement |
|---------------|------------|
| D1 (1000 frame stability) | Less drift from reconvergence errors |
| D4 (Small jitter) | Warm-started solver doesn't oscillate |
| C2/C3 (Rocking/settling) | Faster convergence to rest |
| All resting contacts | Smoother force profiles |

---

## Implementation Approach

### Step 1: Create ContactCache

```cpp
// msd-sim/src/Physics/Constraints/ContactCache.hpp

namespace msd_sim {

class ContactCache {
public:
  using BodyPairKey = std::pair<uint32_t, uint32_t>;

  /// Look up cached lambdas for a body pair
  /// @return Cached lambdas if contact persists, empty vector otherwise
  std::vector<float> getWarmStart(
    uint32_t bodyA, uint32_t bodyB,
    const Vector3D& currentNormal,
    const std::vector<Coordinate>& currentPoints) const;

  /// Store solved lambdas for a body pair
  void update(
    uint32_t bodyA, uint32_t bodyB,
    const Vector3D& normal,
    const std::vector<float>& lambdas,
    const std::vector<Coordinate>& points);

  /// Remove entries older than maxAge frames
  void expireOldEntries(uint32_t maxAge = 10);

  /// Increment age of all entries (call once per frame)
  void advanceFrame();

  /// Clear all cached data
  void clear();

private:
  static BodyPairKey makeKey(uint32_t a, uint32_t b);
  std::unordered_map<BodyPairKey, CachedContact, PairHash> cache_;

  float normalThreshold_{0.966f};  // cos(15°)
  float pointMatchRadius_{0.02f};  // 2cm
};

} // namespace msd_sim
```

### Step 2: Integrate into WorldModel

```cpp
// In WorldModel class:
ContactCache contactCache_;

void WorldModel::updateCollisions(double dt) {
  // 1. Detect collisions
  // 2. Build constraints

  // 3. Warm-start from cache
  auto initialLambda = contactCache_.getWarmStart(...);

  // 4. Solve with initial guess
  solver_.solve(constraints, dt, initialLambda);

  // 5. Position correction (from 0040b)

  // 6. Update cache with solved values
  contactCache_.update(...);
  contactCache_.advanceFrame();
  contactCache_.expireOldEntries();
}
```

### Step 3: Modify Solver

In `ConstraintSolver::solve()`, initialize λ from the provided vector instead of zeros:

```cpp
if (!initialLambda.empty()) {
  lambda_ = initialLambda;
} else {
  lambda_ = Eigen::VectorXd::Zero(n);
}
```

---

## Test Plan

### Unit Tests

```cpp
// Cache basic operations
TEST(ContactCache, StoreAndRetrieve_MatchingContact)
TEST(ContactCache, NormalChange_InvalidatesCache)
TEST(ContactCache, PointMatching_NearestNeighbor)
TEST(ContactCache, Expiry_RemovesOldEntries)
TEST(ContactCache, BodyOrderIndependent)
TEST(ContactCache, Clear_RemovesAllEntries)

// Warm-starting
TEST(WarmStart, RestingContact_ConvergesInFewerIterations)
TEST(WarmStart, NormalFlip_StartsFromZero)
TEST(WarmStart, NewContact_StartsFromZero)
TEST(WarmStart, PartialMatch_MixedInitialization)
```

### Integration Tests

```cpp
// Resting contact stability
TEST(WarmStart, RestingCube_ReducedJitter)
// Compare position variance with and without warm-starting over 1000 frames

TEST(WarmStart, StackedCubes_StableWithWarmStart)
// 3-cube stack stable for 500 frames

TEST(WarmStart, BouncingCube_CacheInvalidationOnBounce)
// Contact breaks and reforms — cache should invalidate, not inject stale impulses
```

### Regression Tests

- All existing collision tests must pass
- All 0039b linear collision tests must pass
- All 0040b split impulse tests must pass

---

## Acceptance Criteria

1. [ ] **AC1**: `ContactCache` class implemented with body-pair keying
2. [ ] **AC2**: `ConstraintSolver` accepts initial λ vector
3. [ ] **AC3**: Cache invalidation on normal change >15°
4. [ ] **AC4**: Cache expiry after N frames without contact
5. [ ] **AC5**: Resting contacts show reduced position variance with warm-starting
6. [ ] **AC6**: No regression in existing collision tests
7. [ ] **AC7**: Stale cache entries don't cause energy spikes

---

## Files to Modify

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Add initial λ parameter to `solve()` |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Use initial λ instead of zero vector |
| `msd-sim/src/Environment/WorldModel.hpp` | Add `ContactCache` member |
| `msd-sim/src/Environment/WorldModel.cpp` | Integrate cache into `updateCollisions()` |

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/src/Physics/Constraints/ContactCache.hpp` | Contact cache declaration |
| `msd-sim/src/Physics/Constraints/ContactCache.cpp` | Contact cache implementation |
| `msd-sim/test/Physics/Constraints/ContactCacheTest.cpp` | Unit tests |
| `msd-sim/test/Physics/Constraints/WarmStartTest.cpp` | Integration tests |

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-07
- **Notes**: Polish step for convergence quality. Most effective after 0040b changes the solver behavior. Addresses the "constraints cleared each frame" architectural gap noted in MEMORY.md.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
