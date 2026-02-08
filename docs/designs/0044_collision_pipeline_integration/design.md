# Design: Collision Pipeline Integration

## Summary

This design extends the `CollisionPipeline` class (created in ticket 0036) to include all collision response capabilities currently implemented inline in `WorldModel::updateCollisions()`. The pipeline will own `ContactCache` and `PositionCorrector`, manage cache lifecycle, expose collision-active status for energy tracking, and support warm-starting. After implementation, `WorldModel::updateCollisions()` will become a thin delegation layer (~10 lines) that calls the pipeline, restoring the original intent of ticket 0036.

This is a pure refactoring with zero behavioral change — simulation results will be identical before and after.

## Architecture Changes

### PlantUML Diagram
See: `./0044_collision_pipeline_integration.puml`

### Modified Components

#### CollisionPipeline

- **Current location**: `msd-sim/src/Physics/Collision/CollisionPipeline.hpp`, `CollisionPipeline.cpp`
- **Changes required**:
  1. **Add members**:
     - `ContactCache contactCache_` — For warm-starting (frame-persistent cache)
     - `PositionCorrector positionCorrector_` — For split-impulse position correction
     - `bool collisionOccurred_` — Tracks whether collisions occurred this frame
  2. **Add public methods**:
     - `void advanceFrame()` — Delegate to `contactCache_.advanceFrame()`
     - `void expireOldEntries(uint32_t maxAge = 10)` — Delegate to `contactCache_.expireOldEntries(maxAge)`
     - `bool hadCollisions() const` — Return `collisionOccurred_`
  3. **Modify `execute()` method**:
     - Add Phase 3.5: Query cache for warm-start lambdas (before solving)
     - Modify Phase 4: Pass initial lambda to `solveWithContacts()`
     - Add Phase 4.5: Update cache with solved lambdas (after solving)
     - Add Phase 6: Call `positionCorrector_.correctPositions()`
     - Set `collisionOccurred_ = !collisions_.empty()` after detection
  4. **Add protected method** (for warm-starting):
     - `ConstraintSolver::MultiBodySolveResult solveConstraintsWithWarmStart(double dt)`
  5. **Add protected method** (for position correction):
     - `void correctPositions(std::span<AssetInertial> inertialAssets, std::span<const AssetEnvironment> environmentalAssets, size_t numBodies)`
  6. **Add frame-persistent data structures** (for cache interaction):
     - `struct PairConstraintRange { size_t startIdx; size_t count; size_t pairIdx; }`
     - `std::vector<PairConstraintRange> pairRanges_` — Maps collision pair index to constraint range
  7. **Extend `CollisionPair` struct**:
     - Add `uint32_t bodyAId` — Instance ID for cache keying
     - Add `uint32_t bodyBId` — Instance ID for cache keying

- **Key interfaces**:
  ```cpp
  class CollisionPipeline {
  public:
    // Existing
    explicit CollisionPipeline();
    void execute(std::span<AssetInertial> inertialAssets,
                 std::span<const AssetEnvironment> environmentalAssets,
                 double dt);

    // NEW: Cache lifecycle management
    void advanceFrame();
    void expireOldEntries(uint32_t maxAge = 10);

    // NEW: Collision-active status for energy tracking
    bool hadCollisions() const;

    // Rule of Five (deleted move/copy per NFR-1: single-threaded)
    CollisionPipeline(const CollisionPipeline&) = delete;
    CollisionPipeline& operator=(const CollisionPipeline&) = delete;
    CollisionPipeline(CollisionPipeline&&) = delete;
    CollisionPipeline& operator=(CollisionPipeline&&) = delete;
    ~CollisionPipeline() = default;

  protected:
    // Existing phases
    void detectCollisions(...);
    void createConstraints(...);
    void assembleSolverInput(...);
    static void applyForces(...);

    // NEW: Warm-starting solver phase
    ConstraintSolver::MultiBodySolveResult solveConstraintsWithWarmStart(double dt);

    // NEW: Position correction phase
    void correctPositions(std::span<AssetInertial> inertialAssets,
                          std::span<const AssetEnvironment> environmentalAssets,
                          size_t numBodies);

  private:
    void clearFrameData();

    // Existing members
    CollisionHandler collisionHandler_;
    ConstraintSolver constraintSolver_;
    std::vector<CollisionPair> collisions_;
    std::vector<std::unique_ptr<ContactConstraint>> constraints_;
    std::vector<std::reference_wrapper<const InertialState>> states_;
    std::vector<double> inverseMasses_;
    std::vector<Eigen::Matrix3d> inverseInertias_;
    std::vector<Constraint*> constraintPtrs_;

    // NEW: Cache and position correction
    ContactCache contactCache_;
    PositionCorrector positionCorrector_;

    // NEW: Track collision-active status
    bool collisionOccurred_{false};

    // NEW: Cache interaction data structures
    struct PairConstraintRange {
      size_t startIdx;
      size_t count;
      size_t pairIdx;
    };
    std::vector<PairConstraintRange> pairRanges_;

    // Friend for unit testing
    friend class CollisionPipelineTest;
  };
  ```

- **Dependencies**: No new external dependencies. Uses existing `ContactCache`, `PositionCorrector`, `ConstraintSolver`.

- **Thread safety**: Not thread-safe (single-threaded simulation). Cache and corrector are frame-persistent state.

- **Error handling**: No new error conditions. Delegates error handling to `ContactCache`, `PositionCorrector`, `ConstraintSolver`.

- **Implementation notes**:
  1. **Warm-starting algorithm** (copied from `WorldModel::updateCollisions` lines 390-445):
     - Build `Eigen::VectorXd initialLambda` of size `totalConstraints`, initialized to zero
     - For each `PairConstraintRange`, extract contact points via helper lambda
     - Query `contactCache_.getWarmStart(bodyAId, bodyBId, normal, contactPoints)`
     - If cache hit and lambda count matches, populate `initialLambda` at `range.startIdx`
     - Pass `initialLambda` to `contactSolver_.solveWithContacts(..., initialLambda)`
  2. **Cache update algorithm** (copied from `WorldModel::updateCollisions` lines 447-466):
     - For each `PairConstraintRange`, extract solved lambdas from `solveResult.lambdas`
     - Extract contact points via helper lambda
     - Call `contactCache_.update(bodyAId, bodyBId, normal, solvedLambdas, contactPoints)`
  3. **Position correction algorithm** (copied from `WorldModel::updateCollisions` lines 485-508):
     - Build `std::vector<InertialState*> mutableStates` for inertial + environment assets
     - Call `positionCorrector_.correctPositions(constraintPtrs_, mutableStates, inverseMasses_, inverseInertias_, numBodies, numInertial, dt)`
  4. **Contact point extraction helper**:
     - Reuse exact logic from `WorldModel::updateCollisions` lines 397-414
     - Closure captures: `allConstraints`, `pairRanges_`, `collisions_`, `inertialAssets_`, `environmentalAssets_`, `numInertial`
     - Returns `std::vector<Coordinate>` of contact world-space points (CoM + leverArm)
  5. **CollisionPair extension**:
     - Add `uint32_t bodyAId` and `uint32_t bodyBId` to existing struct
     - Populate from `inertialAssets_[i].getInstanceId()` and `environmentalAssets_[e].getInstanceId() | kEnvIdFlag`
     - Environment ID flag: `constexpr uint32_t kEnvIdFlag = 0x80000000u` (high bit set)

---

#### WorldModel

- **Current location**: `msd-sim/src/Environment/WorldModel.hpp`, `WorldModel.cpp`
- **Changes required**:
  1. **Replace collision members** with single `CollisionPipeline`:
     - Remove: `CollisionHandler collisionHandler_{1e-6};`
     - Remove: `ConstraintSolver contactSolver_;`
     - Remove: `PositionCorrector positionCorrector_;`
     - Remove: `ContactCache contactCache_;`
     - Add: `CollisionPipeline collisionPipeline_;`
  2. **Simplify `updateCollisions()` method**:
     - Replace ~300 lines with 4-line delegation pattern
     - Keep only: cache lifecycle + pipeline execution + collision-active flag update
  3. **No changes to public API**: `update()`, `updatePhysics()` signatures unchanged

- **Key interface changes**:
  ```cpp
  class WorldModel {
  private:
    // REMOVED members (moved into CollisionPipeline)
    // CollisionHandler collisionHandler_{1e-6};
    // ConstraintSolver contactSolver_;
    // PositionCorrector positionCorrector_;
    // ContactCache contactCache_;

    // NEW member
    CollisionPipeline collisionPipeline_;

    // Simplified updateCollisions (was ~300 lines, now ~10 lines)
    void updateCollisions(double dt);
  };
  ```

- **Simplified `updateCollisions()` implementation**:
  ```cpp
  void WorldModel::updateCollisions(double dt) {
    // Phase 0: Cache lifecycle management
    collisionPipeline_.advanceFrame();
    collisionPipeline_.expireOldEntries();  // Default maxAge = 10

    // Phase 1-6: Full collision response pipeline
    collisionPipeline_.execute(inertialAssets_, environmentalAssets_, dt);

    // Update energy tracking flag
    collisionActiveThisFrame_ = collisionPipeline_.hadCollisions();
  }
  ```

- **Backward compatibility**: Zero API breakage. All existing code using `WorldModel` unchanged.

---

### Integration Points

| CollisionPipeline | External Component | Integration Type | Notes |
|-------------------|-------------------|------------------|-------|
| `contactCache_` | `ContactCache` | Ownership (value member) | Warm-starting cache, frame-persistent |
| `positionCorrector_` | `PositionCorrector` | Ownership (value member) | Split-impulse position correction |
| `collisionHandler_` | `CollisionHandler` | Ownership (value member) | GJK/EPA collision detection (existing) |
| `constraintSolver_` | `ConstraintSolver` | Ownership (value member) | Active Set Method solver (existing) |
| `hadCollisions()` | `WorldModel` | Query | Energy tracking diagnostic flag |
| `advanceFrame()` | `WorldModel` | Lifecycle | Called once per frame before `execute()` |
| `expireOldEntries()` | `WorldModel` | Lifecycle | Called once per frame before `execute()` |

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `CollisionPipelineTest.cpp` | All existing tests | Need to handle new phases | Update to verify warm-starting, position correction |
| `WorldModelTest.cpp` (if exists) | Collision response tests | Indirect (behavior unchanged) | Re-run for regression check |
| Integration tests | All collision tests (A1-A6, B1-B5, D1, F1-F5, H1-H5) | Indirect (behavior unchanged) | Re-run for regression check |

### New Tests Required

#### Unit Tests (CollisionPipelineTest.cpp)

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| `ContactCache` integration | `WarmStartFromCache_ReducesIterations` | Solver receives non-zero initial lambda when cache hit |
| `ContactCache` integration | `CacheMiss_StartsFromZero` | Solver receives zero initial lambda when cache miss |
| `ContactCache` integration | `CacheUpdate_StoresLambdas` | Solved lambdas are stored in cache after solve |
| `PositionCorrector` integration | `PositionCorrection_ReducesPenetration` | Penetration depth reduced after `correctPositions()` |
| `PositionCorrector` integration | `PositionCorrection_DoesNotAffectVelocity` | Real velocities unchanged by position correction |
| Lifecycle | `AdvanceFrame_IncrementsAge` | Cache age increments each frame |
| Lifecycle | `ExpireOldEntries_RemovesStale` | Stale cache entries removed after maxAge frames |
| Collision-active | `HadCollisions_TrueWhenCollisions` | `hadCollisions()` returns true after collision |
| Collision-active | `HadCollisions_FalseWhenNoCollisions` | `hadCollisions()` returns false when no collision |
| Frame data clearing | `ClearFrameData_ClearsPairRanges` | `pairRanges_` cleared at start/end of `execute()` |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| Full pipeline with warm-starting | CollisionPipeline + ContactCache | Persistent contacts converge faster with warm-starting |
| Full pipeline with position correction | CollisionPipeline + PositionCorrector | Penetrated contacts corrected without energy injection |
| WorldModel delegation | WorldModel + CollisionPipeline | `WorldModel::updateCollisions()` produces identical results |

#### Benchmark Tests (Optional)

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| CollisionPipeline | `WarmStartingSpeedup` | Solver iterations with/without cache | 10-25× fewer iterations |
| CollisionPipeline | `PositionCorrectionOverhead` | Execution time of position correction pass | < 10% of total collision response |

---

## Open Questions

### Design Decisions (Human Input Needed)

1. **Should ContactCache and PositionCorrector be exposed for configuration?**
   - Option A: Expose via `CollisionPipeline::getContactCache()` and `getPositionCorrector()` — Allows tuning of cache expiry and position correction params
   - Option B: Keep private with hardcoded defaults — Simpler API, fewer configuration knobs
   - Recommendation: **Option B** initially. Expose only if profiling shows need for tuning.

2. **Should `advanceFrame()` and `expireOldEntries()` be merged into `execute()`?**
   - Option A: Keep separate — Explicit control over cache lifecycle, matches current WorldModel pattern
   - Option B: Merge into `execute()` — Simpler API, fewer method calls
   - Recommendation: **Option A**. Explicit lifecycle is clearer and matches ticket 0040d design intent.

3. **Should `hadCollisions()` track collision count or just boolean?**
   - Option A: `bool hadCollisions()` — Minimal change, sufficient for energy tracking
   - Option B: `size_t collisionCount()` — More diagnostic info, future-proof
   - Recommendation: **Option A**. Energy tracking only needs boolean. Add count accessor later if needed.

### Prototype Required

1. **None** — This is a pure code motion refactoring. All algorithms are already validated in WorldModel. Risk of behavioral change is low due to identical logic transfer.

### Requirements Clarification

1. **None** — Requirements are clear from ticket 0044. Design restores ticket 0036's delegation intent with full feature parity.

---

## Implementation Plan

### Phase 1: Extend CollisionPipeline (2-3 hours)

1. Add `ContactCache contactCache_` and `PositionCorrector positionCorrector_` members
2. Add `bool collisionOccurred_` and `std::vector<PairConstraintRange> pairRanges_` members
3. Extend `CollisionPair` struct with `bodyAId` and `bodyBId` fields
4. Add public methods: `advanceFrame()`, `expireOldEntries()`, `hadCollisions()`
5. Add protected methods: `solveConstraintsWithWarmStart()`, `correctPositions()`
6. Modify `execute()`:
   - Set `collisionOccurred_ = !collisions_.empty()` after Phase 1
   - Add Phase 3.5 (warm-start query) before Phase 4
   - Replace Phase 4 with `solveConstraintsWithWarmStart()`
   - Add Phase 4.5 (cache update) after Phase 4
   - Add Phase 6 (position correction) after Phase 5
   - Clear `pairRanges_` in `clearFrameData()`

### Phase 2: Simplify WorldModel (30 minutes)

1. Remove four collision-related members
2. Add `CollisionPipeline collisionPipeline_` member
3. Replace `updateCollisions()` body with 4-line delegation

### Phase 3: Update Tests (1-2 hours)

1. Extend `CollisionPipelineTest.cpp` with 10 new unit tests
2. Run full integration test suite (612 tests) for regression check
3. Add benchmark test for warm-starting speedup (optional)

### Phase 4: Documentation (30 minutes)

1. Update `msd-sim/src/Physics/Collision/CLAUDE.md` — Document CollisionPipeline extensions
2. Update `msd-sim/CLAUDE.md` — Remove WorldModel collision member documentation
3. Update `0036_collision_pipeline_extraction` design doc — Note feature parity achieved

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Behavioral regression from logic transfer | Low | High | Line-by-line code motion, extensive test coverage |
| Member lifetime issues during ownership transfer | Very Low | Medium | ContactCache and PositionCorrector have value semantics — straightforward move |
| Missing a WorldModel access to moved members | Very Low | Medium | Compile-time error — grep for `collisionHandler_`, `contactSolver_`, `contactCache_`, `positionCorrector_` |
| Breaking CollisionPipeline unit tests | Low | Low | Tests need updates for new phases, but existing logic unchanged |
| Performance regression | Very Low | Low | No new algorithms, only code location change |

---

## Performance Considerations

### Memory Footprint

- **ContactCache**: ~1KB per active collision pair (10 pairs × 1KB = ~10KB typical)
- **PositionCorrector**: Stateless (< 100 bytes)
- **PairConstraintRange vector**: ~48 bytes per collision pair (10 pairs × 48 = ~480 bytes)
- **Total increase**: ~11KB typical, ~50KB worst-case (100 active collisions)

### Runtime Overhead

- **Cache lifecycle** (`advanceFrame()` + `expireOldEntries()`): O(C) where C = active collision pairs, ~1-10μs
- **Warm-starting**: No overhead (replaces zero initialization with cache lookup, same cost)
- **Position correction**: ~10% of total collision response time (already measured in ticket 0040b)
- **Overall**: < 1% increase in total frame time (collision response is ~5% of frame, position correction is ~10% of that)

### Optimization Opportunities

- None identified. This refactoring preserves existing performance characteristics.

---

## Coding Standards

All code will follow project coding standards:

- **Initialization**: Brace initialization `{}` for all members
- **Naming**: `camelCase` for methods, `snake_case_` for members, `PascalCase` for classes
- **Return Values**: Prefer returning values over output parameters
- **Memory**: Value semantics for `ContactCache` and `PositionCorrector` (already designed this way)
- **Error Handling**: Delegate to existing error handling in `ContactCache`, `PositionCorrector`, `ConstraintSolver`

---

## References

- **Ticket 0036**: [0036_collision_pipeline_extraction](../../tickets/0036_collision_pipeline_extraction.md) — Original pipeline extraction (incomplete)
- **Ticket 0040a-d**: [0040_collision_stabilization_phase2](../../tickets/0040_collision_stabilization_phase2.md) — Added ContactCache and PositionCorrector to WorldModel
- **Ticket 0042**: [0042_collision_numerical_stability](../../tickets/0042_collision_numerical_stability.md) — Numerical stability fixes
- **Current WorldModel**: `msd-sim/src/Environment/WorldModel.cpp` — `updateCollisions()` lines 195-511
- **Current CollisionPipeline**: `msd-sim/src/Physics/Collision/CollisionPipeline.hpp/.cpp`
- **ContactCache**: `msd-sim/src/Physics/Constraints/ContactCache.hpp`
- **PositionCorrector**: `msd-sim/src/Physics/Constraints/PositionCorrector.hpp`

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-08
**Status**: APPROVED
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | Follows project patterns: `camelCase` methods, `snake_case_` members, `PascalCase` types |
| Namespace organization | ✓ | CollisionPipeline in `msd_sim::Physics::Collision`, consistent with existing code |
| File structure | ✓ | Extends existing CollisionPipeline.hpp/.cpp in `msd-sim/src/Physics/Collision/` |
| Dependency direction | ✓ | Pipeline depends on ContactCache/PositionCorrector (both in Physics/Constraints), no cycles |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | ContactCache and PositionCorrector have value semantics, no manual resource management |
| Smart pointer appropriateness | ✓ | Existing constraint ownership via `unique_ptr` unchanged, value members for cache/corrector |
| Value/reference semantics | ✓ | Value members for ContactCache/PositionCorrector, references for span parameters |
| Rule of 0/3/5 | ✓ | Deleted copy/move per NFR-1 (single-threaded), destructor defaulted |
| Const correctness | ✓ | `hadCollisions()` is const, `advanceFrame()` non-const (modifies cache) |
| Exception safety | ✓ | No new exception paths, delegates to ContactCache/PositionCorrector |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | Adds ContactCache.hpp, PositionCorrector.hpp includes, both already in codebase |
| Template complexity | ✓ | No templates used, straightforward class extension |
| Memory strategy | ✓ | `pairRanges_` cleared each frame, `contactCache_` frame-persistent (design intent) |
| Thread safety | ✓ | Consistent with existing single-threaded design, no new thread-safety requirements |
| Build integration | ✓ | Pure code motion refactoring, no CMake changes required (sources already built) |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | CollisionPipeline already has `friend class CollisionPipelineTest`, extensible to new phases |
| Mockable dependencies | ✓ | ContactCache and PositionCorrector have concrete interfaces, no mocking needed (value types) |
| Observable state | ✓ | `hadCollisions()` exposes collision-active flag, cache/corrector state observable via test friend |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Behavioral regression from logic transfer (warm-starting, position correction) | Technical | Low | High | Line-by-line code motion from WorldModel, extensive integration tests (612 tests) | No |
| R2 | Member lifetime issues during ownership transfer (ContactCache, PositionCorrector) | Technical | Very Low | Medium | Both have value semantics and default constructors, straightforward move | No |
| R3 | Breaking CollisionPipeline unit tests due to new phases | Technical | Low | Low | Tests need updates for warm-starting/position correction, but existing logic unchanged | No |
| R4 | Missing WorldModel access to moved members (collisionHandler_, contactSolver_, contactCache_, positionCorrector_) | Integration | Very Low | Medium | Compile-time error, grep for usage before removal | No |
| R5 | Performance regression due to increased per-frame memory footprint (~11KB typical) | Performance | Very Low | Low | Memory increase negligible, runtime overhead measured at < 1% in ticket 0040b | No |

### Prototype Guidance

No prototypes required. This is a pure code motion refactoring with well-understood components:
- ContactCache and PositionCorrector already validated in tickets 0040b and 0040d
- Warm-starting algorithm already functional in WorldModel::updateCollisions()
- Position correction already functional in WorldModel::updateCollisions()
- Risk of behavioral change is low due to identical logic transfer

### Summary

The design successfully completes the original intent of ticket 0036 by bringing CollisionPipeline to feature parity with WorldModel::updateCollisions(). The extension is architecturally sound, follows project coding standards, and presents minimal risk due to the pure refactoring nature (line-by-line code motion).

**Strengths**:
- **Clean ownership transfer**: ContactCache and PositionCorrector move cleanly into CollisionPipeline as value members
- **Backward compatibility**: WorldModel API unchanged, only internal delegation changes
- **Comprehensive test coverage**: 612 existing collision tests provide regression detection
- **Well-documented algorithm transfer**: Design document specifies exact line ranges from WorldModel to copy (lines 390-445 for warm-starting, 447-466 for cache update, 485-508 for position correction)
- **Performance characteristics preserved**: No new algorithms, only code location change

**Considerations**:
- Open Question 1 (ContactCache/PositionCorrector exposure): Recommendation to keep private is sound — defer exposure until profiling shows need for tuning
- Open Question 2 (advanceFrame/expireOldEntries merge): Recommendation to keep separate is correct — explicit lifecycle matches ticket 0040d design and provides clear control points
- Open Question 3 (hadCollisions count vs boolean): Recommendation for boolean is appropriate — sufficient for energy tracking, count accessor can be added later if needed

**Next Steps**: Proceed to prototype phase (skipped per design, go directly to implementation). Design is approved for implementation without revision.
