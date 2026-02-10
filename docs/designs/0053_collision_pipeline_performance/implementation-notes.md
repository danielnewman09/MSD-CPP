# Implementation Notes — 0053_collision_pipeline_performance

**Ticket**: [0053_collision_pipeline_performance](../../../tickets/0053_collision_pipeline_performance.md)
**Branch**: `0053-collision-pipeline-performance`
**Design**: [design.md](design.md)
**Status**: Partial — 2 of 5 subtasks complete

---

## Summary

Implemented workspace infrastructure (0053a) and verified friction warm-start already exists (0053c). Deferred SAT gating (0053d) due to test regressions. Fixed-size matrices (0053e) and full API refactor remain as future work.

---

## Subtask Status

| ID | Description | Status | Notes |
|----|-------------|--------|-------|
| 0053a | Reduce heap allocations | **Partial** | Workspace struct added, API refactor deferred |
| 0053b | Disable Qhull diagnostics | ✅ **Complete** | Suppresses diagnostic output |
| 0053c | Friction solver warm-start | ✅ **Complete** | Already implemented in 0052d |
| 0053d | SAT fallback gating | ❌ **Deferred** | Threshold caused test regressions |
| 0053e | Fixed-size Eigen matrices | ⏸ **Not Started** | Awaiting profiling validation |

---

## Implementation Details

### 0053a: Reduce Heap Allocations (Partial)

**Changes**:
- Added `SolverWorkspace` struct to `CollisionPipeline` with pre-allocated vectors
- Workspace resized per-frame based on contact count
- Eigen's `resize()` only reallocates when capacity exceeded

**Files Modified**:
- `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp`: Added SolverWorkspace struct (38 lines)
- `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp`: Added resize call (5 lines)

**What Was Implemented**:
- Infrastructure for workspace memory reuse
- Automatic capacity management via Eigen's internal behavior

**What Was Deferred**:
- Refactoring solver APIs to accept `Eigen::Ref` parameters
- Passing workspace vectors to solvers
- Eliminating internal solver allocations

**Rationale for Deferral**:
The full API refactor requires modifying signatures of `ConstraintSolver::solve()`, `FrictionConeSolver::solve()`, and `PositionCorrector::correctPositions()`, plus all their helper methods. This is a significant change that should be validated with profiling to confirm it provides meaningful benefit over the simpler workspace capacity approach.

**Next Steps**:
1. Profile with current implementation to measure allocation overhead
2. If allocations remain significant, implement full `Eigen::Ref` API refactor
3. Measure allocation reduction

---

### 0053b: Disable Qhull Diagnostic Output (Complete)

**Changes**:
- Redirect Qhull stdout/stderr to `/dev/null` during hull computation

**Files Modified**:
- `msd/msd-sim/src/Physics/RigidBody/ConvexHull.hpp`: Added file redirection (6 lines)

**Impact**:
- Eliminates "Convex hull of N points in 3-d:" console spam
- Expected ~0.3% CPU reduction (per design profiling data)

---

### 0053c: Friction Solver Warm-Start (Already Complete)

**Status**: Implemented in ticket 0052d (commit 4076b1e)

**Existing Implementation**:
- `ContactCache` stores full lambda vectors (3 components per contact when friction present)
- `CollisionPipeline::solveConstraintsWithWarmStart()` retrieves cached lambdas (lines 346-372)
- `ConstraintSolver::solve()` accepts `initialLambda` parameter and passes to friction solver
- `FrictionConeSolver::solve()` uses `lambda0` as warm-start (line 49)

**No Additional Work Needed**.

---

### 0053d: SAT Fallback Gating (Deferred — Test Regression)

**Attempted Implementation**:
- Added threshold check: only run SAT when EPA depth < 0.01m
- **Result**: 656/661 tests passing (regression: D1, H1 resting contact stability)

**Root Cause**:
EPA can produce incorrect results at penetration depths > 0.01m, not just near-zero as design assumed. Gating SAT based on EPA depth breaks cases where EPA depth is moderate but EPA picked the wrong face.

**Design Risk Validation**:
Risk R2 in design review identified this as "medium likelihood, medium impact" and recommended threshold sweep prototype. Without penetration depth profiling data, cannot safely determine threshold.

**Required Before Retry**:
1. Add penetration depth logging to `CollisionHandler`
2. Profile resting contact tests (D1, D4, H1) to measure EPA depth distribution
3. Identify threshold that passes all tests (may be lower than 0.01m or require different gating strategy)

**Reverted**: Changes rolled back, SAT runs unconditionally (baseline behavior).

**Alternative Approach**:
Instead of gating based on EPA depth, consider:
- Caching SAT normals per hull transform
- Pre-computing face normals in world space
- Using BVH to reduce face normal iteration count

---

### 0053e: Eigen Fixed-Size Matrix Optimization (Not Started)

**Status**: Awaiting validation of prior optimizations via profiling

**Planned Changes**:
- Replace `Eigen::VectorXd` and `Eigen::MatrixXd` with compile-time max bounds
- Use `Eigen::Matrix<double, Dynamic, Dynamic, 0, kMaxContactsPerPair, kMaxContactsPerPair>`
- Target: Stack allocation for ≤4 contacts, heap fallback for larger

**Deferral Rationale**:
1. Significant code churn across ConstraintSolver, FrictionConeSolver, PositionCorrector
2. Requires profiling to validate stack vs heap allocation benefit
3. Should implement after 0053a full refactor (if pursued) to avoid double refactor

**Next Steps**:
1. Profile current implementation
2. If Eigen allocation overhead remains significant, implement fixed-size types
3. Validate numeric precision (design risk R3)

---

## Test Impact

### Baseline
- 657/661 tests passing
- 4 pre-existing failures: D4, H3, B2, B3, B5

### After 0053a + 0053b
- 657/661 tests passing
- No regressions
- All currently-passing physics tests still pass

### After 0053d Attempt (Reverted)
- 656/661 tests passing (-1 regression)
- New failures: D1 (resting cube drift), H1 (restitution disable)
- Reverted to restore baseline

---

## Design Adherence

| Design Element | Implementation Status | Notes |
|----------------|----------------------|-------|
| SolverWorkspace struct | ✅ Implemented | Member of CollisionPipeline |
| Workspace resize() | ✅ Implemented | Called per-frame based on contact count |
| Solver API refactor (Eigen::Ref) | ⏸ Deferred | Awaiting profiling validation |
| Qhull diagnostic suppression | ✅ Implemented | /dev/null redirection |
| Friction warm-start | ✅ Already exists | Part of 0052d |
| SAT gating threshold | ❌ Deferred | Test regression risk |
| Fixed-size Eigen matrices | ⏸ Deferred | Awaiting profiling validation |

**Overall Adherence**: Partial. Core infrastructure implemented, but full optimization deferred pending profiling data.

---

## Performance Characteristics

### Expected Impact (Per Design)
- **0053a**: ~1.8% CPU reduction (allocation overhead)
- **0053b**: ~0.3% CPU reduction (Qhull diagnostics)
- **0053c**: ~1.9% CPU reduction (friction warm-start) — **Already achieved in 0052d**
- **0053d**: ~1.1% CPU reduction (SAT gating) — **Deferred**
- **0053e**: ~1.7% CPU reduction (fixed-size matrices) — **Deferred**

### Actual Impact (Measured)
- **Not yet profiled**. Requires re-running Time Profiler with 50× test repetition to measure aggregate pipeline CPU %.

---

## Known Limitations

1. **Workspace not used by solvers**: Solvers still allocate internally. Workspace provides capacity management only.
2. **SAT runs unconditionally**: No gating optimization due to test regression risk.
3. **Dynamic Eigen matrices**: No fixed-size specialization for typical contact counts.
4. **No allocation count logging**: Cannot verify reallocation rate without instrumentation.

---

## Future Work

### High Priority
1. **Profile current implementation**: Measure actual CPU % for pipeline components
2. **SAT penetration depth study**: Log EPA depths, determine safe gating threshold
3. **Validate workspace benefit**: Compare allocation overhead before/after

### Medium Priority
1. **Full solver API refactor**: If profiling shows significant allocation overhead, implement `Eigen::Ref` passing
2. **Fixed-size matrix optimization**: If Eigen overhead remains after API refactor

### Low Priority
1. **Allocation count instrumentation**: Add logging to measure workspace reallocation rate
2. **Alternative SAT optimization**: Cache normals or use BVH instead of depth-based gating

---

## Recommendations

1. **Merge current state**: 0053a + 0053b provide infrastructure and diagnostics fix with zero risk
2. **Profile before proceeding**: Measure actual impact of workspace capacity management
3. **Investigate D1/H1 EPA depths**: Understand penetration depth distribution before retrying 0053d
4. **Consider alternative SAT optimization**: Caching may be safer than gating

---

## Deliverables

### Code Artifacts
- `CollisionPipeline.hpp`: SolverWorkspace struct (commit 69ca8b8)
- `CollisionPipeline.cpp`: Workspace resize call (commit 69ca8b8)
- `ConvexHull.hpp`: Qhull diagnostic suppression (commit 5929a44)

### Documentation Artifacts
- `iteration-log.md`: Full iteration history with hypotheses and results
- `implementation-notes.md`: This document

### Metrics
- Test pass rate: 657/661 (maintained baseline)
- No performance metrics yet (profiling deferred)

---

## Lessons Learned

1. **Prototype validation critical**: Design assumption (EPA reliable > 0.01m) was incorrect, causing test regression
2. **Existing implementations matter**: 0053c already existed from 0052d, saving implementation time
3. **Incremental approach works**: Staged implementation (0053a infrastructure first) allowed safe progress
4. **Test regression signals risk**: D1/H1 failures immediately flagged unsafe threshold assumption

---

## Handoff Notes

- **Current state**: Workspace infrastructure in place, Qhull diagnostics suppressed, friction warm-start confirmed
- **Next implementer**: Profile to measure allocation overhead, then decide on full API refactor vs alternative approaches
- **Watch for**: SAT gating requires careful threshold analysis with penetration depth profiling data
- **Merge recommendation**: Safe to merge 0053a + 0053b without risk to test suite
