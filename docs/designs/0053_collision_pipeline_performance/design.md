# Design: Collision Pipeline Performance Optimization

## Summary

This design addresses five targeted optimizations to the collision and friction constraint pipeline, reducing aggregate CPU cost from 6.8% to approximately 3.6% (47% relative reduction). The optimizations focus on memory allocation overhead, diagnostic output, solver iteration efficiency, conditional logic gating, and fixed-size matrix specializations. All changes are algorithmic or data structure improvements with no changes to physics behavior.

**Type**: Performance Optimization (Investigation-based)
**Ticket**: [0053_collision_pipeline_performance](../../../tickets/0053_collision_pipeline_performance.md)
**Investigation Report**: [investigation-report.md](../../investigations/0053_collision_pipeline_performance/investigation-report.md)

---

## Architecture Changes

### Overview

This design targets five profiling-identified hotspots with no architectural changes to component relationships. All work is internal to existing classes:

| Subtask | Target Component(s) | Change Type | CPU Impact |
|---------|-------------------|-------------|------------|
| 0053a | `CollisionPipeline`, `ConstraintSolver`, `FrictionConeSolver`, `PositionCorrector` | Workspace allocation | 1.8% |
| 0053b | `ConvexHull` | Qhull configuration | 0.3% |
| 0053c | `FrictionConeSolver` | Iteration optimization | 1.9% |
| 0053d | `CollisionHandler` | Conditional gating | 1.1% |
| 0053e | `ConstraintSolver`, `FrictionConeSolver` | Fixed-size matrices | 1.7% |

**No new components.** All changes are modifications to existing classes.

---

## Subtask 0053a: Reduce Heap Allocations (Priority 1)

### Problem Statement

Profiling shows 1.8% CPU time in memory allocator functions (`_xzm_free`, `_xzm_xzone_malloc_tiny`). Dynamic allocations in per-frame solver loop create malloc/free overhead.

**Root causes**:
1. `ConstraintSolver::solve()` creates `Eigen::VectorXd`/`Eigen::MatrixXd` each call
2. `FrictionConeSolver::solve()` allocates workspace vectors each call
3. `PositionCorrector::correctPositions()` allocates per-iteration vectors
4. `CollisionPipeline::execute()` builds constraint lists each frame

### Design

#### Workspace Struct

Introduce a `SolverWorkspace` struct owned by `CollisionPipeline`:

```cpp
struct SolverWorkspace {
  // Pre-allocated vectors for ConstraintSolver
  Eigen::VectorXd lambda;           // Constraint multipliers
  Eigen::VectorXd rhs;              // Right-hand side (b vector)
  Eigen::VectorXd warmStart;        // Initial lambda guess

  // Pre-allocated for FrictionConeSolver
  Eigen::VectorXd frictionLambda;   // 3D friction force per contact
  Eigen::VectorXd residual;         // Residual vector
  Eigen::VectorXd gradient;         // Merit function gradient
  Eigen::VectorXd trialLambda;      // Line search trial vector

  // Pre-allocated for PositionCorrector
  Eigen::VectorXd pseudoVelocities; // Position correction velocities
  Eigen::VectorXd penetrations;     // Penetration depths

  // Contact constraint list (reused each frame)
  std::vector<ContactConstraint*> contactConstraints;
  std::vector<FrictionConstraint*> frictionConstraints;

  // Resize workspace for current contact count
  void resize(size_t numContacts) {
    lambda.resize(numContacts);
    rhs.resize(numContacts);
    warmStart.resize(numContacts);
    frictionLambda.resize(3 * numContacts);
    residual.resize(3 * numContacts);
    gradient.resize(3 * numContacts);
    trialLambda.resize(3 * numContacts);
    pseudoVelocities.resize(numContacts);
    penetrations.resize(numContacts);
    contactConstraints.clear();
    frictionConstraints.clear();
  }
};
```

**Note**: `resize()` only reallocates if capacity is insufficient (Eigen's dynamic matrix behavior). For typical contact counts (1-8), initial allocation handles most frames without reallocation.

#### CollisionPipeline Integration

```cpp
class CollisionPipeline {
public:
  void execute(const std::vector<AssetInertial>& inertial,
               const std::vector<AssetEnvironment>& environmental,
               double dt);

private:
  SolverWorkspace workspace_;  // Reused each frame
  ConstraintSolver constraintSolver_;
  FrictionConeSolver frictionSolver_;
  PositionCorrector positionCorrector_;
  ContactCache contactCache_;
  CollisionHandler collisionHandler_;
};

void CollisionPipeline::execute(...) {
  // Phase 1: Detect collisions
  std::vector<CollisionResult> collisions = detectCollisions(...);

  // Phase 2: Resize workspace for contact count
  workspace_.resize(collisions.size());

  // Phase 3: Create constraints (no allocation, write to workspace vectors)
  for (const auto& collision : collisions) {
    workspace_.contactConstraints.push_back(
      ContactConstraintFactory::createFromCollision(collision, ...));
  }

  // Phase 4: Solve (pass workspace by reference)
  constraintSolver_.solve(workspace_.contactConstraints,
                          workspace_.lambda,
                          workspace_.rhs,
                          workspace_.warmStart);

  // Phase 5: Friction solve (pass workspace by reference)
  frictionSolver_.solve(workspace_.frictionConstraints,
                        workspace_.frictionLambda,
                        workspace_.residual,
                        workspace_.gradient);

  // Phase 6: Position correction (pass workspace by reference)
  positionCorrector_.correctPositions(workspace_.contactConstraints,
                                     workspace_.pseudoVelocities,
                                     workspace_.penetrations);
}
```

#### Modified Components

**ConstraintSolver**:
```cpp
class ConstraintSolver {
public:
  // Old API (creates vectors internally)
  void solve(const std::vector<ContactConstraint*>& constraints,
             const std::vector<AssetInertial*>& bodies);

  // New API (uses pre-allocated workspace)
  void solve(const std::vector<ContactConstraint*>& constraints,
             const std::vector<AssetInertial*>& bodies,
             Eigen::Ref<Eigen::VectorXd> lambda,
             Eigen::Ref<Eigen::VectorXd> rhs,
             Eigen::Ref<Eigen::VectorXd> warmStart);
};
```

**Note**: `Eigen::Ref` allows passing preallocated vectors without copying.

**FrictionConeSolver**: Similar signature changes to accept workspace vectors.

**PositionCorrector**: Similar signature changes to accept workspace vectors.

### Success Criteria

- Memory allocation samples reduced by ≥ 50% (90 → 45 samples)
- No physics test regressions (all 689/693 passing tests still pass)
- Workspace reallocation count logged (should be near-zero after first frame)

### Backward Compatibility

**Breaking change**: Solver APIs change from internal allocation to external workspace.

**Impact**: Only `CollisionPipeline` calls these solvers; no external API surface.

---

## Subtask 0053b: Disable Qhull Diagnostic Output (Priority 0 — Quick Win)

### Problem Statement

`qh_printsummary` consumes 0.3% of samples on diagnostic I/O that provides no value in production builds.

### Design

#### ConvexHull Modification

In `ConvexHull::computeHull()`, disable Qhull diagnostic output:

```cpp
void ConvexHull::computeHull(const std::vector<Coordinate>& points) {
  // Existing setup...
  qh_init_A(qh, nullptr, stdout, stderr, 0, nullptr);

  // NEW: Disable diagnostic output
  qh->NOsummary = True;

  // Existing computation...
  qh_qhull(qh, ...);
  // ...
}
```

**Alternative (if NOsummary insufficient)**: Redirect stdout/stderr to `/dev/null` for the Qhull call:

```cpp
// Redirect stdout during Qhull call
FILE* oldStdout = stdout;
stdout = fopen("/dev/null", "w");
qh_qhull(qh, ...);
fclose(stdout);
stdout = oldStdout;
```

### Success Criteria

- `qh_printsummary` removed from profiling hotspots
- No functional changes to hull computation
- All ConvexHull tests pass unchanged

### Backward Compatibility

None — internal implementation detail.

---

## Subtask 0053c: FrictionConeSolver Iteration Optimization (Priority 2)

### Problem Statement

`FrictionConeSolver::solve()` is the single biggest hotspot (1.9%). Newton solver iterates to convergence, with costs from:
1. Merit function evaluation (builds full residual vector each iteration)
2. Line search step count (Armijo backtracking)
3. Cone projection per line search step
4. Cold-start from zero initial lambda each frame

### Design

#### Warm-Start from ContactCache

Extend `ContactCache` to store friction lambda values:

```cpp
struct ContactCacheEntry {
  uint32_t bodyAId;
  uint32_t bodyBId;
  Eigen::Vector3d normalLambda;      // Existing: normal constraint force
  Eigen::Vector3d frictionLambda;    // NEW: tangential friction forces (3D)
  std::chrono::steady_clock::time_point lastSeen;
};
```

**Integration**:
```cpp
// Before friction solve
Eigen::Vector3d initialLambda = contactCache_.getFrictionLambda(bodyA, bodyB);
frictionSolver_.solve(frictionConstraint, initialLambda, result);

// After friction solve
contactCache_.storeFrictionLambda(bodyA, bodyB, result);
```

**Expected benefit**: Typical convergence from 5-8 iterations to 2-4 iterations (similar to normal constraint warm-starting, which shows 10-25× speedup).

#### Early Termination

Add physics-meaningful tolerance to avoid over-converging:

```cpp
constexpr double kResidualTolerance = 1e-4;  // 0.1mm penetration equivalent

if (residualNorm < kResidualTolerance) {
  return lambda;  // "Good enough" for physics accuracy
}
```

**Rationale**: Current solver iterates to `1e-6` tolerance, which is far below physics-meaningful precision (collision detection is ~1e-3 accurate).

#### Cached Cone Projection

Cone projection is pure function of lambda magnitude and cone angle:

```cpp
// Before line search
Eigen::Vector3d projectionCache = ConeProjection::projectVector(lambda, mu);

// During line search (if lambda direction unchanged)
if (directionUnchanged) {
  trialProjection = projectionCache * alpha;  // Scale cached result
}
```

**Limitation**: Only applies if Newton direction is unchanged. Benefit likely minimal, defer to prototyping.

### Success Criteria

- Friction solver samples reduced by ≥ 25% (93 → 70 samples)
- Average iteration count ≤ 5 (down from 5-8)
- No physics test regressions (especially friction tests)
- Warm-start hit rate logged (expect > 80% for persistent contacts)

### Backward Compatibility

None — internal solver implementation.

---

## Subtask 0053d: SAT Fallback Cost Reduction (Priority 3)

### Problem Statement

`computeSATMinPenetration` (1.1% CPU) runs on every collision pair, even when EPA produces high-quality results. Added in ticket 0047 for resting contact stability (fixes D1, D4, H1 test failures).

**Observation**: SAT is only needed when EPA is unreliable — near-zero penetration where origin lies on Minkowski boundary.

### Design

#### Conditional Gating

Gate SAT execution on EPA penetration depth:

```cpp
CollisionResult CollisionHandler::computeContactInfo(
    const GJKResult& gjkResult,
    const ConvexHull& hullA, const ReferenceFrame& frameA,
    const ConvexHull& hullB, const ReferenceFrame& frameB) {

  // Run EPA first (always)
  CollisionResult epaResult = epa_.computeContactInfo(gjkResult, ...);

  // Only run SAT fallback if EPA penetration is suspiciously low
  constexpr double kEPAUncertaintyThreshold = 0.01;  // 1cm

  if (epaResult.penetrationDepth < kEPAUncertaintyThreshold) {
    // EPA may be unreliable — validate with SAT
    double satDepth = computeSATMinPenetration(...);

    // If EPA depth is wildly inconsistent (>10× mismatch), use SAT
    if (epaResult.penetrationDepth > 10.0 * satDepth) {
      return buildSATContact(...);  // Fallback to SAT result
    }
  }

  return epaResult;  // EPA result is trustworthy
}
```

**Expected benefit**: Eliminate SAT computation for ~90% of collision pairs (typical penetration depths are 0.1-1.0cm during active collisions, far above 0.01cm threshold).

#### Alternative: Cache SAT Normals

If gating is insufficient, precompute face normals in world space and cache:

```cpp
struct ConvexHull {
  mutable std::vector<Coordinate> cachedWorldNormals_;
  mutable ReferenceFrame cachedTransform_;

  const std::vector<Coordinate>& getWorldNormals(const ReferenceFrame& frame) const {
    if (frame != cachedTransform_) {
      cachedWorldNormals_ = transformNormals(frame);
      cachedTransform_ = frame;
    }
    return cachedWorldNormals_;
  }
};
```

**Defer to implementation phase**: Only if conditional gating is insufficient.

### Success Criteria

- SAT fallback samples reduced by ≥ 50% (53 → 26 samples)
- No regressions in resting contact stability tests (D1, D4, H1)
- SAT invocation rate logged (expect < 10% of collision pairs)

### Backward Compatibility

None — internal collision detection implementation.

---

## Subtask 0053e: Eigen Fixed-Size Matrix Optimization (Priority 4)

### Problem Statement

Dynamic Eigen matrices (1.7% CPU) cause heap allocation and indirection overhead. Contact count is typically 1-4 per collision pair, enabling fixed-size specialization.

### Design

#### Fixed-Size Matrix Types

Define fixed-size matrix aliases for typical contact counts:

```cpp
// Maximum contact count per collision pair
constexpr size_t kMaxContactsPerPair = 4;
constexpr size_t kMaxDOFPerBody = 12;  // 6 DOF per body × 2 bodies

// Fixed-size types with dynamic fallback
using EffectiveMassMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                          0, kMaxContactsPerPair, kMaxContactsPerPair>;

using ContactJacobian = Eigen::Matrix<double, 1, kMaxDOFPerBody>;

using LambdaVector = Eigen::Matrix<double, Eigen::Dynamic, 1,
                                   0, kMaxContactsPerPair, 1>;
```

**Behavior**: Stack-allocated for ≤ 4 contacts, falls back to heap for > 4 contacts.

#### ConstraintSolver Changes

Replace `Eigen::VectorXd` and `Eigen::MatrixXd` with fixed-size types:

```cpp
class ConstraintSolver {
private:
  // OLD: Always heap-allocated
  Eigen::MatrixXd assembleEffectiveMass(const std::vector<ContactConstraint*>& constraints);
  Eigen::VectorXd computeLambda(const Eigen::MatrixXd& M, const Eigen::VectorXd& rhs);

  // NEW: Stack-allocated for typical sizes
  EffectiveMassMatrix assembleEffectiveMass(...);
  LambdaVector computeLambda(const EffectiveMassMatrix& M, const LambdaVector& rhs);
};
```

#### FrictionConeSolver Changes

Friction lambda is 3D per contact:

```cpp
using FrictionLambdaVector = Eigen::Matrix<double, Eigen::Dynamic, 1,
                                           0, 3 * kMaxContactsPerPair, 1>;
```

### Success Criteria

- Eigen samples reduced by ≥ 30% (84 → 59 samples)
- Memory allocation samples further reduced (synergy with 0053a)
- No numeric precision regressions in solver tests
- Heap allocation only for > 4 contacts per pair (rare)

### Backward Compatibility

None — internal solver implementation.

---

## Test Impact

### Existing Tests Affected

**No test modifications required.** All optimizations preserve physics behavior:

| Test Suite | Relevance | Expected Impact |
|------------|-----------|-----------------|
| `CollisionTest` | Collision detection | Zero (no GJK/EPA changes) |
| `ConstraintTest` | Constraint solving | Zero (solver output unchanged) |
| `FrictionTest` | Friction behavior | Zero (solver output unchanged) |
| `ContactTest` | Contact manifolds | Zero (manifold generation unchanged) |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| `SolverWorkspace` | `WorkspaceResizeDoesNotReallocate` | Resize only reallocates when capacity exceeded |
| `ConvexHull` | `QhullDiagnosticsSuppressed` | No stdout output during hull computation |
| `ContactCache` | `FrictionLambdaWarmStart` | Friction lambda stored and retrieved correctly |
| `CollisionHandler` | `SATGatingLogsInvocationRate` | SAT only runs when EPA depth < threshold |
| `ConstraintSolver` | `FixedSizeMatricesStackAllocated` | No heap allocation for ≤ 4 contacts |

#### Benchmark Tests

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| `CollisionPipeline` | `BM_CollisionPipeline_PersistentContact_WarmStart` | Warm-start speedup | 2-4× faster vs cold-start |
| `FrictionConeSolver` | `BM_FrictionSolver_IterationCount` | Average iterations per solve | ≤ 5 iterations |
| `ConstraintSolver` | `BM_ConstraintSolver_FixedSizeAllocation` | Allocation count for ≤ 4 contacts | Zero heap allocations |

---

## Open Questions

### Design Decisions (Human Input Needed)

**None.** All optimizations are straightforward improvements with no trade-offs.

### Prototype Required

1. **Friction Solver Warm-Start Convergence**
   - **Uncertainty**: Will warm-starting reduce iteration count by expected 2×?
   - **Validation**: Prototype warm-start in `FrictionConeSolver`, measure iteration count before/after
   - **Risk**: Low — similar warm-start already works for normal constraints

2. **SAT Gating Threshold**
   - **Uncertainty**: Is 0.01m the right threshold to trigger SAT fallback?
   - **Validation**: Profile with various thresholds (0.001m, 0.01m, 0.1m), measure SAT invocation rate vs physics test pass rate
   - **Risk**: Medium — wrong threshold could break resting contact stability (D1, D4, H1 tests)

3. **Fixed-Size Matrix Numeric Stability**
   - **Uncertainty**: Does fixed-size Eigen specialization maintain solver precision?
   - **Validation**: Run full physics test suite with fixed-size matrices, compare solver output to baseline
   - **Risk**: Low — Eigen documentation guarantees equivalent precision

### Requirements Clarification

**None.** Investigation report provides complete requirements.

---

## Implementation Order

As recommended in investigation report:

1. **0053b** (Quick win): Disable Qhull output → 0.3% reduction, trivial implementation
2. **0053a** (Foundation): Workspace allocation → 1.8% reduction, enables subsequent optimizations
3. **0053c** (High impact): Friction solver → 1.9% reduction, builds on workspace infrastructure
4. **0053d** (Independent): SAT gating → 1.1% reduction, orthogonal to other work
5. **0053e** (Final cleanup): Fixed-size matrices → 1.7% reduction, synergizes with 0053a

**Total expected reduction**: ~6.8% → ~3.6% (47% relative reduction in pipeline cost)

---

## Performance Metrics

### Success Metrics

After all optimizations:
- **Aggregate pipeline CPU**: 6.8% → 3.6% (≥ 3.2% absolute reduction)
- **Memory allocation samples**: 90 → 45 (≥ 50% reduction)
- **Friction solver iterations**: 5-8 → 2-4 (≥ 40% reduction)
- **SAT invocation rate**: 100% → < 10% of collision pairs
- **Physics test pass rate**: 689/693 maintained (zero regressions)

### Measurement Strategy

Each optimization measured independently via profiling:
1. Apply optimization
2. Run profiling with collision/friction test filter
3. Compare samples before/after
4. Document reduction in design iteration log
5. Merge only if success criteria met

---

## References

- **Investigation Report**: [`investigation-report.md`](../../investigations/0053_collision_pipeline_performance/investigation-report.md)
- **Collision System**: [`msd-sim/src/Physics/Collision/CLAUDE.md`](../../../msd/msd-sim/src/Physics/Collision/CLAUDE.md)
- **Constraint System**: [`msd-sim/src/Physics/Constraints/CLAUDE.md`](../../../msd/msd-sim/src/Physics/Constraints/CLAUDE.md)
- **Eigen Documentation**: https://eigen.tuxfamily.org/dox/group__TopicFixedSizeVectorizable.html
- **Profiling Guide**: [`docs/profiling.md`](../../profiling.md)
