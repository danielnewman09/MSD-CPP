# Design: Collision Response System

## Summary

Implement elastic collision response for rigid body dynamics using impulse-based physics. When two `AssetInertial` objects collide (detected by existing `CollisionHandler`), the system will compute collision impulses based on coefficient of restitution, apply those impulses to both linear and angular velocities, and separate overlapping objects via position correction. This completes the collision pipeline started in tickets 0027a (EPA) and 0028 (witness points).

## Architecture Changes

### PlantUML Diagram

See: `./0027_collision_response_system.puml`

### New Components

#### CollisionResponse

- **Purpose**: Stateless utility namespace providing collision impulse calculation and position correction for rigid body dynamics
- **Header location**: `msd/msd-sim/src/Physics/CollisionResponse.hpp`
- **Source location**: `msd/msd-sim/src/Physics/CollisionResponse.cpp`
- **Key interfaces**:
  ```cpp
  namespace CollisionResponse {
    /**
     * Compute scalar impulse magnitude for collision resolution.
     *
     * Uses the impulse-based collision response formula:
     *   j = -(1 + e) * (v_rel · n) / denominator
     *
     * Where:
     *   v_rel = relative velocity at contact point
     *   n = contact normal (A → B)
     *   e = combined coefficient of restitution
     *   denominator = (1/m_A + 1/m_B) + angular terms
     *
     * Angular terms account for rotational effects:
     *   denominator += (I_A^-1 * (r_A × n)) × r_A · n
     *                + (I_B^-1 * (r_B × n)) × r_B · n
     *
     * @param assetA First colliding object
     * @param assetB Second colliding object
     * @param result Collision information (normal, contact points)
     * @param combinedRestitution Combined coefficient of restitution [0, 1]
     * @return Scalar impulse magnitude (positive value)
     */
    double computeImpulseMagnitude(
        const AssetInertial& assetA,
        const AssetInertial& assetB,
        const CollisionResult& result,
        double combinedRestitution);

    /**
     * Apply position correction to separate overlapping objects.
     *
     * Uses linear projection with slop tolerance to prevent jitter.
     * Only corrects when penetration exceeds slop threshold.
     *
     * Correction formula:
     *   correction = max(penetrationDepth - kSlop, 0.0) * kCorrectionFactor
     *   separationVector = normal * correction
     *
     * Each object is moved by weighted fraction based on inverse mass:
     *   weight_A = (1/m_A) / (1/m_A + 1/m_B)
     *   weight_B = (1/m_B) / (1/m_A + 1/m_B)
     *
     * @param assetA First colliding object (modified)
     * @param assetB Second colliding object (modified)
     * @param result Collision information (normal, penetration depth)
     */
    void applyPositionCorrection(
        AssetInertial& assetA,
        AssetInertial& assetB,
        const CollisionResult& result);

    /**
     * Combine two coefficients of restitution using geometric mean.
     *
     * Formula: e_combined = sqrt(e_A * e_B)
     *
     * This ensures:
     * - If either object is fully inelastic (e=0), collision is inelastic
     * - If both are fully elastic (e=1), collision is fully elastic
     * - Symmetric: e(A,B) = e(B,A)
     *
     * @param eA Coefficient of restitution for object A [0, 1]
     * @param eB Coefficient of restitution for object B [0, 1]
     * @return Combined coefficient [0, 1]
     */
    double combineRestitution(double eA, double eB);

    // Constants
    constexpr double kSlop = 0.01;              // 1cm slop tolerance [m]
    constexpr double kCorrectionFactor = 0.8;   // Position correction factor [0, 1]
  }
  ```
- **Dependencies**:
  - `AssetInertial` — Collision objects with mass, inertia, state
  - `CollisionResult` — Collision information (normal, penetration, contact points)
  - `Coordinate`, `CoordinateRate`, `AngularRate` — Mathematical primitives
  - `Eigen3` — Matrix/vector operations for inertia tensor calculations
- **Thread safety**: Stateless namespace functions are thread-safe when called with different object instances
- **Error handling**:
  - No exceptions thrown (assumes valid inputs from CollisionHandler)
  - Undefined behavior if objects have zero or infinite mass (caller responsibility)
  - Clamps penetration correction to non-negative values

**Design Rationale**:
- **Namespace over class**: No state needed, pure functions are sufficient
- **Separate impulse/position correction**: Allows independent testing and future customization
- **Geometric mean**: Symmetric, intuitive behavior, standard in physics engines
- **Slop tolerance**: Prevents objects from vibrating at rest due to floating-point jitter
- **Hardcoded constants**: Initial implementation simplicity; can be made configurable in future ticket if needed

### Modified Components

#### AssetInertial

- **Current location**: `msd/msd-sim/src/Physics/RigidBody/AssetInertial.hpp`, `AssetInertial.cpp`
- **Changes required**:
  1. Add private member: `double coefficientOfRestitution_{0.5}`
  2. Add getter: `double getCoefficientOfRestitution() const`
  3. Add setter: `void setCoefficientOfRestitution(double e)` with validation (0.0 ≤ e ≤ 1.0)
  4. Update constructor to accept optional restitution parameter (default 0.5)
- **Backward compatibility**: Fully backward compatible. New constructor parameter is optional with sensible default.

**Implementation details**:
```cpp
class AssetInertial : public AssetPhysical {
public:
  // Existing constructor (unchanged)
  AssetInertial(uint32_t assetId,
                uint32_t instanceId,
                ConvexHull& hull,
                double mass,
                const ReferenceFrame& frame);

  // NEW: Extended constructor with restitution
  AssetInertial(uint32_t assetId,
                uint32_t instanceId,
                ConvexHull& hull,
                double mass,
                const ReferenceFrame& frame,
                double coefficientOfRestitution);

  // NEW: Accessors
  double getCoefficientOfRestitution() const {
    return coefficientOfRestitution_;
  }

  void setCoefficientOfRestitution(double e) {
    if (e < 0.0 || e > 1.0) {
      throw std::invalid_argument("Coefficient of restitution must be in [0, 1]");
    }
    coefficientOfRestitution_ = e;
  }

private:
  // NEW: Member variable
  double coefficientOfRestitution_{0.5};  // Default: 50% elastic
};
```

**Validation rationale**:
- Physical constraint: e ∈ [0, 1] by definition
- e = 0: Fully inelastic (objects stick together)
- e = 1: Fully elastic (perfect bounce)
- e = 0.5: Reasonable default (moderate elasticity)

#### WorldModel

- **Current location**: `msd/msd-sim/src/Environment/WorldModel.hpp`, `WorldModel.cpp`
- **Changes required**:
  1. Add private member: `CollisionHandler collisionHandler_{1e-6}`
  2. Implement `updateCollisions()` method with full collision response logic
  3. Ensure `updatePhysics()` is called AFTER `updateCollisions()` in `update()`
- **Backward compatibility**: No breaking changes. `updateCollisions()` is currently stubbed.

**Implementation details**:

```cpp
class WorldModel {
private:
  void updateCollisions() {
    // Iterate all inertial asset pairs
    const auto& assets = inertialAssets_;
    for (size_t i = 0; i < assets.size(); ++i) {
      for (size_t j = i + 1; j < assets.size(); ++j) {
        AssetInertial& assetA = inertialAssets_[i];
        AssetInertial& assetB = inertialAssets_[j];

        // Check collision
        auto result = collisionHandler_.checkCollision(assetA, assetB);
        if (!result) {
          continue;  // No collision
        }

        // Combine restitution coefficients
        double combinedE = CollisionResponse::combineRestitution(
            assetA.getCoefficientOfRestitution(),
            assetB.getCoefficientOfRestitution());

        // Compute impulse magnitude
        double impulseMagnitude = CollisionResponse::computeImpulseMagnitude(
            assetA, assetB, *result, combinedE);

        // Impulse vector (world space)
        CoordinateRate impulse = result->normal * impulseMagnitude;

        // Apply linear impulse (instantaneous velocity change)
        assetA.getInertialState().velocity += impulse / assetA.getMass();
        assetB.getInertialState().velocity -= impulse / assetB.getMass();

        // Compute lever arms (contact point to center of mass)
        Coordinate leverArmA = result->contactPointA - assetA.getInertialState().position;
        Coordinate leverArmB = result->contactPointB - assetB.getInertialState().position;

        // Apply angular impulse: Δω = I^-1 * (r × J)
        AngularRate angularImpulseA = assetA.getInverseInertiaTensor() *
                                       leverArmA.cross(impulse);
        AngularRate angularImpulseB = assetB.getInverseInertiaTensor() *
                                       leverArmB.cross(-impulse);

        assetA.getInertialState().angularVelocity += angularImpulseA;
        assetB.getInertialState().angularVelocity -= angularImpulseB;

        // Apply position correction to separate objects
        CollisionResponse::applyPositionCorrection(assetA, assetB, *result);
      }
    }
  }

  CollisionHandler collisionHandler_{1e-6};  // NEW: Collision detection
};
```

**Update order**:
1. `updateCollisions()` — Detect collisions, apply impulses, correct positions
2. `updatePhysics(dt)` — Integrate velocities, apply forces, synchronize frames

This order ensures collision response happens before physics integration, allowing the corrected velocities to propagate through the simulation.

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| CollisionResponse | AssetInertial | Read/Write | Reads mass/inertia/state, modifies velocities and position |
| CollisionResponse | CollisionResult | Read | Consumes normal, penetration depth, witness points |
| WorldModel::updateCollisions() | CollisionHandler | Call | Invokes checkCollision() for each asset pair |
| WorldModel::updateCollisions() | CollisionResponse | Call | Stateless function calls for impulse/correction |
| AssetInertial::coefficientOfRestitution_ | WorldModel::updateCollisions() | Read | Used to compute combined restitution |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `msd-sim/test/Physics/AssetInertialTest.cpp` | Constructor tests | Minimal | Add tests for new constructor overload and restitution accessors |
| `msd-sim/test/Environment/WorldModelTest.cpp` | Collision stub tests | Significant | Replace stub validation with actual collision response tests |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| CollisionResponse | combineRestitution_ZeroZero | Fully inelastic (e=0, e=0) → e_combined = 0 |
| CollisionResponse | combineRestitution_OneOne | Fully elastic (e=1, e=1) → e_combined = 1 |
| CollisionResponse | combineRestitution_ZeroOne | Asymmetric (e=0, e=1) → e_combined = 0 |
| CollisionResponse | combineRestitution_Symmetric | e(A,B) = e(B,A) |
| CollisionResponse | computeImpulseMagnitude_HeadOn | Equal mass head-on collision, e=1.0 → swaps velocities |
| CollisionResponse | computeImpulseMagnitude_GlancingCollision | Off-center collision generates angular velocity |
| CollisionResponse | computeImpulseMagnitude_MomentumConservation | Total momentum conserved before/after |
| CollisionResponse | applyPositionCorrection_NoPenetration | penetration < kSlop → no correction |
| CollisionResponse | applyPositionCorrection_DeepPenetration | penetration > kSlop → objects separated |
| CollisionResponse | applyPositionCorrection_MassWeighting | Heavier object moves less than lighter object |
| AssetInertial | getCoefficientOfRestitution_Default | Default construction → e = 0.5 |
| AssetInertial | setCoefficientOfRestitution_Valid | Setting e ∈ [0, 1] succeeds |
| AssetInertial | setCoefficientOfRestitution_Invalid | Setting e < 0 or e > 1 throws |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| WorldModel_CollisionResponse_HeadOn | WorldModel, CollisionHandler, CollisionResponse, AssetInertial | Two objects collide head-on, velocities swap (e=1.0) |
| WorldModel_CollisionResponse_Inelastic | WorldModel, CollisionHandler, CollisionResponse, AssetInertial | Inelastic collision (e=0.0), objects stick together |
| WorldModel_CollisionResponse_GlancingBlow | WorldModel, CollisionHandler, CollisionResponse, AssetInertial | Off-center collision generates spin |
| WorldModel_CollisionResponse_PositionCorrection | WorldModel, CollisionHandler, CollisionResponse, AssetInertial | Overlapping objects are separated |
| WorldModel_CollisionResponse_MultiCollision | WorldModel, CollisionHandler, CollisionResponse, AssetInertial | Multiple simultaneous collisions resolved correctly |

#### Benchmark Tests

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| CollisionResponse | computeImpulseMagnitude_BenchOps | Impulse computation throughput | > 1M ops/sec (trivial calculation) |
| WorldModel | updateCollisions_BenchN | O(n²) collision detection with n objects | < 1ms for n=10, < 10ms for n=50 |
| WorldModel | updateCollisions_BenchWith_EPA | Full collision pipeline (GJK + EPA + response) | < 5ms for n=10 (dominated by EPA) |

**Benchmark rationale**:
- O(n²) pairwise checks acceptable for typical scene sizes (< 100 objects)
- Impulse calculation is trivial (~20 FLOPs), should not be bottleneck
- EPA convergence (~10 iterations) dominates collision cost

## Open Questions

### Design Decisions (Human Input Needed)

1. **Position correction application point**
   - Option A: Correct InertialState.position directly, synchronize ReferenceFrame afterward
     - Pros: Simpler, consistent with physics update flow
     - Cons: Requires synchronization step
   - Option B: Correct ReferenceFrame.origin directly, synchronize InertialState afterward
     - Pros: Matches rendering pipeline expectations
     - Cons: Tighter coupling between collision and rendering
   - Recommendation: **Option A** — InertialState is the source of truth for physics, ReferenceFrame synchronization already happens in updatePhysics()
   **Response** Confirmed Option A (InertialState). Note: This exposes potential design improvement opportunity for InertialState/ReferenceFrame position synchronization (see Future Enhancements).

2. **Inverse inertia tensor world-space transformation**
   - Option A: Transform inverse inertia tensor to world space before angular impulse calculation
     - Formula: `I_world^-1 = R * I_local^-1 * R^T`
     - Pros: Physically correct for oriented objects
     - Cons: Additional matrix multiplications per collision
   - Option B: Use local-space inverse inertia tensor (assumes principal axes align with world)
     - Pros: Simpler, faster
     - Cons: Incorrect for rotated objects
   - Recommendation: **Option A** — Required for correctness with arbitrary orientations
   **Response** Confirmed Option A (world-space). Both approaches have similar computational cost; world-space is more intuitive when everything else is in world-space.

3. **Multiple collision handling within single frame**
   - Option A: Resolve all collisions sequentially in single pass (current design)
     - Pros: Simple, matches most physics engines
     - Cons: Order-dependent results
   - Option B: Iterative collision resolution (multiple passes until convergence)
     - Pros: More stable for simultaneous collisions
     - Cons: Higher computational cost, complexity
   - Recommendation: **Option A** — Sequential resolution sufficient for typical gameplay scenarios; iterative solver can be future enhancement if needed
   **Response** Agreed

### Prototype Required

1. **Angular impulse calculation validation**
   - Uncertainty: Does the current formula `Δω = I^-1 * (r × J)` produce physically plausible angular velocities for off-center collisions?
   - Validation approach: Prototype with analytical test case (sphere colliding with rod at known offset), compare computed angular velocity to hand-calculated expected value
   - Estimated time: 2 hours

2. **Position correction stability**
   - Uncertainty: Will kSlop = 0.01m and kCorrectionFactor = 0.8 prevent jitter without allowing visible overlap?
   - Validation approach: Prototype with two cubes in continuous contact (resting on surface), measure position drift over 1000 frames
   - Estimated time: 2 hours

### Requirements Clarification

1. **Collision pairs to support**
   - Current ticket scope: Dynamic-Dynamic (AssetInertial to AssetInertial)
   - Deferred scope: Dynamic-Static (AssetInertial to AssetEnvironment)
   - Question: Confirm that Dynamic-Static collisions are intentionally out of scope for this ticket
   - Impact: If in scope, needs additional logic for infinite mass case (static objects don't move)
   **Response** yes, keep out of scope for now

2. **Velocity clamping**
   - Question: Should we clamp post-collision velocities to prevent numerical explosion?
   - Context: High-velocity collisions can produce arbitrarily large impulses
   - Proposed: No clamping in initial implementation; add if stability issues arise
   - Impact: Minimal (can be added later without API changes)
   **Response**do not do clamping now

3. **Coefficient of restitution range**
   - Ticket specifies [0.0, 1.0]
   - Question: Allow super-elastic collisions (e > 1.0) for special effects?
   - Proposed: Enforce [0.0, 1.0] constraint per physics definition
   - Impact: Throws exception if e > 1.0 in setCoefficientOfRestitution()
   **Response**yes, enforce that constraint

## Design Complexity Sanity Checks

### Red Flag Assessment

**No red flags detected.** This design:
- Adds **1 new component** (CollisionResponse namespace)
- Modifies **2 existing components** (AssetInertial, WorldModel)
- No combinatorial overloads (single function signatures)
- No optional wrappers for legacy paths
- Backward compatible (new AssetInertial constructor parameter is optional)

The ratio of new-to-modified components is healthy, indicating a clean extension of the existing architecture rather than a retrofit.

## Dependencies

### External Libraries
- Eigen3 — Matrix operations for inertia tensor transformations
- msd-assets — Geometry types (inherited through AssetInertial)

### Internal Dependencies
- `AssetInertial` — Collision objects
- `CollisionHandler` — Collision detection
- `CollisionResult` — Collision information
- `Coordinate`, `CoordinateRate`, `AngularRate` — Mathematical primitives
- `ReferenceFrame` — Coordinate transformations
- `InertialState` — Kinematic state representation

### Build Impact
- New source files: `CollisionResponse.cpp`
- New header files: `CollisionResponse.hpp`
- Modified files: `AssetInertial.hpp`, `AssetInertial.cpp`, `WorldModel.hpp`, `WorldModel.cpp`
- CMakeLists.txt: Add `CollisionResponse.cpp` to `msd_sim` library sources

## Performance Considerations

### Computational Complexity
- **Collision detection**: O(n²) pairwise checks for n inertial objects
  - Acceptable for n < 100 (typical gameplay scenario)
  - Future optimization: Spatial partitioning (broadphase) in separate ticket
- **Impulse calculation**: O(1) per collision (~50 FLOPs)
- **Position correction**: O(1) per collision (~10 FLOPs)

### Memory Impact
- **AssetInertial**: +8 bytes per instance (double for coefficientOfRestitution)
- **WorldModel**: +16 bytes for CollisionHandler instance (epsilon tolerance)
- **Stack usage**: Negligible (no heap allocations during collision response)

### Optimization Opportunities (Future Work)
1. **Broadphase collision detection**: Spatial hashing or BVH to reduce O(n²) → O(n log n)
2. **SIMD vectorization**: Batch impulse calculations for multiple collisions
3. **Multi-threading**: Parallel collision detection (requires thread-safe AssetInertial state access)

## Code Quality Notes

### Adherence to Coding Standards
- **Brace initialization**: `coefficientOfRestitution_{0.5}`
- **NaN for uninitialized floats**: Not applicable (restitution has sensible default)
- **References for non-owning access**: CollisionResponse functions take `const AssetInertial&`
- **Return values over output parameters**: `computeImpulseMagnitude()` returns double
- **Rule of Zero**: AssetInertial uses `= default` for copy/move

### Static Analysis Readiness
- No raw pointers in public interfaces
- All functions have well-defined preconditions (documented)
- No global mutable state
- Exception safety: Basic guarantee (no resource leaks)

### Const-Correctness
- `getCoefficientOfRestitution()` marked const
- CollisionResponse utility functions take const references where appropriate
- Mutable access only where state modification is intended

## Future Enhancements (Out of Scope)

These enhancements are intentionally deferred to future tickets:

1. **Dynamic-Static collision response** (AssetInertial to AssetEnvironment)
   - Requires handling infinite mass case
   - Simpler logic (static object doesn't move)
   - Estimated complexity: Small

2. **Friction** (tangential impulse)
   - Requires contact manifold with tangent vectors
   - Coulomb friction model: j_tangent ≤ μ * j_normal
   - Estimated complexity: Medium

3. **Continuous collision detection** (swept volumes)
   - Prevents tunneling at high velocities
   - Requires time-of-impact calculation
   - Estimated complexity: Large

4. **Contact manifold** (multiple contact points per collision)
   - More stable than single-point contact
   - Requires EPA modification to extract multiple witness points
   - Estimated complexity: Medium

5. **Broadphase optimization** (spatial partitioning)
   - Reduces O(n²) → O(n log n) via BVH or grid
   - Requires scene refactoring
   - Estimated complexity: Large

6. **Configurable collision parameters** (slop, correction factor as WorldModel settings)
   - Simple API extension
   - Estimated complexity: Trivial

7. **InertialState/ReferenceFrame position unification**
   - Evaluate whether position should have single source of truth
   - Current design requires synchronization between InertialState.position and ReferenceFrame.origin
   - Potential improvement: Single position storage with derived accessors
   - Estimated complexity: Medium (touches multiple components)

---

## Acceptance Criteria Mapping

This design addresses all acceptance criteria from ticket 0027:

| AC | Requirement | Design Component |
|----|-------------|------------------|
| AC1 | GJK exposes getSimplex() | Already implemented (ticket 0027a) |
| AC2 | EPA penetration depth within 1e-6 | Already implemented (ticket 0027a) |
| AC3 | AssetInertial has coefficientOfRestitution_ | Modified AssetInertial component |
| AC4 | Equal mass head-on e=1.0 swaps velocities | CollisionResponse::computeImpulseMagnitude() |
| AC5 | Momentum conserved | CollisionResponse impulse formulation |
| AC6 | Glancing collision generates angular velocity | CollisionResponse angular impulse via witness points |
| AC7 | Position correction eliminates overlap | CollisionResponse::applyPositionCorrection() |
| AC8 | updateCollisions() processes all pairs | Modified WorldModel::updateCollisions() |

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-01-24
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | CollisionResponse namespace follows project patterns. Member variable `coefficientOfRestitution_` uses snake_case_ convention. Function names use camelCase (computeImpulseMagnitude, applyPositionCorrection). |
| Namespace organization | ✓ | CollisionResponse is logically placed in `msd-sim/src/Physics/` alongside GJK, EPA, and CollisionHandler. Consistent with existing Physics module organization. |
| File structure | ✓ | Follows `msd/{component}/src/` pattern. CollisionResponse.hpp/.cpp in `msd-sim/src/Physics/`. AssetInertial modifications in `msd-sim/src/Physics/RigidBody/`. |
| Dependency direction | ✓ | Correct layering: CollisionResponse depends on AssetInertial, CollisionResult (same layer). WorldModel depends on CollisionHandler and CollisionResponse (top-down). No cycles detected. |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | No dynamic resources. Stateless namespace functions rely on stack allocation. AssetInertial uses brace initialization for coefficientOfRestitution_. |
| Smart pointer appropriateness | ✓ | No smart pointers needed. CollisionResponse uses plain references for non-owning access (const AssetInertial&). WorldModel stores CollisionHandler by value. Adheres to CLAUDE.md: "Prefer plain references for non-owning access". |
| Value/reference semantics | ✓ | Deliberate and appropriate: CollisionResponse functions take const references for read-only access and mutable references where state modification is needed (applyPositionCorrection). CoordinateRate impulse returned by value via RVO. |
| Rule of 0/3/5 | ✓ | AssetInertial uses Rule of Zero with explicit `= default` (existing pattern from ticket 0026). CollisionHandler uses `= default` for all special member functions. No manual implementations needed. |
| Const correctness | ✓ | getCoefficientOfRestitution() marked const. CollisionResponse utility functions use const references appropriately. Mutable access only where intended (AssetInertial& in applyPositionCorrection). |
| Exception safety | ✓ | Basic guarantee provided. setCoefficientOfRestitution() throws std::invalid_argument for out-of-range values before modifying state. No resource leaks possible (no heap allocations in response logic). |
| Initialization | ✓ | Brace initialization used: `coefficientOfRestitution_{0.5}`. NaN used for uninitialized penetrationDepth in CollisionResult (already established pattern from ticket 0027a). |
| Return values | ✓ | Prefers returning values: combineRestitution() returns double, computeImpulseMagnitude() returns double. No output parameters. Follows CLAUDE.md standard: "Prefer returning values over modifying parameters passed by reference". |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | No circular headers. CollisionResponse.hpp includes AssetInertial.hpp, CollisionResult.hpp, Coordinate.hpp, Eigen3 (all forward-declarable or already available). WorldModel.hpp already includes AssetInertial.hpp and CollisionHandler.hpp. |
| Template complexity | ✓ | No templates involved. Pure function-based design with concrete types. |
| Memory strategy | ✓ | Clear and achievable: Stack-based computation, no heap allocations during collision response. WorldModel stores CollisionHandler by value (16 bytes). AssetInertial adds 8 bytes for coefficientOfRestitution_. |
| Thread safety | ✓ | Not required per NFR. Stateless CollisionResponse functions are thread-safe when called with different instances. Single-threaded simulation assumed. |
| Build integration | ✓ | Straightforward: Add CollisionResponse.cpp to msd_sim library sources in CMakeLists.txt. Modify existing files (AssetInertial, WorldModel). No new external dependencies. |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | CollisionResponse functions are pure and testable in isolation with mock AssetInertial instances and synthetic CollisionResult data. AssetInertial restitution accessors trivially testable. |
| Mockable dependencies | ✓ | Dependencies are concrete classes but can be instantiated with test data. No hidden singletons or global state. CollisionResult is a simple struct. |
| Observable state | ✓ | All effects are observable: computeImpulseMagnitude returns scalar, applyPositionCorrection modifies InertialState (accessible via getInertialState()), restitution accessible via getter. |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | World-space inverse inertia tensor transformation may introduce numerical instability for objects with extreme aspect ratios or high angular velocities | Performance | Low | Medium | Formula is standard physics engine practice. Design uses Eigen3 for numerically stable matrix operations. If issues arise, add validation tests with extreme geometries. | No |
| R2 | Sequential collision resolution (single pass) may produce order-dependent or unstable results when object participates in multiple simultaneous collisions | Technical | Medium | Low | Acknowledged trade-off. Design notes this can be enhanced to iterative solver in future ticket if needed. Sufficient for typical gameplay (< 5 simultaneous collisions per object). | Yes (P2) |
| R3 | Hardcoded constants (kSlop = 0.01m, kCorrectionFactor = 0.8) may require tuning for different simulation scales or timesteps | Maintenance | Medium | Low | Constants chosen based on common physics engine values. Design explicitly notes these can be made configurable in future ticket if needed (see "Future Enhancements" section). | Yes (P2) |
| R4 | Position correction modifies InertialState.position directly, requiring ReferenceFrame synchronization in updatePhysics(). If synchronization is missed, rendering and collision detection desync | Integration | Low | High | Design explicitly addresses this: "Position correction must sync ReferenceFrame after updating InertialState position" (Constraints section). updatePhysics() already performs this sync per ticket 0023 (Force Application System). Pattern is established. | No |

### Prototype Guidance

The design document already proposes two prototypes. I'm refining the specifications to ensure measurable success criteria:

#### Prototype P1: Angular Impulse Validation

**Risk addressed**: R1 (partial - validates correctness, not numerical stability under extremes)
**Question to answer**: Does the formula `Δω = I^-1 * (r × J)` produce physically correct angular velocities for off-center collisions with known analytical solutions?

**Success criteria**:
- Hand-calculated expected angular velocity matches computed value within 1e-6 relative error
- Test case: Uniform-density rod struck perpendicular to its axis at known offset
- Expected torque: τ = r × F (analytically computable)
- Expected angular acceleration: α = I^-1 * τ (analytically computable for rod)
- Verify impulse produces correct Δω after single timestep

**Prototype approach**:
```
Location: prototypes/0027_collision_response_system/p1_angular_impulse/
Type: Catch2 test harness

Steps:
1. Create AssetInertial with rod geometry (analytical inertia tensor)
2. Construct synthetic CollisionResult with known normal, contact point
3. Call computeImpulseMagnitude() with test parameters
4. Apply impulse manually: Δω = I^-1 * (r × J)
5. Compare computed Δω to hand-calculated expected value
6. Test with multiple geometries: sphere, cube, rod
7. Validate momentum conservation: L_before + Δt*τ = L_after
```

**Time box**: 2 hours

**If prototype fails**:
- Investigate formula implementation for sign errors or coordinate system issues
- Verify inverse inertia tensor transformation from local to world space
- Check cross product order (r × J vs J × r)

#### Prototype P2: Position Correction Stability

**Risk addressed**: R2, R3
**Question to answer**: Will kSlop = 0.01m and kCorrectionFactor = 0.8 prevent jitter in resting contact scenarios without allowing visible overlap?

**Success criteria**:
- Two cubes resting in contact (gravity + ground) for 1000 frames @ 60 FPS
- Position drift < 0.001m over entire simulation (no visible separation)
- No oscillation: position change between consecutive frames < 1e-6 after settling (frame 100+)
- No penetration: overlap depth < kSlop throughout simulation
- Settling time: < 100 frames to reach stable rest

**Prototype approach**:
```
Location: prototypes/0027_collision_response_system/p2_position_correction/
Type: Standalone executable with frame-by-frame logging

Steps:
1. Create WorldModel with two AssetInertial cubes (1kg each)
2. Position cubes with initial small overlap (0.005m penetration)
3. Run simulation loop for 1000 frames (dt = 16.67ms)
4. Log per frame: penetrationDepth, position delta, velocity magnitude
5. Analyze logs for jitter patterns (oscillation detection)
6. Measure maximum position drift and settling time
7. Visualize position over time (gnuplot or CSV export)
```

**Time box**: 2 hours

**If prototype fails**:
- Try alternative constants: kSlop ∈ [0.001, 0.1], kCorrectionFactor ∈ [0.2, 1.0]
- Investigate Baumgarte stabilization as alternative to linear projection
- Consider splitting position correction across multiple sub-steps

### Summary

This design demonstrates excellent alignment with project coding standards and architectural patterns. The collision response system is well-structured as a stateless utility namespace, properly integrates with the existing collision detection pipeline (GJK/EPA), and addresses all acceptance criteria from the ticket.

**Strengths**:
- Follows established patterns from recent tickets (0023 Force Application, 0026 Mirtich Inertia, 0028 Witness Points)
- Clean separation of concerns: CollisionResponse is pure functions, WorldModel orchestrates
- Proper use of const references, value returns, and RAII
- Explicit handling of all open questions from design phase with human confirmation
- Comprehensive test plan covering unit, integration, and benchmark tests
- Realistic acknowledgment of limitations (sequential vs iterative resolution, hardcoded constants)

**Minor notes for implementation**:
1. **Validation of inverse inertia tensor transformation**: While the formula `I_world^-1 = R * I_local^-1 * R^T` is standard, ensure rotation matrix `R` is extracted correctly from AssetInertial's ReferenceFrame (existing getter: `getRotation()`)
2. **Lever arm calculation**: The design shows `leverArmA = contactPointA - centerOfMass`. Verify that AssetInertial's center of mass is stored in world-space coordinates (it should be in local coordinates per CLAUDE.md Physics module, requiring transformation)
3. **Update order dependency**: The design specifies updateCollisions() before updatePhysics(). Ensure this order is enforced in WorldModel::update() implementation to prevent race between collision response and physics integration

**Recommended next steps**:
1. Execute Prototype P1 (Angular Impulse Validation) to verify torque formula correctness - 2 hours
2. Execute Prototype P2 (Position Correction Stability) to validate hardcoded constants - 2 hours
3. Upon successful prototype validation, proceed to implementation phase
4. Total estimated prototype time: **4 hours**

The design is **APPROVED WITH NOTES** for prototype phase. The notes above are implementation guidance, not blocking issues requiring design revision.
