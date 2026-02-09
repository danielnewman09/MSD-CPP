# Design: Restitution-Gravity Coupling Fix

## Summary

Decouple restitution from gravity in collision response by introducing a velocity-bias parameter to the constraint solver RHS. This prevents the extra `e * J * g * dt` term that currently causes spurious rotation (B3 regression) and modifies energy injection patterns (H3 regression) introduced by gravity pre-apply in ticket 0047.

## Problem Statement

Ticket 0047 introduced gravity pre-apply (applying `g*dt` to velocities before collision solving), which is the standard Box2D/Bullet approach for resting contact stability. However, this couples restitution with gravity in the solver RHS:

```
b = -(1+e) * J * (v + g*dt)
  = -(1+e) * J * v  -  (1+e) * J * g*dt
                       ^^^^^^^^^^^^^^^
                       Unwanted coupling
```

The extra `e * J * g * dt` term (~0.08 m/s for e=0.5, g=9.81, dt=0.016) causes:

1. **B3 regression**: Sphere acquires spurious rotation (1.48 rad/s vs 0.5 threshold) because off-center EPA contact point amplifies the restitution-gravity coupling into angular velocity
2. **H3 regression**: ERP energy injection pattern disappears at all timesteps (test expects specific ERP-driven energy pattern, but gravity pre-apply changes the energy dynamics)

## Architecture Changes

### PlantUML Diagram
See: `./0051_restitution_gravity_coupling.puml`

### New Components

#### SpatialVector

- **Purpose**: Represent 6-DOF spatial velocity bias (linear + angular)
- **Header location**: `msd/msd-sim/src/DataTypes/SpatialVector.hpp`
- **Key interfaces**:
  ```cpp
  struct SpatialVector {
    Vector3D linear;   // Linear velocity bias [m/s]
    Vector3D angular;  // Angular velocity bias [rad/s]

    SpatialVector() = default;
    SpatialVector(const Vector3D& lin, const Vector3D& ang);

    // Rule of Zero
    SpatialVector(const SpatialVector&) = default;
    SpatialVector& operator=(const SpatialVector&) = default;
    SpatialVector(SpatialVector&&) noexcept = default;
    SpatialVector& operator=(SpatialVector&&) noexcept = default;
    ~SpatialVector() = default;
  };
  ```
- **Dependencies**: `Vector3D` (existing)
- **Thread safety**: Value type, thread-safe after construction
- **Error handling**: No failure modes (pure aggregate)

### Modified Components

#### ConstraintSolver

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`
- **Changes required**:
  1. Add `velocityBias` parameter to `solve()`:
     ```cpp
     SolveResult solve(
       const std::vector<Constraint*>& contactConstraints,
       const std::vector<std::reference_wrapper<const InertialState>>& states,
       const std::vector<double>& inverseMasses,
       const std::vector<Eigen::Matrix3d>& inverseInertias,
       size_t numBodies,
       double dt,
       const std::optional<Eigen::VectorXd>& initialLambda = std::nullopt,
       const std::vector<SpatialVector>& velocityBias = {});  // NEW
     ```
  2. Add `velocityBias` parameter to `assembleRHS()`:
     ```cpp
     [[nodiscard]] static Eigen::VectorXd assembleRHS(
       const std::vector<Constraint*>& contactConstraints,
       const std::vector<Eigen::MatrixXd>& jacobians,
       const std::vector<std::reference_wrapper<const InertialState>>& states,
       double dt,
       const std::vector<SpatialVector>& velocityBias);  // NEW
     ```
  3. Modify RHS assembly logic to include bias term:
     ```cpp
     // OLD:
     b(i) = -(1 + restitution) * J_i.dot(v_minus);

     // NEW:
     b(i) = -(1 + restitution) * J_i.dot(v_minus)
            - J_i.dot(v_bias);  // Bias NOT multiplied by (1+e)
     ```
- **Backward compatibility**: New parameter defaults to empty vector (no bias), preserving existing behavior

#### CollisionPipeline

- **Current location**: `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp`
- **Changes required**:
  1. Add `velocityBias` parameter to `execute()`:
     ```cpp
     void execute(std::span<AssetInertial> inertialAssets,
                  std::span<const AssetEnvironment> environmentalAssets,
                  double dt,
                  const std::vector<SpatialVector>& velocityBias = {});  // NEW
     ```
  2. Store bias reference in member variable for use in `solveConstraintsWithWarmStart()`:
     ```cpp
     private:
       const std::vector<SpatialVector>* velocityBias_{nullptr};  // Non-owning
     ```
  3. Thread bias through to `ConstraintSolver::solve()`:
     ```cpp
     SolveResult solveConstraintsWithWarmStart(double dt) {
       return constraintSolver_.solve(
         constraintPtrs_,
         states_,
         inverseMasses_,
         inverseInertias_,
         numBodies,
         dt,
         initialLambda,
         velocityBias_ ? *velocityBias_ : std::vector<SpatialVector>{});
     }
     ```
- **Backward compatibility**: New parameter defaults to empty vector (no bias)

#### WorldModel

- **Current location**: `msd/msd-sim/src/Environment/WorldModel.cpp`
- **Changes required**:
  1. Add `computeVelocityBias()` method:
     ```cpp
     [[nodiscard]] std::vector<SpatialVector> computeVelocityBias(double dt) const;
     ```
  2. Implement bias computation from potential energies:
     ```cpp
     std::vector<SpatialVector> WorldModel::computeVelocityBias(double dt) const {
       std::vector<SpatialVector> bias(inertialAssets_.size());

       for (size_t i = 0; i < inertialAssets_.size(); ++i) {
         const auto& asset = inertialAssets_[i];

         // Accumulate forces from all potential energies
         Vector3D netForce{0, 0, 0};
         Vector3D netTorque{0, 0, 0};
         for (const auto& potential : potentialEnergies_) {
           netForce += potential->computeForce(asset.getState(), asset.getMass());
           netTorque += potential->computeTorque(asset.getState(), asset.getInertiaTensor());
         }

         // Convert to velocity bias (no direct mutation!)
         bias[i].linear = (netForce / asset.getMass()) * dt;
         bias[i].angular = (asset.getInverseInertiaTensor() * netTorque) * dt;
       }

       return bias;
     }
     ```
  3. Modify `updateCollisions()` to compute and pass bias:
     ```cpp
     void WorldModel::updateCollisions(double dt) {
       collisionPipeline_.advanceFrame();
       collisionPipeline_.expireOldEntries();

       // NEW: Compute velocity bias from potential energies
       auto velocityBias = computeVelocityBias(dt);

       // Pass bias to pipeline
       collisionPipeline_.execute(
         std::span{inertialAssets_},
         std::span{environmentalAssets_},
         dt,
         velocityBias);

       collisionActiveThisFrame_ = collisionPipeline_.hadCollisions();
     }
     ```
  4. **REMOVE** direct velocity mutation from `update()`:
     ```cpp
     // DELETE THIS CODE BLOCK:
     // for (auto& asset : inertialAssets_) {
     //   for (const auto& potential : potentialEnergies_) {
     //     Vector3D force = potential->computeForce(asset.getState(), asset.getMass());
     //     asset.applyLinearAcceleration(force / asset.getMass(), dt);
     //     ...
     //   }
     // }
     ```
- **Backward compatibility**: No public API changes (internal implementation detail)

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| SpatialVector | ConstraintSolver | Data parameter | Passed via `solve()` and `assembleRHS()` |
| SpatialVector | CollisionPipeline | Data parameter | Passed via `execute()`, stored as reference |
| SpatialVector | WorldModel | Created | `computeVelocityBias()` produces vector of biases |
| WorldModel::computeVelocityBias | PotentialEnergy | Query | Accumulates forces from existing potential energy system |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `test/Physics/Collision/BasicCollisionTests.cpp` | `B3_SphereOnFloor_NoRotation` | **FIX EXPECTED** | Should pass after bias implementation (omega < 0.5 rad/s) |
| `test/Physics/Collision/ParameterIsolationTests.cpp` | `H3_TimestepSensitivity_EnergyConverges` | **FIX OR UPDATE** | May pass with correct physics, or test assertion may need update to reflect improved behavior |
| `test/Physics/Collision/DriftTests.cpp` | `D1_CubeAtRest_NoDrift`, `D4_CubeOnFloor_NoDrift` | **NO REGRESSION** | Must continue to pass (0047 fixes preserved) |
| `test/Physics/Collision/HighEnergyTests.cpp` | `H1_HighDropTest_EnergyDissipation` | **NO REGRESSION** | Must continue to pass (0047 fixes preserved) |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| SpatialVector | `SpatialVectorTest.cpp` | Construction, value semantics, Rule of Zero |
| WorldModel | `WorldModel_computeVelocityBias` | Bias computed correctly from gravity (linear only for uniform field) |
| ConstraintSolver | `ConstraintSolver_RHSWithBias` | Bias term added to RHS without (1+e) factor |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `CollisionPipeline_VelocityBias` | WorldModel, CollisionPipeline, ConstraintSolver | Bias correctly threaded through pipeline, used in RHS assembly |
| `VelocityBias_NoRestitutionCoupling` | WorldModel, CollisionPipeline, ConstraintSolver | Restitution does NOT multiply bias term in RHS |

#### Benchmark Tests (if performance-critical)

None required (no hot-path changes, bias computation is O(n) in bodies, negligible overhead).

## Open Questions

### Design Decisions (Human Input Needed)

1. **SpatialVector location**
   - Option A: Create new file `msd-sim/src/DataTypes/SpatialVector.hpp` — Pros: Clean separation, Option B: Add to existing `Vector3D.hpp` as a utility struct — Pros: Fewer files
   - Recommendation: **Option A** (new file) — SpatialVector is a distinct concept (6-DOF vs 3-DOF), deserves its own header

2. **H3 test disposition**
   - Option A: Update test assertion to reflect correct physical behavior (0% energy growth is GOOD) — Pros: Validates improved physics
   - Option B: Remove test entirely as no longer meaningful — Pros: Simplifies test suite
   - Recommendation: **Option A** (update test) — The test still validates timestep independence, just with correct energy behavior

3. **Bias parameter default**
   - Option A: Empty vector `= {}` (current proposal) — Pros: Zero allocations for legacy paths, Cons: Silent no-op if bias forgotten
   - Option B: `std::nullopt` for `std::optional<std::vector<SpatialVector>>` — Pros: Explicit opt-in, Cons: Allocation overhead, complexity
   - Recommendation: **Option A** (empty vector) — Simpler, no allocation overhead, WorldModel always provides bias post-implementation

### Requirements Clarification

None. Requirements are clear from ticket 0047 regression analysis.

---

## DD-0051-001: Use Velocity-Bias Instead of Direct Mutation

- **Affects**: `WorldModel::updateCollisions`, `CollisionPipeline::execute`, `ConstraintSolver::solve`, `ConstraintSolver::assembleRHS`
- **Rationale**: Direct velocity mutation before collision solving couples restitution with gravity (`b = -(1+e)*(v+g*dt) = -(1+e)*v - (1+e)*g*dt`). Threading gravity as a separate bias term decouples restitution: `b = -(1+e)*v - g*dt` (no `e` factor on bias).
- **Alternatives Considered**:
  - **Alt A: Post-collision gravity application**: Apply gravity AFTER collision solving. Rejected: Resting contacts would have zero relative velocity (`J*v = 0`), producing zero support force (objects penetrate).
  - **Alt B: Modify restitution to exclude bias term**: Compute `v_effective = v - v_bias`, apply restitution only to `v_effective`. Rejected: More complex than separate bias parameter, conceptually unclear.
- **Trade-offs**: Requires threading new parameter through 3 components (WorldModel, CollisionPipeline, ConstraintSolver). Benefit: Fixes 2 regressions, preserves standard gravity pre-apply semantics.
- **Status**: active

## DD-0051-002: SpatialVector as Simple Aggregate

- **Affects**: `SpatialVector`
- **Rationale**: A 6-DOF spatial quantity (linear + angular velocity bias) is naturally represented as two `Vector3D` members. No custom logic needed, so use plain aggregate with Rule of Zero.
- **Alternatives Considered**:
  - **Alt A: Inherit from Eigen::Matrix<double, 6, 1>**: Single 6-vector representation. Rejected: Less readable (index-based access), harder to extend (e.g., add frame information later).
  - **Alt B: Use std::pair<Vector3D, Vector3D>**: Standard library aggregate. Rejected: `.first`/`.second` less readable than `.linear`/`.angular`.
- **Trade-offs**: Two members instead of one 6-vector. Benefit: Clearer intent, easier extension.
- **Status**: active

## DD-0051-003: Remove Direct Velocity Mutation from WorldModel

- **Affects**: `WorldModel::update`
- **Rationale**: The gravity pre-apply introduced in ticket 0047 directly mutated velocities before collision solving. This code is now redundant with the velocity-bias approach and must be removed to avoid double-counting gravity.
- **Alternatives Considered**:
  - **Alt A: Keep mutation, zero out bias**: Keep the direct mutation, pass empty bias to pipeline. Rejected: Conceptually inconsistent, doesn't solve restitution coupling.
  - **Alt B: Hybrid approach (mutation + bias correction)**: Mutate velocities, then subtract bias from RHS. Rejected: Overly complex, achieves same result as pure bias approach.
- **Trade-offs**: Requires deleting working code from ticket 0047. Benefit: Single source of truth for gravity application (bias parameter).
- **Status**: active
