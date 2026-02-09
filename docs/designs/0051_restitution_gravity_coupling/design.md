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

None. The velocity bias reuses the existing `InertialState` struct (see DD-0051-002).

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
       const std::optional<std::vector<InertialState>>& velocityBias = std::nullopt);  // NEW
     ```
  2. Add `velocityBias` parameter to `assembleRHS()`:
     ```cpp
     [[nodiscard]] static Eigen::VectorXd assembleRHS(
       const std::vector<Constraint*>& contactConstraints,
       const std::vector<Eigen::MatrixXd>& jacobians,
       const std::vector<std::reference_wrapper<const InertialState>>& states,
       double dt,
       const std::optional<std::vector<InertialState>>& velocityBias);  // NEW
     ```
  3. Modify RHS assembly logic to include bias term:
     ```cpp
     // OLD:
     b(i) = -(1 + restitution) * J_i.dot(v_minus);

     // NEW:
     b(i) = -(1 + restitution) * J_i.dot(v_minus)
            - J_i.dot(v_bias);  // Bias NOT multiplied by (1+e)
     ```
- **Backward compatibility**: New parameter defaults to `std::nullopt` (no bias), preserving existing behavior

#### CollisionPipeline

- **Current location**: `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp`
- **Changes required**:
  1. Add `velocityBias` parameter to `execute()`:
     ```cpp
     void execute(std::span<AssetInertial> inertialAssets,
                  std::span<const AssetEnvironment> environmentalAssets,
                  double dt,
                  const std::optional<std::vector<InertialState>>& velocityBias = std::nullopt);  // NEW
     ```
  2. Pass bias through to `solveConstraintsWithWarmStart()` and `ConstraintSolver::solve()`:
     ```cpp
     SolveResult solveConstraintsWithWarmStart(
         double dt,
         const std::optional<std::vector<InertialState>>& velocityBias) {
       return constraintSolver_.solve(
         constraintPtrs_,
         states_,
         inverseMasses_,
         inverseInertias_,
         numBodies,
         dt,
         initialLambda,
         velocityBias);
     }
     ```
- **Backward compatibility**: New parameter defaults to `std::nullopt` (no bias)

#### WorldModel

- **Current location**: `msd/msd-sim/src/Environment/WorldModel.cpp`
- **Changes required**:
  1. Add `computeVelocityBias()` method:
     ```cpp
     [[nodiscard]] std::vector<InertialState> computeVelocityBias(double dt) const;
     ```
  2. Implement bias computation from potential energies:
     ```cpp
     std::vector<InertialState> WorldModel::computeVelocityBias(double dt) const {
       std::vector<InertialState> bias(inertialAssets_.size());

       for (size_t i = 0; i < inertialAssets_.size(); ++i) {
         const auto& asset = inertialAssets_[i];

         // Accumulate forces from all potential energies
         Vector3D netForce{0, 0, 0};
         Vector3D netTorque{0, 0, 0};
         for (const auto& potential : potentialEnergies_) {
           netForce += potential->computeForce(asset.getState(), asset.getMass());
           netTorque += potential->computeTorque(asset.getState(), asset.getInertiaTensor());
         }

         // Convert to velocity bias via InertialState velocity fields
         // Only velocity and angular velocity are meaningful; other fields remain default
         bias[i].velocity = (netForce / asset.getMass()) * dt;
         bias[i].setAngularVelocity(asset.getInverseInertiaTensor() * netTorque * dt);
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

| Component | Existing Component | Integration Type | Notes |
|-----------|-------------------|------------------|-------|
| InertialState (as bias) | ConstraintSolver | Data parameter | Passed via `solve()` and `assembleRHS()`; solver extracts `velocity` and `getAngularVelocity()` — same pattern as state extraction |
| InertialState (as bias) | CollisionPipeline | Data parameter | Passed via `execute()`, stored as pointer |
| WorldModel::computeVelocityBias | InertialState | Created | Produces `std::vector<InertialState>` with velocity fields populated |
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
| WorldModel | `WorldModel_computeVelocityBias` | Bias InertialState has correct velocity fields from gravity (linear only for uniform field) |
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

1. **H3 test disposition**
   - Option A: Update test assertion to reflect correct physical behavior (0% energy growth is GOOD) — Pros: Validates improved physics
   - Option B: Remove test entirely as no longer meaningful — Pros: Simplifies test suite
   - Recommendation: **Option A** (update test) — The test still validates timestep independence, just with correct energy behavior

2. **Bias parameter default** — **Resolved: Option B**
   - `std::optional<std::vector<InertialState>>` with `std::nullopt` default
   - Explicit opt-in prevents silent no-op if bias is accidentally omitted

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

## DD-0051-002: Reuse InertialState for Velocity Bias

- **Affects**: `ConstraintSolver::solve`, `ConstraintSolver::assembleRHS`, `CollisionPipeline::execute`, `WorldModel::computeVelocityBias`
- **Rationale**: `InertialState` already carries `velocity` (Vector3D) and angular velocity (via `getAngularVelocity()`/`setAngularVelocity()`), which are exactly the fields needed for a velocity bias. The solver already extracts these fields from `InertialState` for the state vector, so bias extraction uses the identical pattern — no new data access code. Reusing `InertialState` also provides future flexibility to inject other bias types (e.g., acceleration bias) without introducing new types. Only the velocity fields are populated; position, orientation, and acceleration fields remain at defaults and are ignored.
- **Alternatives Considered**:
  - **Alt A: New SpatialVector aggregate** (`struct { Vector3D linear; Vector3D angular; }`): Dedicated 6-DOF type. Rejected: Introduces a new type with no additional expressiveness; duplicates fields already present in InertialState; requires new extraction code in the solver.
  - **Alt B: Inherit from Eigen::Matrix<double, 6, 1>**: Single 6-vector representation. Rejected: Less readable (index-based access), no semantic field names.
  - **Alt C: Use std::pair<Vector3D, Vector3D>**: Standard library aggregate. Rejected: `.first`/`.second` less readable than `.velocity`/`getAngularVelocity()`.
- **Trade-offs**: InertialState carries unused fields (position, orientation, acceleration) per body — minor memory overhead (~100 bytes/body) that is negligible for typical body counts. Benefit: Zero new types, consistent extraction pattern, future extensibility.
- **Status**: active

## DD-0051-003: Remove Direct Velocity Mutation from WorldModel

- **Affects**: `WorldModel::update`
- **Rationale**: The gravity pre-apply introduced in ticket 0047 directly mutated velocities before collision solving. This code is now redundant with the velocity-bias approach and must be removed to avoid double-counting gravity.
- **Alternatives Considered**:
  - **Alt A: Keep mutation, zero out bias**: Keep the direct mutation, pass empty bias to pipeline. Rejected: Conceptually inconsistent, doesn't solve restitution coupling.
  - **Alt B: Hybrid approach (mutation + bias correction)**: Mutate velocities, then subtract bias from RHS. Rejected: Overly complex, achieves same result as pure bias approach.
- **Trade-offs**: Requires deleting working code from ticket 0047. Benefit: Single source of truth for gravity application (bias parameter).
- **Status**: active

---

## Design Review — Initial Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-02-09
**Status**: REVISION_REQUESTED
**Iteration**: 0 of 1

### Issues Requiring Revision

| ID | Issue | Category | Required Change |
|----|-------|----------|-----------------|
| I1 | PlantUML diagram contradicts DD-0051-002 | Architectural/Documentation Consistency | Update diagram to show `InertialState` (reused) instead of `SpatialVector` (new) |
| I2 | WorldModel::computeVelocityBias return type mismatch | C++ Quality/Consistency | Update design to use `std::vector<InertialState>` consistently (matches DD-0051-002, not `std::vector<SpatialVector>`) |

### Revision Instructions for Architect

The following changes must be made before final review:

1. **PlantUML Diagram Inconsistency (I1)**: The PlantUML diagram at `0051_restitution_gravity_coupling.puml` shows a new `SpatialVector` struct in the "ValueTypes" package, but DD-0051-002 explicitly decided to **reuse InertialState** for the velocity bias instead of creating a new type. The diagram must be updated to:
   - **Remove** the `SpatialVector` struct and its package
   - **Add** a note showing that `InertialState` is reused for velocity bias (similar to how other designs show dual-purpose usage)
   - Update the relationships to show `WorldModel`, `CollisionPipeline`, and `ConstraintSolver` working with `InertialState` for bias
   - Add a note explaining that only `velocity` and angular velocity fields are populated for bias; position/orientation remain at defaults

2. **Return Type Consistency (I2)**: The design document body shows `computeVelocityBias()` returning `std::vector<SpatialVector>` in the PlantUML diagram notes and some code snippets, but DD-0051-002 establishes that the bias uses `InertialState`. Update all references to be consistent:
   - Diagram: Change return type note to `std::vector<InertialState>`
   - Code examples: Ensure all snippets show `std::vector<InertialState>` as the bias type
   - This applies to: WorldModel::computeVelocityBias signature, CollisionPipeline::execute parameter, ConstraintSolver::solve parameter, ConstraintSolver::assembleRHS parameter

**Rationale for Revision**: The design decision DD-0051-002 provides excellent reasoning for reusing InertialState (zero new types, consistent extraction pattern, future extensibility). However, the diagram does not reflect this decision. Consistency between design decisions, diagrams, and code examples is critical for reviewability and implementation correctness.

### Items Passing Review (No Changes Needed)

#### Architectural Fit
- **Naming conventions**: ✓ Pass — `computeVelocityBias`, `assembleRHS`, `execute` follow project camelCase for methods
- **Namespace organization**: ✓ Pass — Changes are within existing `msd_sim` namespace
- **File/folder structure**: ✓ Pass — Modifications stay within `msd/msd-sim/src/` hierarchy
- **Dependency direction**: ✓ Pass — No new dependencies; bias flows WorldModel → CollisionPipeline → ConstraintSolver (correct direction)

#### C++ Design Quality (after revision)
- **RAII usage**: ✓ Pass — InertialState is a simple aggregate, no resource management needed
- **Smart pointer appropriateness**: ✓ Pass — Uses `std::vector` for bias storage (value semantics), `std::optional` wrapper for optional parameter (correct per CLAUDE.md)
- **Value/reference semantics**: ✓ Pass — `std::vector<InertialState>` passed by value to pipeline, then by `const std::optional<...>&` to solver (correct for optional parameters)
- **Rule of 0/3/5**: ✓ Pass — InertialState is a struct with compiler-generated special members (Rule of Zero)
- **Const correctness**: ✓ Pass — Bias is `const` after creation, solver receives `const std::optional<...>&`
- **Brace initialization**: ✓ Pass — Code examples use brace initialization `bias[i].velocity = ...`
- **Return values**: ✓ Pass — `computeVelocityBias()` returns value (not output parameter), consistent with CLAUDE.md preference

#### Feasibility
- **Header dependencies**: ✓ Pass — No new headers needed; InertialState already known to all components
- **Template complexity**: ✓ Pass — No templates involved
- **Memory strategy**: ✓ Pass — `std::vector<InertialState>` allocated once per frame in WorldModel, passed through pipeline
- **Thread safety**: ✓ Pass — No threading concerns (existing components are not thread-safe, bias doesn't change that)
- **Build integration**: ✓ Pass — No build system changes needed

#### Testability
- **Isolation possible**: ✓ Pass — Each component (WorldModel, ConstraintSolver) can be tested independently with mock bias inputs
- **Mockable dependencies**: ✓ Pass — Bias is data (not a dependency), easily mocked as `std::vector<InertialState>{...}`
- **Observable state**: ✓ Pass — Solver RHS can be inspected via existing debug/test hooks

---
