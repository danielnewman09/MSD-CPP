# Design: Split Impulse Position Correction

## Summary

Replace Baumgarte stabilization with split impulse (pseudo-velocity) position correction in the contact constraint solver. This decouples position correction from velocity correction, preventing Baumgarte bias from injecting energy into the velocity state. The implementation modifies `ConstraintSolver::assembleContactRHS()` to remove the Baumgarte term, introduces a new `PositionCorrector` class that applies position-only corrections using pseudo-velocities, and integrates the position correction pass into `WorldModel::updateCollisions()`.

This is the highest-impact change in the 0040 stabilization series, directly targeting ~9 of the 11 remaining test failures.

## Problem Statement

### Root Cause: Baumgarte Bias Creates Real Kinetic Energy

The current `ConstraintSolver::assembleContactRHS()` (line 476-540 of `ConstraintSolver.cpp`) assembles a combined RHS that mixes restitution and position correction:

```cpp
// Current: ConstraintSolver.cpp line 529-530
double const baumgarteCorrection =
    std::min((erp / dt) * penetration, kMaxBaumgarteCorrection);
b(static_cast<Eigen::Index>(i)) = (-(1.0 + e) * jv) + baumgarteCorrection;
```

The solver computes `lambda` from `A * lambda = b`, then `extractContactBodyForces()` converts lambda to force = `J^T * lambda / dt`. This force is accumulated on the body and integrated by `SemiImplicitEulerIntegrator::step()`, which updates velocity: `v_new = v_old + a * dt`.

The Baumgarte component of lambda becomes a **real velocity change** that persists as kinetic energy. For resting contacts, `v_rel ~= 0` and `e ~= 0`, so the RHS is dominated by the Baumgarte term. The resulting velocity change has nowhere to dissipate, creating a positive energy feedback loop that:

- Causes resting cubes to bounce (H1, D1)
- Amplifies rocking oscillations (C2, H8)
- Prevents tilted cubes from settling (C3)
- Injects energy during rotation (F4, F4b)

### Why the ERP Clamp (0039e) Was Insufficient

Ticket 0039e added `constexpr double kMaxBaumgarteCorrection = 5.0` (m/s) to cap the Baumgarte correction velocity. This prevented extreme energy spikes from large penetrations but did not eliminate the fundamental problem: any non-zero Baumgarte correction in the velocity RHS becomes permanent kinetic energy. The clamp reduced the magnitude but not the mechanism.

### Split Impulse Solution

Split impulse solves this by separating the constraint solve into two independent passes:

1. **Velocity solve**: RHS contains only restitution (physics-correct energy exchange)
2. **Position correction**: Uses pseudo-velocities that are applied as position changes and then discarded (never become kinetic energy)

## Architecture Changes

### PlantUML Diagram

See: `./0040b-split-impulse-position-correction.puml`

### Modified Components

#### 1. ConstraintSolver::assembleContactRHS() (ConstraintSolver.cpp)

**File**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`
**Lines**: 476-540

Remove the Baumgarte bias term from the velocity RHS. The RHS should only contain the restitution term.

**Current code** (lines 516-530):

```cpp
if (contact != nullptr)
{
    double const e = contact->getRestitution();
    double const erp =
        contact->alpha();  // alpha() returns ERP for ContactConstraint
    double const penetration = contact->getPenetrationDepth();

    // RHS: -(1+e) * J*v^- + (ERP/dt) * penetration
    // Ticket: 0039e -- Clamp Baumgarte correction velocity to prevent
    // energy injection from large penetrations or high ERP/dt ratios
    constexpr double kMaxBaumgarteCorrection = 5.0;  // m/s
    double const baumgarteCorrection =
        std::min((erp / dt) * penetration, kMaxBaumgarteCorrection);
    b(static_cast<Eigen::Index>(i)) =
        (-(1.0 + e) * jv) + baumgarteCorrection;
}
```

**Modified code**:

```cpp
if (contact != nullptr)
{
    double const e = contact->getRestitution();

    // Ticket: 0040b — Split impulse position correction
    // Velocity RHS contains only restitution, no Baumgarte bias.
    // Position correction is handled separately by PositionCorrector
    // to prevent energy injection into the velocity state.
    b(static_cast<Eigen::Index>(i)) = -(1.0 + e) * jv;
}
```

**What is removed**:
- The `erp` variable read from `contact->alpha()`
- The `penetration` variable read from `contact->getPenetrationDepth()`
- The `kMaxBaumgarteCorrection` constant (0039e clamp)
- The `baumgarteCorrection` computation and addition to RHS

**What is kept**:
- The `dynamic_cast<const ContactConstraint*>` check
- The restitution coefficient `e` read
- The restitution RHS formula `-(1.0 + e) * jv`

Note: The `erp_` member in `ContactConstraint` (default 0.2) and `alpha()` accessor remain in the class for now. They are no longer read by the solver but could be useful for diagnostics or future use. They can be removed in a cleanup pass if desired.

#### 2. PositionCorrector (New Class)

**New files**:
- `msd/msd-sim/src/Physics/Constraints/PositionCorrector.hpp`
- `msd/msd-sim/src/Physics/Constraints/PositionCorrector.cpp`

##### Interface Design

```cpp
// msd-sim/src/Physics/Constraints/PositionCorrector.hpp
// Ticket: 0040b_split_impulse_position_correction
// Design: docs/designs/0040b-split-impulse-position-correction/design.md

#ifndef MSD_SIM_PHYSICS_POSITION_CORRECTOR_HPP
#define MSD_SIM_PHYSICS_POSITION_CORRECTOR_HPP

#include <vector>
#include <Eigen/Dense>

#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

/// @brief Position-level contact correction using pseudo-velocities
///
/// Corrects penetration depth by computing position-level impulses that
/// modify body positions and orientations without affecting real velocities.
/// This is the position correction pass of the split impulse approach.
///
/// Algorithm:
/// 1. For each contact, compute position-level bias: b_pos = (beta/dt) * max(depth - slop, 0)
/// 2. Use the same effective mass matrix A = J * M^-1 * J^T from the velocity solve
/// 3. Solve A * lambda_pos = b_pos with lambda_pos >= 0 (using ASM)
/// 4. Compute pseudo-velocity: dv_pseudo = M^-1 * J^T * lambda_pos
/// 5. Apply as position change: dx = dv_pseudo * dt
/// 6. Discard pseudo-velocity (never becomes kinetic energy)
///
/// @ticket 0040b_split_impulse_position_correction
class PositionCorrector
{
public:
  /// @brief Configuration parameters for position correction
  struct Config
  {
    double beta{0.2};          ///< Position correction factor [0, 1]
    double slop{0.005};        ///< Penetration tolerance [m] (no correction below this)
    int maxIterations{4};      ///< Maximum position correction iterations
  };

  PositionCorrector() = default;
  ~PositionCorrector() = default;

  /// @brief Correct body positions to resolve penetration
  ///
  /// Computes and applies position corrections for all active contacts.
  /// Only modifies position and orientation of inertial bodies (indices < numInertial).
  /// Environment bodies (indices >= numInertial) have zero inverse mass and are unaffected.
  ///
  /// @param contactConstraints Active contact constraints with per-contact depth
  /// @param states Inertial states for all bodies (mutable for position updates)
  /// @param inverseMasses Per-body inverse mass [1/kg] (0 for static)
  /// @param inverseInertias Per-body inverse inertia tensors (world frame)
  /// @param numBodies Total bodies in solver (inertial + environment)
  /// @param numInertial Number of inertial (dynamic) bodies
  /// @param dt Timestep [s]
  /// @param config Position correction parameters
  void correctPositions(
      const std::vector<TwoBodyConstraint*>& contactConstraints,
      std::vector<InertialState*>& states,
      const std::vector<double>& inverseMasses,
      const std::vector<Eigen::Matrix3d>& inverseInertias,
      size_t numBodies,
      size_t numInertial,
      double dt,
      const Config& config = {});

  // Rule of Five
  PositionCorrector(const PositionCorrector&) = default;
  PositionCorrector& operator=(const PositionCorrector&) = default;
  PositionCorrector(PositionCorrector&&) noexcept = default;
  PositionCorrector& operator=(PositionCorrector&&) noexcept = default;

private:
  static constexpr double kRegularizationEpsilon = 1e-8;
};

} // namespace msd_sim

#endif // MSD_SIM_PHYSICS_POSITION_CORRECTOR_HPP
```

##### Algorithm Detail

The `correctPositions()` method performs the following steps:

**Step 1: Build position-level RHS**

For each contact constraint `i`, compute the position-level bias:

```
correctedDepth_i = max(depth_i - slop, 0)
b_pos_i = (beta / dt) * correctedDepth_i
```

Where `depth_i` is the per-contact penetration depth from 0040a (accessed via `ContactConstraint::getPenetrationDepth()`).

If all `b_pos_i == 0` (all penetrations within slop), return early with no correction.

**Step 2: Reuse effective mass matrix**

The effective mass matrix `A = J * M^-1 * J^T` is identical to the one used for the velocity solve. Rather than recomputing it, the `correctPositions()` method reuses the same `assembleContactJacobians()` and `assembleContactEffectiveMass()` static methods from `ConstraintSolver`.

Design decision: These methods are static and only depend on constraint geometry and mass properties, which haven't changed between the velocity solve and position correction. Making PositionCorrector call ConstraintSolver's static methods directly avoids code duplication. However, since these methods are private to ConstraintSolver, PositionCorrector will reimplement the assembly logic (Jacobian computation, effective mass assembly) using the same algorithm. This maintains encapsulation while avoiding API changes to ConstraintSolver.

Alternative considered: Caching A and jacobians from the velocity solve and passing them to PositionCorrector. This would be more efficient but would require changing the solveWithContacts() interface to expose internal solver state, coupling the two passes. The recomputation cost is negligible for typical contact counts (1-10 contacts).

**Step 3: Solve with lambda >= 0**

The position correction solve uses the same Active Set Method as the velocity solve:

```
asmResult = solveActiveSet(A, b_pos, numContacts)
```

The `lambda_pos >= 0` constraint ensures position correction only pushes bodies apart, never pulls them together.

Design decision: PositionCorrector will contain its own lightweight ASM implementation rather than calling ConstraintSolver's private `solveActiveSet()`. This keeps PositionCorrector self-contained and avoids friend access or public API pollution. The ASM algorithm is compact (~50 lines) and the position correction solve is typically small (< 10 contacts).

**Step 4: Compute pseudo-velocity and apply as position change**

For each body `k` touched by contacts:

```
// Compute pseudo-velocity from position correction impulse
dv_linear_k = inverseMass_k * sum_over_contacts(J_i_k^T * lambda_pos_i)
dw_angular_k = inverseInertia_k * sum_over_contacts(J_i_k_angular^T * lambda_pos_i)

// Apply as position change (pseudo-velocity * dt cancels with 1/dt in bias)
dx_k = dv_linear_k * dt
dtheta_k = dw_angular_k * dt
```

Position update:

```cpp
state.position += dx;
```

Orientation update using quaternion exponential:

```cpp
// Small angle approximation for quaternion update
// For small dtheta, the rotation quaternion is approximately:
//   dq = [1, dtheta/2]  (unnormalized, then normalize)
double halfAngle = dtheta.norm() * 0.5;
if (halfAngle > 1e-12)
{
    Vector3D axis = dtheta.normalized();
    QuaternionD dq{Eigen::AngleAxisd{dtheta.norm(), axis}};
    state.orientation = dq * state.orientation;
    state.orientation.normalize();
}
```

**Step 5: Discard pseudo-velocity**

The pseudo-velocity (`dv_linear`, `dw_angular`) is a local variable and goes out of scope. It is never written to `state.velocity` or `state.quaternionRate`. This is the key property that prevents energy injection.

##### Iteration Strategy

The `maxIterations` parameter (default: 4) controls how many times the position correction loop runs. Each iteration:

1. Re-reads penetration depth from contact constraints (unchanged from frame detection)
2. Computes corrected depth with slop
3. Solves and applies position changes

For the initial implementation, a single pass (iteration) is sufficient because:
- The per-contact depth from 0040a provides accurate depth at each point
- Beta = 0.2 means only 20% correction per pass, preventing overcorrection
- Multiple iterations would require re-running collision detection, which is expensive

Therefore, the implementation will use `config.maxIterations` as a future extension point but will initially run exactly **one** iteration. The config parameter is exposed for tuning and future warm-starting scenarios (0040d).

#### 3. WorldModel::updateCollisions() (WorldModel.cpp)

**File**: `msd/msd-sim/src/Environment/WorldModel.cpp`
**Lines**: 195-378

Add position correction call after the velocity solve. The position correction pass operates on the same contact constraints and body states as the velocity solve.

**Current flow** (Phase 4-5, lines 355-377):

```cpp
// ===== Phase 4: Solve PGS =====
auto solveResult = contactSolver_.solveWithContacts(
    constraintPtrs, states, inverseMasses, inverseInertias, numBodies, dt);

// ===== Phase 5: Apply Constraint Forces to Inertial Bodies =====
for (size_t k = 0; k < numInertial; ++k)
{
    const auto& forces = solveResult.bodyForces[k];
    if (forces.linearForce.norm() < 1e-12 &&
        forces.angularTorque.norm() < 1e-12)
    {
        continue;
    }
    inertialAssets_[k].applyForce(forces.linearForce);
    inertialAssets_[k].applyTorque(forces.angularTorque);
}
```

**Modified flow**:

```cpp
// ===== Phase 4: Velocity Solve (restitution only, no Baumgarte) =====
auto solveResult = contactSolver_.solveWithContacts(
    constraintPtrs, states, inverseMasses, inverseInertias, numBodies, dt);

// ===== Phase 5: Apply Constraint Forces to Inertial Bodies =====
for (size_t k = 0; k < numInertial; ++k)
{
    const auto& forces = solveResult.bodyForces[k];
    if (forces.linearForce.norm() < 1e-12 &&
        forces.angularTorque.norm() < 1e-12)
    {
        continue;
    }
    inertialAssets_[k].applyForce(forces.linearForce);
    inertialAssets_[k].applyTorque(forces.angularTorque);
}

// ===== Phase 6: Position Correction (pseudo-velocity, positions only) =====
// Ticket: 0040b_split_impulse_position_correction
// Build mutable state pointer list for position correction
std::vector<InertialState*> mutableStates;
mutableStates.reserve(numBodies);
for (auto& asset : inertialAssets_)
{
    mutableStates.push_back(&asset.getInertialState());
}
for (auto& envAsset : environmentalAssets_)
{
    // Environment states are conceptually immutable, but position corrector
    // handles this via zero inverse mass (no position change applied)
    mutableStates.push_back(
        const_cast<InertialState*>(&envAsset.getInertialState()));
}

positionCorrector_.correctPositions(
    constraintPtrs, mutableStates, inverseMasses, inverseInertias,
    numBodies, numInertial, dt);
```

**Key design decisions for WorldModel integration**:

1. **Member variable**: `PositionCorrector positionCorrector_` added to `WorldModel` private section. Stateless, lightweight — no configuration needed at construction.

2. **Mutable state access**: The position corrector needs to modify `InertialState::position` and `InertialState::orientation` directly. For inertial assets, this is via `asset.getInertialState()` (returns mutable reference). For environment assets, `const_cast` is safe because position corrector skips bodies with `inverseMass == 0`.

3. **After velocity forces, before physics integration**: Position correction runs after constraint forces are accumulated but before `updatePhysics()` integrates them. This means the corrected positions are used as the starting point for the next frame's integration, and the velocity state going into integration is uncontaminated by Baumgarte.

4. **ReferenceFrame sync**: Position correction modifies `InertialState::position` and `InertialState::orientation`. The `updatePhysics()` method (which runs after `updateCollisions()`) already synchronizes `ReferenceFrame` with `InertialState` at the end of each physics step (lines 185-188). Therefore, no additional ReferenceFrame sync is needed in the position corrector.

#### 4. WorldModel Header Changes (WorldModel.hpp)

**File**: `msd/msd-sim/src/Environment/WorldModel.hpp`

Add the `PositionCorrector` include and member variable:

```cpp
#include "msd-sim/src/Physics/Constraints/PositionCorrector.hpp"

// In private section, after contactSolver_:
PositionCorrector positionCorrector_;
```

### Unchanged Components

| Component | Why Unchanged |
|-----------|---------------|
| `ContactConstraint` | Per-contact depth already provided by 0040a. `erp_` and `alpha()` remain but are no longer read by solver. |
| `ContactConstraintFactory` | Already uses per-contact depth from 0040a. No change needed. |
| `CollisionHandler` / `GJK` / `EPA` | Detection pipeline unchanged |
| `SemiImplicitEulerIntegrator` | Integration unchanged — only receives velocity-level forces |
| `ConstraintSolver::solveActiveSet()` | ASM algorithm unchanged |
| `ConstraintSolver::assembleContactEffectiveMass()` | Effective mass matrix unchanged |
| `ConstraintSolver::assembleContactJacobians()` | Jacobian assembly unchanged |
| `EnergyTracker` | Energy computation unchanged |

## Data Flow

### Before (Baumgarte in Velocity RHS)

```
updateCollisions(dt)
    │
    ├── Phase 1-3: Detect collisions, build constraints, build solver arrays
    │
    ├── Phase 4: solveWithContacts()
    │   ├── assembleContactRHS():  b_i = -(1+e)*Jv + (ERP/dt)*depth
    │   │                                  └── velocity ──┘  └── position ──┘
    │   │                                  (Both become real velocity change)
    │   ├── solveActiveSet(A, b) → lambda
    │   └── extractContactBodyForces() → per-body force = J^T*lambda/dt
    │
    └── Phase 5: applyForce() / applyTorque()
            │
            ▼
        updatePhysics(dt)
            └── integrator.step(): v += (F/m)*dt ← Baumgarte energy injected here
```

### After (Split Impulse)

```
updateCollisions(dt)
    │
    ├── Phase 1-3: Detect collisions, build constraints, build solver arrays
    │
    ├── Phase 4: solveWithContacts()
    │   ├── assembleContactRHS():  b_i = -(1+e)*Jv   ← velocity ONLY
    │   ├── solveActiveSet(A, b) → lambda_vel
    │   └── extractContactBodyForces() → per-body force (restitution only)
    │
    ├── Phase 5: applyForce() / applyTorque() ← clean velocity forces
    │
    └── Phase 6: positionCorrector_.correctPositions()
        ├── b_pos_i = (beta/dt) * max(depth_i - slop, 0)
        ├── Build A = J * M^-1 * J^T
        ├── Solve A * lambda_pos = b_pos (lambda_pos >= 0)
        ├── dv_pseudo = M^-1 * J^T * lambda_pos
        ├── position += dv_pseudo * dt    ← position change ONLY
        └── discard dv_pseudo             ← never becomes KE
            │
            ▼
        updatePhysics(dt)
            └── integrator.step(): v += (F/m)*dt ← no Baumgarte, energy-clean
```

## ERP Clamp Removal

The ERP velocity clamp from ticket 0039e (`constexpr double kMaxBaumgarteCorrection = 5.0`) is removed as part of this change. The clamp was:

```cpp
// Ticket: 0039e
constexpr double kMaxBaumgarteCorrection = 5.0;  // m/s
double const baumgarteCorrection =
    std::min((erp / dt) * penetration, kMaxBaumgarteCorrection);
```

This entire block is removed because:
1. The Baumgarte term is no longer in the velocity RHS
2. Position correction uses its own `beta` and `slop` parameters
3. The `max(depth - slop, 0)` formulation naturally handles micro-penetrations
4. The `beta = 0.2` factor prevents overcorrection more gracefully than a hard velocity clamp

## Energy Conservation Proof

### Velocity Solve

After removing Baumgarte, the velocity RHS is `b_i = -(1+e)*J_i*v_minus`. For `e <= 1`:

```
v_rel_post = -e * v_rel_pre
|v_rel_post| <= |v_rel_pre|   (for e in [0, 1])
```

The kinetic energy along the constraint normal is:

```
KE_post = 0.5 * m_eff * v_rel_post^2
        = 0.5 * m_eff * e^2 * v_rel_pre^2
        = e^2 * KE_pre
        <= KE_pre              (for e in [0, 1])
```

Therefore, `E_post <= E_pre` is guaranteed for the velocity solve.

### Position Correction

The pseudo-velocity `dv_pseudo` modifies only positions. It is never added to `state.velocity` or `state.quaternionRate`. Therefore, the kinetic energy `KE = 0.5*m*v^2 + 0.5*omega^T*I*omega` is **unchanged** by position correction. The potential energy may change slightly (position changes affect gravity potential), but the change is proportional to `beta * depth` which is geometrically bounded.

### Combined Guarantee

```
E_post = KE_post + PE_post
       = KE_vel_only + PE(position_corrected)
       <= KE_pre + PE(position_corrected)         (from velocity solve)
       ~= KE_pre + PE_pre - m*g*beta*depth_avg   (gravity PE shift)
       <= KE_pre + PE_pre                          (for objects pushed upward)
       = E_pre
```

The position correction pushes penetrating objects apart, which for gravity-loaded contacts means pushing the upper object upward (increasing PE, decreasing KE). This is energy-neutral or energy-dissipating.

## Parameter Selection

### Beta (Position Correction Factor)

| Value | Behavior |
|-------|----------|
| 0.0 | No position correction (objects may interpenetrate) |
| 0.1 | Conservative, very stable, slow penetration resolution |
| **0.2** | **Default. Good balance of stability and response (matches ERP default)** |
| 0.5 | Aggressive, may cause jitter for large penetrations |
| 1.0 | Full correction per step (unstable, don't use) |

Beta = 0.2 matches the existing ERP default, providing behavioral continuity. The critical difference is that the correction is now applied to positions instead of velocities.

### Slop (Penetration Tolerance)

| Value | Behavior |
|-------|----------|
| 0.0 | No tolerance, corrects all penetration (may cause jitter) |
| 0.001 | 1mm tolerance, tight fit |
| **0.005** | **Default. 5mm tolerance, absorbs numerical noise** |
| 0.01 | 1cm tolerance, visible gaps in resting contacts |
| 0.05 | 5cm tolerance, clearly visible floating |

Slop = 0.005m (5mm) absorbs numerical noise from floating-point collision detection without causing visible gaps. This matches industry-standard values used in Bullet and Box2D.

### Max Iterations

Default: 4. For the initial implementation, only 1 iteration is executed. The parameter is exposed for future tuning when warm-starting (0040d) is implemented.

## Test Strategy

### New Unit Tests (SplitImpulseTest.cpp)

**File**: `msd/msd-sim/test/Physics/Constraints/SplitImpulseTest.cpp`

| Test | Description | Verification |
|------|-------------|--------------|
| `VelocityRHS_NoBaumgarteBias` | Resting contact (v_rel ~= 0) | RHS ~= 0, not (ERP/dt)*depth |
| `VelocityRHS_RestitutionOnly` | Moving contact (v_rel = -1 m/s, e = 0.5) | RHS = -(1+0.5)*(-1) = 1.5 |
| `PositionCorrection_ReducesPenetration` | 10mm penetration, slop=5mm, beta=0.2 | Position change > 0 |
| `PositionCorrection_DoesNotChangeVelocity` | Any contact | velocity unchanged after correctPositions() |
| `PositionCorrection_DoesNotChangeAngularVelocity` | Any contact | angular velocity unchanged |
| `Slop_NoCorrectionBelowThreshold` | Penetration < slop | No position change |
| `Slop_PartialCorrectionAboveThreshold` | Penetration = slop + delta | Correction proportional to delta |
| `EnergyConservation_RestingContact` | Resting cube on floor, 100 frames | E_post <= E_pre every frame |
| `EnergyConservation_RockingCube` | Tilted cube on floor, 200 frames | Energy monotonically decreasing |
| `MultiContact_IndependentCorrection` | 4-point manifold | Each contact corrected proportionally to its depth |
| `EnvironmentBody_NotMoved` | Inertial on static floor | Floor position unchanged |
| `OrientationCorrection_TiltedCube` | Cube penetrating at angle | Orientation updated, not just position |
| `BetaZero_NoCorrection` | beta = 0.0 | No position change despite penetration |
| `SlopZero_FullCorrection` | slop = 0.0 | All penetration corrected (proportional to beta) |

### Integration Tests

All existing diagnostic tests from 0039b/0039c/0039d should be re-run after implementation:

| Category | Expected Result |
|----------|----------------|
| A1-A6 (Linear collisions) | All pass (no regression) |
| F1-F5 (Energy conservation) | All pass (no regression) |
| B1 (Corner impact) | Pass (energy within tolerance, no Baumgarte injection) |
| C2, C3 (Rocking/settling) | Pass (amplitude decreases, cube settles) |
| D1, D4 (Stability) | Pass (no drift over 1000 frames, no amplification) |
| F4, F4b (Rotation energy) | Pass (energy conserved/decreased) |
| H1, H8 (Parameter isolation) | Pass (no energy injection at rest) |

### Regression Tests

- `WorldModelContactIntegrationTest` — All existing tests pass
- `ConstraintSolverContactTest` — All existing tests pass (velocity solve behavior unchanged for non-penetrating contacts)
- `ConstraintSolverASMTest` — All existing tests pass
- Stacking: 3-5 cube stack stable for 500 frames (position correction pushes apart without sinking)

## CMake Integration

### Source Files

Add to `msd/msd-sim/src/Physics/Constraints/CMakeLists.txt` (or parent CMakeLists.txt that lists constraint sources):

```cmake
target_sources(msd_sim PRIVATE
    # ... existing sources ...
    src/Physics/Constraints/PositionCorrector.cpp
)
```

### Test Files

Add to `msd/msd-sim/test/Physics/Constraints/CMakeLists.txt`:

```cmake
target_sources(msd_sim_test PRIVATE
    # ... existing test sources ...
    SplitImpulseTest.cpp
)
```

## Files Summary

### Modified Files

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Remove Baumgarte bias from velocity RHS (lines 516-530) |
| `msd-sim/src/Environment/WorldModel.hpp` | Add `PositionCorrector` include and member variable |
| `msd-sim/src/Environment/WorldModel.cpp` | Add Phase 6 position correction call in `updateCollisions()` |
| `msd-sim/src/CMakeLists.txt` | Add PositionCorrector.cpp to build |
| `msd-sim/test/Physics/Constraints/CMakeLists.txt` | Add SplitImpulseTest.cpp to test build |

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/src/Physics/Constraints/PositionCorrector.hpp` | Position correction class header |
| `msd-sim/src/Physics/Constraints/PositionCorrector.cpp` | Position correction implementation |
| `msd-sim/test/Physics/Constraints/SplitImpulseTest.cpp` | Unit and integration tests |

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Objects sink through floor (correction too weak) | Medium | High | Default beta=0.2 provides adequate correction. Stacking regression test validates. Increase beta if needed. |
| Jitter at rest (correction oscillates) | Medium | Medium | Slop=5mm absorbs micro-penetrations. Resting velocity threshold disables restitution. |
| Position correction overcorrects (objects bounce) | Low | Medium | Beta=0.2 limits per-step correction to 20%. Single iteration prevents feedback. |
| Orientation correction instability | Low | Medium | Small-angle approximation valid for beta=0.2 corrections. Quaternion normalized after update. |
| Performance regression | Low | Low | Position correction reuses same matrix assembly. ASM solve for small systems is < 0.1ms. |
| Existing tests regress | Low | High | Velocity solve unchanged for bouncing contacts (b = -(1+e)*jv same as before when e > 0 and no penetration). |

## Performance Impact

The position correction pass adds:
- One Jacobian assembly: O(C) for C contacts
- One effective mass matrix assembly: O(C^2) for C contacts
- One ASM solve: O(C^3) worst case, typically < C iterations
- Per-body position update: O(B) for B bodies

For typical contact scenarios (1-10 contacts, 1-10 bodies), the total overhead is < 0.1ms. This is comparable to the existing velocity solve and well within the 16.67ms frame budget at 60 FPS.

## References

- Ticket: [0040b_split_impulse_position_correction](../../../tickets/0040b_split_impulse_position_correction.md)
- Parent ticket: [0040_collision_stabilization_phase2](../../../tickets/0040_collision_stabilization_phase2.md)
- Dependency: [0040a_per_contact_penetration_depth](../../../tickets/0040a_per_contact_penetration_depth.md) (per-contact depth)
- Blocks: [0040d_contact_persistence_warm_starting](../../../tickets/0040d_contact_persistence_warm_starting.md) (warm-starting)
- 0039e findings: EPA normal-space fix + ERP clamp (clamp removed by this ticket)
- Erin Catto, "Iterative Dynamics with Temporal Coherence" (GDC 2005) — split impulse technique
- Erin Catto, Box2D documentation — position correction and slop parameters
- Erwin Coumans, Bullet Physics — split impulse implementation reference
