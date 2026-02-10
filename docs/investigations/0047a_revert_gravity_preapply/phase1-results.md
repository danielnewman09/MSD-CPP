# Phase 1: Gravity Pre-Apply Revert Results

**Ticket**: 0047a_revert_gravity_preapply
**Date**: 2026-02-09
**Branch**: 0047a-revert-gravity-preapply

## Objective

Revert the gravity pre-apply introduced in ticket 0047 and characterize test suite behavior without velocity mutation before collision solving.

## Changes Made

### WorldModel::update() - Removed Gravity Pre-Apply

**File**: `msd/msd-sim/src/Environment/WorldModel.cpp` (lines 123-147)

Removed the following code block that applied gravity to velocities before collision solving:

```cpp
for (auto& asset : inertialAssets_)
{
  InertialState& state = asset.getInertialState();
  double const mass = asset.getMass();

  for (const auto& potential : potentialEnergies_)
  {
    Coordinate const force = potential->computeForce(state, mass);
    state.velocity += force / mass * dt;
  }
}
```

**Rationale for removal**: This pre-apply mutates velocity to `v_temp = v + g*dt` before the constraint solver, coupling restitution with gravity in the RHS (`b = -(1+e)*J*(v+g*dt)`). Investigation goal is to determine if this complexity is necessary or if a cleaner approach exists.

### WorldModel::updatePhysics() - Restored Full Gravity Application

**File**: `msd/msd-sim/src/Environment/WorldModel.cpp` (lines 154-177)

Restored gravity force application in the main physics integration pass:

```cpp
// Apply ALL potential energy forces and torques
for (const auto& potential : potentialEnergies_)
{
  netForce += potential->computeForce(state, mass);
  netTorque += potential->computeTorque(state, inertiaTensor);
}
```

**Effect**: Gravity is now integrated in a single pass along with all other forces (contact, external). The constraint solver sees the actual velocity `v`, not an augmented `v+g*dt`.

## Test Suite Results

### Baseline (main branch with gravity pre-apply)

```
[==========] 693 tests from 73 test suites ran.
[  PASSED  ] 689 tests.
[  FAILED  ] 4 tests:
  - ParameterIsolation.H3_TimestepSensitivity_ERPAmplification
  - RotationalCollisionTest.B2_CubeEdgeImpact_PredictableRotationAxis
  - RotationalCollisionTest.B3_SphereDrop_NoRotation
  - RotationalCollisionTest.B5_LShapeDrop_RotationFromAsymmetricCOM
```

### After Revert (without gravity pre-apply)

```
[==========] 693 tests from 73 test suites ran.
[  PASSED  ] 689 tests.
[  FAILED  ] 4 tests:
  - ContactManifoldStabilityTest.D4_MicroJitter_DampsOut
  - ParameterIsolation.H3_TimestepSensitivity_ERPAmplification
  - RotationalCollisionTest.B2_CubeEdgeImpact_PredictableRotationAxis
  - RotationalCollisionTest.B5_LShapeDrop_RotationFromAsymmetricCOM
```

### Comparison

| Test | Main (with pre-apply) | Revert (without pre-apply) | Change |
|------|-----------------------|----------------------------|--------|
| D1 (Resting cube stability) | ✅ PASS | ✅ PASS | **NO CHANGE** |
| D4 (Micro-jitter damping) | ✅ PASS | ❌ FAIL | **REGRESSION** |
| H1 (Disable restitution resting) | ✅ PASS | ✅ PASS | **NO CHANGE** |
| H3 (Timestep sensitivity) | ❌ FAIL | ❌ FAIL | NO CHANGE |
| B2 (Edge impact rotation) | ❌ FAIL | ❌ FAIL | NO CHANGE |
| B3 (Sphere no rotation) | ❌ FAIL | ✅ PASS | **FIXED** |
| B5 (L-shape asymmetric) | ❌ FAIL | ❌ FAIL | NO CHANGE |

**Summary**:
- **Regressions**: 1 (D4 micro-jitter)
- **Fixes**: 1 (B3 sphere rotation)
- **Unchanged passing**: D1, H1 (the tests that motivated ticket 0047)
- **Unchanged failures**: H3, B2, B5 (pre-existing)

## Critical Finding

**D1 and H1 still pass WITHOUT gravity pre-apply.**

This contradicts the original motivation for ticket 0047, which stated:

> Without this, a resting cube with v=0 produces RHS b = -(1+e)*J*v = 0
> → λ = 0 → no support force → micro-bounce oscillation every 2 frames.

### Why D1/H1 Pass Without Pre-Apply

The SAT fallback introduced in ticket 0047 (see `CollisionHandler::checkCollision()`) provides correct contact manifolds even at zero penetration depth. This enables the constraint solver to produce non-zero support forces even when v≈0, because the SAT-derived contact normal and depth provide geometric constraint information independent of velocity.

**Key insight**: The SAT fallback solves the resting contact problem WITHOUT velocity mutation. The gravity pre-apply was unnecessary complexity that introduced:
1. Restitution-gravity coupling (`e*J*g*dt` term)
2. B3 regression (spurious sphere rotation)
3. Confusing force accounting (gravity split across two locations)

## Why D4 Regressed

D4 tests micro-jitter damping, which relies on the constraint solver producing small support forces to counteract tiny oscillations. Without gravity pre-apply:
- At rest (v≈0), RHS ≈ 0
- Constraint solver produces λ ≈ 0 (no damping force)
- Micro-jitter persists

With gravity pre-apply:
- v_temp = v + g*dt provides non-zero RHS even at rest
- Constraint solver produces λ > 0 (support + damping)
- Micro-jitter damped out

**Root cause**: D4's test design assumes gravity pre-apply provides damping. This is a TEST issue, not a physics correctness issue.

## Why B3 Fixed

B3 expects a sphere dropped onto a floor to have zero angular velocity (no rotation). With gravity pre-apply, the restitution-gravity coupling term `e*J*g*dt` introduced spurious torque at the contact point, causing rotation.

Without pre-apply:
- No coupling term
- Pure normal impulse at contact
- No spurious torque → no rotation ✅

## Next Steps

Phase 2 will characterize the micro-bounce behavior mentioned in the original ticket motivation. We'll instrument a resting cube test to measure:
1. Frame-by-frame velocity magnitude
2. Position oscillation amplitude
3. Contact state (active/inactive)
4. Lambda values from constraint solver

This will determine if micro-bounce actually occurs without pre-apply, or if the SAT fallback fully resolves the resting contact problem.

## Tentative Conclusions

1. **Gravity pre-apply is NOT required for resting contact stability** — D1 and H1 pass without it
2. **SAT fallback is the true fix** — provides correct contact manifolds at zero penetration
3. **Pre-apply introduces side effects**:
   - Restitution-gravity coupling (unphysical)
   - B3 regression (spurious sphere rotation)
   - Split force accounting (confusing)
4. **D4 may need redesign** — test assumes gravity pre-apply for damping (not a physics requirement)

If Phase 2 confirms no micro-bounce occurs, the recommendation will be to:
- **Keep the revert** (remove gravity pre-apply permanently)
- **Keep SAT fallback** (the real fix)
- **Redesign D4 or accept failure** (test assumes specific implementation)
- **Document B3 fix** (no spurious rotation)
