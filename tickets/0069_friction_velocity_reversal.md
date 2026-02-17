# Ticket 0069: Friction Velocity Reversal During Sustained Contact

## Status
- [x] Draft
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Ready for Implementation
**Type**: Bug
**Priority**: High
**Created**: 2026-02-17
**Generate Tutorial**: No
**Parent Ticket**: None
**Depends On**: [0068_nlopt_friction_cone_solver](0068_nlopt_friction_cone_solver.md)

---

## Overview

During sustained low-speed contact (e.g., a tumbling cube sliding on a floor), the friction solver produces impulses that reverse the sliding direction. This is physically impossible — Coulomb friction can only decelerate sliding, never reverse it. The result is oscillatory velocity zero-crossings that keep the cube tumbling indefinitely instead of settling to rest.

The fix introduces a **sliding friction mode** on the existing `FrictionConstraint`. When `ContactCache` detects sustained sliding contact, the friction tangent basis is aligned with the sliding direction and the tangent lambda bounds are made unilateral — preventing the solver from ever producing friction that accelerates in the sliding direction, regardless of normal-tangent coupling in the effective mass matrix.

---

## Observed Behavior

In the `RotationalEnergyTest_F4` test (1m cube, tilted 45°, dropped onto floor with e=1.0, mu=0.5):

- After initial bounce phase (~250 frames), cube enters sustained tumbling contact
- Linear velocity components (vx, vy) oscillate in sign on ~100-frame timescale
- At frame 500 (8s), cube still has KE ~ 0.1 J instead of settling to rest
- Cube "balances on edge" (CoM z ~ 0.66) rather than toppling to a stable face

### Velocity trace (frames 300-500):

| Frame | vx (m/s) | vy (m/s) |
|-------|----------|----------|
| 300   | +0.26    | +0.12    |
| 320   | +0.43    | +0.24    |
| 375   | -0.45    | -0.27    |
| 400   | -0.56    | -0.15    |
| 450   | +0.69    | +0.38    |
| 500   | -0.25    | -0.14    |

At these low speeds with no impacts occurring, friction should monotonically decelerate toward zero. Velocity reversals are non-physical.

---

## Root Cause Analysis

### Current state (post-0068 clamps)

Ticket 0068 added two post-solve clamps in `ConstraintSolver`:

1. **`clampPositiveWorkFriction`**: Zeros tangent lambdas if `lambda_t * jv > 0` (friction accelerating). Also caps tangent impulses that would reverse velocity by >10% of pre-solve speed.

2. **`clampImpulseEnergy`**: Scales entire impulse if total delta-KE > 0 (prevents restitution energy injection on rotating contacts).

These clamps eliminated gross energy injection (F4 energy profile: 15.3 J -> 6.6 J at frame 500) and fixed the A6 glancing collision test. But they don't prevent friction from reversing the sliding direction when the reversal is small per-frame but accumulates over many frames.

### Mechanism

The NLopt coupled QP solves normal + tangent impulses simultaneously. Cross-coupling in the effective mass matrix A means:
- A normal impulse at an off-center contact changes the tangent velocity through lever-arm coupling
- The tangent impulse the solver computes is based on pre-solve tangent velocity
- After the coupled normal+tangent impulse is applied, the tangent velocity may have reversed

The post-solve clamp catches cases where the tangent lambda itself does positive work, but NOT cases where the normal lambda (through coupling) reverses the tangent velocity and the tangent lambda just happens to be small.

### Why post-solve clamping is insufficient

The current clamp compares post-solve tangent velocity against **this frame's** pre-solve tangent velocity. But the pre-solve velocity is already corrupted by previous frames' reversals. Each frame's reversal may be small enough to pass the 10% threshold, but they accumulate into large oscillations over many frames.

### The fix: sliding friction mode

In Coulomb friction theory, kinetic (sliding) friction has a stricter constraint than static friction:
- **Static**: `|f_t| <= mu * f_n`, force can oppose any direction (sticking)
- **Kinetic**: `f_t = -mu * f_n * v_t/|v_t|`, force must oppose the sliding direction

The current solver enforces only the friction cone (static constraint). The fix adds the kinetic constraint: when sustained sliding is detected, friction can only oppose the established sliding direction.

---

## Design: Sliding Friction Mode

### Approach

When `ContactCache` indicates a contact pair has been in sustained sliding (tangent velocity above threshold for multiple frames), modify the `FrictionConstraint` for that contact:

1. **Align tangent basis with sliding direction**: Set `t1 = -d_slide` (opposing motion), `t2` perpendicular to both `t1` and `n`. This makes the `lambda_t1` component directly correspond to "deceleration along sliding direction."

2. **Make `lambda_t1` unilateral**: With `t1 = -d_slide`, a positive `lambda_t1` decelerates the body. Adding the constraint `lambda_t1 >= 0` prevents the solver from ever applying friction that accelerates in the sliding direction — regardless of A matrix coupling.

3. **Track sliding state in ContactCache**: Store sliding direction (`Vector3D`) and frame count (`int`) per body pair. Update each frame during cache update (Phase 4.5).

### Component changes

#### ContactCache
- Add `Vector3D slidingDirection` and `int slidingFrameCount` to `CachedContact`
- Update sliding state each frame: if tangent velocity > threshold, increment count and update direction; otherwise reset
- Provide `getSlidingState(bodyA, bodyB)` query returning direction + whether sliding mode is active

#### FrictionConstraint
- Add `setSlidingMode(Vector3D slidingDirection)` method
- When in sliding mode: override tangent basis computation to align t1 with `-slidingDirection`
- Expose sliding mode flag for NLopt solver to query

#### NLoptFrictionSolver
- Accept optional per-contact tangent lower bounds (currently assumes symmetric cone)
- For sliding contacts: add inequality constraint `lambda_t1 >= 0` to the NLopt problem
- NLopt handles arbitrary nonlinear inequality constraints natively

#### CollisionPipeline
- In `createConstraints()`: query ContactCache for sliding state
- If sliding: call `FrictionConstraint::setSlidingMode(direction)` before adding to constraint list
- In cache update (Phase 4.5): compute tangent velocity at contact and update sliding state

---

## Requirements

### R1: Track sliding contact state
Extend `ContactCache` to store per-body-pair sliding direction and duration. A contact is "sliding" when:
- Contact has persisted for >= 3 frames (cache age)
- Tangent velocity magnitude > threshold (e.g., 0.01 m/s)

### R2: Implement sliding friction mode
When a contact is in sliding mode:
- FrictionConstraint aligns t1 with `-slidingDirection`
- NLoptFrictionSolver enforces `lambda_t1 >= 0` for that contact
- The combined effect prevents friction from ever accelerating in the sliding direction

### R3: Settling behavior
The F4 test cube should settle to a face-resting position (CoM z ~ 0.5, KE ~ 0) within 500 frames (8s). Add assertions:
- `KE_final < 0.01 J` (kinetic energy within numerical precision of zero)
- `|total_E - PE| < 0.01 J` (total energy equals potential energy)

### R4: No regression
All existing tests pass at 691/697 or better (same baseline as 0068).

---

## Files Likely Affected

| File | Change |
|------|--------|
| `msd/msd-sim/src/Physics/Collision/ContactCache.hpp` | Add sliding state to CachedContact |
| `msd/msd-sim/src/Physics/Collision/ContactCache.cpp` | Update/query sliding state |
| `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.hpp` | Add setSlidingMode(), aligned tangent basis |
| `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.cpp` | Implement sliding tangent basis |
| `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.hpp` | Accept per-contact tangent lower bounds |
| `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.cpp` | Add lambda_t1 >= 0 inequality constraint |
| `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp` | Wire sliding state to FrictionConstraint |
| `msd/msd-sim/test/Physics/Collision/RotationalEnergyTest.cpp` | Tightened F4 assertions |

---

## Reproduction

```bash
# Build and run
cmake --build --preset debug-sim-only
./build/Debug/debug/msd_sim_test --gtest_filter="*RotationalEnergyTest_F4*"

# Analyze recording
# Load: replay/recordings/ReplayEnabledTest_RotationalEnergyTest_F4_RotationEnergyTransfer_EnergyConserved.db
# Check body state at frames 300-500 for velocity sign changes
```

---

## Notes

- The sliding mode is a constraint-level fix, not a post-solve clamp — the solver respects it as a hard bound
- Static friction (sticking) is unaffected — sliding mode only activates after sustained sliding is detected
- The 0068 post-solve clamps (`clampPositiveWorkFriction`, `clampImpulseEnergy`) remain as safety nets for transient contacts
- If a sliding contact transitions to sticking (velocity drops below threshold), the mode reverts to bilateral friction

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-17 (ticket creation)
- **Completed**: 2026-02-17
- **Branch**: 0069-friction-velocity-reversal
- **PR**: N/A (not yet created)
- **Artifacts**:
  - `tickets/0069_friction_velocity_reversal.md`
- **Notes**: Bug fix ticket with implementation approach already specified. Design phase skipped as this is an extension of existing friction constraint system with clear requirements and no architectural changes needed.
