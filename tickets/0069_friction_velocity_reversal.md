# Ticket 0069: Friction Velocity Reversal During Sustained Contact

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Bug
**Priority**: High
**Created**: 2026-02-17
**Generate Tutorial**: No
**Parent Ticket**: None
**Depends On**: [0068_nlopt_friction_cone_solver](0068_nlopt_friction_cone_solver.md)

---

## Overview

During sustained low-speed contact (e.g., a tumbling cube sliding on a floor), the friction solver produces impulses that reverse the sliding direction. This is physically impossible — Coulomb friction can only decelerate sliding, never reverse it. The result is oscillatory velocity zero-crossings that keep the cube tumbling indefinitely instead of settling to rest.

---

## Observed Behavior

In the `RotationalEnergyTest_F4` test (1m cube, tilted 45°, dropped onto floor with e=1.0, mu=0.5):

- After initial bounce phase (~250 frames), cube enters sustained tumbling contact
- Linear velocity components (vx, vy) oscillate in sign on ~100-frame timescale
- At frame 500 (8s), cube still has KE ≈ 0.1 J instead of settling to rest
- Cube "balances on edge" (CoM z ≈ 0.66) rather than toppling to a stable face

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

1. **`clampPositiveWorkFriction`**: Zeros tangent lambdas if `λ_t · jv > 0` (friction accelerating). Also caps tangent impulses that would reverse velocity by >10% of pre-solve speed.

2. **`clampImpulseEnergy`**: Scales entire impulse if total ΔKE > 0 (prevents restitution energy injection on rotating contacts).

These clamps eliminated gross energy injection (F4 energy profile: 15.3 J → 6.6 J at frame 500) and fixed the A6 glancing collision test. But they don't prevent friction from reversing the sliding direction when the reversal is small per-frame but accumulates over many frames.

### Suspected mechanism

The NLopt coupled QP solves normal + tangent impulses simultaneously. Cross-coupling in the effective mass matrix A means:
- A normal impulse at an off-center contact changes the tangent velocity through lever-arm coupling
- The tangent impulse the solver computes is based on pre-solve tangent velocity
- After the coupled normal+tangent impulse is applied, the tangent velocity may have reversed

The post-solve clamp catches cases where the tangent lambda itself does positive work, but NOT cases where the normal lambda (through coupling) reverses the tangent velocity and the tangent lambda just happens to be small.

### Expected behavior

At low sliding speeds without impacts:
- Friction should monotonically decelerate to zero
- Once velocity reaches zero in a tangent direction, it should stay zero
- Cube should settle to a face-resting position within a few seconds

---

## Requirements

### R1: Diagnose velocity reversal mechanism
Instrument the solver to log, per-contact per-frame:
- Pre-solve tangent velocity (from b)
- Post-solve tangent velocity (from A*λ)
- Contribution of normal coupling vs tangent impulse to the velocity change

### R2: Prevent friction-driven velocity reversal
After the NLopt solve, ensure the post-solve tangent velocity at each contact point does not reverse sign relative to pre-solve. Options to investigate:
- Per-row tangent clamping using diagonal of A
- Decoupled normal-then-tangent solve (solve normal first, update velocity, then solve tangent)
- Tangent impulse capping: `|λ_t| ≤ |jv_t| / A[t,t]` (arrest, don't reverse)

### R3: Settling behavior
The F4 test cube should settle to a face-resting position (CoM z ≈ 0.5, KE ≈ 0) within 500 frames (8s). Add an assertion:
- `KE_final < 0.01 J` (kinetic energy within numerical precision of zero)
- `|total_E - PE| < 0.01 J` (total energy equals potential energy)

### R4: No regression
All existing tests pass at 691/697 or better (same baseline as 0068).

---

## Files Likely Affected

| File | Change |
|------|--------|
| `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Post-solve tangent velocity clamp |
| `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Updated clamp signature if needed |
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

- The issue may also affect other sustained-contact scenarios (stacking, resting contact)
- A decoupled normal-then-tangent solve would be the most principled fix but changes the solver architecture significantly
- The current post-solve clamps from 0068 are a necessary but insufficient step
