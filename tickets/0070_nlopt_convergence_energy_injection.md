# Ticket 0070: NLopt Friction Solver Convergence Failure Injects Energy

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
**Depends On**: [0068_nlopt_friction_cone_solver](0068_nlopt_friction_cone_solver.md), [0069_friction_velocity_reversal](0069_friction_velocity_reversal.md)

---

## Overview

When the NLopt friction cone solver fails to converge (hits 100-iteration limit), the unconverged solution injects energy into the simulation. This produces sustained energy growth during corner contacts where the effective mass matrix has strong normal-tangent coupling through rotational lever arms.

The existing post-solve `clampImpulseEnergy` safety net catches gross violations but cannot prevent the steady per-frame energy injection from unconverged solutions. The result is a sawtooth energy profile: gradual injection over 10-15 frames, then a large dissipation clamp, then injection resumes.

---

## Observed Behavior

In the `RotationalEnergyTest_F4` test (1m cube, tilted 45deg, e_cube=1.0, e_floor=0.5 (default), mu=0.5), during the corner-sliding phase (frames 210-240):

### Solver diagnostics (frames 210-222):

| Frame | Iterations | Converged | Residual | delta_E (J) |
|-------|-----------|-----------|----------|-------------|
| 210   | 16        | yes       | 2.58     | -0.009      |
| 211   | 100       | no        | 4.28     | -0.00005    |
| 212   | 100       | no        | 4.03     | +0.012      |
| 213   | 100       | no        | 3.80     | +0.023      |
| 214   | 100       | no        | 3.59     | +0.034      |
| 215   | 100       | no        | 3.40     | +0.045      |
| 216   | 100       | no        | 3.23     | +0.057      |
| 217   | 100       | no        | 3.09     | +0.069      |
| 218   | 100       | no        | 2.99     | +0.082      |
| 219   | 100       | no        | 2.86     | +0.094      |
| 220   | 100       | no        | 2.85     | +0.107      |
| 221   | 100       | no        | 2.86     | +0.120      |
| 222   | 31        | yes       | 2.54     | -0.833 (clamp) |

### Energy profile pattern:

1. NLopt hits 100-iteration limit without converging (residual stays 2-4)
2. Unconverged solution injects energy — **accelerating** each frame (0.01 -> 0.12 J/frame)
3. Post-solve `clampImpulseEnergy` eventually catches gross violation (-0.5 to -0.8 J clamp)
4. Cycle repeats

Total energy at frame 210: 7.30 J. By frame 221: 7.95 J (+0.65 J injected over 11 frames).

### Contact geometry during failures:

- Single off-center contact point (cube corner on floor)
- Large lever arm from contact to CoM (~0.7m)
- Strong rotational coupling in effective mass matrix (normal impulse changes tangent velocity through angular momentum)
- The coupled QP has poor conditioning for this geometry

---

## Root Cause Analysis

The NLopt coupled QP solves:

```
min  0.5 * lambda^T * A * lambda - b^T * lambda
s.t. lambda_n >= 0
     ||lambda_t|| <= mu * lambda_n  (friction cone)
```

For a single off-center contact, the 3x3 effective mass matrix `A = J * M_inv * J^T` has significant off-diagonal terms due to rotational coupling through the lever arm cross products. The condition number of A depends on the lever arm geometry.

When the cube is balanced on a corner with a large lever arm (~0.7m for a 1m cube), the off-diagonal coupling is strong enough that NLopt's COBYLA/SLSQP optimizer oscillates without converging within 100 iterations. The "best effort" solution at iteration 100 violates the KKT conditions (residual 2-4), and applying this solution injects energy.

### Why the clamp is insufficient:

The `clampImpulseEnergy` post-solve check compares total delta-KE against zero. But:
- Each individual frame's injection is small (0.01-0.12 J)
- The clamp threshold may not catch small injections
- Energy accumulates over many frames before a large enough violation triggers the clamp
- The clamp itself is a blunt instrument — it scales the entire impulse, which may not produce a physically correct result

---

## Potential Solutions

### Option A: Zero friction on convergence failure
When NLopt fails to converge, discard the friction solution and apply only the normal contact force (from the Active Set Method, which always converges). This is conservative but safe — the cube would slide without friction for those frames rather than gaining energy.

### Option B: Increase iteration limit
Raise from 100 to 500 or 1000. May help if the solver is slowly converging, but the residual data suggests it's oscillating rather than converging (residual bounces between 2.8-4.3).

### Option C: Better warm-starting
Use previous frame's friction lambdas as the NLopt initial point. Currently the solver starts from zero each frame. Warm-starting from a nearby solution might help convergence.

### Option D: Fallback to box friction
When NLopt fails, fall back to the box-constrained friction approximation (inscribed square in cone). The box approximation can be solved exactly via the Active Set Method, which always converges. Less accurate than the cone but energy-safe.

### Option E: Energy-aware convergence check
After NLopt returns, compute the energy change from the proposed solution. If energy would increase, scale or discard the friction component. More targeted than the current clamp.

---

## Requirements

### R1: No energy injection from unconverged solver
When the friction solver fails to converge, the applied impulse must not increase total system energy. Energy must be monotonically non-increasing during sustained contact.

### R2: Graceful degradation
The solution should degrade gracefully (reduced friction accuracy) rather than catastrophically (energy injection or simulation instability).

### R3: No regression on converged cases
All existing tests where the solver converges normally must maintain current behavior (691/697 baseline).

---

## Reproduction

```bash
cmake --build --preset debug-sim-only
./build/Debug/debug/msd_sim_test --gtest_filter="*RotationalEnergyTest_F4*"

# Load recording and check frames 210-230:
# replay/recordings/ReplayEnabledTest_RotationalEnergyTest_F4_RotationEnergyTransfer_EnergyConserved.db
```

---

## Files Likely Affected

| File | Change |
|------|--------|
| `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Handle NLopt non-convergence |
| `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.cpp` | Return convergence status |
| `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.hpp` | Add convergence info to result |

---

## Notes

- The sliding friction mode from ticket 0069 successfully brings the cube to rest by frame ~410, but the energy injection during the corner-sliding phase (frames 210-240) is non-physical
- The NLopt solver uses COBYLA (derivative-free) for the cone constraint — SLSQP or other gradient-based methods may converge better for this problem
- The Active Set Method for normal-only contacts always converges — the issue is specific to the coupled friction QP
- Post-solve clamps from ticket 0068 (`clampPositiveWorkFriction`, `clampImpulseEnergy`) remain as safety nets but should not be the primary defense

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-17
- **Completed**: 2026-02-17
- **Branch**: 0070-nlopt-convergence-energy-injection
- **Artifacts**:
  - `tickets/0070_nlopt_convergence_energy_injection.md`
- **Notes**: Issue discovered during investigation of 0069 corner-lifting behavior. Root cause is NLopt convergence failure during off-center corner contacts with strong rotational coupling.
