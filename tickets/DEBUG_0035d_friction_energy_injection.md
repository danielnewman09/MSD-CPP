# DEBUG TICKET: Friction Energy Injection Root Cause Analysis

## Status
- [x] Phase 1: Reproduce and Characterize
- [x] Phase 2: Isolate Energy Source
- [x] Phase 3: Root Cause Identification
- [ ] Phase 4: Fix and Validate

**Created**: 2026-02-01
**Parent Ticket**: [0035d_friction_hardening_and_validation](0035d_friction_hardening_and_validation.md)
**Trigger**: `FrictionEnergyTest::EnergyMonotonicDecreaseForSliding` — block launched off floor by first friction impulse

---

## Problem Statement

A block with initial lateral velocity sliding on a surface with friction should decelerate and lose kinetic energy. Instead, the ECOS SOCP friction solver produces a wildly incorrect impulse on the first timestep that launches the block off the floor and into free-fall, after which no further collisions occur.

### Observed Behavior (Phase 1 Characterization)

**Test setup:**
- 10 kg block (unit cube, 1.0m) at `z=0.49` (0.01m overlap with floor top at z=0.0)
- Floor: 10m cube at `z=-5.0`
- Initial velocity: `v = (10, 0, 0)` m/s (pure horizontal)
- Friction coefficient: `μ = 0.5`, restitution: `e = 0.0`
- Gravity: `(0, 0, -9.81)` m/s²
- 100 timesteps at 16ms each

**After step 1:**
```
vel = (6.87, 3.67, 4.91)      ← WRONG: gained vy and vz, should have only lost vx
pos = (0.11, 0.06, 0.57)      ← Block launched upward (pos_z increased from 0.49 to 0.57)
E_lateral = 303.37 J           ← Decreased from 500 J but with wrong direction
```

**After step 1, velocity is CONSTANT for all remaining steps** — the block left the floor and is in free-fall with no further collisions (only gravity decelerating vz).

**Expected (analytical):**
- `dv_x = -μ·g·dt = -0.5 × 9.81 × 0.016 = -0.0785 m/s`
- `dv_y = 0` (no lateral force in y)
- `dv_z ≈ 0` (normal force balances gravity)

**Actual:**
- `dv_x = -3.13` (40× expected magnitude)
- `dv_y = +3.67` (should be zero — spurious y-impulse)
- `dv_z = +4.91` (launched upward — spurious z-impulse)

---

## Phase 2: Isolation Results

### Test 1: Friction-off (μ = 0) — **Normal contacts work correctly**
```
vx = 10.0 constant, vy = 0.0, vz oscillates near 0 and damps
pos_z settles to ~0.487 (Baumgarte stabilization working)
E_lateral = 500.0 constant (no friction = no lateral energy change)
```
**Conclusion**: Without friction, the normal contact solver (Active Set Method) works correctly. The block stays on the floor. **The problem is in the friction solver path (ECOS).**

### Test 2: Gravity-off — **Same wrong impulse**
```
Step 1: vel = (6.87, 3.67, 5.06) — same wrong first impulse
Velocity constant thereafter (no gravity to change it, no collisions)
```
**Conclusion**: Gravity is not the cause. The ECOS solver produces wrong friction lambdas regardless of gravity.

### Test 3: No gravity + no friction — **Clean baseline**
```
vx = 10.0 constant, vy = 0.0, vz = 0.125 (small Baumgarte push)
Block drifts horizontally with constant velocity.
```
**Conclusion**: The physics engine without friction/gravity produces correct behavior.

### Test 4: Single-step energy audit
```
dE_total = -76.3 J (energy removed, but...)
dv = (-3.13, +3.67, +4.91)    ← Impulse applied in wrong directions
```

**Conclusion**: The ECOS solver returns lambda values that produce forces in all 3 directions, including spurious y and z components. The contact was with a horizontal floor — friction should only act in x (opposing initial velocity), not in y or z.

---

## Phase 3: Root Cause — Two Confirmed Bugs

### ROOT CAUSE 1: ECOS Problem Formulation is Fundamentally Wrong (H5 — CRITICAL)

**File**: [ECOSProblemBuilder.cpp:11-77](msd/msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.cpp#L11-L77)

The ECOS problem is formulated as:
```
min 0^T·x  s.t.  A·x = b,  G·x + s = 0,  s ∈ K_SOC
```

Where:
- `A_eq = A` (effective mass matrix, 3C × 3C, full rank)
- `b_eq = b` (RHS vector, 3C × 1)
- `G = diag(-μ, -1, -1, ...)` (cone constraint matrix)
- `c = 0` (zero objective)

**The problem**: This is an **over-constrained system**. With 3C equality constraints (`A·x = b`) and 3C variables, the solution is uniquely determined: `x = A⁻¹·b`. The cone constraints `s ∈ K_SOC` then become **infeasible** because the unique solution from the equality constraints will generally NOT satisfy the friction cone.

ECOS then returns a "closest" point or an infeasible indicator, producing incorrect lambda values.

**The correct formulation** is a Quadratic Program with conic constraints (SOCP):
```
min (1/2)·λ^T·A·λ - b^T·λ   s.t.  ||[λ_t1_i, λ_t2_i]|| ≤ μ_i·λ_n_i,  λ_n_i ≥ 0
```

This is equivalent to solving the contact LCP while respecting the friction cone. The A·λ = b relationship is the **KKT optimality condition** (gradient of the Lagrangian = 0), NOT an equality constraint. It emerges at the optimum, not as a hard constraint.

**To fix**: Reformulate as SOCP with:
- `c = -b` (linear objective from LCP: min -b^T·λ + (1/2)·λ^T·A·λ)
- NO equality constraints (`A_eq` removed)
- Quadratic term encoded via additional SOC constraint or objective reformulation
- Cone constraints remain as-is

Alternatively, since ECOS is a conic solver (not QP), the quadratic objective needs to be lifted into SOCP form using the standard epigraph reformulation.

### ROOT CAUSE 2: `anyPositive` Filter Drops Friction Forces (H1 — HIGH)

**File**: [ConstraintSolver.cpp:691-706](msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp#L691-L706)

```cpp
// Skip if all lambdas for this constraint are non-positive
bool anyPositive = false;
for (int row = 0; row < dim_i; ++row)
{
  if (lambda_i(row) > 0.0)
  {
    anyPositive = true;
    break;
  }
}
if (!anyPositive) continue;  // DROPS CONSTRAINT
```

For `FrictionConstraint` (dim=2), lambda values represent tangential impulses in `t1` and `t2` directions. **These are signed** — negative lambda means force in the -t direction. A friction force opposing motion will have the sign that opposes the tangential velocity, which can be negative in both components.

This filter was written for `ContactConstraint` (dim=1) where `λ ≥ 0` is a physical requirement (normal contact forces can only push, not pull). But for friction, it incorrectly drops valid forces.

**To fix**: Only apply the `anyPositive` check to `ContactConstraint`, not `FrictionConstraint`. For friction constraints, always apply the force (the cone constraint already bounds the magnitude).

---

## Diagnostic Evidence Summary

| Test | vx after step 1 | vy after step 1 | vz after step 1 | Block on floor? |
|------|-----------------|-----------------|-----------------|-----------------|
| With friction (mu=0.5) | 6.87 | 3.67 | 4.91 | NO — launched |
| Friction-off (mu=0) | 10.0 | 0.0 | -0.032 | YES — stable |
| Gravity-off + friction | 6.87 | 3.67 | 5.06 | NO — launched |
| No gravity + no friction | 10.0 | 0.0 | 0.125 | Drifts (no gravity) |

The friction solver (ECOS path) is the sole source of the incorrect impulse. The normal-only path (Active Set Method) works correctly.

---

## Fix Priority

1. **RC1 (ECOS Formulation)**: This is the primary bug. The ECOS problem needs to be reformulated as a proper SOCP/QP for the contact LCP with friction cones. This is a significant change to `ECOSProblemBuilder::build()` and potentially `solveWithECOS()`.

2. **RC2 (anyPositive filter)**: This is a secondary bug that would silently drop friction forces even if RC1 is fixed. Quick fix: add a type check to skip the filter for friction constraints.

---

## Diagnostic Test Files

- [FrictionEnergyDiagnosticTest.cpp](msd/msd-sim/test/Physics/FrictionEnergyDiagnosticTest.cpp) — 5 diagnostic tests created for this investigation

---

## Success Criteria

1. Root cause identified and documented with evidence — **DONE**
2. Energy monotonically non-increasing (within tolerance 1e-6 J/step) for the sliding block scenario
3. All existing tests pass (zero regressions)
4. Fix is minimal and targeted

---

## References

- **ECOS solver documentation**: https://github.com/embotech/ecos — SOCP standard form
- **Friction pipeline**: Tickets 0035a through 0035c
- **Contact constraint solver**: Ticket 0032, 0034
- **ECOS integration**: Ticket 0035b4
- **SOCP formulation for friction**: Todorov (2011), "A convex, smooth and invertible contact model"
- **LCP to SOCP lifting**: Lobo et al. (1998), "Applications of second-order cone programming"
