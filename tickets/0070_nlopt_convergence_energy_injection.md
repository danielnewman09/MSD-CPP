# Ticket 0070: Decouple Normal/Friction Solve to Eliminate Energy Injection

## Status
- [x] Draft
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Ready for Implementation
**Type**: Bug / Architecture
**Priority**: High
**Created**: 2026-02-17
**Generate Tutorial**: No
**Parent Ticket**: None
**Depends On**: [0068_nlopt_friction_cone_solver](0068_nlopt_friction_cone_solver.md)

---

## Overview

The coupled normal+friction QP solver injects energy because its objective function targets restitution bounce velocity, not energy conservation. When the friction cone constraint is active with rotational coupling (off-diagonal terms in the effective mass matrix from lever arms), the cone-constrained solution diverges from the energy-conserving one. This is a formulation-level problem — post-solve clamps cannot fix it.

The fix is to decouple normal and friction into sequential solves, which is the standard approach used by Box2D, Bullet, and most production physics engines.

---

## Root Cause

### The QP objective ≠ energy conservation

The coupled friction solver minimizes:

```
min  0.5·λᵀAλ + bᵀλ
s.t. λ_n ≥ 0, ||λ_t|| ≤ μ·λ_n
```

where the RHS encodes:
- Normal rows: `b_n = -(1+e)·Jv` (targets restitution bounce velocity)
- Tangent rows: `b_t = -Jv` (targets zero tangent velocity)

The energy change from applying impulse λ is:

```
ΔKE = λᵀ·Jv + 0.5·λᵀAλ
```

Note: the QP objective uses `b` (with the `(1+e)` factor on normals), but ΔKE uses `Jv` (without it). **The QP minimizes velocity error relative to a bounce target, not energy.**

### Why unconstrained solve is energy-safe

For the normal-only unconstrained case: `λ = -(1+e)·Jv / A_nn`

```
ΔKE = v²/A · (1+e) · (e-1)/2
```

- e=1: ΔKE = 0 (energy conserved) ✓
- e<1: ΔKE < 0 (energy dissipated) ✓

### Why coupled solve injects energy

When the friction cone constraint `||λ_t|| ≤ μ·λ_n` is active AND A has off-diagonal normal-tangent coupling (from rotational lever arms), the cone pushes λ away from the unconstrained optimum. The shifted λ achieves a different velocity than the unconstrained one, and the mismatch between what the QP optimizes (velocity error with `(1+e)` factor) and what physics requires (energy conservation against `Jv`) means the cone-constrained solution can inject energy.

This is observable in two scenarios:
1. **F4 tumbling cube**: NLopt fails to converge during corner contact (frames 210-240), unconverged solution injects 0.01-0.12 J/frame
2. **A6 glancing collision**: Even when converged, the coupled solution injects 0.286 J in a single frame because the cone constraint distorts the restitution impulse

### Why clamps are insufficient

- `clampImpulseEnergy`: Scales the entire λ vector, destroying both normal and friction components. Physically incorrect — produces neither correct bounce nor correct friction.
- `clampPositiveWorkFriction`: Per-contact tangent check. Misses energy injection from normal-tangent coupling through the off-diagonal A terms.
- Both are reactive (fix after the fact) rather than preventive (don't generate bad impulses).

---

## Solution: Decoupled Normal-Then-Friction Solve

Split the single coupled QP into two sequential solves:

### Step 1: Solve normals with ASM (existing code)

```
min  0.5·λ_nᵀ·A_nn·λ_n + b_nᵀ·λ_n
s.t. λ_n ≥ 0
```

- ASM always converges (finite iterations, exact LCP)
- Energy-correct by construction: no friction cone to distort the restitution impulse
- This is the existing no-friction code path

### Step 2: Update velocities with normal impulse

```
v_post_normal = v_pre + M⁻¹·Jᵀ·λ_n
```

Compute the post-normal velocity in constraint space for friction rows.

### Step 3: Solve friction per-contact

With λ_n known from step 1, the friction cone `||λ_t|| ≤ μ·λ_n` becomes a ball constraint with **known radius** `r = μ·λ_n`. The friction problem per-contact is:

```
min  0.5·λ_tᵀ·A_tt·λ_t + b_t_updatedᵀ·λ_t
s.t. ||λ_t|| ≤ μ·λ_n
```

where `b_t_updated = -Jv_t_post_normal` (tangent velocity after normal impulse, no `(1+e)` anywhere).

This is analytically solvable per-contact:
1. Compute unconstrained optimum: `λ_t* = -A_tt⁻¹·b_t_updated`
2. If `||λ_t*|| ≤ μ·λ_n`: use `λ_t*` (inside cone)
3. Else: project onto ball boundary: `λ_t = λ_t* · (μ·λ_n / ||λ_t*||)`

### Why this is energy-safe

- Normal solve: energy-correct by the math shown above
- Friction solve: `b_t = -Jv_t` with no restitution factor. The unconstrained friction optimum zeros the tangent velocity (maximum dissipation). The ball projection can only reduce the friction magnitude → less dissipation, never injection. Friction can only decelerate sliding.
- No cross-coupling: the `(1+e)` restitution factor lives entirely in the normal solve, isolated from the friction cone constraint

### What this removes

- `clampImpulseEnergy` — unnecessary, formulation is energy-safe
- `clampPositiveWorkFriction` — unnecessary, friction can only decelerate
- NLopt dependency for friction — replaced by analytic per-contact projection
- NLoptFrictionSolver class — no longer needed (NLopt still used if needed elsewhere)

---

## Requirements

### R1: No energy injection from friction
The friction impulse must not increase total system kinetic energy. Energy must be monotonically non-increasing from friction during sustained contact.

### R2: Correct restitution behavior
Normal impulse must produce correct restitution bounce. Energy conserved for e=1, dissipated for e<1.

### R3: Friction cone satisfied
Per-contact: `||λ_t|| ≤ μ·λ_n`. The exact second-order cone, not a box approximation.

### R4: No regression
All existing tests maintain current behavior (690/697 baseline).

---

## Implementation Plan

### Step 1: Add velocity update helper

In `ConstraintSolver`, add a method to compute post-impulse velocities in constraint space:

```cpp
Eigen::VectorXd computePostImpulseVelocity(
  const Eigen::MatrixXd& A,
  const Eigen::VectorXd& preVelocity,
  const Eigen::VectorXd& lambda);
```

### Step 2: Split solve() friction path

Replace the current `solveWithFriction()` call with:
1. Extract normal-only subproblem (rows where `rowType == Normal`)
2. Solve normal with ASM
3. Compute post-normal tangent velocities via `A` and `λ_n`
4. Per-contact: solve 2D friction with ball projection
5. Assemble combined λ vector

### Step 3: Remove clamp functions

Remove `clampImpulseEnergy` and `clampPositiveWorkFriction` — they are no longer needed.

### Step 4: Remove NLoptFrictionSolver dependency (optional, deferred)

The NLoptFrictionSolver class is no longer called from the main solve path. It can be removed in a cleanup ticket if desired.

---

## Files Affected

| File | Change |
|------|--------|
| `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Decoupled solve, remove clamps |
| `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Updated method signatures |

---

## Verification

```bash
cmake --build --preset debug-sim-only
./build/Debug/debug/msd_sim_test --gtest_filter="*RotationalEnergyTest_F4*"
./build/Debug/debug/msd_sim_test --gtest_filter="*A6*"
./build/Debug/debug/msd_sim_test  # Full suite: 690/697 baseline
```

For F4 recording, check frames 206-240:
- `delta_e` ≤ 0 at every frame (no energy injection)
- No sawtooth pattern — smooth dissipation

---

## Notes

- The decoupled approach is standard in production physics engines (Box2D, Bullet, PhysX)
- The per-contact analytic friction projection is O(1) per contact — faster than NLopt
- Multi-contact friction coupling (contacts on the same body sharing the A matrix) is handled by iterating the per-contact projection a few times (Gauss-Seidel style), or simply solving independently per-contact as a first pass
- The velocity update between normal and friction solves is the key insight: friction sees post-bounce velocities, so it can never interfere with restitution

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-17
- **Completed**: 2026-02-17
- **Branch**: 0070-nlopt-convergence-energy-injection
- **Artifacts**:
  - `tickets/0070_nlopt_convergence_energy_injection.md`
- **Notes**: Originally diagnosed as NLopt convergence failure. Root cause analysis revealed a formulation-level problem: the coupled QP objective targets restitution velocity, not energy conservation, and the friction cone constraint distorts the solution when rotational coupling is present. Fix is to decouple normal and friction solves, which is the standard approach in production engines.

### Ready for Implementation Phase
- **Started**: 2026-02-17
- **Branch**: 0070-nlopt-convergence-energy-injection
- **PR**: N/A (will be created during implementation)
- **Notes**: Ticket specifies no math design required. Requirements, root cause analysis, and implementation plan are complete. Ready for cpp-implementer agent to execute decoupled normal-then-friction solve.

### Implementation Phase (Partial - Escalation Required)
- **Started**: 2026-02-17
- **Completed**: 2026-02-17 (partial)
- **Commit**: 3ad6a78
- **Branch**: 0070-nlopt-convergence-energy-injection
- **PR**: N/A
- **Artifacts**:
  - `docs/designs/0070-nlopt-convergence-energy-injection/implementation-notes.md`
  - `docs/designs/0070-nlopt-convergence-energy-injection/iteration-log.md`
- **Test Results**: 685/697 (baseline: 690/697)
  - Fixed: F4 tumbling cube energy injection
  - Regressions: A4, A6, D4, H5, H6 (5 new failures)
- **Status**: PARTIAL IMPLEMENTATION - ESCALATION REQUIRED
- **Issue**: Per-contact independent friction solve causes regressions due to ignoring multi-contact coupling through A matrix. A6 worse than baseline (1.39J vs 0.286J claimed).
- **Next Steps**: Human decision needed on:
  1. Gauss-Seidel iteration over contacts (simple, may converge slowly)
  2. Joint tangent SOCP solve (exact, requires NLopt or similar)
  3. Hybrid approach (G-S with SOCP fallback)
- **Notes**: Normal solve works correctly and fixes F4. Friction formulation needs refinement to handle multi-contact coupling. See implementation-notes.md for detailed analysis and recommendations.
