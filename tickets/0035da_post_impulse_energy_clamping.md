# Ticket 0035da: Post-Impulse Energy Clamping

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Quality Gate
**Assignee**: N/A
**Created**: 2026-02-02
**Type**: Bug Fix
**Requires Math Design**: No
**Generate Tutorial**: No
**Parent Ticket**: [0035d_friction_hardening_and_validation](0035d_friction_hardening_and_validation.md)
**Debug Ticket**: [DEBUG_0035d_oblique_contact_instability](DEBUG_0035d_oblique_contact_instability.md)

---

## Summary

Contact constraint forces inject energy into the system when a rotating or tilted body bounces on a surface. The restitution term in the contact RHS amplifies angular velocity contributions at the contact point, creating impulses that increase total kinetic energy at each bounce. This ticket adds a post-impulse energy clamp to Phase 5 of `WorldModel::updateCollisions()` to prevent net energy injection by constraint forces.

---

## Problem Statement

### Root Cause

The contact constraint RHS formula is:
```
b_i = -(1+e) * (J_i * v_minus) + (ERP/dt) * penetration
```

The Jacobian `J_i` includes angular velocity terms via the lever arm cross product:
```
J = [n^T, (r x n)^T, -n^T, -(r x n)^T]
```

For a rotating body, `J_i * v_minus` includes the `omega x r` contribution at the contact point, which can be much larger than the COM approach velocity. The `-(1+e)` factor amplifies this, and the resulting impulse creates both translational and rotational velocity changes. When the rotational energy gain exceeds the translational energy dissipation, net energy is injected.

This is the classic "restitution + angular velocity = energy injection" problem, documented in game physics literature (Bullet, Box2D, PhysX all implement fixes for this).

### Diagnostic Evidence

From `ObliqueContactDiagnosticTest::D2_TiltedBox_NoFriction` (mu=0, e=0.5, tilt=0.1 rad):

| Bounce | omega_y before | omega_y after | KE before | KE after | Energy change |
|--------|---------------|---------------|-----------|----------|---------------|
| 1 (step 0) | 0.0 | -18.95 | 501.25 | 345.99 | -155.25 J (OK) |
| 2 (step 76) | 2.34 | 10.0 | 164.7 | 152.5 | initially OK |
| 2→3 (steps 85-86) | -16.2 | 4.93 | 264.7 | 339.4 | +74.7 J (INJECTED) |

The second and subsequent bounces inject energy because the angular velocity contribution to the contact-point approach velocity is amplified by restitution.

This occurs at ALL tilt angles including tilt=0 (flat box with friction also shows 48 KE violations).

### Affected Tests

- `FrictionValidationTest::oblique_contact` — 32 KE violations
- Potentially: `FrictionStabilityTest`, `FrictionEnergyTest`

---

## Proposed Fix

### Location

`WorldModel::updateCollisions()` Phase 5 (force application), lines 567-634 in `WorldModel.cpp`.

### Algorithm

After computing the total constraint forces (ASM normal + ECOS friction) for each inertial body, and before applying them via `applyForce()`/`applyTorque()`:

```
For each inertial body k with non-zero constraint forces:
  1. Compute pre-impulse kinetic energy:
     KE_pre = (1/2) * m * |v|^2 + (1/2) * omega^T * I * omega

  2. Compute velocity change from constraint forces:
     dv = (F_total / m) * dt
     domega = I_inv_world * T_total * dt

  3. Compute post-impulse kinetic energy:
     v_post = v + dv
     omega_post = omega + domega
     KE_post = (1/2) * m * |v_post|^2 + (1/2) * omega_post^T * I * omega_post

  4. If KE_post > KE_pre (energy injection):
     scale = KE_pre / KE_post   (scale < 1)
     // Scale the velocity change, not the force, to preserve direction
     // Recompute forces from scaled velocity change:
     F_total *= scale
     T_total *= scale

  5. Apply (possibly scaled) forces:
     applyForce(F_total)
     applyTorque(T_total)
```

### Rationale

Contact forces are reactive — they respond to interpenetration and approach velocity. They should never inject energy into the system. An inelastic collision (e<1) must remove energy; even a perfectly elastic collision (e=1) must conserve it exactly. If the computed impulse would increase KE, the excess is an artifact of the discrete-time restitution formulation interacting with angular velocity.

### Risks

- The energy clamp may slightly over-dampen some correct physics (e.g., a body rotating into a wall could lose some valid rotational energy). This is acceptable because the alternative is unbounded energy growth.
- The clamp operates per-body, not per-contact-pair. For multi-body scenarios, energy could transfer between bodies in unexpected ways. For the single-body-on-floor case (this ticket's scope), per-body clamping is exact.

---

## Acceptance Criteria

1. `ObliqueContactDiagnosticTest::D2_TiltedBox_NoFriction` — zero KE increases exceeding 2% tolerance
2. `FrictionValidationTest::oblique_contact` — criterion (1): zero KE violations
3. No regressions in existing test suite (`msd_sim_test` passes)
4. Energy clamp fires only when needed (not on every timestep for well-behaved scenarios)

---

## Key Files

| File | Change |
|------|--------|
| `msd-sim/src/Environment/WorldModel.cpp` | Phase 5: Add energy clamping logic before `applyForce()`/`applyTorque()` |
| `msd-sim/src/Environment/WorldModel.hpp` | (No changes expected) |
| `msd-sim/test/Physics/ObliqueContactDiagnosticTest.cpp` | Diagnostic tests already created |

---

## References

- [Debug session findings](.debug-sessions/debug_oblique_contact_instability.md)
- [Related debug ticket: friction energy injection](DEBUG_0035d_friction_energy_injection.md)
- Erin Catto, "Computing Distance" (GDC 2010) — discusses restitution clamping
- Erwin Coumans, Bullet Physics — `btSequentialImpulseConstraintSolver` energy correction
