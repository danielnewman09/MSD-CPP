# Ticket 0035db: Friction Direction Clamping

## Status
- [x] Draft
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Ready for Implementation
**Assignee**: N/A
**Created**: 2026-02-02
**Type**: Bug Fix
**Requires Math Design**: No
**Generate Tutorial**: No
**Parent Ticket**: [0035d_friction_hardening_and_validation](0035d_friction_hardening_and_validation.md)
**Debug Ticket**: [DEBUG_0035d_oblique_contact_instability](DEBUG_0035d_oblique_contact_instability.md)
**Predecessor**: [0035da_post_impulse_energy_clamping](0035da_post_impulse_energy_clamping.md) — Energy clamping should be applied first

---

## Summary

The ECOS friction solver produces tangential impulses that can reverse the lateral motion direction of a sliding body. Friction should only oppose motion (decelerate), never reverse it. This ticket adds a post-solve direction clamp to the friction force application path so that friction forces are limited to bringing the body to rest in the sliding direction without overshooting into reversal.

---

## Problem Statement

### Root Cause

The ECOS SOCP solver computes friction lambda values that satisfy the friction cone constraint (`||[lambda_t1, lambda_t2]|| <= mu * lambda_n`), but the resulting impulse can exceed the momentum needed to stop the body. This causes the lateral velocity to reverse sign — the body begins moving in the opposite direction.

Physically, kinetic friction opposes relative sliding. Once the relative velocity reaches zero, the body transitions to static friction (no motion). The friction force should never overshoot past the zero-velocity point.

### Diagnostic Evidence

From `ObliqueContactDiagnosticTest::D6_TiltSweep`:

| Tilt (rad) | Direction reversals (with friction) | Direction reversals (no friction) |
|------------|-------------------------------------|-----------------------------------|
| 0.0 | 38 | 0 |
| 0.01 | 21 | 0 |
| 0.1 | 52 | 0 |
| 0.5 | 100 | 0 |

Direction reversals occur exclusively when friction is enabled. The `D2_TiltedBox_NoFriction` test (mu=0, e=0.5) shows zero direction reversals, confirming the friction solver is the sole source.

From `FrictionValidationTest::oblique_contact`:
- Initial velocity: `vx = +0.5 m/s`
- After first contact: `vx` reverses to negative values
- 52 direction reversals over 1000 steps

### Affected Tests

- `FrictionValidationTest::oblique_contact` — criterion (3): 52 direction reversals (expected: 0)

---

## Proposed Fix

### Location

`WorldModel::updateCollisions()` Phase 5 (force application), in `WorldModel.cpp`.

### Algorithm

After computing the ECOS friction body forces and before applying them, clamp the friction component so it cannot reverse the lateral velocity:

```
For each inertial body k with friction forces:
  1. Get current velocity: v = state.velocity
  2. Compute lateral velocity change from friction force:
     dv_friction = (F_friction / m) * dt
  3. For each lateral component (x, y):
     v_post_i = v_i + dv_friction_i
     If sign(v_post_i) != sign(v_i) and |v_i| > rest_threshold:
       // Friction would reverse this component — clamp to stop
       dv_friction_i_clamped = -v_i   (brings component exactly to zero)
  4. Compute scale factor from clamped vs original dv_friction
  5. Apply scale to both friction force and friction torque
  6. Apply clamped forces
```

### Alternative approach (simpler)

Check the dot product of the post-friction lateral velocity with the pre-friction lateral velocity. If negative (direction reversed), scale the entire friction force down:

```
v_lateral = (vx, vy)
dv_lateral = (F_friction_x / m * dt, F_friction_y / m * dt)
v_post_lateral = v_lateral + dv_lateral

If dot(v_post_lateral, v_lateral) < 0:
  // Find scale that brings dot product to zero (friction stops the body)
  // v_lateral + scale * dv_lateral should have zero projection onto v_lateral
  // dot(v_lateral + scale * dv_lateral, v_lateral) = 0
  // |v_lateral|^2 + scale * dot(dv_lateral, v_lateral) = 0
  // scale = -|v_lateral|^2 / dot(dv_lateral, v_lateral)
  scale = clamp(-dot(v_lateral, v_lateral) / dot(dv_lateral, v_lateral), 0.0, 1.0)
  F_friction *= scale
  T_friction *= scale
```

### Rationale

Coulomb friction opposes relative sliding motion. The maximum work done by friction is to bring relative sliding to zero (stick). Beyond that point, static friction holds the body at rest — it does not actively push the body in the reverse direction. Over-application of friction impulses is a common discrete-time artifact that produces non-physical behavior.

### Risks

- Clamping friction force also scales the friction torque, which may under-apply rotational friction in some edge cases. This is acceptable because the alternative (direction reversal) is a more severe physics violation.
- The clamp is based on the COM lateral velocity, not the contact-point velocity. For rapidly rotating bodies, the contact-point velocity may differ from the COM velocity. For the scenarios in this ticket (small tilt, moderate angular velocity), the COM approximation is sufficient.

---

## Acceptance Criteria

1. `FrictionValidationTest::oblique_contact` — criterion (3): zero direction reversals
2. `ObliqueContactDiagnosticTest::D1_FlatBox_WithFriction` — zero direction reversals
3. `FrictionValidationTest::KineticFrictionInclinedPlane_SlowsMotion` — still passes (friction still decelerates)
4. `FrictionValidationTest::SlidingDeceleration_ObjectStops` — still passes (box still stops)
5. No regressions in existing test suite

---

## Key Files

| File | Change |
|------|--------|
| `msd-sim/src/Environment/WorldModel.cpp` | Phase 5: Add friction direction clamping before `applyForce()`/`applyTorque()` |
| `msd-sim/test/Physics/ObliqueContactDiagnosticTest.cpp` | Diagnostic tests already created |

---

## References

- [Debug session findings](../.debug-sessions/debug_oblique_contact_instability.md)
- Erin Catto, "Iterative Dynamics with Temporal Coherence" (GDC 2005) — friction clamping
- Bullet Physics `btSolverBody::internalApplyImpulse()` — clamps friction to prevent reversal
