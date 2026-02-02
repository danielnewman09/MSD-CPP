# Ticket 0035dc: Angular Velocity Rest Threshold Adjustment

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
**Predecessor**: [0035da_post_impulse_energy_clamping](0035da_post_impulse_energy_clamping.md) — Energy clamping may make this unnecessary; evaluate after 0035da

---

## Summary

The angular velocity rest threshold (`angularVelocityRestThreshold = 0.1` rad/s) is too aggressive. It clamps angular velocity to zero even when a body has meaningful rotational energy from a legitimate contact impulse. This creates an energy pump effect: the contact solver creates angular velocity, the threshold clamps it, and the next contact re-creates it from scratch. This ticket reduces the threshold to match the linear velocity threshold or removes it in favor of the energy clamp from 0035da.

---

## Problem Statement

### Root Cause

In `WorldModel::updatePhysics()` (lines 160-163), angular velocity is clamped to zero when its magnitude is below 0.1 rad/s:

```cpp
const double angularVelocityMagnitude = state.getAngularVelocity().norm();
if (angularVelocityMagnitude < angularVelocityRestThreshold)
{
  state.setAngularVelocity(AngularRate{0.0, 0.0, 0.0});
}
```

For a unit cube (half-width 0.5m), 0.1 rad/s corresponds to a surface velocity of 0.05 m/s at the face center — a non-trivial speed. When a tilted body makes contact with a floor, the normal impulse creates angular velocity (e.g., `omega_y = -13.0` rad/s in diagnostic test D5). After the body bounces and the angular velocity decays below 0.1 rad/s, the threshold clamps it to zero. On the next contact, the solver sees a non-rotating body and applies a fresh rotational impulse, restarting the cycle.

This creates a pump effect where energy is repeatedly destroyed (by clamping) and re-created (by contact), preventing the body from smoothly settling to rest.

### Diagnostic Evidence

From `ObliqueContactDiagnosticTest::D5_TiltedBox_NoFriction_NoRestitution` (mu=0, e=0, tilt=0.1 rad):

| Step | omega_y | Note |
|------|---------|------|
| 0 | -13.000 | Normal impulse creates angular velocity |
| 1 | 0.000 | **Clamped by 0.1 rad/s threshold** (13.0 → 0.0) |
| 14 | 0.000 | Body in free flight, no angular velocity |
| 15 | 0.477 | Contact re-creates angular velocity |
| 16 | 0.701 | Growing |
| 17 | 0.913 | Growing |
| 18 | 1.117 | Growing |

The angular velocity at step 0 was -13.0 rad/s — far above the threshold. But after integration, the semi-implicit Euler step combined with zero angular velocity (clamped at step 1) means the rotation information is lost. Subsequent contacts then recreate rotation from zero.

### Comparison: Linear vs Angular Thresholds

| Parameter | Value | Physical equivalent |
|-----------|-------|-------------------|
| `velocity_rest_threshold_` | 0.01 m/s | Very small translational motion |
| `angularVelocityRestThreshold` | 0.1 rad/s | 0.05 m/s at face center for unit cube |

The angular threshold is 10x larger in relative terms, making it much more aggressive at destroying legitimate motion.

---

## Proposed Fix

### Option 1 (Recommended): Reduce threshold to match linear scale

```cpp
double angularVelocityRestThreshold{0.01};  // Match linear velocity threshold
```

This is the simplest change. It preserves the anti-jitter behavior while being much less aggressive about destroying legitimate angular velocity.

### Option 2: Remove angular velocity clamping entirely

If the energy clamp from 0035da is sufficient to prevent instability, the angular velocity rest threshold may be unnecessary. Remove the clamping block entirely:

```cpp
// Removed: angular velocity rest threshold
// Energy clamping (0035da) handles near-rest stability
```

### Option 3: Only apply threshold when body is at rest (no contacts)

Only clamp angular velocity when the body hasn't had contact forces applied this frame:

```cpp
if (angularVelocityMagnitude < angularVelocityRestThreshold
    && asset.getAccumulatedForce().norm() < 1e-12)
{
  state.setAngularVelocity(AngularRate{0.0, 0.0, 0.0});
}
```

### Recommendation

Start with Option 1 (reduce to 0.01). If the energy clamp from 0035da makes this threshold unnecessary, switch to Option 2.

---

## Acceptance Criteria

1. `ObliqueContactDiagnosticTest::D5_TiltedBox_NoFriction_NoRestitution` — angular velocity is NOT clamped to zero after a legitimate contact impulse
2. `FrictionValidationTest::StaticFrictionInclinedPlane_HoldsObject` — still passes (object at rest stays at rest)
3. No regressions in existing test suite
4. Near-rest jitter does not worsen compared to current behavior

---

## Key Files

| File | Change |
|------|--------|
| `msd-sim/src/Environment/WorldModel.hpp` | Reduce `angularVelocityRestThreshold` from 0.1 to 0.01 |
| `msd-sim/src/Environment/WorldModel.cpp` | No changes (threshold is used as-is) |

---

## Dependency Note

This ticket should be evaluated AFTER 0035da (energy clamping) is implemented. The energy clamp may make the angular velocity threshold less critical, in which case Option 2 (removal) becomes the preferred approach. If 0035da alone resolves the near-rest instability for tilted geometry, this ticket may be closed as "resolved by 0035da" with only the threshold reduction applied as a defensive measure.

---

## References

- [Debug session findings](../.debug-sessions/debug_oblique_contact_instability.md)
- Erin Catto, "Iterative Dynamics" (GDC 2005) — sleep thresholds
