# Ticket 0051: Restitution-Gravity Coupling Fix

## Status
- [x] Draft
- [ ] Ready for Investigation
- [ ] Investigation Complete
- [ ] Ready for Implementation
- [ ] Merged / Complete

**Current Phase**: Draft
**Assignee**: TBD
**Created**: 2026-02-09
**Generate Tutorial**: No
**Related Tickets**: [0047_face_contact_manifold_generation](0047_face_contact_manifold_generation.md) (introduced regressions), [0042_collision_numerical_stability](0042_collision_numerical_stability.md)
**Type**: Bug Fix

---

## Problem Statement

Ticket 0047 introduced gravity pre-apply (applying gravity to velocities before collision solving), which is the standard Box2D/Bullet approach for resting contact stability. However, this couples restitution with gravity in the constraint solver's RHS:

```
b = -(1+e) * J * (v + g*dt)
```

The extra `e * J * g * dt` term (~0.08 m/s for e=0.5, g=9.81, dt=0.016) causes:

1. **B3 regression**: Sphere acquires spurious rotation (1.48 rad/s vs 0.5 threshold) because off-center EPA contact point amplifies the restitution-gravity coupling into angular velocity
2. **H3 regression**: ERP energy injection pattern disappears at all timesteps (test expects specific ERP-driven energy pattern, but gravity pre-apply changes the energy dynamics). Actually better physical behavior, but test assertion fails.

### Affected Tests (2 tests)

| Test | Suite | Failure Mode | Root Cause |
|------|-------|-------------|------------|
| `B3_SphereOnFloor_NoRotation` | BasicCollision | omega = 1.48 rad/s (threshold 0.5) | Restitution amplifies gravity at off-center contact |
| `H3_TimestepSensitivity_EnergyConverges` | ParameterIsolation | 0% energy growth at all timesteps (expects ERP pattern) | Gravity pre-apply eliminates ERP-driven energy injection |

---

## Proposed Solution: Velocity-Bias Approach

Thread a separate velocity bias through the collision pipeline and constraint solver. The bias represents pre-applied gravity and should be added to the solver RHS **without** the `(1+e)` restitution factor:

```
b = -(1+e) * J * v_current  -  J * v_bias
```

Where `v_bias = g * dt` for each body.

This gives:
- Correct support force at rest: `b = -(1+e)*J*0 - J*g*dt = -J*g*dt` → non-zero lambda
- No restitution coupling: the `e` factor only multiplies the actual velocity, not the gravity bias

### Implementation Scope

1. Add `velocityBias` parameter to `ConstraintSolver::assembleRHS()` and `solve()`
2. Thread bias through `CollisionPipeline::execute()`
3. Compute per-body bias in `WorldModel::update()` from potential energies
4. Remove direct velocity mutation (gravity pre-apply) from WorldModel::update()

---

## Key Files

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp/cpp` | Add velocityBias to assembleRHS/solve |
| `msd-sim/src/Physics/Collision/CollisionPipeline.hpp/cpp` | Thread velocityBias through execute() |
| `msd-sim/src/Environment/WorldModel.cpp` | Compute bias, pass to pipeline, revert direct velocity mutation |

---

## Acceptance Criteria

1. **AC1**: B3 passes (sphere omega < 0.5 rad/s)
2. **AC2**: H3 passes or test is updated to reflect correct physical behavior
3. **AC3**: D1, D4, H1 continue to pass (no regression of 0047 fixes)
4. **AC4**: No regressions in other passing tests (689/693 baseline)

---

## Deliverables

### D1: Velocity-Bias Implementation
ConstraintSolver accepts and uses velocity bias in RHS assembly.

### D2: Test Results
Full test suite results showing B3, H3 fixed with no regressions.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

---

## Workflow Log

### Draft
- **Timestamp**: 2026-02-09
- **Action**: Created as follow-on from ticket 0047
- **Notes**: B3 and H3 regressions from gravity pre-apply in 0047. Root cause identified: restitution-gravity coupling in solver RHS.
