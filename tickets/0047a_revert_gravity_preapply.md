# Ticket 0047a: Investigate Reverting Gravity Pre-Apply

## Status
- [x] Draft
- [x] Ready for Investigation
- [x] Investigation Complete
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Investigation Complete
**Assignee**: TBD
**Created**: 2026-02-09
**Generate Tutorial**: No
**Related Tickets**: [0047_face_contact_manifold_generation](0047_face_contact_manifold_generation.md) (introduced the change), [0051_restitution_gravity_coupling](0051_restitution_gravity_coupling.md) (attempted to fix coupling side-effect)
**Type**: Investigation / Bug Fix

---

## Problem Statement

Ticket 0047 made two changes to fix resting contact stability (D1, D4, H1):
1. **SAT fallback** in `CollisionHandler` — validates EPA results against SAT minimum penetration
2. **Gravity pre-apply** in `WorldModel` — applies `v += g*dt` before collision solving, skips gravity in `updatePhysics`

The SAT fallback is a targeted fix for EPA producing wrong faces at zero penetration. The gravity pre-apply is a more invasive change that reorders the entire physics update loop.

### Symptoms observed in the executable

A cube dropped at a slight angle onto a flat floor exhibits:
1. **Spurious off-axis rotation** — torque appearing on axes where there should be none
2. **Failure to come to rest** — body oscillates indefinitely instead of settling

These are fundamental physics correctness issues that should not require friction to resolve. A frictionless cube dropped onto a frictionless floor should bounce, lose energy via restitution, and eventually settle with zero angular velocity about non-contact axes.

### Why the gravity pre-apply is suspect

The pre-apply changes the fundamental physics update order from:

```
solve_collisions(v) → integrate(v, F_gravity + F_contact)
```

to:

```
v_temp = v + g*dt → solve_collisions(v_temp) → integrate(v_temp, F_contact)
```

This means:
- The solver sees an artificial velocity that doesn't reflect the body's actual state
- The solver RHS includes `b = -(1+e)*J*(v+g*dt)`, coupling restitution with gravity
- Contact forces are computed against a velocity that hasn't actually been achieved yet
- The final integration step doesn't apply gravity (already "consumed"), making force accounting non-standard

The original motivation was that resting bodies with `v=0` produce `RHS = 0 → lambda = 0 → no support force`. But this should be solvable without mutating velocities — the solver needs to know about gravity, but not by pretending the body is already falling.

### What 0051 revealed

Ticket 0051 attempted to fix the coupling by threading a separate velocity bias to the solver (`b = -(1+e)*J*v - J*v_bias`). The implementation was test-neutral (689/693 on both main and 0051 branch), confirming:
- The 4 failures (B2, B3, B5, H3) are pre-existing on main
- The coupling term `e*J*g*dt` ≈ 0.08 m/s is small but non-zero
- The real issues may be elsewhere (contact manifold quality, solver force extraction)

---

## Investigation Plan

### Phase 1: Revert gravity pre-apply on a clean branch from main

1. Revert the velocity mutation loop added by 0047 in `WorldModel::update()`
2. Restore `updatePhysics()` to apply ALL forces (gravity + contact)
3. Keep the SAT fallback (that fix is independent and correct)
4. Run full test suite — document which tests regress (expect D1, D4, H1)

### Phase 2: Characterize the resting contact problem without pre-apply

With gravity pre-apply reverted, the resting body problem returns: `v=0 → RHS=0 → lambda=0 → no support → micro-bounce`. Characterize this precisely:
1. What is the actual frame-by-frame behavior? (velocity, position, contact state)
2. How many frames does the micro-bounce cycle take?
3. What is the magnitude of the oscillation?

### Phase 3: Investigate alternative approaches to resting contact support

The core issue is: how does the solver produce a support force when `v=0`? Standard approaches that don't mutate velocity:

1. **Baumgarte position correction in RHS**: Add a penetration-dependent term `b = -(1+e)*J*v - beta*C/dt` where `C` is penetration depth. We currently use PositionCorrector for this but it's a separate pass — incorporating it into the main RHS may produce the support force naturally.
2. **Predicted velocity**: Compute `v_pred = v + M^{-1}*F_ext*dt` and use that in the RHS without mutating state. This is what the velocity-bias approach (0051) attempted, but the key difference is that `updatePhysics` should NOT also apply gravity — the predicted velocity IS the gravity contribution.
3. **Split impulse**: Use a split-impulse approach where position correction and velocity correction are handled in separate solver passes.
4. **Contact sleeping/caching**: Detect resting contacts and apply cached support forces directly.

### Phase 4: Implement and validate the chosen approach

1. Implement the approach that best addresses micro-bounce without mutating velocities
2. Verify D1, D4, H1 pass
3. Verify no regressions from the 689/693 baseline
4. Verify the executable "smell test" — cube at slight angle settles cleanly

---

## Key Files

| File | Relevance |
|------|-----------|
| `msd-sim/src/Environment/WorldModel.cpp` | Contains the gravity pre-apply loop and physics update ordering |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | `assembleRHS()` — where the solver RHS is constructed |
| `msd-sim/src/Physics/Collision/CollisionPipeline.cpp` | Full collision pipeline including PositionCorrector |
| `msd-sim/src/Physics/Collision/CollisionHandler.cpp` | SAT fallback (keep this, independent fix) |
| `msd-sim/test/Physics/Collision/ContactManifoldStabilityTest.cpp` | D1, D4 tests |
| `msd-sim/test/Physics/Collision/ParameterIsolationTest.cpp` | H1, H3 tests |

---

## Acceptance Criteria

1. **AC1**: Gravity pre-apply removed from `WorldModel::update()` — gravity applied only in `updatePhysics()`
2. **AC2**: SAT fallback in `CollisionHandler` retained (independent of this change)
3. **AC3**: Resting contact works correctly — D1, D4, H1 pass via the chosen alternative approach
4. **AC4**: No regressions from 689/693 baseline
5. **AC5**: Executable smell test — cube at slight angle settles to rest without spurious off-axis rotation

---

## Deliverables

### D1: Investigation Report
Document the frame-by-frame behavior of resting contact without pre-apply, characterize the micro-bounce, and evaluate alternative approaches.

### D2: Implementation
Chosen approach implemented, tested, and validated.

### D3: Test Results
Full test suite results showing no regressions and improved executable behavior.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

---

## Workflow Log

### Draft
- **Timestamp**: 2026-02-09
- **Action**: Created to investigate reverting the gravity pre-apply introduced in ticket 0047
- **Notes**: The gravity pre-apply was one of two changes in 0047. The SAT fallback is a targeted correct fix; the pre-apply is an invasive ordering change that may be causing spurious rotation and failure to rest. Ticket 0051 attempted to fix the coupling side-effect but was test-neutral, suggesting the real issues lie elsewhere. This ticket investigates whether removing the pre-apply and using a cleaner approach to resting contact support produces better overall physics behavior.

### Investigation Phase (Phase 1)
- **Started**: 2026-02-09
- **Completed**: 2026-02-09
- **Branch**: 0047a-revert-gravity-preapply
- **Commit**: b1418df
- **Artifacts**:
  - `docs/investigations/0047a_revert_gravity_preapply/phase1-results.md`
  - Modified: `msd/msd-sim/src/Environment/WorldModel.cpp` (revert implemented)
- **Results**: **CRITICAL FINDING — D1 and H1 still pass WITHOUT gravity pre-apply**
  - Test suite: 689/693 (same as main)
  - Regressions: D4 (micro-jitter, test design issue)
  - Fixes: B3 (sphere rotation, pre-apply introduced spurious torque)
  - Unchanged: D1, H1, H3, B2, B5
- **Key Insight**: The SAT fallback is the true fix for resting contacts. Gravity pre-apply was unnecessary complexity that introduced restitution-gravity coupling and B3 regression.
- **Notes**: Phase 1 objective complete. The original motivation for ticket 0047 (D1/H1 failures without pre-apply) is contradicted by test results — both tests pass without it. The SAT fallback provides correct contact manifolds at zero penetration, enabling non-zero support forces even when v≈0. Gravity pre-apply should be removed permanently, keeping only the SAT fallback.
