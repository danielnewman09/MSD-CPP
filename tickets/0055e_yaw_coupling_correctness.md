# Ticket 0055e: Yaw Coupling Physical Correctness Assessment

## Status
- [x] Draft
- [ ] Investigation Complete
- [ ] Implementation Complete (if needed)
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Investigation
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-11
**Branch**: TBD
**Parent Ticket**: [0055_tilted_cube_friction_direction](0055_tilted_cube_friction_direction.md)
**Dependencies**: [0055b](0055b_friction_direction_root_cause.md)
**Prototype**: No
**Generate Tutorial**: No

---

## Objective

Determine whether the spurious yaw rotation observed in the `Compound_NoSpuriousYaw` test is physically correct behavior for a corner-impact collision with friction, or whether it indicates a bug in the contact point placement or tangent basis computation.

---

## Problem Statement

When a tilted cube hits a flat floor at a corner, friction at the off-center contact point creates torque about the vertical axis (yaw). The `Compound_NoSpuriousYaw` test expects peak yaw rate < 0.05 rad/s, but the simulation produces ~0.315 rad/s.

**Key question**: Is this yaw physically real?

A cube tilted around X (pitch) hitting a floor at a single corner WILL have a contact point offset from the CoM in the XZ plane. Friction force in the XY plane at this offset point creates torque r × F that has a Z component (yaw). This is real physics — asymmetric contact geometry creates asymmetric forces.

However, the MAGNITUDE may be wrong if:
- The contact point location is incorrect (EPA single-point vs true contact geometry)
- The tangent basis directions are suboptimal for the contact geometry
- The friction coefficient is being applied incorrectly for single-point contacts

---

## Investigation Plan

### Phase 1: Analytical verification
1. Compute the expected yaw torque analytically for a 1m cube tilted at the test angles, given:
   - Contact point location (single corner)
   - Friction coefficient (μ=0.5)
   - Normal force magnitude
   - Contact sliding velocity
2. Compare analytical prediction against simulation output

### Phase 2: Contact point analysis
1. Instrument the collision pipeline to log the actual contact point location for the tilted cube scenario
2. Compare against the geometrically correct contact point (the lowest corner of the tilted cube)
3. Check if EPA's single contact point is at the right location

### Phase 3: Determine outcome
- **If yaw is physically correct**: Update `Compound_NoSpuriousYaw` test to allow realistic yaw magnitude. The test name itself may need revision — it should test for correct yaw, not zero yaw.
- **If yaw magnitude is wrong**: Identify the source of error and create a targeted fix (likely in contact point placement or tangent basis).

---

## Evidence from 0055c

- **Iteration 7**: Projecting lever arm onto tangent plane reduced peak yaw from 2.06 to 1.93 rad/s — negligible effect. The yaw is NOT primarily from the normal component of the lever arm.
- **Iteration 10**: Contact normal is stable at (0,0,-1), tangent basis stable at t1=(0,1,0), t2=(1,0,0). No wobbling or instability in the contact frame.
- **Iteration 11**: No-friction world has vy=0 always. Friction world gets vy=+0.10 and wz=-1.09 from the VERY FIRST collision frame. The yaw is directly caused by friction torque at the off-center contact.
- **Iteration 12b**: Capped coupled solve did not fix yaw (peak 0.315 vs threshold 0.05), confirming this is a direction issue not a magnitude issue.

---

## Expected Outcome

The initial expectation is that the yaw IS physically correct — a corner impact with friction should produce yaw. The likely resolution is updating the test expectations to match physically correct behavior.

---

## Acceptance Criteria

1. Clear determination: yaw is physically correct OR root cause of incorrect yaw identified
2. If correct: test updated with appropriate yaw tolerance
3. If incorrect: targeted fix with no regressions
