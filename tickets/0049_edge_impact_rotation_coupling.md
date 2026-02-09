# Ticket 0049: Edge Impact Rotation Coupling

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
**Related Tickets**: [0040c_edge_contact_manifold](0040c_edge_contact_manifold.md), [0047_face_contact_manifold_generation](0047_face_contact_manifold_generation.md)
**Type**: Investigation / Bug Fix

---

## Problem Statement

A cube rotated 45 degrees about the Y-axis (edge-down, edge parallel to Y-axis) dropped onto a flat floor produces **no rotation** upon impact. The angular velocity after collision is effectively zero (7.3e-8 rad/s), while the test expects at least 0.1 rad/s. Physically, an edge impact with a floor should convert some linear momentum into angular momentum — the contact point is directly below the center of mass, but the contact normal force applied at the edge creates a lever arm about the COM that should induce rotation.

### Affected Test (1 test)

| Test | Suite | Failure Mode | Current Values |
|------|-------|-------------|----------------|
| `B2_CubeEdgeImpact_PredictableRotationAxis` | RotationalCollisionTest | omega = 7.3e-8 rad/s (threshold 0.1) | Effectively zero rotation from edge impact |

### Test Setup

```cpp
// Cube rotated 45 degrees about Y-axis (edge-down)
Eigen::Quaterniond q{Eigen::AngleAxisd{M_PI / 4.0, Eigen::Vector3d::UnitY()}};
double const halfDiag2D = std::sqrt(2.0) / 2.0;
ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 1.0 + halfDiag2D}, q};
// Dropped from height with gravity, e=0.7
```

### Root Cause Hypotheses

**Hypothesis A: Contact normal aligned with COM**
The EPA contact normal for an edge-on-plane contact may be purely vertical (0, 0, 1), and the contact point may lie directly below the COM on the Z-axis. If `r x n = 0` (lever arm parallel to normal), no torque is generated. This would happen if the contact point is at the midpoint of the bottom edge, which is directly below the COM for a symmetric cube.

**Hypothesis B: Edge contact manifold offset insufficient**
Ticket 0040c added 2-point edge contact manifolds. If the 2 edge contact points are symmetric about the COM projection, their torques cancel (equal and opposite). The net torque would be zero even with correct lever arms.

**Hypothesis C: Face contact instead of edge contact**
Despite the 45-degree rotation, EPA/GJK may detect this as a face-face contact (the rotated bottom face of the cube vs the floor face), producing a manifold that doesn't create the expected off-center torque.

**Hypothesis D: Rotation axis mismatch**
The test expects rotation about Y (the edge-parallel axis), but the physics may produce rotation about X. If the contact forces resolve in a way that cancels the expected torque about Y but produces torque about X, the test's Y-specific check would still fail while total omega might be nonzero. However, the test checks total `omega.norm()` which is also ~0, ruling this out.

---

## Investigation Plan

### Phase 1: Diagnostic Instrumentation

1. Add temporary diagnostic output to `CollisionPipeline::execute()` for the B2 test scenario:
   - Contact normal direction
   - Contact point locations (world space)
   - Lever arms `r_A = contactPoint - COM_A`
   - Angular Jacobian components `(r_A x n)`
   - Lambda values from solver
2. Run B2 with diagnostics to determine which hypothesis applies

### Phase 2: Contact Geometry Analysis

1. For the 45-degree rotated cube on plane:
   - Compute expected contact point(s) analytically
   - Compute expected lever arm(s)
   - Compute expected torque direction
2. Compare analytical expectation with EPA output
3. Determine if the zero-torque is geometric (symmetric cancellation) or algorithmic (wrong manifold)

### Phase 3: Root Cause Determination

Based on Phase 1-2 findings:
- If **geometric symmetry**: The edge-down cube is inherently symmetric about the edge axis. Contact forces at the edge cannot create net torque in this configuration because the edge is the lowest point and the COM is directly above it. The test expectation may be physically incorrect.
- If **algorithmic**: Fix the contact manifold or torque computation as needed.

### Phase 4: Resolution

**If test expectation is wrong**: Update the test to reflect correct physics. An edge-down symmetric drop should NOT produce rotation because the contact is directly below the COM. Consider replacing with a test geometry that genuinely produces rotation from an edge impact (e.g., cube dropped with slight lateral offset, or asymmetric body).

**If algorithmic bug**: Fix the contact manifold generation or torque computation and verify the cube rotates as expected.

---

## Key Files

| File | Relevance |
|------|-----------|
| `msd-sim/src/Physics/Collision/EPA.cpp` | Contact manifold generation, `generateEdgeContacts()` |
| `msd-sim/src/Physics/Collision/CollisionPipeline.cpp` | ContactConstraint creation from manifold |
| `msd-sim/src/Physics/Constraints/ContactConstraint.cpp` | Lever arm computation `r_A x n` |
| `msd-sim/test/Physics/Collision/RotationalCollisionTest.cpp` | B2 test (line 259) |

---

## Acceptance Criteria

1. **AC1**: Root cause determined and documented (geometric symmetry vs algorithmic bug)
2. **AC2**: Either:
   - (a) `B2_CubeEdgeImpact_PredictableRotationAxis` passes with a fix, OR
   - (b) Test expectation corrected if current physics behavior is physically correct
3. **AC3**: Contact manifold diagnostics documented for the edge-on-plane configuration
4. **AC4**: No regressions in existing passing tests (675/681 baseline)

---

## Deliverables

### D1: Contact Geometry Diagnosis
Documented contact point locations, normals, and lever arms for the 45-degree cube edge impact.

### D2: Root Cause Report
Determination of whether the zero rotation is physically correct or a bug.

### D3: Fix or Test Correction
Either an algorithmic fix or updated test expectations with physical justification.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
