# Ticket 0050: Asymmetric Hull Test Geometry Correction

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
**Related Tickets**: [0047_face_contact_manifold_generation](0047_face_contact_manifold_generation.md), [0049_edge_impact_rotation_coupling](0049_edge_impact_rotation_coupling.md)
**Type**: Investigation / Test Fix

---

## Problem Statement

Test `B5_LShapeDrop_RotationFromAsymmetricCOM` expects an L-shaped body to rotate upon impact with a floor due to its asymmetric center of mass. However, the body exhibits **zero rotation** (omega = 3.4e-15 rad/s). The test itself acknowledges this possibility in its diagnostic comment: "convex hull may fill the L-shape, reducing asymmetry."

### Affected Test (1 test)

| Test | Suite | Failure Mode | Current Values |
|------|-------|-------------|----------------|
| `B5_LShapeDrop_RotationFromAsymmetricCOM` | RotationalCollisionTest | omega = 3.4e-15 rad/s (threshold 0.01) | Effectively zero rotation |

### Root Cause

The `createLShapePoints()` helper generates points intended to form an L-shaped body. However, `ConvexHull` computes the **convex hull** of these points, which fills in the concavity of the L-shape, resulting in a roughly rectangular body. The convex hull's COM is nearly centered, so when the body drops onto the floor, the contact forces are symmetric about the COM and no net torque is produced.

This is a **fundamental limitation of convex hull representation** — concave shapes cannot be represented by a single convex hull. The test geometry needs to be redesigned to use a shape that is:
1. Genuinely convex
2. Has a measurably asymmetric mass distribution relative to its bottom face

### Test Comment (from source)

```
// NOTE: Whether or not rotation occurs depends on whether the convex hull
// of the L-shape points creates a shape with sufficiently offset COM.
// The convex hull may "fill in" the L-shape, making it more symmetric.
// If no rotation is observed, that may indicate the convex hull COM is
// close enough to the geometric center of the convex hull that torques
// cancel out. This is still a valid diagnostic result.
```

---

## Investigation Plan

### Phase 1: Verify Convex Hull Geometry

1. Compute the actual convex hull of `createLShapePoints()` and visualize/log the resulting vertices
2. Compute the COM of the convex hull
3. Compute the bottom face contact region
4. Measure the offset between the bottom face centroid and the COM projection — confirm it's near zero

### Phase 2: Design Alternative Test Geometry

Design a convex shape that genuinely has an asymmetric COM relative to its contact surface. Options:

**Option A: Wedge/Ramp shape**
A triangular prism (wedge) with a flat bottom has its COM offset toward the thicker end. When dropped on the flat bottom, the normal force centroid is at the center of the bottom face, but the COM is offset, creating net torque.

**Option B: Truncated tetrahedron**
A tetrahedron with one face truncated to create a small flat bottom. The COM is above the original apex, far from the center of the small flat base.

**Option C: Asymmetric box (heavy side)**
Use the existing cube geometry but shift the convex hull points so one side is wider than the other (e.g., a trapezoidal prism). The convex hull COM will be offset toward the wider side.

**Option D: L-shape via compound test**
Instead of a single convex hull, test torque generation using a single object with an intentionally offset spawn position relative to the contact surface (not centered above it). This tests the lever arm mechanism without requiring an asymmetric hull.

### Phase 3: Implement and Validate

1. Replace `createLShapePoints()` usage in B5 with the chosen geometry
2. Compute the expected rotation direction analytically
3. Verify the test passes with meaningful rotation values
4. Verify no regressions

---

## Key Files

| File | Relevance |
|------|-----------|
| `msd-sim/test/Physics/Collision/RotationalCollisionTest.cpp` | B5 test (line 464), `createLShapePoints()` helper |
| `msd-sim/src/Physics/RigidBody/ConvexHull.hpp`, `ConvexHull.cpp` | Convex hull computation, COM, geometry |

---

## Acceptance Criteria

1. **AC1**: Actual convex hull geometry of `createLShapePoints()` documented (vertex list, COM, bottom face centroid)
2. **AC2**: New test geometry designed that is genuinely convex with measurable COM asymmetry
3. **AC3**: `B5_LShapeDrop_RotationFromAsymmetricCOM` passes with updated geometry (omega > 0.01 rad/s)
4. **AC4**: Test validates the correct physical behavior: asymmetric COM produces rotation upon flat-bottom impact
5. **AC5**: No regressions in existing passing tests (675/681 baseline)

---

## Deliverables

### D1: Geometry Analysis
Document the current L-shape convex hull geometry showing why it's symmetric.

### D2: New Test Geometry
A convex shape with proven COM asymmetry and predicted rotation behavior.

### D3: Updated Test
B5 test passing with the new geometry and meaningful physics validation.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
