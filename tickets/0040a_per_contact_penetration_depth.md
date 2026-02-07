# Ticket 0040a: Per-Contact Penetration Depth

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Quality Gate
**Assignee**: TBD
**Created**: 2026-02-07
**Generate Tutorial**: No
**Parent Ticket**: [0040_collision_stabilization_phase2](0040_collision_stabilization_phase2.md)
**Dependencies**: None
**Blocks**: [0040b](0040b_split_impulse_position_correction.md)
**Type**: Implementation

---

## Overview

`CollisionResult` has a single `penetrationDepth` shared by all `ContactPoint`s in the manifold. When EPA clipping produces N contact points, each contact constraint gets the full penetration depth in its Baumgarte term, resulting in N× overcorrection. This ticket adds individual depth values to each contact point.

---

## Problem

### Current Behavior

In `EPA::extractContactManifold()`, Sutherland-Hodgman clipping produces multiple contact points from the reference/incident face pair. All of these points share the single `CollisionResult::penetrationDepth` value computed from the EPA polytope.

When `ContactConstraintFactory` creates constraints, each constraint uses this shared depth:

```
Baumgarte bias = (ERP / dt) * penetrationDepth
```

For a 4-contact manifold, the total correction is 4× what's needed, injecting energy.

### Desired Behavior

Each `ContactPoint` carries its own `depth` value computed by projecting the point onto the contact normal relative to the opposing surface. The constraint factory uses per-contact depth, so total correction is proportional to actual penetration at each point.

---

## Requirements

### R1: Add Depth Field to ContactPoint

Add a `depth` field to the `ContactPoint` struct:

```cpp
struct ContactPoint {
  Coordinate point;    // existing
  float depth{0.0f};  // NEW: penetration depth at this contact point [m]
};
```

### R2: Compute Per-Point Depth During Clipping

In `EPA::extractContactManifold()`, after Sutherland-Hodgman clipping produces the contact polygon, compute individual depth for each clipped point:

```
depth_i = (reference_point - contact_point_i) · contact_normal
```

Where:
- `reference_point` is a point on the reference face
- `contact_point_i` is the i-th clipped contact point
- `contact_normal` is the EPA contact normal (pointing from incident to reference)

The reference point can be any vertex of the reference face since all vertices lie on the same plane.

### R3: Use Per-Contact Depth in Constraint Factory

Modify `ContactConstraintFactory` to use `contactPoint.depth` instead of `collisionResult.penetrationDepth` when computing the Baumgarte bias term.

### R4: Backward Compatibility

Keep `CollisionResult::penetrationDepth` as the maximum depth across all contact points (useful for broad-phase and early-out checks). Per-contact depth supplements but does not replace it.

---

## Affected Tests

This change contributes to fixing (in combination with 0040b):

| Test | How This Helps |
|------|---------------|
| F4_RotationEnergyTransfer | Correct per-contact depth prevents N× overcorrection |
| F4b_ZeroGravity_RotationalEnergyTransfer | Same mechanism without gravity |
| H1_DisableRestitution_RestingCube | Reduced overcorrection reduces energy injection |
| H8_TiltedCube_FeedbackLoop | Asymmetric depth values break positive feedback |
| C2_RockingCube_AmplitudeDecreases | Correct depth per contact reduces rocking amplification |
| C3_TiltedCubeSettles_ToFlatFace | Reduced energy injection allows settling |
| D1_RestingCube_1000Frames | Less overcorrection reduces drift |
| D4_SmallJitter_NoAmplification | Proportional correction prevents amplification |
| B1_CubeCornerImpact | Correct depth reduces corner energy injection |

**Note**: This ticket alone may not fully fix these tests. The primary fix is 0040b (split impulse), but 0040b depends on per-contact depth for correct position correction.

---

## Implementation Approach

### Step 1: Modify ContactPoint

In `CollisionResult.hpp`:

```cpp
struct ContactPoint {
  Coordinate point;
  float depth{0.0f};
};
```

### Step 2: Compute Depth in EPA Clipping

In `EPA::extractContactManifold()`, after the Sutherland-Hodgman clipping loop:

```cpp
// After clipping produces contactPoints vector
Coordinate refPoint = referenceFace.vertices[0];  // Any point on reference face
for (auto& cp : contactPoints) {
  Vector3D diff{
    refPoint.x() - cp.point.x(),
    refPoint.y() - cp.point.y(),
    refPoint.z() - cp.point.z()
  };
  cp.depth = diff.x() * normal.x() + diff.y() * normal.y() + diff.z() * normal.z();
  if (cp.depth < 0.0f) cp.depth = 0.0f;  // Clamp negative depths
}
```

### Step 3: Update Constraint Factory

In `ContactConstraintFactory`, replace:
```cpp
float depth = collisionResult.penetrationDepth;
```
with:
```cpp
float depth = contactPoint.depth;
```

---

## Test Plan

### Unit Tests

```cpp
// Verify per-contact depth computation
TEST(PerContactDepth, SingleContact_DepthMatchesEPA)
// Single contact point depth should match CollisionResult::penetrationDepth

TEST(PerContactDepth, FourContact_FlatLanding_EqualDepths)
// Cube landing flat: all 4 contacts should have approximately equal depth

TEST(PerContactDepth, TiltedCube_AsymmetricDepths)
// Tilted cube: contacts on the "deeper" side should have larger depth

TEST(PerContactDepth, GrazingContact_NearZeroDepth)
// Contact points near the edge of the manifold should have near-zero depth

TEST(PerContactDepth, MaxDepth_MatchesPenetrationDepth)
// max(contactPoint.depth) should approximately equal CollisionResult::penetrationDepth
```

### Regression Tests

- All existing collision tests must pass
- All 0039b linear collision tests (A1-A6, F1-F5) must pass

---

## Acceptance Criteria

1. [x] **AC1**: `ContactPoint` has `depth` field
2. [x] **AC2**: EPA clipping computes per-point depth via normal projection
3. [x] **AC3**: `ContactConstraintFactory` uses per-contact depth for Baumgarte bias
4. [x] **AC4**: `CollisionResult::penetrationDepth` still holds maximum depth
5. [x] **AC5**: All existing collision tests pass (no regression) — 619/630 pass, 11 pre-existing failures unchanged
6. [x] **AC6**: Tilted cube manifold has asymmetric depth values (unit test)

---

## Files to Modify

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Collision/CollisionResult.hpp` | Add `depth` field to `ContactPoint` |
| `msd-sim/src/Physics/Collision/EPA.cpp` | Compute per-point depth during Sutherland-Hodgman clipping |
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` | Use `contactPoint.depth` instead of `collisionResult.penetrationDepth` |

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/Collision/PerContactDepthTest.cpp` | Unit tests for per-contact depth computation |

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-07
- **Notes**: Extracted from 0039e follow-on analysis. Addresses the N× overcorrection caused by shared penetration depth across multi-contact manifolds.

### Design Phase
- **Started**: 2026-02-07
- **Completed**: 2026-02-07
- **Artifacts**:
  - `docs/designs/0040a-per-contact-penetration-depth/design.md`
  - `docs/designs/0040a-per-contact-penetration-depth/0040a-per-contact-penetration-depth.puml`
- **Notes**: Design is minimal and focused. Three files modified (ContactPoint struct, EPA clipping loop, ContactConstraintFactory). The depth computation reuses the existing `dist` variable already computed in the Sutherland-Hodgman clipping loop, requiring only `-dist` and a clamp. No new public interfaces, no performance impact, full backward compatibility via `CollisionResult::penetrationDepth` retained as max depth. Ticket type is Implementation (no separate design review phase), advanced directly to Ready for Implementation.

### Implementation Phase
- **Started**: 2026-02-07
- **Completed**: 2026-02-07
- **Artifacts**:
  - `msd/msd-sim/src/Physics/Collision/CollisionResult.hpp` — Added `double depth{0.0}` to ContactPoint, updated constructor
  - `msd/msd-sim/src/Physics/Collision/EPA.cpp` — Per-contact depth computation in extractContactManifold() (3 locations)
  - `msd/msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` — Uses `contactPair.depth` instead of `result.penetrationDepth`
  - `msd/msd-sim/test/Physics/Collision/PerContactDepthTest.cpp` — 6 unit tests (all passing)
- **Notes**: Implementation follows design document exactly. Three source files modified: ContactPoint gets depth field with backward-compatible default constructor, EPA computes `pointDepth = std::max(-dist, 0.0)` per contact using existing `dist` variable, factory substitutes per-contact depth. All 6 new tests pass. 619/630 total tests pass; 11 failures are pre-existing from tickets 0039c/0039d (rotational collision, parameter isolation, contact manifold stability). No regressions introduced.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
