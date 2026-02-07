# Design: Per-Contact Penetration Depth

## Summary

Extend the collision system to compute individual penetration depth values for each contact point in a manifold, replacing the shared `CollisionResult::penetrationDepth` value currently used by all contacts. When EPA clipping produces N contact points, each point gets its own depth computed by projecting it onto the contact normal relative to the reference face plane. The constraint factory then uses per-contact depth for the Baumgarte bias term, preventing N-times overcorrection that injects energy into the system.

## Problem Statement

In the current implementation, `CollisionResult` carries a single `penetrationDepth` value computed from the EPA polytope's closest face distance. When `EPA::extractContactManifold()` uses Sutherland-Hodgman clipping to generate up to 4 contact points, every contact constraint receives the same penetration depth for its Baumgarte stabilization term:

```
b_i = -(1 + e) * J_i * v_minus + (ERP / dt) * penetrationDepth
```

For a 4-contact manifold, the total Baumgarte correction is 4x what is geometrically correct. This overcorrection injects energy, causing resting objects to bounce, tilted cubes to amplify rocking, and long-running simulations to drift.

The fix is straightforward: compute depth at each contact point individually and feed that per-point depth into each contact constraint.

## Architecture Changes

### PlantUML Diagram

See: `./0040a-per-contact-penetration-depth.puml`

### Modified Components

#### ContactPoint (CollisionResult.hpp)

- **Current location**: `msd/msd-sim/src/Physics/Collision/CollisionResult.hpp`
- **Changes required**:
  1. Add `double depth{0.0}` field to `ContactPoint` struct
  2. Update constructor to accept optional depth parameter
- **Backward compatible**: Existing code constructing `ContactPoint{pointA, pointB}` still works; depth defaults to 0.0

**Implementation**:

```cpp
struct ContactPoint
{
  Coordinate pointA;   // Contact point on object A's surface (world space) [m]
  Coordinate pointB;   // Contact point on object B's surface (world space) [m]
  double depth{0.0};   // Penetration depth at this contact point [m]

  ContactPoint() = default;

  ContactPoint(Coordinate pA, Coordinate pB, double d = 0.0)
    : pointA{std::move(pA)}, pointB{std::move(pB)}, depth{d}
  {
  }

  ContactPoint(const ContactPoint&) = default;
  ContactPoint(ContactPoint&&) noexcept = default;
  ContactPoint& operator=(const ContactPoint&) = default;
  ContactPoint& operator=(ContactPoint&&) noexcept = default;
  ~ContactPoint() = default;
};
```

**Design Rationale**:
- Default `depth{0.0}` instead of NaN because depth=0 is a physically valid sentinel (no penetration at this point) and avoids propagating NaN through the Baumgarte term for contacts where depth is not computed (e.g., single-contact fallback paths).
- The `d = 0.0` default parameter in the constructor maintains backward compatibility with existing `ContactPoint{pointA, pointB}` construction throughout the codebase.

#### EPA::extractContactManifold() (EPA.cpp)

- **Current location**: `msd/msd-sim/src/Physics/Collision/EPA.cpp`
- **Changes required**:
  1. After Sutherland-Hodgman clipping and reference plane filtering, compute per-point depth before building `ContactPoint` instances
  2. Pass computed depth into `ContactPoint` constructor

**Current code** (lines 551-566 of EPA.cpp):

```cpp
// Build contact pairs: project incident points onto reference plane
for (size_t i = 0; i < count; ++i)
{
  const Coordinate& incPoint = finalPoints[i];
  double const dist = refNormalWorld.dot(incPoint) - refPlaneD;
  Coordinate const refPoint = incPoint - refNormalWorld * dist;

  if (refIsA)
  {
    contacts[i] = ContactPoint{refPoint, incPoint};
  }
  else
  {
    contacts[i] = ContactPoint{incPoint, refPoint};
  }
}
```

**Modified code**:

```cpp
// Build contact pairs: project incident points onto reference plane
for (size_t i = 0; i < count; ++i)
{
  const Coordinate& incPoint = finalPoints[i];
  double const dist = refNormalWorld.dot(incPoint) - refPlaneD;
  Coordinate const refPoint = incPoint - refNormalWorld * dist;

  // Per-contact depth: distance from incident point to reference plane
  // along the contact normal. The variable 'dist' is the signed distance
  // from the incident point to the reference face plane (negative = penetrating).
  // Depth is the magnitude of penetration (positive when overlapping).
  double const pointDepth = std::max(-dist, 0.0);

  if (refIsA)
  {
    contacts[i] = ContactPoint{refPoint, incPoint, pointDepth};
  }
  else
  {
    contacts[i] = ContactPoint{incPoint, refPoint, pointDepth};
  }
}
```

**Why `-dist`**: The `dist` variable is computed as `refNormalWorld.dot(incPoint) - refPlaneD`. For points on the reference face plane, `dist == 0`. For penetrating incident points (below the reference plane), `dist < 0`. Therefore, depth = `-dist` gives a positive penetration depth value. We clamp to 0.0 to handle any floating-point cases where a point barely survived the `dist <= epsilon_` filter but is technically above the plane.

**Degenerate fallback paths**: The two fallback paths in `extractContactManifold()` (lines 499-501 and 542-544) that produce a single EPA centroid contact point should use `epaFace.offset` as the depth, matching the existing `CollisionResult::penetrationDepth`:

```cpp
// Fallback: degenerate case
Coordinate const contactPoint = epaFace.normal * epaFace.offset;
contacts[0] = ContactPoint{contactPoint, contactPoint, epaFace.offset};
return 1;
```

**Update CollisionResult::penetrationDepth**: After the contact manifold loop, update `CollisionResult::penetrationDepth` to hold the maximum of all per-contact depths. This is done in `computeContactInfo()` after `extractContactManifold()` returns. The current code already sets penetration depth from `closestFace.offset`. We keep this as the overall penetration depth (R4 backward compatibility). An assertion can verify that `max(contactPoint.depth) <= closestFace.offset + epsilon` since EPA's polytope distance is an upper bound on any individual contact depth.

#### ContactConstraintFactory::createFromCollision() (ContactConstraintFactory.cpp)

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp`
- **Changes required**:
  1. Replace `result.penetrationDepth` with `contactPair.depth` when constructing each `ContactConstraint`

**Current code** (line 55):

```cpp
constraints.push_back(
  std::make_unique<ContactConstraint>(bodyAIndex,
                                      bodyBIndex,
                                      result.normal,
                                      contactPair.pointA,
                                      contactPair.pointB,
                                      result.penetrationDepth,
                                      comA,
                                      comB,
                                      effectiveRestitution,
                                      relVelNormal));
```

**Modified code**:

```cpp
constraints.push_back(
  std::make_unique<ContactConstraint>(bodyAIndex,
                                      bodyBIndex,
                                      result.normal,
                                      contactPair.pointA,
                                      contactPair.pointB,
                                      contactPair.depth,
                                      comA,
                                      comB,
                                      effectiveRestitution,
                                      relVelNormal));
```

This is a one-line change. The `ContactConstraint` constructor, its `penetration_depth_` member, the `getPenetrationDepth()` accessor, and the `ConstraintSolver::assembleContactRHS()` Baumgarte term all remain unchanged. The only difference is that each constraint now receives its own depth value instead of the shared maximum.

### Unchanged Components

| Component | Why Unchanged |
|-----------|---------------|
| `ContactConstraint` | Already accepts `penetrationDepth` parameter; no change needed |
| `ConstraintSolver::assembleContactRHS()` | Already reads `contact->getPenetrationDepth()` per constraint; correct behavior automatically follows |
| `CollisionResult` struct | `penetrationDepth` field kept as max-depth for backward compatibility (R4) |
| `CollisionHandler` | Orchestration unchanged |
| `GJK` | Detection unchanged |

## Data Flow

### Before (Shared Depth)

```
EPA polytope closest face
    │
    ├── penetrationDepth = face.offset    (single value)
    │
    └── extractContactManifold()
        ├── ContactPoint{pA, pB}          (no depth)
        ├── ContactPoint{pA, pB}          (no depth)
        ├── ContactPoint{pA, pB}          (no depth)
        └── ContactPoint{pA, pB}          (no depth)
                    │
                    ▼
        ContactConstraintFactory
        ├── Constraint(depth = penetrationDepth)   ← same for all 4
        ├── Constraint(depth = penetrationDepth)
        ├── Constraint(depth = penetrationDepth)
        └── Constraint(depth = penetrationDepth)
                    │
                    ▼
        Baumgarte total = 4 * (ERP/dt) * penetrationDepth   ← 4x overcorrection
```

### After (Per-Contact Depth)

```
EPA polytope closest face
    │
    ├── penetrationDepth = face.offset    (max depth, backward compat)
    │
    └── extractContactManifold()
        ├── ContactPoint{pA, pB, depth_0}  (individual depth)
        ├── ContactPoint{pA, pB, depth_1}
        ├── ContactPoint{pA, pB, depth_2}
        └── ContactPoint{pA, pB, depth_3}
                    │
                    ▼
        ContactConstraintFactory
        ├── Constraint(depth = depth_0)
        ├── Constraint(depth = depth_1)
        ├── Constraint(depth = depth_2)
        └── Constraint(depth = depth_3)
                    │
                    ▼
        Baumgarte total = (ERP/dt) * (d0 + d1 + d2 + d3)   ← proportional
```

For a flat-landing cube where all 4 contacts have equal depth d:
- Before: total = 4 * (ERP/dt) * d
- After: total = 4 * (ERP/dt) * d (same, because d_i = d for all i)

Wait -- this is the same. The key difference is for tilted cubes where contact depths differ. For a cube tilted so only one edge is deep:
- Before: total = 4 * (ERP/dt) * d_max (overcorrects at shallow contacts)
- After: total = (ERP/dt) * (d_deep + d_deep + d_shallow + d_shallow) (correct)

And for the general case, the Baumgarte correction at each contact point is proportional to the actual penetration at that specific location, rather than the maximum penetration anywhere in the manifold.

## Test Strategy

### Unit Tests (PerContactDepthTest.cpp)

| Test | Description | Verification |
|------|-------------|--------------|
| `SingleContact_DepthMatchesEPA` | Single contact point (edge case) | `contactPoint.depth` approximately equals `CollisionResult::penetrationDepth` |
| `FourContact_FlatLanding_EqualDepths` | Cube landing flat on plane | All 4 contact depths approximately equal |
| `TiltedCube_AsymmetricDepths` | Cube tilted 15 degrees landing on plane | Deeper-side contacts have larger depth than shallow-side contacts |
| `GrazingContact_NearZeroDepth` | Barely touching contact | Near-zero depth at edge contacts |
| `MaxDepth_MatchesPenetrationDepth` | Any multi-contact collision | `max(contactPoint.depth) <= CollisionResult::penetrationDepth` |
| `DepthNonNegative` | Any collision | All `contactPoint.depth >= 0` |

### Regression Tests

- All existing collision tests must continue to pass
- All 0039b linear collision tests (A1-A6, F1-F5) must pass
- Existing `ContactManifoldStabilityTest`, `EnergyAccountingTest`, `ParameterIsolationTest` must pass

### Integration Validation

The per-contact depth alone may not fix all failing tests (0040b split impulse is needed for that), but it should not cause any regressions and may improve some stability metrics.

## Risk Assessment

### Low Risk
- The change is minimal: one new field, one computation, one substitution
- Backward compatible: `CollisionResult::penetrationDepth` unchanged
- No new allocations or performance impact
- The depth computation reuses the `dist` variable already computed in the clipping loop

### Edge Cases
- **Single-contact manifold**: depth equals EPA penetration depth (no change in behavior)
- **All contacts at same depth**: total Baumgarte correction identical to before
- **Degenerate geometry**: Fallback paths produce single contact with EPA offset as depth

## Files Modified

| File | Change | Lines |
|------|--------|-------|
| `msd-sim/src/Physics/Collision/CollisionResult.hpp` | Add `depth` field to `ContactPoint` | ~3 |
| `msd-sim/src/Physics/Collision/EPA.cpp` | Compute per-point depth in `extractContactManifold()` | ~8 |
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` | Use `contactPair.depth` instead of `result.penetrationDepth` | 1 |

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/Collision/PerContactDepthTest.cpp` | Unit tests for per-contact depth computation |

## Performance Impact

Negligible. The depth computation adds one dot product and one subtraction per contact point, reusing the `dist` variable already computed in the existing clipping loop. No additional memory allocations.

## References

- Ticket: [0040a_per_contact_penetration_depth](../../../tickets/0040a_per_contact_penetration_depth.md)
- Parent ticket: [0040_collision_stabilization_phase2](../../../tickets/0040_collision_stabilization_phase2.md)
- Blocks: [0040b_split_impulse_position_correction](../../../tickets/0040b_split_impulse_position_correction.md)
- Related: Ericson (2004) "Real-Time Collision Detection", Section 5.5 (contact manifold generation)
