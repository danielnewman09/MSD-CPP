# Ticket 0035d4: Centroid Friction Constraint Factory Methods

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Quality Gate
**Assignee**: N/A
**Created**: 2026-02-01
**Type**: New Feature
**Requires Math Design**: No
**Generate Tutorial**: No
**Parent Ticket**: [0035d_friction_hardening_and_validation](0035d_friction_hardening_and_validation.md)

---

## Summary

Add factory methods to `ContactConstraintFactory` that create a single friction constraint and a single normal constraint at the centroid of the contact manifold. When a collision produces N contact points (e.g., 4 for a cube face on a plane), these methods reduce the friction representation to a single equivalent contact at the geometric center, producing a well-conditioned 3×3 system for the ECOS solver instead of the ill-conditioned 3N×3N system.

---

## Problem Statement

The current `createFrictionConstraints()` creates one `FrictionConstraint` per contact point. For face-face collisions with 4 contact points, this produces 4 friction constraints (8 Jacobian rows). Combined with 4 normal constraints (4 rows), the effective mass matrix is 12×12 and near-singular because the 4 contacts are geometrically redundant.

The centroid reduction creates 1 friction constraint + 1 normal constraint at the manifold centroid, producing a 3×3 effective mass matrix that is always well-conditioned.

---

## Technical Approach

### New Method 1: `createCentroidFrictionConstraint()`

**Signature:**
```cpp
std::unique_ptr<FrictionConstraint> createCentroidFrictionConstraint(
    size_t bodyAIndex,
    size_t bodyBIndex,
    const CollisionResult& result,
    const Coordinate& comA,
    const Coordinate& comB,
    double frictionCoefficientA,
    double frictionCoefficientB);
```

**Algorithm:**
1. Combine friction coefficients: `μ = sqrt(μA * μB)` (existing `combineFrictionCoefficient`)
2. Return nullptr if μ == 0.0 or contactCount == 0
3. Compute centroid of contact points:
   - `centroidA = (1/N) * Σ contacts[i].pointA`
   - `centroidB = (1/N) * Σ contacts[i].pointB`
4. Create single `FrictionConstraint` at centroid with combined μ

**Returns:** `std::unique_ptr<FrictionConstraint>` or nullptr

### New Method 2: `createCentroidContactConstraint()`

**Signature:**
```cpp
std::unique_ptr<ContactConstraint> createCentroidContactConstraint(
    size_t bodyAIndex,
    size_t bodyBIndex,
    const CollisionResult& result,
    const InertialState& stateA,
    const InertialState& stateB,
    const Coordinate& comA,
    const Coordinate& comB,
    double restitution);
```

**Algorithm:**
1. Return nullptr if contactCount == 0
2. Compute centroid (same as above)
3. Compute lever arms: `leverArmA = centroidA - comA`, `leverArmB = centroidB - comB`
4. Compute relative normal velocity at centroid via `computeRelativeNormalVelocity()`
5. Apply rest velocity threshold for restitution (same logic as `createFromCollision`)
6. Create single `ContactConstraint` at centroid

**Returns:** `std::unique_ptr<ContactConstraint>` or nullptr

### Centroid Computation

For a collision with N contacts:
```
centroidA.x = (1/N) * Σ contacts[i].pointA.x()
centroidA.y = (1/N) * Σ contacts[i].pointA.y()
centroidA.z = (1/N) * Σ contacts[i].pointA.z()
```
Same for centroidB. This can be extracted into a private helper `computeContactCentroid()`.

---

## Files

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp` | Add `createCentroidFrictionConstraint()` and `createCentroidContactConstraint()` declarations |
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` | Implement both methods + centroid helper |

### Test Files

| File | Changes |
|------|---------|
| `msd-sim/test/Physics/Constraints/ContactConstraintFactoryTest.cpp` | Add unit tests for centroid methods |

---

## Acceptance Criteria

- [x] **AC1**: `createCentroidFrictionConstraint()` returns a single FrictionConstraint at the centroid of a 4-contact manifold
- [x] **AC2**: `createCentroidFrictionConstraint()` returns nullptr when μ = 0
- [x] **AC3**: `createCentroidFrictionConstraint()` returns nullptr when contactCount = 0
- [x] **AC4**: `createCentroidContactConstraint()` returns a single ContactConstraint at the centroid
- [x] **AC5**: Centroid computation is correct: `p_centroid = (1/N) Σ p_i` for both A and B sides
- [x] **AC6**: Centroid FrictionConstraint has correct lever arms (centroid_point - com)
- [x] **AC7**: All existing tests pass (zero regressions)

---

## Dependencies

- **Requires**: [0035a](0035a_tangent_basis_and_friction_constraint.md) — FrictionConstraint class
- **Requires**: [0032](0032_contact_constraint_refactor.md) — ContactConstraint class
- **Blocks**: [0035d5](0035d5_two_phase_friction_solve.md) — WorldModel integration uses these methods

---

## Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Centroid point is not on either object surface | Low | Low | Acceptable — centroid is a geometric approximation for friction, not a physical contact point |
| Single-contact degenerate case (N=1) | Low | Low | Centroid equals the single point — no special handling needed |

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-01
- **Notes**: Part of centroid reduction plan. Adds factory methods that ticket 0035d5 will consume. Can be implemented and tested independently.

### Implementation Phase
- **Started**: 2026-02-01
- **Completed**: 2026-02-01
- **Artifacts**:
  - `msd/msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp` — Added createCentroidFrictionConstraint() and createCentroidContactConstraint() declarations
  - `msd/msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` — Implemented both methods with private computeContactCentroid() helper
  - `msd/msd-sim/test/Physics/Constraints/ContactConstraintFactoryTest.cpp` — Added 9 unit tests covering all acceptance criteria
- **Notes**:
  - Implemented straightforward factory methods following existing patterns from createFromCollision() and createFrictionConstraints()
  - Private helper computeContactCentroid() computes geometric centroid as arithmetic mean of contact points
  - Both methods return nullptr for degenerate cases (no contacts, zero friction)
  - Centroid methods apply same validation and logic as per-point methods (rest velocity threshold, friction coefficient validation)
  - All 9 new tests pass, no regressions in existing 8 ContactConstraintFactory tests
  - Implementation ready for quality gate
