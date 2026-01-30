# Ticket 0032c: WorldModel Contact Integration

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete — Awaiting Tutorial
- [ ] Merged / Complete

**Current Phase**: Ready for Implementation
**Assignee**: cpp-implementer
**Created**: 2026-01-29
**Generate Tutorial**: No
**Parent Ticket**: [0032_contact_constraint_refactor](0032_contact_constraint_refactor.md)

---

## Summary

Refactor `WorldModel::update()` to replace the standalone `CollisionResponse` namespace with the constraint-based contact pipeline. The collision detection output (GJK/EPA) feeds into `ContactConstraintFactory` to create transient `ContactConstraint` objects, which are solved via `ConstraintSolver::solveWithContacts()`. This is the integration ticket that wires the infrastructure (0032a) and solver (0032b) into the simulation loop.

---

## Motivation

The current `WorldModel` calls `CollisionResponse::applyConstraintResponse()` directly after collision detection, operating outside the Lagrangian constraint framework. This ticket unifies the pipeline so all constraint forces (bilateral + contact) flow through the same solver, producing consistent and physically correct results.

---

## Technical Approach

### Updated Pipeline

```
WorldModel::update(dt):
  1. detectCollisions()           -- GJK/EPA pairwise detection (existing)
  2. createContactConstraints()   -- NEW: Build transient ContactConstraint objects
  3. solveConstraints(dt)         -- NEW: Solve bilateral + contact constraints together
  4. integrateMotion(dt)          -- SemiImplicitEuler for each body
  5. synchronizeFrames()          -- ReferenceFrame = InertialState
  6. clearForces()                -- Reset accumulators
```

### New Types

```cpp
struct CollisionPair
{
    size_t bodyAIndex;
    size_t bodyBIndex;
    CollisionResult result;
    bool isStaticB{false};  // True if body B is an AssetEnvironment
};
```

### New Private Methods

```cpp
/// Detect all pairwise collisions, return with body indices
std::vector<CollisionPair> detectCollisions();

/// Create transient contact constraints from collision results
std::vector<std::unique_ptr<ContactConstraint>> createContactConstraints(
    const std::vector<CollisionPair>& collisions);

/// Solve unified constraint system (bilateral + contacts)
void solveConstraints(
    const std::vector<std::unique_ptr<ContactConstraint>>& contacts,
    double dt);

/// Integrate motion for all bodies using solved constraint forces
void integrateMotion(double dt);
```

### Key Changes

1. **Replace** all `CollisionResponse::applyConstraintResponse()` calls with constraint creation + solver invocation
2. **Replace** `CollisionResponse::applyPositionStabilization()` with Baumgarte stabilization built into `ContactConstraint`
3. **Replace** `CollisionResponse::combineRestitution()` with `ContactConstraintFactory::combineRestitution()`
4. **Remove** `#include "CollisionResponse.hpp"` from WorldModel.cpp
5. **Add** `#include "ContactConstraintFactory.hpp"` and solver headers

### Memory Lifecycle

Contact constraints are transient: created at step 2, consumed at step 3, destroyed at end of frame (unique_ptr scope). No state persists across frames.

---

## Requirements

### Functional Requirements

1. **FR-1**: `WorldModel::update()` uses constraint-based contact pipeline instead of `CollisionResponse`
2. **FR-2**: Head-on collision with e=1.0 swaps velocities (matches current behavior)
3. **FR-3**: Total momentum conserved before/after collision (within 1e-6 tolerance)
4. **FR-4**: Stacked objects remain stable for 1000 frames without drift exceeding 0.01m
5. **FR-5**: Glancing collision produces appropriate angular velocity
6. **FR-6**: Dynamic-static collisions work via unified solver (AssetEnvironment inverse mass = 0)
7. **FR-7**: Multiple simultaneous contacts handled correctly

### Non-Functional Requirements

1. **NFR-1**: Public `WorldModel` interface unchanged
2. **NFR-2**: Existing collision detection (GJK/EPA) retained as input source
3. **NFR-3**: Per-frame allocation under 10 KB for typical scenes (< 20 contacts)
4. **NFR-4**: Performance matches or exceeds current impulse-based approach

---

## Acceptance Criteria

These correspond directly to the parent ticket's acceptance criteria:

1. [ ] AC1: Head-on collision with e=1.0 swaps velocities (parent AC4)
2. [ ] AC2: Total momentum conserved within 1e-6 tolerance (parent AC5)
3. [ ] AC3: Stacked objects stable for 1000 frames, drift < 0.01m (parent AC6)
4. [ ] AC4: Glancing collision produces angular velocity (parent AC7)
5. [ ] AC5: Dynamic-static collision: dynamic bounces, static unmoved
6. [ ] AC6: Multiple simultaneous contacts resolved correctly
7. [ ] AC7: Zero-penetration (touching) case handled without explosion
8. [ ] AC8: All existing bilateral constraint tests pass (parent AC9)
9. [ ] AC9: `CollisionResponse` no longer referenced from WorldModel.cpp

---

## Dependencies

- **Ticket 0032a**: Two-Body Constraint Infrastructure (provides ContactConstraint, Factory)
- **Ticket 0032b**: PGS Solver Extension (provides solveWithContacts())
- **Blocks**: [0032d](0032d_collision_response_cleanup.md)

---

## Files

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/src/Environment/WorldModel.hpp` | Add CollisionPair struct, new private methods, remove CollisionResponse include |
| `msd-sim/src/Environment/WorldModel.cpp` | Rewrite collision pipeline to use constraints |

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/test/Environment/WorldModelContactIntegrationTest.cpp` | Integration tests for constraint-based contacts |

### Modified Test Files

| File | Changes |
|------|---------|
| `msd-sim/test/Environment/WorldModelCollisionTest.cpp` | Update assertions to match constraint-based behavior (if needed) |

---

## Test Plan

### Integration Tests (WorldModelContactIntegrationTest.cpp)

| Test Case | What It Validates | Parent AC |
|-----------|-------------------|-----------|
| Head-on collision velocity swap (e=1.0) | Equal mass bodies swap velocities | AC4 |
| Momentum conservation | Total momentum before == after (1e-6) | AC5 |
| Resting contact stability (1000 frames) | Stacked objects drift < 0.01m | AC6 |
| Glancing collision angular response | Off-center impact produces angular velocity | AC7 |
| Dynamic-static collision | Dynamic bounces off static; static unmoved | — |
| Multiple simultaneous contacts | Object touching two surfaces resolved | — |
| Zero penetration (touching) | Contact handled without instability | — |

---

## References

- **Design document**: `docs/designs/0032_contact_constraint_refactor/design.md` (Section "WorldModel")
- **Current WorldModel**: `msd-sim/src/Environment/WorldModel.cpp`
- **Current CollisionResponse**: `msd-sim/src/Physics/CollisionResponse.hpp` (code being replaced)

---

## Workflow Log

### Design Phase
- **Completed**: 2026-01-29
- **Notes**: Design shared with parent ticket 0032.

### Implementation Phase
- **Started**:
- **Completed**:
- **Notes**:

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
