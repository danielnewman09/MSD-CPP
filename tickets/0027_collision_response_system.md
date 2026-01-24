# Feature Ticket: Collision Response System

## Status
- [x] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete — Awaiting Tutorial (if Generate Tutorial: Yes)
- [ ] Tutorial Complete — Ready to Merge (if Generate Tutorial: Yes)
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-23
- **Author**: Daniel Newman
- **Priority**: High
- **Estimated Complexity**: Large
- **Target Component(s)**: msd-sim (Physics module)
- **Generate Tutorial**: No

---

## Summary
Implement elastic collision response for rigid body dynamics using impulse-based physics. When two AssetInertial objects collide (detected by existing GJK algorithm), the system will compute penetration depth and contact normal using EPA, apply collision impulses based on coefficient of restitution, and separate overlapping objects via position correction.

## Motivation
The current physics simulation has GJK collision detection that returns boolean intersection results, but no collision response. Objects pass through each other. This ticket enables realistic bouncing behavior where:
- Objects bounce off each other with configurable elasticity (coefficient of restitution)
- Momentum and angular momentum are properly transferred
- Objects don't overlap or sink into each other

This is foundational for any physics-based gameplay or simulation involving interacting objects.

## Requirements

### Functional Requirements
1. The system shall extend GJK to expose the terminating simplex for EPA input
2. The system shall implement EPA algorithm to extract penetration depth and contact normal
3. The system shall add coefficient of restitution as a per-object property on AssetInertial
4. The system shall compute and apply collision impulses affecting both linear and angular velocity
5. The system shall apply position correction to separate overlapping objects
6. The system shall handle Dynamic-Dynamic collisions (AssetInertial to AssetInertial)

### Non-Functional Requirements
- **Performance**: O(n²) pairwise collision checks acceptable for current scene sizes (<100 objects)
- **Memory**: No heap allocations during collision detection/response (use stack-based structures)
- **Thread Safety**: Not required (single-threaded simulation)
- **Backward Compatibility**: Must not break existing GJK API; add new methods only

## Constraints
- Must use existing GJK as foundation (not replace it)
- Coefficient of restitution range: [0.0, 1.0]
- Position correction must sync ReferenceFrame after updating InertialState position
- Angular impulse calculation requires world-space inverse inertia tensor transformation

## Acceptance Criteria
- [ ] AC1: GJK exposes `getSimplex()` method returning terminating simplex
- [ ] AC2: EPA extracts penetration depth within 1e-6 tolerance for unit cube overlap scenarios
- [ ] AC3: AssetInertial has `coefficientOfRestitution_` property with getter/setter
- [ ] AC4: Equal mass head-on collision with e=1.0 swaps velocities exactly (within tolerance)
- [ ] AC5: Total momentum is conserved before and after collision (within tolerance)
- [ ] AC6: Glancing collision generates appropriate angular velocity
- [ ] AC7: Position correction eliminates visible overlap after collision
- [ ] AC8: `updateCollisions()` in WorldModel processes all inertial asset pairs

---

## Design Decisions (Human Input)

### Preferred Approaches
- EPA should be a separate class (not merged into GJK) for clean separation of concerns
- Use geometric mean for combining restitution: `e = sqrt(e_A * e_B)`
- Position correction should use slop tolerance (0.01m) to prevent jitter
- Contact manifold as simple struct, not class (data-only)

### Things to Avoid
- Don't use shared_ptr for contact data (value semantics preferred)
- Don't implement broadphase optimization yet (future ticket)
- Don't handle Dynamic-Static collisions in this ticket (separate scope)

### Open Questions
- Should EPA have configurable max iterations? (Suggest: yes, default 64)
- Should position correction factor be configurable? (Suggest: hardcode 0.8 initially)

---

## References

### Related Code
- `msd-sim/src/Physics/GJK.hpp` — Current GJK implementation, needs `getSimplex()` added
- `msd-sim/src/Physics/GJK.cpp` — Contains `supportMinkowski()` which EPA will reuse
- `msd-sim/src/Physics/RigidBody/AssetInertial.hpp` — Add restitution property here
- `msd-sim/src/Physics/RigidBody/AssetInertial.cpp` — Force application pattern to follow
- `msd-sim/src/Environment/WorldModel.hpp` — Contains stubbed `updateCollisions()`
- `msd-sim/src/Environment/WorldModel.cpp` — Implement collision response loop here
- `msd-sim/test/Physics/GJKTest.cpp` — Test patterns to follow for EPA tests

### Related Documentation
- `docs/msd/msd-sim/Physics/gjk-asset-physical.puml` — GJK architecture diagram
- `docs/msd/msd-sim/Physics/force-application.puml` — Force system pattern
- Brian Mirtich GJK/EPA references (external)

### Related Tickets
- `0022_gjk_asset_physical_transform` — GJK with world-space transforms (prerequisite)
- `0023_force_application_system` — Force/impulse application pattern (reference)
- `0026_mirtich_inertia_tensor` — Inertia tensor calculation (dependency for angular impulse)

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**:
- **Completed**:
- **Artifacts**:
  - `docs/designs/0027_collision_response_system/design.md`
  - `docs/designs/0027_collision_response_system/0027_collision_response_system.puml`
- **Notes**:

### Design Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Prototype Phase
- **Started**:
- **Completed**:
- **Prototypes**:
  - P1: {name} — {result}
- **Artifacts**:
  - `docs/designs/0027_collision_response_system/prototype-results.md`
- **Notes**:

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
- **Files Modified**:
- **Artifacts**:
  - `docs/designs/0027_collision_response_system/implementation-notes.md`
- **Notes**:

### Implementation Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Documentation Update Phase
- **Started**:
- **Completed**:
- **CLAUDE.md Updates**:
- **Diagrams Indexed**:
- **Notes**:

### Tutorial Documentation Phase (if Generate Tutorial: Yes)
- **Started**:
- **Completed**:
- **Tutorial Location**: `docs/tutorials/0027_collision_response_system/`
- **Artifacts**:
  - `docs/tutorials/0027_collision_response_system/README.md`
  - `docs/tutorials/0027_collision_response_system/example.cpp`
  - `docs/tutorials/0027_collision_response_system/presentation.html`
- **Concepts Covered**:
- **Notes**:

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}

### Feedback on Tutorial (if Generate Tutorial: Yes)
{Your comments on the tutorial content, concepts to cover, or presentation style}
