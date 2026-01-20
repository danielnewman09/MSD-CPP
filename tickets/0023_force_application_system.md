# Feature Ticket: Force Application System for Rigid Body Physics

## Status
- [x] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-19
- **Author**: Human + AI
- **Priority**: High
- **Estimated Complexity**: Medium
- **Target Component(s)**: msd-sim (Physics/RigidBody, Environment)

---

## Summary
Implement a force application system for `AssetInertial` objects that supports point forces at arbitrary locations (generating torque), direct torque application, and gravity. Forces are integrated using semi-implicit Euler to update object velocities and positions.

## Motivation
The current physics simulation has infrastructure for rigid body dynamics (`AssetInertial` with mass, inertia tensor, `InertialState`) but no mechanism to apply forces or integrate motion. The `WorldModel::updatePhysics()` method is a placeholder that only moves objects by a fixed dummy step. Without force application, objects cannot respond to gravity, thrust, collisions, or any other physical interactions, making the physics engine non-functional for realistic simulation.

## Requirements

### Functional Requirements
1. The system shall allow applying a force at an object's center of mass (linear force only)
2. The system shall allow applying a force at an arbitrary world-space point, generating both linear force and torque
3. The system shall allow applying direct torque to objects
4. The system shall accumulate multiple forces/torques per frame before integration
5. The system shall clear accumulated forces after each physics step
6. The system shall support configurable gravity acceleration in `WorldModel`
7. The system shall integrate forces using semi-implicit Euler (velocity-first)
8. The system shall synchronize `ReferenceFrame` with `InertialState` after integration

### Non-Functional Requirements
- **Performance**: Force accumulation should be O(1) per force application
- **Memory**: No heap allocations during force application or integration
- **Numerical Stability**: Semi-implicit Euler provides better stability than explicit Euler for oscillatory systems
- **Breaking Change**: `InertialState` will change angular velocity/acceleration from `EulerAngles` to `Eigen::Vector3d` (required for correct physics)

## Constraints
- Must use existing `AssetInertial` class for physics objects
- Must use existing `ReferenceFrame` class for coordinate transformations
- Must follow project coding standards: brace initialization, Rule of Zero/Five, NaN for uninitialized floats
- Must avoid `shared_ptr` in favor of references for non-owning access per project standards
- Angular velocity must be represented as a 3D vector, not Euler angle rates (mathematically required)

## Acceptance Criteria
- [ ] `InertialState::angularVelocity` and `angularAcceleration` are `Eigen::Vector3d` (not `EulerAngles`)
- [ ] `AssetInertial::applyForce(const Coordinate& force)` applies force at center of mass
- [ ] `AssetInertial::applyForceAtPoint(const Coordinate& force, const Coordinate& worldPoint)` applies force and computes torque
- [ ] `AssetInertial::applyTorque(const Eigen::Vector3d& torque)` applies direct torque
- [ ] `AssetInertial::clearForces()` resets accumulated force and torque to zero
- [ ] `WorldModel::setGravity(const Coordinate& gravity)` configures gravity acceleration
- [ ] `WorldModel::updatePhysics()` implements semi-implicit Euler integration
- [ ] Objects fall under gravity when no other forces are applied
- [ ] Forces at offset points generate torque (verify `tau = r x F`)
- [ ] Unit tests cover force accumulation, torque generation, and integration
- [ ] Integration tests verify projectile motion and rotation behavior
- [ ] All existing tests pass after `InertialState` breaking change

---

## Design Decisions (Human Input)

### Preferred Approaches
- Store accumulated force and torque as members of `AssetInertial` (keeps physics data together)
- Use `Eigen::Vector3d` for angular velocity and torque (mathematically correct representation) **Response** Agreed
- Apply gravity as acceleration in `updatePhysics()` rather than per-object force (more efficient) 
- Use semi-implicit Euler: update velocity first, then use new velocity for position update

### Things to Avoid
- Using Euler angle rates for angular velocity (incorrect physics, causes integration errors)
- Creating separate "ForceAccumulator" class (over-engineering for this use case)
- Using quaternions for orientation (future enhancement, out of scope)
- Implementing collision response (separate ticket)
- Adding damping forces (future enhancement)

### Open Questions
- Should forces be applied in world space or local space? **Response**: World space for forces and application points
- Should gravity be a force (F = m*g) or direct acceleration? **Response**: Direct acceleration (more efficient, physically equivalent)
- Should torque be in world space or body space? **Response**: World space (consistent with force convention)

---

## References

### Related Code
- `msd/msd-sim/src/Physics/RigidBody/InertialState.hpp` — Current kinematic state (to be modified)
- `msd/msd-sim/src/Physics/RigidBody/AssetInertial.hpp` — Dynamic physics object (to be extended)
- `msd/msd-sim/src/Environment/WorldModel.hpp` — Simulation container (to add gravity and integration)
- `msd/msd-sim/src/Environment/ReferenceFrame.hpp` — Coordinate transformations

### Related Documentation
- Semi-implicit Euler integration: https://en.wikipedia.org/wiki/Semi-implicit_Euler_method
- Rigid body dynamics: https://en.wikipedia.org/wiki/Rigid_body_dynamics
- CLAUDE.md coding standards (brace initialization, Rule of Zero/Five, memory management)

### Related Tickets
- `0022_gjk_asset_physical_transform` — GJK collision detection (prerequisite for future collision response)

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: {timestamp}
- **Completed**: {timestamp}
- **Artifacts**: {list of design documents}
- **Notes**: {summary of design decisions}

### Design Review Phase
- **Started**: {timestamp}
- **Completed**: {timestamp}
- **Status**: {APPROVED / CHANGES REQUESTED}
- **Reviewer Notes**: {summary}

### Prototype Phase
- **Started**: {timestamp}
- **Completed**: {timestamp}
- **Prototypes**: {list}
- **Artifacts**: {list}
- **Notes**: {summary}

### Implementation Phase
- **Started**: {timestamp}
- **Completed**: {timestamp}
- **Files Created**: {list}
- **Files Modified**: {list}
- **Artifacts**: {list}
- **Notes**: {summary}

### Implementation Review Phase
- **Started**: {timestamp}
- **Completed**: {timestamp}
- **Status**: {APPROVED / CHANGES REQUESTED}
- **Artifacts**: {list}
- **Reviewer Notes**: {summary}

### Documentation Update Phase
- **Started**: {timestamp}
- **Completed**: {timestamp}
- **CLAUDE.md Updates**: {list}
- **Diagrams Indexed**: {list}
- **Artifacts**: {list}
- **Notes**: {summary}

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design approach}

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}
