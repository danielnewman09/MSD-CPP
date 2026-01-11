# Feature Ticket: Physics State Consolidation

## Status
- [X] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-10
- **Author**: Daniel M Newman
- **Priority**: High
- **Estimated Complexity**: Medium
- **Target Component(s)**: msd-sim

---

## Summary

Consolidate the physics module's dynamic state representation by removing the redundant `DynamicState` class, replacing all state references to the `InertialState`. Also, currently and deleting the unused `AssetPhysical`/`AssetInertial`/`AssetEnvironment` class hierarchy. This establishes a single, functional kinematic state representation for the physics system.

## Motivation

The current physics implementation has architectural redundancy that causes confusion and maintenance burden:

1. **Dual State Representations**: `DynamicState` (Physics module) and `InertialState` (Environment module) serve identical purposes—storing linear and angular velocities and accelerations—but are separate implementations.

2. **Placeholder Implementations**: `DynamicState` methods are all marked `[[maybe_unused]]` with no actual implementation. Setters don't store values, getters return defaults, making the class non-functional.

3. **Dead Code**: The `AssetPhysical`/`AssetInertial`/`AssetEnvironment` hierarchy is redundant with the `Object` component-based pattern and contains broken code (syntax error in `AssetPhysical.cpp` line 31: `return collisionGeometry_.get().;`).

This refactoring is a prerequisite for implementing force application (ticket 0018) and motion integration (ticket 0019).

## Requirements

### Functional Requirements
1. Remove `InertialState` class and consolidate on `DynamicState` as the single kinematic state representation
2. Implement all `DynamicState` placeholder methods with actual functionality:
   - Setters shall store values in member variables
   - Getters shall return stored values
   - `getKineticEnergy()` shall compute correct value from mass and velocity
   - `applyLinearImpulse()` and `applyAngularImpulse()` shall modify velocity
   - `isAtRest()` shall return true when velocities are below threshold
   - `reset()` shall zero all velocities and accelerations
3. Remove dead code: `AssetPhysical`, `AssetInertial`, `AssetEnvironment` classes
4. Update `Platform` (Environment) to use `DynamicState` or remain stateless if appropriate
5. Ensure `PhysicsComponent` correctly uses the now-functional `DynamicState`

### Non-Functional Requirements
- **Performance**: No runtime overhead compared to current implementation
- **Memory**: No increase in per-object memory usage
- **Backward Compatibility**: All existing tests shall pass after refactoring

## Constraints
- Must preserve existing `PhysicsComponent` force/torque calculation logic (proven working)
- Must not change the `Object` component-based pattern
- `ReferenceFrame` remains the authoritative source of object transforms

## Acceptance Criteria
- [ ] `InertialState` class deleted from `msd/msd-sim/src/Environment/`
- [ ] All references to `InertialState` updated to use `DynamicState`
- [ ] `DynamicState` setters store values in member variables
- [ ] `DynamicState` getters return stored values (not defaults)
- [ ] `DynamicState::getKineticEnergy()` returns `0.5 * mass * velocity²` (requires mass parameter or redesign)
- [ ] `DynamicState::applyLinearImpulse(impulse, mass)` updates velocity by `impulse / mass`
- [ ] `DynamicState::applyAngularImpulse(impulse, inertia)` updates angular velocity
- [ ] `DynamicState::isAtRest(threshold)` returns true when speeds below threshold
- [ ] `DynamicState::reset()` zeros all velocities and accelerations
- [ ] `AssetPhysical.hpp` and `AssetPhysical.cpp` deleted
- [ ] `AssetInertial.hpp` and `AssetInertial.cpp` deleted
- [ ] `AssetEnvironment.hpp` deleted
- [ ] `Platform` class updated or confirmed stateless
- [ ] No `[[maybe_unused]]` attributes remain in `DynamicState`
- [ ] All existing tests pass (excluding pre-existing failures)
- [ ] No new compiler warnings introduced

---

## Design Decisions (Human Input)

### Preferred Approaches
- **State Storage**: Keep `DynamicState` in the Physics module where it logically belongs
- **Kinetic Energy**: May need to pass mass as parameter, or redesign to have DynamicState hold mass reference
- **Platform State**: If Platform objects are always static, they may not need DynamicState at all

### Things to Avoid
- Don't add new dependencies between Physics and Environment modules
- Don't change the Object/PhysicsComponent relationship
- Don't implement motion integration in this ticket (that's ticket 0019)

### Open Questions
1. Should `DynamicState` store mass, or receive it as a parameter for kinetic energy calculation? **Response** Any Kinetic-Energy related calculations should be visible at the `PhysicsCompn
2. Should `Platform` use `DynamicState`, or should it remain stateless (static environment objects)? 
3. What velocity threshold should `isAtRest()` use? (Suggest: 1e-6 m/s)

---

## References

### Related Code

#### Files to Remove
- `msd/msd-sim/src/Physics/RigidBody/AssetPhysical.hpp`
- `msd/msd-sim/src/Physics/RigidBody/AssetPhysical.cpp` (contains syntax error)
- `msd/msd-sim/src/Physics/RigidBody/AssetInertial.hpp`
- `msd/msd-sim/src/Physics/RigidBody/AssetInertial.cpp`
- `msd/msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp`
- `msd/msd-sim/src/Environment/InertialState.hpp` (after migrating dependents)

#### Files to Modify
- `msd/msd-sim/src/Physics/RigidBody/DynamicState.hpp` — Implement all placeholder methods
- `msd/msd-sim/src/Physics/RigidBody/PhysicsComponent.hpp/cpp` — Ensure uses functional DynamicState
- `msd/msd-sim/src/Environment/Platform.hpp` — Update state representation
- `msd/msd-sim/src/Environment/Object.hpp/cpp` — May need minor adjustments
- `msd/msd-sim/CMakeLists.txt` — Remove deleted source files

#### Key Existing Files (Preserve)
- `msd/msd-sim/src/Physics/RigidBody/ConvexHull.hpp/cpp` — Collision geometry (working)
- `msd/msd-sim/src/Physics/GJK.hpp/cpp` — Collision detection (working)
- `msd/msd-sim/src/Physics/RigidBody/InertialCalculations.hpp/cpp` — Inertia tensor (working)

### Related Tickets
- **0018**: Force Application Mechanism (depends on this ticket)
- **0019**: Physics Motion Integration (depends on this ticket)
- **0020**: Physics GUI Propagation (depends on 0019)

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**:
- **Completed**:
- **Artifacts**:
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
- **Artifacts**:
- **Notes**:

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
- **Files Modified**:
- **Artifacts**:
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
- **Diagrams**:
- **Artifacts**:
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

---

## Appendix: Current State Analysis

### DynamicState vs InertialState Comparison

| Aspect | DynamicState | InertialState |
|--------|--------------|---------------|
| Location | `Physics/RigidBody/DynamicState.hpp` | `Environment/InertialState.hpp` |
| Linear State | velocity, acceleration (Coordinate) | position, velocity, acceleration (Coordinate) |
| Angular State | velocity, acceleration (Eigen::Vector3d) | position, velocity, acceleration (EulerAngles) |
| Implementation | Placeholder (all methods no-op) | Functional (plain struct) |
| Used By | PhysicsComponent (but not functional) | Platform |

### Dead Asset Hierarchy

| Class | File | Status | Issue |
|-------|------|--------|-------|
| `AssetPhysical` | `AssetPhysical.hpp/cpp` | Broken | Syntax error line 31, base class for unused hierarchy |
| `AssetInertial` | `AssetInertial.hpp/cpp` | Unused | Duplicates PhysicsComponent functionality |
| `AssetEnvironment` | `AssetEnvironment.hpp` | Unused | Factory methods without implementation |

### DynamicState Placeholder Methods (All Currently No-Op)

```cpp
// All marked [[maybe_unused]] with empty/default implementations:
void setLinearVelocity(const Coordinate& velocity);
Coordinate getLinearVelocity() const;  // returns Coordinate{}
void setAngularVelocity(const Eigen::Vector3d& velocity);
Eigen::Vector3d getAngularVelocity() const;  // returns Zero()
void setLinearAcceleration(const Coordinate& acceleration);
Coordinate getLinearAcceleration() const;  // returns Coordinate{}
void setAngularAcceleration(const Eigen::Vector3d& acceleration);
Eigen::Vector3d getAngularAcceleration() const;  // returns Zero()
void applyLinearImpulse(const Coordinate& impulse);  // no-op
void applyAngularImpulse(const Eigen::Vector3d& impulse);  // no-op
double getSpeed() const;  // returns 0.0
double getAngularSpeed() const;  // returns 0.0
double getKineticEnergy(double mass) const;  // returns 0.0
double getRotationalKineticEnergy(const Eigen::Matrix3d& inertia) const;  // returns 0.0
bool isAtRest(double threshold) const;  // returns true
void reset();  // no-op
```
