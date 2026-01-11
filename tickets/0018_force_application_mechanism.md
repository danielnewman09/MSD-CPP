# Feature Ticket: Force Application Mechanism

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

Design and implement a force application mechanism for the physics system, including a `ForceGenerator` interface for external force sources, concrete implementations for gravity and collision response, and a `ForceAccumulator` that collects forces from all generators per timestep. This provides the foundation for realistic physics simulation.

## Motivation

The current physics implementation can calculate the effect of forces on objects (`PhysicsComponent::applyForce()` and `applyTorque()` work correctly), but there are no sources of forces:

1. **No Gravity**: Objects have mass but no gravitational acceleration is applied
2. **No Collision Response**: GJK can detect collisions, but there's no impulse or penalty force generated
3. **No Contact Forces**: Objects would fall through static geometry without normal forces
4. **No Extensibility**: Adding new force types (springs, drag, magnetism) has no established pattern

This ticket establishes the force generation infrastructure that feeds into the motion integration (ticket 0019).

## Requirements

### Functional Requirements
1. Design and implement a `ForceGenerator` abstract interface for external force sources
2. Implement `GravityForce` generator that applies constant gravitational acceleration (-9.81 m/s² default)
3. Implement `CollisionResponse` force generator that:
   - Uses existing GJK collision detection
   - Computes penetration depth and contact normal
   - Applies separation impulse to resolve interpenetration
4. Implement `ContactForce` for resting contact:
   - Applies normal force to prevent objects falling through static geometry
   - Handles friction (static and kinetic) at contact points
5. Create a `ForceAccumulator` that:
   - Maintains a collection of `ForceGenerator` instances
   - Iterates all generators and accumulates forces per object per timestep
   - Clears accumulated forces after integration step

### Non-Functional Requirements
- **Performance**: Force accumulation for 100 objects shall complete in under 0.5ms
- **Extensibility**: New force types shall be addable without modifying existing code
- **Accuracy**: Gravity shall produce correct free-fall acceleration
- **Stability**: Collision response shall prevent tunneling at reasonable velocities

## Constraints
- Must use existing `GJK` collision detection (do not reimplement)
- Must use existing `PhysicsComponent::applyForce()` and `applyTorque()` methods
- Force generators shall not directly modify object positions (that's the integrator's job)
- Collision response shall work with `ConvexHull` geometry

## Acceptance Criteria
- [ ] `ForceGenerator` abstract interface defined with `apply(objects, dt)` method
- [ ] `GravityForce` applies `mass * g` downward force to all dynamic objects
- [ ] `GravityForce` configurable gravity vector (default: 0, -9.81, 0)
- [ ] `CollisionResponse` detects penetration using GJK/EPA or GJK with margin
- [ ] `CollisionResponse` computes contact normal and penetration depth
- [ ] `CollisionResponse` applies impulse based on relative velocity and restitution
- [ ] `ContactForce` applies normal force to objects resting on static geometry
- [ ] `ContactForce` applies friction force opposing tangential motion
- [ ] `ForceAccumulator` collects and applies forces from all registered generators
- [ ] `ForceAccumulator::clearForces()` resets all object accelerations
- [ ] Unit tests verify gravity produces correct acceleration
- [ ] Unit tests verify collision impulse separates interpenetrating objects
- [ ] Unit tests verify contact force prevents falling through floor
- [ ] No new compiler warnings introduced

---

## Design Decisions (Human Input)

### Preferred Approaches
- **Force Generator Pattern**: Simple polymorphic interface; avoid over-engineering with registries or event systems
- **Collision Response Model**: Impulse-based (instantaneous velocity change) vs penalty-based (spring force)
- **Friction Model**: Coulomb friction with static/kinetic coefficients
- **Contact Detection**: Use GJK with small margin, or implement EPA for penetration depth

### Things to Avoid
- Don't implement constraint solvers (joints, ragdolls)—focus on basic rigid body forces
- Don't implement broad-phase collision detection yet—assume N² narrow-phase is acceptable for now
- Don't implement sleeping/deactivation—get forces working first
- Don't modify object positions in force generators—only apply forces/impulses

### Open Questions
1. **Impulse vs Penalty**: Should collision response use impulses (instantaneous) or penalty forces (spring-like)?
2. **EPA Implementation**: Do we need EPA for penetration depth, or is GJK with margin sufficient?
3. **Friction Coefficients**: Where should friction coefficients be stored? (PhysicsComponent? Material?)
4. **Restitution**: Where should coefficient of restitution be stored?
5. **Static vs Dynamic Collisions**: How to handle dynamic-dynamic vs dynamic-static collisions?

---

## References

### Related Code

#### Files to Create
- `msd/msd-sim/src/Physics/Forces/ForceGenerator.hpp` — Abstract interface
- `msd/msd-sim/src/Physics/Forces/GravityForce.hpp` — Gravity implementation (header-only)
- `msd/msd-sim/src/Physics/Forces/GravityForce.cpp` — Gravity implementation
- `msd/msd-sim/src/Physics/Forces/CollisionResponse.hpp` — Collision impulses
- `msd/msd-sim/src/Physics/Forces/CollisionResponse.cpp` — Collision impulses
- `msd/msd-sim/src/Physics/Forces/ContactForce.hpp` — Resting contact
- `msd/msd-sim/src/Physics/Forces/ContactForce.cpp` — Resting contact
- `msd/msd-sim/src/Physics/ForceAccumulator.hpp` — Force collection
- `msd/msd-sim/src/Physics/ForceAccumulator.cpp` — Force collection
- `msd/msd-sim/test/Physics/Forces/GravityForceTest.cpp` — Unit tests
- `msd/msd-sim/test/Physics/Forces/CollisionResponseTest.cpp` — Unit tests

#### Files to Modify
- `msd/msd-sim/CMakeLists.txt` — Add new source files
- `msd/msd-sim/src/Physics/RigidBody/PhysicsComponent.hpp` — May need friction/restitution properties

#### Key Existing Files (Use)
- `msd/msd-sim/src/Physics/GJK.hpp/cpp` — Collision detection
- `msd/msd-sim/src/Physics/RigidBody/ConvexHull.hpp/cpp` — Collision geometry
- `msd/msd-sim/src/Physics/RigidBody/PhysicsComponent.hpp/cpp` — Force application
- `msd/msd-sim/src/Physics/RigidBody/DynamicState.hpp` — Velocity storage (after 0017)

### Related Documentation
- Impulse-based collision response: Game Physics Engine Development (Millington)
- GJK/EPA algorithms for penetration depth
- Coulomb friction model

### Related Tickets
- **0017**: Physics State Consolidation (prerequisite—must be completed first)
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

## Appendix: Technical Background

### ForceGenerator Interface Design

```cpp
// Proposed interface
class ForceGenerator {
public:
    virtual ~ForceGenerator() = default;

    // Apply forces to all applicable objects
    // dt is timestep for velocity-dependent forces (drag, damping)
    virtual void apply(std::span<Object*> objects, double dt) = 0;

    // Optional: Check if generator applies to specific object
    virtual bool appliesTo(const Object& object) const { return true; }
};
```

### Gravity Force Implementation Sketch

```cpp
class GravityForce : public ForceGenerator {
public:
    explicit GravityForce(Coordinate gravity = {0, -9.81, 0});

    void apply(std::span<Object*> objects, double dt) override {
        for (auto* obj : objects) {
            if (obj->hasPhysicsComponent() && obj->isInertial()) {
                auto& physics = obj->getPhysicsComponent();
                // F = m * g
                Coordinate force = gravity_ * physics.getMass();
                physics.applyForce(force);
            }
        }
    }

private:
    Coordinate gravity_;
};
```

### Collision Response Options

**Option A: Impulse-Based (Recommended)**
- Instantaneous velocity change at collision
- Good for rigid bodies
- Formula: `j = -(1 + e) * v_rel · n / (1/m1 + 1/m2)`
- Apply: `v1 += j/m1 * n`, `v2 -= j/m2 * n`

**Option B: Penalty-Based**
- Spring force proportional to penetration depth
- Can cause instability if stiffness too high
- Formula: `F = k * penetration * n`
- Requires tuning stiffness and damping

### Friction Model

**Coulomb Friction**:
- Static friction: `|F_s| ≤ μ_s * N` (prevents sliding)
- Kinetic friction: `F_k = μ_k * N` (opposes motion)
- Typical coefficients: μ_s = 0.5, μ_k = 0.3

### Existing PhysicsComponent Force Methods

```cpp
// Already implemented and working:
void PhysicsComponent::applyForce(const Coordinate& force);
void PhysicsComponent::applyForceAtPoint(const Coordinate& force, const Coordinate& point);
void PhysicsComponent::applyTorque(const Eigen::Vector3d& torque);
void PhysicsComponent::clearForces();
```

These methods correctly compute acceleration from force (F=ma) and angular acceleration from torque (τ=Iα).
