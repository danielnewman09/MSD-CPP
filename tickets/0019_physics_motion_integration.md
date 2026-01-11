# Feature Ticket: Physics Motion Integration

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

Implement the physics integration step that converts accumulated forces into motion, creating a `PhysicsWorld` orchestrator that manages the simulation loop. This includes numerical integration (acceleration → velocity → position), object transform updates, and proper handling of static vs dynamic objects.

## Motivation

The current physics implementation has a critical gap: forces can be applied and accelerations computed, but there is no integration step to convert these into actual motion:

1. **No Integration**: `PhysicsComponent` computes accelerations from forces, but nothing converts acceleration → velocity → position
2. **No Simulation Loop**: No orchestrator exists to run physics updates each frame
3. **Objects Don't Move**: Despite working force calculations, objects remain stationary
4. **No Timestep Management**: No fixed timestep or accumulator pattern for frame-rate independence

This ticket bridges the gap between force application (ticket 0018) and visual updates (ticket 0020).

## Requirements

### Functional Requirements
1. Implement a numerical integrator for rigid body motion:
   - Integrate linear acceleration → linear velocity → position
   - Integrate angular acceleration → angular velocity → orientation
   - Support at minimum Semi-implicit Euler; prefer Velocity Verlet for stability
2. Create a `PhysicsWorld` class that orchestrates the physics loop:
   - Maintains collection of physics-enabled objects
   - Runs force accumulation each timestep (using ForceAccumulator from 0018)
   - Calls integrator to update velocities and positions
   - Clears forces after integration
   - Updates object transforms (ReferenceFrame) based on integrated state
3. Implement fixed timestep with accumulator pattern:
   - Physics runs at fixed rate (e.g., 1/60s = 16.67ms)
   - Accumulate frame time and step physics appropriately
   - Handle variable render frame rates gracefully
4. Handle static vs dynamic object separation:
   - Environmental/Boundary objects shall not be integrated (infinite mass)
   - Only Inertial objects have their positions updated

### Non-Functional Requirements
- **Performance**: Integration for 100 objects shall complete in under 0.5ms
- **Stability**: Integration shall be stable at 60Hz with typical masses and forces
- **Accuracy**: Free-falling object shall match kinematic equations within 1% over 1 second
- **Frame Independence**: Physics behavior shall be consistent regardless of render frame rate

## Constraints
- Must use `DynamicState` for velocity/acceleration storage (from ticket 0017)
- Must use `ForceAccumulator` for force collection (from ticket 0018)
- `ReferenceFrame` is the authoritative source of object transforms—update it after integration
- Must preserve existing `Object` type semantics (Inertial, Environmental, Boundary, Graphical)

## Acceptance Criteria

### Integrator
- [ ] `Integrator` class/function implemented with Semi-implicit Euler minimum
- [ ] Linear integration: `v += a * dt`, `p += v * dt`
- [ ] Angular integration: `ω += α * dt`, `q = q * quaternion(ω * dt)` (or Euler angle equivalent)
- [ ] Velocity Verlet variant available if Semi-implicit Euler proves unstable

### PhysicsWorld
- [ ] `PhysicsWorld` class created with `step(dt)` method
- [ ] `PhysicsWorld::addObject(Object&)` registers objects for simulation
- [ ] `PhysicsWorld::removeObject(Object&)` unregisters objects
- [ ] `PhysicsWorld::step(dt)` executes: accumulate forces → integrate → clear forces
- [ ] Only `Object::Type::Inertial` objects are integrated
- [ ] `Object::Type::Environmental` and `Boundary` objects remain stationary

### Transform Updates
- [ ] After integration, `Object::getPosition()` reflects new position
- [ ] After integration, `Object::getOrientation()` reflects new orientation
- [ ] `ReferenceFrame` is updated to match integrated state
- [ ] Position updates are relative to center of mass

### Fixed Timestep
- [ ] Fixed physics timestep configurable (default: 1/60s)
- [ ] Accumulator pattern handles variable frame times
- [ ] Multiple physics steps per frame when frame time > physics timestep
- [ ] Interpolation state available for smooth rendering (optional)

### Verification
- [ ] Unit test: Object with initial velocity moves correct distance after `step(dt)`
- [ ] Unit test: Object under gravity accelerates at 9.81 m/s²
- [ ] Integration test: Ball dropped from 10m reaches ground at t ≈ 1.43s
- [ ] Performance test: 100 objects integrate in under 0.5ms

---

## Design Decisions (Human Input)

### Preferred Approaches
- **Integration Method**: Semi-implicit Euler for simplicity; upgrade to Velocity Verlet if needed
- **Timestep**: Fixed 1/60s (16.67ms) with accumulator pattern
- **Orientation**: Quaternions preferred for rotation integration (avoid gimbal lock)
- **PhysicsWorld Ownership**: Hold references to objects, not ownership

### Things to Avoid
- Don't implement sub-stepping within a single physics step (keep it simple)
- Don't add sleeping/deactivation yet—get basic motion working first
- Don't implement interpolation for rendering initially (can add later)
- Don't change Object ownership model—PhysicsWorld observes, doesn't own

### Open Questions
1. Should `PhysicsWorld` use quaternions or Euler angles for orientation?
2. Should we support variable timestep as an option, or always use fixed?
3. How to handle objects added/removed during simulation step?
4. Should `PhysicsWorld` be a singleton, or allow multiple instances?

---

## References

### Related Code

#### Files to Create
- `msd/msd-sim/src/Physics/Integrator.hpp` — Integration algorithms
- `msd/msd-sim/src/Physics/Integrator.cpp` — Integration implementation
- `msd/msd-sim/src/Physics/PhysicsWorld.hpp` — Simulation orchestrator
- `msd/msd-sim/src/Physics/PhysicsWorld.cpp` — Simulation orchestrator
- `msd/msd-sim/test/Physics/IntegratorTest.cpp` — Integration tests
- `msd/msd-sim/test/Physics/PhysicsWorldTest.cpp` — Simulation tests

#### Files to Modify
- `msd/msd-sim/CMakeLists.txt` — Add new source files
- `msd/msd-sim/src/Environment/Object.hpp/cpp` — May need position update methods
- `msd/msd-sim/src/Geometry/ReferenceFrame.hpp/cpp` — May need update methods

#### Key Existing Files (Use)
- `msd/msd-sim/src/Physics/RigidBody/DynamicState.hpp` — Velocity storage (from 0017)
- `msd/msd-sim/src/Physics/RigidBody/PhysicsComponent.hpp/cpp` — Mass, inertia, forces
- `msd/msd-sim/src/Physics/ForceAccumulator.hpp/cpp` — Force collection (from 0018)
- `msd/msd-sim/src/Environment/Object.hpp/cpp` — Entity management

### Related Documentation
- Semi-implicit Euler integration
- Velocity Verlet integration
- Fixed timestep game loop pattern
- Quaternion integration for rotations

### Related Tickets
- **0017**: Physics State Consolidation (prerequisite)
- **0018**: Force Application Mechanism (prerequisite)
- **0020**: Physics GUI Propagation (depends on this ticket)

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

### Integration Methods

#### Semi-implicit Euler (Symplectic Euler)
```cpp
// Update velocity first, then position
v_new = v + a * dt
p_new = p + v_new * dt  // Use NEW velocity

// Angular
ω_new = ω + α * dt
θ_new = θ + ω_new * dt  // Simplified for Euler angles
```
- Simple to implement
- More stable than explicit Euler
- Good enough for many games
- O(dt) error per step

#### Velocity Verlet
```cpp
// Position uses velocity at half-step
p_new = p + v * dt + 0.5 * a * dt²
a_new = computeAcceleration(p_new)  // Requires force recalculation
v_new = v + 0.5 * (a + a_new) * dt
```
- More accurate than Euler
- Better energy conservation
- Requires computing forces twice per step
- O(dt²) error per step

### Fixed Timestep with Accumulator

```cpp
const double PHYSICS_DT = 1.0 / 60.0;  // 60 Hz
double accumulator = 0.0;

void update(double frameTime) {
    accumulator += frameTime;

    while (accumulator >= PHYSICS_DT) {
        physicsWorld.step(PHYSICS_DT);
        accumulator -= PHYSICS_DT;
    }

    // Optional: interpolation factor for rendering
    double alpha = accumulator / PHYSICS_DT;
    render(alpha);
}
```

### Quaternion Integration

For stable 3D rotation integration:

```cpp
// Convert angular velocity to quaternion derivative
Quaternion qdot = 0.5 * Quaternion(0, ω.x, ω.y, ω.z) * q;

// Integrate
q_new = q + qdot * dt;
q_new.normalize();  // Maintain unit quaternion
```

### PhysicsWorld Design Sketch

```cpp
class PhysicsWorld {
public:
    explicit PhysicsWorld(double timestep = 1.0/60.0);

    void addObject(Object& object);
    void removeObject(Object& object);

    void addForceGenerator(std::unique_ptr<ForceGenerator> generator);

    // Main simulation step
    void step(double dt);

    // Accessors
    double getTimestep() const { return timestep_; }
    void setGravity(const Coordinate& gravity);

private:
    void accumulateForces();
    void integrate();
    void clearForces();
    void updateTransforms();

    double timestep_;
    std::vector<std::reference_wrapper<Object>> objects_;
    ForceAccumulator forceAccumulator_;
};
```

### Verification: Free Fall Kinematics

For an object dropped from height h with gravity g:
- Time to fall: `t = sqrt(2h/g)`
- Final velocity: `v = sqrt(2gh)`

Example: h = 10m, g = 9.81 m/s²
- t = sqrt(20/9.81) ≈ 1.43s
- v = sqrt(196.2) ≈ 14.0 m/s

This provides a concrete test case for integration accuracy.
