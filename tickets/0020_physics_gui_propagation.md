# Feature Ticket: Physics GUI Propagation

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
- **Target Component(s)**: msd-sim, msd-gui

---

## Summary

Establish the pathway from physics state updates to visual rendering, ensuring that `ReferenceFrame` changes from the physics simulation propagate correctly to the renderer's transform matrices. This includes optional frame interpolation for smooth rendering when physics runs at a fixed timestep different from the render rate, and debug visualization for forces, velocities, and collision contacts.

## Motivation

Even with working physics integration (ticket 0019), objects won't visually move unless the rendering system receives the updated transforms:

1. **No Render Update**: Physics updates `ReferenceFrame`, but the renderer may cache transforms
2. **Timestep Mismatch**: Physics at 60Hz, rendering at 144Hz causes visual stuttering without interpolation
3. **No Debug Visualization**: Developing and debugging physics requires seeing forces, velocities, contacts
4. **Integration Gap**: msd-sim (physics) and msd-gui (rendering) need a clean interface

This ticket completes the physics pipeline from force application through visual output.

## Requirements

### Functional Requirements

#### Core Propagation
1. Ensure `Object::getPosition()` and `Object::getOrientation()` reflect current physics state
2. Ensure `ReferenceFrame` transform matrices are updated after physics step
3. Verify renderer reads current `ReferenceFrame` each frame (no stale caching)
4. Establish clear update order: physics step → transform update → render

#### Frame Interpolation (if physics timestep ≠ render timestep)
1. Store previous and current physics state for interpolation
2. Compute interpolation factor: `alpha = accumulator / physics_dt`
3. Interpolate position: `render_pos = lerp(prev_pos, curr_pos, alpha)`
4. Interpolate orientation: `render_rot = slerp(prev_rot, curr_rot, alpha)`
5. Apply interpolated transforms to renderer without modifying authoritative physics state

#### Debug Visualization (Optional but Recommended)
1. Render velocity vectors as arrows from object center of mass
2. Render force vectors (scaled) as arrows at application points
3. Render collision contact points and normals
4. Render bounding boxes / convex hulls for collision geometry
5. Toggle debug visualization via runtime flag

### Non-Functional Requirements
- **Performance**: Transform propagation shall add < 0.1ms per frame for 100 objects
- **Latency**: Visual updates shall appear within 1 frame of physics update
- **Smoothness**: With interpolation, motion shall appear smooth at any render rate
- **Debuggability**: Debug visualization shall be toggleable without recompilation

## Constraints
- Must not modify `ReferenceFrame` during interpolation (authoritative state preserved)
- Must work with existing msd-gui render loop architecture
- Debug visualization must use existing rendering infrastructure (no new render backends)
- Must handle objects added/removed between physics and render steps

## Acceptance Criteria

### Core Propagation
- [ ] After `PhysicsWorld::step()`, `Object::getPosition()` returns updated position
- [ ] After `PhysicsWorld::step()`, `ReferenceFrame::getTransformMatrix()` returns updated matrix
- [ ] Renderer uses current `ReferenceFrame` transform each frame
- [ ] Demo: ball falls visually when gravity is applied

### Frame Interpolation
- [ ] Previous physics state stored before each physics step
- [ ] Interpolation factor computed from accumulator
- [ ] `getInterpolatedPosition(alpha)` returns lerp'd position
- [ ] `getInterpolatedOrientation(alpha)` returns slerp'd orientation
- [ ] Renderer can use interpolated transforms for smooth motion
- [ ] Physics runs at 60Hz, render at 144Hz: motion is smooth (no stuttering)

### Demo Scene
- [ ] Ball dropped onto static platform bounces visibly
- [ ] Multiple objects fall and collide realistically
- [ ] Camera can observe physics in real-time

### Debug Visualization (Optional)
- [ ] Velocity vectors rendered as colored arrows (e.g., green)
- [ ] Force vectors rendered as colored arrows (e.g., red)
- [ ] Contact points rendered as small spheres
- [ ] Contact normals rendered as arrows
- [ ] Convex hull wireframes toggleable
- [ ] All debug rendering toggleable via single flag

---

## Design Decisions (Human Input)

### Preferred Approaches
- **Transform Update**: Direct update of ReferenceFrame after physics step
- **Interpolation Storage**: Store previous state in DynamicState or separate interpolation buffer
- **Debug Rendering**: Use existing line/point rendering if available; otherwise simple GL_LINES

### Things to Avoid
- Don't duplicate transform storage unnecessarily
- Don't modify physics state during interpolation (read-only interpolation)
- Don't make debug visualization the critical path (keep it optional)
- Don't tightly couple physics and rendering (maintain separation)

### Open Questions
1. Does msd-gui cache transforms, or does it read ReferenceFrame each frame?
2. What rendering primitives are available for debug visualization (lines, points, meshes)?
3. Should interpolation be opt-in, or always active when timesteps differ?
4. How should debug visualization interact with the existing scene graph?

---

## References

### Related Code

#### Files to Create
- `msd/msd-sim/src/Physics/InterpolationState.hpp` — State storage for interpolation
- `msd/msd-sim/src/Physics/DebugRenderer.hpp` — Debug visualization interface
- `msd/msd-gui/src/Physics/PhysicsDebugRenderer.hpp/cpp` — GUI-side debug rendering

#### Files to Modify
- `msd/msd-sim/src/Physics/PhysicsWorld.hpp/cpp` — Add interpolation support
- `msd/msd-sim/src/Environment/Object.hpp/cpp` — Add interpolated getters
- `msd/msd-gui/src/Renderer/...` — Ensure transforms read each frame
- `msd/msd-gui/src/...` — Integrate physics loop with render loop

#### Key Existing Files (Understand)
- `msd/msd-sim/src/Geometry/ReferenceFrame.hpp/cpp` — Transform storage
- `msd/msd-gui/src/...` — Render loop structure (need to explore)
- `msd/msd-sim/src/Environment/Object.hpp/cpp` — Entity interface

### Related Documentation
- Interpolation for fixed timestep game loops (Fix Your Timestep! - Glenn Fiedler)
- Spherical linear interpolation (slerp) for quaternions
- Debug visualization patterns in game engines

### Related Tickets
- **0017**: Physics State Consolidation (prerequisite)
- **0018**: Force Application Mechanism (prerequisite)
- **0019**: Physics Motion Integration (prerequisite)

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

### Fixed Timestep with Interpolation

```cpp
const double PHYSICS_DT = 1.0 / 60.0;  // 60 Hz physics
double accumulator = 0.0;

void gameLoop(double frameTime) {
    accumulator += frameTime;

    // Store previous state for interpolation
    for (auto& obj : objects) {
        obj.storePreviousState();
    }

    // Physics steps at fixed rate
    while (accumulator >= PHYSICS_DT) {
        physicsWorld.step(PHYSICS_DT);
        accumulator -= PHYSICS_DT;
    }

    // Interpolation factor
    double alpha = accumulator / PHYSICS_DT;

    // Render with interpolated transforms
    for (auto& obj : objects) {
        auto pos = obj.getInterpolatedPosition(alpha);
        auto rot = obj.getInterpolatedOrientation(alpha);
        renderer.setTransform(obj, pos, rot);
    }

    renderer.render();
}
```

### Linear Interpolation (lerp)

```cpp
Coordinate lerp(const Coordinate& a, const Coordinate& b, double t) {
    return a + (b - a) * t;
}
```

### Spherical Linear Interpolation (slerp)

```cpp
Quaternion slerp(const Quaternion& a, const Quaternion& b, double t) {
    double dot = a.dot(b);

    // Handle opposite quaternions
    Quaternion b_adj = (dot < 0) ? -b : b;
    dot = std::abs(dot);

    // Linear interpolation for very close quaternions
    if (dot > 0.9995) {
        return (a + (b_adj - a) * t).normalized();
    }

    double theta = std::acos(dot);
    double sin_theta = std::sin(theta);

    double w_a = std::sin((1 - t) * theta) / sin_theta;
    double w_b = std::sin(t * theta) / sin_theta;

    return (a * w_a + b_adj * w_b).normalized();
}
```

### Debug Visualization Data

```cpp
struct DebugVisualization {
    // Velocity vectors
    struct VelocityVector {
        Coordinate origin;
        Coordinate direction;
        float magnitude;
    };

    // Force vectors
    struct ForceVector {
        Coordinate applicationPoint;
        Coordinate direction;
        float magnitude;
    };

    // Contact points
    struct ContactPoint {
        Coordinate position;
        Coordinate normal;
        float penetrationDepth;
    };

    std::vector<VelocityVector> velocities;
    std::vector<ForceVector> forces;
    std::vector<ContactPoint> contacts;

    void clear() {
        velocities.clear();
        forces.clear();
        contacts.clear();
    }
};
```

### Rendering Debug Primitives

```cpp
void renderDebug(const DebugVisualization& debug) {
    // Velocity vectors (green)
    glColor3f(0.0f, 1.0f, 0.0f);
    for (const auto& v : debug.velocities) {
        drawArrow(v.origin, v.origin + v.direction * v.magnitude);
    }

    // Force vectors (red)
    glColor3f(1.0f, 0.0f, 0.0f);
    for (const auto& f : debug.forces) {
        drawArrow(f.applicationPoint,
                  f.applicationPoint + f.direction * f.magnitude * FORCE_SCALE);
    }

    // Contact points (yellow spheres)
    glColor3f(1.0f, 1.0f, 0.0f);
    for (const auto& c : debug.contacts) {
        drawSphere(c.position, 0.05f);
        drawArrow(c.position, c.position + c.normal * 0.2f);
    }
}
```

### Transform Update Flow

```
PhysicsWorld::step(dt)
    ├── ForceAccumulator::apply()           // Compute forces
    ├── Integrator::integrate()             // Update velocities & positions
    ├── PhysicsComponent::clearForces()     // Reset for next frame
    └── Object::updateTransform()           // Sync ReferenceFrame
            └── ReferenceFrame::setPosition/Orientation()
                    └── ReferenceFrame::updateMatrix()
                            └── (Renderer reads matrix each frame)
```
