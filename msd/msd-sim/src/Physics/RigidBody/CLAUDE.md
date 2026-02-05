# RigidBody Sub-Module Architecture

> Architectural context for the rigid body physics system.
> For API details, use MCP: `find_class AssetInertial`, `get_class_members ConvexHull`, etc.

## Component Hierarchy

```
AssetInertial (Complete dynamic object)
    ├── AssetPhysical (Base: geometry + transform)
    │   ├── ConvexHull (Collision geometry via Qhull)
    │   └── ReferenceFrame (Position/Orientation)
    ├── InertialState (14-component kinematic state)
    │   ├── Position/Velocity/Acceleration (Linear: 3+3+3)
    │   └── Quaternion/QuaternionRate (Angular: 4+4, no gimbal lock)
    ├── Mass Properties
    │   ├── Mass, Inertia Tensor (Mirtich algorithm)
    │   └── Center of Mass (from Qhull)
    └── Constraints (Lagrange multipliers)

AssetEnvironment (Static object - no dynamics)
    └── AssetPhysical (hull + transform only)
```

---

## Design Decisions

### Why Quaternions Instead of Euler Angles?

Euler angles suffer from gimbal lock singularities when pitch approaches ±90°. Quaternions provide singularity-free rotation representation.

**Implementation**:
- `InertialState::orientation` stores unit quaternion Q (|Q| = 1)
- `InertialState::quaternionRate` stores Q̇ directly (avoids conversion every frame)
- `UnitQuaternionConstraint` maintains normalization via Baumgarte stabilization

**Conversions**:
- ω = 2 Q̄ ⊗ Q̇ (quaternion rate → angular velocity)
- Q̇ = ½ Q ⊗ [0, ω] (angular velocity → quaternion rate)

**14-Component State Vector**: Position(3) + Quaternion(4) + Velocity(3) + QuaternionRate(4)

### Why Mirtich Algorithm for Inertia?

Previous tetrahedron decomposition had ~10-15% error with ad-hoc scaling. Mirtich (1996) provides machine-precision accuracy (< 1e-10 error) through hierarchical integral computation:

1. Projection integrals (2D line integrals over edges)
2. Face integrals (3D surface integrals)
3. Volume integrals (accumulated across faces)
4. Apply parallel axis theorem for centroid-relative inertia

**Key detail**: Includes vertex winding correction to align Qhull output with Mirtich's requirements.

### Force vs. Impulse Application

| Type | Method | Behavior | Use Case |
|------|--------|----------|----------|
| Force | `applyForce()` | Accumulated, integrated: Δv = (F/m)·dt | Gravity, thrust, drag |
| Impulse | `applyImpulse()` | Instantaneous: Δv = J/m | Collision response |

Forces are accumulated during a frame, then cleared after integration. Multiple systems (gravity, constraints, user input) can contribute independently.

### Move-Only AssetInertial

`AssetInertial` is move-only because it owns constraints via `std::vector<std::unique_ptr<Constraint>>`. Copy semantics would require deep-copying constraints, which is rarely needed and potentially expensive.

---

## Cross-Cutting Concerns

### Units (SI Throughout)

| Quantity | Unit |
|----------|------|
| Mass | kg |
| Force | N |
| Torque | N·m |
| Impulse | N·s |
| Position | m |
| Velocity | m/s |
| Angular velocity | rad/s |
| Inertia | kg·m² |

### Memory Ownership

| Component | Ownership |
|-----------|-----------|
| ConvexHull vertices/facets | Owned by value (`std::vector`) |
| AssetPhysical hull reference | Non-owning (caller ensures lifetime) |
| AssetInertial constraints | Owned via `std::unique_ptr` |
| InertialState | Value in AssetInertial |
| Inertia tensors | Cached by value |

### Thread Safety

- **ConvexHull**: Read-only ops thread-safe after construction
- **InertialState**: Value semantics, safe to copy
- **AssetInertial**: Not thread-safe (mutable state)
- **InertialCalculations**: Pure functions, thread-safe

---

## Integration with Physics Pipeline

```
1. Force Accumulation
   └── applyForce(), applyTorque(), applyForceAtPoint()

2. Constraint Enforcement
   └── UnitQuaternionConstraint, joints, contacts

3. Integration (Lagrangian mechanics)
   └── Integrator::step() computes accelerations, updates state

4. Collision Response
   └── applyImpulse(), applyAngularImpulse()

5. Force Clearing
   └── clearForces() prepares for next frame
```

Every `AssetInertial` owns a `UnitQuaternionConstraint` by default. Additional constraints added via `addConstraint()`.

---

## Related Documentation

- **Collision Detection**: [Collision/CLAUDE.md](../Collision/CLAUDE.md)
- **Constraints**: [Constraints/CLAUDE.md](../Constraints/CLAUDE.md)
- **Integration**: [Integration/CLAUDE.md](../Integration/CLAUDE.md)
- **Diagrams**: `docs/msd/msd-sim/Physics/` (force-application.puml, convex-hull.puml, mirtich-inertia-tensor.puml)
