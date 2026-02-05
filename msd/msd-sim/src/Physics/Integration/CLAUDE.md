# Integration Module Architecture

> Architectural context for the numerical integration system.
> For API details, use MCP: `find_class Integrator`, `get_class_members SemiImplicitEulerIntegrator`, etc.

## Architecture Overview

```
Integrator (Abstract interface)
    └── SemiImplicitEulerIntegrator (Symplectic, velocity-first)
        └── ConstraintSolver (Lagrange multipliers)

WorldModel owns Integrator via std::unique_ptr (Strategy pattern)
```

---

## Design Decisions

### Why Semi-Implicit Euler?

Semi-implicit (symplectic) Euler updates velocity before position:
1. `v_new = v_old + a*dt`
2. `x_new = x_old + v_new*dt` (uses NEW velocity)

This preserves phase space volume, providing better energy conservation than explicit Euler. Energy drift < 1% over 10000 steps for unconstrained motion.

### Why Strategy Pattern?

The abstract `Integrator` interface enables:
- Swappable integrators (Euler, RK4, Verlet) at runtime
- Isolated testing of integration mathematics
- WorldModel remains a pure orchestrator

### Constraint Integration

The integrator uses `ConstraintSolver` to compute Lagrange multipliers:
1. Compute unconstrained accelerations: `a_free = F_ext / m`
2. Solve constraint system: `solver.solve(constraints, ...)`
3. Apply total accelerations: `a_total = a_free + F_constraint / m`
4. Update velocities and positions
5. Normalize quaternion (implicit constraint enforcement)

**Graceful degradation**: If solver doesn't converge, integration proceeds without constraint forces rather than crashing.

---

## Integration with Physics Pipeline

```
WorldModel::updatePhysics()
    ├── Compute external forces (gravity, user forces)
    └── For each AssetInertial:
        ├── Gather constraints (quaternion normalization, joints, etc.)
        └── integrator_->step(state, force, torque, mass, inertia, constraints, dt)
```

Every `AssetInertial` automatically includes `UnitQuaternionConstraint` to maintain `|Q| = 1`.

---

## Performance Characteristics

| Metric | Value |
|--------|-------|
| Complexity | O(1) per object (+ O(n³) for n constraints) |
| Memory | Single ConstraintSolver instance (< 100 bytes) |
| Typical timestep | 16.67ms (60 FPS) |
| Energy drift | < 1% over 10000 steps |

---

## Querying This Module

Use MCP tools for API details:
- `find_class Integrator` — Abstract interface
- `find_class SemiImplicitEulerIntegrator` — Default implementation
- `get_class_members SemiImplicitEulerIntegrator` — step() signature
- `search_documentation integration` — Find integration-related docs

---

## Related Documentation

- **Constraints**: [Constraints/CLAUDE.md](../Constraints/CLAUDE.md) — ConstraintSolver details
- **State representation**: `InertialState.hpp` — 14-component state vector
- **Design document**: `docs/designs/0030_lagrangian_quaternion_physics/`

## References

- [Symplectic Integration](https://en.wikipedia.org/wiki/Symplectic_integrator)
- [Semi-Implicit Euler Method](https://en.wikipedia.org/wiki/Semi-implicit_Euler_method)
