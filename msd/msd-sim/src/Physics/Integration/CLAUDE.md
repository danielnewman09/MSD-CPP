# Integration Module Architecture

> Architectural context for the numerical integration system.
> For API details, use MCP: `find_class Integrator`, `get_class_members SemiImplicitEulerIntegrator`, etc.

## Architecture Overview

```
Integrator (Abstract interface)
    └── SemiImplicitEulerIntegrator (Symplectic, velocity-first)
        └── Quaternion normalization via state.orientation.normalize()

WorldModel owns Integrator via std::unique_ptr (Strategy pattern)
```

**Note**: As of ticket 0045, the integrator no longer owns a `ConstraintSolver` or accepts constraints. Quaternion drift correction is handled via direct normalization.

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

### Quaternion Normalization

The integrator maintains quaternion normalization via direct normalization:
1. Compute accelerations: `a = F_ext / m`, `α = I⁻¹ * τ_ext`
2. Update velocities: `v_new = v_old + a*dt`, `ω_new = ω_old + α*dt`
3. Update positions: `x_new = x_old + v_new*dt`
4. Update quaternion: `Q_new = Q_old + Q̇*dt` (where `Q̇ = ½*Q⊗[0,ω]`)
5. **Normalize quaternion**: `Q_new.normalize()` — Maintains `|Q| = 1` within machine precision

**Historical note**: Prior to ticket 0045, the integrator used `ConstraintSolver` to enforce `UnitQuaternionConstraint` via Lagrange multipliers. This was removed as redundant with direct normalization, eliminating ~250 lines of solver code and the integrator's ConstraintSolver dependency.

---

## Integration with Physics Pipeline

```
WorldModel::updatePhysics()
    ├── Compute external forces (gravity, user forces)
    └── For each AssetInertial:
        └── integrator_->step(state, force, torque, mass, inertia, dt)
            └── Normalizes quaternion via state.orientation.normalize()
```

**Note**: As of ticket 0045, the integrator no longer accepts a `constraints` parameter. WorldModel does not gather or pass constraints to the integrator. Quaternion normalization is the integrator's responsibility via direct `normalize()` call.

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
