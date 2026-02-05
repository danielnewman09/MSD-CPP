# PotentialEnergy System Architecture

> Architectural context for environmental potential energy fields.
> For API details, use MCP: `find_class PotentialEnergy`, `get_class_members GravityPotential`, etc.

## Architecture Overview

```
PotentialEnergy (Abstract interface)
    └── GravityPotential (Uniform gravitational field)
    └── [Future: TidalPotential, MagneticPotential, DragPotential]

WorldModel owns vector of PotentialEnergy instances
```

---

## Lagrangian Mechanics Context

The PotentialEnergy system implements generalized forces from energy gradients:

```
L = T - V  (Lagrangian = Kinetic - Potential)

F = -∂V/∂X  (Force from position gradient)
τ = -∂V/∂Q  (Torque from orientation gradient)
```

This formulation:
- Guarantees conservative forces
- Enables energy conservation tracking
- Provides a unified interface for all environmental forces

---

## Design Decisions

### Why Three Methods (force, torque, energy)?

- **Performance**: Callers can query only what they need
- **Energy tracking**: Explicit `computeEnergy()` for total energy (T + V)
- **Debugging**: Independent validation during testing

### Why Environmental Potentials Only?

- Per-object forces (thrusters, springs) handled by `AssetInertial` directly
- Environmental potentials apply uniformly to all objects (gravity, tidal)
- Clear separation enables efficient force accumulation

### GravityPotential Physics

```
V(r) = -m * g · r           (Potential energy)
F = m * g                    (Constant force, position-independent)
τ = 0                        (Uniform fields produce no torque)
```

Default: g = (0, 0, -9.81) m/s² (Earth standard, z-up)

---

## Integration with Physics Pipeline

```cpp
for (const auto& potential : potentialEnergies_) {
  netForce += potential->computeForce(state, mass);
  netTorque += potential->computeTorque(state, inertia);
}
integrator_->step(asset, netForce, netTorque, dt);
```

---

## Future Extensions

| Potential | Physics | Torque? |
|-----------|---------|---------|
| TidalPotential | Differential gravity | Yes (orientation-dependent) |
| MagneticPotential | τ = m × B | Yes (magnetic dipole) |
| DragPotential | F ∝ -v² | No (dissipative) |

---

## Querying This Module

Use MCP tools for API details:
- `find_class PotentialEnergy` — Abstract interface
- `find_class GravityPotential` — Uniform gravity implementation
- `get_class_members GravityPotential` — Force/torque/energy methods
- `search_documentation potential` — Find potential-related docs

---

## Related Documentation

- **Integration**: [Integration/CLAUDE.md](../Integration/CLAUDE.md) — How forces are applied
- **Design document**: `docs/designs/0030_lagrangian_quaternion_physics/`
