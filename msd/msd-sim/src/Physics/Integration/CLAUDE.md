# Integration Module Architecture Guide

> This document provides architectural context for the numerical integration system within msd-sim.
> It references PlantUML diagrams in `docs/designs/` for detailed integration schemes.

**Diagram**: [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml)

## Module Overview

The **Integration** module provides an abstract interface for numerical integration schemes in rigid body dynamics, enabling swappable integrators (Euler, RK4, Verlet, etc.) without modifying the physics orchestration layer. The module decouples integration mathematics from physics computation, allowing isolated testing and runtime algorithm selection.

**Introduced**: [Ticket: 0030_lagrangian_quaternion_physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md)
**Modified**: [Ticket: 0031_generalized_lagrange_constraints](../../../../../tickets/0031_generalized_lagrange_constraints.md) — Constraint vector support

## Architecture Overview

### Component Hierarchy

```
Integrator (Abstract interface)
    └── SemiImplicitEulerIntegrator (Symplectic implementation)
        └── ConstraintSolver (Lagrange multiplier constraints)
```

### Design Pattern

The module uses the **Strategy Pattern** to enable runtime selection of integration algorithms:
- `Integrator` defines the abstract interface for state advancement
- Concrete implementations (e.g., `SemiImplicitEulerIntegrator`) provide specific numerical methods
- `WorldModel` owns the integrator via `std::unique_ptr<Integrator>` and delegates physics updates

### Integration with Physics Pipeline

```
WorldModel::updatePhysics()
    ├── Compute external forces (gravity, user forces)
    ├── For each AssetInertial:
    │   ├── Gather constraints (quaternion normalization, joints, etc.)
    │   └── integrator_->step(state, force, torque, mass, inertia, constraints, dt)
    │       ├── Compute unconstrained accelerations
    │       ├── ConstraintSolver::solve() computes constraint forces
    │       ├── Apply total accelerations (external + constraint)
    │       ├── Update velocities
    │       ├── Update positions/orientations
    │       └── Enforce constraints (normalization)
    └── Synchronize ReferenceFrames
```

---

## Core Components

| Component | Location | Purpose |
|-----------|----------|---------|
| Integrator | `Integrator.hpp` | Abstract interface for numerical integration |
| SemiImplicitEulerIntegrator | `SemiImplicitEulerIntegrator.hpp`, `SemiImplicitEulerIntegrator.cpp` | Symplectic Euler integrator with constraint enforcement |

---

## Component Details

### Integrator

**Location**: `Integrator.hpp`
**Type**: Abstract interface (header-only)
**Introduced**: [Ticket: 0030_lagrangian_quaternion_physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md)
**Modified**: [Ticket: 0031_generalized_lagrange_constraints](../../../../../tickets/0031_generalized_lagrange_constraints.md)

#### Purpose

Abstract interface for numerical integration of rigid body dynamics. Decouples the integration scheme from physics computation, enabling:
- Swappable integrators (Euler, RK4, Verlet, etc.)
- Isolated testing of integration mathematics
- WorldModel as pure orchestrator without numerical method details

#### Key Interfaces

```cpp
class Integrator {
public:
  virtual ~Integrator() = default;

  /**
   * @brief Integrate state forward by one timestep
   * @param state Current inertial state (modified in place)
   * @param force Net force in world frame [N]
   * @param torque Net torque in world frame [N·m]
   * @param mass Object mass [kg]
   * @param inverseInertia Inverse inertia tensor in body frame [1/(kg·m²)]
   * @param constraints Vector of constraint pointers (non-owning)
   * @param dt Timestep [s]
   *
   * Ticket 0031 breaking change: Parameter changed from single QuaternionConstraint&
   * to std::vector<Constraint*>& to support multiple constraints per object.
   */
  virtual void step(InertialState& state,
                    const Coordinate& force,
                    const Coordinate& torque,
                    double mass,
                    const Eigen::Matrix3d& inverseInertia,
                    const std::vector<Constraint*>& constraints,
                    double dt) = 0;

protected:
  Integrator() = default;
  Integrator(const Integrator&) = default;
  Integrator& operator=(const Integrator&) = default;
  Integrator(Integrator&&) noexcept = default;
  Integrator& operator=(Integrator&&) noexcept = default;
};
```

#### Usage Example

```cpp
// Create integrator instance
auto integrator = std::make_unique<SemiImplicitEulerIntegrator>();

// Inject into WorldModel
worldModel.setIntegrator(std::move(integrator));

// Integration happens automatically in WorldModel::updatePhysics()
worldModel.update(std::chrono::milliseconds{16});
```

#### Thread Safety

**Stateless interface** — Implementations should be stateless and thread-safe for concurrent use with different state instances.

#### Error Handling

No exceptions defined at the interface level. Implementations may throw for invalid inputs (e.g., negative mass, singular inertia tensor).

#### Memory Management

- Pure abstract interface with protected constructor
- Owned by WorldModel via `std::unique_ptr<Integrator>`
- Default implementations can be swapped at runtime via `WorldModel::setIntegrator()`

#### Breaking Changes

**Ticket 0031**: Replaced single `QuaternionConstraint&` parameter with `std::vector<Constraint*>&` to support arbitrary constraint combinations per object. All integrator implementations must be updated to handle constraint vectors.

---

### SemiImplicitEulerIntegrator

**Location**: `SemiImplicitEulerIntegrator.hpp`, `SemiImplicitEulerIntegrator.cpp`
**Diagram**: [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml)
**Type**: Library component
**Introduced**: [Ticket: 0030_lagrangian_quaternion_physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md)
**Modified**: [Ticket: 0031_generalized_lagrange_constraints](../../../../../tickets/0031_generalized_lagrange_constraints.md)

#### Purpose

Symplectic integrator for rigid body dynamics using the semi-implicit Euler method. Provides better energy conservation than explicit Euler while remaining computationally efficient. This is the default integrator for WorldModel.

#### Key Features

- **First-order accurate**: O(dt) local truncation error
- **Symplectic**: Preserves phase space volume, better long-term energy behavior than explicit Euler
- **Velocity-first integration**: Updates velocities before positions using new velocities
- **Quaternion support**: Integrates quaternion orientation with constraint enforcement
- **Constraint integration**: Uses ConstraintSolver to enforce arbitrary constraint combinations
- **Simple and fast**: O(1) complexity per step, no matrix inversions beyond constraint solve

#### Integration Order

```
1. Compute unconstrained accelerations: a_free = F_ext / m, α_free = I^-1 * τ_ext
2. Solve constraint system: result = solver.solve(constraints, ...)
3. Apply constraint forces: a_total = a_free + F_c / m, α_total = α_free + I^-1 * τ_c
4. Update velocities: v_new = v_old + a_total * dt
5. Update positions: x_new = x_old + v_new * dt (uses NEW velocity)
6. Integrate quaternion: Q_new = Q_old + Q̇ * dt
7. Normalize quaternion (implicit constraint enforcement)
```

**Symplectic property**: Using the new velocity in the position update (step 5) is what makes this method symplectic, providing better energy conservation than explicit Euler which would use `v_old`.

#### Key Interfaces

```cpp
class SemiImplicitEulerIntegrator : public Integrator {
public:
  /**
   * @brief Construct integrator with default solver
   */
  SemiImplicitEulerIntegrator();

  /**
   * @brief Construct integrator with custom solver
   * @param solver ConstraintSolver instance (copied)
   */
  explicit SemiImplicitEulerIntegrator(ConstraintSolver solver);

  ~SemiImplicitEulerIntegrator() override = default;

  void step(InertialState& state,
            const Coordinate& force,
            const Coordinate& torque,
            double mass,
            const Eigen::Matrix3d& inverseInertia,
            const std::vector<Constraint*>& constraints,
            double dt) override;

  // Rule of Five
  SemiImplicitEulerIntegrator(const SemiImplicitEulerIntegrator&) = default;
  SemiImplicitEulerIntegrator& operator=(const SemiImplicitEulerIntegrator&) = default;
  SemiImplicitEulerIntegrator(SemiImplicitEulerIntegrator&&) noexcept = default;
  SemiImplicitEulerIntegrator& operator=(SemiImplicitEulerIntegrator&&) noexcept = default;

private:
  ConstraintSolver solver_;  // Constraint solver instance
};
```

#### Usage Example

```cpp
// Default construction (used by WorldModel constructor)
auto integrator = std::make_unique<SemiImplicitEulerIntegrator>();

// Or with custom solver configuration
ConstraintSolver customSolver{/* params */};
auto integratorCustom = std::make_unique<SemiImplicitEulerIntegrator>(customSolver);

// Inject into WorldModel
worldModel.setIntegrator(std::move(integrator));

// Integration occurs automatically during WorldModel::updatePhysics()
// User does not call step() directly
```

#### Constraint Enforcement

The integrator uses `ConstraintSolver` to compute Lagrange multipliers for all constraints:

```cpp
// Internal to step() method
ConstraintSolver::SolveResult result = solver_.solve(
    constraints, state, force, torque, mass, inverseInertia, dt);

if (result.converged) {
  // Apply constraint forces
  Coordinate constraintLinearForce = result.linearConstraintForce;
  Coordinate constraintAngularTorque = result.angularConstraintForce;

  linearAccelTotal = linearAccelFree + (constraintLinearForce / mass);
  angularAccelTotal = angularAccelFree + (inverseInertia * constraintAngularTorque);
}
// If solver didn't converge, proceed without constraint forces (graceful degradation)
```

**Graceful degradation**: If the constraint solver fails to converge (e.g., overconstrained system), the integrator proceeds with unconstrained motion rather than crashing. This allows simulation to continue with diagnostic warnings.

#### Thread Safety

**Stateless** — Thread-safe for concurrent use with different state instances. The `ConstraintSolver` member is value-semantic and does not maintain state between calls.

#### Error Handling

No exceptions thrown during normal operation. All computations are numerically stable for valid inputs. Invalid inputs (e.g., negative mass, singular inertia) are assumed to be validated at object creation.

#### Memory Management

- Contains single `ConstraintSolver` member (value semantics)
- Owned by WorldModel via `std::unique_ptr<Integrator>`
- No dynamic allocations during `step()` calls

#### Performance Characteristics

- **Complexity**: O(1) per object, dominated by constraint solve (O(n³) for n constraints)
- **Memory**: Single `ConstraintSolver` instance (< 100 bytes)
- **Typical timestep**: 16.67ms (60 FPS) provides stable integration
- **Energy drift**: < 1% over 10000 steps for unconstrained motion (validated by tests)

#### Numerical Stability

The semi-implicit Euler method has better stability than explicit Euler:
- **Explicit Euler**: Can exhibit exponential energy growth for stiff systems
- **Semi-Implicit Euler**: Energy-preserving for Hamiltonian systems (symplectic)
- **Quaternion normalization**: Maintains `|Q| = 1` within machine precision via constraint enforcement

**Recommended for**: General-purpose rigid body simulation with moderate timesteps (10-20 ms). For high-accuracy requirements, consider implementing higher-order integrators (RK4, Verlet) as future extensions.

#### Breaking Changes

**Ticket 0031**: Uses `ConstraintSolver` instead of direct `QuaternionConstraint::enforceConstraint()` call. Now supports arbitrary constraint combinations rather than single hard-coded quaternion constraint.

---

## Integration with Constraint System

The Integration module works closely with the Constraint system (see [`../Constraints/CLAUDE.md`](../Constraints/CLAUDE.md)):

### Constraint Flow

1. **Ownership**: `AssetInertial` owns constraints via `std::vector<std::unique_ptr<Constraint>>`
2. **Collection**: Before integration, WorldModel gathers non-owning `std::vector<Constraint*>` from each asset
3. **Solving**: Integrator passes constraints to `ConstraintSolver::solve()`
4. **Application**: Constraint forces are added to external forces before velocity update
5. **Enforcement**: Quaternion normalization applied after integration (implicit constraint)

### Default Constraints

Every `AssetInertial` automatically includes:
- **UnitQuaternionConstraint**: Maintains `Q^T * Q = 1` within 1e-10 tolerance

Users can add additional constraints:
```cpp
asset.addConstraint(std::make_unique<DistanceConstraint>(targetDistance));
asset.addConstraint(std::make_unique<MyCustomConstraint>(/* params */));
```

All constraints are enforced simultaneously by the solver, which computes Lagrange multipliers satisfying:

```
J·M⁻¹·Jᵀ·λ = -J·M⁻¹·F_ext - J̇·q̇ - α·C - β·Ċ
```

Where:
- `J` = constraint Jacobian matrix (stacked from all constraints)
- `M⁻¹` = inverse mass matrix
- `λ` = Lagrange multipliers
- `α`, `β` = Baumgarte stabilization parameters

---

## Future Extensions

### Additional Integrators

The abstract `Integrator` interface enables future implementations:

- **RK4Integrator**: Fourth-order Runge-Kutta for high-accuracy simulations
- **VerletIntegrator**: Velocity Verlet for energy-conserving long-term dynamics
- **ImplicitEulerIntegrator**: Unconditionally stable for stiff systems
- **AdaptiveIntegrator**: Variable timestep with error control

### Implementation Pattern

```cpp
class RK4Integrator : public Integrator {
public:
  void step(InertialState& state,
            const Coordinate& force,
            const Coordinate& torque,
            double mass,
            const Eigen::Matrix3d& inverseInertia,
            const std::vector<Constraint*>& constraints,
            double dt) override {
    // RK4 implementation with 4 evaluations per step
    // K1 = f(t, y)
    // K2 = f(t + dt/2, y + K1*dt/2)
    // K3 = f(t + dt/2, y + K2*dt/2)
    // K4 = f(t + dt, y + K3*dt)
    // y_new = y + (K1 + 2*K2 + 2*K3 + K4) * dt / 6
  }
};
```

### Timestep Adaptation

Future integrators may support adaptive timesteps based on error estimates:

```cpp
struct AdaptiveResult {
  bool accepted;
  double suggestedDt;
};

class AdaptiveIntegrator : public Integrator {
  AdaptiveResult stepWithErrorControl(/* params */, double dtInitial);
};
```

---

## Testing

### Test Organization

```
test/Physics/Integration/
└── QuaternionPhysicsTest.cpp  # 16 integration tests covering acceptance criteria
```

### Test Coverage

- Quaternion normalization maintenance over 10000 steps
- Energy conservation validation for unconstrained motion
- Constraint enforcement accuracy (1e-10 tolerance)
- Angular velocity to quaternion rate conversion
- Symplectic property validation (energy drift bounds)

### Running Tests

```bash
cmake --build --preset debug-tests-only
ctest --preset debug -R QuaternionPhysics
```

---

## Coding Standards

This module follows the project-wide coding standards defined in the [root CLAUDE.md](../../../../../CLAUDE.md#coding-standards).

Key standards applied:
- **Initialization**: Brace initialization `{}` for constructors
- **Naming**: `PascalCase` for classes, `camelCase` for methods
- **Return Values**: Modify state in place via reference parameter (integration convention)
- **Memory**: Value semantics for ConstraintSolver, unique ownership for Integrator instances

See the [root CLAUDE.md](../../../../../CLAUDE.md#coding-standards) for complete details.

---

## Getting Help

### For AI Assistants

1. This document provides complete architectural context for numerical integration
2. Review [`../Constraints/CLAUDE.md`](../Constraints/CLAUDE.md) for constraint system details
3. Review [`../RigidBody/InertialState.hpp`](../RigidBody/InertialState.hpp) for state representation
4. Check [ticket 0030](../../../../../tickets/0030_lagrangian_quaternion_physics.md) for original design rationale
5. Check [ticket 0031](../../../../../tickets/0031_generalized_lagrange_constraints.md) for constraint generalization

### For Developers

- **Swapping integrators**: Use `WorldModel::setIntegrator(std::move(newIntegrator))`
- **Custom integrators**: Inherit from `Integrator` and implement `step()` method
- **Timestep selection**: 16.67ms (60 FPS) recommended for semi-implicit Euler
- **Constraint integration**: See [`../Constraints/CLAUDE.md`](../Constraints/CLAUDE.md) for constraint API

---

## References

### Design Documents

- [Ticket 0030: Lagrangian Quaternion Physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md) — Original integration system design
- [Ticket 0031: Generalized Lagrange Constraints](../../../../../tickets/0031_generalized_lagrange_constraints.md) — Constraint vector support
- [Design: 0030_lagrangian_quaternion_physics.puml](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml) — Integration architecture diagram

### Related Components

- [`../Constraints/`](../Constraints/) — Constraint system for Lagrange multiplier enforcement
- [`../PotentialEnergy/`](../PotentialEnergy/) — Environmental forces (gravity, etc.)
- [`../RigidBody/InertialState.hpp`](../RigidBody/InertialState.hpp) — State representation
- [`../../Environment/WorldModel.hpp`](../../Environment/WorldModel.hpp) — Physics orchestration

### External References

- [Symplectic Integration](https://en.wikipedia.org/wiki/Symplectic_integrator) — Phase space preservation
- [Semi-Implicit Euler Method](https://en.wikipedia.org/wiki/Semi-implicit_Euler_method) — Stability analysis
- [Lagrangian Mechanics](https://en.wikipedia.org/wiki/Lagrangian_mechanics) — Theoretical foundation
