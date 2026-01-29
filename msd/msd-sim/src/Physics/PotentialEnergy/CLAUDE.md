# PotentialEnergy System Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams for detailed component relationships.

## System Overview

**PotentialEnergy** is a sub-module within `msd-sim/Physics` that provides an extensible framework for environmental potential energy fields in Lagrangian mechanics. It enables computation of generalized forces and torques from energy gradients, supporting physics simulations with gravity, tidal forces, magnetic fields, and other environmental potentials.

**Diagram**: [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml)

---

## Lagrangian Mechanics Context

The PotentialEnergy system implements the potential energy term in Lagrangian mechanics:

```
L = T - V  (Lagrangian = Kinetic Energy - Potential Energy)

Generalized forces derived from potential energy gradients:
  F = -∂V/∂X  (Linear force from position gradient)
  τ = -∂V/∂Q  (Torque from orientation gradient)
```

This formulation enables:
- **Unified force computation**: All environmental forces derived from energy gradients
- **Energy conservation**: Symplectic integrators preserve total energy (T + V)
- **Extensibility**: New force fields added by implementing PotentialEnergy interface
- **Physical correctness**: Gradient formulation guarantees conservative forces

---

## Architecture Overview

### Component Hierarchy

```
PotentialEnergy (Abstract interface)
    ├── GravityPotential (Uniform gravitational field)
    ├── TidalPotential (Future: Orientation-dependent tidal forces)
    ├── MagneticPotential (Future: Magnetic torques)
    └── DragPotential (Future: Velocity-dependent dissipation)

WorldModel owns vector of PotentialEnergy instances
    └── Integrator queries potentials during physics updates
```

### Core Components

| Component | Location | Purpose | Type |
|-----------|----------|---------|------|
| PotentialEnergy | `PotentialEnergy.hpp` | Abstract interface for energy fields | Header-only |
| GravityPotential | `GravityPotential.hpp`, `GravityPotential.cpp` | Uniform gravity implementation | Library component |

---

## Component Details

### PotentialEnergy

**Location**: `PotentialEnergy.hpp`
**Type**: Abstract interface (header-only)
**Introduced**: [Ticket: 0030_lagrangian_quaternion_physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md)

#### Purpose
Abstract interface for environmental potential energy fields in Lagrangian mechanics. Enables extensible force computation without modifying WorldModel or integration infrastructure.

#### Key Features
- **Force computation**: Compute linear force F = -∂V/∂X from position gradient
- **Torque computation**: Compute torque τ = -∂V/∂Q from orientation gradient
- **Energy queries**: Compute potential energy V for energy conservation tracking
- **Extensibility**: New force fields added by implementing three pure virtual methods
- **Type safety**: Protected constructor prevents direct instantiation

#### Key Interfaces
```cpp
class PotentialEnergy {
public:
  virtual ~PotentialEnergy() = default;

  /**
   * @brief Compute linear force from potential energy gradient
   * @param state Current inertial state (position, velocity, orientation)
   * @param mass Object mass [kg]
   * @return Generalized force F = -∂V/∂X [N]
   */
  virtual Coordinate computeForce(const InertialState& state, double mass) const = 0;

  /**
   * @brief Compute torque from potential energy gradient
   * @param state Current inertial state
   * @param inertia Inertia tensor in world frame [kg·m²]
   * @return Generalized torque τ = -∂V/∂Q [N·m]
   */
  virtual Coordinate computeTorque(const InertialState& state,
                                   const Eigen::Matrix3d& inertia) const = 0;

  /**
   * @brief Compute potential energy
   * @param state Current inertial state
   * @param mass Object mass [kg]
   * @return Potential energy V [J]
   */
  virtual double computeEnergy(const InertialState& state, double mass) const = 0;

protected:
  PotentialEnergy() = default;
  PotentialEnergy(const PotentialEnergy&) = default;
  PotentialEnergy& operator=(const PotentialEnergy&) = default;
  PotentialEnergy(PotentialEnergy&&) noexcept = default;
  PotentialEnergy& operator=(PotentialEnergy&&) noexcept = default;
};
```

#### Implementation Requirements

Concrete implementations must:
1. **Implement all three pure virtual methods**: `computeForce()`, `computeTorque()`, `computeEnergy()`
2. **Return consistent results**: Force/torque must be negative gradients of energy
3. **Be thread-safe for queries**: Read-only after construction
4. **Handle edge cases**: Invalid state (negative mass) should throw `std::invalid_argument`
5. **Use SI units**: Forces [N], torques [N·m], energy [J]

#### Usage Example
```cpp
// Implement custom potential energy field
class TidalPotential : public PotentialEnergy {
public:
  TidalPotential(const Coordinate& moonPosition, double moonMass)
    : moonPosition_{moonPosition}, moonMass_{moonMass} {}

  Coordinate computeForce(const InertialState& state, double mass) const override {
    // Compute tidal force from moon's gravitational gradient
    Coordinate r = state.position - moonPosition_;
    double r3 = std::pow(r.norm(), 3);
    return -G * moonMass_ * mass * r / r3;  // Simplified
  }

  Coordinate computeTorque(const InertialState& state,
                           const Eigen::Matrix3d& inertia) const override {
    // Tidal torques arise from differential gravitational forces
    // across the object's extent (orientation-dependent)
    // [Complex calculation omitted]
    return Coordinate{0, 0, 0};  // Placeholder
  }

  double computeEnergy(const InertialState& state, double mass) const override {
    Coordinate r = state.position - moonPosition_;
    return -G * moonMass_ * mass / r.norm();  // Gravitational potential
  }

private:
  Coordinate moonPosition_;
  double moonMass_;
  static constexpr double G = 6.674e-11;  // Gravitational constant [m³/kg/s²]
};

// Add to WorldModel
worldModel.addPotentialEnergy(
    std::make_unique<TidalPotential>(moonPosition, moonMass));
```

#### Thread Safety
**Read-only after construction** — Implementations must be thread-safe for concurrent force/torque queries.

#### Error Handling
- Implementations may throw `std::invalid_argument` for invalid state (negative mass)
- Interface itself does not throw exceptions

#### Memory Management
- Pure abstract interface with protected constructor
- Owned by WorldModel via `std::vector<std::unique_ptr<PotentialEnergy>>`
- Implementations should use value semantics for internal state

#### Design Rationale

**Why abstract interface?**
- Extensibility: New force fields added without modifying WorldModel
- Polymorphism: WorldModel iterates over heterogeneous potential energy vector
- Separation of concerns: Force computation decoupled from integration

**Why three methods instead of one?**
- Performance: Energy gradient computation can be expensive; separate methods allow callers to query only what they need
- Energy conservation: Explicit energy query enables tracking total energy (T + V)
- Debugging: Independent force/torque/energy validation during testing

**Why environmental potentials only?**
- Per-object forces (thrusters, springs) handled by `AssetInertial` directly
- Environmental potentials apply uniformly to all objects (gravity, tidal)
- Clear separation enables efficient force accumulation

---

### GravityPotential

**Location**: `GravityPotential.hpp`, `GravityPotential.cpp`
**Type**: Library component
**Introduced**: [Ticket: 0030_lagrangian_quaternion_physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md)

#### Purpose
Implements uniform gravitational field potential energy. Produces constant force F = m*g independent of position and orientation, with zero torque (uniform fields do not couple to orientation).

#### Key Features
- **Constant force**: F = m*g regardless of position or orientation
- **Zero torque**: Uniform gravity does not produce torque (τ = 0)
- **Configurable gravity vector**: Arbitrary direction and magnitude
- **Default Earth gravity**: g = (0, 0, -9.81) m/s² in z-up convention

#### Physics

```
Potential energy: V(r) = -m * g · r
  where r is position, g is gravity vector

Force (linear): F = -∂V/∂r = m * g  (constant, independent of position)

Torque (angular): τ = -∂V/∂Q = 0  (orientation-independent field)

Energy sign convention:
  - For downward gravity (0, 0, -9.81) and z-up coordinate system
  - Object at height z=100m has V = -m*g·r = -m*(0,0,-9.81)·(x,y,100) = 981*m J
  - Positive potential energy above origin (matches intuition)
```

#### Key Interfaces
```cpp
class GravityPotential : public PotentialEnergy {
public:
  /**
   * @brief Construct gravitational field with specified acceleration vector
   * @param gravityVector Gravitational acceleration [m/s²], e.g. (0, 0, -9.81)
   */
  explicit GravityPotential(const Coordinate& gravityVector);

  ~GravityPotential() override = default;

  // PotentialEnergy interface implementation
  Coordinate computeForce(const InertialState& state, double mass) const override;
  Coordinate computeTorque(const InertialState& state,
                           const Eigen::Matrix3d& inertia) const override;
  double computeEnergy(const InertialState& state, double mass) const override;

  // Gravity configuration
  void setGravity(const Coordinate& gravityVector);
  const Coordinate& getGravity() const;

  // Rule of Five (all defaulted for value semantics)
  GravityPotential(const GravityPotential&) = default;
  GravityPotential& operator=(const GravityPotential&) = default;
  GravityPotential(GravityPotential&&) noexcept = default;
  GravityPotential& operator=(GravityPotential&&) noexcept = default;

private:
  Coordinate g_{0.0, 0.0, -9.81};  // Gravitational acceleration [m/s²]
};
```

#### Implementation Details

**computeForce()**:
```cpp
Coordinate computeForce(const InertialState& state, double mass) const {
  // Uniform gravitational force: F = m * g
  // Force is independent of position (constant field)
  return g_ * mass;
}
```

**computeTorque()**:
```cpp
Coordinate computeTorque(const InertialState& state,
                         const Eigen::Matrix3d& inertia) const {
  // Uniform gravity produces no torque (orientation-independent)
  // τ = -∂V/∂Q = 0 since V does not depend on orientation Q
  return Coordinate{0.0, 0.0, 0.0};
}
```

**computeEnergy()**:
```cpp
double computeEnergy(const InertialState& state, double mass) const {
  // Gravitational potential energy: V = -m * g · r
  // For uniform field g = (0, 0, -9.81) at height z=10:
  //   V = -m * (0, 0, -9.81) · (0, 0, 10) = -m * (-98.1) = +98.1 * m
  // This gives positive potential energy for objects above the origin,
  // which correctly reflects their potential to do work by falling.
  return -mass * g_.dot(state.position);
}
```

#### Usage Example
```cpp
// Create uniform gravity field (Earth standard, z-up convention)
auto gravity = std::make_unique<GravityPotential>(Coordinate{0, 0, -9.81});

// Add to WorldModel
worldModel.addPotentialEnergy(std::move(gravity));

// Forces automatically computed during physics updates
// WorldModel::updatePhysics() iterates over all potentials:
for (auto& asset : inertialAssets) {
  Coordinate netForce{0, 0, 0};
  for (const auto& potential : potentialEnergies_) {
    netForce += potential->computeForce(asset.getState(), asset.getMass());
  }
  // Integrate using netForce...
}

// Manual force computation for 10 kg object
InertialState state;
state.position = Coordinate{0, 0, 100};  // 100m above origin
Coordinate force = gravity->computeForce(state, 10.0);
// Returns: (0, 0, -98.1) N

double energy = gravity->computeEnergy(state, 10.0);
// Returns: 9810 J (positive, as expected at height)
```

#### Configuration

**Changing gravity** (should only be done during initialization):
```cpp
// Low gravity (Moon: 1.62 m/s²)
gravity.setGravity(Coordinate{0, 0, -1.62});

// Zero gravity (free space)
gravity.setGravity(Coordinate{0, 0, 0});

// Sideways gravity (unusual but valid)
gravity.setGravity(Coordinate{-9.81, 0, 0});
```

**Warning**: `setGravity()` modifies mutable state. Only call during initialization to preserve thread safety.

#### Thread Safety
**Immutable after construction** — Thread-safe for concurrent force queries if `setGravity()` is not called after initialization.

#### Error Handling
- No exceptions thrown
- Zero gravity vector is valid (free-space dynamics)
- Negative mass handled by caller (not validated here)

#### Memory Management
- Single `Coordinate` member (24 bytes)
- Value semantics with compiler-generated copy/move
- Owned by WorldModel via `std::unique_ptr<PotentialEnergy>`

#### Validation

From `test/Physics/Integration/QuaternionPhysicsTest.cpp`:
```cpp
TEST_CASE("Gravity applies constant downward force") {
  GravityPotential gravity{Coordinate{0, 0, -9.81}};
  InertialState state;
  state.position = Coordinate{0, 0, 100};  // Height irrelevant

  Coordinate force = gravity.computeForce(state, 10.0);
  REQUIRE(force.z() == Approx(-98.1));  // 10kg * -9.81 m/s²

  Coordinate torque = gravity.computeTorque(state, Eigen::Matrix3d::Identity());
  REQUIRE(torque.norm() == Approx(0.0));  // No torque
}
```

---

## Integration with Physics Pipeline

### WorldModel Integration

The PotentialEnergy system integrates with WorldModel's physics update loop:

```cpp
class WorldModel {
public:
  void addPotentialEnergy(std::unique_ptr<PotentialEnergy> potential) {
    potentialEnergies_.push_back(std::move(potential));
  }

  void updatePhysics(double dt) {
    for (auto& asset : getInertialObjects()) {
      // Accumulate forces from all potential energies
      Coordinate netForce{0, 0, 0};
      Coordinate netTorque{0, 0, 0};

      for (const auto& potential : potentialEnergies_) {
        netForce += potential->computeForce(
            asset.getInertialState(), asset.getMass());
        netTorque += potential->computeTorque(
            asset.getInertialState(), asset.getInertiaTensor());
      }

      // Pass to integrator (SemiImplicitEulerIntegrator by default)
      integrator_->step(asset, netForce, netTorque, dt);
    }
  }

private:
  std::vector<std::unique_ptr<PotentialEnergy>> potentialEnergies_;
  std::unique_ptr<Integrator> integrator_;
};
```

### Integration with Integrator

See [`../Integration/CLAUDE.md`](../Integration/CLAUDE.md) for detailed integrator documentation.

The integrator receives accumulated forces from potential energies and applies them during numerical integration:

```cpp
void SemiImplicitEulerIntegrator::step(AssetInertial& asset,
                                       const Coordinate& externalForce,
                                       const Coordinate& externalTorque,
                                       double dt) {
  // Compute accelerations from external forces (includes potential energies)
  Coordinate linearAccel = externalForce / asset.getMass();
  AngularRate angularAccel = asset.getInverseInertiaTensor() * externalTorque;

  // Semi-implicit Euler: velocity-first integration
  asset.velocity += linearAccel * dt;
  asset.position += asset.velocity * dt;

  asset.angularVelocity += angularAccel * dt;
  asset.orientation += asset.angularVelocity * dt;

  // Enforce constraints (quaternion normalization, etc.)
  constraintSolver_.solve(asset, dt);
}
```

---

## Design Patterns in Use

### Strategy Pattern
**Applied to**: PotentialEnergy interface with multiple implementations (GravityPotential, future TidalPotential, etc.)
**Purpose**: Enables runtime polymorphism for heterogeneous force fields without modifying WorldModel.

### Template Method (implicit)
**Applied to**: PotentialEnergy interface structure
**Purpose**: Enforces consistent interface (force, torque, energy) while allowing implementation variation.

### Dependency Inversion
**Applied to**: WorldModel depends on PotentialEnergy abstraction, not concrete implementations
**Purpose**: WorldModel doesn't need to know about specific force field types, only the interface.

---

## Future Extensions

### TidalPotential
**Purpose**: Orientation-dependent tidal forces from nearby massive bodies
**Physics**: Differential gravitational forces across object extent
**Torque**: Non-zero (couples to orientation)

```cpp
class TidalPotential : public PotentialEnergy {
  // Compute gravitational gradient torque: τ = 3μ/r³ * (n × I·n)
  // where n = direction to massive body, I = inertia tensor
};
```

### MagneticPotential
**Purpose**: Magnetic torques for spacecraft attitude control
**Physics**: Interaction between magnetic dipole and ambient field
**Torque**: τ = m × B (dipole moment cross magnetic field)

```cpp
class MagneticPotential : public PotentialEnergy {
  // Compute magnetic torque for spacecraft with magnetic dipole
  // Used in magnetic attitude control systems
};
```

### DragPotential
**Purpose**: Velocity-dependent atmospheric dissipation
**Physics**: Non-conservative force, dissipates energy
**Note**: Requires extension to support velocity-dependent potentials

```cpp
class DragPotential : public PotentialEnergy {
  // F = -½ ρ v² C_d A * v̂
  // where ρ = air density, v = velocity, C_d = drag coefficient, A = area
};
```

### SpringPotential
**Purpose**: Per-object elastic connections (springs, cables)
**Physics**: Hooke's law F = -k*x for spring constant k, displacement x
**Note**: May require multi-object interface extension

```cpp
class SpringPotential : public PotentialEnergy {
  // V = ½ k (|r_A - r_B| - L₀)²
  // F = -k (|r| - L₀) * r̂
};
```

---

## Cross-Cutting Concerns

### Units
All quantities use SI units:
- Force: Newtons [N]
- Torque: Newton-meters [N·m]
- Energy: Joules [J]
- Mass: kilograms [kg]
- Acceleration: meters per second squared [m/s²]
- Position: meters [m]

### Error Handling Strategy
- **Invalid mass**: Implementations should throw `std::invalid_argument` for mass <= 0
- **No exceptions in interface**: Base class does not throw
- **Validation in constructors**: Implementations validate parameters at construction

### Memory Management
- **Ownership**: WorldModel owns potentials via `std::unique_ptr`
- **Value semantics**: Implementations should use value types for internal state
- **No heap allocations**: Avoid dynamic allocations during force queries (performance)

### Thread Safety Conventions
- **Read-only after construction**: Implementations must be thread-safe for concurrent queries
- **Configuration methods**: `setGravity()` etc. should only be called during initialization
- **Const correctness**: All query methods (computeForce, computeTorque, computeEnergy) are const

---

## Performance Considerations

### Force Computation Frequency
- Called every physics update (typically 60 Hz = 60 times per second)
- For n objects and m potentials: n × m force queries per frame
- Example: 100 objects, 3 potentials = 300 calls/frame = 18,000 calls/second

### Optimization Strategies
1. **Avoid heap allocations**: Use stack temporaries for force computation
2. **Cache constants**: Precompute values in constructor (gravitational constant, etc.)
3. **Early exit**: Return zero for potentials that don't apply torque (GravityPotential)
4. **Vectorization**: Eigen uses SIMD for vector operations (automatic)

### Profiling Results
From prototype benchmarks (ticket 0030):
- GravityPotential::computeForce(): ~5 ns per call (negligible overhead)
- Total potential energy overhead: < 1% of frame time for 100 objects

---

## Build & Configuration

### Dependencies
- **Eigen3**: Linear algebra (Coordinate, Matrix3d)
- **msd-sim/Environment**: InertialState, Coordinate types

### Building This Component

This module is built as part of `msd-sim`:

```bash
# From project root
cmake --preset conan-debug
cmake --build --preset debug-sim-only
```

### Test Targets

PotentialEnergy tests are included in:
- `test/Physics/Integration/QuaternionPhysicsTest.cpp` — Integration tests with Lagrangian mechanics
- Future: Dedicated `test/Physics/PotentialEnergy/PotentialEnergyTest.cpp` for unit tests

---

## Testing

### Current Test Coverage

From `test/Physics/Integration/QuaternionPhysicsTest.cpp` (ticket 0030):
```cpp
TEST_CASE("GravityPotential produces constant downward force") {
  GravityPotential gravity{Coordinate{0, 0, -9.81}};
  InertialState state;
  state.position = Coordinate{0, 0, 50};  // Position irrelevant

  Coordinate force = gravity.computeForce(state, 10.0);
  REQUIRE(force.x() == Approx(0.0));
  REQUIRE(force.y() == Approx(0.0));
  REQUIRE(force.z() == Approx(-98.1));

  // Verify torque is zero
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
  Coordinate torque = gravity.computeTorque(state, inertia);
  REQUIRE(torque.norm() == Approx(0.0));
}

TEST_CASE("Free-fall simulation conserves energy") {
  // 10 kg object dropped from 100m height
  // Verify that E_total = E_kinetic + E_potential remains constant
  // [Full test implementation validates energy conservation]
}
```

### Future Testing Needs
- Unit tests for GravityPotential in isolation
- Edge case tests (zero gravity, negative mass handling)
- Performance benchmarks for force computation
- Tests for future potential energy implementations (Tidal, Magnetic)

---

## Coding Standards

This module follows the project-wide coding standards defined in the [root CLAUDE.md](../../../../../CLAUDE.md#coding-standards).

Key standards applied in this module:
- **Initialization**: Brace initialization `{}`, default member initialization
- **Naming**: `PascalCase` for classes, `camelCase` for methods, `snake_case_` for members
- **Return Values**: Return by value (Coordinate, double), no output parameters
- **Memory**: Value semantics for implementations, unique_ptr for ownership transfer

See the [root CLAUDE.md](../../../../../CLAUDE.md#coding-standards) for complete details and examples.

---

## Getting Help

### For AI Assistants
1. This document provides complete architectural context for the PotentialEnergy system
2. Review [`../Integration/CLAUDE.md`](../Integration/CLAUDE.md) for integrator interaction
3. Review [`../../Environment/CLAUDE.md`](../../Environment/CLAUDE.md) for InertialState and Coordinate types
4. Review [`../CLAUDE.md`](../CLAUDE.md) for overall Physics module architecture
5. Check [root CLAUDE.md](../../../../../CLAUDE.md) for project-wide conventions

### For Developers
- **Adding new potential energies**: Implement PotentialEnergy interface with three methods
- **Configuring gravity**: Use GravityPotential::setGravity() during initialization
- **Energy conservation**: Query computeEnergy() to track total system energy
- **Performance tuning**: Avoid heap allocations in force computation methods
- **Debugging forces**: Print netForce from WorldModel::updatePhysics() accumulation loop
