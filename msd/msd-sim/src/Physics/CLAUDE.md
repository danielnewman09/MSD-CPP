# Physics Module Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/msd/msd-sim/Physics/` for detailed component relationships.

**Diagram**: [`physics-core.puml`](../../../../../docs/msd/msd-sim/Physics/physics-core.puml)

## Project Overview

**Physics** is a core module within `msd-sim` that provides force-based rigid body dynamics and collision detection. It includes convex hull representation, collision detection and response (see [Collision/CLAUDE.md](Collision/CLAUDE.md)), inertia tensor calculation, dynamic state management, and physics component integration for simulation objects.

## Architecture Overview

### High-Level Architecture

The Physics module provides a layered architecture for rigid body simulation:

```
PhysicsComponent (Object integration)
    ├── ConvexHull (Collision geometry)
    │   └── Qhull (External dependency)
    ├── DynamicState (Velocities/accelerations)
    └── InertialCalculations (Inertia tensors)

Collision System (see Collision/CLAUDE.md)
    ├── GJK (Collision detection algorithm)
    ├── EPA (Contact information extraction)
    └── ContactConstraint (Constraint-based response)
```

### Core Components

| Component | Location | Purpose | Documentation |
|-----------|----------|---------|---------------|
| **RigidBody System** | **`RigidBody/`** | **Rigid body representation, convex hulls, inertia tensors, quaternion orientation** | **[RigidBody/CLAUDE.md](RigidBody/CLAUDE.md)** |
| **Collision System** | **`Collision/`** | **GJK/EPA collision detection and constraint-based response** | **[Collision/CLAUDE.md](Collision/CLAUDE.md)** |
| **Integration System** | **`Integration/`** | **Numerical integration framework** | **[Integration/CLAUDE.md](Integration/CLAUDE.md)** |
| **PotentialEnergy System** | **`PotentialEnergy/`** | **Environmental potential energy fields for Lagrangian mechanics** | **[PotentialEnergy/CLAUDE.md](PotentialEnergy/CLAUDE.md)** |
| **Constraint System** | **`Constraints/`** | **Lagrange multiplier constraint framework** | **[Constraints/CLAUDE.md](Constraints/CLAUDE.md)** |

---

## Component Details

### RigidBody System

**Location**: `RigidBody/`
**Documentation**: [RigidBody/CLAUDE.md](RigidBody/CLAUDE.md)

#### Overview

The RigidBody sub-module provides the core rigid body representation for dynamic objects in the simulation. It includes convex hull geometry, inertial properties calculation, quaternion-based orientation, and complete dynamic state representation.

**Key Components**:
- **AssetInertial**: Complete dynamic rigid body with mass, inertia, forces, impulses, and constraints
- **AssetPhysical**: Base geometric element with collision hull and reference frame
- **InertialState**: Complete kinematic state with quaternion-based orientation (14-component state vector)
- **ConvexHull**: 3D convex hull via Qhull for collision detection and mass properties
- **InertialCalculations**: Mirtich algorithm for machine-precision inertia tensor computation
- **AssetEnvironment**: Stationary environmental objects (no dynamics)

#### Quick Example

```cpp
// Create dynamic object
ConvexHull hull{collisionPoints};
ReferenceFrame frame{Coordinate{0, 0, 10}};
AssetInertial asset{1, 100, hull, 50.0, frame, 0.8};  // 50kg, e=0.8

// Apply forces
asset.applyForce(CoordinateRate{10, 0, 0});
asset.applyForceAtPoint(CoordinateRate{5, 0, 0}, Coordinate{0, 0, 2.0});

// Integration (see Integration/CLAUDE.md)
integrator->step(asset, constraints, dt);
asset.clearForces();

// Impulses for collision response (see Collision/CLAUDE.md)
asset.applyImpulse(Coordinate{0, 0, 100});
```

**For complete RigidBody documentation, see [RigidBody/CLAUDE.md](RigidBody/CLAUDE.md)**.

---

### Collision System

**Location**: Collision components are mixed into the main Physics directory
**Documentation**: [Collision/CLAUDE.md](Collision/CLAUDE.md)

#### Overview
The collision system provides GJK/EPA-based collision detection and Lagrangian constraint-based collision response. For complete architectural details, algorithm descriptions, usage examples, and integration with the physics pipeline, see the dedicated collision documentation.

#### Key Components
- **CollisionHandler**: Orchestrates GJK and EPA algorithms
- **GJK**: Gilbert-Johnson-Keerthi collision detection
- **EPA**: Expanding Polytope Algorithm for contact information
- **CollisionResult**: Contact manifold with up to 4 contact pairs
- **ContactConstraint**: Two-body non-penetration constraint with Baumgarte stabilization and restitution
- **ContactConstraintFactory**: Creates ContactConstraint instances from CollisionResult manifolds

#### Quick Example
```cpp
#include "msd-sim/src/Physics/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp"

CollisionHandler collisionHandler{1e-6};
auto result = collisionHandler.checkCollision(assetA, assetB);

if (result) {
  // Access collision data
  std::cout << "Penetration: " << result->penetrationDepth << " m\n";
  std::cout << "Contacts: " << result->contactCount << "\n";

  // Create and solve contact constraint (see Collision/CLAUDE.md for details)
  auto constraint = ContactConstraintFactory::createFromCollision(*result, assetA, assetB);
  constraintSolver.solveWithContacts({constraint.get()}, {&assetA, &assetB});
}
```

For detailed documentation, see [Collision/CLAUDE.md](Collision/CLAUDE.md).

---

### Integration System

**Location**: `Integration/` (see [`Integration/CLAUDE.md`](Integration/CLAUDE.md) for detailed documentation)
**Diagram**: [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml)
**Type**: Sub-module

#### Overview

The Integration sub-module provides an abstract interface for numerical integration schemes, enabling swappable integrators (Euler, RK4, Verlet) without modifying WorldModel. Uses the Strategy pattern to decouple integration mathematics from physics orchestration.

**Key components**:
- **Integrator interface**: Abstract base class defining `step()` method for state advancement
- **SemiImplicitEulerIntegrator**: Symplectic integrator with velocity-first integration (default)
- **ConstraintSolver integration**: Automatically enforces Lagrange multiplier constraints during integration

#### Integration with Physics Pipeline

Every `AssetInertial` object's state is advanced each frame by calling `integrator_->step()` with:
- External forces (gravity, user forces)
- Constraint vector (quaternion normalization, joints, etc.)
- Mass properties (mass, inverse inertia tensor)
- Timestep (typically 16.67ms for 60 FPS)

The integrator computes unconstrained accelerations, solves the constraint system using `ConstraintSolver`, applies total accelerations, updates velocities and positions, and enforces constraints.

#### Detailed Documentation

For complete integration architecture, mathematical formulation, all integrator types, constraint integration, and usage examples, see:

**[`Integration/CLAUDE.md`](Integration/CLAUDE.md)**

---

### PotentialEnergy System

**Location**: `PotentialEnergy/`
**Documentation**: [PotentialEnergy/CLAUDE.md](PotentialEnergy/CLAUDE.md)
**Diagram**: [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml)
**Introduced**: [Ticket: 0030_lagrangian_quaternion_physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md)

#### Overview
Extensible framework for environmental potential energy fields in Lagrangian mechanics. Enables computation of generalized forces and torques from energy gradients for gravity, tidal forces, magnetic fields, and other environmental potentials.

#### Key Components
- **PotentialEnergy interface**: Abstract base class defining `computeForce()`, `computeTorque()`, `computeEnergy()`
- **GravityPotential**: Uniform gravitational field implementation producing constant force F = m*g
- **Future extensions**: TidalPotential, MagneticPotential, DragPotential, SpringPotential

#### Integration with Physics
WorldModel owns a vector of PotentialEnergy instances and accumulates forces from all potentials during physics updates:

```cpp
for (const auto& potential : potentialEnergies_) {
  netForce += potential->computeForce(asset.getState(), asset.getMass());
  netTorque += potential->computeTorque(asset.getState(), asset.getInertiaTensor());
}
integrator_->step(asset, netForce, netTorque, dt);
```

#### Detailed Documentation
For complete potential energy system architecture, implementation requirements, concrete implementations, and usage examples, see:

**[PotentialEnergy/CLAUDE.md](PotentialEnergy/CLAUDE.md)**

---

### Constraint System

**Location**: `Constraints/` (see [`Constraints/CLAUDE.md`](Constraints/CLAUDE.md) for detailed documentation)
**Diagram**: [`0043_constraint_hierarchy_refactor.puml`](../../../../../docs/designs/0043_constraint_hierarchy_refactor/0043_constraint_hierarchy_refactor.puml)
**Type**: Sub-module

#### Overview

The Constraints sub-module provides an extensible constraint framework using Lagrange multipliers. Users define arbitrary constraints by implementing the `Constraint` interface, with all constraints enforced by a unified solver infrastructure.

**Key components**:
- **Constraint hierarchy**: Flat 2-level design where all concrete constraints inherit directly from `Constraint` base class (ticket 0043 eliminated intermediate `BilateralConstraint`, `UnilateralConstraint`, and `TwoBodyConstraint` classes)
- **LambdaBounds**: Value type encoding constraint multiplier semantics (bilateral/unilateral/box-constrained) via factory methods
- **ConstraintSolver**: Computes Lagrange multipliers for bilateral constraints using direct LLT solve (O(n³), suitable for n < 100); computes contact constraint forces using Active Set Method for exact LCP solution
- **Concrete implementations**: `UnitQuaternionConstraint` (single-body unit quaternion normalization), `DistanceConstraint` (single-body fixed distance from origin), `ContactConstraint` (two-body non-penetration with restitution), `FrictionConstraint` (two-body Coulomb friction with box constraints)
- **Deprecated**: `QuaternionConstraint` (ticket 0030) — use `UnitQuaternionConstraint` instead

#### Integration

Every `AssetInertial` owns a vector of constraints (`std::vector<std::unique_ptr<Constraint>>`). The `SemiImplicitEulerIntegrator` automatically uses `ConstraintSolver` to compute constraint forces, which are added to external forces before integration.

**Default behavior**: Every `AssetInertial` automatically includes a `UnitQuaternionConstraint` to maintain quaternion normalization.

#### Adding Custom Constraints

```cpp
// Single-body constraint
class MyConstraint : public Constraint {
public:
  MyConstraint(size_t bodyIndex) : Constraint(bodyIndex) {}

  int dimension() const override { return 1; }

  Eigen::VectorXd evaluate(const InertialState& stateA,
                           const InertialState& /* stateB */,
                           double time) const override;

  Eigen::MatrixXd jacobian(const InertialState& stateA,
                           const InertialState& /* stateB */,
                           double time) const override;

  LambdaBounds lambdaBounds() const override {
    return LambdaBounds::bilateral();
  }

  std::string typeName() const override { return "MyConstraint"; }
};

// Add to asset
asset.addConstraint(std::make_unique<MyConstraint>(assetIndex));
```

#### Detailed Documentation

For complete constraint system architecture, mathematical formulation, all constraint types, solver details, and usage examples, see:

**[`Constraints/CLAUDE.md`](Constraints/CLAUDE.md)**

---

## Design Patterns in Use

### Factory Pattern
**Used in**: Object creation, DynamicState::createAtRest()
**Purpose**: Enforces valid construction with proper initialization.

### Strategy Pattern
**Used in**: GJK simplex handling
**Purpose**: Different simplex cases (line, triangle, tetrahedron) handled by dedicated methods.

### Value Semantics
**Used in**: DynamicState, Coordinate (from Environment module)
**Purpose**: Clear ownership, safe copying, efficient for small types.

### RAII
**Used in**: Qhull resource management in ConvexHull::computeHull
**Purpose**: Automatic cleanup of external library resources.

---

## Cross-Cutting Concerns

### Units
All quantities use SI units:
- Mass: kilograms [kg]
- Force: Newtons [N]
- Torque: Newton-meters [N⋅m]
- Linear acceleration: meters per second squared [m/s²]
- Angular acceleration: radians per second squared [rad/s²]
- Position: meters [m]
- Inertia: kilogram-meters squared [kg⋅m²]

### Error Handling Strategy
- **Exceptions**: Thrown for invalid parameters, degenerate geometry
- **Validation**: Constructors validate input parameters
- **Qhull errors**: Wrapped and rethrown as `std::runtime_error`

### Memory Management
- **ConvexHull**: Owns vertex and facet data
- **PhysicsComponent**: Caches inverse inertia tensor
- **DynamicState**: Pure value type
- **GJK**: Temporary simplex storage, references to hulls

### Thread Safety Conventions
- **Read-only operations**: Thread-safe for ConvexHull after construction
- **Mutable operations**: Not thread-safe (DynamicState modification, force application)
- **GJK**: Safe to use concurrently with different instances
- **CollisionHandler**: Stateless after construction, safe to call from multiple threads
- **EPA**: Not thread-safe (mutable state during expansion)
- **CollisionResult**: Value type, safe to copy across threads

---

## Physics Calculations

### Linear Acceleration
```
F_net = Σ F_i
a = F_net / m
```

### Angular Acceleration
```
τ_net = Σ (r_i × F_i)    where r_i = applicationPoint - centerOfMass
α = I⁻¹ * τ_net
```

### Kinetic Energy
```
KE = 0.5 * m * v² + 0.5 * ω^T * I * ω
```

---

## Build & Configuration

### Build Requirements
- **C++ Standard**: C++20
- **Dependencies**:
  - Eigen3 — Linear algebra
  - Qhull — Convex hull computation
  - msd-assets — Collision geometry types

### Building This Component

This module is built as part of `msd-sim`:

```bash
# From project root
cmake --preset conan-debug
cmake --build --preset debug-sim-only
```

---

## Testing

### Test Organization
```
test/Physics/
├── ConvexHullTest.cpp      # Hull computation, containment, intersection
└── README.md               # Physics testing documentation
```

### Running Tests
```bash
cmake --build --preset debug-tests-only
ctest --preset debug
```

---

## Coding Standards

This module follows the project-wide coding standards defined in the [root CLAUDE.md](../../../../CLAUDE.md#coding-standards).

Key standards applied in this module:
- **Initialization**: Brace initialization `{}`, `NaN` for uninitialized floats
- **Naming**: `PascalCase` for classes, `camelCase` for methods, `snake_case_` for members
- **Return Values**: Return values/structs over output parameters
- **Memory**: Value semantics for state, `std::unique_ptr` for ownership, RAII for external resources

See the [root CLAUDE.md](../../../../CLAUDE.md#coding-standards) for complete details and examples.

---

## Getting Help

### For AI Assistants
1. This document provides high-level context for the Physics module
2. Review sub-module CLAUDE.md files for detailed component documentation:
   - [RigidBody/CLAUDE.md](RigidBody/CLAUDE.md) — Rigid body representation, convex hulls, inertia
   - [Collision/CLAUDE.md](Collision/CLAUDE.md) — Collision detection and response
   - [Integration/CLAUDE.md](Integration/CLAUDE.md) — Numerical integration
   - [Constraints/CLAUDE.md](Constraints/CLAUDE.md) — Constraint system
   - [PotentialEnergy/CLAUDE.md](PotentialEnergy/CLAUDE.md) — Potential energy fields
3. Review [Environment/CLAUDE.md](../Environment/CLAUDE.md) for Coordinate and related types
4. Review [msd-sim/CLAUDE.md](../../CLAUDE.md) for overall simulation architecture
5. Check [root CLAUDE.md](../../../../CLAUDE.md) for project-wide conventions

### For Developers
- **Rigid bodies**: See [RigidBody/CLAUDE.md](RigidBody/CLAUDE.md) for AssetInertial, ConvexHull, InertialState, InertialCalculations
- **Collision detection**: See [Collision/CLAUDE.md](Collision/CLAUDE.md) for GJK/EPA and collision response
- **Numerical integration**: See [Integration/CLAUDE.md](Integration/CLAUDE.md) for integrator system
- **Constraint system**: See [Constraints/CLAUDE.md](Constraints/CLAUDE.md) for constraint framework
- **Potential energy**: See [PotentialEnergy/CLAUDE.md](PotentialEnergy/CLAUDE.md) for environmental forces
