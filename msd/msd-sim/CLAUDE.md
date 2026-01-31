# msd-sim Library Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/msd/msd-sim/` for detailed component relationships.

## Library Overview

**msd-sim** is the core simulation library for the MSD project. It provides a comprehensive simulation engine with mathematical primitives, rigid body physics, coordinate transformations, collision detection, and extensible agent control.

**Diagram**: [`msd-sim-core.puml`](../../docs/msd/msd-sim/msd-sim-core.puml)

## Architecture Overview

### High-Level Architecture

The library is organized into distinct modules with clear responsibilities:

```
Engine (Top-level orchestrator)
    ├── AssetRegistry (from msd-assets)
    └── WorldModel (Simulation container)
        └── Object (Unified entity type)
            ├── ReferenceFrame (Position/Orientation)
            ├── ConvexHull (Collision geometry)
            └── PhysicsComponent (Dynamics)
```

### Core Modules

| Module | Location | Purpose | Documentation |
|--------|----------|---------|---------------|
| Engine | `src/` | Top-level simulation orchestrator | This document |
| Agent | `src/Agent/` | Autonomous control interface | [`Agent/CLAUDE.md`](src/Agent/CLAUDE.md) |
| Environment | `src/Environment/` | Mathematical primitives & world model | [`Environment/CLAUDE.md`](src/Environment/CLAUDE.md) |
| Physics | `src/Physics/` | Rigid body dynamics & collision | [`Physics/CLAUDE.md`](src/Physics/CLAUDE.md) |
| Utils | `src/Utils/` | Common helper functions | [`Utils/CLAUDE.md`](src/Utils/CLAUDE.md) |

---

## Module Summary

### Agent Module

**Location**: `src/Agent/`
**Documentation**: [`Agent/CLAUDE.md`](src/Agent/CLAUDE.md)

Abstract interface for autonomous control logic using the Strategy pattern. Enables polymorphic agent behavior where different control algorithms can be swapped at runtime.

**Key Components**:
- `BaseAgent` — Abstract base class with `updateState()` pure virtual method
- Designed for ownership via `std::unique_ptr` by Platform/Object

### Environment Module

**Location**: `src/Environment/`
**Documentation**: [`Environment/CLAUDE.md`](src/Environment/CLAUDE.md)
**Diagrams**: [`docs/msd/msd-sim/Environment/`](../../docs/msd/msd-sim/Environment/)

Core mathematical primitives and simulation entity management. Provides type-safe angle handling, 3D coordinates, kinematic state representation, coordinate transformations, and unified object management.

**Key Components**:
- `Coordinate` — 3D vector wrapper (Eigen-based)
- `Angle` — Type-safe angle with lazy normalization
- `AngularCoordinate` — Orientation angles with deferred normalization
- `AngularRate` — Angular velocity/acceleration without normalization
- `InertialState` — Complete kinematic state (6 DOF)
- `ReferenceFrame` — Coordinate transformations
- `Object` — Unified simulation entity (Graphical/Inertial/Environmental/Boundary)
- `WorldModel` — Container and manager for all simulation objects

### Physics Module

**Location**: `src/Physics/`
**Documentation**: [`Physics/CLAUDE.md`](src/Physics/CLAUDE.md)
**Diagrams**: [`docs/msd/msd-sim/Physics/`](../../docs/msd/msd-sim/Physics/)

Force-based rigid body dynamics and collision detection. Provides rigid body representation, convex hull geometry, collision detection and response system, inertia tensor calculation, numerical integration systems, and constraint enforcement.

**Key Components**:
- **RigidBody System** — Rigid body representation, convex hulls, inertial properties, quaternion orientation (see [`src/Physics/RigidBody/CLAUDE.md`](src/Physics/RigidBody/CLAUDE.md))
- **Collision System** — GJK/EPA collision detection and response (see [`src/Physics/Collision/CLAUDE.md`](src/Physics/Collision/CLAUDE.md))
- **Integration** — Numerical integration framework (see [`src/Physics/Integration/CLAUDE.md`](src/Physics/Integration/CLAUDE.md))
- **PotentialEnergy** — Environmental potential energy fields for Lagrangian mechanics (see [`src/Physics/PotentialEnergy/CLAUDE.md`](src/Physics/PotentialEnergy/CLAUDE.md))
- **Constraints** — Lagrange multiplier constraint system (see [`src/Physics/Constraints/CLAUDE.md`](src/Physics/Constraints/CLAUDE.md))
- `WorldModel` — Physics integration with gravity and constraint enforcement

### Utils Module

**Location**: `src/Utils/`
**Documentation**: [`Utils/CLAUDE.md`](src/Utils/CLAUDE.md)

Common helper functions for numerical operations.

**Key Components**:
- `almostEqual()` — Floating-point comparison with tolerance
- `TOLERANCE` — Default comparison tolerance (1e-10)

---

## Engine Component

**Location**: `src/Engine.hpp`, `src/Engine.cpp`

### Purpose
Top-level simulation orchestrator that coordinates asset loading, world management, and simulation updates.

### Key Interfaces
```cpp
class Engine {
  Engine(const std::string& dbPath);

  void update(std::chrono::milliseconds simTime);

  void spawnInertialObject(const std::string assetName,
                           const Coordinate& position,
                           const AngularCoordinate& orientation);

  msd_assets::AssetRegistry& getAssetRegistry();

private:
  msd_assets::AssetRegistry assetRegistry_;
  WorldModel worldModel_;
};
```

### Usage Example
```cpp
#include "msd-sim/src/Engine.hpp"

// Create engine with database path
msd_sim::Engine engine{"assets.db"};

// Spawn objects into simulation
engine.spawnInertialObject("cube",
                           Coordinate{0, 0, 10},
                           AngularCoordinate{});

// Run simulation loop
auto simTime = std::chrono::milliseconds{0};
while (running) {
  engine.update(simTime);
  simTime += std::chrono::milliseconds{16};  // ~60 FPS
}
```

### Thread Safety
**Not thread-safe** — Single-threaded simulation assumed.

### Memory Management
- Owns `AssetRegistry` and `WorldModel` by value
- Objects spawned are owned by WorldModel

### Dependencies
- `msd-assets` — Asset loading and management
- Environment module — WorldModel, Coordinate, AngularCoordinate

---

## Cross-Cutting Concerns

### Coordinate System Convention
- **Type**: Right-handed Cartesian (Aerospace convention)
- **X-axis**: Forward (velocity direction)
- **Y-axis**: Right (starboard)
- **Z-axis**: Up (zenith)

### Units
All quantities use SI units:
- Position: meters [m]
- Mass: kilograms [kg]
- Force: Newtons [N]
- Torque: Newton-meters [N·m]
- Angles: radians [rad]
- Time: seconds [s] or milliseconds [ms]

### Error Handling Strategy
- **Exceptions**: For invalid parameters and degenerate geometry
- **std::optional**: For cache lookups and queries that may fail
- **Validation**: Constructors and factory methods validate inputs

### Memory Management
- **Value semantics**: Mathematical primitives (Coordinate, Angle, etc.)
- **Unique ownership**: `std::unique_ptr` for agents
- **Optional components**: `std::optional` for physics/collision in Object
- **Move semantics**: WorldModel accepts objects via move

### Thread Safety Conventions
- **Value types**: Safe to copy across threads after construction
- **Container types**: Not thread-safe, single-threaded simulation assumed
- **Read-only operations**: Thread-safe for ConvexHull after construction

---

## Build & Configuration

### Build Requirements
- **C++ Standard**: C++20
- **Compiler**: GCC 11+, Clang 14+, or MSVC 2019+
- **Build System**: CMake 3.15+ with Conan 2.x

### Dependencies
- **Eigen3** — Linear algebra
- **Qhull** — Convex hull computation
- **spdlog** — Logging
- **msd-assets** — Asset management (public dependency)

### Building This Library

```bash
# Install dependencies
conan install . --build=missing -s build_type=Debug

# Configure and build
cmake --preset conan-debug
cmake --build --preset debug-sim-only
```

### Build Targets
- `msd_sim` — Main library
- `msd_sim_test` — Test suite

---

## Testing

### Test Organization
```
test/
├── Environment/
│   ├── AngleTest.cpp           # Angle normalization, arithmetic
│   ├── EnvironmentTest.cpp     # Overall environment tests
│   └── ReferenceFrameTest.cpp  # Transform validation
└── Physics/
    └── ConvexHullTest.cpp      # Hull computation, containment, intersection
```

### Running Tests
```bash
cmake --build --preset debug-tests-only
ctest --preset debug
```

---

## Diagrams Index

| Diagram | Description | Location |
|---------|-------------|----------|
| [`msd-sim-core.puml`](../../docs/msd/msd-sim/msd-sim-core.puml) | High-level library architecture | `docs/msd/msd-sim/` |
| [`mathematical-primitives.puml`](../../docs/msd/msd-sim/Environment/mathematical-primitives.puml) | Coordinate, Angle, AngularCoordinate, AngularRate, InertialState | `docs/msd/msd-sim/Environment/` |
| [`angular-coordinate.puml`](../../docs/msd/msd-sim/Environment/angular-coordinate.puml) | AngularCoordinate and AngularRate detailed design | `docs/msd/msd-sim/Environment/` |
| [`reference-frame.puml`](../../docs/msd/msd-sim/Environment/reference-frame.puml) | Coordinate transformations | `docs/msd/msd-sim/Environment/` |
| [`object.puml`](../../docs/msd/msd-sim/Environment/object.puml) | Unified Object entity | `docs/msd/msd-sim/Environment/` |
| [`world-model.puml`](../../docs/msd/msd-sim/Environment/world-model.puml) | WorldModel container | `docs/msd/msd-sim/Environment/` |
| [`physics-core.puml`](../../docs/msd/msd-sim/Physics/physics-core.puml) | Physics module overview | `docs/msd/msd-sim/Physics/` |
| [`convex-hull.puml`](../../docs/msd/msd-sim/Physics/convex-hull.puml) | ConvexHull geometry | `docs/msd/msd-sim/Physics/` |
| [`physics-component.puml`](../../docs/msd/msd-sim/Physics/physics-component.puml) | PhysicsComponent rigid body | `docs/msd/msd-sim/Physics/` |
| [`dynamic-state.puml`](../../docs/msd/msd-sim/Physics/dynamic-state.puml) | DynamicState kinematics | `docs/msd/msd-sim/Physics/` |
| [`gjk-asset-physical.puml`](../../docs/msd/msd-sim/Physics/gjk-asset-physical.puml) | GJK collision detection with AssetPhysical transforms | `docs/msd/msd-sim/Physics/` |
| [`epa.puml`](../../docs/msd/msd-sim/Physics/epa.puml) | EPA contact information extraction and CollisionHandler orchestration | `docs/msd/msd-sim/Physics/` |
| [`witness-points.puml`](../../docs/msd/msd-sim/Physics/witness-points.puml) | Witness point tracking for accurate torque calculation | `docs/msd/msd-sim/Physics/` |
| [`force-application.puml`](../../docs/msd/msd-sim/Physics/force-application.puml) | Force application system with semi-implicit Euler integration | `docs/msd/msd-sim/Physics/` |
| [`mirtich-inertia-tensor.puml`](../../docs/msd/msd-sim/Physics/mirtich-inertia-tensor.puml) | Mirtich algorithm for inertia tensor calculation | `docs/msd/msd-sim/Physics/` |
| [`collision-response.puml`](../../docs/msd/msd-sim/Physics/collision-response.puml) | Collision response system with impulse-based physics | `docs/msd/msd-sim/Physics/` |
| [`0030_lagrangian_quaternion_physics.puml`](../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml) | Lagrangian quaternion physics with potential energy and constraints | `docs/designs/0030_lagrangian_quaternion_physics/` |
| [`generalized-constraints.puml`](../../docs/msd/msd-sim/Physics/generalized-constraints.puml) | Generalized Lagrange multiplier constraint system with extensible constraint library | `docs/msd/msd-sim/Physics/` |
| [`two-body-constraints.puml`](../../docs/msd/msd-sim/Physics/two-body-constraints.puml) | Two-body constraint infrastructure with ContactConstraint for collision response | `docs/msd/msd-sim/Physics/` |
| [`0034_active_set_method_contact_solver.puml`](../../docs/designs/0034_active_set_method_contact_solver/0034_active_set_method_contact_solver.puml) | Active Set Method contact solver replacing PGS with exact LCP solution | `docs/designs/0034_active_set_method_contact_solver/` |

---

## Recent Architectural Changes

### Active Set Method Contact Solver — 2026-01-31
**Ticket**: [0034_active_set_method_contact_solver](../../tickets/0034_active_set_method_contact_solver.md)
**Diagram**: [`0034_active_set_method_contact_solver.puml`](../../docs/designs/0034_active_set_method_contact_solver/0034_active_set_method_contact_solver.puml)
**Type**: Internal Replacement (Breaking Change to Solver Implementation)

Replaced the Projected Gauss-Seidel (PGS) iterative solver in `ConstraintSolver::solveWithContacts()` with an Active Set Method (ASM) for solving the contact constraint Linear Complementarity Problem (LCP). The ASM partitions contacts into active (compressive) and inactive (separating) sets, solving an equality-constrained subproblem at each step via direct LLT decomposition, and iterates by adding/removing constraints until all KKT conditions are satisfied. This provides finite convergence to the exact solution, eliminating PGS's sensitivity to iteration count, constraint ordering, and high mass ratios.

**Key components**:
- **ConstraintSolver modifications** — Replaced `solvePGS()` with `solveActiveSet()`, replaced `PGSResult` with `ActiveSetResult`, renamed `max_iterations_` to `max_safety_iterations_` (default: 100), updated `convergence_tolerance_` default from 1e-4 to 1e-6
- **ActiveSetResult struct** — New result type with `lambda`, `converged`, `iterations`, and `active_set_size` fields
- **Active Set algorithm** — Computes exact LCP solution using primal-dual feasibility checks and Bland's anti-cycling rule
- **Safety iteration cap** — Per-solve effective limit is `min(2*numContacts, max_safety_iterations_)` to prevent infinite loops in degenerate cases

**Algorithm**:
- Initializes with all contacts in active set (W = {0, 1, ..., C-1})
- Each iteration: Solves A_W·λ_W = b_W via LLT, checks primal feasibility (λ ≥ 0), checks dual feasibility (w = Aλ-b ≥ 0)
- Removes negative lambdas from active set (constraint wants to pull, not push)
- Adds most violated inactive constraint to active set (w < -tolerance)
- Converges when all KKT conditions satisfied: λ ≥ 0, Aλ-b ≥ 0, λ^T(Aλ-b) ≈ 0

**Benefits over PGS**:
- Exact solution: Converges to exact LCP solution in finite iterations (typically ≤ C iterations for non-degenerate systems)
- Mass ratio robustness: Handles mass ratios up to 1e6:1 without convergence failure (LLT handles condition numbers up to ~1e12)
- Deterministic: Solution independent of constraint processing order (no order dependence)
- No iteration tuning: Algorithm terminates naturally when KKT conditions satisfied (safety cap is defensive, not convergence control)
- Theoretical foundation: Well-understood convergence proofs (Nocedal & Wright, Numerical Optimization, Chapter 16)

**Performance**:
- Per-iteration cost: O(|W|^3) for LLT solve on active subset (higher than PGS's O(C^2))
- Total iterations: Variable, typically ≤ C (vs PGS's fixed 10)
- Typical contact counts (1-10): ASM provides exact solutions at comparable or better wall-clock time than PGS
- Large contact counts (> 20): Cubic per-iteration cost dominates (project does not target such scenarios)

**Public interface changes** (backward compatible with caveats):
- `solveWithContacts()` signature unchanged — returns same `MultiBodySolveResult` struct
- `setMaxIterations()` semantic change: Now sets safety cap (default 100) rather than iteration budget (was 10)
- `setConvergenceTolerance()` semantic change: Now sets violation threshold (default 1e-6) rather than PGS convergence check (was 1e-4)
- `MultiBodySolveResult::iterations` now counts active set changes (was PGS iterations)
- `MultiBodySolveResult::residual` semantic change: Now dual residual norm (complementarity measure) rather than PGS convergence metric

**Breaking changes**:
- Internal method `solvePGS()` removed (was private)
- Internal struct `PGSResult` removed (was private)
- Test impact: One existing test (`MaxIterationsReached_ReportsNotConverged_0033`) required scenario modification because ASM converges faster than PGS for trivial cases

**Key files**:
- `src/Physics/Constraints/ConstraintSolver.hpp`, `ConstraintSolver.cpp` — Replaced PGS with ASM solver kernel
- `test/Physics/Constraints/ConstraintSolverASMTest.cpp` — 12 new unit tests for ASM-specific behavior
- `test/Physics/Constraints/ConstraintSolverContactTest.cpp` — Modified 1 of 24 existing tests for ASM convergence behavior

**Thread safety**: `solveActiveSet()` is a const method with only local state (thread-safe for different inputs).

**Memory management**: No heap allocations beyond Eigen dynamic matrices (same as PGS). All subproblem matrices locally scoped and freed after solve.

**Error handling**: Returns `converged = false` for singular/ill-conditioned matrices or when safety cap reached. Regularization epsilon (1e-8) prevents singularity in practice.

---

### Two-Body Constraint Infrastructure — 2026-01-29
**Ticket**: [0032a_two_body_constraint_infrastructure](../../tickets/0032a_two_body_constraint_infrastructure.md)
**Diagram**: [`two-body-constraints.puml`](../../docs/msd/msd-sim/Physics/two-body-constraints.puml)
**Type**: Feature Enhancement (Additive)

Extended the generalized constraint framework (ticket 0031) to support two-body constraints — constraints that couple the motion of two rigid bodies. Introduced `TwoBodyConstraint` abstract class and `ContactConstraint` concrete implementation for non-penetration constraints with Baumgarte stabilization and restitution handling. This lays the foundation for replacing the impulse-based collision response with a unified constraint-based approach.

**Key components**:
- **TwoBodyConstraint** — Abstract interface for constraints operating on two rigid bodies with 12-DOF velocity-level Jacobian [v_A, ω_A, v_B, ω_B]
- **ContactConstraint** — Concrete non-penetration constraint C(q) = (x_B - x_A) · n ≥ 0 with dimension=1 per contact point
- **ContactConstraintFactory** — Stateless utility namespace for creating ContactConstraint instances from CollisionResult manifolds
- **AssetEnvironment extensions** — Added `getInverseMass()` (returns 0.0) and `getInverseInertiaTensor()` (returns zero matrix) for unified solver path with infinite-mass static objects
- **AssetInertial extensions** — Added `getInverseMass()` convenience method returning 1.0/mass

**Architecture**:
- `TwoBodyConstraint` extends `UnilateralConstraint` with two-body evaluation methods: `evaluateTwoBody()`, `jacobianTwoBody()`, `isActiveTwoBody()`
- Single-body methods (`evaluate()`, `jacobian()`, `isActive()`) throw `std::logic_error` to enforce API correctness — two-body constraints must use two-body interface
- `ContactConstraint::dimension()` returns 1 — one constraint row per contact point, enabling multi-point contact manifolds via multiple ContactConstraint objects
- Jacobian structure (1 × 12): `J = [-n^T, -(r_A × n)^T, n^T, (r_B × n)^T]` for linear and angular constraint coupling
- Baumgarte stabilization uses Error Reduction Parameter (ERP) formulation: ERP=0.2 default, equivalent to α_accel≈781 [1/s²] at 60 FPS
- Restitution handling: Stores pre-impact relative normal velocity for constraint RHS computation with correct formula `v_target = -e·v_pre`
- `ContactConstraintFactory::createFromCollision()` generates one ContactConstraint per contact point in CollisionResult manifold
- `AssetEnvironment` with inverseMass=0 enables unified solver code path — static bodies receive zero velocity change from constraint impulses

**Thread safety**: ContactConstraint immutable after construction (thread-safe reads). Factory functions stateless (thread-safe).

**Memory management**: ContactConstraint instances created per-frame and owned via `std::unique_ptr` (transient lifecycle). Pre-computed lever arms stored at construction to avoid recomputation.

**Error handling**:
- ContactConstraint constructor validates normal is unit length (within 1e-6), penetration depth ≥ 0, restitution in [0, 1]
- Single-body method overrides throw `std::logic_error` for API misuse detection
- AssetEnvironment constructor validates restitution in [0, 1], throws `std::invalid_argument` otherwise

**Design rationale**:
- Subclassing approach avoids modifying existing `Constraint` interface, preserving backward compatibility for all single-body constraints (UnitQuaternionConstraint, DistanceConstraint)
- Solver can use `dynamic_cast<TwoBodyConstraint*>` to dispatch between single-body and two-body constraint evaluation
- ERP formulation (velocity-level bias) preferred over acceleration-level Baumgarte (α·C + β·Ċ) to match physics engine industry standard and avoid parameter unit confusion
- Zero inverse mass for AssetEnvironment enables treating static objects identically to dynamic objects in solver, eliminating duplicate code paths

**Key files**:
- `src/Physics/Constraints/TwoBodyConstraint.hpp`, `TwoBodyConstraint.cpp` — Abstract two-body constraint interface
- `src/Physics/Constraints/ContactConstraint.hpp`, `ContactConstraint.cpp` — Non-penetration constraint implementation
- `src/Physics/Constraints/ContactConstraintFactory.hpp`, `ContactConstraintFactory.cpp` — Factory for creating constraints from collisions
- `src/Physics/RigidBody/AssetEnvironment.hpp`, `AssetEnvironment.cpp` — Extended with inverse mass/inertia and restitution property
- `src/Physics/RigidBody/AssetInertial.hpp`, `AssetInertial.cpp` — Added getInverseMass() convenience method
- `test/Physics/Constraints/ContactConstraintTest.cpp` — 33 unit tests covering constraint evaluation, Jacobian validation, factory functions

**Limitations** (to be addressed in subsequent sub-tickets):
- No solver integration yet — PGS solver extension deferred to ticket 0032b
- No WorldModel integration — contact constraint creation and solving deferred to ticket 0032c
- Transient constraints only — no warm starting or contact persistence (future enhancement)
- Single normal constraint per contact point — friction constraints deferred to future work

---

### Generalized Lagrange Multiplier Constraint System — 2026-01-28
**Ticket**: [0031_generalized_lagrange_constraints](../../tickets/0031_generalized_lagrange_constraints.md)
**Diagram**: [`generalized-constraints.puml`](../../docs/msd/msd-sim/Physics/generalized-constraints.puml)
**Type**: Breaking Change (Extensibility Enhancement)

Replaced the hard-coded `QuaternionConstraint` with an extensible constraint framework that enables users to define arbitrary constraints by implementing the `Constraint` interface. The system separates constraint definition (evaluation, Jacobian) from constraint solving (Lagrange multiplier computation), enabling a library of constraint types that all use the same solver infrastructure.

**Key components**:
- **Constraint interface** — Abstract base class defining mathematical requirements: `dimension()`, `evaluate()`, `jacobian()`, `typeName()`
- **BilateralConstraint** — Abstract subclass for equality constraints (C = 0) with unrestricted Lagrange multipliers
- **UnilateralConstraint** — Abstract subclass for inequality constraints (C ≥ 0) with complementarity conditions (interface only, solver deferred)
- **ConstraintSolver** — Computes Lagrange multipliers λ using direct LLT solve on J·M⁻¹·Jᵀ, returns `SolveResult` with convergence status and condition number
- **UnitQuaternionConstraint** — Concrete implementation replacing deprecated `QuaternionConstraint`, enforces C(Q) = Q^T*Q - 1 = 0
- **DistanceConstraint** — Example constraint enforcing fixed distance between positions, demonstrates single-object constraints
- **AssetInertial modifications** — Owns constraints via `std::vector<std::unique_ptr<Constraint>>`, default includes `UnitQuaternionConstraint`
- **Integrator signature change** — `step()` now accepts `std::vector<Constraint*>` instead of single `QuaternionConstraint&`

**Architecture**:
- `Constraint::evaluate(state, time)` returns constraint violation C(q, t)
- `Constraint::jacobian(state, time)` returns ∂C/∂q matrix (dimension × 7 for single-object constraints)
- `Constraint::alpha()` and `beta()` provide Baumgarte stabilization parameters (default: 10.0)
- `ConstraintSolver::solve()` computes λ satisfying: J·M⁻¹·Jᵀ·λ = -J·M⁻¹·F_ext - J̇·q̇ - α·C - β·Ċ
- Constraint forces applied: F_constraint = Jᵀ·λ
- AssetInertial owns constraints, `getConstraints()` returns non-owning `std::vector<Constraint*>`
- SemiImplicitEulerIntegrator uses ConstraintSolver automatically for all constraints

**Breaking changes**:
- `Integrator::step()` signature: `QuaternionConstraint&` → `std::vector<Constraint*>&`
- `AssetInertial` constraint management: Single `QuaternionConstraint` → `std::vector<std::unique_ptr<Constraint>>`
- Deprecated: `QuaternionConstraint` class (use `UnitQuaternionConstraint` with `ConstraintSolver` instead)

**Benefits**:
- Extensibility: New constraint types (joints, contacts, limits) added by implementing `Constraint` interface
- Separation of concerns: Constraint definition separate from solving algorithm
- Multi-constraint support: Multiple constraints per object enforced simultaneously
- Unified solver: Single solving algorithm for all constraint types

**Key files**:
- `src/Physics/Constraints/Constraint.hpp`, `Constraint.cpp` — Abstract constraint interface
- `src/Physics/Constraints/BilateralConstraint.hpp` — Equality constraint marker
- `src/Physics/Constraints/UnilateralConstraint.hpp` — Inequality constraint interface (solver deferred)
- `src/Physics/Constraints/UnitQuaternionConstraint.hpp`, `UnitQuaternionConstraint.cpp` — Unit quaternion constraint
- `src/Physics/Constraints/DistanceConstraint.hpp`, `DistanceConstraint.cpp` — Distance constraint example
- `src/Physics/Constraints/ConstraintSolver.hpp`, `ConstraintSolver.cpp` — Lagrange multiplier solver
- `src/Physics/Integration/SemiImplicitEulerIntegrator.hpp`, `SemiImplicitEulerIntegrator.cpp` — Updated to use ConstraintSolver
- `test/Physics/Constraints/ConstraintTest.cpp` — 30 unit tests covering all framework components

---

### Lagrangian Quaternion Physics — 2026-01-28
**Ticket**: [0030_lagrangian_quaternion_physics](../../tickets/0030_lagrangian_quaternion_physics.md)
**Diagram**: [`0030_lagrangian_quaternion_physics.puml`](../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml)
**Type**: Breaking Change

Replaced Euler angle-based orientation with quaternion representation to eliminate gimbal lock singularities and implemented Lagrangian mechanics with general potential energy abstraction. The InertialState now uses a 14-component state vector (X, Q, Ẋ, Q̇) with quaternion normalization maintained via Baumgarte stabilization constraints.

**Key components**:
- **Integrator interface** — Abstract interface for numerical integration schemes, enabling swappable integrators (Euler, RK4, Verlet)
- **SemiImplicitEulerIntegrator** — Symplectic integrator with velocity-first integration for energy conservation
- **PotentialEnergy interface** — Abstract interface for environmental potential energy fields (gravity, tidal forces, magnetic fields)
- **GravityPotential** — Uniform gravitational field implementation producing constant force F = m*g
- **QuaternionConstraint** — Lagrange multiplier constraint with Baumgarte stabilization (α=10, β=10) maintaining |Q|=1 within 1e-10
- **InertialState modifications** — Added quaternionRate (Q̇) alongside quaternion orientation, with ω ↔ Q̇ conversion utilities
- **ReferenceFrame modifications** — Added setQuaternion()/getQuaternion() for quaternion-based transforms
- **WorldModel refactoring** — Owns Integrator and PotentialEnergy instances, refactored updatePhysics() for Lagrangian mechanics

**Architecture**:
- `Integrator` abstract interface decouples integration scheme from physics computation
- `SemiImplicitEulerIntegrator` implements symplectic integration: v_new = v_old + a*dt, x_new = x_old + v_new*dt
- `PotentialEnergy::computeForce()` and `computeTorque()` derive generalized forces from energy gradients (F = -∂V/∂X, τ = -∂V/∂Q)
- `QuaternionConstraint::enforceConstraint()` applies Baumgarte stabilization: λ = -α*g - β*ġ where g = Q^T*Q - 1
- InertialState stores both quaternion Q and quaternion rate Q̇, with conversion utilities: ω = 2*Q̄⊗Q̇ and Q̇ = ½*Q⊗[0,ω]
- WorldModel owns integrator (default: SemiImplicitEuler) and potential energies (default: GravityPotential)
- AssetInertial owns its own QuaternionConstraint instance (one per asset)

**Breaking changes**:
- `InertialState::orientation`: Changed from `AngularCoordinate` to `Eigen::Quaterniond`
- `InertialState::quaternionRate`: New member (Eigen::Vector4d) representing Q̇
- `ReferenceFrame` internal storage: Changed from `AngularCoordinate` to `Eigen::Quaterniond`
- Deprecated accessors: `InertialState::getEulerAngles()`, `ReferenceFrame::setRotation(AngularCoordinate)`, `ReferenceFrame::getAngularCoordinate()`
- State vector size increased from 13 to 14 components (added quaternionRate)

**Benefits**:
- No gimbal lock: Quaternions have no singularities at any orientation
- Numerical stability: Constraint maintains |Q|=1 within 1e-10 tolerance over 10000 steps
- Extensibility: Potential energy abstraction enables future force fields (tidal, magnetic, atmospheric)
- Mathematical rigor: Lagrangian formulation provides clear separation between kinematics and dynamics

**Key files**:
- `src/Physics/Integration/Integrator.hpp` — Abstract integrator interface
- `src/Physics/Integration/SemiImplicitEulerIntegrator.hpp`, `SemiImplicitEulerIntegrator.cpp` — Symplectic integrator
- `src/Physics/PotentialEnergy/PotentialEnergy.hpp` — Potential energy interface
- `src/Physics/PotentialEnergy/GravityPotential.hpp`, `GravityPotential.cpp` — Uniform gravity implementation
- `src/Physics/Constraints/QuaternionConstraint.hpp`, `QuaternionConstraint.cpp` — Baumgarte stabilization
- `src/Physics/RigidBody/InertialState.hpp`, `InertialState.cpp` — Quaternion state with conversion utilities
- `src/Environment/ReferenceFrame.hpp`, `ReferenceFrame.cpp` — Quaternion-based transforms
- `src/Environment/WorldModel.hpp`, `WorldModel.cpp` — Lagrangian mechanics integration
- `test/Physics/Integration/QuaternionPhysicsTest.cpp` — 16 integration tests covering all acceptance criteria

---

### Collision Response System — 2026-01-24
**Ticket**: [0027_collision_response_system](../../tickets/0027_collision_response_system.md)
**Diagram**: [`collision-response.puml`](../../docs/msd/msd-sim/Physics/collision-response.puml)
**Type**: Feature Enhancement

Implemented impulse-based collision response for rigid body dynamics. When two AssetInertial objects collide (detected by CollisionHandler), the system computes collision impulses based on coefficients of restitution, applies both linear and angular impulses, and separates overlapping objects via position correction.

**Key components**:
- **CollisionResponse namespace** — Stateless utility functions for impulse calculation and position correction
- **AssetInertial modification** — Added `coefficientOfRestitution_` property with validation ([0, 1] range)
- **WorldModel integration** — Implemented full collision response in `updateCollisions()` method
- **Impulse formulas** — Uses standard rigid body collision response: `j = -(1 + e) * v_rel · n / (1/m_A + 1/m_B + angular_terms)`

**Architecture**:
- `CollisionResponse::combineRestitution()` — Combines restitution coefficients using geometric mean `sqrt(e_A * e_B)`
- `CollisionResponse::computeImpulseMagnitude()` — Computes scalar impulse including angular effects via lever arms
- `CollisionResponse::applyPositionCorrection()` — Separates objects with slop tolerance (0.01m) to prevent jitter
- `WorldModel::updateCollisions()` — O(n²) pairwise collision detection and response, called before physics integration
- Impulses applied to both linear and angular velocities using witness points from EPA for accurate torque calculation

**Performance**: Sequential collision resolution in single pass, sufficient for typical scenarios (< 5 simultaneous collisions per object). Broadphase optimization intentionally deferred.

**Key files**:
- `src/Physics/CollisionResponse.hpp`, `CollisionResponse.cpp` — Stateless collision response utilities
- `src/Physics/RigidBody/AssetInertial.hpp`, `AssetInertial.cpp` — Added restitution property and accessors
- `src/Environment/WorldModel.hpp`, `WorldModel.cpp` — Integrated collision response in update loop
- `test/Physics/CollisionResponseTest.cpp` — 11 unit tests for impulse calculation and position correction
- `test/Physics/AssetInertialTest.cpp` — 7 unit tests for restitution property
- `test/Environment/WorldModelCollisionTest.cpp` — 7 integration tests for collision behavior

---

### EPA Witness Points for Accurate Torque Calculation — 2026-01-24
**Ticket**: [0028_epa_witness_points](../../tickets/0028_epa_witness_points.md)
**Diagram**: [`witness-points.puml`](../../docs/msd/msd-sim/Physics/witness-points.puml)
**Type**: Feature Enhancement (Breaking Change)

Extended the EPA implementation to track witness points—the actual contact locations on each colliding object's surface. The previous implementation computed a single contact point in Minkowski difference space, which could not be used for accurate torque calculation during collision response. Witness points provide the physical lever arms needed for the torque formula τ = r × F, enabling realistic angular dynamics in glancing collisions and off-center impacts.

**Key components**:
- **SupportResult** — New struct returned by `supportMinkowskiWithWitness()` containing Minkowski point and both witness points
- **MinkowskiVertex** — New internal EPA structure that tracks witness points alongside Minkowski vertices
- **CollisionResult breaking change** — Replaced single `contactPoint` with `contactPointA` and `contactPointB` on object surfaces
- **EPA witness extraction** — Added `computeWitnessA()` and `computeWitnessB()` methods using barycentric interpolation

**Architecture**:
- `supportMinkowskiWithWitness()` extends support function queries to track original surface points
- EPA vertices changed from `std::vector<Coordinate>` to `std::vector<MinkowskiVertex>`
- Witness points extracted from converged polytope faces via barycentric centroid
- Breaking change to CollisionResult has minimal impact (only one future consumer not yet implemented)

**Performance**: < 5% overhead compared to previous EPA implementation. Memory overhead ~500-1000 bytes per collision (temporary, freed after EPA completion).

**Key files**:
- `src/Physics/SupportFunction.hpp`, `SupportFunction.cpp` — Added `SupportResult` struct and `supportMinkowskiWithWitness()` function
- `src/Physics/EPA.hpp`, `EPA.cpp` — Added `MinkowskiVertex` struct, witness tracking, and extraction methods
- `src/Physics/CollisionResult.hpp` — Breaking change: `contactPoint` → `contactPointA` + `contactPointB`
- `test/Physics/EPATest.cpp` — 6 new witness point tests (219 total tests pass)
- `test/Physics/CollisionHandlerTest.cpp` — Updated contact point assertions

---

### Expanding Polytope Algorithm (EPA) for Contact Information — 2026-01-23
**Ticket**: [0027a_expanding_polytope_algorithm](../../tickets/0027a_expanding_polytope_algorithm.md)
**Diagram**: [`epa.puml`](../../docs/msd/msd-sim/Physics/epa.puml)
**Type**: Feature Enhancement

Implemented the Expanding Polytope Algorithm (EPA) to extract detailed contact information (penetration depth, contact normal, contact point) from GJK collision detection. When GJK detects an intersection, EPA expands the terminating simplex to find the closest point on the Minkowski difference boundary, yielding the geometric data required for realistic collision response.

**Key components**:
- **CollisionHandler** — Orchestrates GJK→EPA workflow, returns `std::optional<CollisionResult>` (nullopt = no collision)
- **EPA** — Expanding Polytope Algorithm that computes contact information from GJK simplex
- **CollisionResult** — Struct containing penetration depth, contact normal (world space, A→B), and contact point
- **GJK modification** — Added `getSimplex()` method to expose terminating simplex for EPA input

**Architecture**:
- CollisionHandler provides clean entry point for collision detection, extensible for future enhancements (broadphase, continuous collision)
- std::optional pattern avoids redundant boolean field in CollisionResult
- All coordinates in world space, contact normal points from object A toward object B
- EPA includes simplex completion logic for robustness (handles simplices < 4 vertices)

**Performance**: Typical convergence in 4-11 iterations, < 1ms execution time (debug build), < 10KB memory footprint.

**Key files**:
- `src/Physics/CollisionHandler.hpp`, `CollisionHandler.cpp` — Orchestration layer
- `src/Physics/EPA.hpp`, `EPA.cpp` — EPA algorithm implementation
- `src/Physics/CollisionResult.hpp` — Contact information struct
- `src/Physics/GJK.hpp` — Added `getSimplex()` accessor
- `test/Physics/EPATest.cpp` — 9 unit tests
- `test/Physics/CollisionHandlerTest.cpp` — 8 integration tests

---

### Mirtich Algorithm for Inertia Tensor Calculation — 2026-01-22
**Ticket**: [0026_mirtich_inertia_tensor](../../tickets/0026_mirtich_inertia_tensor.md)
**Diagram**: [`mirtich-inertia-tensor.puml`](../../docs/msd/msd-sim/Physics/mirtich-inertia-tensor.puml)
**Type**: Algorithm Replacement (Internal Refactor)

Replaced the inaccurate tetrahedron decomposition approach in `InertialCalculations::computeInertiaTensorAboutCentroid()` with Brian Mirtich's mathematically exact algorithm from "Fast and Accurate Computation of Polyhedral Mass Properties" (1996). This eliminates the ~10-15% accuracy error and ad-hoc scaling factors present in the previous implementation.

The Mirtich algorithm uses the divergence theorem to convert volume integrals to surface integrals through three hierarchical layers: projection integrals (2D line integrals over face edges) → face integrals (3D surface integrals) → volume integrals (accumulated across all faces). From the volume integrals, it computes volume, center of mass, and inertia tensor about the origin, then applies the parallel axis theorem to shift to the centroid.

**Accuracy improvements**:
- Unit cube: Machine-perfect precision (< 1e-10 error vs analytical solution)
- Rectangular box: Exact match to analytical formulas within 1e-10
- Regular tetrahedron: Diagonal inertia elements equal within 1e-10
- Previous implementation: ~10-15% error with ad-hoc `/10.0` scaling factor
- New implementation: < 1e-10 relative error (effectively machine precision)

**Key implementation details**:
- **Vertex winding correction**: Added `getWindingCorrectedIndices()` to ensure Qhull's vertex order aligns with facet normals, as required by Mirtich's algorithm
- **Volume validation**: Computed volume as byproduct matches `ConvexHull::getVolume()` within 1e-10
- **No API changes**: Function signature unchanged, internal algorithm replaced
- **Reference implementation**: Cross-validated against Mirtich's public domain `volInt.c`

**Performance**: O(F) complexity where F = number of facets. Higher constant factor than tetrahedron decomposition (~50-100 operations per facet vs ~10-20), but still negligible for typical hulls (10-100 facets) since inertia calculation is one-time at object creation.

**Known limitation resolved**: This fixes the pre-existing NaN bug that prevented angular physics validation in ticket 0023. Angular dynamics now fully operational.

**Key files**:
- `src/Physics/RigidBody/InertialCalculations.cpp` — Complete algorithm replacement with Mirtich implementation
- `src/Physics/RigidBody/InertialCalculations.hpp` — Updated documentation with algorithm reference
- `test/Physics/InertialCalculationsTest.cpp` — 13 test cases validating analytical solutions and edge cases

---

### Force Application System for Rigid Body Physics — 2026-01-21
**Ticket**: [0023_force_application_system](../../tickets/0023_force_application_system.md)
**Diagram**: [`force-application.puml`](../../docs/msd/msd-sim/Physics/force-application.puml)
**Type**: Feature Enhancement

Implemented a complete force application system for rigid body physics, enabling realistic dynamics simulation with gravity, forces at arbitrary points generating torque, and semi-implicit Euler integration. This completes the scaffolding from ticket 0023a and uses the angular types introduced in ticket 0024.

**Key features**:
- **Force accumulation**: `AssetInertial` accumulates forces and torques per frame via `applyForce()`, `applyForceAtPoint()`, and `applyTorque()` methods
- **Torque computation**: Forces applied at offset points generate torque using the cross product `τ = r × F`
- **Semi-implicit Euler integration**: `WorldModel::updatePhysics()` integrates motion using velocity-first integration for better numerical stability
- **Gravity**: Applied as direct acceleration `F = m * g` for efficiency
- **ReferenceFrame synchronization**: Position and orientation synchronized after physics updates to ensure consistency with collision detection and rendering
- **World-space convention**: All forces, torques, and application points use world-space coordinates

**Physics integration order**:
1. Apply gravity to all inertial objects
2. Compute linear acceleration: `a = F_net / m`
3. Update velocity: `v += a * dt` (semi-implicit)
4. Update position: `x += v * dt`
5. Compute angular acceleration: `α = I⁻¹ * τ_net`
6. Update angular velocity: `ω += α * dt` (semi-implicit)
7. Update orientation: `θ += ω * dt`
8. Synchronize ReferenceFrame with InertialState
9. Clear accumulated forces for next frame

**Performance**: O(n) complexity per physics update where n = number of inertial objects. Recommended timestep: 16.67ms (60 FPS).

**Note**: Angular physics integration was initially blocked by a bug in inertia tensor calculation (ticket 0025) that produced NaN values. This was resolved by ticket 0026 which implemented the Mirtich algorithm. Angular dynamics now fully operational.

**Key files**:
- `src/Physics/RigidBody/AssetInertial.hpp`, `AssetInertial.cpp` — Force application and torque computation
- `src/Environment/WorldModel.hpp`, `WorldModel.cpp` — Physics integration with semi-implicit Euler
- `src/Environment/ReferenceFrame.hpp`, `ReferenceFrame.cpp` — Added const overload for getAngularCoordinate()
- `src/Physics/RigidBody/InertialState.hpp` — Kinematic state representation

---

### AngularCoordinate and AngularRate — 2026-01-21
**Ticket**: [0024_angular_coordinate](../../tickets/0024_angular_coordinate.md)
**Diagram**: [`angular-coordinate.puml`](../../docs/msd/msd-sim/Environment/angular-coordinate.puml)
**Type**: Breaking Change

Introduced two type-safe classes for angular quantity representation: `AngularCoordinate` for orientation with deferred normalization, and `AngularRate` for angular velocity/acceleration without normalization. Both inherit from `Eigen::Vector3d` for full matrix operations while providing semantic pitch/roll/yaw accessors.

**Breaking changes**:
- Removed `EulerAngles` class entirely
- `InertialState::orientation` changed from `EulerAngles` to `AngularCoordinate`
- `InertialState::angularVelocity` changed from `Coordinate` to `AngularRate`
- `InertialState::angularAcceleration` changed from `Coordinate` to `AngularRate`
- `ReferenceFrame` constructor, `setRotation()`, and `getAngularCoordinate()` now use `AngularCoordinate`
- Internal `ReferenceFrame::euler_` changed to `ReferenceFrame::angular_`

**Key improvements**:
- **Type safety**: Prevents accidental assignment of rates to orientations (compile-time error)
- **Performance**: Deferred normalization with 100π threshold is 10x faster than eager normalization (validated by prototypes)
- **Semantic clarity**: `orientation.yaw()` vs `angularVelocity.yaw()` makes intent explicit
- **Memory efficiency**: 24 bytes per instance (same as `Eigen::Vector3d`)

**Migration**:
```cpp
// Old (EulerAngles)
state.angularPosition.yaw.getRad()

// New (AngularCoordinate)
state.orientation.yaw()

// Old (Coordinate for rates)
state.angularVelocity.z()

// New (AngularRate)
state.angularVelocity.yaw()
```

**Key files**:
- `src/Environment/AngularCoordinate.hpp` — Orientation with deferred normalization
- `src/Environment/AngularRate.hpp` — Angular velocity/acceleration without normalization
- `src/Physics/RigidBody/InertialState.hpp` — Updated angular field types
- `src/Environment/ReferenceFrame.hpp`, `ReferenceFrame.cpp` — Migrated to AngularCoordinate

---

### GJK AssetPhysical Transform Support — 2026-01-18
**Ticket**: [0022_gjk_asset_physical_transform](../../tickets/0022_gjk_asset_physical_transform.md)
**Diagram**: [`gjk-asset-physical.puml`](../../docs/msd/msd-sim/Physics/gjk-asset-physical.puml)
**Type**: Breaking Change

Refactored GJK collision detection to work exclusively with `AssetPhysical` objects that include `ReferenceFrame` transformations. This enables collision detection between objects with arbitrary positions and orientations in world space by applying transformations on-the-fly during support function computation.

**Breaking changes**:
- Removed `GJK(const ConvexHull&, const ConvexHull&)` constructor
- Removed `gjkIntersects(const ConvexHull&, const ConvexHull&)` convenience function
- Removed `ConvexHull::intersects()` method
- Added `GJK(const AssetPhysical&, const AssetPhysical&)` constructor
- Added `gjkIntersects(const AssetPhysical&, const AssetPhysical&)` convenience function

**Performance**: < 2% overhead compared to identity transform baseline (validated by prototypes).

**Migration**: Wrap `ConvexHull` objects in `AssetPhysical` with identity `ReferenceFrame` for untransformed collision detection.

**Key files**:
- `src/Physics/GJK/GJK.hpp`, `GJK.cpp` — AssetPhysical-based GJK implementation
- `src/Physics/RigidBody/ConvexHull.hpp`, `ConvexHull.cpp` — Removed intersects() method
- `src/Physics/RigidBody/AssetPhysical.hpp` — Documentation updates

---

## Coding Standards

This library follows the project-wide coding standards defined in the [root CLAUDE.md](../../CLAUDE.md#coding-standards).

Key standards applied in this library:
- **Initialization**: Brace initialization `{}`, `NaN` for uninitialized floats
- **Naming**: `PascalCase` for classes, `camelCase` for methods, `snake_case_` for members
- **Return Values**: Prefer returning values/structs over output parameters
- **Memory**: Value semantics for primitives, `std::unique_ptr` for ownership

See the [root CLAUDE.md](../../CLAUDE.md#coding-standards) for complete details and examples.

---

## Getting Help

### For AI Assistants
1. Start with this document for library-level context
2. Review module-specific CLAUDE.md files for detailed component documentation:
   - [`Agent/CLAUDE.md`](src/Agent/CLAUDE.md) — Agent interface
   - [`Environment/CLAUDE.md`](src/Environment/CLAUDE.md) — Mathematical primitives
   - [`Physics/CLAUDE.md`](src/Physics/CLAUDE.md) — Rigid body dynamics
   - [`Utils/CLAUDE.md`](src/Utils/CLAUDE.md) — Helper utilities
3. Reference PlantUML diagrams in `docs/msd/msd-sim/` for visual architecture
4. Check [root CLAUDE.md](../../CLAUDE.md) for project-wide conventions

### For Developers
- **Mathematical primitives**: See Environment module (Coordinate, Angle, etc.)
- **Physics simulation**: See Physics module (ConvexHull, PhysicsComponent)
- **Custom agents**: Extend BaseAgent from Agent module
- **Object creation**: Use Object factory methods (createInertial, createGraphical, etc.)
- **Simulation loop**: Use Engine.update() with time stepping
