# Constraints Module Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/msd/msd-sim/Physics/` for detailed component relationships.

**Diagram**: [`generalized-constraints.puml`](../../../../../../docs/msd/msd-sim/Physics/generalized-constraints.puml)

## Module Overview

**Constraints** is a sub-module within the Physics module that provides an extensible constraint framework using Lagrange multipliers. The system enables users to define arbitrary constraints by implementing the `Constraint` interface, with all constraints enforced by a unified solver infrastructure.

**Key benefit**: New constraint types (joints, contacts, limits) can be added by implementing the `Constraint` interface without modifying the solver or integration infrastructure.

**Introduced**: [Ticket: 0031_generalized_lagrange_constraints](../../../../../../tickets/0031_generalized_lagrange_constraints.md)

---

## Architecture Overview

### High-Level Architecture

The Constraints module provides a layered architecture for constraint-based physics:

```
AssetInertial (Owns constraints)
    ├── std::vector<std::unique_ptr<Constraint>>
    │   ├── UnitQuaternionConstraint (default)
    │   ├── DistanceConstraint
    │   └── ... (user-defined constraints)
    │
    └── SemiImplicitEulerIntegrator
        └── ConstraintSolver
            ├── Assembles constraint matrix: A = J·M⁻¹·Jᵀ
            ├── Builds RHS: b = -J·M⁻¹·F_ext - α·C - β·Ċ
            ├── Solves: A·λ = b (LLT decomposition)
            └── Extracts forces: F_constraint = Jᵀ·λ
```

### Core Components

| Component | Location | Purpose | Type |
|-----------|----------|---------|------|
| Constraint | `Constraint.hpp` | Abstract constraint interface | Abstract base class |
| BilateralConstraint | `BilateralConstraint.hpp` | Equality constraint marker (C = 0) | Abstract subclass |
| UnilateralConstraint | `UnilateralConstraint.hpp` | Inequality constraint marker (C ≥ 0) | Abstract subclass |
| ConstraintSolver | `ConstraintSolver.hpp` | Lagrange multiplier solver | Utility class |
| UnitQuaternionConstraint | `UnitQuaternionConstraint.hpp` | Unit quaternion normalization | Concrete implementation |
| DistanceConstraint | `DistanceConstraint.hpp` | Fixed distance from origin | Concrete implementation |
| QuaternionConstraint | `QuaternionConstraint.hpp` | **DEPRECATED** - Use UnitQuaternionConstraint | Legacy class |

---

## Mathematical Framework

### Constraint Definition

A constraint is mathematically defined by:

1. **Constraint function**: `C(q, t) = 0` (holonomic) or `C(q, q̇, t) = 0` (non-holonomic)
2. **Constraint Jacobian**: `J = ∂C/∂q` (how constraint changes with configuration)
3. **Constraint time derivative**: `Ċ = J·q̇ + ∂C/∂t`
4. **Baumgarte stabilization terms**: `α` (position gain), `β` (velocity gain)

### Lagrange Multiplier Formulation

The Lagrange multiplier λ is computed to satisfy:
```
J·M⁻¹·Jᵀ·λ = -J·M⁻¹·F_ext - J̇·q̇ - α·C - β·Ċ
```

The constraint force applied is:
```
F_constraint = Jᵀ·λ
```

Where:
- `J` is the constraint Jacobian (dimension × 7 for single-object constraints)
- `M⁻¹` is the inverse mass matrix (block diagonal: [m⁻¹·I₃, I⁻¹])
- `F_ext` is the external force vector
- `α` is the position error gain (default: 10.0)
- `β` is the velocity error gain (default: 10.0)
- `C` is the constraint violation
- `Ċ` is the constraint velocity violation

### Baumgarte Stabilization

The Baumgarte stabilization method prevents constraint drift by adding feedback terms:
- **Position term**: `-α·C` corrects position-level violations
- **Velocity term**: `-β·Ċ` corrects velocity-level violations

Default parameters: `α = 10.0`, `β = 10.0` (validated for dt ≈ 0.016s / 60 FPS)

---

## Component Details

### Constraint (Abstract Base Class)

**Location**: `Constraint.hpp`, `Constraint.cpp`
**Type**: Abstract interface

#### Purpose
Defines the mathematical interface for arbitrary constraint definitions in Lagrangian mechanics. All concrete constraint types inherit from this class and provide constraint evaluation, Jacobian computation, and optional time derivatives.

#### Key Interfaces
```cpp
class Constraint {
public:
  virtual ~Constraint() = default;

  // Number of scalar constraint equations
  virtual int dimension() const = 0;

  // Evaluate constraint function C(q, t)
  // Returns vector of constraint violations
  virtual Eigen::VectorXd evaluate(
      const InertialState& state,
      double time) const = 0;

  // Compute constraint Jacobian J = ∂C/∂q
  // Returns (dimension × 7) matrix for single-object constraints
  virtual Eigen::MatrixXd jacobian(
      const InertialState& state,
      double time) const = 0;

  // Compute constraint time derivative ∂C/∂t (optional, default: zero)
  virtual Eigen::VectorXd partialTimeDerivative(
      const InertialState& state,
      double time) const;

  // Baumgarte stabilization parameters
  virtual double alpha() const { return 10.0; }
  virtual double beta() const { return 10.0; }

  // Constraint type for debugging/logging
  virtual std::string typeName() const = 0;

protected:
  Constraint() = default;
  // Rule of Five with = default
};
```

#### Usage Example
```cpp
// Users implement Constraint interface for custom constraints
class MyCustomConstraint : public BilateralConstraint {
public:
  int dimension() const override { return 1; }

  Eigen::VectorXd evaluate(const InertialState& state, double time) const override {
    // Compute constraint violation C(q, t)
    Eigen::VectorXd C(1);
    C(0) = /* constraint function */;
    return C;
  }

  Eigen::MatrixXd jacobian(const InertialState& state, double time) const override {
    // Compute ∂C/∂q
    Eigen::MatrixXd J(1, 7);
    J << /* analytical Jacobian */;
    return J;
  }

  std::string typeName() const override { return "MyCustomConstraint"; }
};
```

#### Thread Safety
**Read-only operations thread-safe after construction** — Evaluation and Jacobian computation are const methods.

#### Error Handling
Implementations may throw `std::invalid_argument` for invalid state or parameters.

---

### BilateralConstraint (Abstract Subclass)

**Location**: `BilateralConstraint.hpp`
**Type**: Abstract subclass

#### Purpose
Specialization for equality constraints `C(q, t) = 0` with unrestricted Lagrange multipliers. This is a semantic marker that enables future solver optimizations distinguishing between equality and inequality constraints.

#### Key Features
- Enforces exact equalities using unrestricted Lagrange multipliers (λ can be positive or negative)
- Examples: Unit quaternion, fixed distance, ball-socket joint

#### Usage Example
```cpp
class UnitQuaternionConstraint : public BilateralConstraint {
  // C(Q) = Q^T*Q - 1 = 0 (equality constraint)
  // λ unrestricted
};
```

#### Thread Safety
Same as `Constraint` base class.

---

### UnilateralConstraint (Abstract Subclass)

**Location**: `UnilateralConstraint.hpp`
**Type**: Abstract subclass

#### Purpose
Specialization for inequality constraints `C(q, t) ≥ 0` with complementarity conditions. Provides interface for contact activation logic.

#### Key Features
- Enforces one-sided inequalities with complementarity conditions:
  - `C ≥ 0` (constraint satisfied when non-negative)
  - `λ ≥ 0` (force only pushes, never pulls)
  - `λ·C = 0` (force is zero when constraint is inactive)
- Examples: Contact non-penetration, joint angle limits

#### Key Interfaces
```cpp
class UnilateralConstraint : public Constraint {
public:
  /**
   * @brief Check if constraint is active (C ≈ 0)
   * Inactive constraints (C > threshold) are not included in solver.
   */
  virtual bool isActive(const InertialState& state, double time) const = 0;

  // ... inherited Constraint interface
};
```

#### Implementation Status
**Interface defined, solver not yet implemented**. Current `ConstraintSolver` only handles bilateral constraints. Unilateral solver (projected Gauss-Seidel) deferred to future ticket.

#### Thread Safety
Same as `Constraint` base class.

---

### ConstraintSolver

**Location**: `ConstraintSolver.hpp`, `ConstraintSolver.cpp`
**Type**: Utility class

#### Purpose
Computes Lagrange multipliers for arbitrary constraint systems using direct linear solve (LLT decomposition). Suitable for small constraint counts (n < 100, typical: 1-10 per object).

#### Algorithm Overview
1. **Assemble constraint Jacobian** J by stacking all constraint Jacobians
2. **Compute mass matrix inverse** M⁻¹ (block diagonal: [m⁻¹·I₃, I⁻¹])
3. **Form constraint matrix**: A = J·M⁻¹·Jᵀ
4. **Build RHS**: b = -J·M⁻¹·F_ext - J̇·q̇ - α·C - β·Ċ
5. **Solve A·λ = b** using Eigen LLT decomposition
6. **Extract forces**: F_constraint = Jᵀ·λ

#### Key Interfaces
```cpp
class ConstraintSolver {
public:
  struct SolveResult {
    Eigen::VectorXd lambdas;               // Lagrange multipliers
    Coordinate linearConstraintForce;      // Net linear force [N]
    Coordinate angularConstraintForce;     // Net angular torque [N·m]
    bool converged{false};                 // Solver convergence flag
    double conditionNumber{NaN};           // Matrix conditioning
  };

  ConstraintSolver() = default;

  /**
   * @brief Solve constraint system for Lagrange multipliers
   * @return SolveResult with forces and convergence status
   */
  SolveResult solve(
      const std::vector<Constraint*>& constraints,
      const InertialState& state,
      const Coordinate& externalForce,
      const Coordinate& externalTorque,
      double mass,
      const Eigen::Matrix3d& inverseInertia,
      double dt);

  // Rule of Five with = default
};
```

#### Usage Example
```cpp
// Called internally by SemiImplicitEulerIntegrator::step()
ConstraintSolver solver;
auto result = solver.solve(
    asset.getConstraints(),  // Non-owning pointers
    asset.getInertialState(),
    externalForce,
    externalTorque,
    asset.getMass(),
    asset.getInverseInertiaTensor(),
    dt);

if (result.converged) {
  // Apply constraint forces
  Coordinate totalForce = externalForce + result.linearConstraintForce;
  Coordinate totalTorque = externalTorque + result.angularConstraintForce;
  // Proceed with integration...
} else {
  // Handle singular constraint matrix (rare)
  // Condition number available in result.conditionNumber
}
```

#### Performance
- **Complexity**: O(n³) where n = total constraint dimension (direct LLT solve)
- **Typical usage**: n < 10 constraints per object (< 1ms solve time)
- **Overhead**: ~1-2% of physics loop for typical constraint sets

#### Thread Safety
**Stateless utility** — Safe to call from multiple threads with different constraint sets.

#### Error Handling
Returns `converged = false` for singular/ill-conditioned matrices. No exceptions thrown.

#### Memory Management
- Stateless class (no member variables beyond vtable)
- All matrices are local variables (freed after solve)

---

### UnitQuaternionConstraint

**Location**: `UnitQuaternionConstraint.hpp`, `UnitQuaternionConstraint.cpp`
**Type**: Concrete implementation (BilateralConstraint)

#### Purpose
Maintains unit quaternion normalization constraint |Q| = 1 during numerical integration. Drop-in replacement for the deprecated `QuaternionConstraint` class.

#### Mathematical Formulation
- **Constraint**: C(Q) = Q^T·Q - 1 = 0 (scalar constraint)
- **Jacobian**: J = 2·Q^T (1×4 row vector w.r.t. quaternion components)
- **Time derivative**: ∂C/∂t = 0 (time-independent)

#### Key Interfaces
```cpp
class UnitQuaternionConstraint : public BilateralConstraint {
public:
  explicit UnitQuaternionConstraint(double alpha = 10.0, double beta = 10.0);

  int dimension() const override { return 1; }
  Eigen::VectorXd evaluate(const InertialState& state, double time) const override;
  Eigen::MatrixXd jacobian(const InertialState& state, double time) const override;
  std::string typeName() const override { return "UnitQuaternionConstraint"; }

  // Baumgarte parameter setters/getters
  void setAlpha(double alpha);
  void setBeta(double beta);
  double alpha() const override;
  double beta() const override;

  // Rule of Five with = default
};
```

#### Usage Example
```cpp
// Automatically added to every AssetInertial by default
// (user does not need to manually add this constraint)

// Custom Baumgarte parameters if needed:
auto constraint = std::make_unique<UnitQuaternionConstraint>(15.0, 12.0);
asset.addConstraint(std::move(constraint));
```

#### Accuracy
Maintains |Q²-1| < 1e-10 over 10000 integration steps with default parameters (validated by unit tests).

#### Thread Safety
**Read-only operations thread-safe after construction**.

#### Memory Management
- Two `double` members (16 bytes total)
- Value semantics with compiler-generated copy/move

---

### DistanceConstraint

**Location**: `DistanceConstraint.hpp`, `DistanceConstraint.cpp`
**Type**: Concrete implementation (BilateralConstraint)

#### Purpose
Maintains fixed distance from origin constraint |X| = d. Single-object constraint demonstrating the constraint framework. Multi-object distance constraints (between two objects) deferred to future ticket.

#### Mathematical Formulation
- **Constraint**: C(X) = |X|² - d² = 0 (scalar constraint)
- **Jacobian**: J = 2·X^T (1×3 row vector w.r.t. position components)
- **Time derivative**: ∂C/∂t = 0 (time-independent)

#### Use Cases
- Orbital mechanics: fixed orbital radius
- Pendulum: fixed rod length
- Tethered objects: cable length constraint

#### Key Interfaces
```cpp
class DistanceConstraint : public BilateralConstraint {
public:
  explicit DistanceConstraint(double targetDistance,
                              double alpha = 10.0,
                              double beta = 10.0);

  int dimension() const override { return 1; }
  Eigen::VectorXd evaluate(const InertialState& state, double time) const override;
  Eigen::MatrixXd jacobian(const InertialState& state, double time) const override;
  std::string typeName() const override { return "DistanceConstraint"; }

  double getTargetDistance() const;

  // Rule of Five with = default
};
```

#### Usage Example
```cpp
// Add distance constraint to keep object at 5m from origin
asset.addConstraint(std::make_unique<DistanceConstraint>(5.0));

// Object will maintain fixed distance while still free to rotate
// and move tangentially around origin (orbital motion)
```

#### Thread Safety
**Read-only operations thread-safe after construction**.

#### Error Handling
Throws `std::invalid_argument` if `targetDistance <= 0`.

#### Memory Management
- Three `double` members (24 bytes total)
- Value semantics with compiler-generated copy/move

---

### QuaternionConstraint (DEPRECATED)

**Location**: `QuaternionConstraint.hpp`, `QuaternionConstraint.cpp`
**Type**: Legacy class (ticket 0030)

#### Deprecation Notice
**This class is deprecated**. Use `UnitQuaternionConstraint` with `ConstraintSolver` instead.

#### Migration Path
The old `QuaternionConstraint` uses a different API:
```cpp
// Old API (deprecated)
QuaternionConstraint constraint{10.0, 10.0};
constraint.enforceConstraint(Q, Qdot);

// New API (recommended)
auto constraint = std::make_unique<UnitQuaternionConstraint>(10.0, 10.0);
asset.addConstraint(std::move(constraint));
// Constraint enforced automatically by SemiImplicitEulerIntegrator
```

#### Removal Timeline
This class will be removed in a future release after all callers have migrated to the new constraint framework.

---

## Integration with Physics Pipeline

### Ownership Model

**AssetInertial owns constraints**:
```cpp
class AssetInertial {
private:
  std::vector<std::unique_ptr<Constraint>> constraints_;

public:
  // Non-owning access for solver
  std::vector<Constraint*> getConstraints() const;

  // Add custom constraint
  void addConstraint(std::unique_ptr<Constraint> constraint);
};
```

### Default Constraints

Every `AssetInertial` automatically includes a `UnitQuaternionConstraint` to maintain quaternion normalization.

### Integration Workflow

1. **AssetInertial ownership**: Each `AssetInertial` owns a vector of constraints via `std::vector<std::unique_ptr<Constraint>>`
2. **Default constraint**: Every `AssetInertial` automatically includes a `UnitQuaternionConstraint`
3. **Constraint gathering**: `WorldModel::updatePhysics()` gathers constraints from all assets
4. **Solver invocation**: `SemiImplicitEulerIntegrator::step()` uses `ConstraintSolver` to compute constraint forces
5. **Force application**: Constraint forces are added to external forces before integration

### Integrator Signature Change (Breaking)

Ticket 0031 changed the integrator signature:

```cpp
// Old (ticket 0030)
void Integrator::step(InertialState& state,
                      const Coordinate& force,
                      const Coordinate& torque,
                      double mass,
                      const Eigen::Matrix3d& inverseInertia,
                      QuaternionConstraint& constraint,
                      double dt);

// New (ticket 0031)
void Integrator::step(InertialState& state,
                      const Coordinate& force,
                      const Coordinate& torque,
                      double mass,
                      const Eigen::Matrix3d& inverseInertia,
                      const std::vector<Constraint*>& constraints,
                      double dt);
```

---

## Adding Custom Constraints

### Step 1: Define Constraint Class

Inherit from `BilateralConstraint` or `UnilateralConstraint`:

```cpp
class MyJointConstraint : public BilateralConstraint {
public:
  explicit MyJointConstraint(/* parameters */)
    : /* initialize members */ {}

  int dimension() const override {
    return 3;  // 3 scalar constraint equations
  }

  Eigen::VectorXd evaluate(const InertialState& state, double time) const override {
    Eigen::VectorXd C(3);
    // Compute constraint violations
    C(0) = /* constraint 1 */;
    C(1) = /* constraint 2 */;
    C(2) = /* constraint 3 */;
    return C;
  }

  Eigen::MatrixXd jacobian(const InertialState& state, double time) const override {
    Eigen::MatrixXd J(3, 7);
    // Compute analytical Jacobian ∂C/∂q
    // J has 3 rows (one per constraint) and 7 columns (3 position + 4 quaternion)
    return J;
  }

  std::string typeName() const override {
    return "MyJointConstraint";
  }
};
```

### Step 2: Add to AssetInertial

```cpp
AssetInertial asset{/* ... */};

// Add custom constraint
asset.addConstraint(std::make_unique<MyJointConstraint>(/* args */));

// Constraint automatically enforced during physics updates
```

### Step 3: Test Constraint

Validate constraint evaluation, Jacobian correctness, and convergence:

```cpp
TEST_CASE("MyJointConstraint: Jacobian correctness") {
  MyJointConstraint constraint{/* args */};
  InertialState state = /* setup test state */;

  // Analytical Jacobian
  Eigen::MatrixXd J_analytical = constraint.jacobian(state, 0.0);

  // Numerical Jacobian (finite differences)
  Eigen::MatrixXd J_numerical = /* compute via finite differences */;

  // Validate
  REQUIRE((J_analytical - J_numerical).norm() < 1e-6);
}
```

---

## Design Patterns

### Strategy Pattern
Constraint implementations use the Strategy pattern, allowing different constraint types to be swapped at runtime without changing the solver.

### Template Method Pattern
`Constraint` base class defines the overall algorithm structure (evaluate, Jacobian, stabilization parameters), with concrete classes filling in specific behavior.

### Factory Pattern (Implicit)
AssetInertial automatically creates a `UnitQuaternionConstraint` in its constructor, following the Factory pattern for default initialization.

---

## Performance Considerations

### Solver Complexity
- **Direct solve**: O(n³) where n = total constraint dimension
- **Recommended**: n < 100 for real-time performance (typical: n = 1-10)
- **Future optimization**: Sparse matrices, iterative solvers for large n

### Virtual Dispatch Overhead
- **Measured**: < 1% overhead compared to non-virtual baseline (validated by prototypes)
- **Negligible**: Dominated by matrix operations in solver

### Memory Allocation
- **Hot path**: No heap allocations during constraint solving
- **Matrices**: Pre-allocated in local scope, freed after solve
- **Constraint storage**: Owned by AssetInertial, allocated once at creation

---

## Limitations and Future Work

### Current Limitations

1. **Single-object constraints only**: Current constraints operate on single object state (X, Q)
   - Multi-object constraints (joints, contacts) deferred to future ticket
2. **Bilateral solver only**: `ConstraintSolver` handles only equality constraints
   - Unilateral solver (complementarity) deferred to future ticket
3. **Direct solve only**: LLT decomposition suitable for small constraint counts
   - Iterative solvers (projected Gauss-Seidel) deferred for large-scale systems

### Future Extensions

- **Multi-object constraints**: Joints, contacts, cable/spring connections
- **Unilateral solver**: Projected Gauss-Seidel for inequality constraints
- **Motor/actuator constraints**: Constraints with target velocities
- **Soft constraints**: Spring-damper approximations with compliance
- **Constraint graphs**: Hierarchical constraint dependencies
- **Warm starting**: Use previous λ as initial guess for iterative solvers
- **Sparse matrices**: Memory-efficient representation for large constraint systems

---

## Testing

### Test Organization
```
test/Physics/Constraints/
└── ConstraintTest.cpp      # 30 unit tests covering all framework components
```

### Test Coverage

**UnitQuaternionConstraint**: 9 tests
- Dimension verification
- Constraint evaluation (normalized/violated)
- Jacobian correctness (analytical vs numerical)
- Time derivative (zero)
- Baumgarte parameter modification
- Type name

**DistanceConstraint**: 9 tests
- Dimension verification
- Constraint evaluation (satisfied/violated)
- Jacobian correctness (analytical vs numerical)
- Time derivative (zero)
- Parameter validation (invalid distance throws)
- Type name

**ConstraintSolver**: 4 tests
- Single constraint solving
- Multiple constraint solving
- Convergence flag validation
- Force extraction

**AssetInertial Integration**: 5 tests
- Default constraint addition (UnitQuaternionConstraint)
- Custom constraint addition
- Multiple constraint ownership
- Constraint retrieval (non-owning pointers)

**Integration Tests**: 3 tests
- Constraint enforcement during integration step
- Multiple constraints enforced simultaneously
- Convergence over multiple timesteps

### Running Tests
```bash
cmake --build --preset debug-tests-only
ctest --preset conan-debug --tests-regex ConstraintTest
```

---

## Coding Standards

This module follows the project-wide coding standards defined in the [root CLAUDE.md](../../../../../../CLAUDE.md#coding-standards).

Key standards applied in this module:
- **Initialization**: `std::numeric_limits<double>::quiet_NaN()` for uninitialized floats
- **Naming**: `PascalCase` for classes, `camelCase` for methods, `snake_case_` for members
- **Return Values**: Return `Eigen::VectorXd` / `Eigen::MatrixXd` over output parameters
- **Memory**: `std::unique_ptr` for constraint ownership, raw pointers for non-owning access

See the [root CLAUDE.md](../../../../../../CLAUDE.md#coding-standards) for complete details and examples.

---

## References

### Academic Literature
- Baraff, D. (1996). "Linear-Time Dynamics using Lagrange Multipliers"
- Cline, M. B. (2002). "Rigid Body Simulation with Contact and Constraints"
- Baumgarte, J. (1972). "Stabilization of Constraints and Integrals of Motion in Dynamical Systems"

### Physics Engine Implementations
- Bullet Physics Library: `btSequentialImpulseConstraintSolver`
- ODE (Open Dynamics Engine): Constraint solving architecture
- Box2D: Contact constraint resolution

---

## Getting Help

### For AI Assistants
1. This document provides complete architectural context for the Constraints module
2. Review [`Physics/CLAUDE.md`](../CLAUDE.md) for overall Physics module architecture
3. Review [`msd-sim/CLAUDE.md`](../../../../CLAUDE.md) for simulation engine context
4. Check [root CLAUDE.md](../../../../../../CLAUDE.md) for project-wide conventions

### For Developers
- **Custom constraints**: Implement `Constraint` interface (see "Adding Custom Constraints")
- **Solver internals**: See `ConstraintSolver` documentation above
- **Integration**: Constraints automatically enforced by `SemiImplicitEulerIntegrator`
- **Debugging**: Check `SolveResult::converged` and `conditionNumber` for solver health
