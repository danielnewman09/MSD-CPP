# Constraints Module Architecture

> Extensible constraint framework using Lagrange multipliers.
> For API details, use MCP: `find_class ConstraintSolver`, `get_class_members Constraint`, etc.

## Architecture Overview

```
AssetInertial (Owns constraints)
    └── std::vector<std::unique_ptr<Constraint>>
        ├── UnitQuaternionConstraint (default)
        ├── DistanceConstraint
        └── ... (user-defined constraints)

SemiImplicitEulerIntegrator
    └── ConstraintSolver
        ├── Bilateral constraints → LLT direct solve
        ├── Contacts (no friction) → Active Set Method
        └── Contacts (with friction) → ECOS SOCP solver
```

**Key benefit**: New constraint types (joints, limits) can be added by implementing the `Constraint` interface without modifying the solver.

---

## Mathematical Framework

### Lagrange Multiplier Formulation

The multiplier λ is computed to satisfy:
```
J·M⁻¹·Jᵀ·λ = -J·M⁻¹·F_ext - J̇·q̇ - α·C - β·Ċ
```

Constraint force: `F_constraint = Jᵀ·λ`

Where:
- `J` = constraint Jacobian (∂C/∂q)
- `α`, `β` = Baumgarte stabilization gains (default: 10.0)
- `C` = constraint violation, `Ċ` = velocity violation

### Baumgarte Stabilization

Prevents constraint drift by adding feedback:
- Position term: `-α·C` corrects position errors
- Velocity term: `-β·Ċ` corrects velocity errors

---

## Solver Methods

| Constraint Type | Method | Complexity | Notes |
|-----------------|--------|------------|-------|
| Bilateral (single-body) | LLT direct | O(n³) | n < 100 for real-time |
| Contact (no friction) | Active Set Method | Finite iterations | Exact LCP solution |
| Contact (with friction) | ECOS SOCP | ~5-30 iterations | Exact friction cone |

### Active Set Method (Tickets 0032b, 0034)

Replaces Projected Gauss-Seidel for exact LCP solution:
- Handles mass ratios up to 1e6:1
- Deterministic (no ordering sensitivity)
- No iteration tuning required

### ECOS SOCP Solver (Ticket 0035b4)

For friction constraints:
- Satisfies ||λ_t|| ≤ μ·λ_n exactly
- Interior-point with superlinear convergence
- See [ECOS/CLAUDE.md](ECOS/CLAUDE.md) for integration details

---

## Constraint Hierarchy

**Diagram**: [`0043_constraint_hierarchy_refactor.puml`](../../../../../docs/designs/0043_constraint_hierarchy_refactor/0043_constraint_hierarchy_refactor.puml)
**Introduced**: [Ticket: 0043_constraint_hierarchy_refactor](../../../../../tickets/0043_constraint_hierarchy_refactor.md)

The constraint type hierarchy is a flat 2-level design where all concrete constraints inherit directly from the `Constraint` base class. This eliminates excessive inheritance depth and provides a clean, extensible interface.

```
Constraint (abstract base)
├── UnitQuaternionConstraint (single-body, bilateral)
├── DistanceConstraint (single-body, bilateral)
├── ContactConstraint (two-body, unilateral)
└── FrictionConstraint (two-body, box-constrained)
```

### Key Base Class Features

The `Constraint` base class provides:

- **Unified evaluation interface**: All constraints use `evaluate(stateA, stateB, time)` and `jacobian(stateA, stateB, time)` signatures. Single-body constraints ignore `stateB`.
- **Body index storage**: `bodyAIndex()` and `bodyBIndex()` accessible on all constraints. Single-body constraints use only `bodyAIndex()`.
- **Body count query**: `bodyCount()` returns 1 or 2, enabling solver dispatch without type casting.
- **Multiplier bounds**: `lambdaBounds()` returns a `LambdaBounds` struct specifying constraint type (bilateral/unilateral/box-constrained).
- **Activation query**: `isActive(stateA, stateB, time)` determines if constraint is currently enforced (default: always active).
- **Baumgarte stabilization**: `alpha()` and `beta()` provide position and velocity correction gains (default: 10.0).

### LambdaBounds Value Type

The `LambdaBounds` struct encodes constraint multiplier semantics:

```cpp
struct LambdaBounds {
  double lower;
  double upper;

  static LambdaBounds bilateral();      // (-∞, +∞) for equality constraints
  static LambdaBounds unilateral();     // (0, +∞) for inequality constraints
  static LambdaBounds boxConstrained(double lo, double hi);  // Custom bounds

  bool isBilateral() const;
  bool isUnilateral() const;
  bool isBoxConstrained() const;
};
```

This replaces the previous `BilateralConstraint` and `UnilateralConstraint` marker classes with a data-oriented approach.

### Implementing Custom Constraints

**Single-body constraint example:**

```cpp
class MyConstraint : public Constraint {
public:
  MyConstraint(size_t bodyIndex)
    : Constraint(bodyIndex) {}  // Single-body: only bodyAIndex

  int dimension() const override { return 1; }

  Eigen::VectorXd evaluate(const InertialState& stateA,
                           const InertialState& /* stateB */,
                           double /* time */) const override {
    return Eigen::VectorXd::Constant(1, /* C(q_A) */);
  }

  Eigen::MatrixXd jacobian(const InertialState& stateA,
                           const InertialState& /* stateB */,
                           double /* time */) const override {
    Eigen::MatrixXd J(1, 7);  // 1 constraint × 7 DOF (position-level)
    J << /* ∂C/∂q_A */;
    return J;
  }

  LambdaBounds lambdaBounds() const override {
    return LambdaBounds::bilateral();
  }

  std::string typeName() const override { return "MyConstraint"; }
};

// Usage
asset.addConstraint(std::make_unique<MyConstraint>(assetIndex));
```

**Two-body constraint example:**

```cpp
class MyJointConstraint : public Constraint {
public:
  MyJointConstraint(size_t bodyAIndex, size_t bodyBIndex)
    : Constraint(bodyAIndex, bodyBIndex) {}  // Two-body constructor

  int dimension() const override { return 3; }  // 3 constraint rows
  int bodyCount() const override { return 2; }  // Two-body constraint

  Eigen::VectorXd evaluate(const InertialState& stateA,
                           const InertialState& stateB,
                           double /* time */) const override {
    // Constraint violation using both states
    return Eigen::VectorXd::Constant(3, /* C(q_A, q_B) */);
  }

  Eigen::MatrixXd jacobian(const InertialState& stateA,
                           const InertialState& stateB,
                           double /* time */) const override {
    Eigen::MatrixXd J(3, 12);  // 3 constraints × 12 DOF (velocity-level)
    J << /* ∂C/∂(v_A, ω_A, v_B, ω_B) */;
    return J;
  }

  LambdaBounds lambdaBounds() const override {
    return LambdaBounds::bilateral();
  }

  std::string typeName() const override { return "MyJointConstraint"; }
};

// Usage
worldModel.addConstraint(std::make_unique<MyJointConstraint>(indexA, indexB));
```

---

## Integration with Physics Pipeline

1. **Ownership**: AssetInertial owns constraints via `unique_ptr`
2. **Default**: Every AssetInertial includes UnitQuaternionConstraint
3. **Gathering**: WorldModel collects constraints from all assets
4. **Solving**: SemiImplicitEulerIntegrator invokes ConstraintSolver
5. **Application**: Constraint forces added before integration

---

## Design Decisions

### Why Baumgarte Stabilization?

Numerical drift causes constraint violations to accumulate. Baumgarte adds proportional feedback to push the system back toward the constraint manifold. Default gains (α=β=10) validated for dt ≈ 0.016s.

### Why Active Set Method over PGS?

PGS (Projected Gauss-Seidel) produced approximate solutions sensitive to iteration count, constraint ordering, and mass ratios. ASM provides exact LCP solutions with finite iteration guarantees.

### Why ECOS for Friction?

Box friction approximation (linearized cone) had 29% error. ECOS solves second-order cone programs exactly, giving proper ||λ_t|| ≤ μ·λ_n behavior.

---

## Limitations and Future Work

**Current**:
- Two-body constraints limited to contacts
- Direct solve for bilateral (n < 100)
- No warm starting

**Planned**:
- Multi-body joints (hinges, ball-socket)
- Sparse solvers for large systems
- Warm starting from previous frame

---

## Related Documentation

- **ECOS Integration**: [ECOS/CLAUDE.md](ECOS/CLAUDE.md)
- **Physics Overview**: [../CLAUDE.md](../CLAUDE.md)
- **Diagrams**: `docs/msd/msd-sim/Physics/generalized-constraints.puml`

## References

- Baraff (1996): "Linear-Time Dynamics using Lagrange Multipliers"
- Baumgarte (1972): "Stabilization of Constraints and Integrals of Motion"
