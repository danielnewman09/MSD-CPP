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

```
Constraint (abstract)
├── BilateralConstraint (C = 0, λ unrestricted)
│   ├── UnitQuaternionConstraint
│   └── DistanceConstraint
└── UnilateralConstraint (C ≥ 0, λ ≥ 0)
    └── ContactConstraint (via TwoBodyConstraint)
```

### Implementing Custom Constraints

```cpp
class MyConstraint : public BilateralConstraint {
public:
  int dimension() const override { return 1; }

  Eigen::VectorXd evaluate(const InertialState& state, double) const override {
    return Eigen::VectorXd::Constant(1, /* C(q) */);
  }

  Eigen::MatrixXd jacobian(const InertialState& state, double) const override {
    Eigen::MatrixXd J(1, 7);
    J << /* ∂C/∂q */;
    return J;
  }

  std::string typeName() const override { return "MyConstraint"; }
};

// Usage
asset.addConstraint(std::make_unique<MyConstraint>());
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
