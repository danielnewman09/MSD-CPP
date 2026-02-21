# Ticket 0071d: Constraint Interface Static Dispatch Investigation

## Status
- [x] Draft
- [x] Investigation Complete
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Implementation
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Investigation Complete — No Implementation Required
**Type**: Performance / Investigation
**Priority**: Medium
**Created**: 2026-02-21
**Generate Tutorial**: No
**Parent Ticket**: [0071_collision_pipeline_profiling](0071_collision_pipeline_profiling.md)
**Related Tickets**: [0071c_eigen_fixed_size_matrices](0071c_eigen_fixed_size_matrices.md) (direct conversions without interface changes)

---

## Summary

The `Constraint` base class uses a virtual interface with `Eigen::MatrixXd` / `Eigen::VectorXd` return types for `evaluate()` and `jacobian()`. Every concrete constraint type has a compile-time-known fixed dimension (ContactConstraint: 1, FrictionConstraint: 2), yet the virtual interface forces heap-allocated dynamic returns on every call. This ticket investigates whether CRTP, `std::variant`-based dispatch, or another static polymorphism approach could eliminate virtual dispatch overhead and enable fixed-size Eigen return types at the interface level.

This is an **investigation and design** ticket. The direct, non-architectural fixed-size conversions (solver-internal data structures) are handled in [0071c](0071c_eigen_fixed_size_matrices.md). This ticket explores whether more fundamental changes to the constraint type system are warranted and feasible.

---

## Problem

### Virtual Interface Costs

The `Constraint` base class declares:
```cpp
virtual Eigen::VectorXd evaluate(const InertialState& stateA,
                                  const InertialState& stateB,
                                  double time) const = 0;

virtual Eigen::MatrixXd jacobian(const InertialState& stateA,
                                  const InertialState& stateB,
                                  double time) const = 0;
```

Each virtual call to `jacobian()` heap-allocates a `MatrixXd`. In `flattenConstraints()`, this is called once per constraint per frame — for 32 bodies with 50+ contacts (150+ constraints including friction), that's 150+ heap allocations per frame just for Jacobian computation.

### Current Constraint Type Census

| Constraint Type | Dimension | Jacobian Size | Body Count | Actively Used |
|-----------------|-----------|---------------|------------|---------------|
| `ContactConstraint` | 1 | 1x12 | 2 | Yes (CollisionPipeline) |
| `FrictionConstraint` | 2 | 2x12 | 2 | Yes (CollisionPipeline) |
| `UnitQuaternionConstraint` | 1 | 1x7 | 1 | No (vestigial, ticket 0058) |
| `DistanceConstraint` | 1 | 1x7 | 1 | No (example only) |

Only `ContactConstraint` and `FrictionConstraint` are actively instantiated. Both are two-body velocity-level constraints with 12-column Jacobians. The single-body 7-column variants are vestigial.

### Where the Virtual Interface Is Called

- `flattenConstraints()` — calls `constraint->jacobian()` for every constraint
- `assembleJacobians()` — calls `constraint->jacobian()` for every constraint (no-friction path)
- `assembleRHS()` / `assembleFlatRHS()` — uses pre-computed Jacobians (no virtual call)
- `PositionCorrector::correctPositions()` — calls `constraint->evaluate()` and `constraint->jacobian()` per iteration
- `ContactConstraint::isActive()` — calls `evaluate()` on self (non-polymorphic, could be direct)

---

## Investigation Directions

### Option A: CRTP (Curiously Recurring Template Pattern)

Replace virtual dispatch with static polymorphism:
```cpp
template<typename Derived, int Dim, int Cols>
class ConstraintBase {
public:
  Eigen::Matrix<double, Dim, 1> evaluate(...) const {
    return static_cast<const Derived*>(this)->evaluateImpl(...);
  }
  Eigen::Matrix<double, Dim, Cols> jacobian(...) const {
    return static_cast<const Derived*>(this)->jacobianImpl(...);
  }
};

class ContactConstraint : public ConstraintBase<ContactConstraint, 1, 12> { ... };
class FrictionConstraint : public ConstraintBase<FrictionConstraint, 2, 12> { ... };
```

**Trade-offs**:
- Eliminates virtual dispatch and enables fixed-size returns
- Cannot store heterogeneous constraints in a single `vector<Constraint*>` — would require `std::variant`, `std::tuple`, or separate typed containers
- `CollisionPipeline`, `ConstraintSolver`, `PositionCorrector` all operate on `vector<Constraint*>` today — significant refactor

### Option B: std::variant-Based Dispatch

Replace `vector<unique_ptr<Constraint>>` with `vector<variant<ContactConstraint, FrictionConstraint>>`:
```cpp
using AnyConstraint = std::variant<ContactConstraint, FrictionConstraint>;
std::vector<AnyConstraint> constraints;

// Dispatch via std::visit
std::visit([&](const auto& c) {
  auto j = c.jacobian(stateA, stateB, time);  // Fixed-size return
  // ...
}, constraints[i]);
```

**Trade-offs**:
- No virtual dispatch, fixed-size returns via overload resolution
- Closed type set — adding a new constraint type requires modifying the variant definition
- `std::visit` can be slower than virtual dispatch for large variant sets (but we have only 2 active types)
- Value semantics — constraints stored inline, no heap allocation for the constraint objects themselves

### Option C: Hybrid — Keep Virtual Interface, Add Non-Virtual Fixed-Size Accessors

Keep the existing virtual interface for extensibility but add non-virtual concrete-type methods:
```cpp
class ContactConstraint : public Constraint {
public:
  // Existing virtual override (for generic code paths)
  Eigen::VectorXd evaluate(...) const override;
  Eigen::MatrixXd jacobian(...) const override;

  // New non-virtual fixed-size accessors (for hot paths)
  Eigen::Matrix<double, 1, 1> evaluateFixed(...) const;
  Eigen::Matrix<double, 1, 12> jacobianFixed(...) const;
};
```

Hot paths that already know the concrete type (via `dynamic_cast` or separate typed containers) call the fixed-size methods directly.

**Trade-offs**:
- Minimal architectural change — additive, not breaking
- Duplicates logic between virtual and non-virtual methods (maintenance burden)
- Requires call sites to know concrete types — already the case in several places
- Does not eliminate the virtual call in paths that remain generic

### Option D: Output Buffer Pattern

Keep virtual interface but write into caller-provided fixed-size buffers:
```cpp
class Constraint {
public:
  virtual void jacobianInto(double* output, int rows, int cols) const = 0;
};

// Caller provides stack-allocated buffer
Eigen::Matrix<double, 1, 12> j;
constraint->jacobianInto(j.data(), 1, 12);
```

**Trade-offs**:
- Preserves polymorphism and extensibility
- Eliminates heap allocation (caller owns the buffer)
- Less ergonomic API — caller must know dimensions before calling
- Still has virtual dispatch overhead (but eliminates the allocation overhead)

---

## Investigation Questions

1. **What is the actual cost split?** Of the 24 samples in `flattenConstraints()`, how much is virtual dispatch overhead vs. heap allocation overhead? If allocation dominates, Option C or D may suffice. If vtable indirection matters, Options A or B are needed.

2. **Is the constraint type set truly closed?** The CLAUDE.md documents future plans for joints (hinges, ball-socket). If new constraint types are expected, Options A and B impose a maintenance cost. If the type set is effectively {Contact, Friction} for the foreseeable future, variant-based dispatch is attractive.

3. **How does 0071c's direct conversion interact?** After 0071c converts `FlattenedConstraints::jacobianRows` to `Matrix<double, 1, 12>`, the virtual `jacobian()` call in `flattenConstraints()` still returns `MatrixXd` but is immediately copied into fixed-size storage. The remaining cost is just the temporary `MatrixXd` allocation. Is that residual cost worth an architectural change?

4. **PositionCorrector impact**: `correctPositions()` calls `evaluate()` and `jacobian()` per constraint per iteration (multiple iterations per frame). This is a separate hot path from the velocity solver. Any solution must cover this path as well.

---

## Success Criteria

- Investigation produces a clear recommendation with profiling data supporting the choice
- Design document evaluates at least Options A-D above with concrete impact estimates
- If proceeding to implementation: no physics test regressions, no constraint extensibility regression beyond what's explicitly accepted
- Measurable improvement in `flattenConstraints()` and/or `PositionCorrector` profiling samples beyond what 0071c achieves

---

## Profiling Artifacts

- Parent ticket profiling: `analysis/profile_results/profile_0071_multibody_*.json`
- Benchmark: `msd/msd-sim/bench/MultiBodyCollisionBench.cpp`
- Post-0071c profiling (when available) will establish the residual cost baseline

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-21
- **Notes**:
  - Ticket created to separate architectural interface investigation from direct conversions (0071c)
  - Four candidate approaches documented (CRTP, variant, hybrid, output buffer)
  - Investigation questions defined to guide design phase
  - Depends on 0071c completing first to establish residual cost baseline

### Investigation Phase
- **Started**: 2026-02-21
- **Completed**: 2026-02-21
- **Branch**: 0071d-constraint-interface-static-dispatch
- **PR**: N/A
- **Artifacts**:
  - `docs/designs/0071d_constraint_interface_static_dispatch/design.md`
  - `docs/investigations/0071d_constraint_interface_static_dispatch/iteration-log.md`
- **Notes**:
  - Investigated all four options (CRTP, std::variant, hybrid, output buffer)
  - Confirmed 0071c delivered the boundary pattern: fixed-size types in solver
    compute zone, MatrixXd only at virtual call boundary in flattenConstraints
    and PositionCorrector
  - Profiling attribution: remaining virtual-interface MatrixXd allocations
    contribute <0.2% of CPU vs 3.6% Eigen BLAS baseline — not justifiable
  - All four architectural options: cost/benefit unfavorable (closed type set
    or maintenance burden, <0.2% gain)
  - **Recommendation: Do not proceed with any architectural change**
  - Identified PositionCorrector::jacobians_ (vector<MatrixXd>) as the only
    remaining worthwhile non-architectural conversion — suggest follow-on ticket
  - Investigation questions from Draft all answered:
    1. Cost split: allocation cost is minor vs BLAS compute; virtual dispatch
       overhead itself is negligible. Neither justifies architectural change.
    2. Type set: NOT closed (joints planned). Strengthens case against CRTP/variant.
    3. 0071c residual: after boundary pattern, remaining alloc is flattenConstraints
       temporary MatrixXd per constraint — confirmed minor.
    4. PositionCorrector: already uses reused workspace (0053f); after frame 1,
       no allocation — only virtual call + copy overhead.
