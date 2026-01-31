# Ticket 0032b: Projected Gauss-Seidel Solver Extension

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Documentation Complete — Awaiting Tutorial
- [x] Merged / Complete

**Current Phase**: Superseded by Ticket 0034
**Assignee**: N/A
**Created**: 2026-01-29
**Generate Tutorial**: No
**Parent Ticket**: [0032_contact_constraint_refactor](0032_contact_constraint_refactor.md)

---

> **SUPERSEDED**: This ticket's scope (PGS solver extension) was implemented and then replaced by Ticket 0034 (Active Set Method). The `solveWithContacts()` public interface, `MultiBodySolveResult`, and `BodyForces` structs described here were implemented as designed, but the internal PGS algorithm was replaced by an Active Set Method that provides exact LCP solutions with finite convergence. All acceptance criteria from this ticket are satisfied by the 0034 implementation. See [0034_active_set_method_contact_solver](0034_active_set_method_contact_solver.md) for current solver details.

## Summary

Extend `ConstraintSolver` with a `solveWithContacts()` method that implements the Projected Gauss-Seidel (PGS) algorithm for unilateral contact constraints. This adds multi-body constraint solving capability while preserving the existing single-body bilateral solver (`solve()`) unchanged.

---

## Motivation

The existing `ConstraintSolver::solve()` uses direct LLT decomposition for bilateral constraints (equality, unrestricted lambda). Contact constraints are unilateral (inequality, lambda >= 0), requiring an iterative solver with clamping. PGS is the industry-standard approach (Bullet, ODE, Box2D).

This ticket isolates the solver algorithm from the integration layer (WorldModel), enabling:
1. Unit testing of PGS convergence with synthetic constraints
2. Independent review of the solving algorithm
3. Clear separation between "how to solve" (this ticket) and "when to solve" (0032c)

---

## Technical Approach

### New Structs

```cpp
struct BodyForces
{
    Coordinate linearForce;    // Net linear constraint force [N]
    Coordinate angularTorque;  // Net angular constraint torque [N*m]
};

struct MultiBodySolveResult
{
    std::vector<BodyForces> bodyForces;  // Per-body constraint forces
    Eigen::VectorXd lambdas;             // Lagrange multipliers
    bool converged{false};               // Solver convergence flag
    int iterations{0};                   // PGS iterations used
    double residual{std::numeric_limits<double>::quiet_NaN()};

    MultiBodySolveResult() = default;
};
```

### New Method: `solveWithContacts()`

```cpp
MultiBodySolveResult solveWithContacts(
    const std::vector<std::vector<Constraint*>>& bilateralConstraints,
    const std::vector<TwoBodyConstraint*>& contactConstraints,
    const std::vector<std::reference_wrapper<const InertialState>>& states,
    const std::vector<Coordinate>& externalForces,
    const std::vector<Coordinate>& externalTorques,
    const std::vector<double>& masses,
    const std::vector<Eigen::Matrix3d>& inverseInertias,
    double dt);
```

### Two-Phase Solving Strategy

**Phase 1 — Bilateral (per-body, existing LLT)**:
Each body's bilateral constraints solved independently via existing `solve()`. Produces per-body bilateral forces added to external forces before Phase 2.

**Phase 2 — Contact (multi-body PGS)**:
1. Assemble effective mass matrix `A = J * M^-1 * J^T` (C x C for C contacts)
2. Compute RHS `b` with Baumgarte bias and restitution terms
3. Iterate PGS: for each contact `i`, compute `lambda_i = max(0, (b_i - sum(A_ij * lambda_j, j != i)) / A_ii)`
4. Check convergence: `|lambda_new - lambda_old| < tolerance`
5. Compute per-body forces: `F = J^T * lambda`

### PGS Parameters

| Parameter | Default | Setter |
|-----------|---------|--------|
| Max iterations | 10 | `setMaxIterations(int)` |
| Convergence tolerance | 1e-4 | `setConvergenceTolerance(double)` |
| Regularization epsilon | 1e-8 | (internal, added to diagonal) |

### Baumgarte Bias Term

Uses velocity-level ERP formulation (per P1 prototype validation):
```
b_i += (ERP / dt) * penetration_depth_i
```
Where ERP defaults to 0.2. The constraint's `alpha()` method returns the ERP value.

### Restitution RHS Term

Uses the constraint-level formulation (per P2 prototype validation):
```
b_i += -(1 + e) * (J_i * q_dot)
```
Where `e` is the coefficient of restitution and `J_i * q_dot` is the pre-impact relative normal velocity. This produces the correct impulse when solved via `A * lambda = b`.

---

## Requirements

### Functional Requirements

1. **FR-1**: `solveWithContacts()` solves mixed bilateral + unilateral constraint systems
2. **FR-2**: PGS enforces `lambda >= 0` for all contact constraints
3. **FR-3**: Effective mass matrix assembly handles multi-body coupling (contacts sharing a body)
4. **FR-4**: Bilateral constraints solved via existing LLT (Phase 1), contacts via PGS (Phase 2)
5. **FR-5**: Per-body force decomposition returned in `MultiBodySolveResult`
6. **FR-6**: PGS iteration count and convergence tolerance configurable

### Non-Functional Requirements

1. **NFR-1**: Existing `solve()` method unchanged — no regressions for single-body constraints
2. **NFR-2**: PGS converges within 10 iterations for typical scenes (< 20 contacts)
3. **NFR-3**: Regularization prevents divide-by-zero for degenerate configurations
4. **NFR-4**: No heap allocation beyond Eigen dynamic matrices

---

## Acceptance Criteria

1. [ ] AC1: Head-on equal-mass collision with e=1.0 produces lambda matching analytical result (within 1e-6)
2. [ ] AC2: PGS converges within max iterations for 1-10 contact systems
3. [ ] AC3: All lambda values >= 0 after PGS solve
4. [ ] AC4: Separating contacts produce lambda = 0 (complementarity)
5. [ ] AC5: Static-dynamic collision works correctly with inverseMass = 0 for static body
6. [ ] AC6: Multiple contacts sharing a body produce coupled solution
7. [ ] AC7: Effective mass matrix A is symmetric positive semi-definite
8. [ ] AC8: Existing bilateral constraint tests pass without modification
9. [ ] AC9: Unit tests cover all solver paths (10 test cases)

---

## Dependencies

- **Ticket 0032a**: Two-Body Constraint Infrastructure (provides `TwoBodyConstraint`, `ContactConstraint`)
- **Ticket 0031**: Generalized Lagrange Constraints (provides `ConstraintSolver`, `Constraint` hierarchy)
- **Blocks**: [0032c](0032c_worldmodel_contact_integration.md)

---

## Files

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Add `MultiBodySolveResult`, `BodyForces`, `solveWithContacts()`, PGS config setters |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Implement PGS solver, effective mass assembly, two-phase solve |

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/Constraints/ConstraintSolverPGSTest.cpp` | PGS solver unit tests |

---

## Test Plan

### Unit Tests (ConstraintSolverPGSTest.cpp)

| Test Case | What It Validates |
|-----------|-------------------|
| Head-on equal mass lambda | Analytical lambda for simple 1D collision |
| PGS convergence count | Converges in <= 10 iterations |
| Lambda non-negativity | All lambdas >= 0 after solve |
| Separating contact lambda zero | Complementarity: lambda = 0 when not in contact |
| Static-dynamic collision | Correct with inverseMass = 0 for one body |
| Multiple contacts coupled | Off-diagonal coupling terms handled |
| Effective mass matrix symmetry | A = A^T |
| Regularization prevents NaN | Degenerate zero-diagonal case |
| Bilateral constraints unchanged | Phase 1 produces same result as standalone solve() |
| PGS config setters | Max iterations and tolerance respected |

---

## References

- **Design document**: `docs/designs/0032_contact_constraint_refactor/design.md` (Section "Constraint Solving Strategy")
- **Math formulation**: `docs/designs/0032_contact_constraint_refactor/math-formulation.md` (Section 7: PGS algorithm)
- **P1 Prototype**: `prototypes/0032_contact_constraint_refactor/p1_pgs_convergence/` (PGS pseudocode in main.cpp)
- **P2 Prototype**: `prototypes/0032_contact_constraint_refactor/p2_energy_conservation/` (restitution RHS validation)

---

## Workflow Log

### Design Phase
- **Completed**: 2026-01-29
- **Notes**: Design shared with parent ticket 0032.

### Prototype Phase
- **Completed**: 2026-01-29
- **Notes**: P1 validated PGS convergence with ERP=0.2. P2 validated restitution formula.

### Implementation Phase
- **Started**: 2026-01-29
- **Completed**: 2026-01-31 (via Ticket 0034)
- **Notes**: The `solveWithContacts()` interface, `MultiBodySolveResult`, `BodyForces`, effective mass assembly, RHS assembly, Jacobian assembly, and force extraction were all implemented as designed. The internal PGS solver was initially implemented and then replaced by an Active Set Method (Ticket 0034) for exact LCP solutions. All 0032b acceptance criteria are satisfied by the final implementation. PGS parameters (`max_iterations`, `convergence_tolerance`) were retained as `max_safety_iterations_` and `convergence_tolerance_` with updated semantics (safety cap and violation check tolerance, respectively). 24 contact solver tests + 12 ASM-specific tests pass (417 total tests).

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
