# Ticket 0052c: Newton Solver Core

## Status
- [x] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Draft
**Assignee**: N/A
**Created**: 2026-02-10
**Generate Tutorial**: No
**Parent Ticket**: [0052_custom_friction_cone_solver](0052_custom_friction_cone_solver.md)

---

## Summary

Implement the Newton iteration loop for the friction contact QP-SOCP. This is the core solver: given the effective mass matrix A, RHS vector b, friction coefficients μ, and an optional warm-start guess, compute the impulse vector λ that minimizes the QP objective subject to friction cone constraints.

---

## Motivation

This is the central component of the custom solver. It ties together:
- The QP objective evaluation and gradient (`Aλ - b`)
- The cone projection operator (0052b)
- The Newton step computation (solve linearized KKT system)
- The line search (ensure sufficient decrease in merit function)
- Convergence detection (primal + dual residuals below tolerance)

---

## Technical Approach

### `FrictionConeSolver` Class

```cpp
struct FrictionConeSolverResult {
    Eigen::VectorXd lambda;      // Solution impulse vector (3C × 1)
    bool converged;              // Whether solver converged
    int iterations;              // Number of Newton iterations used
    double residual;             // Final KKT residual norm
};

class FrictionConeSolver {
public:
    FrictionConeSolverResult solve(
        const Eigen::MatrixXd& A,         // Effective mass matrix (3C × 3C)
        const Eigen::VectorXd& b,         // RHS vector (3C × 1)
        const Eigen::VectorXd& mu,        // Friction coefficients (C × 1)
        const Eigen::VectorXd& lambda0    // Initial guess (warm start)
    ) const;

    // Configuration
    void setMaxIterations(int n);          // Default: 20
    void setTolerance(double eps);         // Default: 1e-8
    void setRegularization(double eps);    // Default: 1e-10
};
```

### Algorithm Outline

1. **Initialize**: `λ = lambda0` (or project onto cones if infeasible)
2. **Iterate** (up to maxIterations):
   a. Compute gradient: `g = Aλ - b`
   b. Evaluate merit function and convergence criterion
   c. If converged (residual < tolerance): return
   d. Compute Newton direction `Δλ` by solving linearized KKT system
   e. Backtracking line search: find step size `α` satisfying Armijo condition
   f. Update: `λ ← λ + α Δλ`
   g. Project onto cone constraints (if using projected Newton variant)
3. **Return**: solution λ, convergence status, iteration count

### Newton System

The specific Newton system structure depends on the math formulation (0052a). Two main approaches:

**Option 1: Projected Newton** — Solve unconstrained QP step, then project onto cones. Simpler, works well when most contacts are sliding (exterior of cone).

**Option 2: Interior-point Newton** — Include barrier terms for cone constraints in the Hessian. More complex but better convergence for mixed stick/slide.

The math formulation will recommend the approach.

### Warm Starting

Accept `lambda0` from previous timestep's solution. ContactCache (Ticket 0040d) provides the contact correspondence. If `lambda0` is infeasible (contacts changed), project onto cones before starting iteration.

---

## Acceptance Criteria

- [ ] **AC1**: Solver converges in ≤ 8 iterations (cold start) for all M8 examples
- [ ] **AC2**: Solver converges in ≤ 3 iterations (warm start) for near-previous solutions
- [ ] **AC3**: KKT residual `‖Aλ - b‖ < 10⁻⁸` at solution for all test cases
- [ ] **AC4**: Cone feasibility `‖λ_t‖ ≤ μλ_n + 10⁻⁸` at solution for all contacts
- [ ] **AC5**: Returns `converged = false` gracefully when max iterations exceeded (no throw)
- [ ] **AC6**: Handles 1-contact through 50-contact problems
- [ ] **AC7**: Handles mass ratios up to 10⁶:1 with regularization fallback
- [ ] **AC8**: Deterministic: same input produces same output (no random seed, no non-deterministic iteration)
- [ ] **AC9**: Performance: single-contact solve < 10 μs, 10-contact solve < 100 μs

---

## Dependencies

- **Requires**: 0052a (math formulation), 0052b (cone projection)
- **Blocks**: 0052d (integration), 0052e (validation)

---

## Files

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/src/Physics/Constraints/FrictionConeSolver.hpp` | Solver class declaration |
| `msd-sim/src/Physics/Constraints/FrictionConeSolver.cpp` | Solver implementation |
| `msd-sim/test/Physics/Constraints/FrictionConeSolverTest.cpp` | Unit tests: all M8 examples, convergence, edge cases |

---

## References

- 0052a math formulation (M2: Newton derivation, M4: convergence, M5: warm starting)
- Todorov (2014) — MuJoCo Newton solver
- Castro et al. (2022) — Drake SAP solver
- Nocedal & Wright (2006), Ch. 12 — Constrained Newton methods

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-10
- **Notes**: Core solver implementation. Design depends on math formulation output (0052a), specifically the Newton system structure and convergence strategy.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
