# Ticket 0052: Custom Friction Cone Solver

## Status
- [x] Draft
- [ ] Ready for Math Formulation
- [ ] Math Formulation Complete — Awaiting Review
- [ ] Math Review Approved — Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Draft
**Assignee**: N/A
**Created**: 2026-02-10
**Generate Tutorial**: Yes
**Parent Ticket**: [0035_friction_constraints](0035_friction_constraints.md)
**Related Tickets**: [0035b_box_constrained_asm_solver](0035b_box_constrained_asm_solver.md), [0035d_friction_hardening_and_validation](0035d_friction_hardening_and_validation.md)

### Subtasks

| Subtask | Description | Dependencies |
|---------|-------------|--------------|
| [0052a](0052a_solver_math_formulation.md) | Newton solver mathematical formulation: QP objective, cone projection, KKT system, convergence proof | None |
| [0052b](0052b_cone_projection_and_linear_algebra.md) | Cone projection operator + block-sparse linear algebra primitives | 0052a |
| [0052c](0052c_newton_solver_core.md) | Newton solver core: iterate, line search, convergence | 0052a, 0052b |
| [0052d](0052d_solver_integration.md) | Integration with ConstraintSolver dispatch (replace ECOS path) | 0052c |
| [0052e](0052e_solver_validation_suite.md) | Validation suite: known-solution tests, energy monotonicity, convergence diagnostics | 0052c, 0052d |

---

## Summary

Replace the ECOS-based SOCP solver with a custom Newton-based convex solver for friction contact constraints. The ECOS formulation was built with the wrong objective — it treated the friction contact problem as a feasibility problem (`min 0ᵀλ` s.t. `Aλ = b` with cone constraints), when the correct formulation is a Quadratic Program with second-order cone constraints:

```
min  (1/2) λᵀ A λ - bᵀ λ
s.t. ‖λ_tᵢ‖ ≤ μᵢ λ_nᵢ   (Coulomb friction cones)
     λ_nᵢ ≥ 0              (normal force non-negativity)
```

The equality `Aλ = b` is NOT an input constraint — it emerges as the KKT optimality condition at the solution. This fundamental error means the ECOS path cannot produce physically correct results regardless of implementation quality.

The custom solver follows the approach proven in MuJoCo (Todorov 2014) and Drake SAP (Castro et al. 2022): a Newton method for the convex QP with exact second-order cone constraints, yielding quadratic convergence in 3-8 iterations with no external dependencies.

---

## Motivation

### Why Not Fix ECOS?

Three independent reasons make replacing ECOS the right choice over fixing it:

1. **Wrong problem class**: ECOS solves LP/SOCP, not QP+SOCP. Encoding the quadratic objective requires error-prone epigraph lifting that inflates the problem dimension and obscures the physics. The current formulation bug is a direct consequence of this mismatch.

2. **GPLv3 license**: ECOS is copyleft-licensed. Removing it eliminates a licensing concern for the project.

3. **No warm starting**: ECOS uses interior-point methods that always start from the analytic center. A custom Newton solver can warm-start from the previous timestep's impulses, which is critical for simulation stepping where contact configurations change slowly between frames.

### Why a Custom Solver (Not Clarabel/OSQP)?

The [solver research](../docs/designs/0035d1_ecos_socp_reformulation/solver-research.md) evaluated 9 solver options. A custom Newton solver ranked #1 because:

- **No external dependency** — pure C++20, ~400-600 LOC
- **Exact Coulomb cones** — no polyhedral approximation (OSQP has 5-29% error)
- **Optimal for problem scale** — 1-50 contacts (3-150 variables) is well below where general-purpose solvers gain advantage
- **Warm starting** — Newton converges faster from nearby initial point
- **Production-proven** — MuJoCo and Drake use this approach in production

### What This Replaces

- The ECOS Conan dependency and all `msd-sim/src/Physics/Constraints/ECOS/` code
- The box-constrained ASM path for friction (0035b) — the custom solver handles friction natively with exact cones, making box approximation unnecessary

---

## Mathematical Analysis Requirements

This ticket requires rigorous mathematical formulation before design. The math formulation must establish:

### M1. QP-SOCP Problem Formulation

Derive the complete problem statement from first principles:
- Start from the velocity-level constraint equation `Aλ = b` and Coulomb friction model
- Show why `Aλ = b` must be the objective's KKT condition (not an equality constraint)
- Derive the QP objective `(1/2)λᵀAλ - bᵀλ` from the minimum complementary energy principle
- State the complete problem with second-order cone constraints and normal force bounds
- Prove convexity of the full problem (A is positive semidefinite)

### M2. Newton Method Derivation

Derive the Newton step for the QP with cone constraints:
- KKT conditions for the QP-SOCP
- Newton system: linearize KKT conditions to get the Newton step `Δλ`
- Block structure: show how the 3×3 per-contact structure enables efficient factorization
- Hessian of the Lagrangian (A + barrier/penalty terms for cone constraints)
- Gradient computation at each iteration

### M3. Second-Order Cone Projection

Derive the projection operator onto the second-order cone `K = {(t, x) : ‖x‖ ≤ t}`:
- Closed-form projection for 3D cone: `Proj_K(λ_n, λ_t1, λ_t2)`
- Three cases: interior (no projection needed), exterior-above (project to cone surface), exterior-below (project to origin)
- Gradient of the projection operator (needed for Newton Hessian)
- Scaled cone projection: `‖λ_t‖ ≤ μ λ_n` (scale factor μ)

### M4. Line Search and Convergence

Derive convergence properties:
- Merit function for the QP-SOCP (e.g., augmented Lagrangian or primal-dual gap)
- Backtracking line search with Armijo condition
- Prove quadratic convergence rate near the solution
- Convergence criterion: primal residual + dual residual + complementary gap < ε
- Worst-case iteration bound (3-8 iterations typical, prove finite termination)

### M5. Warm Starting Strategy

Formulate warm starting from previous timestep:
- Initial guess: `λ⁰ = λ_prev` (previous frame's impulses)
- Feasibility restoration: project `λ⁰` onto cone constraints if needed
- Expected iteration reduction: 1-3 iterations vs 3-8 from cold start
- Contact correspondence: map previous contacts to current contacts by proximity

### M6. Regularization and Degenerate Cases

Analyze numerical robustness:
- Near-singular A matrix: diagonal regularization `A + εI` with `ε = 10⁻¹⁰`
- Zero normal force: `λ_n = 0` implies `λ_t = 0` (tip of cone)
- Grazing contact: `λ_n → 0⁺` with finite tangential velocity
- High mass ratios (up to 10⁶:1): conditioning analysis
- Single-contact vs multi-contact: verify solver handles both

### M7. Energy Monotonicity Proof

Prove that the solver output satisfies energy dissipation:
- Friction power `P_f = λ_t · v_t ≤ 0` (friction opposes sliding)
- The QP objective ensures minimum energy solution
- No energy injection from numerical artifacts (contrast with the broken ECOS formulation)

### M8. Numerical Examples with GTest Mappings

Provide concrete worked examples for solver validation:

1. **Single contact, pure normal**: 1 contact, μ = 0, verify solver reduces to normal-only case (`λ_t = 0`)
2. **Single contact, sticking**: 1 contact, μ = 0.5, tangential velocity within cone, verify `Aλ = b` exactly
3. **Single contact, sliding**: 1 contact, μ = 0.3, tangential velocity exceeds cone, verify `‖λ_t‖ = μλ_n` and sliding direction correct
4. **Two contacts, different μ**: Contacts with μ₁ = 0.8, μ₂ = 0.2, verify independent cone constraints
5. **Warm start convergence**: Same problem solved cold (λ⁰ = 0) vs warm (λ⁰ = λ_prev), verify iteration reduction
6. **Degenerate: grazing contact**: λ_n → 0, verify graceful handling (no division by zero, no NaN)
7. **Known analytical solution**: Inclined plane at exactly friction angle (arctan(μ)), verify stick/slip boundary

---

## Technical Approach (High-Level)

The detailed technical approach will be determined during the Design phase. Expected architecture:

### Solver Class: `FrictionConeSolver`

A self-contained Newton solver for the friction contact QP-SOCP:

```
Input:  A (effective mass matrix, 3C×3C)
        b (RHS vector, 3C×1)
        mu (friction coefficients, C×1)
        lambda_init (warm start, optional, 3C×1)

Output: lambda (impulse vector, 3C×1)
        converged (bool)
        iterations (int)
```

### Integration Point

Replaces the ECOS dispatch path in `ConstraintSolver`. When contacts have nonzero friction coefficients, the solver dispatches to `FrictionConeSolver` instead of the ASM normal-only path.

### Expected Components

1. **`ConeProjection`** — Second-order cone projection operator with gradient
2. **`FrictionConeSolver`** — Newton iteration loop with line search
3. **`FrictionConeKKT`** — KKT system assembly and factorization (or merged into solver)

### Estimated Size

~400-600 lines of C++ (solver core), plus ~800-1200 lines of tests.

---

## Acceptance Criteria

### Mathematical Formulation Phase

- [ ] **MF1**: QP-SOCP problem derived from first principles with convexity proof
- [ ] **MF2**: Newton step derived with block structure analysis
- [ ] **MF3**: Cone projection operator derived with closed-form expressions for all cases
- [ ] **MF4**: Convergence proof with iteration bound
- [ ] **MF5**: Warm starting strategy formulated with feasibility restoration
- [ ] **MF6**: Regularization strategy for all degenerate cases
- [ ] **MF7**: Energy monotonicity proof
- [ ] **MF8**: All 7 numerical examples worked through with hand-verified results

### Implementation Phase

- [ ] **AC1**: Cone projection passes unit tests for all three cases (interior, exterior-above, exterior-below)
- [ ] **AC2**: Solver converges in ≤ 8 iterations for all M8 examples (cold start)
- [ ] **AC3**: Solver converges in ≤ 3 iterations for warm-started cases
- [ ] **AC4**: Solution satisfies `‖Aλ - b‖ < 10⁻⁸` (KKT residual) for all test cases
- [ ] **AC5**: Solution satisfies `‖λ_t‖ ≤ μλ_n + 10⁻⁸` (cone feasibility) for all contacts
- [ ] **AC6**: Energy is monotonically non-increasing for friction-only scenarios (no external forces)
- [ ] **AC7**: Solver handles mass ratios up to 10⁶:1 without failure (may use regularization)
- [ ] **AC8**: All existing normal-contact tests pass (zero regressions)
- [ ] **AC9**: Performance: friction solve < 2× wall time of normal-only ASM for equivalent contact count
- [ ] **AC10**: ECOS dependency fully removed from build system

---

## Dependencies

### Required (Must Exist)

| Dependency | Status | Notes |
|------------|--------|-------|
| Ticket 0035a (FrictionConstraint + tangent basis) | Complete | Provides FrictionConstraint class and tangent basis computation |
| Ticket 0034 (Active Set Method solver) | Complete | Provides ConstraintSolver dispatch architecture |
| Ticket 0032 (Contact Constraint Refactor) | Complete | Provides ContactConstraint, effective mass matrix assembly |

### Supersedes

| Ticket | Notes |
|--------|-------|
| 0035b (Box-constrained ASM solver) | Custom solver handles friction natively — box approximation no longer needed |
| 0035e (ECOS SOCP math derivation) | ECOS path being removed entirely |

---

## Files

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/src/Physics/Constraints/ConeProjection.hpp` | Second-order cone projection operator |
| `msd-sim/src/Physics/Constraints/FrictionConeSolver.hpp` | Newton solver for QP-SOCP |
| `msd-sim/src/Physics/Constraints/FrictionConeSolver.cpp` | Solver implementation |
| `msd-sim/test/Physics/Constraints/ConeProjectionTest.cpp` | Cone projection unit tests |
| `msd-sim/test/Physics/Constraints/FrictionConeSolverTest.cpp` | Solver unit tests (all M8 examples) |
| `msd-sim/test/Physics/Constraints/FrictionConeSolverConvergenceTest.cpp` | Convergence diagnostics and warm start tests |

### Modified Files

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Dispatch to FrictionConeSolver for contacts with friction |
| `msd-sim/CMakeLists.txt` | Add new source/test files |
| `conanfile.py` | Remove ECOS dependency |

### Removed Files

| File | Reason |
|------|--------|
| `msd-sim/src/Physics/Constraints/ECOS/*` | ECOS wrapper code replaced by custom solver |

---

## Risks and Mitigations

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Newton method fails to converge for edge cases | High | Low | Fallback to PGS with cone projection (Option C from M5); regularization |
| Implementation exceeds 600 LOC estimate | Low | Medium | Core algorithm is well-defined; extra LOC would be in tests (acceptable) |
| Warm starting requires contact tracking across frames | Medium | Low | ContactCache (Ticket 0040d) already provides this infrastructure |
| Hessian factorization unstable for high mass ratios | Medium | Low | Diagonal regularization + iterative refinement |
| Removing ECOS breaks existing friction test paths | Low | Medium | ECOS path is not on main branch; friction-integration branch is the integration point |

---

## References

### Academic

- Todorov, E. (2014). "Convex and analytically-invertible dynamics with contacts and constraints." ICRA 2014.
- Castro, A. et al. (2022). "An unconstrained convex formulation of compliant contact." IEEE T-RO. [arXiv:2110.10107](https://arxiv.org/abs/2110.10107)
- Boyd, S. & Vandenberghe, L. (2004). *Convex Optimization*, Chapters 4 (SOCP) and 11 (Interior-point methods).
- Nocedal, J. & Wright, S. (2006). *Numerical Optimization*, Chapter 12 (Constrained optimization).

### Project Documents

- [Solver Research](../docs/designs/0035d1_ecos_socp_reformulation/solver-research.md) — Comprehensive evaluation of 9 solver options
- [M3: Coulomb Cone](../docs/designs/0035_friction_constraints/M3-coulomb-cone.md) — Friction cone formulation
- [M5: Solver Extension](../docs/designs/0035_friction_constraints/M5-solver-extension.md) — Solver approach analysis
- [0035b: Box-Constrained ASM](0035b_box_constrained_asm_solver.md) — Previous solver approach (superseded)

### Production Implementations

- [MuJoCo computation docs](https://mujoco.readthedocs.io/en/stable/computation/index.html) — Newton solver for contact
- [Drake SAP solver](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html) — Semi-Analytical Primal method

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-10
- **Notes**: Created based on findings from 0035d1 solver research and the identification that the ECOS formulation was built with the wrong objective (feasibility problem instead of QP). The custom Newton solver is the #1 recommendation from the solver research, proven in MuJoCo and Drake. This ticket supersedes the ECOS path (0035b) and replaces it with a dependency-free, exact Coulomb cone solver.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
