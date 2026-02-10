# Ticket 0052a: Friction Cone Solver — Mathematical Formulation

## Status
- [x] Draft
- [x] Ready for Math Formulation
- [x] Math Formulation Complete — Awaiting Review
- [x] Math Review Approved — Ready for Design
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

Produce the rigorous mathematical formulation for the custom Newton-based friction cone solver. This covers the QP-SOCP problem derivation, Newton step computation, cone projection operator, convergence analysis, warm starting, regularization, and concrete numerical examples. This document is the mathematical foundation that all implementation subtasks build on.

---

## Motivation

The solver research (0035d1) identified the correct problem formulation but did not derive the solution algorithm in detail. Before implementing the solver, we need:
- Complete Newton step derivation with block structure
- Closed-form cone projection with gradient
- Convergence proof with iteration bounds
- Worked numerical examples that become GTest cases

---

## Technical Approach

Produce a single document at `docs/designs/0052_custom_friction_cone_solver/math-formulation.md` covering all M1-M8 requirements from the parent ticket:

### M1. QP-SOCP Problem Formulation
- Derive from minimum complementary energy principle
- Show why `Aλ = b` is the KKT condition, not a constraint
- Prove convexity (A positive semidefinite from `J M⁻¹ Jᵀ` structure)

### M2. Newton Method Derivation
- KKT system for QP with SOC constraints
- Newton linearization and step computation
- 3×3 block structure per contact for efficient factorization
- Hessian assembly (A + barrier/cone terms)

### M3. Second-Order Cone Projection
- Closed-form `Proj_K(λ_n, λ_t1, λ_t2)` for `‖λ_t‖ ≤ μ λ_n`
- Three geometric cases with explicit formulas
- Gradient of projection (Jacobian matrix)

### M4. Line Search and Convergence
- Merit function selection (augmented Lagrangian or gap)
- Backtracking Armijo line search
- Quadratic convergence proof near solution
- Convergence criterion: primal + dual residual + gap < ε

### M5. Warm Starting
- Feasibility restoration via cone projection
- Contact correspondence between frames
- Expected iteration reduction analysis

### M6. Regularization and Degenerate Cases
- Diagonal regularization for near-singular A
- Zero normal force, grazing contact, high mass ratios
- Numerical stability bounds

### M7. Energy Monotonicity
- Prove friction power ≤ 0 from QP structure
- No energy injection from solver artifacts

### M8. Numerical Examples (7 cases)
All examples from parent ticket, worked through with hand-computed solutions and GTest mappings.

---

## Acceptance Criteria

- [ ] **AC1**: QP-SOCP problem derived from first principles with convexity proof
- [ ] **AC2**: Newton step derived with explicit block structure (3×3 per contact)
- [ ] **AC3**: Cone projection closed-form for all three geometric cases
- [ ] **AC4**: Convergence proof with quadratic rate and iteration bound
- [ ] **AC5**: Warm starting strategy with feasibility restoration
- [ ] **AC6**: Regularization for all degenerate cases enumerated
- [ ] **AC7**: Energy monotonicity proof
- [ ] **AC8**: All 7 numerical examples hand-verified with explicit intermediate values
- [ ] **AC9**: Document is self-contained (reviewer can follow without consulting external sources)

---

## Dependencies

- **Requires**: None (foundational math document)
- **Blocks**: 0052b (cone projection), 0052c (Newton solver core)

---

## Files

### New Files

| File | Purpose |
|------|---------|
| `docs/designs/0052_custom_friction_cone_solver/math-formulation.md` | Complete mathematical formulation |

---

## References

- Todorov (2014), Castro et al. (2022) — Newton solver for contact QP
- Boyd & Vandenberghe (2004) — Convex optimization, SOCP, interior-point methods
- [Solver Research](../docs/designs/0035d1_ecos_socp_reformulation/solver-research.md)
- [M3: Coulomb Cone](../docs/designs/0035_friction_constraints/M3-coulomb-cone.md)
- [M5: Solver Extension](../docs/designs/0035_friction_constraints/M5-solver-extension.md)

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-10
- **Notes**: Mathematical foundation for the custom friction cone solver. Must be completed and reviewed before any implementation begins.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
