# Ticket 0052b: Cone Projection and Linear Algebra Primitives

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

Implement the second-order cone projection operator and any block-sparse linear algebra primitives needed by the Newton solver. The cone projection is the most mathematically precise component — it must handle all geometric cases correctly, including the degenerate cone tip.

---

## Motivation

The Newton solver requires two core building blocks:
1. **Cone projection**: Project an arbitrary 3D vector `(λ_n, λ_t1, λ_t2)` onto the scaled friction cone `‖λ_t‖ ≤ μ λ_n`. This is used for feasibility restoration (warm starting), merit function evaluation, and constraint satisfaction checks.
2. **Projection gradient**: The Jacobian of the projection operator, needed for assembling the Newton Hessian when using a projected Newton approach.

These are self-contained, heavily testable components that can be built and validated before the solver itself.

---

## Technical Approach

### Cone Projection Operator

Implement `ConeProjection::project(λ_n, λ_t1, λ_t2, μ) → (λ_n', λ_t1', λ_t2')`:

**Three cases** (from M3 of math formulation):
1. **Interior**: `‖λ_t‖ ≤ μ λ_n` — already feasible, return input unchanged
2. **Below cone** (dual cone exterior): `‖λ_t‖ ≤ -λ_n / μ` — project to origin `(0, 0, 0)`
3. **Exterior**: otherwise — project to nearest point on cone surface

The exterior case projection formula (for scaled cone `‖λ_t‖ ≤ μ λ_n`):
```
λ_n' = (μ ‖λ_t‖ + λ_n) / (μ² + 1)
λ_t' = μ λ_n' · (λ_t / ‖λ_t‖)
```

### Projection Gradient

Implement `ConeProjection::gradient(λ_n, λ_t1, λ_t2, μ) → 3×3 Jacobian matrix`:
- Interior: identity matrix
- Below cone: zero matrix
- Exterior: rank-2 matrix (derivative of the projection formula)

### Linear Algebra Primitives (if needed)

Depending on the Newton solver design (0052c), may need:
- Block-3×3 diagonal matrix operations
- Sparse Cholesky for the 3C×3C Newton system
- Or: leverage Eigen's existing sparse/dense solvers

This will be scoped more precisely after the math formulation (0052a) and design phases.

---

## Acceptance Criteria

- [ ] **AC1**: Cone projection returns correct result for all three geometric cases
- [ ] **AC2**: Projection is continuous (no discontinuities at case boundaries)
- [ ] **AC3**: Projection is idempotent: `project(project(x)) == project(x)`
- [ ] **AC4**: Projection gradient correct for all cases (verified by finite-difference)
- [ ] **AC5**: Handles degenerate inputs: `λ_t = (0,0)`, `λ_n = 0`, `μ = 0`
- [ ] **AC6**: Handles μ = 0 (frictionless): projection reduces to `max(λ_n, 0)` with `λ_t = 0`
- [ ] **AC7**: Handles μ → ∞ (infinite friction): projection reduces to `λ_n ≥ 0` with `λ_t` unconstrained
- [ ] **AC8**: Unit tests cover all cases with exact expected values from M3 numerical examples

---

## Dependencies

- **Requires**: 0052a (math formulation — cone projection derivation)
- **Blocks**: 0052c (Newton solver core)

---

## Files

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/src/Physics/Constraints/ConeProjection.hpp` | Cone projection operator (header-only or hpp+cpp) |
| `msd-sim/test/Physics/Constraints/ConeProjectionTest.cpp` | Comprehensive unit tests |

---

## References

- 0052a math formulation, Section M3
- Boyd & Vandenberghe (2004), Section 8.1.1 — Projection onto second-order cone

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-10
- **Notes**: Self-contained building block with clear mathematical specification. Ideal for early implementation and thorough testing before the solver is assembled.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
