# Ticket 0068: NLopt Friction Cone Solver

## Status
- [x] Draft
- [ ] Math Formulation Verified
- [x] Ready for Design
- [x] Design
- [x] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Implementation
- [ ] Verified
- [ ] Merged / Complete

**Current Phase**: Design Approved — Ready for Prototype
**Type**: Feature
**Priority**: High
**Created**: 2026-02-16
**Branch From**: main
**Related Tickets**: [0067](0067_contact_phase_energy_injection.md) (energy injection root cause), [0066](0066_friction_cone_solver_saturation_bug.md) (saturation direction bug), [0052](0052_custom_friction_cone_solver.md) (original Newton solver)

---

## Summary

Replace the custom projected Newton solver (`FrictionConeSolver`) with NLopt for solving the friction cone constrained QP. The current solver fails to converge for cone-surface contacts during sustained tumbling, producing non-converged lambda that injects energy (ticket 0067). A well-tested, general-purpose NLP solver will provide robust convergence without the fragility of maintaining a custom solver.

---

## Motivation

The custom `FrictionConeSolver` (ticket 0052) implements a projected Newton method with:
- Cholesky factorization + regularization
- Reduced-space Newton direction via cone projection Jacobian
- Armijo line search with cone projection
- Spectral Projected Gradient (SPG) fallback

This fails for cone-surface (saturated friction) contacts because:
1. The projection Jacobian `J_proj` is rank-2 for a 3x3 system at the cone boundary
2. The reduced Hessian `H_r = J^T A J` becomes rank-deficient
3. Cholesky fails or produces poor step direction
4. The SPG fallback also stalls on the cone surface
5. Non-converged lambda does net positive work, injecting 0.03-0.07 J/frame

Increasing iterations from 50 to 200 was tested and ruled out (ticket 0067) -- the solver plateaus, not iteration-starved.

---

## Mathematical Formulation

### Current QP Problem

The friction cone QP solved each frame:

```
minimize    f(lambda) = (1/2) lambda^T A lambda - b^T lambda
subject to  ||lambda_t_i|| <= mu_i * lambda_n_i    for each contact i
            lambda_n_i >= 0                          for each contact i
```

Where:
- `lambda` is a `3C x 1` vector (C contacts, each with [lambda_n, lambda_t1, lambda_t2])
- `A = J M^{-1} J^T` is the `3C x 3C` effective mass matrix (SPD)
- `b` encodes restitution and velocity terms
- `mu_i` is the friction coefficient for contact i
- The cone constraint `||lambda_t|| <= mu * lambda_n` is a second-order cone (Lorentz cone)

### NLopt Formulation

**Objective**: Same quadratic objective, provided as `f(lambda)` with gradient `g = A*lambda - b`.

**Constraints**: The cone constraint can be expressed as a nonlinear inequality:

```
c_i(lambda) = mu_i^2 * lambda_n_i^2 - lambda_t1_i^2 - lambda_t2_i^2 >= 0
```

with gradient:
```
dc/d(lambda_n)  = 2 * mu_i^2 * lambda_n_i
dc/d(lambda_t1) = -2 * lambda_t1_i
dc/d(lambda_t2) = -2 * lambda_t2_i
```

**Bounds**: `lambda_n_i >= 0` (lower bound on every 3rd variable).

### Questions to Verify

1. **Algorithm choice**: SLSQP vs COBYLA vs MMA vs AUGLAG? SLSQP is gradient-based and handles quadratic objectives with nonlinear constraints efficiently. Likely the right choice for this small dense QP.

2. **Problem size**: Typically 3-12 variables (1-4 contacts). NLopt overhead acceptable at this scale? The solver runs every frame during contact.

3. **Warm starting**: NLopt SLSQP accepts an initial point. Can we warm-start from the previous frame's lambda (via ContactCache)?

4. **Convergence guarantees**: Does SLSQP reliably converge for the cone-surface case where our Newton solver fails? The cone constraint is smooth everywhere except at the apex (lambda_n = 0, lambda_t = 0).

5. **Constraint formulation**: Is the squared form `mu^2 * n^2 - t1^2 - t2^2 >= 0` numerically better than `sqrt(t1^2 + t2^2) - mu * n <= 0`? The squared form avoids the sqrt singularity at the apex.

---

## Implementation Plan (Preliminary)

### Phase 1: Add NLopt Dependency
- Add `nlopt` to `conanfile.py`
- Verify builds on current toolchain

### Phase 2: Implement NLopt Solver
- Create `NLoptFrictionSolver` class with same interface as `FrictionConeSolver`
- Implement objective + gradient callback
- Implement cone inequality constraint + gradient callback
- Wire warm-start from lambda0

### Phase 3: Integration
- Replace `FrictionConeSolver` usage in `ConstraintSolver::solveWithFriction()`
- Remove old `FrictionConeSolver` class and related `ConeProjection` utilities

### Phase 4: Verification
- All existing physics tests pass (734/741 baseline)
- F4 test: energy injection during tumbling contact eliminated or < 0.01 J/frame
- FrictionConeSolverTest replay: correct friction direction at saturation
- Performance: solver time per frame within 2x of current (acceptable for correctness)

---

## Acceptance Criteria

1. NLopt SLSQP (or chosen algorithm) reliably converges for cone-surface contacts
2. Energy injection during sustained contact < 0.01 J/frame (ticket 0067 acceptance)
3. Friction direction correct at saturation (ticket 0066 acceptance)
4. No regression in existing physics tests
5. ECOS dependency can be removed after this lands (separate cleanup ticket)

---

## Files of Interest

| File | Relevance |
|------|-----------|
| `msd-sim/src/Physics/Constraints/FrictionConeSolver.hpp` | Current solver to replace |
| `msd-sim/src/Physics/Constraints/FrictionConeSolver.cpp` | Current solver implementation |
| `msd-sim/src/Physics/Constraints/ConeProjection.hpp` | Cone projection utilities |
| `msd-sim/src/Physics/Constraints/ConeProjection.cpp` | Cone projection implementation |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Call site for friction solver |
| `conanfile.py` | Dependency management |

---

## Notes

- NLopt is MIT-licensed, well-maintained, available via Conan
- Problem sizes are small (3-12 vars) so general-purpose solver overhead should be acceptable
- This replaces the custom solver entirely -- not a fallback/hybrid approach
- The ECOS SOCP path (ticket 0035b4) is also a candidate for removal once NLopt proves reliable

---

## Workflow Log

### Draft → Ready for Design
- **Started**: 2026-02-16 (workflow orchestrator)
- **Completed**: 2026-02-16
- **Branch**: N/A
- **PR**: N/A
- **Notes**: Ticket created with complete mathematical formulation. No separate math design phase required since the QP formulation, NLopt constraint mapping, and gradients are already documented. Math design flag not set. Advancing directly to architectural design phase.

### Design Phase
- **Started**: 2026-02-16 (workflow orchestrator → cpp-architect agent)
- **Completed**: 2026-02-16
- **Branch**: 0068-nlopt-friction-cone-solver
- **PR**: #71 (draft)
- **Issue**: #70
- **Artifacts**:
  - `docs/designs/0068_nlopt_friction_cone_solver/design.md`
  - `docs/designs/0068_nlopt_friction_cone_solver/0068_nlopt_friction_cone_solver.puml`
- **Notes**: Architectural design complete. New component `NLoptFrictionSolver` replaces `FrictionConeSolver` and `ConeProjection`. Modified `ConstraintSolver` to integrate NLopt. SLSQP algorithm recommended as default. Design includes comprehensive unit tests, integration tests, and benchmarks. Open questions documented for human review: algorithm selection, tolerance tuning, performance threshold. Ready for design review phase.

### Design Review Phase
- **Started**: 2026-02-16 (workflow orchestrator → design-reviewer agent)
- **Completed**: 2026-02-16
- **Branch**: 0068-nlopt-friction-cone-solver
- **PR**: #71 (draft)
- **Artifacts**:
  - Design review appended to `docs/designs/0068_nlopt_friction_cone_solver/design.md`
  - PR comment summary posted to #71
- **Notes**: Design approved with notes. All architectural, quality, feasibility, and testability criteria passed. Human-resolved open questions (SLSQP, 1e-6 tolerance, 2x slowdown acceptable, 100 max iterations) remove design-phase blockers. Three prototypes required before implementation: P1 (SLSQP convergence validation, 3 hours), P2 (performance benchmark, 2 hours), P3 (warm-start effectiveness, 2 hours). All prototypes include concrete success criteria and fallback strategies. Design follows project C++ standards (Rule of Zero, brace initialization, value semantics, NaN for uninitialized floats). Clean drop-in replacement for `FrictionConeSolver` with improved diagnostics. Ready for prototype phase.
