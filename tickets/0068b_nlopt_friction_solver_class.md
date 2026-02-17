# Ticket 0068b: Implement NLoptFrictionSolver Class

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Review
**Type**: Feature
**Priority**: High
**Created**: 2026-02-16
**Generate Tutorial**: No
**Parent Ticket**: [0068_nlopt_friction_cone_solver](0068_nlopt_friction_cone_solver.md)
**Depends On**: [0068a](0068a_nlopt_conan_dependency.md)

---

## Overview

Implement the `NLoptFrictionSolver` class that wraps NLopt's SLSQP algorithm to solve the friction cone constrained QP. This is a standalone class with no integration into the existing solver pipeline yet (that's 0068c).

The class replaces `FrictionConeSolver` with the same conceptual interface: takes effective mass matrix A, RHS vector b, per-contact friction coefficients mu, and optional warm-start lambda0. Returns optimal impulse vector with convergence diagnostics.

---

## Requirements

### R1: Class Definition

Create `NLoptFrictionSolver.hpp` at `msd/msd-sim/src/Physics/Constraints/`:

- `Algorithm` enum: `SLSQP` (default), `COBYLA`, `MMA`, `AUGLAG_SLSQP`
- `SolveResult` struct: `lambda`, `converged`, `iterations`, `residual`, `objective_value`, `constraint_violations`
- Constructor: default to SLSQP, tolerance 1e-6, max iterations 100
- Configuration: `setTolerance()`, `setMaxIterations()`, `setAlgorithm()`
- Rule of Zero: all special members `= default`

### R2: Solve Method

Create `NLoptFrictionSolver.cpp`:

```
solve(A, b, mu, lambda0) -> SolveResult
```

1. Create `nlopt::opt` with selected algorithm (local, 3C variables)
2. Set objective: `f(lambda) = (1/2) lambda^T A lambda - b^T lambda`, gradient `grad = A*lambda - b`
3. Add C inequality constraints: `c_i = mu_i^2 * n_i^2 - t1_i^2 - t2_i^2 >= 0`
4. Set lower bounds: `lambda_n_i >= 0`, `lambda_t` unbounded
5. Warm-start from lambda0 if valid size, else cold start from zeros
6. Call `opt.optimize()`, map NLopt result code to `converged`
7. Compute per-contact constraint violations for diagnostics

### R3: Error Handling

- `NLOPT_INVALID_ARGS`, `NLOPT_OUT_OF_MEMORY` → throw `std::runtime_error`
- `NLOPT_MAXEVAL_REACHED`, `NLOPT_XTOL_REACHED` → return `converged = false` with best solution
- `NLOPT_SUCCESS`, `NLOPT_FTOL_REACHED`, `NLOPT_XTOL_REACHED` → return `converged = true`
- Invalid mu (< 0) → clamp to 0.0 with spdlog warning

### R4: Static Callbacks

- `objective()`: static function matching NLopt callback signature
- `coneConstraint()`: static function matching NLopt callback signature
- Use `ObjectiveData` and `ConstraintData` structs for callback context

---

## Acceptance Criteria

1. `NLoptFrictionSolver` compiles and links against NLopt
2. `solve()` returns correct results for a hand-computed 1-contact QP
3. All algorithm variants (SLSQP, COBYLA, MMA, AUGLAG) can be selected
4. `SolveResult` diagnostics are populated correctly
5. Thread-safe for concurrent `solve()` calls (NLopt instance local to function)

---

## Files to Create

| File | Purpose |
|------|---------|
| `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.hpp` | Class definition |
| `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.cpp` | Implementation |

---

## Design Reference

Full interface specification in `docs/designs/0068_nlopt_friction_cone_solver/design.md` § "New Components → NLoptFrictionSolver"

---

## Notes

- NLopt C++ API uses `nlopt::opt` class with `std::vector<double>` (not Eigen). Conversion needed in `solve()`.
- Squared cone form `mu^2*n^2 - t1^2 - t2^2 >= 0` avoids sqrt singularity at apex
- SLSQP expects constraints as `c(x) >= 0` via `add_inequality_constraint` with sign convention check (NLopt uses `c(x) <= 0` — may need negation)

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-02-16 22:25 (workflow orchestrator)
- **Completed**: 2026-02-16 22:32
- **Branch**: 0068-nlopt-friction-cone-solver
- **PR**: #71 (draft)
- **Commit**: 1cea9ea
- **Artifacts**:
  - `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.hpp` (156 LOC)
  - `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.cpp` (200 LOC)
  - `msd/msd-sim/test/Physics/Constraints/NLoptFrictionSolverTest.cpp` (329 LOC, 13 tests)
  - `docs/designs/0068_nlopt_friction_cone_solver/implementation-notes-0068b.md`
  - `docs/designs/0068_nlopt_friction_cone_solver/iteration-log.md` (iteration 2)
- **Notes**:
  - Implemented standalone NLoptFrictionSolver class with full interface
  - Algorithm enum (SLSQP/COBYLA/MMA/AUGLAG), SolveResult struct, solve() method
  - Static callbacks for objective and cone constraint (negated for NLopt convention)
  - Warm-start support, invalid mu clamping, dimension validation
  - 13 comprehensive unit tests covering all acceptance criteria
  - Test results: 820/827 passing (13 new tests, 0 regressions)
  - All acceptance criteria met: compiles/links, correct results, algorithm selection, diagnostics, thread-safe
  - Ready for integration in ticket 0068c
