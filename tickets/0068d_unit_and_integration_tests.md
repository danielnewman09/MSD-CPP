# Ticket 0068d: NLopt Friction Solver Tests

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Ready for Implementation
**Type**: Test
**Priority**: High
**Created**: 2026-02-16
**Generate Tutorial**: No
**Parent Ticket**: [0068_nlopt_friction_cone_solver](0068_nlopt_friction_cone_solver.md)
**Depends On**: [0068c](0068c_constraint_solver_integration.md)

---

## Overview

Comprehensive unit and integration tests for `NLoptFrictionSolver`. These tests serve double duty: they validate the solver implementation AND fulfill the deferred prototype validation criteria (P1: convergence, P3: warm-start).

---

## Requirements

### R1: Unit Tests — NLoptFrictionSolverTest.cpp

Create `msd/msd-sim/test/Physics/Constraints/NLoptFrictionSolverTest.cpp`:

| Test Case | Description | Success Criterion |
|-----------|-------------|-------------------|
| `solve_unconstrained_optimum` | mu=0 (no friction), should match unconstrained QP `lambda = A^{-1} b` | `\|lambda - A^{-1}b\| < 1e-6` |
| `solve_cone_interior` | Small mu=0.1, lambda lands inside cone | `converged == true`, `\|lambda_t\| < mu * lambda_n` |
| `solve_cone_surface` | Large mu=1.0, lambda saturates at cone boundary | `converged == true`, `\|lambda_t\| ≈ mu * lambda_n` (within 1e-4) |
| `solve_warm_start_reduces_iterations` | Same problem twice: cold start vs warm start from solution | `warm_iters < 0.7 * cold_iters` |
| `solve_multiple_contacts` | 2-contact (6 vars) and 4-contact (12 vars) problems | `converged == true` for both |
| `algorithm_selection` | SLSQP vs COBYLA on same problem | Both converge, `\|lambda_slsqp - lambda_cobyla\| < 1e-3` |
| `constraint_violation_diagnostic` | Verify `constraint_violations` field matches hand-computed values | `violations[i] ≈ mu^2*n^2 - t1^2 - t2^2` |
| `zero_rhs_returns_zero` | b=0, should return lambda=0 (trivial) | `lambda.norm() < 1e-10` |
| `negative_mu_clamped` | mu < 0, should clamp to 0 and warn | `converged == true`, lambda_t ≈ 0 |

### R2: Integration Tests — Existing Test Verification

Run and verify these existing test files pass after 0068c integration:

| Test File | Key Cases | What to Verify |
|-----------|-----------|----------------|
| `test/Physics/Collision/LinearCollisionTest.cpp` | F1-F5 | Friction tests unchanged or improved |
| `test/Physics/Collision/RotationalEnergyTest.cpp` | F4 | Tumbling contact — target fix for energy injection |
| `test/Replay/FrictionConeSolverTest.cpp` | Saturation direction | Friction opposes velocity at saturation (ticket 0066) |

### R3: Friction Direction Test

Create or update `msd/msd-sim/test/Physics/Collision/FrictionDirectionTest.cpp`:
- Sliding cube on floor with known velocity direction
- Verify friction impulse opposes tangential velocity
- Verify deceleration matches `mu * g` within tolerance
- Verify energy injection < 0.01 J/frame over 100 frames

---

## Acceptance Criteria

1. All unit tests in R1 pass
2. All existing friction tests (F1-F5, F4) pass at baseline or better
3. FrictionConeSolverTest replay test passes with correct friction direction
4. Energy injection < 0.01 J/frame during sustained contact (P1 criterion)
5. Warm-start iteration reduction > 30% (P3 criterion)

---

## Files to Create

| File | Purpose |
|------|---------|
| `msd/msd-sim/test/Physics/Constraints/NLoptFrictionSolverTest.cpp` | Unit tests |

## Files to Update

| File | Change |
|------|--------|
| `msd/msd-sim/test/Replay/CMakeLists.txt` | Add new test if needed |
| `msd/msd-sim/test/Physics/Collision/FrictionDirectionTest.cpp` | May already exist — verify or create |

---

## Notes

- Unit tests use synthetic A/b/mu matrices — no WorldModel needed
- Integration tests run full physics simulation — depend on 0068c being complete
- Test matrices should include well-conditioned (diagonal-dominant) and mildly ill-conditioned cases
- For warm-start test: solve same problem with lambda0=zeros vs lambda0=previous solution
