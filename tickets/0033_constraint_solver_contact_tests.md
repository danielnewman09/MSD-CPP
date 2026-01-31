# Ticket 0033: Constraint Solver Contact Integration Tests

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Ready for Merge
**Created**: 2026-01-29
**Generate Tutorial**: No
**Related Tickets**: [0032a_two_body_constraint_infrastructure](0032a_two_body_constraint_infrastructure.md), [0031_generalized_lagrange_constraints](0031_generalized_lagrange_constraints.md), [0034_active_set_method_contact_solver](0034_active_set_method_contact_solver.md)

---

> **IMPLEMENTATION NOTE**: All 24 tests specified in this ticket have been implemented in `ConstraintSolverContactTest.cpp`. The solver backend was initially PGS (per 0032b) and has since been replaced by the Active Set Method (Ticket 0034). All 24 tests pass under the ASM solver without modification (except `MaxIterationsReached_ReportsNotConverged_0033` which was updated by 0034 to use a scenario that requires multiple active set changes instead of a simple resting contact). Additionally, 12 ASM-specific tests were added in `ConstraintSolverASMTest.cpp` by Ticket 0034.

## Summary

Add unit tests that verify `ConstraintSolver::solveWithContacts()` produces physically correct results when given `ContactConstraint` inputs. Ticket 0032a introduced the two-body constraint classes and the `solveWithContacts()` PGS solver interface, but the test suite only validates the constraint math in isolation — no tests feed `ContactConstraint` objects through the solver to verify the end-to-end pipeline.

---

## Motivation

The current test coverage has a gap between two well-tested layers:

1. **Constraint math layer** (33 tests in `ContactConstraintTest.cpp`): Validates `evaluateTwoBody()`, `jacobianTwoBody()`, `isActiveTwoBody()`, Baumgarte parameters, accessors, and input validation.
2. **Single-body solver layer** (4 tests in `ConstraintTest.cpp`): Validates `ConstraintSolver::solve()` with `UnitQuaternionConstraint` and `DistanceConstraint`.

**Missing**: Tests for `ConstraintSolver::solveWithContacts()` — the two-body PGS solver path that will be used for all contact resolution. Without these tests, we cannot verify:
- PGS iteration converges for contact constraints
- Lambda non-negativity clamping works (unilateral: push only, never pull)
- Per-body force extraction produces physically correct impulses
- Solver handles edge cases (static bodies, zero-mass, multiple contacts, degenerate configurations)
- Baumgarte ERP stabilization bias is correctly applied
- Restitution RHS term produces correct bounce velocity

---

## Technical Approach

### Test File

- **Location**: `msd-sim/test/Physics/Constraints/ConstraintSolverContactTest.cpp`
- **Test fixture**: Uses `ContactConstraint` and `ConstraintSolver::solveWithContacts()` together
- **CMake**: Add to `test/Physics/Constraints/CMakeLists.txt`

### Test Categories

#### 1. Basic PGS Convergence

| Test | Description |
|------|-------------|
| `EmptyContactSet_ReturnsConverged` | Zero contacts returns converged with empty forces |
| `SingleContact_Converges` | One penetrating contact produces converged result |
| `MultipleContacts_Converges` | 2-4 simultaneous contacts converge |
| `MaxIterationsReached_ReportsNotConverged` | Set max_iterations=1 on a multi-contact scenario that needs more iterations |

#### 2. Lambda Non-Negativity (Unilateral Enforcement)

| Test | Description |
|------|-------------|
| `SeparatingBodies_LambdaZero` | Bodies moving apart produce lambda=0 (no adhesion) |
| `ApproachingBodies_LambdaPositive` | Bodies approaching produce lambda>0 (repulsive force) |
| `RestingContact_LambdaNonNegative` | Bodies at rest on surface produce lambda>=0 |

#### 3. Per-Body Force Correctness

| Test | Description |
|------|-------------|
| `EqualMass_SymmetricForces` | Two equal-mass bodies receive equal and opposite forces |
| `StaticBody_ZeroForceOnStatic` | Body with inverseMass=0 receives zero velocity change (force applied but infinite mass absorbs it) |
| `ForceDirection_AlongContactNormal` | Constraint force is along the contact normal direction |
| `AngularForces_LeverArmProducesTorque` | Off-center contact produces angular constraint torque |

#### 4. Physical Correctness

| Test | Description |
|------|-------------|
| `HeadOnCollision_EqualMass_VelocityExchange` | Two equal-mass bodies approaching head-on exchange velocities (e=1) |
| `BaumgarteStabilization_ReducesPenetration` | ERP bias term produces force that reduces penetration depth |
| `Restitution_ZeroBounce` | e=0 contact produces zero rebound (bodies stick) |
| `Restitution_FullBounce` | e=1 contact produces full velocity reversal |
| `RestVelocityThreshold_DisablesRestitution` | Slow contact (below 0.5 m/s) disables restitution to prevent jitter |

#### 5. Edge Cases

| Test | Description |
|------|-------------|
| `BothBodiesStatic_AllLambdasZero` | Two infinite-mass bodies: degenerate but should not crash |
| `ParallelContacts_SameNormal` | Multiple contacts with same normal converge correctly |
| `OrthogonalContacts_IndependentResolution` | Contacts on perpendicular faces resolve independently |
| `HighMassRatio_Converges` | Mass ratio of 1000:1 still converges |
| `ZeroPenetration_NoBias` | Contact at surface (penetration=0) produces no Baumgarte bias |

#### 6. Solver Configuration

| Test | Description |
|------|-------------|
| `SetMaxIterations_Respected` | `setMaxIterations()` limits PGS iteration count |
| `SetConvergenceTolerance_EarlyExit` | Tight tolerance requires more iterations; loose tolerance exits early |
| `DefaultConfiguration_ReasonableDefaults` | Default max_iterations=10, tolerance=1e-4 |

---

## Requirements

### Functional Requirements

1. **FR-1**: All tests in "Basic PGS Convergence" pass
2. **FR-2**: All tests in "Lambda Non-Negativity" pass
3. **FR-3**: All tests in "Per-Body Force Correctness" pass
4. **FR-4**: All tests in "Physical Correctness" pass
5. **FR-5**: All tests in "Edge Cases" pass
6. **FR-6**: All tests in "Solver Configuration" pass

### Non-Functional Requirements

1. **NFR-1**: No modifications to production code — test-only ticket
2. **NFR-2**: All existing tests continue to pass
3. **NFR-3**: Tests run in under 5 seconds total (no long simulations)

---

## Acceptance Criteria

1. [x] AC1: Test file `ConstraintSolverContactTest.cpp` exists with all specified test cases
2. [x] AC2: All new tests pass in Debug build
3. [x] AC3: All existing tests continue to pass (no regressions) — 417 total tests pass as of 0034 completion
4. [x] AC4: CMakeLists.txt updated to include new test source
5. [x] AC5: Each test has a descriptive name with `_0033` suffix for ticket traceability
6. [x] AC6: Tests use `ContactConstraint` objects fed through `solveWithContacts()` (not testing constraint math in isolation)

---

## Dependencies

- **Ticket 0032a**: Two-Body Constraint Infrastructure (prerequisite — provides ContactConstraint and solveWithContacts)
- **Ticket 0031**: Generalized Lagrange Multiplier Constraint System (prerequisite — provides ConstraintSolver)

---

## Files

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/Constraints/ConstraintSolverContactTest.cpp` | All tests specified above |

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/test/Physics/Constraints/CMakeLists.txt` | Add `ConstraintSolverContactTest.cpp` to test sources |

---

## References

- **ConstraintSolver interface**: `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` (solveWithContacts method)
- **ContactConstraint**: `msd-sim/src/Physics/Constraints/ContactConstraint.hpp`
- **Existing solver tests**: `msd-sim/test/Physics/Constraints/ConstraintTest.cpp` (single-body solver tests)
- **Existing constraint tests**: `msd-sim/test/Physics/Constraints/ContactConstraintTest.cpp` (constraint math tests)
- **P1 prototype findings**: `prototypes/0032_contact_constraint_refactor/p1_pgs_convergence/Debug_Findings.md`
- **P2 prototype findings**: `prototypes/0032_contact_constraint_refactor/p2_energy_conservation/Debug_Findings.md`
