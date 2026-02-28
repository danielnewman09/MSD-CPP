# Ticket 0082a: BlockPGS Solver Unit Tests

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Implementation Complete
**Type**: Testing
**Priority**: High
**Created**: 2026-02-26
**Generate Tutorial**: No
**Parent Ticket**: [0082](0082_collision_test_restructure.md)
**Blocked By**: None
**Related Tickets**: [0075b](0075b_block_pgs_solver.md) (Block PGS implementation)

---

## Summary

Add dedicated unit tests for `BlockPGSSolver`, the two-phase friction contact solver introduced in 0075b. Currently this solver has zero unit tests — all coverage is indirect through integration tests exercising the full collision pipeline. This is the highest priority sub-ticket because the BlockPGSSolver is the active solver for all friction contacts.

---

## Problem

`BlockPGSSolver` (src/Physics/Constraints/BlockPGSSolver.hpp/cpp) implements:
- `buildBlockK()` — 3x3 effective mass matrix per contact
- `projectCoulombCone()` — Coulomb cone projection with ball-projection fallback
- `applyRestitutionPreSolve()` — Phase A: velocity-level restitution
- `sweepOnce()` — Phase B: block Gauss-Seidel sweep with cone projection
- `solve()` — orchestrates warm-start, Phase A, iterative Phase B sweeps

None of these methods have unit tests. Bugs in cone projection, effective mass construction, or sweep convergence are only caught when they cause visible integration test failures, which is how the current sliding bugs are manifesting.

---

## Scope

### Test File
`msd/msd-sim/test/Physics/Constraints/BlockPGSSolverTest.cpp`

### Required Tests

#### buildBlockK — Effective Mass Matrix
1. **SingleContact_IdentityInertia_DiagonalK** — With identity inertia and contact at COM, K should be diagonal with entries = 2/m (two-body contribution)
2. **SingleContact_OffCenter_HasOffDiagonalTerms** — Contact offset from COM produces cross-coupling terms K_nt, K_tn
3. **SingleContact_InfiniteMassBody_HalfContribution** — When one body is static (infinite mass), only one body contributes to K

#### projectCoulombCone — Cone Projection
4. **InsideCone_NoChange** — Lambda already inside cone (|t| < mu*n) is unchanged
5. **OutsideCone_ProjectedToSurface** — Lambda outside cone projects to nearest point on cone surface: tangent scaled to mu*n
6. **NegativeNormal_ClampedToZero** — Negative lambda_n clamps entire impulse to zero
7. **ZeroFriction_TangentZeroed** — With mu=0, tangent components are zeroed regardless of input
8. **OnConeSurface_Unchanged** — Lambda exactly on cone boundary is unchanged (within tolerance)

#### applyRestitutionPreSolve — Phase A
9. **ApproachingContact_PositiveRestitutionBias** — Approaching velocity with e>0 produces positive velocity target
10. **SeparatingContact_NoBias** — Separating velocity produces zero restitution bias
11. **RestVelocity_RestitutionSuppressed** — Below rest velocity threshold, restitution is zero regardless of e

#### sweepOnce — Block GS Sweep
12. **SingleContact_ConvergesInOneSweep** — Single contact with well-conditioned K converges in one sweep
13. **TwoContacts_SamePair_Converges** — Two contacts on same body pair (4-point manifold) converge

#### Full solve()
14. **SingleSlidingContact_FrictionSaturatesAtCone** — Horizontal velocity with mu>0 produces friction at cone boundary
15. **SingleRestingContact_GravitySupported** — Vertical gravity load produces lambda_n = mg/N_contacts, lambda_t ≈ 0
16. **WarmStart_ReducesIterations** — Pre-loaded lambda from previous frame reduces iteration count
17. **ZeroVelocity_ZeroImpulse** — Quiescent contact with no penetration produces zero impulse
18. **HighMassRatio_Converges** — 1000:1 mass ratio converges without explosion

### Test Approach

Tests should construct `BlockPGSSolver` directly using `BlockPGSSolver::ContactData` structs, bypassing the full `ConstraintSolver` dispatch path. This isolates the solver logic from constraint construction and island decomposition.

```cpp
// Example test pattern
BlockPGSSolver solver;
std::vector<BlockPGSSolver::ContactData> contacts;
// ... populate contacts with known physics ...
auto result = solver.solve(contacts, dt);
// ... assert on result.lambdas, result.converged, etc. ...
```

---

## Acceptance Criteria

1. >= 15 unit tests covering all public methods of BlockPGSSolver
2. Each test has a descriptive name (no ticket suffixes)
3. Tests pass with current solver implementation
4. At least one test validates cone projection numerically against hand-computed values
5. At least one test validates effective mass matrix against hand-computed values

---

## Notes

- The 0075b ticket explicitly deferred "Additional unit tests (BlockPGSSolver-specific, CoulombConeProjection, etc.)" to follow-on work — this ticket fulfills that
- ProjectedGaussSeidelTest.cpp is a good reference for test structure/patterns
- The existing ConstraintSolverContactTest only tests the normal-only ASM path, not the Block PGS friction path

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-02-26 00:00
- **Completed**: 2026-02-26 00:00
- **Branch**: 0075b-block-pgs-solver
- **PR**: N/A
- **Artifacts**:
  - `msd/msd-sim/test/Physics/Constraints/BlockPGSSolverTest.cpp` (created, 21 tests)
  - `msd/msd-sim/test/Physics/Constraints/CMakeLists.txt` (updated)
- **Notes**: 21 tests written and passing (21/21). Covers all public methods of BlockPGSSolver:
  - `buildBlockK` (3 tests): identity inertia diagonal K, off-center contact coupling, infinite mass body
  - `projectCoulombCone` (5 tests): quiescent inside cone, sliding saturated at cone boundary, separating zero impulse, near-zero friction, on-surface saturation
  - `applyRestitutionPreSolve`/Phase A (3 tests): restitution increases normal lambda, separating no-op, small velocity small bounce
  - `sweepOnce`/Phase B (2 tests): single contact convergence, two contacts same body pair
  - `solve` end-to-end (8 tests): sliding saturates, gravity support with hand-computed expected value, warm-start reduces iterations, quiescent zero impulse, high mass ratio stability, empty constraints, body forces size, hand-computed effective mass validation
  - All acceptance criteria met: 21 >= 15 tests, descriptive names (no ticket suffixes), all pass, hand-computed cone validation (Test 8/21), hand-computed K validation (Test 21)
