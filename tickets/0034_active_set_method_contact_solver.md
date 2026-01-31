# Ticket 0034: Active Set Method Contact Solver

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [x] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Documentation Complete — Ready for Merge
**Assignee**: docs-updater
**Created**: 2026-01-31
**Generate Tutorial**: No
**Related Tickets**: [0032b_pgs_solver_extension](0032b_pgs_solver_extension.md), [0032_contact_constraint_refactor](0032_contact_constraint_refactor.md), [0033_constraint_solver_contact_tests](0033_constraint_solver_contact_tests.md)

---

## Summary

Replace the Projected Gauss-Seidel (PGS) iterative solver in `ConstraintSolver::solveWithContacts()` with an Active Set Method (ASM) for solving the contact constraint Linear Complementarity Problem (LCP). The ASM partitions contacts into active (compressive) and inactive (separating) sets, solving an equality-constrained subproblem at each step via direct LLT decomposition, and iterates by adding/removing constraints until all KKT conditions are satisfied. This provides finite convergence to the exact solution, eliminating PGS's sensitivity to iteration count, constraint ordering, and high mass ratios.

---

## Motivation

### PGS Limitations

The current PGS solver (ticket 0032b) has known limitations that become problematic as simulation complexity grows:

1. **Approximate solution**: PGS produces an iterative approximation. With `max_iterations=10`, the solution may not satisfy complementarity conditions exactly. This manifests as visible jitter in resting contacts and energy drift over long simulations.

2. **High mass ratio sensitivity**: PGS convergence degrades significantly for mass ratios > 100:1. A heavy object resting on a light platform requires many more iterations (50-100+) to converge, which is impractical at real-time rates.

3. **Order dependence**: PGS results depend on the order in which constraints are processed. Reordering the contact list produces different lambda values, making behavior non-deterministic and harder to debug.

4. **Iteration count tuning**: The `max_iterations` parameter is a heuristic — too few iterations leave residual error, too many waste computation. There is no principled way to set this value for all scenarios.

5. **No convergence guarantee for LCPs**: PGS is guaranteed to converge only for symmetric positive definite systems. The contact LCP with complementarity conditions (λ ≥ 0) weakens this guarantee for certain constraint configurations.

### Active Set Method Advantages

The Active Set Method addresses all of these limitations:

1. **Exact solution**: Converges to the exact LCP solution in a finite number of iterations, satisfying all complementarity conditions precisely.

2. **Mass ratio robustness**: Each iteration uses a direct LLT solve on the active subproblem, which is insensitive to mass ratios (Eigen's LLT handles condition numbers up to ~1e12).

3. **Deterministic**: The solution depends only on the constraint geometry and physics, not on processing order.

4. **No iteration tuning**: The algorithm terminates naturally when KKT conditions are satisfied. No `max_iterations` heuristic needed (though a safety cap is retained for degenerate cases).

5. **Well-understood theory**: The Active Set Method for convex QPs with box constraints has rigorous convergence proofs (Nocedal & Wright, Chapter 16).

---

## Technical Approach

### Algorithm: Active Set Method for Contact LCP

The contact constraint problem is a convex Quadratic Program (QP) with non-negativity constraints:

```
minimize    ½ λᵀ A λ - bᵀ λ
subject to  λ ≥ 0
```

Where:
- `A = J · M⁻¹ · Jᵀ` is the symmetric positive semi-definite effective mass matrix (C × C)
- `b` is the RHS vector encoding restitution and Baumgarte terms
- `λ` is the vector of contact impulse magnitudes

This is equivalent to the LCP: find λ such that `A λ - b ≥ 0`, `λ ≥ 0`, `λᵀ(Aλ - b) = 0`.

#### Active Set Algorithm

```
ActiveSetSolve(A, b):
  // Initialize: assume all contacts active (compressive)
  W ← {1, 2, ..., C}            // Working set (active constraints with λ > 0)

  for iter = 1 to max_safety_iterations:
    // Step 1: Solve equality-constrained subproblem for active set
    A_W = A[W, W]                // Extract active rows/columns
    b_W = b[W]                   // Extract active RHS
    λ_W = solve(A_W, b_W)       // Direct LLT solve

    // Step 2: Set inactive lambdas to zero
    λ[~W] = 0

    // Step 3: Check for negative lambdas in active set (KKT violation)
    if any λ_W[i] < 0:
      // Find most negative lambda
      j = argmin(λ_W)
      // Remove constraint j from active set (it wants to pull, not push)
      W = W \ {j}
      continue

    // Step 4: Check inactive constraints for violation
    // Compute constraint accelerations for inactive contacts
    w = A · λ - b               // Residual for all constraints
    violated = {i ∉ W : w[i] < -tolerance}

    if violated is empty:
      // All KKT conditions satisfied — exact solution found
      return λ, converged=true, iter

    // Step 5: Add most violated constraint to active set
    k = argmin(w[i] for i in violated)
    W = W ∪ {k}

  return λ, converged=false, max_safety_iterations
```

#### KKT Conditions for Contact LCP

The Active Set Method converges when all Karush-Kuhn-Tucker conditions hold simultaneously:

1. **Primal feasibility**: `λ_i ≥ 0` for all i (contacts can only push)
2. **Dual feasibility**: `w_i = (Aλ - b)_i ≥ 0` for all i (no constraint violation)
3. **Complementarity**: `λ_i · w_i = 0` for all i (either force is zero or constraint is exactly satisfied)

The active set partition enforces complementarity by construction:
- Active set (W): `λ_i > 0` and `w_i = 0` (compressive contacts)
- Inactive set (~W): `λ_i = 0` and `w_i ≥ 0` (separating contacts)

### Unchanged Components

The following components from tickets 0032a/0032b remain unchanged:

- **Jacobian assembly** (`assembleContactJacobians`): Same 1×12 per-contact Jacobians
- **Effective mass matrix** (`assembleContactEffectiveMass`): Same A = J·M⁻¹·Jᵀ with regularization
- **RHS assembly** (`assembleContactRHS`): Same restitution and Baumgarte terms
- **Force extraction** (`extractContactBodyForces`): Same Jᵀλ/dt decomposition
- **Public interface**: `solveWithContacts()` signature, `MultiBodySolveResult`, `BodyForces` structs

### Modified Components

| Component | Change |
|-----------|--------|
| `solvePGS()` | **Replace** with `solveActiveSet()` |
| `PGSResult` | **Replace** with `ActiveSetResult` (add `activeSetSize` field) |
| `max_iterations_` | **Rename** to `max_safety_iterations_` (semantic change: safety cap, not convergence control) |
| `convergence_tolerance_` | **Repurpose**: Tolerance for constraint violation check in Step 4 |

### New Private Method

```cpp
/**
 * @brief Solve contact LCP using Active Set Method
 *
 * Partitions contacts into active (compressive) and inactive (separating)
 * sets, solving an equality subproblem at each iteration via LLT decomposition
 * until all KKT conditions are satisfied.
 *
 * @param A Effective mass matrix (C × C), symmetric positive semi-definite
 * @param b RHS vector (C × 1) with restitution and Baumgarte terms
 * @return ActiveSetResult with lambda vector, convergence info, and active set size
 */
ActiveSetResult solveActiveSet(const Eigen::MatrixXd& A,
                               const Eigen::VectorXd& b) const;
```

### Initialization Strategy

The algorithm initializes with all contacts in the active set (W = {1, ..., C}). This is the optimal starting point for resting contact scenarios (most contacts compressive). For separating contacts, the algorithm removes them from the active set in the first few iterations.

Alternative: Warm starting from previous frame's active set (deferred to future enhancement).

### Complexity Analysis

| Metric | PGS (current) | Active Set Method |
|--------|---------------|-------------------|
| Per-iteration cost | O(C²) | O(\|W\|³) for LLT solve on active subset |
| Total iterations | Fixed (max_iterations) | Finite, typically C or fewer |
| Worst case | O(max_iter · C²) | O(2^C · C³) (pathological, never observed in practice) |
| Typical (5 contacts) | ~10 iterations × O(25) = O(250) | ~5 iterations × O(125) = O(625) |
| Typical (20 contacts) | ~10 iterations × O(400) = O(4000) | ~10 iterations × O(8000) = O(80000) |
| Exact solution? | No (approximate) | Yes |

For the typical contact counts in this simulation (1-10 contacts), the ASM per-iteration cost is higher but the iteration count is lower and the solution is exact. For large contact counts (> 50), PGS may be faster in wall-clock time but less accurate. This project's typical use case (< 20 contacts) is well within ASM's efficient regime.

---

## Requirements

### Functional Requirements

1. **FR-1**: `solveActiveSet()` produces the exact LCP solution satisfying all KKT conditions (primal feasibility, dual feasibility, complementarity)
2. **FR-2**: All contact lambdas are non-negative (λ ≥ 0) in the returned solution
3. **FR-3**: Separating contacts produce λ = 0 (complementarity enforced exactly)
4. **FR-4**: The `solveWithContacts()` public interface and return types remain unchanged
5. **FR-5**: Restitution and Baumgarte RHS terms computed identically to current implementation
6. **FR-6**: Safety iteration cap prevents infinite loops for degenerate inputs
7. **FR-7**: `MultiBodySolveResult::iterations` reports the number of active set changes performed

### Non-Functional Requirements

1. **NFR-1**: Existing `solve()` bilateral method completely unchanged — no regressions
2. **NFR-2**: All existing `ConstraintSolverContactTest` tests pass with the new solver (same physical behavior)
3. **NFR-3**: Converges within C iterations (number of contacts) for non-degenerate systems
4. **NFR-4**: Handles mass ratios up to 1e6:1 without convergence failure
5. **NFR-5**: No heap allocation beyond Eigen dynamic matrices (same as current)
6. **NFR-6**: Regularization epsilon preserved on diagonal for numerical stability

---

## Acceptance Criteria

1. [ ] AC1: `solvePGS()` replaced by `solveActiveSet()` with equivalent public behavior
2. [ ] AC2: 23 of 24 existing `ConstraintSolverContactTest` tests pass without modification; `MaxIterationsReached_ReportsNotConverged_0033` requires scenario update (ASM converges in 1 iteration for the original resting-contact scenario)
3. [ ] AC3: All existing bilateral constraint tests pass without modification
4. [ ] AC4: Head-on equal-mass collision (e=1.0) produces lambda matching analytical result within 1e-10 (tighter than PGS's 1e-6)
5. [ ] AC5: Mass ratio 1e6:1 converges successfully (test case added)
6. [ ] AC6: Active set size reported in result (new `activeSetSize` field in result struct or iterations field reused)
7. [ ] AC7: Safety iteration cap of 2C (twice the number of contacts) enforced
8. [ ] AC8: Constraint ordering does not affect result — test with shuffled contact lists produces identical lambdas within 1e-12
9. [ ] AC9: New unit tests cover: active set cycling detection, degenerate configurations (redundant contacts), single-contact exact solve, all-separating contacts
10. [ ] AC10: No performance regression for typical contact counts (1-10): measured via existing test execution time

---

## Dependencies

- **Ticket 0032a**: Two-Body Constraint Infrastructure (provides `TwoBodyConstraint`, `ContactConstraint`) — prerequisite
- **Ticket 0032b**: PGS Solver Extension (provides current `solveWithContacts()` implementation) — this ticket replaces the PGS solver from 0032b
- **Ticket 0033**: Constraint Solver Contact Tests (provides comprehensive test suite) — tests should pass unchanged
- **Blocks**: [0032c](0032c_worldmodel_contact_integration.md) — WorldModel integration depends on stable solver interface

---

## Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Active set cycling (same constraint added/removed repeatedly) | Low | Medium | Bland's rule anti-cycling: always add/remove constraint with smallest index among tied candidates |
| Higher per-iteration cost for large contact counts | Medium | Low | Typical contact count is < 20; degenerate cases capped by safety limit; PGS fallback could be retained as option |
| Degenerate configurations (redundant contacts on same surface) | Medium | Low | Regularization epsilon on diagonal prevents singular subproblems |
| Test behavior changes due to exact vs. approximate solutions | Low | Low | Exact solution should satisfy all existing test tolerances (which were designed for PGS approximation) |

---

## Files

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Replace `PGSResult` with `ActiveSetResult`, replace `solvePGS()` with `solveActiveSet()`, rename `max_iterations_` to `max_safety_iterations_` |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Implement `solveActiveSet()`, remove `solvePGS()` |

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/Constraints/ConstraintSolverASMTest.cpp` | Active Set Method-specific unit tests (cycling, ordering, exact solve, mass ratios) |

### Files to Remove

None. The PGS code is replaced in-place within `ConstraintSolver.cpp`.

---

## Test Plan

### Existing Tests (Must Pass Unchanged)

All 24 tests in `ConstraintSolverContactTest.cpp` must pass without modification, validating behavioral equivalence.

### New Tests (ConstraintSolverASMTest.cpp)

| Test Case | What It Validates |
|-----------|-------------------|
| `SingleContact_ExactSolution` | Single contact lambda matches analytical formula within 1e-12 |
| `OrderIndependence_ShuffledContacts` | Two contact orderings produce identical lambda within 1e-12 |
| `HighMassRatio_1e6_Converges` | Mass ratio 1e6:1 converges successfully |
| `AllSeparating_EmptyActiveSet` | All contacts separating → all λ = 0, active set empty |
| `AllCompressive_FullActiveSet` | All contacts compressive → all λ > 0, active set = full set |
| `MixedActiveInactive_CorrectPartition` | Some active, some inactive — verify correct partition |
| `RedundantContacts_RegularizationPreventsFailure` | Duplicate contacts at same point don't crash |
| `SafetyCapReached_ReportsNotConverged` | Artificial degenerate input hits safety cap, returns converged=false |
| `KKTConditions_Verified` | Post-solve: λ ≥ 0, Aλ-b ≥ 0, λᵀ(Aλ-b) ≈ 0 verified explicitly |
| `IterationCount_FiniteBound` | Verify iterations ≤ 2C for non-degenerate systems |

---

## Future Extensions

- **Warm starting**: Initialize active set from previous frame's solution for faster convergence in persistent contacts
- **Friction constraints**: Extend to 3-DOF contacts (1 normal + 2 tangential) with friction cone approximation — ASM naturally handles the box-constrained QP formulation for friction
- **Sparse subproblem solve**: Use sparse LLT (Eigen::SimplicialLLT) for large active sets (> 50 contacts)
- **PGS fallback**: Retain PGS as a solver option selectable at runtime for comparison/benchmarking

---

## References

### Academic Literature
- Nocedal, J. & Wright, S. (2006). "Numerical Optimization", Chapter 16: Quadratic Programming — Active Set Methods
- Cottle, R., Pang, J., Stone, R. (1992). "The Linear Complementarity Problem" — Theoretical foundations for LCP
- Baraff, D. (1994). "Fast Contact Force Computation for Nonpenetrating Rigid Bodies" — Active set approach for rigid body contacts
- Murty, K. (1988). "Linear Complementarity, Linear and Nonlinear Programming" — Principal pivoting methods

### Physics Engine Implementations
- Bullet Physics: `btDantzigSolver` — Active set (Dantzig/Lemke) solver option alongside PGS
- DART (Dynamic Animation and Robotics Toolkit): Uses Dantzig LCP solver as default
- MuJoCo: Uses projected Newton with active set identification
- ODE: `dSolveLCP` with both PGS and pivot (active set) options

---

## Workflow Log

### Draft Phase
- **Created**: 2026-01-31
- **Notes**: Ticket created to replace PGS with Active Set Method for improved convergence, determinism, and mass ratio robustness.

### Design Phase
- **Started**: 2026-01-31
- **Completed**: 2026-01-31
- **Notes**: Design document and PlantUML diagrams created at docs/designs/0034_active_set_method_contact_solver/. Covers algorithm pseudocode, KKT conditions, interface changes, performance analysis, and test strategy.
- **Artifacts**:
  - `docs/designs/0034_active_set_method_contact_solver/design.md`
  - `docs/designs/0034_active_set_method_contact_solver/0034_active_set_method_contact_solver.puml`

### Design Review Phase
- **Completed**: 2026-01-31
- **Status**: APPROVED WITH NOTES (2 iterations)
- **Notes**: Initial review found 4 issues (I1: test breakage, I4: residual semantics, I5: unnecessary copy, I6: parameter redundancy). All addressed in revision. 1 test (`MaxIterationsReached_ReportsNotConverged_0033`) confirmed to require modification. AC2 updated accordingly.

### Prototype Phase
- **Skipped**: No prototype required. ASM is well-established algorithm; assembly pipeline validated by 0032b prototypes. Design reviewer confirmed no prototype needed.

### Implementation Phase
- **Started**: 2026-01-31
- **Completed**: 2026-01-31
- **Notes**: Replaced PGS solver with Active Set Method. All 417 tests pass (24 existing contact tests + 12 new ASM tests + 381 other tests). One existing test (`MaxIterationsReached_ReportsNotConverged_0033`) modified per design to use mixed compressive/separating scenario. AC2 updated accordingly in ticket.
- **Artifacts**:
  - `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` — ActiveSetResult struct, solveActiveSet() declaration
  - `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` — solveActiveSet() implementation
  - `msd-sim/test/Physics/Constraints/ConstraintSolverASMTest.cpp` — 12 new unit tests
  - `msd-sim/test/Physics/Constraints/ConstraintSolverContactTest.cpp` — 1 test modified
  - `msd-sim/test/Physics/Constraints/CMakeLists.txt` — Added new test file

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
