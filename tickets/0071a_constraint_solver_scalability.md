# Ticket 0071a: Constraint Solver Scalability

## Status
- [x] Draft
- [x] Investigation Complete
- [x] Design Complete — Awaiting Review
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Design Complete — Awaiting Review
**Type**: Performance / Investigation
**Priority**: High
**Created**: 2026-02-17
**Generate Tutorial**: No
**Parent Ticket**: [0071_collision_pipeline_profiling](0071_collision_pipeline_profiling.md)

---

## Summary

The Active Set Method constraint solver exhibits super-linear scaling with body count. Profiling ClusterDrop/32 (ticket 0071) showed a 15x time increase from 16→32 bodies (1.38ms → 20.8ms) despite only a 2x increase in body count. The root cause is O(n³) dense Cholesky factorization on a constraint system that grows quadratically (O(n²) contact pairs).

Eigen BLAS operations (gebp_kernel, triangular_solve, LLT) account for 3.6% of total CPU in the multi-body benchmark — all driven by the constraint solver's dense matrix operations.

---

## Problem

`ConstraintSolver::solveActiveSet` uses `Eigen::LLT` (dense Cholesky) to factor the full effective mass matrix `J * M_inv * J^T`. With 32 bodies generating 50+ contacts (each with normal + 2 friction rows = 150+ constraint rows), this matrix is large and factored repeatedly during Active Set pivoting.

### Profiling Evidence (ClusterDrop/32, 18,146 samples)

| Function | Samples | % |
|----------|---------|---|
| `Eigen::gebp_kernel` (matrix multiply) | 227 | 1.3% |
| `Eigen::triangular_solve_matrix` | 178 | 1.0% |
| `ConstraintSolver::solveActiveSet` | 90 | 0.5% |
| `ConstraintSolver::solve` | 86 | 0.5% |
| `Eigen::general_matrix_vector_product` | 147 | 0.8% |
| `Eigen::LLT::compute` | 38 | 0.2% |

### Scaling Data

| Bodies | ClusterDrop Time (ms) | Ratio vs 4 |
|--------|-----------------------|-------------|
| 4 | 0.152 | 1x |
| 8 | 0.873 | 5.7x |
| 16 | 1.38 | 9.1x |
| 32 | 20.8 | 137x |

---

## Investigation Directions

### Option A: Block-Diagonal Approximation

Solve per-contact-pair rather than global system. Each body-body pair typically has 1-4 contacts (3-12 constraint rows). Solving many small systems independently is O(n) overall vs O(n³) global.

**Trade-off**: Loses inter-pair coupling. Two bodies simultaneously contacting a third won't see each other's impulses within a single iteration. May require more Gauss-Seidel outer iterations to converge.

### Option B: Sparse Matrix Solvers

Replace `Eigen::LLT` (dense) with `Eigen::SimplicialLLT` or `Eigen::SparseLU`. The effective mass matrix is naturally sparse — each constraint row only references 2 bodies (6 DOF each), so most entries are zero.

**Trade-off**: Sparse factorization has higher per-element overhead. May only win for large systems (20+ contacts).

### Option C: Hybrid Solver

Use Active Set Method for small systems (≤ ~20 constraint rows) and switch to iterative PGS/Jacobi for large systems. The Active Set Method gives exact solutions for small problems; PGS scales linearly but converges slowly.

**Trade-off**: Two code paths to maintain. Need a good heuristic for the switching threshold.

### Option D: Constraint Reduction

Reduce the number of constraints entering the solver:
- Island detection: partition bodies into independent contact groups, solve each separately
- Contact point merging: reduce 4-point manifolds to fewer representative contacts
- Distance-based pruning: skip constraint generation for near-zero penetration pairs

**Trade-off**: Island detection is O(n) with union-find and doesn't affect per-island solver complexity. Contact reduction may lose physical accuracy.

---

## Success Criteria

- ClusterDrop/32 benchmark time reduced by at least 3x (from 20.8ms)
- ClusterDrop scaling from 16→32 bodies is at most 4x (currently 15x)
- No physics test regressions (697/697 passing)
- No energy conservation regressions in existing collision tests

---

## Profiling Artifacts

- Parent ticket profiling: `analysis/profile_results/profile_0071_multibody_*.json`
- Benchmark: `msd/msd-sim/bench/MultiBodyCollisionBench.cpp`

---

## Workflow Log

### Investigation Phase
- **Started**: 2026-02-17
- **Completed**: 2026-02-17
- **Branch**: `0071a-constraint-solver-scalability`
- **Artifacts**: See parent ticket 0071 profiling results in `analysis/profile_results/`
- **Notes**: Investigation complete via parent ticket 0071. Root cause confirmed as O(n³) dense
  Cholesky on a globally assembled effective mass matrix that grows O(n²) with contact count.
  Eigen BLAS (gebp_kernel, triangular_solve) accounts for 3.6% of total CPU at 32 bodies.
  Recommended approach: island-based decomposition (Option D from investigation directions) using
  union-find to partition the contact graph into independent connected components.

### Design Phase
- **Started**: 2026-02-17
- **Completed**: 2026-02-17
- **Branch**: `0071a-constraint-solver-scalability`
- **PR**: N/A (not yet created)
- **Artifacts**:
  - `docs/designs/0071a_constraint_solver_scalability/design.md`
  - `docs/designs/0071a_constraint_solver_scalability/0071a_constraint_solver_scalability.puml`
- **Notes**:
  - Selected Option D (island detection) over Options A/B/C based on risk/reward analysis:
    - Option A (block-diagonal): Loses inter-pair coupling, risks physics regressions
    - Option B (sparse solver): Higher per-element overhead, only wins for very large systems
    - Option C (hybrid PGS): Two code paths, accuracy regression for large systems
    - Option D (island detection): O(n) overhead, zero accuracy loss, no solver changes needed
  - New component: `ConstraintIslandBuilder` (static utility, no Eigen dependency)
  - `ConstraintSolver` is UNCHANGED — purely orchestration change in `CollisionPipeline`
  - Open question: need to validate that islands actually form in ClusterDrop/32 scenario (if all
    32 bodies are in one island, speedup is zero — prototype/diagnostic step required)
  - `PositionCorrector` island decomposition recommended alongside velocity solve (same partition)
