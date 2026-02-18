# Ticket 0072: Hybrid PGS Solver for Large Islands

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Implementation
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Design Approved — Ready for Implementation
**Type**: Performance
**Priority**: High
**Created**: 2026-02-17
**Generate Tutorial**: No
**Parent Ticket**: [0071_collision_pipeline_profiling](0071_collision_pipeline_profiling.md)
**Related**: [0071a_constraint_solver_scalability](0071a_constraint_solver_scalability.md)

---

## Summary

The Active Set Method (ASM) provides exact LCP solutions but has O(k^3) cost per pivot where k = active set size. Island decomposition (ticket 0071a) improved ClusterDrop/32 from 20.8ms to 11.1ms (1.87x), but diagnostic logging revealed the largest island still contains 70-80% of all constraints (56-160 rows in ClusterDrop/32). The cubic cost of ASM on these large islands is the dominant remaining bottleneck.

A hybrid solver strategy would use ASM for small islands (exact, fast at small sizes) and Projected Gauss-Seidel (PGS) for large islands (approximate but O(n) per sweep, linearly scaling).

---

## Problem

Benchmark diagnostic data from ticket 0071a (ClusterDrop/32, 20 frames):

| Metric | Value |
|--------|-------|
| Total islands per frame | 3-6 |
| Largest island constraints | 56-160 |
| Largest island % of total | 70-80% |
| StackCollapse/16 islands | Usually 1 |

The largest island dominates solver time. ASM on a 160-constraint system involves repeated dense Cholesky factorization of matrices up to 160x160, with multiple pivot iterations. PGS would process the same system in O(n) per sweep with fixed iteration count.

### Scaling comparison

| Solver | Per-iteration cost | Iterations | Total | Notes |
|--------|-------------------|------------|-------|-------|
| ASM | O(k^3) per pivot | Finite (exact) | O(k^3 * pivots) | Exact solution |
| PGS | O(n) per sweep | Fixed (e.g. 20-50) | O(n * sweeps) | Approximate |

For k=160 constraints: ASM pivot ~ 160^3 = 4M FLOPs; PGS sweep ~ 160 * 12 = 2K FLOPs. Even 50 PGS sweeps = 100K FLOPs, ~40x cheaper than a single ASM pivot.

---

## Proposed Solution

### Hybrid Dispatch in CollisionPipeline

After island decomposition (from 0071a), dispatch each island to the appropriate solver based on constraint count:

```
For each island:
  if island.constraints.size() <= kASMThreshold:
    solve with Active Set Method (exact)
  else:
    solve with PGS (approximate, warm-started)
```

### Threshold Selection

- **kASMThreshold**: ~20-30 constraints (tunable)
- Below threshold: ASM is fast and exact, cubic cost is negligible
- Above threshold: PGS with warm-start from ContactCache provides good approximate solutions at linear cost

### PGS Implementation

Standard Projected Gauss-Seidel for mixed LCP:
1. For each constraint row i:
   - Compute delta_lambda_i = (b_i - A_i * lambda) / A_ii
   - Project: lambda_i = clamp(lambda_i + delta_lambda_i, lo_i, hi_i)
2. Repeat for N sweeps (default: 30-50)
3. Convergence check: early exit if max |delta_lambda| < tolerance

Key details:
- Normal constraints: lambda >= 0 (unilateral)
- Friction constraints: -mu*lambda_n <= lambda_t <= mu*lambda_n (box-constrained, coupled to normal)
- Warm-start: Initialize lambda from ContactCache (existing infrastructure from ticket 0040b/0044)
- Friction cone: Ball projection onto friction disk `||λ_t|| ≤ μ·λ_n` (same approach as current decoupled solver from ticket 0070)

### Warm-Starting

The existing ContactCache already stores lambda values per contact pair. For PGS on large islands:
1. Query ContactCache for previous frame's lambda (same as current ASM path)
2. Use as initial lambda_0 for PGS iteration
3. Store solved lambda back to cache (same as current path)

This is the most impactful optimization: warm-started PGS typically converges in 5-10 sweeps vs 30-50 cold.

---

## Success Criteria

1. **ClusterDrop/32 speedup**: Target < 5ms (currently 11.1ms with islands, was 20.8ms without)
2. **16-to-32 scaling ratio**: Target < 4x (currently ~8x from 1.32ms to 11.1ms)
3. **No physics regressions**: All 708 tests pass (PGS only used on large islands; small-island tests still use exact ASM)
4. **Acceptable accuracy**: Resting contact stability maintained (PGS with warm-start + position correction)

---

## Future Extension: ML Warm-Starting

The PGS solver is a natural target for ML-accelerated warm-starting:
- A Graph Neural Network (GNN) could predict lambda from contact graph topology + body states
- GNN output feeds into PGS as initial lambda, reducing sweeps from 30-50 to 3-5
- This is the most practical ML application for the constraint solver since:
  - PGS is tolerant of approximate initial guesses (self-correcting)
  - Variable-sized contact graphs map naturally to GNNs
  - Training data is abundant (simulation recordings)

This ticket does NOT implement ML — it establishes the PGS infrastructure that ML would accelerate.

---

## Implementation Notes

### Files to Create
- `msd/msd-sim/src/Physics/Constraints/ProjectedGaussSeidel.hpp` — PGS solver class
- `msd/msd-sim/src/Physics/Constraints/ProjectedGaussSeidel.cpp` — Implementation
- `msd/msd-sim/test/Physics/Constraints/ProjectedGaussSeidelTest.cpp` — Unit tests

### Files to Modify
- `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp` — Hybrid dispatch logic in `solveConstraintsWithWarmStart()`
- `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp` — Add PGS member or configuration
- `msd/msd-sim/bench/CMakeLists.txt` — If new benchmarks needed

### Key Design Decisions
- PGS uses ball-projection friction (same as current solver from ticket 0070) — maintains accuracy parity
- Threshold is compile-time constant initially; can be made configurable later
- PGS reuses the same Jacobian/mass-matrix assembly as ASM (shared infrastructure in ConstraintSolver)
- Position correction (PositionCorrector) still runs after PGS — no change to split-impulse pipeline

---

## Workflow Log

### Design Phase
- **Started**: 2026-02-17 00:00
- **Completed**: 2026-02-17 00:00
- **Branch**: `0073-hybrid-pgs-large-islands`
- **PR**: #76 (draft)
- **Artifacts**:
  - `docs/designs/0073_hybrid_pgs_large_islands/design.md`
  - `docs/designs/0073_hybrid_pgs_large_islands/0073_hybrid_pgs_large_islands.puml`
- **Notes**: No math design required (standard PGS algorithm per Catto 2005). Design introduces `ProjectedGaussSeidel` class with threshold dispatch at n=20 constraints in `ConstraintSolver::solve()`. Key open questions: threshold as constexpr vs runtime config (recommendation: constexpr); box friction approximation on large islands accepted. Confirm whether ticket 0071a island decomposition is a prerequisite.

### Design Review Phase
- **Started**: 2026-02-17 00:00
- **Completed**: 2026-02-17 00:00
- **Branch**: `0073-hybrid-pgs-large-islands`
- **PR**: #76
- **Artifacts**:
  - `docs/designs/0073_hybrid_pgs_large_islands/design.md` (review appended)
- **Notes**: APPROVED WITH NOTES. All resolved design decisions (kASMThreshold=20 constexpr, ball-projection friction, pre-allocated workspace, 0071a available) confirmed in design. Four risks identified: R1 (medium — ball-projection vs FrictionConstraint::lambdaBounds() box bounds; PGS must apply ball-projection explicitly, not rely on lambdaBounds()); R2 (low — first-frame cold-start penalty); R3 (low — flattenConstraints() reuse recommended); R4 (low — SolverDispatch diagram cleanup). No revision required. PR comment posted.

---

## References

- Catto (2005): "Iterative Dynamics with Temporal Coherence" — PGS for game physics
- Bullet Physics: btSequentialImpulseConstraintSolver — production PGS implementation
- Box2D: b2ContactSolver — PGS with warm-starting and position correction
- Ticket 0071a: Island decomposition and diagnostic data
- Ticket 0034: Active Set Method (current exact solver)
