# Ticket 0071c: Eigen Fixed-Size Matrices — Direct Conversions

## Status
- [x] Draft
- [x] Investigation Complete
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Implementation
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Investigation Complete
**Type**: Performance
**Priority**: Medium
**Created**: 2026-02-21
**Generate Tutorial**: No
**Parent Ticket**: [0071_collision_pipeline_profiling](0071_collision_pipeline_profiling.md)
**Related Tickets**: [0071d_constraint_interface_static_dispatch](0071d_constraint_interface_static_dispatch.md) (architectural changes to the Constraint virtual interface)

---

## Summary

The constraint solver pipeline uses `Eigen::MatrixXd` and `Eigen::VectorXd` in several locations where the matrix dimensions are already effectively compile-time constants — not because of the virtual `Constraint` interface, but because the solver's internal data structures and assembly routines operate on known-size rows and blocks. This ticket covers the straightforward conversions that can be made without changing the `Constraint` class hierarchy or its virtual interface.

Architectural changes to the `Constraint` virtual interface (CRTP, static dispatch, etc.) are tracked separately in [0071d](0071d_constraint_interface_static_dispatch.md).

---

## Problem

### Profiling Evidence (from parent ticket 0071, ClusterDrop/32)

| Subsystem | Total Samples | % of Total |
|-----------|---------------|------------|
| Eigen BLAS (gebp_kernel, triangular_solve, LLT, matrix-vector) | ~650 | ~3.6% |
| Constraint Solver (solve, flattenConstraints, assembleFlatEffectiveMass) | ~280 | ~1.5% |
| Memory Allocator (_xzm_free, _xzm_xzone_malloc_tiny) | ~80 | ~0.4% |

Memory allocator samples (0.4%) and Eigen BLAS overhead suggest per-frame heap allocation from dynamic Eigen types even where dimensions are predictable.

### Scope: What Can Be Directly Converted

These are dynamic Eigen types in solver internals where the size is known without needing to change the `Constraint` interface:

**1. `FlattenedConstraints::jacobianRows` — `vector<Eigen::RowVectorXd>` → `vector<Eigen::Matrix<double, 1, 12>>`**

All two-body velocity-level Jacobian rows are 1x12 (3 linear + 3 angular per body, two bodies). `flattenConstraints()` splits multi-row constraints into individual rows, each of which is always 1x12. This is the most impactful single conversion — it removes a heap allocation per constraint row per frame and enables compile-time block extraction in `assembleFlatEffectiveMass()`.

**2. Per-constraint `evaluate()` return in `assembleRHS` / `assembleFlatRHS`**

The RHS assembly routines compute `Jv` products using `(jacobians[i] * v)(0)`. If the Jacobian row is fixed-size `Matrix<double, 1, 12>` and the velocity vector is already `Matrix<double, 12, 1>` (it is, since ticket 0053f), the product becomes a fully compile-time-known `(1x12) * (12x1)` dot product.

**3. `assembleJacobians()` return type — `vector<Eigen::MatrixXd>` → `vector<Eigen::Matrix<double, 1, 12>>`**

The no-friction path uses `assembleJacobians()` which returns a vector of per-constraint Jacobians. For the current contact-only usage (all `ContactConstraint`, dimension 1, 12 columns), this can be a fixed-size type.

**4. `assembleEffectiveMass()` inner loop sub-blocks**

Already uses `block<1,6>()` on dynamic matrices. With fixed-size source types from (3), these extractions become compile-time optimizable.

### What's Already Fixed-Size (from ticket 0053f)

- `Eigen::Matrix<double, 6, 6>` — per-body inverse mass matrices
- `Eigen::Matrix<double, 1, 6>` — Jacobian sub-blocks in extractBodyForces
- `Eigen::Matrix<double, 12, 1>` — velocity vectors in assembleRHS/assembleFlatRHS
- `Eigen::Matrix2d`, `Eigen::Vector2d` — friction tangent 2x2 solve
- `Eigen::Matrix3d` — inverse inertia tensors

### Out of Scope (tracked in 0071d)

- Changes to the `Constraint` virtual interface (`evaluate()`, `jacobian()` return types)
- CRTP or static dispatch patterns to eliminate virtual call overhead
- Any modification to the `Constraint` class hierarchy

### Out of Scope (not planned)

- Assembled system matrices (`A`, `b`, `lambda`) — truly runtime-sized, must remain dynamic
- Template instantiation over island sizes — code bloat with diminishing returns

---

## Investigation Findings

### Key Observation: The Solver Already Decouples from the Virtual Interface

`flattenConstraints()` calls `constraint->jacobian()` (virtual, returns `MatrixXd`) but then immediately splits the result into individual 1x12 rows stored in `FlattenedConstraints::jacobianRows`. From that point forward, the solver never calls the virtual interface again — all assembly and solve operations work on the flattened rows. This means the storage type of `jacobianRows` can be changed to fixed-size `Matrix<double, 1, 12>` independently of the virtual interface return type. The virtual call still returns `MatrixXd`, but the copy into the flattened structure converts to fixed-size.

Similarly, `assembleJacobians()` in the no-friction path collects virtual `jacobian()` returns into a vector. The vector element type can be fixed-size even though the source is dynamic — it's a narrowing copy at the boundary.

### Conversion Points

| Location | Current Type | Target Type | Notes |
|----------|-------------|-------------|-------|
| `FlattenedConstraints::jacobianRows` | `vector<RowVectorXd>` | `vector<Matrix<double, 1, 12>>` | All rows are 1x12 |
| `assembleJacobians()` return | `vector<MatrixXd>` | `vector<Matrix<double, 1, 12>>` | Contact-only path, all 1x12 |
| `assembleEffectiveMass()` jacobians param | `vector<MatrixXd>&` | `vector<Matrix<double, 1, 12>>&` | Cascading from above |
| `assembleRHS()` jacobians param | `vector<MatrixXd>&` | `vector<Matrix<double, 1, 12>>&` | Cascading from above |
| `extractBodyForces()` jacobians param | `vector<MatrixXd>&` | `vector<Matrix<double, 1, 12>>&` | Cascading from above |

---

## Success Criteria

- `FlattenedConstraints::jacobianRows` uses `Matrix<double, 1, 12>` (zero heap allocation per row)
- `assembleJacobians()` returns fixed-size row type
- No physics test regressions (all existing tests passing)
- `Constraint` virtual interface unchanged — no breaking changes to extensibility
- Measurable reduction in memory allocator samples in ClusterDrop/32 profiling

---

## Profiling Artifacts

- Parent ticket profiling: `analysis/profile_results/profile_0071_multibody_*.json`
- Benchmark: `msd/msd-sim/bench/MultiBodyCollisionBench.cpp`

---

## Workflow Log

### Investigation Phase
- **Started**: 2026-02-21
- **Completed**: 2026-02-21
- **Notes**:
  - Catalogued all dynamic Eigen usage in Constraints/ and Collision/ directories
  - Identified that solver internals decouple from the virtual interface after `flattenConstraints()` / `assembleJacobians()` — fixed-size storage is possible at the boundary
  - Confirmed all two-body velocity-level Jacobian rows are 1x12
  - Scoped to direct conversions only; architectural interface changes deferred to 0071d
