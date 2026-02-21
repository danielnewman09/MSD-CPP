# Investigation Iteration Log: 0071d Constraint Interface Static Dispatch

## Iteration 1 — Initial Investigation

**Date**: 2026-02-21

### Work Performed

Read all relevant source files and profiling data to evaluate whether static
dispatch changes to the `Constraint` virtual interface are warranted:

- `msd/msd-sim/src/Physics/Constraints/Constraint.hpp` — virtual interface definition
- `msd/msd-sim/src/Physics/Constraints/ContactConstraint.hpp` — concrete type, dim=1, cols=12
- `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.hpp` — concrete type, dim=2, cols=12
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` — `flattenConstraints()`,
  `assembleJacobians()`, and all caller implementations
- `msd/msd-sim/src/Physics/Constraints/PositionCorrector.cpp` — `correctPositions()`,
  `jacobians_` workspace
- `tickets/0071_collision_pipeline_profiling.md` — ClusterDrop/32 profiling data
- `tickets/0071c_eigen_fixed_size_matrices.md` — what 0071c delivered and its
  key insight about the interface/compute boundary

### Key Findings

1. **The 0071c boundary pattern is already in place.** The solver decouples from
   the virtual interface after `flattenConstraints()` / `assembleJacobians()`.
   All downstream compute (effective mass, RHS, force extraction) uses fixed-size
   `Matrix<double, 1, 12>` with zero heap allocation.

2. **Remaining virtual interface `MatrixXd` allocation sites are two:**
   - `flattenConstraints()`: one `MatrixXd` per constraint per frame (150 max)
   - `PositionCorrector::correctPositions()`: workspace `jacobians_[i]` is a
     reused `MatrixXd` member; after the first frame, assignment is a copy not
     an allocation

3. **Profiling attribution:** Memory allocator samples (~80 total in 18,146
   sample run) span all sources. Attributing even half to the remaining virtual
   interface sites gives <0.2% CPU. The dominant cost is Eigen BLAS (3.6%).

4. **All four architectural options evaluated:**
   - CRTP (A): Full refactor of solver + pipeline + visitor. Closed type set.
     Not justified.
   - std::variant (B): High architectural cost, closed type set, negligible gain.
     Not justified.
   - Hybrid fixed-size accessors (C): Dual implementations per type, ongoing
     maintenance burden, gain <0.1%. Not worth it.
   - Output buffer (D): Breaks ergonomics and extensibility. Not worth it.

5. **Real optimization opportunity identified:** `PositionCorrector::jacobians_`
   workspace could be converted from `vector<MatrixXd>` to
   `vector<Matrix<double, 1, 12>>` following the 0071c pattern, entirely within
   `PositionCorrector.hpp/.cpp`, with no virtual interface change. This is the
   most attractive remaining non-architectural optimization.

### Recommendation Formed

Do not proceed with any architectural change to the `Constraint` virtual
interface. Close 0071d as "No Implementation Required — Investigation Only."
Create an optional follow-on ticket for the `PositionCorrector` fixed-size
Jacobian workspace.

### Artifacts Produced

- `docs/designs/0071d_constraint_interface_static_dispatch/design.md`
  — Full investigation document with cost-benefit table, option analysis,
    profiling data interpretation, and recommendation
