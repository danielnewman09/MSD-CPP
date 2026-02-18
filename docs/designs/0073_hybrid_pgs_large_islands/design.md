# Design: Hybrid PGS Solver for Large Constraint Islands

## Summary

The Active Set Method (ASM) provides exact LCP solutions but at O(k³) cost per pivot where k is the active constraint set size. Profiling via ticket 0071a revealed that large constraint islands (56–160 constraints) dominate solver time in scenes like ClusterDrop/32: the biggest island captures 70–80% of all constraints, and ASM's repeated dense Cholesky factorization on systems this large is the bottleneck.

This design adds a `ProjectedGaussSeidel` solver class and a threshold-based dispatch inside `ConstraintSolver::solve()`: islands with ≤ 20 constraints continue to use the exact ASM path; islands with > 20 constraints are routed to PGS, which has O(n) per sweep and scales linearly. PGS uses warm-starting from the existing `ContactCache` infrastructure, reducing sweep count from ~50 cold to ~5–10 warm. For the friction path on large islands, PGS uses a box friction approximation (already consistent with the FrictionConstraint design), avoiding the quadratic NLopt overhead on large systems.

---

## Architecture Changes

### PlantUML Diagram

See: `./0073_hybrid_pgs_large_islands.puml`

---

### New Components

#### ProjectedGaussSeidel

- **Purpose**: O(n)-per-sweep iterative LCP solver. Replaces ASM on constraint islands with more than `kASMThreshold` rows. Handles both unilateral (normal) and box-constrained (friction) lambda bounds through the existing `LambdaBounds` interface.
- **Header location**: `msd/msd-sim/src/Physics/Constraints/ProjectedGaussSeidel.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Constraints/ProjectedGaussSeidel.cpp`
- **Key interfaces**:
  ```cpp
  namespace msd_sim {

  class ProjectedGaussSeidel {
  public:
      ProjectedGaussSeidel() = default;
      ~ProjectedGaussSeidel() = default;

      ProjectedGaussSeidel(const ProjectedGaussSeidel&) = default;
      ProjectedGaussSeidel& operator=(const ProjectedGaussSeidel&) = default;
      ProjectedGaussSeidel(ProjectedGaussSeidel&&) noexcept = default;
      ProjectedGaussSeidel& operator=(ProjectedGaussSeidel&&) noexcept = default;

      /// Solve mixed LCP with per-row lambda bounds.
      /// constraints must be ContactConstraint and/or FrictionConstraint.
      /// initialLambda: warm-start vector (same size as active rows); zeros if empty.
      [[nodiscard]] ConstraintSolver::SolveResult solve(
          const std::vector<Constraint*>& constraints,
          const std::vector<std::reference_wrapper<const InertialState>>& states,
          const std::vector<double>& inverseMasses,
          const std::vector<Eigen::Matrix3d>& inverseInertias,
          size_t numBodies,
          double dt,
          const std::optional<Eigen::VectorXd>& initialLambda = std::nullopt);

      void setMaxSweeps(int n) { maxSweeps_ = n; }
      void setConvergenceTolerance(double tol) { convergenceTolerance_ = tol; }

  private:
      // Assemble per-row Jacobian rows (1 row per constraint dimension)
      [[nodiscard]] static std::vector<Eigen::Matrix<double, 1, 12>>
      assembleJacobianRows(
          const std::vector<Constraint*>& constraints,
          const std::vector<std::reference_wrapper<const InertialState>>& states);

      // Compute effective-mass diagonal: A_ii = J_i M^{-1} J_i^T
      [[nodiscard]] static Eigen::VectorXd computeDiagonal(
          const std::vector<Eigen::Matrix<double, 1, 12>>& jacobianRows,
          const std::vector<size_t>& bodyAIndices,
          const std::vector<size_t>& bodyBIndices,
          const std::vector<double>& inverseMasses,
          const std::vector<Eigen::Matrix3d>& inverseInertias,
          size_t numBodies);

      // Assemble RHS: b_i = -(1+e_i) * J_i * v
      [[nodiscard]] static Eigen::VectorXd assembleRHS(
          const std::vector<Constraint*>& constraints,
          const std::vector<Eigen::Matrix<double, 1, 12>>& jacobianRows,
          const std::vector<std::reference_wrapper<const InertialState>>& states);

      // Execute one Gauss-Seidel sweep; returns max |delta_lambda| this sweep
      static double sweepOnce(
          const Eigen::VectorXd& diag,
          const std::vector<Eigen::Matrix<double, 1, 12>>& jacobianRows,
          const std::vector<size_t>& bodyAIndices,
          const std::vector<size_t>& bodyBIndices,
          const Eigen::VectorXd& b,
          Eigen::VectorXd& lambda,              // in/out
          const std::vector<Constraint*>& constraints,
          const std::vector<double>& inverseMasses,
          const std::vector<Eigen::Matrix3d>& inverseInertias,
          size_t numBodies);

      // Extract per-body forces from final lambda
      [[nodiscard]] static std::vector<ConstraintSolver::BodyForces> extractBodyForces(
          const std::vector<Constraint*>& constraints,
          const std::vector<Eigen::Matrix<double, 1, 12>>& jacobianRows,
          const std::vector<size_t>& bodyAIndices,
          const std::vector<size_t>& bodyBIndices,
          const Eigen::VectorXd& lambda,
          size_t numBodies,
          double dt);

      int maxSweeps_{50};
      double convergenceTolerance_{1e-6};
  };

  } // namespace msd_sim
  ```
- **PGS Algorithm** (per constraint row i in each sweep):
  1. Compute off-diagonal contribution: `off_i = sum_{j != i} A_ij * lambda_j`
  2. Compute unconstrained update: `delta = (b_i - off_i) / A_ii`
  3. Clamp: `lambda_i = clamp(lambda_i + delta, lo_i, hi_i)` using `LambdaBounds`
  4. For `FrictionConstraint` rows: `hi_i = mu * lambda_normal_i`, `lo_i = -hi_i`
     (normal lambda is always the immediately-preceding ContactConstraint row in the interleaved layout)

  **Efficient off-diagonal computation**: Rather than materializing the full A matrix, the off-diagonal contribution is computed implicitly using the impulse-velocity product:
  - Maintain a velocity residual vector `v_res` (6 DOF × numBodies)
  - After updating lambda_i, update `v_res += M^{-1} J_i^T * delta_lambda_i`
  - Compute `A_ij * lambda_j` sum = `J_i * v_res` minus the diagonal term

  This keeps PGS memory-efficient at O(n) workspace rather than O(n²).

- **Friction bounds coupling**: In the interleaved constraint layout `[CC_0, FC_0, CC_1, FC_1, ...]`:
  - When processing FC row (tangent direction t), the normal lambda is the most recently solved CC row lambda.
  - The box bounds for tangent row are `[-mu * lambda_normal, +mu * lambda_normal]`.
  - Because constraints are processed in order and CC is solved before its corresponding FC, the most recent CC lambda is always available.

- **SolveResult mapping**:
  - `iterations` = number of completed sweeps
  - `converged` = true if max |delta_lambda| < tolerance before maxSweeps
  - `residual` = max |delta_lambda| on final sweep
  - `lambdas` = final lambda vector (same ordering as constraint rows)

- **Dependencies**:
  - `Constraint` — reads `lambdaBounds()`, `dimension()`, `jacobian()`
  - `InertialState` — reads velocities for RHS assembly
  - `ConstraintSolver::SolveResult` — return type reuse (no new return types)
  - Eigen3 — matrix/vector arithmetic

- **Thread safety**: Not thread-safe (stateful workspace vectors during solve)
- **Error handling**: If `A_ii == 0.0` (degenerate contact), skip row (delta = 0). No exceptions thrown.

---

### Modified Components

#### ConstraintSolver

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp/.cpp`
- **Changes required**:
  1. Add `ProjectedGaussSeidel pgsSolver_` as a private member (owns the PGS instance for workspace reuse).
  2. Add compile-time constant `static constexpr size_t kASMThreshold = 20;` (exposed as a public constant so tests can check it).
  3. Modify `solve()` to count active constraint rows before dispatching:
     - Count rows = sum of `constraint->dimension()` over all constraint pointers
     - **No-friction path**: if `numRows > kASMThreshold` → call `pgsSolver_.solve()`, else call existing `solveActiveSet()`
     - **Friction path** (FrictionConstraint detected): if `numRows > kASMThreshold` → call `pgsSolver_.solve()` (PGS handles interleaved layout with box friction coupling), else call existing NLopt path
  4. Include `ProjectedGaussSeidel.hpp`.
- **Backward compatibility**: Existing behavior unchanged for `numRows <= kASMThreshold`. The threshold constant is tunable at compile time (a `constexpr`).
- **Row counting**: `numRows` = total Jacobian rows across all constraints = `numContacts * 3` for full contact+friction system (1 normal + 2 tangent per contact), or `numContacts` for contact-only.

#### CollisionPipeline

- **Current location**: `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp/.cpp`
- **Changes required**:
  - No structural changes to `CollisionPipeline` itself. The warm-starting path in `solveConstraintsWithWarmStart()` already assembles a single `initialLambda` vector from `ContactCache`. This vector is passed as-is to `ConstraintSolver::solve()`, which now transparently routes to PGS if the system is large. The warm-start assembly and cache update code remains unchanged.
- **Backward compatibility**: No API changes. The pipeline is unaware of which solver backend was used.

---

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---|---|---|---|
| `ProjectedGaussSeidel` | `ConstraintSolver` | Ownership (member) | `ConstraintSolver` owns `pgsSolver_` by value; calls `solve()` |
| `ProjectedGaussSeidel` | `Constraint` | Read | Reads `lambdaBounds()`, `dimension()`, `jacobian()` |
| `ProjectedGaussSeidel` | `ConstraintSolver::SolveResult` | Return type | Reuses existing result struct; no new types |
| `ConstraintSolver` | `CollisionPipeline` | Existing (unchanged) | Pipeline passes `initialLambda` from cache; solver dispatches transparently |

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|---|---|---|---|
| `test/Physics/Constraints/ConstraintSolverTest.cpp` | All solver tests | Low — small systems use ASM as before | None expected; verify no regression |
| `test/Physics/Collision/CollisionPipelineTest.cpp` | Large island scenarios | Low — pipeline API unchanged | None expected |
| All physics integration tests (708 tests) | Any | Regression risk for n > 20 contacts | Run full suite; PGS path must produce equivalent results |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|---|---|---|
| `ProjectedGaussSeidel` | `SingleContactNormal` | PGS converges to correct lambda for one unilateral constraint; matches ASM result |
| `ProjectedGaussSeidel` | `SingleContactWithFriction` | PGS with box friction coupling produces lambda within ASM result ± 1e-4 |
| `ProjectedGaussSeidel` | `WarmStartReducesSweeps` | Starting from correct lambda converges in 1 sweep vs 20+ cold |
| `ProjectedGaussSeidel` | `ConvergenceToleranceEarlyExit` | max |delta_lambda| < tolerance triggers early exit before maxSweeps |
| `ProjectedGaussSeidel` | `DegenerateContactSkip` | Row with A_ii = 0 does not crash; result is valid for other rows |
| `ProjectedGaussSeidel` | `LargeFrictionIsland` | 60-constraint friction system converges; energy not injected |
| `ConstraintSolver` | `ThresholdDispatchASM` | n = 20 constraints routes to ASM path (check `iterations` consistent with ASM behavior) |
| `ConstraintSolver` | `ThresholdDispatchPGS` | n = 21 constraints routes to PGS path (check `iterations` field is sweep count) |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|---|---|---|
| `ClusterDropEnergyStability` | `CollisionPipeline`, `ProjectedGaussSeidel`, `ContactCache` | PGS on large island does not inject energy; total energy non-increasing after collision |
| `StackCollapseResting` | `CollisionPipeline`, `ProjectedGaussSeidel` | 16-object stack-collapse reaches resting state without drift |

#### Benchmark Tests

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|---|---|---|---|
| `CollisionPipeline` | `BM_ClusterDrop32_Pipeline` | Full pipeline time per frame for ClusterDrop/32 | < 5ms (down from 11.1ms) |
| `ConstraintSolver` | `BM_PGS_160Constraints` | PGS solve time for 160-constraint island, warm-started | < 0.5ms |
| `ConstraintSolver` | `BM_ASM_20Constraints` | ASM solve time for 20-constraint island (regression baseline) | < 0.1ms |

---

## Open Questions

### Design Decisions (Human Input Needed)

1. **Threshold value — compile-time constant vs runtime configuration**
   - Option A: `static constexpr size_t kASMThreshold = 20` — zero runtime overhead, tuned by recompiling
     - Pros: no configuration plumbing, enforces a single well-tested value
     - Cons: cannot tune per-scene without recompile
   - Option B: Runtime-configurable via `ConstraintSolver::setASMThreshold(size_t n)` — tunable without recompile
     - Pros: easier benchmarking exploration; tunable per simulation mode
     - Cons: adds configuration complexity; default still needs validation
   - **Recommendation**: Start with Option A for simplicity. Option B can be added later if benchmarking reveals scene-dependent optima.

2. **PGS friction handling for large islands — box vs cone**
   - The existing NLopt path uses an exact friction cone for small islands. For large islands, PGS necessarily uses box friction approximation (the FrictionConstraint already encodes `[-mu*lambda_n, +mu*lambda_n]` box bounds via `LambdaBounds`).
   - This means physics accuracy slightly degrades for large-island friction — contacts that are close to the cone boundary may slip when they should not.
   - **Is this accuracy trade-off acceptable?** The ticket explicitly accepts this (PGS is already approximate). Confirming: large islands occur primarily in densely packed multi-body pileups where the incremental accuracy difference between box and exact cone is not simulation-critical.
   - **Recommendation**: Accept box friction for PGS path. Document the trade-off in the implementation.

3. **Velocity-residual workspace allocation strategy**
   - Option A: Allocate `v_res` (6 × numBodies vector) inside `sweepOnce()` — simple, heap allocation per sweep
   - Option B: Pre-allocate `v_res` as a member of `ProjectedGaussSeidel` and resize as needed — avoids per-sweep allocation
   - **Recommendation**: Option B. Similar to how `ConstraintSolver` has `asmAw_`, `asmBw_`, `asmW_` workspace members to avoid allocation on hot paths.

### Prototype Required

None — the algorithm is well-established (Catto 2005, Box2D, Bullet). The threshold value (20) is derived from the FLOP analysis in the ticket and can be validated empirically during benchmarking.

### Requirements Clarification

1. **Island decomposition prerequisite**: The ticket references ticket 0071a (island decomposition) as an antecedent. The hybrid dispatch described here operates on a single `std::vector<Constraint*>` passed to `ConstraintSolver::solve()` — it does NOT depend on island decomposition infrastructure. The dispatch decision is simply: "how many rows does this constraint set have?" If island decomposition (0071a) is not yet merged, this ticket still provides benefit by routing any large per-frame constraint bundle to PGS.

   Confirm: Is ticket 0071a merged and its island-dispatch infrastructure available, or should this design treat the entire per-frame constraint set as a single "island" for dispatch purposes?

2. **SolverData reporting**: `CollisionPipeline::SolverData` currently tracks `iterations`, `residual`, `converged` from the ASM/NLopt solver path. When PGS is used, these fields map naturally (iterations = sweep count, residual = max |delta_lambda|, converged = tolerance met). No API change needed. Confirm this mapping is acceptable for recording/diagnostics.
