# Design: Hybrid PGS Solver for Large Constraint Islands

## Summary

The Active Set Method (ASM) provides exact LCP solutions but at O(k³) cost per pivot where k is the active constraint set size. Profiling via ticket 0071a revealed that large constraint islands (56–160 constraints) dominate solver time in scenes like ClusterDrop/32: the biggest island captures 70–80% of all constraints, and ASM's repeated dense Cholesky factorization on systems this large is the bottleneck.

This design adds a `ProjectedGaussSeidel` solver class and a threshold-based dispatch inside `ConstraintSolver::solve()`: islands with ≤ 20 constraints continue to use the exact ASM path; islands with > 20 constraints are routed to PGS, which has O(n) per sweep and scales linearly. PGS uses warm-starting from the existing `ContactCache` infrastructure, reducing sweep count from ~50 cold to ~5–10 warm. For friction, PGS uses the same ball-projection approach as the current decoupled solver (ticket 0070): after solving both tangent rows for a contact, the 2D tangent lambda vector is projected onto the friction disk (`||λ_t|| ≤ μ·λ_n`). This maintains the same friction accuracy as the existing solver without adding complexity.

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
  3. For normal rows: clamp `lambda_i = max(0, lambda_i + delta)` (unilateral)
  4. For friction rows: solve both tangent components, then ball-project the 2D tangent vector onto the friction disk `||λ_t|| ≤ μ·λ_n` (same approach as the current decoupled solver from ticket 0070)

  **Efficient off-diagonal computation**: Rather than materializing the full A matrix, the off-diagonal contribution is computed implicitly using the impulse-velocity product:
  - Maintain a velocity residual vector `v_res` (6 DOF × numBodies)
  - After updating lambda_i, update `v_res += M^{-1} J_i^T * delta_lambda_i`
  - Compute `A_ij * lambda_j` sum = `J_i * v_res` minus the diagonal term

  This keeps PGS memory-efficient at O(n) workspace rather than O(n²).

- **Friction coupling**: In the interleaved constraint layout `[CC_0, FC_0, CC_1, FC_1, ...]`:
  - When processing a contact's friction rows (2 tangent directions), the normal lambda is the most recently solved CC row lambda.
  - After solving both tangent components independently via PGS update, the 2D tangent vector `(λ_t1, λ_t2)` is ball-projected onto the friction disk: if `||λ_t|| > μ·λ_n`, scale to `λ_t *= μ·λ_n / ||λ_t||`.
  - This is identical to the ball-projection approach in the current decoupled solver (ticket 0070), maintaining friction accuracy parity.

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
     - **Friction path** (FrictionConstraint detected): if `numRows > kASMThreshold` → call `pgsSolver_.solve()` (PGS handles interleaved layout with ball-projection friction coupling), else call existing decoupled normal-then-friction path
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

## Resolved Design Decisions

1. **Threshold value**: `static constexpr size_t kASMThreshold = 20` — compile-time constant, zero runtime overhead. Can be made configurable later if benchmarking reveals scene-dependent optima.

2. **PGS friction handling**: Ball projection (same as current decoupled solver from ticket 0070). After solving both tangent rows for a contact, the 2D tangent vector is projected onto the friction disk `||λ_t|| ≤ μ·λ_n`. This maintains friction accuracy parity with the existing solver — no degradation for large islands.

3. **Velocity-residual workspace**: Pre-allocated as a member of `ProjectedGaussSeidel`, resized as needed. Avoids per-sweep heap allocation on hot path (same pattern as `ConstraintSolver`'s `asmAw_`, `asmBw_`, `asmW_` workspace members).

4. **Island decomposition prerequisite**: Branch is rebased on 0071a. Island decomposition is available. The threshold dispatch operates per-island: small islands → ASM (exact), large islands → PGS (approximate, warm-started).

5. **SolverData reporting**: PGS maps naturally to existing fields — iterations = sweep count, residual = max |delta_lambda|, converged = tolerance met. No API change needed.

### Prototype Required

None — the algorithm is well-established (Catto 2005, Box2D, Bullet). The threshold value (20) is derived from the FLOP analysis in the ticket and can be validated empirically during benchmarking.

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-17
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | `ProjectedGaussSeidel` PascalCase, members `maxSweeps_` / `convergenceTolerance_` trailing underscore, `kASMThreshold` kPascalCase constant — all conform to project standards |
| Namespace organization | ✓ | All new symbols in `msd_sim` namespace, consistent with every existing constraint-system class |
| File structure | ✓ | `.hpp` + `.cpp` pair under `msd/msd-sim/src/Physics/Constraints/` mirrors `NLoptFrictionSolver`, `ConstraintSolver`, `FrictionConstraint` — correct location |
| Dependency direction | ✓ | `ProjectedGaussSeidel` depends on `Constraint`, `InertialState`, `ConstraintSolver::SolveResult`, Eigen3 — no upward or circular dependencies. `ConstraintSolver` owns `pgsSolver_` by value, consistent with how it already owns `nloptSolver_` |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | No raw resources; workspace vectors are value members managed by implicit destructor |
| Smart pointer appropriateness | ✓ | `ProjectedGaussSeidel` owned by value inside `ConstraintSolver` — matches the pattern for `nloptSolver_`. No shared_ptr anywhere in the design |
| Value/reference semantics | ✓ | `solve()` takes inputs by const-reference and returns `SolveResult` by value. Workspace updated in-place as a member (pre-allocation pattern matches existing `asmAw_`/`asmBw_`/`asmW_`) |
| Rule of 0/3/5 | ✓ | Design explicitly shows all five special members as `= default` (identical to how `NLoptFrictionSolver` is declared). Stateful workspace member makes Rule of Zero impractical but the explicit-default Rule of Five is correct |
| Const correctness | N | `assembleJacobianRows`, `computeDiagonal`, `assembleRHS`, `extractBodyForces` are declared `[[nodiscard]] static` — correct. `sweepOnce` mutates `lambda` (in/out) and `v_res` (implied by velocity-residual scheme) — the design should mark the workspace mutation path clearly. No const violations observed, but the in-place `lambda` mutation means `sweepOnce` cannot be `const`; this is acceptable and the design already takes lambda by non-const reference |
| Exception safety | ✓ | Design explicitly states degenerate rows (A_ii == 0) skip without throwing. No exceptions thrown for normal operation — consistent with `CollisionPipeline` error handling convention |
| Initialization | ✓ | `maxSweeps_{50}`, `convergenceTolerance_{1e-6}` use brace initialization. Note: the design does not show floating-point uninitialized member values but all members are either int or double with explicit initial values — no NaN needed here |
| Return values | ✓ | All helpers return by value; `sweepOnce` is the only function mutating a parameter (lambda, which is the solved output), which is the correct pattern for an iterative updater |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | `ProjectedGaussSeidel.hpp` will need `Constraint.hpp`, `InertialState.hpp`, `ConstraintSolver.hpp` (for `SolveResult`/`BodyForces`) — all already included by `ConstraintSolver.hpp`. No new transitive includes required |
| Template complexity | ✓ | No templates introduced. `Eigen::Matrix<double, 1, 12>` is a fixed-size Eigen type with no instantiation complexity |
| Memory strategy | ✓ | O(n) workspace pre-allocated as a member. The velocity-residual vector `v_res` (6 × numBodies doubles) is well-defined and bounded. No per-sweep or per-row heap allocations on the hot path |
| Thread safety | ✓ | Design explicitly states not thread-safe (stateful workspace vectors). Consistent with `ConstraintSolver`, `CollisionPipeline`, and `NLoptFrictionSolver` — all single-threaded |
| Build integration | ✓ | Two new `.cpp` files added to the existing `msd-sim` CMake target — straightforward `target_sources()` additions matching the existing pattern |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | `ProjectedGaussSeidel::solve()` takes all inputs by value/const-ref. Can be exercised from a unit test without any simulation infrastructure by constructing `Constraint*` vectors directly |
| Mockable dependencies | ✓ | `Constraint` is polymorphic; test stubs can trivially override `jacobian()`, `lambdaBounds()`, `dimension()`. `InertialState` is a value type |
| Observable state | ✓ | `SolveResult` exposes `iterations`, `converged`, `residual`, `lambdas` — sufficient to verify threshold dispatch, sweep count, and convergence behavior from tests |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|-----------------|----------|------------|--------|------------|------------|
| R1 | Ball-projection friction and box-constraints: existing `FrictionConstraint::lambdaBounds()` returns `boxConstrained(-μ/√2·λn, +μ/√2·λn)` (inscribed-square approximation), but the design's PGS friction step uses ball-projection (`||λ_t|| ≤ μ·λ_n`). These are different constraint sets. The ball-projection cone is larger than the inscribed square (up to 41% at diagonals). During the PGS sweep, `FrictionConstraint::lambdaBounds()` will be queried for box bounds, but PGS then replaces those bounds with a ball-projection step. This inconsistency needs to be made explicit in the implementation: PGS should call ball-projection on friction rows regardless of what `lambdaBounds()` returns, and this should be documented clearly. | Integration | Medium | Medium | Implementation must explicitly detect FrictionConstraint rows and apply ball-projection, ignoring `lambdaBounds()` for those rows. Add a design note clarifying that PGS uses ball-projection rather than `lambdaBounds()` for friction coupling. | No |
| R2 | PGS convergence on large friction islands: warm-started PGS is expected to converge in 5-10 sweeps. However, if ContactCache provides stale or zero warm-start on the first frame after a scene change (new objects spawned, or after a high-energy collision), the solver may need 30-50 sweeps. With maxSweeps_=50 at 160 constraints per sweep this is still ~100x cheaper than ASM, but the 5ms target may not be met on first-frame spikes. | Performance | Low | Medium | Accept this risk. First-frame cold-start penalty is bounded and temporary. Monitor with BM_ClusterDrop32_Pipeline benchmark. | No |
| R3 | `FlattenedConstraints` vs direct `Constraint*` interface: the existing `ConstraintSolver` uses `flattenConstraints()` to expand multi-row constraints (FrictionConstraint dim=2) into per-row entries before assembly. The proposed `ProjectedGaussSeidel::solve()` takes a raw `std::vector<Constraint*>` and handles interleaved layout internally. This is a divergence from the established assembly pattern. During implementation, the implementer must be careful not to double-count FrictionConstraint rows (each FC has dimension=2, producing 2 Jacobian rows but is one element in the constraints vector). The design's `assembleJacobianRows` signature returns a `vector<Matrix<double,1,12>>` which implies it handles per-row expansion — this should be verified against `flattenConstraints()` for consistency. | Technical | Low | Low | Design intent is correct; `assembleJacobianRows` should replicate the per-row expansion from `flattenConstraints()`. Implementation should reuse or call `ConstraintSolver::flattenConstraints()` to avoid duplicating that logic. Note this in the implementation ticket. | No |
| R4 | `SolverDispatch` conceptual class in the PlantUML diagram is not an actual C++ class — the diagram shows it as a new component but the design text clarifies it is dispatch logic embedded in `ConstraintSolver::solve()`. This discrepancy between diagram and implementation could confuse the implementer. | Maintenance | Low | Low | Update the PlantUML diagram to remove `SolverDispatch` as a separate class and instead show the threshold dispatch as a note on `ConstraintSolver::solve()`. Minor polish item; does not block implementation. | No |

### Notes (Not Blocking)

1. **Re-use of `flattenConstraints()` recommended**: The implementation of `ProjectedGaussSeidel::assembleJacobianRows()` will need to expand multi-row constraints (FrictionConstraint dim=2) into per-row entries. This is identical to what `ConstraintSolver::flattenConstraints()` already does. The implementer should call `flattenConstraints()` (promoted to at least `protected` or passed as a parameter) rather than duplicating the per-row expansion logic. This is a recommendation, not a blocker — the design is correct as written, and the re-use can be decided during implementation.

2. **`FrictionConstraint::setNormalLambda()` coupling**: The existing NLopt path calls `setNormalLambda()` on each `FrictionConstraint` to update the box bounds before each constraint evaluation. The PGS ball-projection approach bypasses `lambdaBounds()` for friction rows, so `setNormalLambda()` is not needed on the PGS path. The implementer should confirm this and not call `setNormalLambda()` during PGS sweeps (it would be a no-op but adds confusion).

3. **`SolverDispatch` in PlantUML**: The diagram shows `SolverDispatch` as a separate class with a `static kASMThreshold` member, but the design text is clear this is conceptual. The note at the bottom of the diagram already says "Extracted here conceptually." The diagram should be updated before the PR is merged to avoid confusing future readers; recommend doing so in the implementation or docs phase.

4. **`kASMThreshold` access for tests**: The design states `kASMThreshold` is `static constexpr size_t` exposed as a public constant. The tests `ThresholdDispatchASM` (n=20, ASM path) and `ThresholdDispatchPGS` (n=21, PGS path) depend on this value being publicly visible. This is the correct approach — the implementer should ensure `kASMThreshold` is declared `public` in `ConstraintSolver`.

5. **Row count vs constraint count for threshold**: The design uses `numRows > kASMThreshold` where `numRows = sum of constraint->dimension()`. For a system with 7 contacts plus friction (7 CC + 7 FC = 21 constraints), numRows = 7*1 + 7*2 = 21 rows, which exceeds the threshold. This is consistent with the FLOP analysis (ASM cost is O(k³) in rows, not in constraint objects). The threshold of 20 rows corresponds to approximately 6-7 contacts with friction — reasonable for the ClusterDrop/32 scenario where large islands have 56-160 rows.

### Summary

The design is well-specified and architecturally sound. It introduces `ProjectedGaussSeidel` as an owned member of `ConstraintSolver`, follows the exact same ownership and workspace patterns as the existing `NLoptFrictionSolver` and ASM workspace members, reuses `ConstraintSolver::SolveResult` without API changes, and is transparent to `CollisionPipeline`. The principal risk (R1) is the discrepancy between `FrictionConstraint::lambdaBounds()` returning box bounds and PGS using ball-projection — this is an intentional design choice (parity with the ticket 0070 decoupled solver) that must be explicitly handled during implementation. All other risks are low-impact. The design is approved to proceed to implementation.
