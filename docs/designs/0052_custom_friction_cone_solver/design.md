# Design: Custom Friction Cone Solver

## Summary

Replace the ECOS-based SOCP solver with a custom projected Newton solver for friction contact constraints. The new solver computes contact impulses by minimizing the QP objective `(1/2) lambda^T A lambda - b^T lambda` subject to second-order cone constraints `||lambda_t|| <= mu * lambda_n` per contact. This provides exact Coulomb friction cones, warm starting from previous frames, and removes the ECOS external dependency. The design introduces two new components (`ConeProjection`, `FrictionConeSolver`) and modifies three existing components (`ConstraintSolver`, `CollisionPipeline`, `ContactCache`).

## Architecture Changes

### PlantUML Diagram

See: `./0052_custom_friction_cone_solver.puml`

---

### New Components

#### ConeProjection

- **Purpose**: Stateless utility providing the Euclidean projection of a 3-vector onto the friction cone `{(s, u) : ||u|| <= mu * s, s >= 0}` and its Jacobian matrix.
- **Header location**: `msd/msd-sim/src/Physics/Constraints/ConeProjection.hpp`
- **Source location**: Header-only (all functions are `static` and small enough to inline).
- **Key interfaces**:
  ```cpp
  namespace msd_sim
  {

  /// Result of projecting a single contact's impulse onto its friction cone
  struct ProjectionResult
  {
    double lambda_n{0.0};
    double lambda_t1{0.0};
    double lambda_t2{0.0};
    int case_{0};  // 1 = interior, 2 = origin, 3 = cone surface
  };

  class ConeProjection
  {
  public:
    /// Project (lambda_n, lambda_t1, lambda_t2) onto cone K_mu
    /// Three geometric cases from math formulation M3:
    ///   Case 1: ||p_t|| <= mu * p_n  -> identity (interior)
    ///   Case 2: mu * ||p_t|| <= -p_n -> zero (dual cone)
    ///   Case 3: otherwise            -> cone surface (Eq. 24-25)
    [[nodiscard]] static ProjectionResult project(
      double lambda_n, double lambda_t1, double lambda_t2, double mu);

    /// Compute 3x3 Jacobian of projection (Eq. 32-33 from M3)
    /// Case 1: I_3, Case 2: 0, Case 3: rank-2 matrix
    [[nodiscard]] static Eigen::Matrix3d gradient(
      double lambda_n, double lambda_t1, double lambda_t2, double mu);

    /// Project entire lambda vector (3C x 1) onto product cone K
    /// Processes C independent 3-tuples at indices [3i, 3i+1, 3i+2]
    [[nodiscard]] static Eigen::VectorXd projectVector(
      const Eigen::VectorXd& lambda,
      const std::vector<double>& mu,
      int numContacts);

    ConeProjection() = delete;  // Stateless utility, no instances
  };

  }  // namespace msd_sim
  ```
- **Dependencies**: Eigen3 (for `Matrix3d`, `VectorXd`), `<cmath>` (for `sqrt`)
- **Thread safety**: Fully thread-safe (stateless, pure functions)
- **Error handling**: No exceptions. Handles edge cases inline:
  - `mu = 0`: degenerates to half-line projection `(max(p_n, 0), 0, 0)`
  - `||p_t|| = 0` in apparent Case 3: actually Case 2 (see M3 analysis)

#### FrictionConeSolver

- **Purpose**: Projected Newton solver for the QP with second-order cone constraints. Takes the pre-assembled effective mass matrix, RHS vector, and per-contact friction coefficients, and returns the optimal impulse vector.
- **Header location**: `msd/msd-sim/src/Physics/Constraints/FrictionConeSolver.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Constraints/FrictionConeSolver.cpp`
- **Key interfaces**:
  ```cpp
  namespace msd_sim
  {

  class FrictionConeSolver
  {
  public:
    /// Result of the friction cone solve
    struct SolveResult
    {
      Eigen::VectorXd lambda;    // Optimal impulse vector (3C x 1)
      bool converged{false};     // True if projected gradient residual < tolerance
      int iterations{0};         // Newton iterations performed
      double residual{std::numeric_limits<double>::quiet_NaN()};  // Final projected gradient norm
    };

    FrictionConeSolver() = default;
    ~FrictionConeSolver() = default;

    /// Solve the friction cone QP:
    ///   min (1/2) lambda^T A lambda - b^T lambda
    ///   s.t. ||lambda_t_i|| <= mu_i * lambda_n_i, lambda_n_i >= 0
    ///
    /// @param A  Effective mass matrix (3C x 3C), symmetric positive semi-definite
    /// @param b  RHS vector (3C x 1) with restitution terms
    /// @param mu Per-contact friction coefficients (C entries)
    /// @param lambda0  Optional warm-start vector (3C x 1). If empty or wrong size,
    ///                 cold start from zero.
    /// @return SolveResult with optimal lambda and diagnostics
    [[nodiscard]] SolveResult solve(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& b,
      const std::vector<double>& mu,
      const Eigen::VectorXd& lambda0 = Eigen::VectorXd{}) const;

    void setTolerance(double eps) { tolerance_ = eps; }
    void setMaxIterations(int n) { max_iterations_ = n; }

    [[nodiscard]] double getTolerance() const { return tolerance_; }
    [[nodiscard]] int getMaxIterations() const { return max_iterations_; }

    // Rule of Five
    FrictionConeSolver(const FrictionConeSolver&) = default;
    FrictionConeSolver& operator=(const FrictionConeSolver&) = default;
    FrictionConeSolver(FrictionConeSolver&&) noexcept = default;
    FrictionConeSolver& operator=(FrictionConeSolver&&) noexcept = default;

  private:
    double tolerance_{1e-8};
    int max_iterations_{50};
    double armijo_c1_{1e-4};
    double armijo_beta_{0.5};
    int max_line_search_{10};
    static constexpr double kRegularizationEpsilon = 1e-10;
  };

  }  // namespace msd_sim
  ```
- **Dependencies**: Eigen3 (`MatrixXd`, `VectorXd`, `LLT`), `ConeProjection`
- **Thread safety**: Thread-safe for concurrent calls to `solve()` (all mutable state is local to the function). Configuration setters are not thread-safe.
- **Error handling**:
  - If Cholesky factorization fails, increases regularization epsilon by factors of 10 up to `1e-6`, then returns `converged = false`.
  - If max iterations reached, returns best solution found with `converged = false`.
  - If all line search steps fail, accepts smallest step and continues.

**Algorithm** (from M2 of math formulation):

```
solve(A, b, mu, lambda0):
  1. Regularize: A_reg = A + eps * I
  2. Cholesky factor: L L^T = A_reg
  3. Compute unconstrained optimum: lambda_unc = solve(A_reg, b)
  4. Initialize: lambda = Proj_K(lambda0)  (or Proj_K(lambda_unc) if cold start)
  5. For iter = 1..maxIter:
     a. Gradient: g = A_reg * lambda - b
     b. Projected gradient residual: r = ||lambda - Proj_K(lambda - g)||
     c. If r < tol: return (converged)
     d. Newton direction: delta = solve(A_reg, -g)  (reuses Cholesky)
     e. Line search with projection (Armijo):
        alpha = 1.0
        For ls = 1..maxLineSearch:
          trial = Proj_K(lambda + alpha * delta)
          If f(trial) <= f(lambda) + c1 * g^T * (trial - lambda): break
          alpha *= beta
     f. lambda = trial
  6. Return (not converged, best lambda)
```

#### FrictionSpec

- **Purpose**: Lightweight data structure replacing `FrictionConeSpec` without ECOS dependency. Carries per-contact friction coefficients for the solver.
- **Header location**: `msd/msd-sim/src/Physics/Constraints/FrictionSpec.hpp`
- **Source location**: Header-only (trivial struct).
- **Key interfaces**:
  ```cpp
  namespace msd_sim
  {

  /// Friction specification for the cone solver (replaces FrictionConeSpec)
  struct FrictionSpec
  {
    int numContacts{0};
    std::vector<double> frictionCoefficients;  // mu per contact, size = numContacts
  };

  }  // namespace msd_sim
  ```
- **Dependencies**: `<vector>`
- **Thread safety**: Value type, safe to copy/move.
- **Error handling**: None (plain data).

---

### Modified Components

#### ConstraintSolver

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` / `.cpp`
- **Changes required**:

  1. **Remove ECOS**: Delete `solveWithECOS()`, `buildFrictionConeSpec()`, ECOS configuration members (`ecos_abs_tol_`, `ecos_rel_tol_`, `ecos_max_iters_`), ECOS configuration methods (`setECOSTolerance()`, `setECOSMaxIterations()`, `getECOSTolerance()`, `getECOSMaxIterations()`), ECOS diagnostic fields from `ActiveSetResult` (`solver_type`, `ecos_exit_flag`, `primal_residual`, `dual_residual`, `gap`). Remove ECOS includes.

  2. **Add friction dispatch**: New `solveWithFriction()` private method that:
     - Builds `FrictionSpec` from the constraint list (scans for `FrictionConstraint*` via `dynamic_cast`)
     - Flattens the constraint list into per-row Jacobians (see Constraint Flattening below)
     - Assembles the 3C x 3C effective mass matrix and 3C x 1 RHS vector
     - Delegates to `FrictionConeSolver::solve()`
     - Maps the 3C-dimensional lambda back to per-body forces

  3. **Constraint flattening**: New `flattenConstraints()` static helper that expands the interleaved `[ContactConstraint, FrictionConstraint, ContactConstraint, FrictionConstraint, ...]` list into a flat list of 1x12 Jacobian rows ordered as `[n_0, t1_0, t2_0, n_1, t1_1, t2_1, ...]`. Each FrictionConstraint (dim=2) produces two separate 1x12 rows. This preserves the existing `block<1,6>` assembly code without modification.

  4. **Fix `extractBodyForces()`**: Remove the `lambda(i) <= 0.0` skip at line 502. For the friction path, all lambda values (including negative friction forces) must contribute to body forces. Two options:
     - **Option A** (recommended): Remove the skip entirely for the friction path (separate `extractBodyForcesFlat()` method).
     - **Option B**: Condition the skip on constraint type (only skip for unilateral normal constraints).

  5. **Friction RHS convention**: For friction constraint rows, the RHS is `b_t = -J_t * v` (no restitution term). The existing `assembleRHS()` already handles this: it applies `-(1+e)` only for `ContactConstraint*` (via `dynamic_cast`) and falls back to `-jv` for other constraints. However, this must work correctly with the flattened per-row Jacobians.

- **Backward compatibility**: The no-friction path (`solveActiveSet`) is completely unchanged. The friction dispatch replaces the ECOS path, which was not used in production (only validation tests).

#### CollisionPipeline

- **Current location**: `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp` / `.cpp`
- **Changes required**:

  1. **Create FrictionConstraints alongside ContactConstraints**: In `createConstraints()`, for each contact point, create both a `ContactConstraint` (normal, dim=1) and a `FrictionConstraint` (tangential, dim=2). The FrictionConstraint uses the same contact geometry (contact points, CoM, lever arms) and the combined friction coefficient from the colliding bodies.

  2. **Storage**: Add `std::vector<std::unique_ptr<FrictionConstraint>> frictionConstraints_` alongside the existing `constraints_` (which holds `ContactConstraint`). Build the interleaved `constraintPtrs_` as `[contact_0, friction_0, contact_1, friction_1, ...]`.

  3. **Friction coefficient source**: `AssetInertial` and `AssetEnvironment` need a friction coefficient accessor. For this design, we use a default friction coefficient (`kDefaultFrictionCoefficient = 0.5`) and combine using the geometric mean `mu_combined = sqrt(mu_A * mu_B)`, matching the existing restitution combination pattern.

  4. **Warm-start adaptation (3 lambdas per contact)**: The `solveConstraintsWithWarmStart()` method currently builds an `initialLambda` vector with one entry per constraint. With friction, the solver returns 3C lambdas (3 per contact). The warm-start assembly must:
     - Query `ContactCache` for 3 values per contact (normal + 2 tangential)
     - Assemble the `initialLambda` vector in the interleaved ordering `[n_0, t1_0, t2_0, ...]`
     - After solving, extract 3 values per contact to store back in the cache

  5. **Position correction filtering**: The `correctPositions()` method passes `constraintPtrs_` to `PositionCorrector`. With friction constraints in the list, the corrector must only operate on normal (ContactConstraint) constraints. The current `PositionCorrector` implementation uses `dynamic_cast<ContactConstraint*>` internally to access `getPenetrationDepth()`. **FrictionConstraints will fail this cast and be skipped automatically**, so no change to PositionCorrector is needed. However, CollisionPipeline should pass only ContactConstraint pointers to the corrector for clarity and efficiency. This means building a separate `normalConstraintPtrs_` vector.

  6. **`clearFrameData()`**: Add `frictionConstraints_.clear()`.

#### ContactCache

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ContactCache.hpp` / `.cpp`
- **Changes required**:

  1. **3 lambdas per contact**: The `CachedContact::lambdas` vector changes from storing 1 value per contact point to 3 values per contact point in flat layout: `[lambda_n_0, lambda_t1_0, lambda_t2_0, lambda_n_1, ...]`.

  2. **`getWarmStart()` changes**: Currently returns `vector<double>` with one entry per contact point. Now returns `vector<double>` with 3 entries per contact point. The nearest-neighbor matching copies 3 values per matched contact instead of 1: `cached.lambdas[3*bestIdx]`, `cached.lambdas[3*bestIdx+1]`, `cached.lambdas[3*bestIdx+2]`.

  3. **`update()` changes**: Accepts `vector<double>` with 3 entries per contact point.

  4. **Tangent basis sensitivity**: When the tangent basis rotates between frames (e.g., due to in-plane body rotation), the cached tangential lambdas become stale even if the normal is unchanged. The design applies a simple heuristic: if the normal similarity check passes but the contact was matched by proximity, the tangential lambdas are still used as-is. The cone projection in the warm-start feasibility restoration (Eq. 38) will correct any infeasible values. This is acceptable because:
     - Small tangent rotations cause small errors in warm-start, corrected in 1-2 iterations
     - Large tangent rotations typically accompany large normal rotations, which trigger cache invalidation (>15 degree threshold)

- **Backward compatibility**: The interface changes are internal to the collision pipeline. No external callers of ContactCache are affected.

---

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| FrictionConeSolver | ConstraintSolver | Composition (called by) | solveWithFriction() delegates to FrictionConeSolver::solve() |
| ConeProjection | FrictionConeSolver | Static dependency | FrictionConeSolver calls ConeProjection::project/projectVector |
| FrictionSpec | ConstraintSolver | Data flow | Built by ConstraintSolver from constraint list, passed to FrictionConeSolver |
| FrictionConstraint | CollisionPipeline | Ownership | Pipeline creates and owns FrictionConstraints alongside ContactConstraints |
| ContactCache (3-lambda) | CollisionPipeline | Bidirectional | Pipeline reads/writes 3 lambdas per contact |

---

## Constraint Flattening Strategy

### Problem

The existing assembly functions (`assembleEffectiveMass`, `assembleRHS`, `extractBodyForces`) all assume each entry in the constraint list produces exactly one Jacobian row, using `block<1,6>(0,0)` to extract sub-blocks. `FrictionConstraint` has `dimension() = 2`, producing a 2x12 Jacobian, which breaks this assumption.

### Solution: Pre-flatten to 1-row-per-entry

Before passing constraints to the assembly functions, expand the constraint list into a flat list where each entry has exactly one Jacobian row:

```
Input constraints (interleaved):
  [ContactConstraint_0 (dim=1), FrictionConstraint_0 (dim=2),
   ContactConstraint_1 (dim=1), FrictionConstraint_1 (dim=2), ...]

Flattened representation:
  Row 0: ContactConstraint_0 Jacobian row 0  (normal)
  Row 1: FrictionConstraint_0 Jacobian row 0 (tangent 1)
  Row 2: FrictionConstraint_0 Jacobian row 1 (tangent 2)
  Row 3: ContactConstraint_1 Jacobian row 0  (normal)
  Row 4: FrictionConstraint_1 Jacobian row 0 (tangent 1)
  Row 5: FrictionConstraint_1 Jacobian row 1 (tangent 2)
  ...
```

This is implemented as a helper struct:

```cpp
struct FlattenedConstraints
{
  std::vector<Eigen::Matrix<double, 1, 12>> jacobianRows;  // 3C rows
  std::vector<size_t> bodyAIndices;  // body A index per row
  std::vector<size_t> bodyBIndices;  // body B index per row
  std::vector<bool> isNormalRow;     // true for normal rows (restitution applies)
  std::vector<double> restitutions;  // e per row (0 for friction rows)
  int numContacts{0};
};
```

The flattened structure feeds directly into the existing assembly pattern but with clear per-row metadata. The `isNormalRow` flag determines whether the `-(1+e)` factor applies to the RHS.

### Why not modify assembly functions?

Modifying `assembleEffectiveMass()` et al. to handle variable-dimension constraints would require changing the `block<1,6>` template parameter to a runtime value, which loses compile-time optimization and complicates the code. The flatten approach keeps the hot-path code unchanged and localizes the complexity to a single expansion step.

---

## Data Flow

### Sequence: Collision to Force Application (with Friction)

```
WorldModel::updateCollisions(dt)
  |
  v
CollisionPipeline::execute(inertial, env, dt)
  |
  |-- Phase 1: detectCollisions()
  |     GJK/EPA -> CollisionResult per pair
  |
  |-- Phase 2: createConstraints()
  |     For each contact point:
  |       Create ContactConstraint (normal, dim=1)
  |       Create FrictionConstraint (tangential, dim=2)
  |     Build interleaved constraintPtrs_:
  |       [CC_0, FC_0, CC_1, FC_1, ...]
  |     Build normalConstraintPtrs_ (for position correction):
  |       [CC_0, CC_1, ...]
  |
  |-- Phase 3: assembleSolverInput()
  |     Gather states, inverseMasses, inverseInertias
  |
  |-- Phase 3.5: Warm-Start
  |     Query ContactCache for 3 lambdas per contact
  |     Assemble initialLambda vector (3C x 1)
  |
  |-- Phase 4: ConstraintSolver::solve(constraintPtrs_, ...)
  |     |
  |     |-- Detect friction (hasFriction = true)
  |     |-- flattenConstraints() -> FlattenedConstraints
  |     |-- assembleFlatEffectiveMass() -> A (3C x 3C)
  |     |-- assembleFlatRHS() -> b (3C x 1)
  |     |-- buildFrictionSpec() -> FrictionSpec
  |     |-- solveWithFriction(A, b, spec):
  |     |     |
  |     |     v
  |     |   FrictionConeSolver::solve(A, b, mu, lambda0)
  |     |     |-- Cholesky factor A
  |     |     |-- Compute lambda_unc = A^{-1} b
  |     |     |-- lambda = Proj_K(lambda0)
  |     |     |-- Newton loop:
  |     |     |     g = A*lambda - b
  |     |     |     r = ||lambda - Proj_K(lambda - g)||
  |     |     |     delta = A^{-1}(-g)  (Cholesky solve)
  |     |     |     Armijo line search with Proj_K
  |     |     |-- Return SolveResult{lambda, converged, iters}
  |     |
  |     |-- extractBodyForcesFlat(lambdas) -> per-body forces
  |     |-- Return SolveResult
  |
  |-- Phase 4.5: Update ContactCache with 3 lambdas per contact
  |
  |-- Phase 5: applyForces(inertial, solveResult)
  |
  |-- Phase 6: correctPositions(normalConstraintPtrs_)
  |     PositionCorrector operates on normal constraints only
```

---

## ECOS Removal

### Files to Delete

**Source files** (all in `msd/msd-sim/src/Physics/Constraints/ECOS/`):
- `ECOSData.hpp` / `ECOSData.cpp`
- `ECOSProblemBuilder.hpp` / `ECOSProblemBuilder.cpp`
- `ECOSSparseMatrix.hpp` / `ECOSSparseMatrix.cpp`
- `FrictionConeSpec.hpp` / `FrictionConeSpec.cpp`
- `CLAUDE.md`
- `CMakeLists.txt`

**Test files** (all in `msd/msd-sim/test/Physics/Constraints/ECOS/`):
- `FrictionConeSpecTest.cpp`
- `ECOSProblemBuilderTest.cpp`
- `ECOSSolveTest.cpp`
- `ECOSFrictionValidationTest.cpp`
- `CMakeLists.txt`

### Build System Changes

1. `msd/msd-sim/CMakeLists.txt`: Remove `find_package(ecos REQUIRED CONFIG)` and `ecos::ecos` from `target_link_libraries`
2. `msd/msd-sim/src/Physics/Constraints/CMakeLists.txt`: Remove `add_subdirectory(ECOS)`
3. `conanfile.py`: Remove `self.requires("ecos/2.0.10")`

### Header Changes

- `ConstraintSolver.hpp`: Remove `#include "ECOS/FrictionConeSpec.hpp"`
- `ConstraintSolver.cpp`: Remove all ECOS includes (`<ecos/ecos.h>`, `<ecos/glblopts.h>`, `"ECOS/ECOSData.hpp"`, `"ECOS/ECOSProblemBuilder.hpp"`, `"ECOS/FrictionConeSpec.hpp"`)

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `ECOS/FrictionConeSpecTest.cpp` | 11 tests | Deleted | Replaced by FrictionSpec tests |
| `ECOS/ECOSProblemBuilderTest.cpp` | ~15 tests | Deleted | Not replaced (ECOS-specific) |
| `ECOS/ECOSSolveTest.cpp` | ~11 tests | Deleted | Replaced by FrictionConeSolverTest |
| `ECOS/ECOSFrictionValidationTest.cpp` | ~12 tests | Deleted | Replaced by FrictionConeSolverTest |
| All existing physics tests | ~693 tests | No change | Zero regressions expected (no-friction path unchanged) |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| ConeProjection | Case 1: point inside cone | Returns input unchanged |
| ConeProjection | Case 2: point in dual cone | Returns zero vector |
| ConeProjection | Case 3: point outside cone | Projects to cone surface (Eq. 24-25) |
| ConeProjection | mu = 0 (frictionless) | Returns (max(p_n, 0), 0, 0) |
| ConeProjection | Continuity at Case 1/3 boundary | Projection and gradient continuous |
| ConeProjection | Continuity at Case 2/3 boundary | Projection and gradient continuous |
| ConeProjection | Gradient Case 1 | Returns I_3 |
| ConeProjection | Gradient Case 2 | Returns 0 |
| ConeProjection | Gradient Case 3 | Returns rank-2 matrix (Eq. 32) |
| ConeProjection | projectVector multi-contact | Projects each contact independently |
| FrictionConeSolver | M8 Ex1: single contact, mu=0 | Normal-only, lambda_n = 15.0 |
| FrictionConeSolver | M8 Ex2: single contact, sticking | Unconstrained optimum feasible |
| FrictionConeSolver | M8 Ex3: single contact, sliding | Cone surface projection |
| FrictionConeSolver | M8 Ex4: two contacts, different mu | Independent cone constraints |
| FrictionConeSolver | M8 Ex5: warm start convergence | Fewer iterations than cold start |
| FrictionConeSolver | M8 Ex6: grazing contact | No NaN, graceful zero |
| FrictionConeSolver | M8 Ex7: inclined plane | Stick/slip boundary |
| FrictionConeSolver | Cholesky failure recovery | Increases regularization, returns result |
| FrictionConeSolver | Max iterations cap | Returns best solution, converged = false |
| FrictionSpec | Construction | numContacts and coefficients stored correctly |

**Test file locations**:
- `msd/msd-sim/test/Physics/Constraints/ConeProjectionTest.cpp`
- `msd/msd-sim/test/Physics/Constraints/FrictionConeSolverTest.cpp`

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| ConstraintSolver friction dispatch | ConstraintSolver + FrictionConeSolver | Friction constraints route to FrictionConeSolver, no-friction routes to ASM |
| Constraint flattening correctness | ConstraintSolver | [CC, FC, CC, FC] -> [n, t1, t2, n, t1, t2] row ordering |
| extractBodyForcesFlat negative lambda | ConstraintSolver | Friction forces with negative lambda contribute to body forces |
| CollisionPipeline friction creation | CollisionPipeline | Creates both ContactConstraint and FrictionConstraint per contact |
| ContactCache 3-lambda round-trip | ContactCache + CollisionPipeline | Store 3 lambdas, retrieve 3 lambdas, matched by proximity |
| Position correction with friction | CollisionPipeline + PositionCorrector | Only normal constraints used for position correction |
| Energy monotonicity | WorldModel + full pipeline | Friction never injects energy (friction power <= 0) |

**Test file location**:
- `msd/msd-sim/test/Physics/Constraints/FrictionConeSolverIntegrationTest.cpp`

---

## Open Questions

### Design Decisions (Human Input Needed)

1. **Friction coefficient source for AssetInertial / AssetEnvironment**
   - Option A: Add `frictionCoefficient_` member to `AssetInertial` and `AssetEnvironment` with constructor parameter and accessor, defaulting to 0.5 -- Pros: per-object control, consistent with restitution. Cons: changes constructor signature, minor API change.
   - Option B: Use a global default `kDefaultFrictionCoefficient = 0.5` in CollisionPipeline, no asset changes -- Pros: zero API change. Cons: no per-object friction control.
   - Recommendation: Option A. Per-object friction is physically important (rubber vs. ice) and follows the restitution pattern already in the codebase. The constructor change is minor and matches existing conventions.

2. **PositionCorrector constraint filtering**
   - Option A: Pass only `normalConstraintPtrs_` (ContactConstraint pointers) to the corrector -- Pros: explicit, efficient. Cons: requires maintaining a separate vector.
   - Option B: Pass all constraints and let PositionCorrector filter internally via `dynamic_cast<ContactConstraint*>` -- Pros: no CollisionPipeline change for this. Cons: wastes time on casts, implicit filtering.
   - Recommendation: Option A. Building a separate `normalConstraintPtrs_` vector costs ~C pointer copies and makes the intent explicit. PositionCorrector already uses `dynamic_cast` internally but filtering at the call site is cleaner.

### Requirements Clarification

None. The math formulation and integration notes provide sufficient detail for implementation.

---

## Migration Plan

### Implementation Order

The design supports incremental implementation across subtasks 0052b through 0052d:

**Phase 1 (0052b): ConeProjection + Linear Algebra**
1. Implement `ConeProjection` header (project, gradient, projectVector)
2. Implement `FrictionSpec` header
3. Write `ConeProjectionTest.cpp` (all projection and gradient test cases)
4. All tests pass independently -- no integration with existing code

**Phase 2 (0052c): FrictionConeSolver Core**
1. Implement `FrictionConeSolver.hpp` / `.cpp` (Newton loop, Armijo, convergence)
2. Write `FrictionConeSolverTest.cpp` (all M8 numerical examples)
3. All tests pass independently -- solver takes `(A, b, mu, lambda0)` with no ConstraintSolver dependency

**Phase 3 (0052d): Integration + ECOS Removal**
1. Add `FrictionSpec.hpp` include to ConstraintSolver
2. Implement constraint flattening in ConstraintSolver
3. Implement `solveWithFriction()` dispatch
4. Implement `extractBodyForcesFlat()` (no negative-lambda skip)
5. Modify CollisionPipeline to create FrictionConstraints
6. Modify ContactCache for 3 lambdas per contact
7. Add friction coefficient to `AssetInertial` / `AssetEnvironment`
8. Build normalConstraintPtrs_ for PositionCorrector
9. Delete ECOS files, update CMakeLists, update conanfile
10. Write integration tests
11. Verify zero regressions on existing test suite (~693 tests)

**Phase 4 (0052e): Validation Suite**
1. Energy monotonicity tests
2. Convergence diagnostics
3. Warm-start effectiveness measurement
4. Known-analytical-solution tests (inclined plane, etc.)

### What Can Be Done Incrementally

- Phases 1 and 2 are completely independent of each other and of the existing codebase. They can be implemented and tested in parallel.
- Phase 3 is a single integration commit that replaces the ECOS path. It should be done atomically to avoid a state where ECOS is removed but the custom solver is not yet integrated.
- Phase 4 builds on Phase 3 and validates the full pipeline.
