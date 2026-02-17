# Design: NLopt Friction Cone Solver

## Summary

Replace the custom projected Newton solver (`FrictionConeSolver`) with NLopt for solving the friction cone constrained QP. The current solver fails to converge for cone-surface contacts during sustained tumbling, producing non-converged lambda that injects energy. A well-tested, general-purpose NLP solver will provide robust convergence without the fragility of maintaining a custom solver. This change introduces one new component (`NLoptFrictionSolver`), modifies one existing component (`ConstraintSolver`), and removes two obsolete components (`FrictionConeSolver`, `ConeProjection`).

## Architecture Changes

### PlantUML Diagram

See: `./0068_nlopt_friction_cone_solver.puml`

### New Components

#### NLoptFrictionSolver

- **Purpose**: NLopt-based solver for the friction cone QP. Takes the pre-assembled effective mass matrix, RHS vector, and per-contact friction coefficients, and returns the optimal impulse vector using SLSQP or other NLopt algorithms.
- **Header location**: `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.cpp`
- **Key interfaces**:
  ```cpp
  namespace msd_sim
  {

  class NLoptFrictionSolver
  {
  public:
    /// Algorithm selection for NLopt
    enum class Algorithm
    {
      SLSQP,          // Sequential Quadratic Programming (gradient-based, default)
      COBYLA,         // Constrained Optimization BY Linear Approximation (derivative-free)
      MMA,            // Method of Moving Asymptotes (gradient-based)
      AUGLAG_SLSQP    // Augmented Lagrangian with SLSQP sub-solver
    };

    /// Result of the friction cone solve
    struct SolveResult
    {
      Eigen::VectorXd lambda;           // Optimal impulse vector (3C x 1)
      bool converged{false};            // True if NLopt returned success/ftol_reached
      int iterations{0};                // Iterations performed (if available)
      double residual{std::numeric_limits<double>::quiet_NaN()};  // Final objective value
      double objective_value{std::numeric_limits<double>::quiet_NaN()};
      std::vector<double> constraint_violations;  // Per-contact cone constraint values
    };

    NLoptFrictionSolver();
    explicit NLoptFrictionSolver(Algorithm algo);
    ~NLoptFrictionSolver() = default;

    /// Solve the friction cone QP:
    ///   minimize f(lambda) = (1/2) lambda^T A lambda - b^T lambda
    ///   subject to mu_i^2 * lambda_n_i^2 - lambda_t1_i^2 - lambda_t2_i^2 >= 0
    ///              lambda_n_i >= 0
    ///
    /// @param A  Effective mass matrix (3C x 3C), symmetric positive definite
    /// @param b  RHS vector (3C x 1) with restitution/velocity terms
    /// @param mu Per-contact friction coefficients (C entries)
    /// @param lambda0  Warm-start vector (3C x 1). If empty or wrong size, cold start from zero.
    /// @return SolveResult with optimal lambda and diagnostics
    [[nodiscard]] SolveResult solve(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& b,
      const std::vector<double>& mu,
      const Eigen::VectorXd& lambda0 = Eigen::VectorXd{});

    void setTolerance(double tol) { tolerance_ = tol; }
    void setMaxIterations(int n) { max_iterations_ = n; }
    void setAlgorithm(Algorithm algo) { algorithm_ = algo; }

    [[nodiscard]] double getTolerance() const { return tolerance_; }
    [[nodiscard]] int getMaxIterations() const { return max_iterations_; }
    [[nodiscard]] Algorithm getAlgorithm() const { return algorithm_; }

    // Rule of Zero: no special member functions needed
    NLoptFrictionSolver(const NLoptFrictionSolver&) = default;
    NLoptFrictionSolver& operator=(const NLoptFrictionSolver&) = default;
    NLoptFrictionSolver(NLoptFrictionSolver&&) noexcept = default;
    NLoptFrictionSolver& operator=(NLoptFrictionSolver&&) noexcept = default;

  private:
    /// Objective function callback for NLopt: f(lambda) = (1/2) lambda^T A lambda - b^T lambda
    /// @param lambda Current iterate (3C x 1)
    /// @param grad Output gradient (3C x 1): grad = A*lambda - b
    /// @param data Pointer to ObjectiveData struct
    static double objective(const std::vector<double>& lambda,
                            std::vector<double>& grad,
                            void* data);

    /// Cone constraint callback for contact i: c_i(lambda) = mu^2*n^2 - t1^2 - t2^2
    /// @param lambda Current iterate (3C x 1)
    /// @param grad Output gradient (3 x 1): [2*mu^2*n, -2*t1, -2*t2]
    /// @param data Pointer to ConstraintData struct
    static double coneConstraint(const std::vector<double>& lambda,
                                 std::vector<double>& grad,
                                 void* data);

    /// Data passed to objective function
    struct ObjectiveData
    {
      const Eigen::MatrixXd* A;
      const Eigen::VectorXd* b;
    };

    /// Data passed to constraint function
    struct ConstraintData
    {
      int contactIndex;  // Which contact (0..C-1)
      double mu;         // Friction coefficient for this contact
    };

    double tolerance_{1e-8};
    int max_iterations_{100};
    Algorithm algorithm_{Algorithm::SLSQP};
  };

  }  // namespace msd_sim
  ```

- **Dependencies**: NLopt (via Conan), Eigen3 (`MatrixXd`, `VectorXd`)
- **Thread safety**: Thread-safe for concurrent calls to `solve()` (NLopt instance is local to the function). Configuration setters are not thread-safe.
- **Error handling**:
  - If NLopt returns failure codes (`NLOPT_INVALID_ARGS`, `NLOPT_OUT_OF_MEMORY`), throws `std::runtime_error`
  - If NLopt returns `NLOPT_MAXEVAL_REACHED` or `NLOPT_XTOL_REACHED`, returns `converged = false` with best solution
  - If `A` is not positive definite (detected by negative objective value trend), logs warning and returns best solution
  - Invalid `mu` values (< 0) are clamped to 0.0 with a warning

**Algorithm Details**:

**SLSQP (Default)**:
- Gradient-based Sequential Quadratic Programming
- Handles quadratic objectives efficiently
- Requires smooth constraints (our squared cone form is smooth)
- Convergence: Superlinear for well-conditioned problems
- Typical iterations: 5-20 for small contact sets (1-4 contacts)

**COBYLA (Fallback)**:
- Derivative-free constrained optimization
- Slower but more robust for ill-conditioned problems
- Useful if Hessian becomes singular (near-zero mass ratios)
- Typical iterations: 20-50

**MMA**:
- Method of Moving Asymptotes (gradient-based)
- Designed for large-scale problems
- Overkill for 3-12 variables but available for experimentation

**AUGLAG_SLSQP**:
- Augmented Lagrangian wrapper around SLSQP
- Handles equality/inequality constraints differently
- May provide better convergence for highly constrained problems

**Warm Starting**:

NLopt accepts an initial point via `nlopt::opt::optimize(x)`. The solver will:
1. Check if `lambda0` is non-empty and has correct size (3C)
2. If yes, copy `lambda0` to NLopt's `x` vector before calling `optimize()`
3. If no, initialize `x` to all zeros (cold start)

This enables warm-starting from `ContactCache` as in the current solver.

### Modified Components

#### ConstraintSolver

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` / `.cpp`
- **Changes required**:

  1. **Replace solver member**:
     ```cpp
     // OLD
     FrictionConeSolver frictionConeSolver_;

     // NEW
     NLoptFrictionSolver nloptSolver_;
     ```

  2. **Update solveWithFriction()** (private method):
     - No changes to constraint flattening, A/b assembly, or force extraction
     - Replace solver invocation:
       ```cpp
       // OLD
       auto result = frictionConeSolver_.solve(A, b, spec.frictionCoefficients, lambda0);

       // NEW
       auto nloptResult = nloptSolver_.solve(A, b, spec.frictionCoefficients, lambda0);

       // Map NLoptFrictionSolver::SolveResult to ActiveSetResult
       ActiveSetResult result;
       result.lambda = nloptResult.lambda;
       result.converged = nloptResult.converged;
       result.iterations = nloptResult.iterations;
       result.active_set_size = 0;  // Not applicable for NLopt
       ```

  3. **Remove FrictionConeSolver configuration**:
     - Delete `frictionConeSolver_.setTolerance()` and `frictionConeSolver_.setMaxIterations()` calls from constructor
     - Configuration now happens via `nloptSolver_.setTolerance()`, `nloptSolver_.setMaxIterations()`, `nloptSolver_.setAlgorithm()`

  4. **Update includes**:
     ```cpp
     // Remove
     #include "msd-sim/src/Physics/Constraints/FrictionConeSolver.hpp"

     // Add
     #include "msd-sim/src/Physics/Constraints/NLoptFrictionSolver.hpp"
     ```

- **Backward compatibility**: None affected. `ConstraintSolver` is an internal implementation detail of `CollisionPipeline`. Public API unchanged.

### Removed Components

#### FrictionConeSolver

- **Location**: `msd/msd-sim/src/Physics/Constraints/FrictionConeSolver.{hpp,cpp}`
- **Reason for removal**: Replaced by `NLoptFrictionSolver`. The custom projected Newton implementation is fragile and fails for cone-surface contacts.
- **Migration**: All call sites in `ConstraintSolver::solveWithFriction()` replace `frictionConeSolver_.solve()` with `nloptSolver_.solve()`. Interface is similar enough that the change is minimal.

#### ConeProjection

- **Location**: `msd/msd-sim/src/Physics/Constraints/ConeProjection.{hpp,cpp}`
- **Reason for removal**: Cone projection is now handled internally by NLopt's constraint satisfaction. The explicit projection utility is no longer needed.
- **Migration**: No call sites exist outside of `FrictionConeSolver`. Removing `FrictionConeSolver` makes this component obsolete.

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| NLoptFrictionSolver | ConstraintSolver | Member replacement | Drop-in replacement for `FrictionConeSolver` member |
| NLoptFrictionSolver | ContactCache | Warm-start via lambda0 | Existing warm-start mechanism unchanged |
| NLoptFrictionSolver | FrictionSpec | Data input | Unchanged friction coefficient API |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `test/Physics/Collision/LinearCollisionTest.cpp` | F1-F5 (friction tests) | Solver replacement | Verify convergence, energy conservation |
| `test/Physics/Collision/RotationalEnergyTest.cpp` | F4 (tumbling contact) | Bug fix target | Should now PASS with correct friction direction |
| `test/Replay/FrictionConeSolverTest.cpp` | Saturation direction test | Solver replacement | Should PASS with NLopt (ticket 0066 acceptance) |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| NLoptFrictionSolver | `solve_unconstrained_optimum` | Zero friction (mu=0) converges to unconstrained solution |
| NLoptFrictionSolver | `solve_cone_interior` | Small friction (mu=0.1) converges to interior solution |
| NLoptFrictionSolver | `solve_cone_surface` | High friction (mu=1.0) converges to cone surface |
| NLoptFrictionSolver | `solve_warm_start` | Warm-start from previous lambda reduces iterations |
| NLoptFrictionSolver | `solve_multiple_contacts` | 2-4 contact problem converges correctly |
| NLoptFrictionSolver | `algorithm_selection` | All algorithms (SLSQP, COBYLA, MMA, AUGLAG) produce similar results |
| NLoptFrictionSolver | `constraint_violation_diagnostic` | `constraint_violations` field accurately reports cone satisfaction |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `friction_energy_conservation` | NLoptFrictionSolver, ConstraintSolver, CollisionPipeline | Energy injection < 0.01 J/frame during sustained contact (ticket 0067) |
| `friction_direction_correctness` | NLoptFrictionSolver, ConstraintSolver, ContactCache | Friction opposes tangential velocity at saturation (ticket 0066) |
| `solver_convergence_regression` | NLoptFrictionSolver, ConstraintSolver | No regression in existing physics test pass rate (baseline: 734/741) |

#### Benchmark Tests

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| NLoptFrictionSolver | `solve_1_contact` | Time to solve 1-contact QP (3 vars) | < 50 μs (2x custom solver acceptable) |
| NLoptFrictionSolver | `solve_4_contacts` | Time to solve 4-contact QP (12 vars) | < 200 μs (2x custom solver acceptable) |
| NLoptFrictionSolver | `warm_start_speedup` | Iteration reduction from warm-start | > 30% fewer iterations |

## Dependency Changes

### Add NLopt via Conan

**Conan package**: `nlopt/2.7.1`

**conanfile.py changes**:
```python
def requirements(self):
    # ... existing dependencies
    self.requires("nlopt/2.7.1")
```

**CMakeLists.txt changes** (msd-sim):
```cmake
find_package(nlopt REQUIRED)

target_link_libraries(msd_sim
  PUBLIC
    # ... existing
  PRIVATE
    nlopt::nlopt
)
```

**License compatibility**: NLopt is dual-licensed under MIT and LGPL. We will use the MIT-licensed version, which is compatible with this project's license.

**Build verification**: After adding the dependency, verify that:
1. `conan install` successfully downloads and builds NLopt
2. CMake configuration succeeds with `find_package(nlopt)`
3. Test build of a minimal NLopt program succeeds

### Remove ECOS Dependency (Future Work)

**Not part of this ticket**. ECOS is still used by the legacy SOCP path in `ConstraintSolver`. After this ticket lands and is validated, a follow-up ticket will:
1. Remove the ECOS-based solver path from `ConstraintSolver`
2. Remove ECOS from `conanfile.py` dependencies
3. Remove ECOS-related configuration methods

This keeps the scope focused on NLopt integration without breaking the build if NLopt fails.

## Open Questions

### Design Decisions (Human Input Needed)

1. **Algorithm selection for default**
   - Option A: SLSQP — Gradient-based, fast convergence for smooth problems. Best choice for typical contact scenarios.
   - Option B: COBYLA — Derivative-free, more robust for singular cases but slower. Safer default if Hessian singularity is common.
   - Option C: Make configurable, default to SLSQP with COBYLA fallback on failure.
   - **Recommendation**: Option A (SLSQP default). The QP is well-conditioned for typical contact scenarios. COBYLA can be a future fallback if needed.

2. **Convergence tolerance**
   - Option A: Match current solver (1e-8) for direct comparison.
   - Option B: Relax to 1e-6 to allow faster convergence (NLopt may be more conservative).
   - Option C: Make configurable per-instance.
   - **Recommendation**: Option A (1e-8 initially). Can relax after validation if convergence is reliably achieved.

3. **Constraint formulation verification**
   - Question: Is the squared cone form `mu^2 * n^2 - t1^2 - t2^2 >= 0` numerically better than `sqrt(t1^2 + t2^2) - mu * n <= 0`?
   - Analysis: The squared form avoids the sqrt singularity at the apex (n=0, t=0) and has smooth gradients everywhere except the origin. This is the standard formulation for SOCPs.
   - **Recommendation**: Use squared form. It's well-established and avoids sqrt numerical issues.

4. **Performance regression threshold**
   - Question: What is the acceptable solver time increase over the custom solver?
   - Option A: 2x slower is acceptable (robustness > speed for contact solving).
   - Option B: No regression allowed (must match or beat custom solver).
   - **Recommendation**: Option A (2x acceptable). The custom solver is ~20-50 μs per contact. 2x slower (40-100 μs) is still negligible compared to frame time (16.67 ms @ 60 FPS).

### Prototype Required

1. **NLopt SLSQP convergence for cone-surface case**
   - Validate that SLSQP reliably converges when friction is saturated (||λ_t|| = μ * λ_n).
   - Test case: Tumbling cube on floor (ticket 0067 scenario).
   - Success criterion: Converged solution with energy injection < 0.01 J/frame.

2. **Warm-start effectiveness**
   - Measure iteration reduction when warm-starting from previous frame's lambda.
   - Test case: Steady-state contact (box resting on floor).
   - Success criterion: > 30% fewer iterations compared to cold start.

3. **Performance benchmark**
   - Measure solve time for 1-contact and 4-contact scenarios.
   - Compare against current `FrictionConeSolver` baseline.
   - Success criterion: < 2x slowdown.

### Requirements Clarification

1. **Iteration count limit**
   - Current solver uses 50 max iterations (increased from 20 in ticket 0067).
   - Should NLopt use 50, 100, or make configurable?
   - **Recommendation**: Start with 100 (NLopt may need more iterations for superlinear convergence). Make configurable for tuning.

2. **Constraint violation reporting**
   - Should `SolveResult` include per-contact constraint violations for debugging?
   - **Recommendation**: Yes. Add `std::vector<double> constraint_violations` to `SolveResult`. Useful for diagnosing non-convergence.

3. **Fallback behavior on non-convergence**
   - Current solver returns best-effort solution with `converged = false`.
   - Should NLopt do the same, or should it throw an exception?
   - **Recommendation**: Return best-effort solution (match current behavior). CollisionPipeline can log warnings for non-converged frames.

## Implementation Plan

### Phase 1: Add NLopt Dependency (1 hour)
- Add `nlopt/2.7.1` to `conanfile.py`
- Update `msd-sim/CMakeLists.txt` with `find_package(nlopt)` and link
- Verify build succeeds with `cmake --preset conan-debug && cmake --build --preset debug-sim-only`
- Write minimal NLopt test program to verify linkage

### Phase 2: Implement NLoptFrictionSolver (4 hours)
- Create `NLoptFrictionSolver.hpp` with class definition, enums, and `SolveResult` struct
- Create `NLoptFrictionSolver.cpp` with:
  - `solve()` main logic: create `nlopt::opt`, set objective, add constraints, set bounds, optimize
  - `objective()` static callback: compute `f = (1/2) lambda^T A lambda - b^T lambda` and gradient `grad = A*lambda - b`
  - `coneConstraint()` static callback: compute `c = mu^2*n^2 - t1^2 - t2^2` and gradient
- Add unit tests:
  - `solve_unconstrained_optimum` (mu=0)
  - `solve_cone_interior` (small mu)
  - `solve_cone_surface` (large mu)
  - `solve_warm_start` (verify iteration reduction)
  - `solve_multiple_contacts` (2-4 contacts)

### Phase 3: Integration with ConstraintSolver (2 hours)
- Replace `FrictionConeSolver frictionConeSolver_` with `NLoptFrictionSolver nloptSolver_` in `ConstraintSolver.hpp`
- Update `solveWithFriction()` in `ConstraintSolver.cpp`:
  - Replace solver call
  - Map `NLoptFrictionSolver::SolveResult` to `ActiveSetResult`
- Update includes
- Build and verify no compilation errors

### Phase 4: Verification (3 hours)
- Run existing physics tests: `cmake --build --preset debug-tests-only && ctest --preset debug`
  - Verify no regressions (baseline: 734/741 pass)
- Run replay test for ticket 0066 (saturation direction):
  - Verify friction opposes velocity at saturation
- Run energy injection test for ticket 0067 (tumbling contact):
  - Verify energy injection < 0.01 J/frame
- Benchmark solver performance:
  - 1-contact solve time
  - 4-contact solve time
  - Compare against `FrictionConeSolver` baseline
- Document results in `docs/designs/0068_nlopt_friction_cone_solver/verification-results.md`

### Phase 5: Cleanup (1 hour)
- Remove `FrictionConeSolver.{hpp,cpp}` and `ConeProjection.{hpp,cpp}`
- Remove includes/references in test files if any
- Update `msd/msd-sim/src/Physics/Constraints/CLAUDE.md` to document new solver
- Commit with message: `impl: replace FrictionConeSolver with NLoptFrictionSolver`

### Phase 6: Documentation Update (1 hour)
- Update `docs/designs/0068_nlopt_friction_cone_solver/design.md` with verification results
- Add architecture diagram comment to NLoptFrictionSolver.hpp
- Update root `CLAUDE.md` if relevant (unlikely, internal change)

**Total Estimated Time**: 12 hours over 2-3 days

## Acceptance Criteria

1. NLopt SLSQP (or chosen algorithm) reliably converges for cone-surface contacts (ticket 0067 acceptance)
2. Energy injection during sustained contact < 0.01 J/frame (ticket 0067 acceptance)
3. Friction direction correct at saturation (ticket 0066 acceptance)
4. No regression in existing physics tests (734/741 or better)
5. Solver performance within 2x of current `FrictionConeSolver` (40-100 μs per contact)
6. ECOS dependency can be removed after this lands (follow-up ticket)

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| NLopt convergence worse than custom solver | Low | High | Prototype phase validates convergence before integration |
| Performance regression > 2x | Medium | Medium | Benchmark early, consider COBYLA fallback if SLSQP slow |
| NLopt build issues on macOS/Linux | Low | Medium | Conan packages are well-maintained, fallback to manual build if needed |
| Breaking existing tests | Low | High | Integration phase runs full test suite before cleanup |
| Non-deterministic behavior from NLopt | Very Low | Low | NLopt is deterministic for given initial point and settings |

## Future Work

1. **Remove ECOS dependency** (follow-up ticket after validation)
   - Remove `solveWithECOS()` path from `ConstraintSolver`
   - Remove ECOS from Conan dependencies
   - Remove ECOS configuration methods

2. **Multi-algorithm fallback**
   - If SLSQP fails to converge, retry with COBYLA
   - Log algorithm switch for debugging

3. **Adaptive tolerance**
   - Relax tolerance for high-speed impacts (less critical)
   - Tighten tolerance for resting contacts (more critical)

4. **Sparse solver for large contact sets**
   - For > 10 contacts, consider sparse matrix formulation
   - NLopt may not be optimal for large-scale problems (revisit if needed)

## References

- [NLopt Documentation](https://nlopt.readthedocs.io/)
- [NLopt Algorithms](https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/)
- [SLSQP Algorithm](https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/#slsqp)
- [Coulomb Friction Cone](https://en.wikipedia.org/wiki/Friction#Coulomb_friction)
- Ticket 0052: Custom Friction Cone Solver (design context)
- Ticket 0066: Friction Cone Solver Saturation Bug (motivation)
- Ticket 0067: Contact Phase Energy Injection (motivation)
