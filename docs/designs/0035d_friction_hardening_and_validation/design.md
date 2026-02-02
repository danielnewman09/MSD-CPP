# Design: Friction Hardening and Validation

## Summary

This design hardens the friction constraint system with energy monitoring, stick-slip velocity thresholds, and regularization fallbacks. It then validates the complete system through the 6 M8 numerical examples as automated tests and benchmarks performance against the normal-only solver baseline. The design is primarily test-focused with minimal production code changes, ensuring the friction system is robust, physically correct, and production-ready.

**Parent Ticket**: [0035_friction_constraints](../../../tickets/0035_friction_constraints.md)
**Current Ticket**: [0035d_friction_hardening_and_validation](../../../tickets/0035d_friction_hardening_and_validation.md)

---

## Architecture Changes

### PlantUML Diagram
See: [`./0035d_friction_hardening_and_validation.puml`](./0035d_friction_hardening_and_validation.puml)

---

### New Components

#### EnergyMonitor (Test Utility)

- **Purpose**: Compute kinetic energy for test assertions validating M6 energy dissipation
- **Header location**: `msd/msd-sim/test/Physics/EnergyMonitor.hpp`
- **Source location**: `msd/msd-sim/test/Physics/EnergyMonitor.cpp`
- **Key interfaces**:
  ```cpp
  class EnergyMonitor {
  public:
      // Compute linear kinetic energy: 0.5 * sum(m_i * v_i^2)
      static double computeLinearKE(
          const std::vector<InertialState>& states,
          const std::vector<double>& masses);

      // Compute angular kinetic energy: 0.5 * sum(omega_i^T * I_i * omega_i)
      static double computeAngularKE(
          const std::vector<InertialState>& states,
          const std::vector<Eigen::Matrix3d>& inertias);

      // Compute total kinetic energy (linear + angular)
      static double computeTotalKE(
          const std::vector<InertialState>& states,
          const std::vector<double>& masses,
          const std::vector<Eigen::Matrix3d>& inertias);

      // Validate energy is non-increasing
      // Returns true if E_after <= E_before + tolerance
      static bool validateNonIncreasing(
          double E_before,
          double E_after,
          double tolerance = 1e-6);

      ~EnergyMonitor() = delete;  // Static-only utility
  };
  ```
- **Dependencies**: Eigen3 (for matrix operations), InertialState
- **Thread safety**: Pure functions, thread-safe
- **Error handling**: No exceptions (returns bool for validation)
- **Design rationale**:
  - Static-only class (no instance state)
  - Separate linear/angular energy for granular testing
  - Tolerance parameter allows per-test precision control
  - Zero runtime overhead (test-only compilation)

---

#### M8ScenarioBuilder (Test Utility)

- **Purpose**: Construct test scenarios corresponding to M8 numerical examples
- **Header location**: `msd/msd-sim/test/Physics/M8ScenarioBuilder.hpp`
- **Source location**: `msd/msd-sim/test/Physics/M8ScenarioBuilder.cpp`
- **Key interfaces**:
  ```cpp
  struct Scenario {
      // Initial conditions
      std::vector<AssetInertial> bodies;
      std::vector<ContactConstraint> contacts;
      Coordinate externalForce;
      double dt;

      // Expected final state
      double expectedNormalForce;
      double expectedFrictionForce;
      Coordinate expectedVelocity;
      double expectedAcceleration;

      // Tolerance specifications
      double forceTolerance;
      double velocityTolerance;
      double accelerationTolerance;
  };

  class M8ScenarioBuilder {
  public:
      // M8 Example 1: Static friction on inclined plane
      static Scenario createStaticFrictionIncline(
          double slope_angle,
          double friction_coeff);

      // M8 Example 2: Kinetic friction on inclined plane
      static Scenario createKineticFrictionIncline(
          double slope_angle,
          double friction_coeff);

      // M8 Example 3: Sliding deceleration on flat surface
      static Scenario createSlidingDeceleration(
          double initial_velocity,
          double friction_coeff);

      // M8 Example 4: Glancing collision with spin
      static Scenario createGlancingCollision(
          double impact_angle,
          double friction_coeff);

      // M8 Example 5: Friction cone saturation (stick-to-slip)
      static Scenario createConeSaturation(
          const std::vector<double>& applied_forces,
          double friction_coeff);

      // M8 Example 6: Two-body friction (Newton's third law)
      static Scenario createTwoBodyFriction(
          double applied_force,
          double friction_coeff);

      ~M8ScenarioBuilder() = delete;  // Static-only utility
  };
  ```
- **Dependencies**: AssetInertial, ContactConstraint, Coordinate
- **Thread safety**: Pure functions, thread-safe
- **Error handling**: Throws `std::invalid_argument` for invalid parameters (negative angles, coefficients)
- **Design rationale**:
  - Encapsulates M8 numerical example parameters
  - Centralizes expected values from hand computations
  - Scenario struct bundles all test data (inputs + outputs + tolerances)
  - Each factory method maps 1:1 to an M8 example

---

#### FrictionEnergyTest (Test Fixture)

- **Purpose**: Validate M6 energy dissipation properties
- **Location**: `msd/msd-sim/test/Physics/FrictionEnergyTest.cpp`
- **Test cases**:
  1. `EnergyMonotonicDecreaseForSliding` — Sliding deceleration shows E(t+1) ≤ E(t) for 1000 timesteps
  2. `EnergyConservationForStatic` — Static friction (stick regime) shows E(t+1) ≈ E(t) within floating-point error
  3. `EnergyNeverIncreases` — Random force applications never inject energy (Monte Carlo test, 100 trials)
- **Dependencies**: EnergyMonitor, M8ScenarioBuilder, ConstraintSolver
- **Design rationale**:
  - Directly validates M6 theorem (friction dissipates energy)
  - Long-duration tests (1000 timesteps) catch accumulation errors
  - Monte Carlo approach covers edge cases not in M8 examples

---

#### FrictionValidationTest (Test Fixture)

- **Purpose**: Validate all 6 M8 numerical examples as automated tests
- **Location**: `msd/msd-sim/test/Physics/FrictionValidationTest.cpp`
- **Test cases** (mapped from validation matrix in ticket):
  1. `StaticFrictionInclinedPlane` — M8 Example 1: v_t = 0, λ_t < μ·λ_n
  2. `KineticFrictionInclinedPlane` — M8 Example 2: a = g(sinθ - μ·cosθ) ± 5%
  3. `SlidingDeceleration` — M8 Example 3: stops at v₀²/(2μg) ± 5%
  4. `GlancingCollisionSpin` — M8 Example 4: ω = 16.90 rad/s ± 0.5
  5. `FrictionConeSaturation` — M8 Example 5: transition at F = 23.54 N
  6. `TwoBodyNewtonsThirdLaw` — M8 Example 6: λ_t,A = -λ_t,B
- **Dependencies**: M8ScenarioBuilder, ConstraintSolver, ContactConstraintFactory
- **Design rationale**:
  - One test case per M8 example (traceability)
  - Tolerances match M8 specifications (5% for approximate, 0.5 for precise)
  - Tests verify correct regime transitions (stick → slip)

---

#### FrictionStabilityTest (Test Fixture)

- **Purpose**: Validate M7 numerical stability mitigations
- **Location**: `msd/msd-sim/test/Physics/FrictionStabilityTest.cpp`
- **Test cases**:
  1. `RegularizationHandlesExtremeMassRatio` — Mass ratio 10⁶:1 converges within 10% of expected
  2. `VelocityThresholdPreventsJitter` — Object on incline shows zero velocity jitter over 1000 timesteps
  3. `StickSlipTransitionSmoothness` — Cone saturation test shows smooth transition (no oscillation)
- **Dependencies**: ConstraintSolver
- **Design rationale**:
  - Explicitly tests M7 degenerate cases
  - Jitter measurement via velocity standard deviation
  - Extreme mass ratios validate regularization fallback

---

#### FrictionBenchmark (Benchmark Fixture)

- **Purpose**: Measure friction solver performance vs. normal-only baseline
- **Location**: `msd/msd-sim/benchmark/FrictionBenchmark.cpp`
- **Benchmark cases**:
  1. `BM_SolveNormalOnly10Contacts` — Baseline: 10 contacts, normal constraints only
  2. `BM_SolveFriction10Contacts` — Friction: 10 contacts with friction (3x constraints)
  3. `BM_SolveFriction20Contacts` — Scaling: 20 contacts with friction
- **Dependencies**: Google Benchmark, ConstraintSolver, ContactConstraintFactory
- **Design rationale**:
  - Paired comparison (normal vs friction at same contact count)
  - Target: friction < 2x normal-only (AC6 from ticket)
  - Scaling test validates O(C³) complexity claim

---

### Modified Components

#### ConstraintSolver

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`, `ConstraintSolver.cpp`
- **Changes required**:
  1. **Add regularization fallback method** (private):
     ```cpp
     private:
       /**
        * @brief Apply regularization to ill-conditioned matrix
        * A_reg = A + epsilon * I
        * Triggered when LLT decomposition fails (non-positive-definite)
        * @param A Effective mass matrix (modified in-place)
        * @return true if regularization applied, false otherwise
        */
       bool applyRegularizationFallback(Eigen::MatrixXd& A) const;

       double regularization_epsilon_{1e-10};  // Default from M7
     ```
  2. **Add configuration setters/getters** (public):
     ```cpp
     public:
       void setRegularizationEpsilon(double epsilon);
       double getRegularizationEpsilon() const;
     ```
  3. **Modify `solveWithContacts()` to use regularization**:
     - Attempt LLT decomposition
     - If `llt.info() != Eigen::Success`, call `applyRegularizationFallback(A)`
     - Log warning when regularization triggered
     - Retry LLT decomposition on regularized matrix
     - If still fails, return `converged = false`
- **Backward compatibility**: Fully compatible (new private method, optional configuration)
- **Design rationale**:
  - Regularization is fallback-only (zero overhead for well-conditioned systems)
  - Epsilon configurable for sensitivity analysis
  - Logging enables debugging of degenerate cases

---

#### WorldModel (Collision Response Integration)

- **Current location**: `msd/msd-sim/src/Environment/WorldModel.hpp`, `WorldModel.cpp`
- **Changes required**:
  1. **Add velocity threshold post-processing** (private):
     ```cpp
     private:
       /**
        * @brief Clamp near-zero tangential velocities to prevent jitter
        * If ||v_t|| < v_rest, set v_t = 0 (force stick regime)
        * Applied after constraint solve, before state update
        * @param states Inertial states (modified in-place)
        * @param threshold Rest velocity threshold (default 0.01 m/s)
        */
       void applyVelocityThreshold(
           std::vector<std::reference_wrapper<InertialState>>& states,
           double threshold = 0.01) const;
     ```
  2. **Modify `updateCollisions()` to call velocity threshold**:
     - After `solver.solveWithContacts()` completes
     - Before applying constraint forces to body velocities
     - Apply `applyVelocityThreshold(states, velocity_rest_threshold_)`
  3. **Add member variable**:
     ```cpp
     private:
       double velocity_rest_threshold_{0.01};  // m/s, from M7
     ```
- **Backward compatibility**: Fully compatible (internal post-processing, no API change)
- **Design rationale**:
  - Post-processing approach avoids modifying LCP formulation
  - Threshold matches existing restitution rest velocity (Ticket 0032)
  - Applied in WorldModel collision handler (correct architectural level)

---

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| EnergyMonitor | InertialState | Data consumer | Reads position, velocity, angular velocity from state |
| EnergyMonitor | AssetInertial | Data consumer | Reads mass, inertia tensor |
| M8ScenarioBuilder | AssetInertial | Factory pattern | Constructs bodies with specific mass/inertia |
| M8ScenarioBuilder | ContactConstraintFactory | Factory pattern | Constructs friction constraints for scenario |
| FrictionEnergyTest | EnergyMonitor | Test utility | Calls energy computation in assertions |
| FrictionValidationTest | M8ScenarioBuilder | Test setup | Builds scenarios for each test case |
| FrictionStabilityTest | ConstraintSolver | Test target | Invokes solver with degenerate inputs |
| FrictionBenchmark | ConstraintSolver | Benchmark target | Measures `solveWithContacts()` wall time |
| ConstraintSolver::applyRegularizationFallback | Eigen::LLT | Algorithm | Detects decomposition failure via `.info()` |
| WorldModel::applyVelocityThreshold | InertialState | State mutation | Modifies tangential velocity component |

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `ConstraintSolverContactTest.cpp` | All contact solver tests | Regularization may change solve path | Monitor for convergence flag changes (should still pass) |
| `WorldModelTest.cpp` | Physics integration tests | Velocity threshold may clamp near-zero velocities, energy monitoring available | Add assertions for threshold behavior |

**Expected**: All existing tests pass with zero modifications. Regularization and velocity threshold are edge-case mitigations that should not affect typical test scenarios.

---

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| EnergyMonitor | `ComputeLinearKE_SingleBody` | Linear KE formula: 0.5·m·v² |
| EnergyMonitor | `ComputeAngularKE_SingleBody` | Angular KE formula: 0.5·ω^T·I·ω |
| EnergyMonitor | `ComputeTotalKE_MultipleBodies` | Sum over all bodies |
| EnergyMonitor | `ValidateNonIncreasing_Pass` | Returns true when E_after < E_before |
| EnergyMonitor | `ValidateNonIncreasing_Fail` | Returns false when E_after > E_before + tolerance |
| M8ScenarioBuilder | `CreateStaticFrictionIncline_ValidParams` | Correct initial conditions, expected values |
| M8ScenarioBuilder | `CreateKineticFrictionIncline_ValidParams` | Acceleration formula matches M8 |
| M8ScenarioBuilder | `CreateSlidingDeceleration_ValidParams` | Stopping distance formula matches M8 |
| M8ScenarioBuilder | `CreateInvalidAngle_Throws` | Negative angle throws std::invalid_argument |
| M8ScenarioBuilder | `CreateInvalidFrictionCoeff_Throws` | Negative μ throws std::invalid_argument |
| ConstraintSolver | `ApplyRegularizationFallback_ImprovesCond` | Condition number decreases after regularization |
| ConstraintSolver | `ApplyRegularizationFallback_LogsWarning` | Logging infrastructure captures warning message |
| WorldModel | `ApplyVelocityThreshold_ClampsNearZero` | ||v_t|| < 0.01 → v_t = 0 |
| WorldModel | `ApplyVelocityThreshold_PreservesLarge` | ||v_t|| > 0.01 → v_t unchanged |

**Total new unit tests**: 14

---

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `StaticFrictionInclinedPlane` | M8ScenarioBuilder, ConstraintSolver, ContactConstraintFactory | M8 Example 1: stick regime, force balance |
| `KineticFrictionInclinedPlane` | M8ScenarioBuilder, ConstraintSolver, ContactConstraintFactory | M8 Example 2: slip regime, acceleration |
| `SlidingDeceleration` | M8ScenarioBuilder, ConstraintSolver, EnergyMonitor | M8 Example 3: deceleration, stopping distance, energy monotonicity |
| `GlancingCollisionSpin` | M8ScenarioBuilder, ConstraintSolver | M8 Example 4: tangential impulse, angular velocity |
| `FrictionConeSaturation` | M8ScenarioBuilder, ConstraintSolver | M8 Example 5: stick-to-slip transition threshold |
| `TwoBodyNewtonsThirdLaw` | M8ScenarioBuilder, ConstraintSolver | M8 Example 6: action-reaction pairs |
| `EnergyMonotonicDecreaseForSliding` | M8ScenarioBuilder, EnergyMonitor, ConstraintSolver | Energy dissipation over 1000 timesteps |
| `EnergyConservationForStatic` | M8ScenarioBuilder, EnergyMonitor, ConstraintSolver | Energy conservation in stick regime |
| `EnergyNeverIncreases` | EnergyMonitor, ConstraintSolver | Monte Carlo: random forces never inject energy |
| `RegularizationHandlesExtremeMassRatio` | ConstraintSolver | Convergence with mass ratio 10⁶:1 |
| `VelocityThresholdPreventsJitter` | WorldModel, ConstraintSolver | Zero jitter on incline (1000 timesteps) |
| `StickSlipTransitionSmoothness` | M8ScenarioBuilder, ConstraintSolver | Smooth transition without oscillation |

**Total integration tests**: 12

---

#### Benchmark Tests

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| ConstraintSolver | `BM_SolveNormalOnly10Contacts` | 10 contacts, normal constraints only | Baseline reference time |
| ConstraintSolver | `BM_SolveFriction10Contacts` | 10 contacts with friction (30 constraints) | < 2x baseline (AC6) |
| ConstraintSolver | `BM_SolveFriction20Contacts` | 20 contacts with friction (60 constraints) | < 8x baseline (cubic scaling) |

**Total benchmark tests**: 3

**Baseline expectation notes**:
- Friction constraint count: 3x normal-only (normal + 2 tangent constraints per contact)
- Expected complexity: O(C³) due to LLT decomposition
- Performance target (AC6): friction < 2x wall time of normal-only at same contact count
- This is achievable because friction uses box-constrained ASM which has similar per-iteration cost to normal-only ASM, with slightly higher iteration count due to variable friction bounds

---

## Open Questions

### Design Decisions (Human Input Needed)

None. The design is fully specified based on M6, M7, M8 mathematical formulation and existing codebase patterns.

### Prototype Required

None. Mathematical formulation complete, implementation is straightforward validation/testing work.

### Requirements Clarification

None. Ticket requirements are unambiguous and directly mapped to M6, M7, M8 mathematical analysis.

---

## Implementation Notes

### Energy Monitoring Performance

**Zero runtime overhead**: `EnergyMonitor` is compiled only in test executables (not linked into production library). No conditional compilation needed — standard CMake test target separation.

### Regularization Trigger Frequency

**Expected**: < 1% of solves for typical scenarios. Only triggered for:
- Degenerate contact geometry (nearly parallel normals)
- Extreme mass ratios (> 10⁶:1)
- Numerically singular configurations

**Logging**: Warning logged each time regularization applied. Production deployments should monitor log frequency to detect systematic issues.

### Velocity Threshold Interaction with Restitution

**No conflict**: Restitution applies to normal velocity, threshold applies to tangential velocity. Both use same 0.01 m/s threshold value for consistency (established in Ticket 0032).

### Test Execution Time

**Estimated**:
- Unit tests (14): < 50ms total (analytical computations)
- Integration tests (12): ~5-10 seconds total (1000 timestep simulations)
- Benchmark tests (3): ~30 seconds total (statistical repetitions)

**CI impact**: Negligible. Integration tests run only when friction code modified (not on every commit). Benchmarks are optional (manual execution for performance validation).

### M8 Example Tolerances

Tolerances from M8 document:
- Force/impulse comparisons: ± 0.1 N or 1% (whichever is larger)
- Velocity comparisons: ± 1e-6 m/s (zero velocity), ± 5% (non-zero velocity)
- Acceleration comparisons: ± 5%
- Angular velocity comparisons: ± 0.5 rad/s
- Energy comparisons: ± 1e-6 J per timestep (accumulation tolerance)

Rationale: Discrete timestep integration introduces ~1-5% error vs. analytical continuous solutions. Tolerances account for numerical integration error without masking physics bugs.

---

## File Manifest

### New Files

| File | Purpose | Lines of Code (Est.) |
|------|---------|---------------------|
| `msd-sim/test/Physics/EnergyMonitor.hpp` | Energy computation utility | ~50 |
| `msd-sim/test/Physics/EnergyMonitor.cpp` | Energy computation implementation | ~80 |
| `msd-sim/test/Physics/M8ScenarioBuilder.hpp` | Scenario factory interface | ~80 |
| `msd-sim/test/Physics/M8ScenarioBuilder.cpp` | Scenario factory implementation | ~400 (6 scenarios × ~70 LOC each) |
| `msd-sim/test/Physics/FrictionEnergyTest.cpp` | Energy dissipation tests | ~200 |
| `msd-sim/test/Physics/FrictionValidationTest.cpp` | M8 numerical examples as tests | ~400 (6 tests × ~70 LOC each) |
| `msd-sim/test/Physics/FrictionStabilityTest.cpp` | Stability/degenerate case tests | ~200 |
| `msd-sim/benchmark/FrictionBenchmark.cpp` | Performance benchmarks | ~150 |

**Total new code**: ~1560 lines (primarily test infrastructure)

### Modified Files

| File | Changes | Lines Changed (Est.) |
|------|---------|---------------------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Add regularization methods, member variable | +15 |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Implement regularization fallback, modify `solveWithContacts()` | +40 |
| `msd-sim/src/Environment/WorldModel.cpp` | Add velocity threshold post-processing | +25 |

**Total modified code**: ~80 lines (production code)

---

## Acceptance Criteria Mapping

| AC | Test Case(s) | Validation Method |
|----|--------------|-------------------|
| AC1 | `EnergyMonotonicDecreaseForSliding` | Assert E(t+1) ≤ E(t) + 1e-6 for all timesteps in sliding deceleration scenario |
| AC2 | `VelocityThresholdPreventsJitter` | Measure velocity std dev over 1000 timesteps, assert < 1e-6 m/s |
| AC3 | `FrictionConeSaturation` | Assert transition at F = 23.54 N ± 0.1 N (M8 Example 5) |
| AC4 | `TwoBodyNewtonsThirdLaw` | Assert λ_t,A + λ_t,B < 1e-6 N (M8 Example 6) |
| AC5 | `RegularizationHandlesExtremeMassRatio` | Mass ratio 10⁶:1 converges, result within 10% of expected |
| AC6 | `BM_SolveFriction10Contacts` | Wall time < 2x `BM_SolveNormalOnly10Contacts` |
| AC7 | All 6 `FrictionValidationTest` cases | Each M8 example passes within specified tolerance |
| AC8 | Existing test suite | Run full test suite, assert 100% pass rate |

**Traceability**: Each AC maps to at least one automated test case. AC7 (all M8 examples) is implicitly validated by AC1-AC4 plus the two remaining validation tests.

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-01
**Status**: REVISION_REQUESTED
**Iteration**: 0 of 1

### Issues Requiring Revision

| ID | Issue | Category | Required Change |
|----|-------|----------|-----------------|
| I1 | CollisionResponse component does not exist in codebase | Architectural Fit | Replace references to `CollisionResponse` with `WorldModel` (which handles collision integration) |
| I2 | PlantUML diagram shows non-existent CollisionResponse class | Architectural Fit | Update diagram to show WorldModel as modified component instead of CollisionResponse |

### Revision Instructions for Architect

The following changes must be made before final review:

1. **Issue I1: CollisionResponse component reference**:
   - The design document references `CollisionResponse` as a modified component in the "Modified Components" section
   - Per msd-sim/CLAUDE.md, `CollisionResponse` namespace was removed in ticket 0032d_collision_response_cleanup (2026-01-31)
   - Collision response is now handled entirely by `WorldModel::updateCollisions()` using the constraint framework
   - **Required change**: Replace all references to `CollisionResponse` with `WorldModel`
   - Update the "Modified Components" section to show WorldModel instead of CollisionResponse
   - Update the Integration Points table entry for velocity threshold from "CollisionResponse::applyVelocityThreshold" to "WorldModel::applyVelocityThreshold"
   - Update file manifest to show `msd-sim/src/Environment/WorldModel.cpp` instead of `msd-sim/src/Physics/CollisionResponse.cpp`

2. **Issue I2: PlantUML diagram component**:
   - The PlantUML diagram shows `CollisionResponse <<modified>>` in the "msd-sim::Physics::Constraints" package
   - This class does not exist (removed in ticket 0032d)
   - **Required change**: Update the diagram to show `WorldModel <<modified>>` in the "msd-sim::Environment" package instead
   - Ensure the note describes velocity threshold post-processing correctly
   - Ensure relationships show WorldModel::applyVelocityThreshold modifying InertialState, not CollisionResponse

### Items Passing Review (No Changes Needed)

The following aspects of the design are approved and should NOT be modified:

- **Test infrastructure design**: EnergyMonitor, M8ScenarioBuilder, test fixtures all well-designed and appropriate
- **ConstraintSolver modifications**: Regularization fallback approach is sound and follows existing patterns
- **Test coverage**: Complete coverage of M6, M7, M8 validation requirements with proper traceability
- **Performance benchmarking**: Appropriate paired comparison methodology for AC6 validation
- **C++ design quality**: Follows project standards (Rule of Zero, brace initialization, static utilities)
- **Naming conventions**: All components follow PascalCase/camelCase/snake_case_ conventions correctly
- **Memory management**: Appropriate use of static utilities, value semantics, no shared_ptr
- **Error handling**: Appropriate exception types (std::invalid_argument for validation errors)
- **Tolerances**: M8 example tolerances properly justified based on numerical integration error
- **File organization**: Test files properly located in msd-sim/test/Physics/, benchmarks in msd-sim/benchmark/
- **Build integration**: Relies on standard CMake test target separation, no special build configuration needed
- **Acceptance criteria mapping**: Complete and accurate mapping from ACs to test cases

---

## Architect Revision Notes

**Date**: 2026-02-01
**Responding to**: Design Review — Initial Assessment

### Changes Made

| Issue ID | Original | Revised | Rationale |
|----------|----------|---------|-----------|
| I1 | Modified Components section referenced `CollisionResponse` | Changed to `WorldModel (Collision Response Integration)` | `CollisionResponse` class was removed in ticket 0032d. Velocity threshold post-processing belongs in `WorldModel::updateCollisions()` which handles collision integration using the constraint framework. |
| I2 | PlantUML diagram showed `CollisionResponse <<modified>>` in "msd-sim::Physics::Constraints" package | Changed to `WorldModel <<modified>>` in "msd-sim::Environment" package | Corrected to show actual component that exists in codebase. WorldModel is the collision integration layer that applies constraint forces. |

### Diagram Updates

- Replaced `CollisionResponse <<modified>>` with `WorldModel <<modified>>`
- Moved WorldModel to correct package: "msd-sim::Environment" (not "msd-sim::Physics::Constraints")
- Updated relationship: `WorldModel ..> ConstraintSolver : uses` (WorldModel uses ConstraintSolver, not bidirectional)
- Preserved velocity threshold note: "Post-processing after LCP solve, If ||v_t|| < v_rest: clamp v_t = 0, Prevents stick-slip jitter"

### Unchanged (Per Reviewer Guidance)

- Test infrastructure design (EnergyMonitor, M8ScenarioBuilder, test fixtures)
- ConstraintSolver regularization fallback implementation
- Test coverage plan and traceability matrix
- Performance benchmarking methodology
- C++ design quality, naming conventions, memory management
- Error handling approach
- M8 example tolerances
- File organization and build integration
- Acceptance criteria mapping

### Note on Design Document Consistency

The design document's "Modified Components" section already correctly referenced `WorldModel (Collision Response Integration)` rather than `CollisionResponse`. The issue was isolated to the PlantUML diagram, which has now been corrected to match the design document text.

---

## Design Review — Final Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-02-01
**Status**: APPROVED
**Iteration**: 1 of 1

### Revision Verification

All issues from the initial assessment have been successfully addressed:

| Issue ID | Status | Verification |
|----------|--------|--------------|
| I1 | ✓ Resolved | Design document Modified Components section correctly references `WorldModel (Collision Response Integration)`. All references to non-existent `CollisionResponse` removed. |
| I2 | ✓ Resolved | PlantUML diagram now shows `WorldModel <<modified>>` in "msd-sim::Environment" package. Relationships updated to show `WorldModel ..> ConstraintSolver : uses`. |

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ Pass | All components follow project patterns (EnergyMonitor, M8ScenarioBuilder, FrictionEnergyTest) |
| Namespace organization | ✓ Pass | Test utilities in test/, benchmarks in benchmark/, production code in src/ |
| File structure | ✓ Pass | Follows msd/msd-sim/test/Physics/ and msd/msd-sim/benchmark/ patterns |
| Dependency direction | ✓ Pass | Test code depends on production code, not vice versa. WorldModel correctly uses ConstraintSolver. |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ Pass | Appropriate for test utilities (static-only classes, no resource management needed) |
| Smart pointer appropriateness | ✓ Pass | No smart pointers needed (static utilities, value semantics for Scenario struct) |
| Value/reference semantics | ✓ Pass | Scenario struct uses value semantics, test utilities use static methods |
| Rule of 0/3/5 | ✓ Pass | Static utilities use `= delete` destructor, Scenario uses compiler-generated defaults |
| Const correctness | ✓ Pass | Static methods take const references where appropriate |
| Exception safety | ✓ Pass | M8ScenarioBuilder throws std::invalid_argument for invalid input, EnergyMonitor is noexcept |
| Initialization | ✓ Pass | Brace initialization used throughout, NaN for uninitialized floats (regularization_epsilon_ initialized to 1e-10) |
| Return values | ✓ Pass | Functions return values/structs (Scenario) rather than output parameters |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ Pass | Test utilities depend on InertialState, AssetInertial, ConstraintSolver (all exist) |
| Template complexity | ✓ Pass | No templates used |
| Memory strategy | ✓ Pass | Stack allocation for test scenarios, no dynamic allocation in test utilities |
| Thread safety | ✓ Pass | Static pure functions are thread-safe |
| Build integration | ✓ Pass | Standard CMake test targets, no special configuration required |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ Pass | Test utilities are standalone, can be tested independently |
| Mockable dependencies | ✓ Pass | Not applicable (test infrastructure) |
| Observable state | ✓ Pass | EnergyMonitor returns values for assertions, M8ScenarioBuilder builds observable Scenario structs |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Velocity threshold may be too aggressive, clamping velocities that should slide slowly | Technical | Low | Low | Threshold value (0.01 m/s) matches existing restitution rest velocity. Can be tuned via member variable if needed. | No |
| R2 | Regularization may mask underlying numerical issues in solver | Technical | Low | Low | Regularization logs warning each time triggered. Production deployments can monitor log frequency. Expected < 1% of solves. | No |
| R3 | M8 example tolerances (5%) may be too loose, masking physics bugs | Technical | Low | Medium | Tolerances justified by discrete timestep integration error. Can be tightened if integration tests pass with margin. | No |

**Risk Summary**: All risks are Low likelihood and Low-Medium impact with clear mitigation strategies. No prototypes required.

### Prototype Guidance

No prototypes required. Design is straightforward test infrastructure with minimal production code changes (80 LOC). Mathematical formulation (M6, M7, M8) is complete and provides hand-verified numerical examples.

### Summary

The design successfully addresses all initial review issues. The architecture is sound, follows project coding standards, and provides comprehensive test coverage for friction hardening and validation. The design correctly identifies:

1. **WorldModel** as the collision response integration layer where velocity threshold post-processing belongs
2. **ConstraintSolver** as the location for regularization fallback
3. Complete test infrastructure for M6 (energy dissipation), M7 (numerical stability), and M8 (numerical examples) validation

The design is **approved** and ready for human review. After human approval, implementation can proceed directly without prototyping (mathematical formulation complete, implementation is straightforward).

**Next Steps**:
1. Human reviews design (should be quick approval, no open questions)
2. Advance to "Ready for Implementation"
3. Implement test infrastructure and hardening features (estimated 1560 LOC tests + 80 LOC production code)
4. Execute quality gate (run test suite, benchmarks)
5. Documentation update phase

---
