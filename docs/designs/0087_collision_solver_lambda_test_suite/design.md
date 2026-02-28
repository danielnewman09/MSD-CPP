# Design: Collision Solver Lambda Test Suite (0087)

## Summary

This design introduces a bottom-up verification chain for the collision pipeline by decomposing existing multi-frame end-to-end tests into isolated single-step tests that inspect solver lambda outputs directly. The work is organized into five stages:

- **Stage A (0087a)**: Test infrastructure — `CollisionScenario` and `CollisionScenarioBuilder`
- **Stage B (0087b)**: Normal impulse lambda tests (6+ tests)
- **Stage C (0087c)**: Friction impulse lambda tests (6+ tests, Coulomb cone verification)
- **Stage D (0087d)**: Position correction magnitude and energy tests (6+ tests)
- **Stage E (0087e)**: Single-frame pipeline integration tests (4+ tests)

No production solver code is modified. The only production code change is a single `friend class CollisionScenario` declaration on `CollisionPipeline`, following the pattern already established by `friend class CollisionPipelineTest`.

---

## Architecture Changes

### PlantUML Diagram

See: [`./0087_collision_solver_lambda_test_suite.puml`](./0087_collision_solver_lambda_test_suite.puml)

---

### New Components

#### CollisionScenario

- **Purpose**: Owns the assets, environments, and pipeline for a single isolated collision scenario. Provides `stepOnce()` which runs the pipeline's sub-phases and returns the `ConstraintSolver::SolveResult` for direct lambda inspection.
- **Header location**: `msd-sim/test/Helpers/CollisionScenario.hpp`
- **Source location**: `msd-sim/test/Helpers/CollisionScenario.cpp`
- **Key interface**:
  ```cpp
  namespace msd_sim::test
  {

  class CollisionScenario
  {
  public:
    explicit CollisionScenario(double dt = 0.016);

    CollisionScenario(const CollisionScenario&) = delete;
    CollisionScenario& operator=(const CollisionScenario&) = delete;
    CollisionScenario(CollisionScenario&&) noexcept = default;
    CollisionScenario& operator=(CollisionScenario&&) noexcept = default;
    ~CollisionScenario() = default;

    // Spawn helpers — return index for later state access
    size_t addInertial(const std::vector<Coordinate>& points,
                       const ReferenceFrame& frame,
                       double mass,
                       double restitution = 0.0,
                       double friction = 0.5,
                       const Coordinate& velocity = Coordinate{0, 0, 0});

    size_t addEnvironment(const std::vector<Coordinate>& points,
                          const ReferenceFrame& frame);

    // Run one pipeline step and capture the SolveResult.
    // applyForces: if true, also calls applyForces() after solving
    // correctPositions: if true, also calls correctPositions() after solving
    ConstraintSolver::SolveResult stepOnce(bool applyForces = true,
                                           bool correctPositions = false);

    // State access for post-step verification
    const InertialState& getInertialState(size_t idx) const;
    const CollisionPipeline& pipeline() const;
    bool hadCollisions() const;

  private:
    double dt_;
    std::vector<AssetInertial> inertials_;
    std::vector<AssetEnvironment> environments_;
    CollisionPipeline pipeline_;
    ConstraintSolver::SolveResult lastResult_;
  };

  }  // namespace msd_sim::test
  ```
- **Dependencies**: `CollisionPipeline`, `AssetInertial`, `AssetEnvironment`, `ConvexHull`, `ReferenceFrame`, `ConstraintSolver::SolveResult`
- **Thread safety**: Not thread-safe; single-threaded test use only
- **Error handling**: Assertions on invalid indices; pipeline errors propagate normally

**Design rationale**: `CollisionScenario` is a lightweight value-type test fixture that owns its state. The `stepOnce()` method calls the protected pipeline sub-phases in sequence (via `friend` access) and captures the `SolveResult` before it would otherwise be discarded. This is the minimal surface needed to expose lambdas without making `CollisionPipeline` broadly public.

#### CollisionScenarioBuilder

- **Purpose**: Static factory methods that construct common collision setups, eliminating copy-paste boilerplate across the many individual lambda tests.
- **Header location**: `msd-sim/test/Helpers/CollisionScenarioBuilder.hpp`
- **Source location**: header-only (all methods return by value, simple enough)
- **Key interface**:
  ```cpp
  namespace msd_sim::test
  {

  class CollisionScenarioBuilder
  {
  public:
    // Sphere-like object (small cube approximation from existing test DB)
    // dropped onto a flat floor at z=0
    static CollisionScenario sphereOnFloor(
      double height,
      double velocityZ,
      double mass,
      double restitution,
      double friction,
      double dt = 0.016);

    // Cube (unit_cube geometry) placed/dropped onto floor
    static CollisionScenario cubeOnFloor(
      double height,
      double velocityZ,
      double velocityX,  // for friction tests
      double mass,
      double restitution,
      double friction,
      double dt = 0.016);

    // Two cubes approaching each other along X axis (zero gravity context)
    static CollisionScenario twoCubes(
      double separation,
      double velA,   // x-velocity of body A (positive = toward B)
      double velB,   // x-velocity of body B (negative = toward A)
      double massA,
      double massB,
      double restitution,
      double friction,
      double dt = 0.016);

    // Two cubes with controlled overlap (for penetration depth tests)
    static CollisionScenario overlappingCubes(
      double overlap,
      double mass,
      double restitution,
      double friction,
      double dt = 0.016);
  };

  }  // namespace msd_sim::test
  ```
- **Dependencies**: `CollisionScenario`
- **Note on geometry**: Builders use hardcoded unit cube geometry (8 vertices at ±0.5) and floor-slab geometry (matching existing test patterns). They do not require the test asset database, making them self-contained and fast.

#### LambdaAssertions

- **Purpose**: Reusable assertion helpers for verifying solver lambda structure, values, and Coulomb cone compliance. Centralizes tolerance logic and error messages.
- **Header location**: `msd-sim/test/Helpers/LambdaAssertions.hpp`
- **Source location**: header-only
- **Key interface**:
  ```cpp
  namespace msd_sim::test
  {

  /// Assert that the solver converged
  void assertConverged(const ConstraintSolver::SolveResult& result,
                       const std::string& context = "");

  /// Assert lambda at index idx >= 0 (non-negative normal impulse)
  void assertNonNegativeNormal(const ConstraintSolver::SolveResult& result,
                               int lambdaIdx,
                               const std::string& context = "");

  /// Assert normal lambda is within tolerance of expected value
  /// lambdaIdx: index into result.lambdas for the normal component
  void assertNormalLambda(const ConstraintSolver::SolveResult& result,
                          int lambdaIdx,
                          double expectedLambda,
                          double tol,
                          const std::string& context = "");

  /// Assert Coulomb cone compliance: ||[t1, t2]|| <= mu * n
  /// normalIdx: index of lambda_n in result.lambdas
  /// tangentIdx: index of lambda_t1 (lambda_t2 = tangentIdx + 1)
  void assertCoulombCone(const ConstraintSolver::SolveResult& result,
                         int normalIdx,
                         int tangentIdx,
                         double frictionCoeff,
                         const std::string& context = "");

  /// Assert Block PGS output has exactly numContacts * 3 lambda values
  /// (one [n, t1, t2] triplet per contact)
  void assertBlockPGSLayout(const ConstraintSolver::SolveResult& result,
                             int numContacts,
                             const std::string& context = "");

  }  // namespace msd_sim::test
  ```
- **Dependencies**: `ConstraintSolver::SolveResult`, `<gtest/gtest.h>`
- **Note**: Uses `EXPECT_*` rather than `ASSERT_*` so failures do not abort test execution mid-way, allowing all lambda violations to be reported in a single run.

---

### Modified Components

#### CollisionPipeline

- **Current location**: `msd-sim/src/Physics/Collision/CollisionPipeline.hpp`
- **Changes required**: Add `friend class CollisionScenario;` to the `private:` section's existing friend declarations, immediately after `friend class CollisionPipelineTest;`
- **Backward compatibility**: No behavior change; pure access grant. The change is one line.

#### msd-sim/test/Physics/Collision/CMakeLists.txt

- **Changes required**: Add the four new stage test files (`SolverLambdaTest.cpp`, `FrictionLambdaTest.cpp`, `PositionCorrectionTest.cpp`, `PipelineIntegrationTest.cpp`) to `target_sources`.
- **New CMakeLists entry required**: `msd-sim/test/Helpers/` directory needs a `CMakeLists.txt` that adds `CollisionScenario.cpp` to `target_sources`. Alternatively, a `msd-sim/test/Helpers/CMakeLists.txt` can be included from the parent test `CMakeLists.txt`.

---

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---|---|---|---|
| `CollisionScenario` | `CollisionPipeline` | `friend` access for protected sub-phases | One line change to production header |
| `CollisionScenario` | `ConstraintSolver::SolveResult` | Returns by value | Read-only inspection; no ownership issues |
| `CollisionScenarioBuilder` | `CollisionScenario` | Factory creates scenario | All geometry is hardcoded (no DB dependency) |
| `LambdaAssertions` | `ConstraintSolver::SolveResult` | Const-ref inspection | Only uses `result.lambdas`, `result.converged` |
| `SolverLambdaTest` | `CollisionScenarioBuilder` + `LambdaAssertions` | Test uses both | Stage B test file |
| `FrictionLambdaTest` | `CollisionScenarioBuilder` + `LambdaAssertions` | Test uses both | Stage C test file |
| `PositionCorrectionTest` | `CollisionScenarioBuilder` | Test uses builder | Stage D test file |
| `PipelineIntegrationTest` | `CollisionScenarioBuilder` + `LambdaAssertions` | Test uses both | Stage E test file |

---

## Analytical Expected Lambda Values

The following formulas define the expected lambda values that tests will assert against. These are documented here so the implementer can derive test tolerances without re-deriving the physics.

### Normal Impulse (Stage B)

For a single body A colliding with a static floor (body B = infinite mass), the constraint solver computes a normal impulse λ_n such that the post-collision normal velocity satisfies v_n_after = -e · v_n_before.

**Analytical formula** (one contact, no friction):

```
lambda_n = m * (1 + e) * |v_n_before| / (J * M^-1 * J^T * dt)
         ≈ m * (1 + e) * |v_n| for unit effective mass
```

In the solver's LCP formulation, λ has units of N·s (impulse), and:

```
lambda_n = m_eff * (1 + e) * |v_n|
```

where `m_eff = 1 / (J M^-1 J^T)`. For a unit mass sphere against a static floor with a purely vertical contact normal, `m_eff ≈ mass`.

**Specific cases**:

| Test Case | e | Expected λ_n (approx) | Tolerance guidance |
|---|---|---|---|
| Perfectly inelastic (e=0) | 0.0 | `m * |v_z|` | ±10% (discretization) |
| Partial restitution (e=0.7) | 0.7 | `m * 1.7 * |v_z|` | ±10% |
| Perfectly elastic (e=1.0) | 1.0 | `m * 2 * |v_z|` | ±10% |
| Resting contact (v_z≈0) | any | `≥ 0` (gravity balanced) | non-negative |

**Note**: The solver's RHS includes a Baumgarte restitution term, so the exact value is slightly different from the pure-impulse formula. Tests should use `EXPECT_NEAR` with ±15% tolerance for the impact cases, and `EXPECT_GE(lambda, 0)` for resting contact. Tolerances can be tightened empirically during Stage B development.

### Friction Impulse (Stage C)

For a sliding contact with Coulomb friction:

**Saturation (kinetic friction)**:
```
||[lambda_t1, lambda_t2]|| = mu * lambda_n   (cone boundary)
```

**Static friction (inside cone)**:
```
||[lambda_t1, lambda_t2]|| < mu * lambda_n
```

**Zero friction (mu=0)**:
```
lambda_t1 = lambda_t2 = 0
```

The direction of the tangent impulse opposes the sliding velocity direction.

### Position Correction (Stage D)

```
correction_displacement = beta * max(depth - slop, 0)
```

With default config: `beta = 0.2`, `slop = 0.005m`.

For depth = 0.05m:
```
correction ≈ 0.2 * (0.05 - 0.005) = 0.009m per iteration (max 4 iterations)
```

The split-impulse guarantee: position correction pseudo-velocities are never accumulated into the real velocity vector. After `correctPositions()`, the body's `InertialState::velocity` must be unchanged.

---

## Test Organization

### Stage A: Infrastructure (0087a)

**Files**:
- `msd-sim/test/Helpers/CollisionScenario.hpp` / `.cpp`
- `msd-sim/test/Helpers/CollisionScenarioBuilder.hpp`
- `msd-sim/test/Helpers/LambdaAssertions.hpp`

**Acceptance gate**: At least 3 existing test scenarios in `LinearCollisionTest.cpp` or `CollisionPipelineTest.cpp` can be rewritten using `CollisionScenarioBuilder` in fewer lines of code, proving the boilerplate reduction.

### Stage B: Normal Impulse Lambda Tests (0087b)

**File**: `msd-sim/test/Physics/Collision/SolverLambdaTest.cpp`

**Test class**: `SolverLambdaTest : public ::testing::Test` (uses `CollisionScenarioBuilder`, no GTest fixture inheritance needed)

**Tests** (minimum 6):

| Test Name | Scenario | What's Verified |
|---|---|---|
| `NormalImpulse_E0_ArrestsVelocity` | Sphere drop, e=0 | λ_n > 0, post-step v_z ≈ 0 |
| `NormalImpulse_E07_PartialRestitution` | Sphere drop, e=0.7 | λ_n ≈ m * 1.7 * |v_z| |
| `NormalImpulse_E1_FullReversal` | Sphere drop, e=1.0 | λ_n ≈ m * 2 * |v_z|, v_z_after ≈ +|v_z_before| |
| `NormalImpulse_EqualMass_VelocitySwap` | Two cubes, equal mass, zero gravity | λ_n produces velocity exchange |
| `NormalImpulse_UnequalMass_Consistent` | Two cubes, 2:1 mass ratio | λ_n consistent with classical elastic formula |
| `NormalImpulse_RestingContact_NonNegative` | Sphere at rest on floor | λ_n ≥ 0, solver converged |
| `NormalImpulse_AlwaysNonNegative` | All above cases | `assertNonNegativeNormal` for all |

### Stage C: Friction Lambda Tests (0087c)

**File**: `msd-sim/test/Physics/Collision/FrictionLambdaTest.cpp`

**Tests** (minimum 6):

| Test Name | Scenario | What's Verified |
|---|---|---|
| `FrictionLambda_Mu05_Saturates` | Sliding cube, mu=0.5, v_x=2 | `||[t1,t2]|| ≈ mu * n` |
| `FrictionLambda_ZeroMu_TangentIsZero` | Sliding cube, mu=0 | `lambda_t1 = lambda_t2 = 0` |
| `FrictionLambda_HighMu_StaticFriction` | Slow slide, mu=2.0 | `||[t1,t2]|| < mu * n` |
| `FrictionLambda_ObliqueSiding_DirectionCorrect` | Diagonal slide (v_x, v_y both nonzero) | Tangent λ direction opposes velocity |
| `FrictionLambda_CoulombCone_AllContacts` | Cube on floor (4 contacts) | `assertCoulombCone` for every contact |
| `FrictionLambda_BlockPGSLayout_ThreeComponent` | Any friction scenario | `assertBlockPGSLayout(result, numContacts)` |

### Stage D: Position Correction Tests (0087d)

**File**: `msd-sim/test/Physics/Collision/PositionCorrectionTest.cpp`

**Tests** (minimum 6):

| Test Name | Scenario | What's Verified |
|---|---|---|
| `PositionCorrection_ShallowPenetration` | 1mm overlap | Correction ≤ slop tolerance |
| `PositionCorrection_DeepPenetration` | 5cm overlap | Partial correction (beta=0.2 limits) |
| `PositionCorrection_ZeroPenetration` | Touching, no overlap | No position change |
| `PositionCorrection_DoesNotInjectVelocity` | Any penetrating pair | `getInertialState(0).velocity` unchanged |
| `PositionCorrection_FlatContact_Consistent` | Cube flat on floor | All 4 contacts corrected consistently |
| `PositionCorrection_MagnitudeMatchesBeta` | Known overlap depth | `displacement ≈ beta * max(depth - slop, 0)` |

**Note on test approach**: These tests call `stepOnce(/*applyForces=*/false, /*correctPositions=*/true)` and compare positions before/after. The `CollisionScenario` captures pre-step state internally and allows comparison.

### Stage E: Pipeline Integration Tests (0087e)

**File**: `msd-sim/test/Physics/Collision/PipelineIntegrationTest.cpp`

**Tests** (minimum 4):

| Test Name | Scenario | Preconditions | What's Verified |
|---|---|---|---|
| `Pipeline_Drop_AllOutputsConsistent` | Sphere drop, one frame | Stage B normal lambda correct | λ_n > 0, correction reasonable, v_z changes sign |
| `Pipeline_Sliding_AllThreeOutputs` | Cube sliding, one frame | Stage C friction lambda correct | λ_n, λ_t correct, cube stays on floor |
| `Pipeline_Bounce_UPwardVelocityAfterFrame` | Elastic bounce, e=1.0 | Stage B elastic lambda | v_z_after > 0 after one frame |
| `Pipeline_MultiFrame_EnergyDissipation` | 5-frame sequence | Stages B-D all pass | Sum(λ · v) per frame ≈ ΔKE |

---

## Test Impact

### Existing Tests Affected

| Test File | Impact | Action Required |
|---|---|---|
| `CollisionPipelineTest.cpp` | None | No change; tests remain |
| `LinearCollisionTest.cpp` | None | Existing tests remain as multi-frame regression tests |
| `FrictionConeSolverTest.cpp` | None | Existing tests remain |
| `ConstraintSolverContactTest.cpp` | None | Complementary (lower-level than new tests) |

### New Tests Required

All new test files listed in the Stage descriptions above.

### Build System Changes Required

1. Add `friend class CollisionScenario;` to `CollisionPipeline` private section
2. Create `msd-sim/test/Helpers/CMakeLists.txt` and include from `msd-sim/test/CMakeLists.txt`
3. Add stage test files to `msd-sim/test/Physics/Collision/CMakeLists.txt`

---

## Open Questions

### Design Decisions — Resolved by Ticket Metadata

1. **Builder vs. fixture inheritance** — Use builder/factory pattern. Confirmed by ticket: "Use a builder/factory pattern rather than inheritance."

2. **SolveResult access mechanism** — Use `friend class CollisionScenario` on `CollisionPipeline`. Confirmed by ticket: "Expose SolveResult via a friend declaration or protected accessor."

3. **Test isolation** — One thing per test: normal lambda accuracy in Stage B, friction in Stage C, position correction in Stage D. Confirmed by ticket: "Test one thing per test."

### Open Questions Remaining

1. **Tolerance calibration**: The ±10-15% tolerance guidance above is analytical. During Stage B implementation, some tests may require wider tolerance (especially for Baumgarte-modified RHS) or tighter tolerance (for well-conditioned contact geometries). The implementer should calibrate by running tests against known-good builds and adjusting `EXPECT_NEAR` tolerances to pass consistently while still catching real regressions.

2. **`CollisionScenario` namespace**: Should the test helpers live in `msd_sim::test` or in an anonymous namespace within the test directory? Recommendation: use `msd_sim::test` for the shared helpers (allowing Stage E tests to compose from Stage B-D scenarios), but keeping all within test-only `msd-sim/test/` paths.

3. **PositionCorrection test approach**: The ticket asks "Should position correction tests use PositionCorrector directly, or go through CollisionPipeline?" The design chooses to go through the pipeline (via `stepOnce(applyForces=false, correctPositions=true)`) because it exercises the same code path that production uses, making regressions detectable. Direct `PositionCorrector` tests (for mathematical properties) already exist in `SplitImpulseTest.cpp`.

---

## Constraints

Per ticket requirements:
- Solver code (`ConstraintSolver`, `BlockPGSSolver`, `PositionCorrector`) is NOT modified
- `CollisionPipeline` receives exactly one production change: a `friend class` declaration
- Tests do not duplicate existing multi-frame regression test assertions
- All new code is in `test/` paths only (except the one-line `friend` addition)

---

## Coding Standards Applied

The following project guidelines apply to this design:

- Brace initialization throughout (`CollisionScenario` members initialized with `{}`).
- Rule of Zero/Five: `CollisionScenario` deletes copy operations (owns `CollisionPipeline` which is non-copyable) and defaults move operations.
- `NaN` initialization: floating-point members (e.g., `dt_`) initialized from constructor arguments, not left uninitialized.
- `snake_case_` for member variables in `CollisionScenario`.
- `PascalCase` for class names.
- Headers in `src/` for production code; `test/Helpers/` for test-only shared utilities.
- No `shared_ptr` — `CollisionScenario` owns assets by value in vectors.
- `std::span` for non-owning views passed to `CollisionPipeline::execute()`.
