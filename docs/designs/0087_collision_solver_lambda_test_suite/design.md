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

    // CollisionPipeline deletes all copy and move, so CollisionScenario
    // owns it via unique_ptr. This makes CollisionScenario movable (the
    // unique_ptr transfers ownership) and non-copyable.
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
    // Returns the SolveResult for direct lambda inspection.
    ConstraintSolver::SolveResult stepOnce(bool applyForces = true,
                                           bool correctPositions = false);

    // Re-fetch the SolveResult from the most recent stepOnce() call
    // (for tests that want to inspect after additional state queries)
    const ConstraintSolver::SolveResult& getLastSolveResult() const;

    // State access for post-step verification
    const InertialState& getInertialState(size_t idx) const;
    const CollisionPipeline& pipeline() const;
    bool hadCollisions() const;

  private:
    double dt_;
    std::vector<AssetInertial> inertials_;
    std::vector<AssetEnvironment> environments_;
    std::unique_ptr<CollisionPipeline> pipeline_;  // non-movable; owned via unique_ptr
    ConstraintSolver::SolveResult lastResult_;
  };

  }  // namespace msd_sim::test
  ```
- **Dependencies**: `CollisionPipeline`, `AssetInertial`, `AssetEnvironment`, `ConvexHull`, `ReferenceFrame`, `ConstraintSolver::SolveResult`
- **Thread safety**: Not thread-safe; single-threaded test use only
- **Error handling**: Assertions on invalid indices; pipeline errors propagate normally

**Design rationale**: `CollisionScenario` owns its `CollisionPipeline` via `std::unique_ptr<CollisionPipeline>` because `CollisionPipeline` deletes all copy and move constructors (see `CollisionPipeline.hpp` lines 218–221). Owning via `unique_ptr` makes `CollisionScenario` movable (the pointer transfers ownership) while keeping the pipeline itself non-movable at the type level. `CollisionScenarioBuilder` factory methods return `CollisionScenario` by value; NRVO applies, and if a move is needed, the `unique_ptr` member enables it.

The `stepOnce()` method calls the protected pipeline sub-phases in sequence (via `friend` access) and captures the `SolveResult` before it would otherwise be discarded. `getLastSolveResult()` provides a const reference to the stored result for tests that wish to re-inspect without re-running the pipeline. This is the minimal surface needed to expose lambdas without making `CollisionPipeline` broadly public.

**Note on friend declaration correctness**: The existing `friend class CollisionPipelineTest` declaration in `CollisionPipeline.hpp` grants access to a GTest suite name (which is not a C++ class), making it functionally dead code. The new `friend class CollisionScenario` declaration will work correctly because `CollisionScenario` is a proper C++ class defined in `msd-sim/test/Helpers/CollisionScenario.hpp`. Cleanup of the dead `CollisionPipelineTest` friend declaration is out of scope for this ticket.

#### CollisionScenarioBuilder

- **Purpose**: Static factory methods that construct common collision setups, eliminating copy-paste boilerplate across the many individual lambda tests.
- **Header location**: `msd-sim/test/Helpers/CollisionScenarioBuilder.hpp`
- **Source location**: header-only (all methods return `CollisionScenario` by value; move is valid because `CollisionScenario` owns its pipeline via `std::unique_ptr`)
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
| `CollisionScenario` | `CollisionPipeline` | `friend` access for protected sub-phases; owned via `unique_ptr` | One line change to production header |
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
- Rule of Five: `CollisionScenario` deletes copy constructor and copy assignment (pipeline is non-copyable); defaults move constructor and move assignment (enabled by `std::unique_ptr<CollisionPipeline>` member). `CollisionPipeline` itself deletes all four — `unique_ptr` ownership is the bridge that makes `CollisionScenario` movable.
- `NaN` initialization: floating-point members (e.g., `dt_`) initialized from constructor arguments, not left uninitialized.
- `snake_case_` for member variables in `CollisionScenario`.
- `PascalCase` for class names.
- Headers in `src/` for production code; `test/Helpers/` for test-only shared utilities.
- No `shared_ptr` — `CollisionScenario` owns the pipeline via `unique_ptr` and assets by value in vectors.
- `std::span` for non-owning views passed to `CollisionPipeline::execute()`.

---

## Design Revision Notes (Autonomous Iteration 1)

**Date**: 2026-02-28
**Triggered by**: Design Review Initial Assessment (REVISION_REQUESTED)

### Changes Made

#### I1 — CollisionScenario move semantics (BLOCKING — fixed)

Changed `pipeline_` member from `CollisionPipeline pipeline_` (by value, non-movable) to `std::unique_ptr<CollisionPipeline> pipeline_` (owned via pointer, movable).

`CollisionPipeline` deletes all copy and move constructors (`= delete` for all four in `CollisionPipeline.hpp` lines 218–221). Owning via `std::unique_ptr` transfers ownership through pointer during move, enabling `CollisionScenario` to declare `CollisionScenario(CollisionScenario&&) noexcept = default` and `operator=(CollisionScenario&&) noexcept = default` with correct semantics.

`CollisionScenarioBuilder` return-by-value semantics are unchanged — NRVO or move via `unique_ptr` member applies.

Updated:
- Interface block in `CollisionScenario` section (private member `pipeline_` type)
- Design rationale paragraph (explains the `unique_ptr` bridge)
- Coding Standards section (Rule of Five explanation updated)
- Integration points table
- PUML diagram (member type, stepOnce() note)

#### I2 — Friend declaration correctness (MAJOR — clarified)

Added a note in the `CollisionScenario` design rationale acknowledging that the existing `friend class CollisionPipelineTest` in `CollisionPipeline.hpp` is dead code (GTest `TEST()` suite names are not C++ classes). Confirmed that `friend class CollisionScenario` will be functional because `CollisionScenario` IS a proper C++ class. Cleanup of the dead declaration is deferred as out of scope. Updated the PUML diagram comment accordingly.

#### I3 — Interface reconciliation (MINOR — resolved)

Added `getLastSolveResult()` to the design document's public interface block (`CollisionScenario`), matching what was already shown in the PUML. Removed `stepOnceRaw()` from the PUML (it was not in the design doc and has no defined purpose). The final interface is:
- `stepOnce(bool, bool) : ConstraintSolver::SolveResult` — runs pipeline, stores result, returns by value
- `getLastSolveResult() const : const ConstraintSolver::SolveResult&` — returns const ref to stored result

### Parts NOT Modified (Passing Criteria)

Per the reviewer's instruction, the following were not modified:
- Five-stage verification chain (A–E structure, stage dependencies)
- Analytical expected lambda values and tolerance guidance
- `LambdaAssertions` design (EXPECT_* over ASSERT_*)
- `CollisionScenarioBuilder` geometry (hardcoded unit cube, no DB dependency)
- Position correction test approach (through pipeline, not PositionCorrector directly)
- Namespace `msd_sim::test`
- Test file naming and directory structure
- CMakeLists.txt plan

---

## Design Review — Initial Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-02-28
**Status**: REVISION_REQUESTED
**Iteration**: 0 of 1

### Issues Requiring Revision

| ID | Issue | Category | Required Change |
|----|-------|----------|-----------------|
| I1 | `CollisionScenario` declares `CollisionScenario(CollisionScenario&&) noexcept = default` but owns `CollisionPipeline pipeline_` which has ALL move and copy operations explicitly `= delete`. A defaulted move constructor cannot be synthesized when a member is non-movable — this will produce a compile error. | C++ Design Quality / Feasibility | Remove the defaulted move constructor and move-assign operator; declare `CollisionScenario(CollisionScenario&&) = delete` and `CollisionScenario& operator=(CollisionScenario&&) = delete`. Update `CollisionScenarioBuilder` factory methods to return by `std::unique_ptr<CollisionScenario>` (heap-allocated) or restructure `CollisionScenario` to own `std::unique_ptr<CollisionPipeline>` (then move of the owning pointer is valid). |
| I2 | The design claims to follow the "existing pattern established by `friend class CollisionPipelineTest`" — but inspection of `CollisionPipeline.hpp` shows `friend class CollisionPipelineTest;` while the actual test file uses `TEST(CollisionPipelineTest, ...)` macros, which define free functions, NOT a C++ class named `CollisionPipelineTest`. The friend declaration in the existing code is dead code granting access to a non-existent class. The new design should not perpetuate this error — it must define a real C++ class for the friend declaration to be meaningful. | Architectural Fit / Feasibility | Define a real helper class that will hold the protected-phase access and is declared as `friend`. The cleanest approach for `0087` is: define `CollisionScenario` (or an inner `Accessor` class) as the friend target, and ensure a C++ class with that exact name is defined in the test binary. Alternatively, make `CollisionScenario` a nested class of a `friend` wrapper. |
| I3 | The PUML diagram shows `stepOnceRaw()` and `getLastSolveResult()` as distinct public methods on `CollisionScenario`, but the design document's interface block shows only `stepOnce(bool, bool)`. The discrepancy leaves implementers with ambiguous API surface. | Architectural Fit | Reconcile: choose one authoritative interface. If `getLastSolveResult()` is retained as a separate accessor, add it to the design document's interface block. If it is removed in favor of `stepOnce()` returning `SolveResult` directly, remove it from the PUML. |

### Revision Instructions for Architect

The following changes must be made before final review:

1. **I1 — CollisionScenario move semantics**: `CollisionPipeline` is non-movable (all special members `= delete` in `CollisionPipeline.hpp` lines 218–221). The design proposes `CollisionScenario` to be movable (`CollisionScenario(CollisionScenario&&) noexcept = default`) while owning a `CollisionPipeline` by value — this is a compile-time contradiction. Two valid resolutions exist:

   **Option A (preferred)**: Own `CollisionPipeline` via `std::unique_ptr<CollisionPipeline>`. The `unique_ptr` itself is movable, so `CollisionScenario` can be move-constructed. `CollisionScenarioBuilder` factory methods return `CollisionScenario` by value (move eligible). Update the Rule of Zero/Five section: copy deleted, move defaulted via the `unique_ptr` member.

   **Option B**: Make `CollisionScenario` non-movable and non-copyable (delete all), and have `CollisionScenarioBuilder` return `std::unique_ptr<CollisionScenario>`. Tests hold unique_ptr and call via pointer. More verbose in test code but avoids indirection in the scenario itself.

   The design document must update the interface block, the Rule of Zero/Five commentary in the Coding Standards section, and the builder return types accordingly.

2. **I2 — Friend declaration correctness**: Before adding `friend class CollisionScenario` to `CollisionPipeline`, confirm that `CollisionScenario` will be defined in the same translation unit as the code that calls protected methods. GTest `TEST()` suite names are not C++ classes; only `TEST_F()` fixtures backed by a `::testing::Test` subclass exist as real classes. The existing `friend class CollisionPipelineTest` is non-functional. For `0087`, the friend declaration `friend class CollisionScenario;` will work correctly IF `CollisionScenario` is a proper C++ class (which the design intends). Update the design document to: (a) acknowledge that the existing `CollisionPipelineTest` friend is dead code; (b) confirm that `CollisionScenario` as designed IS a real C++ class and thus a valid friend target; (c) note that future cleanup of the dead `CollisionPipelineTest` friend is out of scope for this ticket.

3. **I3 — Interface reconciliation**: Decide between these two options and update both `design.md` and the PUML to be consistent:
   - Option A: `stepOnce()` returns `SolveResult` directly; no `getLastSolveResult()` accessor needed.
   - Option B: `stepOnce()` stores internally; `getLastSolveResult()` fetches it. Add `getLastSolveResult()` to the design doc's public interface block.

### Items Passing Review (No Changes Needed)

The following criteria pass and should NOT be modified during revision:

- **Overall architecture decomposition**: The five-stage bottom-up verification chain (A through E) is well-structured. Stage dependencies (B/C/D all depend on A, E depends on B/C/D) are correct and clearly stated.
- **Analytical expected values**: The lambda derivations in the "Analytical Expected Lambda Values" section are physically correct. The ±10–15% tolerance guidance is appropriate for the Baumgarte-augmented solver RHS.
- **`LambdaAssertions` design**: Using `EXPECT_*` over `ASSERT_*` to report all cone violations in a single run is the correct choice.
- **`CollisionScenarioBuilder` geometry**: Hardcoded unit cube vertices (no DB dependency) is exactly right for isolated unit tests — fast, deterministic, no file I/O.
- **Position correction test approach**: Routing through `CollisionPipeline` rather than `PositionCorrector` directly exercises the production code path. Correct decision.
- **Namespace `msd_sim::test`**: Consistent with project conventions for test-shared utilities.
- **Stage D: split-impulse guarantee test**: The `PositionCorrection_DoesNotInjectVelocity` test is a valuable correctness check for the invariant that `correctPositions()` never modifies `InertialState::velocity`.
- **No solver code modification**: The constraint that `ConstraintSolver`, `BlockPGSSolver`, and `PositionCorrector` are not modified is correctly enforced.
- **Test file naming and locations**: `SolverLambdaTest.cpp`, `FrictionLambdaTest.cpp`, `PositionCorrectionTest.cpp`, `PipelineIntegrationTest.cpp` under `test/Physics/Collision/` — consistent with existing test structure.
- **CMakeLists.txt plan**: `msd-sim/test/Helpers/CMakeLists.txt` for shared helpers is the right build system pattern.

---

## Design Review — Final Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-02-28
**Status**: APPROVED WITH NOTES
**Iteration**: 1 of 1

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | `CollisionScenario`, `CollisionScenarioBuilder`, `LambdaAssertions` — PascalCase; `stepOnce()`, `getLastSolveResult()` — camelCase; `pipeline_`, `dt_` — snake_case_ |
| Namespace organization | ✓ | `msd_sim::test` for test-only shared utilities; test files use `using namespace msd_sim` consistent with existing pattern |
| File structure | ✓ | `msd-sim/test/Helpers/` for shared test infrastructure; test files under `test/Physics/Collision/` matching existing structure |
| Dependency direction | ✓ | All new components depend only on existing production types; no circular headers. `CollisionScenario` depends on `CollisionPipeline`, `AssetInertial`, `AssetEnvironment`, `ConstraintSolver::SolveResult` — all valid |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII | ✓ | `std::unique_ptr<CollisionPipeline>` ensures pipeline cleanup; assets owned by value in vectors |
| Smart pointer appropriateness | ✓ | `unique_ptr` for exclusive ownership of non-movable `CollisionPipeline`; no `shared_ptr` used anywhere |
| Value/reference semantics | ✓ | `CollisionScenario` movable via `unique_ptr` member; `LambdaAssertions` takes `const SolveResult&` |
| Rule of 0/3/5 | ✓ | `CollisionScenario` explicitly declares all five: copy deleted, move defaulted (both ctor and assign), dtor defaulted |
| Const correctness | ✓ | `getLastSolveResult() const`, `getInertialState(size_t) const`, `hadCollisions() const` all correct |
| Exception safety | ✓ | No throwing operations in test code; pipeline errors propagate normally |
| Initialization | ✓ | Brace initialization throughout; `dt_` initialized from constructor parameter |
| Return values | ✓ | `stepOnce()` returns `SolveResult` by value; `getLastSolveResult()` returns const ref for zero-copy re-inspection |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | `CollisionScenario.hpp` includes `CollisionPipeline.hpp` via forward-declare-compatible unique_ptr pattern |
| Template complexity | ✓ | No templates introduced; header-only builders use only concrete types |
| Memory strategy | ✓ | Pipeline on heap (unique_ptr), assets in vectors (value), lambdas by value — deterministic ownership |
| Thread safety | ✓ | Single-threaded test use only; explicitly documented |
| Build integration | ✓ | New `test/Helpers/CMakeLists.txt` plus additions to `test/Physics/Collision/CMakeLists.txt` — straightforward |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | `CollisionScenarioBuilder` creates scenarios without DB or Engine dependencies |
| Mockable dependencies | ✓ | `LambdaAssertions` takes const-ref SolveResult — easily tested in isolation |
| Observable state | ✓ | `getLastSolveResult()`, `getInertialState()`, `hadCollisions()` provide full post-step inspection |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Tolerance calibration for lambda assertions: ±10–15% analytical guidance may not match solver's Baumgarte-augmented RHS for all test cases, requiring empirical tuning during Stage B | Technical | Med | Low | Design explicitly notes this; implementer calibrates against known-good builds. Tolerances documented per-test. | No |
| R2 | `unique_ptr<CollisionPipeline>` indirection adds one heap allocation per scenario creation. For unit tests this is negligible, but worth noting if many hundreds of scenarios are created in tight loops | Performance | Low | Low | Test scenarios are created once per TEST_F case; allocation is not in a hot path | No |
| R3 | `CollisionScenario::stepOnce()` calls protected pipeline sub-phases via friend access. If sub-phase method signatures change in future pipeline refactors, this will be a compilation break point | Maintenance | Low | Med | `CollisionScenario` is in test code only; breakage at compilation is immediately visible and caught by CI | No |

### Resolved Issues

All three issues from the Initial Assessment have been addressed:

- **I1 (BLOCKING — resolved)**: `CollisionPipeline` is now owned via `std::unique_ptr<CollisionPipeline>`, resolving the compile-time conflict between defaulted move semantics and a non-movable by-value member.
- **I2 (MAJOR — resolved)**: Design document now explicitly documents the dead `friend class CollisionPipelineTest` declaration and confirms that `friend class CollisionScenario` will be functional.
- **I3 (MINOR — resolved)**: Both `stepOnce()` and `getLastSolveResult()` are present and consistent between `design.md` and the PUML. `stepOnceRaw()` removed from PUML.

### Notes for Human Review

1. **Prototype phase**: This ticket skips the Prototype phase (ticket status goes directly to Ready for Implementation) per the original workflow — the five stages are self-contained test-only work, and Stage A's acceptance gate (3 existing tests rewritten using builder) serves as the proof-of-concept validation.

2. **Dead friend cleanup**: The dead `friend class CollisionPipelineTest` declaration in `CollisionPipeline.hpp` is a latent technical debt item. Recommend filing a follow-up ticket to clean it up and convert the `CollisionPipelineTest` test suite to use `TEST_F()` with a real fixture class if sub-phase access is ever needed.

3. **Stage E multi-frame energy test**: The `Pipeline_MultiFrame_EnergyDissipation` test (5-frame sequence, KE accounting) is the most complex test in the suite. Implementer should treat tolerance calibration for this test as empirical — the analytical bound is `|sum(λ·v) per frame - ΔKE| < ε` where ε depends on integration step size.

### Summary

The design is architecturally sound, well-scoped, and directly addresses the motivation of building a bottom-up verification chain for the collision pipeline. The three issues from the initial review have all been resolved in the autonomous revision. The design is ready for implementation beginning with Stage A (0087a).

---
