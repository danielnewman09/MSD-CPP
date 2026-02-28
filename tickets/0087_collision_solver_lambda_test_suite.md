# Ticket 0087: Collision Solver Lambda Test Suite

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-02-28
- **Author**: Daniel Newman
- **Priority**: High
- **Estimated Complexity**: Large (multi-stage)
- **Target Component(s)**: msd-sim (test), msd-sim (src — test infrastructure only)
- **Languages**: C++
- **Generate Tutorial**: No
- **Requires Math Design**: No
- **GitHub Issue**: #117
- **Branch**: 0087-collision-solver-lambda-test-suite
- **PR**: TBD
- **Design Revision Count**: 0
- **Previous Design Approaches**: []

---

## Summary

Decompose the existing multi-frame friction sliding and linear collision (bouncing/dropping) tests into isolated single-collision tests that validate the constraint solver's lambda outputs directly. The goal is to trace collision mechanisms from the root — starting with the solver producing correct lambdas, then building up through position correction and multi-frame integration — creating a bottom-up verification chain for the collision pipeline.

This also addresses test infrastructure: a shared scenario builder (fixtures/factories) to eliminate copy-paste boilerplate across the many individual collision tests.

---

## Motivation

### Current Gap

The existing FrictionSlidingTest (17 tests) and LinearCollisionTest (6 tests) validate end-to-end behavior: spawn objects, simulate N frames, check final state. This is valuable but opaque — when a test fails, it's unclear whether the root cause is:

1. **Solver**: Wrong lambda values (normal impulse, friction impulse)
2. **Position correction**: Excessive or insufficient penetration correction
3. **Force application**: Constraint forces applied incorrectly
4. **Integration accumulation**: Small per-frame errors compounding over many frames

By isolating individual collisions and inspecting solver lambdas directly, we can:
- Verify the solver produces physically correct impulses for known scenarios
- Verify position correction magnitudes are reasonable (not over/under-correcting)
- Build confidence that multi-frame behavior is the natural consequence of correct per-frame mechanics
- Catch solver regressions immediately rather than through indirect multi-frame symptoms

### What "Isolate Individual Collisions" Means

Each existing multi-frame test contains a recurring collision pattern:
- **Dropping tests**: One collision event (ball hits floor), then possibly bounces (repeated collisions)
- **Sliding tests**: Sustained contact over many frames (floor provides normal force + friction each frame)
- **Bounce-then-slide**: Impact collision, then transition to sliding contact

We want to extract representative single-frame collisions from these scenarios and test them in isolation:
- Set up bodies at a known pre-collision state
- Run **one** pipeline execution (one dt step)
- Inspect the `SolveResult.lambdas` vector
- Verify position correction displacement
- Verify post-collision velocities

---

## Requirements

### Functional Requirements

#### Stage A: Test Infrastructure — Scenario Builders (0087a)

1. Create a `CollisionScenarioBuilder` utility that constructs common collision setups without copy-pasting:
   - `sphereOnFloor(height, velocity, mass, restitution, friction)` — sphere at given height with downward velocity, floor at z=0
   - `cubeOnFloor(height, velocity, mass, restitution, friction)` — cube sliding/dropping onto floor
   - `twoSpheres(separation, velA, velB, massA, massB, restitution, friction)` — head-on or glancing pair
   - Each builder returns a lightweight struct with references to the spawned assets and world state
2. Builders must work with `ReplayEnabledTest` fixture (recording infrastructure)
3. Builders must expose the `CollisionPipeline` for direct lambda inspection after `execute()`
4. Add a `stepOnce()` helper that runs exactly one pipeline step and returns the `SolveResult`

#### Stage B: Solver Lambda Tests — Normal Impulse (0087b)

Validate that the solver computes correct normal impulse lambdas for isolated collisions:

1. **Sphere drop onto floor (e=0)** — Lambda should arrest vertical velocity in one step: λ_n ≈ m·|v_z|/dt
2. **Sphere drop onto floor (e=0.7)** — Lambda should reverse velocity with coefficient of restitution: λ_n ≈ m·(1+e)·|v_z|/dt
3. **Sphere drop onto floor (e=1.0)** — Lambda should fully reverse velocity: λ_n ≈ 2·m·|v_z|/dt
4. **Equal-mass elastic collision (zero gravity)** — Lambda produces velocity swap
5. **Unequal-mass elastic collision** — Lambda consistent with classical elastic formulas
6. **Resting contact (sphere on floor, zero velocity)** — Lambda supports weight: λ_n ≈ m·g·dt (gravity impulse balanced)

All tests verify `SolveResult.lambdas` directly, not just post-collision velocities.

#### Stage C: Solver Lambda Tests — Friction Impulse (0087c)

Validate friction lambda values for known friction scenarios:

1. **Sliding cube (mu=0.5, v_x=2.0)** — Tangent lambda should decelerate: |λ_t| ≈ mu·λ_n (Coulomb cone saturation)
2. **Sliding cube (mu=0, v_x=2.0)** — Tangent lambda should be zero
3. **Sliding cube (mu=2.0, small v_x)** — Static friction: |λ_t| < mu·λ_n (inside cone)
4. **Oblique sliding (v_x, v_y both nonzero)** — Tangent lambda direction opposes velocity, magnitude bounded by cone
5. **Coulomb cone verification** — For every test: assert `||[λ_t1, λ_t2]|| <= mu · λ_n`
6. **Block PGS 3-component output** — Verify lambdas vector has correct structure: [λ_n, λ_t1, λ_t2] per contact

#### Stage D: Position Correction Tests (0087d)

Validate that position correction is reasonable and doesn't over/under-correct:

1. **Shallow penetration** — Bodies with 1mm overlap: correction moves them to within slop tolerance (5mm default)
2. **Deep penetration** — Bodies with 5cm overlap: correction reduces penetration but doesn't teleport (beta=0.2 limits correction per step)
3. **Zero penetration** — Touching but not penetrating: no position correction applied
4. **Correction doesn't inject velocity** — After position correction, verify real velocity is unchanged (split-impulse guarantee)
5. **Multi-contact correction** — Cube flat on floor: all 4 contacts corrected consistently
6. **Correction magnitude** — Verify correction ≈ beta/dt · max(depth - slop, 0) · dt = beta · max(depth - slop, 0)

#### Stage E: Pipeline Integration Tests (0087e)

Build up from stages B-D to verify single-frame pipeline behavior:

1. **Drop + correct + settle** — Sphere dropped from rest: one frame produces correct lambda AND reasonable position correction, resulting in a physically sensible post-frame state
2. **Sliding + friction + correct** — Cube sliding: one frame produces correct normal lambda, correct friction lambda, and position correction that keeps the cube on the surface
3. **Bounce frame** — Elastic bounce: one frame produces restitution-correct lambda, and next frame's state shows upward velocity
4. **Multi-frame energy accounting** — 5-frame sequence: track lambda·velocity product each frame to verify energy dissipation matches KE change

### Non-Functional Requirements

1. No copy-paste scenario setup — all tests use shared builders
2. Each stage is independently buildable and testable
3. New test file per stage: `SolverLambdaTest.cpp`, `FrictionLambdaTest.cpp`, `PositionCorrectionTest.cpp`, `PipelineIntegrationTest.cpp`
4. Tests should not duplicate existing FrictionSlidingTest/LinearCollisionTest assertions — those remain as multi-frame regression tests

---

## Constraints

- Solver code (`ConstraintSolver`, `BlockPGSSolver`, `PositionCorrector`) must NOT be modified
- CollisionPipeline may need minor additions to expose `SolveResult` for test inspection (protected/friend access)
- Tests should use analytical expected values where possible, with tolerances justified by discretization effects

---

## Acceptance Criteria

### Stage A (0087a)
- [ ] `CollisionScenarioBuilder` supports sphere-on-floor, cube-on-floor, two-sphere setups
- [ ] `stepOnce()` returns `SolveResult` with lambdas accessible
- [ ] At least 3 existing test scenarios rewritten using builder (proving boilerplate reduction)

### Stage B (0087b)
- [ ] >= 6 normal-impulse lambda tests
- [ ] All tests verify `SolveResult.lambdas` directly
- [ ] Analytical expected values documented for each test

### Stage C (0087c)
- [ ] >= 6 friction-impulse lambda tests
- [ ] Every test verifies Coulomb cone compliance: `||[λ_t1, λ_t2]|| <= mu · λ_n`
- [ ] Block PGS 3-component structure verified

### Stage D (0087d)
- [ ] >= 6 position correction tests
- [ ] Split-impulse energy guarantee verified (velocity unchanged after correction)
- [ ] Correction magnitude tests with analytical bounds

### Stage E (0087e)
- [ ] >= 4 single-frame pipeline integration tests
- [ ] At least one multi-frame energy accounting test
- [ ] Tests reference stages B-D results as preconditions

---

## Design Decisions (Human Input)

### Preferred Approaches
- Use a builder/factory pattern rather than inheritance for scenario construction — avoids deep fixture hierarchies
- Expose `SolveResult` via a `friend` declaration or protected accessor on CollisionPipeline rather than making it public
- Test one thing per test: separate normal lambda accuracy from friction lambda accuracy from position correction
- Derive expected lambda values analytically where possible (e.g., impulse = m·Δv/dt for known Δv)

### Things to Avoid
- Do NOT modify solver algorithms — this is a test-only ticket
- Do NOT relax existing multi-frame test tolerances
- Do NOT create a parallel simulation path just for testing — use the same pipeline with inspection hooks
- Do NOT make CollisionPipeline's internals broadly public — use targeted friend/protected access

### Open Questions
1. Should `CollisionScenarioBuilder` live in test utilities or as a new test helper class? (Likely `test/Helpers/`)
2. What tolerance is appropriate for lambda comparisons? The solver computes impulse (λ has units of N·s), and discretization means λ won't match the continuous-time analytical value exactly. Need to determine per-test tolerances empirically during Stage B.
3. Should position correction tests use the PositionCorrector directly, or always go through CollisionPipeline? (Direct testing is cleaner but doesn't exercise the pipeline's integration of correction with solving.)

---

## Sub-Tickets

| Sub-Ticket | Stage | Summary | Dependencies |
|------------|-------|---------|--------------|
| 0087a | A | Test infrastructure: scenario builders and `stepOnce()` helper | None |
| 0087b | B | Normal impulse lambda tests (6+ tests) | 0087a |
| 0087c | C | Friction impulse lambda tests (6+ tests) | 0087a |
| 0087d | D | Position correction magnitude and energy tests (6+ tests) | 0087a |
| 0087e | E | Single-frame pipeline integration tests (4+ tests) | 0087b, 0087c, 0087d |

Stages B, C, D can proceed in parallel once Stage A is complete.

---

## References

### Related Code
- `msd/msd-sim/test/Physics/Collision/FrictionSlidingTest.cpp` — 17 existing multi-frame friction tests
- `msd/msd-sim/test/Physics/Collision/LinearCollisionTest.cpp` — 6 existing multi-frame collision tests
- `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp` — Pipeline with `SolveResult` access
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` — `SolveResult` struct with lambdas
- `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.hpp` — Block PGS with 3-component lambdas
- `msd/msd-sim/src/Physics/Constraints/PositionCorrector.hpp` — Split-impulse position correction
- `msd/msd-sim/test/Replay/ReplayEnabledTest.hpp` — Existing test fixture with spawn helpers

### Related Tickets
- [0082b](0082b_friction_sliding_test_coverage.md) — FrictionSlidingTest suite (17 tests)
- [0086a](0086a_split_step_test_validation.md) — Split-step validation and calibration
- [0084](0084_block_pgs_solver_rework.md) — Block PGS solver rework
- [0086](0086_split_step_block_pgs.md) — Split-step two-pass Block PGS

### Architectural Context
- `SolveResult.lambdas` — For Block PGS with friction: 3 values per contact [λ_n, λ_t1, λ_t2]. For ASM without friction: 1 value per contact [λ_n].
- `SolveResult.warmStartLambdas` — Phase B lambdas only (excludes restitution bounce from Phase A)
- `PositionCorrector::Config` — beta=0.2, slop=0.005m, maxIterations=4
- `CollisionPipeline` already has `friend class CollisionPipelineTest` — can extend this pattern

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-28
- **Status**: Draft — awaiting review before design phase

### Design Phase
- **Started**: 2026-02-28 (automated)
- **Completed**: 2026-02-28
- **Branch**: 0087-collision-solver-lambda-test-suite
- **PR**: TBD (draft — to be created after push)
- **Artifacts**:
  - `docs/designs/0087_collision_solver_lambda_test_suite/design.md`
  - `docs/designs/0087_collision_solver_lambda_test_suite/0087_collision_solver_lambda_test_suite.puml`
- **Notes**: Design covers all five stages (A-E). Key decisions: CollisionScenario as lightweight owner with friend access to CollisionPipeline sub-phases; CollisionScenarioBuilder as static factories for hardcoded unit-cube geometry (no DB dependency); LambdaAssertions for Coulomb cone and Block PGS layout checks. Requires math design: No — skipped per metadata. Generate tutorial: No. Single production change: one `friend class CollisionScenario` declaration on CollisionPipeline.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Implementation
{Your comments on the implementation}
