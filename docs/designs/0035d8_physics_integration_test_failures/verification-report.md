# Physics Integration Test Failures — Verification Report
# Ticket: 0035d8_physics_integration_test_failures
# Date: 2026-02-01
# Status: Verification FAILED — All 4 tests still failing after upstream fixes

---

## Executive Summary

All 4 physics integration tests remain FAILING after the implementation of upstream fixes (0035d1 SOCP reformulation, 0035d2 lambda filter fix). The ECOS solver integration tests (ECOSSolveTest suite) PASS, indicating the SOCP formulation is mathematically correct in isolation. The failures occur at the WorldModel integration level, suggesting issues in the two-phase solve pipeline or force extraction/combination.

---

## Test Results

### Test Suite Run
- **Date**: 2026-02-01
- **Build**: Debug preset (conan-debug)
- **Command**: `/Users/danielnewman/Documents/GitHub/MSD-CPP/build/Debug/debug/msd_sim_test`
- **Total tests**: 538
- **Passed**: 534
- **Failed**: 4

### Failing Tests

#### Test 1: MultipleSimultaneousContacts_ResolvedCorrectly
- **File**: `msd-sim/test/Environment/WorldModelContactIntegrationTest.cpp:347`
- **Failure**: Box penetrates wall
  - **Expected**: `finalPos.x() >= -0.5`
  - **Actual**: `finalPos.x() = -35.04`
  - **Error**: Box penetrated wall by 34.5 meters
- **Analysis**: The two-phase solve (ASM normals + ECOS friction) produces incorrect friction impulses that overwhelm the ASM normal forces, launching the box through the floor and wall.

#### Test 2: EnergyMonotonicDecreaseForSliding
- **File**: `msd-sim/test/Physics/FrictionEnergyTest.cpp`
- **Failure**: Block acquires spurious vertical velocity
  - **Expected**: `|finalVel.z()| < 1e-9` (block stays on floor)
  - **Actual**: `finalVel.z() = -5.02 m/s`
  - **Error**: Block launched off floor with 5.02 m/s vertical velocity
- **Analysis**: ECOS produces spurious vertical impulse component, injecting energy and launching the block off the surface. This is the exact scenario from DEBUG_0035d Phase 1.

#### Test 3: KineticFrictionInclinedPlane_SlowsMotion
- **File**: `msd-sim/test/Physics/FrictionValidationTest.cpp:152`
- **Failure**: Friction barely slows the box
  - **Expected**: `speedFriction < speedFrictionless * 0.9`
  - **Actual**: `speedFriction = 4.96 m/s` vs `speedFrictionless = 5.00 m/s` (only 0.8% reduction)
  - **Error**: Friction should reduce speed by at least 10%, but only reduces by 0.8%
- **Analysis**: ECOS friction forces are either incorrect (energy-injecting) or being dropped by the pipeline. The lambda filter fix (0035d2) should prevent dropping, so the ECOS solution itself is likely wrong.

#### Test 4: StickSlipTransitionSmoothness
- **File**: `msd-sim/test/Physics/FrictionStabilityTest.cpp:270`
- **Failure**: Excessive oscillation in stick-slip transition
  - **Expected**: `oscillationCount < 20`
  - **Actual**: `oscillationCount = 42`
  - **Error**: 42 oscillations exceeds the threshold of 20
- **Analysis**: Incorrect ECOS solutions cause the friction force to oscillate between incorrect values, creating numerical instability at the stick-slip boundary.

---

## Upstream Dependency Status

### 0035d1: ECOS SOCP Reformulation
- **Status**: Merged / Complete
- **Implementation**: Auxiliary-variable SOCP formulation with epigraph lifting
- **Variables**: `x = [λ; y; t]` (6C+1 dimensions)
- **Equality constraints**: `Lᵀλ - y = 0` (3C constraints)
- **Cone constraints**: Epigraph cone (3C+2 dimensions) + friction cones (3 per contact)
- **Verification**: ECOSSolveTest suite passes (12/12 tests)

### 0035d2: Friction Lambda Sign Filter Fix
- **Status**: Merged / Complete
- **Implementation**: Dimension-aware filter in `extractContactBodyForces()`
- **Fix**: Bypasses `anyPositive` check for friction constraints (dim=2)
- **Verification**: Filter no longer drops friction forces with negative tangential lambdas

### 0035d7: ECOS Solver Integration Test Failures
- **Status**: Implementation Complete — Awaiting Quality Gate
- **Note**: User confirmed to treat as complete for 0035d8 verification
- **Verification**: ECOSSolveTest suite passes (all ECOS solver integration tests)

---

## Root Cause Analysis

### Observation 1: ECOS Solver Tests Pass
The `ECOSSolveTest` suite (12 tests) all pass:
- `SingleContactStickRegime` ✓
- `SingleContactSlipRegime` ✓
- `WellConditionedConvergence` ✓
- `DiagnosticsPopulated` ✓
- `MultiContactDifferentMu` ✓
- `HighMassRatio` ✓
- etc.

**Implication**: The SOCP formulation (0035d1) is mathematically correct when tested in isolation with synthetic constraint data. The ECOS solver produces valid solutions for simplified test scenarios.

### Observation 2: Physics Integration Tests Fail
All 4 physics integration tests fail with the same symptoms as before:
- Energy injection (spurious vertical velocity)
- Incorrect friction magnitude (barely slows motion)
- Instability (excessive oscillations)
- Contact resolution failure (box launched through walls)

**Implication**: The failure occurs at the WorldModel integration level, where the two-phase solve combines ASM normal forces with ECOS friction forces.

### Hypothesis: Two-Phase Solve Pipeline Issues

The WorldModel two-phase friction solve (implemented in 0035d5) has two phases:
1. **Phase 1 (ASM)**: Solve for normal impulses only (friction coefficients zeroed)
2. **Phase 2 (ECOS)**: Solve for friction impulses with normals fixed from Phase 1

**Potential issues**:

#### H1: Normal Lambda Extraction
`extractFrictionOnlyBodyForces()` in `ConstraintSolver.cpp` is supposed to zero out normal lambdas before computing body forces from the ECOS solution. If this is incorrect, the friction forces would include incorrect normal components.

**Verification needed**:
- Check that `extractFrictionOnlyBodyForces()` correctly zeroes `lambda_i(0)` for `ContactConstraint` + `FrictionConstraint` pairs
- Verify that the force calculation `J^T * lambda` uses only the friction rows when `lambda_i(0) = 0`

#### H2: Force Combination
WorldModel combines ASM normal forces with ECOS friction forces. If the forces are combined incorrectly (e.g., friction forces added twice, normal forces discarded), the net impulse would be wrong.

**Verification needed**:
- Trace WorldModel::constraintResolution() to verify force accumulation
- Check that ASM forces and ECOS forces are added correctly (not overwritten)

#### H3: ECOS Input Construction
The ECOS solver receives fixed normal lambdas from Phase 1. If the effective mass matrix or RHS vector is constructed incorrectly for the fixed-normal case, the ECOS solution would be wrong.

**Verification needed**:
- Check that `setNormalLambda()` on FrictionConstraint correctly updates the friction bounds
- Verify that the effective mass matrix `A = J·M⁻¹·Jᵀ` accounts for the fixed normals
- Check that the RHS vector `b` is constructed correctly for the friction-only problem

#### H4: Constraint Ordering
If the contact manifold or constraint list ordering is non-deterministic, the normal lambdas from Phase 1 might be paired with the wrong friction constraints in Phase 2.

**Verification needed**:
- Check that constraint ordering is deterministic (same contact always gets same index)
- Verify that the mapping from Phase 1 normal lambdas to Phase 2 friction constraints is correct

---

## Recommended Next Steps

### Step 1: Diagnostic Instrumentation
Add detailed logging to the two-phase solve pipeline:
1. **Before Phase 1**: Log contact configuration (positions, normals, tangents)
2. **After Phase 1**: Log ASM normal lambdas, check `lambda_n >= 0`, compute normal impulse magnitudes
3. **Between phases**: Log how normal lambdas are passed to friction constraints via `setNormalLambda()`
4. **Before Phase 2**: Log ECOS input (effective mass A, RHS b, friction bounds)
5. **After Phase 2**: Log ECOS lambdas, check friction cone constraints, compute friction impulse magnitudes
6. **Force combination**: Log ASM forces, ECOS forces, combined forces, verify no double-counting

### Step 2: Minimal Reproduction
Create a minimal test case that isolates the two-phase pipeline:
- Single box on floor with friction
- Horizontal initial velocity
- Run 1 timestep only
- Log all intermediate values

Compare with expected physics:
- Normal impulse should exactly cancel penetration velocity
- Friction impulse should oppose tangential velocity
- Friction magnitude should satisfy `|f_t| <= mu * f_n`

### Step 3: Unit Test Coverage
The two-phase solve is tested end-to-end via physics integration tests, but lacks unit-level coverage:
- Unit test for `extractFrictionOnlyBodyForces()` with known lambda values
- Unit test for normal lambda passing from Phase 1 to Phase 2
- Unit test for ECOS input construction with fixed normal lambdas

### Step 4: Comparison with ASM
Run the same scenarios with the ASM solver (approximate friction via linearization):
- If ASM produces physically correct results, the ECOS formulation or integration is the issue
- If ASM also fails, the problem is upstream (contact detection, effective mass, etc.)

---

## Files Under Investigation

### Production Files
| File | Relevance |
|------|-----------|
| `msd-sim/src/Environment/WorldModel.cpp` | Two-phase solve pipeline, force combination |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | `extractFrictionOnlyBodyForces()`, `solveWithECOS()` |
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` | Constraint creation, ordering |
| `msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.cpp` | SOCP construction with fixed normals |

### Test Files
| File | Test | Status |
|------|------|--------|
| `msd-sim/test/Environment/WorldModelContactIntegrationTest.cpp:346` | MultipleSimultaneousContacts | FAIL |
| `msd-sim/test/Physics/FrictionEnergyTest.cpp:100` | EnergyMonotonicDecrease | FAIL |
| `msd-sim/test/Physics/FrictionValidationTest.cpp:152` | KineticFrictionInclinedPlane | FAIL |
| `msd-sim/test/Physics/FrictionStabilityTest.cpp:270` | StickSlipTransition | FAIL |

---

## Conclusion

The physics integration tests remain FAILED despite the implementation of 0035d1 (SOCP reformulation) and 0035d2 (lambda filter fix). The ECOS solver tests pass in isolation, indicating the SOCP formulation is mathematically correct. The failure occurs at the WorldModel integration level, where the two-phase solve combines ASM normal forces with ECOS friction forces.

**The most likely root cause is in the two-phase solve pipeline (0035d5)**, specifically:
1. Incorrect extraction of friction-only forces from ECOS solution
2. Incorrect combination of ASM normal forces with ECOS friction forces
3. Incorrect construction of ECOS input with fixed normal lambdas
4. Non-deterministic constraint ordering causing normal/friction mismatch

**Recommendation**: Before advancing this ticket, investigate the two-phase solve pipeline with diagnostic instrumentation and unit-level tests. The upstream fixes (0035d1, 0035d2) appear to be correct, but the integration is faulty.
