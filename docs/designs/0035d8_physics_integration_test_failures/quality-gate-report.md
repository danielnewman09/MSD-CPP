# Quality Gate Report: 0035d8_physics_integration_test_failures
# Date: 2026-02-01
# Verdict: FAILED — All 4 physics integration tests still failing

---

## Summary

**Ticket**: [0035d8_physics_integration_test_failures](../../tickets/0035d8_physics_integration_test_failures.md)
**Type**: Verification Ticket
**Status**: Verification FAILED
**Verdict**: FAILED

This is a verification ticket that tests whether upstream fixes (0035d1 SOCP reformulation, 0035d2 lambda filter fix) resolve 4 failing physics integration tests. All 4 tests remain FAILING. The quality gate verdict is FAILED because the acceptance criteria (tests passing) are not met.

---

## Gate 1: Build Verification

**Status**: PASS ✓

**Command**:
```bash
cmake --build --preset conan-debug --target msd_sim_test
```

**Results**:
- Exit code: 0
- Build successful
- No warnings or errors
- Test executable created: `/Users/danielnewman/Documents/GitHub/MSD-CPP/build/Debug/debug/msd_sim_test`

**Verdict**: PASS — Code compiles without errors or warnings

---

## Gate 2: Test Verification

**Status**: FAIL ✗

**Command**:
```bash
/Users/danielnewman/Documents/GitHub/MSD-CPP/build/Debug/debug/msd_sim_test
```

**Results**:
- Total tests run: 538
- Passed: 534
- Failed: 4
- Exit code: 1

### Failing Tests

#### 1. WorldModelContactIntegrationTest.MultipleSimultaneousContacts_ResolvedCorrectly

**File**: `msd-sim/test/Environment/WorldModelContactIntegrationTest.cpp:347`

**Failure**:
```
Expected: (finalPos.x()) >= (-0.5), actual: -35.035900226953309 vs -0.5
Box penetrated wall
```

**Analysis**: A box in a corner (floor + wall) should be resolved by contact constraints. The two-phase solve produces incorrect friction impulses that overwhelm the ASM normal forces, launching the box through the floor and wall.

**Acceptance Criterion**: AC1 — FAILED

---

#### 2. FrictionEnergyTest.EnergyMonotonicDecreaseForSliding

**File**: `msd-sim/test/Physics/FrictionEnergyTest.cpp`

**Failure**:
```
The difference between finalVel.z() and 0 is 5.017750736820819, which exceeds 1e-9, where
finalVel.z() evaluates to -5.017750736820819,
0 evaluates to 0, and
1e-9 evaluates to 1.0000000000000001e-09.
```

**Analysis**: A block sliding on a surface with friction should decelerate (energy monotonically decreases). ECOS produces spurious vertical impulse (+5.02 m/s), launching the block off the floor. This is the exact scenario from DEBUG_0035d Phase 1 reproduction.

**Acceptance Criterion**: AC2 — FAILED

---

#### 3. FrictionValidationTest.KineticFrictionInclinedPlane_SlowsMotion

**File**: `msd-sim/test/Physics/FrictionValidationTest.cpp:152`

**Failure**:
```
Expected: (speedFriction) < (speedFrictionless * 0.9), actual: 4.9619529823986683 vs 4.5000000000000293
Friction should significantly slow the box. Frictionless speed: 5.0000000000000329, Friction speed: 4.9619529823986683
```

**Analysis**: A box sliding on an inclined plane with μ=0.5 should be significantly slowed by friction. Friction speed (4.96 m/s) is only marginally less than frictionless speed (5.00 m/s) — only 0.8% reduction instead of the expected ≥10%. ECOS friction forces are either wrong (energy-injecting) or dropped by the pipeline.

**Acceptance Criterion**: AC3 — FAILED

---

#### 4. FrictionStabilityTest.StickSlipTransitionSmoothness

**File**: `msd-sim/test/Physics/FrictionStabilityTest.cpp:270`

**Failure**:
```
Expected: (oscillationCount) < (20), actual: 42 vs 20
Stick-slip transition should be smooth without excessive oscillation. Oscillation count: 42
```

**Analysis**: A block transitioning from sliding to static friction should do so smoothly. 42 oscillations observed (threshold is < 20). Incorrect ECOS solutions cause the friction force to oscillate between incorrect values, creating numerical instability.

**Acceptance Criterion**: AC4 — FAILED

---

**Verdict**: FAIL — 4 critical tests failing (0/4 acceptance criteria met)

---

## Gate 3: Benchmark Regression Detection

**Status**: N/A (Not applicable for verification ticket)

**Reason**: This is a verification ticket with no new implementation code. No benchmarks are specified in the ticket.

---

## Upstream Dependency Status

### 0035d1: ECOS SOCP Reformulation
- **Status**: Merged / Complete ✓
- **Implementation**: Auxiliary-variable SOCP formulation with epigraph lifting
- **Verification**: ECOSSolveTest suite passes (12/12 tests) ✓

### 0035d2: Friction Lambda Sign Filter Fix
- **Status**: Merged / Complete ✓
- **Implementation**: Dimension-aware filter in `extractContactBodyForces()`
- **Verification**: Filter no longer drops friction forces with negative tangential lambdas ✓

### 0035d7: ECOS Solver Integration Test Failures
- **Status**: Implementation Complete — Awaiting Quality Gate
- **Verification**: ECOSSolveTest suite passes (all ECOS solver integration tests) ✓

**Key Observation**: All ECOS solver tests pass in isolation, but physics integration tests fail. This indicates the SOCP formulation (0035d1) is mathematically correct, but the integration with WorldModel's two-phase solve pipeline has issues.

---

## Root Cause Analysis

### Discrepancy: ECOS Tests Pass, Physics Tests Fail

The ECOSSolveTest suite (12 tests) verifies the ECOS solver with synthetic constraint data:
- Single contact stick regime ✓
- Single contact slip regime ✓
- Multiple contacts with different friction coefficients ✓
- High mass ratio scenarios ✓
- Well-conditioned convergence ✓

All pass, indicating the SOCP formulation is mathematically sound.

However, all 4 physics integration tests fail with the same symptoms as before the fixes:
- Energy injection (spurious velocities)
- Incorrect friction magnitude
- Instability (oscillations)
- Contact resolution failure (penetrations)

### Hypothesis: Two-Phase Solve Pipeline Issues

The failure occurs at the **WorldModel integration level**, where the two-phase solve (implemented in 0035d5) combines ASM normal forces with ECOS friction forces.

**Potential root causes**:

1. **Normal Lambda Extraction**: `extractFrictionOnlyBodyForces()` may not correctly zero out normal lambdas before computing body forces from the ECOS solution.

2. **Force Combination**: WorldModel may combine ASM normal forces and ECOS friction forces incorrectly (double-counting, overwriting, wrong sign).

3. **ECOS Input Construction**: The effective mass matrix or RHS vector may be constructed incorrectly for the fixed-normal-lambda case in Phase 2.

4. **Constraint Ordering**: Non-deterministic constraint ordering may cause normal lambdas from Phase 1 to be paired with wrong friction constraints in Phase 2.

---

## Acceptance Criteria Assessment

| Criterion | Status | Evidence |
|-----------|--------|----------|
| AC1: MultipleSimultaneousContacts passes | FAIL | Box penetrates wall (x = -35.04 vs >= -0.5) |
| AC2: EnergyMonotonicDecrease passes | FAIL | Spurious vertical velocity (vz = -5.02 vs ~0) |
| AC3: KineticFrictionInclinedPlane passes | FAIL | Friction barely slows (4.96 vs 5.00, 0.8% reduction) |
| AC4: StickSlipTransition passes | FAIL | Excessive oscillation (42 vs < 20) |
| AC5: Energy monotonically non-increasing | FAIL | AC2 failure implies energy injection |

**Overall**: 0/5 acceptance criteria met

---

## Recommended Actions

### Action 1: Investigate Two-Phase Solve Pipeline (CRITICAL)

The two-phase solve pipeline (0035d5) is the most likely source of the failures:

**Files to investigate**:
- `msd-sim/src/Environment/WorldModel.cpp` — Two-phase solve, force combination
- `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` — `extractFrictionOnlyBodyForces()`, `solveWithECOS()`
- `msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` — Constraint creation, ordering

**Diagnostic steps**:
1. Add logging to trace normal lambda values from Phase 1
2. Add logging to trace how normals are passed to Phase 2 via `setNormalLambda()`
3. Add logging to trace ECOS input (A, b, friction bounds)
4. Add logging to trace ECOS output lambdas
5. Add logging to trace force extraction and combination
6. Create minimal reproduction test (single box, single contact, 1 timestep)

### Action 2: Unit Test Coverage for Force Extraction (HIGH)

The `extractFrictionOnlyBodyForces()` method lacks unit-level tests:
- Test that normal lambdas are zeroed before force calculation
- Test that friction lambdas are preserved
- Test that forces are computed correctly from zeroed-normal lambda vector

### Action 3: Consider Alternative Hypothesis (MEDIUM)

If the two-phase pipeline is correct, the issue may be in:
- ECOS problem construction for fixed-normal case (check `setNormalLambda()` effect on friction bounds)
- Effective mass matrix construction (check if A is correct for friction-only problem)
- RHS vector construction (check if b accounts for fixed normals)

### Action 4: Comparison Test with ASM (LOW)

Run the same scenarios with ASM approximate friction (linearized cone):
- If ASM passes, the issue is ECOS-specific (formulation or integration)
- If ASM also fails, the issue is upstream (contact detection, effective mass, etc.)

---

## Verdict

**FAILED**

All 4 physics integration tests remain failing after implementation of upstream fixes (0035d1, 0035d2). The ECOS solver is mathematically correct in isolation, but the WorldModel integration is faulty. **Root cause is in the two-phase solve pipeline (0035d5)**.

**This ticket cannot advance until the two-phase pipeline issues are resolved.**

**Recommended next ticket**: Create new bug fix ticket to investigate and fix the two-phase solve pipeline force extraction/combination bugs.

---

## Artifacts

- Verification report: `docs/designs/0035d8_physics_integration_test_failures/verification-report.md`
- Quality gate report: `docs/designs/0035d8_physics_integration_test_failures/quality-gate-report.md`
- Test output: Captured in verification report

---

## Workflow Status

- Build: PASS ✓
- Tests: FAIL ✗ (4/538 tests failing)
- Benchmarks: N/A
- **Overall Verdict**: FAILED
- **Iteration Count**: 0 (first attempt)
- **Re-run implementer**: No (this is a verification ticket, not an implementation ticket)
- **Next step**: Create new investigation/fix ticket for two-phase solve pipeline bugs
