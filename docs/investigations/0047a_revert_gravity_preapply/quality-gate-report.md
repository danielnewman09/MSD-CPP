# Quality Gate Report: 0047a_revert_gravity_preapply

**Date**: 2026-02-10
**Branch**: 0047a-revert-gravity-preapply
**Commit**: be904fe

---

## Gate 1: Build Verification ✅ PASS

**Command**: `cmake --build --preset conan-debug`

**Result**: Clean build with no warnings or errors

**Files Modified**:
- `msd/msd-sim/src/Environment/WorldModel.cpp`
- `msd/msd-sim/src/Environment/CLAUDE.md`
- `msd/msd-sim/test/Physics/Collision/ContactManifoldStabilityTest.cpp`
- `msd/msd-sim/test/Physics/Collision/RotationalCollisionTest.cpp`

**Outcome**: ✅ PASS

---

## Gate 2: Test Verification ✅ PASS (with documented exceptions)

**Command**: `./build/Debug/debug/msd_sim_test`

**Results**:
```
[==========] 693 tests from 73 test suites ran.
[  PASSED  ] 689 tests.
[  FAILED  ] 4 tests
```

**Failed Tests**:
1. `ContactManifoldStabilityTest.D4_MicroJitter_DampsOut`
2. `ParameterIsolation.H3_TimestepSensitivity_ERPAmplification`
3. `RotationalCollisionTest.B2_CubeEdgeImpact_PredictableRotationAxis`
4. `RotationalCollisionTest.B5_LShapeDrop_RotationFromAsymmetricCOM`

### Failure Analysis

**D4 (NEW, DOCUMENTED)**:
- **Status**: Expected failure, documented in test source
- **Root Cause**: Test assumes gravity pre-apply for micro-jitter damping
- **User Decision**: Accept failure with rationale: "The micro-jitter failure is due to the velocity being exactly gravity after one timestep, which I'm not sure has a desirable solution."
- **Documentation**: Added comprehensive comment explaining why this test fails without gravity pre-apply

**H3, B2, B5 (PRE-EXISTING)**:
- **Status**: Pre-existing failures on main branch (present before this change)
- **Root Cause**: Unrelated to gravity pre-apply removal
- **Impact**: No change from baseline

### Comparison with Main Branch

| Test | Main (with pre-apply) | This Branch (without) | Change |
|------|-----------------------|-----------------------|--------|
| D1 (Resting stability) | ✅ PASS | ✅ PASS | No regression |
| D4 (Micro-jitter) | ✅ PASS | ❌ FAIL | **Documented exception** |
| H1 (Restitution off) | ✅ PASS | ✅ PASS | No regression |
| B3 (Sphere rotation) | ❌ FAIL | ✅ PASS | **FIX** |
| H3 (Timestep sensitivity) | ❌ FAIL | ❌ FAIL | Pre-existing |
| B2 (Edge impact) | ❌ FAIL | ❌ FAIL | Pre-existing |
| B5 (L-shape) | ❌ FAIL | ❌ FAIL | Pre-existing |

**Summary**: 689/693 pass (same as main). One regression (D4, documented as expected), one fix (B3, documented improvement).

**Outcome**: ✅ PASS (689/693 is baseline performance, D4 failure is documented and accepted)

---

## Gate 3: Code Quality ✅ PASS

### Documentation Quality

**WorldModel.cpp**:
- ✅ Clear comments explaining revert rationale
- ✅ Ticket references present (0047a)
- ✅ Physics update order simplified and correct

**CLAUDE.md**:
- ✅ Updated WorldModel update order (removed pre-apply step)
- ✅ Documented SAT fallback as the true resting contact fix
- ✅ Added historical note explaining pre-apply removal

**Test Files**:
- ✅ D4: Comprehensive comment explaining known failure
- ✅ B3: Comment documenting fix (no spurious rotation)
- ✅ Both comments reference ticket 0047a

### Code Standards Compliance

- ✅ Follows project coding standards (brace initialization, NaN for floats)
- ✅ No new raw pointers or memory leaks
- ✅ Proper use of references vs values
- ✅ Clear variable naming

**Outcome**: ✅ PASS

---

## Overall Quality Gate Result: ✅ PASS

All gates passed with documented exceptions. The implementation:
1. Builds cleanly with no warnings
2. Maintains baseline test performance (689/693)
3. Fixes B3 (sphere rotation regression from gravity pre-apply)
4. D4 failure is documented and accepted by user decision
5. All changes properly documented in code and CLAUDE.md

**Recommendation**: Proceed to Implementation Review phase.

---

## Notes

This ticket removes unnecessary complexity (gravity pre-apply) while maintaining physics correctness. The SAT fallback (from ticket 0047) is the true fix for resting contacts. The D4 micro-jitter test expects aggressive damping that requires gravity pre-apply, but this is a test design assumption rather than a physics requirement.

The user explicitly accepted the D4 failure with clear rationale, and the physics is demonstrably correct (D1 and H1 resting contact tests pass without pre-apply).
