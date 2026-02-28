# Implementation Notes — 0086_split_step_block_pgs

**Ticket**: 0086_split_step_block_pgs
**Branch**: 0084-block-pgs-solver-rework
**Implementation Date**: 2026-02-28
**Status**: Implemented — Partial success, escalation required

---

## Summary

Implemented the split-step two-pass Block PGS solver as specified in the design document.
The implementation replaces the P4 asymmetric-decoupled 3x3 block solve with a split-step
algorithm: Pass 1 solves normal impulses using scalar K_nn, Pass 2 solves tangent impulses
using 2x2 K_tt^{-1} after all normal impulses have been applied.

The implementation is correct and produces measurably different physics for sphere-sphere
collision tests (better energy conservation for elastic collisions). However, the acceptance
criteria tests for floor contacts (FrictionProducesTippingTorque, TiltedDropTest, SlidingCube)
still fail with identical failure signatures, indicating their root cause is NOT K_nt coupling
but rather multi-point contact geometry asymmetry.

---

## Files Created

None.

## Files Modified

### `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.hpp`

**Changes**:
1. Updated ticket comment to include 0086
2. Changed `sweepOnce` parameter: `const std::vector<Eigen::Matrix3d>& blockKInvs` →
   `const std::vector<Eigen::Matrix2d>& tangentKInvs`
3. Updated Doxygen for `sweepOnce` to describe the split-step two-pass algorithm
4. Added `updateVResTangentOnly` private method declaration with Doxygen

**Lines**: 22 lines net change

### `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.cpp`

**Changes**:
1. Updated ticket/design header comment to reference 0086
2. Replaced 3x3 LDLT `blockKInvs` pre-computation with 2x2 LDLT `tangentKInvs`
   on `K.block<2,2>(1,1)` tangent sub-block
3. Updated `sweepOnce` call to pass `tangentKInvs`
4. Added `updateVResTangentOnly` implementation (delegates to `updateVRes3` with dN=0)
5. Rewrote `sweepOnce` as two-pass split-step (Pass 1 normals, Pass 2 tangents)
6. Retained `projectCoulombCone` as unused static utility (per design reviewer guidance)

**Lines**: 217 lines net change

---

## Design Adherence Matrix

| Design Item | Status | Notes |
|-------------|--------|-------|
| Pre-compute 2x2 K_tt LDLT | Implemented | K.block<2,2>(1,1), fallback to zero |
| updateVResTangentOnly | Implemented | Delegates to updateVRes3 with dN=0 (DD-0086-002) |
| sweepOnce Pass 1 (normals) | Implemented | scalar K_nn, max(0,...) clamp |
| sweepOnce Pass 2 (tangents) | Implemented | 2x2 K_tt_inv, fixed lambda_n from Pass 1 |
| Coulomb cone projection in Pass 2 | Implemented | lambda_n FIXED from Pass 1 (DD-0086-003) |
| Single maxDelta from both passes | Implemented | max(|delta_n|, ||delta_t||) (DD-0086-004) |
| epsilon threshold = 1e-24 | Implemented | Matches existing code pattern |
| Phase A unchanged | Preserved | No modifications to applyRestitutionPreSolve |
| Warm-start unchanged | Preserved | No modifications to warm-start logic |
| Force extraction unchanged | Preserved | No modifications to lambda assembly |
| projectCoulombCone retained | Preserved | With note it is no longer called from sweepOnce |
| External interface unchanged | Confirmed | solve() signature identical |

---

## Prototype Application Notes

No formal prototype was run — per the design, "the prototype IS the implementation."
The design reviewer's guidance was followed: run `FrictionProducesTippingTorque` as
the first validation. The result was the go/no-go gate failure described in the
Outcome section.

---

## Deviations from Design

None. The implementation exactly matches the design specification pseudocode.

---

## Test Coverage Summary

**Build**: PASS, no warnings.
**Test suite**: 772/789 pass — same as baseline (17 failing).

### Tests That Changed Behavior (split-step working correctly)

| Test | Old behavior | New behavior | Status |
|------|-------------|--------------|--------|
| VelocitySwap KE | 1.522J | 1.999J | Physically correct (e=1 → conserve energy). Pinned value needs update. |
| ZeroGravity finalKE | 1.748J | 1.923J | Better energy conservation. Pinned value needs update. |
| VelocitySwap omegaAz | 0.198 rad/s | 3.14 rad/s | Dramatic change. Pinned value needs update. |
| PerfectlyElastic maxHeight | ~0.7m | Different | Values changed. |

### Tests That Still Fail (root cause is NOT K_nt)

| Test | Failure | Root Cause Hypothesis |
|------|---------|----------------------|
| FrictionProducesTippingTorque | omegaZ=0.088 dominates omegaY=6.78e-7 | 4-corner contact asymmetry generates Z torque independent of solver |
| TiltedDropTest (9 tests) | Cross-axis ratio 18.8% > 5% threshold | Polyhedral icosphere off-axis normals; multi-contact asymmetry |
| SphereDrop_NoRotation | maxOmega=16.9 rad/s (expected <0.5) | Icosphere polyhedral contacts create off-axis impulses |
| SlidingCubeX/Y | 0.026m lateral drift | 4-corner sliding asymmetry |
| InelasticBounce | ratio=0.057 < 0.125 | Tangential over-dissipation from polyhedral contact |

---

## Outcome

### What Worked

The split-step correctly eliminates K_nt algebraic coupling from the tangent solve.
For sphere-sphere collision tests, energy conservation improves significantly for
elastic collisions (e=1). The 2x2 K_tt^{-1} solve is physically correct and well-conditioned.

### What Did Not Work

The FrictionProducesTippingTorque go/no-go gate FAILED. The tipping torque failure
is NOT due to K_nt coupling — it is due to 4-corner floor contact geometry producing
net Z-axis torque independent of the solver algorithm.

### Mathematical Insight

For contacts where vErr(0) ≈ 0 (steady-state non-penetrating contacts in sliding mode),
the difference between the old code (`K_inv.block<2,3>(1,0) * (-vErr)`) and the new
code (`K_tt_inv * (-vErr_t)`) vanishes: the K_nt term `K_inv(1:2,0)*(-vErr(0))` is
near zero. This means the split-step and old code produce essentially IDENTICAL
tangent impulses in steady-state sliding — explaining why the sliding tests are unchanged.

The split-step makes a difference only for transient contacts where vErr(0) is
substantially non-zero (initial impacts, bouncing). For these cases, the new code
correctly separates the normal and tangent solves.

---

## Escalation

**Decision required**: The acceptance criteria cannot be fully met by the split-step.
Options:
1. Keep split-step + update pinned values for Category A tests (improves energy conservation
   for collision tests, ~5 tests would transition from fail to pass after re-pinning)
2. Investigate root causes of Category B failures (contact geometry, tangent basis) separately
3. Revert to P4 if the split-step's improved energy conservation is not worth the complexity

Per design document guidance: if FrictionProducesTippingTorque fails, "the split-step
hypothesis is invalidated — revert and reassess."

---

## Known Limitations

1. `projectCoulombCone` static method is no longer called from `sweepOnce` but retained.
2. The split-step calls `computeBlockVelocityError` twice per contact per sweep (once per
   pass) vs once in the old code — 2x velocity error computation cost per sweep.
3. The 2x2 K_tt LDLT is cheaper than the 3x3 LDLT it replaces in pre-computation.

---

## Future Considerations

If the split-step is kept:
- Update pinned values for VelocitySwap (vAxFinal≈0.332, vBxFinal≈1.668, omegaAz≈3.14,
  finalKE≈1.999) and ZeroGravity (finalKE≈1.923, velocities differ from pinned)
- Investigate contact manifold geometry for 4-corner floor contacts to fix tipping torque
- Investigate icosphere polyhedral normal off-axis effects on rotation tests
