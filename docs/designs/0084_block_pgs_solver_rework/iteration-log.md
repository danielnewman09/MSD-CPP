# Iteration Log: Block PGS Solver Rework

**Ticket**: [0084_block_pgs_solver_rework](../../../tickets/0084_block_pgs_solver_rework.md)

---

## Iteration 1 — Prototype P1: Warm-Start Disable Diagnostic

**Date**: 2026-02-28
**Phase**: Prototype
**Change**: Set `const bool hasWarmStart = false` in `BlockPGSSolver::solve()`
**Goal**: Confirm that warm-start contamination is the root cause of all 12 failures

### Result: NEGATIVE

All 12 tests still fail with identical failure values:

| Test | Z-Vel / KE ratio before | After warm-start disable | Change |
|------|------------------------|--------------------------|--------|
| Oblique45 | 21.1 m/s | 21.1 m/s | None |
| HighSpeedOblique | 43.4 m/s | 43.4 m/s | None |
| Oblique45_Medium | 10.0 m/s | 10.0 m/s | None |
| Oblique45_Slow | 3.66 m/s | 3.66 m/s | None |
| InelasticBounce | 0.057 ratio | 0.057 ratio | None |
| EqualMassElastic | omegaZ=3.14 | omegaZ=3.14 | None |

### Finding

The Z-velocity injection occurs within the first 10 simulation frames, independent of
warm-start state. This is a per-frame Phase B problem. The K_nt off-diagonal coupling
in `K_inv` maps tangential velocity error (`vErr(1,2) != 0`) to a normal impulse
correction (`unconstrained(0) != 0`) even when `vErr(0) = 0`.

### Next Step

Design revision required. The warm-start fix (F1) does not address the actual root
cause. Need to investigate:
1. Zero the normal correction when `vErr(0) >= 0` (contact not approaching)
2. Or use a decoupled normal/tangent solve

---

## Iteration 2 — Design Revision: Decoupled Normal/Tangent Solve

**Date**: 2026-02-28
**Phase**: Design Revision
**Change**: Revised design document and PlantUML diagram. New root cause identified:
K_nt off-diagonal coupling in 3x3 block solve drives normal impulse from tangential
velocity error on every frame. Fix: decouple normal row (scalar K_nn) from tangent
rows (2x2 K_tt subblock) in Phase B `sweepOnce`.

### Human Decision

The human reviewed the prototype P1 findings and approved the following approach:
- **Full decoupled solve**: scalar K_nn for normal row, 2x2 K_tt for tangent rows independently
- **Do NOT use Hypothesis B** (zeroing `unconstrained(0)` when `vErr(0) >= 0`) — rejected as
  a band-aid
- **Preserve two-phase architecture** (Phase A restitution + Phase B dissipative)
- **Prototype P2 required** to validate decoupled solve before full implementation

### Design Changes from Revision 1

| What changed | Old approach | New approach |
|---|---|---|
| Primary fix | F1: Phase B-only cache storage (phaseBLambdas) | F1 (revised): Decoupled normal/tangent solve in sweepOnce |
| Root cause | Warm-start contamination (Phase A bounce in cache) | K_nt coupling in 3x3 block solve drives normal impulse from tangential vErr |
| Files changed for primary fix | BlockPGSSolver.hpp, BlockPGSSolver.cpp, ConstraintSolver, CollisionPipeline | BlockPGSSolver.cpp only (sweepOnce function) |
| Secondary fix | N/A | phaseBLambdas cache split retained as secondary correctness improvement |
| Previous blockKInvs | Precomputed 3x3 K^{-1}, passed to sweepOnce | Removed; use blockKs directly with per-contact scalar/2x2 LDLT |

### Prototype P2 Target

Before full implementation, prototype P2 must validate:
- Replace `blockKInvs[ci] * (-vErr)` with decoupled scalar/2x2 solve in sweepOnce
- All 5 oblique sliding tests must pass (Z-velocity < 2.0 m/s)
- At least 9 of 12 total failing tests must pass
- Zero regression in a sample of currently-passing tests

---

## Iteration 3 — Prototype P2: Decoupled Solve Validation

**Date**: 2026-02-28
**Phase**: Prototype
**Change**: Implemented decoupled solve in `sweepOnce` as specified in design revision:
  `delta_lambda_n = (-vErr(0)) / K_nn`  (scalar normal)
  `delta_lambda_t = K_tt.ldlt().solve(-vErr_t)`  (2x2 tangent)
**Goal**: Validate decoupled solve fixes oblique sliding without regressions

### Variants Tested

| Variant | Normal row | Tangent rows | SlidingCubeX | Oblique45 | SlidingCube_TippingTorque |
|---------|-----------|--------------|--------------|-----------|---------------------------|
| Original (coupled) | K_inv(0,:)*(-vErr) | K_inv(1:2,:)*(-vErr) | PASS | FAIL | PASS |
| P2-V1: K_tt.ldlt() | (-vErr(0))/K_nn | K_tt^{-1}*(-vErr_t) | FAIL | PASS | FAIL |
| P2-V2: K_inv.block<2,2>(1,1) | (-vErr(0))/K_nn | K_inv.block<2,2>(1,1)*(-vErr_t) | FAIL | PASS | FAIL |
| P2-V3: K_inv(0,0)*(-vErr(0)) | K_inv(0,0)*(-vErr(0)) | K_inv(1:2,:)*(-vErr) | FAIL | PASS | FAIL |

### Result: NEGATIVE — new regressions introduced

**Tests fixed (6 of original 12 failures now pass):**
- SlidingCube_ConeCompliantEveryFrame_Oblique45_Slow: PASS
- SlidingCube_ConeCompliantEveryFrame_Oblique45_Medium: PASS
- SlidingCube_ConeCompliantEveryFrame_Oblique45: PASS
- SlidingCube_ConeCompliantEveryFrame_HighSpeedOblique: PASS
- ParameterIsolation_TimestepSensitivity_ERPAmplification: PASS
- RotationDampingTest_RockingCube_AmplitudeDecreases: PASS

**Tests still failing (6 of original 12):**
- FrictionWithRestitution_BounceThenSlide: FAIL
- InelasticBounce_KEReducedByESquared: FAIL
- PerfectlyElastic_EnergyConserved: FAIL
- EqualMassElastic_VelocitySwap: FAIL
- SphereDrop_NoRotation: FAIL
- ZeroGravity_RotationalEnergyTransfer_Conserved: FAIL

**New regressions (originally passing, now failing):**
- SlidingCubeX_DeceleratesAndStops: FAIL (lateral drift 0.019m vs 0.002m threshold)
- SlidingCubeY_DeceleratesAndStops: FAIL (lateral drift 0.022m vs 0.002m)
- SlidingCube_FrictionProducesTippingTorque: FAIL (omega_Z=0.066 dominates instead of omega_Y)

### Root Cause of Regressions

The K_tt^{-1} tangent solve loses the Schur complement correction:
  `K_tt^{-1}` is the inverse of the diagonal block only.
  `K_inv.block<2,2>(1,1)` = `(K_tt - K_nt * K_nn^{-1} * K_nt^T)^{-1}` (Schur complement).

Both variants of tangent solve produce wrong friction torques for corner contacts because
they omit the K_inv(1:2, 0)*(-vErr(0)) normal-to-tangent coupling terms. For a cube sliding
in X on a flat floor, these cross-terms provide per-contact force balance that ensures correct
tipping torque (omega_Y >> omega_Z). Without them, Z-rotation dominates instead.

Specifically: `FrictionProducesTippingTorque` requires omega_Y > omega_Z.
With decoupled tangent: omega_Z = 0.066 >> omega_Y ≈ 0 — complete torque inversion.
With original code: omega_Y >> omega_Z — correct physics.

The coupling terms K_inv(1,0)*(-vErr(0)) and K_inv(2,0)*(-vErr(0)) are REQUIRED
for correct per-contact angular impulse balance across the 4 corner contacts.
These are the same terms the design revision intended to KEEP (rows 1-2 coupling preserved),
but even keeping full rows 1-2 doesn't fix it — the scalar K_inv(0,0) in the normal row
changes the Coulomb cone bound, which changes what tangent impulses get through,
which causes wrong angular impulse balance.

### Additional Finding: Restitution Failures Are Warm-Start Related

Testing with warm-start disabled reveals:
- InelasticBounce: PASS (with warm-start disabled)
- PerfectlyElastic: PASS (with warm-start disabled)
- EqualMassElastic: PASS (with warm-start disabled)
- SphereDrop_FlatContact_MinimalTorque: FAIL (new failure without warm-start)

This means the 3 restitution failures (InelasticBounce, PerfectlyElastic, EqualMassElastic)
are caused by warm-start interfering with bounce frame dynamics. Fix F2 (phaseBLambdas —
cache only Phase B lambdas) would likely address these 3 remaining failures.

### Conclusion

The decoupled solve as specified fixes oblique sliding (primary fix) but introduces
wrong-torque-axis failures for axis-aligned corner contacts. The full K_inv rows are
needed for correct per-contact angular impulse balance. Design revision required to
address the coupling incompatibility.

Possible directions for Design Revision 2:
1. Apply decoupled normal row ONLY for contacts where K_nt is large (oblique geometry
   detector based on off-diagonal K magnitude)
2. Keep full K_inv coupled solve but normalize the normal row contribution to prevent
   unbounded growth — e.g., cap lambda_n growth per frame
3. Investigate whether the Coulomb cone projection itself can be modified to prevent
   normal lambda from growing via coupling (cone clamping at warm-start level)

### Next Step

Design revision required before proceeding to implementation. The decoupled solve
does not satisfy the zero-regression requirement.

---

## Iteration 4 — Design Revision 2: Revert P2 + Implement Fix F2

**Date**: 2026-02-28
**Phase**: Design Revision + Prototype
**Change**: Reverted P2 decoupled solve. Implemented Fix F2 (phaseBLambdas warm-start cache
split). Ran full test suite to validate.
**Goal**: Confirm Fix F2 fixes InelasticBounce, PerfectlyElastic, EqualMassElastic without
regressions (per human approval of Option A from Design Revision 2 decision).

### Changes Made

1. **Reverted P2**: Restored coupled K_inv solve in `sweepOnce`:
   - `sweepOnce` parameter restored to `blockKInvs` (K^{-1})
   - `blockKInvs` precomputation loop restored in `solve()`
   - Coupled solve: `unconstrained = blockKInvs[ci] * (-vErr)` restored

2. **Fix F2 implemented**:
   - `BlockPGSSolver::SolveResult::phaseBLambdas` added
   - `ConstraintSolver::SolveResult::warmStartLambdas` added
   - BlockPGS path wires `warmStartLambdas = phaseBLambdas`
   - ASM/PGS paths wire `warmStartLambdas = lambdas`
   - `CollisionPipeline` uses `globalWarmStartLambdas` for cache write

### Result: NEGATIVE — Fix F2 does not fix the 3 restitution tests

**Test results**: 768 passing, 12 failing (unchanged — same 12 as before Fix F2)

**Critical diagnostic**: Disabling warm-start entirely (`hasWarmStart = false` diagnostic)
with the restored coupled K_inv solve shows the 3 restitution tests **still fail**:
- `InelasticBounce`: ratio = 0.057 (independent of warm-start state)
- `EqualMassElastic`: omegaZ = 3.14 rad/s (independent of warm-start state)

This disproves the Iteration 3 finding that "InelasticBounce passes with warm-start disabled".
That finding was **only true with the P2 decoupled solve active**, not with the coupled solve.

### Root Cause Update

With the coupled K_inv solve, ALL 12 failures are caused by the K_nt coupling in Phase B's
block solve. The warm-start is NOT the root cause for the restitution tests. Even on the first
bounce frame (frame where contact first occurs, no prior warm-start), the K_nt coupling causes
the omegaZ = 3.14 spurious angular velocity because:
- Phase A computes a bounce impulse (normal-only, correct)
- Phase B runs 50 sweeps
- Each Phase B sweep computes `unconstrained = K_inv * (-vErr)`
- `K_inv(1,0)` and `K_inv(2,0)` are non-zero → normal row drives tangential correction
- `K_inv(0,1)` and `K_inv(0,2)` are non-zero → tangential vErr drives normal correction
- The spurious normal correction injects Z-velocity (oblique) or spurious angular impulse
  (elastic collision) on every sweep of every frame

Fix F2 (phaseBLambdas) is semantically correct as a secondary improvement and causes no
harm. It is retained in the implementation. However, it does not address the root cause.

### Next Step

Design Revision 3 required. The core problem is a structural incompatibility in the coupled
3x3 block solve:
- Full K_inv solve: correct angular impulse balance, but K_nt injects spurious normal impulse
- Decoupled solve: eliminates K_nt injection, but breaks angular impulse balance in tipping tests

Need to find an approach that addresses K_nt normal injection WITHOUT losing the angular
impulse balance that K_inv(1:2, 0) cross-terms provide.

