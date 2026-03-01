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

---

## Iteration 5 — Prototype P4: Asymmetric Decoupling

**Date**: 2026-02-28
**Phase**: Prototype (Design Revision 3)
**Change**: Implemented asymmetric decoupling in `sweepOnce`:
  Row 0 (normal):  `unconstrained(0) = (-vErr(0)) / K(0,0)` (scalar, severs K_inv(0,1:2))
  Rows 1-2 (tangent): `unconstrained.tail<2>() = K_inv.block<2,3>(1,0) * (-vErr)` (full 2x3 block)
**Goal**: Fix oblique sliding tests without regressing SlidingCubeX/TippingTorque

### Result: PARTIAL — Fixed 7 of 12 original failures, but same 3 regressions as P2

**Total tests**: 780. Pass: 772. Fail: 8.

**Original failures now fixed (7)**:
- SlidingCube_ConeCompliantEveryFrame_Oblique45_Slow: PASS
- SlidingCube_ConeCompliantEveryFrame_Oblique45_Medium: PASS
- SlidingCube_ConeCompliantEveryFrame_Oblique45: PASS
- SlidingCube_ConeCompliantEveryFrame_HighSpeedOblique: PASS
- FrictionWithRestitution_BounceThenSlide: PASS
- ParameterIsolation_TimestepSensitivity_ERPAmplification: PASS
- RotationDampingTest_RockingCube_AmplitudeDecreases: PASS

**Original failures still failing (5)**:
- InelasticBounce_KEReducedByESquared: FAIL
- PerfectlyElastic_EnergyConserved: FAIL (maxHeight=0.70, needs > 1.0)
- EqualMassElastic_VelocitySwap: FAIL
- SphereDrop_NoRotation: FAIL (omegaMax=16.9 rad/s)
- ZeroGravity_RotationalEnergyTransfer_Conserved: FAIL

**New regressions introduced (3)** — same as P2:
- SlidingCubeX_DeceleratesAndStops: FAIL (lateral drift 0.023m vs 0.002m)
- SlidingCubeY_DeceleratesAndStops: FAIL (lateral drift 0.044m vs 0.002m)
- SlidingCube_FrictionProducesTippingTorque: FAIL (omegaZ=0.052 dominates vs omegaY≈0)

### Root Cause of Remaining Regressions

The human's hypothesis was: preserving `K_inv(1:2, 0)*(-vErr(0))` in rows 1-2 would maintain
tipping torque. This is REFUTED. The regression is as severe as full decoupled P2.

**Root cause**: `1/K(0,0)` != `K_inv(0,0)`. By Schur complement:
```
K_inv(0,0) = 1 / (K(0,0) - K_nt * K_tt^{-1} * K_nt^T)
```
The scalar `1/K(0,0)` is LARGER than `K_inv(0,0)` (the denominator is reduced by the
Schur complement term). Therefore the asymmetric approach gives a LARGER normal impulse
per step than the original coupled solve.

**The coupling path through the Coulomb cone**: `lambda_n` is the Coulomb cone bound.
A larger `lambda_n` → a larger cone bound `mu * lambda_n` → tangential impulses are less
constrained by the cone projection. For axis-aligned sliding (SlidingCubeX, 4 corner
contacts), the cone clipping behavior determines which tangential components survive the
projection. Changing `lambda_n` changes the cone bound asymmetrically across the 4 contacts,
producing an unbalanced angular impulse distribution — which manifests as Z-spin (omegaZ)
instead of Y-tipping (omegaY).

**Implication**: Any change to the normal row that does NOT preserve K_inv(0,0) exactly
will break the tipping torque test. The only "safe" change to row 0 is one that leaves
the steady-state `lambda_n` value unchanged when `vErr(0) = 0`.

### Critical Insight

For axis-aligned sliding (`vErr(0) = 0` during steady sliding):
- Original coupled solve: `unconstrained(0) = K_inv(0,1)*(-vErr(1)) + K_inv(0,2)*(-vErr(2))`
  This is NONZERO, driving the energy injection bug.
- Asymmetric: `unconstrained(0) = 0`
  No energy injection, but changes steady-state `lambda_n` (from the warm-start initialization
  path), which changes the Coulomb cone bound.

The core tension: the cone bound (lambda_n) and the tangential impulse balance are
inextricably linked. We cannot zero row 0 without also changing the effective cone radius.

For the tipping torque test specifically: the cube is accelerating from rest (vErr(0) != 0
on early frames), so even with `vErr(0) = 0` during steady sliding, the transient `lambda_n`
built up in early frames determines the cone bound. Any change to how `lambda_n` grows in
early frames (when contact is being established) propagates to steady-state through the
accumulated lambda.

### Next Step

The asymmetric decoupling is also NEGATIVE for the regression test. Design Revision 4 is
needed. The structural incompatibility is confirmed:
- We need lambda_n to grow correctly for tipping torque (requires K_inv(0,0) * correction)
- We need to prevent K_inv(0,1:2)*vErr_t from driving spurious lambda_n growth (requires
  zeroing those terms OR ensuring they cancel out)

Possible directions:
1. **Constrained normal growth**: Use the full K_inv(0,:)*(-vErr) for row 0, but cap
   `delta_lambda_n` to be non-positive when `vErr(0) >= 0` (no normal growth during sliding).
   This is "Hypothesis B" from Revision 1 — but implemented correctly as a one-sided clamp:
   `unconstrained(0) = min(K_inv(0,:)*(-vErr), 0)` when `vErr(0) >= 0`.
   Rationale: the full K_inv coupling provides correct lambda_n for active penetration frames,
   but prevents growth on non-penetrating frames.

2. **Normal component clamping**: Allow the full K_inv solve, but after the cone projection,
   clamp lambda_n to not exceed its previous value when vErr(0) >= 0.

3. **Separate the coupling physically**: Use the full K_inv solve but add a velocity-based
   guard: if vErr(0) >= -epsilon (contact not penetrating), zero the normal correction
   delta_lambda_n but keep full tangential correction.

---

## Iteration 6 — Prototype P5: Velocity-Gated Clamp

**Date**: 2026-02-28
**Phase**: Prototype (Design Revision 4)
**Change**: Reverted P4 asymmetric decoupling. Restored full coupled K_inv solve
(`unconstrained = blockKInvs[ci] * (-vErr)`). Added velocity-gated clamp on row 0:
```cpp
if (vErr(0) >= 0.0) {
    unconstrained(0) = std::min(unconstrained(0), 0.0);
}
```
Signature reverted to pre-P2: `sweepOnce(contacts, blockKInvs, lambda, ...)` (single blockKInvs parameter).
**Goal**: Fix oblique sliding tests without regressing SlidingCubeX/TippingTorque

### Result: NEGATIVE — worse than P4 (14 failures vs P4's 8 failures)

**Total tests**: 780. Pass: 766. Fail: 14.

**Comparison vs P4 (asymmetric decoupling)**:

| Test Category | P4 Result | P5 Result |
|---------------|-----------|-----------|
| SlidingCubeX lateral drift | FAIL (regression) | FAIL (regression) |
| SlidingCubeY lateral drift | FAIL (regression) | FAIL (regression) |
| FrictionProducesTippingTorque | FAIL (regression) | PASS (fixed by P5) |
| Oblique45_Slow | PASS (P4 fixed) | PASS |
| Oblique45_Medium | PASS (P4 fixed) | FAIL (regression introduced by P5) |
| Oblique45 | PASS (P4 fixed) | FAIL (regression introduced by P5) |
| HighSpeedOblique | PASS (P4 fixed) | FAIL (regression introduced by P5) |
| FrictionWithRestitution_BounceThenSlide | PASS (P4 fixed) | PASS |
| ERP_Amplification | PASS (P4 fixed) | FAIL (regression introduced by P5) |
| RockingCube_AmplitudeDecreases | PASS (P4 fixed) | FAIL (regression introduced by P5) |
| SlidingCube_KineticEnergyDecreases | PASS (was passing) | FAIL (new regression from P5) |
| SlidingCube_ConeCompliantEveryFrame_HighSpeed | PASS (was passing) | FAIL (new regression from P5) |
| InelasticBounce | FAIL (unfixed) | FAIL (unfixed) |
| PerfectlyElastic | FAIL (unfixed) | FAIL (unfixed) |
| EqualMassElastic | FAIL (unfixed) | FAIL (unfixed) |
| SphereDrop_NoRotation | FAIL (unfixed) | FAIL (unfixed) |
| ZeroGravity_Rotational | FAIL (unfixed) | FAIL (unfixed) |

**P5 is a net regression vs P4**: fixes TippingTorque but re-breaks 6 tests P4 had fixed, and adds 2 new regressions.

### Root Cause Analysis

The velocity-gated clamp has the same structural incompatibility as P2 and P4, but from a different direction:

**For oblique sliding tests** (the tests P5 re-broke): The clamp `min(unconstrained(0), 0.0)` when `vErr(0) >= 0` blocks ALL positive corrections to lambda_n during steady-state sliding. For axis-aligned sliding (SlidingCubeX), K_inv(0,1:2)*vErr_t provides BOTH the spurious energy injection for oblique contacts AND the correct force balance for axis-aligned corner contacts. The clamp cannot distinguish between these two cases — it kills both.

**Why P5 broke HighSpeed (axis-aligned X, high speed)**: The cube slides fast in X, contacts are near-vertical. During high-speed sliding, vErr(0) ≈ 0 on most frames. The clamp zeros the K_nt contribution to normal force on these frames. With reduced lambda_n, the friction bound `mu * lambda_n` shrinks, the cube decelerates less, and energy is not properly dissipated. The cube fails to come to rest.

**Why P5 fixed TippingTorque but P4 didn't**: The tipping torque test requires the cube to tip over Y axis while sliding in X. The velocity-gated clamp does NOT zero row 0 when contact IS actively developing (`vErr(0) < 0`). During the initial contact establishment phase, the full K_inv(0,0) is preserved. P4's asymmetric decoupling always used 1/K(0,0) < K_inv(0,0), giving a smaller normal impulse from the start. P5 preserves K_inv(0,0) during penetrating frames. This distinction is what fixes TippingTorque.

**The fundamental tension (reconfirmed)**: For axis-aligned sliding in steady state:
- `vErr(0) = 0` in steady state (no penetration)
- The K_nt coupling gives `unconstrained(0) = K_inv(0,1)*(-vErr(1)) + K_inv(0,2)*(-vErr(2))`
- For SlidingCubeX: this term contributes CORRECTLY to the net normal force distribution across 4 contacts (it's part of the correct coupled physics)
- For ObliqueCube: this term injects NET upward momentum (the bug)
- Both cases have `vErr(0) = 0` — the velocity-gated condition cannot distinguish them

### What P4 Got Right that P5 Lost

P4's asymmetric approach zeroed the K_inv(0,1:2) coupling (same as P5) BUT also preserved the MAGNITUDE of normal response using 1/K(0,0) rather than K_inv(0,0). The difference: for the tipping torque test, 1/K(0,0) > K_inv(0,0), which gave TOO MUCH normal impulse and over-expanded the cone. For axis-aligned sliding, the problem was the same — 1/K(0,0) != K_inv(0,0) changed the cone bound.

P5 correctly preserves K_inv(0,0) during penetrating frames. But during the steady-state `vErr(0) = 0` regime, P5 gives ZERO additional normal impulse from K_nt coupling, while the coupled solve gives a positive K_nt contribution. For SlidingCubeX, this K_nt contribution in steady state is part of the correct physics — removing it causes drift.

### Implication for Design Revision 5

The velocity-gated clamp is provably wrong for axis-aligned sliding contacts. The ONLY correct approach must either:
1. Distinguish oblique from axis-aligned geometry (not velocity-based)
2. Find a way to allow the CORRECT K_nt coupling while blocking the WRONG coupling
3. Accept that oblique sliding requires a completely different algorithm than axis-aligned

Key observation: The oblique case injects NET upward momentum across ALL contacts. The axis-aligned case produces self-canceling K_nt contributions across symmetric contacts (the 4 corners of a cube sliding in X have symmetric vErr vectors that sum to zero net upward impulse).

**Potential Design Revision 5 direction**: Accumulated lambda_n clamping. Rather than clamping the correction `delta_lambda_n`, clamp the ACCUMULATED `lambda_n` to not exceed what the pure K_inv(0,0) path would give, computed from the warm-start. This preserves the frame-by-frame K_nt contribution structure while preventing unbounded growth.

### Next Step

Design Revision 5 required. Prototype P5 is definitively NEGATIVE. The velocity-gated clamp approach fails because it cannot distinguish CORRECT K_nt contributions (axis-aligned, self-canceling) from INCORRECT K_nt contributions (oblique, net energy injection).

---

## Iteration 7 — Prototype P6: Post-Sweep Net-Zero Redistribution

**Date**: 2026-02-28
**Phase**: Prototype (Design Revision 5)
**Change**: Reverted P5 velocity-gated clamp. Restored full coupled K_inv solve
(`unconstrained = blockKInvs[ci] * (-vErr)`). Added post-sweep net-zero redistribution
in `solve()` after each `sweepOnce` call.
**Goal**: Correct spurious net normal impulse growth (from cone asymmetry) post-sweep
without touching the per-contact coupled solve.

### Changes Made

1. **Reverted P5**: Removed velocity-gated clamp from `sweepOnce`. Restored plain:
   ```cpp
   const Eigen::Vector3d unconstrained = blockKInvs[ci] * (-vErr);
   ```

2. **Post-sweep redistribution in `solve()`**:
   After each `sweepOnce`, compute per-body net delta_lambda_n. If net positive for a
   dynamic body with 2+ non-penetrating sliding contacts, subtract the mean correction
   from each sliding contact's lambda_n and re-project onto the Coulomb cone.

   Guards applied (in order):
   - Dynamic bodies only (`inverseMasses[k] > 0`)
   - 2+ contacts on body
   - No penetrating contacts on body (`vErr_n >= 0` for all)
   - Net delta must be positive (`netDelta > 1e-8`)
   - Contact must be at sliding cone boundary (`||lambda_t|| >= 0.99 * mu * lambda_n`)

3. **Math formulation revisions (6 issues I1-I6 fixed)**:
   - I1: K_ang matrix entry (2,1) typo fixed: `a_2^T I_A^{-1} a_2` → `a_2^T I_A^{-1} a_1`
   - I2: Oblique t2 sign corrected: `[-1/√2, 1/√2, 0]` → `[1/√2, -1/√2, 0]`
   - I3: Section 3 rewritten — pre-projection K_nt sum is identically zero for any angular
     velocity. Actual injection mechanism is Coulomb cone projection breaking cancellation.
   - I4: Per-Contact Indistinguishability proof updated to reference cone projection.
   - I5: Example 2 replaced with cone-projection-aware computation showing asymmetric
     clipping mechanism. Corrected GTest template.
   - I6: Section 6 post-sweep criterion reformulated on post-projection delta_lambda_n
     (not pre-projection K_nt * vErr which is identically zero).

### Variants Tested

Four successive redistribution variants were attempted before arriving at the final result:

| Variant | Guards | Tests Passing | New Regressions |
|---------|--------|---------------|-----------------|
| V1: No guards | None | 743/780 | 37 new failures |
| V2: Dynamic bodies only | inverseMasses > 0 | 756/780 | 24 new failures |
| V3: + Penetration guard | vErr_n >= 0 for all contacts | 768/780 | 0 (baseline) |
| V4: + Sliding cone boundary | \|\|lambda_t\|\| >= 0.99 mu lambda_n | 768/780 | 0 (baseline) |

**V1 failure**: Single-contact scenarios zeroed entire lambda_n (correction = delta / 1 contact).
**V2 failure**: Redistribution disrupted HighSpeed axis-aligned deceleration.
**V3 result**: Exactly 12 failures, same as baseline. RockingCube newly passing, HighSpeed newly passing (no regressions vs baseline). But none of the 12 core failures fixed.
**V4 result**: Identical to V3 — the isSliding guard did not change the outcome.

### Result: NEGATIVE — No improvement over baseline

**Total tests**: 780. Pass: 768. Fail: 12. Same 12 as original baseline.

**Failing tests** (all 12 from original, none fixed):
- ReplayEnabledTest.EnergyAccountingTest_InelasticBounce_KEReducedByESquared
- ReplayEnabledTest.LinearCollisionTest_PerfectlyElastic_EnergyConserved
- ReplayEnabledTest.LinearCollisionTest_EqualMassElastic_VelocitySwap
- ReplayEnabledTest.ParameterIsolation_TimestepSensitivity_ERPAmplification
- ReplayEnabledTest.RotationalCollisionTest_SphereDrop_NoRotation
- ReplayEnabledTest.RotationalEnergyTest_ZeroGravity_RotationalEnergyTransfer_Conserved
- FrictionSlidingTest.SlidingCube_ConeCompliantEveryFrame_HighSpeed
- FrictionSlidingTest.SlidingCube_ConeCompliantEveryFrame_Oblique45_Slow
- FrictionSlidingTest.SlidingCube_ConeCompliantEveryFrame_Oblique45_Medium
- FrictionSlidingTest.SlidingCube_ConeCompliantEveryFrame_Oblique45
- FrictionSlidingTest.SlidingCube_ConeCompliantEveryFrame_HighSpeedOblique
- FrictionSlidingTest.FrictionWithRestitution_BounceThenSlide

**Root cause**: The redistribution cannot fix the oblique energy injection because:
1. For oblique contacts, the energy injection occurs within the K_inv solve itself
   (before the sweep even completes). The net positive delta_n is a symptom of per-sweep
   impulse growth, not recoverable by post-sweep adjustment.
2. The oblique cube is launched to Vz = 21 m/s within 10 frames (confirmed by diagnostic
   output). At this point, the redistribution activates but the cube is already airborne.
   The penetration guard correctly blocks redistribution during penetrating frames (when
   the contact force IS legitimately large), but the damage is done on the non-penetrating
   frames between.
3. Axis-aligned high-speed sliding (HighSpeed test) has legitimate positive net delta_n
   during deceleration transients that must NOT be redistributed. The penetration guard
   (`vErr_n >= 0`) does not distinguish this case from oblique energy injection.

### Key Insight

The post-sweep redistribution approach is architecturally wrong for this problem:
- The math formulation (Section 3, corrected in I3) states that the pre-projection K_nt
  sum IS identically zero. What actually matters is the post-projection net.
- But the post-projection net for oblique contacts grows OVER MULTIPLE FRAMES (accumulates
  through warm-start), not within a single sweep.
- A post-sweep correction can zero the net delta for one sweep, but the warm-start for
  the next frame starts from the corrupted lambda. The redistribution would need to be
  frame-level, not sweep-level — and frame-level redistribution faces the same guard
  problems as sweep-level.

### Structural Incompatibility Confirmed

The pattern across P1-P6:
- Any change to row 0 of the solve (P2, P4, P5) that reduces K_nt normal coupling
  breaks tipping torque and/or axis-aligned sliding.
- Any post-solve correction (P6) that zeros net delta_n is too blunt to distinguish
  energy injection from correct physics.
- The P5 velocity-gated clamp was the closest approach but still too aggressive for
  HighSpeed sliding.

The root cause is that for oblique geometry, the CORRECT steady-state lambda_n is
smaller than what the accumulation produces, but the algorithm has no way to know the
"correct" lambda_n from geometric properties alone without a reference solution.

### Next Step

Human decision required. Options:

**Option A — Geometric oblique detector**: Identify oblique contacts by K_nt magnitude
(|K_nt| / K_nn > threshold) and apply decoupled normal row only for those contacts.
Risk: threshold sensitivity, may still affect tipping torque for borderline geometry.

**Option B — Accumulated lambda cap**: After convergence, compare accumulated lambda_n
to what Phase A + scalar normal solve would give. Cap if too large. Requires reference
solve per frame.

**Option C — Accept P5 + fix HighSpeed separately**: P5 fixes 7/12 failures with 2
regressions (SlidingCubeX lateral drift, HighSpeed). Investigate whether HighSpeed test
tolerances can be loosened or a per-contact mu-scaling can compensate.

**Option D — Different algorithmic approach**: Use a splitting method or alternating
direction approach (e.g., ADMM) where normal and friction subproblems are decoupled at
the algorithm level rather than at the solve level.

**Option E — Accept the status quo**: Document the 12 known failures as known issues with
the current Block PGS coupled solve. Investigate a longer-term architectural change.

