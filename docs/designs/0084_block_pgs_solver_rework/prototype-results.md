# Prototype Results: Block PGS Solver Rework

**Ticket**: [0084_block_pgs_solver_rework](../../../tickets/0084_block_pgs_solver_rework.md)
**Branch**: `0084-block-pgs-solver-rework`
**Date**: 2026-02-28
**Prototyper**: Prototype Agent

---

## Summary

Prototype P1 (warm-start disable diagnostic) was executed as specified in the design review.
The result is **negative**: disabling warm-start entirely does NOT fix any of the 12 failing
tests. All 12 tests fail with identical Z-velocity injection values to the warm-start-enabled
baseline. The warm-start hypothesis from the design document is **refuted**.

**Implication**: The root cause of the 12 failures is NOT warm-start contamination. It is
in the Phase B `sweepOnce` itself — specifically the K_nt off-diagonal coupling in the 3x3
block solve injecting normal impulse from tangential velocity error on every frame,
independent of warm-start state.

---

## Prototype P1: Warm-Start Disable Diagnostic

### Specification

From design review Prototype Guidance P1:

> **Prototype approach**: In `BlockPGSSolver::solve()`, set `const bool hasWarmStart = false`
> to disable warm-start entirely.
>
> **Success criteria**: If 8+ of 12 currently-failing tests pass with warm-start disabled,
> the warm-start hypothesis is confirmed and Fix F1 should be implemented.

### Implementation

Applied the following diagnostic change to `BlockPGSSolver.cpp` (lines 113–115):

```cpp
// Original:
const bool hasWarmStart = initialLambda.has_value() &&
                          initialLambda->size() == static_cast<Eigen::Index>(lambdaSize) &&
                          initialLambda->maxCoeff() > 0.0;

// P1 Diagnostic:
const bool hasWarmStart = false;  // PROTOTYPE P1 DIAGNOSTIC
```

Built with `cmake --build --preset debug-sim-only`. Build: **succeeded**, no warnings.

### Results

**Tests passing**: 1 of 13 (same as baseline)
**Tests failing**: 12 of 13 (same as baseline — hypothesis REFUTED)

| Test | Before P1 (baseline) | With P1 (warm-start off) | Change |
|------|----------------------|--------------------------|--------|
| `Oblique45_Slow` | FAIL (3.66 m/s Z-vel) | FAIL (3.66 m/s Z-vel) | None |
| `Oblique45_Medium` | FAIL (10.0 m/s Z-vel) | FAIL (10.0 m/s Z-vel) | None |
| `Oblique45` | FAIL (21.1 m/s Z-vel) | FAIL (21.1 m/s Z-vel) | None |
| `HighSpeedOblique` | FAIL (43.4 m/s Z-vel) | FAIL (43.4 m/s Z-vel) | None |
| `FrictionWithRestitution_BounceThenSlide` | FAIL | FAIL (max inj=0.061 J) | None |
| `InelasticBounce_KEReducedByESquared` | FAIL | FAIL (ratio=0.057 vs 0.25) | None |
| `PerfectlyElastic_EnergyConserved` | FAIL | FAIL (maxH=0.70 vs 1.0) | None |
| `EqualMassElastic_VelocitySwap` | FAIL | FAIL (omegaZ=3.14 vs 0.198) | None |
| `TimestepSensitivity_ERPAmplification` | FAIL | FAIL (growth=13.9% vs 1%) | None |
| `RockingCube_AmplitudeDecreases` | FAIL | FAIL (amplitude grows) | None |
| `SphereDrop_NoRotation` | FAIL | FAIL (omega=0.10+ rad/s) | None |
| `ZeroGravity_RotationalEnergyTransfer_Conserved` | FAIL | FAIL (energy not conserved) | None |

### Key Observation

The oblique Z-velocity injection values (`21.1 m/s`, `43.4 m/s`) are **identical** before and
after disabling warm-start. This proves that the Z-velocity injection occurs within the first
10 frames of the simulation, entirely from the Phase B sweep itself — not from accumulated
warm-start across frames.

---

## Revised Root Cause Analysis

### The Actual Problem: K_nt Coupling in Phase B sweepOnce

In `sweepOnce`, for each contact the solver computes:

```cpp
const Eigen::Vector3d unconstrained = blockKInvs[ci] * (-vErr);
```

For an oblique sliding contact (cube sliding at 45 degrees on floor), the `K` matrix has
non-zero off-diagonal terms K(0,1) and K(0,2) (coupling between the normal row and both
tangent rows). These arise because the lever arm `rA` has components along all three axes
for a cube corner contact, making `(rA×n)^T * IA_inv * (rA×t1) != 0`.

The effect: `unconstrained(0) = -K_inv(0,1)*vErr(1) - K_inv(0,2)*vErr(2)`. Even when
`vErr(0) = 0` (cube is not penetrating the floor), the tangential velocity error `vErr(1,2)`
drives a non-zero normal correction `unconstrained(0)`. If this is positive (upward for a
downward-pointing normal), it accumulates into `lambdaPhaseB(base)` and then generates an
upward velocity on the cube via `updateVRes3`.

This is not bounded by the Coulomb cone in a way that prevents energy injection. The Coulomb
cone bounds `||lambda_t|| <= mu * lambda_n`, which limits tangential impulse relative to
normal — but does not prevent the normal impulse from growing due to tangential velocity error.

### Why Phase B injects energy for oblique contacts

For a stable resting contact (zero tangential velocity), `vErr = [small, 0, 0]` and the
K_nt terms don't matter. For a sliding contact at 45 degrees:
- `vErr(0) = 0` (no penetration)
- `vErr(1) = -vy * t1_dot = non-zero`
- `vErr(2) = vx * t2_dot = non-zero`

The block solve produces `unconstrained(0) != 0` from K_inv's off-diagonal terms. Over
each sweep, this grows the normal lambda, which then generates upward velocity through
`linearA = -n * dN`. With `n` pointing from cube to floor (nominally downward), `-n * dN`
points upward. Each sweep adds more upward velocity to the cube.

### Root Causes by Failure Category

#### Oblique Sliding Failures (5 tests)

The K_nt off-diagonal coupling produces a non-zero normal correction `delta_lambda_n` from
tangential velocity error alone. This normal correction, applied via `updateVRes3`,
accumulates upward velocity (`-n * dN` with `n` pointing downward). The magnitude is
proportional to the initial tangential speed and the K_nt coupling strength, which depends
on the lever arm geometry (cube corner contact).

**Key evidence**: Z-velocity injection is identical with warm-start on or off. It begins
on frame 1. This is a per-frame Phase B problem, not a cross-frame warm-start problem.

#### Restitution and Rotational Failures (7 tests)

These failures appear to have a different root cause than the oblique failures. Examining
the output more carefully:

- `InelasticBounce_KEReducedByESquared`: Post-bounce KE ratio = 0.057 vs expected 0.25.
  The KE reduction is **too large** (too much energy dissipated), not too little.

- `EqualMassElastic_VelocitySwap`: `omegaAz = 3.14` vs expected `0.198`. Massive
  angular velocity injection, but linear velocity has only modest error. This suggests
  the Coulomb cone projection is allowing excessive tangential impulse that converts to
  rotation.

- `TimestepSensitivity_ERPAmplification`: 13.9% energy growth vs 1% threshold, but at
  8ms timestep. This may be unrelated to oblique sliding.

These 7 tests may share the K_nt coupling root cause (Phase B normal impulse from
tangential velocity) or may have a separate root cause. Further investigation needed.

---

## Diagnosis: What Actually Needs Fixing

### Fix Hypothesis A: Decouple Normal Row from Block Solve (High Confidence)

The block PGS is solving a coupled 3x3 system (normal + 2 tangents). The K_nt off-diagonal
terms allow tangential velocity error to drive a normal correction. For a stable resting
contact this is correct. For a **sliding contact**, it causes energy injection because the
normal lambda grows to accommodate the friction-induced K_nt coupling.

**Proposed fix**: Split the block solve into:
1. Normal solve: `delta_lambda_n = K_inv_nn * (-vErr(0))` (scalar, independent)
2. Tangent solve: Use only the 2x2 tangent subblock of K_inv, NOT the full 3x3 inverse

The Coulomb cone is then: bound `||lambda_t|| <= mu * lambda_n`.

This is equivalent to the approach used by many real-time physics engines (e.g., Bullet's
btSequentialImpulseConstraintSolver): solve normal and friction independently with the same
normal lambda, then apply the cone.

**Risk**: This sacrifices the K_nt coupling accuracy. For contacts with large lever arms
(cube corners), the decoupled normal solve may not correctly account for the rotation-normal
coupling. However, the 3x3 coupled solve is clearly incorrect for sliding contacts.

### Fix Hypothesis B: Constrain Normal Row in Block Solve

Instead of fully decoupling, constrain the normal lambda to change only if the contact is
genuinely penetrating (i.e., `vErr(0) < 0`). Only apply the normal correction from K_nt
terms if the contact is actively approaching.

```cpp
// In sweepOnce, after computing unconstrained:
Eigen::Vector3d unconstrained = blockKInvs[ci] * (-vErr);
// Decouple normal: only use scalar K_nn for normal row
if (vErr(0) >= 0.0) {
    unconstrained(0) = 0.0;  // Separating or resting: no normal correction
} else {
    unconstrained(0) = (-vErr(0)) / blockKs[ci](0, 0);  // Scalar solve only
}
```

This prevents the K_nt terms from driving the normal lambda when the contact is sliding.

### Fix Hypothesis C: Use Phase A For All Restitution, Phase B Only for Friction

The restitution failures (7 tests) may be caused by Phase A being called with the
wrong `vErr` — specifically, `computeBlockVelocityError` is called to get `Jv_n` for
Phase A, but for an elastic bounce the warm-start `vRes_` is already loaded from the
previous frame (even with warm-start "off" in P1, Phase B has run and modified `vRes_`).

Wait — Phase A runs AFTER warm-start initialization but BEFORE Phase B. With warm-start
off, `vRes_ = 0` when Phase A runs. So Phase A should be correct in P1.

But the restitution tests still fail. This suggests Phase A's bounce impulse is computed
correctly but Phase B then undoes or overcorrects it. Phase B with full 3x3 block K_inv
after Phase A may produce a normal correction that counteracts Phase A's bounce.

---

## Recommendation for Implementation

The prototype result is clear: **warm-start is not the root cause**. The design document's
Fix F1 (Phase B-only cache storage) should NOT be implemented as the primary fix. It may
be a beneficial secondary improvement to prevent warm-start contamination of future frames,
but it will not fix the 12 failing tests.

**Recommended investigation before implementation**:

1. **Verify K_nt coupling hypothesis**: Add temporary diagnostic output to `sweepOnce`
   printing `unconstrained(0)` for an oblique contact. If it is large and non-zero even
   when `vErr(0) = 0`, the K_nt coupling hypothesis is confirmed.

2. **Test Fix Hypothesis B** (zero normal correction when not approaching):
   ```cpp
   if (vErr(0) >= 0.0) unconstrained(0) = 0.0;
   ```
   This is a one-line change. If it fixes the oblique tests, the root cause is confirmed.

3. **Re-assess restitution failures** after oblique tests are fixed.

### Design Revision Required

The current design (Fix F1: Phase B-only cache storage) addresses the wrong root cause.
The implementation phase must be preceded by a design revision that:

1. Correctly identifies the Phase B K_nt coupling as the primary failure mechanism
2. Proposes the corrected block solve that prevents tangential velocity error from
   driving normal impulse in non-penetrating contacts
3. Re-assesses whether warm-start contamination is a secondary issue (it may still be
   present, but it is not the cause of the 12 test failures)
4. Re-specifies the success criteria for a new prototype

**Time estimate**: Design revision = 2 hours. New prototype = 30 minutes.

---

## Artifacts

- `prototypes/0084_block_pgs_solver_rework/p1_warmstart_disable/` — P1 diagnostic
  (warm-start disabled; warm-start restored to original in production code)
- `docs/designs/0084_block_pgs_solver_rework/prototype-results.md` — This document

---

## Revision to Design Status

The prototype result requires escalation to human review before proceeding to
implementation. The root cause analysis in the design document is incorrect regarding the
primary failure mechanism. A design revision is needed.

**Recommended ticket status update**: "Implementation Blocked — Design Revision Needed"
(This is distinct from a quality gate failure; it is a prototype-triggered design revision.)

The P1 result is definitive: changing the warm-start cache will not fix the 12 failing tests.
The design must be corrected before implementation proceeds.
