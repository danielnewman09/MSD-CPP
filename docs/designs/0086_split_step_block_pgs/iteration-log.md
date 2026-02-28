# Iteration Log — 0086_split_step_block_pgs

> **Purpose**: Track every build-test cycle during implementation or investigation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/designs/0086_split_step_block_pgs/iteration-log.md`
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0086_split_step_block_pgs
**Branch**: 0084-block-pgs-solver-rework
**Baseline**: 772 pass / 789 total (17 failing) at start

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 1 — 2026-02-28 14:15
**Commit**: 274f1bd
**Hypothesis**: Replace P4 asymmetric decoupled 3x3 block solve with split-step two-pass algorithm (Pass 1: scalar K_nn normal solve; Pass 2: 2x2 K_tt tangent solve). This should eliminate K_nt energy injection while preserving coupling via sequential vRes_ updates.
**Changes**:
- `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.hpp`: Added `updateVResTangentOnly` declaration; changed `sweepOnce` parameter from `vector<Matrix3d> blockKInvs` to `vector<Matrix2d> tangentKInvs`; updated Doxygen for `sweepOnce` with split-step description
- `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.cpp`: Updated ticket/design header comment; replaced 3x3 LDLT pre-computation with 2x2 K_tt LDLT; added `updateVResTangentOnly` delegate; rewrote `sweepOnce` as two-pass split-step
**Build Result**: PASS (no warnings)
**Test Result**: 772/789 — Same 17 failures as baseline. Split-step IS changing behavior (VelocitySwap KE goes from 1.522J to 1.999J — correct energy conservation) but not fixing all acceptance-criteria tests.
**Impact vs Previous**: 0 net change in pass/fail count. VelocitySwap, ZeroGravityRotational show different pinned values (split-step is working correctly). FrictionProducesTippingTorque, TiltedDropTest show same failure patterns.

**Assessment**: The split-step implementation is correct — it produces measurably different physics for sphere-sphere collision tests. However, the acceptance-criteria tests still fail. Two categories:

**Category A — Pinned-value tests that are now BETTER (need value updates):**
- `VelocitySwap`: KE went from 1.522J to 1.999J (correct! e=1 should conserve energy). Values need re-pinning.
- `ZeroGravity`: finalKE = 1.923J vs pinned 1.748J. Better energy conservation.
- `PerfectlyElastic`: Different bounce heights — maxHeight now different from pinned.

**Category B — Tests that still fail with IDENTICAL failure signatures:**
- `FrictionProducesTippingTorque`: omegaZ=0.088, omegaY=6.78e-7 — same as before.
- `TiltedDropTest`: Cross-axis ratio ~18.8% — same as before split-step.
- `SphereDrop_NoRotation`: maxOmega = 16.9 rad/s — enormous for vertical drop.
- `SlidingCubeX/Y`: 0.026m lateral drift — same as before.
- `InelasticBounce`: ratio = 0.057, below 0.125 threshold.

**Root cause analysis for Category B:**

1. For axis-aligned contacts (flat floor, contact normal = Z), the K matrix has K_nt components from the lever arm cross products. However, the KEY mathematical observation: `(K^{-1})[1:3, 1:3] == (K[1:3, 1:3])^{-1}` only when K_nt=0. For our contacts K_nt IS non-zero. But the difference between the old code's K_inv.block<2,3>(1,0)*(-vErr) and the new code's K_tt_inv*(-vErr_t) is PRIMARILY from the K_nt terms — and K_nt in vErr(0) term only matters when vErr(0) != 0.

2. In STEADY-STATE SLIDING (FrictionProducesTippingTorque test), the contacts are non-penetrating (vErr(0) ≈ 0) within each sweep iteration. So the K_nt contribution `K_inv(1:2,0)*(-vErr(0))` in the old code is near zero anyway. The split-step and old code produce essentially the same tangent impulse in this regime.

3. For the TIPPING TORQUE test: The failure is not from K_nt. The cube has 4 corners in contact. With sliding mode aligned to X, t1=-X, t2=+Y. The friction impulse from t1 creates Y-axis torque (correct). But the test shows omegaZ dominates omegaY. This suggests each corner contact has a different tangential velocity at the contact point (due to cube rotation), and the asymmetric t2 impulses across 4 corners produce net Z-torque.

4. For `SphereDrop_NoRotation`: The icosphere is polyhedral — contact normals are slightly off-axis. Even a "vertical" drop produces off-axis normal impulse → angular velocity from cross-product with lever arm. This is a geometry issue, not a K_nt issue.

**Go/No-Go assessment**: Per the design document, `FrictionProducesTippingTorque` is the go/no-go gate. The test still fails with the same pattern. The split-step hypothesis is partially invalidated: it correctly improves energy conservation for collision tests, but cannot fix the tipping torque or cross-axis rotation tests because their root cause is multi-point contact geometry asymmetry, not K_nt coupling.

**ESCALATION**: Cannot meet acceptance criteria with split-step alone. Need human decision on path forward:
1. Keep split-step (it improves energy conservation), update Category A pinned values, and accept that the remaining 12 tests require separate investigation.
2. Revert to P4 (no improvement) and investigate contact manifold/tangent basis issues separately.
3. Investigate why 4-corner floor contacts produce wrong tipping torque direction — this may be a tangent basis orientation bug in the contact manifold, not a solver bug.
