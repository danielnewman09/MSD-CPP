# Design: Block PGS Solver Rework

**Ticket**: [0084_block_pgs_solver_rework](../../../tickets/0084_block_pgs_solver_rework.md)
**Branch**: `0084-block-pgs-solver-rework`
**GitHub Issue**: #112
**PR**: #113

## Summary

The Block PGS solver (0075b) has 12 failing tests across two categories: oblique sliding
produces massive Z-velocity injection (up to 43 m/s from an initial speed of 6 m/s), and
replay-enabled collision tests fail for elastic/inelastic restitution, rotational dynamics, and
ERP amplification. This document diagnoses each failure category to its root cause in
`BlockPGSSolver.cpp`, proposes precise algorithmic fixes, specifies implementation order, and
identifies prototype validation targets. No architectural changes are required — the two-phase
structure (Phase A: restitution, Phase B: dissipative block PGS) is sound. The bugs are in
the implementation details of warm-starting, velocity error computation, and the
`computeTangentBasis` convention relative to the oblique-contact tangent impulse projection.

---

## Architecture Changes

### PlantUML Diagram

See: [`./0084_block_pgs_solver_rework.puml`](./0084_block_pgs_solver_rework.puml)

The architecture is unchanged. The diagram documents the corrected data flows and bug
locations within `BlockPGSSolver`.

### New Components

None. This ticket modifies one existing class (`BlockPGSSolver`).

### Modified Components

#### BlockPGSSolver

- **Current location**: `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.cpp`
- **Header location**: `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.hpp`
- **Changes required**: See Root Cause Analysis and Fix Specifications below
- **Backward compatibility**: All frictionless (ASM path) tests are unaffected. The frictionless
  path in `ConstraintSolver` does not invoke `BlockPGSSolver`.

---

## Root Cause Analysis

### Category 1: Oblique Sliding Z-Velocity Injection (5 tests)

**Affected tests**: `Oblique45_Slow`, `Oblique45_Medium`, `Oblique45`, `HighSpeedOblique`,
`FrictionWithRestitution_BounceThenSlide` (partially).

**Symptom**: A cube sliding at 45 degrees in the XY plane gains large Z-velocity (normal
direction), ranging from 3.66 to 43.4 m/s depending on initial speed. The Z-velocity should
remain near zero (floor contact normal is Z-up).

**Root cause: warm-start energy injection from Phase A bounce impulses**.

The cache stores the **total assembled lambda** from `BlockPGSSolver::solve()`:
```cpp
result.lambdas(base) = lambdaPhaseB(base) + bounceLambdas_[ci];  // Phase A + Phase B
result.lambdas(base + 1) = lambdaPhaseB(base + 1);               // Phase B only
result.lambdas(base + 2) = lambdaPhaseB(base + 2);               // Phase B only
```

On the **next frame**, this total is fed back as `initialLambda` (warm-start). The warm-start
path:
```cpp
if (hasWarmStart)
{
  lambda = *initialLambda;          // Seeds lambdaPhaseB from previous TOTAL
  for (size_t ci = 0; ci < numContacts; ++ci)
    updateVRes3(*contacts[ci], warmBlock, ...);  // vRes_ += M^{-1} J^T lambda_warm_TOTAL
}
// ...
lambdaPhaseB = lambda;              // Phase B accumulator = previous TOTAL
```

For a resting contact (e=0, Phase A bounce = 0), this is correct: `lambda_warm = lambdaPhaseB_prev`,
and Phase B sees the correct residual.

For a **sliding contact with restitution** (oblique45 tests use e=0 in the sliding phase, but
the `FrictionWithRestitution` test uses e=0.6): after the first bounce, `bounceLambdas_[ci] > 0`
is included in the cached total. The next frame's warm-start seeds `lambdaPhaseB` with a normal
impulse that includes the bounce component. Phase B then sweeps against this inflated warm-start,
and the Coulomb cone projection — which bounds `||lambda_t|| <= mu * lambda_n` — allows a larger
tangential impulse because `lambda_n` is incorrectly inflated by the prior bounce term.

**But wait: oblique45 tests use e=0**. So why do they fail?

For **oblique contacts at 45 degrees**, the tangent basis (`t1`, `t2`) computed by
`tangent_basis::computeTangentBasis(n)` where `n = [0, 0, 1]` (Z-up floor normal) produces:
- Smallest `|ni|`: `|nx|` = 0, so `ex = [1,0,0]` is selected
- `t1 = (ex × n) / ||ex × n|| = ([1,0,0] × [0,0,1]) / 1 = [0,-1,0] × ... = [0,-0,1] ...`

Let me verify: `[1,0,0] × [0,0,1] = [0*1 - 0*0, 0*0 - 1*1, 1*0 - 0*0] = [0,-1,0]`.
So `t1 = [0,-1,0]` (Y direction, negated) and `t2 = n × t1 = [0,0,1] × [0,-1,0] = [0*0-1*(-1), 1*0-0*0, 0*(-1)-0*0] = [1,0,0]`.

For a cube sliding in the XY plane at 45 degrees, the contact point velocity has components
along both X and Y. The velocity error in the tangent frame:
- `vErr(1) = t1 · (vContactA - vContactB) = [0,-1,0] · vSliding`
- `vErr(2) = t2 · (vContactA - vContactB) = [1,0,0] · vSliding`

The block solve computes: `delta_lambda = K_inv * (-vErr)`. The projection clamps to the
Coulomb cone. The `updateVRes3` then applies:
```cpp
const Eigen::Vector3d linearA = -n * dN + t1 * dT1 + t2 * dT2;
```

With `n=[0,0,1]`, `t1=[0,-1,0]`, `t2=[1,0,0]`, `dN` (normal correction from Phase B) should
be zero (floor does not sink). If `dN` drifts non-zero due to K matrix coupling, the linear
contribution becomes `[0,0,-dN]` — a Z-velocity on body A.

The **actual root cause** for e=0 oblique tests: in the warm-start initialization block,
`updateVRes3` is called with the **full 3-vector** `warmBlock = lambda.segment<3>(base)`:
```cpp
updateVRes3(*contacts[ci], warmBlock, inverseMasses, inverseInertias);
```

This applies all three lambda components (n, t1, t2) to `vRes_`. For a stable resting contact,
`lambda_n_warm > 0` and `lambda_t_warm` encodes the friction impulse from last frame. This is
intentional — the warm-start is supposed to pre-load the velocity state.

However, the issue is that for a **continuously sliding contact**, `lambdaPhaseB` from Phase B
contains a normal component that accounts for the **pressure** needed to prevent penetration AND
the cross-coupling reaction to tangential motion (the K_nt off-diagonal terms). This normal
component, when warm-started, seeds `vRes_[idxA][2] -= wA * n * lambda_n_warm` (upward velocity
on body A). The Phase B sweep then computes a `vErr` against this pre-loaded vRes_ state. For
an oblique contact where the K matrix has strong K_nt coupling (because the lever arm produces
off-diagonal rotation-translation coupling), the unconstrained correction `delta_lambda = K_inv * (-vErr)`
has a non-zero normal component `dN`. This `dN`, when accumulated and applied via `updateVRes3`,
produces the Z-velocity injection.

**The fundamental issue**: the warm-start `lambda_n` for a resting floor contact is `m*g*dt`
(balances gravity). This is correct. But when a cube slides obliquely, the K matrix's off-diagonal
terms (K_nt: coupling between normal and tangential rows) mean that the Phase B correction to the
normal component is affected by the tangential state. After warm-start loads `vRes_` from
`lambda_n_warm`, `Phase B` computes `vErr(0)` — and if this is non-zero (because the tangential
velocity from the warm-start shifts things slightly), Phase B applies a correction `dN` to the
normal lambda. This `dN`, multiplied by `n = [0,0,1]` in `updateVRes3`, injects Z-velocity
directly.

**The fix**: Separate the warm-start into normal-only and tangential components for initialization:

Instead of calling `updateVRes3` with the full 3-vector (which applies all three impulse
directions including normal), the warm-start should use **only the Phase B dissipative component**
(subtracting out the Phase A bounce, which should not be included in the warm-start). Furthermore,
for the oblique sliding case, the warm-start normal component can be initialized correctly by
initializing from the **previous Phase B normal lambda only** (not total = Phase A + Phase B).

**Fix specification for Category 1**:

1. **Cache only Phase B lambdas** (not the Phase A + Phase B total). In `BlockPGSSolver::solve()`,
   the returned `result.lambdas` currently includes `bounceLambdas_[ci]` added to the normal
   component. `CollisionPipeline` stores this total in the cache. The fix is to store Phase B
   lambdas separately so warm-start does not include Phase A's bounce contribution.

   Since `CollisionPipeline` is the cache owner and `BlockPGSSolver` assembles the total, the
   cleanest fix is to **expose Phase B lambdas** from the solver for caching, while still
   returning the total in `result.lambdas` for force application.

   **Proposed change to `SolveResult`**:
   ```cpp
   struct SolveResult {
     std::vector<BodyForces> bodyForces;
     Eigen::VectorXd lambdas;           // TOTAL (Phase A + Phase B) — for force application
     Eigen::VectorXd phaseBLambdas;     // Phase B only — for warm-starting
     bool converged{};
     int iterations{};
     double residual{};
   };
   ```
   `CollisionPipeline` uses `result.phaseBLambdas` for cache storage instead of `result.lambdas`.

2. **Guard the warm-start condition more carefully**. The current check is:
   ```cpp
   const bool hasWarmStart = initialLambda.has_value() &&
                             initialLambda->size() == static_cast<Eigen::Index>(lambdaSize) &&
                             initialLambda->maxCoeff() > 0.0;
   ```
   This uses `maxCoeff() > 0.0` which will fail if all lambdas are zero (new contact). But it
   also accepts any positive value — including stale warm-starts from a bounce frame. With the
   Phase B-only cache fix, this check becomes correct: a bouncing contact's Phase A lambda is
   no longer in the warm-start.

### Category 2: Restitution Test Failures (3 tests)

**Affected tests**: `PerfectlyElastic_EnergyConserved`, `EqualMassElastic_VelocitySwap`,
`InelasticBounce_KEReducedByESquared`.

**Symptom**: Elastic collisions fail to conserve energy, and inelastic bounces do not reduce KE
by the correct e² factor.

**Root cause: warm-start contaminates Phase A velocity baseline**.

For an elastic collision (e=1, friction=0.5 → BlockPGS path):

Frame N (collision):
- No warm-start (first contact): `vRes_` = 0
- Phase A: `Jv_n = n·(vB - vA)` (approaching, so Jv_n < 0)
- Phase A bounce: `lambda_bounce = (1+e)*(-Jv_n)/K_nn` — **correct**
- Phase B (dissipative): refines the solution
- Total lambda cached: `lambda_n_total = lambdaPhaseB_n + lambda_bounce`

Frame N+1 (post-bounce, bodies now separating):
- Warm-start from cache: `lambda_warm` includes `lambda_bounce` from frame N
- `vRes_` initialized with `updateVRes3` using `lambda_warm` (includes bounce component)
- Phase A runs: `computeBlockVelocityError` reads `v_pre + vRes_`
- `vRes_` already contains the bounce impulse from last frame's `lambda_bounce`
- So `Jv_n` (the Phase A velocity check) = normal velocity including warm-start bounce
- For separating bodies: `Jv_n > 0` (correct, Phase A skips) — but the **normal magnitude**
  computed by Phase B is wrong because it starts from a warm-start that contains last frame's
  bounce term

For a **multi-bounce scenario** (PerfectlyElastic test bounces repeatedly):
- Frame 2: warm-start = Phase A + Phase B from frame 1
- Phase A in frame 2 uses `computeBlockVelocityError` which includes the warm-start vRes_
- The warm-start vRes_ contains a large upward velocity component from `lambda_bounce_frame1`
- Phase A sees the combined state and computes a **wrong** `Jv_n`, potentially missing the
  bounce or computing an excessive one
- Over multiple bounces, energy is not conserved

**The fix (same as Category 1)**: Store only Phase B lambdas in the cache. Phase A bounce
impulses are inherently non-persistent — they are computed fresh each frame from the current
velocity. Including them in the warm-start causes incorrect velocity baseline for Phase A on the
next frame.

### Category 3: TimestepSensitivity / ERP Amplification (1 test)

**Affected test**: `ParameterIsolation_TimestepSensitivity_ERPAmplification`.

**Symptom**: A resting cube (e=0, mu=0.5, mass=10kg) shows energy growth > 1% of initial PE
at larger timesteps (32ms). The ASM path (frictionless) passes; the BlockPGS path (with
friction=0.5) fails.

**Root cause: warm-start overcorrects the velocity state at large timesteps**.

For a resting contact with gravity:
- Pre-impact velocity: `v_pre[z] = -g*dt` (downward from gravity)
- Correct warm-start: `lambda_n_warm = m*g*dt`, which applies `vRes_[z] += wA * n * lambda_n`
  with `n = [0,0,1]` (upward), giving `vRes_[z] = g*dt`. Total velocity = 0. Correct.

With the Phase B-only cache (fix from Categories 1/2), the normal warm-start is correct. But
there is a secondary issue: the Baumgarte position correction (alpha=0.2, beta=0.0 in
`ContactConstraint`) adds a position error term to the normal constraint. For larger timesteps,
this position correction term is larger, but the Block PGS (being a velocity-level solver) does
not account for this correctly.

**Specifically**: the `PositionCorrector` in `CollisionPipeline` applies a split-impulse
correction *after* the velocity solve. The split-impulse generates a corrective velocity that
moves bodies out of penetration without affecting the main velocity. However, with warm-starting
from a frame with a large Phase B correction (which itself was inflated by the Baumgarte term),
the warm-start carries forward an overcorrection.

The warm-start fix (Category 1) should substantially mitigate this. The remaining ERP sensitivity
is expected to be within the 1% threshold once Phase A bounce contamination is removed from the
cache.

**Secondary fix**: The Baumgarte position correction is encoded in the normal constraint RHS via
the `evaluate()` return value multiplied by `alpha`. However, `BlockPGSSolver` does NOT use
`evaluate()` or Baumgarte terms — it uses `computeBlockVelocityError` which is purely
velocity-level. This is correct for a velocity-level solver. The `PositionCorrector` handles
position correction separately. **No change needed here** — the ERP sensitivity should resolve
with the warm-start fix.

### Category 4: Rotational Failures (3 tests)

**Affected tests**: `RockingCube_AmplitudeDecreases`, `SphereDrop_NoRotation`,
`ZeroGravity_RotationalEnergyTransfer_Conserved`.

#### 4a: SphereDrop_NoRotation

**Symptom**: A sphere dropped vertically with friction (mu=0.5) generates spurious angular
velocity (>0.5 rad/s). The sphere has no tangential velocity pre-impact, so no tangential
impulse should be generated.

**Root cause: warm-start tangential contamination**.

For a sphere dropped vertically (velocity = [0,0,-v]):
- Contact normal: n = [0,0,1] (floor)
- Tangent basis: `t1 = [0,-1,0]`, `t2 = [1,0,0]` (from `computeTangentBasis([0,0,1])`)
- No tangential velocity: `vErr(1) = 0`, `vErr(2) = 0`
- Correct Phase B: `delta_lambda_t1 = 0`, `delta_lambda_t2 = 0`

**On first collision frame**: No warm-start, vRes_=0. Phase B correctly computes zero tangential
impulse. **However**, for a multi-contact sphere (multiple contact points at different lever arms),
the off-diagonal K matrix entries (K_nt coupling) may produce a small non-zero `dN` from the
block solve even with zero tangential velocity, IF the normal component is not handled correctly.

More precisely: when the sphere has 4 contact points (the typical EPA manifold), each contact has
a different lever arm `rA`. The `buildBlockK` computes `K` for each contact independently. The
Phase B sweep applies `delta_lambda_i` independently per contact. If one contact's block solve
produces a non-zero `delta_t1` (due to numerical noise or K_nt coupling from the lever arm
rotation term), and `updateVRes3` applies this, the next contact's `vErr` is shifted. For a
symmetric sphere, these should cancel. But with the warm-start bug: on the second bounce frame,
`vRes_` is seeded from `lambda_warm` which includes Phase A bounce (a large upward impulse).
This large normal warm-start shifts `vErr(0)` significantly, which shifts `vErr(1)` through K
coupling, which produces non-zero tangential corrections.

**Fix**: The warm-start fix (Category 1) should resolve this. With Phase B-only warm-start, the
normal warm-start for a resting sphere is much smaller (gravity balance only), and the K_nt
coupling effects are proportional to the warm-start magnitude.

#### 4b: RockingCube_AmplitudeDecreases

**Symptom**: A cube tilted 15 degrees rocks on its edge; amplitude should decrease (friction
dissipates rotational energy), but instead it grows or stays constant.

**Root cause: Phase B warm-start includes Phase A bounce**.

The rocking cube generates intermittent contact (contact lost and re-established each half-cycle).
Each contact frame: Phase A computes a bounce impulse (e=0.3 for the test). This bounce is
stored in the cache as part of the total lambda. The next contact frame, the warm-start includes
the previous bounce, which:
1. Seeds `vRes_` with an incorrect upward impulse
2. Phase A sees a different (incorrect) approaching velocity, computing wrong bounce magnitude
3. Phase B then sweeps from this incorrect state

Over multiple rocking cycles, energy is injected instead of dissipated.

**Fix**: Same warm-start fix.

#### 4c: ZeroGravity_RotationalEnergyTransfer_Conserved

**Symptom**: Zero-gravity, e=1.0 cube dropped at 45 degrees. Kinetic energy should be conserved
(linear → rotational conversion). Energy is not conserved.

**Root cause: Phase A bounce magnitude error due to warm-start contamination**.

For e=1 with no warm-start (first collision): Phase A computes `lambda_bounce = 2 * (-Jv_n) / K_nn`.
This should be correct. The body bounces, and the Phase B dissipative sweep should make no
correction (since Phase A already zeroed the normal velocity, Phase B finds vErr=0).

But on the **next bounce** (after the cube bounces off the floor and returns due to zero gravity):
the warm-start includes the previous total lambda. Phase A `computeBlockVelocityError` uses
`vRes_` initialized from this warm-start. The warm-start includes the Phase A bounce impulse from
the first collision. This makes Phase A compute an incorrect `Jv_n`, leading to a wrong bounce
impulse magnitude.

**Fix**: Same warm-start fix.

### Category 5: FrictionWithRestitution_BounceThenSlide

**Symptom**: An object should bounce (high restitution) and then slide to rest (friction). The
transition is broken — either too much energy is injected during the slide phase or the bounce
is incorrectly resolved.

**Root cause**: Combination of Category 1 (warm-start contamination of Phase A) and Category 2
(bounce in cache inflating Phase B warm-start).

**Fix**: Same warm-start fix resolves the core issue.

---

## Fix Specifications

### Fix F1: Phase B-Only Cache Storage (Primary Fix)

**Files modified**: `BlockPGSSolver.hpp`, `BlockPGSSolver.cpp`, `CollisionPipeline.cpp`

**Change to `SolveResult`** in `BlockPGSSolver.hpp`:
```cpp
struct SolveResult {
  std::vector<BodyForces> bodyForces;
  Eigen::VectorXd lambdas;        // Total (Phase A + Phase B) — for force extraction
  Eigen::VectorXd phaseBLambdas;  // Phase B only — for warm-start caching
  bool converged{false};
  int iterations{0};
  double residual{std::numeric_limits<double>::quiet_NaN()};
};
```

**Change to `BlockPGSSolver::solve()`** — populate `phaseBLambdas`:
```cpp
// After Phase B completes, before assembling total lambdas:
result.phaseBLambdas = lambdaPhaseB;  // Phase B accumulator only

// Assemble total for force extraction (unchanged):
for (size_t ci = 0; ci < numContacts; ++ci)
{
  const auto base = static_cast<Eigen::Index>(ci * 3);
  result.lambdas(base)     = lambdaPhaseB(base) + bounceLambdas_[ci];
  result.lambdas(base + 1) = lambdaPhaseB(base + 1);
  result.lambdas(base + 2) = lambdaPhaseB(base + 2);
}
```

**Change to `CollisionPipeline`** cache update (lines ~656–690): use `blockResult.phaseBLambdas`
instead of `result.lambdas` when writing to `ContactCache`. The force application path uses
`result.lambdas` (unchanged). The warm-start path uses `phaseBLambdas`.

This requires the `ConstraintSolver` to propagate `phaseBLambdas` from `BlockPGSSolver`'s
result back to `CollisionPipeline`. The simplest propagation: add `phaseBLambdas` to
`ConstraintSolver::SolveResult` and populate it when the BlockPGS path is taken.

**ConstraintSolver::SolveResult change**:
```cpp
struct SolveResult {
  std::vector<BodyForces> bodyForces;
  Eigen::VectorXd lambdas;
  Eigen::VectorXd warmStartLambdas;  // For cache storage (Phase B only if BlockPGS)
  bool converged{};
  int iterations{};
  double residual{};
};
```

For the ASM and PGS paths, `warmStartLambdas = lambdas` (no split needed — those paths have no
Phase A equivalent). For the BlockPGS path, `warmStartLambdas = blockResult.phaseBLambdas`.

**CollisionPipeline cache write**: change to use `solveResult.warmStartLambdas` for cache.

### Fix F2: Oblique Contact Tangent Basis Alignment (Secondary Fix for Category 1)

**Analysis**: After applying Fix F1, does Z-velocity injection persist?

For a floor normal `n = [0,0,1]`, `computeTangentBasis` produces `t1 = [0,-1,0]`, `t2 = [1,0,0]`.
For a cube sliding at 45 degrees in the XY plane: `vSliding = [v/√2, v/√2, 0]`.

The velocity error in the tangent frame:
- `vErr(1) = t1 · (vContactA - vContactB) = [0,-1,0] · vSliding = -v/√2`
- `vErr(2) = t2 · (vContactA - vContactB) = [1,0,0] · vSliding = v/√2`

These are non-zero, so Phase B computes a tangential correction. The block solve:
`delta_lambda = K_inv * (-vErr) = K_inv * [0, v/√2, -v/√2]^T`

For a symmetric, diagonal K matrix (no lever arm off-diagonal), this gives:
`delta_lambda_n = 0`, `delta_lambda_t1 = v/(√2 * K_t1t1)`, `delta_lambda_t2 = -v/(√2 * K_t2t2)`.

After Coulomb cone projection, these tangential lambdas are bounded by `mu * lambda_n`. The
normal component `delta_lambda_n = 0` because K is diagonal in this ideal case.

**However**, for a real contact with lever arm (cube corner), K has off-diagonal K_nt terms:
K_nt(0,1) = `wA * (-n) · (t1) + (rA×(-n)) · IA_inv · (rA×t1) + wB * (n) · (-t1) + ...`

Let us expand: `K(0,1) = J_A_row0 · M_A^{-1} · J_A_col1^T + J_B_row0 · M_B^{-1} · J_B_col1^T`
where `J_A_row0 = [-n^T, -(rA×n)^T]` and `J_A_col1 = [t1^T, (rA×t1)^T]^T`.
`K(0,1) = wA * (-n · t1) + (-(rA×n))^T · IA_inv · (rA×t1) + wB * (n · (-t1)) + (rB×n)^T · IB_inv · (-(rB×t1))`
`= wA * 0 + (-1)*(rA×n)^T · IA_inv · (rA×t1) + 0 + (-1)*(rB×n)^T · IB_inv · (rB×t1)`

Since `n ⊥ t1`, the linear terms vanish. The angular terms are non-zero when the lever arms
have components in both the normal and tangential directions. For a cube corner contact, `rA`
has components along all three axes, so K(0,1) ≠ 0.

This means the block solve produces a **non-zero** `delta_lambda_n` from a tangential velocity
error. This `dN` is physically correct — it represents the normal impulse needed to prevent
penetration given the constraint coupling. But in the context of Phase B warm-start corruption
(Fix F1), this normal component gets artificially amplified.

**After Fix F1**: Phase B's normal warm-start is `lambda_n_phaseb` (not inflated by Phase A
bounce). The coupling-induced `delta_lambda_n` should be small and balanced within each sweep.
Fix F1 is expected to resolve the oblique Z-velocity injection without additional changes to
the K matrix coupling.

**If oblique tests still fail after Fix F1**: The tangent basis may need to be aligned with the
sliding direction for each contact (the `setSlidingMode()` path already exists in
`ContactConstraint`). The `CollisionPipeline` currently does not call `setSlidingMode()` on
newly created constraints. Enabling this for constraints where initial tangential velocity is
detectable would reduce K_nt coupling effects by aligning the basis with the actual sliding
direction. However, this is a secondary fix — do not implement until Fix F1 is validated against
the oblique tests.

### Fix F3: Warm-Start Guard Improvement

**Current code**:
```cpp
const bool hasWarmStart = initialLambda.has_value() &&
                          initialLambda->size() == ... &&
                          initialLambda->maxCoeff() > 0.0;
```

The `maxCoeff() > 0.0` check is weak: it accepts any lambda vector with at least one positive
entry, even if the contact geometry has changed significantly. With Fix F1 (Phase B-only cache),
the normal lambda in the warm-start is the Phase B normal component only. For a bouncing contact,
Phase B's normal is typically small (Phase A handles the bounce, Phase B handles the post-bounce
resting correction). The warm-start guard remains correct but should be documented.

**No code change required for Fix F3** — the `maxCoeff() > 0.0` guard is adequate once Fix F1
is applied. The guard comment should be updated to explain what `initialLambda` contains.

---

## Integration Points

| Modified Component | Interacts With | Integration Type | Notes |
|--------------------|----------------|------------------|-------|
| `BlockPGSSolver::SolveResult` | `ConstraintSolver` | Data structure change | `phaseBLambdas` field added |
| `ConstraintSolver::SolveResult` | `CollisionPipeline` | Data structure change | `warmStartLambdas` field added |
| `CollisionPipeline` (cache write) | `ContactCache` | Write path | Use `warmStartLambdas` instead of `lambdas` |

---

## Implementation Order

The fixes must be applied in this order to avoid introducing new regressions:

1. **Step 1**: Add `phaseBLambdas` to `BlockPGSSolver::SolveResult` and populate in `solve()`.
   Run existing passing tests to confirm no regression. No behavior change yet (the field is
   unused).

2. **Step 2**: Add `warmStartLambdas` to `ConstraintSolver::SolveResult`. For BlockPGS path,
   assign from `blockResult.phaseBLambdas`. For ASM/PGS paths, assign from `lambdas` (identity).

3. **Step 3**: Change `CollisionPipeline` cache write to use `warmStartLambdas`. This is the
   behavioral change. Run all tests. The 12 failing tests should start to pass; verify that the
   768 currently-passing tests do not regress.

4. **Step 4** (if oblique tests still fail after Step 3): Enable `setSlidingMode()` in
   `CollisionPipeline::createConstraints()` for contacts with detectable initial tangential
   velocity. Measure against all oblique tests.

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Expected Outcome After Fix |
|-----------|-----------|--------|---------------------------|
| `FrictionSlidingTest.cpp` | `Oblique45_Slow` | Failing | Pass after Fix F1 |
| `FrictionSlidingTest.cpp` | `Oblique45_Medium` | Failing | Pass after Fix F1 |
| `FrictionSlidingTest.cpp` | `Oblique45` | Failing | Pass after Fix F1 |
| `FrictionSlidingTest.cpp` | `HighSpeedOblique` | Failing | Pass after Fix F1 |
| `FrictionSlidingTest.cpp` | `FrictionWithRestitution_BounceThenSlide` | Failing | Pass after Fix F1 |
| `ReplayEnabledTest.cpp` | `InelasticBounce_KEReducedByESquared` | Failing | Pass after Fix F1 |
| `ReplayEnabledTest.cpp` | `PerfectlyElastic_EnergyConserved` | Failing | Pass after Fix F1 |
| `ReplayEnabledTest.cpp` | `EqualMassElastic_VelocitySwap` | Failing | Pass after Fix F1 |
| `ParameterIsolationTest.cpp` | `TimestepSensitivity_ERPAmplification` | Failing | Pass after Fix F1 |
| `RotationDampingTest.cpp` | `RockingCube_AmplitudeDecreases` | Failing | Pass after Fix F1 |
| `RotationalCollisionTest.cpp` | `SphereDrop_NoRotation` | Failing | Pass after Fix F1 |
| `RotationalEnergyTest.cpp` | `ZeroGravity_RotationalEnergyTransfer_Conserved` | Failing | Pass after Fix F1 |
| All frictionless tests (768) | ASM path | Unaffected | Must remain passing |

### New Tests Required

The existing test suite is comprehensive. The 12 failing tests already cover the scenarios
diagnosed above. No new test cases are required for this ticket. The acceptance criterion is
that all 780 tests pass after implementation.

### Design Verification

The implementation should verify the following invariants after each sweep (useful for
debugging, may be gated behind a compile-time flag):

1. `result.lambdas(base) >= 0` for all contacts (non-negative normal impulse)
2. `||result.lambdas.tail<2>(base)|| <= mu * result.lambdas(base)` (Coulomb cone satisfied)
3. Phase B normal component: `result.phaseBLambdas(base) >= 0`
4. Phase A bounce: `bounceLambdas_[ci] >= 0` for all ci

---

## Open Questions

### Design Decisions

None blocking. The warm-start fix (F1) is clearly the correct approach: Phase A bounce impulses
are computed fresh each frame from current velocity and should not persist in the cache. Only
Phase B's dissipative contribution (which represents the sustained contact force) should be
warm-started.

### Prototype Required

**Yes** — but limited in scope. Before full implementation, the following diagnostic should be
run to confirm the warm-start hypothesis:

**Diagnostic prototype**: In `BlockPGSSolver::solve()`, temporarily zero out the bounce
component from the warm-start before initializing `vRes_`:
```cpp
// Diagnostic: strip Phase A from warm-start
// (This requires knowing Phase A from prior frame — not available, so instead:)
// Zero the warm-start entirely to confirm warm-start is the root cause.
const bool hasWarmStart = false;  // Temporary diagnostic
```

Run the oblique sliding tests and the restitution tests. If they pass (or substantially improve)
with warm-start disabled, Fix F1 is confirmed. If they still fail, there is a different root cause.

This diagnostic can be confirmed with the Replay MCP tools by inspecting frame-by-frame Z-velocity
in the `Oblique45` recording after each bounce.

### Requirements Clarification

None. All 12 failing tests have clear analytical expectations documented in the ticket. No test
assertion changes are permitted — the fix must make the physics correct, not relax the tests.

---

## Design Constraints

From the ticket's human design decisions:
- **Preserve two-phase architecture**: Phase A (restitution) and Phase B (dissipative) are retained
  without modification to their internal logic.
- **No (1+e) in Phase B RHS**: The dissipative guarantee of Phase B is maintained. Only the
  cache storage path changes.
- **No test-specific hacks**: The fix addresses the physics mechanism, not test tolerances.
- **Body force extraction**: Remains `J^T * lambda_total / dt` (not `vRes_ / dt`).
- **Frictionless (ASM) path**: Completely unaffected by all proposed changes.

---

## Iteration Log

This design will be tracked in:
`docs/designs/0084_block_pgs_solver_rework/iteration-log.md`

The implementer should create this file and record each prototype/implementation iteration
with findings.

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-28
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | `phaseBLambdas`, `warmStartLambdas` follow project `camelCase` member naming. `SolveResult` structs follow `PascalCase`. |
| Namespace organization | ✓ | All changes remain in `msd_sim` namespace. No new namespace introductions. |
| File structure | ✓ | Three existing files modified (`BlockPGSSolver.hpp`, `BlockPGSSolver.cpp`, `CollisionPipeline.cpp`). One transitional file (`ConstraintSolver.hpp`) modified. No new files. |
| Dependency direction | ✓ | No new dependencies introduced. Data flows in the existing direction: `BlockPGSSolver` → `ConstraintSolver` → `CollisionPipeline`. Adding a field to `SolveResult` does not alter dependency direction. |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | `Eigen::VectorXd phaseBLambdas` is a value type — no raw resource management. |
| Smart pointer appropriateness | ✓ | No new ownership introduced. Existing patterns unchanged. |
| Value/reference semantics | ✓ | `phaseBLambdas` and `warmStartLambdas` are value members in `SolveResult` structs (moved/copied with the struct). Correct choice for result aggregates. |
| Rule of 0/3/5 | ✓ | `BlockPGSSolver` and `ConstraintSolver` already satisfy Rule of Zero. Adding a `VectorXd` field to `SolveResult` preserves Rule of Zero (VectorXd has correct move semantics). |
| Const correctness | ✓ | `phaseBLambdas` is populated in `solve()` and returned. No const violations identified. |
| Exception safety | ✓ | `lambdaPhaseB` is already computed before the return path. Copying it into `phaseBLambdas` is the same cost as populating `result.lambdas`. No new exception risk. |
| Initialization | N | `warmStartLambdas` field proposed for `ConstraintSolver::SolveResult` is not shown with a default initializer. `ConstraintSolver::SolveResult` currently initializes `residual` to `quiet_NaN()`. The `warmStartLambdas` field will default-construct to an empty `VectorXd`, which is well-defined but should be documented. See Note 1. |
| Return values | ✓ | Design preserves the existing pattern of returning `SolveResult` by value. No output parameters introduced. |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | No new headers required. `Eigen::VectorXd` is already included in both solver headers. |
| Template complexity | ✓ | No templates involved. |
| Memory strategy | ✓ | `phaseBLambdas = lambdaPhaseB` is an O(3N) copy at solve completion — identical cost to the existing `result.lambdas` population loop. Since `lambdaPhaseB` is a member workspace, this is a copy from an already-computed vector. |
| Thread safety | ✓ | `CollisionPipeline` is single-threaded by design. No new concurrency concern. |
| Build integration | ✓ | Pure in-source change. No new compilation units, no new Conan dependencies. |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | `BlockPGSSolver` can be tested in isolation — the new `phaseBLambdas` field is accessible in `SolveResult`. A unit test can verify `phaseBLambdas(base) == lambdaPhaseB_n` and `phaseBLambdas(base) + bounceLambdas[ci] == lambdas(base)`. |
| Observable state | ✓ | The split between `lambdas` (total) and `phaseBLambdas` (Phase B only) is directly inspectable in `SolveResult`. The diagnostic prototype (disable warm-start) confirms the hypothesis without modifying production code. |
| No hidden global state | ✓ | `vRes_` and `bounceLambdas_` are per-instance workspace members. `BlockPGSSolver` is owned by `ConstraintSolver` by value. No singletons. |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | After Fix F1, oblique tests still fail due to K_nt coupling independent of warm-start | Technical | Low | Medium | Design specifies Fix F2 (setSlidingMode) as contingency; implement only if needed after prototype confirms F1 | Yes |
| R2 | `warmStartLambdas` empty in ASM/PGS path causes a cache update of zeros | Technical | Low | Medium | Design specifies `warmStartLambdas = lambdas` for non-BlockPGS paths. Must not be forgotten in the `ConstraintSolver::solve()` implementation. | No |
| R3 | Warm-start guard `maxCoeff() > 0.0` rejects valid all-zero warm-starts (first contact) | Technical | Low | Low | Correct by design — first contact has no cache entry. Design correctly notes this is not a bug. | No |
| R4 | `phaseBLambdas` copy adds ~3N double allocations per solve call | Performance | Low | Low | Amortized by existing `result.lambdas` loop. For N=12 contacts, 36 doubles = 288 bytes. Negligible. | No |

### Prototype Guidance

#### Prototype P1: Warm-Start Disable Diagnostic

**Risk addressed**: R1 (confirms warm-start is root cause before implementing F1)

**Question to answer**: Does disabling warm-start entirely eliminate the Z-velocity injection in oblique sliding tests and restore correct restitution behavior?

**Success criteria**:
- `Oblique45` Z-velocity drops below 2.0 m/s threshold
- `PerfectlyElastic_EnergyConserved` passes
- `InelasticBounce_KEReducedByESquared` passes
- At least 8 of 12 currently-failing tests pass with warm-start disabled

**Prototype approach**:
```
Location: prototypes/0084_block_pgs_solver_rework/p1_warmstart_disable/
Type: Source code patch (temporary flag in BlockPGSSolver.cpp)

Steps:
1. In BlockPGSSolver::solve(), add a constexpr bool kDisableWarmStart = true;
2. Guard the hasWarmStart block: if (hasWarmStart && !kDisableWarmStart)
3. Build debug-sim-only
4. Run the 12 failing tests and record pass/fail for each
5. If 8+ pass: warm-start hypothesis confirmed; proceed to Fix F1
6. If < 8 pass: investigate alternative root causes before implementing F1
7. Remove the diagnostic flag before committing
```

**Time box**: 30 minutes

**If prototype fails** (< 8 tests pass with warm-start disabled):
- Investigate `buildBlockK` for asymmetric lever arm errors
- Inspect `computeBlockVelocityError` sign convention for oblique contacts
- Use Replay MCP tools to inspect frame-by-frame lambda values

### Notes

**Note 1 — warmStartLambdas default initialization**: The proposed `ConstraintSolver::SolveResult` extension should explicitly document the default state of `warmStartLambdas`. An empty `VectorXd` (default-constructed) is the correct initial state (no warm-start available). The implementation must ensure that for all code paths through `ConstraintSolver::solve()` (ASM, PGS, and BlockPGS), `warmStartLambdas` is assigned before `CollisionPipeline` reads it. A defensive approach: initialize `warmStartLambdas = lambdas` at the top of `solve()` so that even an unexpected code path produces a valid (if non-optimal) value.

**Note 2 — Fix F3 (warm-start guard comment)**: The design correctly concludes no code change is needed for Fix F3. The comment improvement (documenting what `initialLambda` contains after Fix F1) should be included in the implementation commit to prevent future confusion about the cache invariant.

**Note 3 — Implementation order**: The three-step implementation order (add field → wire through → change cache write) is sound. Step 2 (wire `warmStartLambdas` through `ConstraintSolver`) must not be skipped even though it has no behavior change — it is necessary for the cache write change in Step 3 to be correct.

**Note 4 — ASM path warm-start identity**: For the ASM and PGS paths, setting `warmStartLambdas = lambdas` is semantically correct. Those solvers have no Phase A split, so the full lambda vector is the appropriate warm-start seed.

### Summary

The root cause analysis is rigorous: Phase A bounce impulses contaminating the warm-start cache is a plausible and well-supported single explanation for all 12 failures. The fix is surgically minimal — three files changed, two `VectorXd` fields added to two result structs, and one cache write redirected. No architectural changes, no Phase A or Phase B internal logic changes, and no test assertion changes.

The design should proceed to the prototype phase. The P1 diagnostic prototype (disable warm-start) should run first to confirm the hypothesis before implementing Fix F1. All acceptance criteria and design constraints from the ticket are satisfied by this approach.
