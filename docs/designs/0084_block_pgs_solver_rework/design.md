# Design: Block PGS Solver Rework

**Ticket**: [0084_block_pgs_solver_rework](../../../tickets/0084_block_pgs_solver_rework.md)
**Branch**: `0084-block-pgs-solver-rework`
**GitHub Issue**: #112
**PR**: #113

## Summary

The Block PGS solver (0075b) has 12 failing tests across two categories: oblique sliding
produces massive Z-velocity injection (up to 43 m/s from an initial speed of 6 m/s), and
replay-enabled collision tests fail for elastic/inelastic restitution, rotational dynamics, and
ERP amplification.

The prototype P1 (warm-start disable diagnostic) definitively refuted the original root cause
hypothesis: all 12 tests fail with identical values whether warm-start is enabled or disabled.
The failure occurs in Phase B `sweepOnce` itself — the 3x3 block solve's K_nt off-diagonal
terms allow tangential velocity error to drive a non-zero normal impulse correction on every
frame, even when the contact is not penetrating (`vErr(0) = 0`). This normal impulse
accumulates and injects upward velocity through `updateVRes3`.

**Revised fix**: Decouple the normal and tangent solves in Phase B's `sweepOnce`. Solve the
normal row using the scalar `K_nn` coefficient independently; solve the tangent rows using the
2x2 tangent subblock of K independently. This eliminates the K_nt coupling while preserving
the two-phase architecture (Phase A restitution + Phase B dissipative), the Coulomb cone
projection, and all existing data structures from 0075a.

A second prototype (P2) must validate that the decoupled solve fixes the oblique sliding tests
before full implementation proceeds.

---

## Architecture Changes

### PlantUML Diagram

See: [`./0084_block_pgs_solver_rework.puml`](./0084_block_pgs_solver_rework.puml)

The architecture is unchanged from 0075a. The diagram documents the corrected data flow within
`BlockPGSSolver::sweepOnce` and the decoupled solve algorithm.

### New Components

None. This ticket modifies one existing class (`BlockPGSSolver`).

### Modified Components

#### BlockPGSSolver

- **Current location**: `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.cpp`
- **Header location**: `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.hpp`
- **Changes required**: Revise `sweepOnce` to use decoupled normal/tangent solve (see Fix
  Specifications below). No header changes required — the `SolveResult` struct and public
  interface are unchanged.
- **Backward compatibility**: All frictionless (ASM path) tests are unaffected. The frictionless
  path in `ConstraintSolver` does not invoke `BlockPGSSolver`.

---

## Root Cause Analysis (Revised — Post-Prototype P1)

### Prototype P1 Finding

Setting `const bool hasWarmStart = false` in `BlockPGSSolver::solve()` produced zero change in
the 12 failing tests:

| Test | Before P1 | After P1 | Change |
|------|-----------|----------|--------|
| `Oblique45` | 21.1 m/s Z-vel | 21.1 m/s Z-vel | None |
| `HighSpeedOblique` | 43.4 m/s Z-vel | 43.4 m/s Z-vel | None |
| `InelasticBounce_KEReducedByESquared` | KE ratio 0.057 | KE ratio 0.057 | None |
| `EqualMassElastic_VelocitySwap` | omegaZ 3.14 | omegaZ 3.14 | None |

The Z-velocity injection for oblique contacts begins on frame 1, before any warm-start could
contribute. The original design's Fix F1 (Phase B-only cache storage) is NOT the primary fix
and will not resolve any of the 12 failing tests.

### The Actual Root Cause: K_nt Coupling in sweepOnce

In the current `sweepOnce`, for each contact the solver computes:

```cpp
const Eigen::Vector3d unconstrained = blockKInvs[ci] * (-vErr);
```

The 3x3 `blockKInvs[ci]` matrix (`K^{-1}`) has non-zero off-diagonal entries K_inv(0,1) and
K_inv(0,2). These exist because the 3x3 `K` matrix itself has non-zero off-diagonal K(0,1)
and K(0,2) terms, arising from the lever arm rotation coupling:

```
K(0,1) = -(rA×n)^T * IA_inv * (rA×t1) - (rB×n)^T * IB_inv * (rB×t1)
```

For a cube corner contact, `rA` has components in all directions, so `(rA×n)` and `(rA×t1)`
are non-parallel, giving K(0,1) != 0. The inverse of a matrix with non-zero K(0,1) will have
K_inv(0,1) != 0 as well (in general).

**The effect on an oblique sliding contact:**

For a cube sliding at 45 degrees on a flat floor (normal n = [0,0,1]):
- `vErr(0) = 0` (contact is not penetrating — cube is not sinking into floor)
- `vErr(1) != 0` (sliding in t1 direction)
- `vErr(2) != 0` (sliding in t2 direction)

The full 3x3 block solve computes:
```
unconstrained(0) = -K_inv(0,1) * vErr(1) - K_inv(0,2) * vErr(2)
```

Even though `vErr(0) = 0`, the normal row of the unconstrained correction is non-zero due to
the K_nt coupling terms. This `unconstrained(0)` accumulates into `lambdaPhaseB(base)` and
then `updateVRes3` applies `-n * dN` as a normal velocity contribution to body A. With
`n = [0,0,1]` pointing up, a positive `dN` gives upward velocity to body A (away from the
floor). Over multiple sweeps and frames, this grows without bound.

**Why this is not bounded by the Coulomb cone:**

The Coulomb cone projects the tangent component: `||lambda_t|| <= mu * lambda_n`. This limits
how large the tangential impulse can be relative to normal. But it does not prevent the normal
impulse from growing due to K_nt coupling. The normal component can grow freely as long as it
is positive (the cone allows any positive `lambda_n`). Each frame, `unconstrained(0)` from
K_nt adds more to `lambda_n`, and the cone constraint does not restrain this growth.

### Why All 12 Tests Share This Root Cause

**Oblique sliding (5 tests)**: K_nt coupling directly drives `unconstrained(0)` from the
non-zero `vErr(1,2)`. Energy injected via upward velocity accumulation. Magnitude scales with
sliding speed (explains 3.66 to 43.4 m/s range).

**Elastic/inelastic restitution (3 tests)**: Phase A computes the correct bounce impulse
(K_nn-based, unchanged). But Phase B then runs 50 sweeps after Phase A. For an elastic bounce
(e=1), Phase A has driven `vErr(0)` to zero. But the body is now moving in 3D with contact
point velocity having tangential components (from the orbital approach angle). Phase B's block
solve runs `K_inv * (-vErr)` where `vErr(1,2)` is non-zero (angular velocity from the bounce
creates tangential contact-point velocity). The K_nt coupling produces `unconstrained(0) != 0`,
driving a normal correction that counteracts Phase A's bounce. Result: bounce height lower than
expected (`InelasticBounce` KE ratio 0.057 vs expected 0.25).

**Rotational failures (3 tests)**: `EqualMassElastic_VelocitySwap` produces `omegaZ = 3.14`
(excessive angular velocity). The K_nt coupling allows tangential impulse to bleed into
normal correction, which in turn bleeds back to angular velocity via `updateVRes3`'s angular
term `-rA.cross(n) * dN`. This creates spurious angular velocity components.

**ERP amplification (1 test)**: At large timesteps (32ms), the velocity errors are larger,
the K_nt coupling-induced normal corrections are larger, and the Baumgarte ERP terms interact
with these corrections. The energy growth (13.9% vs 1% threshold) is amplified by the larger
timestep.

---

## Fix Specifications (Revised)

### Fix F1 (Revised): Decoupled Normal/Tangent Solve in sweepOnce

**File modified**: `BlockPGSSolver.cpp` (only — no header changes required)

**Previous algorithm in `sweepOnce`** (coupled 3x3 block):
```cpp
// Step 2: Unconstrained impulse correction (target: drive v_err to zero)
const Eigen::Vector3d unconstrained = blockKInvs[ci] * (-vErr);
```

**New algorithm** (decoupled normal + 2x2 tangent):

```cpp
// Step 2 (revised): Decoupled normal + tangent solve.
//
// The 3x3 block solve allows tangential velocity error (vErr(1,2)) to drive
// a normal correction via K_nt off-diagonal terms. For sliding contacts, this
// injects normal impulse on every frame even when vErr(0) = 0, causing upward
// velocity accumulation (the root cause of all 12 failing tests).
//
// Fix: solve normal row independently using scalar K_nn, then solve tangent
// rows independently using the 2x2 t1-t2 subblock. No K_nt cross-coupling.
//
// Normal row: delta_lambda_n = K_nn^{-1} * (-vErr(0))
//   K_nn = blockKs[ci](0,0)  (precomputed 3x3 K matrix, not K_inv)
//   For a non-penetrating contact (vErr(0) >= 0), this is zero or positive.
//
// Tangent rows: solve K_tt * delta_lambda_t = -vErr_t
//   K_tt = blockKs[ci].block<2,2>(1,1)  (lower-right 2x2 subblock)
//   Solved via LDLT or direct 2x2 inverse.

const double K_nn = blockKs[ci](0, 0);
const double delta_lambda_n = (K_nn > 1e-12) ? (-vErr(0)) / K_nn : 0.0;

// Solve 2x2 tangent subblock
const Eigen::Matrix2d K_tt = blockKs[ci].block<2, 2>(1, 1);
const Eigen::Vector2d vErr_t = vErr.tail<2>();
Eigen::Vector2d delta_lambda_t = Eigen::Vector2d::Zero();
{
  Eigen::LDLT<Eigen::Matrix2d> ldlt{K_tt};
  if (ldlt.info() == Eigen::Success)
    delta_lambda_t = ldlt.solve(-vErr_t);
}

Eigen::Vector3d unconstrained;
unconstrained(0)    = delta_lambda_n;
unconstrained.tail<2>() = delta_lambda_t;
```

The remaining `sweepOnce` logic (Steps 3–7: accumulate, project cone, update vRes) is
unchanged.

**Why this is correct:**

The normal row `K_nn` represents how a pure normal impulse changes the normal relative
velocity. Solving it independently means: "apply only as much normal impulse as is needed to
stop the interpenetration", without any contribution from tangential velocity.

The 2x2 tangent subblock `K_tt` represents how tangential impulses change tangential relative
velocity. Solving it independently (without K_nt coupling) means: "apply only as much tangential
impulse as is needed to reduce sliding", without any normal impulse from tangential state.

The Coulomb cone projection then bounds `||lambda_t|| <= mu * lambda_n`, which is applied after
accumulation, exactly as before.

**Energy safety of the revised algorithm:**

Phase B's dissipative guarantee requires that the block solve is non-energy-injecting. The
decoupled solve is equivalent to the original proof applied independently to each subspace:

- Normal subspace: `delta_lambda_n = -vErr(0) / K_nn`. After projection: `lambda_n >= 0`.
  For a non-penetrating contact (`vErr(0) >= 0`), this gives `delta_lambda_n <= 0`, which
  reduces or holds normal lambda — dissipative.
- Tangent subspace: `K_tt * delta_lambda_t = -vErr_t` (scalar/2x2 independent solve). The
  Coulomb cone then bounds `||lambda_t|| <= mu * lambda_n`. This is the standard decoupled
  friction proof used by Bullet and ODE.

**Performance impact:**

The 2x2 LDLT solve is added per contact per sweep. For N contacts, this is O(N) 2x2
decompositions per sweep vs O(N) 3x3 decompositions (the old `K_inv` precomputed once).
Since 2x2 LDLT is trivially fast (4 operations), the performance impact is negligible.
The `blockKInvs` precomputed array becomes unused and can be removed as cleanup.

**Header changes:**

None. `blockKs` (the precomputed 3x3 K matrices) are already passed to `sweepOnce` in the
current signature. The `blockKInvs` parameter can be replaced with the `blockKs` parameter,
or both can be passed temporarily. The simplest implementation change:

Replace `blockKInvs` parameter with `blockKs` in `sweepOnce` signature:

```cpp
// Old:
double sweepOnce(
  const std::vector<ContactConstraint*>& contacts,
  const std::vector<Eigen::Matrix3d>& blockKInvs,  // K^{-1}, full 3x3
  ...);

// New:
double sweepOnce(
  const std::vector<ContactConstraint*>& contacts,
  const std::vector<Eigen::Matrix3d>& blockKs,      // K (not K^{-1}) — decoupled solve
  ...);
```

This requires updating the `solve()` function to pass `blockKs` instead of `blockKInvs` to
`sweepOnce`. The `blockKInvs` vector and the LDLT precomputation loop in `solve()` can be
removed entirely.

### Fix F2: Retain phaseBLambdas for Cache Correctness (Secondary)

The original design's Fix F1 (Phase B-only cache storage) was shown by prototype P1 to NOT
fix the 12 failing tests. However, it remains conceptually correct as a secondary improvement:
Phase A bounce impulses should not persist in the warm-start cache, because they are stateless
(recomputed from current velocity each frame).

**Decision**: Implement Fix F2 (phaseBLambdas) as part of this revision. It is low-cost
(one `VectorXd` field addition to `SolveResult`), prevents a class of future bugs where
bounce-contaminated warm-start might cause issues at very large timesteps, and completes the
semantic correctness of the solver.

Fix F2 is unchanged from the original design's Fix F1 specification:
- Add `phaseBLambdas` to `BlockPGSSolver::SolveResult`
- Add `warmStartLambdas` to `ConstraintSolver::SolveResult`
- Change `CollisionPipeline` cache write to use `warmStartLambdas`

**Implementation order**: Fix F1 (decoupled solve) MUST be implemented and prototype-validated
before Fix F2. Fix F2 is a secondary correctness improvement, not a primary fix.

---

## Integration Points

| Modified Component | Interacts With | Integration Type | Notes |
|--------------------|----------------|------------------|-------|
| `BlockPGSSolver::sweepOnce` | `BlockPGSSolver::solve` | Internal | Pass `blockKs` instead of `blockKInvs` |
| `BlockPGSSolver::solve` | `ConstraintSolver` | No change | `SolveResult` interface unchanged for F1 |
| `BlockPGSSolver::SolveResult` (F2) | `ConstraintSolver` | Data structure | `phaseBLambdas` field added |
| `ConstraintSolver::SolveResult` (F2) | `CollisionPipeline` | Data structure | `warmStartLambdas` field added |
| `CollisionPipeline` cache write (F2) | `ContactCache` | Write path | Use `warmStartLambdas` instead of `lambdas` |

---

## Implementation Order

The fixes must be applied in this order to avoid introducing new regressions:

1. **Step 1 (F1 — primary)**: Revise `sweepOnce` to use decoupled normal/tangent solve:
   - Replace `blockKInvs` parameter with `blockKs`
   - Remove precomputed `blockKInvs` from `solve()` (simplification)
   - Implement decoupled: scalar K_nn for normal, 2x2 LDLT for tangent
   - Build and run all 780 tests. The 12 failing tests should pass.

2. **Step 2 (F2 — secondary)**: Add `phaseBLambdas` to `BlockPGSSolver::SolveResult`:
   - Populate `phaseBLambdas = lambdaPhaseB` before assembling total lambdas
   - Build; no behavior change yet

3. **Step 3 (F2 — secondary)**: Wire `warmStartLambdas` through `ConstraintSolver::SolveResult`:
   - For BlockPGS path: `warmStartLambdas = blockResult.phaseBLambdas`
   - For ASM/PGS paths: `warmStartLambdas = lambdas` (identity — no Phase A split)
   - Initialize `warmStartLambdas = lambdas` defensively at top of `solve()` (reviewer Note 1 from original design review)
   - Build; no behavior change yet

4. **Step 4 (F2 — secondary)**: Change `CollisionPipeline` cache write to use `warmStartLambdas`:
   - This is the behavioral change for F2
   - Run all 780 tests; verify no regression

**Why this order**: Step 1 is the primary fix with the behavior change. Steps 2–4 are
secondary correctness improvements that add the warm-start cache split. Running tests after
Step 1 confirms F1 fixes the 12 failures independently. Steps 2–4 add secondary correctness
without relying on F1 for their correctness.

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
3. `result.phaseBLambdas(base) >= 0` (non-negative Phase B normal)
4. `bounceLambdas_[ci] >= 0` for all ci (non-negative Phase A bounce)

---

## Open Questions

### Prototype Required (Yes)

Before full implementation, prototype P2 must validate the decoupled solve:

**Prototype P2**: In `BlockPGSSolver::sweepOnce`, replace the coupled 3x3 block solve with
the decoupled scalar/2x2 solve as specified in Fix F1. Run the 12 failing tests.

**Success criteria**: All 5 oblique sliding tests pass. At least 9 of 12 total tests pass.
If oblique sliding passes but restitution tests still fail, investigate Phase A's K_nn
computation (the `(1+e) * (-Jv_n) / K_nn` formula) — it may need adjustment if the block K
diagonal changes meaning when the solve is decoupled.

**Time box**: 30 minutes for the prototype, 30 minutes for analysis.

### What If P2 Fails for Restitution Tests?

If the oblique sliding tests pass after the decoupled solve but the restitution tests (3)
and/or rotational tests (3) still fail:

The remaining failures may have a separate root cause in Phase A. The `applyRestitutionPreSolve`
uses `computeBlockVelocityError` which computes `vErr(0) = n dot (vContactB - vContactA)`.
After Phase A updates `vRes_` with the bounce impulse, Phase B's first sweep computes a new
`vErr`. For an elastic bounce (e=1), Phase A should have zeroed `vErr(0)`. But if the contact
point velocity includes tangential components (from angular velocity), the 2x2 tangent solve
may drive a Phase B tangential correction that generates angular impulse feedback.

**Secondary investigation** (only if P2 restitution still fails): Check whether Phase A's
`computeBlockVelocityError` for the `Jv_n` (normal velocity check) is correct for contacts
with angular velocity components. The current formula is:
```cpp
vErr(0) = n.dot(vContactB - vContactA);
```
where `vContactA = vA + omegaA.cross(rA)`. This is velocity-level, not position-level. For a
falling body, `vA` is non-zero but `vContactA` should be the relative velocity at the contact
point. This is correct for a single-contact body. For multi-contact bodies (4 contact points),
each contact's `rA` and `rB` differ, so each Phase A computation sees a different `Jv_n`. No
action needed unless P2 still fails for these tests.

---

## Design Constraints

From the ticket's human design decisions:
- **Preferred approach**: Full decoupled solve — solve normal row with scalar K_nn
  independently, solve tangent with 2x2 subblock independently. **CONFIRMED as Fix F1.**
- **Do NOT use Hypothesis B** (zeroing `unconstrained(0)` when `vErr(0) >= 0`). The human
  explicitly rejected this as a band-aid that masks the coupling issue.
- **Preserve two-phase architecture**: Phase A (restitution) and Phase B (dissipative) are
  retained without modification to their internal logic. Only the block solve within Phase B
  `sweepOnce` changes.
- **No (1+e) in Phase B RHS**: The dissipative guarantee of Phase B is maintained.
- **No test-specific hacks**: The fix addresses the physics mechanism.
- **Body force extraction**: Remains `J^T * lambda_total / dt` (not `vRes_ / dt`).
- **Frictionless (ASM) path**: Completely unaffected by all proposed changes.
- **Existing data structures from 0075a**: Preserved. No header changes to `BlockPGSSolver.hpp`
  are required for Fix F1.

---

## Iteration Log

This design will be tracked in:
`docs/designs/0084_block_pgs_solver_rework/iteration-log.md`

The implementer should update this file with each prototype/implementation iteration finding.

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-28
**Status**: APPROVED WITH NOTES (original — for warm-start fix)
**Iteration**: 0 of 1 (no revision needed for that design)

*(Original design review retained below for traceability. The revised design replaces the
root cause analysis and fix specification. The architectural assessment, coding standards,
and risk table below require re-evaluation for the revised approach — see Design Revision
Review section.)*

### Original Design Review Criteria Assessment

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

#### Risks (Original — Superseded by Prototype P1 Findings)

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | After Fix F1, oblique tests still fail due to K_nt coupling independent of warm-start | Technical | **MATERIALIZED** | Medium | Prototype P1 confirmed: warm-start fix does not address K_nt coupling. **Design revision required.** | Yes |
| R2 | `warmStartLambdas` empty in ASM/PGS path causes a cache update of zeros | Technical | Low | Medium | Design specifies `warmStartLambdas = lambdas` for non-BlockPGS paths. | No |
| R3 | Warm-start guard `maxCoeff() > 0.0` rejects valid all-zero warm-starts | Technical | Low | Low | Correct by design. | No |
| R4 | `phaseBLambdas` copy adds ~3N double allocations per solve call | Performance | Low | Low | Negligible. | No |

### Original Notes

**Note 1 — warmStartLambdas default initialization**: Initialize `warmStartLambdas = lambdas`
at the top of `solve()` so that even an unexpected code path produces a valid value. This
defensive initialization pattern is mandatory for correctness.

**Note 2 — Fix F3 (warm-start guard comment)**: The comment improvement (documenting what
`initialLambda` contains after Fix F2 in this revision) should be included in the
implementation commit.

**Note 3 — Implementation order**: The three-step F2 implementation order (add field → wire
through → change cache write) is sound and unchanged from the original design.

**Note 4 — ASM path warm-start identity**: For the ASM and PGS paths, setting
`warmStartLambdas = lambdas` is semantically correct. Those solvers have no Phase A split,
so the full lambda vector is the appropriate warm-start seed.

---

## Design Revision Review

**Reviewer**: Design Reviewer (per workflow)
**Date**: 2026-02-28
**Revision trigger**: Prototype P1 refuted original root cause (warm-start contamination).
**Human decision**: Approve revision with full decoupled solve (scalar K_nn + 2x2 K_tt).

### Revised Root Cause Assessment

The prototype P1 result is unambiguous: all 12 tests fail identically with warm-start disabled.
The K_nt coupling hypothesis is the correct explanation. The revised root cause analysis is
logically coherent:

1. K(0,1) and K(0,2) are non-zero for cube corner contacts (lever arm cross terms confirmed
   in `buildBlockK` — the `JA.rightCols<3>() * IA_inv * JA.rightCols<3>().transpose()` term
   includes `(rA×n)^T * IA_inv * (rA×t1)` when expanded, which is non-zero for off-axis `rA`).

2. The 3x3 `K_inv` therefore has non-zero K_inv(0,1) and K_inv(0,2), mapping tangential
   `vErr` to normal impulse correction.

3. `updateVRes3` applies normal corrections as `linearA = -n * dN`, which is a Z-velocity
   injection when `n = [0,0,1]` and `dN > 0`.

4. The Coulomb cone does not prevent normal lambda growth — it only bounds tangential lambda
   relative to normal.

5. The decoupled solve directly severs the K_nt coupling path.

### Revised Architectural Assessment

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | No new names introduced for F1. `blockKs` replaces `blockKInvs` as parameter name — follows existing convention. |
| File structure | ✓ | Only `BlockPGSSolver.cpp` changes for F1. `sweepOnce` signature change (parameter rename) requires updating `BlockPGSSolver.hpp` declaration only for parameter name (not type). |
| Dependency direction | ✓ | No new dependencies. Removal of `blockKInvs` precomputation simplifies `solve()`. |
| RAII / Rule of Zero | ✓ | `Eigen::Matrix2d` and `Eigen::LDLT<Eigen::Matrix2d>` are stack-allocated value types. No heap or ownership changes. |
| Const correctness | ✓ | `blockKs` passed by const reference. `sweepOnce` const-correctness unchanged. |
| Exception safety | ✓ | `Eigen::LDLT<Eigen::Matrix2d>::solve()` does not throw. Fallback to zero on failure is already the pattern for the 3x3 case. |
| Performance | ✓ | Removes the 3x3 LDLT precomputation loop. Replaces with per-contact 2x2 LDLT in sweepOnce. Net cost equivalent or lower (2x2 vs 3x3 LDLT amortized). |
| Energy safety | ✓ | Decoupled solve proven dissipative for each subspace independently. See Fix F1 energy safety argument above. |

### Revised Risks

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Decoupled solve fixes oblique but not restitution tests | Technical | Medium | Medium | See Open Questions — secondary Phase A investigation path defined | Yes (P2) |
| R2 | Decoupled solve causes regression in currently-passing tests | Technical | Low | High | P2 prototype runs all 780 tests. Implementation Step 1 verifies before adding F2. | Yes (P2) |
| R3 | 2x2 K_tt subblock is singular for degenerate contacts | Technical | Low | Low | Same `LDLT::info() == Success` guard used as existing 3x3 path. Falls back to zero delta on failure. | No |
| R4 | Removing blockKInvs precomputation breaks any other caller | Technical | None | Low | `blockKInvs` is only used in `sweepOnce`. Confirmed by inspecting `BlockPGSSolver.cpp`. | No |

### Prototype P2 Specification

**Goal**: Validate that the decoupled normal/tangent solve in `sweepOnce` fixes the 12
failing tests without regressing the 768 passing tests.

**Implementation approach**:
```
Location: prototypes/0084_block_pgs_solver_rework/p2_decoupled_solve/
Type: Source code patch to BlockPGSSolver.cpp

Steps:
1. In sweepOnce(), replace the single line:
       const Eigen::Vector3d unconstrained = blockKInvs[ci] * (-vErr);
   with the decoupled solve as specified in Fix F1.

2. Update sweepOnce() parameter from blockKInvs to blockKs (or pass blockKs
   alongside blockKInvs and use blockKs[ci](0,0) and blockKs[ci].block<2,2>(1,1)).

3. Build with cmake --build --preset debug-sim-only.

4. Run the 12 failing tests; record pass/fail for each.

5. Run a representative sample of 10 currently-passing tests to check for regression.

6. If all 12 pass: proceed to full implementation (all 780 tests).
   If oblique tests pass but restitution fails: document secondary root cause.
   If regression found: investigate whether decoupled K_tt is missing rA/rB coupling.
```

**Success criteria**:
- All 5 oblique sliding tests pass (Z-velocity < 2.0 m/s threshold)
- At least 9 of 12 total failing tests pass
- Zero regression in the tested passing tests

**Time box**: 45 minutes.

### Summary

The revised design is mechanically sound. The decoupled normal/tangent solve is the correct
approach (per human decision) and is well-motivated by the K_nt coupling root cause. The
implementation change is surgical: one function (`sweepOnce`) is modified, one pre-computation
step in `solve()` is simplified, and no interface changes are required for Fix F1.

Fix F2 (phaseBLambdas warm-start cache split) is a secondary correctness improvement retained
from the original design, correctly ordered after F1 validation.

The design should proceed to Prototype P2 immediately.

---

## Design Revision 2 (Post-P2: Fix F2 Implementation)

**Trigger**: Prototype P2 (decoupled solve) produced new regressions in SlidingCubeX,
SlidingCubeY, and FrictionProducesTippingTorque. Human approved Option A: implement Fix F2
(phaseBLambdas warm-start cache) first, revert P2 changes, then address oblique sliding in
a subsequent pass.

**Human decision**: See ticket feedback section "Feedback on Design Revision 2".

### What Was Done in Design Revision 2

**Step 1 — Revert P2 prototype**: The decoupled solve changes in `sweepOnce` were reverted.
The coupled K_inv solve (`blockKInvs[ci] * (-vErr)`) was restored. The `blockKInvs`
precomputation loop in `solve()` was restored. The `sweepOnce` parameter was reverted to
`blockKInvs`.

**Step 2 — Implement Fix F2**: The phaseBLambdas warm-start cache split was implemented:
- `BlockPGSSolver::SolveResult::phaseBLambdas` field added (Phase B lambdas only)
- `ConstraintSolver::SolveResult::warmStartLambdas` field added (warm-start seed for next frame)
- BlockPGS path: `warmStartLambdas = blockResult.phaseBLambdas` (Phase B only, excludes bounce)
- ASM/PGS paths: `warmStartLambdas = lambdas` (no Phase A split, identity mapping)
- `CollisionPipeline` cache write changed to use `globalWarmStartLambdas` (from `warmStartLambdas`)

### Prototype P3: Fix F2 Validation

**Date**: 2026-02-28
**Goal**: Validate Fix F2 fixes the 3 restitution tests (InelasticBounce, PerfectlyElastic,
EqualMassElastic) without regressions.

**Result: NEGATIVE (partial)**

Fix F2 was implemented correctly and confirmed semantically sound:
- 768 tests pass (unchanged — no regressions introduced)
- 12 tests still fail (same 12 as before — unchanged)

**Critical finding**: The 3 restitution tests do NOT pass after Fix F2, even with warm-start
completely disabled (`hasWarmStart = false` diagnostic). This reveals that the iteration 3
finding ("InelasticBounce passes with warm-start disabled") was **specific to the P2 decoupled
solve context** and does NOT apply to the original coupled K_inv solve.

With the coupled K_inv solve:
- `InelasticBounce`: KE ratio = 0.057 (expected 0.125+), **regardless of warm-start state**
- `EqualMassElastic`: omegaZ = 3.14 rad/s (expected ~0.198), **regardless of warm-start state**
- `PerfectlyElastic`: maxHeight = 0.70 (expected > 1.0), **regardless of warm-start state**

The omegaZ = 3.14 signature is the K_nt coupling mechanism identified in the root cause
analysis: tangential vErr drives a normal correction that feeds back as angular velocity via
`updateVRes3`. This is not a warm-start issue — it is a Phase B K_nt coupling issue that
affects all contact frames, including the first bounce frame.

**Conclusion**: Fix F2 is a correct semantic improvement (prevents a class of warm-start
contamination at future timesteps) but is not the fix for the restitution test failures.
The root cause for ALL 12 failures is the K_nt coupling in the coupled 3x3 block solve.

### Status After Revision 2

| Fix | Status | Effect |
|-----|--------|--------|
| Fix F2 (phaseBLambdas cache) | Implemented | Correct semantic improvement; no regressions; does NOT fix any of the 12 failing tests |
| Fix for K_nt coupling | Required | Must address the 3x3 coupled block solve injecting normal impulse from tangential vErr |

The primary challenge remains: how to address K_nt coupling without introducing the
tipping-torque regression that P2's decoupled solve caused. The next pass must find an
approach that:
1. Eliminates the K_nt-driven normal impulse injection (fixes oblique sliding + restitution)
2. Preserves the K_inv row structure for correct per-contact angular impulse balance
   (prevents SlidingCubeX/TippingTorque regression)

This is a structural incompatibility in the 3x3 block solve that requires further investigation.
Design Revision 3 is needed to explore alternative approaches.
