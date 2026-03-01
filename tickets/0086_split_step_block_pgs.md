# Feature Ticket: Split-Step Block PGS Solver

## Status
- [ ] Draft
- [ ] Ready for Design
- [x] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Blocked — Design Revision Needed
- [ ] Implementation Complete — Awaiting Test Writing
- [ ] Test Writing Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-02-28
- **Author**: Daniel Newman
- **Priority**: High
- **Estimated Complexity**: Medium
- **Target Component(s)**: msd-sim (Physics/Constraints)
- **Languages**: C++
- **Generate Tutorial**: No
- **Requires Math Design**: No
- **GitHub Issue**: TBD
- **Design Revision Count**: 0
- **Previous Design Approaches**: []

---

## Summary

Replace the single-pass asymmetric decoupled block solve in `BlockPGSSolver::sweepOnce` with a split-step algorithm that separates each sweep into two sequential passes: (1) normal-only solve using scalar `K(0,0)`, then (2) tangent-only solve using 2x2 `K_tt` sub-block with fixed normal lambdas. This resolves all 17 current test failures by eliminating K_nt coupling energy injection while preserving physics coupling through the velocity residual (`vRes_`) sequential update.

## Motivation

Ticket 0084 identified that the 3x3 coupled K-inverse in the block PGS solver creates an irreconcilable tension: K_nt off-diagonal terms simultaneously inject energy through oblique contacts AND provide correct tipping torque for symmetric corner contacts. Six prototype attempts (P1-P6) failed to resolve this within a single-pass block structure because every per-contact algebraic manipulation that reduces K_nt coupling for oblique contacts equally disrupts it for axis-aligned corner contacts.

### Current State: 772 pass, 17 fail

The 17 failures cluster into 4 categories, all traced to the BlockPGS friction solver:

| Category | Tests | Root Cause |
|----------|-------|------------|
| Cross-axis rotation leakage | 9 TiltedDropTest + FrictionProducesTippingTorque | Asymmetric decoupling `1/K(0,0)` changes Coulomb cone bound, disrupts tangential impulse balance |
| Lateral drift in axis-aligned sliding | SlidingCubeX, SlidingCubeY | Same cone bound distortion breaks sliding symmetry |
| Restitution/energy errors | InelasticBounce, PerfectlyElastic, VelocitySwap, SphereDrop_NoRotation | Phase A scalar K(0,0) + coupled tangent terms inject/lose energy |
| Zero-gravity rotational energy | ZeroGravity_RotationalEnergyTransfer | Coulomb cone projection asymmetrically clips tangential impulses |

### Why Split-Step Works Where P1-P6 Failed

Previous prototypes attempted to fix the block solve within a single pass per contact — either modifying K_inv rows, gating the normal correction, or post-processing the result. All failed because within a single pass, the tangent velocity error is computed with stale pre-pass-1 velocity state.

The split-step structurally eliminates this: Pass 2 (tangents) sees velocity errors computed AFTER Pass 1 (normals) has updated `vRes_` for ALL contacts. The angular velocity state already reflects normal impulses, so tangent solve naturally incorporates coupling through physics (sequential vRes_ updates) rather than through K matrix algebra (K_nt terms). This is a different information pathway that doesn't suffer from the Coulomb cone asymmetry issue.

## Requirements

### Functional Requirements
1. Rewrite `BlockPGSSolver::sweepOnce` as a two-pass split-step algorithm
2. Pass 1 (normals): scalar `K(0,0)` solve, `lambda_n >= 0` clamping, `updateVResNormalOnly`
3. Pass 2 (tangents): 2x2 `K_tt^{-1}` solve, Coulomb cone projection using fixed `lambda_n` from Pass 1, tangent-only vRes update
4. Pre-compute 2x2 `K_tt = K.block<2,2>(1,1)` inverse per contact (replaces 3x3 K^{-1})
5. Add `updateVResTangentOnly` helper (delegates to `updateVRes3` with zero normal)

### Non-Functional Requirements
- Must not regress any of the 772 currently passing tests
- Must not change Phase A (restitution pre-solve) or warm-start logic
- Must not change the `ConstraintSolver`, `CollisionPipeline`, or `ContactConstraint` interfaces
- Convergence behavior (max 50 sweeps, tolerance 1e-6) must be preserved

## Constraints
- Only `BlockPGSSolver.hpp` and `BlockPGSSolver.cpp` should change
- Tests that pin exact numerical values (RotationalEnergyTest, LinearCollisionTest) may need expected value updates if the split-step converges to a different (but physically correct) fixed point
- The 2x2 K_tt inversion inherits CFM regularization from the full K diagonal

## Acceptance Criteria
- [ ] All 9 TiltedDropTest failures fixed (cross-axis omega < 5% of norm)
- [ ] SlidingCubeX lateral drift < 0.002m
- [ ] SlidingCubeY lateral drift < 0.002m
- [ ] FrictionProducesTippingTorque: omega_Y dominates omega_Z
- [ ] PerfectlyElastic bounce height > 1.0m
- [ ] InelasticBounce KE ratio > 0.125
- [ ] SphereDrop_NoRotation: maxOmega < 0.5 rad/s
- [ ] ZeroGravity rotational energy drift < 1%
- [ ] All 772 currently passing tests still pass
- [ ] No new test regressions

---

## Design Decisions (Human Input)

### Preferred Approaches
- Split-step (normal pass then tangent pass) as described in the algorithm section
- Physics coupling preserved through sequential `vRes_` updates rather than K_nt matrix terms
- 2x2 `K_tt` inversion via Eigen LDLT (robust for symmetric PD)
- `updateVResTangentOnly` delegates to `updateVRes3({0, dT1, dT2})` to avoid code duplication

### Things to Avoid
- Do NOT reintroduce full 3x3 coupled solve — proven energy injection source
- Do NOT use K_inv rows 1-2 for tangent (that's the P4 asymmetric approach which has the cone bound issue)
- Do NOT modify Phase A, warm-start, or force extraction — these are correct
- Do NOT add geometric detectors, velocity gates, or post-sweep redistribution — all proven ineffective in P1-P6

### Open Questions
- Will the 2x2 K_tt solve produce sufficient tipping torque for `FrictionProducesTippingTorque`? The coupling enters through sequential `vRes_` updates rather than K_nt terms. This is the key risk — empirical validation needed.
- What are the correct pinned values for `EqualMassElastic_VelocitySwap` and `ZeroGravity_RotationalEnergyTransfer` under the split-step? These will need re-derivation after implementation.

---

## Algorithm Detail

### sweepOnce — Split-Step Per Sweep

```
Pass 1 — Normal-only:
  for each contact ci:
    vErr = computeBlockVelocityError(ci)
    delta_n = -vErr(0) / K(0,0)
    lambda_n = max(0, lambda_n_old + delta_n)
    updateVResNormalOnly(ci, lambda_n - lambda_n_old)

Pass 2 — Tangent-only:
  for each contact ci:
    vErr = computeBlockVelocityError(ci)     // reflects ALL Pass 1 normal updates
    delta_t = K_tt_inv * (-vErr.tail<2>())
    lambda_t = lambda_t_old + delta_t
    project: ||lambda_t|| <= mu * lambda_n   // lambda_n FIXED from Pass 1
    updateVResTangentOnly(ci, lambda_t - lambda_t_old)
```

### Pre-computation Change

Replace `std::vector<Eigen::Matrix3d> blockKInvs` with `std::vector<Eigen::Matrix2d> tangentKInvs` in `solve()`. Compute via:
```cpp
Eigen::Matrix2d K_tt = K.block<2,2>(1,1);
Eigen::LDLT<Eigen::Matrix2d> ldlt{K_tt};
tangentKInvs.push_back(ldlt.solve(Eigen::Matrix2d::Identity()));
```

---

## References

### Related Code
- `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.hpp` — solver interface (sweepOnce signature change)
- `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.cpp` — core algorithm (sweepOnce rewrite, pre-computation change)
- `msd/msd-sim/test/Physics/Collision/TiltedDropTest.cpp` — 9 cross-axis rotation tests
- `msd/msd-sim/test/Physics/Collision/FrictionSlidingTest.cpp` — lateral drift + tipping torque tests
- `msd/msd-sim/test/Physics/Collision/LinearCollisionTest.cpp` — elastic/inelastic bounce tests
- `msd/msd-sim/test/Physics/Collision/RotationalEnergyTest.cpp` — zero-gravity rotational energy test

### Related Documentation
- `docs/designs/0084_block_pgs_solver_rework/math-formulation.md` — K_nt coupling analysis, Schur complement, cone projection injection mechanism
- `docs/designs/0084_block_pgs_solver_rework/iteration-log.md` — P1-P6 prototype history with root cause analysis
- `docs/designs/0084_block_pgs_solver_rework/design.md` — Original design with 5 revision sections

### Related Tickets
- [0084](0084_block_pgs_solver_rework.md) — Parent ticket (6 failed prototypes, blocked at Design Revision 6)
- [0085](0085_friction_force_distribution_regressions.md) — Superseded by this ticket (targeted 3 regressions from P4; split-step addresses all 17)
- [0084a](0084a_tilted_drop_rotation_tests.md) — TiltedDropTest suite (quality gate passed, tests now fail under P4)
- [0075b](0075b_block_pgs_solver.md) — Original Block PGS implementation

### Replay Recordings
- `replay/recordings/TiltedDropTest_TiltedDrop_X_Small_BounceIsolation.db`
- `replay/recordings/FrictionSlidingTest_SlidingCubeX_DeceleratesAndStops.db`
- `replay/recordings/FrictionSlidingTest_SlidingCube_FrictionProducesTippingTorque.db`
- `replay/recordings/ReplayEnabledTest_LinearCollisionTest_PerfectlyElastic_EnergyConserved.db`
- `replay/recordings/ReplayEnabledTest_RotationalEnergyTest_ZeroGravity_RotationalEnergyTransfer_Conserved.db`

---

## Workflow Log

### Design Phase
- **Started**: 2026-02-28 (advanced from Draft, Requires Math Design: No)
- **Completed**: 2026-02-28
- **Branch**: 0084-block-pgs-solver-rework
- **PR**: TBD
- **Artifacts**:
  - `docs/designs/0086_split_step_block_pgs/design.md`
  - `docs/designs/0086_split_step_block_pgs/0086_split_step_block_pgs.puml`
- **Notes**: Design documents the split-step algorithm in full detail. Math design skipped
  per metadata (Requires Math Design: No) — parent ticket 0084 math-formulation.md covers
  K_nt coupling theory. Key design decisions recorded: DD-0086-001 (2x2 K_tt sub-block),
  DD-0086-002 (updateVResTangentOnly delegates to updateVRes3 with dN=0),
  DD-0086-003 (fixed lambda_n cone bound in Pass 2), DD-0086-004 (aggregate convergence metric).
  Open risk: FrictionProducesTippingTorque coupling via sequential vRes_ updates (not K_nt).

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}
