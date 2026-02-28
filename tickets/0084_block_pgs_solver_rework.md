# Feature Ticket: Block PGS Solver Rework

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review (P1: warm-start disable — NEGATIVE)
- [x] Ready for Implementation
- [x] Implementation Blocked — Design Revision Needed (Revision 1: decoupled solve)
- [x] Design Approved — Ready for Prototype (P2)
- [x] Prototype Complete — Awaiting Review (P2: decoupled K_tt — NEGATIVE, new regressions)
- [x] Implementation Blocked — Design Revision Needed (Revision 2 required)
- [x] Design Revision 2 Complete — Fix F2 Implemented (Prototype P3 NEGATIVE — Revision 3 required)
- [x] Design Revision 3 Complete — Asymmetric Decoupling (Prototype P4 PARTIAL — Revision 4 required)
- [ ] Implementation Complete — Awaiting Test Writing
- [ ] Test Writing Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-02-27
- **Author**: Daniel Newman
- **Priority**: High
- **Estimated Complexity**: Large
- **Target Component(s)**: msd-sim (Physics/Constraints)
- **Languages**: C++
- **Generate Tutorial**: No
- **Requires Math Design**: No
- **GitHub Issue**: #112
- **Design Revision Count**: 3
- **Previous Design Approaches**: [Warm-start contamination (Phase A bounce in cache) — refuted by Prototype P1; Full decoupled K_tt solve — fixes oblique but breaks SlidingCubeX/TippingTorque, refuted by Prototype P2; Fix F2 warm-start cache only — no test changes, refuted by Prototype P3; Asymmetric decoupling (scalar row 0, K_inv rows 1-2) — same 3 regressions as P2 because 1/K(0,0) != K_inv(0,0) changes cone bound, refuted by Prototype P4]

---

## Summary

Rework the Block PGS solver implementation (originally 0075b) to fix 12 test failures exposed by the expanded test suite from 0082-series tickets. The new tests reveal that oblique sliding contacts produce excessive Z-velocity (up to 43 m/s vs 2 m/s threshold), several replay-enabled collision tests fail energy/velocity assertions, and the friction-with-restitution bounce-then-slide transition is broken. This ticket re-enters the 0075b work through the updated workflow (from 0079) with proper design revision support, quality gates, and structured diagnosis.

## Motivation

The 0082-series tickets added comprehensive test coverage that the original 0075b implementation was never validated against. These tests expose real physics bugs in the Block PGS solver:

1. **Oblique sliding produces vertical energy injection** — A cube sliding at 45 degrees gains massive Z-velocity (3.6 to 43 m/s depending on initial speed). This indicates the coupled normal-friction solve is injecting energy through the K_nt cross-coupling terms during oblique contact, exactly the class of bug the two-phase design was supposed to prevent.

2. **Elastic collision tests fail** — `PerfectlyElastic_EnergyConserved`, `EqualMassElastic_VelocitySwap`, `InelasticBounce_KEReducedByESquared` all fail. The restitution pre-solve (Phase A) may have incorrect impulse application or the velocity residual tracking is wrong.

3. **Timestep sensitivity / ERP amplification** — `ParameterIsolation_TimestepSensitivity_ERPAmplification` fails, suggesting position correction (Baumgarte) is interacting badly with the Block PGS warm-start.

4. **Rotational tests fail** — `RotationDampingTest_RockingCube`, `RotationalCollisionTest_SphereDrop_NoRotation`, `RotationalEnergyTest_ZeroGravity_RotationalEnergyTransfer_Conserved` all fail, indicating the 3x3 block solve may have incorrect rotational coupling.

5. **Friction+restitution transition** — `FrictionWithRestitution_BounceThenSlide` fails, indicating the Phase A → Phase B handoff is incorrect when both restitution and friction are active.

## Requirements

### Functional Requirements
1. The Block PGS solver shall pass all 12 currently-failing tests
2. The solver shall not regress any of the 768 currently-passing tests
3. Phase A (restitution pre-solve) shall correctly compute bounce impulses for elastic and inelastic collisions
4. Phase B (dissipative block PGS) shall not inject energy through K_nt coupling during oblique sliding
5. The Coulomb cone projection shall correctly bound tangential impulse for all contact orientations
6. Warm-start initialization shall correctly account for Phase A contributions to velocity residual
7. Body force extraction shall remain `J^T * lambda / dt` (not `vRes_ / dt`)

### Non-Functional Requirements
- **Energy Safety**: Total system energy shall be monotonically non-increasing for e=0 contacts (Phase B dissipative guarantee)
- **Convergence**: Block PGS shall converge within the configured iteration limit for all test scenarios
- **Backward Compatibility**: All frictionless (ASM path) tests must remain unaffected

## Constraints
- Must build on the 0075a data structure unification (already merged)
- Must preserve the two-phase architecture (Phase A restitution + Phase B dissipative) — the design decision separating `(1+e)` from the friction cone is sound (DD-0070-H2 prevention)
- NLopt remains behind runtime toggle until this solver is validated (0075c deferred)

## Acceptance Criteria
- [ ] All 780 tests pass (768 currently passing + 12 currently failing)
- [ ] No energy injection in oblique sliding scenarios (Z-velocity stays bounded)
- [ ] Elastic collisions conserve kinetic energy within tolerance
- [ ] Inelastic bounces reduce KE by e^2 factor
- [ ] Rocking cube amplitude decreases monotonically
- [ ] Friction+restitution transition produces correct bounce-then-slide behavior
- [ ] Rotational energy transfer conserved in zero-gravity scenarios

---

## Design Decisions (Human Input)

### Preferred Approaches
- Start with diagnosis of the 12 failures — classify them by root cause before changing any solver code
- The two-phase architecture (Phase A: restitution, Phase B: dissipative block PGS) is the right structure. The bug is likely in the implementation, not the architecture
- Use the replay MCP tools to inspect frame-by-frame state for failing tests — the recordings capture exactly what the solver produces
- Consider whether the oblique sliding failures share a root cause with the restitution failures (both involve velocity residual tracking)

### Things to Avoid
- Do NOT revert to decoupled normal-then-friction solve — the coupling is physically correct and needed
- Do NOT embed `(1+e)` in the Phase B RHS — this is the DD-0070-H2 energy injection mechanism
- Do NOT add test-specific hacks or tolerances to make tests pass without fixing the underlying physics
- Avoid changing test assertions unless the test expectation is provably wrong (the tests from 0082 are based on analytical physics)

### Open Questions
- Are the oblique sliding failures caused by Phase A (restitution) injecting energy that Phase B can't dissipate, or by Phase B itself injecting energy through cone projection?
- Is the velocity residual (`vRes_`) correctly initialized from warm-start for all contact orientations (not just axis-aligned)?
- Does the `buildBlockK()` computation correctly handle asymmetric lever arms (cube corner contacts)?
- Are the rotational failures related to the block solve or to a pre-existing issue in the angular velocity integration?

---

## References

### Related Code
- `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.hpp` — Two-phase solver interface
- `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.cpp` — Implementation to rework
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` — Friction dispatch to BlockPGSSolver
- `msd/msd-sim/src/Physics/Constraints/ContactConstraint.hpp` — Unified contact constraint (from 0075a)
- `msd/msd-sim/test/Physics/Collision/FrictionSlidingTest.cpp` — Oblique sliding tests (from 0082b)
- `msd/msd-sim/test/Physics/Collision/ReplayEnabledTest.cpp` — Collision/energy tests (from 0082d)

### Related Documentation
- `docs/designs/0075_unified_contact_constraint/design.md` — Original design (Phase 2 section)
- `docs/designs/0075_unified_contact_constraint/pre-implementation-review.md` — Energy injection analysis

### Related Tickets
- [0075](0075_unified_contact_constraint.md) — Parent architecture ticket
- [0075a](0075a_unified_constraint_data_structure.md) — Phase 1 data structure (merged, foundation for this work)
- [0075b](0075b_block_pgs_solver.md) — Original implementation (this ticket supersedes)
- [0075c](0075c_nlopt_removal.md) — Phase 3 cleanup (blocked by this ticket)
- [0082](0082_collision_test_restructure.md) — Test restructure that exposed these failures
- [0082b](tickets/0082b_friction_sliding_test_coverage.md) — Friction/sliding test coverage
- [0082c](0082c_multi_body_contact_tests.md) — Multi-body contact tests
- [0082d](0082d_test_rename_and_reorganize.md) — Test rename/reorganize
- [0079](0079_design_revision_feedback_loop.md) — Workflow improvements used by this ticket

---

## Current Failure Inventory

### Oblique Sliding Failures (5 tests)
| Test | Initial Speed | Z-Velocity (actual vs threshold) | Final Speed |
|------|--------------|----------------------------------|-------------|
| `Oblique45_Slow` | Low | 3.66 vs 2.0 | 1.62 (vs 0.001) |
| `Oblique45_Medium` | Medium | 10.0 vs 2.0 | 0.58 (vs 0.001) |
| `Oblique45` | Default | 21.1 vs 2.0 | 1.18 (vs 0.001) |
| `HighSpeedOblique` | High | 43.4 vs 2.0 | 2.52 (vs 0.001) |
| `FrictionWithRestitution_BounceThenSlide` | — | — | — |

### Replay-Enabled Collision Failures (7 tests)
| Test | Category |
|------|----------|
| `InelasticBounce_KEReducedByESquared` | Restitution |
| `PerfectlyElastic_EnergyConserved` | Restitution |
| `EqualMassElastic_VelocitySwap` | Restitution |
| `TimestepSensitivity_ERPAmplification` | Position correction |
| `RockingCube_AmplitudeDecreases` | Rotational damping |
| `SphereDrop_NoRotation` | Rotational symmetry |
| `ZeroGravity_RotationalEnergyTransfer_Conserved` | Rotational energy |

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-02-28 00:00
- **Completed**: 2026-02-28 12:00
- **Branch**: 0084-block-pgs-solver-rework
- **PR**: #113 (draft)
- **Artifacts**:
  - `docs/designs/0084_block_pgs_solver_rework/design.md`
  - `docs/designs/0084_block_pgs_solver_rework/0084_block_pgs_solver_rework.puml`
- **Notes**: Root cause identified across all 12 failing tests: Phase A bounce impulses
  (`bounceLambdas_`) were included in the total lambda stored in `ContactCache`. On the next
  frame, this inflated warm-start caused Phase A to compute incorrect bounce magnitudes (wrong
  velocity baseline) and Phase B to sweep from an incorrect initial state. The oblique Z-velocity
  injection (up to 43 m/s) results from K_nt coupling amplified by the inflated warm-start
  normal component. Fix: add `phaseBLambdas` to `BlockPGSSolver::SolveResult` and
  `warmStartLambdas` to `ConstraintSolver::SolveResult`; change `CollisionPipeline` cache
  writes to use Phase B-only lambdas. Two-phase architecture is sound — no structural changes
  needed. A diagnostic prototype (disable warm-start) should confirm the hypothesis before
  full implementation.

### Design Review Phase
- **Started**: 2026-02-28 13:00
- **Completed**: 2026-02-28 13:30
- **Branch**: 0084-block-pgs-solver-rework
- **PR**: #113 (draft)
- **Status**: APPROVED WITH NOTES
- **Artifacts**:
  - `docs/designs/0084_block_pgs_solver_rework/design.md` (review appended)
- **Reviewer Notes**: Root cause analysis accepted as rigorous and well-supported. Fix F1 (Phase B-only cache storage) is the correct surgical fix. Notes: (1) `warmStartLambdas` in `ConstraintSolver::SolveResult` should be defensively initialized to `lambdas` at top of solve() to cover all code paths; (2) Fix F3 comment update should be included in impl commit; (3) three-step implementation order is mandatory. Prototype P1 (warm-start disable diagnostic) must run before implementing Fix F1.

### Prototype Phase (P1)
- **Started**: 2026-02-28 06:00
- **Completed**: 2026-02-28 06:30
- **Branch**: 0084-block-pgs-solver-rework
- **PR**: #113 (draft)
- **Prototypes**:
  - P1: Warm-start disable diagnostic — NEGATIVE (hypothesis refuted)
- **Artifacts**:
  - `docs/designs/0084_block_pgs_solver_rework/prototype-results.md`
  - `docs/designs/0084_block_pgs_solver_rework/iteration-log.md`
  - `prototypes/0084_block_pgs_solver_rework/p1_warmstart_disable/patch.diff`
- **Notes**: P1 result is definitive: setting `hasWarmStart = false` in `BlockPGSSolver::solve()`
  produces identical failure values for all 12 tests. Root cause is the K_nt off-diagonal coupling
  in Phase B's `sweepOnce`. Design revision required before implementation.

### Prototype Phase (P2)
- **Started**: 2026-02-28 15:00
- **Completed**: 2026-02-28 16:00
- **Branch**: 0084-block-pgs-solver-rework
- **PR**: #113 (draft)
- **Prototypes**:
  - P2: Decoupled normal/tangent solve — NEGATIVE (new regressions introduced)
- **Artifacts**:
  - `docs/designs/0084_block_pgs_solver_rework/iteration-log.md` (Iteration 3 added)
  - `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.cpp` (prototype in place)
- **Notes**: P2 implemented the design-specified decoupled solve:
  `delta_lambda_n = (-vErr(0)) / K_nn` and `delta_lambda_t = K_tt.ldlt().solve(-vErr_t)`.
  Result: 6 of 12 original failures fixed (all 4 oblique variants, ERP, RockingCube).
  6 of 12 still fail (InelasticBounce, PerfectlyElastic, EqualMassElastic, SphereDrop,
  ZeroGravity, BounceThenSlide). NEW REGRESSIONS: SlidingCubeX, SlidingCubeY,
  SlidingCube_FrictionProducesTippingTorque (correct tipping torque is now Z-spin instead
  of Y-tipping — physically wrong).
  Root cause: K_tt^{-1} tangent solve loses the K_inv(1:2, 0) normal→tangent coupling terms
  that are required for correct per-contact angular impulse balance across 4 corner contacts.
  Additional finding: InelasticBounce/PerfectlyElastic/EqualMassElastic pass when warm-start
  is disabled — Fix F2 (phaseBLambdas cache) would likely address these 3.
  **Design revision required** to address coupling incompatibility in tangent solve.

### Design Revision Phase (Revision 3)
- **Started**: 2026-02-28 19:00
- **Completed**: 2026-02-28 20:00
- **Revision Number**: 3
- **Trigger**: Prototype P3 — Fix F2 only produced zero test changes
- **Human Gate Decision**: Approved Option 3 — Asymmetric decoupling (scalar row 0, K_inv rows 1-2)
- **Prototype P4 Result**: PARTIAL / NEGATIVE for regression goal
  - Fixed 7 of 12 original failures (all oblique tests, BounceThenSlide, ERP, RockingCube)
  - Still failing: InelasticBounce, PerfectlyElastic, EqualMassElastic, SphereDrop, ZeroGravity
  - Regressions: SlidingCubeX, SlidingCubeY, FrictionProducesTippingTorque (same 3 as P2)
- **Root cause of continued regression**: `1/K(0,0)` != `K_inv(0,0)`. The asymmetric scalar
  gives a larger normal impulse than the Schur-complement-reduced K_inv(0,0). This changes
  the Coulomb cone bound (mu * lambda_n), which disrupts the tangential impulse balance across
  4 corner contacts. The tipping torque result (omegaZ dominates vs omegaY) is equally severe
  as in P2. The cone coupling between lambda_n and lambda_t makes ANY change to the normal
  row affect tipping torque, regardless of what rows 1-2 use.
- **Conclusion**: The asymmetric decoupling approach is NEGATIVE. Design Revision 4 required.
  The structural incompatibility is confirmed: any change to row 0 that differs from K_inv(0,0)
  will affect the cone bound and break tipping torque. The correct fix must either: (a) use
  full K_inv for row 0 but conditionally clamp growth when vErr(0) >= 0 (velocity-gated
  Hypothesis B), or (b) find a different mechanism to prevent spurious lambda_n growth.
- **Branch**: 0084-block-pgs-solver-rework
- **PR**: #113 (draft)
- **Artifacts**:
  - `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.cpp` (P4 asymmetric decoupling implemented)
  - `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.hpp` (sweepOnce signature updated)
  - `docs/designs/0084_block_pgs_solver_rework/design.md` (Revision 3 section appended)
  - `docs/designs/0084_block_pgs_solver_rework/iteration-log.md` (Iteration 5 added)

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
- **Files Modified**:
- **Artifacts**:
  - `docs/designs/0084_block_pgs_solver_rework/implementation-notes.md`
- **Notes**:

### Design Revision Phase (Revision 1)
- **Started**: 2026-02-28 14:00
- **Completed**: 2026-02-28 14:30
- **Revision Number**: 1 of 2 maximum
- **Trigger**: Prototype P1 — warm-start disable produced zero change in all 12 failing tests
- **Findings Artifact**: `docs/designs/0084_block_pgs_solver_rework/prototype-results.md`
- **Human Gate Decision**: Approved — full decoupled solve preferred
- **Approach Ruled Out**: Fix F1 (Phase B-only cache storage) as primary fix — does not address
  actual root cause (K_nt coupling in 3x3 block solve)
- **Approach Also Ruled Out**: Hypothesis B (zero unconstrained(0) when vErr(0) >= 0) —
  rejected by human as band-aid masking the coupling issue
- **Delta Design Changes**:
  - Primary fix changed from "Phase B-only cache storage" to "decoupled normal/tangent solve in sweepOnce"
  - Root cause revised: K_nt off-diagonal coupling in 3x3 block solve (not warm-start contamination)
  - phaseBLambdas cache split retained as secondary correctness improvement (Fix F2)
  - blockKInvs precomputation removed from solve(); blockKs used directly in sweepOnce
  - sweepOnce parameter changed from blockKInvs to blockKs
- **What to Preserve**:
  - All existing data structures from 0075a (no header changes for Fix F1)
  - Two-phase architecture (Phase A restitution + Phase B dissipative) — both phases unchanged internally
  - Phase A applyRestitutionPreSolve — no change needed
  - Coulomb cone projectCoulombCone — no change needed
  - updateVRes3 and updateVResNormalOnly — no change needed
  - computeBlockVelocityError — no change needed
  - CollisionPipeline, ConstraintSolver — no changes for Fix F1 (only for secondary Fix F2)
- **Prototype Required**: Yes — Prototype P2 (decoupled solve validation)
- **Branch**: 0084-block-pgs-solver-rework
- **PR**: #113 (draft)
- **Artifacts**:
  - `docs/designs/0084_block_pgs_solver_rework/design.md` (revised — root cause + fix updated)
  - `docs/designs/0084_block_pgs_solver_rework/0084_block_pgs_solver_rework.puml` (revised)
  - `docs/designs/0084_block_pgs_solver_rework/iteration-log.md` (Iteration 2 added)
- **Notes**: Design revision approved by human. Next step is Prototype P2 (decoupled solve)
  to validate the fix before proceeding to full implementation. The prototype is a
  one-function change to sweepOnce in BlockPGSSolver.cpp.

### Design Revision Phase (Revision 2)
- **Started**: 2026-02-28 17:00
- **Completed**: 2026-02-28 18:00
- **Revision Number**: 2 of 3
- **Trigger**: Prototype P2 — decoupled solve introduced tipping-torque regressions in
  SlidingCubeX, SlidingCubeY, FrictionProducesTippingTorque
- **Human Gate Decision**: Approved Option A — implement Fix F2 (phaseBLambdas) first, revert P2
- **Action 1 — P2 Revert**: Restored coupled K_inv solve in `sweepOnce`; restored
  `blockKInvs` precomputation loop in `solve()`
- **Action 2 — Fix F2**: Implemented phaseBLambdas warm-start cache split across 4 files
- **Prototype P3 Result**: NEGATIVE — Fix F2 does NOT fix the 3 restitution tests.
  Warm-start diagnostic confirms: disabling warm-start entirely with coupled K_inv solve
  still fails all 3 restitution tests (ratio = 0.057, omegaZ = 3.14 regardless of warm-start).
  The iteration 3 finding "InelasticBounce passes with warm-start disabled" was specific to
  the P2 decoupled solve and does not apply to the coupled solve.
- **Conclusion**: ALL 12 failures require addressing K_nt coupling in the coupled 3x3 block
  solve. Fix F2 is a correct semantic improvement (no regressions, reduces a class of warm-start
  contamination at large timesteps) but does not fix any test. Design Revision 3 required.
- **Branch**: 0084-block-pgs-solver-rework
- **PR**: #113 (draft)
- **Artifacts**:
  - `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.cpp` (P2 reverted + F2 implemented)
  - `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.hpp` (F2: phaseBLambdas field)
  - `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` (F2: warmStartLambdas wired)
  - `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` (F2: warmStartLambdas field)
  - `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp` (F2: cache write uses warmStartLambdas)
  - `docs/designs/0084_block_pgs_solver_rework/design.md` (Revision 2 section appended)
  - `docs/designs/0084_block_pgs_solver_rework/iteration-log.md` (Iteration 4 added)
- **Notes**: Fix F2 is now in production. The restitution and oblique sliding failures all
  require a solution to the K_nt coupling incompatibility. The next design revision must
  explore options that preserve angular impulse balance while eliminating spurious normal
  impulse from K_nt coupling.

### Test Writing Phase
- **Started**:
- **Completed**:
- **Test Files Created**:
- **Test Coverage Summary**:
- **Test Failures Documented for Implementer**:
- **Notes**:

### Implementation Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Documentation Update Phase
- **Started**:
- **Completed**:
- **CLAUDE.md Updates**:
- **Diagrams Indexed**:
- **Notes**:

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}

### Feedback on Design Revision 1 (Decoupled Solve)
- **Decision**: Approve revision
- **Preferred approach**: Full decoupled solve — solve normal row with scalar K_nn independently, solve tangent with 2x2 subblock independently. Do NOT use Hypothesis B (zeroing unconstrained(0) when vErr(0) >= 0) as it's a band-aid that masks the coupling issue.
- **What to preserve**: Two-phase architecture (Phase A restitution + Phase B dissipative), existing data structures from 0075a
- **Prototype decision**: Yes — validate the decoupled solve fixes the oblique sliding tests before full implementation

### Feedback on Design Revision 2 (Post-P2)
- **Decision**: Approve revision — Option A
- **Preferred approach**: Implement Fix F2 (phaseBLambdas warm-start cache) first. This safely fixes the 3 restitution tests (InelasticBounce, PerfectlyElastic, EqualMassElastic) without regression risk. Revert the P2 decoupled solve code before implementing. The oblique sliding failures will be addressed in a subsequent pass after F2 is validated.
- **What to preserve**: Full coupled K_inv solve in sweepOnce (revert P2 changes), two-phase architecture, existing data structures
- **Prototype decision**: Yes — validate Fix F2 fixes the 3 restitution tests and causes no regressions

### Feedback on Design Revision 3 (Post-P3)
- **Decision**: Approve revision — Option 3 (Asymmetric decoupling)
- **Preferred approach**: Asymmetric decoupling — decouple only row 0 (normal row uses scalar K_nn, severing tangent→normal coupling that causes energy injection) while keeping rows 1-2 fully coupled (tangent rows use K_inv(1:2, 0:2), preserving normal→tangent coupling needed for tipping torque). This combines the best of both worlds: P2 showed oblique tests pass when row 0 is decoupled, and tipping torque only needs the rows 1-2 coupling.
- **What to preserve**: Fix F2 (phaseBLambdas warm-start cache) stays in place as a correct semantic improvement. Two-phase architecture preserved.
- **Prototype decision**: Yes — validate asymmetric decoupling fixes oblique sliding tests without regressing SlidingCubeX/TippingTorque

### Feedback on Tests
{Your comments on test coverage, test quality, or missing test scenarios}
