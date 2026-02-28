# Feature Ticket: Friction Force Distribution Regressions

## Status
- [x] Draft
- [x] Ready for Design
- [ ] Design Complete — Awaiting Review
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
- **Priority**: Medium
- **Estimated Complexity**: Medium
- **Target Component(s)**: msd-sim (Physics/Constraints)
- **Languages**: C++
- **Generate Tutorial**: No
- **Requires Math Design**: Yes
- **GitHub Issue**: TBD
- **Design Revision Count**: 0
- **Previous Design Approaches**: []

---

## Summary

Fix 3 test regressions introduced by ticket 0084's asymmetric decoupling in `BlockPGSSolver::sweepOnce`. The asymmetric decoupling (scalar `1/K(0,0)` for normal row, coupled `K_inv(1:2, 0:2)` for tangent rows) successfully eliminated oblique energy injection but changed the effective normal impulse magnitude at each contact. Since `1/K(0,0)` differs from `K_inv(0,0)` by the Schur complement factor (~1.818x for unit cube corners), the Coulomb cone bound (`mu * lambda_n`) is different, causing incorrect friction force distribution across the 4 corner contacts of a sliding cube.

## Motivation

The 0084 fix is a net positive (fixes 7/12 failures, stabilizes multi-body stacking), but introduces 3 regressions in axis-aligned sliding friction tests. These regressions indicate a friction force imbalance across corner contacts that causes:

1. **Lateral drift** — cube sliding in X drifts laterally in Y (and vice versa), exceeding the 0.002m threshold
2. **Tipping torque axis inversion** — friction at 4 corners should produce net torque about Y (forward tipping), but instead produces dominant omega_Z (Z-spin)

Both symptoms point to the same root cause: the scalar `1/K(0,0)` normal solve produces a different `lambda_n` per contact than the full coupled `K_inv(0,0)` would, and the resulting Coulomb cone clipping distributes friction forces unevenly across the 4 corners.

## Requirements

### Functional Requirements
1. Fix the 3 regressed tests without undoing the oblique energy injection fix from 0084
2. `SlidingCubeX_DeceleratesAndStops` — lateral Y drift must stay below 0.002m
3. `SlidingCubeY_DeceleratesAndStops` — lateral X drift must stay below 0.002m
4. `SlidingCube_FrictionProducesTippingTorque` — omega_Y must dominate omega_Z

### Non-Functional Requirements
- Must not regress the 7 tests fixed by 0084 (oblique sliding, ERP, RockingCube, BounceThenSlide)
- Must not regress the 768 tests that were passing before 0084
- Multi-body stacking stability (stack_8 benchmark) must be preserved

## Constraints
- Must build on 0084's asymmetric decoupling — the tangent→normal coupling severance is correct and must remain
- The normal→tangent coupling (K_inv rows 1-2) must be preserved for angular impulse balance
- The challenge is finding the right effective normal impulse that preserves the cone bound without reintroducing tangent→normal energy injection

## Acceptance Criteria
- [ ] SlidingCubeX lateral drift < 0.002m
- [ ] SlidingCubeY lateral drift < 0.002m
- [ ] FrictionProducesTippingTorque: omega_Y dominates omega_Z
- [ ] All 7 tests fixed by 0084 remain passing
- [ ] All 768 pre-0084 passing tests remain passing
- [ ] stack_8 benchmark remains stable

---

## Design Decisions (Human Input)

### Preferred Approaches
- Investigate whether the Schur complement ratio can be used to correct the scalar normal solve (e.g., use `K_inv(0,0)` for the normal row instead of `1/K(0,0)`, but suppress the off-diagonal terms)
- Consider the math formulation from 0084 (`docs/designs/0084_block_pgs_solver_rework/math-formulation.md`) which derives the Schur complement relationship and the per-contact indistinguishability theorem
- The recordings in `replay/recordings/` from the P4 prototype can be used to visualize the friction force imbalance

### Things to Avoid
- Do NOT revert to the full coupled 3x3 K_inv solve for row 0 — this reintroduces oblique energy injection
- Do NOT use per-contact velocity gates or geometry detection — proven ineffective in 0084 P5
- Do NOT use post-sweep redistribution — proven ineffective in 0084 P6

### Open Questions
- Can we use `K_inv(0,0)` (Schur complement value) instead of `1/K(0,0)` for the normal row while still severing the off-diagonal terms? This would preserve the cone bound magnitude.
- Is the lateral drift caused by the normal impulse magnitude difference, the tangent coupling change, or both?
- Are the 5 remaining original failures (3 restitution, 2 rotational) related to this same cone bound issue?

---

## References

### Related Code
- `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.cpp` — sweepOnce with asymmetric decoupling
- `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.hpp` — solver interface
- `msd/msd-sim/test/Physics/Collision/FrictionSlidingTest.cpp` — failing tests

### Related Documentation
- `docs/designs/0084_block_pgs_solver_rework/math-formulation.md` — K_nt coupling analysis, Schur complement derivation
- `docs/designs/0084_block_pgs_solver_rework/iteration-log.md` — Full prototype history (P1-P6)
- `docs/designs/0084_block_pgs_solver_rework/design.md` — Original design with revision history

### Related Tickets
- [0084](0084_block_pgs_solver_rework.md) — Parent ticket that introduced this regression
- [0075b](0075b_block_pgs_solver.md) — Original Block PGS implementation
- [0082b](0082b_friction_sliding_test_coverage.md) — Friction sliding test coverage

### Replay Recordings
- `replay/recordings/FrictionSlidingTest_SlidingCubeX_DeceleratesAndStops.db`
- `replay/recordings/FrictionSlidingTest_SlidingCubeY_DeceleratesAndStops.db`
- `replay/recordings/FrictionSlidingTest_SlidingCube_FrictionProducesTippingTorque.db`

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**:
- **Completed**:
- **Artifacts**:
  - `docs/designs/0085_friction_force_distribution_regressions/design.md`
  - `docs/designs/0085_friction_force_distribution_regressions/0085_friction_force_distribution_regressions.puml`
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
