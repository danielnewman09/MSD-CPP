# Feature Ticket: Block PGS Solver Rework

## Status
- [x] Draft
- [x] Ready for Design
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
- **Created**: 2026-02-27
- **Author**: Daniel Newman
- **Priority**: High
- **Estimated Complexity**: Large
- **Target Component(s)**: msd-sim (Physics/Constraints)
- **Languages**: C++
- **Generate Tutorial**: No
- **Requires Math Design**: No
- **GitHub Issue**: #112
- **Design Revision Count**: 0
- **Previous Design Approaches**: []

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
- **Started**: 2026-02-27 00:00
- **Completed**: 2026-02-27 00:00
- **Branch**: 0084-block-pgs-solver-rework
- **PR**: #113 (draft)
- **Artifacts**:
  - `docs/designs/0084_block_pgs_solver_rework/design.md`
  - `docs/designs/0084_block_pgs_solver_rework/0084_block_pgs_solver_rework.puml`
- **Notes**: Root cause analysis identified four distinct failure mechanisms:
  (1) K_nt cross-coupling energy injection in Phase B RHS construction for oblique
  sliding, (2) Phase A uses current velocity not pre_impact_rel_vel_normal for
  restitution, (3) Phase B vRes_ not seeded from Phase A contributions,
  (4) ERP Baumgarte correction incorrectly embedded in Phase A rather than Phase B.
  Design introduces BlockPGSSolver replacing the decoupled normal-then-friction path.
  Two blocking open questions: FlattenedConstraints penetrationDepths extension
  (recommended: yes) and Phase B 3x3 vs tangent-only blocks (recommended: 3x3).

### Design Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Prototype Phase
- **Started**:
- **Completed**:
- **Prototypes**:
  - P1: {name} — {result}
- **Artifacts**:
  - `docs/designs/0083_block_pgs_solver_rework/prototype-results.md`
- **Notes**:

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
- **Files Modified**:
- **Artifacts**:
  - `docs/designs/0083_block_pgs_solver_rework/implementation-notes.md`
- **Notes**:

### Design Revision Phase (if escalated from Implementation Blocked)
- **Revision Number**: {N of 2 maximum}
- **Trigger**: {Circle Detection / 3rd Quality Gate Failure / 3rd CHANGES REQUESTED}
- **Findings Artifact**: `docs/designs/0083_block_pgs_solver_rework/implementation-findings.md`
- **Human Gate Decision**: {Approved / Closed}
- **Approach Ruled Out**: {description of the prior approach now in Previous Design Approaches}
- **Delta Design Changes**:
  - {list design decisions that changed}
- **Warm-Start Preserved**:
  - {list files/modules preserved from prior implementation}
- **Prototype Required**: Yes / No
- **Notes**:

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

### Feedback on Design Revision (if Implementation Blocked)
{Your decision at the human gate: approve revision OR close ticket}
{Notes on scope of revision — what to change, what to preserve}
{Prototype decision for this revision: Yes / No}

### Feedback on Tests
{Your comments on test coverage, test quality, or missing test scenarios}
