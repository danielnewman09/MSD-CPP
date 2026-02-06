# Ticket 0038e: Fix Implementation and Regression Testing

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Draft
**Assignee**: TBD
**Created**: 2026-02-05
**Generate Tutorial**: No
**Parent Ticket**: [0038_collision_energy_stabilization_debug](0038_collision_energy_stabilization_debug.md)
**Dependencies**: [0038d_parameter_isolation_root_cause](0038d_parameter_isolation_root_cause.md)
**Type**: Implementation

---

## Overview

This ticket implements the fix for the root cause(s) identified in 0038d, verifies all test scenarios pass, and ensures no regressions in existing collision tests.

**Prerequisite**: 0038d must be complete with documented root cause(s) before this ticket can begin.

---

## Requirements

### R1: Implement Fix Based on Root Cause

The specific fix depends on 0038d's findings. Common fixes by root cause:

#### If Baumgarte/ERP Energy Injection (H3.3):
- Clamp Baumgarte correction velocity to not exceed separation velocity
- Use slop-based threshold to avoid micro-corrections
- Consider switching to split impulse (position correction separate from velocity)

**Note (from Gemini review)**: If switching to Split Impulse is required, spawn a sub-ticket (0038f) as this refactors the Solver interface significantly.

#### If Jacobian Sign Error (H2.1 / H4.2):
- Correct `r × n` sign in `ContactConstraint::jacobianTwoBody()`
- Ensure consistent body-A/body-B convention throughout

#### If Integration Order Error (H5.1):
- Reorder phases in `WorldModel::update()` to correct sequence
- Ensure collision detection uses post-force-integration state

#### If EPA Contact Point Jitter (H1.3):
- Add contact point smoothing/filtering
- Implement contact caching with distance threshold
- Consider persistent contact IDs

#### If Pre-Impact Velocity Timing (H2.2):
- Ensure pre-impact velocity is captured at consistent state
- Verify quaternion and velocity states are synchronized

### R2: Verify All Test Scenarios Pass

After fix implementation, all tests from 0038a-0038d must pass:

| Suite | Tests | Expected Result |
|-------|-------|-----------------|
| 0038b | A1-A5, F1-F3, F5 | All pass (baseline) |
| 0038c | B1-B4, C1-C4, D1-D4, F4 | All pass (rotation) |
| 0038d | G1-G2, H1-H3 | All pass (parameter isolation) |

### R3: Regression Testing

Ensure no regression in existing tests:

| Test Suite | File | Expected Result |
|------------|------|-----------------|
| WorldModel Integration | `WorldModelContactIntegrationTest.cpp` | All pass |
| Constraint Solver Contact | `ConstraintSolverContactTest.cpp` | All pass |
| Constraint Solver ASM | `ConstraintSolverASMTest.cpp` | All pass |
| Collision Handler | `CollisionHandlerTest.cpp` | All pass |
| **Stacking Stability** | *New test required* | **3-5 cube vertical stack stable for 500 frames** |

**Note (from Gemini review)**: Fixes that limit energy injection often have the side effect of causing objects to sink into the ground (engine becomes too "timid" to push them out). The stacking test catches this regression.

### R4: Energy Invariant Verification

Add permanent test that enforces the energy invariant:

```cpp
TEST(EnergyInvariant, RestitutionImpulseNeverInjectsEnergy)
{
  // For any collision with e ≤ 1:
  // The RESTITUTION impulse component must not add energy
  //
  // NOTE (from Gemini review): If using Baumgarte stabilization,
  // the BIAS impulse may add energy to resolve penetration.
  // This is expected and bounded by penetration depth.
  //
  // This test specifically validates:
  // 1. Restitution impulse: E_post ≤ E_pre (within numerical tolerance)
  // 2. Bias impulse: Energy increase bounded by penetration potential energy
  //
  // Run multiple scenarios (linear, rotational, multi-contact)
  // Assert total energy increase never exceeds bias tolerance
}
```

**Critical distinction (from Gemini review)**:
- **Restitution impulse**: Must NEVER add energy (e ≤ 1 guarantees this)
- **Baumgarte/Bias impulse**: MAY add energy to resolve penetration, but must be bounded

If using Split Impulse (position correction separate from velocity), the strict E_post ≤ E_pre invariant applies.

This test should be part of the permanent test suite, not just a diagnostic.

---

## Implementation Checklist

### Pre-Implementation
- [ ] Read 0038d debug report and understand root cause
- [ ] Review recommended fix approach from 0038d
- [ ] Identify all files that need modification

### Implementation
- [ ] Implement fix in identified file(s)
- [ ] Add inline comments explaining the fix and referencing ticket
- [ ] Ensure fix follows project coding standards

### Verification
- [ ] Run 0038b linear collision tests — all pass
- [ ] Run 0038c rotational coupling tests — all pass
- [ ] Run 0038d parameter isolation tests — all pass
- [ ] Run existing collision test suites — no regressions
- [ ] Run full `msd_sim_test` suite — no regressions

### Documentation
- [ ] Update CLAUDE.md if fix changes architectural behavior
- [ ] Add regression test that prevents recurrence
- [ ] Document fix in 0038 workflow log

---

## Acceptance Criteria

From parent ticket 0038:

1. [ ] **AC6**: Angled cube drop eventually stabilizes
2. [ ] **AC7**: No regression in existing collision tests
3. [ ] **AC8**: Restitution impulse never increases energy across collision frames for e ≤ 1 (bias impulse may add bounded energy)

Additional:

4. [ ] **AC9**: All 0038b/c/d test scenarios pass
5. [ ] **AC10**: Permanent energy invariant test added to test suite
6. [ ] **AC11**: Fix documented with ticket reference in code comments
7. [ ] **AC12**: Stacking stability test passes (3-5 cube stack, no sinking)
8. [ ] **AC13**: Visual jitter check - velocity approaches zero after settling (sleep criteria)

---

## Files to Modify

*To be determined after 0038d identifies root cause.*

Likely candidates:
| File | Potential Change |
|------|------------------|
| `ConstraintSolver.cpp` | Baumgarte clamping, RHS assembly |
| `ContactConstraint.cpp` | Jacobian sign correction |
| `CollisionPipeline.cpp` | Phase ordering, force application |
| `ContactConstraintFactory.cpp` | Pre-impact velocity timing |
| `WorldModel.cpp` | Integration order |

---

## Regression Test Strategy

### Minimal Reproducing Case

Convert the minimal failing case from 0038d into a permanent regression test:

```cpp
TEST(CollisionRegression, Ticket0038_AngledCubeStabilizes)
{
  // Setup: Cube at 45° angle, drop onto floor
  // Run: 500 frames
  // Assert: Cube comes to rest (velocity < threshold)
  // Assert: Energy never increased during any frame
}
```

### Energy Invariant Test

```cpp
TEST(CollisionRegression, Ticket0038_EnergyMonotonicDecrease)
{
  // For various scenarios (sphere bounce, cube bounce, rocking cube):
  // Assert: E(frame N+1) ≤ E(frame N) + ε for all N
}
```

---

## Quality Gate

Before marking implementation complete:

1. **Code Review Checklist**:
   - [ ] Fix is minimal (no unrelated changes)
   - [ ] Fix addresses documented root cause
   - [ ] No new warnings introduced
   - [ ] Follows coding standards

2. **Test Coverage**:
   - [ ] All new code paths have tests
   - [ ] Edge cases covered (e=0, e=1, mass ratios)
   - [ ] Multi-body scenarios tested

3. **Performance**:
   - [ ] No significant performance regression
   - [ ] Benchmark collision detection if changes are in hot path
   - [ ] If EPA smoothing/filtering implemented, benchmark "1000 colliding pairs" to ensure no CPU spike

4. **Visual Verification** (from Gemini review):
   - [ ] No visible jitter (object may oscillate between valid states even with conserved energy)
   - [ ] Velocity ≈ 0 after N seconds of settling (sleep criteria verification)

---

## Estimated Effort

Highly dependent on root cause complexity:

| Root Cause Type | Estimated Fix Time |
|-----------------|-------------------|
| Simple sign error | 2-4 hours |
| Baumgarte tuning | 4-8 hours |
| Integration order | 4-8 hours |
| Contact point stability | 8-16 hours |
| Multiple interacting bugs | 16-24 hours |

Regression testing and documentation: ~4 hours regardless of fix complexity.

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-05
- **Notes**: Subticket split from parent 0038. Implementation details to be filled in after 0038d identifies root cause.

### Gemini Review (2026-02-05)
**Status: Approved with Changes**

Key changes incorporated:
1. **R1**: Added note about spawning 0038f if Split Impulse refactor is required
2. **R3**: Added stacking stability regression test (3-5 cube stack)
3. **R4**: Clarified energy invariant - distinguish restitution impulse (must not add energy) from bias impulse (may add bounded energy)
4. **AC8**: Refined to specify "restitution impulse" not total energy (Baumgarte may add bounded energy)
5. **AC12, AC13**: Added stacking stability and visual jitter verification
6. **Quality Gate**: Added EPA smoothing performance benchmark and visual jitter check

Critical theoretical note:
- If using standard Baumgarte stabilization, `E_post ≤ E_pre` is theoretically impossible for penetrating objects (bias adds velocity to push apart)
- The test must either use Split Impulse OR relax the invariant to allow energy increase proportional to penetration depth

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

