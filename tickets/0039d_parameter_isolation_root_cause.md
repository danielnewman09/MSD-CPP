# Ticket 0038d: Parameter Isolation and Root Cause Identification

## Status
- [x] Draft
- [ ] Ready for Investigation
- [ ] Investigation Complete — Root Cause Identified
- [ ] Ready for Implementation
- [ ] Merged / Complete

**Current Phase**: Draft
**Assignee**: TBD
**Created**: 2026-02-05
**Generate Tutorial**: No
**Parent Ticket**: [0038_collision_energy_stabilization_debug](0038_collision_energy_stabilization_debug.md)
**Dependencies**: [0038a_energy_tracking_diagnostic_infrastructure](0038a_energy_tracking_diagnostic_infrastructure.md), [0038c_rotational_coupling_test_suite](0038c_rotational_coupling_test_suite.md)
**Type**: Investigation

---

## Overview

This ticket executes the systematic investigation to identify the root cause of the energy injection bug. Using the diagnostic infrastructure from 0038a and the failing tests from 0038c, we methodically isolate parameters and components until the culprit is found.

**Output**: A documented root cause with evidence, ready for 0038e to implement the fix.

---

## Requirements

### R1: Scenario Category G — Warm-Start Validation

| Test ID | Scenario | Expected Outcome | Validates |
|---------|----------|------------------|-----------|
| G1 | Cube bounces, teleport high mid-air, drop again | No "jump" or violent reaction on next impact | Cached impulse invalidation |
| G2 | Contact breaks and reforms with different normal | No energy spike on contact reformation | Normal-change handling |

### R2: Scenario Category H — Parameter Isolation

| Test ID | Scenario | Expected Outcome | Validates |
|---------|----------|------------------|-----------|
| H1 | Set Baumgarte bias (β) to 0.0 | Object sinks but energy doesn't increase | Baumgarte as energy source |
| H2 | Set restitution (e) to 0.0 | Immediate stabilization on contact | Restitution logic correctness |
| H3 | Set ERP to 0.0 (disable position correction) | Energy stable (object may penetrate) | Position correction as energy source |
| H4 | Set Friction (μ) to 0.0 | Energy stable (may slide forever) | Friction as energy source |
| H5 | Set Linear/Angular Damping to 0.0 | Bug may worsen (removes damping) | Damping sign/implementation correctness |
| H6 | Disable Warm Starting (initial λ = 0) | More iterations needed, but stable | Cached impulse corruption |

**Note (from Gemini review)**: H1 and H3 may be the same variable in some engines (Baumgarte bias = ERP / Δt). Clarify whether these are distinct in this codebase; if identical, merge tests.

### R3: Priority Investigation Protocol

Execute in order (stop when root cause found):

#### Priority 1: Baumgarte Stabilization (H3.3)
**Action**: Set Baumgarte bias/ERP to 0.0 temporarily.
**If energy stabilizes** (even if object sinks): Baumgarte is injecting energy. Tune the bias or clamp the correction impulse.
**Rationale**: This fixes ~60% of energy injection bugs in physics engines.

#### Priority 1.5: Warm Starting / Cached Impulses (NEW - from Gemini review)
**Action**: Disable warm starting (set initial λ to 0.0 for all contacts).
**If energy spikes stop**: Bug is in contact ID matching or lambda capping logic.
**Expected Result**: Convergence takes more iterations, but stability increases.
**Rationale**: In Sequential Impulse solvers, cached λ from previous frame applied without proper clamping when contact manifold changes introduces massive energy.

#### Priority 2: Integration Phase Order (H5.1)
**Action**: Verify the update loop order in `WorldModel::update()`:
- **Good (Semi-Implicit Euler)**: `Integrate Velocity → Detect Collision → Solve Constraints → Integrate Position`
- **Bad**: `Detect → Integrate → Solve` or `Solve → Integrate → Detect`
**Evidence**: Code inspection and state logging at each phase boundary.
**Specifically check**: Collision detection uses position/velocity from same timestep (no off-by-one frame artifacts).

#### Priority 3: Jacobian Cross Product Sign (H2.1 / H4.2)
**Action**: Verify:
- `r × n` direction is correct in `ContactConstraint::jacobianTwoBody()`
- `ApplyImpulse` computes `torque += cross(r, impulse)` with correct sign
- **Critical**: A sign flip causes object to spin *into* the floor, creating a feedback loop.
**Evidence**: Finite difference test of Jacobian vs numerical differentiation.

#### Priority 4: EPA Witness Point Stability (H1.3)
**Action**: Log contact point position AND contact normal frame-to-frame for a rocking cube.
**If variance is high** (>1cm position or >1-2° normal): Contact point/normal jitter causes lever arm changes that pump energy.
**Evidence**: CSV log of contact point coordinates AND normal vectors over 100 frames.
**Note (from Gemini review)**: Check both point jitter AND normal vector jitter. Even stable points with flipping normals cause solver to apply impulses in conflicting directions.

#### Priority 5: Pre-Impact Velocity Timing (H2.2)
**Action**: Verify that when body rotates, the velocity at contact point (`v + ω × r`) uses consistent state.
**Check**: If position/rotation updated *before* collision detection, but *old* velocities used for constraints (or vice versa), this creates a time-step mismatch that acts like negative damping.
**Evidence**: Log quaternion timestamp vs velocity timestamp in constraint creation.

#### Priority 6: Solver Iteration Count (NEW - from Gemini review)
**Action**: Check solver iteration count and convergence.
**If iteration count is too low** (1-2 iterations): Solver may not converge, leaving residual velocities that look like energy creation.
**Verification**: Reproduce bug at HIGH iteration counts (e.g., 100) to rule out simple non-convergence.

---

## Investigation Protocol

### Step 1: Reproduce Failing Test

Select the simplest failing test from 0038c (likely C2: Rocking Cube or C3: Tilted Cube Settles).

```cpp
// Run with diagnostic logging enabled
worldModel.setDiagnosticLogger(logger);
runTest_C2_RockingCube();
```

### Step 2: Energy Timeline Analysis

Using 0038a's energy tracker, generate:
1. Frame-by-frame energy CSV
2. Identify first frame where ΔE > ε (energy increased)
3. Correlate with collision events

**Questions to answer**:
- Does energy increase happen on collision frames?
- Does energy increase happen on non-collision frames?
- Is the increase gradual or sudden?

### Step 3: Parameter Knockout

Run the failing test with each parameter disabled:

| Test | Baumgarte (ERP) | Restitution | Friction | Warm Start | Expected Finding |
|------|-----------------|-------------|----------|------------|------------------|
| Baseline | ON | e=0.5 | ON | ON | Energy increases (bug reproduced) |
| Test H1 | OFF (=0) | e=0.5 | ON | ON | If stable → Baumgarte is culprit |
| Test H2 | ON | e=0 | ON | ON | If stable → Restitution logic error |
| Test H3 | 0.0 | e=0.5 | ON | ON | If stable → ERP is culprit |
| Test H4 | ON | e=0.5 | OFF (=0) | ON | If stable → Friction is culprit |
| Test H6 | ON | e=0.5 | ON | OFF | If stable → Warm start is culprit |
| Test X | ON | e=0.5 | ON | ON | Gravity OFF → If stable → Gravity×dt interaction |

### Step 4: Phase Bisection

For the failing configuration, log state at each pipeline phase:

```
Frame N:
  Pre-Detection:  E = 10.5 J
  Post-Detection: E = 10.5 J (no change expected)
  Post-Solve:     E = 10.7 J ← Energy increased here!
  Post-Apply:     E = 10.7 J
```

This identifies which phase introduces the error.

### Step 5: Component Drill-Down

Based on Step 4 findings:

#### If error in Constraint Creation (Post-Detection):
- Log lever arm vectors
- Verify `r_A = contactPoint - comA` is correct
- Check if pre-impact velocity uses stale state

#### If error in Solver (Post-Solve):
- Log effective mass matrix A
- Verify A is positive definite
- Log RHS vector b
- Verify restitution term sign: `b = -(1+e) * v_rel`
- Log solved λ values
- Verify λ ≥ 0 (non-negative impulse)

#### If error in Force Application (Post-Apply):
- Log linear force and torque applied
- Verify `torque = r × (λ * n)` with correct sign
- Verify force applied to correct body (A vs B)
- Check impulse-to-force conversion: `F = λ / dt` vs `F = λ` (impulse directly)

---

## Evidence Documentation Template

For each hypothesis investigated, document:

```markdown
### Hypothesis H{X}.{Y}: {Description}

**Test Performed**: {What was done}

**Data Collected**:
- {Metric 1}: {Value}
- {Metric 2}: {Value}

**Result**: CONFIRMED / RULED OUT / INCONCLUSIVE

**Evidence**:
{Paste relevant log output, numbers, or observations}

**Conclusion**:
{If confirmed: This is the root cause / contributes to the bug}
{If ruled out: Energy behavior unchanged when this parameter modified}
```

---

## Acceptance Criteria

1. [ ] **AC1**: Scenario G tests (G1-G2) implemented and documented
2. [ ] **AC2**: Scenario H tests (H1-H3) implemented and documented
3. [ ] **AC3**: Priority Investigation Order executed (at least through first confirmed root cause)
4. [ ] **AC4**: Root cause(s) identified with supporting evidence
5. [ ] **AC5**: At least one failing test case that reproducibly demonstrates the bug
6. [ ] **AC6**: Debug report written with all investigated hypotheses documented
7. [ ] **AC7**: Clear recommendation for fix approach ready for 0038e

---

## Deliverables

### D1: Debug Report

A markdown document containing:
1. Summary of root cause(s)
2. Evidence supporting the conclusion
3. All hypotheses investigated with results
4. Recommended fix approach

Location: `docs/designs/0038_collision_energy_stabilization_debug/debug-report.md`

### D2: Minimal Reproducing Test

A single test case that:
- Reproduces the bug reliably
- Is as simple as possible (minimal configuration)
- Can be run to verify the fix works

### D3: Diagnostic Logs

Raw CSV/JSON logs from the investigation:
- Energy timeline for failing test
- Contact state logs
- Force audit trail

Location: `docs/designs/0038_collision_energy_stabilization_debug/diagnostic-logs/`

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/Collision/WarmStartValidationTest.cpp` | Scenario G tests |
| `msd-sim/test/Physics/Collision/ParameterIsolationTest.cpp` | Scenario H tests |
| `docs/designs/0038_collision_energy_stabilization_debug/debug-report.md` | Investigation findings |

### Modified Files
| File | Change |
|------|--------|
| `msd-sim/test/CMakeLists.txt` | Add new test files |

---

## Estimated Effort

- Scenario G tests: ~2 hours
- Scenario H tests: ~2 hours
- Priority investigation execution: ~8 hours
- Documentation and report: ~4 hours

**Total**: ~16 hours

**Note**: Investigation time is highly variable. If root cause is found quickly (e.g., Priority 1 confirms Baumgarte), this could be much shorter. If multiple interacting bugs exist, it could be longer.

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Multiple root causes | Document all findings; fix may require multiple changes |
| Root cause not in hypothesis space | Add new hypotheses as discovered; update parent ticket |
| Inconclusive results | Increase logging granularity; add intermediate state captures |
| Investigation takes too long | Time-box each priority; escalate if >2 hours on single hypothesis |

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-05
- **Notes**: Subticket split from parent 0038. This is the detective work phase using diagnostics and failing tests to find the root cause.

### Gemini Review (2026-02-05)
**Status: Approved with Modifications**

Key changes incorporated:
1. **H4, H5, H6**: Added friction knockout, damping knockout, and warm-start disable tests
2. **Priority 1.5**: Added "Disable Warm Starting" as high-priority investigation step
3. **Priority 4**: Added normal vector jitter check (not just contact point jitter)
4. **Priority 6**: Added solver iteration count verification
5. **Step 3**: Expanded knockout table with friction and warm-start columns
6. **Note**: Added clarification about H1/H3 potentially being same variable (Baumgarte = ERP/Δt)

Additional hypotheses to consider if needed:
- Mass ratio bugs (heavy vs light object, infinite mass floor)
- Delta time (Δt) dependency (does bug vanish with fixed timestep?)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

