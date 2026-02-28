# Design: Design-Revision Feedback Loop

## Summary

This design adds a structured "Implementation Blocked → Diagnosis → Human Gate → Design Revision → Re-entry" path to the workflow orchestrator. Currently, when implementation stalls on repeated failures, the workflow dead-ends at a generic "escalate to human" with no mechanism to feed implementation findings back to the designer and re-enter the workflow at the correct phase. The design introduces a new ticket status, a structured findings artifact, three escalation triggers that produce that artifact, a mandatory human gate, designer Mode 3 (revision from findings), design-reviewer revision awareness, orchestrator revision tracking with a cap and oscillation prevention, and an updated ticket template. No new agent files are created — diagnosis is an extended responsibility of existing agents.

## Architecture Changes

### PlantUML Diagram
See: `./0079_design_revision_feedback_loop.puml`

---

## New Components

### `implementation-findings.md.template`
- **Purpose**: Structured escalation artifact produced by the implementer, quality gate, or implementation reviewer when implementation stalls on a design-level problem. Replaces the generic "escalate to human" instruction in each of those agents.
- **Location**: `.claude/templates/implementation-findings.md.template`
- **Produced by**: cpp-implementer (circle detection), code-quality-gate (3rd consecutive failure), implementation-reviewer (3rd CHANGES REQUESTED for same fundamental issue)
- **Consumed by**: human (at the gate), cpp-architect (Mode 3), design-reviewer (revision-aware review)

**Template structure**:
```markdown
# Implementation Findings: {feature-name}

**Date**: {YYYY-MM-DD}
**Produced by**: {Implementer / Quality Gate / Implementation Reviewer}
**Trigger**: {Circle Detection / 3rd Quality Gate Failure / 3rd CHANGES REQUESTED}
**Revision Count**: {N of 2 maximum}

---

## What Was Attempted

### Iteration Summary
| Iteration | Hypothesis | Outcome | Files Changed |
|-----------|------------|---------|---------------|
| N | {hypothesis} | {PASS/FAIL — details} | {list} |

### Approaches Ruled Out
- **{Approach A}**: {Why it cannot work — specific technical reason}
- **{Approach B}**: {Why it cannot work}

---

## Failure Classification

**Category**: {one of the following}
- `DESIGN_FLAW` — The design's core approach is fundamentally incompatible with requirements
- `MISSING_ABSTRACTION` — The design does not expose the hook needed for this behavior
- `INCORRECT_INVARIANT` — The design encodes an invariant that contradicts the requirement
- `PERFORMANCE_CEILING` — The designed approach cannot meet the performance requirement
- `INTEGRATION_CONFLICT` — The design conflicts with an existing interface not anticipated at design time

**Classification Rationale**: {1-2 sentences explaining why this category fits}

---

## Root Cause Analysis

**Design Artifact with the Flaw**: `docs/designs/{feature-name}/design.md` — {section or decision ID}

**The Flawed Assumption**: {Precise description of what the design assumed that is false}

**Evidence**: {Concrete evidence from implementation: error messages, failing test output, or logical contradiction}

**Could Implementation Have Worked Around It?**
- [ ] Yes — but it would have required violating the design (not permitted)
- [x] No — the flaw is in the design itself and cannot be papered over at the implementation layer

---

## Proposed Design Change

**Scope**: {Delta — describe what needs to change. Be specific about which design decisions (DD-NNNN) are affected. Do NOT redesign what is still correct.}

**Proposed Approach**: {1-3 sentences describing the revised approach}

**What to Preserve**: {List design decisions and implementation files that are still valid and should not change}

**Warm-Start Hints for Implementer**:
- {File or module that can be preserved unchanged}
- {File that needs modification — describe the nature of the change}
- {File that must be discarded and reimplemented}

---

## Oscillation Check

**Have we tried this proposed approach before?**
- [ ] No — this is a new approach
- [ ] Yes — describe: {what happened when we tried it}

**Is there risk of oscillating back to the original (now-failed) approach?**
- [ ] No — the original approach is ruled out by: {evidence}
- [ ] Yes — risk is: {describe the attractor}; mitigation: {how to prevent it}

---

## Human Gate Checklist

The human should verify before approving the revision path:
- [ ] The root cause analysis is credible (the implementation was genuinely stuck, not just lacking effort)
- [ ] The proposed design change is scoped appropriately (delta, not full redesign)
- [ ] The oscillation check is complete
- [ ] This is revision N of maximum 2 — if at cap, close ticket and re-scope instead
- [ ] Prototype after revision: **Yes / No** (human decides per revision)
```

---

### New Ticket Status: `Implementation Blocked — Design Revision Needed`

- **Position in workflow**: Between "Ready for Implementation" and "Implementation Complete — Awaiting Test Writing"
- **Meaning**: Implementation has stalled on a design-level problem; an `implementation-findings.md` artifact has been produced; awaiting human gate approval to enter the Design Revision Loop
- **Transition into**: From any escalation trigger (circle detection, 3rd quality gate failure, 3rd CHANGES REQUESTED)
- **Transition out**: To Design Revision Loop (if human approves) or to closed (if human decides to re-scope)

---

## Modified Components

### `cpp-implementer.md` — Circle Detection Escalation Path

**Current behavior**: When a circle is detected (same file modified 3+ times with similar changes, oscillating test results, or recycled hypothesis), the agent documents the pattern in the iteration log and "escalates to the human with a summary." This is unstructured.

**Change required**: Replace the generic "escalate to human" instruction with a structured escalation that produces `implementation-findings.md`:

In **Phase 2.5, Circle Detection** section, replace:
```
If a circle is detected:
1. STOP making changes
2. Document the pattern in the iteration log under "Circle Detection Flags"
3. Escalate to the human with a summary of what has been tried and why approaches are cycling
```

With:
```
If a circle is detected:
1. STOP making changes
2. Document the pattern in the iteration log under "Circle Detection Flags"
3. Produce `docs/designs/{feature-name}/implementation-findings.md` using the template at
   `.claude/templates/implementation-findings.md.template`:
   - Set Produced by: Implementer
   - Set Trigger: Circle Detection
   - Fill "What Was Attempted" from the iteration log entries
   - Classify the failure (DESIGN_FLAW / MISSING_ABSTRACTION / INCORRECT_INVARIANT / etc.)
   - Complete the Root Cause Analysis section
   - Propose a scoped design change
   - Complete the Oscillation Check
4. Commit the findings artifact:
   git add docs/designs/{feature-name}/implementation-findings.md
   git commit -m "impl: circle detected — implementation-findings for {feature-name}"
   git push
5. Inform the orchestrator that ticket status should advance to
   "Implementation Blocked — Design Revision Needed"
```

**Constraint addition**: The implementer MUST NOT attempt any further implementation changes after detecting a circle. All subsequent work happens at the design level.

---

### `code-quality-gate.md` — 3rd Consecutive Failure Escalation Path

**Current behavior**: "If quality gate fails 3 times consecutively: Escalate to human operator. May indicate systemic issue requiring design revision. Document the pattern in the report."

**Change required**: Replace with a structured path that produces `implementation-findings.md` and adds a "Design Revision Recommendation" section to the quality gate report. In the **Maximum Iterations** section, replace:

```
### Maximum Iterations
If quality gate fails 3 times consecutively:
- Escalate to human operator
- May indicate systemic issue requiring design revision
- Document the pattern in the report
```

With:
```
### Maximum Iterations
If quality gate fails 3 times consecutively:
1. Add a "Design Revision Recommendation" section to the quality gate report:
   ```markdown
   ## Design Revision Recommendation

   **Status**: DESIGN REVISION RECOMMENDED
   **Consecutive Failures**: 3
   **Pattern**: {describe the persistent failure — what keeps failing and why fixes don't stick}
   **Hypothesis**: {Why this likely indicates a design-level issue rather than an implementation issue}
   **Recommended Action**: Produce implementation-findings.md and route to human gate
   ```
2. Produce `docs/designs/{feature-name}/implementation-findings.md` using the template at
   `.claude/templates/implementation-findings.md.template`:
   - Set Produced by: Quality Gate
   - Set Trigger: 3rd Quality Gate Failure
   - Fill "What Was Attempted" from the three quality gate reports
   - Classify the failure based on which gates keep failing
   - Complete remaining sections
3. Commit the findings artifact and the updated quality gate report
4. Inform the orchestrator that ticket status should advance to
   "Implementation Blocked — Design Revision Needed"
```

---

### `implementation-reviewer.md` — 3rd CHANGES REQUESTED Escalation Path

**Current behavior**: "If CHANGES REQUESTED 3 times for the same fundamental issue: Escalate to human operator. May indicate design problem, not implementation problem. Document the pattern for human review."

**Change required**: In the **Iteration Handling / Escalation** section, replace:

```
### Escalation
If CHANGES REQUESTED 3 times for the same fundamental issue:
- Escalate to human operator
- May indicate design problem, not implementation problem
- Document the pattern for human review
```

With:
```
### Escalation — 3rd CHANGES REQUESTED (Same Fundamental Issue)
If CHANGES REQUESTED 3 times for the same fundamental issue:
1. Determine that the issue is design-level (not merely an implementation shortcoming):
   - The same root cause appears in all three review cycles
   - The implementer cannot fix it without violating the design
2. Produce `docs/designs/{feature-name}/implementation-findings.md` using the template at
   `.claude/templates/implementation-findings.md.template`:
   - Set Produced by: Implementation Reviewer
   - Set Trigger: 3rd CHANGES REQUESTED
   - Fill "What Was Attempted" from the three review cycles (what the implementer tried each time)
   - Classify the failure
   - Complete Root Cause Analysis (cite the specific design decision that is flawed)
   - Propose a scoped design change
   - Complete the Oscillation Check
3. Commit the findings artifact:
   git add docs/designs/{feature-name}/implementation-findings.md
   git commit -m "review: 3rd escalation — implementation-findings for {feature-name}"
   git push
4. Inform the orchestrator that ticket status should advance to
   "Implementation Blocked — Design Revision Needed"
```

---

### `cpp-architect.md` — Mode 3: Revision from Implementation Findings

**Current behavior**: Two modes exist — Mode 1 (initial design) and Mode 2 (revision from design reviewer feedback).

**Change required**: Add a third operating mode. In the **Operating Modes** section, add:

```markdown
### Mode 3: Revision from Implementation Findings
Revise an existing design based on implementation failure findings. This mode is triggered when:
- The orchestrator sets ticket status to "Implementation Blocked — Design Revision Needed"
- `docs/designs/{feature-name}/implementation-findings.md` exists
- The human has approved the revision at the human gate

**In Mode 3**:
1. Read `docs/designs/{feature-name}/implementation-findings.md` in full
2. Read the existing `docs/designs/{feature-name}/design.md` (current design)
3. **Oscillation Guard**: Check the ticket metadata "Previous Design Approaches" list.
   If the proposed design change in findings.md matches an approach in that list, STOP and
   report to the orchestrator that the proposed revision would oscillate — do not proceed.
4. Identify the specific design decisions (DD-NNNN) that the findings cite as flawed
5. Produce a **delta design** — modify only the decisions cited in the findings:
   - Do NOT redesign sections that the findings identify as still correct
   - Do NOT restructure the overall architecture unless the findings require it
   - Apply the "What to Preserve" list from the findings
6. Append a "Design Revision Notes" section to design.md documenting:
   - Which design decisions changed and why (reference the findings)
   - What warm-start guidance was incorporated for the implementer
   - What was explicitly preserved
7. Update the PlantUML diagram only if the structural changes require it
8. Commit with prefix `design:` and message indicating this is a revision from findings
```

Also add to the **Process** section a step for Mode 3 handoff:

```markdown
### After Revision from Findings (Mode 3):
1. Confirm findings have been addressed — each cited flaw has a corresponding design change
2. Confirm oscillation guard was applied — the revision does not return to a prior approach
3. List warm-start hints for the implementer (which files can be preserved)
4. Commit revised design artifacts:
   git add docs/designs/{feature-name}/design.md
   git add docs/designs/{feature-name}/{feature-name}.puml  # if diagram changed
   git commit -m "design: revision from implementation findings for {feature-name}"
   git push
5. The design will proceed to design-reviewer in revision-aware mode
```

---

### `design-reviewer.md` — Revision-Aware Review Context

**Current behavior**: Review assumes the design is either an initial design (Iteration 0) or a reviewer-requested revision (Iteration 1). No concept of implementation-findings-driven revision.

**Change required**: Add a section describing revision-aware context. In the **Autonomous Iteration Protocol** section, add:

```markdown
### Revision-Aware Context (Mode 3 Revisions)
When reviewing a design that was revised in Mode 3 (from implementation findings):

**Before standard review, additionally:**
1. Read `docs/designs/{feature-name}/implementation-findings.md`
2. Verify that each flaw cited in the findings has a corresponding change in the revised design
3. Check the Oscillation Check section of findings.md — confirm the revision does not return
   to an approach documented in the ticket's "Previous Design Approaches" metadata
4. Verify that the delta scope is appropriate — the revision should not unnecessarily expand
   beyond what the findings required

**Additional review criteria for Mode 3**:
| Criterion | Question |
|-----------|----------|
| Root cause addressed | Does the revision directly address the flaw identified in findings? |
| Oscillation-free | Does the revision avoid returning to any prior approach? |
| Delta scope | Is the revision appropriately scoped (not an unnecessary full redesign)? |
| Warm-start preserved | Are the preservable components explicitly identified? |
| DD citation | Does the "Design Revision Notes" section cite the relevant DD-NNNN decisions? |

**Status determination for Mode 3 revisions**:
- Use the same status table as standard reviews
- REVISION_REQUESTED (Iteration 0) triggers autonomous architect iteration (still max 1)
- After autonomous iteration, produce final assessment with the additional Mode 3 criteria
```

---

### `workflow-orchestrator.md` — Design Revision Loop

**Current behavior**: Quality gate loop has a 3-failure escalation that terminates at "escalate to human" with no re-entry path.

**Change required**: Add a "Design Revision Loop" section to the orchestrator. Insert after the **Quality Gate Loop** section:

```markdown
### Design Revision Loop

When ticket status is "Implementation Blocked — Design Revision Needed":

**Step 1: Verify findings artifact exists**
- Confirm `docs/designs/{feature-name}/implementation-findings.md` exists
- If missing, log error and request human guidance — cannot proceed without findings

**Step 2: Human Gate (REQUIRED — no auto-skip)**
- Report to human with findings summary:
  - Which trigger fired (circle / quality gate / reviewer escalation)
  - The failure classification and root cause from findings.md
  - The proposed design change
  - Current revision count (N of 2 maximum)
- Wait for explicit human approval before proceeding
- Present the human gate checklist from findings.md
- Human decision options:
  A. Approve revision path → proceed to Step 3
  B. Close ticket (fundamental re-scope needed) → mark ticket Closed, stop workflow

**Step 3: Revision Tracking (Orchestrator)**
- Increment `Design Revision Count` in ticket metadata
- If count exceeds 2 (i.e., this would be the 3rd revision = 4th design attempt):
  - STOP — do not invoke the architect
  - Report to human: "Design revision cap reached (2 revisions). Ticket requires fundamental re-scoping."
  - Mark ticket as requiring human intervention
- Append the proposed approach from findings.md to `Previous Design Approaches` in ticket metadata
- Update ticket status to "Ready for Design" (re-entering design phase)

**Step 4: Execute Designer (Mode 3)**
- Invoke cpp-architect with Mode 3 context:
  - Path to implementation-findings.md
  - Current design.md
  - Previous Design Approaches list (for oscillation guard)
  - Instruction: delta design, not full redesign

**Step 5: Execute Design Reviewer (Revision-Aware)**
- Invoke design-reviewer with Mode 3 context:
  - Path to implementation-findings.md
  - Instruction to apply revision-aware criteria

**Step 6: Optional Prototype**
- Human decides whether a prototype is required for this revision (recorded at the human gate)
- If YES: invoke cpp-prototyper before re-entering implementation
- If NO: proceed directly to implementation re-entry

**Step 7: Implementation Re-entry**
- Update ticket status to "Ready for Implementation"
- Invoke cpp-implementer with:
  - Updated design.md
  - Warm-start hints from the findings artifact (which files can be preserved)
  - Full iteration log from previous implementation attempts
```

Also update the **Quality Gate Loop** section's "3rd consecutive failure" item to remove the generic escalation and instead reference the Design Revision Loop:

Replace:
```
3. **On 3rd consecutive failure**: Escalate to human, may indicate design issue
```
With:
```
3. **On 3rd consecutive failure**: Quality gate agent produces implementation-findings.md
   and advances ticket to "Implementation Blocked — Design Revision Needed". The Design
   Revision Loop then handles routing through the human gate.
```

---

### `ticket.md.template` — Status, Metadata, and Workflow Log Updates

**Change required**: Three additions to the ticket template.

**1. New status checkbox** (insert after "Ready for Implementation"):
```markdown
- [ ] Implementation Blocked — Design Revision Needed
```

**2. New metadata fields** (insert after "Requires Math Design"):
```markdown
- **Design Revision Count**: 0 (incremented by orchestrator; max 2)
- **Previous Design Approaches**: [] (populated by orchestrator; used for oscillation prevention)
```

**3. New Workflow Log section** (insert after Implementation Phase):
```markdown
### Design Revision Phase (if escalated from Implementation Blocked)
- **Revision Number**: {N of 2 maximum}
- **Trigger**: {Circle Detection / 3rd Quality Gate Failure / 3rd CHANGES REQUESTED}
- **Findings Artifact**: `docs/designs/{feature-name}/implementation-findings.md`
- **Human Gate Decision**: {Approved / Closed}
- **Approach Ruled Out**: {description of the prior approach now in Previous Design Approaches}
- **Delta Design Changes**:
  - {list design decisions that changed}
- **Warm-Start Preserved**:
  - {list files/modules preserved from prior implementation}
- **Prototype Required**: Yes / No
- **Notes**:
```

**4. New Human Feedback section** (insert after "Feedback on Implementation"):
```markdown
### Feedback on Design Revision (if Implementation Blocked)
{Your decision at the human gate: approve revision OR close ticket}
{Notes on scope of revision — what to change, what to preserve}
{Prototype decision for this revision: Yes / No}
```

---

## Test Impact

### Existing Tests Affected
None — this ticket modifies agent markdown files and templates, not C++ source. No existing tests are affected.

### New Tests Required
This is a workflow/process design. There are no automated tests for agent markdown files. Validation is via the acceptance criteria walk-through described in the ticket.

#### Manual Validation
| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| Walk-through 0075 scenario | Orchestrator, implementer, architect (Mode 3), design-reviewer | New path handles the Block PGS energy injection case correctly |
| Circle detection produces findings | cpp-implementer circle detection | findings.md is produced, not a generic escalation |
| 3rd QG failure produces findings | code-quality-gate max iterations | findings.md produced, Design Revision Recommendation in report |
| 3rd CHANGES REQUESTED produces findings | implementation-reviewer escalation | findings.md produced |
| Oscillation guard fires | cpp-architect Mode 3 | Refuses approach that matches Previous Design Approaches |
| Cap at 2 revisions | Orchestrator Design Revision Loop | 3rd revision attempt is blocked, human informed |
| Human gate is required | Orchestrator | No automatic advancement past blocked status |

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-27
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | Agent names, status labels, and artifact names all follow existing project conventions |
| Namespace organization | ✓ | N/A — no C++ namespaces; workflow agent responsibilities are clearly scoped |
| File/folder structure | ✓ | New template at `.claude/templates/`, design at `docs/designs/0079.../`, no new agent files per constraint |
| Dependency direction | ✓ | Escalation flows upward (implementer/QG/reviewer → findings artifact → orchestrator → designer). No cycles introduced. |

#### C++ Design Quality
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII / resource management | N/A | This is a workflow/process design; no C++ code produced |
| Smart pointer appropriateness | N/A | No C++ code |
| Rule of 0/3/5 | N/A | No C++ code |
| Const correctness | N/A | No C++ code |
| Exception safety | N/A | No C++ code |

#### Feasibility
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Agent modifications are specific | ✓ | Each modified agent section provides exact before/after text replacements — unambiguous to implement |
| Template completeness | ✓ | `implementation-findings.md.template` covers all five structural sections required by the functional requirements |
| Orchestrator loop is actionable | ✓ | Seven-step Design Revision Loop is specific enough for orchestrator implementation without ambiguity |
| Backward compatibility preserved | ✓ | All changes activate only on escalation triggers; the common (no-escalation) path is explicitly unchanged |
| Ticket template additions are additive | ✓ | New status checkbox, two metadata fields, one new workflow log section, one new feedback section — none alter existing fields |

#### Testability
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Validation approach exists | ✓ | Manual validation table in Test Impact section covers all acceptance criteria scenarios |
| Walk-through scenario defined | ✓ | The 0075 Block PGS scenario is explicitly identified as the primary validation case |
| Automated testing gap acknowledged | ✓ | Design correctly notes no automated tests exist for agent markdown files; manual walk-through is the appropriate method |

### Requirements Coverage

All 11 functional requirements from the ticket are addressed in the design:

| FR | Description | Design Section | Status |
|----|-------------|----------------|--------|
| FR1 | New ticket status | New Ticket Status section | ✓ |
| FR2 | Structured `implementation-findings.md` with 5 sections | Template structure in New Components | ✓ |
| FR3 | Implementer produces findings on circle detection | cpp-implementer changes | ✓ |
| FR4 | Quality gate Design Revision Recommendation on 3rd failure | code-quality-gate changes | ✓ |
| FR5 | Implementation reviewer produces findings on 3rd CHANGES REQUESTED | implementation-reviewer changes | ✓ |
| FR6 | Designer Mode 3 with delta design and oscillation guard | cpp-architect Mode 3 section | ✓ |
| FR7 | Design reviewer revision-aware context | design-reviewer changes | ✓ |
| FR8 | Orchestrator Design Revision Loop | workflow-orchestrator changes | ✓ |
| FR9 | Human gate REQUIRED | DD-0079-002, orchestrator Step 2 | ✓ |
| FR10 | Revision count tracked, cap at 2 | Orchestrator Step 3, ticket template | ✓ |
| FR11 | Previous Design Approaches tracked for oscillation prevention | DD-0079-005, orchestrator Step 3, architect Mode 3 | ✓ |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Ticket status re-uses "Ready for Design" on revision re-entry — an agent reading only the current status checkbox cannot distinguish initial design from revision | Maintenance | Low | Low | Orchestrator passes explicit Mode 3 context to architect agent; the distinction is in the invocation parameters, not the status alone | No |
| R2 | `implementation-findings.md` template in design.md may diverge from the actual `.claude/templates/implementation-findings.md.template` if the template is updated post-implementation | Maintenance | Low | Low | The template in design.md is illustrative; the canonical template is the file. This is documented in the New Components section. No action required. | No |
| R3 | Oscillation guard relies on string-matching of "proposed approach" descriptions in the Previous Design Approaches list — ambiguous wording could allow genuine oscillation to pass undetected | Technical | Low | Medium | The architect's Mode 3 oscillation guard is a judgment call, not a mechanical string-match. The human gate provides a final backstop before any revision is executed. The cap at 2 provides the hard limit. | No |

### Notes on Open Questions

The three open questions in the design all have good recommendations; none are blocking. They will need explicit human decisions before or at the time of implementation:

1. **Prototype after revision — required or optional?** Recommendation (Option B: per-revision human decision) is sound. The human gate checklist in the findings template already includes this decision point.

2. **Oscillation check — hard block or soft warning?** Recommendation (Option A: hard block) is the right call. A soft warning with human override creates ambiguity that erodes the value of the oscillation prevention mechanism.

3. **Warm-start hints in findings template?** Recommendation (Option A: include) is correct — the implementer is the best source of preservation guidance, and this is already reflected in the template's "Warm-Start Hints for Implementer" section.

### Prototype Guidance

No prototypes are warranted. This is a workflow meta-design with no C++ implementation uncertainty. The validation method is a manual walk-through of the 0075 scenario per the Test Impact section.

### Summary

The design is complete, internally consistent, and covers all 11 functional requirements. The five design decisions (DD-0079-001 through DD-0079-005) are well-reasoned with alternatives considered. The PlantUML state diagram accurately represents the described flow. The exact before/after text replacements for each modified agent file make implementation unambiguous.

The three open questions are informational — they have clear recommendations and are not blocking. The human should confirm the three open question recommendations (particularly the hard-block oscillation check) before or during implementation to lock those decisions.

**Design is ready to proceed to implementation.**

---

## Open Questions

### Design Decisions (Human Input Needed)

1. **Prototype after revision — required or optional?**
   - Option A: Always require a prototype after a design revision — ensures the revised approach is validated before another full implementation cycle
   - Option B: Human decides per-revision — allows skipping the prototype when the revision is trivial and the implementer has high confidence
   - **Recommendation**: Option B (per ticket requirements: "should the human decide per-revision?"). The overhead of a required prototype may be disproportionate for minor delta designs.
   - **Status**: Open — awaiting human confirmation per ticket "Open Questions" section

2. **Oscillation check — hard block or soft warning?**
   - Option A: Hard block (STOP) — architect cannot proceed if proposed approach matches Previous Design Approaches; must propose a different approach
   - Option B: Soft warning (WARN but allow human override) — architect flags the oscillation risk but human can override
   - **Recommendation**: Option A (hard block). Allowing human override creates a path to unbounded cycles; if the human truly needs to revisit a prior approach, they can remove it from Previous Design Approaches manually.
   - **Status**: Open — per ticket "Open Questions"

3. **Warm-start hints in findings template — include implementer file preservation hints?**
   - Option A: Include — implementer specifies which files can be preserved (warm-start hints)
   - Option B: Exclude — keep findings focused on root cause; let the architect specify warm-start guidance in the revised design
   - **Recommendation**: Option A (include in findings template). The implementer has the most context about which files are salvageable; feeding this to the architect reduces waste in the revision.
   - **Status**: Open — per ticket "Open Questions"

### Prototype Required
None — this is a workflow meta-design. No prototype is warranted.

### Requirements Clarification
None — the ticket provides comprehensive requirements and design decisions.

---

## Design Decisions

### DD-0079-001: No New Agent Files
- **Affects**: cpp-implementer, code-quality-gate, implementation-reviewer (escalation paths)
- **Rationale**: The agent closest to the failure has the most context for producing accurate findings. Creating a dedicated "diagnosis agent" would require that agent to reconstruct context it doesn't directly have. This matches the ticket constraint: "No new agent files."
- **Alternatives Considered**:
  - Dedicated diagnosis agent: Would need to read iteration logs, quality gate reports, and review histories to reconstruct what the implementer/gate/reviewer already knows. More complexity, less accuracy.
- **Trade-offs**: Each of the three triggering agents must carry the additional responsibility of producing `implementation-findings.md`. This is a modest increase in scope for each.
- **Status**: active

### DD-0079-002: Human Gate is Mandatory
- **Affects**: workflow-orchestrator Design Revision Loop
- **Rationale**: Design revision changes the fundamental approach of the feature. Automating this decision risks cascading wasted work if the revised approach is also wrong. The human gate ensures a human has reviewed the findings, agreed the root cause is correctly identified, and confirmed the proposed revision is sound.
- **Alternatives Considered**:
  - Auto-approve if findings are complete: Would eliminate the human checkpoint entirely. Rejected per requirement: "The human gate shall be REQUIRED."
- **Trade-offs**: Adds latency to the revision path. Acceptable given the weight of the decision.
- **Status**: active

### DD-0079-003: Design Revision Cap at 2 (3 Total Design Attempts)
- **Affects**: Orchestrator Revision Tracking, ticket metadata
- **Rationale**: If three distinct design approaches have all failed during implementation, the problem scope is likely wrong — not the designs. Beyond 2 revisions, the ticket should be closed and the feature re-scoped with better requirements.
- **Alternatives Considered**:
  - No cap: Risk of unbounded cycles.
  - Cap at 1 (2 total): Too aggressive — the first design is sometimes wrong in a way that a single well-targeted revision can fix, but the revision itself may have a residual flaw requiring one more revision.
- **Trade-offs**: Cap at 2 means the 3rd design attempt is the last. This is judged appropriate given the investment already made by that point.
- **Status**: active

### DD-0079-004: Delta Design, Not Full Redesign
- **Affects**: cpp-architect Mode 3
- **Rationale**: A full redesign wastes the decisions that are still correct. The findings artifact identifies the specific flaw; the revision should be surgical. Preserving correct decisions also reduces implementation rework (warm-start).
- **Alternatives Considered**:
  - Full redesign: Simpler architect logic (no need to identify what to preserve). Rejected because it throws away valid prior work and extends implementation time unnecessarily.
- **Trade-offs**: Architect must carefully scope the delta. The oscillation guard and "What to Preserve" section in findings.md provide the scaffolding for this.
- **Status**: active

### DD-0079-005: Oscillation Prevention via Previous Design Approaches List
- **Affects**: ticket metadata, orchestrator revision tracking, cpp-architect Mode 3 oscillation guard
- **Rationale**: Without an explicit record of ruled-out approaches, an architect could inadvertently propose a revision that returns to a previously-failed design. This is particularly likely if the revision is being done in a fresh session with limited context.
- **Alternatives Considered**:
  - Rely on iteration log only: The iteration log captures implementation attempts, not design approaches. A design approach can fail for reasons not visible in the iteration log.
  - No oscillation prevention: Risk of cycles that consume revision quota without making progress.
- **Trade-offs**: Requires the orchestrator to maintain and pass the Previous Design Approaches list. Small overhead; high value.
- **Status**: active
