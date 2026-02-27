# Feature Ticket: Design-Revision Feedback Loop

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Test Writing
- [ ] Test Writing Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-02-26
- **Author**: Daniel Newman
- **Priority**: Medium
- **Estimated Complexity**: Medium
- **Target Component(s)**: workflow orchestrator, agent definitions, ticket templates
- **Languages**: C++
- **Generate Tutorial**: No
- **Requires Math Design**: No
- **GitHub Issue**:

---

## Summary

Add a structured "Implementation Blocked → Diagnosis → Human Gate → Design Revision → Re-entry" path to the workflow orchestrator. Currently, when implementation fails repeatedly, the workflow dead-ends at "escalate to human" with no mechanism to feed implementation findings back to the designer and re-enter the workflow at the right phase.

## Motivation

Ticket 0075 demonstrated this gap concretely: the Block PGS solver design embedded `(1+e)` restitution in the PGS RHS, which reproduced the DD-0070-H2 energy injection bug. No amount of implementation iteration could fix it — the design itself needed to change to a two-phase approach (Phase A resolve-to-zero + Phase B restitution bias). This design revision was handled ad-hoc across multiple sessions with no structured diagnosis, no durable artifact capturing what was tried, and no formal re-entry into the workflow.

The current escalation paths (quality gate 3rd failure, implementer circle detection, reviewer 3x escalation) all terminate at "escalate to human" — an unstructured dead end. This ticket adds a structured alternative: produce a diagnosis artifact, route through a human gate, invoke the designer in revision mode, and re-enter the implementation workflow.

## Requirements

### Functional Requirements

1. The system shall add a new ticket status "Implementation Blocked — Design Revision Needed" between "Ready for Implementation" and "Implementation Complete"
2. The system shall produce a structured `implementation-findings.md` artifact when implementation stalls, containing: what was attempted, failure classification, root cause analysis, proposed design change, and oscillation check
3. The implementer agent shall produce `implementation-findings.md` on circle detection (replacing generic "escalate to human")
4. The quality gate agent shall add a "Design Revision Recommendation" section to its report on 3rd consecutive failure
5. The implementation reviewer agent shall produce `implementation-findings.md` on 3rd `CHANGES REQUESTED` for the same fundamental issue
6. The designer agent shall support a new "Mode 3: revision from implementation findings" that produces a delta design (not a full redesign) informed by the findings artifact
7. The design reviewer agent shall have awareness of design revision context (read findings, verify revision addresses root cause, check for oscillation)
8. The orchestrator shall implement a Design Revision Loop that routes from "Implementation Blocked" through human gate → designer → reviewer → optional prototype → implementation re-entry
9. The human gate shall be REQUIRED — design revision shall never be fully automated
10. The system shall track design revision count in ticket metadata and cap at 2 revisions (3 total design attempts)
11. The system shall track previous design approaches in ticket metadata and prevent oscillation (returning to a previously-failed approach)

### Non-Functional Requirements
- **Backward Compatibility**: The common case (no revision needed) must be completely unchanged. The new status and logic only activate when escalation triggers fire.
- **Traceability**: All revision artifacts (findings, revised design, revision snapshots) must be durable and committed to the repo.

## Constraints
- No new agent files — diagnosis is an extended responsibility of existing agents (implementer, quality gate, reviewer)
- The revision path should be usable from any post-design phase (prototype findings could also trigger it in future, though this ticket focuses on implementation)
- Must not break existing ticket workflows for in-progress tickets

## Acceptance Criteria
- [ ] New `implementation-findings.md.template` exists in `.claude/templates/`
- [ ] Ticket template has "Implementation Blocked — Design Revision Needed" status checkbox
- [ ] Ticket template has revision tracking metadata (Design Revision Count, Previous Design Approaches)
- [ ] Ticket template has Design Revision Phase in Workflow Log and Feedback sections
- [ ] Implementer agent circle detection produces `implementation-findings.md` instead of generic escalation
- [ ] Quality gate agent 3rd failure adds Design Revision Recommendation to report
- [ ] Implementation reviewer agent 3rd escalation produces `implementation-findings.md`
- [ ] Designer agent has Mode 3 with oscillation guard and warm-start guidance
- [ ] Design reviewer has revision-aware review context
- [ ] Orchestrator has Design Revision Loop with human gate, revision tracking, and oscillation prevention
- [ ] Walking through the 0075 scenario on paper follows the new path correctly

---

## Design Decisions (Human Input)

### Preferred Approaches

- Implementer produces findings directly (no dedicated diagnosis agent) — the entity closest to the failure has the most context
- Design revision cap at 2 (3 total attempts) — beyond that, the problem needs fundamental re-scoping
- Delta design, not full redesign — preserve unaffected decisions, provide warm-start guidance
- `implementation-findings.md` is the structured escalation artifact — it replaces the generic "ask human"

### Things to Avoid

- Fully automated design revision without human approval
- Creating a new "diagnosis agent" — keep the responsibility with the agent that detected the failure
- Allowing unbounded revision cycles (oscillation between approaches)
- Breaking existing ticket workflows for in-progress tickets

### Open Questions

- Should the prototype phase be required after every design revision, or should the human decide per-revision?
- Should the `implementation-findings.md` template include a section for the implementer to suggest which implementation files can be preserved (warm-start hints)?
- Should the oscillation check be a hard block (STOP) or a soft warning (WARN but allow human override)?

---

## References

### Related Code
- `.claude/agents/workflow-orchestrator.md` — Central coordination, quality gate loop, revision handling
- `.claude/agents/cpp-implementer.md` — Circle detection, iteration logging
- `.claude/agents/code-quality-gate.md` — Maximum iterations escalation
- `.claude/agents/cpp-architect.md` — Operating modes (Mode 1: initial, Mode 2: reviewer revision)
- `.claude/agents/design-reviewer.md` — Review criteria, revision context
- `.claude/agents/implementation-reviewer.md` — Escalation path
- `.claude/templates/ticket.md.template` — Status checkboxes, metadata, workflow log

### Related Documentation
- `docs/designs/0075_unified_contact_constraint/` — Real-world example of ad-hoc design revision
- Ticket 0070: NLopt convergence energy injection — DD-0070-H2 mechanism that motivated the 0075 revision

### Related Tickets
- 0075_unified_contact_constraint — The ticket that demonstrated the need for this feature
- 0075b_block_pgs_solver — The sub-ticket where implementation stalled on a design flaw

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-02-26 00:00
- **Completed**: 2026-02-26 00:00
- **Branch**: 0079-design-revision-feedback-loop
- **PR**: N/A (draft PR pending push)
- **Artifacts**:
  - `docs/designs/0079_design_revision_feedback_loop/design.md`
  - `docs/designs/0079_design_revision_feedback_loop/0079_design_revision_feedback_loop.puml`
  - `.claude/templates/implementation-findings.md.template` (new template)
- **Notes**: Initial design complete. Design covers all 11 functional requirements from the ticket. Five design decisions captured (DD-0079-001 through DD-0079-005). Three open questions identified for human input: (1) prototype after revision — optional vs. required, (2) oscillation check — hard block vs. soft warning, (3) warm-start hints in findings template. Ticket modifies: cpp-implementer, code-quality-gate, implementation-reviewer, cpp-architect (Mode 3), design-reviewer (revision-aware), workflow-orchestrator (Design Revision Loop), ticket.md.template (new status + metadata + workflow log section). No new agent files created per constraint.

### Design Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
- **Files Modified**:
- **Artifacts**:
  - `docs/designs/0079_design_revision_feedback_loop/implementation-notes.md`
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

### Feedback on Tests
{Your comments on test coverage, test quality, or missing test scenarios}
