# Feature Implementation Workflow

## Overview

This workflow guides the implementation of **architectural C++ changes**—new classes, interfaces, and their interactions with existing code.

**Scope**: Use this workflow for changes that:
- Add new classes that interact with existing libraries
- Introduce new abstraction layers
- Create new interfaces between components
- Add new subsystems or modules

**Do NOT use** for: bug fixes, minor refactors, single-class changes.

---

## Workflow Phases

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                                                                             │
│  ┌──────────┐     ┌────────────────┐     ┌────────────┐                    │
│  │  DESIGN  │────▶│ DESIGN REVIEW  │────▶│ PROTOTYPE  │                    │
│  └──────────┘     └────────────────┘     └────────────┘                    │
│       │                   │                    │                            │
│       │ Human             │ Human              │ Human                      │
│       │ Review            │ Review             │ Review                     │
│       ▼                   ▼                    ▼                            │
│  [Approve/Edit]     [Approve/Edit]       [Approve/Edit]                    │
│                                                                             │
│                      ┌─────────────┐     ┌────────────────────┐            │
│                      │ IMPLEMENT   │────▶│ IMPLEMENTATION     │            │
│                      │             │     │ REVIEW             │            │
│                      └─────────────┘     └────────────────────┘            │
│                            │                    │                           │
│                            │ Human              │ Human                     │
│                            │ Review             │ Review                    │
│                            ▼                    ▼                           │
│                       [Approve/Edit]       [Approve/Merge]                 │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

Each phase produces artifacts that the human operator reviews before the next phase begins.

---

## Phase Definitions

### Phase 1: Design
**Agent**: `.claude/agents/designer.md`  
**Input**: Feature requirements from user  
**Output**: 
- `docs/designs/{feature-name}/design.md`
- `docs/designs/{feature-name}/{feature-name}.puml`

**Human Gate**: 
- Review design document and PlantUML
- Answer Open Questions (design decisions, clarifications)
- Edit artifacts if needed
- Signal approval to proceed to Design Review

**Revision Loop**:
- Human edits design → Re-run Design phase to validate changes
- OR Human approves → Proceed to Design Review

---

### Phase 2: Design Review
**Agent**: `.claude/agents/design-reviewer.md`  
**Input**: Design artifacts + human decisions on open questions  
**Output**: Review appended to `docs/designs/{feature-name}/design.md`

**Human Gate**:
- Review the Design Review assessment
- For APPROVED/APPROVED WITH NOTES: Review prototype guidance
- For NEEDS REVISION: Review required changes
- For BLOCKED: Resolve blocking issues

**Revision Loop**:
- If NEEDS REVISION: Return to Phase 1 with revision guidance
- If BLOCKED: Human resolves blockers, then re-review
- If APPROVED: Proceed to Prototype (or Implementation if no prototypes needed)

---

### Phase 3: Prototype
**Agent**: `.claude/agents/prototyper.md`  
**Input**: Design with review + prototype guidance  
**Output**: 
- `docs/designs/{feature-name}/prototype-results.md`
- `prototypes/{feature-name}/` (prototype code)

**Human Gate**:
- Review prototype results
- Verify success criteria evaluations
- Review implementation ticket
- Approve or request changes

**Revision Loop**:
- If prototype INVALIDATED: Return to Phase 1 with findings
- If PARTIAL: Human decides whether to proceed or revise design
- If VALIDATED: Proceed to Implementation

---

### Phase 4: Implementation
**Agent**: `.claude/agents/implementer.md`  
**Input**: Design + prototype results + implementation ticket  
**Output**: 
- Implemented code in codebase
- `docs/designs/{feature-name}/implementation-notes.md`

**Human Gate**:
- Review implementation notes
- Spot-check code changes
- Verify tests pass
- Approve or request changes

**Revision Loop**:
- Human identifies issues → Return to Phase 4 with feedback
- Human approves → Proceed to Implementation Review

---

### Phase 5: Implementation Review
**Agent**: `.claude/agents/impl-reviewer.md`  
**Input**: All previous artifacts + implemented code  
**Output**: `docs/designs/{feature-name}/implementation-review.md`

**Human Gate**:
- Review the implementation review
- Final approval for merge

**Revision Loop**:
- If CHANGES REQUESTED: Return to Phase 4 with required changes
- If BLOCKED: May need to return to earlier phases
- If APPROVED: Feature complete, ready to merge

---

## Orchestration Commands

To run each phase, use the Task tool with the appropriate agent:

```
Phase 1 - Design:
Task: Read .claude/agents/designer.md and execute design phase
Input: Feature "{name}": {description}
Output: Design artifacts in docs/designs/{name}/

Phase 2 - Design Review:
Task: Read .claude/agents/design-reviewer.md and execute design review
Input: Design document at docs/designs/{name}/design.md
       Human decisions: {any answers to open questions}
Output: Review appended to design document

Phase 3 - Prototype:
Task: Read .claude/agents/prototyper.md and execute prototyping
Input: Design document with review at docs/designs/{name}/design.md
Output: Prototype results at docs/designs/{name}/prototype-results.md
        Prototype code at prototypes/{name}/

Phase 4 - Implementation:
Task: Read .claude/agents/implementer.md and execute implementation
Input: Design document at docs/designs/{name}/design.md
       Prototype results at docs/designs/{name}/prototype-results.md
Output: Code changes in codebase
        Implementation notes at docs/designs/{name}/implementation-notes.md

Phase 5 - Implementation Review:
Task: Read .claude/agents/impl-reviewer.md and execute review
Input: All artifacts in docs/designs/{name}/
       Implemented code
Output: Implementation review at docs/designs/{name}/implementation-review.md
```

---

## Context Passing Between Phases

Each phase MUST read and reference artifacts from previous phases:

| Phase | Must Read | May Reference |
|-------|-----------|---------------|
| Design | Requirements | Existing architecture, .puml files |
| Design Review | Design document, .puml | Codebase for validation |
| Prototype | Design document with review | Codebase for types/interfaces |
| Implementation | Design, Review, Prototype results | Full codebase |
| Impl Review | ALL previous artifacts | Full codebase |

---

## Human Intervention Points

### Modifying Artifacts

At any human gate, you can:

1. **Edit design artifacts directly** → Next phase will pick up changes
2. **Add annotations** → Add `## Human Notes` section to any artifact
3. **Answer open questions** → Add answers to the design document
4. **Request re-run** → Ask agent to re-execute with new information

### Feedback Format

When providing feedback to resume a phase:

```markdown
## Human Feedback for {Phase}

### Decisions Made
- {Open question}: {Decision and rationale}

### Requested Changes
- {Specific change to make}

### Additional Context
- {Any context not captured in artifacts}
```

---

## Error Recovery

### Design Phase Failures
- Designer cannot proceed due to unclear requirements → Provide clarification
- Existing architecture unclear → Point to relevant code/docs

### Design Review Failures
- BLOCKED status → Resolve blocking issue, re-run review
- NEEDS REVISION → Update design, re-run review

### Prototype Failures
- INVALIDATED → Review findings, revise design, re-run from Design
- Time box exceeded → Decide: extend, simplify, or accept risk

### Implementation Failures
- Build failures → Debug with implementer context
- Test failures → Determine if bug or design issue
- Design deviation needed → Approve or revise design

### Review Failures
- CHANGES REQUESTED → Make changes, re-run review
- Repeated failures (3x) → Escalate, may need design revision

---

## Artifacts Summary

After successful completion, the following artifacts exist:

```
docs/designs/{feature-name}/
├── design.md                 # Original design + design review
├── {feature-name}.puml       # Architecture diagram
├── prototype-results.md      # Prototype findings + impl ticket
├── implementation-notes.md   # What was implemented
└── implementation-review.md  # Final review

prototypes/{feature-name}/    # Prototype code (may delete after)
├── p1_{name}/
└── p2_{name}/

{codebase}/                   # Actual implementation
├── include/{path}/           # New headers
├── src/{path}/               # New sources
└── test/{path}/              # New and updated tests
```

---

## Quick Reference

| What You Want | Command |
|---------------|---------|
| Start new feature | Run Design phase with requirements |
| Revise design | Edit artifacts, re-run Design or Design Review |
| Skip prototypes | If Design Review has no prototypes, go straight to Implementation |
| Fix implementation issue | Edit code or provide feedback, re-run Implementation |
| Complete feature | Get APPROVED from Implementation Review |