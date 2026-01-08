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
│  ┌──────────┐     ┌────────────────┐                                       │
│  │  DESIGN  │────▶│ DESIGN REVIEW  │◀─┐                                    │
│  └──────────┘     └────────────────┘  │                                    │
│       ▲                   │           │ Autonomous                         │
│       │                   │           │ Iteration (1x)                     │
│       │              ┌────┴────┐      │                                    │
│       │              │ Revision│──────┘                                    │
│       │              │ Needed? │                                           │
│       │              └────┬────┘                                           │
│       │                   │ No (or after 1 iteration)                      │
│       │                   ▼                                                │
│       │           ┌────────────┐                                           │
│       │           │ Human Gate │                                           │
│       │           └─────┬──────┘                                           │
│       │                 │                                                  │
│       └─────────────────┘ (Human requests changes)                         │
│                         │                                                  │
│                         ▼ (Human approves)                                 │
│                 ┌────────────┐                                             │
│                 │ PROTOTYPE  │                                             │
│                 └─────┬──────┘                                             │
│                       │ Human Review                                       │
│                       ▼                                                    │
│                 [Approve/Edit]                                             │
│                                                                             │
│       ┌─────────────┐     ┌────────────────────┐     ┌──────────────┐      │
│       │ IMPLEMENT   │────▶│ IMPLEMENTATION     │────▶│ DOCUMENTATION│      │
│       │             │     │ REVIEW             │     │ SYNC         │      │
│       └─────────────┘     └────────────────────┘     └──────────────┘      │
│            │                    │                          │               │
│            │ Human              │ Human                    │ Human         │
│            │ Review             │ Review                   │ Review        │
│            ▼                    ▼                          ▼               │
│       [Approve/Edit]       [Approve]                  [Approve/Merge]      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

Each phase produces artifacts that the human operator reviews before the next phase begins.

---

## Phase Definitions

### Phase 1: Design
**Agent**: `.claude/agents/cpp-architect.md`
**Input**: Feature requirements from user (or revision feedback from Design Review)
**Output**:
- `docs/designs/{feature-name}/design.md`
- `docs/designs/{feature-name}/{feature-name}.puml`

**Autonomous Flow**:
- Initial design created → Automatically proceeds to Design Review
- If revision requested → Architect updates design based on reviewer feedback
- Design document includes revision history showing changes made

---

### Phase 2: Design Review
**Agent**: `.claude/agents/design-reviewer.md`
**Input**: Design artifacts from Phase 1
**Output**: Review appended to `docs/designs/{feature-name}/design.md`

**Autonomous Iteration** (occurs once before human review):
1. Reviewer evaluates initial design
2. If issues found on first pass → Kicks back to Architect with specific feedback
3. Architect revises design and updates artifacts
4. Reviewer performs final review of revised design
5. Final review presented to human

**Human Gate** (after autonomous iteration):
- Review the complete design history:
  - Initial design
  - First review feedback
  - Architect's revisions (if any)
  - Final review assessment
- For APPROVED/APPROVED WITH NOTES: Review prototype guidance
- For NEEDS REVISION: Review required changes (beyond autonomous iteration)
- For BLOCKED: Resolve blocking issues

**Revision Loop** (human-initiated):
- If human requests changes: Return to Phase 1 with human feedback
- If BLOCKED: Human resolves blockers, then re-review
- If APPROVED: Proceed to Prototype (or Implementation if no prototypes needed)

---

### Phase 3: Prototype
**Agent**: `.claude/agents/cpp-prototyper.md`  
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
**Agent**: `.claude/agents/cpp-implementer.md`  
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
**Agent**: `.claude/agents/implementation-reviewer.md`
**Input**: All previous artifacts + implemented code
**Output**: `docs/designs/{feature-name}/implementation-review.md`

**Human Gate**:
- Review the implementation review
- Approve to proceed to Documentation Sync

**Revision Loop**:
- If CHANGES REQUESTED: Return to Phase 4 with required changes
- If BLOCKED: May need to return to earlier phases
- If APPROVED: Proceed to Documentation Sync

---

### Phase 6: Documentation Sync
**Agent**: `.claude/agents/docs-updater.md`
**Input**: All design artifacts + implemented code
**Output**:
- Updated `docs/msd/{library}/` diagrams and documentation
- Updated `CLAUDE.md` with new components and diagrams index
- `docs/designs/{feature-name}/doc-sync-summary.md`

**Human Gate**:
- Review documentation changes
- Verify diagrams accurately reflect implementation
- Verify CLAUDE.md updates are accurate
- Final approval for merge

**Revision Loop**:
- If changes needed: Edit and re-run Documentation Sync
- If approved: Feature workflow complete, ready to merge

---

## Orchestration Commands

To run each phase, use the Task tool with the appropriate agent:

```
Phase 1+2 - Design with Autonomous Review (combined):
Task: Execute design workflow with autonomous iteration
Input: Feature "{name}": {description}
Flow:
  1. cpp-architect creates initial design
  2. design-reviewer evaluates and provides feedback
  3. If revisions needed: cpp-architect updates design (1 iteration max)
  4. design-reviewer performs final review
Output: Complete design artifacts in docs/designs/{name}/ including:
        - design.md with revision history and final review
        - {name}.puml diagram

Phase 1 - Design (standalone, for human-requested revisions):
Task: Read .claude/agents/cpp-architect.md and execute design phase
Input: Feature "{name}": {description}
       OR Revision feedback: {human feedback}
Output: Design artifacts in docs/designs/{name}/

Phase 2 - Design Review (standalone, for re-review after human changes):
Task: Read .claude/agents/design-reviewer.md and execute design review
Input: Design document at docs/designs/{name}/design.md
       Human decisions: {any answers to open questions}
       Iteration count: {0 for fresh review, 1 if already iterated}
Output: Review appended to design document

Phase 3 - Prototype:
Task: Read .claude/agents/cpp-prototyper.md and execute prototyping
Input: Design document with review at docs/designs/{name}/design.md
Output: Prototype results at docs/designs/{name}/prototype-results.md
        Prototype code at prototypes/{name}/

Phase 4 - Implementation:
Task: Read .claude/agents/cpp-implementer.md and execute implementation
Input: Design document at docs/designs/{name}/design.md
       Prototype results at docs/designs/{name}/prototype-results.md
Output: Code changes in codebase
        Implementation notes at docs/designs/{name}/implementation-notes.md

Phase 5 - Implementation Review:
Task: Read .claude/agents/implementation-reviewer.md and execute review
Input: All artifacts in docs/designs/{name}/
       Implemented code
Output: Implementation review at docs/designs/{name}/implementation-review.md

Phase 6 - Documentation Sync:
Task: Read .claude/agents/docs-updater.md and execute documentation sync
Input: All artifacts in docs/designs/{name}/
       Implemented code
       Target library: {library} (e.g., msd-sim, msd-assets, msd-gui)
Output: Updated docs/msd/{library}/ diagrams
        Updated CLAUDE.md
        Doc sync summary at docs/designs/{name}/doc-sync-summary.md
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
| Doc Sync | ALL previous artifacts, implemented code | Existing docs/msd/{library}/ |

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

### Documentation Sync Failures
- Diagram conflicts with existing library docs → Merge manually or update design
- CLAUDE.md structure mismatch → Adjust to match existing format
- Missing library documentation → Create new library section in CLAUDE.md

---

## Artifacts Summary

After successful completion, the following artifacts exist:

```
docs/designs/{feature-name}/
├── design.md                 # Design with revision history + reviews
│                             #   - Initial design
│                             #   - First review (if revisions requested)
│                             #   - Revision notes (changes made by architect)
│                             #   - Final review
├── {feature-name}.puml       # Architecture diagram (updated through iterations)
├── prototype-results.md      # Prototype findings + impl ticket
├── implementation-notes.md   # What was implemented
├── implementation-review.md  # Final review
└── doc-sync-summary.md       # Documentation sync summary

docs/msd/{library}/           # Library-level documentation (updated by Doc Sync)
├── {library}-core.puml       # Updated/created core overview diagram
└── {component}.puml          # Updated/created component diagrams

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
| Sync library docs | Run Documentation Sync phase after Implementation Review |
| Complete feature | Get APPROVED from Documentation Sync |