# Phase Sub-Issue Body Templates

Body templates for each workflow phase sub-issue created by the `create-issue` skill.

## Design Phase

Title: `[Design] {ticket_title}`

```markdown
Design phase for **{ticket_title}** (`{ticket_id}`).

**Parent ticket**: #{main_issue_number}

## Deliverables

- [ ] Design document: `docs/designs/{ticket_name}/design.md`
- [ ] PlantUML diagram: `docs/designs/{ticket_name}/{ticket_name}.puml`
- [ ] Design review passed

## Scope

Produce an architectural design document covering:
- Component interfaces and class structure
- Integration points with existing code
- Error handling strategy
- Key design decisions and alternatives considered

## Agents

Use the `cpp-architect` agent for design creation and `design-reviewer` agent for review.

---

_Phase sub-issue for `tickets/{filename}`_
```

## Math Formulation Phase

Title: `[Math] {ticket_title}`

```markdown
Math formulation phase for **{ticket_title}** (`{ticket_id}`).

**Parent ticket**: #{main_issue_number}

## Deliverables

- [ ] Math formulation: `docs/designs/{ticket_name}/math-formulation.md`
- [ ] LaTeX equations with full derivations
- [ ] Numerical stability analysis
- [ ] Concrete numerical test examples with expected values
- [ ] Math review passed

## Scope

Establish the mathematical foundation before architectural design:
- Derive all equations from first principles
- Analyze numerical stability and edge cases
- Provide hand-computed test examples that map to GTest assertions

## Agents

Use the `math-designer` agent for formulation and `math-reviewer` agent for review.

---

_Phase sub-issue for `tickets/{filename}`_
```

## Prototype Phase

Title: `[Prototype] {ticket_title}`

```markdown
Prototype phase for **{ticket_title}** (`{ticket_id}`).

**Parent ticket**: #{main_issue_number}

## Deliverables

- [ ] Prototype code: `prototypes/{ticket_name}/`
- [ ] Prototype results: `docs/designs/{ticket_name}/prototype-results.md`
- [ ] Design assumptions validated or revised

## Scope

Validate design assumptions through isolated experiments:
- Test algorithms and data structures in isolation
- Verify performance characteristics
- Confirm API ergonomics and integration feasibility
- Document findings and any design revisions needed

## Agents

Use the `cpp-prototyper` agent for prototype creation and validation.

---

_Phase sub-issue for `tickets/{filename}`_
```

## Implementation Phase

Title: `[Implementation] {ticket_title}`

```markdown
Implementation phase for **{ticket_title}** (`{ticket_id}`).

**Parent ticket**: #{main_issue_number}

## Deliverables

- [ ] Production code written following design document
- [ ] Unit tests written and passing
- [ ] Implementation review passed
- [ ] Documentation updated (CLAUDE.md, diagrams)

## Scope

Translate the validated design into production-quality C++ code:
- Follow project coding standards from CLAUDE.md
- Apply learnings from prototype phase
- Write comprehensive tests matching the test plan
- Pass implementation review quality gate

## Agents

Use the `cpp-implementer` agent for implementation and `implementation-reviewer` for review.

---

_Phase sub-issue for `tickets/{filename}`_
```

---

# Investigation Phase Templates

## Investigate Phase

Title: `[Investigate] {ticket_title}`

```markdown
Investigate phase for **{ticket_title}** (`{ticket_id}`).

**Parent ticket**: #{main_issue_number}

## Deliverables

- [ ] Suspect code identified and reviewed
- [ ] Reproduction test case written (currently fails)
- [ ] Instrumentation added (logging, energy tracking, etc.)
- [ ] Initial evidence gathered

## Scope

Instrument the codebase and reproduce the issue:
- Review suspect files, headers, and existing tests
- Write a minimal test case that demonstrates the bug
- Add diagnostic instrumentation (logging, sanitizers, energy tracking)
- Gather initial evidence about when and where the issue occurs

## Agents

Use the `cpp-systematic-debugger` skill for guided investigation.

---

_Phase sub-issue for `tickets/{filename}`_
```

## Diagnose Phase

Title: `[Diagnose] {ticket_title}`

```markdown
Diagnose phase for **{ticket_title}** (`{ticket_id}`).

**Parent ticket**: #{main_issue_number}

## Deliverables

- [ ] Hypothesis space documented
- [ ] Diagnostic tests written and executed
- [ ] Hypotheses systematically eliminated with evidence
- [ ] Root cause identified and documented

## Scope

Systematic hypothesis elimination to identify root cause:
- Test each hypothesis with targeted diagnostic tests
- Document evidence for/against each hypothesis
- Narrow down to specific component and code path
- Confirm root cause with a definitive test

## Agents

Use the `cpp-systematic-debugger` skill for hypothesis tracking. Run diagnostic tests with sanitizers (ASan, UBSan) enabled.

---

_Phase sub-issue for `tickets/{filename}`_
```

## Fix Phase

Title: `[Fix] {ticket_title}`

```markdown
Fix phase for **{ticket_title}** (`{ticket_id}`).

**Parent ticket**: #{main_issue_number}

## Deliverables

- [ ] Fix implemented targeting confirmed root cause
- [ ] Reproduction test now passes
- [ ] No regressions in existing test suite

## Scope

Implement the fix based on diagnosed root cause:
- Write the minimal fix that addresses the root cause
- Verify the reproduction test now passes
- Run full test suite to check for regressions
- Document the fix with rationale

## Agents

Use the `cpp-implementer` agent if fix requires significant new code. Otherwise implement directly.

---

_Phase sub-issue for `tickets/{filename}`_
```

## Verify Phase

Title: `[Verify] {ticket_title}`

```markdown
Verify phase for **{ticket_title}** (`{ticket_id}`).

**Parent ticket**: #{main_issue_number}

## Deliverables

- [ ] Regression tests added to prevent recurrence
- [ ] Full test suite passes (no regressions)
- [ ] Sanitizer runs clean (ASan, UBSan)
- [ ] Debug report documented

## Scope

Validate the fix and prevent recurrence:
- Add regression tests for the specific bug and related edge cases
- Run full test suite including existing and new tests
- Run with sanitizers (AddressSanitizer, UndefinedBehaviorSanitizer)
- Write debug report documenting root cause, fix, and lessons learned

## Agents

Use the `cpp-code-reviewer` agent for fix review and `implementation-reviewer` for quality gate.

---

_Phase sub-issue for `tickets/{filename}`_
```

---

## Template Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `{ticket_id}` | Ticket number prefix | `0040b` |
| `{ticket_title}` | Human-readable title from ticket header | `Split Impulse Position Correction` |
| `{ticket_name}` | Full filename stem (for paths) | `0040b_split_impulse_position_correction` |
| `{main_issue_number}` | GitHub issue number of the parent work ticket | `42` |
| `{filename}` | Full filename with extension | `0040b_split_impulse_position_correction.md` |

## Notes

- Phase sub-issues are intentionally lightweight. The detailed requirements live in the main ticket issue.
- Each phase sub-issue links back to the main ticket issue via `**Parent ticket**: #{N}`.
- The deliverable checklists serve as completion criteria — close the phase issue when all boxes are checked.
- Agent references help the user (or workflow orchestrator) know which specialized agent to invoke for each phase.
