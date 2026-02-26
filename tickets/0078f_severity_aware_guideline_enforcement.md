# Ticket 0078f: Severity-Aware Guideline Enforcement

## Status
- [x] Draft
- [-] Design Complete — Awaiting Review (skipped — prompt-only change)
- [-] Design Approved — Ready for Implementation (skipped — prompt-only change)
- [x] Implementation Complete
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Implementation Complete
**Type**: Tooling / Infrastructure
**Priority**: High
**Created**: 2026-02-26
**Generate Tutorial**: No
**Parent**: 0078_cpp_guidelines_mcp_server

---

## Summary

Update agent prompts to enforce severity-aware guideline compliance. Violations of `severity: required` rules must be BLOCKING findings in reviews and must-fix items in designs. Currently agents query guidelines but treat all rules equally regardless of severity, which means a required rule violation could be downgraded to a MINOR or NIT finding.

---

## Problem

The guidelines database has 92 active rules with `severity: required`, 16 `recommended`, and 8 `advisory`. However:

1. **No severity mapping** — Agent prompts (0078a) direct agents to query `search_guidelines` and cite rule IDs, but don't instruct them to check rule severity or map it to finding severity
2. **Required rules can be downgraded** — A reviewer could flag a `severity: required` violation as MINOR or NIT, allowing it to pass review
3. **No systematic compliance check** — No agent step queries all required rules for relevant categories and verifies the design/implementation complies with each one
4. **Architects unaware of mandatory constraints** — The cpp-architect doesn't filter by severity when discovering rules, so required rules have no special weight during design

The `search_guidelines` tool already supports a `severity` filter parameter — the infrastructure exists, but agents don't use it.

---

## Solution

### 1. Severity-to-Finding Mapping Directive

Add to all four agent prompts (cpp-architect, design-reviewer, cpp-code-reviewer, implementation-reviewer):

```
### Severity Enforcement Policy

Guidelines have three severity levels. Map them to finding severity as follows:

| Guideline Severity | Minimum Finding Severity | Review Impact |
|--------------------|-----------------------------|--------------|
| `required`         | BLOCKING                     | Cannot approve with open violations |
| `recommended`      | MAJOR                        | Should fix before merge; document if deferred |
| `advisory`         | MINOR or NIT                 | Discretionary; cite for awareness |

When citing a rule, always include its severity. Example:
"Violates MSD-INIT-001 (required): Use NaN for uninitialized floating-point members → BLOCKING"
```

### 2. Required-Rules Compliance Check Step

Add a systematic compliance step to the **design-reviewer** and **implementation-reviewer** agents:

```
### Required Rules Compliance Check

Before finalizing your review verdict:
1. Identify the categories relevant to this design/implementation
2. For each relevant category, query `get_category(name, detailed=True)`
3. Filter for rules with `severity: required`
4. Verify the design/implementation complies with each required rule
5. Any required-rule violation that is not addressed is a BLOCKING finding

This is a systematic sweep — do not rely only on pattern-matched `search_guidelines` queries.
```

### 3. Architect Required-Rules Discovery

Add to **cpp-architect**:

```
### Required Rules Discovery

When starting a design:
1. Identify categories relevant to your design (e.g., Resource Management, Initialization)
2. Query `search_guidelines` with `severity=required` for each relevant category
3. List the required rules as design constraints in the "Constraints" section of the design document
4. Ensure the proposed design satisfies all listed required rules
```

### 4. Code Reviewer Severity-Filtered Query

Add to **cpp-code-reviewer**:

```
### Required Rules Sweep

After your pattern-based review, perform a targeted sweep:
1. Query `search_guidelines(query="", severity="required")` for the categories touched by the diff
2. For each required rule in those categories, check if the diff introduces a violation
3. Any new violation of a required rule is a BLOCKING finding
```

---

## Affected Agents

| Agent | Changes |
|-------|---------|
| `cpp-architect` | Add severity mapping table + required-rules discovery step |
| `design-reviewer` | Add severity mapping table + required-rules compliance check |
| `cpp-code-reviewer` | Add severity mapping table + required-rules sweep step |
| `implementation-reviewer` | Add severity mapping table + required-rules compliance check |

---

## Implementation Steps

1. Add severity-to-finding mapping table to all four agent prompts
2. Add required-rules compliance check to design-reviewer (in review process)
3. Add required-rules compliance check to implementation-reviewer (before verdict)
4. Add required-rules discovery step to cpp-architect (at design start)
5. Add required-rules sweep to cpp-code-reviewer (after pattern-based review)
6. Smoke test: invoke cpp-code-reviewer on a file and verify it queries with severity filter and maps required violations to BLOCKING

---

## Acceptance Criteria

- [x] All four agent prompts include the severity-to-finding mapping table
- [x] design-reviewer includes systematic required-rules compliance check before verdict
- [x] implementation-reviewer includes systematic required-rules compliance check before verdict
- [x] cpp-architect includes required-rules discovery step at design start
- [x] cpp-code-reviewer includes required-rules sweep after pattern-based review
- [x] Severity is included when citing rules (e.g., "MSD-INIT-001 (required)")
- [ ] Smoke test confirms cpp-code-reviewer maps a required-rule violation to BLOCKING

---

## Dependencies

- **Requires**: 0078a_agent_prompt_guidelines_integration (Complete)
- **Requires**: 0078_cpp_guidelines_mcp_server (Complete)
- **Blocked By**: None

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-02-26 00:00
- **Completed**: 2026-02-26 00:00
- **Branch**: 0078f-severity-aware-guideline-enforcement
- **PR**: N/A
- **Artifacts**:
  - `.claude/agents/cpp-architect.md`
  - `.claude/agents/design-reviewer.md`
  - `.claude/agents/cpp-code-reviewer.md`
  - `.claude/agents/implementation-reviewer.md`
- **Notes**: Skipped design/review phases — this ticket modifies only agent prompt markdown files. All four agents updated with: (1) Severity Enforcement Policy table mapping required→BLOCKING, recommended→MAJOR, advisory→MINOR/NIT; (2) agent-specific required-rules steps (Required Rules Discovery for cpp-architect, Required Rules Compliance Check for design-reviewer and implementation-reviewer, Required Rules Sweep for cpp-code-reviewer). Steps renumbered in design-reviewer (old Step 5 became Step 6 after inserting Required Rules Compliance Check as Step 5).
