# Ticket 0078a: Agent Prompt Guidelines Integration

## Status
- [x] Draft
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Implementation
- [x] Implementation Complete
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Implementation Complete
**Type**: Tooling / Infrastructure
**Priority**: Medium
**Created**: 2026-02-26
**Generate Tutorial**: No
**Parent**: 0078_cpp_guidelines_mcp_server

---

## Summary

Update agent prompts (cpp-architect, design-reviewer, cpp-code-reviewer, implementation-reviewer) to reference the guidelines MCP tools introduced in 0078. Agents should query `search_guidelines` during design and review workflows and cite rule IDs in their feedback.

---

## Problem

The guidelines MCP server (0078) is live, but no agent prompts reference it. Agents currently rely on CLAUDE.md prose for coding standards, which means:

1. **No structured citations** — Agents cannot cite specific rule IDs when flagging violations
2. **Inconsistent enforcement** — Each agent independently interprets CLAUDE.md prose
3. **Underutilized investment** — The guidelines DB exists but agents don't query it

---

## Solution

Add directives to the following agent prompts:

### Affected Agents

| Agent | Integration Point |
|-------|-------------------|
| `cpp-architect` | Query guidelines during design to ensure proposed architecture follows conventions |
| `design-reviewer` | Query guidelines when evaluating designs for standards compliance |
| `cpp-code-reviewer` | Query guidelines when reviewing code for violations |
| `implementation-reviewer` | Query guidelines when verifying implementation conformance |

### Directive Template

Each agent prompt should include a directive similar to:

```
When reviewing/designing C++ code, query the guidelines MCP server to retrieve applicable rules:
- Use `search_guidelines` to find rules relevant to the code patterns under review
- Cite specific rule IDs (e.g., MSD-INIT-001) when flagging violations or recommending patterns
- Only cite rules returned by `search_guidelines`. Do not invent rule IDs.
- Use `get_rule` to retrieve full rationale when providing detailed feedback
```

### Key Constraint

**Only cite rules returned by `search_guidelines`. Do not invent rule IDs.** This prevents hallucinated rule references.

---

## Implementation Steps

1. Identify where agent prompts are defined (likely in Task tool agent type definitions or CLAUDE.md agent sections)
2. Add guidelines query directive to `cpp-architect` agent prompt
3. Add guidelines query directive to `design-reviewer` agent prompt
4. Add guidelines query directive to `cpp-code-reviewer` agent prompt
5. Add guidelines query directive to `implementation-reviewer` agent prompt
6. Verify each agent can successfully call `search_guidelines` and `get_rule`

---

## Acceptance Criteria

- [x] `cpp-architect` prompt includes directive to query guidelines during design
- [x] `design-reviewer` prompt includes directive to query guidelines during review
- [x] `cpp-code-reviewer` prompt includes directive to query guidelines during review
- [x] `implementation-reviewer` prompt includes directive to query guidelines during review
- [x] All directives include "Do not invent rule IDs" constraint
- [ ] Each agent successfully queries `search_guidelines` in a test interaction (manual verification)

---

## Dependencies

- **Requires**: 0078_cpp_guidelines_mcp_server (Complete)
- **Blocked By**: None (0078 is complete)

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-02-26 00:00
- **Completed**: 2026-02-26 00:00
- **Branch**: 0078a-agent-prompt-guidelines-integration
- **PR**: N/A (pending creation)
- **Artifacts**:
  - `.claude/agents/cpp-architect.md` — Added "Guidelines MCP Integration" section before "Coding Standards to Apply"
  - `.claude/agents/design-reviewer.md` — Embedded guidelines query directive in Step 1 of Review Process
  - `.claude/agents/cpp-code-reviewer.md` — Added "Guidelines MCP Integration" section and Step 1 query instruction
  - `.claude/agents/implementation-reviewer.md` — Added "Phase 2.5: Guidelines MCP Lookup" between Prototype Learning and Code Quality phases
- **Notes**: All four agents now include `search_guidelines`, `get_rule`, and the "Do not invent rule IDs" constraint. Directives are positioned at the natural entry point for each agent's workflow. The cpp-architect directive also recommends `list_categories`/`get_category` for breadth-first rule discovery at design start. The implementation-reviewer directive uses a numbered phase (2.5) so it integrates cleanly between the existing Phase 2 and Phase 3 without renumbering downstream phases.
