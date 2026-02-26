# Ticket 0078d: CLAUDE.md Coding Standards Migration

## Status
- [x] Draft
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Implementation
- [x] Implementation Complete
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Merged / Complete

**Current Phase**: Merged / Complete
**Type**: Tooling / Infrastructure
**Priority**: Low
**Created**: 2026-02-26
**Generate Tutorial**: No
**Parent**: 0078_cpp_guidelines_mcp_server

---

## Summary

Migrate the "Coding Standards" section of `CLAUDE.md` from inline prose to a directive that points agents to the guidelines MCP server as the canonical source of truth. This implements DD-0078-004 (Source of truth migration).

---

## Problem

With the guidelines database live (0078), coding standards exist in two places:

1. **`CLAUDE.md`** — Prose descriptions with code examples (current canonical source)
2. **Guidelines DB** — Structured rules with rationale, tags, cross-refs, and FTS search

This split-brain creates risks:
- Rules updated in one location may not be updated in the other
- Agents may get conflicting information from the two sources
- New rules added to the DB won't appear in CLAUDE.md and vice versa

---

## Solution

### Replace Coding Standards Section

Replace the current "Coding Standards" section in `CLAUDE.md` (which contains inline examples for initialization, memory management, naming conventions, etc.) with a concise directive:

```markdown
## Coding Standards

Coding standards are managed in the **Guidelines MCP Server** (`search_guidelines`, `get_rule`, `list_categories`).
Query the server for authoritative rules, rationale, and examples. See the Guidelines MCP Server section under Code Quality for setup.

Key categories: Initialization, Resource Management, Naming Conventions, Function Design, Code Organization, Documentation.
```

### What to Preserve in CLAUDE.md

- **General naming conventions** (PascalCase, camelCase, etc.) — These are quick-reference conventions that benefit from being inline
- **Code organization** ("One class per header", etc.) — Brief structural guidance
- **Guidelines MCP Server** section under Code Quality — Already documents the DB

### What to Remove from CLAUDE.md

- Detailed code examples for each coding standard (brace initialization, NaN initialization, Rule of Five, memory management, etc.)
- These are fully captured in the guidelines DB with rule IDs, rationale, and examples

---

## Implementation Steps

1. Verify all CLAUDE.md coding standards are captured in the guidelines DB (cross-check each rule)
2. Replace the detailed coding standards section with a pointer directive
3. Keep general naming conventions and code organization as brief inline references
4. Verify agents can still discover all rules via `search_guidelines`
5. Update any documentation that references CLAUDE.md coding standards to also mention the guidelines DB

---

## Acceptance Criteria

- [x] Every coding standard currently in CLAUDE.md has a corresponding rule in the guidelines DB
- [x] CLAUDE.md coding standards section replaced with directive to query guidelines MCP server
- [x] General naming conventions (PascalCase, camelCase, etc.) remain inline in CLAUDE.md
- [x] Code organization brief guidance remains inline in CLAUDE.md
- [x] `search_guidelines` returns all rules previously described in CLAUDE.md prose
- [x] No information loss — all rationale, examples, and guidance preserved in guidelines DB

---

## Design Decisions

### DD-0078-004 (from parent ticket): Source of truth migration
**Rationale**: Once the guidelines DB is active, `CLAUDE.md` coding standards section becomes a pointer to the DB rather than the canonical source. This avoids split-brain between prose and structured data.

---

## Dependencies

- **Requires**: 0078_cpp_guidelines_mcp_server (Complete)
- **Should follow**: 0078a (agents can query guidelines), 0078b (CppCore rules populated)
- **Blocked By**: None (0078 is complete, but best done after 0078a/0078b)

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-02-26 00:00
- **Completed**: 2026-02-26 00:00
- **Branch**: 0078d-claude-md-guidelines-migration
- **PR**: N/A
- **Artifacts**:
  - `CLAUDE.md` — Coding Standards section migrated
- **Notes**:
  Cross-checked all 7 CLAUDE.md coding standards against guidelines DB YAML seed files.
  Confirmed full coverage: MSD-INIT-001, MSD-INIT-002, MSD-RES-001, MSD-RES-002, MSD-RES-003,
  MSD-NAME-002, MSD-FUNC-001 all present in `scripts/guidelines/data/project_rules.yaml`
  with rationale, examples, and tags. Removed ~250 lines of inline prose and code examples
  from CLAUDE.md. Replaced with 9-line directive block pointing agents to the Guidelines MCP
  Server. Preserved General Naming Conventions table, Code Organization bullets, and
  Documentation bullets inline as specified. No design/review phase needed — pure
  documentation change implementing DD-0078-004.
