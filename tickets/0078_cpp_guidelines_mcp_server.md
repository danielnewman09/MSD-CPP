# Ticket 0078: C++ Guidelines MCP Server

## Status
- [x] Draft
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Implementation
- [x] Implementation Complete
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Approved — Ready for Documentation Update
**Type**: Tooling / Infrastructure
**Priority**: Medium
**Created**: 2026-02-26
**Generate Tutorial**: No

---

## Summary

Build a FastMCP server backed by SQLite + FTS5 that stores C++ coding guidelines from three sources: C++ Core Guidelines, MISRA, and project-specific conventions. The AI assistant queries it during design and code review to provide rationale-backed recommendations with traceable rule IDs.

This is a lightweight RAG system where FTS5 handles retrieval, MCP handles augmentation, and the AI handles generation.

---

## Problem

C++ coding standards are currently embedded in `CLAUDE.md` as free-text prose. This has limitations:

1. **No structured query** — Cannot search for rules by category, severity, or source
2. **No cross-referencing** — Cannot link project conventions to the C++ Core Guidelines or MISRA rules they derive from
3. **No rationale retrieval** — Agents can't cite specific rule IDs when flagging violations
4. **No incremental growth** — Adding CppCoreGuidelines or MISRA rules means bloating CLAUDE.md

---

## Solution

### Architecture

```
YAML seed files → seed_guidelines.py → guidelines.db (SQLite + FTS5)
                                              ↓
                                    guidelines_server.py (FastMCP)
                                              ↓
                                    MCP tools (search, get_rule, etc.)
                                              ↓
                                    AI agents (architect, reviewer, etc.)
```

### New Files

```
scripts/guidelines/
├── guidelines_schema.py          # Schema module (CREATE TABLE + FTS5)
├── guidelines_server.py          # FastMCP server + CLI
├── seed_guidelines.py            # YAML → SQLite indexer
└── data/
    ├── project_rules.yaml        # MSD conventions extracted from CLAUDE.md
    ├── cpp_core_guidelines.yaml  # Stub for incremental population
    └── misra_rules.yaml          # Stub for incremental population
```

### Schema

**Tables:**
- `categories` — Broad groupings (Memory Management, Initialization, etc.)
- `rules` — Individual guidelines with rule_id, source, severity, status, title, rationale, enforcement_notes, enforcement_check, good_example, bad_example
- `rule_cross_refs` — Relationships between rules (derived_from, related, supersedes, conflicts_with)
- `tags` — Cross-cutting concern labels (memory, ownership, safety, performance)
- `rule_tags` — Many-to-many join
- `rules_fts` — FTS5 virtual table over title + rationale + enforcement_notes + examples

**Rule ID conventions:**
- Project: `MSD-{CATEGORY}-{NNN}` (e.g., MSD-INIT-001)
- C++ Core Guidelines: `CPP-{section}.{number}` (e.g., CPP-R.11)
- MISRA: `MISRA-{rule}` (e.g., MISRA-6.2)

### MCP Tools

| Tool | Signature | Description |
|------|-----------|-------------|
| `search_guidelines` | `(query, source?, category?, severity?, limit?)` | FTS search with optional filters |
| `get_rule` | `(rule_id)` | Full rule details + cross-refs + tags (includes related rules) |
| `list_categories` | `()` | All categories with rule counts |
| `get_category` | `(name, detailed?)` | Rules in a category; `detailed=false` (default) returns rule_id + title only, `detailed=true` includes full rationale and examples |
| `get_rules_by_tag` | `(tag)` | All rules with a given tag (e.g., "safety", "ownership") |

### Initial Seed Data (from CLAUDE.md)

| Rule ID | Category | Title |
|---------|----------|-------|
| MSD-INIT-001 | Initialization | Use NaN for uninitialized floating-point members |
| MSD-INIT-002 | Initialization | Always use brace initialization |
| MSD-RES-001 | Resource Management | All-or-Nothing Rule (Rule of Five/Zero) |
| MSD-RES-002 | Resource Management | unique_ptr for ownership, references for non-owning |
| MSD-RES-003 | Resource Management | Optional reference wrapper only for truly optional lookups |
| MSD-NAME-001 | Naming Conventions | Follow project naming conventions |
| MSD-NAME-002 | Naming Conventions | No misleading `cached` prefix |
| MSD-FUNC-001 | Function Design | Prefer return values over output parameters |
| MSD-ORG-001 | Code Organization | One class per header |
| MSD-DOC-001 | Documentation | Public APIs require Doxygen comments |

---

## Design Decisions

### DD-0078-001: Merge `get_related_rules` into `get_rule`
**Rationale**: `get_rule` already returns cross-refs and tags. A separate `get_related_rules` tool adds a round-trip for information that `get_rule` can return in one call. Replaced with `get_rules_by_tag` which queries an axis that no other tool covers.

### DD-0078-002: Add `status` column to rules
**Rationale**: Rules have lifecycles (proposed → active → deprecated). Without status, deprecated rules would still be returned and enforced. Agents need to distinguish current rules from historical ones.

### DD-0078-003: Add `enforcement_check` column to rules
**Rationale**: Many rules map to clang-tidy or cppcheck IDs (e.g., `cppcoreguidelines-special-member-functions`). Storing this mapping enables future tooling: given a linter warning, look up the project rule and rationale. Initially nullable — populated incrementally.

### DD-0078-004: Source of truth migration
**Rationale**: Once the guidelines DB is active, `CLAUDE.md` coding standards section becomes a pointer to the DB rather than the canonical source. This avoids split-brain between prose and structured data. The `CLAUDE.md` section will be replaced with a directive to query the guidelines MCP server. (Tracked in follow-up 0078d.)

### DD-0078-005: Default `get_category` to summary mode
**Rationale**: CppCoreGuidelines categories can contain 100+ rules. Returning full rationale + examples for all of them would blow out the AI context window. Default to returning `{rule_id, title, severity, status}` only; `detailed=true` includes everything.

---

## Implementation Steps

### Phase 1: Schema + Indexer + Seed Data
1. Create `scripts/guidelines/guidelines_schema.py` (includes `status` and `enforcement_check` columns)
2. Create `scripts/guidelines/seed_guidelines.py` with pydantic validation of YAML structure
3. Create `scripts/guidelines/data/project_rules.yaml` (10 rules from CLAUDE.md)
4. Create stub files: `cpp_core_guidelines.yaml`, `misra_rules.yaml`
5. Add `pyyaml` to `python/requirements.txt`

### Phase 2: MCP Server
6. Create `scripts/guidelines/guidelines_server.py` with 5 tools + CLI mode (`search_guidelines`, `get_rule`, `list_categories`, `get_category`, `get_rules_by_tag`)
7. Follow pattern from `scripts/mcp_codebase_server.py` (GuidelinesServer class + create_mcp_server wrapper)

### Phase 3: Registration + Build
8. Add `guidelines` entry to `.mcp.json`
9. Add to `.claude/settings.local.json` (enabledMcpjsonServers + permissions)
10. Add `guidelines-seed` target to `CMakeLists.txt`
11. Add `debug-guidelines` preset to `CMakeUserPresets.json`

### Phase 4: Verification
12. Seed the database, run CLI smoke tests
13. Restart session, verify MCP tools work

---

## Acceptance Criteria

- [ ] `search_guidelines("brace initialization")` returns MSD-INIT-002 with rationale and examples
- [ ] `get_rule("MSD-RES-001")` returns full rule with category, tags, cross-refs, and status
- [ ] `list_categories()` returns all categories with correct rule counts
- [ ] `get_category("Initialization")` returns summary (rule_id + title) by default
- [ ] `get_category("Initialization", detailed=True)` returns full rationale and examples
- [ ] `get_rules_by_tag("safety")` returns all rules tagged with "safety"
- [ ] CLI mode works for all 5 commands
- [ ] Database is fully rebuildable from YAML seed files
- [ ] FTS5 supports porter stemming (e.g., "initialize" matches "initialization")
- [ ] Seed script validates YAML structure before insertion (pydantic)

---

## Follow-Up Work

- **0078a**: Update agent prompts (cpp-architect, design-reviewer, cpp-code-reviewer, implementation-reviewer) to reference guidelines MCP tools. Include directive: "Only cite rules returned by `search_guidelines`. Do not invent rule IDs."
- **0078b**: Populate C++ Core Guidelines rules (start with R, C, ES sections). Include `enforcement_check` mappings to clang-tidy where applicable.
- **0078c**: Populate MISRA rules (start with memory and initialization categories)
- **0078d**: Migrate CLAUDE.md coding standards — replace prose with directive to query guidelines MCP server (DD-0078-004). Keep CLAUDE.md as a pointer, not the canonical source.

---

## Workflow Log

### Design Phase
- **Started**: 2026-02-26 00:00
- **Completed**: 2026-02-26 00:00
- **Branch**: `0078-cpp-guidelines-mcp-server`
- **PR**: #98 (draft)
- **Artifacts**:
  - `docs/designs/0078_cpp_guidelines_mcp_server/design.md`
  - `docs/designs/0078_cpp_guidelines_mcp_server/0078_cpp_guidelines_mcp_server.puml`
- **Notes**: Ticket contained a complete design specification. Design documents formalize it into the standard format with full schema detail, component interfaces, integration points, and phased implementation order. All 5 DDs (DD-0078-001 through DD-0078-005) are documented in the design. No open questions — all resolved upfront in the ticket.

### Design Review Phase
- **Started**: 2026-02-26 00:00
- **Completed**: 2026-02-26 00:00
- **Branch**: `0078-cpp-guidelines-mcp-server`
- **PR**: #98 (draft)
- **Artifacts**:
  - `docs/designs/0078_cpp_guidelines_mcp_server/design.md` (review appended)
- **Notes**: Design approved with 4 non-blocking notes. Status: APPROVED WITH NOTES. Key notes for implementer: (N1) define `get_rule` not-found contract in tool docstring; (N2) add `pyyaml` with pinned version to `python/requirements.txt`; (N3) no automated pytest required for this ticket; (N4) mark `enforcement_check` as optional in YAML schema with explanatory comment. No risks require prototype validation. Proceed directly to implementation.

### Implementation Phase
- **Started**: 2026-02-26 16:00
- **Completed**: 2026-02-26 17:00
- **Branch**: `0078-cpp-guidelines-mcp-server`
- **PR**: #98 (ready for review)
- **Artifacts**:
  - `scripts/guidelines/guidelines_schema.py`
  - `scripts/guidelines/seed_guidelines.py`
  - `scripts/guidelines/guidelines_server.py`
  - `scripts/guidelines/data/project_rules.yaml`
  - `scripts/guidelines/data/cpp_core_guidelines.yaml`
  - `scripts/guidelines/data/misra_rules.yaml`
  - `docs/designs/0078_cpp_guidelines_mcp_server/implementation-notes.md`
  - `docs/designs/0078_cpp_guidelines_mcp_server/iteration-log.md`
- **Notes**: All 4 implementation phases completed in a single iteration. All 8 acceptance criteria pass (CLI smoke tests). All 4 design review notes (N1–N4) incorporated. No deviations from design. FTS5 porter stemming verified. CMake target `guidelines-seed` and preset `debug-guidelines` added. Server registered in `.mcp.json` and enabled in `settings.local.json`. PR #98 marked ready for review.

### Quality Gate Phase
- **Started**: 2026-02-26 18:00
- **Completed**: 2026-02-26 18:10
- **Branch**: `0078-cpp-guidelines-mcp-server`
- **PR**: #98 (ready for review)
- **Artifacts**:
  - `docs/designs/0078_cpp_guidelines_mcp_server/quality-gate-report.md`
- **Notes**: All gates passed. Gate 1 (Build): Release build clean with 0 warnings in ticket-modified files. Gate 2 (Tests): 811/812 pass; 1 pre-existing failure in WarmStart.TwoCubes_NoInstability attributable to 0075b branch (0 changes in msd/ from 0078). Gate 3 (clang-tidy): N/A — Python-only ticket; Python syntax validated. Gate 4 (Benchmarks): N/A — no benchmarks in design. Gate 5 (Python): All 10 acceptance criteria verified via CLI smoke tests including FTS5 porter stemming and not-found contract. Overall: PASSED.

### Implementation Review Phase
- **Started**: 2026-02-26 18:10
- **Completed**: 2026-02-26 18:20
- **Branch**: `0078-cpp-guidelines-mcp-server`
- **PR**: #98 (ready for review)
- **Artifacts**:
  - `docs/designs/0078_cpp_guidelines_mcp_server/implementation-review.md`
- **Notes**: APPROVED. All 5 MCP tools implemented correctly, all 5 DDs applied, all 4 design review notes (N1–N4) incorporated, all 10 acceptance criteria pass. No blocking issues. Two non-blocking minor suggestions (m1: add thread-safety comment to __init__; m2: replace type:ignore with explicit assert). Design conformance PASS, code quality PASS, test coverage PASS. Ready to merge after documentation update.
