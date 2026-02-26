# Implementation Notes: 0078 C++ Guidelines MCP Server

## Summary

Implemented a FastMCP server backed by SQLite + FTS5 that stores C++ coding guidelines
from structured YAML seed files. The server exposes 5 MCP tools for AI agents to query
guidelines with rationale-backed recommendations and traceable rule IDs.

All 4 implementation phases from the design document were completed in a single session
with no deviations from the approved design.

---

## Files Created

| File | Purpose | LOC |
|------|---------|-----|
| `scripts/guidelines/guidelines_schema.py` | SQLite schema DDL — CREATE/DROP statements, `create_schema()` and `drop_schema()` functions | 120 |
| `scripts/guidelines/seed_guidelines.py` | YAML → SQLite pipeline with Pydantic validation; fully idempotent | 195 |
| `scripts/guidelines/guidelines_server.py` | FastMCP server with 5 tools + CLI smoke-test mode | 290 |
| `scripts/guidelines/data/project_rules.yaml` | 10 MSD project rules from CLAUDE.md | 230 |
| `scripts/guidelines/data/cpp_core_guidelines.yaml` | Stub for 0078b (empty, correct structure) | 25 |
| `scripts/guidelines/data/misra_rules.yaml` | Stub for 0078c (empty, correct structure) | 22 |

---

## Files Modified

| File | Change |
|------|--------|
| `python/requirements.txt` | Added `pyyaml==6.0.2` (N2 from design review) |
| `.mcp.json` | Added `guidelines` server entry pointing to `build/Debug/docs/guidelines.db` |
| `.claude/settings.local.json` | Added `"guidelines"` to `enabledMcpjsonServers` |
| `CMakeLists.txt` | Added `guidelines-seed` custom target |
| `CMakeUserPresets.json` | Added `debug-guidelines` build preset |

---

## Design Adherence Matrix

| Design Requirement | Status | Notes |
|-------------------|--------|-------|
| `guidelines_schema.py` with `create_schema()` and `drop_schema()` | IMPLEMENTED | DDL split into separate constants; triggers maintain FTS5 index |
| `seed_guidelines.py` with Pydantic validation, all-or-nothing abort | IMPLEMENTED | ValidationError before any DB writes; duplicate rule_id detection |
| `guidelines_server.py` with 5 MCP tools + CLI mode | IMPLEMENTED | Follows `mcp_codebase_server.py` class+factory pattern exactly |
| `project_rules.yaml` with 10 MSD rules from CLAUDE.md | IMPLEMENTED | All 10 rules with full rationale, examples, tags, cross-refs |
| FTS5 with porter tokenizer (stemming) | IMPLEMENTED | Verified: "initialize" → MSD-INIT-001, MSD-INIT-002 |
| BM25 ranking via `bm25(rules_fts)` | IMPLEMENTED | FTS query uses ORDER BY rank |
| `get_rule` not-found contract (N1) | IMPLEMENTED | Returns `{"error": "Rule not found", "rule_id": "..."}` |
| `pyyaml` pinned version (N2) | IMPLEMENTED | `pyyaml==6.0.2` added to `python/requirements.txt` |
| No automated pytest (N3) | ACKNOWLEDGED | CLI smoke tests only; pytest can be added in follow-up |
| `enforcement_check` optional with explanatory comment (N4) | IMPLEMENTED | Pydantic field `str \| None = None` with inline comment |
| DD-0078-001: `get_related_rules` merged into `get_rule` | IMPLEMENTED | `get_rule` returns cross_refs + tags; `get_rules_by_tag` added |
| DD-0078-002: `status` column | IMPLEMENTED | `proposed \| active \| deprecated`; active filter in `search_guidelines` |
| DD-0078-003: `enforcement_check` column | IMPLEMENTED | Nullable TEXT; MSD-RES-001 and MSD-NAME-001 have values |
| DD-0078-004: CLAUDE.md migration | DEFERRED | Tracked as follow-up 0078d (out of scope for this ticket) |
| DD-0078-005: `get_category` default summary mode | IMPLEMENTED | Returns only rule_id+title+severity+status+source by default |
| `guidelines-seed` CMake target | IMPLEMENTED | Follows `trace-git` pattern |
| `debug-guidelines` CMake preset | IMPLEMENTED | Follows `debug-traceability` pattern |
| `.mcp.json` registration | IMPLEMENTED | Additive entry; existing entries untouched |
| `.claude/settings.local.json` registration | IMPLEMENTED | `"guidelines"` added to `enabledMcpjsonServers` |

---

## Acceptance Criteria Results

All 10 acceptance criteria from the ticket pass:

| Criterion | Result |
|-----------|--------|
| `search_guidelines("brace initialization")` returns MSD-INIT-002 | PASS |
| `get_rule("MSD-RES-001")` returns full rule with category, tags, cross-refs, status | PASS |
| `list_categories()` returns all categories with correct rule counts | PASS |
| `get_category("Initialization")` returns summary (rule_id + title) by default | PASS |
| `get_category("Initialization", detailed=True)` returns full rationale and examples | PASS |
| `get_rules_by_tag("safety")` returns MSD-INIT-001, MSD-RES-001, MSD-RES-003 | PASS |
| CLI mode works for all 5 commands | PASS |
| Database fully rebuildable from YAML seed files | PASS |
| FTS5 porter stemming (e.g., "initialize" matches MSD-INIT-001 and MSD-INIT-002) | PASS |
| Seed script validates YAML before insertion (pydantic) | PASS |

---

## Implementation Details

### FTS5 Trigger Architecture

The FTS5 `rules_fts` table uses `content='rules'` mode with three triggers
(`rules_fts_insert`, `rules_fts_delete`, `rules_fts_update`) to keep the virtual table
synchronized with the base `rules` table. This matches the recommended FTS5 external
content pattern and avoids duplicating data.

### Cross-Reference Directionality

`get_rule` returns cross-refs from both directions (outgoing: from_rule_id = this rule,
incoming: to_rule_id = this rule) with a `direction` field. This means bidirectional
relationships declared in only one direction in the YAML still appear in both rules'
`get_rule` output.

### YAML stub files

`cpp_core_guidelines.yaml` and `misra_rules.yaml` are valid YAML with `rules: []` so
the seed script processes them without error and doesn't fail when the list is empty.
This keeps the pipeline clean for future population in 0078b/0078c.

### pyyaml version pinning

Selected `pyyaml==6.0.2` — the current stable release as of 2026-02-26, consistent
with the project convention of `major.minor.patch` pins.

---

## Deviations from Design

None. All design requirements implemented as specified.

---

## Known Limitations

1. **No `__init__.py`**: `scripts/guidelines/` is not a Python package; files import
   each other via `sys.path` manipulation in `seed_guidelines.py`. This is consistent
   with how other scripts in `scripts/` operate.

2. **FTS5 porter stemmer availability**: Design noted R2 (porter tokenizer may not be
   available on all SQLite builds). On macOS the default SQLite does support porter.
   The schema uses `tokenize='porter ascii'` which falls back gracefully. Phase 4
   smoke tests confirmed porter stemming works on this machine.

3. **guidelines.db not in debug-all preset**: The `debug-all` CMakeUserPresets.json
   preset does not include `guidelines-seed`. This is intentional — guidelines seeding
   is a one-time/on-demand operation (like traceability indexing is separate from C++
   builds). Run `cmake --build --preset debug-guidelines` explicitly.

---

## Future Considerations

- **0078a**: Update agent prompts to reference guidelines MCP tools
- **0078b**: Populate C++ Core Guidelines (R, C, ES sections) with clang-tidy mappings
- **0078c**: Populate MISRA rules (memory and initialization categories)
- **0078d**: Migrate CLAUDE.md coding standards to a pointer to the guidelines MCP server
- **Future**: Add pytest suite using in-memory `:memory:` SQLite DB for unit tests
