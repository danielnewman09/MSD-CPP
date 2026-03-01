# Feature Ticket: Consolidate Traceability into Workflow Engine

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Test Writing
- [x] Test Writing Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-02-28
- **Author**: Daniel Newman
- **Priority**: Medium
- **Estimated Complexity**: Large
- **Target Component(s)**: workflow-engine, scripts/traceability, .mcp.json
- **Languages**: Python
- **Generate Tutorial**: No
- **Requires Math Design**: No
- **GitHub Issue**:

---

## Summary
Move the traceability MCP server and its indexers into the workflow engine as a first-class module. The workflow engine already manages tickets and phases for a specific repository — traceability (git history, design decisions, symbol snapshots, record mappings) is a natural extension of that same per-repo concern. After consolidation, a single MCP server (the workflow engine's SSE endpoint) serves both workflow coordination tools and traceability query tools. The traceability database remains a separate SQLite file, ATTACHed by the workflow engine at startup.

## Motivation

### Current state
- **Two MCP servers** doing related per-repo work: workflow (SSE, Docker) and traceability (stdio, in-process)
- **Traceability requires a CMake rebuild** (`cmake --build --preset debug-traceability`) to update git history, symbol snapshots, and design decisions — even though this data could be indexed incrementally as tickets close
- **The workflow engine claims to be project-agnostic**, but it already reads project-specific ticket markdown, phase definitions, and configuration from `.workflow/` — it is bound to the consuming repo

### After consolidation
- **One MCP server** for all per-repo workflow and traceability concerns
- **Incremental indexing** triggered by ticket lifecycle events (specifically when terminal phases complete), eliminating routine CMake rebuilds
- **Honest architecture** — the workflow engine is a project workflow service, not a generic framework
- **codebase.db stays separate** — it's cheap to rebuild via Doxygen and requires build artifacts, so it remains a CMake target and a separate MCP server

## Requirements

### Functional Requirements
1. Add a `traceability` module to the workflow engine package (`workflow_engine.traceability`)
2. Move indexer scripts into the module: `index_git_history`, `index_decisions`, `index_symbols`, `index_record_mappings`
3. Move `traceability_schema.py` into the module for schema creation and migration
4. Move traceability MCP tool implementations into the module (query logic from `traceability_server.py`)
5. Register traceability MCP tools alongside workflow tools in the same `create_mcp_server()`
6. ATTACH `traceability.db` as a separate database file (not merged into `workflow.db`)
7. Support optional ATTACH of `codebase.db` for cross-referencing (existing behavior)
8. Trigger incremental indexing when a ticket's terminal phase completes:
   - `index_git_history` — index new commits since last run (by SHA deduplication)
   - `index_decisions` — scan the completed ticket's design docs
   - `index_symbols` — snapshot symbols at HEAD (single tree-sitter parse, no worktree juggling)
   - `index_record_mappings` — re-parse Python models (cheap full rebuild)
9. Add `.workflow/config.yaml` settings for traceability: db path, source directories, tree-sitter toggle
10. Retain CMake target `debug-traceability` as a full seed/rebuild escape hatch (calls the same indexers in batch mode)

### Non-Functional Requirements
- **No query regression** — all 9 existing traceability MCP tools must return identical results
- **Incremental indexing must complete in <5s** for a typical ticket (a few commits, one design doc)
- **tree-sitter remains optional** — if not installed, symbol indexing is skipped with a warning (existing behavior)

## Constraints
- `traceability.db` and `workflow.db` are separate files with separate lifecycles (workflow state is ephemeral coordination, traceability is accumulated history)
- `codebase.db` stays in MSD-CPP as a CMake target and separate MCP server
- The indexer scripts can be called both incrementally (from lifecycle events) and in batch (from CMake target)
- tree-sitter and tree-sitter-cpp become dependencies of the workflow-engine package (optional, for symbol indexing)

## Acceptance Criteria
- [ ] `workflow_engine.traceability` module exists with indexers, schema, and query tools
- [ ] Single MCP server exposes both workflow tools (27) and traceability tools (9)
- [ ] `traceability` entry removed from `.mcp.json` (tools served by `workflow` server)
- [ ] `traceability.db` is ATTACHed (not merged) — can be deleted and rebuilt independently
- [ ] Completing a ticket's terminal phase triggers incremental indexing
- [ ] Incremental symbol indexing snapshots HEAD only (no git worktree per commit)
- [ ] `cmake --build --preset debug-traceability` still works as a full rebuild
- [ ] All 9 traceability MCP tools return identical results to the current standalone server
- [ ] `.workflow/config.yaml` gains a `traceability:` section for DB path and settings
- [ ] `scripts/traceability/` is removed from MSD-CPP after migration

---

## Design Decisions (Human Input)

### Preferred Approaches
- Separate DB files, ATTACHed at runtime (proven pattern from current traceability server)
- Incremental indexing on ticket close, not on every commit or phase change
- Symbol indexing at HEAD only for incremental mode (full history remains available via CMake batch)
- First-class module in workflow engine, not a plugin system

### Things to Avoid
- Do not merge traceability tables into workflow.db — different data lifecycles
- Do not add a generic plugin/extension framework — traceability is a direct module
- Do not move codebase.db into the workflow engine — it depends on Doxygen build artifacts
- Do not change the traceability query logic — this is a migration, not a rewrite

### Open Questions
1. **tree-sitter as a dependency** — Should it be a required or optional dependency of workflow-engine? Currently optional in scripts/traceability. Recommend keeping it optional with a `[traceability]` extras group in pyproject.toml.
2. **Incremental trigger mechanism** — Should `complete_phase` call indexers synchronously (simpler, blocks the MCP response briefly) or asynchronously (background thread, non-blocking)? Recommend synchronous for the initial implementation — <5s is acceptable latency for a terminal phase completion.
3. **CMake target after migration** — Should `debug-traceability` invoke the workflow engine CLI (`workflow-engine reindex`) or continue calling standalone Python scripts? Recommend the CLI approach for consistency.

---

## References

### Related Code
- `scripts/traceability/` — Current indexers and server (to be moved)
- `scripts/traceability/traceability_server.py` — Query logic (694 lines)
- `scripts/traceability/traceability_schema.py` — Schema creation (216 lines)
- `scripts/traceability/index_git_history.py` — Git commit indexer
- `scripts/traceability/index_decisions.py` — Design decision extractor
- `scripts/traceability/index_symbols.py` — tree-sitter symbol snapshotter
- `scripts/traceability/index_record_mappings.py` — Pydantic model indexer
- `.mcp.json` — MCP server configuration (traceability entry to be removed)
- `.workflow/config.yaml` — Workflow configuration (to gain traceability section)

### Related Tickets
- `tickets/0083_database_agent_orchestration.md` — Workflow engine (parent)
- `tickets/0083a_workflow_engine_repo_extraction.md` — Standalone repo extraction
- `tickets/0045_traceability_database.md` — Original traceability implementation (if exists)

---

## Workflow Log

### Design Phase
- **Started**: 2026-02-28 15:21
- **Completed**: 2026-02-28 15:34
- **Branch**: 0085-traceability-workflow-consolidation
- **PR**: N/A (workflow-engine repo, works on main)
- **Artifacts**:
  - `docs/designs/0085_traceability_workflow_consolidation/design.md`
  - `docs/designs/0085_traceability_workflow_consolidation/0085_traceability_workflow_consolidation.puml`
- **Notes**: Design covers module structure, ATTACH pattern, incremental indexing on commit, tool registration, Docker changes.

### Design Review Phase
- **Started**: 2026-02-28 16:07
- **Completed**: 2026-02-28 16:08
- **Branch**: 0085-traceability-workflow-consolidation
- **PR**: N/A
- **Notes**: Approved. Implementation plan: 5 sequential phases on single branch.

### Python Design Phase
- **Started**: 2026-02-28 16:10
- **Completed**: 2026-02-28 16:15
- **Branch**: 0085-traceability-workflow-consolidation
- **PR**: N/A
- **Notes**: Python design covered in main design doc. No additional Python-specific design document needed.

### Python Design Review Phase
- **Started**: 2026-02-28 18:41
- **Completed**: 2026-02-28 18:41
- **Notes**: Approved after one revision round. Three comments addressed: DB location (Docker volume), absolute imports, indexing trigger moved to commit_and_push.

### Prototype Phase
- **Completed**: 2026-02-28 18:42
- **Notes**: Prototype skipped — direct migration of existing working code, no new algorithms.

### Implementation Phase
- **Started**: 2026-02-28 18:55
- **Completed**: 2026-02-28 19:42
- **Artifacts**:
  - `workflow_engine/traceability/__init__.py`
  - `workflow_engine/traceability/schema.py`
  - `workflow_engine/traceability/server.py`
  - `workflow_engine/traceability/index_git.py`
  - `workflow_engine/traceability/index_decisions.py`
  - `workflow_engine/traceability/index_symbols.py`
  - `workflow_engine/traceability/index_records.py`
  - `workflow_engine/traceability/reindex.py`
  - `.workflow/config.yaml` (traceability section added)
  - `.mcp.json` (standalone traceability entry removed)
- **Notes**: Full migration complete. 9 traceability MCP tools registered in workflow server. Incremental reindex triggered after commit_and_push. tree-sitter extras added to Dockerfile.

### Test Writing Phase
- **Started**: 2026-02-28 19:49
- **Completed**: 2026-02-28 (orchestrator session)
- **Artifacts**:
  - `tests/test_traceability_schema.py` (21 tests)
  - `tests/test_traceability_server.py` (53 tests)
  - `tests/test_traceability_indexers.py` (36 tests)
  - `tests/test_traceability_reindex.py` (9 tests)
- **Notes**: 119 tests, all passing. Previous test-writer agent stalled (no heartbeat after claim); cleaned up stale claim and re-ran. Documented a SQLite executescript() limitation: ensure_schema(prefix='trace.') is not supported; the production ATTACH workflow correctly calls create_schema() on standalone DB before ATTACHing (per design Section 5.3).

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Implementation
{Your comments on the implementation}
