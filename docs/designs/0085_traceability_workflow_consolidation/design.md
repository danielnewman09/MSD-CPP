# Design: Consolidate Traceability into Workflow Engine

**Ticket**: 0085_traceability_workflow_consolidation
**Author**: Claude (python-architect)
**Date**: 2026-02-28

---

## 1. Overview

Move the traceability MCP server (~2,200 lines across 6 Python files) from `scripts/traceability/` in MSD-CPP into the workflow engine as a first-class `workflow_engine.traceability` module. After consolidation, one MCP server exposes both workflow tools (30+) and traceability tools (9). The traceability database remains a separate SQLite file, ATTACHed at startup.

## 2. Current Architecture

```
MSD-CPP/.mcp.json
  ├── traceability (stdio, python/.venv)
  │     └── traceability_server.py → traceability.db (+ ATTACH codebase.db)
  ├── codebase (stdio, python/.venv)
  │     └── mcp_codebase_server.py → codebase.db
  └── workflow (HTTP, Docker :8081)
        └── workflow_engine → workflow.db
```

**Problems:**
- Two servers doing related per-repo work
- Traceability requires CMake rebuild to index (`cmake --build --preset debug-traceability`)
- No integration between workflow events and traceability indexing

## 3. Target Architecture

```
MSD-CPP/.mcp.json
  ├── codebase (stdio, python/.venv)  ← unchanged
  │     └── mcp_codebase_server.py → codebase.db
  └── workflow (HTTP, Docker :8081)
        └── workflow_engine
              ├── engine/ → workflow.db
              └── traceability/ → traceability.db (ATTACHed)
                    ├── schema.py
                    ├── server.py (query tools)
                    ├── index_git.py
                    ├── index_symbols.py
                    ├── index_decisions.py
                    └── index_records.py
```

**Result:** Single MCP server, 39+ tools, incremental indexing on ticket completion.

## 4. Module Structure

### 4.1 New Package: `workflow_engine/traceability/`

```
workflow_engine/traceability/
├── __init__.py           # Public API: register_tools(), run_indexers()
├── schema.py             # Schema creation + migration (from traceability_schema.py)
├── server.py             # TraceabilityServer class + register_tools() (from traceability_server.py)
├── index_git.py          # Git history indexer (from index_git_history.py)
├── index_symbols.py      # Symbol snapshot indexer (from index_symbols.py)
├── index_decisions.py    # Decision extractor (from index_decisions.py)
├── index_records.py      # Record mapping indexer (from index_record_mappings.py)
└── reindex.py            # Orchestrator: run all indexers in sequence
```

### 4.2 Migration Strategy

Each file is a **direct move** with minimal changes:
- Update relative imports (e.g., `from .schema import ensure_schema`)
- Remove `if __name__ == "__main__"` CLI blocks (replaced by `reindex.py` orchestrator)
- Keep all query logic, indexer logic, and SQL unchanged

**No functional rewrites.** This is a migration, not a refactor.

### 4.3 File-by-File Mapping

| Source (MSD-CPP) | Target (workflow-engine) | Changes |
|---|---|---|
| `traceability_schema.py` (215 lines) | `traceability/schema.py` | Import paths only |
| `traceability_server.py` (778 lines) | `traceability/server.py` | Extract server class + `register_tools(mcp, server)` function; remove CLI/main |
| `index_git_history.py` (235 lines) | `traceability/index_git.py` | Import paths, accept `conn` instead of `db_path` |
| `index_symbols.py` (430 lines) | `traceability/index_symbols.py` | Import paths, accept `conn` instead of `db_path` |
| `index_decisions.py` (344 lines) | `traceability/index_decisions.py` | Import paths, accept `conn` instead of `db_path` |
| `index_record_mappings.py` (191 lines) | `traceability/index_records.py` | Import paths, accept `conn` instead of `db_path` |

## 5. Database Architecture

### 5.1 Separate Files, ATTACHed at Runtime

```
/data/workflow.db          # Docker volume — workflow coordination state
/data/traceability.db      # Docker volume — accumulated traceability history
/project/build/.../codebase.db  # Bind mount — Doxygen symbols (optional ATTACH)
```

Both `workflow.db` and `traceability.db` live on the Docker `workflow-data` volume (`/data/`), **not** in the project repository. They persist across container restarts but are not committed to git. The `db_path` in `.workflow/config.yaml` is resolved relative to the container's project root — in Docker, this becomes `/data/traceability.db` via the `DB_PATH`-style env var pattern.

**Rationale:** Different lifecycles. `workflow.db` is coordination state that resets. `traceability.db` is accumulated history that can be rebuilt from the repo. Merging them would complicate backup, migration, and rebuild. Neither belongs in the git repo.

### 5.2 ATTACH Pattern

At `WorkflowServer.__init__()`:

```python
# ATTACH traceability database
trace_db_path = self.config.traceability_db_path
if trace_db_path:
    ensure_traceability_schema(trace_db_path)  # Create if missing
    self.conn.execute("ATTACH DATABASE ? AS trace", (trace_db_path,))
    self.has_traceability = True

# Optional: ATTACH codebase database
codebase_db_path = self.config.codebase_db_path
if codebase_db_path and Path(codebase_db_path).exists():
    self.conn.execute("ATTACH DATABASE ? AS codebase", (codebase_db_path,))
    self.has_codebase = True
```

**Schema prefix convention:** All traceability queries use `trace.table_name` prefix. The traceability server class receives the connection and queries against the `trace` schema alias.

### 5.3 Schema Initialization

`traceability/schema.py` provides `ensure_schema(db_path)` which:
1. Opens a temporary connection to `traceability.db`
2. Creates all 10 tables + 3 FTS tables if they don't exist
3. Runs migrations if `schema_info.version` is behind
4. Closes the temporary connection

This runs **before** the ATTACH, so the traceability DB is ready when ATTACHed.

**Important:** After ATTACH, all traceability SQL in `server.py` must use `trace.` prefix on table names. This is the main mechanical change in the server query code.

## 6. Tool Registration

### 6.1 Integration Pattern

The traceability module exposes a `register_tools(mcp, server)` function that adds 9 tools to an existing FastMCP instance:

```python
# In workflow_engine/traceability/server.py

class TraceabilityServer:
    """Query interface for traceability database (ATTACHed as 'trace')."""

    def __init__(self, conn: sqlite3.Connection, has_codebase: bool = False):
        self.conn = conn
        self.has_codebase = has_codebase

def register_tools(mcp: "FastMCP", server: TraceabilityServer) -> None:
    """Register traceability MCP tools on the given FastMCP instance."""

    @mcp.tool()
    def search_decisions(query: str, ticket: str | None = None, ...) -> str:
        return json.dumps(server.search_decisions(query, ticket, ...), indent=2, default=str)

    # ... 8 more tools
```

### 6.2 Integration in `create_mcp_server()`

```python
# In workflow_engine/server/server.py

def create_mcp_server(db_path: str, project_root: str) -> "FastMCP":
    ws = WorkflowServer(db_path, project_root)
    mcp = FastMCP("workflow")

    # Register workflow tools (existing)
    _register_workflow_tools(mcp, ws)

    # Register traceability tools (new)
    if ws.has_traceability:
        from ..traceability.server import TraceabilityServer, register_tools
        trace_server = TraceabilityServer(ws.conn, ws.has_codebase)
        register_tools(mcp, trace_server)

    return mcp
```

### 6.3 Tool Names

All 9 traceability tools keep their existing names — no prefix needed because they're already distinct from workflow tool names:

| Traceability Tools | Workflow Tools (sample) |
|---|---|
| `search_decisions` | `register_agent` |
| `get_decision` | `claim_phase` |
| `get_symbol_history` | `complete_phase` |
| `get_ticket_impact` | `get_ticket_status` |
| `get_commit_context` | `list_tickets` |
| `why_symbol` | `setup_branch` |
| `get_snapshot_symbols` | `commit_and_push` |
| `get_record_mappings` | `list_agents` |
| `check_record_drift` | `run_scheduler` |

No naming conflicts.

## 7. Incremental Indexing

### 7.1 Trigger Point

Incremental indexing runs in `commit_and_push()` after a successful push. Every commit creates indexable artifacts (new git history, potentially new design decisions, changed symbols), so indexing at commit time keeps traceability data current throughout development — not just at ticket completion.

The indexing flow inside `commit_and_push`:
1. Stage, commit, push (existing logic)
2. Post PlantUML comments (existing logic)
3. Run incremental indexing for the just-pushed commit SHA

### 7.2 Indexing Scope

```python
# In workflow_engine/traceability/reindex.py

def run_incremental(conn: sqlite3.Connection, project_root: Path, ticket_id: str | None = None) -> dict:
    """Run incremental indexing. If ticket_id is provided, scope decisions to that ticket."""
    results = {}

    # 1. Git history — always incremental (skip existing SHAs)
    results["git"] = index_git.index_incremental(conn, project_root)

    # 2. Decisions — scope to ticket's design docs if ticket_id provided
    results["decisions"] = index_decisions.index_incremental(conn, project_root, ticket_id)

    # 3. Symbols — snapshot HEAD only (no worktree per commit)
    results["symbols"] = index_symbols.index_head_only(conn, project_root)

    # 4. Record mappings — cheap full rebuild
    results["records"] = index_records.index_all(conn, project_root)

    return results

def run_full(conn: sqlite3.Connection, project_root: Path) -> dict:
    """Full rebuild — equivalent to current CMake target."""
    results = {}
    results["git"] = index_git.index_all(conn, project_root)
    results["decisions"] = index_decisions.index_all(conn, project_root)
    results["symbols"] = index_symbols.index_all(conn, project_root)
    results["records"] = index_records.index_all(conn, project_root)
    return results
```

### 7.3 Symbol Indexing at Commit Time

Symbol indexing runs **after each commit**, not at ticket completion. Since `commit_and_push` just created a new commit, the SHA exists and the working tree matches it exactly — no git worktree needed.

```python
def index_at_commit(conn, project_root, commit_sha):
    """Snapshot symbols for the commit that was just made."""
    # The commit was just created — look it up in snapshots (git indexer ran first)
    row = conn.execute("SELECT id FROM trace.snapshots WHERE sha = ?", (commit_sha,)).fetchone()
    if row is None:
        return {"skipped": True, "reason": "commit not in snapshots"}
    snapshot_id = row[0]

    # Parse files directly from working tree (matches commit exactly)
    symbols = extract_symbols_from_directory(parser, project_root / "msd")

    # Store and compute changes vs previous snapshot
    store_symbols(conn, snapshot_id, symbols)
    compute_changes(conn, snapshot_id)

    return {"snapshot_id": snapshot_id, "symbols": len(symbols)}
```

The full-history mode (worktree per commit) remains available via `run_full()` for CMake batch rebuilds.

### 7.4 Synchronous Execution

Incremental indexing runs **synchronously** in `commit_and_push()`, after a successful push. Expected latency: <5 seconds (one git log update, one tree-sitter parse of HEAD, one decision scan).

```python
# In workflow_engine/engine/github.py — after successful commit_and_push

from workflow_engine.traceability.reindex import run_incremental
run_incremental(conn, project_root, ticket_id)
```

**Import convention:** All cross-package imports use absolute paths (`from workflow_engine.traceability.reindex import ...`), never relative imports (`from ..traceability`). The traceability module is a proper subpackage of `workflow_engine`, installed via pip, so absolute imports work everywhere.

## 8. Configuration

### 8.1 New Config Section

Add to `.workflow/config.yaml`:

```yaml
traceability:
  db_path: "/data/traceability.db"  # Docker volume (or relative to project root for local dev)
  codebase_db_path: "build/Debug/docs/codebase.db"  # Optional, for cross-ref (bind mount)
  source_directories:
    - "msd/"            # C++ sources for symbol indexing
  design_directories:
    - "docs/designs/"   # Design docs for decision extraction
    - "tickets/"        # Ticket markdown for decision extraction
  index_on_completion: true    # Trigger incremental indexing on ticket completion
  tree_sitter: true            # Enable symbol indexing (requires tree-sitter)
```

### 8.2 Model Changes

Add to `WorkflowConfig`:

```python
# Traceability settings
traceability_db_path: str | None = None
codebase_db_path: str | None = None
traceability_source_dirs: list[str] = field(default_factory=lambda: ["msd/"])
traceability_design_dirs: list[str] = field(default_factory=lambda: ["docs/designs/", "tickets/"])
traceability_index_on_completion: bool = True
traceability_tree_sitter: bool = True
```

### 8.3 Config Parsing

In `config.py`, add parsing of the `traceability` section:

```python
trace_section = config_doc.get("traceability", {})
traceability_db_path = trace_section.get("db_path")
if traceability_db_path and not Path(traceability_db_path).is_absolute():
    traceability_db_path = str(project_root / traceability_db_path)

codebase_db_path = trace_section.get("codebase_db_path")
if codebase_db_path and not Path(codebase_db_path).is_absolute():
    codebase_db_path = str(project_root / codebase_db_path)
```

## 9. Docker Changes

### 9.1 tree-sitter in Docker

Add tree-sitter as an optional dependency in `pyproject.toml`:

```toml
[project.optional-dependencies]
traceability = [
    "tree-sitter>=0.21",
    "tree-sitter-cpp>=0.21",
]
dev = [
    "pytest>=8.0",
]
```

Update Dockerfile to install with traceability extras:

```dockerfile
RUN pip install --no-cache-dir ".[traceability]"
```

### 9.2 Volume Mounts

The current container mount (`-v "$(pwd):/project"`) already provides access to:
- `.git/` for git history indexing
- `msd/` source files for symbol indexing
- `docs/designs/` and `tickets/` for decision indexing
- `replay/replay/models.py` for record mapping indexing

No additional mounts needed.

## 10. CMake Target Retention

The `debug-traceability` CMake target remains as a full rebuild escape hatch. Update it to call the workflow engine CLI:

```cmake
add_custom_target(trace-full
    COMMAND docker exec workflow-engine
        python -m workflow_engine.traceability.reindex
        --db-path /data/traceability.db
        --project-root /project
        --full
    COMMENT "Full traceability rebuild via workflow engine"
)
```

Alternatively, if Docker isn't running, fall back to a standalone script that imports the traceability module directly. The `reindex.py` module can be run as `python -m workflow_engine.traceability.reindex` with CLI args.

## 11. MSD-CPP Changes

### 11.1 `.mcp.json` Update

Remove the `traceability` entry:

```json
{
  "mcpServers": {
    "codebase": { ... },
    "replay": { ... },
    "guidelines": { ... },
    "workflow": {
      "type": "http",
      "url": "http://localhost:8081/mcp"
    }
  }
}
```

### 11.2 Cleanup

After migration is complete and verified:
- Remove `scripts/traceability/` directory
- Update `CLAUDE.md` references to traceability
- Update `python/requirements.txt` (remove tree-sitter if no other consumer)
- Update CMake targets to use workflow engine CLI

## 12. Implementation Plan

### Phase 1: Module Scaffold (workflow-engine repo)
1. Create `workflow_engine/traceability/` package
2. Move `schema.py` with import updates
3. Move indexer files with import updates and `conn` parameter changes
4. Move `server.py` with `register_tools()` pattern
5. Create `reindex.py` orchestrator
6. Add `[traceability]` extras to `pyproject.toml`

### Phase 2: Integration
7. Add traceability config fields to `WorkflowConfig` model
8. Parse `traceability` section in `config.py`
9. ATTACH traceability.db in `WorkflowServer.__init__()`
10. Call `register_tools()` in `create_mcp_server()`
11. Add `trace.` prefix to all traceability SQL queries

### Phase 3: Incremental Indexing
12. Add `index_head_only()` to symbol indexer
13. Add `run_incremental()` to `reindex.py`
14. Hook into `complete_phase()` for ticket completion trigger
15. Add `reindex` CLI subcommand

### Phase 4: Docker & Config
16. Update Dockerfile with `[traceability]` install
17. Add `traceability:` section to `.workflow/config.yaml`
18. Rebuild and test Docker image

### Phase 5: MSD-CPP Cleanup
19. Remove `traceability` from `.mcp.json`
20. Update CMake targets
21. Remove `scripts/traceability/` (after verification)
22. Update documentation

## 13. Risk Mitigation

| Risk | Mitigation |
|---|---|
| Query regression from `trace.` prefix | Run existing test queries against both old and new servers, compare results |
| tree-sitter install fails in Docker | Make it optional; skip symbol indexing with warning if unavailable |
| ATTACH path issues in Docker | Use absolute paths from config; ensure traceability.db directory exists |
| Incremental indexing too slow | Profile; the <5s budget is generous for typical incremental work |
| Breaking existing workflow tools | Traceability registration is additive; no changes to workflow tool code |

## 14. Open Questions (Resolved)

1. **tree-sitter as dependency** → Optional via `[traceability]` extras group. *(per ticket recommendation)*
2. **Incremental trigger** → Synchronous in `complete_phase()`. *(per ticket recommendation)*
3. **CMake target** → Calls workflow engine CLI (`python -m workflow_engine.traceability.reindex`). *(per ticket recommendation)*
