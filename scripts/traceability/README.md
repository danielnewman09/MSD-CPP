# Traceability System

Design decision traceability tools that index design decisions from ticket artifacts, snapshot symbol locations at each commit, and correlate both with git history. Exposed as MCP tools alongside the existing codebase server.

## Quick Start

```bash
# Install dependencies (one-time)
scripts/.venv/bin/pip install tree-sitter tree-sitter-cpp

# Build the full traceability database
cmake --build --preset debug-traceability

# Or run indexers individually
scripts/.venv/bin/python3 scripts/traceability/index_git_history.py build/Debug/docs/traceability.db --repo .
scripts/.venv/bin/python3 scripts/traceability/index_decisions.py build/Debug/docs/traceability.db --repo .
scripts/.venv/bin/python3 scripts/traceability/index_symbols.py build/Debug/docs/traceability.db --repo .

# Record mappings: generator populates 4 layers, indexer handles composites
scripts/.venv/bin/python3 scripts/generate_record_layers.py --check-only --update-traceability build/Debug/docs/traceability.db
scripts/.venv/bin/python3 scripts/traceability/index_record_mappings.py build/Debug/docs/traceability.db --repo .
```

## Architecture

The traceability database (`traceability.db`) is separate from `codebase.db` because codebase.db is rebuilt from scratch on every `doxygen-db` run, while traceability data accumulates over time and must survive rebuilds. The database is built into `build/Debug/docs/traceability.db` (gitignored). All indexers are incremental — after the initial build (~30s), updates after new commits take seconds.

```
scripts/traceability/
├── traceability_schema.py       # Shared schema (imported by all indexers)
├── index_git_history.py         # Git log → snapshots + file_changes
├── index_symbols.py             # Tree-sitter C++ parsing → symbol_snapshots + symbol_changes
├── index_decisions.py           # Ticket/design doc parsing → design_decisions + links
├── index_record_mappings.py     # Cross-layer record mapping indexer
├── traceability_server.py       # MCP server (9 tools) + CLI mode
└── README.md                    # This file
```

## Scripts

### `index_git_history.py`

Walks all git commits in chronological order, extracts metadata from commit messages (prefix like `impl:`, ticket number like `0045`, workflow phase), and records file-level diffs.

- **Tables populated**: `snapshots`, `file_changes`
- **Incremental**: Skips commits already in the database
- **Typical output**: ~135 snapshots, ~2400 file changes

### `index_symbols.py`

For each snapshot, checks out the commit via `git worktree`, parses all `.hpp`/`.cpp` files under `msd/` with tree-sitter, and records symbol locations. Computes diffs between consecutive snapshots to track additions, removals, modifications, and moves.

- **Tables populated**: `symbol_snapshots`, `symbol_changes`
- **Depends on**: `snapshots` table (run `index_git_history.py` first)
- **Incremental**: Skips snapshots already indexed
- **Typical output**: ~700 symbols per recent commit, ~2500 symbol changes total
- **Runtime**: ~30s for full history (0.2s per commit)

### `index_decisions.py`

Parses ticket files and design documents to extract design decisions. Supports two modes:

- **Structured**: Parses explicit `### DD-NNNN-NNN:` blocks in design docs (preferred for new tickets)
- **Heuristic**: Scans for decision-indicating patterns (`**Design rationale**:`, `RESOLVED`, `**Recommendation**:`, `### Root Cause:`, etc.)

Links decisions to affected symbols (via `Affects` field or PascalCase identifier matching) and to implementing commits (via ticket number matching).

- **Tables populated**: `design_decisions`, `decision_symbols`, `decision_commits`, `design_decisions_fts`
- **Depends on**: `snapshots` table (run `index_git_history.py` first)
- **Incremental**: Skips decisions already in the database
- **Typical output**: ~160 decisions, ~60 symbol links

### `index_record_mappings.py`

Indexes **hand-written composite Pydantic models** (BodyState, FrameData, etc.) into the traceability database. These are models that aggregate generated leaf models and cannot be auto-generated.

**Introduced**: [Ticket 0061](../../tickets/0061_cross_layer_record_mapping.md)

The four authoritative layers (cpp, sql, pybind, leaf-pydantic) are now populated directly by `scripts/generate_record_layers.py --update-traceability` (see [Ticket 0062](../../tickets/0062_pybind_codegen_from_boost_describe.md)). This indexer only handles what the generator cannot: composite Pydantic models from `replay/replay/models.py`.

- **Tables populated**: `record_layer_fields` (pydantic layer, notes='composite')
- **Clears and rebuilds**: Only composite entries, preserving generator data
- **Typical output**: ~14 composite Pydantic models

### `traceability_server.py`

MCP server providing 9 traceability tools. Also supports CLI mode for testing.

## MCP Tools

| Tool | Description |
|------|-------------|
| `search_decisions(query, ticket?, status?)` | FTS search across design decision rationale |
| `get_decision(dd_id)` | Full details of a decision with linked symbols and commits |
| `get_symbol_history(qualified_name)` | Timeline of changes to a symbol across commits |
| `get_ticket_impact(ticket_number)` | All commits, file changes, symbol changes, and decisions for a ticket |
| `get_commit_context(commit_sha)` | Context for a commit: ticket, phase, decisions, symbol changes |
| `why_symbol(qualified_name)` | Design decision(s) that created/modified a symbol, with rationale |
| `get_snapshot_symbols(commit_sha, file_path?)` | All symbols at a specific point in time |
| `get_record_mappings(record_name)` | Cross-layer field comparison for a C++ record (cpp/sql/pybind/pydantic) with drift analysis |
| `check_record_drift()` | All records with fields missing from downstream layers (pybind/Pydantic) |

## CLI Usage

```bash
# Search for decisions about energy conservation
scripts/.venv/bin/python3 scripts/traceability/traceability_server.py build/Debug/docs/traceability.db search_decisions energy

# Why does PositionCorrector exist?
scripts/.venv/bin/python3 scripts/traceability/traceability_server.py build/Debug/docs/traceability.db why_symbol PositionCorrector

# What changed in ticket 0045?
scripts/.venv/bin/python3 scripts/traceability/traceability_server.py build/Debug/docs/traceability.db get_ticket_impact 0045

# What happened in commit 3ca903f?
scripts/.venv/bin/python3 scripts/traceability/traceability_server.py build/Debug/docs/traceability.db get_commit_context 3ca903f

# History of ConstraintSolver changes
scripts/.venv/bin/python3 scripts/traceability/traceability_server.py build/Debug/docs/traceability.db get_symbol_history ConstraintSolver

# Symbols at a point in time
scripts/.venv/bin/python3 scripts/traceability/traceability_server.py build/Debug/docs/traceability.db get_snapshot_symbols 3ca903f ConstraintSolver.hpp

# Cross-layer record field comparison
scripts/.venv/bin/python3 scripts/traceability/traceability_server.py build/Debug/docs/traceability.db get_record_mappings EnergyRecord

# Check for record drift (C++ fields missing from pybind/Pydantic)
scripts/.venv/bin/python3 scripts/traceability/traceability_server.py build/Debug/docs/traceability.db check_record_drift

# Show all available commands
scripts/.venv/bin/python3 scripts/traceability/traceability_server.py build/Debug/docs/traceability.db --cli
```

## CMake Targets

| Target | Description | Dependencies |
|--------|-------------|--------------|
| `trace-git` | Index git history | None |
| `trace-symbols` | Index symbol snapshots | `trace-git` |
| `trace-decisions` | Index design decisions | `trace-git` |
| `trace-record-mappings` | Index cross-layer record mappings | `trace-git` |
| `traceability` | Build complete database | All four above |

Build preset: `cmake --build --preset debug-traceability`

## Database Schema

| Table | Description |
|-------|-------------|
| `snapshots` | One row per git commit (SHA, date, message, prefix, ticket, phase) |
| `file_changes` | Per-commit file diffs (path, change type, insertions, deletions) |
| `symbol_snapshots` | Symbol locations at each commit (qualified name, kind, file, line) |
| `symbol_changes` | Computed diffs between consecutive snapshots (added/removed/modified/moved) |
| `design_decisions` | Extracted decisions with DD ID, ticket, title, rationale, status |
| `decision_symbols` | Links decisions to affected symbol names |
| `decision_commits` | Links decisions to implementing commits |
| `design_decisions_fts` | FTS5 index over decision title, rationale, alternatives, trade-offs |
| `record_layer_fields` | Field lists across four layers (cpp/sql/pybind/pydantic) for each C++ record |
| `record_layer_mapping` | Cross-layer record name mapping (C++ record → Pydantic model → pybind class → SQL table) |
| `record_layer_fields_fts` | FTS5 index over record field names, types, and notes |

## Adding Structured Design Decisions

For new tickets, add explicit DD blocks to `docs/designs/{ticket}/design.md`:

```markdown
### DD-0045-001: Remove single-body solver path
- **Affects**: msd_sim::ConstraintSolver, msd_sim::SemiImplicitEulerIntegrator
- **Rationale**: Single-body path was redundant with quaternion normalization
- **Alternatives Considered**:
  - Keep both paths: Adds maintenance burden for no benefit
  - Merge into one path: Over-engineering for current needs
- **Trade-offs**: Breaking change to internal API
- **Status**: active
```

ID format: `DD-{ticket_number}-{3-digit sequence}` (e.g., `DD-0045-001`).

## Dependencies

- Python 3.12+ (via `scripts/.venv`)
- `mcp` package (for MCP server mode)
- `tree-sitter` + `tree-sitter-cpp` (for symbol indexing)
- Git (for commit history and worktree checkouts)
