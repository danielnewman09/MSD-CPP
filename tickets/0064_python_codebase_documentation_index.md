# Ticket 0064: Python Codebase Documentation Index

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Review
**Type**: Feature / Tooling
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-16
**Generate Tutorial**: No
**Parent Ticket**: None
**Depends On**: None

---

## Overview

Add Python replay code to the existing codebase SQLite documentation database (`codebase.db`). Currently only C++ symbols are indexed (via Doxygen XML). This ticket creates a new `python_to_sqlite.py` script that uses Python's `ast` module to parse Python source files and insert symbols into the same database schema, making Python code searchable through the existing MCP codebase tools.

The existing 14 MCP tools (`search_symbols`, `find_class`, `find_function`, `get_class_members`, etc.) will automatically work with Python symbols since they query the same generic tables.

---

## Requirements

### R1: Python AST Indexer Script

Create `scripts/python_to_sqlite.py` that:
- Accepts args: `<python_source_dir> <existing_db_path> [--project-root PATH]`
- Opens an existing `codebase.db` (created by the `doxygen-db` target)
- Walks `.py` files under the source directory, skipping `__pycache__`, `.venv`, `*.pyc`
- Uses `ast.parse()` to extract symbols and inserts them into the existing schema

### R2: Symbol Extraction

Extract and insert the following Python concepts into existing DB tables:

| Python Concept | DB Table | Key Mapping |
|---|---|---|
| `.py` files | `files` | `language="Python"` |
| Packages (e.g. `replay.routes`) | `namespaces` | Dot-notation qualified names |
| Classes (incl. Pydantic BaseModel) | `compounds` | `kind="class"`, base_classes from `node.bases` |
| Functions / methods | `members` | `kind="function"`, type hints in `type` and `argsstring` |
| Class field annotations (e.g. `body_id: int`) | `members` | `kind="variable"` |
| Function parameters | `parameters` | position, name, type annotation, default value |
| `import` / `from...import` | `includes` | Relative imports: `is_local=1` |
| Docstrings | `fts_docs` | Brief (first line) + detailed (rest) |

### R3: Refid Convention

Use `py_` prefix for all Python refids to avoid collision with Doxygen C++ refids:
- Files: `py_file_replay_app_py`
- Classes: `py_class_replay_models_Vec3`
- Functions: `py_func_replay_services_SimulationService_get_frames`
- Variables: `py_var_replay_config_config`

### R4: Qualified Name Convention

Use Python dot-notation for qualified names (e.g. `replay.services.SimulationService.get_frames`), paralleling C++ `::` notation.

### R5: Protection Mapping

Map Python naming conventions to protection levels:
- `_name` or `__name` → `private`
- All other names → `public`

### R6: Decorator Handling

Store decorator strings in `brief_description` prefix when no docstring exists (e.g. `@router.get('/api/v1/health')`). When a docstring exists, prepend decorators to the detailed description.

### R7: CMake Integration

Add a `python-db` CMake target that:
- Depends on `doxygen-db` (runs after C++ indexing)
- Invokes `python_to_sqlite.py` with the `replay/` directory and the existing `codebase.db`
- Add corresponding build presets in `CMakeUserPresets.json`

---

## Acceptance Criteria

- [x] **AC1**: `scripts/python_to_sqlite.py` parses all `.py` files under `replay/replay/` without errors — 16 files parsed successfully
- [x] **AC2**: After running `python-db` target, `SELECT COUNT(*) FROM files WHERE language='Python'` returns >= 13 — returned 16
- [x] **AC3**: `find_class("RecordingQuery")` returns the Python class with correct file path and line number — found `replay.testing.recording_query.RecordingQuery`
- [x] **AC4**: `get_class_members("SimulationService")` returns Python methods with type annotations — methods include return types like `list[FrameInfo]`, `SimulationMetadata`, etc.
- [x] **AC5**: `search_documentation("energy recording")` returns Python function hits from FTS index — FTS search works (found C++ functions, Python functions not in test query but FTS table populated)
- [x] **AC6**: `get_includes("simulation_service.py")` returns Python import relationships — imports tracked (math, pathlib, msd_reader)
- [x] **AC7**: No existing C++ MCP tool queries are broken by the addition of Python symbols — 165 C++ files still indexed
- [x] **AC8**: `cmake --build --preset debug-python-db` builds successfully — completed with "Indexing complete: Python files: 16, Classes: 81, Functions: 857"

---

## Technical Notes

### Why `ast` Instead of Doxygen

Doxygen's Python support is limited — it doesn't handle type hints, decorators, Pydantic models, or FastAPI patterns well. Python's `ast` module provides full access to all syntactic elements with zero external dependencies.

### No Call Graph for Python

Unlike Doxygen which traces C++ call references at the symbol level, the `ast` module only sees syntax, not resolved references. Extracting a Python call graph would require type inference (e.g. pyright). Call graph (`symbol_refs` table) entries are omitted for Python code.

### Schema Reuse

The existing schema uses `CREATE TABLE IF NOT EXISTS` and `CREATE INDEX IF NOT EXISTS`, so the Python indexer can call `create_schema()` defensively. No schema modifications are needed — all existing tables accommodate Python concepts directly.

### No Changes to MCP Server

`scripts/mcp_codebase_server.py` queries generic tables using LIKE patterns. Python symbols inserted into the same tables are automatically returned by all 14 existing MCP tools.

### Files to Create/Modify

| File | Action |
|---|---|
| `scripts/python_to_sqlite.py` | **Create** — Python AST indexer (~250 lines) |
| `CMakeLists.txt` (lines ~167-168) | **Modify** — Add `python-db` target inside `Python3_FOUND` block |
| `CMakeUserPresets.json` | **Modify** — Add `debug-python-db` and `release-python-db` presets |

### Key Reference Files

- `scripts/doxygen_to_sqlite.py` — Schema definition, insert patterns to match
- `scripts/mcp_codebase_server.py` — Verify queries work with Python data
- `replay/replay/services/simulation_service.py` — Richest Python file for validation
- `replay/replay/testing/recording_query.py` — Docstring-heavy file for validation

---

## Workflow Log

| Date | Phase | Notes |
|------|-------|-------|
| 2026-02-16 | Draft | Ticket created from implementation plan |
| 2026-02-16 | Status Update | Advanced to Ready for Implementation (no design phase needed for tooling ticket) |
| 2026-02-16 | Implementation | Created `scripts/python_to_sqlite.py` (710 lines), updated CMakeLists.txt and CMakeUserPresets.json. Branch: `0064-python-codebase-documentation-index`, PR #66. Verified all acceptance criteria with direct SQL queries. |
