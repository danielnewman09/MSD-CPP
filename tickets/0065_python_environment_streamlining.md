# Ticket 0065: Python Environment Streamlining

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Tooling / Infrastructure
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-16
**Generate Tutorial**: No
**Parent Ticket**: None
**Depends On**: None

---

## Overview

The project has fragmented Python environment management that causes failures on clean clones and across git worktrees. Currently there are multiple implicit Python environments with undocumented dependencies:

1. **`scripts/.venv`** — Used by traceability targets (`trace-git`, `trace-symbols`, `trace-decisions`, `trace-record-mappings`) and MCP servers. Requires `tree-sitter`, `tree-sitter-cpp`, `fastmcp`.
2. **`replay/.venv`** (or system Python) — Used by the replay FastAPI server. Requires `fastapi`, `uvicorn`, `pydantic`, `msd_reader`.
3. **CMake `${Python3_EXECUTABLE}`** — Used by `doxygen-db` and `python-db` targets. Whatever `find_package(Python3)` discovers (could be system Python, Xcode Python, or a venv).

This inconsistency means:
- `cmake --build --preset debug-all` fails if `scripts/.venv` doesn't exist
- New worktrees require manual, undocumented venv setup
- Different targets may use different Python interpreters with different packages available
- `.mcp.json` assumes `scripts/.venv` exists

---

## Requirements

### R1: Single Unified Python Environment

Consolidate all Python tooling into one venv at `python/.venv`. This single environment contains all dependencies for every Python context: traceability indexing, MCP servers, code generation, documentation indexing, replay server, and testing. No separate venvs.

### R2: Python Directory Structure

Create a `python/` directory at the project root as the home for all Python environment configuration:

```
python/
├── README.md              # Developer setup guide (the authoritative doc)
├── requirements.txt       # All pinned dependencies for the single venv
├── setup.sh               # Bootstrap script (idempotent)
└── .venv/                 # The single venv (gitignored)
```

`setup.sh` must:
- Create `python/.venv` from a clean state
- Install all packages from `python/requirements.txt`
- Install the `replay` package in editable mode (`pip install -e replay/`)
- Validate the environment is functional (import checks)
- Be idempotent (safe to re-run on an existing venv)

### R3: Requirements File

A single `python/requirements.txt` covering all contexts:

| Context | Dependencies |
|---------|-------------|
| Traceability indexing | `tree-sitter`, `tree-sitter-cpp` |
| MCP servers (codebase + traceability) | `fastmcp` |
| Code generation (`generate_record_layers.py`) | stdlib only |
| Documentation indexing (`doxygen_to_sqlite.py`, `python_to_sqlite.py`) | stdlib only |
| Replay server | `fastapi`, `uvicorn`, `pydantic` |
| Replay testing | `httpx`, `pytest` |
| Replay server runtime | `msd_reader` (C++ pybind11 module — added to `PYTHONPATH` from build dir, not pip-installed) |

### R4: CMake Uses the Documented Venv

All CMake Python targets use `python/.venv/bin/python3` — no `find_package(Python3)`, no `${Python3_EXECUTABLE}`, no `${TRACE_VENV}`. One interpreter, one venv, everywhere:

```cmake
set(PROJECT_PYTHON ${CMAKE_SOURCE_DIR}/python/.venv/bin/python3)
```

Remove the split between `${Python3_EXECUTABLE}` and `${TRACE_VENV}`. All targets (`doxygen-db`, `python-db`, `trace-git`, `trace-symbols`, `trace-decisions`, `trace-record-mappings`) use `${PROJECT_PYTHON}`.

### R5: Documentation in `python/`

`python/README.md` is the single authoritative document for Python environment setup. It must cover:
- Prerequisites (Python 3.x version requirement)
- How to set up from a clean clone (`python/setup.sh`)
- How to set up in a new git worktree
- How to add new Python dependencies (edit `requirements.txt`, re-run `setup.sh`)
- How `msd_reader` works (requires C++ build first, added via `PYTHONPATH`)
- How the replay server uses the venv
- How MCP servers use the venv

The root `CLAUDE.md` should add a brief section pointing to `python/README.md`.

### R6: Clean Clone Validation

The setup must work for:
- Fresh `git clone` with no existing venvs
- New `git worktree add` from an existing repo
- CI environment with only system Python available

---

## Acceptance Criteria

- [ ] **AC1**: A single command sets up all Python environments from a clean clone
- [ ] **AC2**: `cmake --build --preset debug-all` succeeds after running the setup command
- [ ] **AC3**: MCP servers in `.mcp.json` start successfully after setup
- [ ] **AC4**: All CMake Python targets use a consistent interpreter
- [ ] **AC5**: Requirements are pinned in version-controlled files
- [ ] **AC6**: Setup documentation exists and covers clean clone, worktree, and CI scenarios
- [ ] **AC7**: `replay/start_server.sh` works after setup (with msd_reader from C++ build)

---

## Technical Notes

### Current Pain Points

- `CMakeLists.txt:194` hardcodes `scripts/.venv/bin/python3` for traceability
- `CMakeLists.txt:160,172` uses `${Python3_EXECUTABLE}` for doxygen/python-db
- The deleted Python venv discovery block (lines 30-37 of the original CMakeLists.txt) was removed in the 0064 branch — this needs to be addressed
- `replay/pyproject.toml` defines replay package deps but doesn't cover scripts/ deps
- `replay/start_server.sh` creates its own venv independently
- `.mcp.json` references `scripts/.venv` which may not exist in a worktree

### Migration from `scripts/.venv`

- Move venv location from `scripts/.venv` to `python/.venv`
- Update `.mcp.json` to reference `python/.venv/bin/python3`
- Update `CMakeLists.txt` to use `${PROJECT_PYTHON}` everywhere
- Update `replay/start_server.sh` to use the shared venv instead of creating its own
- Update `.gitignore` to ignore `python/.venv/` (remove old `scripts/.venv/` entry if present)

### Scope

This ticket is infrastructure/tooling only. No changes to Python source code, only to:
- Build system (`CMakeLists.txt`, `CMakeUserPresets.json`)
- MCP config (`.mcp.json`)
- Setup scripts (`python/setup.sh` — new)
- Requirements files (`python/requirements.txt` — new)
- Documentation (`python/README.md` — new, `CLAUDE.md` — brief pointer)
- Replay startup (`replay/start_server.sh` — use shared venv)

---

## Workflow Log

| Date | Phase | Notes |
|------|-------|-------|
| 2026-02-16 | Draft | Ticket created to address fragmented Python environment management |
