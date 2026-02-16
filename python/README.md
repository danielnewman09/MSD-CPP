# Python Environment Setup

**Ticket:** 0065_python_environment_streamlining

This directory contains the unified Python environment configuration for the entire MSD-CPP project. All Python tooling—traceability indexing, MCP servers, replay server, testing, code generation, and documentation indexing—uses a single virtual environment at `python/.venv`.

---

## Quick Start

### From a Clean Clone

```bash
# 1. Set up Python environment
./python/setup.sh

# 2. Build C++ project (required for msd_reader pybind11 module)
conan install . --build=missing -s build_type=Debug
cmake --preset conan-debug
cmake --build --preset conan-debug

# 3. Run traceability indexing (optional)
cmake --build --preset debug-traceability

# 4. Start replay server (optional)
./replay/start_server.sh
```

### From a Git Worktree

If you've created a new worktree from an existing repo, the Python venv won't exist in the new worktree. Run the setup script to create it:

```bash
# In your new worktree
./python/setup.sh
```

---

## What's Included

The unified venv at `python/.venv` contains dependencies for:

| Context | Dependencies | Purpose |
|---------|--------------|---------|
| **Traceability Indexing** | `tree-sitter`, `tree-sitter-cpp` | Parse C++ source for symbol snapshots |
| **MCP Servers** | `fastmcp` | Serve codebase and traceability databases to Claude Code |
| **Replay Server** | `fastapi`, `uvicorn`, `pydantic` | FastAPI backend for simulation replay visualization |
| **Testing** | `pytest`, `httpx` | Unit tests and integration tests for replay server |
| **Code Generation** | stdlib only | `generate_record_layers.py` uses stdlib |
| **Documentation Indexing** | stdlib only | `doxygen_to_sqlite.py`, `python_to_sqlite.py` use stdlib |
| **Replay Package** | Installed in editable mode (`pip install -e replay/`) | Python backend package |

### Special Case: `msd_reader` (C++ pybind11 module)

The `msd_reader` module is a C++ pybind11 extension built by CMake, **not** a pip-installable package. It is made available to Python by adding the build directory to `PYTHONPATH`:

```bash
export PYTHONPATH="$PROJECT_ROOT/build/Debug/debug:$PYTHONPATH"
```

The replay server startup script (`replay/start_server.sh`) handles this automatically.

---

## Directory Structure

```
python/
├── README.md              # This file (authoritative setup guide)
├── requirements.txt       # All pinned dependencies
├── setup.sh               # Bootstrap script (idempotent)
└── .venv/                 # Virtual environment (gitignored)
```

---

## Setup Script

### Usage

```bash
# Create or update the venv (idempotent)
./python/setup.sh

# Remove and recreate from scratch
./python/setup.sh --clean
```

### What It Does

1. Creates `python/.venv` if it doesn't exist
2. Upgrades pip
3. Installs all dependencies from `python/requirements.txt`
4. Installs the `replay` package in editable mode (`pip install -e replay/`)
5. Validates that all imports work (tree_sitter, fastmcp, fastapi, uvicorn, pydantic, replay, pytest, httpx)
6. Reports a summary of installed packages

### Idempotency

The script is safe to re-run. If `python/.venv` already exists, it will skip creation and just ensure packages are up-to-date.

---

## Adding New Dependencies

1. Edit `python/requirements.txt` to add the new package with a pinned version
2. Re-run the setup script:
   ```bash
   ./python/setup.sh
   ```
3. Commit `python/requirements.txt` to version control

---

## How CMake Uses the Venv

All CMake Python targets use the unified venv via `${PROJECT_PYTHON}`:

```cmake
set(PROJECT_PYTHON ${CMAKE_SOURCE_DIR}/python/.venv/bin/python3)
```

Targets that use this:
- `doxygen-db` — Convert Doxygen XML to SQLite
- `python-db` — Add Python symbols to codebase database
- `trace-git` — Index git history
- `trace-symbols` — Index symbol snapshots
- `trace-decisions` — Index design decisions
- `trace-record-mappings` — Index record layer mappings

If CMake reports that the venv is not found, run `python/setup.sh`.

---

## How MCP Servers Use the Venv

The `.mcp.json` file at the project root references the unified venv:

```json
{
  "mcpServers": {
    "codebase": {
      "command": "python/.venv/bin/python3",
      "args": ["scripts/mcp_codebase_server.py", "build/Debug/docs/codebase.db"]
    },
    "traceability": {
      "command": "python/.venv/bin/python3",
      "args": [
        "scripts/traceability/traceability_server.py",
        "build/Debug/docs/traceability.db",
        "--codebase-db",
        "build/Debug/docs/codebase.db"
      ]
    }
  }
}
```

Both servers require `fastmcp`, which is installed by `python/setup.sh`.

---

## How the Replay Server Uses the Venv

The replay server startup script (`replay/start_server.sh`) activates the unified venv and verifies the `replay` package is installed:

```bash
source "$PROJECT_ROOT/python/.venv/bin/activate"
```

The script also adds the C++ build directory to `PYTHONPATH` so the `msd_reader` pybind11 module is available.

---

## CI Environment

In a CI environment with only system Python available:

```bash
# Install dependencies from requirements.txt
python3 -m venv python/.venv
python/.venv/bin/pip install -r python/requirements.txt
python/.venv/bin/pip install -e replay/

# Build C++ project
conan install . --build=missing -s build_type=Debug
cmake --preset conan-debug
cmake --build --preset conan-debug

# Run CMake Python targets
cmake --build --preset debug-traceability
```

---

## Troubleshooting

### `python/.venv` not found

**Symptom:** CMake reports `Python venv not found at python/.venv/bin/python3`

**Solution:** Run the setup script:
```bash
./python/setup.sh
```

### Import errors (tree_sitter, fastmcp, etc.)

**Symptom:** Scripts fail with `ModuleNotFoundError`

**Solution:** The venv may be incomplete or corrupted. Recreate it:
```bash
./python/setup.sh --clean
```

### `msd_reader` not found

**Symptom:** Replay server fails with `ImportError: No module named 'msd_reader'`

**Solution:** The C++ project must be built first. The `msd_reader` module is a pybind11 extension:
```bash
conan install . --build=missing -s build_type=Debug
cmake --preset conan-debug
cmake --build --preset conan-debug
```

Then ensure `PYTHONPATH` includes the build directory (the replay server startup script does this automatically):
```bash
export PYTHONPATH="$PROJECT_ROOT/build/Debug/debug:$PYTHONPATH"
```

### MCP servers fail to start

**Symptom:** Claude Code reports that MCP servers cannot start

**Solution:** Ensure the venv is set up and the databases are built:
```bash
./python/setup.sh
cmake --preset conan-debug
cmake --build --preset doxygen-db
cmake --build --preset debug-traceability
```

---

## Migration from `scripts/.venv`

This unified environment replaces the previous fragmented setup:
- **Old:** `scripts/.venv` for traceability, system Python or `.venv` for replay server
- **New:** `python/.venv` for everything

If you have an old `scripts/.venv`, you can safely remove it:
```bash
rm -rf scripts/.venv
```

---

## Summary

- **One venv for everything:** `python/.venv`
- **One setup command:** `./python/setup.sh`
- **One requirements file:** `python/requirements.txt`
- **CMake, MCP servers, and replay server all use the same venv**
- **`msd_reader` is not pip-installed; it's added via `PYTHONPATH` from the C++ build**

For questions or issues, see the troubleshooting section or check the root `CLAUDE.md` for project-wide context.
