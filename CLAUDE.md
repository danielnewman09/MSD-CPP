# MSD-CPP Project Guide

> This document provides high-level context for AI assistants and developers working on the MSD-CPP repository.

## Project Overview

MSD-CPP (Multi-Spacecraft Dynamics) is a C++ project for spacecraft dynamics simulation and visualization. The core libraries are located in the [`msd/`](msd/) directory.

**For detailed library architecture and component documentation, see [`msd/CLAUDE.md`](msd/CLAUDE.md).**

---

## Repository Structure

```
MSD-CPP/
├── msd/                      # Core libraries (see msd/CLAUDE.md)
│   ├── msd-transfer/         # Database transfer objects (DTOs)
│   ├── msd-assets/           # Asset management and geometry factories
│   ├── msd-sim/              # Physics simulation engine
│   ├── msd-gui/              # GPU-accelerated 3D rendering
│   ├── msd-exe/              # Main executable
│   └── msd-asset-gen/        # Asset generation tool
│
├── docs/                     # Documentation
│   ├── designs/              # Feature design documents and PlantUML diagrams
│   ├── msd/                  # Library-specific diagrams
│   ├── workflows/            # Development workflow documentation
│   ├── benchmarking.md       # Benchmarking guide
│   └── profiling.md          # Profiling guide (macOS)
│
├── tickets/                  # Feature tickets (see Ticketing System below)
├── prototypes/               # Prototype code for design validation
│
├── scripts/                  # Tooling and automation
│   ├── generate_record_layers.py  # Auto-generates pybind11 bindings and Pydantic models from C++ records
│   └── traceability/         # Design decision traceability (see scripts/traceability/README.md)
│
├── analysis/                 # Performance analysis infrastructure (see analysis/CLAUDE.md)
│   ├── scripts/              # Benchmarking and profiling scripts
│   ├── benchmark_baselines/  # Golden baselines for benchmark comparison
│   └── profile_baselines/    # Golden baselines for profiling comparison
│
├── build/                    # CMake build output (gitignored)
├── conan/                    # Conan package manager configuration
├── test/                     # Integration test resources
│
├── CMakeLists.txt            # Root CMake configuration
├── CMakeUserPresets.json     # Build presets for component builds
├── conanfile.py              # Conan dependency specification
└── CLAUDE.md                 # This file
```

---

## Ticketing System

The project uses a ticket-based workflow for feature development. Tickets live in [`tickets/`](tickets/) and follow a structured format.

### Ticket Lifecycle

1. **New** — Ticket created, requirements defined
2. **Design** — Architectural design in `docs/designs/{ticket-name}/`
3. **Design Review** — Design reviewed and approved
4. **Prototype** — Validation code in `prototypes/{ticket-name}/`
5. **Implementation** — Production code written
6. **Implementation Review** — Code reviewed
7. **Documentation** — CLAUDE.md and diagrams updated
8. **Complete** — Ticket closed

### Ticket Naming Convention

Tickets use a numeric prefix for ordering: `NNNN_descriptive_name.md`

Examples:
- `0011_add_google_benchmark.md`
- `0015_profiling_trace_parser.md`

### Design Documents

Each ticket with architectural changes has a design folder:
```
docs/designs/{ticket-name}/
├── design.md                 # Architectural design document
├── {ticket-name}.puml        # PlantUML diagram
└── prototype-results.md      # Prototype findings (if applicable)
```

---

## Build & Configuration

### Build Requirements
- C++ Standard: C++20
- Compiler: GCC 11+, Clang 14+, or MSVC 2019+
- Build System: CMake 3.15+ with Conan 2.x package manager
- Dependencies: Managed via Conan (see `conanfile.py`)

### Build Process

This project uses **Conan** for dependency management and **CMake presets** for build configuration.

#### Prerequisites: Install Dependencies with Conan

**IMPORTANT**: Before building, you must run Conan to install dependencies and generate CMake configuration files:

```bash
# For Debug build
conan install . --build=missing -s build_type=Debug

# For Release build
conan install . --build=missing -s build_type=Release

# For Debug build with code coverage enabled
conan install . --build=missing -s build_type=Debug -o "&:enable_coverage=True"

# For Release build with benchmarks enabled
conan install . --build=missing -s build_type=Release -o "&:enable_benchmarks=True"
```

This generates CMake preset files in `build/Debug/generators/CMakePresets.json` and `build/Release/generators/CMakePresets.json` which are referenced by [`CMakeUserPresets.json`](CMakeUserPresets.json).

#### Building the Entire Project

After running `conan install`, configure and build:

```bash
# Configure with Debug preset
cmake --preset conan-debug

# Build everything (Debug)
cmake --build --preset conan-debug

# Or for Release
cmake --preset conan-release
cmake --build --preset conan-release
```

#### Building Specific Components

The project provides component-specific build presets in [`CMakeUserPresets.json`](CMakeUserPresets.json) to build individual libraries without building the entire project:

**Debug builds:**
```bash
cmake --build --preset debug-utils-only      # msd-utils library + tests
cmake --build --preset debug-transfer-only   # msd-transfer interface library
cmake --build --preset debug-sim-only        # msd-sim library + tests
cmake --build --preset debug-assets-only     # msd-assets library + tests
cmake --build --preset debug-gui-only        # msd-gui library
cmake --build --preset debug-exe-only        # msd-exe executable
cmake --build --preset debug-asset-gen-only  # generate_assets executable
cmake --build --preset debug-tests-only      # All test targets only
```

**Release builds:**
```bash
cmake --build --preset release-utils-only      # msd-utils library + tests
cmake --build --preset release-transfer-only   # msd-transfer interface library
cmake --build --preset release-sim-only        # msd-sim library + tests
cmake --build --preset release-assets-only     # msd-assets library + tests
cmake --build --preset release-gui-only        # msd-gui library
cmake --build --preset release-exe-only        # msd-exe executable
cmake --build --preset release-asset-gen-only  # generate_assets executable
cmake --build --preset release-tests-only      # All test targets only
```

### Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_TESTING` | `ON` | Build the testing tree |
| `ENABLE_COVERAGE` | `OFF` | Enable code coverage (GCC/Clang only) |
| `ENABLE_BENCHMARKS` | `OFF` | Build performance benchmarks (requires Conan option) |
| `ENABLE_PROFILING` | `OFF` | Enable profiling support with debug symbols (macOS only, requires Conan option) |

---

## Python Environment

The project uses a unified Python virtual environment for all Python tooling: traceability indexing, MCP servers, replay server, testing, code generation, and documentation indexing.

### Quick Setup

```bash
# From a clean clone or new worktree
./python/setup.sh
```

This creates `python/.venv` and installs all dependencies from `python/requirements.txt`.

### What's Included

- **Traceability indexing:** tree-sitter, tree-sitter-cpp
- **MCP servers:** fastmcp (for codebase, traceability, and guidelines servers); pyyaml (for guidelines seeder)
- **Replay server:** fastapi, uvicorn, pydantic
- **Testing:** pytest, httpx
- **Replay package:** Installed in editable mode

**Note:** The `msd_reader` C++ pybind11 module is not pip-installed. It is made available via `PYTHONPATH` from the build directory. See [`python/README.md`](python/README.md) for details.

### Full Documentation

For complete setup instructions, troubleshooting, and usage in different contexts (clean clone, git worktree, CI), see [`python/README.md`](python/README.md).

---

## Testing

### Test Organization
```
msd/
├── msd-assets/test/     # Unit tests for asset management
├── msd-sim/test/        # Unit tests for simulation engine
└── msd-gui/test/        # Integration tests through msd-exe
```

### Running Tests
```bash
# All tests
cmake --build --preset conan-debug --target test

# Library-specific tests
cmake --build --preset debug-assets-only --target msd_assets_test
cmake --build --preset debug-sim-only --target msd_sim_test
```

### Test Conventions
- Test files mirror source structure: `src/foo/bar.cpp` → `test/foo/bar_test.cpp`
- Ticket references in test descriptions: `TEST_CASE("ClassName: behavior [ticket-name]")`

---

## Code Quality

The project provides infrastructure for performance benchmarking and profiling to detect regressions and identify optimization opportunities.

**For scripts and detailed usage, see [`analysis/CLAUDE.md`](analysis/CLAUDE.md).**

### Benchmarking

Uses Google Benchmark for micro-benchmarking performance-critical code paths. Benchmarks are optional and disabled by default to avoid extending build times.

- **Purpose**: Measure execution time of critical operations (e.g., ConvexHull construction, collision detection)
- **Regression Detection**: Compare results against golden baselines to catch performance regressions
- **Full documentation**: [`docs/benchmarking.md`](docs/benchmarking.md)

### Profiling (macOS)

Uses Xcode Instruments for CPU profiling and memory analysis on macOS. Provides deep call graph visualization and hotspot identification.

- **Purpose**: Identify performance bottlenecks and memory issues in production code
- **Regression Detection**: Track function-level CPU usage changes over time
- **Full documentation**: [`docs/profiling.md`](docs/profiling.md)

### Doxygen Documentation

Generates API documentation from source code comments. Automatically enabled when Doxygen is installed.

```bash
# Generate documentation (after cmake --preset conan-debug)
cmake --build --preset doxygen

# Or using the target directly
cmake --build build/Debug --target doxygen
```

Output is generated to `build/{build_type}/docs/html/`.

### Codebase SQLite Database

Generates a SQLite database from Doxygen XML output for programmatic codebase navigation. This enables efficient symbol search, call graph queries, and documentation lookup.

```bash
# Generate documentation and SQLite database
cmake --build --preset doxygen-db

# Or using the target directly
cmake --build build/Debug --target doxygen-db
```

Database is generated at `build/{build_type}/docs/codebase.db`.

### Traceability Database

Indexes design decisions from ticket artifacts, snapshots symbol locations at each git commit, and correlates both with git history. Exposed as MCP tools alongside the codebase database.

```bash
# Build the full traceability database (~30s from scratch, seconds for incremental updates)
cmake --build --preset debug-traceability

# Or run indexers individually
cmake --build build/Debug --target trace-git         # Git history
cmake --build build/Debug --target trace-symbols      # Symbol snapshots (tree-sitter)
cmake --build build/Debug --target trace-decisions     # Design decision extraction
```

Database is generated at `build/{build_type}/docs/traceability.db` (gitignored, rebuilt from repo contents). Requires `tree-sitter` and `tree-sitter-cpp` in `scripts/.venv`.

**Full documentation**: [`scripts/traceability/README.md`](scripts/traceability/README.md)

### Record Layer Code Generation

Automates generation of pybind11 bindings and Pydantic leaf models from msd-transfer C++ record headers. Use the `/sync-records` skill after modifying transfer records, or run `python scripts/generate_record_layers.py` directly.

**Full documentation**: [`msd/msd-pybind/CLAUDE.md`](msd/msd-pybind/CLAUDE.md)

### Guidelines MCP Server

Stores structured C++ coding guidelines (project conventions, C++ Core Guidelines, MISRA) in a SQLite + FTS5 database and exposes them via a FastMCP server. AI agents query it during design and code review to retrieve rationale-backed recommendations with traceable rule IDs. This is a lightweight RAG system: FTS5 handles retrieval, MCP handles augmentation, and the AI handles generation.

```bash
# Seed the guidelines database from YAML source files
cmake --build --preset debug-guidelines

# Or directly
python scripts/guidelines/seed_guidelines.py --db build/Debug/docs/guidelines.db

# CLI smoke test
python scripts/guidelines/guidelines_server.py build/Debug/docs/guidelines.db search_guidelines "brace initialization"
python scripts/guidelines/guidelines_server.py build/Debug/docs/guidelines.db get_rule MSD-RES-001
python scripts/guidelines/guidelines_server.py build/Debug/docs/guidelines.db list_categories
```

Database is generated at `build/Debug/docs/guidelines.db` (gitignored, fully rebuildable from YAML seed files in `scripts/guidelines/data/`).

**MCP Tools** (available when server is registered in `.mcp.json`):
| Tool | Description |
|------|-------------|
| `search_guidelines` | FTS5 full-text search with porter stemming (BM25 ranking) |
| `get_rule` | Full rule details including rationale, examples, tags, and cross-references |
| `list_categories` | All categories with total and active rule counts |
| `get_category` | Rules in a category — summary mode (default) or detailed |
| `get_rules_by_tag` | All rules with a given tag (e.g., "safety", "ownership") |

**Rule ID Conventions**:
- Project rules: `MSD-{CATEGORY}-{NNN}` (e.g., `MSD-INIT-001`, `MSD-RES-002`)
- C++ Core Guidelines: `CPP-{section}.{number}` (e.g., `CPP-R.11`) — 57 rules across R, C, and ES sections ([Ticket: 0078b](tickets/0078b_cpp_core_guidelines_population.md))
- MISRA: `MISRA-{rule}` (e.g., `MISRA-21.3`) — 19 rules across Memory Management and Initialization categories ([Ticket: 0078c](tickets/0078c_misra_rules_population.md))
- Clang-tidy: `TIDY-{group}-{check}` (e.g., `TIDY-bugprone-use-after-move`, `TIDY-modernize-use-override`) — 41 rules (30 active + 11 deprecated) across 9 check groups, with `enforcement_check` mappings and CheckOptions documentation ([Ticket: 0078e](tickets/0078e_clang_tidy_rules_population.md))

**Total rules**: 127 across 18 categories (10 MSD-*, 57 CPP-*, 19 MISRA-*, 41 TIDY-*)

**Source files**: `scripts/guidelines/` — see `guidelines_schema.py` (schema), `seed_guidelines.py` (YAML→SQLite indexer), `guidelines_server.py` (FastMCP server + CLI)

**Agent Integration**: The following agents query the guidelines server during their workflows ([Ticket: 0078a](tickets/0078a_agent_prompt_guidelines_integration.md)) with severity-aware enforcement ([Ticket: 0078f](tickets/0078f_severity_aware_guideline_enforcement.md)):

| Agent | Integration Point | Tools Used | Severity Enforcement |
|-------|-------------------|------------|----------------------|
| `cpp-architect` | Before finalizing design decisions; required-rules discovery at design start | `search_guidelines`, `get_rule`, `list_categories`, `get_category` | Lists `required` rules as design constraints; maps `required → BLOCKING` |
| `design-reviewer` | Step 1 (context gathering) + Step 5 (required-rules compliance sweep before verdict) | `search_guidelines`, `get_rule`, `get_category` | Systematic sweep via `get_category(detailed=True)`; any `required` violation is BLOCKING |
| `cpp-code-reviewer` | Before and during review; required-rules sweep after pattern-based review | `search_guidelines`, `get_rule`, `get_category` | Queries `search_guidelines(severity="required")`; any new violation is BLOCKING |
| `implementation-reviewer` | Phase 2.5 (between prototype learning and code quality); required-rules compliance sweep | `search_guidelines`, `get_rule`, `get_category` | Systematic sweep via `get_category(detailed=True)`; any `required` violation is BLOCKING |

**Severity enforcement policy** (applied by all four agents):

| Guideline Severity | Minimum Finding Severity | Review Impact |
|--------------------|--------------------------|---------------|
| `required`         | BLOCKING                 | Cannot approve with open violations |
| `recommended`      | MAJOR                    | Should fix before merge; document if deferred |
| `advisory`         | MINOR or NIT             | Discretionary; cite for awareness |

All agents include the constraints: **Only cite rules returned by `search_guidelines`. Do not invent rule IDs.** When citing a rule, always include its severity (e.g., `MSD-INIT-001 (required) → BLOCKING`).

**Ticket**: [0078_cpp_guidelines_mcp_server](tickets/0078_cpp_guidelines_mcp_server.md)

---

## Coding Standards

Coding standards are managed in the **Guidelines MCP Server** (127 rules across Project, C++ Core Guidelines, MISRA, and clang-tidy sources).
Query the server for authoritative rules, rationale, and examples:
- `search_guidelines("<topic>")` — find rules by keyword (e.g., `"brace initialization"`, `"ownership"`)
- `get_rule("<rule_id>")` — full rule with rationale, examples, and cross-references
- `get_category("<name>")` — all rules in a category (e.g., `"Initialization"`, `"Resource Management"`)

Key rule categories: Initialization, Resource Management, Naming Conventions, Function Design, Code Organization, Documentation.

See the **Guidelines MCP Server** section under Code Quality for setup, tool reference, and rule ID conventions.

### General Naming Conventions

- Classes: `PascalCase`
- Functions/Methods: `camelCase`
- Member variables: `snake_case_` (trailing underscore)
- Constants: `kPascalCase`
- Namespaces: `snake_case`

### Code Organization

- One class per header (generally)
- Implementation in `.cpp` unless template/inline
- Headers in `src/` (no separate include directory for MSD libraries)

### Documentation

- Public APIs: Doxygen-style comments
- Ticket references: `// Ticket: {ticket-name}` for non-obvious implementations
- PlantUML diagrams for architectural components

---

## Getting Help

### For AI Assistants

1. Start with this document for repository-level context
2. See [`msd/CLAUDE.md`](msd/CLAUDE.md) for library architecture and component details
3. Check `tickets/` for feature history and requirements
4. Check `docs/designs/{feature}/design.md` for detailed design rationale
5. Use traceability MCP tools (`search_decisions`, `why_symbol`, `get_ticket_impact`) to trace design decisions to code

### For Developers

- Library documentation: [`msd/CLAUDE.md`](msd/CLAUDE.md)
- Benchmarking guide: [`docs/benchmarking.md`](docs/benchmarking.md)
- Profiling guide: [`docs/profiling.md`](docs/profiling.md)
- Design documents: `docs/designs/`
- Tickets with full context: `tickets/`
- Traceability tools: [`scripts/traceability/README.md`](scripts/traceability/README.md)
