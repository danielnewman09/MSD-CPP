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
- **MCP servers:** fastmcp (for codebase and traceability servers)
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

---

## Coding Standards

### Initialization and Construction

#### Uninitialized Member Variables
- **Always** use `std::numeric_limits<T>::quiet_NaN()` for default/uninitialized floating-point values
- **Never** use magic numbers like `-1.0f` or `0.0f` to represent uninitialized state
- **Rationale**: NaN propagates through calculations and makes uninitialized access immediately obvious

```cpp
// GOOD
class Example {
private:
  float volume_{std::numeric_limits<float>::quiet_NaN()};
  float area_{std::numeric_limits<float>::quiet_NaN()};
};

// BAD
class Example {
private:
  float volume_{-1.0f};  // Magic number - unclear if -1 is valid or uninitialized
  float area_{0.0f};      // Could be confused with actual zero value
};
```

#### Brace Initialization
- **Always** use brace initialization `{}` for constructing objects
- **Never** use parentheses `()` for initialization
- **Rationale**: Avoids the Most Vexing Parse problem and provides consistent syntax

```cpp
// GOOD
Coordinate point{1.0f, 2.0f, 3.0f};
std::vector<int> values{1, 2, 3};
auto hull = ConvexHull{points};

// BAD
Coordinate point(1.0f, 2.0f, 3.0f);  // Can be confused with function declaration
std::vector<int> values(10, 0);      // Ambiguous syntax
auto hull = ConvexHull(points);      // Most Vexing Parse risk
```

### All-or-Nothing Rule
- **If** you implement any special member function, implement ALL of them (Rule of Five)
- **Prefer** implementing NONE (Rule of Zero)
- **Never** implement only some special member functions - this leads to subtle bugs and inconsistent behavior
- **Rationale**: If a class needs custom behavior for one operation (e.g., copy), it likely needs custom behavior for related operations (e.g., move, destructor)

```cpp
// GOOD - All-or-Nothing: Implement all five
class ResourceOwner {
public:
  ResourceOwner() : data_{new int[100]} {}
  ~ResourceOwner() { delete[] data_; }

  ResourceOwner(const ResourceOwner& other)
    : data_{new int[100]} {
    std::copy(other.data_, other.data_ + 100, data_);
  }

  ResourceOwner(ResourceOwner&& other) noexcept
    : data_{other.data_} {
    other.data_ = nullptr;
  }

  ResourceOwner& operator=(const ResourceOwner& other) {
    if (this != &other) {
      std::copy(other.data_, other.data_ + 100, data_);
    }
    return *this;
  }

  ResourceOwner& operator=(ResourceOwner&& other) noexcept {
    if (this != &other) {
      delete[] data_;
      data_ = other.data_;
      other.data_ = nullptr;
    }
    return *this;
  }

private:
  int* data_;
};

// GOOD - All-or-Nothing: Implement none (use RAII wrapper)
class SafeResourceOwner {
public:
  SafeResourceOwner() : data_(100) {}
  // Compiler generates correct copy/move/destructor automatically

private:
  std::vector<int> data_;  // RAII wrapper handles resource management
};

// BAD - Partial implementation leads to bugs
class BrokenResourceOwner {
public:
  BrokenResourceOwner() : data_{new int[100]} {}
  ~BrokenResourceOwner() { delete[] data_; }
  // Missing copy constructor - compiler generates shallow copy (double-free!)
  // Missing move constructor - inefficient copies instead of moves
  // Missing assignment operators - resource leaks on assignment

private:
  int* data_;
};
```

### Memory Management
- **Ownership Transfer**: Use `std::unique_ptr` for exclusive ownership and transfer
- **Non-Owning Access**: Prefer plain references (`const T&` or `T&`) for non-owning access
- **Shared Ownership**: Avoid `std::shared_ptr` - prefer establishing clear ownership hierarchies with references
- **Never** use raw pointers - they expose memory leaks and unclear ownership
- **Value Semantics**: Prefer value semantics for member variables where possible
- **Rationale**: References enforce proper ownership traceability, establish clear memory allocation hierarchy, and are more efficient than shared pointers

```cpp
// GOOD - Clear ownership with references
class MeshRenderer {
public:
  // Constructor takes non-owning reference
  explicit MeshRenderer(const AssetRegistry& registry)
    : registry_{registry} {}

  void render(const std::string& meshName) {
    // Access through reference
    if (auto mesh = registry_.getCachedMesh(meshName)) {
      // Render mesh...
    }
  }

private:
  const AssetRegistry& registry_;  // Non-owning reference
};

// BAD - Shared pointer obscures ownership
class MeshRenderer {
public:
  explicit MeshRenderer(std::shared_ptr<AssetRegistry> registry)
    : registry_{std::move(registry)} {}  // Unclear who really owns this

private:
  std::shared_ptr<AssetRegistry> registry_;  // Avoid shared ownership
};
```

### Naming Conventions
- **Don't** use `cached` prefix for member variables unless the value is truly cached (lazily computed)
- **Use** descriptive names that indicate the value's purpose
- **Distinguish** between computed-once values and lazily-cached values

```cpp
// GOOD
class ConvexHull {
private:
  float volume_;                    // Computed once by Qhull
  float surfaceArea_;               // Computed once by Qhull
  mutable Coordinate centroid_;     // Lazily computed
  mutable bool centroidValid_;      // Cache validity flag
};

// BAD
class ConvexHull {
private:
  float cachedVolume_;              // Misleading - not cached, computed once
  float cachedSurfaceArea_;         // Misleading - not cached, computed once
  Coordinate cachedCentroid_;       // Correct - this IS cached
};
```

### Function Return Values
- **Prefer** returning values over modifying parameters passed by reference
- **Use** return values or return structs instead of output parameters
- **Rationale**: Makes code more functional, easier to reason about, and prevents accidental modifications

```cpp
// GOOD - Return a struct
struct BoundingBox {
  Coordinate min;
  Coordinate max;
};

BoundingBox getBoundingBox() const {
  return BoundingBox{boundingBoxMin_, boundingBoxMax_};
}

auto bbox = hull.getBoundingBox();
float width = bbox.max.x() - bbox.min.x();

// BAD - Modify parameters by reference
void getBoundingBox(Coordinate& min, Coordinate& max) const {
  min = boundingBoxMin_;
  max = boundingBoxMax_;
}

Coordinate min, max;
hull.getBoundingBox(min, max);  // Harder to understand data flow
float width = max.x() - min.x();
```

### Non-Owning Optional References
- **Use** `std::optional<std::reference_wrapper<const T>>` ONLY for returning non-owning references that are truly optional (may not exist)
- **Prefer** plain references (`const T&` or `T&`) in most circumstances for non-owning access
- **Rationale**: Plain references enforce ownership traceability, establish clear memory allocation hierarchy, and are more efficient than both `std::shared_ptr` and `std::optional<std::reference_wrapper>`
- **Never** use raw pointers - they expose memory leaks and unclear ownership

```cpp
// GOOD - Optional reference for cache lookup (value may not exist)
class AssetRegistry {
public:
  std::optional<std::reference_wrapper<const Asset>>
  getAsset(const std::string& name) const {
    auto it = cache_.find(name);
    if (it != cache_.end()) {
      return std::cref(it->second);
    }
    return std::nullopt;
  }

private:
  std::unordered_map<std::string, Asset> cache_;
};

// Usage
if (auto asset = registry.getAsset("cube")) {
  // Access via .get()
  const Asset& assetRef = asset->get();
  // Use assetRef...
}

// GOOD - Plain reference when value always exists
class MeshRenderer {
public:
  explicit MeshRenderer(const AssetRegistry& registry)
    : registry_{registry} {}  // Value always exists

  void render(const Asset& asset) {  // Asset always exists
    // Use asset directly...
  }

private:
  const AssetRegistry& registry_;  // Non-owning reference
};

// BAD - Returning raw pointer (unsafe, unclear ownership)
const Asset* getAsset(const std::string& name) const {
  auto it = cache_.find(name);
  return (it != cache_.end()) ? &it->second : nullptr;
}

// BAD - Using shared_ptr for non-owning access
class MeshRenderer {
private:
  std::shared_ptr<AssetRegistry> registry_;  // Obscures ownership hierarchy
};
```

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
