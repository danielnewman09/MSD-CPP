# Project Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/designs/` for detailed component relationships.

## Project Overview

{Brief description of what this project does and its primary purpose}

## Architecture Overview

### High-Level Architecture

See: [`docs/architecture/overview.puml`](docs/architecture/overview.puml)

{Description of the overall system architecture, major subsystems, and how they interact}

### Core Components

| Component | Location | Purpose | Diagram |
|-----------|----------|---------|---------|
| msd-transfer | `msd/msd-transfer/` | Database transfer objects (DTOs) | [`msd-transfer-core.puml`](docs/msd/msd-transfer/msd-transfer-core.puml) |

---

## Component Details

### msd-transfer

**Location**: `msd/msd-transfer/src/`
**Diagram**: [`docs/msd/msd-transfer/msd-transfer-core.puml`](docs/msd/msd-transfer/msd-transfer-core.puml)
**Type**: Header-only interface library

#### Purpose
Defines lightweight, header-only structs that represent database records for the MSD asset management system. These structs serve as pure data transfer objects (DTOs) used by `cpp_sqlite` to automatically generate SQL schema and provide type-safe ORM functionality.

This library acts as the shared contract between database storage and domain logic, with no dependencies on simulation or rendering components.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `MeshRecord` | `MeshRecord.hpp` | Visual mesh geometry storage (vertex data as BLOB) |
| `ObjectRecord` | `MeshRecord.hpp` | Complete object definition with visual/collision mesh references |
| `MaterialRecord` | `MaterialRecord.hpp` | Rendering material properties and shader references |
| `PhysicsTemplateRecord` | `PhysicsTemplateRecord.hpp` | Rigid body template with physical properties |
| `Records` | `Records.hpp` | Convenience header including all record types |

#### Key Interfaces
```cpp
// All records inherit from BaseTransferObject
struct MeshRecord : public cpp_sqlite::BaseTransferObject {
    std::vector<uint8_t> vertex_data;  // Serialized vertex array
    uint32_t vertex_count{0};
};

struct ObjectRecord : public cpp_sqlite::BaseTransferObject {
    std::string name;
    std::string category;
    cpp_sqlite::ForeignKey<MeshRecord> meshRecord;           // Visual geometry
    cpp_sqlite::ForeignKey<MeshRecord> collisionMeshRecord;  // Collision geometry
};

struct PhysicsTemplateRecord : public cpp_sqlite::BaseTransferObject {
    std::string name;
    cpp_sqlite::ForeignKey<MeshRecord> mesh;
    double mass{1.0};
    double friction{0.5};
    double restitution{0.3};
    // ... additional physical properties
};
```

#### Usage Example
```cpp
#include <msd-transfer/src/Records.hpp>

// Create database connection
cpp_sqlite::Database db{"assets.db", true};

// Get DAO for record type
auto& meshDAO = db.getDAO<msd_transfer::MeshRecord>();

// Insert a mesh record
msd_transfer::MeshRecord mesh;
mesh.id = meshDAO.incrementIdCounter();
mesh.vertex_count = 18;
mesh.vertex_data = serializeVertices(pyramidVertices);
meshDAO.insert(mesh);

// Query records
auto allMeshes = meshDAO.selectAll();
auto singleMesh = meshDAO.selectById(1);

// Work with foreign keys
msd_transfer::ObjectRecord obj;
obj.meshRecord.id = 1;  // Reference to mesh ID 1
if (obj.meshRecord.isSet()) {
    auto mesh = obj.meshRecord.resolve(db);
}
```

#### Thread Safety
- **Immutable after creation**: Transfer objects are pure data containers with no mutable state
- **No synchronization needed**: Safe to read from multiple threads after construction
- **Thread-safe database access**: Thread safety depends on `cpp_sqlite::Database` implementation

#### Error Handling
- No exceptions thrown by transfer objects (pure data)
- Foreign key resolution returns `std::optional<T>` (nullopt if not found)
- Database errors propagate from `cpp_sqlite` layer

#### Memory Management
- **Value semantics**: All records use value types (strings, vectors)
- **No ownership complexity**: Pure data containers with no pointers
- **BLOB storage**: Binary data stored as `std::vector<uint8_t>`, serialization handled by consumers
- **Foreign keys**: References by ID, not by pointer - resolved on demand

#### Dependencies
- `cpp_sqlite` — ORM framework for database operations
- `Boost.Describe` — Compile-time reflection for automatic schema generation

#### Related Components
- [`msd-assets`](#msd-assets) — Consumes records to build domain objects
- [`msd-gui`](#msd-gui) — Uses AssetDatabase to load/save records

---

## Design Patterns in Use

### {Pattern Name}
**Used in**: `{location}`  
**Purpose**: {Why this pattern is used here}

See implementation: [`docs/designs/{feature}/{feature}.puml`](docs/designs/{feature}/{feature}.puml)

---

## Cross-Cutting Concerns

### Error Handling Strategy
{Project-wide error handling approach}

### Logging
{Logging conventions and levels}

### Memory Management
- **Ownership Transfer**: Use `std::unique_ptr` for exclusive ownership and transfer
- **Non-Owning Access**: Prefer plain references (`const T&` or `T&`) for non-owning access
- **Shared Ownership**: Avoid `std::shared_ptr` - prefer establishing clear ownership hierarchies with references
- **Never** use raw pointers in public interfaces - they expose memory leaks and unclear ownership
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

### Thread Safety Conventions
{Project-wide threading approach}

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

#### VSCode Integration

The [`.vscode/tasks.json`](.vscode/tasks.json) file provides tasks for common build operations:
- **Conan Build**: Full build with dependency installation
- **Conan Install**: Install dependencies only
- **CMake Build**: Build using CMake directly
- **Compile Shaders**: Compile shaders using DXC and Shadercross

In VSCode with the CMake Tools extension, the component-specific build presets appear in the build preset selector.

### Project Components

| Component | Target Name | Location | Type |
|-----------|-------------|----------|------|
| **msd-utils** | `msd_utils` | `msd/msd-utils/` | Library |
| **msd-transfer** | `msd_transfer` | `msd/msd-transfer/` | Interface Library |
| **msd-sim** | `msd_sim` | `msd/msd-sim/` | Library |
| **msd-assets** | `msd_assets` | `msd/msd-assets/` | Library |
| **msd-gui** | `msd_gui` | `msd/msd-gui/` | Library |
| **msd-exe** | `msd_exe` | `msd/msd-exe/` | Executable |
| **msd-asset-gen** | `generate_assets` | `msd/msd-asset-gen/` | Executable |

### Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_TESTING` | `ON` | Build the testing tree |
| `ENABLE_COVERAGE` | `OFF` | Enable code coverage (GCC/Clang only) |
| `ENABLE_BENCHMARKS` | `OFF` | Build performance benchmarks (requires Conan option) |

---

## Testing

### Test Organization
```
test/
├── unit/           # Unit tests (isolated, fast)
├── integration/    # Integration tests (component interaction)
└── e2e/            # End-to-end tests (full system)
```

### Running Tests
```bash
# Test commands
```

### Test Conventions
- Test files mirror source structure: `src/foo/bar.cpp` → `test/unit/foo/bar_test.cpp`
- Ticket references in test descriptions: `TEST_CASE("ClassName: behavior [ticket-name]")`

---

## Benchmarking

**Ticket**: [0011_add_google_benchmark](tickets/0011_add_google_benchmark.md)
**Design**: [`docs/designs/0011_add_google_benchmark/design.md`](docs/designs/0011_add_google_benchmark/design.md)

The project uses Google Benchmark for micro-benchmarking performance-critical code paths. Benchmarks are optional and disabled by default to avoid extending build times.

### Building Benchmarks

**Prerequisites**: Install dependencies with benchmarks enabled:
```bash
# Release build recommended for accurate measurements
conan install . --build=missing -s build_type=Release -o "&:enable_benchmarks=True"
```

**Configure and Build**:
```bash
# Configure with benchmarks enabled
cmake --preset conan-release -DENABLE_BENCHMARKS=ON

# Build benchmark executable(s)
cmake --build --preset conan-release --target msd_sim_bench
```

### Running Benchmarks

**Basic execution**:
```bash
# Run all benchmarks
./build/Release/release/msd_sim_bench

# Run with specific filters
./build/Release/release/msd_sim_bench --benchmark_filter=ConvexHull_Construction

# Run with repetitions for statistical significance
./build/Release/release/msd_sim_bench --benchmark_repetitions=10
```

**Output formats**:
```bash
# JSON output for analysis
./build/Release/release/msd_sim_bench --benchmark_out=results.json --benchmark_out_format=json

# CSV output for spreadsheets
./build/Release/release/msd_sim_bench --benchmark_out=results.csv --benchmark_out_format=csv
```

**Performance options**:
```bash
# Control minimum benchmark time (default 0.5s per benchmark)
./build/Release/release/msd_sim_bench --benchmark_min_time=1.0s

# Set CPU affinity to reduce variance (Linux/macOS)
./build/Release/release/msd_sim_bench --benchmark_enable_random_interleaving=true
```

### Generating Benchmark Reports

Use the `run_benchmarks.sh` script to generate JSON reports for tracking performance over time:

```bash
# Generate JSON report (default: benchmark_results/ directory)
./scripts/run_benchmarks.sh

# Custom output directory with 5 repetitions
./scripts/run_benchmarks.sh -o reports -r 5

# Console output only (no file)
./scripts/run_benchmarks.sh -f console

# Show all options
./scripts/run_benchmarks.sh --help
```

**Script options**:
| Option | Default | Description |
|--------|---------|-------------|
| `-o, --output DIR` | `benchmark_results` | Output directory for JSON reports |
| `-f, --format FMT` | `json` | Output format: `json` or `console` |
| `-b, --build-type` | `Release` | Build type: `Debug` or `Release` |
| `-r, --repetitions N` | `3` | Number of repetitions per benchmark |

The script organizes results by executable name:
```
benchmark_results/
└── msd_sim_bench/
    ├── benchmark_20260108_143000.json
    ├── benchmark_20260108_150000.json
    └── benchmark_latest.json -> benchmark_20260108_150000.json
```

Each suite folder contains timestamped JSON files and a `benchmark_latest.json` symlink for convenience.

### Available Benchmark Suites

| Benchmark Suite | Executable | Location | Purpose |
|-----------------|------------|----------|---------|
| **ConvexHull** | `msd_sim_bench` | `msd/msd-sim/bench/` | Convex hull construction, containment, distance, and GJK collision |

**ConvexHull benchmarks**:
- `BM_ConvexHull_Construction` — Hull construction from point clouds (8, 64, 512, 4096 points)
- `BM_ConvexHull_Contains` — Point containment queries (collision detection hot path)
- `BM_ConvexHull_SignedDistance` — Signed distance calculations (proximity queries)
- `BM_ConvexHull_Intersects` — GJK intersection tests (collision detection)

### Interpreting Results

**Example output**:
```
--------------------------------------------------------------------------
Benchmark                                Time             CPU   Iterations
--------------------------------------------------------------------------
BM_ConvexHull_Construction/8         36974 ns        36833 ns         4174
BM_ConvexHull_Construction/64        94272 ns        93823 ns         1341
BM_ConvexHull_Construction/512      254729 ns       253761 ns          566
BM_ConvexHull_Construction/4096    1055573 ns      1048969 ns          127
BM_ConvexHull_Construction_BigO     261.71 N        260.09 N
BM_ConvexHull_Construction_RMS          21 %            21 %
BM_ConvexHull_Contains                2263 ns         2248 ns        64469
```

**Key metrics**:
- **Time**: Wall-clock time per iteration
- **CPU**: CPU time per iteration (excludes I/O wait)
- **Iterations**: Number of times benchmark ran (auto-adjusted for min_time)
- **BigO**: Algorithmic complexity estimate (for parameterized benchmarks)
- **RMS**: Root-mean-square deviation (measure of consistency)

### Benchmark Organization

Benchmarks follow the same directory structure as tests:
```
msd/msd-sim/
├── src/            # Source code
├── test/           # Unit/integration tests
└── bench/          # Performance benchmarks
    ├── CMakeLists.txt
    └── ConvexHullBench.cpp
```

### Writing New Benchmarks

**Benchmark template**:
```cpp
// Ticket: {ticket-name}
// Design: docs/designs/{ticket-name}/design.md

#include <benchmark/benchmark.h>
#include "msd-sim/src/YourComponent.hpp"

static void BM_YourComponent_Operation(benchmark::State& state) {
  // Setup (outside timing loop)
  YourComponent component{/* ... */};

  // Benchmark loop
  for (auto _ : state) {
    auto result = component.operation();
    benchmark::DoNotOptimize(result);  // Prevent optimization
  }
}
BENCHMARK(BM_YourComponent_Operation);

// Parameterized benchmark
static void BM_YourComponent_Scaled(benchmark::State& state) {
  auto data = generateData(state.range(0));
  for (auto _ : state) {
    auto result = component.process(data);
    benchmark::DoNotOptimize(result);
  }
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_YourComponent_Scaled)
    ->Args({10})
    ->Args({100})
    ->Args({1000})
    ->Complexity();
```

**Best practices**:
- Use `benchmark::DoNotOptimize()` to prevent dead code elimination
- Place setup code outside the timing loop
- Use fixed seeds for random data to ensure reproducibility
- Add complexity analysis for parameterized benchmarks
- Include ticket references in file header and function documentation

### Benchmark Regression Detection

**Ticket**: [0014_benchmark_metrics_tracker](tickets/0014_benchmark_metrics_tracker.md)
**Design**: [`docs/designs/0014_benchmark_metrics_tracker/design.md`](docs/designs/0014_benchmark_metrics_tracker/design.md)

The project uses `compare_benchmarks.py` to detect performance regressions by comparing results against golden baseline files.

**Basic workflow**:
```bash
# Run benchmarks
./scripts/run_benchmarks.sh

# Compare against baseline
./scripts/compare_benchmarks.py

# Update baseline (when performance changes are intentional)
./scripts/compare_benchmarks.py --set-baseline
```

**Interpreting results**:
- **GREEN**: Performance within threshold or improved
- **YELLOW**: New/missing benchmarks (review if expected)
- **RED**: Regression detected (exceeds threshold)

**Default threshold**: 10% slower than baseline triggers regression

**Advanced options**:
```bash
# Use custom threshold (5% instead of default 10%)
./scripts/compare_benchmarks.py --threshold 5.0

# Strict mode: exit code 1 on regression (for CI)
./scripts/compare_benchmarks.py --strict

# Compare specific result file
./scripts/compare_benchmarks.py --current benchmark_results/msd_sim_bench/benchmark_20260108.json

# Disable colors (for CI logs)
./scripts/compare_benchmarks.py --no-color

# Output JSON report only (no console output)
./scripts/compare_benchmarks.py --output-json-only
```

**Comparison reports**:
- Location: `benchmark_results/{suite}/comparison_{timestamp}.json`
- Format: JSON with per-benchmark diff, summary statistics
- Useful for: Design review, pull request analysis

**Baseline files**:
- Location: `benchmark_baselines/{suite}/baseline.json`
- Committed to git for team-wide consistency
- Update when intentional performance changes occur

**When to update baselines**:
1. After performance optimizations that improve metrics
2. When algorithmic changes intentionally trade performance for correctness
3. When refactoring changes performance characteristics
4. Always commit baseline updates with code changes that affect them

**Example workflow for optimization**:
```bash
# Verify current performance
./scripts/run_benchmarks.sh
./scripts/compare_benchmarks.py

# Make optimization changes
# ... edit code ...

# Run benchmarks again
./scripts/run_benchmarks.sh
./scripts/compare_benchmarks.py

# If improved, update baseline
./scripts/compare_benchmarks.py --set-baseline

# Commit code and baseline together
git add src/optimized_code.cpp
git add benchmark_baselines/msd_sim_bench/baseline.json
git commit -m "Optimize ConvexHull construction

Performance improvement:
- BM_ConvexHull_Construction/512: 266ms -> 180ms (-32%)

Updated benchmark baseline."
```

### CI Integration

**Status**: Local execution only (no CI integration yet)

**Future enhancement**: Automated benchmark execution on pull requests with baseline comparison for performance regression detection using the `--strict` flag.

---

## Recent Architectural Changes

### Benchmark Metrics Tracker — 2026-01-08
**Ticket**: [0014_benchmark_metrics_tracker](tickets/0014_benchmark_metrics_tracker.md)
**Design**: [`docs/designs/0014_benchmark_metrics_tracker/design.md`](docs/designs/0014_benchmark_metrics_tracker/design.md)

Added Python-based benchmark regression detection tool that compares Google Benchmark results against golden baseline files. The system detects performance regressions with configurable thresholds (default 10%), generates JSON comparison reports, and provides color-coded console output for local development and CI integration.

**Key files added**:
- `scripts/compare_benchmarks.py` — Main comparison script with CLI interface
- `benchmark_baselines/msd_sim_bench/baseline.json` — Initial golden baseline for ConvexHull benchmarks

**Features**:
- Automatic comparison against committed baselines
- Configurable regression threshold (default 10%)
- Color-coded console output (GREEN/YELLOW/RED)
- JSON comparison reports with detailed metrics
- Strict mode for CI integration (exit code 1 on regression)
- Baseline management via `--set-baseline` flag

**Workflow integration**:
- Run after `run_benchmarks.sh` to detect regressions
- Update baselines when performance changes are intentional
- Commit baseline updates alongside code changes

### Google Benchmark Infrastructure — 2026-01-08
**Ticket**: [0011_add_google_benchmark](tickets/0011_add_google_benchmark.md)
**Design**: [`docs/designs/0011_add_google_benchmark/design.md`](docs/designs/0011_add_google_benchmark/design.md)

Added Google Benchmark infrastructure for micro-benchmarking performance-critical code paths. The implementation provides optional build integration via Conan, benchmark executable configuration mirroring test infrastructure, and initial ConvexHull benchmarks for msd-sim.

**Key files added**:
- `msd/msd-sim/bench/CMakeLists.txt` — Benchmark executable build configuration
- `msd/msd-sim/bench/ConvexHullBench.cpp` — ConvexHull performance benchmarks (construction, containment, signed distance, GJK intersection)

**Build system changes**:
- `conanfile.py` — Added `enable_benchmarks` option and conditional `benchmark/1.9.1` dependency
- `CMakeLists.txt` — Added `ENABLE_BENCHMARKS` CMake option (default OFF)
- `msd/msd-sim/CMakeLists.txt` — Added conditional `bench/` subdirectory inclusion

### msd-transfer Documentation — 2026-01-01
**Diagram**: [`docs/msd/msd-transfer/msd-transfer-core.puml`](docs/msd/msd-transfer/msd-transfer-core.puml)

Added comprehensive documentation for the msd-transfer header-only library. This library defines database transfer objects (DTOs) for the MSD asset management system, providing the shared contract between database storage and domain logic.

**Key files documented**:
- `msd/msd-transfer/src/Records.hpp` — Convenience header including all record types
- `msd/msd-transfer/src/MeshRecord.hpp` — Visual mesh geometry and object records
- `msd/msd-transfer/src/MaterialRecord.hpp` — Rendering material definitions
- `msd/msd-transfer/src/PhysicsTemplateRecord.hpp` — Rigid body physics templates

---

## Diagrams Index

| Diagram | Description | Last Updated |
|---------|-------------|--------------|
| [`overview.puml`](docs/architecture/overview.puml) | High-level system architecture | {date} |
| [`msd-transfer-core.puml`](docs/msd/msd-transfer/msd-transfer-core.puml) | msd-transfer high-level architecture overview | 2026-01-01 |
| [`records.puml`](docs/msd/msd-transfer/records.puml) | msd-transfer database records detailed design | 2026-01-01 |
| [`input-state-management.puml`](docs/designs/input-state-management/input-state-management.puml) | Input state tracking and management system | 2026-01-05 |
| [`0011_add_google_benchmark.puml`](docs/designs/0011_add_google_benchmark/0011_add_google_benchmark.puml) | Google Benchmark build system integration | 2026-01-08 |

---

## Conventions

### Naming Conventions
- Classes: `PascalCase`
- Functions/Methods: `camelCase` or `snake_case` (choose one)
- Member variables: `snake_case_` (trailing underscore)
- Constants: `kPascalCase` or `SCREAMING_SNAKE_CASE`

### Code Organization
- One class per header (generally)
- Implementation in `.cpp` unless template/inline
- Public headers in `include/`, private in `src/`

### Documentation
- Public APIs: Doxygen-style comments
- Ticket references: `// Ticket: {ticket-name}` for non-obvious implementations
- PlantUML diagrams for architectural components

--

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

### Rule of Zero/Five
- **Prefer** the Rule of Zero: use compiler-generated special member functions when possible
- **Only** implement copy/move constructors/assignment if you need custom behavior
- **Use** `= default` explicitly to document that you're using the compiler's implementation
- **Remove** manual implementations that just copy all members (let the compiler do it)

```cpp
// GOOD - Rule of Zero with explicit default
class ConvexHull {
public:
  ConvexHull(const ConvexHull&) = default;
  ConvexHull(ConvexHull&&) noexcept = default;
  ConvexHull& operator=(const ConvexHull&) = default;
  ConvexHull& operator=(ConvexHull&&) noexcept = default;
  ~ConvexHull() = default;

private:
  std::vector<Coordinate> vertices_;
  float volume_{std::numeric_limits<float>::quiet_NaN()};
};

// BAD - Unnecessary manual implementation
class ConvexHull {
public:
  ConvexHull(const ConvexHull& other)
    : vertices_(other.vertices_), volume_(other.volume_) {}
  // ... manual implementations for all special members
};
```

### All-or-Nothing Rule
- **If** you implement any special member function, implement ALL of them (Rule of Five)
- **Prefer** implementing NONE and using `= default` for all (Rule of Zero)
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

---

## Getting Help

### For AI Assistants
1. Start with this document for architectural context
2. Reference the linked PlantUML diagrams for component relationships
3. Check `tickets/` for feature history and design decisions
4. Look at `docs/designs/{feature}/design.md` for detailed design rationale

### For Developers
- Design documents: `docs/designs/`
- API documentation: `docs/api/` (if generated)
- Tickets with full context: `tickets/`
