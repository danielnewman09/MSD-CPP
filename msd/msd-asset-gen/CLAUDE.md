# msd-asset-gen Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/designs/` for detailed component relationships.

## Project Overview

**msd-asset-gen** is a command-line asset generation tool that creates SQLite databases populated with primitive geometry assets (cubes, pyramids, etc.). It generates both visual and collision geometry data using the msd-assets GeometryFactory and stores them as msd-transfer database records. This tool is used to bootstrap the MSD project with standard geometric primitives.

## Architecture Overview

### High-Level Architecture

**msd-asset-gen** is a simple command-line executable that orchestrates three main libraries:
1. **msd-assets** — Provides `GeometryFactory` to create primitive geometries
2. **msd-transfer** — Provides database record structures (`MeshRecord`, `ObjectRecord`)
3. **cpp_sqlite** — Provides database abstraction and ORM capabilities

**Data Flow**:
```
GeometryFactory::createCube()
  → MeshRecord (vertex data)
    → Database insertion
      → ObjectRecord creation (with FK references)
        → Database insertion
```

### Core Components

| Component | Location | Purpose | Diagram |
|-----------|----------|---------|---------|
| Asset Generator | `src/generate_assets.cpp` | Creates database with primitive assets | [`msd-asset-gen.puml`](../../../docs/msd/msd-asset-gen/msd-asset-gen.puml) |

---

## Component Details

### Asset Generator

**Location**: `src/generate_assets.cpp`
**Type**: Executable
**Diagram**: [`docs/msd/msd-asset-gen/msd-asset-gen.puml`](../../../docs/msd/msd-asset-gen/msd-asset-gen.puml)
**Introduced**: Initial implementation

#### Purpose
The asset generator executable creates an SQLite database populated with geometric primitive assets. Each asset includes both visual geometry (for rendering) and collision geometry (for physics). The tool currently supports:
- **Cube** — 1.0 unit cube primitive
- **Pyramid** — 1.0 base × 1.0 height pyramid primitive

#### Key Functions

| Function | Responsibility |
|----------|----------------|
| `createCubeAsset()` | Generate cube geometry and insert into database |
| `createPyramidAsset()` | Generate pyramid geometry and insert into database |
| `main()` | CLI entry point, database setup, asset orchestration |

#### Usage Example
```bash
# Create an asset database
./generate_assets assets.db

# The resulting database will contain:
# - 2 MeshRecord entries (cube visual, pyramid visual)
# - 2 MeshRecord entries (cube collision, pyramid collision)
# - 2 ObjectRecord entries (cube object, pyramid object)
```

#### Data Flow Details
For each primitive asset:
1. Call `GeometryFactory::createCube()` or `createPyramid()` → returns `MeshRecord`
2. Insert visual `MeshRecord` into database → get visual mesh ID
3. Create collision `MeshRecord` (same geometry)
4. Insert collision `MeshRecord` into database → get collision mesh ID
5. Create `ObjectRecord` with name, category, and FK references to both meshes
6. Insert `ObjectRecord` into database → complete asset definition

#### Thread Safety
Single-threaded executable — no concurrency concerns.

#### Error Handling
Uses C++ exceptions:
- `cpp_sqlite::Database` throws on database errors
- Command-line validation for required arguments
- Top-level try/catch in `main()` for graceful error reporting

#### Dependencies
- `msd-assets` — GeometryFactory for primitive generation
- `msd-transfer` — MeshRecord, ObjectRecord database schemas
- `cpp_sqlite` — Database, DAO, ORM functionality
- `spdlog` — Logging (via cpp_sqlite dependency)

---

## Design Patterns in Use

### Factory Pattern (via msd-assets)
**Used in**: `msd_assets::GeometryFactory`
**Purpose**: Encapsulates primitive geometry creation logic, providing clean interface for generating standardized mesh data.

### Data Transfer Object Pattern
**Used in**: `msd_transfer::MeshRecord`, `msd_transfer::ObjectRecord`
**Purpose**: Decouples database schema from domain objects, enabling serialization/deserialization via cpp_sqlite ORM.

---

## Cross-Cutting Concerns

### Error Handling Strategy
- **Exceptions**: Uses C++ exception handling for errors
- Database operations throw exceptions from cpp_sqlite layer
- CLI validates arguments and reports errors to stderr
- Top-level exception handler in `main()` ensures graceful shutdown

### Logging
- Uses `cpp_sqlite::Logger` for database-level logging
- Console output via `std::cout` for progress tracking
- Error reporting via `std::cerr` for failures

### Memory Management
- Stack-allocated records (MeshRecord, ObjectRecord)
- Database manages record lifetimes internally
- No raw pointers or manual memory management required

### Thread Safety Conventions
Not applicable — single-threaded command-line tool.

---

## Build & Configuration

### Build Requirements
- **C++ Standard**: C++20
- **Compiler**: GCC 11+, Clang 14+, or MSVC 2019+
- **Build System**: CMake 3.15+ with Conan 2.x package manager
- **Dependencies**: Managed via Conan (see `conanfile.py`)
  - `msd-assets` — Geometry factory
  - `msd-transfer` — Database record schemas
  - `cpp_sqlite` — Database ORM
  - `spdlog` — Logging library
  - Boost.Describe — Reflection for ORM

### Build Process

This component uses **Conan** for dependency management and **CMake presets** for build configuration.

#### Prerequisites: Install Dependencies with Conan

**IMPORTANT**: Before building, you must run Conan to install dependencies and generate CMake configuration files:

```bash
# From project root - For Debug build
conan install . --build=missing -s build_type=Debug

# For Release build
conan install . --build=missing -s build_type=Release
```

#### Building This Component Only

After running `conan install`, use the component-specific build preset:

```bash
# Configure with Debug preset (from project root)
cmake --preset conan-debug

# Build only msd-asset-gen (Debug)
cmake --build --preset debug-asset-gen-only

# Or for Release
cmake --preset conan-release
cmake --build --preset release-asset-gen-only
```

#### Building the Entire Project

To build all components including msd-asset-gen:

```bash
# Configure and build everything (Debug)
cmake --preset conan-debug
cmake --build --preset conan-debug
```

#### Running the Generator

```bash
# Debug build
./build/Debug/debug/generate_assets output.db

# Release build
./build/Release/release/generate_assets output.db
```

### Installation
```bash
cmake --install build
# Installs generate_assets to <prefix>/bin/
```

---

## Testing

### Test Organization
```
test/
└── (Test infrastructure TBD)
```

### Running Tests
No automated tests currently — manual verification via:
```bash
# Generate test database
./generate_assets test.db

# Inspect with sqlite3
sqlite3 test.db
> .tables
> SELECT * FROM ObjectRecord;
> SELECT id, name, category FROM ObjectRecord;
```

### Test Conventions
Testing strategy to be defined when test framework is added.

---

## Recent Architectural Changes

### Initial Asset Generator Implementation — 2025-12-28
Initial implementation of the asset generation tool.

**Key files added**:
- `src/generate_assets.cpp` — Main executable implementing cube and pyramid asset generation
- `src/CMakeLists.txt` — Build configuration
- `CLAUDE.md` — This architectural documentation

**Capabilities**:
- Creates SQLite database with primitive assets
- Generates cube and pyramid geometries
- Stores visual and collision mesh data
- Links geometry via ObjectRecord foreign keys

---

## Conventions

### Naming Conventions
- Functions: `camelCase` (following msd-assets conventions)
- Variables: `snake_case` or `camelCase`
- Database records: Follow `msd_transfer` naming (PascalCase structs)

### Code Organization
- Executable source in `src/`
- No public headers (executable only, not a library)
- Helper functions in same file as `main()`

### Documentation
- Doxygen-style comments for file-level documentation
- Inline comments for non-obvious database operations
- Asset format documented in code comments

---

## Getting Help

### For AI Assistants
1. This document provides complete architectural context for msd-asset-gen
2. Review [msd-assets/CLAUDE.md](../msd-assets/CLAUDE.md) for GeometryFactory details
3. Review [msd-transfer/CLAUDE.md](../msd-transfer/CLAUDE.md) for database schema details
4. Check parent [CLAUDE.md](../CLAUDE.md) for overall MSD project architecture

### For Developers
**Usage**:
```bash
./generate_assets <output_database_path>
```

**Output**: SQLite database containing:
- `MeshRecord` table with visual and collision geometry
- `ObjectRecord` table with named primitive assets

**Adding New Primitives**:
1. Create a new `create{Primitive}Asset()` function following the pattern of `createCubeAsset()`
2. Call the appropriate `GeometryFactory::create{Primitive}()` method
3. Insert visual and collision meshes into database
4. Create and insert `ObjectRecord` with appropriate name and category
5. Call the new function from `main()`

**Related Components**:
- [msd-assets](../msd-assets/) — Provides geometry generation
- [msd-transfer](../msd-transfer/) — Provides database schema
- [cpp_sqlite](../../libs/cpp_sqlite/) — Provides database layer
