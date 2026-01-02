# msd-assets Library Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/msd/` for detailed component relationships.

## Project Overview

The msd-assets library provides asset management and geometry handling for the MSD system. It manages loading, caching, and accessing 3D geometry assets from a SQLite database, including both visual and collision meshes. The library handles STL file loading, primitive geometry generation, and maintains an in-memory asset registry for efficient access.

## Architecture Overview

### High-Level Architecture

See: [`docs/msd/msd-assets/msd-assets-core.puml`](../../docs/msd/msd-assets/msd-assets-core.puml)

The library consists of four main subsystems:
- **Asset Management** — Registry and caching of complete assets
- **Geometry System** — Visual and collision geometry containers
- **Geometry Factory** — Procedural generation of primitive shapes
- **File Loaders** — STL file import and conversion

### Core Components

| Component | Location | Purpose | Diagram |
|-----------|----------|---------|---------|
| AssetRegistry | `src/` | Singleton cache for assets loaded from database | [`asset-registry.puml`](../../docs/msd/msd-assets/asset-registry.puml) |
| Asset | `src/` | Container for complete asset with visual/collision geometry | [`asset-geometry.puml`](../../docs/msd/msd-assets/asset-geometry.puml) |
| Geometry | `src/` | Template-based geometry storage (visual/collision) | [`asset-geometry.puml`](../../docs/msd/msd-assets/asset-geometry.puml) |
| GeometryFactory | `src/` | Factory for creating primitive geometries | [`geometry-factory.puml`](../../docs/msd/msd-assets/geometry-factory.puml) |
| STLLoader | `src/` | STL file loader (binary and ASCII formats) | [`stl-loader.puml`](../../docs/msd/msd-assets/stl-loader.puml) |

---

## Component Details

### AssetRegistry

**Location**: `src/AssetRegistry.hpp`, `src/AssetRegistry.cpp`
**Diagram**: [`docs/msd/msd-assets/asset-registry.puml`](../../docs/msd/msd-assets/asset-registry.puml)

#### Purpose
Singleton registry that provides lazy-loading and caching of geometry assets from the SQLite database. Assets are loaded on-demand when first accessed and cached in memory for subsequent use.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `AssetRegistry` | `AssetRegistry.hpp` | Manages asset cache and database connection |

#### Key Interfaces
```cpp
class AssetRegistry {
public:
    AssetRegistry(const std::string& dbPath);

    std::optional<std::reference_wrapper<const Asset>>
    getAsset(const std::string& objectName);

    std::optional<std::reference_wrapper<const VisualGeometry>>
    loadVisualGeometry(const std::string& objectName);

    std::optional<std::reference_wrapper<const CollisionGeometry>>
    loadCollisionGeometry(const std::string& objectName);

    size_t getCacheMemoryUsage() const;
};
```

#### Usage Example
```cpp
AssetRegistry registry{"assets.db"};

// Load complete asset
auto asset = registry.getAsset("cube");
if (asset) {
    if (asset->get().hasVisualGeometry()) {
        auto visualGeom = asset->get().getVisualGeometry();
        // Use visual geometry...
    }
}

// Load just visual geometry
auto visualGeom = registry.loadVisualGeometry("pyramid");
if (visualGeom) {
    const auto& vertices = visualGeom->get().getVertices();
    // Render vertices...
}
```

#### Thread Safety
- All public methods are thread-safe via internal `std::mutex`
- Cache access is protected by `cacheMutex_`
- Safe for concurrent reads and writes from multiple threads

#### Error Handling
- Returns `std::optional` for asset not found (no exceptions on miss)
- Database connection errors throw exceptions during construction
- Invalid database schema throws exceptions during load

#### Dependencies
- `cpp_sqlite::Database` — SQLite database abstraction
- `msd_transfer::ObjectRecord`, `msd_transfer::MeshRecord` — Database record types
- `Asset` — Complete asset container

---

### Asset

**Location**: `src/Asset.hpp`, `src/Asset.cpp`
**Diagram**: [`docs/msd/msd-assets/asset-geometry.puml`](../../docs/msd/msd-assets/asset-geometry.puml)

#### Purpose
Represents a complete asset loaded from the database, encapsulating an object with its associated visual and/or collision geometry. Assets are immutable once created.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `Asset` | `Asset.hpp` | Immutable container for asset metadata and geometry |

#### Key Interfaces
```cpp
class Asset {
public:
    static Asset fromObjectRecord(msd_transfer::ObjectRecord& record,
                                  cpp_sqlite::Database& db);

    uint32_t getId() const;
    const std::string& getName() const;
    const std::string& getCategory() const;

    std::optional<std::reference_wrapper<const VisualGeometry>>
    getVisualGeometry() const;

    std::optional<std::reference_wrapper<const CollisionGeometry>>
    getCollisionGeometry() const;

    bool hasVisualGeometry() const;
    bool hasCollisionGeometry() const;
};
```

#### Thread Safety
- Immutable after construction — safe to read from multiple threads
- No internal synchronization needed (read-only operations)

#### Error Handling
- Factory method `fromObjectRecord()` may throw on database errors
- Geometry accessors return `std::optional` for missing geometry

#### Dependencies
- `VisualGeometry`, `CollisionGeometry` — Geometry containers
- `msd_transfer::ObjectRecord` — Database representation

---

### Geometry (VisualGeometry / CollisionGeometry)

**Location**: `src/Geometry.hpp`, `src/Geometry.cpp`
**Diagram**: [`docs/msd/msd-assets/asset-geometry.puml`](../../docs/msd/msd-assets/asset-geometry.puml)

#### Purpose
Template-based geometry container (`BaseGeometry<T>`) that stores vertex data for either visual rendering or collision detection. VisualGeometry stores vertices with normals and colors; CollisionGeometry stores raw 3D coordinates.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `BaseGeometry<T>` | `Geometry.hpp` | Template base for geometry storage |
| `VisualGeometry` | `Geometry.hpp` | Alias for `BaseGeometry<Vertex>` |
| `CollisionGeometry` | `Geometry.hpp` | Alias for `BaseGeometry<Eigen::Vector3d>` |
| `Vertex` | `Geometry.hpp` | Vertex structure (position, color, normal) |
| `BoundingBox` | `Geometry.hpp` | Bounding volume metadata |

#### Key Interfaces
```cpp
template <typename T>
class BaseGeometry {
public:
    explicit BaseGeometry(const msd_transfer::MeshRecord& record,
                          uint32_t objectId = 0);

    size_t getVertexCount() const;
    const std::vector<T>& getVertices() const;

    std::vector<uint8_t> serializeVertices() const;
    msd_transfer::MeshRecord populateMeshRecord() const;
};

using CollisionGeometry = BaseGeometry<Eigen::Vector3d>;
using VisualGeometry = BaseGeometry<Vertex>;

struct Vertex {
    float position[3];  // Position (x, y, z)
    float color[3];     // Color (r, g, b)
    float normal[3];    // Normal vector (x, y, z)
};
```

#### Usage Example
```cpp
// Create from database record
msd_transfer::MeshRecord meshRecord = loadFromDatabase();
VisualGeometry visualGeom{meshRecord, objectId};

// Access vertex data
const auto& vertices = visualGeom.getVertices();
for (const auto& vertex : vertices) {
    // vertex.position, vertex.color, vertex.normal
}

// Serialize back to database format
auto blob = visualGeom.serializeVertices();
```

#### Thread Safety
- Immutable after construction — safe for concurrent reads
- No internal synchronization needed

#### Error Handling
- Constructor throws `std::runtime_error` if vertex data size is invalid
- Uses `constexpr if` to specialize behavior for visual vs collision geometry

#### Dependencies
- `Eigen::Vector3d` — 3D coordinate representation
- `msd_transfer::MeshRecord` — Database serialization format

---

### GeometryFactory

**Location**: `src/GeometryFactory.hpp`, `src/GeometryFactory.cpp`
**Diagram**: [`docs/msd/msd-assets/geometry-factory.puml`](../../docs/msd/msd-assets/geometry-factory.puml)

#### Purpose
Factory class for procedurally generating common 3D geometric primitives (cubes, pyramids, etc.). All geometries are triangulated and centered at the origin.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `GeometryFactory` | `GeometryFactory.hpp` | Static factory methods for primitive shapes |

#### Key Interfaces
```cpp
class GeometryFactory {
public:
    static msd_transfer::MeshRecord createCube(double size);
    static msd_transfer::MeshRecord createCubeWireframe(double size);
    static msd_transfer::MeshRecord createPyramid(double baseSize, double height);
};
```

#### Usage Example
```cpp
// Create a cube mesh
auto cubeMesh = GeometryFactory::createCube(2.0);  // 2x2x2 cube

// Create wireframe for debugging
auto wireframe = GeometryFactory::createCubeWireframe(2.0);

// Create a pyramid
auto pyramidMesh = GeometryFactory::createPyramid(3.0, 4.0);
```

#### Thread Safety
- All methods are static and stateless — inherently thread-safe
- No shared state

#### Error Handling
- Returns valid MeshRecord or throws on allocation failure
- No invalid input validation (caller responsible)

#### Dependencies
- `Eigen::Vector3d` — Vector math
- `msd_transfer::MeshRecord` — Output format

---

### STLLoader

**Location**: `src/STLLoader.hpp`, `src/STLLoader.cpp`
**Diagram**: [`docs/msd/msd-assets/stl-loader.puml`](../../docs/msd/msd-assets/stl-loader.puml)

#### Purpose
Loader for STL (STereoLithography) 3D model files. Supports both binary and ASCII STL formats with automatic format detection.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `STLLoader` | `STLLoader.hpp` | Static methods for loading STL files |
| `STLTriangle` | `STLLoader.hpp` | Triangle data structure |

#### Key Interfaces
```cpp
class STLLoader {
public:
    static std::unique_ptr<msd_transfer::MeshRecord> loadSTL(
        const std::string& filename);

    static std::unique_ptr<msd_transfer::MeshRecord> loadBinarySTL(
        const std::string& filename);

    static std::unique_ptr<msd_transfer::MeshRecord> loadASCIISTL(
        const std::string& filename);

    static bool isBinarySTL(const std::string& filename);

    static msd_transfer::MeshRecord trianglesToMeshRecord(
        const std::vector<STLTriangle>& triangles);
};

struct STLTriangle {
    Eigen::Vector3f normal;
    Eigen::Vector3f vertex1;
    Eigen::Vector3f vertex2;
    Eigen::Vector3f vertex3;
};
```

#### Usage Example
```cpp
// Auto-detect format and load
auto mesh = STLLoader::loadSTL("model.stl");
if (mesh) {
    // Use mesh...
}

// Explicitly load binary format
auto binaryMesh = STLLoader::loadBinarySTL("model_binary.stl");

// Check format first
if (STLLoader::isBinarySTL("unknown.stl")) {
    // Handle as binary...
}
```

#### Thread Safety
- All methods are static and stateless — thread-safe
- File I/O is not synchronized (caller should not load same file concurrently)

#### Error Handling
- Returns `nullptr` on file not found or parse error
- Returns empty vector for triangle read failures
- Does not throw exceptions (uses return value for error signaling)

#### Dependencies
- `Eigen::Vector3f` — Triangle vertex representation
- `msd_transfer::MeshRecord` — Output format
- Standard C++ file I/O

---

## Design Patterns in Use

### Template Specialization
**Used in**: `BaseGeometry<T>`
**Purpose**: Single template handles both visual geometry (with normals/colors) and collision geometry (raw vertices) using compile-time specialization

### Singleton Pattern
**Used in**: `AssetRegistry`
**Purpose**: Centralized asset cache with single database connection

### Factory Pattern
**Used in**: `GeometryFactory`
**Purpose**: Encapsulates primitive geometry creation logic

### RAII
**Used in**: Throughout library
**Purpose**: Database connections, file handles, and memory management using smart pointers

---

## Cross-Cutting Concerns

### Error Handling Strategy
- **Factory/Loaders**: Return `std::unique_ptr` (nullptr on error) or `std::optional` (no exceptions)
- **Registry/Asset**: Return `std::optional<std::reference_wrapper>` for cache misses
- **Construction**: Throw exceptions for invalid state (database errors, corrupt data)

### Memory Management
- `std::unique_ptr` for ownership transfer (loaders)
- `std::optional` with `std::reference_wrapper` for non-owning cached access
- No raw pointers in public interfaces
- Member variables use value semantics where possible

### Thread Safety Conventions
- Immutable objects after construction are inherently thread-safe
- Mutable shared state (AssetRegistry cache) protected by `std::mutex`
- Static factory methods are stateless and thread-safe
- Documented in class-level Doxygen comments

### Coding Standards

This library follows the project-wide coding standards defined in the [root CLAUDE.md](../../CLAUDE.md#coding-standards).

Key standards applied in this library:
- **Initialization**: NaN for uninitialized floats, brace initialization `{}`, Rule of Zero with `= default`
- **Naming**: `snake_case_` for members, `PascalCase` for classes, `camelCase` for functions
- **Return Values**: Prefer returning values over output parameters
- **Memory**: `std::optional<std::reference_wrapper<const T>>` for cached non-owning access

See the [root CLAUDE.md](../../CLAUDE.md#coding-standards) for complete details and examples.

---

## Build & Configuration

### Build Requirements
- C++ Standard: C++20
- Compiler: GCC 11+, Clang 14+, or MSVC 2019+
- Build System: CMake 3.15+ with Conan 2.x package manager
- Dependencies (managed via Conan):
  - Eigen3 — Vector math and matrix operations
  - cpp_sqlite — Database abstraction layer
  - msd-transfer — Database record definitions (internal)

### Building this Library

This library is built as part of the MSD-CPP project using Conan and CMake presets. See the [root CLAUDE.md](../../CLAUDE.md#build--configuration) for complete build instructions.

**Quick start:**
```bash
# Install dependencies with Conan
conan install . --build=missing -s build_type=Debug

# Build just this library (Debug)
cmake --preset conan-debug
cmake --build --preset debug-assets-only

# Or for Release
conan install . --build=missing -s build_type=Release
cmake --preset conan-release
cmake --build --preset release-assets-only
```

**Component-specific presets:**
- Debug: `debug-assets-only` — Builds `msd-assets` library and tests
- Release: `release-assets-only` — Builds `msd-assets` library and tests

### Configuration Options
| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_TESTING` | `ON` | Build the testing tree |
| `ENABLE_COVERAGE` | `OFF` | Enable code coverage (GCC/Clang only) |

---

## Testing

### Test Organization
```
test/
├── unit/           # Unit tests for individual components
│   ├── asset_test.cpp
│   ├── geometry_test.cpp
│   ├── factory_test.cpp
│   └── stl_loader_test.cpp
└── integration/    # Integration tests with database
    └── asset_registry_test.cpp
```

### Running Tests
```bash
# All tests
ctest --test-dir build

# Specific test
ctest --test-dir build -R asset_test
```

### Test Conventions
- Test files mirror source structure: `src/Asset.cpp` → `test/unit/asset_test.cpp`
- Use Catch2 or GoogleTest framework
- Test both success and failure paths

---

## Recent Architectural Changes

### Asset Management System — 2025-12-28
**Diagram**: [`docs/msd/msd-asset-gen/msd-asset-gen.puml`](../../docs/msd/msd-asset-gen/msd-asset-gen.puml)

Initial implementation of the msd-assets library with complete asset management, geometry handling, and database integration.

**Key files added**:
- `src/AssetRegistry.hpp`, `src/AssetRegistry.cpp` — Singleton asset cache
- `src/Asset.hpp`, `src/Asset.cpp` — Complete asset container
- `src/Geometry.hpp`, `src/Geometry.cpp` — Template-based geometry system
- `src/GeometryFactory.hpp`, `src/GeometryFactory.cpp` — Primitive generation
- `src/STLLoader.hpp`, `src/STLLoader.cpp` — STL file import
- `src/GeometryTraits.hpp` — Type traits for geometry specialization

---

## Diagrams Index

| Diagram | Description | Last Updated |
|---------|-------------|--------------|
| [`msd-assets-core.puml`](../../docs/msd/msd-assets/msd-assets-core.puml) | High-level architecture overview | 2025-12-28 |
| [`asset-registry.puml`](../../docs/msd/msd-assets/asset-registry.puml) | AssetRegistry singleton cache system | 2025-12-28 |
| [`asset-geometry.puml`](../../docs/msd/msd-assets/asset-geometry.puml) | Asset and Geometry container system | 2025-12-28 |
| [`geometry-factory.puml`](../../docs/msd/msd-assets/geometry-factory.puml) | Primitive geometry generation | 2025-12-28 |
| [`stl-loader.puml`](../../docs/msd/msd-assets/stl-loader.puml) | STL file loading and parsing | 2025-12-28 |

---

## Conventions

### Naming Conventions
- Classes: `PascalCase`
- Functions/Methods: `camelCase`
- Member variables: `snake_case_` (trailing underscore)
- Constants: `kPascalCase`
- Namespaces: `snake_case`

### Code Organization
- Headers in `src/` (no separate include directory for this library)
- Implementation in `.cpp` files
- Templates in headers (defined inline)
- One class per header file

### Documentation
- Public APIs: Doxygen-style comments with `@brief`, `@param`, `@return`
- Brief class-level documentation explaining purpose and thread safety
- Implementation notes in source files where non-obvious

---

## Getting Help

### For AI Assistants
1. Start with this document for architectural context
2. Reference the PlantUML diagrams in `docs/msd/msd-assets/` for component relationships:
   - [`msd-assets-core.puml`](../../docs/msd/msd-assets/msd-assets-core.puml) for high-level overview
   - Component-specific diagrams for detailed implementation
3. Check header files for detailed interface documentation
4. Refer to the main project [CLAUDE.md](../../CLAUDE.md) for overall coding standards

### For Developers
- API documentation: See header file comments
- Example usage: See test files in `test/`
- PlantUML diagrams: `docs/msd/msd-assets/`
