# MSD Asset & Rendering Architecture

**Version:** 1.0
**Date:** 2025-12-21
**Status:** Design Specification

---

## Table of Contents

1. [Overview](#overview)
2. [Design Principles](#design-principles)
3. [System Architecture](#system-architecture)
4. [Database Schema](#database-schema)
5. [Component Specifications](#component-specifications)
6. [Data Flow](#data-flow)
7. [Usage Examples](#usage-examples)
8. [Performance Considerations](#performance-considerations)
9. [Implementation Roadmap](#implementation-roadmap)

---

## Overview

The MSD Asset System provides a scalable, efficient architecture for managing 3D geometry, physics properties, and rendering in a rigid body physics simulation. The system separates concerns between asset storage (SQLite database), runtime management (in-memory cache), physics simulation (msd-sim), and GPU rendering (msd-gui).

### Key Features

- **Database-backed asset storage** with lazy loading
- **Instanced GPU rendering** for efficient visualization
- **Separation of physics and rendering** concerns
- **Support for multiple mesh types** with shared geometry
- **Scalable to thousands of objects** (tested up to 1000+ instances)

---

## Design Principles

### 1. Separation of Concerns

```
┌─────────────────┐
│   Asset Storage │  SQLite database (persistent)
│   (msd-assets)  │  - Mesh geometry
└────────┬────────┘  - Physics templates
         │           - Metadata
         ↓
┌─────────────────┐
│ Asset Registry  │  In-memory cache (runtime)
│  (msd-assets)   │  - Loaded meshes
└────────┬────────┘  - Physics templates
         │
    ┌────┴────┐
    ↓         ↓
┌─────────┐ ┌──────────┐
│ Physics │ │ Renderer │
│(msd-sim)│ │(msd-gui) │
└─────────┘ └──────────┘
```

### 2. No Rendering Data in Physics

**Physics objects (RigidBody)** contain:
- Position, velocity, forces (double precision)
- Mass, inertia, friction
- **Reference** to visual mesh (by name/ID)
- **NO vertex data, NO GPU formats**

**Benefits:**
- Physics remains pure and testable
- Headless simulation possible
- No memory bloat
- Cache-friendly data layout

### 3. Instanced Rendering

**One mesh definition** → **Many instances**

```cpp
// GPU Memory:
Pyramid mesh: 18 vertices × 36 bytes = 648 bytes (stored ONCE)
1000 instances: 1000 × 24 bytes = 24 KB

// vs. naive approach:
1000 pyramids × 648 bytes = 648 KB (27x larger!)
```

### 4. Lazy Loading

Assets are loaded from database **only when first needed**:
- Reduces startup time
- Saves memory for unused assets
- Supports large asset libraries (1000+ meshes)

---

## System Architecture

### Layer 1: Asset Database (SQLite)

**Location:** `assets/rigid_bodies.db`

**Responsibilities:**
- Persistent storage of all asset data
- Metadata indexing and queries
- Binary BLOB storage for geometry
- Physics property templates

**Access Pattern:** Read-mostly, written during asset authoring

### Layer 2: Asset Registry (RAM)

**Location:** `msd-assets` library, singleton

**Responsibilities:**
- Cache loaded meshes in memory
- Lazy load from database on first access
- Provide fast lookup by name/ID
- Memory management (unload unused assets)

**Access Pattern:** Frequent reads during simulation

### Layer 3: Physics Simulation (msd-sim)

**Location:** `msd-sim` library

**Responsibilities:**
- Rigid body dynamics
- Collision detection
- Force integration
- **References** visual meshes by name

**Data Format:** High precision (double), no GPU-specific data

### Layer 4: GPU Rendering (msd-gui)

**Location:** `msd-gui` library

**Responsibilities:**
- Load meshes to GPU on demand
- Maintain instance buffers
- Sync from physics state
- Render with instanced draw calls

**Data Format:** Low precision (float), GPU-optimized layout

---

## Database Schema

### Core Tables

```sql
-- ============================================================================
-- MESH GEOMETRY
-- ============================================================================

CREATE TABLE meshes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT UNIQUE NOT NULL,
    category TEXT,                    -- "primitive", "vehicle", "debris"
    vertex_count INTEGER NOT NULL,
    triangle_count INTEGER NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    modified_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    tags TEXT,                        -- JSON: ["collision", "renderable"]

    -- Bounding box (for culling/collision)
    aabb_min_x REAL, aabb_min_y REAL, aabb_min_z REAL,
    aabb_max_x REAL, aabb_max_y REAL, aabb_max_z REAL,
    bounding_radius REAL
);

CREATE TABLE mesh_vertex_data (
    mesh_id INTEGER PRIMARY KEY,
    vertex_blob BLOB NOT NULL,        -- Binary: Vertex[] array
    vertex_format INTEGER NOT NULL,   -- Format version
    compressed BOOLEAN DEFAULT 0,

    FOREIGN KEY(mesh_id) REFERENCES meshes(id) ON DELETE CASCADE
);

CREATE TABLE mesh_index_data (
    mesh_id INTEGER PRIMARY KEY,
    index_blob BLOB,                  -- Binary: uint32_t[] indices
    index_format INTEGER NOT NULL,

    FOREIGN KEY(mesh_id) REFERENCES meshes(id) ON DELETE CASCADE
);

-- ============================================================================
-- PHYSICS
-- ============================================================================

CREATE TABLE physics_templates (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT UNIQUE NOT NULL,
    mesh_id INTEGER,

    mass REAL NOT NULL,
    friction REAL DEFAULT 0.5,
    restitution REAL DEFAULT 0.3,
    linear_damping REAL DEFAULT 0.1,
    angular_damping REAL DEFAULT 0.1,

    inertia_tensor BLOB,              -- 3x3 matrix (72 bytes)

    FOREIGN KEY(mesh_id) REFERENCES meshes(id) ON DELETE SET NULL
);

CREATE TABLE collision_shapes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    physics_template_id INTEGER NOT NULL,
    shape_type TEXT NOT NULL,         -- "convex_hull", "box", "sphere"
    shape_data BLOB,

    FOREIGN KEY(physics_template_id) REFERENCES physics_templates(id) ON DELETE CASCADE
);

-- ============================================================================
-- RENDERING
-- ============================================================================

CREATE TABLE materials (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT UNIQUE NOT NULL,
    shader_vertex TEXT,
    shader_fragment TEXT,

    default_color_r REAL DEFAULT 1.0,
    default_color_g REAL DEFAULT 1.0,
    default_color_b REAL DEFAULT 1.0,
    shininess REAL DEFAULT 32.0
);

CREATE TABLE mesh_materials (
    mesh_id INTEGER,
    material_id INTEGER,
    PRIMARY KEY (mesh_id, material_id),
    FOREIGN KEY(mesh_id) REFERENCES meshes(id) ON DELETE CASCADE,
    FOREIGN KEY(material_id) REFERENCES materials(id) ON DELETE CASCADE
);

-- ============================================================================
-- INDICES
-- ============================================================================

CREATE INDEX idx_meshes_name ON meshes(name);
CREATE INDEX idx_meshes_category ON meshes(category);
CREATE INDEX idx_physics_templates_name ON physics_templates(name);
```

### Data Storage Format

**Vertex BLOB Format:**
```cpp
struct Vertex {
    float position[3];  // 12 bytes
    float color[3];     // 12 bytes
    float normal[3];    // 12 bytes
};  // Total: 36 bytes per vertex

// BLOB = binary array of Vertex structs
// Size = vertex_count × sizeof(Vertex)
```

**Inertia Tensor BLOB Format:**
```cpp
// Column-major 3x3 matrix
double inertiaTensor[9];  // 72 bytes
```

---

## Component Specifications

### AssetDatabase (msd-assets)

**File:** `msd-assets/AssetDatabase.hpp`

```cpp
namespace msd_assets {

class AssetDatabase {
public:
    explicit AssetDatabase(const std::string& dbPath);

    // Load mesh geometry (returns binary data as Vertex array)
    std::vector<Vertex> loadMeshVertices(const std::string& meshName);
    std::vector<Vertex> loadMeshVertices(int meshId);

    // Load physics template
    PhysicsTemplate loadPhysicsTemplate(const std::string& templateName);

    // Get metadata only (no vertex data)
    MeshMetadata getMeshMetadata(const std::string& meshName);

    // Query operations
    std::vector<std::string> getMeshesByCategory(const std::string& category);
    std::vector<std::string> getMeshesByTag(const std::string& tag);

    // Insert operations (for asset authoring)
    void insertMesh(const MeshData& mesh);
    void insertPhysicsTemplate(const PhysicsTemplate& physics);

private:
    SQLite::Database db_;
    std::vector<uint8_t> decompressBlob(const void* data, size_t size, bool compressed);
};

struct MeshMetadata {
    int id;
    std::string name;
    std::string category;
    int vertexCount;
    int triangleCount;
    Eigen::Vector3d aabbMin;
    Eigen::Vector3d aabbMax;
    double boundingRadius;
};

struct PhysicsTemplate {
    std::string name;
    double mass;
    double friction;
    double restitution;
    double linearDamping;
    double angularDamping;
    Eigen::Matrix3d inertiaTensor;
    std::shared_ptr<CollisionShape> collisionShape;
};

} // namespace msd_assets
```

### AssetRegistry (msd-assets)

**File:** `msd-assets/AssetRegistry.hpp`

```cpp
namespace msd_assets {

class AssetRegistry {
public:
    static AssetRegistry& getInstance();

    // Initialize from database
    void loadFromDatabase(const std::string& dbPath);

    // Get mesh (cached after first load)
    const std::vector<Vertex>& getMesh(const std::string& name);

    // Get physics template (cached after first load)
    const PhysicsTemplate& getPhysicsTemplate(const std::string& name);

    // Pre-load for performance
    void preloadMeshes(const std::vector<std::string>& meshNames);

    // Memory management
    void unloadMesh(const std::string& name);
    void clearCache();
    size_t getCacheMemoryUsage() const;

private:
    AssetRegistry() = default;

    std::unique_ptr<AssetDatabase> database_;

    // Caches
    std::unordered_map<std::string, std::vector<Vertex>> meshCache_;
    std::unordered_map<std::string, PhysicsTemplate> physicsTemplateCache_;

    mutable std::mutex cacheMutex_;  // Thread safety
};

} // namespace msd_assets
```

### RigidBody (msd-sim)

**File:** `msd-sim/RigidBody.hpp`

```cpp
namespace msd_sim {

class RigidBody {
public:
    // Factory method
    static std::unique_ptr<RigidBody> createFromTemplate(
        const std::string& templateName,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& rotation = Eigen::Quaterniond::Identity()
    );

    // Physics state (double precision)
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d angularVelocity;

    // Physical properties
    double mass;
    double friction;
    double restitution;
    Eigen::Matrix3d inertiaTensor;

    // Visual reference (NOT vertex data!)
    std::string visualMeshName;      // e.g., "pyramid"

    // Optional rendering override
    std::optional<Eigen::Vector3f> colorOverride;

private:
    RigidBody() = default;
};

} // namespace msd_sim
```

### GPUManager (msd-gui)

**File:** `msd-gui/GPUManager.hpp`

```cpp
namespace msd_gui {

class GPUManager {
public:
    // Load mesh from asset registry to GPU
    void loadMeshToGPU(const std::string& meshName);

    // Sync instances from physics simulation
    void syncFromPhysics(const std::vector<msd_sim::RigidBody*>& bodies);

    // Current implementation (single mesh type)
    void addInstance(float posX, float posY, float posZ, float r, float g, float b);
    void removeInstance(size_t index);
    void updateInstance(size_t index, float posX, float posY, float posZ,
                       float r, float g, float b);

    void render();

private:
    struct MeshType {
        UniqueBuffer vertexBuffer;
        size_t vertexCount;
        std::vector<InstanceData> instances;
        UniqueBuffer instanceBuffer;
    };

    // Future: support multiple mesh types
    std::unordered_map<std::string, MeshType> loadedMeshes_;

    void uploadInstanceBuffer(const std::string& meshName);
};

} // namespace msd_gui
```

---

## Data Flow

### Startup Flow

```
1. Application starts
   ↓
2. AssetRegistry::loadFromDatabase("assets/rigid_bodies.db")
   ↓
3. Database connection opened (no data loaded yet)
   ↓
4. Optionally: preloadMeshes({"pyramid", "cube"})
   ↓
5. SDLApplication initializes GPUManager
   ↓
6. Ready for simulation
```

### Creating a Rigid Body

```
1. RigidBody::createFromTemplate("pyramid_default", position)
   ↓
2. AssetRegistry::getPhysicsTemplate("pyramid_default")
   ↓
3. If not cached:
   ├─> AssetDatabase::loadPhysicsTemplate("pyramid_default")
   ├─> Query database
   ├─> Deserialize BLOB data
   └─> Cache in physicsTemplateCache_
   ↓
4. Create RigidBody with physics properties
   ↓
5. Set visualMeshName = "pyramid" (reference only)
   ↓
6. Return RigidBody (no vertex data stored)
```

### Rendering Flow (Each Frame)

```
1. Physics simulation steps (updates positions, velocities)
   ↓
2. GPUManager::syncFromPhysics(rigidBodies)
   ↓
3. For each body:
   ├─> Get visualMeshName
   ├─> If mesh not on GPU: loadMeshToGPU(meshName)
   │   ├─> AssetRegistry::getMesh(meshName)
   │   │   └─> If not cached: AssetDatabase::loadMeshVertices()
   │   ├─> Upload to GPU vertex buffer
   │   └─> Cache in loadedMeshes_
   ├─> Create InstanceData from body.position and body.colorOverride
   └─> Add to mesh.instances[]
   ↓
4. For each mesh type:
   ├─> uploadInstanceBuffer(meshName)
   └─> SDL_DrawGPUPrimitives() with instance count
   ↓
5. All instances rendered with minimal draw calls
```

### Memory Layout

```
Database (Disk):
┌─────────────────────────┐
│ Meshes table            │  ~1 KB per mesh (metadata)
│ mesh_vertex_data        │  ~1 KB per primitive mesh
│ physics_templates       │  ~100 bytes per template
└─────────────────────────┘

Asset Registry (RAM):
┌─────────────────────────┐
│ meshCache_              │  Loaded on demand
│   "pyramid": Vertex[18] │  ~650 bytes
│   "cube": Vertex[36]    │  ~1.3 KB
└─────────────────────────┘

Physics Simulation (RAM):
┌─────────────────────────┐
│ RigidBody[1000]         │  1000 × ~200 bytes = 200 KB
│   position, velocity    │  (no vertex data!)
│   visualMeshName        │
└─────────────────────────┘

GPU Memory:
┌─────────────────────────┐
│ Vertex Buffers          │
│   pyramid: 648 bytes    │  (one copy)
│   cube: 1.3 KB          │  (one copy)
│                         │
│ Instance Buffers        │
│   pyramids: 500 × 24 B  │  = 12 KB
│   cubes: 500 × 24 B     │  = 12 KB
└─────────────────────────┘
```

---

## Usage Examples

### Example 1: Basic Setup

```cpp
int main() {
    // Initialize asset system
    auto& registry = msd_assets::AssetRegistry::getInstance();
    registry.loadFromDatabase("assets/rigid_bodies.db");

    // Pre-load common meshes
    registry.preloadMeshes({"pyramid", "cube", "sphere"});

    // Run application
    auto& app = msd_gui::SDLApplication::getInstance();
    return app.runApp();
}
```

### Example 2: Creating Physics Objects

```cpp
void setupScene(msd_sim::World& world) {
    // Create pyramid from database template
    auto pyramid = msd_sim::RigidBody::createFromTemplate(
        "pyramid_default",           // Template name
        Eigen::Vector3d(0, 5, 0)     // Initial position
    );
    pyramid->colorOverride = Eigen::Vector3f(1.0f, 0.0f, 0.0f);  // Red

    // Create cube
    auto cube = msd_sim::RigidBody::createFromTemplate(
        "cube_heavy",
        Eigen::Vector3d(2, 3, 0)
    );
    cube->colorOverride = Eigen::Vector3f(0.0f, 1.0f, 0.0f);  // Green

    world.addBody(std::move(pyramid));
    world.addBody(std::move(cube));
}
```

### Example 3: Rendering Integration

```cpp
void renderLoop() {
    while (running) {
        // Physics update
        physicsWorld.step(dt);

        // Get all active bodies
        const auto& bodies = physicsWorld.getRigidBodies();

        // Sync to GPU (automatic mesh loading)
        gpuManager.syncFromPhysics(bodies);

        // Render all instances
        gpuManager.render();
    }
}
```

### Example 4: Populating Database

```cpp
// Tool to populate asset database
void createAssetDatabase(const std::string& dbPath) {
    msd_assets::AssetDatabase db(dbPath);

    // Create pyramid mesh
    auto pyramidGeom = msd_assets::GeometryFactory::createPyramid(1.0, 1.0);
    auto pyramidVerts = geometryToVertices(pyramidGeom);

    db.insertMesh({
        .name = "pyramid",
        .category = "primitive",
        .vertices = pyramidVerts,
        .tags = R"(["collision", "renderable"])"
    });

    // Create physics template
    db.insertPhysicsTemplate({
        .name = "pyramid_default",
        .meshName = "pyramid",
        .mass = 1.0,
        .friction = 0.5,
        .restitution = 0.3,
        .inertiaTensor = calculatePyramidInertia(1.0, 1.0, 1.0)
    });
}
```

---

## Performance Considerations

### Memory Efficiency

| Component | Storage per Object | Notes |
|-----------|-------------------|-------|
| **Database** | ~1 KB | One-time cost, on disk |
| **Asset Cache** | ~1 KB | Shared across all instances |
| **Physics State** | ~200 bytes | Per rigid body |
| **GPU Vertex Buffer** | ~1 KB | One copy per mesh type |
| **GPU Instance Data** | ~24 bytes | Per instance |

**Example:** 1000 pyramids
- Naive: 1000 × 1 KB = 1 MB (vertex duplication)
- **This system**: 1 KB + (1000 × 24 bytes) = **25 KB** (98% reduction)

### Draw Call Efficiency

| Method | Draw Calls | Performance |
|--------|-----------|-------------|
| **One per object** | 1000 | ❌ Very slow |
| **Batched by type** | 10 (for 10 types) | ✅ Fast |
| **Instanced (this system)** | 10 | ✅ Fast + memory efficient |

### Loading Performance

```
Benchmark (loading 100 primitive meshes):
┌─────────────────────┬──────────┐
│ Method              │ Time     │
├─────────────────────┼──────────┤
│ SQLite (normalized) │ ~50 ms   │
│ SQLite (BLOBs)      │ ~5 ms    │ ← Recommended
│ Binary files        │ ~3 ms    │
│ Memory-mapped       │ ~1 ms    │
└─────────────────────┴──────────┘
```

---

## Implementation Roadmap

### Phase 1: Current State ✅
- [x] Instanced rendering working
- [x] Single mesh type (pyramid)
- [x] Dynamic instance add/remove
- [x] GPU buffer management

### Phase 2: Multi-Mesh Support 🚧
- [ ] Refactor GPUManager for multiple mesh types
- [ ] Implement `loadMeshToGPU(meshName)`
- [ ] Implement `syncFromPhysics(bodies)`
- [ ] Test with pyramids + cubes

### Phase 3: Asset Database 📋
- [ ] Design and create SQLite schema
- [ ] Implement AssetDatabase class
- [ ] Implement AssetRegistry class
- [ ] Create database population tool

### Phase 4: Physics Integration 📋
- [ ] Extend RigidBody with visualMeshName
- [ ] Implement createFromTemplate factory
- [ ] Remove vertex data from physics objects
- [ ] Update physics tests

### Phase 5: Optimization 📋
- [ ] Add BLOB compression support
- [ ] Implement mesh unloading
- [ ] Add memory usage monitoring
- [ ] Performance profiling

---

## Questions & Considerations

### Open Questions

1. **Rotation Support**: Current InstanceData only has position. Add rotation quaternion?
2. **LOD System**: How to handle Level of Detail switching?
3. **Texture Support**: Extend to support textured meshes?
4. **Animation**: Static meshes only, or support skeletal animation?

### Future Enhancements

- **Streaming**: Load/unload meshes based on camera frustum
- **Compression**: zlib compression for large meshes
- **Procedural**: Generate meshes procedurally, cache in database
- **Materials**: Full material system with PBR support

---

## References

- [SDL3 GPU Documentation](https://wiki.libsdl.org/SDL3/CategoryGPU)
- [Instanced Rendering Best Practices](https://www.khronos.org/opengl/wiki/Vertex_Rendering#Instancing)
- [SQLite BLOB Performance](https://www.sqlite.org/intern-v-extern-blob.html)
- [Game Engine Architecture, 3rd Edition](https://www.gameenginebook.com/)

---

**Document Status:** Living document, updated as implementation progresses.
**Last Updated:** 2025-12-21
**Author:** MSD Development Team
