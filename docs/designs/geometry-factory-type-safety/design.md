# Design: Geometry Factory Type Safety Fix

## Summary
This design fixes a type mismatch bug where `GeometryFactory::verticesToMeshRecord()` produces `MeshRecord` blobs containing `Vertex` structs (36 bytes each), but the method name and documentation suggest it produces raw coordinate data. The fix leverages the existing `BaseGeometry<T>` template infrastructure, which already correctly handles both visual and collision geometry through its `populateMeshRecord()` method, requiring only minimal changes to the factory.

## Architecture Changes

### PlantUML Diagram
See: `./geometry-factory-type-safety.puml`

### Root Cause Analysis

The current implementation has a mismatch between what `GeometryFactory` produces and what callers expect:

**Problem: GeometryFactory stores Vertex but presents a generic interface**
```cpp
// GeometryFactory::verticesToMeshRecord() - CURRENT IMPLEMENTATION
const size_t blobSize = vertexCount * sizeof(Vertex);  // 36 bytes each!
record.vertex_data.resize(blobSize);
Vertex* vertexData = reinterpret_cast<Vertex*>(record.vertex_data.data());
// Stores FULL Vertex structs with position + color + normal
```

The factory always produces Vertex blobs (36 bytes per vertex), which is correct for `VisualGeometry` but incorrect if someone expects to use the output for `CollisionGeometry` (which expects `msd_sim::Vector3D` at 24 bytes per vertex).

**Key Insight: Existing Infrastructure Already Solves This**

The `BaseGeometry<T>` template already provides:

1. **Raw coordinates constructor** that handles type conversion:
   ```cpp
   explicit BaseGeometry(const std::vector<msd_sim::Vector3D>& rawVertices, uint32_t objectId = 0)
   ```
   - For `VisualGeometry` (T=Vertex): Calls `computeVertexData()` to compute normals and convert to Vertex format
   - For `CollisionGeometry` (T=Vector3d): Stores raw coordinates directly

2. **`populateMeshRecord()` method** that correctly serializes `cachedVertices_` as type `T`:
   ```cpp
   msd_transfer::MeshRecord populateMeshRecord() const {
     // Serializes cachedVertices_ (of type vector<T>) correctly
   }
   ```

3. **MeshRecord constructor** that correctly deserializes based on template type `T`:
   ```cpp
   explicit BaseGeometry(const msd_transfer::MeshRecord& record, uint32_t objectId = 0)
   // Casts to T* (already fixed in current codebase)
   ```

### Simplified Solution

The fix requires updating `GeometryFactory::verticesToMeshRecord()` to use the existing `VisualGeometry` class:

**Updated Implementation:**
```cpp
msd_transfer::MeshRecord GeometryFactory::verticesToMeshRecord(
  const std::vector<msd_sim::Vector3D>& vertices)
{
  // Create VisualGeometry from raw coordinates
  // This computes normals and converts to Vertex format
  VisualGeometry geometry{vertices, 0};

  // Use populateMeshRecord() to serialize Vertex structs correctly
  return geometry.populateMeshRecord();
}
```

**Benefits of this approach:**
- **No new API methods needed** - No `createVisualCube()`/`createCollisionCube()` split required
- **Minimal code changes** - Only `verticesToMeshRecord()` needs modification (one function body)
- **Leverages existing infrastructure** - Uses well-tested `BaseGeometry<T>` methods
- **Clear type semantics** - Factory explicitly produces visual geometry; collision geometry users construct from raw vertices

### Modified Components

#### GeometryFactory
- **Current location**: `msd/msd-assets/src/GeometryFactory.hpp`, `GeometryFactory.cpp`
- **Changes required**:
  1. Update `verticesToMeshRecord()` implementation to use `VisualGeometry` class
  2. Update documentation to clarify that factory produces visual geometry blobs
- **Backward compatibility**:
  - API remains unchanged - same method signatures
  - Output format unchanged - still produces Vertex blobs (which was always the case)
  - Callers using output for `VisualGeometry` continue to work correctly

#### Usage Pattern for CollisionGeometry
- **Pattern**: Collision geometry users should construct from raw vertices, not from factory MeshRecords
- **Example**:
  ```cpp
  // For visual geometry (uses factory)
  auto meshRecord = GeometryFactory::createCube(1.0);
  VisualGeometry visual{meshRecord, objectId};  // Works correctly

  // For collision geometry (construct directly from coordinates)
  std::vector<msd_sim::Vector3D> cubeVertices = generateCubeVertices(1.0);
  CollisionGeometry collision{cubeVertices, objectId};  // Works correctly
  auto collisionRecord = collision.populateMeshRecord();  // For database storage
  ```

### Integration Points

| Modified Component | Affected Component | Integration Type | Notes |
|-------------------|-------------------|------------------|-------|
| GeometryFactory | VisualGeometry | Now uses internally | Factory uses geometry class for serialization |
| GeometryFactory | SDLGPUManager | Direct usage | No change - still produces visual geometry |
| GeometryFactory | msd-asset-gen | Direct usage | May need update if storing collision geometry |
| CollisionGeometry | Test fixtures | Usage pattern | Tests needing collision geometry construct from raw vertices |

## Detailed Design

### GeometryFactory::verticesToMeshRecord() Update

**Current Implementation (WORKS but duplicates logic):**
```cpp
msd_transfer::MeshRecord GeometryFactory::verticesToMeshRecord(
  const std::vector<msd_sim::Vector3D>& vertices)
{
  msd_transfer::MeshRecord record;
  const size_t vertexCount = vertices.size();
  record.vertex_count = static_cast<uint32_t>(vertexCount);

  // Allocate vertex_data BLOB
  const size_t blobSize = vertexCount * sizeof(Vertex);
  record.vertex_data.resize(blobSize);

  // Get pointer to BLOB data for writing
  Vertex* vertexData = reinterpret_cast<Vertex*>(record.vertex_data.data());

  // Process triangles and compute normals (duplicates computeVertexData logic)
  for (size_t i = 0; i + 2 < vertexCount; i += 3) {
    // ... normal computation and vertex population ...
  }

  return record;
}
```

**New Implementation (leverages existing infrastructure):**
```cpp
msd_transfer::MeshRecord GeometryFactory::verticesToMeshRecord(
  const std::vector<msd_sim::Vector3D>& vertices)
{
  // Create VisualGeometry from raw coordinates
  // - Computes normals via computeVertexData()
  // - Stores result in cachedVertices_ as vector<Vertex>
  VisualGeometry geometry{vertices, 0};

  // Serialize using existing populateMeshRecord()
  // - Correctly serializes vector<Vertex> to BLOB
  // - Sets vertex_count appropriately
  return geometry.populateMeshRecord();
}
```

**Key Changes:**
1. Remove duplicated normal computation logic (now uses `computeVertexData()`)
2. Use `VisualGeometry` constructor for type-safe conversion
3. Use `populateMeshRecord()` for correct serialization
4. Reduce code duplication between factory and geometry class

### Documentation Updates

The factory class documentation should be updated to clarify intent:

```cpp
/**
 * @brief Factory class for creating visual geometry primitives
 *
 * Provides static methods to generate MeshRecord objects for standard shapes.
 * All geometries are triangulated, centered at the origin, and formatted
 * for visual rendering (Vertex structs with position, color, and normal).
 *
 * For collision geometry, construct CollisionGeometry directly from raw
 * vertex coordinates using the BaseGeometry<msd_sim::Vector3D> constructor.
 *
 * Usage:
 *   // Visual geometry (for rendering)
 *   auto cubeMesh = GeometryFactory::createCube(1.0);
 *   VisualGeometry visual{cubeMesh, objectId};
 *
 *   // Collision geometry (for physics)
 *   std::vector<msd_sim::Vector3D> vertices = getCubeVertices(1.0);
 *   CollisionGeometry collision{vertices, objectId};
 */
class GeometryFactory { ... };
```

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `msd-assets/test/geometry_factory_test.cpp` | Factory output tests | None | Tests should pass unchanged (output format same) |
| `msd-assets/test/geometry_test.cpp` | `BaseGeometry` tests | None | Existing tests validate the infrastructure we're now using |
| `msd-gui/test/sdl_gpu_manager_test.cpp` | Visual geometry tests | None | Uses factory for visual geometry (correct use case) |

### Existing Tests That May Need Review

| Test File | Test Case | Potential Issue | Action Required |
|-----------|-----------|-----------------|------------------|
| Any test using factory output for `CollisionGeometry` | Collision tests | Incorrect usage | Update to construct from raw vertices |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| GeometryFactory | `verticesToMeshRecord_uses_visual_geometry_class` | Factory output matches `VisualGeometry::populateMeshRecord()` output |
| GeometryFactory | `roundtrip_factory_to_visual_geometry` | Factory → MeshRecord → VisualGeometry produces valid geometry |
| CollisionGeometry | `construct_from_raw_vertices` | CollisionGeometry can be constructed from `vector<Vector3d>` |
| CollisionGeometry | `roundtrip_raw_vertices` | Raw vertices → CollisionGeometry → MeshRecord → CollisionGeometry preserves data |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `visual_geometry_end_to_end` | GeometryFactory, VisualGeometry, MeshRecord | Factory → DB → VisualGeometry works correctly |
| `collision_geometry_from_raw_vertices` | CollisionGeometry, MeshRecord | Raw vertices → DB → CollisionGeometry works correctly |

## Open Questions

### Design Decisions (Human Input Needed)

1. **Should GeometryFactory expose raw vertex generation methods?**
   - Option A: Keep factory as-is, users call factory then extract vertices manually
     - Pros: Minimal API change
     - Cons: Awkward for collision geometry use case
   - Option B: Add `getCubeVertices(double size)` static method returning `vector<Vector3d>`
     - Pros: Clean API for collision geometry creation
     - Cons: Slightly more API surface
   - **Recommendation**: Option B is cleaner but not blocking for initial fix

2. **Should we add documentation about collision geometry creation pattern?**
   - Option A: Document in CLAUDE.md and class comments
     - Pros: Clear guidance for developers
     - Cons: More documentation to maintain
   - Option B: Rely on code examples in tests
     - Pros: Less documentation
     - Cons: Pattern may not be obvious
   - **Recommendation**: Option A - add brief guidance to CLAUDE.md

### Prototype Required

1. **Verify factory output is identical before and after change**
   - Uncertainty: Does `computeVertexData()` produce identical normals to current factory implementation?
   - Validation: Add test comparing old and new factory output byte-for-byte
   - Expected outcome: Output should be identical (both use same normal computation algorithm)

### Requirements Clarification

None - the simplified solution requires no additional requirements clarification.

## Implementation Checklist

### Phase 1: Update Factory Implementation
1. Modify `GeometryFactory::verticesToMeshRecord()` to use `VisualGeometry` class
2. Update class-level documentation to clarify visual geometry focus
3. Add tests verifying roundtrip behavior

### Phase 2: Documentation Updates
1. Update `msd/msd-assets/CLAUDE.md` with collision geometry creation pattern
2. Add example code showing both visual and collision geometry creation

## Validation Criteria

The design will be considered successful if:

1. **Factory correctness**: `GeometryFactory::createCube()` produces MeshRecord that can be correctly loaded by `VisualGeometry`
   - Blob size is `vertex_count * 36` bytes (Vertex size)
   - Round-trip produces identical vertex data

2. **Collision geometry pattern works**: `CollisionGeometry` can be constructed from raw vertex vectors
   - Constructor accepts `vector<Vector3d>`
   - `populateMeshRecord()` produces correct 24-byte-per-vertex blobs

3. **Rendering correctness**: GUI objects render at correct size and shape
   - 1.0 meter cube renders as 1.0 meter cube
   - Normals are computed correctly for proper lighting

4. **Test coverage**: All existing tests pass without modification (API unchanged)

5. **Code quality**: Factory no longer duplicates `computeVertexData()` logic
