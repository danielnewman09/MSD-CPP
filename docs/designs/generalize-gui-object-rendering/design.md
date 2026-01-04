# Design: Generalize GUI Object Rendering

## Summary

This design generalizes the msd-gui rendering system to support arbitrary geometry types (pyramids, cubes, spheres, etc.) with arbitrary transforms (position and rotation) while maintaining a single unified vertex buffer for performance. The GPUManager will integrate with the msd-sim `Object` class to extract transform and color information, enabling future simulation-driven rendering. The design introduces a geometry registry to manage multiple geometry types within a single buffer and enhances instance data to include full 4x4 model matrices for rotation support.

## Architecture Changes

### PlantUML Diagram
See: `./generalize-gui-object-rendering.puml`

### New Components

#### GeometryInfo
- **Purpose**: Registry entry tracking a geometry type's location within the unified vertex buffer
- **Header location**: `msd/msd-gui/src/SDLGPUManager.hpp`
- **Key interfaces**:
  ```cpp
  struct GeometryInfo {
      uint32_t baseVertex;   // Starting vertex index in unified buffer
      uint32_t vertexCount;  // Number of vertices for this geometry
  };
  ```
- **Dependencies**: None (pure data struct)
- **Thread safety**: Immutable after registration
- **Error handling**: N/A (data-only struct)

### Modified Components

#### GPUManager
- **Current location**: `msd/msd-gui/src/SDLGPUManager.hpp`, `msd/msd-gui/src/SDLGPUManager.cpp`
- **Changes required**:
  1. **Replace pyramid-specific storage** with geometry registry:
     - Remove: `size_t pyramidVertexCount_`
     - Add: `std::vector<GeometryInfo> geometryRegistry_`
     - Add: `std::unordered_map<std::string, uint32_t> geometryNameToIndex_`
     - Add: `size_t totalVertexCount_`

  2. **Enhance InstanceData structure** to support rotation and geometry selection:
     ```cpp
     struct InstanceData {
         float modelMatrix[16];   // 4x4 transform matrix (translation + rotation)
         float color[3];           // RGB color
         uint32_t geometryIndex;   // Index into geometryRegistry_
         uint32_t padding;         // Alignment padding (16-byte boundary)
     };
     ```
     - Size: 84 bytes per instance (64 + 12 + 4 + 4)
     - Alignment: 16-byte for matrix operations

  3. **Replace instance management interface** with Object-based API:
     - Remove: `addInstance(float posX, float posY, float posZ, float r, float g, float b)`
     - Add: `size_t addObject(const msd_sim::Object& object)`
     - Modify: `removeObject(size_t index)` (keep signature, change implementation)
     - Add: `void updateObjects()` — rebuilds instance data from Objects
     - Modify: `clearObjects()` (keep signature, change implementation)
     - Modify: `size_t getObjectCount()` (keep signature, change implementation)

  4. **Add geometry registration system**:
     ```cpp
     private:
     uint32_t registerGeometry(const std::string& name,
                                const std::vector<Vertex>& vertices);
     ```
     - Called during construction to pre-register pyramid and cube geometries
     - Appends vertices to unified buffer
     - Returns geometry index for future lookups

  5. **Add Object → InstanceData conversion**:
     ```cpp
     private:
     InstanceData buildInstanceData(const msd_sim::Object& object);
     Eigen::Matrix4f createModelMatrix(const msd_sim::ReferenceFrame& transform);
     ```
     - `buildInstanceData()`: Reads Object transform, color, and geometry name
     - `createModelMatrix()`: Converts ReferenceFrame (position + EulerAngles) to 4x4 matrix

  6. **Modify constructor** to pre-register base geometries:
     - Create pyramid geometry via `GeometryFactory::createPyramid()`
     - Create cube geometry via `GeometryFactory::createCube()`
     - Register both in unified vertex buffer
     - Store geometry indices in registry

  7. **Store Object references** instead of instance data:
     - Add: `std::vector<size_t> objectIndices_` — indices into SDLApplication's mock object vector
     - Note: For this ticket, we store indices; future tickets will use Object pointers/references

  8. **Modify render()** to rebuild instance data each frame:
     - Call `updateObjects()` to rebuild instance buffer from Object transforms
     - Upload updated instance data to GPU
     - Use geometry registry to determine draw call parameters

- **Backward compatibility**: Breaking changes to public interface
  - Old `addInstance(float, float, float, float, float, float)` is removed
  - New `addObject(Object&)` and `updateObjects()` replace it
  - This is acceptable per ticket requirement: "do not consider backwards compatibility with the current msd-gui or msd-exe libraries"

#### SDLApplication
- **Current location**: `msd/msd-gui/src/SDLApp.hpp`, `msd/msd-gui/src/SDLApp.cpp`
- **Changes required**:
  1. **Add mock Object storage**:
     ```cpp
     private:
     std::vector<msd_sim::Object> mockObjects_;
     std::vector<msd_assets::Asset> mockAssets_;  // Own assets for Object references
     ```
     - `mockObjects_`: Stores Object instances that SDLApplication manages
     - `mockAssets_`: Owns Asset instances (pyramid, cube) so Objects can reference them

  2. **Modify event handling** in `handleEvents()`:
     - **'Z' key**: Create pyramid Object with random position, orientation, color
     - **'V' key** (NEW): Create cube Object with random position, orientation, color
     - **'X' key**: Remove last Object from `mockObjects_`
     - **'C' key**: Clear all Objects from `mockObjects_`

  3. **Add helper method**:
     ```cpp
     private:
     void spawnRandomObject(const std::string& geometryType);
     ```
     - Creates Object with random transform (position + EulerAngles)
     - Assigns random color
     - Adds to `mockObjects_` vector
     - Notifies GPUManager (or GPUManager reads on next update)

  4. **Modify main loop** in `runApp()`:
     - Before `gpuManager_->render()`, call `gpuManager_->updateObjects()`
     - GPUManager reads `mockObjects_` and rebuilds instance data

  5. **Constructor changes**:
     - Initialize `mockAssets_` with pyramid and cube Assets
     - Pre-load these from AssetRegistry or create via GeometryFactory

- **Backward compatibility**: Event handling changes (new key bindings, modified behavior)
  - This is acceptable per ticket requirements

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| GPUManager (modified) | msd_sim::Object | Read-only access | Reads transform, color, geometry name |
| GPUManager (modified) | msd_assets::VisualGeometry | Geometry conversion | Converts geometry to GPU vertex format |
| GPUManager (modified) | msd_sim::ReferenceFrame | Transform extraction | Builds 4x4 model matrix |
| SDLApplication (modified) | msd_sim::Object | Ownership | Creates and owns mock Objects |
| SDLApplication (modified) | msd_assets::Asset | Ownership | Owns Assets that Objects reference |
| GeometryInfo | GPUManager | Composition | Stored in geometry registry |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| None | N/A | No existing automated tests for msd-gui | Manual testing via msd-exe |

### New Tests Required

#### Unit Tests

No unit tests planned for this ticket (per acceptance criteria: human-in-the-loop verification).

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| Manual: Press 'Z' key multiple times | SDLApplication, GPUManager, Object | Multiple pyramid Objects render with random positions, orientations, colors |
| Manual: Press 'V' key multiple times | SDLApplication, GPUManager, Object | Multiple cube Objects render with random positions, orientations, colors |
| Manual: Mix 'Z' and 'V' keys | SDLApplication, GPUManager, GeometryInfo | Both geometry types render simultaneously in single buffer |
| Manual: Press 'X' to remove | SDLApplication, GPUManager | Last added Object disappears |
| Manual: Press 'C' to clear | SDLApplication, GPUManager | All Objects disappear |
| Manual: Camera movement with Objects visible | Camera3D, GPUManager | Objects transform correctly in view space |

## Design Decisions

### Single Buffer Architecture

**Decision**: Store all geometry types in a single unified vertex buffer.

**Rationale**:
- **Performance requirement**: Ticket explicitly requires "The SDLGPUManager shall store 3D vertex information in a single buffer"
- **GPU efficiency**: Single buffer bind per frame reduces API overhead
- **Cache coherency**: Contiguous vertex data improves GPU cache utilization

**Trade-offs**:
| Aspect | Benefit | Cost |
|--------|---------|------|
| Performance | Fewer buffer binds, better cache | Static geometry (no runtime additions without realloc) |
| Memory | Contiguous allocation | Pre-allocated max size (wasted space if unused) |
| Complexity | Simpler render loop | Geometry registry bookkeeping |

**Implementation details**:
- Geometries registered at startup (pyramid, cube)
- `GeometryInfo` tracks `{baseVertex, vertexCount}` per geometry
- Draw calls use `baseVertex` parameter for indexed drawing

### Model Matrix in Instance Data

**Decision**: Include full 4x4 model matrix (64 bytes) in per-instance data instead of separate position/rotation fields.

**Rationale**:
- **Shader simplicity**: Vertex shader computes `MVP * vertex` directly
- **Future extensibility**: Supports non-uniform scaling, shearing if needed
- **Consistency**: Matches standard 3D rendering practices

**Alternative considered**: Store position (12 bytes) + quaternion (16 bytes) = 28 bytes
- **Rejected because**:
  - Requires shader to build matrix (more complex shader code)
  - Quaternion → matrix conversion on GPU is slower than precomputing on CPU
  - CPU builds matrix once per frame, GPU uses it for every vertex (better to amortize cost)

**Memory impact**:
- 84 bytes per instance (vs. ~50 bytes for position + quaternion + color)
- For 1000 instances: 84 KB vs. 50 KB (34 KB difference)
- **Acceptable**: Well within modern GPU memory constraints

### Geometry Index per Instance

**Decision**: Include `geometryIndex` (4 bytes) in instance data.

**Rationale**:
- **Current use**: Identifies which geometry to draw (pyramid vs. cube)
- **Future extensibility**: Enables per-geometry materials, textures, shader variants
- **Minimal cost**: 4 bytes per instance

**Note**: For this ticket, shader does not use `geometryIndex` (all geometries share same shader). Future tickets can leverage this field for advanced rendering.

### Object Integration Approach

**Decision**: SDLApplication owns mock Objects and GPUManager reads them each frame.

**Rationale**:
- **Ticket constraint**: "The msd-sim library shall not be modified by this ticket"
- **Temporary solution**: Mock Objects satisfy requirement #4 ("SDLGPUManager shall mock its own vector of objects")
- **Future path**: When simulation integration happens, replace `mockObjects_` with `engine_.getWorldModel().getObjects()`

**Ownership model**:
```
SDLApplication
  ├── owns mockObjects_ (vector<Object>)
  ├── owns mockAssets_ (vector<Asset>)
  └── owns GPUManager (unique_ptr)
        └── reads mockObjects_ (non-owning access)
```

**Why not pass Objects to GPUManager?**
- Objects are mutable (simulation updates them)
- GPUManager should not own Objects (not its responsibility)
- Read-only access each frame is cleaner separation of concerns

**Implementation**:
- For this ticket: SDLApplication passes `mockObjects_` reference to GPUManager
- GPUManager stores non-owning reference (`const std::vector<Object>&`)
- `updateObjects()` rebuilds instance data from current Object state

## Design Decisions

### Resolved Questions

1. **Geometry registration timing**
   - **Decision**: Option A — Register geometries at GPUManager construction (pyramid, cube) as a static set
   - **Rationale**: Simple, predictable, meets ticket requirements. External database functionality provides objects registered on startup. Option B (runtime registration) can be added in a future ticket if needed.

2. **Object → GPUManager communication**
   - **Decision**: Option A — GPUManager holds non-owning reference to `mockObjects_` vector, reads on `updateObjects()`
   - **Rationale**: Clean separation of concerns. Vertex data is used by GPUManager but owned by external service. Non-owning reference establishes clear ownership hierarchy per project memory management conventions.

3. **Shader modifications scope**
   - **Decision**: Option A — Minimal changes, only add model matrix to vertex shader
   - **Rationale**: Meets ticket requirements with simplest implementation. `geometryIndex` field is included for future extensibility but unused in this ticket.

### Prototype Required

None. The design is straightforward integration of existing components with well-understood matrix math and GPU buffer management.

### Requirements Clarification

1. **"The SDLGPUManager shall use the `Object` class to derive position, orientation, and color"**
   - **Decision**: Rebuild instance data each frame from SDLApplication's `mockObjects_` vector (stateless GPUManager render)
   - **Rationale**: Duplicated information must be sent to the GPU at render time regardless. Attempting to maintain perfect references to `Object` instances would be problematic.

2. **"Demonstrating this functionality shall be done by adding a handler for the 'V' key"**
   - **Decision**: 'V' always spawns cubes, 'Z' always spawns pyramids (explicit per requirements)

3. **"The use of a single buffer for vertex data"**
   - **Decision**: Single vertex buffer containing all geometry types (pyramid vertices followed by cube vertices)
   - **Performance documentation**: See Performance Analysis section

## Performance Analysis

### Single Buffer Approach

**Benefits**:
1. **Reduced API calls**: One `bindVertexBuffer()` per frame instead of N (one per geometry type)
2. **GPU cache coherency**: Contiguous vertex data improves cache hit rate
3. **Simplified state management**: Single pipeline state for all geometries

**Costs**:
1. **Static geometry set**: Cannot add new geometry types without buffer reallocation
2. **Memory waste**: Must pre-allocate buffer for max vertex count

**Note on Fragmentation**: The vertex buffer (geometry definitions) is static after registration — pyramid and cube vertices are never removed, so no fragmentation occurs. The instance buffer (per-object data) is rebuilt every frame from the `mockObjects_` vector, so removing an object simply shrinks the vector and the next frame's rebuild contains no gaps. This "rebuild each frame" approach is ideal because instance count is small (1000 max), 84 KB upload per frame is trivial for modern GPUs, and object transforms change every frame anyway during simulation.

**Measurements** (estimated for 1000 instances):
| Metric | Single Buffer | Multiple Buffers | Delta |
|--------|---------------|------------------|-------|
| Buffer binds/frame | 1 | 2 (pyramid + cube) | -50% |
| Vertex buffer size | ~2 KB (18 + 36 vertices) | Same | 0% |
| Instance buffer size | 84 KB | Same | 0% |

**Conclusion**: Single buffer approach provides measurable performance benefit (fewer API calls) with acceptable memory trade-offs for this use case.

### Memory Limitations

**Single buffer constraints**:
- SDL3 GPU buffer max size: Typically 256 MB+ (driver dependent)
- Vertex size: ~32 bytes (position + color + normal)
- Max vertices in single buffer: ~8 million vertices
- Pyramid: 18 vertices, Cube: 36 vertices
- **Current usage**: 54 vertices total → 1.7 KB
- **Headroom**: 99.9%+ of max buffer size available for future geometries

**Instance buffer constraints**:
- Instance data size: 84 bytes per instance
- Pre-allocated max instances: 1000 (per current code)
- Instance buffer size: 84 KB
- **Recommendation**: Keep 1000 instance limit for this ticket (adequate for demonstration)

**Conclusion**: Single buffer approach has no practical memory limitations for this application. Limits will be hit on instance count (1000) or geometry complexity (millions of vertices per model) long before buffer size limits.

## Implementation Notes

### Build Integration
- No new libraries required
- Add `#include <msd-sim/src/Environment/Object.hpp>` to GPUManager
- Add `#include <msd-sim/src/Environment/ReferenceFrame.hpp>` to GPUManager
- msd-gui already depends on msd-sim (via Engine), so no CMakeLists.txt changes

### Shader Changes Required

**Position3DColorTransform.vert** (vertex shader):
```glsl
// Current:
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inNormal;
layout(location = 3) in vec3 instancePosition;  // Per-instance
layout(location = 4) in vec3 instanceColor;     // Per-instance

// Modified:
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inNormal;
layout(location = 3) in mat4 instanceModel;     // Per-instance (4 locations: 3-6)
layout(location = 7) in vec3 instanceColor;     // Per-instance
layout(location = 8) in uint instanceGeometryIndex;  // Per-instance

// Vertex transformation:
// Old: gl_Position = ubo.mvp * vec4(inPosition + instancePosition, 1.0);
// New: gl_Position = ubo.viewProjection * instanceModel * vec4(inPosition, 1.0);
```

**Note**: Matrix occupies 4 attribute locations (3, 4, 5, 6). Adjust subsequent attribute locations accordingly.

**Uniform buffer changes**:
```cpp
// Old:
struct UniformData {
    float mvp[16];  // Model-View-Projection matrix
};

// New:
struct UniformData {
    float viewProjection[16];  // View-Projection matrix only (model is per-instance)
};
```

### Testing Strategy

**Manual test plan** (human-in-the-loop):
1. Launch msd-exe
2. Press 'Z' 5 times → Verify 5 pyramids appear with random positions, orientations, colors
3. Press 'V' 3 times → Verify 3 cubes appear with random positions, orientations, colors
4. Verify both pyramids and cubes are visible simultaneously
5. Move camera (WASD, QE, arrows) → Verify objects transform correctly in view
6. Press 'X' → Verify last added object disappears
7. Press 'C' → Verify all objects disappear
8. Press 'Z' → Verify new pyramid appears after clear

**Expected results**: All functional requirements met, visually confirmed.

## Future Enhancements (Out of Scope)

1. **Simulation integration**: Replace `mockObjects_` with `engine_.getWorldModel().getRenderObjectIndices()`
2. **Per-geometry materials**: Use `geometryIndex` to lookup material properties
3. **Texture support**: Add texture coordinates to vertex data, bind textures per geometry
4. **Dynamic geometry loading**: Support adding new geometry types at runtime
5. **Geometry instancing optimization**: Group instances by geometry type for fewer draw calls
6. **Frustum culling**: Skip rendering Objects outside camera view
7. **Level-of-detail (LOD)**: Switch geometry based on distance from camera
