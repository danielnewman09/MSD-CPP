# Design: Modularize SDLGPUManager Shader System

## Summary

This design refactors the SDLGPUManager to support multiple shader types through a shader policy system. Currently, GPUManager is hardcoded to use the PositionRotation3DColorTransform shader with full 4x4 model matrices, but rendering issues have been reported. This design introduces compile-time shader selection via C++20 template policies, allowing the same GPUManager to work with both simple position-only shaders (`Position3DColorTransform`) and full transform shaders (`PositionRotation3DColorTransform`) without runtime overhead. The abstraction enables easy addition of new shader types in the future while maintaining type safety and zero-cost abstraction.

## Architecture Changes

### PlantUML Diagram
See: `./modularize-gpu-shader-system.puml`

### New Components

#### ShaderPolicyBase (Interface)
- **Purpose**: Abstract interface defining shader configuration contract for runtime polymorphism (if Option B selected)
- **Header location**: `msd/msd-gui/src/ShaderPolicy.hpp`
- **Key interfaces**:
  ```cpp
  class ShaderPolicyBase {
  public:
      virtual ~ShaderPolicyBase() = default;

      virtual std::vector<SDL_GPUVertexAttribute> getVertexAttributes() const = 0;
      virtual SDL_GPUVertexBufferDescription getVertexBufferDescription() const = 0;
      virtual SDL_GPUVertexInputState getVertexInputState() const = 0;
      virtual std::vector<uint8_t> buildInstanceData(const msd_sim::Object& object) const = 0;
      virtual std::string getVertexShaderFile() const = 0;
      virtual std::string getFragmentShaderFile() const = 0;
      virtual size_t getInstanceDataSize() const = 0;
  };
  ```
- **Dependencies**: SDL3 GPU types, msd_sim::Object
- **Thread safety**: Stateless interface, implementations must be thread-safe
- **Error handling**: Methods return values, no exceptions from interface

#### PositionOnlyShaderPolicy
- **Purpose**: Shader policy for Position3DColorTransform shader (simple position offset rendering)
- **Header location**: `msd/msd-gui/src/ShaderPolicy.hpp`
- **Key interfaces**:
  ```cpp
  struct PositionOnlyInstanceData {
      float position[3];      // World position offset
      float color[3];         // RGB color
      uint32_t padding[2];    // 16-byte alignment
  };

  class PositionOnlyShaderPolicy : public ShaderPolicyBase {
  public:
      static constexpr const char* kShaderName = "PositionOnly";
      static constexpr const char* kVertexShaderFile = "Position3DColorTransform.vert";
      static constexpr const char* kFragmentShaderFile = "SolidColor.frag";

      std::vector<SDL_GPUVertexAttribute> getVertexAttributes() const override;
      SDL_GPUVertexBufferDescription getVertexBufferDescription() const override;
      SDL_GPUVertexInputState getVertexInputState() const override;
      std::vector<uint8_t> buildInstanceData(const msd_sim::Object& object) const override;
      std::string getVertexShaderFile() const override;
      std::string getFragmentShaderFile() const override;
      size_t getInstanceDataSize() const override { return sizeof(PositionOnlyInstanceData); }
  };
  ```
- **Dependencies**: SDL3 GPU, msd_sim::Object, msd_sim::ReferenceFrame
- **Thread safety**: All methods const and stateless (thread-safe)
- **Error handling**: Returns valid configurations, no exceptions

**Vertex Attribute Configuration:**
```cpp
// Per-vertex attributes (binding 0)
Location 0: Position (vec3, offset 0)
Location 1: Color (vec3, offset 12)
Location 2: Normal (vec3, offset 24)

// Per-instance attributes (binding 1)
Location 3: InstancePosition (vec3, offset 0)
Location 4: InstanceColor (vec3, offset 12)
```

**Instance Data Layout:**
```
Offset  Size  Field
0       12    position[3]     (float x, y, z)
12      12    color[3]        (float r, g, b)
24      8     padding[2]      (align to 16-byte boundary)
Total: 32 bytes
```

#### FullTransformShaderPolicy
- **Purpose**: Shader policy for PositionRotation3DColorTransform shader (full 4x4 model matrix rendering)
- **Header location**: `msd/msd-gui/src/ShaderPolicy.hpp`
- **Key interfaces**:
  ```cpp
  struct FullTransformInstanceData {
      float modelMatrix[16];    // 4x4 transform matrix
      float color[3];           // RGB color
      uint32_t geometryIndex;   // Index into geometry registry
      uint32_t padding[4];      // 16-byte alignment
  };

  class FullTransformShaderPolicy : public ShaderPolicyBase {
  public:
      static constexpr const char* kShaderName = "FullTransform";
      static constexpr const char* kVertexShaderFile = "PositionRotation3DColorTransform.vert";
      static constexpr const char* kFragmentShaderFile = "SolidColor.frag";

      std::vector<SDL_GPUVertexAttribute> getVertexAttributes() const override;
      SDL_GPUVertexBufferDescription getVertexBufferDescription() const override;
      SDL_GPUVertexInputState getVertexInputState() const override;
      std::vector<uint8_t> buildInstanceData(const msd_sim::Object& object) const override;
      std::string getVertexShaderFile() const override;
      std::string getFragmentShaderFile() const override;
      size_t getInstanceDataSize() const override { return sizeof(FullTransformInstanceData); }
  };
  ```
- **Dependencies**: SDL3 GPU, msd_sim::Object, msd_sim::ReferenceFrame, Eigen::Matrix4f
- **Thread safety**: All methods const and stateless (thread-safe)
- **Error handling**: Returns valid configurations, no exceptions

**Vertex Attribute Configuration:**
```cpp
// Per-vertex attributes (binding 0)
Location 0: Position (vec3, offset 0)
Location 1: Color (vec3, offset 12)
Location 2: Normal (vec3, offset 24)

// Per-instance attributes (binding 1)
Location 3-6: InstanceModelRow0-3 (4x vec4, offset 0, stride 16)
Location 7: InstanceColor (vec3, offset 64)
Location 8: InstanceGeometryIndex (uint, offset 76)
```

**Instance Data Layout:**
```
Offset  Size  Field
0       64    modelMatrix[16]  (4x4 float matrix)
64      12    color[3]         (float r, g, b)
76      4     geometryIndex    (uint32_t)
80      16    padding[4]       (align to 16-byte boundary)
Total: 96 bytes (adjusted from original 84 for better alignment)
```

### Modified Components

#### GPUManager
- **Current location**: `msd/msd-gui/src/SDLGPUManager.hpp`, `msd/msd-gui/src/SDLGPUManager.cpp`
- **Changes required**:

**Option A — Compile-Time Template Approach (Recommended):**
  1. Make GPUManager a template class:
     ```cpp
     template<typename ShaderPolicy>
     class GPUManager {
     private:
         ShaderPolicy shaderPolicy_;
         std::vector<typename ShaderPolicy::InstanceDataType> instances_;
         // ... existing members ...
     };
     ```
  2. Remove hardcoded `InstanceData` struct from GPUManager.hpp
  3. Use `ShaderPolicy::buildInstanceData()` in `updateObjects()`
  4. Load shaders via `ShaderPolicy::getVertexShaderFile()` and `getFragmentShaderFile()`
  5. Configure vertex attributes via `ShaderPolicy::getVertexAttributes()` and related methods

**Option B — Runtime Polymorphic Approach:**
  1. Add member: `std::unique_ptr<ShaderPolicyBase> shaderPolicy_`
  2. Modify constructor: `GPUManager(SDL_Window& window, const std::string& basePath, std::unique_ptr<ShaderPolicyBase> policy)`
  3. Replace hardcoded InstanceData with `std::vector<uint8_t>` instance buffer
  4. Use `shaderPolicy_->buildInstanceData()` to create instance data
  5. Load shaders via `shaderPolicy_->getVertexShaderFile()` and `getFragmentShaderFile()`
  6. Configure vertex attributes via `shaderPolicy_->getVertexAttributes()` and related methods

- **Backward compatibility**: Breaking change — constructor signature changes, InstanceData struct removed

**Recommended approach**: Option A (template) for zero-cost abstraction and compile-time type safety

#### SDLApplication
- **Current location**: `msd/msd-gui/src/SDLApp.hpp`, `msd/msd-gui/src/SDLApp.cpp`
- **Changes required**:

**If Option A (Template):**
  1. Choose shader policy at compile time via template parameter or type alias:
     ```cpp
     using AppGPUManager = GPUManager<PositionOnlyShaderPolicy>;  // Or FullTransformShaderPolicy
     std::unique_ptr<AppGPUManager> gpuManager_;
     ```
  2. No runtime selection needed

**If Option B (Runtime):**
  1. Add constructor parameter: `enum class ShaderType { PositionOnly, FullTransform }`
  2. Create appropriate policy in constructor:
     ```cpp
     SDLApplication::SDLApplication(const std::string& dbPath, ShaderType shaderType) {
         std::unique_ptr<ShaderPolicyBase> policy;
         if (shaderType == ShaderType::PositionOnly) {
             policy = std::make_unique<PositionOnlyShaderPolicy>();
         } else {
             policy = std::make_unique<FullTransformShaderPolicy>();
         }
         gpuManager_ = std::make_unique<GPUManager>(window, basePath, std::move(policy));
     }
     ```

- **Backward compatibility**: Constructor signature changes

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| ShaderPolicyBase | GPUManager | Composition (runtime) or template param (compile-time) | Policy provides shader configuration |
| PositionOnlyShaderPolicy | Object | Read-only access | Extracts position and color only |
| FullTransformShaderPolicy | Object | Read-only access | Extracts full transform, color, geometry |
| GPUManager (modified) | ShaderPolicy | Composition/template | Uses policy for all shader-specific config |
| SDLApplication (modified) | ShaderPolicy | Factory creation (if runtime) | Creates appropriate policy instance |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| None | N/A | No existing unit tests for msd-gui | Create new unit tests |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| PositionOnlyShaderPolicy | Vertex attribute configuration | Correct locations, formats, offsets for position-only shader |
| PositionOnlyShaderPolicy | Instance data building | Converts Object to 32-byte PositionOnlyInstanceData |
| PositionOnlyShaderPolicy | Shader file names | Returns correct shader file paths |
| FullTransformShaderPolicy | Vertex attribute configuration | Correct locations, formats, offsets for full transform shader |
| FullTransformShaderPolicy | Instance data building | Converts Object to 96-byte FullTransformInstanceData with correct matrix |
| FullTransformShaderPolicy | Shader file names | Returns correct shader file paths |
| FullTransformShaderPolicy | Model matrix creation | Builds correct 4x4 matrix from ReferenceFrame (position + EulerAngles) |

**Test file location**: `test/unit/msd-gui/shader_policy_test.cpp`

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| Manual: Launch with position-only shader | SDLApplication, GPUManager, PositionOnlyShaderPolicy | Objects render with position offset, no rotation |
| Manual: Launch with full transform shader | SDLApplication, GPUManager, FullTransformShaderPolicy | Objects render with full rotation and translation |
| Manual: Spawn multiple objects with position-only | All components | Multiple instances render correctly without rotation |
| Manual: Spawn multiple objects with full transform | All components | Multiple instances render correctly with rotation |

## Design Decisions

### Compile-Time vs Runtime Shader Selection

**Question**: Should shader selection happen at compile time (templates) or runtime (polymorphism)?

**Option A: Compile-Time Template Approach**
- **Pros**:
  - Zero runtime overhead (no virtual function calls)
  - Type safety enforced by compiler
  - Instance data types are compile-time concrete (`PositionOnlyInstanceData` vs `FullTransformInstanceData`)
  - Better compiler optimizations (inlining, constant folding)
  - Matches ticket requirement: "Ideally, the shader is chosen at compile time (possibly via template metaprogramming)"
- **Cons**:
  - Must recompile to switch shader types
  - GPUManager becomes a template class (header implementation or explicit instantiation required)
  - Cannot choose shader based on runtime configuration file
  - May increase binary size (template instantiation)

**Option B: Runtime Polymorphic Approach**
- **Pros**:
  - Can switch shader types without recompiling
  - Could load shader type from configuration file
  - Simpler forward declarations (no templates)
  - Single GPUManager implementation
- **Cons**:
  - Virtual function call overhead (minimal, but present)
  - Instance data must be type-erased (`std::vector<uint8_t>`)
  - Runtime type checking if different policies need different handling
  - More complex error handling at runtime

**Recommendation**: **Option A (Compile-Time Template)**

**Rationale**:
1. Ticket explicitly prefers compile-time selection: "Ideally, the shader is chosen at compile time (possibly via template metaprogramming)"
2. Performance non-functional requirement: "Performance should not be a huge factor" suggests we should still avoid unnecessary overhead
3. Current use case has no need for runtime shader switching (debugging position-only vs full transform)
4. Type safety benefit outweighs flexibility loss
5. Follows project C++20 modern practices and zero-cost abstraction principles

**Implementation note**: If future requirements demand runtime selection, we can provide both options (template for performance-critical code, polymorphic for flexible code).

### Instance Data Size and Alignment

**Decision**: Adjust FullTransformInstanceData size from 84 bytes to 96 bytes for better alignment.

**Current layout (84 bytes)**:
```
Offset  Size  Field
0       64    modelMatrix[16]
64      12    color[3]
76      4     geometryIndex
80      4     padding (single uint32_t)
Total: 84 bytes
```

**Proposed layout (96 bytes)**:
```
Offset  Size  Field
0       64    modelMatrix[16]
64      12    color[3]
76      4     geometryIndex
80      16    padding[4]
Total: 96 bytes
```

**Rationale**:
- GPU buffer alignment typically prefers 16-byte boundaries
- 96 bytes = 6 × 16 bytes (perfect alignment)
- 84 bytes = 5.25 × 16 bytes (misaligned)
- 12 extra bytes per instance is negligible (12 KB for 1000 instances)
- Avoids potential GPU driver alignment issues

**Alternative**: Keep 84 bytes if GPU drivers handle it correctly
- Must verify on all target platforms (Vulkan, Metal, D3D12)

**Note**: PositionOnlyInstanceData is already 32 bytes = 2 × 16 (well-aligned)

### Shader Policy Interface Design

**Decision**: Provide both abstract base class (ShaderPolicyBase) and concrete policy classes.

**Rationale**:
- Enables both compile-time (template) and runtime (polymorphic) usage
- Concrete policies (PositionOnlyShaderPolicy, FullTransformShaderPolicy) can be used directly as template parameters
- Abstract base class enables runtime selection if needed in the future
- Minimal overhead: abstract base class only used if Option B is selected

**Interface responsibilities**:
1. `getVertexAttributes()` — SDL GPU vertex attribute configuration
2. `getVertexBufferDescription()` — Vertex buffer binding description
3. `getVertexInputState()` — Complete vertex input state for pipeline
4. `buildInstanceData()` — Convert Object to shader-specific instance data
5. `getVertexShaderFile()` / `getFragmentShaderFile()` — Shader file paths
6. `getInstanceDataSize()` — Size of instance data for buffer allocation

**Why separate methods instead of single "getShaderConfig()" struct?**
- Each method has distinct usage point in GPUManager initialization/update
- Easier to test individual components
- Allows partial overrides in future policy variants

## Open Questions

### Design Decisions (Human Input Needed)

1. **Instance data alignment for FullTransformInstanceData**
   - Option A: Keep 84 bytes (current, 4 bytes padding) — Pros: Saves 12 bytes per instance; Cons: Potential alignment issues on some GPUs
   - Option B: Change to 96 bytes (16 bytes padding) — Pros: Perfect 16-byte alignment, safer for all GPUs; Cons: 12 extra bytes per instance
   - **Recommendation**: Option B (96 bytes) for robustness, unless profiling shows memory concerns
   - **Response**: Let's go with option B

2. **Compile-time vs runtime shader selection**
   - Option A: Template-based (compile-time) — Pros: Zero overhead, type safety; Cons: Requires recompilation to switch
   - Option B: Polymorphic (runtime) — Pros: Runtime flexibility; Cons: Virtual function overhead, type erasure
   - **Recommendation**: Option A (template) per ticket requirement and performance best practices
   - **Response**: Let's go with option A

3. **Where to define shader policies**
   - Option A: Single header `ShaderPolicy.hpp` with all policies — Pros: Easy to find, simple include; Cons: Changes to one policy recompile all users
   - Option B: Separate headers per policy (`PositionOnlyShaderPolicy.hpp`, etc.) — Pros: Better compilation isolation; Cons: More includes
   - **Recommendation**: Option A for simplicity (only 2 policies currently, compilation time not a concern)
   - **Response**: Let's go with option A

### Prototype Required

None. This is a straightforward refactoring using well-understood C++ techniques (policy-based design, template metaprogramming).

### Requirements Clarification

1. **Unit test scope**
   - Acceptance criteria states: "New unit tests for msd-gui show metadata for selecting different shaders"
   - **Question**: Should unit tests cover only shader policy metadata (vertex attributes, instance data layout), or also integration with GPUManager?
   - **Proposed**: Unit tests for shader policies only (metadata validation). Integration testing via manual msd-exe testing (consistent with ticket 0001 approach).
   - **Response**: Let's do tests for shader policies only

2. **Transition strategy**
   - **Question**: Should we provide both PositionOnly and FullTransform variants of msd-exe, or a command-line flag to select?
   - **Proposed**: If using Option A (template), provide a preprocessor define or CMake option to select shader type at build time. Example: `cmake -DMSD_GUI_SHADER=PositionOnly` or `cmake -DMSD_GUI_SHADER=FullTransform`.
   - **Response**: I'm fine with hard-coding the option for now in msd-exe

3. **Default shader type**
   - **Question**: Which shader should be the default for msd-exe?
   - **Context**: Ticket motivation says "Reverting the change that attempted to accommodate both position and translation" suggests defaulting to PositionOnly for easier debugging.
   - **Proposed**: Default to `PositionOnlyShaderPolicy` initially to verify ticket 0001 logic without rotation complexity. Once validated, switch back to `FullTransformShaderPolicy`.
   - **Response**: Let's default to `PositionOnlyShaderPolicy`

## Performance Analysis

### Compile-Time Template Approach (Option A)

**Memory impact**:
- **PositionOnly**: 32 bytes per instance (vs 84 bytes current) = 62% reduction
- **FullTransform**: 96 bytes per instance (vs 84 bytes current) = 14% increase
- For 1000 instances:
  - PositionOnly: 32 KB instance buffer
  - FullTransform: 96 KB instance buffer

**CPU overhead**:
- Zero runtime overhead vs current implementation
- `buildInstanceData()` inlined by compiler
- No virtual function call overhead

**GPU upload bandwidth**:
- PositionOnly: 32 KB per frame (for 1000 instances) vs 84 KB current = 62% reduction
- FullTransform: 96 KB per frame vs 84 KB current = 14% increase
- Both well within acceptable range (modern GPUs handle MB/s easily)

### Runtime Polymorphic Approach (Option B)

**Additional overhead vs Option A**:
- Virtual function call per method: ~1-5 CPU cycles per call (negligible)
- Type-erased instance buffer: `std::vector<uint8_t>` instead of typed vector (no performance difference)
- Potential for less compiler optimization (inlining prevented by virtual dispatch)

**Recommendation**: Negligible performance difference for this use case, but Option A aligns better with zero-cost abstraction principle.

## Implementation Notes

### Build Integration

**If Option A (Template) selected:**
- Add `msd/msd-gui/src/ShaderPolicy.hpp` (header-only policies)
- Optionally add explicit template instantiations in `msd/msd-gui/src/ShaderPolicy.cpp`:
  ```cpp
  template class GPUManager<PositionOnlyShaderPolicy>;
  template class GPUManager<FullTransformShaderPolicy>;
  ```
- Update `msd/msd-exe/CMakeLists.txt` to add shader type selection:
  ```cmake
  option(MSD_GUI_SHADER_TYPE "Shader type: PositionOnly or FullTransform" "PositionOnly")
  target_compile_definitions(msd_exe PRIVATE MSD_SHADER_TYPE_${MSD_GUI_SHADER_TYPE})
  ```

**If Option B (Runtime) selected:**
- Add `msd/msd-gui/src/ShaderPolicy.hpp` (base class + concrete policies)
- Add `msd/msd-gui/src/ShaderPolicy.cpp` (implementations)
- Update `msd/msd-gui/CMakeLists.txt` to include new source file

### Shader Compilation

Both shaders already exist and compiled:
- `Position3DColorTransform.vert.hlsl` → SPIRV, MSL, DXIL
- `PositionRotation3DColorTransform.vert.hlsl` → SPIRV, MSL, DXIL

No shader changes required (this is purely a C++ abstraction refactoring).

### Testing Strategy

**Unit tests** (`test/unit/msd-gui/shader_policy_test.cpp`):
```cpp
TEST_CASE("PositionOnlyShaderPolicy: vertex attribute configuration") {
    PositionOnlyShaderPolicy policy;
    auto attrs = policy.getVertexAttributes();

    // Verify per-vertex attributes
    REQUIRE(attrs.size() == 5);
    REQUIRE(attrs[0].location == 0);  // Position
    REQUIRE(attrs[0].format == SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3);
    // ... similar checks for Color, Normal, InstancePosition, InstanceColor
}

TEST_CASE("PositionOnlyShaderPolicy: instance data building") {
    PositionOnlyShaderPolicy policy;
    msd_sim::Object obj = createTestObject();  // Helper function

    auto instanceData = policy.buildInstanceData(obj);

    REQUIRE(instanceData.size() == 32);  // 32 bytes expected
    // Verify position and color values match object
}

TEST_CASE("FullTransformShaderPolicy: model matrix creation") {
    FullTransformShaderPolicy policy;
    msd_sim::ReferenceFrame frame;
    frame.setOrigin({1.0, 2.0, 3.0});
    frame.getEulerAngles().yaw = M_PI / 4;  // 45 degrees

    msd_sim::Object obj = createTestObjectWithTransform(frame);
    auto instanceData = policy.buildInstanceData(obj);

    REQUIRE(instanceData.size() == 96);
    // Verify matrix values (extract first 64 bytes, interpret as float[16])
    // Verify rotation component matches yaw
}
```

**Manual integration testing**:
1. Build with `PositionOnlyShaderPolicy`
2. Launch msd-exe, press 'Z' to spawn pyramids
3. Verify pyramids render at random positions WITHOUT rotation
4. Rebuild with `FullTransformShaderPolicy`
5. Launch msd-exe, press 'Z' to spawn pyramids
6. Verify pyramids render WITH random rotations

## Future Enhancements (Out of Scope)

1. **Additional shader policies**: Textured shader, normal-mapped shader, skeletal animation shader
2. **Shader hot-reloading**: Detect shader file changes and reload pipeline at runtime
3. **Multi-shader rendering**: Support multiple shader types simultaneously (different pipelines per geometry type)
4. **Shader parameter customization**: Allow runtime tweaking of shader constants (e.g., lighting direction)
5. **Automatic shader compilation**: Integrate shader compilation into CMake build process
