# Implementation Notes: Modularize GPU Shader System

**Ticket**: 0002_remove_rotation_from_gpu
**Design Document**: [design.md](./design.md)
**Implementation Date**: 2026-01-03
**Implemented By**: Claude Opus 4.5 (AI Assistant)

---

## Summary

Successfully implemented compile-time template-based shader policy system for SDLGPUManager, enabling support for multiple shader types (PositionOnly vs FullTransform) without runtime overhead. The implementation follows Option A from the design document (compile-time template approach) and includes comprehensive unit tests for shader policy validation.

---

## Files Created

| File | Purpose | Lines of Code |
|------|---------|---------------|
| `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-gui/src/ShaderPolicy.hpp` | Shader policy interface and concrete implementations | 285 |
| `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-gui/src/ShaderPolicy.cpp` | Shader policy implementation methods | 322 |
| `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-gui/test/unit/shader_policy_test.cpp` | Unit tests for shader policies | 187 |

Total new code: **794 lines**

---

## Files Modified

| File | Description of Changes |
|------|------------------------|
| `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-gui/src/SDLGPUManager.hpp` | Template-ized GPUManager class; moved implementation to header (template requirement); removed hardcoded InstanceData; added ShaderPolicy template parameter |
| `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-gui/src/SDLGPUManager.cpp` | Moved to backup (`.backup`); implementation now in header due to template |
| `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-gui/src/SDLApp.hpp` | Added type alias `AppGPUManager = GPUManager<PositionOnlyShaderPolicy>`; updated includes |
| `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-gui/src/SDLApp.cpp` | Updated to use `AppGPUManager` instead of `GPUManager`; updated ticket references |
| `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-gui/src/CMakeLists.txt` | Removed `SDLGPUManager.cpp`; added `ShaderPolicy.cpp` and `ShaderPolicy.hpp` |
| `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-gui/test/CMakeLists.txt` | Added `unit/shader_policy_test.cpp`; linked `msd_gui` and `msd_assets` libraries |

---

## Design Adherence Matrix

| Design Decision | Implemented | Notes |
|-----------------|-------------|-------|
| Compile-time template approach (Option A) | ✅ Yes | `GPUManager` is now template class with `ShaderPolicy` parameter |
| PositionOnlyShaderPolicy | ✅ Yes | Implements position-offset rendering (32-byte instance data) |
| FullTransformShaderPolicy | ✅ Yes | Implements full transform rendering (96-byte instance data) |
| Instance data alignment (96 bytes for FullTransform) | ✅ Yes | `FullTransformInstanceData` is 96 bytes with 16-byte padding |
| Instance data alignment (32 bytes for PositionOnly) | ✅ Yes | `PositionOnlyInstanceData` is 32 bytes with 8-byte padding |
| Single header ShaderPolicy.hpp (Option A) | ✅ Yes | Both policies in single header with separate `.cpp` |
| Unit tests for shader policies only | ✅ Yes | Tests validate vertex attributes, buffer descriptions, sizes, alignment |
| Hard-code shader policy in msd-exe | ✅ Yes | `SDLApp` uses `PositionOnlyShaderPolicy` as default |
| Default shader: PositionOnlyShaderPolicy | ✅ Yes | Set via type alias in `SDLApp.hpp` |

---

## Implementation Details

### Template-ized GPUManager

**Key Changes**:
- Made `GPUManager` a class template: `template<typename ShaderPolicy> class GPUManager`
- Moved all implementation to header file (required for templates)
- Added `using InstanceDataType = typename ShaderPolicy::InstanceDataType` to extract instance data type from policy
- Replaced hardcoded `InstanceData` with policy-provided `InstanceDataType`
- Used `shaderPolicy_.getVertexAttributes()`, `getVertexInputState()`, etc. for pipeline configuration
- Added compile-time branching using `if constexpr (std::is_same_v<ShaderPolicy, ...>)` for rendering logic

**Rendering Logic**:
- PositionOnly: Single draw call for all instances (no geometry sorting needed)
- FullTransform: Grouped by geometry index with multiple draw calls per geometry type

### Shader Policies

**PositionOnlyShaderPolicy**:
- **Vertex shader**: `Position3DColorTransform.vert`
- **Fragment shader**: `SolidColor.frag`
- **Instance data**: 32 bytes (position[3], color[3], padding[2])
- **Vertex attributes**: 5 total (3 per-vertex + 2 per-instance)
- **Use case**: Simple position-offset rendering without rotation

**FullTransformShaderPolicy**:
- **Vertex shader**: `PositionRotation3DColorTransform.vert`
- **Fragment shader**: `SolidColor.frag`
- **Instance data**: 96 bytes (modelMatrix[16], color[3], geometryIndex, padding[4])
- **Vertex attributes**: 9 total (3 per-vertex + 6 per-instance)
- **Use case**: Full 4x4 model matrix rendering with rotation support

### Build Instance Data

Both policies implement `buildInstanceData()`, but with different signatures:
- **PositionOnly**: `buildInstanceData(const Object&) const` — no geometry index needed
- **FullTransform**: `buildInstanceData(const Object&, const geometryNameToIndex&) const` — requires geometry mapping

GPUManager uses `if constexpr` to handle both cases at compile-time.

---

## Deviations from Design

### Minor Deviations

1. **Caching in shader policies**
   - **Design**: Didn't specify caching strategy
   - **Implementation**: Added mutable cached vectors for `getVertexInputState()` to avoid repeated allocations
   - **Rationale**: Performance optimization; SDL expects pointers that remain valid during pipeline creation

2. **buildInstanceData signature**
   - **Design**: Proposed same signature for both policies
   - **Implementation**: Different signatures (PositionOnly doesn't need geometry mapping)
   - **Rationale**: PositionOnly has no geometry index concept, so geometry mapping is unnecessary
   - **Impact**: GPUManager uses `if constexpr` to handle both cases transparently

### No Deviations from Core Design

All major design decisions were adhered to:
- Compile-time template approach
- Zero runtime overhead
- 16-byte aligned instance data
- Default PositionOnlyShaderPolicy in msd-exe

---

## Test Coverage Summary

### Unit Tests

| Test Suite | Test Count | Status |
|------------|-----------|--------|
| PositionOnlyShaderPolicy | 4 | ✅ PASSING |
| FullTransformShaderPolicy | 3 | ✅ PASSING |
| PositionOnlyInstanceData | 1 | ✅ PASSING |
| FullTransformInstanceData | 1 | ✅ PASSING |
| **Total** | **9** | **✅ ALL PASSING** |

### Test Results

```
[==========] Running 15 tests from 5 test suites.
[  PASSED  ] 15 tests (includes existing InstanceDataLayoutTest)
```

### What Was Tested

**PositionOnlyShaderPolicy**:
- Vertex attribute configuration (locations, formats, offsets)
- Vertex buffer descriptions (slots, pitches, input rates)
- Vertex input state (buffer and attribute counts)
- Shader file names

**FullTransformShaderPolicy**:
- Vertex attribute configuration (including 4x4 matrix layout)
- Vertex buffer descriptions
- Shader file names

**Instance Data Structures**:
- Size validation (32 bytes for PositionOnly, 96 bytes for FullTransform)
- 16-byte alignment verification

### Integration Testing

**Status**: Deferred to manual testing
**Rationale**: Per design decision, integration testing via manual msd-exe testing (consistent with ticket 0001 approach)

**Recommended Manual Tests**:
1. Build with PositionOnlyShaderPolicy and verify objects render without rotation
2. Change to FullTransformShaderPolicy and rebuild, verify objects render with rotation
3. Spawn multiple objects and verify correct rendering

---

## Known Limitations

1. **No runtime shader switching**
   - Must recompile to switch between shader policies
   - Mitigated by: This was the intended design (compile-time selection)

2. **PositionOnly doesn't support geometry indexing**
   - All instances rendered from same base geometry (index 0)
   - Mitigated by: Intentional simplification for debugging

3. **Tests don't validate actual instance data building**
   - Test framework complexity prevented creating test Objects with assets
   - Mitigated by: Structural tests validate data layout; runtime testing will validate correctness

---

## Build Verification

**Build Command**: `cmake --build --preset debug-gui-only`
**Result**: ✅ Success (with warnings from unrelated files)

**Test Command**: `/Users/danielnewman/Documents/GitHub/MSD-CPP/build/Debug/debug/msd_gui_test`
**Result**: ✅ 15/15 tests passing

---

## Future Considerations

### Immediate Next Steps

1. **Manual integration testing**
   - Build and run msd-exe with PositionOnlyShaderPolicy
   - Verify rendering without rotation
   - Test with multiple spawned objects

2. **Switching to FullTransformShaderPolicy**
   - Update `SDLApp.hpp` to use `GPUManager<FullTransformShaderPolicy>`
   - Rebuild and test rotation rendering

### Potential Enhancements (Out of Scope)

1. **Additional shader policies**
   - TexturedShaderPolicy for textured rendering
   - NormalMappedShaderPolicy for advanced lighting
   - SkeletalAnimationShaderPolicy for animated characters

2. **Shader hot-reloading**
   - Detect shader file changes
   - Rebuild pipeline at runtime

3. **Multi-shader rendering**
   - Support multiple pipelines simultaneously
   - Different shader policies for different object types

4. **CMake shader selection**
   - Add CMake option to select shader policy at build time
   - Example: `cmake -DMSD_SHADER_POLICY=FullTransform`

---

## Lessons Learned

### What Went Well

1. **Template approach worked as designed**
   - Zero runtime overhead
   - Type safety enforced by compiler
   - Clean abstraction via policy pattern

2. **Compile-time branching with `if constexpr`**
   - Elegantly handled different policy behaviors
   - No need for SFINAE or tag dispatching

3. **16-byte alignment**
   - Achieved via padding fields in instance data structures
   - GPU-friendly layout

### Challenges

1. **Template implementation in header**
   - Required moving all GPUManager implementation to header
   - Increased compilation dependencies

2. **Test complexity**
   - Object/Asset creation too complex for simple unit tests
   - Simplified to structural validation instead

3. **Different buildInstanceData signatures**
   - Needed compile-time branching to handle
   - Could have used SFINAE for more elegance

### Recommendations

1. **Consider explicit template instantiation**
   - Could reduce header bloat by instantiating known policies in `.cpp`
   - Trade-off: Less flexible for adding new policies

2. **Add factory helpers for testing**
   - Create minimal Object instances for testing
   - Reduce test setup complexity

3. **Document switching process**
   - Provide clear instructions for changing shader policies
   - Consider CMake option in future

---

## Acknowledgments

Implementation guided by:
- Design document: `docs/designs/modularize-gpu-shader-system/design.md`
- Design diagram: `docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml`
- Project coding standards: `CLAUDE.md`
- Existing shaders: `Position3DColorTransform.vert.hlsl`, `PositionRotation3DColorTransform.vert.hlsl`

---

**Implementation Status**: ✅ **COMPLETE**

**Ready for**: Implementation Review
