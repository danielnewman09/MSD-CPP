# Ticket 0002 Context for Designer Agent

## Ticket Content

**Title**: Modularize SDLGPUManager Shader Implementation

**Status**: Ready for Design

**Summary**: Based on the current implementation from ticket 0001, the SDLGPUManager now manages a shader that contains position and rotation information. This has caused issues wherein the graphical objects are not rendering properly. Modify the msd-gui library to allow use of multiple shaders. In this case, specifically allow for both `PositionRotation3DColorTransform` and `Position3DColorTransform`.

**Motivation**: Reverting the change that attempted to accommodate both position and translation will hopefully allow for easier debugging and ensuring the rest of the logic implemented in ticket 0001 is correct. It will also improve modularity of the application layer and further modification down the line.

## Functional Requirements

1. The SDLGPUManager shall have a configurable `Vertex` type based on the type of shader in use
2. Based on the `Vertex` type, pre-configured `SDL_GPUVertexAttribute`, `SDL_GPUVertexBufferDescription`, `SDL_GPUVertexInputState`, and `vertexShader` shall be available

## Non-Functional Requirements

- **Performance**: This is primarily a variable bookkeeping ticket. Performance should not be a huge factor here.
- **Memory**: Ideally, the shader is chosen at compile time (possibly via template metaprogramming). Regardless, the SDLGPUManager should contain a member variable that only selects the shaders and related attributes required.
- **Thread Safety**: The GPU operations are assumed to take place on the main thread and are therefore not thread safe

## Constraints

- None specified

## Acceptance Criteria

- New unit tests for msd-gui show metadata for selecting different shaders

## Current State

### Existing Shaders

Two vertex shaders already exist:

1. **Position3DColorTransform.vert.hlsl** (simpler, position-only):
   - TEXCOORD0: Position (per-vertex)
   - TEXCOORD1: Color (per-vertex)
   - TEXCOORD2: Normal (per-vertex)
   - TEXCOORD3: InstancePosition (per-instance, vec3)
   - TEXCOORD4: InstanceColor (per-instance, vec3)
   - Uniform: modelViewProjection (4x4 matrix)
   - Transformation: `MVP * (position + instancePosition)`

2. **PositionRotation3DColorTransform.vert.hlsl** (full transform):
   - TEXCOORD0: Position (per-vertex)
   - TEXCOORD1: Color (per-vertex)
   - TEXCOORD2: Normal (per-vertex)
   - TEXCOORD3-6: InstanceModelRow0-3 (per-instance, 4x vec4 = 4x4 matrix)
   - TEXCOORD7: InstanceColor (per-instance, vec3)
   - TEXCOORD8: InstanceGeometryIndex (per-instance, uint)
   - Uniform: viewProjection (4x4 matrix)
   - Transformation: `viewProjection * instanceModel * position`

### Current InstanceData Structure

From SDLGPUManager.hpp:
```cpp
struct InstanceData {
    float modelMatrix[16];      // 4x4 transform matrix
    float color[3];             // RGB color
    uint32_t geometryIndex;     // Index into geometryRegistry_
    uint32_t padding[4];        // Alignment padding
};
```

This matches the PositionRotation3DColorTransform shader requirements but NOT the Position3DColorTransform shader.

### Current Issue

The implementation currently uses PositionRotation3DColorTransform with full model matrices, but the ticket states "this has caused issues wherein the graphical objects are not rendering properly." The request is to make the shader system modular so both shaders can be supported.

## Design Guidance

### Preferred Approaches

- Template metaprogramming to select shader at compile time (if feasible)
- Single member variable that stores only required shader metadata
- Follow coding best practices from CLAUDE.md

### Things to Avoid

- Runtime overhead for shader selection if avoidable
- Breaking existing Object-based API from ticket 0001
- Duplicating shader metadata configuration

## Related Code

- `msd/msd-gui/src/SDLGPUManager.hpp` - GPU manager header
- `msd/msd-gui/src/SDLGPUManager.cpp` - GPU manager implementation
- `msd/msd-gui/src/SDLApp.cpp` - Application using GPU manager
- `msd/msd-gui/src/Content/Shaders/Source/Position3DColorTransform.vert.hlsl` - Simple shader
- `msd/msd-gui/src/Content/Shaders/Source/PositionRotation3DColorTransform.vert.hlsl` - Full transform shader

## Related Documentation

- `docs/msd/msd-gui/gpu-manager.puml` - Current GPU manager architecture
- `docs/msd/msd-gui/sdl-application.puml` - Application architecture
- `docs/designs/generalize-gui-object-rendering/design.md` - Ticket 0001 design
