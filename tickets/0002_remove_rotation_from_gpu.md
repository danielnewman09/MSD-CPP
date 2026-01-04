# Feature Ticket: Modularize SDLGPUManager Shader Implementation

## Status
- [ ] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [x] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 
- **Author**: Daniel M Newman
- **Priority**: High
- **Estimated Complexity**: Medium
- **Target Component(s)**: msd-gui

---

## Summary
Based on the current implementation from ticket 0001, the SDLGPUManager now manages a shader that contains position and rotation information. This has caused isses wherein the graphical objects are not rendering properly. Modify the msd-gui library to allow use of multiple shaders. In this case, specifically allow for both `PositionRotation3DColorTransform` and `Position3DColorTransform`. 

## Motivation
Reverting the change that attempted to accommodate both position and translation will hopefully allow for easier debugging and ensuring the rest of the logic implemented in ticket 0001 is correct. It will also improve modularity of the application layer and further modification down the line.

## Requirements

### Functional Requirements
1. The SDLGPUManager shall have a configurable `Vertex` type based on the type of shader in use
2. Based on the `Vertex` type, a pre-configured `SDL_GPUVertexAttribute`, `SDL_GPUVertexBufferDescription`,`SDL_GPUVertexInputState`, and`vertexShader`

### Non-Functional Requirements
- **Performance**: This is primarily a variable bookkeeping ticket. Performance should not be a huge factor here.
- **Memory**: Ideally, the shader is chosen at compile time (possibly via template metaprogramming). Regardless, the SDLGPUManager should contain a member variable that only selects the shaders and related attributes required. 
- **Thread Safety**: The GPU operations are assumed to take place on the main thread and are therefore not thread safe
- **Backward Compatibility**: Not a big concern here

## Constraints
- none

## Acceptance Criteria
- [ ] New unit tests for msd-gui show metadata for selecting different shaders

---

## Design Decisions (Human Input)
- See the functional and non-functional requirements

### Preferred Approaches
- See the requirments

### Things to Avoid
- See the CLAUDE.md files for coding best practices

### Open Questions
- None

---

## References

### Related Code
- `msd/msd-sim/src/Environment/Object.hpp` - The current implementation of the general object instance containing all relevant information for rendering and simulation
- `msd/msd-gui/src/SDLGPUManager.hpp` - The object which manages the GPU buffer data
- `msd/msd-gui/src/SDLApp.cpp` - The place where the GPUManager is modified from user inputs.

### Related Documentation
- `docs/msd/msd-gui/gpu-manager.puml`
- `docs/msd/msd-gui/sdl-application.puml`
- `docs/msd/msd-sim/Environment/object.puml`

### Related Tickets
- N/A

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-01-03
- **Completed**: 2026-01-03
- **Artifacts**:
  - `docs/designs/modularize-gpu-shader-system/design.md`
  - `docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml`
- **Notes**:
  - Designed shader policy system to support multiple shader types (PositionOnly vs FullTransform)
  - Key design decisions:
    - Recommended compile-time template approach (Option A) per ticket requirement for compile-time selection
    - Created ShaderPolicyBase interface and two concrete policies: PositionOnlyShaderPolicy and FullTransformShaderPolicy
    - PositionOnlyInstanceData: 32 bytes (position + color only, no rotation)
    - FullTransformInstanceData: 96 bytes (4x4 model matrix + color + geometry index, with improved 16-byte alignment)
    - Both shaders already exist: Position3DColorTransform.vert and PositionRotation3DColorTransform.vert
  - Open questions for human review:
    - Q1: Instance data alignment - 84 bytes (current) vs 96 bytes (recommended for better alignment)
    - Q2: Compile-time (template) vs runtime (polymorphic) selection - Recommended compile-time per ticket
    - Q3: Shader policy organization - Single header vs separate headers per policy
    - Q4: Default shader type for debugging - Recommended PositionOnly initially
  - Performance analysis: PositionOnly reduces instance buffer by 62% (32 KB vs 84 KB for 1000 instances)
  - Memory analysis: Both approaches well within GPU memory constraints
  - No prototype needed - straightforward refactoring using C++20 policy-based design

### Design Review Phase
- **Started**: 2026-01-03
- **Completed**: 2026-01-03
- **Status**: APPROVED
- **Reviewer Notes**:
  - All design decisions approved by human:
    - Instance data alignment: 96 bytes (Option B)
    - Shader selection: Compile-time template approach (Option A)
    - Code organization: Single header ShaderPolicy.hpp (Option A)
    - Unit test scope: Tests for shader policies only
    - Transition strategy: Hard-code the shader option in msd-exe for now
    - Default shader: PositionOnlyShaderPolicy
  - Prototype phase skipped per design document recommendation (straightforward refactoring)

### Prototype Phase
- **Started**: N/A
- **Completed**: N/A
- **Prototypes**: Skipped - straightforward refactoring using well-understood C++ policy-based design
- **Artifacts**: N/A
- **Notes**: Design document explicitly states no prototype required

### Implementation Phase
- **Started**: 2026-01-03
- **Completed**: 2026-01-03
- **Files Created**:
  - `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-gui/src/ShaderPolicy.hpp` (285 lines)
  - `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-gui/src/ShaderPolicy.cpp` (322 lines)
  - `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-gui/test/unit/shader_policy_test.cpp` (187 lines)
- **Files Modified**:
  - `msd/msd-gui/src/SDLGPUManager.hpp` — Template-ized with ShaderPolicy parameter
  - `msd/msd-gui/src/SDLGPUManager.cpp` — Moved to backup (template implementation in header)
  - `msd/msd-gui/src/SDLApp.hpp` — Added AppGPUManager type alias
  - `msd/msd-gui/src/SDLApp.cpp` — Updated to use AppGPUManager
  - `msd/msd-gui/src/CMakeLists.txt` — Added ShaderPolicy.cpp, removed SDLGPUManager.cpp
  - `msd/msd-gui/test/CMakeLists.txt` — Added shader_policy_test.cpp
- **Artifacts**:
  - `docs/designs/modularize-gpu-shader-system/implementation-notes.md`
- **Notes**:
  - Successfully implemented compile-time template-based shader policy system
  - Created PositionOnlyShaderPolicy (32-byte instance data) and FullTransformShaderPolicy (96-byte instance data)
  - All 9 unit tests passing (15 total including existing tests)
  - Build successful with no errors
  - Default shader policy: PositionOnlyShaderPolicy per design decision
  - Ready for manual integration testing

### Implementation Review Phase
- **Started**: 2026-01-03
- **Completed**: 2026-01-03
- **Status**: APPROVED
- **Reviewer Notes**:
  - Human confirmed implementation approved
  - All tests passing (15/15 including 9 new shader policy tests)
  - Ready for documentation update and merge

### Documentation Update Phase
- **Started**: 2026-01-03
- **Completed**: 2026-01-03
- **CLAUDE.md Updates**:
  - Updated `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-gui/CLAUDE.md`:
    - Added ShaderPolicy to Core Components table
    - Added comprehensive ShaderPolicy component detail section
    - Added Policy-Based Design pattern to Design Patterns section
    - Added Recent Architectural Changes entry for shader policy system
    - Updated Diagrams Index with shader policy diagram
- **Diagrams Indexed**:
  - `docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml` — Shader policy system architecture
- **Notes**:
  - Documentation reflects compile-time template approach
  - Includes usage examples for switching shader policies
  - Performance characteristics documented for both policies
  - Design pattern section highlights zero-cost abstraction benefits 

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}
