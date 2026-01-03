# Feature Ticket: Generalize msd-gui object creation

## Status
- [ ] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [X] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 
- **Author**: Daniel M Newman
- **Priority**: High
- **Estimated Complexity**: Medium
- **Target Component(s)**: msd-gui, msd-assets, shaders

---

## Summary
Currently, the SDLGPUManager assumes it stores an arbitrary number of pyramids as generated from the msd-assets library. This feature will allow the GPUManager to generalize in such a way that an arbitrary number of arbitrary geometries (pyramids, cubes, spheres, assets loaded from STL files, etc.) can be generated and placed in the GUI window. This generalization will apply to arbitrary placement, both in translation and rotation. It will also accommodate applying a uniform color to the object. For this specific ticket, it will suffice to use just the existing cube and pyramid geometries as proof of generalization, so long as the containers do not specify the use of those specific geometries specifically.


## Motivation
This feature will provide the basis for rendering general-purpose simulation geometries in support of follow-on work wherein arbitrary objects will interact with one anotther. 

## Requirements

### Functional Requirements
1. The SDLGPUManager shall store 3D vertex information in a single buffer
2. The 3D GUI buffer shall accommodate an arbitrary number of geometry types
3. The SDLGPUManager shall use the `Object` class (defined in msd-Sim/Environment) to derive position, orientation, and color of each object in the buffer
4. The SDLGPUManager shall mock its own vector of objects.
5. The msd-sim library shall not be modified by this ticket. 
6. Demonstrating this functionality shall be done by adding a handler for the "V" key in the SDLApp, where pressing "V" places a cube (currently, "Z" places a pyramid)
7. In addition to randomizing the position and color, the SDLApp shall randomize the orientation (Pitch, roll, and yaw) of the placed object.

### Non-Functional Requirements
- **Performance**: The use of a "single buffer" has performance implications and is to be followed. Document these implications as part of the design
- **Memory**: The use of a "single buffer" has implicit memory limitations. These limitations should fall within our current use-case. Document these limitations as part of the design
- **Thread Safety**: The GPU operations are assumed to take place on the main thread and are therefore not thread safe
- **Backward Compatibility**: Apart from ensuring compatibility with the current object definitions in the msd-assets and msd-sim libraries, do not consider backwards compatibility with the current msd-gui or msd-exe libraries

## Constraints
- The use of a single buffer for vertex data

## Acceptance Criteria
- [ ] human-in-the-loop verification of the msd-exe executable confirms the functional requirements have been met

---

## Design Decisions (Human Input)
- See the functional and non-functional requirements

### Preferred Approaches
- See the requirments

### Things to Avoid
- See the CLAUDE.md files for coding best practices

### Open Questions
- Given the `Object` definition, how can arbitrary geometries be indexed properly in a single buffer?
- How can, for example, multiple objects with differrent geometry but the same number of vertices be stored in this buffer?

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
- **Started**: 2026-01-02 09:05
- **Completed**: 2026-01-02 09:10
- **Artifacts**:
  - `docs/designs/generalize-gui-object-rendering/design.md`
  - `docs/designs/generalize-gui-object-rendering/generalize-gui-object-rendering.puml`
- **Notes**:
  - Addressed both open questions from ticket:
    - Q1: "How can arbitrary geometries be indexed properly in a single buffer?"
      - A: Introduced GeometryInfo registry tracking {baseVertex, vertexCount} per geometry type
    - Q2: "How can multiple objects with different geometry but same vertex count be stored?"
      - A: Each instance stores geometryIndex to identify which GeometryInfo to use for rendering
  - Key design decisions:
    - Enhanced InstanceData to include full 4x4 model matrix (supports rotation requirement #7)
    - Single unified vertex buffer with geometry registry (meets requirement #1)
    - GPUManager reads Object transform/color (meets requirement #3)
    - SDLApplication owns mock Objects (meets requirement #4)
    - Added 'V' key handler for cubes (meets requirement #6)
  - Performance analysis documented: Single buffer reduces API calls, improves cache coherency
  - Memory analysis documented: 84 KB instance buffer for 1000 instances, well within limits
  - Shader modifications required: Vertex shader needs model matrix support
  - No msd-sim modifications required (meets requirement #5) 

### Design Review Phase
- **Started**: 2026-01-02 09:10
- **Completed**: 2026-01-02 09:34
- **Status**: APPROVED
- **Reviewer Notes**: Design approved by human. All open questions resolved:
  - Q1 (geometry indexing): Resolved via GeometryInfo registry approach
  - Q2 (different geometries, same vertex count): Resolved via geometryIndex per instance
  - Design meets all functional and non-functional requirements
  - Performance and memory implications documented and acceptable
  - No prototype needed - proceeding directly to implementation 

### Prototype Phase
- **Started**: 
- **Completed**: 
- **Prototypes**: 
  - P1: {name} — {result}
- **Artifacts**: 
  - `docs/designs/{feature-name}/prototype-results.md`
- **Notes**: 

### Implementation Phase
- **Started**: 
- **Completed**: 
- **Files Created**: 
- **Files Modified**: 
- **Artifacts**: 
  - `docs/designs/{feature-name}/implementation-notes.md`
- **Notes**: 

### Implementation Review Phase
- **Started**: 
- **Completed**: 
- **Status**: 
- **Reviewer Notes**: 

### Documentation Update Phase
- **Started**: 
- **Completed**: 
- **CLAUDE.md Updates**: 
- **Diagrams Indexed**: 
- **Notes**: 

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}
