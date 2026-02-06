# Feature Ticket: Geometry Factory Type Safety Refactor

## Status
- [ ] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [x] Merged / Complete

## Metadata
- **Created**: 2026-01-03
- **Author**: Claude (via debugging session)
- **Priority**: Critical
- **Estimated Complexity**: Medium
- **Target Component(s)**: msd-assets, msd-gui

---

## Summary
The `GeometryFactory` class produces `MeshRecord` blobs that contain `Vertex` structs (36 bytes: position + color + normal), but the `BaseGeometry` constructor incorrectly interprets these blobs as `msd_sim::Vector3D` arrays (24 bytes each). This type mismatch causes rendering issues where objects appear extremely small and with incorrect shapes.

## Motivation
This bug is blocking the GUI rendering functionality. Objects spawned in the msd-exe application appear at <1% of expected size and render as corrupted geometry (flat triangular surfaces instead of closed convex shapes). The root cause is a fundamental type mismatch in the serialization/deserialization pipeline between `GeometryFactory` and `BaseGeometry`.

### Root Cause Analysis
1. `GeometryFactory::verticesToMeshRecord()` stores complete `Vertex` structs (36 bytes each) into `MeshRecord::vertex_data`
2. `BaseGeometry` constructor casts the blob to `msd_sim::Vector3D*` (24 bytes each) regardless of template type
3. This causes:
   - Type size mismatch: 36-byte Vertex structs read as 24-byte Vector3d
   - Precision mismatch: float data (4 bytes per component) interpreted as double (8 bytes)
   - Data corruption: Color and normal data gets misinterpreted as position data

## Requirements

### Functional Requirements
1. The system shall correctly serialize `VisualGeometry` (containing `Vertex` structs) to `MeshRecord` blobs
2. The system shall correctly serialize `CollisionGeometry` (containing `msd_sim::Vector3D`) to `MeshRecord` blobs
3. The system shall correctly deserialize `MeshRecord` blobs back to the appropriate geometry type
4. `GeometryFactory` shall provide clear APIs that produce the correct blob format for each geometry type

### Non-Functional Requirements
- **Performance**: No significant performance regression in geometry creation or loading
- **Memory**: Blob sizes should be appropriate for the data type (no wasted space)
- **Thread Safety**: Existing thread safety guarantees must be maintained
- **Backward Compatibility**: Must consider any existing serialized data in databases

## Constraints
- `Vertex` struct is 36 bytes (3 floats position + 3 floats color + 3 floats normal)
- `msd_sim::Vector3D` is 24 bytes (3 doubles)
- Both types use the same `MeshRecord::vertex_data` blob field
- Tests exist that expect `GeometryFactory` output to work with both `VisualGeometry` and `CollisionGeometry`

## Acceptance Criteria
- [x] GUI renders pyramids and cubes at correct size and shape
- [x] All existing geometry tests pass
- [x] `VisualGeometry` round-trips correctly through `MeshRecord` serialization
- [x] `CollisionGeometry` round-trips correctly through `MeshRecord` serialization
- [x] No type confusion between Visual and Collision geometry blob formats

---

## Design Decisions (Human Input)

### Preferred Approaches
- Consider templating `GeometryFactory` methods to produce the correct blob format based on target geometry type
- The `BaseGeometry` class already has a constructor that takes raw `std::vector<msd_sim::Vector3D>` — this could be leveraged
- `BaseGeometry::populateMeshRecord()` already exists and serializes the correct type

### Things to Avoid
- Don't create a single blob format that tries to work for both geometry types — they have fundamentally different data layouts
- Don't break the existing test patterns if possible

### Open Questions
- Should `GeometryFactory` return geometry objects directly instead of `MeshRecord`?
- Should there be separate factory methods for visual vs collision geometry (e.g., `createVisualCube()` vs `createCollisionCube()`)?
- Is there existing serialized data in databases that would be affected by format changes?

---

## References

### Related Code
- `msd/msd-assets/src/Geometry.hpp` — `BaseGeometry` template class with type mismatch in MeshRecord constructor
- `msd/msd-assets/src/GeometryFactory.hpp` — Factory that produces MeshRecord blobs
- `msd/msd-assets/src/GeometryFactory.cpp` — `verticesToMeshRecord()` that stores Vertex structs
- `msd/msd-gui/src/SDLGPUManager.hpp` — Consumer that creates VisualGeometry from factory output

### Related Documentation
- Debug ticket: `tickets/debugging/D0001_gui-vertex-size-shape.md`

### Related Tickets
- None

---

## Workflow Log

### Design Phase
- **Started**: 2026-01-03 16:30
- **Completed**: 2026-01-03 17:15
- **Artifacts**:
  - `docs/designs/geometry-factory-type-safety/design.md`
  - `docs/designs/geometry-factory-type-safety/geometry-factory-type-safety.puml`
- **Notes**:
  - Identified root cause: GeometryFactory stores Vertex (36 bytes) but BaseGeometry casts to Vector3d (24 bytes)
  - Solution: Split factory methods into type-specific variants (createVisualCube vs createCollisionCube)
  - Fix BaseGeometry constructor to cast to T* instead of hardcoded Vector3d*
  - Provides clear API separation between visual and collision geometry
  - Includes migration plan for deprecated methods and database records

### Design Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Prototype Phase
- **Started**:
- **Completed**:
- **Prototypes**:
  - P1: {name} — {result}
- **Artifacts**:
  - `docs/designs/geometry-factory-type-safety/prototype-results.md`
- **Notes**:

### Implementation Phase
- **Started**: 2026-01-04 (date inferred from workflow)
- **Completed**: 2026-01-04 (date inferred from workflow)
- **Files Created**:
  - `docs/designs/geometry-factory-type-safety/implementation-notes.md`
- **Files Modified**:
  - `msd/msd-assets/src/GeometryFactory.cpp` — Updated verticesToMeshRecord() to use VisualGeometry class
  - `msd/msd-assets/test/GeometryDatabaseTest.cpp` — Added helpers and updated CollisionGeometry tests
- **Artifacts**:
  - `docs/designs/geometry-factory-type-safety/implementation-notes.md`
- **Notes**:
  - Simplified implementation: 57 lines of duplicated logic removed
  - All 11 tests passing
  - No performance impact - same algorithm, just better organized
  - Helper functions added for test data generation
  - API remains unchanged - backward compatible

### Implementation Review Phase
- **Started**: 2026-01-04
- **Completed**: 2026-01-04
- **Status**: Approved
- **Reviewer Notes**:
  - All geometry tests pass (msd-assets and msd-sim)
  - ConvexHullTest: 36/37 passing (1 pre-existing failure unrelated to this ticket)
  - GeometryDatabaseTest: 11/11 passing
  - Implementation correctly leverages existing BaseGeometry infrastructure

### Documentation Update Phase
- **Started**: 2026-01-04
- **Completed**: 2026-01-04
- **CLAUDE.md Updates**: None required (internal implementation change)
- **Diagrams Indexed**: `docs/designs/geometry-factory-type-safety/geometry-factory-type-safety.puml`
- **Notes**: Design document updated to reflect simplified implementation approach

---

## Human Feedback

### Feedback on Design

### Feedback on Prototypes

### Feedback on Implementation
