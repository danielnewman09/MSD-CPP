# Feature Ticket: WorldModel Asset Refactor

## Status
- [ ] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-11
- **Author**: Daniel Newman
- **Priority**: High
- **Estimated Complexity**: Large
- **Target Component(s)**: msd-sim (Environment, Physics)

---

## Summary

Refactor `WorldModel` to deprecate the unified `Object` class in favor of specialized asset types (`AssetEnvironment` and `AssetInertial`), and consolidate boundary handling into a single `ConvexHull`. This provides compile-time type safety, eliminates runtime type checking and index caches, and creates cleaner separation between static environment objects, dynamic physics objects, and simulation boundaries.

## Motivation

The current `Object` class uses a component-based design with optional members to represent four different object types (Graphical, Inertial, Environmental, Boundary). This approach has several drawbacks:

1. **Runtime type checking**: Even when iterating over known types (e.g., physics objects), runtime checks are required
2. **Memory waste**: All objects allocate space for optional components they may not use
3. **Index cache maintenance**: `physicsObjectIndices_`, `collisionObjectIndices_`, etc. must be rebuilt on add/remove
4. **Type safety**: No compile-time guarantees about object capabilities

The specialized `AssetEnvironment` and `AssetInertial` classes already exist and provide cleaner, type-safe abstractions. Migrating `WorldModel` to use these directly will improve performance, reduce complexity, and make the codebase easier to reason about.

## Requirements

### Functional Requirements
1. The system shall store environment assets in a dedicated `std::vector<AssetEnvironment>`
2. The system shall store inertial (dynamic physics) assets in a dedicated `std::vector<AssetInertial>`
3. The system shall support a single simulation boundary as `std::optional<ConvexHull>`
4. The system shall provide typed accessor methods for each asset category
5. The system shall support direct iteration over typed asset vectors for collision detection
6. The system shall deprecate the `Object` class with a clear migration path

### Non-Functional Requirements
- **Performance**: Direct vector iteration shall be at least as fast as current index-based iteration
- **Memory**: Reduced memory footprint by eliminating optional component storage overhead
- **Thread Safety**: Maintains current thread safety guarantees (single-threaded access)
- **Backward Compatibility**: Phase 1 maintains deprecated Object methods; Phase 3 removes them

## Constraints
- Must not break existing simulation behavior during migration
- Asset classes (`AssetEnvironment`, `AssetInertial`) must support value semantics for vector storage
- Collision detection must iterate efficiently over moving objects vs static environment

## Acceptance Criteria
- [ ] `WorldModel` stores `AssetEnvironment` objects in dedicated vector
- [ ] `WorldModel` stores `AssetInertial` objects in dedicated vector
- [ ] `WorldModel` stores single `ConvexHull` boundary via `std::optional<ConvexHull>`
- [ ] `Object` class is marked `[[deprecated]]`
- [ ] All existing tests pass (with deprecation warnings allowed)
- [ ] New unit tests cover typed storage and direct iteration
- [ ] Benchmark shows iteration performance is not degraded
- [ ] `AssetEnvironment` has public constructor and deprecated factory method
- [ ] Engine uses move semantics for `CollisionGeometry` when spawning assets
- [ ] Render color APIs are deprecated in msd-sim classes

---

## Design Decisions (Human Input Needed)

### Preferred Approaches
- Use value semantics (`std::vector<AssetInertial>`) rather than pointer indirection
- Single boundary hull is sufficient; complex boundaries can use `AssetEnvironment` objects
- Use single `ConvexHull` for a `boundary` object in the world model
- Migrate `Platform` to reference `AssetInertial`
- deprecate render color - currently in use in `Object`

### Things to Avoid
- Do not introduce shared_ptr for asset storage in WorldModel
- Do not create a fourth asset type for graphical-only objects

---

## References

### Related Code
- `msd/msd-sim/src/Environment/Object.hpp` — Class to be deprecated
- `msd/msd-sim/src/Environment/WorldModel.hpp` — Primary class to be modified
- `msd/msd-sim/src/Physics/RigidBody/AssetPhysical.hpp` — Base class for physical assets
- `msd/msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp` — Static object asset class
- `msd/msd-sim/src/Physics/RigidBody/AssetInertial.hpp` — Dynamic physics asset class
- `msd/msd-sim/src/Engine.hpp` — Consumer of WorldModel, needs spawn method updates

### Related Documentation
- `docs/designs/worldmodel-asset-refactor/design.md` — Full architectural design
- `docs/designs/worldmodel-asset-refactor/worldmodel-asset-refactor.puml` — PlantUML diagram

### Related Tickets
- None (this is a foundational refactor)

---

## Workflow Log

### Design Phase
- **Started**: 2026-01-11
- **Completed**: 2026-01-11
- **Artifacts**:
  - `docs/designs/worldmodel-asset-refactor/design.md`
  - `docs/designs/worldmodel-asset-refactor/worldmodel-asset-refactor.puml`
- **Notes**: Initial design created by cpp-architect agent

### Design Review Phase — Initial Review
- **Started**: 2026-01-11
- **Completed**: 2026-01-11
- **Status**: NEEDS REVISION
- **Reviewer Notes**: 7 issues identified requiring revision. See design.md for detailed review and revision instructions.

### Design Revision Phase
- **Started**: 2026-01-11
- **Completed**: 2026-01-11
- **Artifacts**:
  - Updated `docs/designs/worldmodel-asset-refactor/design.md` (added revision notes)
  - Updated `docs/designs/worldmodel-asset-refactor/worldmodel-asset-refactor.puml` (removed CollisionIterator, updated types)
- **Notes**: All 7 issues addressed based on human feedback:
  - I1: AssetPhysical uses value semantics for geometry storage
  - I2: AssetEnvironment adds public constructor, deprecates factory
  - I3: Deprecated all render color APIs (msd-gpu responsibility)
  - I4: Removed CollisionIterator (only iterate moving objects)
  - I5: Engine uses move semantics for CollisionGeometry construction
  - I6: Platform delegates to AssetInertial (no separate InertialState)
  - I7: getBoundary() returns `const std::optional<ConvexHull>&`

### Design Review Phase — Final Review
- **Started**: 2026-01-11
- **Completed**: 2026-01-11
- **Status**: APPROVED
- **Reviewer Notes**: All revision issues resolved. Design ready for prototype phase.

### Prototype Phase
- **Status**: SKIPPED (no prototypes required after design revision)
- **Notes**: CollisionIterator was removed from design, eliminating the need for prototype P1. Direct vector iteration performance is well-understood.

### Implementation Phase
- **Started**: 2026-01-11
- **Completed**: 2026-01-11
- **Files Created**:
  - `msd/msd-sim/test/Environment/WorldModelTest.cpp` (322 LOC)
  - `msd/msd-sim/src/Physics/RigidBody/AssetPhysical.cpp` (39 LOC)
  - `msd/msd-sim/src/Physics/RigidBody/AssetEnvironment.cpp` (30 LOC)
- **Files Modified**:
  - `msd/msd-sim/src/Environment/WorldModel.hpp` (+181 LOC)
  - `msd/msd-sim/src/Environment/WorldModel.cpp` (+155 LOC)
  - `msd/msd-sim/src/Physics/RigidBody/AssetPhysical.hpp` (+3 LOC)
  - `msd/msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp` (+23 LOC)
  - `msd/msd-sim/src/Physics/RigidBody/AssetInertial.hpp` (+7 LOC)
  - `msd/msd-sim/src/Physics/RigidBody/AssetInertial.cpp` (+16 LOC)
  - `msd/msd-sim/src/Environment/Object.hpp` (+21 LOC)
  - `msd/msd-sim/src/Engine.hpp` (+50 LOC)
  - `msd/msd-sim/src/Engine.cpp` (+90 LOC)
  - `msd/msd-sim/src/Physics/RigidBody/CMakeLists.txt` (+3 LOC)
  - `msd/msd-sim/test/Environment/CMakeLists.txt` (+1 LOC)
- **Artifacts**:
  - `docs/designs/worldmodel-asset-refactor/implementation-notes.md`
- **Notes**: Implementation complete per design specification. All typed storage methods added, value semantics enforced, deprecation warnings applied. 20 unit tests passing.

### Quality Gate Phase
- **Started**: 2026-01-11 15:30
- **Completed**: 2026-01-11 15:30
- **Status**: FAILED
- **Iteration**: 1
- **Artifacts**:
  - `docs/designs/worldmodel-asset-refactor/quality-gate-report.md`
- **Notes**: Build failed due to deprecation warnings treated as errors in Release mode. Legacy code paths use deprecated Object APIs for backward compatibility (per design Phase 1), but warnings-as-errors policy causes compilation failure. Implementer must suppress deprecation warnings for intentionally deprecated code paths.

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

### Feedback on Design Review (Addressed in Revision)

**Issue I1 (AssetPhysical shared_ptr)**: Agrees with required change - use value storage for geometry ✓

**Issue I2 (AssetEnvironment factory)**: Agrees - use public constructor and deprecate the factory method ✓

**Issue I3 (Color storage)**: All references to render color in msd-sim should be deprecated. Responsibility for render color and texture are in msd-gpu library ✓

**Issue I4 (CollisionIterator)**: Do NOT implement CollisionIterator. Only need to iterate over moving objects (AssetInertial) since environmental and boundary objects are static ✓

**Issue I5 (Engine spawn mismatch)**: Modify design to create CollisionGeometry object and construct AssetInertial using move semantics. Alter AssetInertial constructor to accept rvalue reference to collision geometry rather than shared pointer ✓

**Issue I6 (Platform migration)**: Platform should NOT contain InertialState independent of AssetInertial's state. Platform should delegate to AssetInertial ✓

**Issue I7 (Boundary return type)**: Agrees with required change - use `std::optional<ConvexHull>&` ✓

### Feedback on Implementation
{Your comments on the implementation}
