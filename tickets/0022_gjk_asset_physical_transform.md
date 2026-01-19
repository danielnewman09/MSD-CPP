# Feature Ticket: GJK AssetPhysical Transform Support

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [x] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-18
- **Author**: Human + AI
- **Priority**: High
- **Estimated Complexity**: Medium
- **Target Component(s)**: msd-sim (Physics/GJK, Physics/RigidBody)

---

## Summary
Modify GJK collision detection algorithm to work with `AssetPhysical` objects that include `ReferenceFrame` transformations, enabling collision detection between objects with arbitrary positions and orientations in world space.

## Motivation
The current GJK implementation only works with raw `ConvexHull` objects, which don't incorporate spatial transformations (translations and rotations). `AssetPhysical` combines collision hulls with world-space transformations via `ReferenceFrame`, which is essential for realistic physics simulation. Without this feature, collision detection cannot be performed on objects positioned and oriented in the simulation world, severely limiting the usefulness of the physics engine.

## Requirements

### Functional Requirements
1. The GJK algorithm shall accept `AssetPhysical` references as input for collision detection
2. The system shall apply transformations from `ReferenceFrame` during the support function computation
3. The system shall replace references to the existing `ConvexHull`-only interface with the new `AssetPhysical` implementation
4. The system shall correctly handle collisions between objects with arbitrary translations and rotations
5. The system shall provide collision distance/penetration depth information for transformed objects

### Non-Functional Requirements
- **Performance**: Transformation application should be done on-the-fly in the support function without creating temporary transformed hulls (which would be memory-intensive and slow)
- **Memory**: No additional heap allocations during collision detection; use stack-based transformation math
- **Thread Safety**: GJK algorithm is currently stateless and should remain thread-safe
- **Breaking Change**: This is intentionally a breaking change. The existing `ConvexHull`-only GJK interface will be replaced with an `AssetPhysical`-only interface. Rationale: All physics simulation should use `AssetPhysical` objects with proper world-space transformations; maintaining redundant interfaces adds complexity without clear benefit.

## Constraints
- Must use existing `ReferenceFrame` class for transformations (see `msd/msd-sim/src/Physics/ReferenceFrame.hpp`)
- Must follow project coding standards: brace initialization, Rule of Zero/Five, NaN for uninitialized floats
- Must avoid `shared_ptr` in favor of references for non-owning access per project standards
- Existing GJK tests must be migrated to use `AssetPhysical` with identity `ReferenceFrame` (breaking change approved)

## Acceptance Criteria
- [x] GJK detects collisions between transformed `AssetPhysical` objects correctly
- [x] Old `ConvexHull`-only GJK constructor is removed
- [x] Old `ConvexHull::intersects()` method is removed
- [x] Single new `GJK(AssetPhysical, AssetPhysical)` constructor is implemented
- [x] Single new `gjkIntersects(AssetPhysical, AssetPhysical)` convenience function is implemented
- [x] Existing GJK tests are migrated to use `AssetPhysical` with identity transform
- [x] New tests cover translation-only, rotation-only, and combined transformation scenarios
- [x] New tests cover edge cases (touching objects, separated objects, penetrating objects)
- [x] All tests pass (migrated and new)
- [x] Performance benchmarks show acceptable overhead for transformation application

---

## Design Decisions (Human Input)

### Preferred Approaches
- Transform search directions and support function vertices on-the-fly rather than creating temporary transformed hulls
- Use `ReferenceFrame::globalToLocalRelative()` to transform search directions from world space to local object space **Human Note** The transformation is going to be the key component here. I believe that the convex hull stores a `std::vector<Coordinate>` for its local coordinates. Essentially, these need to be translated to the world coordinate frame for both collision hulls under consideration. Consider a getter for the member variable of the collision hull containing the vertex data. I believe that we will need to convert both hull coordinates into the global frame in order to do this. 
- Use `ReferenceFrame::localToGlobal()` to transform support vertices from local object space back to world space **Human note** see my above comments

### Things to Avoid
- Creating temporary transformed `ConvexHull` objects (expensive memory allocation and vertex copying)
- Using `shared_ptr` for `AssetPhysical` references (violates project memory management standards)
- Maintaining redundant backward-compatible interfaces (breaking change is intentional)
- Complex inheritance hierarchies or runtime polymorphism where compile-time polymorphism would suffice

### Open Questions
- Should the GJK algorithm return collision information in world space or local space? **Response** the collision information should be in world space
- Should we add benchmarks to ensure transformation overhead is acceptable? **Response** Yes
- What's the preferred pattern for accessing the `ConvexHull` from `AssetPhysical` (getter method vs direct member access)? **Response** There is an existing `getCollisionHull()` function in `AssetPhysical`

---

## References

### Related Code
- `msd/msd-sim/src/Physics/GJK/GJK.hpp` — Current GJK implementation using only `ConvexHull`
- `msd/msd-sim/src/Physics/RigidBody/AssetPhysical.hpp` — Contains `ConvexHull` and `ReferenceFrame` members
- `msd/msd-sim/src/Physics/ReferenceFrame.hpp` — Provides transformation methods for position/orientation
- `msd/msd-sim/src/Physics/Geometry/ConvexHull.hpp` — Base collision geometry

### Related Documentation
- GJK algorithm description: https://en.wikipedia.org/wiki/Gilbert%E2%80%93Johnson%E2%80%93Keerthi_distance_algorithm
- CLAUDE.md coding standards (brace initialization, Rule of Zero/Five, memory management)

### Related Tickets
- None directly related, but this is a foundational feature for physics simulation

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-01-18 16:01
- **Completed**: 2026-01-18 16:01
- **Artifacts**:
  - `docs/designs/0022_gjk_asset_physical_transform/design.md`
  - `docs/designs/0022_gjk_asset_physical_transform/0022_gjk_asset_physical_transform.puml`
- **Notes**: Design completed by cpp-architect agent. Key decisions: (1) Transform on-the-fly in support function to avoid temporary hull allocation, (2) Use `std::optional<std::reference_wrapper<const ReferenceFrame>>` for optional transformation storage, (3) Maintain full backward compatibility with existing ConvexHull-only GJK interface, (4) All simplex construction occurs in world space for direct interpretability. Two prototypes recommended: transformation correctness validation and performance overhead measurement.

### Design Review Phase
- **Started**: 2026-01-18 16:30
- **Completed**: 2026-01-18 17:15
- **Status**: APPROVED
- **Reviewer Notes**: Initial review (iteration 0) identified 5 issues regarding backward compatibility constraints. Human decision updated ticket to explicitly authorize breaking changes. Final review (iteration 1) found all issues resolved. Design is architecturally sound, follows project standards, and correctly implements on-the-fly transformation strategy. Two prototypes recommended: P1 (transformation correctness validation, 1 hour) and P2 (performance overhead measurement, 1.5 hours). Total prototype time: 2.5 hours. Design approved to proceed to prototype phase.

### Prototype Phase
- **Started**: 2026-01-18 17:30
- **Completed**: 2026-01-18 19:45
- **Prototypes**:
  - P1: Transformation Correctness Validation — PASS (4/4 tests)
  - P2: Performance Overhead Measurement — PASS (7/7 benchmarks, < 2% overhead)
- **Artifacts**:
  - `docs/designs/0022_gjk_asset_physical_transform/prototype-results.md`
  - `prototypes/0022_gjk_asset_physical_transform/p1_transform_validation/`
  - `prototypes/0022_gjk_asset_physical_transform/p2_performance_overhead/`
- **Notes**: Both prototypes passed all success criteria. P1 validated transformation correctness for identity, translation, rotation, and combined transforms. P2 measured negligible performance overhead (< 2% across all hull sizes, far below 20% threshold). Design is validated and ready for implementation with high confidence. Estimated implementation time: 6-8 hours.

### Implementation Phase
- **Started**: 2026-01-18 20:00
- **Completed**: 2026-01-18 22:38
- **Files Created**:
  - `msd/msd-sim/test/Physics/GJKTest.cpp` (18 new tests)
- **Files Modified**:
  - `msd/msd-sim/src/Physics/GJK.hpp` (AssetPhysical interface)
  - `msd/msd-sim/src/Physics/GJK.cpp` (transformation pipeline)
  - `msd/msd-sim/src/Physics/RigidBody/ConvexHull.hpp` (removed intersects())
  - `msd/msd-sim/src/Physics/RigidBody/ConvexHull.cpp` (bounding box fix)
  - `msd/msd-sim/src/Physics/RigidBody/AssetPhysical.hpp` (documentation)
  - `msd/msd-sim/bench/ConvexHullBench.cpp` (updated benchmarks)
  - `msd/msd-sim/test/Physics/CMakeLists.txt` (added GJKTest)
  - `msd/msd-sim/bench/CMakeLists.txt` (benchmark updates)
- **Artifacts**:
  - N/A (implementation-notes.md not created)
- **Notes**: Implementation completed successfully. All core changes implemented: (1) GJK stores AssetPhysical references, (2) Old ConvexHull constructor removed, (3) Transformation pipeline in supportMinkowski() using ReferenceFrame methods, (4) ConvexHull::intersects() removed, (5) gjkIntersects() convenience function added, (6) Existing tests migrated to AssetPhysical, (7) 18 new transformation tests added, (8) Benchmarks updated. All 143 tests pass. Discovered and fixed ConvexHull bounding box bug (was using Qhull input bounds, now uses actual vertices).

### Implementation Review Phase
- **Started**: 2026-01-18 22:45
- **Completed**: 2026-01-18 23:00
- **Status**: APPROVED
- **Artifacts**:
  - `docs/designs/0022_gjk_asset_physical_transform/implementation-review.md`
- **Reviewer Notes**: Implementation correctly realizes the validated design. All components implemented as specified with only one approved deviation (bounding box bug fix). Transformation pipeline matches prototype validation exactly. Code quality excellent: follows all project standards, proper const correctness, clean implementation. Test coverage comprehensive: 18 new tests covering identity, translation, rotation, combined transforms, and edge cases. All 143 tests pass. No critical or major issues found. Minor benchmark warnings (7 sign-conversion warnings in GJKBench.cpp) are non-blocking. Feature is production-ready and approved to proceed to documentation update phase.

### Documentation Update Phase
- **Started**: 2026-01-18 23:15
- **Completed**: 2026-01-18 23:30
- **CLAUDE.md Updates**:
  - `msd/msd-sim/CLAUDE.md` — Added "Recent Architectural Changes" section, updated Physics module summary and diagrams index
  - `msd/msd-sim/src/Physics/CLAUDE.md` — Complete rewrite of GJK section for AssetPhysical API, removed ConvexHull::intersects() references
- **Diagrams Indexed**:
  - `docs/msd/msd-sim/Physics/gjk-asset-physical.puml` — GJK collision detection with AssetPhysical transforms (adapted from design diagram)
- **Artifacts**:
  - `docs/designs/0022_gjk_asset_physical_transform/doc-sync-summary.md`
- **Notes**: Documentation fully synchronized with implementation. Breaking changes clearly documented with migration guidance. Performance characteristics (< 2% overhead) and transformation pipeline documented. All references to removed ConvexHull::intersects() method cleaned up. Feature is ready to merge.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
**✓ ADDRESSED 2026-01-18**: Design review identified conflicts between the simplified design and original backward-compatibility requirements. Human decision: **Proceed with breaking changes**. Updated ticket constraints and acceptance criteria to explicitly allow:
- Removal of `ConvexHull`-only GJK constructor
- Removal of `ConvexHull::intersects()` method
- Single `AssetPhysical`-only interface
- Migration of existing tests to use `AssetPhysical` with identity transform

Rationale: Simpler API, all physics simulation should use `AssetPhysical` anyway, maintaining redundant interfaces adds complexity without benefit.

**Resolution**: Design review re-run with updated constraints. All backward compatibility issues (I1-I4) resolved. Design approved to proceed to prototype phase.

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}
