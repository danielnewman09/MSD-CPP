# Feature Ticket: Expanding Polytope Algorithm (EPA) for Contact Information

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Documentation Complete — Awaiting Tutorial (if Generate Tutorial: Yes)
- [x] Tutorial Complete — Ready to Merge (if Generate Tutorial: Yes)
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-23
- **Author**: Daniel Newman
- **Priority**: High
- **Estimated Complexity**: Medium
- **Target Component(s)**: msd-sim (Physics module)
- **Parent Ticket**: 0027_collision_response_system
- **Generate Tutorial**: yes

---

## Summary

Implement the Expanding Polytope Algorithm (EPA) to derive detailed contact information from GJK collision detection. When GJK detects an intersection, EPA expands the terminating simplex to find the point on the Minkowski difference boundary closest to the origin, yielding the penetration depth and contact normal required for collision response.

This is a subticket of 0027 that focuses purely on the geometric contact derivation without the physics response.

## Motivation

The current GJK implementation returns only a boolean intersection result. For realistic collision response, we need:
- **Penetration depth**: How far objects overlap (for position correction)
- **Contact normal**: Direction to separate objects (for impulse calculation)
- **Contact point(s)**: Where on the objects the collision occurs

EPA is the standard algorithm for extracting this information from GJK's terminating simplex. It works by iteratively expanding the simplex into a polytope until the closest point on the Minkowski boundary is found.

## Requirements

### Functional Requirements
1. GJK shall expose a `getSimplex()` method returning the terminating simplex when intersection is detected
2. EPA shall accept the GJK simplex and two AssetPhysical objects as input
3. EPA shall compute the penetration depth (scalar distance)
4. EPA shall compute the contact normal (unit vector pointing from A to B in world space)
5. EPA shall compute at least one contact point in world space
6. EPA shall return results in a `CollisionResult` struct

### Non-Functional Requirements
- **Accuracy**: Penetration depth within 1e-6 tolerance for unit cube overlap scenarios (matches AC2 from parent ticket)
- **Performance**: Maximum 64 iterations (configurable)
- **Memory**: Stack-based face/edge storage where practical; minimize heap allocations
- **Backward Compatibility**: GJK API unchanged except for new `getSimplex()` method

## Constraints
- Must reuse GJK's `supportMinkowski()` for Minkowski difference support queries
- Contact normal must be in world space, pointing from object A toward object B
- Must handle degenerate cases (coplanar points, numerical edge cases)
- No changes to ConvexHull or AssetPhysical interfaces

## Acceptance Criteria
- [ ] AC1: GJK exposes `getSimplex()` method returning `const std::vector<Coordinate>&`
- [ ] AC2: EPA returns correct contact normal for axis-aligned cube overlaps (±X, ±Y, ±Z)
- [ ] AC3: EPA penetration depth within 1e-6 of expected value for known overlaps
- [ ] AC4: EPA handles edge-edge contact cases (contact normal perpendicular to both edges)
- [ ] AC5: EPA handles face-vertex contact cases (contact normal equals face normal)
- [ ] AC6: EPA terminates within configurable max iterations
- [ ] AC7: EPA returns meaningful error/fallback when convergence fails
- [ ] AC8: CollisionResult struct contains: `normal`, `penetrationDepth`, `contactPoint` (no `intersecting` - conveyed via std::optional)
- [ ] AC9: CollisionHandler orchestrates GJK/EPA and returns `std::optional<CollisionResult>`

---

## Algorithm Overview

### EPA Algorithm Steps

1. **Initialize Polytope**: Start with GJK's terminating tetrahedron (4 vertices, 4 triangular faces)

2. **Find Closest Face**: Identify the face closest to the origin using distance calculation:
   ```
   distance = |face.normal · face.vertex| / |face.normal|
   ```

3. **Expand Polytope**:
   - Compute support point in direction of closest face's normal
   - If new point is within tolerance of face distance, we've found the closest point → terminate
   - Otherwise, remove faces visible from new point, create new faces connecting new point to horizon edges

4. **Extract Contact Information**:
   - Contact normal = closest face normal (normalized, pointing A→B)
   - Penetration depth = distance from origin to closest face
   - Contact point = barycentric interpolation on the closest face

### Key Data Structures

```cpp
// CollisionHandler orchestrates the GJK → EPA workflow
class CollisionHandler {
public:
    explicit CollisionHandler(double epsilon = 1e-6);
    std::optional<CollisionResult> checkCollision(
        const AssetPhysical& assetA,
        const AssetPhysical& assetB) const;
};

// CollisionResult contains contact info (no 'intersecting' field)
// Collision state conveyed by std::optional: nullopt = no collision
struct CollisionResult {
    Coordinate normal;           // Contact normal (world space, A→B)
    double penetrationDepth;     // Overlap distance [m]
    Coordinate contactPoint;     // Contact location (world space)
};

struct EPAFace {
    std::array<size_t, 3> vertexIndices;
    Coordinate normal;
    double distance;  // Distance from origin to face plane
};

struct EPAEdge {
    size_t v0, v1;  // Vertex indices forming the edge
};
```

---

## Design Decisions (Human Input)

### Preferred Approaches
- EPA as a standalone class (separate from GJK) for clean separation of concerns
- Use priority queue for face selection (closest face first)
- Store Minkowski vertices directly (not original hull vertices) - simpler bookkeeping
- Return single contact point initially; contact manifold can be future enhancement

### Things to Avoid
- Don't modify GJK's internal algorithm - only expose simplex access
- Don't compute multiple contact points (defer to future ticket if needed)
- Don't optimize for broadphase integration yet

### Open Questions
- Should EPA cache the expanded polytope for manifold caching? (Suggest: no, not in this ticket) **response** no
- Epsilon tolerance for face distance comparison? (Suggest: 1e-6 to match acceptance criteria) **response** 1e-6 for now.

---

## References

### Related Code
- `msd-sim/src/Physics/GJK.hpp` — Add `getSimplex()` method
- `msd-sim/src/Physics/GJK.cpp` — Contains `supportMinkowski()` to reuse
- `msd-sim/src/Physics/RigidBody/ConvexHull.hpp` — Reference for facet structure
- `msd-sim/test/Physics/GJKTest.cpp` — Test patterns to follow

### Algorithm References
- Ericson, "Real-Time Collision Detection" (2004), Chapter 9.3
- Muratori, "Implementing GJK/EPA" (2006)
- van den Bergen, "Collision Detection in Interactive 3D Environments" (2003)

### Related Tickets
- `0022_gjk_asset_physical_transform` — GJK with world-space transforms (foundation)
- `0027_collision_response_system` — Parent ticket (will consume EPA output)

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-01-23 (Workflow Orchestrator)
- **Completed**: 2026-01-23
- **Artifacts**:
  - `docs/designs/0027a_expanding_polytope_algorithm/design.md`
  - `docs/designs/0027a_expanding_polytope_algorithm/0027a_expanding_polytope_algorithm.puml`
- **Notes**:
  - User confirmed: no polytope caching, epsilon = 1e-6
  - EPA designed as standalone class separate from GJK
  - Linear search for closest face (simpler than priority queue for typical cases)
  - Barycentric centroid for contact point (adequate accuracy for initial implementation)
  - Exception-based error handling for convergence failures
  - GJK modified only to add getSimplex() accessor (backward compatible)
  - Prototypes needed: convergence validation, horizon edge robustness
  - **Design Revision (Human Feedback)**:
    - Removed `intersecting` boolean from CollisionResult (redundant - EPA only called when collision exists)
    - Added CollisionHandler class to orchestrate GJK→EPA workflow
    - CollisionHandler returns `std::optional<CollisionResult>` (nullopt = no collision)
    - Keeps collision logic abstracted for future enhancements (broadphase, etc.)

### Design Review Phase
- **Started**: 2026-01-23
- **Completed**: 2026-01-23
- **Status**: APPROVED WITH NOTES
- **Reviewer Notes**:
  - Design passes all architectural fit, C++ quality, feasibility, and testability criteria
  - Minor suggestion: Use `std::numeric_limits<double>::quiet_NaN()` for CollisionResult::penetrationDepth default
  - Minor suggestion: Use `CoordinateRate` for direction parameter in `EPA::supportMinkowski()` for consistency with GJK
  - Two prototypes recommended: P1 (EPA convergence validation, 2hr), P2 (horizon edge robustness, 2hr)
  - Total prototype time: 4 hours
  - Ready for human gate review, then prototype phase

### Prototype Phase
- **Started**: 2026-01-23
- **Completed**: 2026-01-23
- **Prototypes**:
  - P1: EPA Convergence Validation — PARTIAL VALIDATION (core algorithm sound, requires integration testing)
  - P2: Horizon Edge Robustness — VALIDATED ✓ (100% success, watertight topology maintained)
- **Artifacts**:
  - `docs/designs/0027a_expanding_polytope_algorithm/prototype-results.md`
  - `prototypes/0027a_expanding_polytope_algorithm/p1_convergence/` — Manual expansion validation
  - `prototypes/0027a_expanding_polytope_algorithm/p2_horizon/` — Topology robustness validation
- **Notes**:
  - Total time: 3.5 hours (within 4-hour budget)
  - P1: Core algorithm logic validated with manual expansion sequences; full convergence validation requires actual ConvexHull support queries (integration testing)
  - P2: Horizon edge construction robust - no duplicate edges, all polytopes remain watertight across 27 expansions
  - No design changes needed; proceed to implementation with comprehensive integration tests
  - TopologyValidator from P2 should be preserved for integration test validation
  - Priority integration tests: analytical solutions (overlapping cubes), GJK→EPA workflow, iteration counting

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
  - `msd-sim/src/Physics/CollisionHandler.hpp`
  - `msd-sim/src/Physics/CollisionHandler.cpp`
  - `msd-sim/src/Physics/EPA.hpp`
  - `msd-sim/src/Physics/EPA.cpp`
  - `msd-sim/src/Physics/CollisionResult.hpp`
  - `msd-sim/test/Physics/EPATest.cpp`
  - `msd-sim/test/Physics/CollisionHandlerTest.cpp`
- **Files Modified**:
  - `msd-sim/src/Physics/GJK.hpp` (add getSimplex)
  - `msd-sim/CMakeLists.txt` (add new sources)
- **Artifacts**:
  - `docs/designs/0027a_expanding_polytope_algorithm/implementation-notes.md`
- **Notes**:

### Implementation Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Documentation Update Phase
- **Started**: 2026-01-23
- **Completed**: 2026-01-23
- **CLAUDE.md Updates**:
  - `msd/msd-sim/src/Physics/CLAUDE.md` — Added CollisionHandler, EPA, and CollisionResult component sections
  - `msd/msd-sim/CLAUDE.md` — Added Recent Architectural Changes entry for EPA implementation
- **Diagrams Indexed**:
  - `docs/msd/msd-sim/Physics/epa.puml` — Copied from design folder with new/modified highlighting removed
  - Added to Diagrams Index in `msd/msd-sim/CLAUDE.md`
  - Added to Core Components table in `msd/msd-sim/src/Physics/CLAUDE.md`
- **Notes**:
  - Documentation sync summary created at `docs/designs/0027a_expanding_polytope_algorithm/doc-sync-summary.md`
  - All component sections include required subsections: Purpose, Interfaces, Usage Example, Thread Safety, Error Handling, Memory Management
  - Fixed existing diagram links in Physics/CLAUDE.md (InertialCalculations → mirtich-inertia-tensor.puml, GJK → gjk-asset-physical.puml)
  - Documentation ready for collision response system (ticket 0027)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design

Looking at the design, I feel like there's an opportunity to improve the workflow. If there is no collision, we will immediately exit the GJK algorithm with a false tag. Otherwise, we will go through the EPA algorithm and get the full data structure with the collision information. For this reason, the `CollisionResult` for `EPA` should - I believe - not have the `intersecting` boolean.

Additionally, I would like to see a `CollisionHandler` skeleton here, showing how the collision is working, when the different algorithms are getting invoked, and how the data should be passed. Can we update the design (and potentially the ticket A/C) to show the very rudimentary layout of how the collision detection algorithm will work? I do not want to just drop functions into the world model - I expect the collision mechanism to become more sophisticated over time.

**Resolution**: Design updated to address both points:
1. Removed `intersecting` from CollisionResult - collision state now conveyed via `std::optional`
2. Added `CollisionHandler` class that orchestrates GJK→EPA workflow and returns `std::optional<CollisionResult>`


### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}
