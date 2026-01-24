# EPA Design Context for cpp-architect Agent

## Task
Design the Expanding Polytope Algorithm (EPA) implementation for ticket 0027a_expanding_polytope_algorithm.

## Ticket Location
`/Users/danielnewman/Documents/GitHub/MSD-CPP/tickets/0027a_expanding_polytope_algorithm.md`

## Key Requirements from Ticket

### Functional Requirements
1. GJK shall expose a `getSimplex()` method returning the terminating simplex when intersection is detected
2. EPA shall accept the GJK simplex and two AssetPhysical objects as input
3. EPA shall compute the penetration depth (scalar distance)
4. EPA shall compute the contact normal (unit vector pointing from A to B in world space)
5. EPA shall compute at least one contact point in world space
6. EPA shall return results in a `CollisionResult` struct

### Non-Functional Requirements
- **Accuracy**: Penetration depth within 1e-6 tolerance for unit cube overlap scenarios
- **Performance**: Maximum 64 iterations (configurable)
- **Memory**: Stack-based face/edge storage where practical; minimize heap allocations
- **Backward Compatibility**: GJK API unchanged except for new `getSimplex()` method

### Constraints
- Must reuse GJK's `supportMinkowski()` for Minkowski difference support queries
- Contact normal must be in world space, pointing from object A toward object B
- Must handle degenerate cases (coplanar points, numerical edge cases)
- No changes to ConvexHull or AssetPhysical interfaces

### User Answers to Open Questions
- **Should EPA cache the expanded polytope for manifold caching?** No
- **Epsilon tolerance for face distance comparison?** 1e-6

## Existing Architecture Context

### Current GJK Implementation
**Location**: `msd-sim/src/Physics/GJK.hpp`, `GJK.cpp`
**Ticket**: 0022_gjk_asset_physical_transform

```cpp
class GJK {
public:
  GJK(const AssetPhysical& assetA, const AssetPhysical& assetB, double epsilon = 1e-6);
  bool intersects(int maxIterations = 64);

private:
  const AssetPhysical& assetA_;
  const AssetPhysical& assetB_;
  double epsilon_;
  std::vector<Coordinate> simplex_;  // Currently private
  Coordinate direction_;

  Coordinate supportMinkowski(const CoordinateRate& dir) const;
  // ... other private methods
};
```

**Key Points**:
- GJK maintains a `simplex_` vector that contains the terminating simplex when intersection is detected
- `supportMinkowski()` applies ReferenceFrame transformations on-the-fly (rotation + translation)
- Returns Minkowski difference support point in world space: `supportA_world - supportB_world`
- GJK API should remain unchanged except for adding `getSimplex()` accessor

### AssetPhysical
**Location**: `msd-sim/src/Physics/RigidBody/AssetPhysical.hpp`

```cpp
class AssetPhysical {
  const ConvexHull& getCollisionHull() const;
  const ReferenceFrame& getReferenceFrame() const;
};
```

### ConvexHull
**Location**: `msd-sim/src/Physics/RigidBody/ConvexHull.hpp`

```cpp
class ConvexHull {
  struct Facet {
    std::array<size_t, 3> vertexIndices;
    Coordinate normal;
    double offset;
  };

  const std::vector<Coordinate>& getVertices() const;
  const std::vector<Facet>& getFacets() const;
  Coordinate getCentroid() const;
};
```

### Coordinate System
- Right-handed Cartesian (Aerospace convention)
- X-axis: Forward, Y-axis: Right, Z-axis: Up
- SI units: meters [m], radians [rad]

## Design Guidance

### Preferred Approaches (from ticket)
- EPA as a standalone class (separate from GJK) for clean separation of concerns
- Use priority queue for face selection (closest face first)
- Store Minkowski vertices directly (not original hull vertices) - simpler bookkeeping
- Return single contact point initially; contact manifold can be future enhancement

### Things to Avoid
- Don't modify GJK's internal algorithm - only expose simplex access
- Don't compute multiple contact points (defer to future ticket if needed)
- Don't optimize for broadphase integration yet

### Algorithm Structure (from ticket)

**EPA Steps**:
1. **Initialize Polytope**: Start with GJK's terminating tetrahedron (4 vertices, 4 triangular faces)
2. **Find Closest Face**: Identify face closest to origin using distance calculation
3. **Expand Polytope**: Get support point in direction of closest face's normal
   - If new point within tolerance of face distance → terminate (found closest point)
   - Otherwise, remove faces visible from new point, create new faces to horizon edges
4. **Extract Contact Information**:
   - Contact normal = closest face normal (normalized, pointing A→B)
   - Penetration depth = distance from origin to closest face
   - Contact point = barycentric interpolation on closest face

**Data Structures** (from ticket):
```cpp
struct CollisionResult {
    bool intersecting;
    Coordinate normal;           // World space, A→B
    double penetrationDepth;     // [m]
    Coordinate contactPoint;     // World space
};

struct EPAFace {
    std::array<size_t, 3> vertexIndices;
    Coordinate normal;
    double distance;  // From origin
};

struct EPAEdge {
    size_t v0, v1;
};
```

### Integration with GJK

**Typical Usage Flow**:
```cpp
// 1. Run GJK
GJK gjk{assetA, assetB};
if (!gjk.intersects()) {
    // No collision
    return CollisionResult{false, ...};
}

// 2. Get terminating simplex
const std::vector<Coordinate>& simplex = gjk.getSimplex();

// 3. Run EPA
EPA epa{assetA, assetB};
CollisionResult result = epa.computeContactInfo(simplex);
```

### Reusing supportMinkowski

EPA needs access to `supportMinkowski()` but it's currently private to GJK. Design options:
1. **Option A**: Make `supportMinkowski()` protected and have EPA inherit from GJK
2. **Option B**: Extract `supportMinkowski()` to a free function in anonymous namespace
3. **Option C**: Make EPA a friend of GJK
4. **Option D**: Have EPA hold references to AssetPhysical and reimplement support function
5. **Option E**: Make `supportMinkowski()` a static member of GJK that EPA can call

**Recommendation**: Prefer option B (free function) or D (EPA reimplements) for clean separation. Option E is also acceptable.

## Files to Review for Architecture

### Existing Physics Components
- `msd-sim/src/Physics/GJK.hpp` — Current GJK interface
- `msd-sim/src/Physics/GJK.cpp` — GJK implementation with supportMinkowski
- `msd-sim/src/Physics/RigidBody/ConvexHull.hpp` — Facet structure reference
- `msd-sim/src/Physics/RigidBody/AssetPhysical.hpp` — AssetPhysical interface

### Diagrams to Reference
- `docs/msd/msd-sim/Physics/gjk-asset-physical.puml` — GJK architecture
- `docs/msd/msd-sim/Physics/physics-core.puml` — Physics module overview

### Test Patterns
- `msd-sim/test/Physics/GJKTest.cpp` — Test patterns to follow

## Coding Standards to Apply

From `/Users/danielnewman/Documents/GitHub/MSD-CPP/CLAUDE.md`:

### Memory Management
- Use plain references (`const AssetPhysical&`) for non-owning access
- Avoid `std::shared_ptr` - establish clear ownership hierarchies
- Value semantics for result structs (CollisionResult)

### Initialization
- Use `std::numeric_limits<double>::quiet_NaN()` for uninitialized floats
- Brace initialization `{}` throughout
- Rule of Zero - use `= default` for special member functions

### Return Values
- Return `CollisionResult` struct by value (not output parameters)
- Use `std::optional` only if result is truly optional (not in this case)

### Naming
- Classes: `PascalCase` (EPA, CollisionResult)
- Methods: `camelCase` (computeContactInfo, getSimplex)
- Members: `snake_case_` (simplex_vertices_, closest_face_)
- Constants: `kPascalCase` (kDefaultEpsilon)

## Open Questions for Designer

### Design Decisions
1. **supportMinkowski reuse**: Which option (A-E) for reusing GJK's support function?
2. **Face storage**: `std::vector<EPAFace>` or `std::priority_queue`? (Ticket suggests priority queue)
3. **Edge storage during expansion**: Stack-allocated array vs. `std::vector`?
4. **Error handling**: Return optional vs. throw exception vs. status in CollisionResult?

### Prototype Required
1. **Convergence validation**: Does EPA reliably converge within 64 iterations for typical cases?
2. **Horizon edge construction**: Does the visible face removal + horizon edge stitching work robustly?

## Target Directory Structure

```
msd-sim/
├── src/
│   └── Physics/
│       ├── GJK.hpp                    # Modified: add getSimplex()
│       ├── GJK.cpp                    # Modified: minor
│       ├── EPA.hpp                    # New
│       ├── EPA.cpp                    # New
│       └── CollisionResult.hpp        # New
└── test/
    └── Physics/
        └── EPATest.cpp                # New
```

## Design Deliverables

1. **PlantUML Diagram**: `docs/designs/0027a_expanding_polytope_algorithm/0027a_expanding_polytope_algorithm.puml`
   - Show EPA class, CollisionResult struct, integration with GJK
   - Mark EPA, CollisionResult as `<<new>>`
   - Mark GJK as `<<modified>>` (getSimplex added)

2. **Design Document**: `docs/designs/0027a_expanding_polytope_algorithm/design.md`
   - Complete architecture per cpp-architect template
   - Address all open questions
   - Include test impact analysis

## Related Tickets
- `0022_gjk_asset_physical_transform` — GJK foundation (completed)
- `0027_collision_response_system` — Parent ticket (will consume EPA output)

## Context Summary

EPA is a natural extension of GJK. When GJK terminates with `intersects() == true`, its terminating simplex (tetrahedron) is guaranteed to contain the origin in Minkowski space. EPA expands this simplex outward until it finds the point on the Minkowski boundary closest to the origin, which directly gives penetration depth and contact normal.

The key challenge is maintaining a consistent polytope topology during expansion (removing visible faces, constructing horizon edges, adding new faces). The design should prioritize correctness and clarity over premature optimization.
