# Design: Expanding Polytope Algorithm (EPA) for Contact Information

## Summary

This design implements the Expanding Polytope Algorithm (EPA) to derive detailed contact information (penetration depth, contact normal, contact point) from GJK collision detection. When GJK detects an intersection, EPA expands the terminating simplex to find the point on the Minkowski difference boundary closest to the origin, yielding the geometric data required for realistic collision response in rigid body physics simulations.

## Architecture Changes

### PlantUML Diagram
See: [`./0027a_expanding_polytope_algorithm.puml`](./0027a_expanding_polytope_algorithm.puml)

### New Components

#### CollisionHandler (Orchestration Layer)

- **Purpose**: Orchestrate GJK and EPA algorithms, providing a clean entry point for collision detection that can grow more sophisticated over time
- **Header location**: `msd-sim/src/Physics/CollisionHandler.hpp`
- **Source location**: `msd-sim/src/Physics/CollisionHandler.cpp`
- **Key interfaces**:
  ```cpp
  /**
   * @brief Orchestrates collision detection algorithms (GJK/EPA).
   *
   * Provides a unified interface for collision detection that:
   * - Runs GJK to detect intersection
   * - If collision detected, runs EPA to compute contact info
   * - Returns std::optional to indicate collision presence
   *
   * This abstraction allows future enhancements (broadphase,
   * continuous collision detection, etc.) without changing callers.
   */
  class CollisionHandler {
  public:
      /**
       * @brief Construct handler with specified tolerance.
       * @param epsilon Numerical tolerance for GJK/EPA (default: 1e-6)
       */
      explicit CollisionHandler(double epsilon = 1e-6);

      /**
       * @brief Check for collision between two physical assets.
       *
       * Runs GJK to detect intersection. If collision detected,
       * runs EPA to compute penetration depth, contact normal,
       * and contact point.
       *
       * @param assetA First physical asset
       * @param assetB Second physical asset
       * @return std::nullopt if no collision, CollisionResult if collision
       */
      std::optional<CollisionResult> checkCollision(
          const AssetPhysical& assetA,
          const AssetPhysical& assetB) const;

      CollisionHandler(const CollisionHandler&) = default;
      CollisionHandler(CollisionHandler&&) noexcept = default;
      CollisionHandler& operator=(const CollisionHandler&) = default;
      CollisionHandler& operator=(CollisionHandler&&) noexcept = default;
      ~CollisionHandler() = default;

  private:
      double epsilon_;
  };
  ```
- **Dependencies**:
  - `GJK` (for intersection detection)
  - `EPA` (for contact information extraction)
  - `AssetPhysical` (collision primitives)
  - `CollisionResult` (return type)
- **Thread safety**: Stateless after construction, safe to call from multiple threads
- **Error handling**: Propagates exceptions from GJK/EPA; does not add additional error conditions

**Implementation Logic**:
```cpp
std::optional<CollisionResult> CollisionHandler::checkCollision(
    const AssetPhysical& assetA,
    const AssetPhysical& assetB) const
{
    // Phase 1: Broad intersection test via GJK
    GJK gjk{assetA, assetB, epsilon_};

    if (!gjk.intersects()) {
        // No collision - return empty optional
        return std::nullopt;
    }

    // Phase 2: Detailed contact info via EPA
    // Only reached when GJK confirms intersection
    EPA epa{assetA, assetB, epsilon_};
    CollisionResult result = epa.computeContactInfo(gjk.getSimplex());

    return result;
}
```

**Usage Pattern**:
```cpp
// In WorldModel or similar
CollisionHandler collisionHandler{1e-6};

// For each pair of inertial assets
auto result = collisionHandler.checkCollision(assetA, assetB);
if (result) {
    // Collision detected - apply physics response
    applyImpulse(assetA, assetB, result->normal, result->penetrationDepth);
}
// else: no collision, nothing to do
```

**Design Rationale**: The CollisionHandler abstraction provides several benefits:
1. **Encapsulation**: WorldModel doesn't need to know about GJK/EPA internals
2. **Extensibility**: Future broadphase optimization, continuous collision detection, or collision filtering can be added here
3. **Clear data flow**: `std::optional<CollisionResult>` makes collision state explicit without redundant boolean
4. **Testability**: Handler can be tested independently of physics response

#### EPA (Expanding Polytope Algorithm)

- **Purpose**: Compute penetration depth, contact normal, and contact point from GJK terminating simplex
- **Header location**: `msd-sim/src/Physics/EPA.hpp`
- **Source location**: `msd-sim/src/Physics/EPA.cpp`
- **Key interfaces**:
  ```cpp
  class EPA {
  public:
      /**
       * @brief Construct EPA solver for two physical assets.
       * @param assetA First physical asset
       * @param assetB Second physical asset
       * @param epsilon Numerical tolerance (default: 1e-6)
       */
      EPA(const AssetPhysical& assetA,
          const AssetPhysical& assetB,
          double epsilon = 1e-6);

      /**
       * @brief Compute contact information from GJK terminating simplex.
       *
       * Assumes simplex contains origin (GJK returned true).
       * Expands polytope until closest face found within tolerance.
       *
       * @param simplex GJK terminating simplex (4 vertices in Minkowski space)
       * @param maxIterations Maximum expansion iterations (default: 64)
       * @return CollisionResult with penetration depth, normal, contact point
       * @throws std::invalid_argument if simplex size != 4
       */
      CollisionResult computeContactInfo(const std::vector<Coordinate>& simplex,
                                         int maxIterations = 64);

      EPA(const EPA&) = default;
      EPA(EPA&&) noexcept = default;
      EPA& operator=(const EPA&) = default;
      EPA& operator=(EPA&&) noexcept = default;
      ~EPA() = default;

  private:
      // Core algorithm
      bool expandPolytope(int maxIterations);
      size_t findClosestFace() const;

      // Topology management
      bool isVisible(const EPAFace& face, const Coordinate& point) const;
      std::vector<EPAEdge> buildHorizonEdges(const Coordinate& newVertex);
      void addFace(size_t v0, size_t v1, size_t v2);

      // Contact extraction
      Coordinate computeContactPoint(const EPAFace& face) const;

      // Support function (matches GJK implementation)
      Coordinate supportMinkowski(const CoordinateRate& dir) const;

      const AssetPhysical& assetA_;
      const AssetPhysical& assetB_;
      double epsilon_;

      std::vector<Coordinate> vertices_;  // Minkowski vertices
      std::vector<EPAFace> faces_;        // Triangular faces
  };
  ```
- **Dependencies**:
  - `AssetPhysical` (non-owning references for hull and transform access)
  - `ConvexHull` (via AssetPhysical, for support queries)
  - `ReferenceFrame` (via AssetPhysical, for coordinate transformations)
  - `Coordinate` (for vertex and direction storage)
- **Thread safety**: Not thread-safe (mutable state during expansion)
- **Error handling**: Throws `std::invalid_argument` for invalid simplex size; throws `std::runtime_error` if expansion fails to converge

#### CollisionResult

- **Purpose**: Return value struct containing complete collision information from EPA
- **Header location**: `msd-sim/src/Physics/CollisionResult.hpp`
- **Key interfaces**:
  ```cpp
  /**
   * @brief Complete collision information for physics response.
   *
   * This struct is returned by EPA when a collision is detected.
   * It does NOT contain an 'intersecting' boolean because:
   * - CollisionHandler returns std::optional<CollisionResult>
   * - std::nullopt indicates no collision
   * - Presence of CollisionResult implies collision exists
   *
   * All coordinates are in world space.
   * Contact normal points from object A toward object B.
   */
  struct CollisionResult {
      Coordinate normal;           // Contact normal (world space, A→B, unit length)
      double penetrationDepth{std::numeric_limits<double>::quiet_NaN()};  // Overlap distance [m]
      Coordinate contactPoint;     // Contact location (world space) [m]

      CollisionResult() = default;
      CollisionResult(const Coordinate& n,
                      double depth,
                      const Coordinate& point)
        : normal{n},
          penetrationDepth{depth},
          contactPoint{point} {}
  };
  ```
- **Dependencies**: `Coordinate`
- **Thread safety**: Value type, safe to copy across threads
- **Error handling**: None (POD struct)
- **Design rationale**: The `intersecting` boolean was removed because EPA is only invoked when GJK has already confirmed collision. The collision state is conveyed by `CollisionHandler::checkCollision()` returning `std::optional<CollisionResult>` — `std::nullopt` means no collision, presence of a value means collision exists.

#### EPAFace (Internal to EPA)

- **Purpose**: Represent triangular face in expanded polytope
- **Header location**: `msd-sim/src/Physics/EPA.hpp` (private struct within EPA)
- **Key interfaces**:
  ```cpp
  struct EPAFace {
      std::array<size_t, 3> vertexIndices;  // Indices into EPA::vertices_
      Coordinate normal;                     // Outward-facing unit normal
      double distance;                       // Distance from origin to face plane

      EPAFace() = default;
      EPAFace(size_t v0, size_t v1, size_t v2,
              const Coordinate& n, double d)
        : vertexIndices{v0, v1, v2}, normal{n}, distance{d} {}
  };
  ```
- **Dependencies**: `Coordinate`
- **Thread safety**: Value type
- **Error handling**: None

#### EPAEdge (Internal to EPA)

- **Purpose**: Represent edge in polytope for horizon construction
- **Header location**: `msd-sim/src/Physics/EPA.hpp` (private struct within EPA)
- **Key interfaces**:
  ```cpp
  struct EPAEdge {
      size_t v0;  // First vertex index
      size_t v1;  // Second vertex index

      EPAEdge() = default;
      EPAEdge(size_t a, size_t b) : v0{a}, v1{b} {}

      // Equality for duplicate detection (order-independent)
      bool operator==(const EPAEdge& other) const {
          return (v0 == other.v0 && v1 == other.v1) ||
                 (v0 == other.v1 && v1 == other.v0);
      }
  };
  ```
- **Dependencies**: None
- **Thread safety**: Value type
- **Error handling**: None

### Modified Components

#### GJK

- **Current location**: `msd-sim/src/Physics/GJK.hpp`, `GJK.cpp`
- **Changes required**:
  1. Add public `getSimplex()` accessor method:
     ```cpp
     /**
      * @brief Get the terminating simplex for EPA input.
      *
      * @pre intersects() must have been called and returned true.
      *      Behavior is undefined if called before intersects() or
      *      after intersects() returned false.
      *
      * @return Const reference to simplex vertices (4 vertices forming
      *         a tetrahedron in Minkowski space that contains the origin)
      */
     const std::vector<Coordinate>& getSimplex() const { return simplex_; }
     ```
  2. Update class documentation to mention EPA integration
- **Backward compatibility**: Full backward compatibility maintained. New method is purely additive; existing functionality unchanged.

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| CollisionHandler | GJK | Creates & uses | Handler creates GJK instance, calls intersects() |
| CollisionHandler | EPA | Creates if collision | Handler creates EPA only when GJK detects intersection |
| CollisionHandler | AssetPhysical | Parameter passing | Assets passed through to GJK/EPA |
| EPA | AssetPhysical | Composition (references) | EPA stores const references to same assets used by GJK |
| EPA | GJK | Data flow | EPA receives GJK's terminating simplex via `getSimplex()` |
| CollisionResult | Coordinate | Composition | Result struct contains Coordinate fields |
| EPA::supportMinkowski | ConvexHull | Function call | Replicates GJK's support function logic for consistency |

## Algorithm Design

### EPA Expansion Algorithm

The algorithm follows these steps:

1. **Initialize Polytope** (`computeContactInfo` entry point):
   - Receive GJK terminating simplex (4 vertices forming a tetrahedron)
   - Copy simplex vertices to `vertices_` vector
   - Construct 4 initial faces from tetrahedron
   - Calculate normal and distance for each face

2. **Iterative Expansion** (`expandPolytope` loop):
   - Find closest face to origin (`findClosestFace`)
   - Query support point in direction of closest face's normal
   - **Convergence check**: If `newPoint.dot(normal) - face.distance < epsilon`, terminate (found closest point)
   - Otherwise, expand polytope:
     - Identify visible faces from new point (`isVisible`)
     - Build horizon edges from visible faces (`buildHorizonEdges`)
     - Remove visible faces
     - Create new faces connecting new point to horizon edges (`addFace`)

3. **Contact Extraction** (after convergence):
   - Penetration depth = distance of closest face
   - Contact normal = normal of closest face (normalized, pointing A→B)
   - Contact point = barycentric centroid of closest face (`computeContactPoint`)

### Key Methods

#### findClosestFace()
```cpp
size_t EPA::findClosestFace() const {
    double minDistance = std::numeric_limits<double>::infinity();
    size_t closestIndex = 0;

    for (size_t i = 0; i < faces_.size(); ++i) {
        if (faces_[i].distance < minDistance) {
            minDistance = faces_[i].distance;
            closestIndex = i;
        }
    }

    return closestIndex;
}
```

**Alternative (Priority Queue)**: The ticket suggested a priority queue for face selection. However, for typical polytope sizes (<100 faces), linear search is simpler and sufficient. Priority queue adds complexity for minimal benefit. **Recommendation**: Start with linear search, optimize if profiling shows this as a bottleneck.

#### isVisible()
```cpp
bool EPA::isVisible(const EPAFace& face, const Coordinate& point) const {
    // Point is visible from face if it's on the positive side of the face plane
    Coordinate toPoint = point - vertices_[face.vertexIndices[0]];
    return face.normal.dot(toPoint) > epsilon_;
}
```

#### buildHorizonEdges()
```cpp
std::vector<EPAEdge> EPA::buildHorizonEdges(const Coordinate& newVertex) {
    std::vector<EPAEdge> horizon;
    std::vector<size_t> visibleFaceIndices;

    // Identify visible faces
    for (size_t i = 0; i < faces_.size(); ++i) {
        if (isVisible(faces_[i], newVertex)) {
            visibleFaceIndices.push_back(i);
        }
    }

    // Extract edges from visible faces
    std::vector<EPAEdge> edgeCandidates;
    for (size_t idx : visibleFaceIndices) {
        const auto& f = faces_[idx];
        edgeCandidates.emplace_back(f.vertexIndices[0], f.vertexIndices[1]);
        edgeCandidates.emplace_back(f.vertexIndices[1], f.vertexIndices[2]);
        edgeCandidates.emplace_back(f.vertexIndices[2], f.vertexIndices[0]);
    }

    // Horizon edges appear exactly once (not shared by two visible faces)
    for (const auto& edge : edgeCandidates) {
        int count = std::count(edgeCandidates.begin(), edgeCandidates.end(), edge);
        if (count == 1) {
            horizon.push_back(edge);
        }
    }

    // Remove visible faces (erase-remove idiom)
    faces_.erase(
        std::remove_if(faces_.begin(), faces_.end(),
            [&visibleFaceIndices](const EPAFace& face) {
                // Check if face index is in visible list
                // (requires comparing face to faces_[index])
                // Simpler approach: mark faces for deletion via flag
            }),
        faces_.end()
    );

    return horizon;
}
```

**Note**: Actual implementation will use face indices or a more efficient marking scheme to avoid O(n²) face comparison. This pseudocode illustrates the algorithm logic.

#### addFace()
```cpp
void EPA::addFace(size_t v0, size_t v1, size_t v2) {
    Coordinate a = vertices_[v0];
    Coordinate b = vertices_[v1];
    Coordinate c = vertices_[v2];

    // Compute normal via cross product
    Coordinate ab = b - a;
    Coordinate ac = c - a;
    Coordinate normal = ab.cross(ac).normalized();

    // Ensure normal points away from origin (outward)
    Coordinate centroid = (a + b + c) / 3.0;
    if (normal.dot(centroid) < 0.0) {
        normal = -normal;
        std::swap(v1, v2);  // Flip winding order
    }

    // Distance from origin to face plane
    double distance = normal.dot(a);

    faces_.emplace_back(v0, v1, v2, normal, distance);
}
```

#### computeContactPoint()
```cpp
Coordinate EPA::computeContactPoint(const EPAFace& face) const {
    // Barycentric centroid (equal weights)
    Coordinate a = vertices_[face.vertexIndices[0]];
    Coordinate b = vertices_[face.vertexIndices[1]];
    Coordinate c = vertices_[face.vertexIndices[2]];

    return (a + b + c) / 3.0;
}
```

**Note**: This returns the centroid of the closest face in Minkowski space. For more accurate contact points, future work could compute the actual closest point on the triangle to the origin via projection. The centroid is a reasonable approximation for initial implementation.

#### supportMinkowski()
```cpp
Coordinate EPA::supportMinkowski(const CoordinateRate& dir) const {
    // Identical to GJK::supportMinkowski implementation
    const ConvexHull& hullA = assetA_.getCollisionHull();
    const ConvexHull& hullB = assetB_.getCollisionHull();
    const ReferenceFrame& frameA = assetA_.getReferenceFrame();
    const ReferenceFrame& frameB = assetB_.getReferenceFrame();

    // Transform direction to local space, find support, transform back to world space
    Coordinate dirA_local = frameA.globalToLocal(dir);
    Coordinate supportA_local = support(hullA, dirA_local);
    Coordinate supportA_world = frameA.localToGlobal(supportA_local);

    Coordinate dirB_local = frameB.globalToLocal(-dir);
    Coordinate supportB_local = support(hullB, dirB_local);
    Coordinate supportB_world = frameB.localToGlobal(supportB_local);

    return supportA_world - supportB_world;
}

Coordinate EPA::support(const ConvexHull& hull, const Coordinate& dir) const {
    // Identical to GJK::support implementation
    const auto& vertices = hull.getVertices();
    double maxDot = -std::numeric_limits<double>::infinity();
    Coordinate furthest{0.0, 0.0, 0.0};

    for (const auto& vertex : vertices) {
        double dotProduct = vertex.dot(dir);
        if (dotProduct > maxDot) {
            maxDot = dotProduct;
            furthest = vertex;
        }
    }

    return furthest;
}
```

**Design Decision**: Duplicate `supportMinkowski` and `support` logic in EPA rather than extracting to shared utility. **Rationale**: Keeps EPA self-contained and avoids introducing new coupling. The duplication is minimal (~30 lines) and algorithmically identical to GJK, making maintenance straightforward.

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `test/Physics/GJKTest.cpp` | All GJK intersection tests | None | No changes (API backward compatible) |
| `test/Physics/GJKTest.cpp` | N/A | Additive | Add test case verifying `getSimplex()` returns 4 vertices after `intersects() == true` |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| EPA | `EPATest.computeContactInfo_unitCubeOverlap_returnsCorrectPenetration` | Penetration depth within 1e-6 for two overlapping unit cubes at known offset |
| EPA | `EPATest.computeContactInfo_axisAlignedCubes_correctContactNormal` | Contact normal points in expected cardinal direction for axis-aligned overlaps |
| EPA | `EPATest.computeContactInfo_edgeEdgeContact_perpendicularNormal` | Edge-edge contact produces normal perpendicular to both edges |
| EPA | `EPATest.computeContactInfo_faceVertexContact_faceNormalReturned` | Face-vertex contact produces normal equal to face normal |
| EPA | `EPATest.computeContactInfo_convergenceWithinIterations` | EPA terminates within 64 iterations for typical convex hulls |
| EPA | `EPATest.computeContactInfo_invalidSimplexSize_throwsException` | Throws `std::invalid_argument` for simplex.size() != 4 |
| EPA | `EPATest.computeContactInfo_rotatedObjects_worldSpaceCorrect` | Contact normal and point correctly transformed to world space for rotated objects |
| CollisionResult | `CollisionResultTest.construction_storesValues` | Constructor correctly stores normal, depth, and contact point |
| CollisionHandler | `CollisionHandlerTest.checkCollision_noIntersection_returnsNullopt` | Returns std::nullopt when objects don't collide |
| CollisionHandler | `CollisionHandlerTest.checkCollision_intersection_returnsCollisionResult` | Returns CollisionResult when objects collide |
| CollisionHandler | `CollisionHandlerTest.checkCollision_resultContainsValidData` | Returned CollisionResult has valid normal, depth, contact point |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `CollisionHandler_Integration.fullCollisionPipeline_overlappingCubes` | CollisionHandler, GJK, EPA | Handler detects intersection, computes contact info, results are physically plausible |
| `CollisionHandler_Integration.noCollision_returnsNullopt` | CollisionHandler, GJK | When objects don't collide, handler returns std::nullopt (EPA not invoked) |
| `CollisionHandler_Integration.multipleOrientations_consistentResults` | CollisionHandler, GJK, EPA, AssetPhysical | Rotating objects produces consistent contact normals relative to object orientations |

#### Benchmark Tests

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| EPA | `EPABenchmark.computeContactInfo_unitCubes` | Time to compute contact info for overlapping unit cubes | < 50 microseconds |
| EPA | `EPABenchmark.computeContactInfo_complexHulls` | Time for hulls with 50+ vertices | < 200 microseconds |
| EPA | `EPABenchmark.expandPolytope_iterationCount` | Average number of iterations to convergence | < 20 iterations typical |

## Open Questions

### Design Decisions (Human Input Needed)

1. **Face selection strategy: Linear search vs. Priority queue**
   - Option A: Linear search through `faces_` vector — Pros: Simple, no heap allocations, sufficient for <100 faces. Cons: O(n) per iteration.
   - Option B: `std::priority_queue<EPAFace, ..., DistanceComparator>` — Pros: O(log n) face selection. Cons: More complex, heap allocations, requires face update on expansion.
   - **Recommendation**: Option A (linear search). Ticket suggested priority queue, but for typical polytope sizes, the overhead of maintaining the heap outweighs the O(log n) benefit. Revisit if profiling shows face selection as bottleneck.

2. **Contact point computation: Centroid vs. Closest point projection**
   - Option A: Barycentric centroid `(a + b + c) / 3` — Pros: Simple, fast. Cons: Not the mathematically closest point on triangle to origin.
   - Option B: Project origin onto triangle plane, compute barycentric coordinates — Pros: More accurate. Cons: ~3x computational cost, edge case handling (point outside triangle).
   - **Recommendation**: Option A (centroid) for initial implementation. AC5 requires "at least one contact point", not necessarily the optimal one. Refinement can be future work if physics response requires higher accuracy.

3. **Error handling for convergence failure**
   - Option A: Throw `std::runtime_error` — Pros: Fails fast, clear error signal. Cons: Requires exception handling in caller.
   - Option B: Return `std::nullopt` from CollisionHandler — Pros: No exceptions. Cons: Ambiguous (can't distinguish non-collision from convergence failure).
   - **Recommendation**: Option A (throw exception). Convergence failure is an exceptional condition that should not occur for well-formed convex hulls. Throwing makes debugging easier and follows existing project conventions (see ConvexHull, InertialCalculations). Note: CollisionHandler propagates the exception; it does not catch and convert to `std::nullopt`.

### Prototype Required

1. **EPA convergence validation**
   - **Uncertainty**: Does EPA reliably converge within 64 iterations for typical convex hulls (10-50 vertices)?
   - **Prototype goal**: Run EPA on diverse hull shapes (cubes, spheres approximations, tetrahedra, irregular hulls) with varying penetration depths. Measure iteration counts and success rate.
   - **Acceptance criteria**: 95%+ success rate with <32 iterations average.

2. **Horizon edge construction robustness**
   - **Uncertainty**: Does the visible face removal + horizon edge detection work correctly for degenerate cases (coplanar faces, near-zero face normals)?
   - **Prototype goal**: Test EPA on edge cases: very shallow penetrations (depth < 1e-4), near-degenerate simplices, high vertex count hulls.
   - **Acceptance criteria**: No topology corruption (all faces remain triangles, no duplicate vertices, normals remain valid).

### Requirements Clarification

None. User has answered all open questions from ticket:
- EPA shall NOT cache expanded polytope
- Epsilon tolerance = 1e-6

## Performance Considerations

### Memory Usage

- **EPA instance**: 2 references (16 bytes) + double (8 bytes) + 2 vectors (~48 bytes overhead) = ~72 bytes
- **Polytope storage**:
  - Vertices: ~4-100 vertices × 24 bytes (Coordinate) = 96-2400 bytes
  - Faces: ~4-200 faces × (24 bytes indices + 24 bytes normal + 8 bytes distance) = 224-11200 bytes
- **Total**: < 15 KB typical, < 50 KB worst case

**Optimization opportunity**: Stack-allocate initial 4 faces to avoid heap allocation for simple cases. Use `std::vector` for expansion beyond initial tetrahedron.

### Computational Complexity

- **Per iteration**:
  - Find closest face: O(F) where F = number of faces
  - Support query: O(V) where V = vertices in each hull
  - Visible face detection: O(F)
  - Horizon edge construction: O(F)
  - Add new faces: O(H) where H = horizon edge count (typically <10)
- **Total**: O(I × (F + V)) where I = iteration count
- **Typical case**: I ≈ 10-20, F ≈ 10-50, V ≈ 20-100 → ~5000-200,000 operations
- **Expected runtime**: < 100 microseconds for typical hulls

### Comparison to GJK

GJK and EPA have similar computational profiles:
- Both iterate with support queries
- GJK maintains simplex (4 vertices), EPA maintains polytope (10-100 vertices)
- EPA has higher memory footprint but similar per-iteration cost

**Benchmark target**: EPA should execute in < 2× GJK time for typical cases.

## Future Work

This design deliberately omits features for future enhancement:

1. **Contact manifold generation**: Multiple contact points for stable stacking (ticket 0027b or later)
2. **Polytope caching**: Reuse expanded polytope across frames for coherent contacts
3. **Adaptive epsilon**: Adjust tolerance based on object size/velocity
4. **Warm starting**: Initialize EPA with previous frame's polytope for moving objects
5. **Contact point refinement**: Project origin onto triangle for mathematically exact contact point

## References

### Algorithm Sources
- Ericson, "Real-Time Collision Detection" (2004), Chapter 9.3
- van den Bergen, "Collision Detection in Interactive 3D Environments" (2003), Chapter 4
- Muratori, "Implementing GJK/EPA" (2006), Casey Muratori's blog

### Related Tickets
- [0022_gjk_asset_physical_transform](../0022_gjk_asset_physical_transform/) — GJK foundation with AssetPhysical transforms
- [0027_collision_response_system](../../tickets/0027_collision_response_system.md) — Parent ticket for full collision response system

### Existing Architecture
- `docs/msd/msd-sim/Physics/gjk-asset-physical.puml` — GJK architecture diagram
- `docs/msd/msd-sim/Physics/physics-core.puml` — Physics module overview
- `msd-sim/src/Physics/CLAUDE.md` — Physics module documentation

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-01-23
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | Pass | `CollisionHandler`, `CollisionResult`, `EPA`, `EPAFace`, `EPAEdge` follow PascalCase; methods use camelCase; members use `snake_case_` with trailing underscore |
| Namespace organization | Pass | All new components in `msd_sim` namespace, consistent with existing Physics module |
| File structure | Pass | `msd-sim/src/Physics/EPA.hpp`, `CollisionHandler.hpp`, `CollisionResult.hpp` follow established `msd-sim/src/Physics/` pattern |
| Dependency direction | Pass | CollisionHandler depends on GJK and EPA (same layer); EPA depends on AssetPhysical (lower layer); no circular dependencies introduced |

**Notes**: The design correctly places new components within the existing Physics module hierarchy. The CollisionHandler abstraction provides a clean orchestration layer that can grow to support broadphase optimization without coupling WorldModel to algorithm internals.

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | Pass | EPA manages `vertices_` and `faces_` vectors via standard container RAII; no manual resource management required |
| Smart pointer appropriateness | Pass | Uses const references for `AssetPhysical` (non-owning access), consistent with GJK pattern |
| Value/reference semantics | Pass | `CollisionResult` is a value type; EPA stores const references to assets; appropriate for each use case |
| Rule of 0/3/5 | Pass | All classes declare `= default` for copy/move/destructor, correctly applying Rule of Zero |
| Const correctness | Pass | `checkCollision()` is const; `getSimplex()` returns const reference; `CollisionHandler` methods appropriately const |
| Exception safety | Pass | Design specifies `std::runtime_error` for convergence failure, `std::invalid_argument` for bad simplex - consistent with project patterns (ConvexHull, InertialCalculations) |

**Minor Note (not blocking)**: The `CollisionResult` struct provides a default constructor but does not initialize `penetrationDepth` with `std::numeric_limits<double>::quiet_NaN()` as recommended by CLAUDE.md for uninitialized floating-point values. Consider:

```cpp
struct CollisionResult {
    Coordinate normal;
    double penetrationDepth{std::numeric_limits<double>::quiet_NaN()};
    Coordinate contactPoint;
    // ...
};
```

This is a minor documentation/defensive programming note, not a blocking issue since the parameterized constructor is the expected path.

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | Pass | No circular dependencies; EPA.hpp includes only AssetPhysical.hpp, ConvexHull.hpp, Coordinate.hpp |
| Template complexity | Pass | No complex templates; straightforward class hierarchies |
| Memory strategy | Pass | Stack-based vectors for faces/vertices; estimated <50KB worst case; no custom allocators needed |
| Thread safety | Pass | EPA is not thread-safe (mutable state during expansion) but documented; CollisionHandler is stateless after construction |
| Build integration | Pass | Straightforward CMakeLists.txt additions; no new external dependencies |

**Notes**: The design's approach of duplicating `supportMinkowski()` in EPA rather than extracting to a shared utility is a reasonable trade-off. While it creates ~30 lines of code duplication, it maintains self-containment and avoids introducing new coupling. This can be refactored in the future if needed.

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | Pass | EPA can be tested in isolation with mock simplex data; CollisionHandler can be tested with known hull configurations |
| Mockable dependencies | Pass | EPA takes `const AssetPhysical&` which can be constructed with test hulls and identity frames (see GJKTest.cpp patterns) |
| Observable state | Pass | `CollisionResult` fields are directly inspectable; penetration depth and normal can be validated against analytical solutions |

**Notes**: The test plan in the design document is comprehensive. The existing `GJKTest.cpp` provides excellent patterns for constructing test hulls and frames. Test cases should validate known analytical solutions (axis-aligned cubes with known penetration depths).

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | EPA may fail to converge for degenerate simplices (coplanar faces, near-zero normals) | Technical | Medium | Medium | Robust epsilon handling in `isVisible()` and face normal computation; fallback for max iterations | Yes |
| R2 | Horizon edge construction may corrupt polytope topology under numerical precision issues | Technical | Medium | High | Careful edge comparison with epsilon tolerance; unit tests with edge cases | Yes |
| R3 | Contact point (barycentric centroid) may be inaccurate for shallow penetrations | Technical | Low | Low | Acceptable for initial implementation per design decision; future refinement possible | No |
| R4 | `supportMinkowski` direction transform may differ subtly from GJK implementation | Integration | Low | Medium | Ensure identical transformation logic; integration tests comparing GJK+EPA pipeline | No |

### Prototype Guidance

#### Prototype P1: EPA Convergence Validation

**Risk addressed**: R1
**Question to answer**: Does EPA reliably converge within 64 iterations for typical convex hulls (10-50 vertices) across diverse penetration scenarios?

**Success criteria**:
- 95%+ success rate across 100+ test configurations
- Average iteration count < 32 for unit cubes
- No infinite loops or NaN outputs
- Handles shallow penetrations (depth < 1e-4) without failure

**Prototype approach**:
```
Location: prototypes/0027a_expanding_polytope_algorithm/p1_convergence/
Type: Standalone Catch2 test harness

Steps:
1. Create test hulls: unit cube, elongated box (10x1x1), regular tetrahedron, sphere approximation (20 vertices)
2. Generate configurations: axis-aligned overlap, angled overlap (15/30/45 degrees), deep penetration (50%), shallow penetration (1%)
3. Run EPA on each configuration, record iteration count and success/failure
4. Validate penetration depth against expected values (unit cube: known analytical solutions)
5. Log failure cases for analysis
```

**Time box**: 2 hours

**If prototype fails**:
- Investigate failure patterns (which hull shapes/orientations fail?)
- Consider adaptive epsilon based on hull scale
- Review face normal computation for degenerate cases

#### Prototype P2: Horizon Edge Construction Robustness

**Risk addressed**: R2
**Question to answer**: Does the horizon edge detection and visible face removal maintain valid polytope topology under numerical edge cases?

**Success criteria**:
- No duplicate edges in horizon after construction
- All new faces have valid normals (non-zero, outward-facing)
- Polytope remains closed (watertight) after each expansion
- No topology corruption for 100+ expansion iterations

**Prototype approach**:
```
Location: prototypes/0027a_expanding_polytope_algorithm/p2_horizon/
Type: Standalone Catch2 test harness with topology validation

Steps:
1. Create initial tetrahedron simplex
2. Implement topology validation: verify each edge shared by exactly 2 faces, all normals outward
3. Run expansion loop with topology check after each iteration
4. Test edge cases: very shallow penetration (depth < epsilon), near-coplanar faces, high vertex count (100+)
5. Verify EPAEdge::operator== handles edge ordering correctly
6. Test with known problematic configurations from literature
```

**Time box**: 2 hours

**If prototype fails**:
- Add explicit duplicate edge detection in `buildHorizonEdges()`
- Consider using std::set with custom comparator for edges
- Add face validity check before adding to polytope

### Required Revisions

None required. Design passes all criteria.

### Minor Suggestions (Non-blocking)

1. **CollisionResult default initialization**: Consider using `std::numeric_limits<double>::quiet_NaN()` for `penetrationDepth` in the default constructor per CLAUDE.md coding standards (Section: Uninitialized Member Variables).

2. **GJK simplex accessor documentation**: When adding `getSimplex()` to GJK, document the precondition that it should only be called after `intersects()` returns `true`. The design mentions this but ensure it appears in the Doxygen comment.

3. **EPA::supportMinkowski parameter type**: The design shows `Coordinate` as the parameter type, but GJK uses `CoordinateRate` for direction vectors. Consider consistency:
   ```cpp
   // Current GJK signature:
   Coordinate supportMinkowski(const CoordinateRate& dir) const;

   // EPA should match:
   Coordinate supportMinkowski(const CoordinateRate& dir) const;
   ```
   This maintains semantic consistency (directions use `CoordinateRate`, positions use `Coordinate`).

### Summary

The EPA design is well-structured, follows project conventions, and integrates cleanly with the existing GJK implementation. The CollisionHandler abstraction addresses the human feedback appropriately, providing a clean orchestration layer for collision detection. The removal of the redundant `intersecting` boolean from CollisionResult and use of `std::optional` is the correct approach.

Two prototypes are recommended to validate EPA convergence behavior and horizon edge construction robustness before implementation. These address the highest-risk technical uncertainties in the algorithm.

**Total estimated prototype time**: 4 hours

**Next steps**:
1. Human gate review of this assessment
2. Upon approval, proceed to prototype phase (P1 and P2)
3. Document prototype results in `docs/designs/0027a_expanding_polytope_algorithm/prototype-results.md`
4. Proceed to implementation upon successful prototype validation
