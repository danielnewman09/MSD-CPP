# Design: Edge Contact Manifold

## Summary

Extend the contact manifold generation in `EPA::extractContactManifold()` to detect edge-edge contact cases and generate 2 contact points with geometric extent along the contact segment. Currently, edge-edge contacts produce a degenerate single-point manifold where the contact point lies on the contact normal axis, yielding `r x n = 0` and preventing torque generation. This design adds edge detection and closest-edge queries to produce physically correct contact manifolds for edge impacts.

## Architecture Changes

### PlantUML Diagram

See: `./0040c-edge-contact-manifold.puml`

### Modified Components

#### EPA::extractContactManifold()

- **Current location**: `msd/msd-sim/src/Physics/Collision/EPA.cpp`
- **Changes required**:
  1. After the existing degenerate-case check (`refVerts.size() < 3 || incidentPoly.size() < 3`), instead of immediately falling back to the single EPA centroid point, attempt edge-edge contact generation
  2. Add private method `generateEdgeContacts()` to EPA class
- **Backward compatibility**: No public API changes. The method signature and return type remain the same. Edge detection is an internal enhancement to the existing fallback path.

**Current behavior** (lines 497-502 of EPA.cpp):
```cpp
// Handle degenerate cases
if (refVerts.size() < 3 || incidentPoly.size() < 3)
{
  Coordinate const contactPoint = epaFace.normal * epaFace.offset;
  contacts[0] = ContactPoint{contactPoint, contactPoint};
  return 1;
}
```

**New behavior**:
```cpp
// Handle degenerate cases - attempt edge contact generation first
if (refVerts.size() < 3 || incidentPoly.size() < 3)
{
  size_t const edgeContactCount =
    generateEdgeContacts(epaFace, contacts);
  if (edgeContactCount >= 2)
  {
    return edgeContactCount;
  }

  // Fallback to single EPA centroid point
  Coordinate const contactPoint = epaFace.normal * epaFace.offset;
  contacts[0] = ContactPoint{contactPoint, contactPoint};
  return 1;
}
```

**Design Rationale**:
- **Minimal insertion point**: The edge detection hooks into the existing degenerate-case branch, which is exactly where edge-edge contacts trigger the single-point fallback
- **Safe fallback**: If edge detection fails for any reason (degenerate geometry, parallel edges, etc.), the existing single-point fallback remains active
- **No impact on face-face contacts**: The edge path only activates when clipping produces < 3 points, which never happens for face-face contacts

#### EPA::generateEdgeContacts() (new private method)

- **Location**: `msd/msd-sim/src/Physics/Collision/EPA.cpp` (implementation), `EPA.hpp` (declaration)
- **Purpose**: Detect edge-edge contact case and generate 2 contact points along the contact segment
- **Signature**:
  ```cpp
  size_t generateEdgeContacts(const Facet& epaFace,
                              std::array<ContactPoint, 4>& contacts) const;
  ```
- **Returns**: Number of contact points generated (2 on success, 0 on failure)

**Algorithm**:

1. **Compute EPA witness points**: Use barycentric interpolation on the closest EPA face to extract witnessA and witnessB (the closest points on each hull's surface in world space).

2. **Transform witnesses to local space**: Convert world-space witness points to each hull's local frame using `getReferenceFrame().globalToLocal()`.

3. **Find closest edge on each hull**: Call `ConvexHull::findClosestEdge()` on each hull with the local-space witness point.

4. **Transform edge endpoints to world space**: Convert edge endpoints back to world space using `getReferenceFrame().localToGlobal()`.

5. **Compute closest segment between edges**: Find the closest pair of points between the two edge line segments using the standard segment-segment closest point algorithm.

6. **Generate 2 contact points**: Distribute contact points at the endpoints of the overlap region on the reference edge. Each contact point stores the point on hull A's surface and the corresponding projected point on hull B's surface.

7. **Validate**: Check that the 2 contact points have sufficient geometric separation (> epsilon). If separation is too small, return 0 to trigger fallback.

**Implementation details**:
```cpp
size_t EPA::generateEdgeContacts(
  const Facet& epaFace,
  std::array<ContactPoint, 4>& contacts) const
{
  const auto& normal = epaFace.normal;

  // 1. Compute witness points from EPA face via barycentric interpolation
  const auto& v0 = vertices_[epaFace.vertexIndices[0]];
  const auto& v1 = vertices_[epaFace.vertexIndices[1]];
  const auto& v2 = vertices_[epaFace.vertexIndices[2]];
  Coordinate const witnessA = (v0.witnessA + v1.witnessA + v2.witnessA) / 3.0;
  Coordinate const witnessB = (v0.witnessB + v1.witnessB + v2.witnessB) / 3.0;

  // 2. Transform witness points to local space for edge query
  const auto& frameA = assetA_.getReferenceFrame();
  const auto& frameB = assetB_.getReferenceFrame();
  Coordinate const localWitnessA = frameA.globalToLocal(witnessA);
  Coordinate const localWitnessB = frameB.globalToLocal(witnessB);

  // 3. Find closest edge on each hull
  auto edgeA = assetA_.getCollisionHull().findClosestEdge(localWitnessA);
  auto edgeB = assetB_.getCollisionHull().findClosestEdge(localWitnessB);

  // 4. Transform edge endpoints to world space
  Coordinate const edgeA_start = frameA.localToGlobal(edgeA.start);
  Coordinate const edgeA_end = frameA.localToGlobal(edgeA.end);
  Coordinate const edgeB_start = frameB.localToGlobal(edgeB.start);
  Coordinate const edgeB_end = frameB.localToGlobal(edgeB.end);

  // 5. Compute closest points between two line segments
  auto [closestOnA, closestOnB] =
    closestPointsBetweenSegments(edgeA_start, edgeA_end,
                                 edgeB_start, edgeB_end);

  // 6. Generate 2 contact points
  // Use the edge endpoints projected onto the contact plane
  // Point 1: closest point between the edges
  Coordinate const midpoint1 = (closestOnA + closestOnB) * 0.5;

  // Point 2: offset along the edge direction to provide geometric extent
  // Use the longer edge's direction to determine offset
  Coordinate const edgeDirA = (edgeA_end - edgeA_start);
  Coordinate const edgeDirB = (edgeB_end - edgeB_start);
  double const edgeLenA = edgeDirA.norm();
  double const edgeLenB = edgeDirB.norm();

  // Choose the shorter edge length for offset (conservative)
  double const halfExtent = std::min(edgeLenA, edgeLenB) * 0.5;

  // Edge direction perpendicular to normal (in the contact plane)
  msd_sim::Vector3D const normalVec{normal.x(), normal.y(), normal.z()};
  Coordinate const edgeDir = (edgeLenA >= edgeLenB)
                               ? edgeDirA.normalized()
                               : edgeDirB.normalized();

  // Project edge direction onto contact plane
  Coordinate const edgeDirInPlane =
    (edgeDir - normal * normal.dot(edgeDir)).normalized();

  // 7. Validate geometric extent
  if (halfExtent < epsilon_)
  {
    return 0;  // Degenerate edge, fall back to single point
  }

  // Generate two contact points offset from the closest point
  Coordinate const offset = edgeDirInPlane * halfExtent;

  Coordinate const cp1_mid = midpoint1 + offset;
  Coordinate const cp2_mid = midpoint1 - offset;

  // Project each contact point onto both surfaces along the normal
  // pointA = point projected onto hull A's edge
  // pointB = point projected onto hull B's edge
  double const depth = epaFace.offset;

  contacts[0] = ContactPoint{
    cp1_mid + normal * (depth * 0.5),
    cp1_mid - normal * (depth * 0.5)};
  contacts[1] = ContactPoint{
    cp2_mid + normal * (depth * 0.5),
    cp2_mid - normal * (depth * 0.5)};

  return 2;
}
```

**Error handling**:
- Returns 0 if edge length < epsilon (degenerate edge)
- Returns 0 if closest-point computation produces NaN
- Caller falls back to single-point on return of 0

**Thread safety**: Const method reading immutable EPA state and const hull references. Thread-safe.

#### closestPointsBetweenSegments() (new free function)

- **Location**: `msd/msd-sim/src/Physics/Collision/EPA.cpp` (anonymous namespace)
- **Purpose**: Compute the closest pair of points between two line segments in 3D
- **Signature**:
  ```cpp
  std::pair<Coordinate, Coordinate> closestPointsBetweenSegments(
    const Coordinate& p1, const Coordinate& q1,
    const Coordinate& p2, const Coordinate& q2);
  ```

**Algorithm**: Standard segment-segment closest point algorithm (Ericson, "Real-Time Collision Detection", Section 5.1.9):

1. Parameterize segments: S1(s) = p1 + s*(q1-p1), S2(t) = p2 + t*(q2-p2) where s,t in [0,1]
2. Compute dot products: d1 = q1-p1, d2 = q2-p2, r = p1-p2
3. Solve 2x2 system for unclamped (s,t)
4. Clamp s and t to [0,1], re-solving dependent parameter after each clamp
5. Return S1(s) and S2(t)

```cpp
std::pair<Coordinate, Coordinate> closestPointsBetweenSegments(
  const Coordinate& p1, const Coordinate& q1,
  const Coordinate& p2, const Coordinate& q2)
{
  Coordinate const d1 = q1 - p1;  // Direction of segment 1
  Coordinate const d2 = q2 - p2;  // Direction of segment 2
  Coordinate const r = p1 - p2;

  double const a = d1.dot(d1);  // Squared length of segment 1
  double const e = d2.dot(d2);  // Squared length of segment 2
  double const f = d2.dot(r);

  double s = 0.0;
  double t = 0.0;

  constexpr double kEpsilon = 1e-10;

  if (a <= kEpsilon && e <= kEpsilon)
  {
    // Both segments degenerate to points
    return {p1, p2};
  }

  if (a <= kEpsilon)
  {
    // Segment 1 degenerates to a point
    s = 0.0;
    t = std::clamp(f / e, 0.0, 1.0);
  }
  else
  {
    double const c = d1.dot(r);
    if (e <= kEpsilon)
    {
      // Segment 2 degenerates to a point
      t = 0.0;
      s = std::clamp(-c / a, 0.0, 1.0);
    }
    else
    {
      // General non-degenerate case
      double const b = d1.dot(d2);
      double const denom = a * e - b * b;  // Always >= 0

      // If segments not parallel, compute closest point on line 1 to line 2
      if (std::abs(denom) > kEpsilon)
      {
        s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
      }
      else
      {
        s = 0.0;  // Parallel segments: pick arbitrary s
      }

      // Compute t from s
      t = (b * s + f) / e;

      // Clamp t and recompute s if needed
      if (t < 0.0)
      {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      }
      else if (t > 1.0)
      {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }

  Coordinate const closest1 = p1 + d1 * s;
  Coordinate const closest2 = p2 + d2 * t;
  return {closest1, closest2};
}
```

**Design Rationale**:
- Standard well-known algorithm from game physics literature
- Handles all degenerate cases (point-point, point-segment, parallel segments)
- No heap allocations, pure computation
- Placed in anonymous namespace since it is only needed by EPA

#### ConvexHull::findClosestEdge() (new public method)

- **Location**: Declaration in `msd/msd-sim/src/Physics/RigidBody/ConvexHull.hpp`, implementation in `ConvexHull.cpp`
- **Purpose**: Find the edge of the convex hull closest to a given point in hull-local space
- **Signature**:
  ```cpp
  struct Edge {
    Coordinate start;
    Coordinate end;
  };

  Edge findClosestEdge(const Coordinate& point) const;
  ```

**Algorithm**:

1. Iterate all facets of the hull
2. For each facet, extract its 3 edges (vertex pairs from `vertexIndices`)
3. Track unique edges using an ordered pair representation to avoid duplicates (each edge shared by 2 triangular facets)
4. For each unique edge, compute the minimum distance from the query point to the edge segment
5. Return the edge with minimum distance

**Implementation details**:
```cpp
ConvexHull::Edge ConvexHull::findClosestEdge(const Coordinate& point) const
{
  double minDist = std::numeric_limits<double>::infinity();
  Edge closestEdge{};

  // Use a set to track processed edges (unordered pair)
  std::set<std::pair<size_t, size_t>> processedEdges;

  for (const auto& facet : facets_)
  {
    for (size_t i = 0; i < 3; ++i)
    {
      size_t const idx0 = facet.vertexIndices[i];
      size_t const idx1 = facet.vertexIndices[(i + 1) % 3];

      // Canonical edge representation (smaller index first)
      auto edgeKey = std::make_pair(std::min(idx0, idx1),
                                     std::max(idx0, idx1));

      if (processedEdges.contains(edgeKey))
      {
        continue;
      }
      processedEdges.insert(edgeKey);

      const Coordinate& v0 = vertices_[idx0];
      const Coordinate& v1 = vertices_[idx1];

      // Compute distance from point to edge segment
      double const dist = pointToSegmentDistance(point, v0, v1);

      if (dist < minDist)
      {
        minDist = dist;
        closestEdge = Edge{v0, v1};
      }
    }
  }

  return closestEdge;
}
```

**Helper function** (private or in anonymous namespace):
```cpp
double pointToSegmentDistance(const Coordinate& point,
                              const Coordinate& segStart,
                              const Coordinate& segEnd)
{
  Coordinate const seg = segEnd - segStart;
  double const segLenSq = seg.dot(seg);

  if (segLenSq < 1e-10)
  {
    return (point - segStart).norm();
  }

  double const t = std::clamp(seg.dot(point - segStart) / segLenSq, 0.0, 1.0);
  Coordinate const projection = segStart + seg * t;
  return (point - projection).norm();
}
```

**Performance**:
- O(E) where E = number of unique edges in the hull
- For a cube (12 edges), this is trivial (~12 distance computations)
- For typical simulation hulls (< 100 edges), performance is negligible
- Called at most twice per collision (once per hull), only in the edge-edge fallback path

**Memory**:
- `Edge` struct: 48 bytes (2 Coordinates)
- `std::set` for deduplication: temporary, freed after method returns
- No heap allocations in hot path (Edge returned by value)

**Thread safety**: Const method on immutable hull data. Thread-safe.

### New Types

#### ConvexHull::Edge

- **Location**: Nested struct in `ConvexHull` class declaration (`ConvexHull.hpp`)
- **Purpose**: Represent an edge of the convex hull as a pair of vertex coordinates

```cpp
struct Edge
{
  Coordinate start;
  Coordinate end;
};
```

**Design Rationale**:
- **Coordinate values, not indices**: The edge is returned with actual vertex coordinates, not indices, because callers need world-space coordinates after transformation. Returning indices would require callers to access the hull's vertex array and transform themselves.
- **Nested struct**: Scoped inside ConvexHull since it is only meaningful in that context
- **Value type**: Small, copyable, no resource management
- **No constructor**: Aggregate initialization via brace initialization is sufficient

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| `EPA::generateEdgeContacts()` | `EPA::extractContactManifold()` | Called from degenerate-case branch | Hooks into existing fallback path |
| `ConvexHull::findClosestEdge()` | `ConvexHull` | New public method | Iterates existing facets/vertices |
| `closestPointsBetweenSegments()` | None | New utility in anonymous namespace | Standard geometry algorithm |
| `pointToSegmentDistance()` | None | New utility in anonymous namespace | Helper for findClosestEdge |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `test/Physics/Collision/*Test.cpp` | All existing collision tests | None | Verify no regression (edge path only activates for degenerate manifolds) |
| `test/Physics/Collision/LinearCollisionTest.cpp` | Face-face A1-A6, F1-F5 | None | These produce 3+ clipped points, never enter edge path |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| `ConvexHull` | `FindClosestEdge_CubeVertex_ReturnsAdjacentEdge` | findClosestEdge returns edge adjacent to the closest vertex on a unit cube |
| `ConvexHull` | `FindClosestEdge_CubeEdgeMidpoint_ReturnsThatEdge` | Query point on an edge midpoint returns that edge |
| `ConvexHull` | `FindClosestEdge_CubeFaceCenter_ReturnsNearestEdge` | Query point at face center returns one of the 4 face edges |
| `ConvexHull` | `FindClosestEdge_TetrahedronVertex_ReturnsAdjacentEdge` | Works correctly for non-cube geometry |
| `closestPointsBetweenSegments` | `ParallelSegments_ReturnsClosestEndpoints` | Parallel segments return correct closest pair |
| `closestPointsBetweenSegments` | `PerpendicularCrossing_ReturnsCrossPoint` | Perpendicular crossing segments return intersection point |
| `closestPointsBetweenSegments` | `SkewSegments_ReturnsClosestPair` | Non-intersecting non-parallel segments return correct closest pair |
| `closestPointsBetweenSegments` | `DegeneratePoint_ReturnsPointToSegment` | Zero-length segment returns point-to-segment distance |

#### Edge Contact Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| `EdgeContact` | `CubeEdgeOnFloor_DetectedAsEdgeContact` | Cube rotated 45 degrees about one axis so edge contacts floor, detected as edge contact |
| `EdgeContact` | `TwoContactPoints_Generated` | Edge-edge contact produces exactly 2 contact points |
| `EdgeContact` | `ContactPoints_HaveGeometricExtent` | Distance between 2 contact points > 0 |
| `EdgeContact` | `LeverArm_CrossNormal_NonZero` | `r x n != 0` for at least one contact point |
| `EdgeContact` | `FallbackToSinglePoint_WhenEdgeDetectionFails` | Degenerate edge cases still produce valid single-point contact |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `CubeEdgeImpact_InitiatesRotation` | WorldModel, EPA, ContactConstraint | B2 scenario: cube with edge parallel to floor, dropped, rotation initiated |
| `EdgeImpact_EnergyConserved` | WorldModel, EPA, ContactConstraint | Total energy (linear + rotational) <= initial energy after edge impact |

#### Regression Tests

- All existing `LinearCollisionTest` tests (A1-A6, F1-F5) must pass
- All existing `RotationalCollisionTest` tests must pass
- All existing `ParameterIsolationTest` tests must pass

### New Test File

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/Collision/EdgeContactTest.cpp` | Unit and integration tests for edge contact manifold |

### CMakeLists.txt Changes

Add `EdgeContactTest.cpp` to `msd-sim/test/Physics/Collision/CMakeLists.txt`:
```cmake
target_sources(msd_sim_test PRIVATE EdgeContactTest.cpp)
```

## Open Questions

### Resolved During Design

1. **How to identify the contacting edges?**
   - **Decision**: Use the EPA witness points (barycentric centroid of closest face) as query points for `ConvexHull::findClosestEdge()`. The witness points represent the closest surface points on each hull, so the nearest edge to each witness point is the contacting edge.

2. **How many contact points for edge contacts?**
   - **Decision**: Generate exactly 2 contact points. This provides geometric extent (non-zero `r x n` cross product) while remaining simpler than attempting to compute the full overlap region of two edge segments.

3. **How to compute per-contact depth?**
   - **Decision**: Use the EPA penetration depth uniformly for both contact points. Per-point depth computation via normal projection was considered but adds complexity without clear benefit for the edge-edge case where depth variation along the edge is typically small.

4. **Where to place the 2 contact points?**
   - **Decision**: Offset from the closest point along the edge direction in the contact plane by +/- half the shorter edge length. This ensures: (a) the points lie in the contact plane, (b) they have maximum geometric extent, and (c) they don't extend beyond either edge.

### No Prototype Required

**Rationale**:
- The closest-segment algorithm is well-established (Ericson, 2004)
- findClosestEdge is a simple brute-force search over hull edges
- The edge contact generation integrates into an existing, well-tested fallback path
- Risk is mitigated by the safe fallback to single-point contact

## Design Complexity Sanity Checks

### Red Flag Assessment

**No red flags detected.** This design:
- Adds **1 new public method** to ConvexHull (`findClosestEdge`)
- Adds **1 new private method** to EPA (`generateEdgeContacts`)
- Adds **2 utility functions** in anonymous namespace (`closestPointsBetweenSegments`, `pointToSegmentDistance`)
- Adds **1 nested struct** to ConvexHull (`Edge`)
- Modifies **1 code path** in EPA (the existing degenerate-case branch)
- **No changes to public API** of EPA or CollisionResult

### Complexity Analysis

**Maintained simplicity**:
- Edge detection hooks into existing fallback path (3 lines of new branching)
- Closest-edge query is O(E) brute force (no complex data structures)
- Segment-segment closest point is a standard algorithm (< 60 lines)
- Contact point generation is direct computation (no iteration or convergence)

**Trade-offs accepted**:
- `std::set` for edge deduplication in `findClosestEdge` — allocates, but called rarely (only in edge fallback path, typically 12-50 edges for simple hulls)
- Could optimize with a half-edge data structure in the future if edge queries become frequent

## Dependencies

### External Libraries
- **Eigen3** — Linear algebra (unchanged)
- **Qhull** — Convex hull computation (unchanged)

### Internal Dependencies
- `ConvexHull` — Extended with findClosestEdge
- `EPA` — Extended with generateEdgeContacts
- `Facet` — Existing hull facet type (read-only access)
- `Coordinate` — 3D vector type
- `ReferenceFrame` — Coordinate transformations
- `AssetPhysical` — Hull and frame access

### Build Impact
- **Modified files**:
  - `ConvexHull.hpp` — Add `Edge` struct, `findClosestEdge()` declaration
  - `ConvexHull.cpp` — Implement `findClosestEdge()`, `pointToSegmentDistance()`
  - `EPA.hpp` — Add `generateEdgeContacts()` declaration
  - `EPA.cpp` — Implement `generateEdgeContacts()`, `closestPointsBetweenSegments()`, modify `extractContactManifold()` fallback
- **New files**:
  - `test/Physics/Collision/EdgeContactTest.cpp` — Edge contact tests
- **CMakeLists.txt**: Add `EdgeContactTest.cpp` to test sources

## Performance Considerations

### Computational Complexity

**Per-collision overhead** (edge-edge case only):
- **findClosestEdge**: O(E) per hull, where E = number of unique edges. For a cube (E=12), ~12 distance computations. Negligible.
- **closestPointsBetweenSegments**: O(1) — Fixed computation, ~30 FLOPs
- **generateEdgeContacts**: O(1) — Fixed computation after edge queries
- **Total overhead**: ~50 FLOPs + 2 x O(E) edge queries (only in fallback path)

**Non-edge cases**: Zero additional overhead. The edge path only activates when both reference and incident polygons have < 3 vertices.

### Memory Impact

- `Edge` struct: 48 bytes (stack-allocated)
- `std::set<pair<size_t,size_t>>` for deduplication: ~50 entries typical, temporary
- No persistent memory increase

## Code Quality Notes

### Adherence to Coding Standards

- **Brace initialization**: `Edge{v0, v1}`, `ContactPoint{pointA, pointB}`, `Coordinate{x, y, z}`
- **NaN for uninitialized floats**: Not applicable (no new member variables with default initialization)
- **References for non-owning access**: `const ConvexHull&`, `const ReferenceFrame&`
- **Return values over output parameters**: `findClosestEdge()` returns `Edge` by value, `closestPointsBetweenSegments` returns `std::pair`
- **Rule of Zero**: `Edge` struct uses aggregate initialization (no custom constructors needed)
- **Const correctness**: `findClosestEdge() const`, `generateEdgeContacts() const`

### Critical Implementation Note: ReferenceFrame Overload

When transforming EPA normals or edge directions from world to local space, care must be taken with `ReferenceFrame::globalToLocal()`:
- `globalToLocal(Coordinate)` applies rotation + translation (for points)
- `globalToLocal(Vector3D)` applies rotation only (for directions)

Since `Coordinate` inherits from `Vector3D`, passing a `Coordinate` will pick the `Coordinate` overload and incorrectly apply translation. When transforming direction vectors (normals, edge directions), always construct an explicit `Vector3D`:
```cpp
// CORRECT for directions
Vector3D v{coord.x(), coord.y(), coord.z()};
frame.globalToLocal(v);

// INCORRECT for directions (applies translation)
frame.globalToLocal(coord);  // Picks Coordinate overload!
```

This bug was identified in ticket 0039e and is documented in the project memory.

## Acceptance Criteria Mapping

| AC | Requirement | Design Component |
|----|-------------|------------------|
| AC1 | Edge-edge contact case detected when clipping produces < 3 points | Modified `extractContactManifold()` fallback branch calls `generateEdgeContacts()` |
| AC2 | 2 contact points generated with geometric extent along edge | `generateEdgeContacts()` returns 2 `ContactPoint` entries offset along edge direction |
| AC3 | `r x n != 0` for edge contacts offset from COM | Contact points offset perpendicular to normal in contact plane, ensuring non-zero cross product |
| AC4 | B2_CubeEdgeImpact test passes (rotation initiated) | Integration test: `CubeEdgeImpact_InitiatesRotation` |
| AC5 | Fallback to single-point when edge detection fails | `generateEdgeContacts()` returns 0 on failure, existing fallback activates |
| AC6 | No regression in face-face contact scenarios | Edge path only activates for < 3 clipped points; face-face contacts always produce >= 3 |

---

## Summary

This design provides a targeted extension to the existing contact manifold generation to handle edge-edge contacts, which currently produce degenerate single-point manifolds with zero angular Jacobian. The approach:

1. **Detects** edge-edge contacts by checking when clipping produces < 3 points (existing degenerate path)
2. **Queries** the closest edge on each hull using a new `ConvexHull::findClosestEdge()` method
3. **Generates** 2 contact points with geometric extent along the contact segment
4. **Falls back** to the existing single-point behavior if edge detection fails

The design is minimal (4 new functions, 1 new struct, 1 modified code path), safe (fallback always available), and targeted (only activates for the specific degenerate case). No public API changes are required.

Ready for implementation phase.
