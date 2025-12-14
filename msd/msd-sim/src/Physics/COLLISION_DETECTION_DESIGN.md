# Collision Detection and Contact Resolution Design

## Overview

This document outlines the architecture for efficient collision detection and contact resolution in the MSD physics simulation. The design prioritizes computational efficiency while providing the necessary information for accurate physics response.

## Core Challenge

**Problem**: `msd_assets::Geometry` objects are position-agnostic (defined in local coordinates). When computing collisions between `PhysicalAsset` objects, we need to ensure their convex hulls are compared in a common reference frame without unnecessary computational overhead.

**Solution**: Store convex hulls in local coordinates and use transform-on-query with GJK/EPA algorithms, combined with contact manifold caching for persistent contacts.

---

## Architecture

### Phase 1: Broad Phase (AABB Culling)

**Purpose**: Quickly eliminate pairs that cannot possibly collide.

**Implementation**:
- Maintain cached world-space Axis-Aligned Bounding Box (AABB) for each `PhysicalAsset`
- Update AABB lazily when reference frame changes (8 corner transforms)
- Use spatial partitioning (BVH/Octree) for O(N log N) complexity instead of O(N²)

**Cost**: ~200 FLOPs per object per frame

```cpp
// In PhysicalAsset.hpp
mutable BoundingBox worldAABB_;  // Cached world-space AABB
mutable bool aabbDirty_;         // Lazy update flag

const BoundingBox& getWorldBoundingBox() const;
void invalidateAABB();  // Called when reference frame changes
```

---

### Phase 2: Narrow Phase (GJK Collision Detection)

**Purpose**: Accurate boolean collision test using Gilbert-Johnson-Keerthi algorithm.

**Key Insight**: GJK uses a **support function** rather than explicit hull vertices, allowing us to incorporate transformations at query time with minimal cost.

#### Support Function

The support function finds the vertex furthest in a given direction:

```cpp
// In ConvexHull.hpp

/**
 * @brief Compute support point in a given direction (local frame).
 *
 * The support point is the vertex furthest in the given direction.
 * This is the core primitive for GJK collision detection.
 *
 * @param direction Direction vector (in local frame, need not be normalized)
 * @return Support point coordinate (in local frame)
 */
Coordinate support(const Eigen::Vector3d& direction) const;

/**
 * @brief Compute transformed support point in world space.
 *
 * Applies the reference frame transformation to compute the support point
 * in world coordinates. Used by GJK when testing collision between objects
 * in different reference frames.
 *
 * Support_world(d) = frame.localToGlobal(support(frame.globalToLocal(d)))
 *
 * @param direction Direction vector in world frame
 * @param frame Reference frame transformation
 * @return Support point in world frame
 */
Coordinate supportWorld(const Eigen::Vector3d& direction,
                       const ReferenceFrame& frame) const;
```

#### GJK Interface

```cpp
// In GJK.hpp

/**
 * @brief Fast collision detection using GJK algorithm.
 *
 * Uses support functions with reference frame transformations to detect
 * collision between two convex hulls without explicitly transforming vertices.
 *
 * @param hullA First convex hull (in local coordinates)
 * @param frameA Reference frame for first hull
 * @param hullB Second convex hull (in local coordinates)
 * @param frameB Reference frame for second hull
 * @param epsilon Numerical tolerance for termination
 * @return true if hulls intersect, false otherwise
 */
bool intersects(const ConvexHull& hullA, const ReferenceFrame& frameA,
                const ConvexHull& hullB, const ReferenceFrame& frameB,
                double epsilon = 1e-6) const;
```

**Cost**: ~500-2000 FLOPs per collision pair (AABB overlaps only)

---

### Phase 3: Contact Resolution (EPA + Manifold Caching)

**Problem**: GJK only provides boolean collision result. For physics response, we need:
- Contact normal vector
- Penetration depth
- Contact point(s)

**Solution**: Use Expanding Polytope Algorithm (EPA) to extract contact information from GJK's final simplex.

#### Collision Result Structure

```cpp
// In GJK.hpp or new CollisionDetection.hpp

struct CollisionResult
{
  bool intersecting;                     // Are objects colliding?
  Coordinate normal;                     // Collision normal (world frame, from A to B)
  double penetrationDepth;               // How deep is the overlap? [m]
  std::vector<Coordinate> contactPoints; // Contact point(s) in world frame

  // Internal data for EPA
  std::vector<Coordinate> simplexVertices;  // Final simplex from GJK
};

/**
 * @brief Detect collision and compute contact information.
 *
 * Uses GJK for fast detection, then EPA (Expanding Polytope Algorithm) to
 * compute contact normal, penetration depth, and contact points if collision
 * is detected.
 *
 * @param hullA First convex hull (in local frame)
 * @param frameA Reference frame for first hull
 * @param hullB Second convex hull (in local frame)
 * @param frameB Reference frame for second hull
 * @param epsilon Numerical tolerance
 * @return CollisionResult with contact information
 */
CollisionResult detectCollision(
  const ConvexHull& hullA, const ReferenceFrame& frameA,
  const ConvexHull& hullB, const ReferenceFrame& frameB,
  double epsilon = 1e-6
) const;
```

**Cost**: ~2000-5000 FLOPs for EPA (only on first frame of contact)

---

### Phase 4: Contact Manifold Caching

**Key Optimization**: Objects in persistent contact (e.g., box resting on ground) only need EPA once. Subsequent frames can update the cached contact manifold cheaply.

#### Contact Point Structure

```cpp
// New file: ContactManifold.hpp

struct ContactPoint
{
  Coordinate pointA;        // Contact point on object A (world frame)
  Coordinate pointB;        // Contact point on object B (world frame)
  Coordinate normal;        // Contact normal (from A to B, world frame)
  double penetration;       // Penetration depth [m]

  // Material properties (can be looked up from assets)
  double restitution;       // Coefficient of restitution [0, 1]
  double friction;          // Friction coefficient (μ)

  // Cached for warm starting
  double normalImpulse;     // Accumulated normal impulse [N⋅s]
  double tangentImpulse[2]; // Accumulated tangent impulses [N⋅s]
};

class ContactManifold
{
public:
  /**
   * @brief Add a new contact point to the manifold.
   */
  void addContact(const ContactPoint& contact);

  /**
   * @brief Update contact points based on new reference frames.
   *
   * Transforms cached local-space contact points to world space using
   * updated reference frames. Much cheaper than re-running GJK+EPA.
   *
   * @param frameA Current reference frame for object A
   * @param frameB Current reference frame for object B
   */
  void update(const ReferenceFrame& frameA, const ReferenceFrame& frameB);

  /**
   * @brief Check if cached manifold is still valid.
   *
   * A manifold becomes invalid if contact points drift too far apart
   * or if penetration depth changes significantly.
   *
   * @param maxDistance Maximum allowed drift [m] (default: 0.01)
   * @return true if manifold can be reused, false if GJK+EPA needed
   */
  bool isValid(double maxDistance = 0.01) const;

  /**
   * @brief Clear all contact points.
   */
  void clear();

  // Accessors
  const std::vector<ContactPoint>& getContacts() const;
  size_t getContactCount() const;
  Coordinate getAverageNormal() const;
  double getAveragePenetration() const;

private:
  std::vector<ContactPoint> contacts_;
  size_t framesSinceUpdate_ = 0;

  // Store contact points in local coordinates for cheap updates
  std::vector<Coordinate> localPointsA_;
  std::vector<Coordinate> localPointsB_;
};
```

**Cost**: ~200 FLOPs per cached manifold update (vs ~2000-5000 for EPA)

---

### Collision Cache Manager

```cpp
// In CollisionDetection.hpp or PhysicsWorld.hpp

class CollisionCache
{
public:
  /**
   * @brief Get or compute collision information between two assets.
   *
   * Checks if a valid cached manifold exists. If so, updates it based on
   * current transforms. If not, runs GJK+EPA to compute fresh contact data.
   *
   * @param assetA First physical asset
   * @param assetB Second physical asset
   * @return Contact manifold (may be empty if no collision)
   */
  const ContactManifold& getOrComputeContacts(
    const PhysicalAsset& assetA,
    const PhysicalAsset& assetB
  );

  /**
   * @brief Invalidate cache entries that are no longer valid.
   *
   * Call this each frame to remove stale manifolds.
   */
  void prune();

  /**
   * @brief Clear all cached manifolds.
   */
  void clear();

private:
  // Use pointer pairs as keys (order-independent)
  using AssetPair = std::pair<const PhysicalAsset*, const PhysicalAsset*>;
  std::map<AssetPair, ContactManifold> cache_;

  AssetPair makePair(const PhysicalAsset* a, const PhysicalAsset* b) const;
};
```

---

## Complete Pipeline

### Frame N (First Contact)

```cpp
// 1. Broad phase: AABB test
if (!aabbOverlap(assetA.getWorldBoundingBox(),
                  assetB.getWorldBoundingBox())) {
  return;  // No possible collision
}

// 2. Check cache first
auto& manifold = collisionCache.get(assetA, assetB);
if (manifold.isValid()) {
  // 3a. Update cached manifold (cheap!)
  manifold.update(assetA.getReferenceFrame(),
                  assetB.getReferenceFrame());

  // 4a. Apply collision response using cached data
  resolveCollision(assetA, assetB, manifold);
  return;
}

// 3b. Cache miss - run full detection
auto result = gjk.detectCollision(
  assetA.getCollisionHull(), assetA.getReferenceFrame(),
  assetB.getCollisionHull(), assetB.getReferenceFrame()
);

if (result.intersecting) {
  // 4b. Create new manifold from collision result
  ContactManifold newManifold = createManifold(result);

  // 5b. Apply collision response
  resolveCollision(assetA, assetB, newManifold);

  // 6b. Cache for next frame
  collisionCache.store(assetA, assetB, newManifold);
}
```

### Frame N+1 (Persistent Contact)

Same pipeline, but step 3a (cached update) executes instead of 3b (full detection).

**Performance gain**: 10-25× faster for persistent contacts!

---

## Performance Characteristics

| Operation | Cost (FLOPs) | Frequency | Notes |
|-----------|--------------|-----------|-------|
| AABB update | ~200 | Per object per frame | 8 corner transforms |
| AABB overlap test | ~50 | All pairs (broad phase) | Can use BVH for O(N log N) |
| GJK detection | ~500-2000 | AABB overlaps only | ~10-30 support queries |
| EPA contact resolution | ~2000-5000 | First frame of contact | Polytope expansion |
| Cached manifold update | ~200 | Subsequent contact frames | Transform local points |

### Example Scene: 100 Objects

**Without caching:**
- Broad phase: 100 objects × 200 FLOPs = 20K FLOPs
- AABB tests: (100 choose 2) × 50 = 247K FLOPs
- GJK (assume 20 overlaps): 20 × 1500 = 30K FLOPs
- EPA (assume 10 collisions): 10 × 3500 = 35K FLOPs
- **Total: ~332K FLOPs**

**With caching (frame 2+, assume 8 persistent contacts):**
- Broad phase: 20K FLOPs
- AABB tests: 247K FLOPs
- GJK (2 new contacts): 2 × 1500 = 3K FLOPs
- EPA (2 new contacts): 2 × 3500 = 7K FLOPs
- Cached updates (8 contacts): 8 × 200 = 1.6K FLOPs
- **Total: ~279K FLOPs** (16% reduction)

For scenes with many persistent contacts (stacked boxes), savings can exceed 50%.

---

## Implementation Roadmap

### Phase 1: Foundation (Required for basic collision)
1. ✅ Add `support()` method to `ConvexHull`
2. ✅ Add `supportWorld()` method to `ConvexHull`
3. ✅ Update `GJK::intersects()` to accept `ReferenceFrame` parameters
4. ✅ Add `getWorldBoundingBox()` to `PhysicalAsset` with caching

### Phase 2: Contact Information (Required for physics response)
5. ✅ Implement EPA algorithm for contact normal/depth
6. ✅ Create `CollisionResult` structure
7. ✅ Add `GJK::detectCollision()` that returns `CollisionResult`
8. ✅ Create `ContactPoint` structure
9. ✅ Create `ContactManifold` class

### Phase 3: Optimization (Performance improvement)
10. ✅ Implement `CollisionCache` manager
11. ✅ Add persistent contact warm-starting
12. ✅ Implement spatial partitioning (BVH/Octree) for broad phase

### Phase 4: Integration
13. ✅ Integrate collision detection into physics timestep
14. ✅ Implement impulse-based collision response
15. ✅ Add material properties (restitution, friction) to assets

---

## Alternative: SAT (Separating Axis Theorem)

For **polyhedral convex hulls only**, SAT can compute contact information directly without EPA.

**Pros**:
- Gets normal + depth in one pass
- Simpler algorithm than EPA
- No need for simplex refinement

**Cons**:
- More expensive than GJK for many potential separating axes
- Requires testing face normals + edge-edge cross products
- Harder to optimize for general convex shapes

**Decision**: Stick with GJK+EPA for flexibility and optimization potential. Revisit SAT if profiling shows EPA as bottleneck.

---

## Open Questions

1. **Material Properties**: Where should restitution/friction be stored?
   - Option A: Add to `PhysicalAsset` base class
   - Option B: Add only to `InertialAsset`
   - Option C: Separate `MaterialProperties` class

2. **Multi-Contact Manifolds**: How many contact points per pair?
   - Typical: 4-8 points for box-box contacts
   - Need algorithm to select most important points

3. **Spatial Partitioning**: Which structure for broad phase?
   - BVH: Better for dynamic scenes
   - Octree: Better for mostly static scenes
   - Grid: Simplest, works for uniform distributions

4. **Sleeping/Island Detection**: Should stationary objects skip physics?
   - Huge performance win for stacked/resting objects
   - Requires velocity thresholds and contact graph analysis

---

## References

- **GJK Algorithm**: Gilbert, Johnson, Keerthi (1988)
- **EPA Algorithm**: van den Bergen (2001)
- **Contact Manifolds**: Ericson, "Real-Time Collision Detection" (2004)
- **Warm Starting**: Catto, "Iterative Dynamics with Temporal Coherence" (2005)

---

## Next Steps

When ready to implement, start with Phase 1:
1. Add support functions to `ConvexHull.hpp`
2. Update `GJK.hpp` to use reference frames
3. Add AABB caching to `PhysicalAsset.hpp`

Then integrate into a simple collision detection test before moving to EPA and contact resolution.
