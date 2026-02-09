# Collision Detection and Response System

> Architectural context for the collision detection and response subsystem.
> For API details, use MCP: `find_class CollisionHandler`, `get_class_members GJK`, etc.

## Architecture Overview

The collision system follows a three-phase pipeline:

```
Phase 1: Broad Phase (AABB culling) [NOT YET IMPLEMENTED]
    └── Eliminate non-overlapping pairs with bounding boxes

Phase 2: Narrow Phase (GJK collision detection)
    └── Boolean intersection test using Minkowski difference

Phase 3: Contact Resolution (EPA + ContactConstraint)
    ├── EPA: Extract penetration depth, normal, contact manifold
    └── Response: Create ContactConstraint and solve via ConstraintSolver
```

### Performance Characteristics

| Operation | Cost (FLOPs) | Frequency | Notes |
|-----------|--------------|-----------|-------|
| AABB overlap test | ~50 | All pairs (broad phase) | Can use BVH for O(N log N) |
| GJK detection | ~500-2000 | AABB overlaps only | ~10-30 support queries |
| EPA contact resolution | ~2000-5000 | First frame of contact | Polytope expansion |

---

## Algorithm Design Rationale

### GJK: Why Minkowski Difference?

Two convex shapes A and B intersect if and only if their Minkowski difference (A ⊖ B) contains the origin. This reduces collision detection to a point-containment test, which GJK solves iteratively by building a simplex that attempts to enclose the origin.

**Transformation handling**: Transformations are applied on-the-fly during support function computation rather than pre-transforming hulls. This avoids temporary allocations and achieves < 2% overhead vs identity transforms.

**Simplex progression**:
- Point → Line → Triangle → Tetrahedron
- Each step narrows the search region toward (or away from) the origin
- Typically converges in < 20 iterations

### EPA: Why Polytope Expansion?

When GJK confirms intersection, we need penetration depth and contact normal. EPA expands the GJK simplex into a polytope, iteratively adding support points until the closest face to the origin is found. This face's normal and distance give us the contact information.

**Witness point tracking** (Ticket 0028): EPA tracks which surface points on each object contributed to each Minkowski vertex. This enables accurate contact point extraction for torque calculation (τ = r × F).

**Contact manifold generation** (Ticket 0029): Up to 4 contact points per collision for improved stability in constraint solving. Fixed array size avoids heap allocation.

**Edge contact manifold** (Ticket 0040c): Edge-edge contacts (where Sutherland-Hodgman clipping produces < 3 points) generate 2 contact points with geometric extent along the contact edge segment, enabling torque generation from edge impacts. Uses `ConvexHull::findClosestEdge()` to identify contacting edges and a segment-segment closest point algorithm (Ericson 2004, Section 5.1.9) to compute contact point placement. Falls back to single-point contact if edge detection produces degenerate results.

**Graceful degradation** (Ticket 0048): When EPA reaches max iterations without converging, it returns the best available result (closest polytope face found so far) instead of throwing an exception. This handles near-degenerate geometry where identical micro-rotations cause EPA to stall. Matches standard practice in production physics engines.

### Why std::optional<CollisionResult>?

Collision is inherently optional—most object pairs don't collide. Using `std::optional` makes the API self-documenting and eliminates the need for a separate `intersecting` boolean field.

---

## Integration with Physics Pipeline

### CollisionPipeline Orchestration

Collision response is orchestrated by the `CollisionPipeline` class, which owns the collision detection and response workflow. `WorldModel::updateCollisions()` delegates to the pipeline rather than implementing collision logic inline.

**Diagram**: [`0044_collision_pipeline_integration.puml`](../../../../../../docs/designs/0044_collision_pipeline_integration/0044_collision_pipeline_integration.puml)

#### Pipeline Phases

The `CollisionPipeline` executes a multi-phase collision response workflow:

1. **Collision Detection**: O(n²) pairwise GJK/EPA checks
2. **Constraint Creation**: ContactConstraintFactory creates constraints from CollisionResult
3. **Solver Input Assembly**: Gather states, masses, inverse inertias for all bodies
4. **Warm-Start Query**: Query ContactCache for previous frame's lambda values (10-25× speedup for persistent contacts)
5. **Constraint Solving**: ConstraintSolver (Active Set Method) solves contact LCP with initial lambda
6. **Cache Update**: Store solved lambda values in ContactCache for next frame
7. **Force Application**: Constraint forces applied to bodies
8. **Position Correction**: PositionCorrector applies split-impulse correction to remove penetration without energy injection

#### CollisionPipeline Architecture

**Location**: `msd-sim/src/Physics/Collision/CollisionPipeline.hpp/.cpp`
**Introduced**: [Ticket: 0036_collision_pipeline_extraction](../../../../../../tickets/0036_collision_pipeline_extraction.md)
**Extended**: [Ticket: 0044_collision_pipeline_integration](../../../../../../tickets/0044_collision_pipeline_integration.md)

**Key components**:
- Owns `ContactCache` for warm-starting (frame-persistent lambda storage)
- Owns `PositionCorrector` for split-impulse position correction
- Owns `ConstraintSolver` for Active Set Method solving
- Owns `CollisionHandler` for GJK/EPA collision detection
- Exposes `hadCollisions()` for energy tracking diagnostics

**WorldModel integration**:
```cpp
void WorldModel::updateCollisions(double dt) {
  collisionPipeline_.advanceFrame();      // Cache lifecycle
  collisionPipeline_.expireOldEntries();  // Remove stale entries
  collisionPipeline_.execute(inertialAssets_, environmentalAssets_, dt);
  collisionActiveThisFrame_ = collisionPipeline_.hadCollisions();
}
```

**Key insight**: Contacts are solved as constraints, not as standalone impulses. This unifies collision response with the Lagrangian constraint framework (quaternion normalization, future joints).

---

## Historical Context: CollisionResponse Removal

**Removed**: Ticket 0032d (2026-01-31)

The original `CollisionResponse` namespace provided impulse-based collision response but was removed because:
1. Duplicated constraint solver logic
2. Inconsistent with Lagrangian mechanics formulation
3. Could not extend to friction or multi-contact stability
4. Made collision a special case rather than a general constraint

**Migration**: All functionality now in ContactConstraint + ConstraintSolver.

---

## Limitations and Future Work

### Current Limitations

1. **Frictionless contacts**: Normal force only, no tangential friction
2. **O(n²) broad phase**: Scales poorly beyond ~100 objects
3. **No CCD**: Fast objects can tunnel through thin geometry
4. **No sleeping**: All objects active every frame

### Planned Enhancements

| Enhancement | Priority | Status | Benefit |
|-------------|----------|--------|---------|
| Friction constraints | High | Not started | Realistic surface interaction |
| Broadphase (BVH/Grid) | Medium | Not started | O(n log n) scaling |
| Contact caching | Medium | ✅ Complete (0040b, 0044) | 10-25× faster for persistent contacts |
| Position correction | Medium | ✅ Complete (0040d, 0044) | Removes penetration without energy injection |
| CCD | Medium | Not started | Prevents tunneling |
| Sleeping/islands | Low | Not started | Skip stationary objects |

---

## Mathematical Reference

### Minkowski Difference

```
A ⊖ B = {a - b | a ∈ A, b ∈ B}
```

**GJK Theorem**: A ∩ B ≠ ∅ ⟺ origin ∈ (A ⊖ B)

### Contact Jacobian

For contact constraint C = (x_B - x_A) · n ≥ 0:

```
J = [n^T, (r_A × n)^T, -n^T, -(r_B × n)^T]  (1×12)
```

Effective mass in normal direction: `m_eff = 1 / (J M⁻¹ Jᵀ)`

---

## References

- **GJK**: Gilbert, Johnson, Keerthi (1988) — "A fast procedure for computing the distance between complex objects"
- **EPA**: van den Bergen (2001) — "Proximity Queries and Penetration Depth Computation on 3D Game Objects"
- **Contact Manifolds**: Ericson (2004) — "Real-Time Collision Detection"
- **Tutorial**: https://computerwebsite.net/writing/gjk

---

## Related Documentation

- **Constraint System**: [Constraints/CLAUDE.md](../Constraints/CLAUDE.md) — ContactConstraint, solver
- **Design Documents**: `docs/designs/0027a_expanding_polytope_algorithm/`, `0028_epa_witness_points/`, `0029_contact_manifold_generation/`, `docs/designs/0040c-edge-contact-manifold/`
- **Diagrams**: [`edge-contact-manifold.puml`](../../../../../../docs/msd/msd-sim/Physics/edge-contact-manifold.puml)
