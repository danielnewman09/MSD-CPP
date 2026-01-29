# Collision Detection and Response System

> This document provides architectural context for the collision detection and response subsystem within the Physics module.
> For overall Physics module context, see [`../CLAUDE.md`](../CLAUDE.md).

## Overview

The Collision subsystem provides comprehensive collision detection and response for rigid body dynamics. It implements the GJK (Gilbert-Johnson-Keerthi) algorithm for efficient boolean collision detection, EPA (Expanding Polytope Algorithm) for contact information extraction, and impulse-based collision response with Lagrangian constraint formulation.

**Location**: Mixed into `msd-sim/src/Physics/` (no separate subdirectory)
**Design Document**: [`../COLLISION_DETECTION_DESIGN.md`](../COLLISION_DETECTION_DESIGN.md)

## Architecture Overview

The collision system follows a three-phase pipeline:

```
Phase 1: Broad Phase (AABB culling)
    └── Eliminate non-overlapping pairs with bounding boxes

Phase 2: Narrow Phase (GJK collision detection)
    └── Boolean intersection test using Minkowski difference

Phase 3: Contact Resolution (EPA + Collision Response)
    ├── EPA: Extract penetration depth, normal, contact manifold
    └── Response: Apply impulse-based physics with Lagrangian constraints
```

### Performance Characteristics

| Operation | Cost (FLOPs) | Frequency | Notes |
|-----------|--------------|-----------|-------|
| AABB overlap test | ~50 | All pairs (broad phase) | Can use BVH for O(N log N) |
| GJK detection | ~500-2000 | AABB overlaps only | ~10-30 support queries |
| EPA contact resolution | ~2000-5000 | First frame of contact | Polytope expansion |

---

## Core Components

### CollisionHandler

**Location**: `../CollisionHandler.hpp`, `../CollisionHandler.cpp`
**Introduced**: [Ticket: 0027a_expanding_polytope_algorithm](../../../../../tickets/0027a_expanding_polytope_algorithm.md)

#### Purpose
Orchestrates GJK and EPA algorithms to provide a unified collision detection interface. Returns `std::optional<CollisionResult>` where `std::nullopt` indicates no collision.

#### Key Interfaces
```cpp
class CollisionHandler {
public:
  explicit CollisionHandler(double epsilon = 1e-6);

  /**
   * Check for collision between two physical assets.
   * Returns std::nullopt if no collision, CollisionResult if collision.
   */
  std::optional<CollisionResult> checkCollision(
      const AssetPhysical& assetA,
      const AssetPhysical& assetB) const;
};
```

#### Usage Example
```cpp
CollisionHandler collisionHandler{1e-6};

auto result = collisionHandler.checkCollision(assetA, assetB);
if (result) {
  // Collision detected - apply physics response
  std::cout << "Penetration: " << result->penetrationDepth << " m\n";
  std::cout << "Normal: " << result->normal << "\n";

  // Extract contact manifold
  for (size_t i = 0; i < result->contactCount; ++i) {
    const ContactPoint& cp = result->contacts[i];
    std::cout << "Contact " << i << ": A=" << cp.pointA
              << ", B=" << cp.pointB << "\n";
  }
}
```

#### Thread Safety
**Stateless after construction** — Safe to call from multiple threads with different asset pairs.

---

### GJK (Gilbert-Johnson-Keerthi Algorithm)

**Location**: `../GJK.hpp`, `../GJK.cpp`
**Introduced**: [Ticket: 0022_gjk_asset_physical_transform](../../../../../tickets/0022_gjk_asset_physical_transform.md)

#### Purpose
Efficient boolean collision detection for convex shapes with world-space transformations. Iteratively constructs a simplex in Minkowski difference space to determine if two objects intersect.

#### Key Insight
Two convex shapes A and B intersect if and only if their Minkowski difference (A ⊖ B) contains the origin. Transformations are applied on-the-fly during support function computation to avoid creating temporary transformed hulls.

#### Algorithm Overview
1. **Initialize**: Pick arbitrary search direction, query first support point
2. **Iterate**: Add support points, update simplex, check if origin contained
3. **Simplex Cases**:
   - Line (2 vertices): Project origin onto line
   - Triangle (3 vertices): Check if origin inside triangle or update to edge
   - Tetrahedron (4 vertices): Check if origin inside tetrahedron
4. **Terminate**: Return true if origin contained, false if search direction reverses

#### Key Interfaces
```cpp
class GJK {
public:
  GJK(const AssetPhysical& assetA,
      const AssetPhysical& assetB,
      double epsilon = 1e-6);

  bool intersects(int maxIterations = 64);

  // For EPA input
  const std::vector<Coordinate>& getSimplex() const;
};

// Convenience function
bool gjkIntersects(const AssetPhysical& assetA,
                   const AssetPhysical& assetB,
                   double epsilon = 1e-6,
                   int maxIterations = 64);
```

#### Transformation Pipeline
1. Transform search direction from world → local space (rotation only)
2. Find support vertex in local hull geometry
3. Transform support vertex from local → world space (rotation + translation)
4. Construct simplex in world space

#### Performance
- **Memory**: 16 bytes (two `AssetPhysical&` references)
- **Transformation overhead**: < 2% vs identity transform
- **Iterations**: Typically < 20 for convergence

#### Thread Safety
**Thread-safe for read-only assets** — Only reads from `AssetPhysical`, `ConvexHull`, `ReferenceFrame`.

---

### EPA (Expanding Polytope Algorithm)

**Location**: `../EPA.hpp`, `../EPA.cpp`
**Introduced**: [Ticket: 0027a_expanding_polytope_algorithm](../../../../../tickets/0027a_expanding_polytope_algorithm.md)
**Extended**: [Ticket: 0028_epa_witness_points](../../../../../tickets/0028_epa_witness_points.md) — Witness point tracking
**Extended**: [Ticket: 0029_contact_manifold_generation](../../../../../tickets/0029_contact_manifold_generation.md) — Multi-point manifolds

#### Purpose
Extracts penetration depth, contact normal, and contact manifold (up to 4 contact points) from GJK terminating simplex. Expands the simplex into a polytope until the closest face to the origin is found.

#### Algorithm Overview
1. **Initialize Polytope**: Start with GJK simplex (4 vertices, 4 faces)
2. **Find Closest Face**: Identify face closest to origin
3. **Query Support**: Get support point in direction of face normal
4. **Check Convergence**: If new point within tolerance, terminate
5. **Update Topology**:
   - Mark visible faces (can see new vertex)
   - Build horizon edges (boundary between visible/invisible)
   - Remove visible faces
   - Add new faces connecting horizon to new vertex
6. **Extract Contact**:
   - Penetration depth = distance from origin to closest face
   - Contact normal = closest face normal (normalized, A→B)
   - Contact manifold = up to 4 contact pairs on closest face

#### Key Interfaces
```cpp
class EPA {
public:
  EPA(const AssetPhysical& assetA,
      const AssetPhysical& assetB,
      double epsilon = 1e-6);

  /**
   * Compute contact information from GJK simplex.
   * @param simplex GJK terminating simplex (4 vertices in Minkowski space)
   * @return CollisionResult with manifold (1-4 contact points)
   */
  CollisionResult computeContactInfo(
      const std::vector<Coordinate>& simplex,
      int maxIterations = 64);
};
```

#### Witness Point Tracking (Ticket 0028)
EPA tracks the original surface points that contributed to each Minkowski vertex:

```cpp
struct MinkowskiVertex {
  Coordinate point;      // Minkowski difference point (A - B)
  Coordinate witnessA;   // Support point on A's surface
  Coordinate witnessB;   // Support point on B's surface
};
```

When the closest face is found, the barycentric centroid of witness points yields physical contact locations for accurate torque calculation: **τ = r × F**.

#### Contact Manifold Generation (Ticket 0029)
EPA generates up to 4 contact points per collision for improved stability:

1. **Feature Detection**: Identify collision feature type (vertex-face, edge-edge, face-face)
2. **Point Selection**: Extract relevant contact pairs from closest face vertices
3. **Manifold Storage**: Store in `CollisionResult.contacts` array with `contactCount`

#### Performance
- **Typical iterations**: 4-11 for simple shapes (well below 64 max)
- **Memory footprint**: < 10KB typical polytope size
- **Complexity**: O(iterations × faces)

#### Thread Safety
**Not thread-safe** — Contains mutable state during expansion (vertices, faces).

---

### SupportFunction

**Location**: `../SupportFunction.hpp`, `../SupportFunction.cpp`
**Introduced**: [Ticket: 0027a_expanding_polytope_algorithm](../../../../../tickets/0027a_expanding_polytope_algorithm.md) — Refactored from GJK/EPA
**Extended**: [Ticket: 0028_epa_witness_points](../../../../../tickets/0028_epa_witness_points.md) — Witness tracking

#### Purpose
Provides support function utilities for computing extremal points on convex hulls. The support function is the fundamental primitive for both GJK and EPA algorithms.

#### Key Concept
The **support function** returns the vertex of a convex hull that is furthest in a given direction:

```
support(hull, dir) = argmax(v · dir) for v in hull.vertices
```

For Minkowski difference:
```
supportMinkowski(A, B, dir) = support(A, dir) - support(B, -dir)
```

#### Key Interfaces
```cpp
namespace SupportFunction {
  // Basic support query (local space)
  Coordinate support(const ConvexHull& hull, const Coordinate& dir);

  // Minkowski support with transformations (world space)
  Coordinate supportMinkowski(const AssetPhysical& assetA,
                              const AssetPhysical& assetB,
                              const CoordinateRate& dir);

  // Minkowski support with witness tracking (for EPA)
  SupportResult supportMinkowskiWithWitness(const AssetPhysical& assetA,
                                            const AssetPhysical& assetB,
                                            const CoordinateRate& dir);
}

struct SupportResult {
  Coordinate minkowski;  // supportA - supportB (Minkowski space)
  Coordinate witnessA;   // Support point on A's surface (world space)
  Coordinate witnessB;   // Support point on B's surface (world space)
};
```

#### Thread Safety
**Stateless functions** — Safe to call from multiple threads.

---

### CollisionResult

**Location**: `../CollisionResult.hpp`
**Introduced**: [Ticket: 0027a_expanding_polytope_algorithm](../../../../../tickets/0027a_expanding_polytope_algorithm.md)
**Modified**: [Ticket: 0028_epa_witness_points](../../../../../tickets/0028_epa_witness_points.md) — Dual witness points
**Modified**: [Ticket: 0029_contact_manifold_generation](../../../../../tickets/0029_contact_manifold_generation.md) — Contact manifold

#### Purpose
Return value struct containing complete collision information with multi-point contact manifold. Used by collision response to resolve penetration and apply accurate torque.

#### Evolution
- **Ticket 0027a**: Single contact point in Minkowski space
- **Ticket 0028**: Replaced with `contactPointA` and `contactPointB` on object surfaces (witness points)
- **Ticket 0029**: Replaced with `contacts` array (up to 4 pairs) and `contactCount`

#### Key Interfaces
```cpp
struct ContactPoint {
  Coordinate pointA;  // Contact point on object A's surface (world space)
  Coordinate pointB;  // Contact point on object B's surface (world space)
};

struct CollisionResult {
  Coordinate normal;           // Contact normal (world space, A→B, unit length)
  double penetrationDepth;     // Overlap distance [m]

  // Contact manifold (Ticket 0029)
  std::array<ContactPoint, 4> contacts;  // Up to 4 contact pairs
  size_t contactCount;                   // Number of valid contacts [1, 4]

  // Constructors
  CollisionResult() = default;

  // Manifold constructor
  CollisionResult(const Coordinate& n, double depth,
                  const std::array<ContactPoint, 4>& contactsArray,
                  size_t count);

  // Single-contact convenience constructor
  CollisionResult(const Coordinate& n, double depth,
                  const Coordinate& pointA, const Coordinate& pointB);
};
```

#### Usage Example
```cpp
auto result = collisionHandler.checkCollision(assetA, assetB);
if (result) {
  // Iterate over contact manifold
  for (size_t i = 0; i < result->contactCount; ++i) {
    const ContactPoint& cp = result->contacts[i];

    // Compute lever arms for each contact
    Coordinate leverArmA = cp.pointA - assetA.getCenterOfMass();
    Coordinate leverArmB = cp.pointB - assetB.getCenterOfMass();

    // Apply impulse at contact point
    Coordinate impulse = result->normal * impulseMagnitude;
    applyImpulseAtPoint(assetA, impulse, leverArmA);
    applyImpulseAtPoint(assetB, -impulse, leverArmB);
  }
}
```

#### Design Rationale
- **No `intersecting` boolean**: Collision state conveyed by `std::optional` at API boundary
- **Witness points**: Enable accurate torque calculation (τ = r × F)
- **Multi-point manifold**: Distributes collision forces over larger area for stability
- **Fixed array size**: 4 contacts sufficient for polyhedral collision, avoids heap allocation

#### Thread Safety
**Value type** — Safe to copy across threads. Stack-allocated (no dynamic allocations).

---

### CollisionResponse

**Location**: `../CollisionResponse.hpp`, `../CollisionResponse.cpp`
**Introduced**: [Ticket: 0027_collision_response_system](../../../../../tickets/0027_collision_response_system.md)

#### Purpose
Stateless utility namespace providing Lagrangian constraint-based collision response. Implements frictionless collision response using Lagrange multipliers with Baumgarte stabilization for position correction.

#### Lagrangian Formulation

For a frictionless contact, the non-penetration constraint is:
```
C(q) = (x_B - x_A) · n ≥ 0
```

At the velocity level:
```
Ċ = v_rel · n ≥ 0
```

The constraint Jacobian is:
```
J = [n^T, (r × n)^T] for each body
```

The Lagrange multiplier λ (constraint force magnitude) is found by solving:
```
(J * M^-1 * J^T) * λ = -J * v
```

For frictionless contacts, the constraint force acts only in the normal direction:
```
F_constraint = λ * n
```

#### Key Functions

```cpp
namespace CollisionResponse {
  // Constants
  constexpr double kSlop = 0.01;              // 1cm slop tolerance [m]
  constexpr double kCorrectionFactor = 0.8;   // Position correction factor
  constexpr double kRestVelocityThreshold = 0.0001;  // Rest velocity [m/s]
  constexpr double kEnvironmentRestitution = 0.5;    // Static object elasticity

  // Dynamic-Dynamic Collision
  double computeLagrangeMultiplier(const AssetInertial& assetA,
                                   const AssetInertial& assetB,
                                   const CollisionResult& result,
                                   double restitution);

  void applyConstraintResponse(AssetInertial& assetA,
                               AssetInertial& assetB,
                               const CollisionResult& result,
                               double restitution);

  void applyPositionStabilization(AssetInertial& assetA,
                                  AssetInertial& assetB,
                                  const CollisionResult& result);

  // Dynamic-Static Collision
  double computeLagrangeMultiplierStatic(const AssetInertial& dynamic,
                                         const AssetEnvironment& staticObj,
                                         const CollisionResult& result,
                                         double restitution);

  void applyConstraintResponseStatic(AssetInertial& dynamic,
                                     const AssetEnvironment& staticObj,
                                     const CollisionResult& result,
                                     double restitution);

  void applyPositionStabilizationStatic(AssetInertial& dynamic,
                                        const AssetEnvironment& staticObj,
                                        const CollisionResult& result);

  // Utility
  double combineRestitution(double eA, double eB);
}
```

#### Response Algorithm

1. **Compute Lagrange Multiplier** (constraint force magnitude):
   ```
   λ = (1 + e) * (v_rel · n) / (J * M^-1 * J^T)
   ```

   Where the denominator expands to:
   ```
   (1/m_A + 1/m_B) + (I_A^-1 * (r_A × n)) · (r_A × n)
                   + (I_B^-1 * (r_B × n)) · (r_B × n)
   ```

2. **Apply Constraint Response** (velocity impulse):
   ```
   Δv_linear = ±λ * n / m
   Δω = I^-1 * (r × (±λ * n))
   ```

3. **Apply Position Stabilization** (Baumgarte):
   ```
   correction = max(penetrationDepth - kSlop, 0.0) * kCorrectionFactor
   separationVector = normal * correction
   ```

#### Multi-Contact Manifold Handling

For manifolds with multiple contact points (contactCount > 1), the response is applied **per-contact**:

```cpp
for (size_t i = 0; i < result.contactCount; ++i) {
  const ContactPoint& cp = result.contacts[i];

  // Compute per-contact Lagrange multiplier
  double lambda = computeLagrangeMultiplierForContact(cp, ...);

  // Apply impulse at this contact point
  applyImpulseAtContact(assetA, assetB, cp, lambda);
}
```

This distributes the collision force over the entire contact area, improving stability for face-face collisions.

#### Coefficient of Restitution

Combined using geometric mean:
```cpp
double combineRestitution(double eA, double eB) {
  return std::sqrt(eA * eB);
}
```

Properties:
- If either object is fully inelastic (e=0), collision is inelastic
- If both are fully elastic (e=1), collision is fully elastic
- Symmetric: e(A,B) = e(B,A)

#### Baumgarte Stabilization

Position correction prevents drift from numerical integration:

```
correction = max(penetrationDepth - kSlop, 0.0) * kCorrectionFactor
```

- **kSlop**: Prevents jitter from floating-point precision (objects penetrating < 1cm are not corrected)
- **kCorrectionFactor**: Fraction of penetration to correct per frame (0.8 = 80%)

#### Thread Safety
**Stateless functions** — Safe to call from multiple threads.

---

## Integration with Physics Pipeline

### WorldModel Integration

The collision system is integrated into `WorldModel::updatePhysics()`:

```cpp
void WorldModel::updatePhysics(std::chrono::milliseconds dt) {
  // 1. Collision Detection and Response (before integration)
  updateCollisions();

  // 2. Apply Forces (gravity, user forces)
  applyForces(dt);

  // 3. Numerical Integration (advance state)
  for (auto& asset : inertialAssets_) {
    integrator_->step(asset, forces, constraints, dt);
  }

  // 4. Update Reference Frames
  synchronizeReferenceFrames();
}
```

### Collision Detection Loop

```cpp
void WorldModel::updateCollisions() {
  // O(n²) pairwise collision detection
  for (size_t i = 0; i < inertialAssets_.size(); ++i) {
    for (size_t j = i + 1; j < inertialAssets_.size(); ++j) {
      auto result = collisionHandler_.checkCollision(
          inertialAssets_[i], inertialAssets_[j]);

      if (result) {
        // Combine restitution coefficients
        double e = CollisionResponse::combineRestitution(
            inertialAssets_[i].getCoefficientOfRestitution(),
            inertialAssets_[j].getCoefficientOfRestitution());

        // Apply velocity impulse
        CollisionResponse::applyConstraintResponse(
            inertialAssets_[i], inertialAssets_[j], *result, e);

        // Apply position correction
        CollisionResponse::applyPositionStabilization(
            inertialAssets_[i], inertialAssets_[j], *result);
      }
    }
  }
}
```

---

## Design Patterns

### Strategy Pattern
**Used in**: GJK simplex handling
**Purpose**: Different simplex cases (line, triangle, tetrahedron) handled by dedicated methods.

### Template Method Pattern
**Used in**: CollisionHandler orchestration
**Purpose**: Fixed algorithm structure (GJK → EPA) with extension points for future enhancements.

### Value Semantics
**Used in**: CollisionResult, ContactPoint, SupportResult
**Purpose**: Clear ownership, safe copying, efficient for small types.

### RAII
**Used in**: EPA polytope management
**Purpose**: Automatic cleanup of temporary data structures.

---

## Performance Optimization

### Broad Phase (Not Yet Implemented)

The current implementation uses O(n²) pairwise collision detection. Future optimizations:

1. **Spatial Partitioning**:
   - BVH (Bounding Volume Hierarchy): Better for dynamic scenes
   - Octree: Better for mostly static scenes
   - Grid: Simplest, works for uniform distributions

2. **AABB Culling**:
   - Maintain cached world-space AABB for each object
   - Update lazily when reference frame changes (8 corner transforms)
   - ~200 FLOPs per object per frame

### Contact Manifold Caching (Not Yet Implemented)

Objects in persistent contact (e.g., box resting on ground) only need EPA once:

1. **First Contact**: Run full GJK+EPA (~2000-5000 FLOPs)
2. **Subsequent Frames**: Update cached manifold cheaply (~200 FLOPs)
3. **Invalidation**: If contact points drift too far, re-run EPA

Expected performance gain: 10-25× faster for persistent contacts.

### Sleeping/Island Detection (Not Yet Implemented)

Stationary objects can skip physics updates:

1. **Velocity Thresholds**: Mark objects "asleep" when velocity < threshold
2. **Contact Graph**: Group connected objects into "islands"
3. **Wake Conditions**: Wake island when external force applied or new contact

Huge performance win for stacked/resting objects.

---

## Limitations and Future Work

### Current Limitations

1. **Frictionless Contacts**: Constraint forces act only along contact normal (no tangential friction)
2. **Sequential Resolution**: Single-pass collision response, can cause artifacts with > 5 simultaneous collisions per object
3. **No Continuous Collision Detection (CCD)**: Fast-moving objects can tunnel through thin obstacles
4. **No Sleeping**: All objects active every frame, even when at rest
5. **O(n²) Broad Phase**: Scales poorly beyond ~100 objects

### Future Enhancements

1. **Friction Constraints** (Priority: High)
   - Add tangential constraint forces (Coulomb friction model)
   - Requires iterative solver for friction cone constraints

2. **Iterative Constraint Solver** (Priority: High)
   - Replace single-pass response with iterative solver (Sequential Impulses, PGS)
   - Handles complex contact scenarios (stacks, chains)

3. **Continuous Collision Detection** (Priority: Medium)
   - Conservative advancement or swept collision tests
   - Prevents tunneling for fast-moving objects

4. **Broadphase Optimization** (Priority: Medium)
   - Implement BVH or spatial grid
   - Reduces from O(n²) to O(n log n)

5. **Contact Manifold Caching** (Priority: Medium)
   - Cache persistent contacts across frames
   - 10-25× speedup for resting objects

6. **Sleeping/Island Detection** (Priority: Low)
   - Skip physics for stationary object groups
   - Significant performance win for large static scenes

---

## Mathematical Reference

### Minkowski Difference

For convex sets A and B:
```
A ⊖ B = {a - b | a ∈ A, b ∈ B}
```

**GJK Theorem**: A and B intersect ⟺ origin ∈ (A ⊖ B)

### Simplex Cases

| Simplex | Vertices | Check |
|---------|----------|-------|
| Point | 1 | Direction toward origin |
| Line | 2 | Origin nearest to line or endpoint |
| Triangle | 3 | Origin inside triangle or nearest to edge/vertex |
| Tetrahedron | 4 | Origin inside tetrahedron |

### Contact Jacobian

For contact constraint C = (x_B - x_A) · n ≥ 0:

**Linear component**:
```
J_v = [n^T, -n^T]  (3×6 for single contact)
```

**Angular component**:
```
J_ω = [(r_A × n)^T, -(r_B × n)^T]  (3×6 for single contact)
```

**Full Jacobian**:
```
J = [J_v | J_ω]  (1×12 for single contact, both bodies)
```

### Effective Mass

The effective mass in the contact normal direction:
```
m_eff = 1 / (J * M^-1 * J^T)
```

Where M is the generalized mass matrix (6×6 block diagonal).

---

## Coding Standards

This subsystem follows the project-wide coding standards defined in the [root CLAUDE.md](../../../../../CLAUDE.md#coding-standards).

Key standards applied:
- **Initialization**: Brace initialization `{}`, `NaN` for uninitialized floats
- **Naming**: `PascalCase` for classes, `camelCase` for methods, `snake_case_` for members
- **Return Values**: Prefer `std::optional` over boolean + output parameters
- **Memory**: Value semantics for result structs, references for non-owning access

---

## Testing

### Test Organization
```
test/Physics/
├── CollisionHandlerTest.cpp     # GJK/EPA integration tests
├── EPATest.cpp                   # EPA algorithm tests (219 tests)
├── CollisionResponseTest.cpp     # Impulse calculation and position correction
└── GJKTest.cpp                   # GJK algorithm tests
```

### Running Tests
```bash
cmake --build --preset debug-sim-only --target msd_sim_test
./build/Debug/msd/msd-sim/msd_sim_test
```

---

## Related Documentation

### Module Documentation
- **Parent Module**: [`../CLAUDE.md`](../CLAUDE.md) — Physics module overview
- **Integration System**: [`../Integration/CLAUDE.md`](../Integration/CLAUDE.md) — Numerical integration
- **Constraint System**: [`../Constraints/CLAUDE.md`](../Constraints/CLAUDE.md) — Lagrange multipliers
- **Environment Module**: [`../../Environment/CLAUDE.md`](../../Environment/CLAUDE.md) — Coordinate system

### Design Documents
- [`../COLLISION_DETECTION_DESIGN.md`](../COLLISION_DETECTION_DESIGN.md) — Original architecture design
- [`docs/designs/0022_gjk_asset_physical_transform/`](../../../../../docs/designs/0022_gjk_asset_physical_transform/) — GJK with transforms
- [`docs/designs/0027a_expanding_polytope_algorithm/`](../../../../../docs/designs/0027a_expanding_polytope_algorithm/) — EPA implementation
- [`docs/designs/0028_epa_witness_points/`](../../../../../docs/designs/0028_epa_witness_points/) — Witness point tracking
- [`docs/designs/0029_contact_manifold_generation/`](../../../../../docs/designs/0029_contact_manifold_generation/) — Multi-point manifolds
- [`docs/designs/0027_collision_response_system/`](../../../../../docs/designs/0027_collision_response_system/) — Response system

### Tickets
- [0022_gjk_asset_physical_transform](../../../../../tickets/0022_gjk_asset_physical_transform.md) — GJK with AssetPhysical
- [0027a_expanding_polytope_algorithm](../../../../../tickets/0027a_expanding_polytope_algorithm.md) — EPA implementation
- [0028_epa_witness_points](../../../../../tickets/0028_epa_witness_points.md) — Witness point tracking
- [0029_contact_manifold_generation](../../../../../tickets/0029_contact_manifold_generation.md) — Contact manifolds
- [0027_collision_response_system](../../../../../tickets/0027_collision_response_system.md) — Collision response

---

## Getting Help

### For AI Assistants
1. This document provides complete collision system architecture
2. Review [`../CLAUDE.md`](../CLAUDE.md) for Physics module context
3. Review [`../../CLAUDE.md`](../../CLAUDE.md) for msd-sim library context
4. Check design documents in `docs/designs/` for detailed rationale

### For Developers
- **Collision detection**: Use `CollisionHandler::checkCollision()`
- **Custom response**: Implement using `CollisionResponse` utilities
- **Algorithm details**: See original papers (GJK: Gilbert 1988, EPA: van den Bergen 2001)
- **Performance tuning**: Start with broadphase optimization, then caching

---

## References

- **GJK Algorithm**: Gilbert, Johnson, Keerthi (1988) — "A fast procedure for computing the distance between complex objects in three-dimensional space"
- **EPA Algorithm**: van den Bergen (2001) — "Proximity Queries and Penetration Depth Computation on 3D Game Objects"
- **Contact Manifolds**: Ericson (2004) — "Real-Time Collision Detection"
- **Warm Starting**: Catto (2005) — "Iterative Dynamics with Temporal Coherence" (GDC)
- **Tutorial**: https://computerwebsite.net/writing/gjk — Excellent visual GJK/EPA explanation
