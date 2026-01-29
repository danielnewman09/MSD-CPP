# Physics Module Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/msd/msd-sim/Physics/` for detailed component relationships.

**Diagram**: [`physics-core.puml`](../../../../../docs/msd/msd-sim/Physics/physics-core.puml)

## Project Overview

**Physics** is a core module within `msd-sim` that provides force-based rigid body dynamics and collision detection. It includes convex hull representation, GJK collision detection, inertia tensor calculation, dynamic state management, and physics component integration for simulation objects.

## Architecture Overview

### High-Level Architecture

The Physics module provides a layered architecture for rigid body simulation:

```
PhysicsComponent (Object integration)
    ├── ConvexHull (Collision geometry)
    │   └── Qhull (External dependency)
    ├── DynamicState (Velocities/accelerations)
    └── InertialCalculations (Inertia tensors)

GJK (Collision detection algorithm)
    └── ConvexHull support functions
```

### Core Components

| Component | Location | Purpose | Diagram |
|-----------|----------|---------|---------|
| ConvexHull | `RigidBody/ConvexHull.hpp` | Convex hull geometry via Qhull | [`convex-hull.puml`](../../../../../docs/msd/msd-sim/Physics/convex-hull.puml) |
| PhysicsComponent | `RigidBody/PhysicsComponent.hpp` | Rigid body physics properties | [`physics-component.puml`](../../../../../docs/msd/msd-sim/Physics/physics-component.puml) |
| DynamicState | `RigidBody/DynamicState.hpp` | Velocities and accelerations | [`dynamic-state.puml`](../../../../../docs/msd/msd-sim/Physics/dynamic-state.puml) |
| InertialCalculations | `RigidBody/InertialCalculations.hpp` | Inertia tensor computation | [`mirtich-inertia-tensor.puml`](../../../../../docs/msd/msd-sim/Physics/mirtich-inertia-tensor.puml) |
| GJK | `GJK.hpp` | Gilbert-Johnson-Keerthi collision detection | [`gjk-asset-physical.puml`](../../../../../docs/msd/msd-sim/Physics/gjk-asset-physical.puml) |
| CollisionHandler | `CollisionHandler.hpp` | GJK/EPA orchestration for collision detection | [`epa.puml`](../../../../../docs/msd/msd-sim/Physics/epa.puml) |
| EPA | `EPA.hpp` | Expanding Polytope Algorithm for contact info | [`epa.puml`](../../../../../docs/msd/msd-sim/Physics/epa.puml) |
| CollisionResult | `CollisionResult.hpp` | Contact information struct with witness points | [`witness-points.puml`](../../../../../docs/msd/msd-sim/Physics/witness-points.puml) |
| SupportFunction | `SupportFunction.hpp` | Support function utilities with witness tracking | [`witness-points.puml`](../../../../../docs/msd/msd-sim/Physics/witness-points.puml) |
| MinkowskiVertex | `EPA.hpp` | Minkowski vertex with witness point tracking | [`witness-points.puml`](../../../../../docs/msd/msd-sim/Physics/witness-points.puml) |
| CollisionResponse | `CollisionResponse.hpp` | Impulse calculation and position correction utilities | [`collision-response.puml`](../../../../../docs/msd/msd-sim/Physics/collision-response.puml) |
| Integrator | `Integration/Integrator.hpp` | Abstract interface for numerical integration | [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml) |
| SemiImplicitEulerIntegrator | `Integration/SemiImplicitEulerIntegrator.hpp` | Symplectic integrator implementation | [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml) |
| PotentialEnergy | `PotentialEnergy/PotentialEnergy.hpp` | Abstract interface for environmental potentials | [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml) |
| GravityPotential | `PotentialEnergy/GravityPotential.hpp` | Uniform gravity implementation | [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml) |
| QuaternionConstraint | `Constraints/QuaternionConstraint.hpp` | Baumgarte stabilization for quaternion normalization (deprecated) | [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml) |
| Constraint | `Constraints/Constraint.hpp` | Abstract constraint interface for Lagrange multipliers | [`generalized-constraints.puml`](../../../../../docs/msd/msd-sim/Physics/generalized-constraints.puml) |
| BilateralConstraint | `Constraints/BilateralConstraint.hpp` | Abstract interface for equality constraints (C = 0) | [`generalized-constraints.puml`](../../../../../docs/msd/msd-sim/Physics/generalized-constraints.puml) |
| UnilateralConstraint | `Constraints/UnilateralConstraint.hpp` | Abstract interface for inequality constraints (C ≥ 0) | [`generalized-constraints.puml`](../../../../../docs/msd/msd-sim/Physics/generalized-constraints.puml) |
| UnitQuaternionConstraint | `Constraints/UnitQuaternionConstraint.hpp` | Unit quaternion normalization constraint | [`generalized-constraints.puml`](../../../../../docs/msd/msd-sim/Physics/generalized-constraints.puml) |
| DistanceConstraint | `Constraints/DistanceConstraint.hpp` | Fixed distance constraint between positions | [`generalized-constraints.puml`](../../../../../docs/msd/msd-sim/Physics/generalized-constraints.puml) |
| ConstraintSolver | `Constraints/ConstraintSolver.hpp` | Lagrange multiplier solver for constraint enforcement | [`generalized-constraints.puml`](../../../../../docs/msd/msd-sim/Physics/generalized-constraints.puml) |

---

## Component Details

### ConvexHull

**Location**: `RigidBody/ConvexHull.hpp`, `RigidBody/ConvexHull.cpp`
**Diagram**: [`convex-hull.puml`](../../../../../docs/msd/msd-sim/Physics/convex-hull.puml)
**Type**: Library component

#### Purpose
Represents a 3D convex hull for collision detection and mass property calculation. Wraps Qhull functionality to compute convex hulls from point clouds or geometry objects.

#### Key Features
- Construct from point clouds or `msd_assets::CollisionGeometry`
- Access to hull vertices and triangular facets
- Point containment testing and signed distance
- Axis-aligned bounding box for broad-phase collision

#### Key Interfaces
```cpp
class ConvexHull {
  struct BoundingBox { Coordinate min; Coordinate max; };
  struct Facet {
    std::array<size_t, 3> vertexIndices;
    Coordinate normal;
    double offset;
  };

  // Construction
  ConvexHull();  // Empty hull
  template <IsVector3 VectorType>
  explicit ConvexHull(const std::vector<VectorType>& points);
  explicit ConvexHull(const msd_assets::CollisionGeometry& geometry);

  // Geometry access
  const std::vector<Coordinate>& getVertices() const;
  const std::vector<Facet>& getFacets() const;
  size_t getVertexCount() const;
  size_t getFacetCount() const;

  // Properties computed by Qhull
  double getVolume() const;
  double getSurfaceArea() const;
  Coordinate getCentroid() const;
  BoundingBox getBoundingBox() const;
  bool isValid() const;

  // Collision queries
  bool contains(const Coordinate& point, double epsilon = 1e-6) const;
  double signedDistance(const Coordinate& point) const;
};
```

#### Usage Example
```cpp
// Create from point cloud
std::vector<Coordinate> points = {
  {0, 0, 0}, {1, 0, 0}, {0.5, 1, 0},
  {0.5, 0.5, 1}  // Tetrahedron
};
ConvexHull hull{points};

std::cout << "Volume: " << hull.getVolume() << std::endl;
std::cout << "Vertices: " << hull.getVertexCount() << std::endl;
std::cout << "Facets: " << hull.getFacetCount() << std::endl;

// Point containment
Coordinate testPoint{0.5, 0.5, 0.25};
if (hull.contains(testPoint)) {
  std::cout << "Point is inside hull" << std::endl;
}

// For collision detection, use GJK with AssetPhysical
// (See GJK section for collision detection examples)
```

#### Thread Safety
**Not thread-safe** — Qhull computation is not reentrant within a single hull.

#### Error Handling
- Throws `std::runtime_error` on empty point set
- Throws `std::runtime_error` on fewer than 4 points
- Throws `std::runtime_error` on degenerate geometry (coplanar points)
- Throws `std::runtime_error` on Qhull computation failure

#### Memory Management
- Owns vertex and facet vectors internally
- Qhull resources cleaned up in computeHull via RAII pattern

---

### PhysicsComponent

**Location**: `RigidBody/PhysicsComponent.hpp`, `RigidBody/PhysicsComponent.cpp`
**Diagram**: [`physics-component.puml`](../../../../../docs/msd/msd-sim/Physics/physics-component.puml)
**Type**: Library component

#### Purpose
Encapsulates rigid body physics properties for dynamic objects. Separated from `Object` to allow efficient storage and iteration during physics updates. Only present in `Inertial` objects.

#### Key Features
- Mass, inertia tensor, center of mass
- Dynamic state (velocities, accelerations)
- Force and torque application
- Kinetic energy calculation

#### Key Interfaces
```cpp
class PhysicsComponent {
  // Construction
  PhysicsComponent(const ConvexHull& hull, double mass);

  // Mass properties
  double getMass() const;
  const Eigen::Matrix3d& getInertiaTensor() const;
  const Eigen::Matrix3d& getInverseInertiaTensor() const;
  const Coordinate& getCenterOfMass() const;

  // Dynamic state access
  const DynamicState& getDynamicState() const;
  DynamicState& getDynamicState();
  double getKineticEnergy() const;

  // Force and torque application
  void applyForce(const Coordinate& force);
  void applyForceAtPoint(const Coordinate& force,
                         const Coordinate& localOffset);
  void applyTorque(const Eigen::Vector3d& torque);
  void clearForces();
};
```

#### Usage Example
```cpp
ConvexHull hull{collisionGeometry};
PhysicsComponent physics{hull, 50.0};  // 50 kg mass

// Apply gravity
double gravity = -9.81;
physics.applyForce(Coordinate{0, 0, gravity * physics.getMass()});

// Apply force at offset point (creates torque)
Coordinate windForce{10.0, 0, 0};
Coordinate sailPosition{0, 0, 2.0};  // Above center of mass
physics.applyForceAtPoint(windForce, sailPosition);

// Access state
double speed = physics.getDynamicState().getSpeed();
double energy = physics.getKineticEnergy();

// After integration, clear forces for next frame
physics.clearForces();
```

#### Thread Safety
**Not thread-safe** — Contains mutable DynamicState.

#### Error Handling
- Throws `std::invalid_argument` if mass <= 0
- Throws `std::invalid_argument` if hull is invalid

#### Memory Management
- Caches inverse inertia tensor for performance
- DynamicState owned by value

---

### DynamicState

**Location**: `RigidBody/DynamicState.hpp`, `RigidBody/DynamicState.cpp`
**Diagram**: [`dynamic-state.puml`](../../../../../docs/msd/msd-sim/Physics/dynamic-state.puml)
**Type**: Library component

#### Purpose
Complete dynamic state for rigid body motion. Encapsulates all time-varying kinematic quantities: linear and angular velocities and accelerations.

#### Key Interfaces
```cpp
class DynamicState {
  // Construction
  DynamicState();  // At rest
  DynamicState(const Coordinate& linearVelocity,
               const Eigen::Vector3d& angularVelocity);

  // Velocity access
  const Coordinate& getLinearVelocity() const;
  const Eigen::Vector3d& getAngularVelocity() const;
  void setLinearVelocity(const Coordinate& velocity);
  void setAngularVelocity(const Eigen::Vector3d& velocity);

  // Acceleration access
  const Coordinate& getLinearAcceleration() const;
  const Eigen::Vector3d& getAngularAcceleration() const;
  void setLinearAcceleration(const Coordinate& acceleration);
  void setAngularAcceleration(const Eigen::Vector3d& acceleration);

  // Convenience methods
  double getSpeed() const;
  double getAngularSpeed() const;
  bool isAtRest(double linearThreshold = 1e-6,
                double angularThreshold = 1e-6) const;
  void reset();

  // Kinetic energy
  double getLinearKineticEnergy(double mass) const;
  double getRotationalKineticEnergy(const Eigen::Matrix3d& inertiaTensor) const;
  double getTotalKineticEnergy(double mass,
                               const Eigen::Matrix3d& inertiaTensor) const;

  // Impulse application
  void applyLinearImpulse(const Coordinate& impulse, double mass);
  void applyAngularImpulse(const Eigen::Vector3d& impulse,
                           const Eigen::Matrix3d& inverseInertia);

  static DynamicState createAtRest();
};
```

#### Usage Example
```cpp
DynamicState state;
state.setLinearVelocity(Coordinate{10.0, 0.0, 0.0});  // Moving in +x
state.setAngularVelocity(Eigen::Vector3d{0, 0, 1.0}); // Rotating around z

// Check if at rest
if (!state.isAtRest()) {
  double speed = state.getSpeed();
  std::cout << "Speed: " << speed << " m/s" << std::endl;
}

// Apply impulse (instantaneous velocity change)
state.applyLinearImpulse(Coordinate{0, 100, 0}, 50.0);  // 100 N⋅s, 50 kg
```

#### Thread Safety
**Value semantics** — Safe to copy across threads.

---

### InertialCalculations

**Location**: `RigidBody/InertialCalculations.hpp`, `RigidBody/InertialCalculations.cpp`
**Diagram**: [`mirtich-inertia-tensor.puml`](../../../../../docs/msd/msd-sim/Physics/mirtich-inertia-tensor.puml)
**Type**: Utility namespace
**Tickets**:
- [0025_fix_inertia_tensor_calculation](../../../../../tickets/0025_fix_inertia_tensor_calculation.md) — Initial scaffolding
- [0026_mirtich_inertia_tensor](../../../../../tickets/0026_mirtich_inertia_tensor.md) — Mirtich algorithm implementation

#### Purpose
Utility function for computing moment of inertia tensor from convex hull geometry, assuming uniform density. Uses Brian Mirtich's algorithm from "Fast and Accurate Computation of Polyhedral Mass Properties" (1996), which applies the divergence theorem to convert volume integrals to surface integrals through three layers: projection integrals → face integrals → volume integrals.

This produces results within machine precision of analytical solutions (< 1e-10 error), a significant improvement over the previous tetrahedron decomposition approach which had ~10-15% error.

#### Algorithm Overview

The Mirtich algorithm computes inertia through hierarchical integral computation:

1. **Projection Integrals** (Layer 1): Iterate over face edges in 2D projection plane, computing line integrals (P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb)
2. **Face Integrals** (Layer 2): Lift projection integrals to 3D surface integrals (Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca)
3. **Volume Integrals** (Layer 3): Accumulate face integrals across all faces to compute T0 (volume), T1 (first moments), T2 (second moments), TP (products)
4. **Final Computation**: Derive inertia about origin from T2 and TP, compute center of mass (T1/T0), apply parallel axis theorem to shift to centroid

**Key Implementation Detail**: Includes vertex winding correction via `getWindingCorrectedIndices()` to ensure Qhull's vertex order aligns with facet normals, as required by the Mirtich algorithm.

#### Key Interfaces
```cpp
namespace InertialCalculations {
  /**
   * Compute inertia tensor about centroid (used for rigid body dynamics).
   * Uses Mirtich algorithm: mathematically exact surface integral formulation.
   * @throws std::invalid_argument if mass <= 0
   * @throws std::runtime_error if hull is degenerate
   */
  Eigen::Matrix3d computeInertiaTensorAboutCentroid(const ConvexHull& hull,
                                                     double mass);
}
```

#### Usage Example
```cpp
ConvexHull hull{collisionPoints};
double mass = 10.0;

// Get inertia about centroid (for dynamics)
Eigen::Matrix3d I = InertialCalculations::computeInertiaTensorAboutCentroid(hull, mass);

// Compute angular acceleration from torque
Eigen::Matrix3d I_inv = I.inverse();
Eigen::Vector3d torque{0, 0, 5.0};  // 5 N⋅m about z-axis
Eigen::Vector3d alpha = I_inv * torque;
```

#### Validation
- **Unit cube**: Ixx = Iyy = Izz = m/6 (exact within 1e-10)
- **Rectangular box (2×3×4)**: Matches analytical formulas (exact within 1e-10)
- **Regular tetrahedron**: Diagonal elements equal (exact within 1e-10)
- **Volume byproduct**: Computed volume matches `ConvexHull::getVolume()` within 1e-10
- **Large coordinate offsets**: Maintains accuracy within 1e-8 even at offsets of 1e6

---

### GJK (Gilbert-Johnson-Keerthi)

**Location**: `GJK.hpp`, `GJK.cpp`
**Diagram**: [`gjk-asset-physical.puml`](../../../../../docs/msd/msd-sim/Physics/gjk-asset-physical.puml)
**Type**: Library component
**Introduced**: [Ticket: 0022_gjk_asset_physical_transform](../../../../../tickets/0022_gjk_asset_physical_transform.md) (Breaking change)

#### Purpose
Efficient collision detection algorithm for convex shapes with world-space transformations. Iteratively constructs a simplex in Minkowski difference space to determine if two `AssetPhysical` objects intersect, applying `ReferenceFrame` transformations on-the-fly.

#### Key Insight
Two convex shapes A and B intersect if and only if their Minkowski difference (A ⊖ B) contains the origin. This implementation applies transformations during support function computation to avoid creating temporary transformed hulls.

#### Key Interfaces
```cpp
class GJK {
  /**
   * Construct solver for two AssetPhysical objects.
   * @param assetA First asset with collision hull and transform
   * @param assetB Second asset with collision hull and transform
   * @param epsilon Numerical tolerance (default: 1e-6)
   */
  GJK(const AssetPhysical& assetA, const AssetPhysical& assetB, double epsilon = 1e-6);

  /**
   * Test intersection.
   * @param maxIterations Maximum iterations (default: 64)
   * @return true if assets' collision hulls intersect in world space
   */
  bool intersects(int maxIterations = 64);
};

/**
 * Convenience function for one-shot intersection testing.
 */
bool gjkIntersects(const AssetPhysical& assetA,
                   const AssetPhysical& assetB,
                   double epsilon = 1e-6,
                   int maxIterations = 64);
```

#### Usage Example
```cpp
// Create collision hulls
ConvexHull hullA{pointsA};
ConvexHull hullB{pointsB};

// Create reference frames with world-space transforms
ReferenceFrame frameA{Coordinate{10.0, 0.0, 0.0}};  // Translated
ReferenceFrame frameB{Coordinate{0.0, 5.0, 0.0}, EulerAngles{0.0, 0.0, M_PI/4}};  // Translated + rotated

// Wrap in AssetPhysical objects
AssetPhysical assetA{0, 0, hullA, frameA};
AssetPhysical assetB{0, 1, hullB, frameB};

// One-shot test with transformations
if (gjkIntersects(assetA, assetB)) {
  std::cout << "Collision detected in world space!" << std::endl;
}

// Or create GJK instance for multiple queries
GJK gjk{assetA, assetB};
if (gjk.intersects()) {
  // Handle collision...
}
```

#### Transformation Pipeline
The GJK algorithm applies transformations on-the-fly during support function computation:

1. Transform search direction from world space to local space (`globalToLocalRelative` - rotation only)
2. Find support vertex in local hull space
3. Transform support vertex from local space to world space (`localToGlobal` - rotation + translation)
4. Construct simplex in world space

This approach avoids creating temporary transformed hulls, preserving memory efficiency.

#### Performance Characteristics
- **Memory**: No heap allocations during collision detection (16 bytes for two `AssetPhysical&` references)
- **Transformation overhead**: < 2% compared to identity transform baseline (validated by prototypes)
- **Complexity**: O(iterations), typically converges in < 20 iterations

#### Thread Safety
**Thread-safe for read-only assets** — GJK maintains its own state and only reads from `AssetPhysical`, `ConvexHull`, and `ReferenceFrame`.

#### Breaking Changes (Ticket 0022)
- **Removed**: `GJK(const ConvexHull&, const ConvexHull&)` constructor
- **Removed**: `gjkIntersects(const ConvexHull&, const ConvexHull&)` function
- **Removed**: `ConvexHull::intersects()` method
- **Added**: `GJK(const AssetPhysical&, const AssetPhysical&)` constructor
- **Added**: `gjkIntersects(const AssetPhysical&, const AssetPhysical&)` function

**Migration**: Wrap `ConvexHull` objects in `AssetPhysical` with identity `ReferenceFrame` for untransformed collision detection.

---

### CollisionHandler

**Location**: `CollisionHandler.hpp`, `CollisionHandler.cpp`
**Diagram**: [`epa.puml`](../../../../../docs/msd/msd-sim/Physics/epa.puml)
**Type**: Library component
**Introduced**: [Ticket: 0027a_expanding_polytope_algorithm](../../../../../tickets/0027a_expanding_polytope_algorithm.md)

#### Purpose
Orchestrates GJK and EPA algorithms to provide a unified collision detection interface that returns complete contact information. This abstraction enables future enhancements like broadphase optimization and continuous collision detection without changing callers.

#### Key Interfaces
```cpp
class CollisionHandler {
  /**
   * Construct handler with specified tolerance.
   * @param epsilon Numerical tolerance for GJK/EPA (default: 1e-6)
   */
  explicit CollisionHandler(double epsilon = 1e-6);

  /**
   * Check for collision between two physical assets.
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
};
```

#### Usage Example
```cpp
// Create collision handler
CollisionHandler collisionHandler{1e-6};

// Check collision between two assets
auto result = collisionHandler.checkCollision(assetA, assetB);
if (result) {
  // Collision detected - apply physics response
  std::cout << "Penetration depth: " << result->penetrationDepth << " m\n";
  std::cout << "Contact normal: " << result->normal << "\n";
  std::cout << "Contact point: " << result->contactPoint << "\n";

  applyImpulse(assetA, assetB, result->normal, result->penetrationDepth);
}
// else: no collision
```

#### Thread Safety
**Stateless after construction** — Safe to call from multiple threads with different asset pairs.

#### Error Handling
Propagates exceptions from GJK/EPA. Does not add additional error conditions.

#### Memory Management
- Lightweight: single double member (epsilon)
- Creates temporary GJK and EPA instances per collision check
- No heap allocations for collision detection

---

### EPA (Expanding Polytope Algorithm)

**Location**: `EPA.hpp`, `EPA.cpp`
**Diagram**: [`epa.puml`](../../../../../docs/msd/msd-sim/Physics/epa.puml)
**Type**: Library component
**Introduced**: [Ticket: 0027a_expanding_polytope_algorithm](../../../../../tickets/0027a_expanding_polytope_algorithm.md)

#### Purpose
Computes penetration depth, contact normal, and contact point from a GJK terminating simplex. When GJK detects an intersection, EPA expands the simplex into a polytope until the closest face to the origin is found, yielding complete collision data for physics response.

#### Algorithm Overview
1. **Initialize Polytope**: Start with GJK terminating simplex (4 vertices forming a tetrahedron)
2. **Find Closest Face**: Identify face closest to origin using distance calculation
3. **Expand Polytope**: Query support point in direction of closest face normal
4. **Convergence Check**: If new point within tolerance, terminate
5. **Topology Update**: Remove visible faces, build horizon edges, add new faces
6. **Extract Contact**: Derive penetration depth, normal, and contact point from closest face

#### Key Interfaces
```cpp
class EPA {
  /**
   * Construct EPA solver for two physical assets.
   * @param assetA First physical asset (includes hull and reference frame)
   * @param assetB Second physical asset (includes hull and reference frame)
   * @param epsilon Numerical tolerance for convergence (default: 1e-6)
   */
  EPA(const AssetPhysical& assetA,
      const AssetPhysical& assetB,
      double epsilon = 1e-6);

  /**
   * Compute contact information from GJK terminating simplex.
   *
   * Assumes simplex contains the origin (GJK returned true).
   * Expands polytope iteratively until closest face found.
   *
   * @param simplex GJK terminating simplex (4 vertices in Minkowski space)
   * @param maxIterations Maximum expansion iterations (default: 64)
   * @return CollisionResult with penetration depth, normal, contact point
   * @throws std::invalid_argument if simplex size > 4
   * @throws std::runtime_error if expansion fails to converge
   */
  CollisionResult computeContactInfo(const std::vector<Coordinate>& simplex,
                                      int maxIterations = 64);
};
```

#### Usage Example
```cpp
// After GJK detects intersection
GJK gjk{assetA, assetB};
if (gjk.intersects()) {
  // Use EPA to extract contact info
  EPA epa{assetA, assetB, 1e-6};
  CollisionResult result = epa.computeContactInfo(gjk.getSimplex());

  std::cout << "Penetration depth: " << result.penetrationDepth << " m\n";
  std::cout << "Contact normal: " << result.normal << "\n";
}
```

#### Performance Characteristics
- **Typical iterations**: 4-11 for simple shapes (well below 64 max)
- **Memory footprint**: < 10KB typical polytope size
- **Complexity**: O(iterations × faces), where faces grows with iterations

#### Thread Safety
**Not thread-safe** — Contains mutable state during expansion (vertices, faces).

#### Error Handling
- Throws `std::invalid_argument` if simplex size > 4
- Throws `std::runtime_error` if expansion fails to converge within max iterations
- Includes simplex completion logic for simplices < 4 vertices (robustness feature)

#### Memory Management
- Stores const references to AssetPhysical objects
- Owns vertex and face vectors internally
- Copy/move assignment deleted (cannot reassign reference members)

---

### CollisionResult

**Location**: `CollisionResult.hpp`
**Diagram**: [`witness-points.puml`](../../../../../docs/msd/msd-sim/Physics/witness-points.puml)
**Type**: Struct (POD)
**Introduced**: [Ticket: 0027a_expanding_polytope_algorithm](../../../../../tickets/0027a_expanding_polytope_algorithm.md)
**Modified**: [Ticket: 0028_epa_witness_points](../../../../../tickets/0028_epa_witness_points.md) — Breaking change to support witness points

#### Purpose
Return value struct containing complete collision information from EPA with witness points on both object surfaces. Used by physics response systems to resolve penetration and apply accurate torque during collision response.

#### Key Interfaces
```cpp
/**
 * Complete collision information for physics response.
 *
 * This struct is returned by EPA when a collision is detected.
 * It does NOT contain an 'intersecting' boolean because:
 * - CollisionHandler returns std::optional<CollisionResult>
 * - std::nullopt indicates no collision
 * - Presence of CollisionResult implies collision exists
 *
 * All coordinates are in world space.
 * Contact normal points from object A toward object B.
 *
 * Ticket 0028: Breaking change from single contactPoint to dual witness points
 * - contactPointA: Contact location on A's surface (world space)
 * - contactPointB: Contact location on B's surface (world space)
 * - Enables accurate torque calculation: τ = (contactPoint - centerOfMass) × impulse
 */
struct CollisionResult {
  Coordinate normal;           // Contact normal (world space, A→B, unit length)
  double penetrationDepth{std::numeric_limits<double>::quiet_NaN()};  // Overlap distance [m]
  Coordinate contactPointA;    // Contact point on A's surface (world space) [m]
  Coordinate contactPointB;    // Contact point on B's surface (world space) [m]

  CollisionResult() = default;
  CollisionResult(const Coordinate& n, double depth,
                  const Coordinate& pointA, const Coordinate& pointB);
};
```

#### Usage Example
```cpp
auto result = collisionHandler.checkCollision(assetA, assetB);
if (result) {
  // Access collision data with witness points
  Coordinate separationVector = result->normal * result->penetrationDepth;

  // Apply position correction
  assetA.translate(-separationVector * 0.5);
  assetB.translate(separationVector * 0.5);

  // Compute lever arms for accurate torque calculation
  Coordinate leverArmA = result->contactPointA - assetA.getCenterOfMass();
  Coordinate leverArmB = result->contactPointB - assetB.getCenterOfMass();

  // Apply torque: τ = r × F
  Coordinate impulse = result->normal * 100.0;
  Coordinate torqueA = leverArmA.cross(impulse);
  Coordinate torqueB = leverArmB.cross(-impulse);

  // Apply impulse for collision response with angular dynamics
  applyImpulse(assetA, assetB, result->normal, result->contactPointA, result->contactPointB);
}
```

#### Thread Safety
**Value type** — Safe to copy across threads.

#### Memory Management
- Stack-allocated struct (80 bytes: 3×24 bytes for Coordinates + 8 bytes for double)
- No dynamic allocations

#### Design Rationale
The `intersecting` boolean was deliberately excluded. Collision state is conveyed by `CollisionHandler::checkCollision()` returning `std::optional<CollisionResult>`:
- `std::nullopt` = no collision
- Presence of value = collision exists

This avoids redundant boolean fields and makes collision state explicit at the API boundary.

#### Breaking Changes (Ticket 0028)
- **Removed**: `Coordinate contactPoint` — Single point in Minkowski space
- **Added**: `Coordinate contactPointA` — Contact point on A's surface
- **Added**: `Coordinate contactPointB` — Contact point on B's surface
- **Rationale**: Single Minkowski-space contact point cannot be used for accurate torque calculation. Witness points provide physical contact locations on each object's surface, enabling correct lever arm computation for angular dynamics.

---

### SupportFunction

**Location**: `SupportFunction.hpp`, `SupportFunction.cpp`
**Diagram**: [`witness-points.puml`](../../../../../docs/msd/msd-sim/Physics/witness-points.puml)
**Type**: Utility namespace
**Introduced**: [Ticket: 0027a_expanding_polytope_algorithm](../../../../../tickets/0027a_expanding_polytope_algorithm.md)
**Extended**: [Ticket: 0028_epa_witness_points](../../../../../tickets/0028_epa_witness_points.md) — Added witness point tracking

#### Purpose
Provides support function utilities for computing extremal points on convex hulls in specified directions. Extended to track witness points (original surface points from both objects) for accurate torque calculation in collision response.

#### Key Interfaces
```cpp
namespace SupportFunction {
  /**
   * Compute support point on a convex hull.
   * Returns the vertex farthest in the given direction.
   */
  Coordinate support(const ConvexHull& hull, const Coordinate& dir);

  /**
   * Compute Minkowski difference support point.
   * Returns supportA - supportB in world space.
   */
  Coordinate supportMinkowski(const AssetPhysical& assetA,
                              const AssetPhysical& assetB,
                              const CoordinateRate& dir);

  /**
   * Compute Minkowski support with witness tracking.
   * Returns Minkowski point along with original witness points from both surfaces.
   * Ticket 0028: New function for EPA witness point tracking.
   */
  SupportResult supportMinkowskiWithWitness(const AssetPhysical& assetA,
                                            const AssetPhysical& assetB,
                                            const CoordinateRate& dir);
}

/**
 * Result of Minkowski support query with witness tracking.
 * Ticket 0028: New struct for EPA witness point extraction.
 */
struct SupportResult {
  Coordinate minkowski;  // supportA - supportB (Minkowski space)
  Coordinate witnessA;   // Support point on A's surface (world space)
  Coordinate witnessB;   // Support point on B's surface (world space)
};
```

#### Usage Example
```cpp
// Basic support query
ConvexHull hull{points};
Coordinate direction{1.0, 0.0, 0.0};
Coordinate extremalPoint = SupportFunction::support(hull, direction);

// Minkowski support with transformations (GJK)
Coordinate minkowskiPoint = SupportFunction::supportMinkowski(assetA, assetB, direction);

// Minkowski support with witness tracking (EPA)
SupportResult result = SupportFunction::supportMinkowskiWithWitness(assetA, assetB, direction);
// result.minkowski = supportA - supportB
// result.witnessA = actual point on A's surface
// result.witnessB = actual point on B's surface
```

#### Thread Safety
**Stateless functions** — Safe to call from multiple threads with different hull instances.

#### Error Handling
No error conditions. Assumes valid convex hull and non-zero direction.

#### Memory Management
- Stateless namespace functions
- `supportMinkowskiWithWitness()` returns `SupportResult` by value (RVO eliminates copy)

#### Design Rationale
Ticket 0028 added `supportMinkowskiWithWitness()` as a new function rather than modifying `supportMinkowski()` to preserve backward compatibility during transition. The existing function remains unchanged for callers that don't need witness points.

---

### MinkowskiVertex

**Location**: `EPA.hpp`
**Diagram**: [`witness-points.puml`](../../../../../docs/msd/msd-sim/Physics/witness-points.puml)
**Type**: Struct (internal to EPA)
**Introduced**: [Ticket: 0028_epa_witness_points](../../../../../tickets/0028_epa_witness_points.md)

#### Purpose
Internal structure used by EPA to track Minkowski difference vertices along with their contributing witness points from both object surfaces. Enables extraction of physical contact locations after polytope convergence.

#### Key Interfaces
```cpp
/**
 * Minkowski difference vertex with witness point tracking.
 * Used internally by EPA to maintain association between
 * Minkowski space points and their contributing surface points.
 * Ticket 0028: Replaces std::vector<Coordinate> vertices_.
 */
struct MinkowskiVertex {
  Coordinate point;      // Minkowski difference point (A - B)
  Coordinate witnessA;   // Support point on A that contributed
  Coordinate witnessB;   // Support point on B that contributed

  MinkowskiVertex() = default;
  MinkowskiVertex(const Coordinate& p, const Coordinate& wA, const Coordinate& wB)
    : point{p}, witnessA{wA}, witnessB{wB} {}
};
```

#### Usage Example
```cpp
// Internal EPA usage (not public API)
// Construct MinkowskiVertex from support query
SupportResult support = SupportFunction::supportMinkowskiWithWitness(assetA, assetB, dir);
MinkowskiVertex vertex{support.minkowski, support.witnessA, support.witnessB};
vertices_.push_back(vertex);

// Extract witness points from converged face
Coordinate witnessA = (vertices_[face.v0].witnessA +
                       vertices_[face.v1].witnessA +
                       vertices_[face.v2].witnessA) / 3.0;
```

#### Thread Safety
**Value type** — Safe to copy across threads.

#### Memory Management
- Stack-allocated struct (72 bytes: 3×24 bytes for Coordinates)
- No dynamic allocations
- Owned by `EPA::vertices_` vector

#### Design Rationale
This struct replaced `std::vector<Coordinate>` in EPA to track witness points alongside Minkowski vertices. The barycentric centroid of face vertices' witness points yields the physical contact location on each object's surface, enabling accurate torque calculation.

---

### CollisionResponse

**Location**: `CollisionResponse.hpp`, `CollisionResponse.cpp`
**Diagram**: [`collision-response.puml`](../../../../../docs/msd/msd-sim/Physics/collision-response.puml)
**Type**: Utility namespace
**Introduced**: [Ticket: 0027_collision_response_system](../../../../../tickets/0027_collision_response_system.md)

#### Purpose
Stateless utility namespace providing collision impulse calculation and position correction for rigid body dynamics. Enables realistic collision response with configurable elasticity using impulse-based physics formulas.

#### Key Interfaces
```cpp
namespace CollisionResponse {
  /**
   * Combine two coefficients of restitution using geometric mean.
   * Formula: e_combined = sqrt(e_A * e_B)
   *
   * This ensures:
   * - If either object is fully inelastic (e=0), collision is inelastic
   * - If both are fully elastic (e=1), collision is fully elastic
   * - Symmetric: e(A,B) = e(B,A)
   */
  double combineRestitution(double eA, double eB);

  /**
   * Compute scalar impulse magnitude for collision resolution.
   *
   * Uses the impulse-based collision response formula:
   *   j = -(1 + e) * (v_rel · n) / denominator
   *
   * Where:
   *   v_rel = relative velocity at contact point
   *   n = contact normal (A → B)
   *   e = combined coefficient of restitution
   *   denominator = (1/m_A + 1/m_B) + angular terms
   *
   * Angular terms account for rotational effects:
   *   denominator += (I_A^-1 * (r_A × n)) × r_A · n
   *                + (I_B^-1 * (r_B × n)) × r_B · n
   */
  double computeImpulseMagnitude(
      const AssetInertial& assetA,
      const AssetInertial& assetB,
      const CollisionResult& result,
      double combinedRestitution);

  /**
   * Apply position correction to separate overlapping objects.
   *
   * Uses linear projection with slop tolerance to prevent jitter.
   * Only corrects when penetration exceeds slop threshold.
   *
   * Correction formula:
   *   correction = max(penetrationDepth - kSlop, 0.0) * kCorrectionFactor
   *   separationVector = normal * correction
   *
   * Each object is moved by weighted fraction based on inverse mass.
   */
  void applyPositionCorrection(
      AssetInertial& assetA,
      AssetInertial& assetB,
      const CollisionResult& result);

  // Constants
  constexpr double kSlop = 0.01;              // 1cm slop tolerance [m]
  constexpr double kCorrectionFactor = 0.8;   // Position correction factor [0, 1]
}
```

#### Usage Example
```cpp
// Within WorldModel::updateCollisions()
auto result = collisionHandler_.checkCollision(assetA, assetB);
if (result) {
  // Combine restitution coefficients
  double combinedE = CollisionResponse::combineRestitution(
      assetA.getCoefficientOfRestitution(),
      assetB.getCoefficientOfRestitution());

  // Compute impulse magnitude
  double j = CollisionResponse::computeImpulseMagnitude(
      assetA, assetB, *result, combinedE);

  // Apply linear impulse
  CoordinateRate impulse = result->normal * j;
  assetA.getInertialState().velocity += impulse / assetA.getMass();
  assetB.getInertialState().velocity -= impulse / assetB.getMass();

  // Apply angular impulse
  Coordinate leverArmA = result->contactPointA - assetA.getInertialState().position;
  AngularRate angularImpulseA = assetA.getInverseInertiaTensor() *
                                 leverArmA.cross(impulse);
  assetA.getInertialState().angularVelocity += angularImpulseA;

  // Apply position correction
  CollisionResponse::applyPositionCorrection(assetA, assetB, *result);
}
```

#### Thread Safety
**Stateless functions** — Safe to call from multiple threads with different object instances.

#### Error Handling
No exceptions thrown. Assumes valid inputs from CollisionHandler (non-zero mass, valid collision result).

#### Memory Management
- Stateless namespace functions
- No heap allocations during collision response
- All calculations use stack-based temporaries

#### Design Rationale
- **Namespace over class**: No state needed, pure functions are sufficient
- **Separate impulse/position correction**: Allows independent testing and future customization
- **Geometric mean**: Symmetric, intuitive behavior, standard in physics engines
- **Slop tolerance**: Prevents objects from vibrating at rest due to floating-point jitter
- **Hardcoded constants**: Initial implementation simplicity; can be made configurable in future ticket if needed

#### Integration Points
- **CollisionHandler**: Provides `CollisionResult` with contact information
- **AssetInertial**: Reads mass/inertia/state, modifies velocities and position
- **WorldModel**: Orchestrates collision detection and response in `updateCollisions()`

---

### Integrator

**Location**: `Integration/Integrator.hpp`
**Type**: Abstract interface (header-only)
**Introduced**: [Ticket: 0030_lagrangian_quaternion_physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md)

#### Purpose
Abstract interface for numerical integration schemes, enabling swappable integrators (Euler, RK4, Verlet, etc.) without modifying WorldModel. Decouples integration math from physics orchestration.

#### Key Interfaces
```cpp
class Integrator {
public:
  virtual ~Integrator() = default;

  /**
   * @brief Integrate state forward by one timestep
   * @param state Current inertial state (modified in place)
   * @param force Net force in world frame [N]
   * @param torque Net torque in world frame [N·m]
   * @param mass Object mass [kg]
   * @param inverseInertia Inverse inertia tensor in body frame [1/(kg·m²)]
   * @param constraint Quaternion constraint for normalization
   * @param dt Timestep [s]
   */
  virtual void step(InertialState& state,
                    const Coordinate& force,
                    const Coordinate& torque,
                    double mass,
                    const Eigen::Matrix3d& inverseInertia,
                    QuaternionConstraint& constraint,
                    double dt) = 0;

protected:
  Integrator() = default;
  // Rule of Five with = default
};
```

#### Usage Example
```cpp
// Create integrator instance
auto integrator = std::make_unique<SemiImplicitEulerIntegrator>();

// Swap integrators at runtime
worldModel.setIntegrator(std::move(integrator));

// Integration happens in WorldModel::updatePhysics()
worldModel.update(std::chrono::milliseconds{16});
```

#### Thread Safety
**Stateless interface** — Implementations should be stateless and thread-safe.

#### Error Handling
No exceptions defined. Implementations may throw for invalid inputs.

#### Memory Management
- Pure abstract interface with protected constructor
- Owned by WorldModel via `std::unique_ptr<Integrator>`

---

### SemiImplicitEulerIntegrator

**Location**: `Integration/SemiImplicitEulerIntegrator.hpp`, `Integration/SemiImplicitEulerIntegrator.cpp`
**Diagram**: [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml)
**Type**: Library component
**Introduced**: [Ticket: 0030_lagrangian_quaternion_physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md)

#### Purpose
Symplectic integrator for rigid body dynamics using semi-implicit Euler method. Provides better energy conservation than explicit Euler while remaining computationally efficient (first-order accurate, symplectic for Hamiltonian systems).

#### Key Features
- **Velocity-first integration**: Updates velocities before positions using new velocities
- **Symplectic**: Preserves phase space volume, better long-term energy behavior
- **Quaternion support**: Integrates quaternion orientation with constraint enforcement
- **Simple and fast**: O(1) complexity per step, no matrix inversions required

#### Integration Order
```
1. Update velocities: v_new = v_old + a * dt
2. Update positions: x_new = x_old + v_new * dt (uses NEW velocity)
3. Update angular velocity: ω_new = ω_old + α * dt
4. Convert to quaternion rate: Q̇_new = ½ * Q ⊗ [0, ω_new]
5. Update quaternion: Q_new = Q_old + Q̇_new * dt
6. Enforce constraint: enforceConstraint(Q_new, Q̇_new)
```

#### Key Interfaces
```cpp
class SemiImplicitEulerIntegrator : public Integrator {
public:
  SemiImplicitEulerIntegrator() = default;
  ~SemiImplicitEulerIntegrator() override = default;

  void step(InertialState& state,
            const Coordinate& force,
            const Coordinate& torque,
            double mass,
            const Eigen::Matrix3d& inverseInertia,
            QuaternionConstraint& constraint,
            double dt) override;

  // Rule of Five with = default
};
```

#### Usage Example
```cpp
// Created by WorldModel constructor (default integrator)
// Or inject via setIntegrator()
auto integrator = std::make_unique<SemiImplicitEulerIntegrator>();
worldModel.setIntegrator(std::move(integrator));

// Integration occurs during WorldModel::updatePhysics()
// User does not call step() directly
```

#### Thread Safety
**Stateless** — Thread-safe for concurrent use with different state instances.

#### Error Handling
No exceptions thrown. All operations are numerically stable for valid inputs.

#### Memory Management
- Stateless implementation (no member variables beyond vtable)
- Owned by WorldModel via `std::unique_ptr<Integrator>`

---

### PotentialEnergy

**Location**: `PotentialEnergy/PotentialEnergy.hpp`
**Type**: Abstract interface (header-only)
**Diagram**: [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml)
**Introduced**: [Ticket: 0030_lagrangian_quaternion_physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md)

#### Purpose
Abstract interface for environmental potential energy fields in Lagrangian mechanics formulation. Enables extensible force computation for gravity, tidal forces, magnetic fields, atmospheric drag, etc. without modifying WorldModel.

#### Lagrangian Mechanics Context
```
L = T - V  (Lagrangian = Kinetic - Potential)
F = -∂V/∂X (Linear force from position gradient)
τ = -∂V/∂Q (Torque from orientation gradient)
```

#### Key Interfaces
```cpp
class PotentialEnergy {
public:
  virtual ~PotentialEnergy() = default;

  /**
   * @brief Compute linear force from potential energy gradient
   * @param state Current inertial state
   * @param mass Object mass [kg]
   * @return Generalized force F = -∂V/∂X [N]
   */
  virtual Coordinate computeForce(const InertialState& state, double mass) const = 0;

  /**
   * @brief Compute torque from potential energy gradient
   * @param state Current inertial state
   * @param inertia Inertia tensor in world frame [kg·m²]
   * @return Generalized torque τ = -∂V/∂Q [N·m]
   */
  virtual Coordinate computeTorque(const InertialState& state,
                                   const Eigen::Matrix3d& inertia) const = 0;

  /**
   * @brief Compute potential energy
   * @param state Current inertial state
   * @param mass Object mass [kg]
   * @return Potential energy V [J]
   */
  virtual double computeEnergy(const InertialState& state, double mass) const = 0;

protected:
  PotentialEnergy() = default;
  // Rule of Five with = default
};
```

#### Usage Example
```cpp
// Add multiple potential energies to WorldModel
worldModel.addPotentialEnergy(
    std::make_unique<GravityPotential>(Coordinate{0, 0, -9.81}));

// Future extensions
worldModel.addPotentialEnergy(
    std::make_unique<TidalPotential>(moonPosition, moonMass));

// Forces accumulate in WorldModel::updatePhysics()
Coordinate netForce{0, 0, 0};
for (const auto& potential : potentialEnergies_) {
  netForce += potential->computeForce(state, mass);
}
```

#### Thread Safety
**Read-only after construction** — Implementations must be thread-safe for concurrent queries.

#### Error Handling
Implementations may throw `std::invalid_argument` for invalid state (e.g., negative mass).

#### Memory Management
- Pure abstract interface with protected constructor
- Owned by WorldModel via `std::vector<std::unique_ptr<PotentialEnergy>>`

#### Future Extensions
- `TidalPotential` — Orientation-dependent tidal forces for multi-body systems
- `MagneticPotential` — Magnetic torques for spacecraft attitude control
- `DragPotential` — Velocity-dependent atmospheric dissipation
- `SpringPotential` — Per-object forces for elastic connections

---

### GravityPotential

**Location**: `PotentialEnergy/GravityPotential.hpp`, `PotentialEnergy/GravityPotential.cpp`
**Diagram**: [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml)
**Type**: Library component
**Introduced**: [Ticket: 0030_lagrangian_quaternion_physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md)

#### Purpose
Implements uniform gravitational field potential energy for Lagrangian mechanics. Produces constant force F = m*g independent of position and orientation, with zero torque (uniform fields do not couple to orientation).

#### Physics
```
Potential energy: V = m * g · r  (where r is position, g is gravity vector)
Force:           F = -∂V/∂r = m * g  (constant)
Torque:          τ = -∂V/∂Q = 0      (orientation-independent)
```

#### Key Interfaces
```cpp
class GravityPotential : public PotentialEnergy {
public:
  /**
   * @brief Construct gravitational field with specified acceleration vector
   * @param gravityVector Gravitational acceleration [m/s²], e.g. (0, 0, -9.81)
   */
  explicit GravityPotential(const Coordinate& gravityVector);

  ~GravityPotential() override = default;

  // PotentialEnergy interface implementation
  Coordinate computeForce(const InertialState& state, double mass) const override;
  Coordinate computeTorque(const InertialState& state,
                          const Eigen::Matrix3d& inertia) const override;
  double computeEnergy(const InertialState& state, double mass) const override;

  // Gravity configuration
  void setGravity(const Coordinate& gravityVector);
  const Coordinate& getGravity() const;

  // Rule of Five with = default

private:
  Coordinate g_{0.0, 0.0, -9.81};  // Gravitational acceleration [m/s²]
};
```

#### Usage Example
```cpp
// Create uniform gravity field (z-up convention)
auto gravity = std::make_unique<GravityPotential>(Coordinate{0, 0, -9.81});

// Add to WorldModel
worldModel.addPotentialEnergy(std::move(gravity));

// Compute force for 10 kg object
Coordinate force = gravity->computeForce(state, 10.0);
// Returns: (0, 0, -98.1) N

// Compute energy at height z = 100 m
double energy = gravity->computeEnergy(state, 10.0);
// Returns: -9810 J (negative because g points down)
```

#### Thread Safety
**Immutable after construction** — Thread-safe for concurrent force queries.

#### Error Handling
No exceptions thrown. Gravity vector can be arbitrary, including zero (free-space dynamics).

#### Memory Management
- Single `Coordinate` member (24 bytes)
- Value semantics with compiler-generated copy/move
- Owned by WorldModel via `std::unique_ptr`

#### Design Notes
- **Sign convention**: Energy is negative below origin for downward gravity (consistent with V = -m*g·r convention)
- **Zero torque**: Uniform fields produce no torque regardless of object orientation
- **Configuration**: `setGravity()` should only be called during initialization to preserve thread safety

---

### QuaternionConstraint

**Location**: `Constraints/QuaternionConstraint.hpp`, `Constraints/QuaternionConstraint.cpp`
**Diagram**: [`0030_lagrangian_quaternion_physics.puml`](../../../../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml)
**Type**: Library component
**Introduced**: [Ticket: 0030_lagrangian_quaternion_physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md)

#### Purpose
Enforces unit quaternion constraint |Q|=1 via Lagrange multipliers with Baumgarte stabilization to correct drift accumulated during numerical integration. Maintains quaternion normalization within 1e-10 tolerance over extended simulations.

#### Constraint Formulation
```
Position constraint: g(Q) = Q^T*Q - 1 = 0    (unit quaternion)
Velocity constraint: ġ = 2*Q^T*Q̇ = 0        (Q̇ perpendicular to Q)

Baumgarte stabilization:
  λ = -α*g - β*ġ    (Lagrange multiplier with feedback)
  F_constraint = G^T * λ  where G = ∂g/∂Q = 2*Q^T

Default parameters: α = 10.0, β = 10.0
```

#### Key Interfaces
```cpp
class QuaternionConstraint {
public:
  /**
   * @brief Construct constraint with Baumgarte parameters
   * @param alpha Position error gain (default: 10.0)
   * @param beta Velocity error gain (default: 10.0)
   */
  explicit QuaternionConstraint(double alpha = 10.0, double beta = 10.0);

  ~QuaternionConstraint() = default;

  /**
   * @brief Enforce unit quaternion constraint with Baumgarte stabilization
   *
   * Modifies Q and Qdot to satisfy:
   * 1. Normalize Q to unit length
   * 2. Project Qdot onto tangent space (perpendicular to Q)
   * 3. Apply Baumgarte correction to reduce drift
   *
   * @param Q Quaternion to constrain (modified in place)
   * @param Qdot Quaternion rate (modified in place)
   */
  void enforceConstraint(Eigen::Quaterniond& Q, Eigen::Vector4d& Qdot);

  /**
   * @brief Compute constraint force for dynamics integration
   * @param Q Current quaternion
   * @param Qdot Current quaternion rate
   * @return Constraint force F_c = G^T * λ where G = ∂g/∂Q
   */
  Eigen::Vector4d computeConstraintForce(const Eigen::Quaterniond& Q,
                                         const Eigen::Vector4d& Qdot) const;

  // Parameter configuration
  void setAlpha(double alpha);
  void setBeta(double beta);
  double getAlpha() const;
  double getBeta() const;

  // Constraint violation queries (for diagnostics)
  double positionViolation(const Eigen::Quaterniond& Q) const;
  double velocityViolation(const Eigen::Quaterniond& Q,
                          const Eigen::Vector4d& Qdot) const;

  // Rule of Five with = default

private:
  double alpha_{10.0};  // Position error gain
  double beta_{10.0};   // Velocity error gain
};
```

#### Usage Example
```cpp
// Each AssetInertial owns its own constraint
QuaternionConstraint constraint{10.0, 10.0};

// Enforce constraint during integration
// (called by SemiImplicitEulerIntegrator::step)
constraint.enforceConstraint(state.orientation, state.quaternionRate);

// Check constraint violation for diagnostics
double posError = constraint.positionViolation(state.orientation);
// Should be < 1e-10 for properly tuned α, β

// Query constraint force for analysis
Eigen::Vector4d force = constraint.computeConstraintForce(
    state.orientation, state.quaternionRate);
```

#### Thread Safety
**Not thread-safe** — Modifies quaternion state during constraint enforcement.

#### Error Handling
No exceptions thrown. All quaternion operations are numerically stable.

#### Memory Management
- Two `double` members (16 bytes total)
- Value semantics with compiler-generated copy/move
- Each AssetInertial owns its own constraint instance (not shared)

#### Design Notes
- **Baumgarte parameters**: α=10, β=10 are literature defaults validated for dt=0.016s (60 FPS)
- **Ownership**: Each asset owns its constraint since each has its own quaternion state
- **Performance**: ~40 FLOPs per enforcement, negligible overhead (< 1% of physics loop)
- **Accuracy**: Maintains |Q²-1| < 1e-10 over 10000 integration steps (validated by tests)

---

### Generalized Constraint Framework

**Location**: `Constraints/Constraint.hpp`, `Constraints/BilateralConstraint.hpp`, `Constraints/UnilateralConstraint.hpp`, `Constraints/ConstraintSolver.hpp`
**Diagram**: [`generalized-constraints.puml`](../../../../../docs/msd/msd-sim/Physics/generalized-constraints.puml)
**Type**: Library infrastructure
**Introduced**: [Ticket: 0031_generalized_lagrange_constraints](../../../../../tickets/0031_generalized_lagrange_constraints.md)

#### Purpose
Provides an extensible constraint framework using Lagrange multipliers that enables users to define arbitrary constraints by implementing the Constraint interface. The system separates constraint definition (evaluation, Jacobian) from constraint solving (Lagrange multiplier computation), enabling a library of constraint types that all use the same solver infrastructure.

**Key benefit**: New constraint types (joints, contacts, limits) can be added by implementing the Constraint interface without modifying the solver or integration infrastructure.

#### Architecture Components

**Constraint (Abstract Base Class)**
- Abstract interface defining mathematical requirements for constraint definitions
- Pure virtual methods: `dimension()`, `evaluate()`, `jacobian()`, `typeName()`
- Optional virtual methods: `partialTimeDerivative()`, Baumgarte parameters `alpha()` and `beta()`
- Location: `Constraints/Constraint.hpp`

**BilateralConstraint (Abstract Subclass)**
- Specialization for equality constraints C(q, t) = 0 with unrestricted Lagrange multipliers
- Semantic marker for bilateral constraints (C must equal zero)
- Location: `Constraints/BilateralConstraint.hpp`

**UnilateralConstraint (Abstract Subclass)**
- Specialization for inequality constraints C(q, t) ≥ 0 with complementarity conditions
- Includes `isActive()` method for contact activation/deactivation
- **Note**: Unilateral solver not yet implemented, interface defined for future use
- Location: `Constraints/UnilateralConstraint.hpp`

**ConstraintSolver**
- Computes Lagrange multipliers λ for a set of constraints using direct LLT solve
- Returns `SolveResult` containing lambdas, constraint forces, convergence status, condition number
- Stateless utility — all matrices are local variables (thread-safe after construction)
- Location: `Constraints/ConstraintSolver.hpp`

**UnitQuaternionConstraint**
- Concrete implementation of BilateralConstraint for unit quaternion enforcement
- Constraint function: C(Q) = Q^T*Q - 1 = 0
- Jacobian: J = 2*Q^T (1x7 matrix, only quaternion components non-zero)
- Replaces deprecated `QuaternionConstraint` class
- Location: `Constraints/UnitQuaternionConstraint.hpp`

**DistanceConstraint**
- Concrete implementation of BilateralConstraint for fixed distance between positions
- Constraint function: C(x) = |x|² - d² = 0
- Jacobian: J = 2*x^T (1x7 matrix, only position components non-zero)
- Example constraint demonstrating single-object constraints
- Location: `Constraints/DistanceConstraint.hpp`

#### Mathematical Framework

A constraint is defined by:
```
1. Constraint function: C(q, t) = 0 (holonomic) or C(q, q̇, t) = 0 (non-holonomic)
2. Constraint Jacobian: J = ∂C/∂q (how constraint changes with configuration)
3. Constraint time derivative: Ċ = J·q̇ + ∂C/∂t
4. Baumgarte stabilization terms: α (position gain), β (velocity gain)
```

The Lagrange multiplier λ is computed to satisfy:
```
J·M⁻¹·Jᵀ·λ = -J·M⁻¹·F_ext - J̇·q̇ - α·C - β·Ċ
```

The constraint force applied is:
```
F_constraint = Jᵀ·λ
```

#### Key Interfaces

**Constraint Interface**:
```cpp
class Constraint {
public:
  virtual ~Constraint() = default;

  // Number of scalar constraint equations
  virtual int dimension() const = 0;

  // Evaluate constraint function C(q, t)
  // Returns vector of constraint violations
  virtual Eigen::VectorXd evaluate(
      const InertialState& state,
      double time) const = 0;

  // Compute constraint Jacobian J = ∂C/∂q
  // Returns (dimension x 7) matrix for single-object constraints
  virtual Eigen::MatrixXd jacobian(
      const InertialState& state,
      double time) const = 0;

  // Compute constraint time derivative ∂C/∂t (optional, default: zero)
  virtual Eigen::VectorXd partialTimeDerivative(
      const InertialState& state,
      double time) const;

  // Baumgarte stabilization parameters
  virtual double alpha() const { return 10.0; }
  virtual double beta() const { return 10.0; }

  // Constraint type for debugging/logging
  virtual std::string typeName() const = 0;

protected:
  Constraint() = default;
  // Rule of Five with = default
};
```

**ConstraintSolver Interface**:
```cpp
class ConstraintSolver {
public:
  struct SolveResult {
    Eigen::VectorXd lambdas;           // Lagrange multipliers
    Eigen::VectorXd constraintForces;  // Generalized forces (Jᵀ·λ)
    bool converged;                    // Whether solve succeeded
    double conditionNumber;            // Condition number (for diagnostics)
  };

  ConstraintSolver() = default;

  /**
   * @brief Solve for Lagrange multipliers given system state
   *
   * Computes λ satisfying: J·M⁻¹·Jᵀ·λ = RHS
   * where RHS includes external forces, Baumgarte stabilization, etc.
   *
   * @param constraints Vector of constraint pointers (non-owning)
   * @param state Current inertial state
   * @param externalForces Currently applied forces
   * @param massMatrix Inverse mass matrix M⁻¹ (7x7 for single object)
   * @param dt Integration timestep
   * @return SolveResult with λ, forces, and convergence status
   */
  SolveResult solve(
      const std::vector<Constraint*>& constraints,
      const InertialState& state,
      const Eigen::VectorXd& externalForces,
      const Eigen::MatrixXd& massMatrix,
      double dt) const;

  // Rule of Five with = default
};
```

**UnitQuaternionConstraint Example**:
```cpp
class UnitQuaternionConstraint : public BilateralConstraint {
public:
  explicit UnitQuaternionConstraint(double alpha = 10.0, double beta = 10.0);

  int dimension() const override { return 1; }  // Scalar constraint

  Eigen::VectorXd evaluate(const InertialState& state, double time) const override;
  Eigen::MatrixXd jacobian(const InertialState& state, double time) const override;

  double alpha() const override { return alpha_; }
  double beta() const override { return beta_; }
  std::string typeName() const override { return "UnitQuaternionConstraint"; }

  // Setters for Baumgarte parameters
  void setAlpha(double alpha);
  void setBeta(double beta);

  // Rule of Five with = default

private:
  double alpha_{10.0};
  double beta_{10.0};
};
```

#### Usage Example
```cpp
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/DistanceConstraint.hpp"

// AssetInertial owns constraints via std::unique_ptr
AssetInertial asset{/* ... */};

// Default: UnitQuaternionConstraint automatically added to every AssetInertial
// asset.getConstraints() returns {UnitQuaternionConstraint*}

// Add additional constraints
asset.addConstraint(std::make_unique<DistanceConstraint>(5.0));  // 5m distance
// asset.getConstraints() now returns {UnitQuaternionConstraint*, DistanceConstraint*}

// SemiImplicitEulerIntegrator uses ConstraintSolver automatically:
// (Called internally by WorldModel::updatePhysics)
ConstraintSolver solver;
auto result = solver.solve(
    asset.getConstraints(),  // Non-owning pointers
    asset.getInertialState(),
    externalForces,
    massMatrix,
    dt);

if (result.converged) {
  // Apply constraint forces: F_total = F_ext + F_constraint
  Eigen::VectorXd totalForces = externalForces + result.constraintForces;
  // Proceed with integration...
} else {
  // Handle singular constraint matrix (rare)
  // Condition number available in result.conditionNumber
}
```

#### Integration with Physics Pipeline

The constraint framework integrates with the existing physics pipeline:

1. **AssetInertial ownership**: Each `AssetInertial` owns a vector of constraints via `std::vector<std::unique_ptr<Constraint>>`
2. **Default constraint**: Every `AssetInertial` automatically includes a `UnitQuaternionConstraint` (maintains quaternion normalization)
3. **Constraint gathering**: `WorldModel::updatePhysics()` gathers constraints from all assets
4. **Solver invocation**: `SemiImplicitEulerIntegrator::step()` uses `ConstraintSolver` to compute constraint forces
5. **Force application**: Constraint forces are added to external forces before integration

**Integrator signature change (breaking)**:
```cpp
// Old (ticket 0030)
void step(InertialState& state,
          double mass,
          const Eigen::Matrix3d& inertia,
          const Coordinate& netForce,
          const Coordinate& netTorque,
          QuaternionConstraint& constraint,
          double dt);

// New (ticket 0031)
void step(InertialState& state,
          double mass,
          const Eigen::Matrix3d& inertia,
          const Coordinate& netForce,
          const Coordinate& netTorque,
          const std::vector<Constraint*>& constraints,  // Non-owning pointers
          double dt);
```

#### Thread Safety
**Read-only operations thread-safe after construction** — Constraint evaluation, Jacobian computation, and solving are const methods.

**Not thread-safe for constraint modification** — Adding/removing constraints from AssetInertial must happen outside physics loop.

#### Error Handling
- **Singular constraint matrices**: `ConstraintSolver::solve()` returns `converged = false` when LLT decomposition fails
- **Invalid parameters**: Constraint constructors throw `std::invalid_argument` for invalid parameters (e.g., negative distance)
- **Empty constraint sets**: Solver handles empty constraint vector gracefully (returns zero forces, converged = true)

#### Memory Management
- **Ownership**: `AssetInertial` owns constraints via `std::vector<std::unique_ptr<Constraint>>`
- **Non-owning access**: `getConstraints()` returns `std::vector<Constraint*>` (raw pointers, non-owning)
- **Lifetime**: Constraint pointers valid as long as owning `AssetInertial` exists
- **Solver stateless**: `ConstraintSolver` allocates matrices locally, no persistent state

#### Performance
- **Solver complexity**: O(n³) where n = total constraint dimension (direct LLT solve)
- **Typical usage**: n < 10 constraints per object (< 1ms solve time)
- **Overhead**: ~1-2% of physics loop for typical constraint sets (validated by prototypes)
- **Condition number**: Reported in `SolveResult` for diagnostics (well-conditioned: < 100)

#### Design Notes
- **Extensibility**: New constraint types added by implementing `Constraint` interface
- **Separation of concerns**: Constraint definition (evaluate, Jacobian) separate from solving
- **Future work**: Unilateral solver (projected Gauss-Seidel), multi-object constraints (joints, contacts)
- **Migration path**: Old `QuaternionConstraint` deprecated, use `UnitQuaternionConstraint` instead
- **Backward compatibility**: Existing code using `QuaternionConstraint` continues to work (separate API)

---

## Design Patterns in Use

### Factory Pattern
**Used in**: Object creation, DynamicState::createAtRest()
**Purpose**: Enforces valid construction with proper initialization.

### Strategy Pattern
**Used in**: GJK simplex handling
**Purpose**: Different simplex cases (line, triangle, tetrahedron) handled by dedicated methods.

### Value Semantics
**Used in**: DynamicState, Coordinate (from Environment module)
**Purpose**: Clear ownership, safe copying, efficient for small types.

### RAII
**Used in**: Qhull resource management in ConvexHull::computeHull
**Purpose**: Automatic cleanup of external library resources.

---

## Cross-Cutting Concerns

### Units
All quantities use SI units:
- Mass: kilograms [kg]
- Force: Newtons [N]
- Torque: Newton-meters [N⋅m]
- Linear acceleration: meters per second squared [m/s²]
- Angular acceleration: radians per second squared [rad/s²]
- Position: meters [m]
- Inertia: kilogram-meters squared [kg⋅m²]

### Error Handling Strategy
- **Exceptions**: Thrown for invalid parameters, degenerate geometry
- **Validation**: Constructors validate input parameters
- **Qhull errors**: Wrapped and rethrown as `std::runtime_error`

### Memory Management
- **ConvexHull**: Owns vertex and facet data
- **PhysicsComponent**: Caches inverse inertia tensor
- **DynamicState**: Pure value type
- **GJK**: Temporary simplex storage, references to hulls

### Thread Safety Conventions
- **Read-only operations**: Thread-safe for ConvexHull after construction
- **Mutable operations**: Not thread-safe (DynamicState modification, force application)
- **GJK**: Safe to use concurrently with different instances
- **CollisionHandler**: Stateless after construction, safe to call from multiple threads
- **EPA**: Not thread-safe (mutable state during expansion)
- **CollisionResult**: Value type, safe to copy across threads

---

## Physics Calculations

### Linear Acceleration
```
F_net = Σ F_i
a = F_net / m
```

### Angular Acceleration
```
τ_net = Σ (r_i × F_i)    where r_i = applicationPoint - centerOfMass
α = I⁻¹ * τ_net
```

### Kinetic Energy
```
KE = 0.5 * m * v² + 0.5 * ω^T * I * ω
```

---

## Build & Configuration

### Build Requirements
- **C++ Standard**: C++20
- **Dependencies**:
  - Eigen3 — Linear algebra
  - Qhull — Convex hull computation
  - msd-assets — Collision geometry types

### Building This Component

This module is built as part of `msd-sim`:

```bash
# From project root
cmake --preset conan-debug
cmake --build --preset debug-sim-only
```

---

## Testing

### Test Organization
```
test/Physics/
├── ConvexHullTest.cpp      # Hull computation, containment, intersection
└── README.md               # Physics testing documentation
```

### Running Tests
```bash
cmake --build --preset debug-tests-only
ctest --preset debug
```

---

## Coding Standards

This module follows the project-wide coding standards defined in the [root CLAUDE.md](../../../../CLAUDE.md#coding-standards).

Key standards applied in this module:
- **Initialization**: `std::numeric_limits<float>::quiet_NaN()` for uninitialized floats
- **Naming**: `PascalCase` for classes, `camelCase` for methods, `snake_case_` for members
- **Return Values**: Return structs (BoundingBox) over output parameters
- **Memory**: Value semantics for DynamicState, RAII for Qhull resources

See the [root CLAUDE.md](../../../../CLAUDE.md#coding-standards) for complete details and examples.

---

## Getting Help

### For AI Assistants
1. This document provides complete architectural context for the Physics module
2. Review [Environment/CLAUDE.md](../Environment/CLAUDE.md) for Coordinate and related types
3. Review [msd-sim/CLAUDE.md](../../CLAUDE.md) for overall simulation architecture
4. Check [root CLAUDE.md](../../../../CLAUDE.md) for project-wide conventions

### For Developers
- **Collision geometry**: Start with ConvexHull
- **Dynamic objects**: Use PhysicsComponent with DynamicState
- **Collision detection**: Use GJK with AssetPhysical objects
- **Inertia calculation**: Use InertialCalculations namespace
- **Integration patterns**: See README.md in this directory
