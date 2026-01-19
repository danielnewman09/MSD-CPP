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
| InertialCalculations | `RigidBody/InertialCalculations.hpp` | Inertia tensor computation | [`physics-component.puml`](../../../../../docs/msd/msd-sim/Physics/physics-component.puml) |
| GJK | `GJK.hpp` | Gilbert-Johnson-Keerthi collision | [`gjk.puml`](../../../../../docs/msd/msd-sim/Physics/gjk.puml) |

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
**Type**: Utility namespace

#### Purpose
Utility functions for computing moment of inertia tensors from convex hull geometry, assuming uniform density.

#### Key Interfaces
```cpp
namespace InertialCalculations {
  /**
   * Compute inertia tensor about origin.
   * @throws std::invalid_argument if mass <= 0
   * @throws std::runtime_error if hull is degenerate
   */
  Eigen::Matrix3d computeInertiaTensorAboutOrigin(const ConvexHull& hull,
                                                   double mass);

  /**
   * Compute inertia tensor about centroid (most useful for dynamics).
   * Uses parallel axis theorem.
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
