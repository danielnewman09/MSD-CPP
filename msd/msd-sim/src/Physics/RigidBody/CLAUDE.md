# RigidBody Sub-Module Architecture Guide

> This document provides architectural context for AI assistants and developers working with the rigid body physics system.
> For collision detection, see [Collision/CLAUDE.md](../Collision/CLAUDE.md).
> For integration, see [Integration/CLAUDE.md](../Integration/CLAUDE.md).
> For constraints, see [Constraints/CLAUDE.md](../Constraints/CLAUDE.md).

## Overview

**RigidBody** is a sub-module within the Physics module that provides the core rigid body representation for dynamic objects in the simulation. It includes convex hull geometry, inertial properties calculation, quaternion-based orientation, and the complete dynamic state representation required for physics simulation.

**Diagrams**: See `docs/msd/msd-sim/Physics/` for component-specific diagrams

---

## Architecture Overview

### Component Hierarchy

The RigidBody system is organized in a layered architecture:

```
AssetInertial (Complete dynamic object)
    ├── AssetPhysical (Base: geometry + transform)
    │   ├── ConvexHull (Collision geometry)
    │   └── ReferenceFrame (Position/Orientation)
    ├── InertialState (Kinematic state)
    │   ├── Position/Velocity/Acceleration (Linear)
    │   └── Quaternion/QuaternionRate/AngularAcceleration (Angular)
    ├── Mass Properties
    │   ├── Mass
    │   ├── Inertia Tensor (via InertialCalculations)
    │   └── Center of Mass
    └── Constraints (Lagrange multipliers)
```

### Core Components

| Component | Location | Purpose | Diagram |
|-----------|----------|---------|---------|
| AssetInertial | `AssetInertial.hpp` | Complete dynamic rigid body with physics | [`force-application.puml`](../../../../../../docs/msd/msd-sim/Physics/force-application.puml) |
| AssetPhysical | `AssetPhysical.hpp` | Base geometric element (visual + collision) | [`gjk-asset-physical.puml`](../../../../../../docs/msd/msd-sim/Physics/gjk-asset-physical.puml) |
| InertialState | `InertialState.hpp` | Complete kinematic state (position, velocity, acceleration) | [`force-application.puml`](../../../../../../docs/msd/msd-sim/Physics/force-application.puml) |
| ConvexHull | `ConvexHull.hpp` | Convex hull geometry via Qhull | [`convex-hull.puml`](../../../../../../docs/msd/msd-sim/Physics/convex-hull.puml) |
| InertialCalculations | `InertialCalculations.hpp` | Inertia tensor computation (Mirtich algorithm) | [`mirtich-inertia-tensor.puml`](../../../../../../docs/msd/msd-sim/Physics/mirtich-inertia-tensor.puml) |
| AssetEnvironment | `AssetEnvironment.hpp` | Stationary environmental objects (no dynamics) | — |

---

## Component Details

### AssetInertial

**Location**: `AssetInertial.hpp`, `AssetInertial.cpp`
**Diagrams**:
- [`force-application.puml`](../../../../../../docs/msd/msd-sim/Physics/force-application.puml) — Force application and integration
- [`collision-response.puml`](../../../../../../docs/msd/msd-sim/Physics/collision-response.puml) — Collision response

#### Purpose

Complete dynamic rigid body representation for objects that respond to forces and participate in physics simulation. Extends `AssetPhysical` by adding mass properties, dynamic state, force accumulation, and constraint management.

#### Key Features

- Mass, inertia tensor, and center of mass computation
- Force and torque accumulation (cleared each physics step)
- Impulse application for collision response
- Coefficient of restitution for collision elasticity
- Constraint management (quaternion normalization, joints, etc.)
- Quaternion-based orientation (no gimbal lock)
- Integration with Lagrangian mechanics via constraints

#### Key Interfaces

```cpp
class AssetInertial : public AssetPhysical {
public:
  // Construction (move-only due to constraint ownership)
  AssetInertial(uint32_t assetId, uint32_t instanceId,
                ConvexHull& hull, double mass,
                const ReferenceFrame& frame,
                double coefficientOfRestitution = 0.5);

  // State access
  InertialState& getInertialState();
  const InertialState& getInertialState() const;

  // Mass properties
  double getMass() const;
  const Eigen::Matrix3d& getInertiaTensor() const;
  const Eigen::Matrix3d& getInverseInertiaTensor() const;

  // Force application (accumulated, cleared each frame)
  void applyForce(const CoordinateRate& force);
  void applyForceAtPoint(const CoordinateRate& force, const Coordinate& worldPoint);
  void applyTorque(const CoordinateRate& torque);
  void clearForces();
  const CoordinateRate& getAccumulatedForce() const;
  const CoordinateRate& getAccumulatedTorque() const;

  // Impulse application (instantaneous velocity changes)
  void applyImpulse(const Coordinate& impulse);
  void applyAngularImpulse(const AngularRate& angularImpulse);

  // Collision properties
  double getCoefficientOfRestitution() const;
  void setCoefficientOfRestitution(double e);

  // Constraint management
  void addConstraint(std::unique_ptr<Constraint> constraint);
  void removeConstraint(size_t index);
  std::vector<Constraint*> getConstraints();
  void clearConstraints();
  size_t getConstraintCount() const;
};
```

#### Usage Example

```cpp
// Create dynamic object
ConvexHull hull{collisionPoints};
ReferenceFrame frame{Coordinate{0, 0, 10}};
AssetInertial asset{1, 100, hull, 50.0, frame, 0.8};  // 50kg, e=0.8

// Apply forces during simulation
asset.applyForce(CoordinateRate{10, 0, 0});  // 10N in +x direction
asset.applyForceAtPoint(
  CoordinateRate{5, 0, 0},     // Force
  Coordinate{0, 0, 2.0}         // World point above center of mass
);  // Creates torque

// Integration happens via Integrator (see Integration/CLAUDE.md)
integrator->step(asset, constraints, dt);

// Clear forces after integration
asset.clearForces();

// Impulses for collision response (see Collision/CLAUDE.md)
asset.applyImpulse(Coordinate{0, 0, 100});  // 100 N·s upward impulse
```

#### Integration with Physics Pipeline

`AssetInertial` is the central entity in the physics simulation:

1. **Force Accumulation**: External forces and torques accumulated via `applyForce()`, `applyTorque()`
2. **Constraint Enforcement**: Constraints (quaternion normalization, joints) stored and passed to integrator
3. **Integration**: `Integrator::step()` computes accelerations, updates velocities/positions (see [Integration/CLAUDE.md](../Integration/CLAUDE.md))
4. **Collision Response**: Impulses applied via `applyImpulse()` and `applyAngularImpulse()` (see [Collision/CLAUDE.md](../Collision/CLAUDE.md))
5. **Force Clearing**: `clearForces()` called after integration to prepare for next frame

#### Thread Safety

**Not thread-safe** — Contains mutable state (forces, velocities, positions).

#### Error Handling

- Throws `std::invalid_argument` if mass ≤ 0
- Throws `std::invalid_argument` if coefficientOfRestitution ∉ [0, 1]
- Throws `std::invalid_argument` if hull is invalid

#### Memory Management

- **Move-only type**: Contains `std::unique_ptr<Constraint>` (prevents copy)
- **Constraint ownership**: Owns constraints via `std::vector<std::unique_ptr<Constraint>>`
- **Hull reference**: Stores const reference to ConvexHull (ownership external)
- **Inertia caching**: Inverse inertia tensor computed once and cached

---

### AssetPhysical

**Location**: `AssetPhysical.hpp`, `AssetPhysical.cpp`
**Diagram**: [`gjk-asset-physical.puml`](../../../../../../docs/msd/msd-sim/Physics/gjk-asset-physical.puml)

#### Purpose

Base class for all geometric objects in the simulation. Provides visual geometry, collision hull, and reference frame (position/orientation). Used directly for static objects (`AssetEnvironment`) and as a base for dynamic objects (`AssetInertial`).

#### Key Features

- Asset ID and instance ID tracking
- Collision hull reference for GJK/EPA collision detection
- Reference frame defining world-space transform
- Base class for both static and dynamic objects

#### Key Interfaces

```cpp
class AssetPhysical {
public:
  AssetPhysical(uint32_t assetId, uint32_t instanceId,
                ConvexHull& hull, const ReferenceFrame& frame);

  virtual ~AssetPhysical() = default;

  const ConvexHull& getCollisionHull() const;
  const ReferenceFrame& getReferenceFrame() const;
  ReferenceFrame& getReferenceFrame();
  uint32_t getAssetId() const;
  uint32_t getInstanceId() const;

protected:
  uint32_t referenceAssetId_;  // Asset type ID
  uint32_t instanceId_;         // Unique instance ID
  const ConvexHull& collisionHull_;  // Non-owning reference
  ReferenceFrame referenceFrame_;    // World-space transform
};
```

#### Usage Example

```cpp
// Typically used via derived classes
ConvexHull hull{points};
ReferenceFrame frame{Coordinate{10, 0, 0}};

// Static environment object
AssetEnvironment staticObj{1, 100, hull, frame};

// Dynamic object
AssetInertial dynamicObj{2, 101, hull, 50.0, frame};

// Access base functionality
const auto& objHull = dynamicObj.getCollisionHull();
auto& objFrame = dynamicObj.getReferenceFrame();
objFrame.setOrigin(Coordinate{20, 0, 0});  // Update position
```

#### Memory Management

- **Hull reference**: Non-owning reference (caller must ensure lifetime)
- **ReferenceFrame**: Owned by value

---

### InertialState

**Location**: `InertialState.hpp`, `InertialState.cpp`
**Tickets**:
- [0030_lagrangian_quaternion_physics](../../../../../../tickets/0030_lagrangian_quaternion_physics.md) — Quaternion representation

#### Purpose

Complete kinematic state representation for rigid body motion using quaternion-based orientation. Stores 14-component state vector: position (3), quaternion orientation (4), velocity (3), quaternion rate (4).

#### Quaternion Representation

The system uses quaternions to eliminate gimbal lock singularities present in Euler angle representations:

- **Orientation**: `Eigen::Quaterniond` (unit quaternion, |Q| = 1)
- **Angular velocity**: Stored as quaternion rate `Q̇` for efficiency
- **Conversion formulas**:
  - `ω = 2 * Q̄ ⊗ Q̇` (quaternion rate → angular velocity)
  - `Q̇ = ½ * Q ⊗ [0, ω]` (angular velocity → quaternion rate)
- **Normalization**: Maintained via `UnitQuaternionConstraint` with Baumgarte stabilization

#### Key Interfaces

```cpp
struct InertialState {
  // Linear components
  Coordinate position;
  Coordinate velocity;
  Coordinate acceleration;

  // Angular components (quaternion representation)
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};  // Identity (w,x,y,z)
  Eigen::Vector4d quaternionRate{0.0, 0.0, 0.0, 0.0};  // Q̇
  AngularRate angularAcceleration;  // α = I⁻¹ * τ

  // Conversion utilities
  AngularRate getAngularVelocity() const;
  void setAngularVelocity(const AngularRate& omega);

  static Eigen::Vector4d omegaToQuaternionRate(const AngularRate& omega,
                                                 const Eigen::Quaterniond& Q);
  static AngularRate quaternionRateToOmega(const Eigen::Vector4d& Qdot,
                                            const Eigen::Quaterniond& Q);

  // Deprecated (backward compatibility only)
  [[deprecated("Use quaternion representation directly")]]
  AngularCoordinate getEulerAngles() const;
};
```

#### Usage Example

```cpp
InertialState state;

// Set position and velocity
state.position = Coordinate{0, 0, 10};
state.velocity = Coordinate{5, 0, 0};  // 5 m/s in +x

// Set angular velocity (automatically converts to quaternion rate)
state.setAngularVelocity(AngularRate{0, 0, 1.0});  // 1 rad/s around z-axis

// Integration updates quaternion directly
// (handled by Integrator, see Integration/CLAUDE.md)

// Query angular velocity
AngularRate omega = state.getAngularVelocity();
std::cout << "Angular velocity: " << omega.transpose() << std::endl;
```

#### State Vector Structure

The 14-component state vector:
- **Position (X)**: 3 components [m]
- **Quaternion (Q)**: 4 components [dimensionless, |Q| = 1]
- **Velocity (Ẋ)**: 3 components [m/s]
- **Quaternion Rate (Q̇)**: 4 components [1/s]

Angular acceleration (α) is not part of the state vector—it's computed from torque: `α = I⁻¹ * τ`

#### Thread Safety

**Value semantics** — Safe to copy across threads after construction.

---

### ConvexHull

**Location**: `ConvexHull.hpp`, `ConvexHull.cpp`
**Diagram**: [`convex-hull.puml`](../../../../../../docs/msd/msd-sim/Physics/convex-hull.puml)

#### Purpose

3D convex hull representation for collision detection and mass property calculation. Wraps Qhull library to compute convex hulls from point clouds or geometry objects.

#### Key Features

- Construct from point clouds or `msd_assets::CollisionGeometry`
- Access to hull vertices and triangular facets
- Point containment testing and signed distance computation
- Axis-aligned bounding box for broad-phase collision
- Volume, surface area, and centroid computed by Qhull

#### Key Interfaces

```cpp
class ConvexHull {
public:
  struct BoundingBox { Coordinate min; Coordinate max; };

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

  // Properties (computed by Qhull)
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
  {0, 0, 0}, {1, 0, 0}, {0.5, 1, 0}, {0.5, 0.5, 1}  // Tetrahedron
};
ConvexHull hull{points};

// Query properties
std::cout << "Volume: " << hull.getVolume() << " m³" << std::endl;
std::cout << "Surface area: " << hull.getSurfaceArea() << " m²" << std::endl;
std::cout << "Vertices: " << hull.getVertexCount() << std::endl;
std::cout << "Facets: " << hull.getFacetCount() << std::endl;

// Point containment
Coordinate testPoint{0.5, 0.5, 0.25};
if (hull.contains(testPoint)) {
  std::cout << "Point is inside hull" << std::endl;
}

// Bounding box (for broad-phase collision)
auto bbox = hull.getBoundingBox();
double width = bbox.max.x() - bbox.min.x();

// For collision detection, see Collision/CLAUDE.md
```

#### Facet Structure

Each facet is a triangle with:
- `vertexIndices`: Indices into vertex array (3 vertices per facet)
- `normal`: Outward-facing unit normal vector
- `offset`: Plane equation offset (distance from origin along normal)

Facets are oriented such that normals point outward from the convex hull.

#### Thread Safety

**Read-only operations are thread-safe** after construction. Qhull computation is not reentrant within a single hull.

#### Error Handling

- Throws `std::runtime_error` on empty point set
- Throws `std::runtime_error` on fewer than 4 points
- Throws `std::runtime_error` on degenerate geometry (coplanar points)
- Throws `std::runtime_error` on Qhull computation failure

#### Memory Management

- **Vertex/facet storage**: Owned internally via `std::vector`
- **Qhull resources**: Cleaned up via RAII pattern in `computeHull()`

---

### InertialCalculations

**Location**: `InertialCalculations.hpp`, `InertialCalculations.cpp`
**Diagram**: [`mirtich-inertia-tensor.puml`](../../../../../../docs/msd/msd-sim/Physics/mirtich-inertia-tensor.puml)
**Tickets**:
- [0025_fix_inertia_tensor_calculation](../../../../../../tickets/0025_fix_inertia_tensor_calculation.md) — Initial scaffolding
- [0026_mirtich_inertia_tensor](../../../../../../tickets/0026_mirtich_inertia_tensor.md) — Mirtich algorithm

#### Purpose

Utility namespace for computing moment of inertia tensor from convex hull geometry, assuming uniform density. Uses Brian Mirtich's algorithm (1996) which provides machine-precision accuracy (< 1e-10 error) compared to analytical solutions.

#### Mirtich Algorithm Overview

The algorithm computes inertia through hierarchical integral computation:

1. **Projection Integrals**: 2D line integrals over face edges in projection plane
2. **Face Integrals**: 3D surface integrals lifted from projection integrals
3. **Volume Integrals**: Accumulate face integrals across all faces
4. **Inertia Computation**: Derive inertia about origin, compute center of mass, apply parallel axis theorem

**Key Implementation Detail**: Includes vertex winding correction to ensure Qhull's vertex order aligns with facet normals (required by Mirtich algorithm).

#### Key Interfaces

```cpp
namespace InertialCalculations {
  /**
   * Compute inertia tensor about centroid using Mirtich algorithm.
   *
   * @param hull Convex hull geometry
   * @param mass Total mass [kg]
   * @return 3x3 inertia tensor about centroid [kg·m²]
   *
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
double mass = 10.0;  // 10 kg

// Compute inertia about centroid
Eigen::Matrix3d I = InertialCalculations::computeInertiaTensorAboutCentroid(hull, mass);

// Use for rigid body dynamics
Eigen::Matrix3d I_inv = I.inverse();
Eigen::Vector3d torque{0, 0, 5.0};  // 5 N·m about z-axis
Eigen::Vector3d alpha = I_inv * torque;  // Angular acceleration

std::cout << "Inertia tensor:\n" << I << std::endl;
```

#### Validation

Validated against analytical solutions for standard shapes:

- **Unit cube**: `Ixx = Iyy = Izz = m/6` (exact within 1e-10)
- **Rectangular box** (2×3×4): Matches analytical formulas (exact within 1e-10)
- **Regular tetrahedron**: Diagonal elements equal (exact within 1e-10)
- **Volume byproduct**: Computed volume matches `ConvexHull::getVolume()` within 1e-10

**Previous implementation**: Tetrahedron decomposition had ~10-15% error with ad-hoc scaling factor.

#### Performance

O(F) complexity where F = number of facets. Higher constant factor than tetrahedron decomposition (~50-100 operations per facet vs ~10-20), but negligible for typical hulls (10-100 facets) since inertia calculation is one-time at object creation.

---

### AssetEnvironment

**Location**: `AssetEnvironment.hpp`, `AssetEnvironment.cpp`

#### Purpose

Stationary environmental objects with no dynamics. Provides collision geometry and visual representation but does not respond to forces or participate in physics simulation.

#### Key Features

- Extends `AssetPhysical` (hull + reference frame)
- No mass properties or dynamic state
- Used for terrain, buildings, obstacles
- Participates in collision detection but not collision response

#### Usage Example

```cpp
ConvexHull groundHull{groundPoints};
ReferenceFrame groundFrame{Coordinate{0, 0, 0}};
AssetEnvironment ground{1, 200, groundHull, groundFrame};

// Can be used for collision detection
CollisionHandler handler{1e-6};
auto result = handler.checkCollision(dynamicObj, ground);
if (result) {
  // Dynamic object collided with static environment
  // Apply collision response only to dynamic object
}
```

---

## Cross-Cutting Concerns

### Quaternion-Based Orientation

**Motivation**: Euler angles suffer from gimbal lock singularities when pitch approaches ±90°. Quaternions provide singularity-free rotation representation.

**Implementation**:
- `InertialState::orientation` stores unit quaternion `Q` (|Q| = 1)
- `InertialState::quaternionRate` stores `Q̇` directly (avoids conversion every frame)
- `UnitQuaternionConstraint` maintains normalization via Baumgarte stabilization
- Conversions between ω and Q̇ provided via utility functions

**Benefits**:
- No gimbal lock at any orientation
- Numerically stable (|Q| maintained within 1e-10 over 10000+ steps)
- Efficient (direct Q̇ storage avoids repeated ω→Q̇ conversions)

### Mass Property Computation

**Inertia Tensor**:
- Computed once at object creation via Mirtich algorithm
- Stored in `AssetInertial` along with inverse (cached for performance)
- Assumed uniform density across convex hull
- Machine-precision accuracy (< 1e-10 relative error)

**Center of Mass**:
- Computed by Qhull during hull construction
- Used as origin for torque calculation: `τ = r × F` where `r = applicationPoint - centerOfMass`

### Force Accumulation Pattern

Forces and torques are accumulated during each frame:

1. **Application**: `applyForce()`, `applyForceAtPoint()`, `applyTorque()` accumulate into member variables
2. **Integration**: Integrator reads accumulated forces, computes accelerations, updates state
3. **Clearing**: `clearForces()` resets accumulators to zero for next frame

**Rationale**: Multiple systems (gravity, user input, constraints) can contribute forces independently before integration.

### Impulse vs. Force Application

**Forces** (accumulated, time-integrated):
- Applied via `applyForce()`, `applyTorque()`
- Integrated over timestep: `Δv = (F/m) * dt`
- Used for continuous forces (gravity, drag, thrust)

**Impulses** (instantaneous velocity changes):
- Applied via `applyImpulse()`, `applyAngularImpulse()`
- Directly modify velocity: `Δv = J/m`
- Timestep-independent
- Used for collision response (see [Collision/CLAUDE.md](../Collision/CLAUDE.md))

### Units

All quantities use SI units:
- Mass: kilograms [kg]
- Force: Newtons [N]
- Torque: Newton-meters [N·m]
- Impulse: Newton-seconds [N·s]
- Angular impulse: Newton-meter-seconds [N·m·s]
- Position: meters [m]
- Velocity: meters per second [m/s]
- Acceleration: meters per second squared [m/s²]
- Angular velocity: radians per second [rad/s]
- Angular acceleration: radians per second squared [rad/s²]
- Inertia: kilogram-meters squared [kg·m²]

---

## Integration with Physics Pipeline

The RigidBody system integrates with other physics subsystems:

### Lagrangian Mechanics

**See**: [Integration/CLAUDE.md](../Integration/CLAUDE.md)

`AssetInertial` objects are advanced each frame via:
```cpp
integrator->step(asset, constraints, timestep);
```

The integrator:
1. Computes unconstrained accelerations from forces
2. Solves constraint system via `ConstraintSolver`
3. Applies total accelerations (forces + constraint forces)
4. Updates velocities and positions
5. Enforces constraints via Baumgarte stabilization

### Constraint System

**See**: [Constraints/CLAUDE.md](../Constraints/CLAUDE.md)

Every `AssetInertial` owns a vector of constraints:
- Default: `UnitQuaternionConstraint` (quaternion normalization)
- Extensible: Add custom constraints via `addConstraint()`
- Enforced automatically during integration

### Collision Detection

**See**: [Collision/CLAUDE.md](../Collision/CLAUDE.md)

`AssetPhysical` provides hull and transform for GJK/EPA:
```cpp
CollisionHandler handler{1e-6};
auto result = handler.checkCollision(assetA, assetB);
if (result) {
  // Use result for collision response
}
```

### Collision Response

**See**: [Collision/CLAUDE.md](../Collision/CLAUDE.md)

Collision response uses impulses for timestep-independent velocity changes:
```cpp
assetA.applyImpulse(impulse);
assetA.applyAngularImpulse(angularImpulse);
```

---

## Design Patterns

### Inheritance Hierarchy

**Pattern**: Single inheritance with virtual destructor
- `AssetPhysical` (base) → `AssetInertial` (derived)
- `AssetPhysical` (base) → `AssetEnvironment` (derived)

**Purpose**: Separate static objects (no dynamics) from dynamic objects (full physics).

### Composition Over Inheritance

`AssetInertial` composes:
- `ConvexHull` (reference)
- `InertialState` (value)
- `ReferenceFrame` (value)
- `std::vector<std::unique_ptr<Constraint>>` (ownership)

**Purpose**: Clear ownership and responsibilities.

### Value Semantics

`InertialState` uses value semantics (POD-like struct):
- Copyable
- No heap allocation
- Efficient for small types
- Safe to copy across threads

### RAII

`ConvexHull::computeHull()` uses RAII for Qhull resource management:
- Qhull context created on stack
- Automatic cleanup on exception or normal return
- No manual resource management needed

### Move-Only Type

`AssetInertial` is move-only due to constraint ownership:
- Copy constructor/assignment deleted
- Move constructor defaulted
- Move assignment deleted (base class has reference member)

---

## Error Handling

### Validation

- **Construction**: Parameters validated in constructor (mass > 0, e ∈ [0,1])
- **Qhull**: Degeneracies detected and reported via exceptions
- **Constraint indices**: Bounds checking with `std::out_of_range`

### Exception Types

- `std::invalid_argument`: Invalid parameters (mass, restitution, hull)
- `std::runtime_error`: Qhull computation failures
- `std::out_of_range`: Constraint index out of bounds

### Graceful Degradation

- Empty hulls detected and rejected during construction
- Degenerate geometry (< 4 points, coplanar) rejected by Qhull

---

## Thread Safety

- **ConvexHull**: Read-only operations thread-safe after construction
- **InertialState**: Value semantics, safe to copy across threads
- **AssetInertial**: Not thread-safe (mutable state)
- **InertialCalculations**: Pure functions, thread-safe

---

## Memory Management

### Ownership Model

| Component | Ownership |
|-----------|-----------|
| `ConvexHull` vertices/facets | Owned by value (`std::vector`) |
| `AssetInertial` constraints | Owned via `std::unique_ptr` |
| `AssetPhysical` hull reference | Non-owning reference (external lifetime) |
| `InertialState` | Owned by value in `AssetInertial` |
| Inertia tensors | Cached by value in `AssetInertial` |

### Lifetime Management

- **Hull lifetime**: Caller ensures hull outlives `AssetPhysical` reference
- **Constraint lifetime**: Owned by `AssetInertial`, destroyed with object
- **Qhull resources**: Scoped RAII cleanup in `computeHull()`

---

## Testing

### Test Organization

```
test/Physics/
├── ConvexHullTest.cpp           # Hull computation, properties, containment
├── InertialCalculationsTest.cpp # Mirtich algorithm validation
├── AssetInertialTest.cpp        # Force application, impulses, constraints
└── RigidBodyIntegrationTest.cpp # End-to-end physics integration
```

### Running Tests

```bash
cmake --build --preset debug-sim-only --target msd_sim_test
./build/Debug/msd_sim_test
```

---

## Coding Standards

This sub-module follows the project-wide coding standards defined in the [root CLAUDE.md](../../../../../../CLAUDE.md#coding-standards).

Key standards applied:
- **Initialization**: Brace initialization `{}`, `NaN` for uninitialized floats
- **Rule of Five**: Explicit special member functions or `= default`
- **Naming**: `PascalCase` classes, `camelCase` methods, `snake_case_` members
- **Return Values**: Return structs (BoundingBox) over output parameters
- **Memory**: Value semantics for state, `std::unique_ptr` for constraints

See the [root CLAUDE.md](../../../../../../CLAUDE.md#coding-standards) for complete details.

---

## Getting Help

### For AI Assistants

1. This document provides complete context for the RigidBody sub-module
2. Review [Integration/CLAUDE.md](../Integration/CLAUDE.md) for numerical integration
3. Review [Constraints/CLAUDE.md](../Constraints/CLAUDE.md) for constraint system
4. Review [Collision/CLAUDE.md](../Collision/CLAUDE.md) for collision detection
5. Review [Physics/CLAUDE.md](../CLAUDE.md) for overall physics architecture
6. Check [root CLAUDE.md](../../../../../../CLAUDE.md) for project conventions

### For Developers

- **Convex hulls**: Start with `ConvexHull` class
- **Dynamic objects**: Use `AssetInertial` factory methods
- **Static objects**: Use `AssetEnvironment`
- **Inertia computation**: Use `InertialCalculations::computeInertiaTensorAboutCentroid()`
- **Quaternion orientation**: Access via `InertialState::orientation`
- **Force application**: Use `applyForce()`, `applyForceAtPoint()`, `applyTorque()`
- **Collision response**: Use `applyImpulse()`, `applyAngularImpulse()`
