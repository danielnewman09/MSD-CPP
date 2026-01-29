# Environment Module Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/msd/msd-sim/` for detailed component relationships.

## Project Overview

**Environment** is a core module within `msd-sim` that provides the foundational mathematical primitives and coordinate transformation system for simulation. It includes type-safe angle handling, 3D coordinates, kinematic state representation, reference frame transformations, and unified object management.

## Architecture Overview

### High-Level Architecture

The Environment module provides a layered architecture of mathematical types:

```
WorldModel (Container)
    └── Object (Unified entity type)
        ├── ReferenceFrame (Position/Orientation)
        │   ├── Coordinate (3D position)
        │   └── EulerAngles (Orientation)
        │       └── Angle (Type-safe angle)
        └── Optional components (Physics, Collision)

Platform (Legacy entity type)
    └── InertialState (Kinematics)
        ├── Coordinate (Position/Velocity/Acceleration)
        └── EulerAngles (Angular state)
```

### Core Components

| Component | Location | Purpose | Diagram |
|-----------|----------|---------|---------|
| Coordinate | `Coordinate.hpp` | 3D coordinate wrapper (Eigen) | [`mathematical-primitives.puml`](../../../../../docs/msd/msd-sim/Environment/mathematical-primitives.puml) |
| Angle | `Angle.hpp` | Type-safe angle with normalization | [`mathematical-primitives.puml`](../../../../../docs/msd/msd-sim/Environment/mathematical-primitives.puml) |
| AngularCoordinate | `AngularCoordinate.hpp` | Orientation angles with deferred normalization | [`angular-coordinate.puml`](../../../../../docs/msd/msd-sim/Environment/angular-coordinate.puml) |
| AngularRate | `AngularRate.hpp` | Angular velocity/acceleration without normalization | [`angular-coordinate.puml`](../../../../../docs/msd/msd-sim/Environment/angular-coordinate.puml) |
| InertialState | `InertialState.hpp` | Complete kinematic state | [`mathematical-primitives.puml`](../../../../../docs/msd/msd-sim/Environment/mathematical-primitives.puml) |
| ReferenceFrame | `ReferenceFrame.hpp` | Coordinate transformations | [`reference-frame.puml`](../../../../../docs/msd/msd-sim/Environment/reference-frame.puml) |
| Object | `Object.hpp` | Unified simulation entity | [`object.puml`](../../../../../docs/msd/msd-sim/Environment/object.puml) |
| WorldModel | `WorldModel.hpp` | Simulation container | [`world-model.puml`](../../../../../docs/msd/msd-sim/Environment/world-model.puml) |
| Platform | `Platform.hpp` | Legacy entity type | [`world-model.puml`](../../../../../docs/msd/msd-sim/Environment/world-model.puml) |

---

## Component Details

### Coordinate

**Location**: `Coordinate.hpp`
**Type**: Header-only, value type

#### Purpose
3D coordinate wrapper inheriting from `Eigen::Vector3d`. Provides convenient construction and full access to Eigen's optimized linear algebra operations with double precision for numerical stability.

#### Key Interfaces
```cpp
class Coordinate : public Eigen::Vector3d {
  Coordinate();                                    // Default: (0, 0, 0)
  Coordinate(double x, double y, double z);        // Direct construction
  Coordinate(const Eigen::Vector3d& vec);          // From Eigen vector

  // Inherits all Eigen::Vector3d operations
  double x(), y(), z();
  double norm(), normalized();
  double dot(other);
  Coordinate cross(other);
};
```

#### Usage Example
```cpp
Coordinate position{100.0, 200.0, 300.0};
Coordinate velocity{1.0, 0.0, 0.0};
Coordinate newPosition = position + velocity * deltaTime;

// Supports std::format
std::cout << std::format("Position: {:.2f}", position);
// Output: Position: (100.00, 200.00, 300.00)
```

#### Thread Safety
**Immutable after creation** — Value semantics make it safe to copy across threads.

#### Memory Management
Value type with Eigen internal storage. Copy-efficient via Eigen optimizations.

---

### Angle

**Location**: `Angle.hpp`
**Type**: Header-only, value type

#### Purpose
Type-safe angle representation with automatic lazy normalization. Stores angles internally in radians and provides two normalization modes:
- `PI`: Normalizes to (-π, π]
- `TWO_PI`: Normalizes to [0, 2π)

#### Key Interfaces
```cpp
class Angle {
  enum class Norm { PI, TWO_PI };

  // Factory methods (preferred)
  static Angle fromRadians(double radians, Norm norm = Norm::PI);
  static Angle fromDegrees(double degrees, Norm norm = Norm::PI);

  // Access (applies lazy normalization)
  double getRad() const;
  double toDeg() const;

  // Full arithmetic operators
  Angle operator+(const Angle& other) const;
  Angle operator-(const Angle& other) const;
  Angle operator*(double scalar) const;
  Angle operator/(double scalar) const;
  Angle operator-() const;  // Unary negation

  // Compound assignment
  Angle& operator+=(const Angle& other);
  Angle& operator-=(const Angle& other);
};
```

#### Usage Example
```cpp
// Create angles
Angle heading = Angle::fromDegrees(350);
Angle turn = Angle::fromDegrees(30);

// Arithmetic preserves normalization mode
Angle newHeading = heading + turn;  // 380° → normalized to 20°
double radians = newHeading.getRad();  // ≈ 0.349 rad
double degrees = newHeading.toDeg();   // 20°

// Different normalization modes
Angle azimuth = Angle::fromDegrees(-30, Angle::Norm::TWO_PI);
azimuth.getRad();  // Returns 330° equivalent in radians
```

#### Thread Safety
**Immutable after creation** — Value semantics, safe to copy across threads.

#### Design Decisions
- **Lazy Normalization**: Normalization is applied on access (`getRad()`, `toDeg()`), not on every operation. This reduces redundant computation in arithmetic chains.
- **Internal storage**: Always radians, regardless of construction method.

---

### AngularCoordinate

**Location**: `AngularCoordinate.hpp`
**Type**: Header-only, value type
**Diagram**: [`angular-coordinate.puml`](../../../../../docs/msd/msd-sim/Environment/angular-coordinate.puml)
**Introduced**: [Ticket: 0024_angular_coordinate](../../../../../tickets/0024_angular_coordinate.md)

#### Purpose
Represents orientation angles with deferred normalization to (-π, π]. Inherits from `Eigen::Vector3d` for full matrix operation support while providing semantic pitch/roll/yaw accessors. Normalization is checked only when values exceed 100π threshold (~50 revolutions), making it 10x faster than eager normalization while maintaining correctness.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `AngularCoordinate` | `AngularCoordinate.hpp` | Orientation angle representation with deferred normalization |

#### Key Interfaces
```cpp
class AngularCoordinate : public Eigen::Vector3d {
public:
  static constexpr double kNormalizationThreshold = 100.0 * M_PI;

  // Construction
  AngularCoordinate();
  AngularCoordinate(double pitch, double roll, double yaw);
  AngularCoordinate(const Eigen::Vector3d& vec);
  template <typename OtherDerived>
  AngularCoordinate(const Eigen::MatrixBase<OtherDerived>& other);

  // Fast accessors (no normalization check - returns raw value)
  double pitch() const;
  double roll() const;
  double yaw() const;

  // Degree accessors
  double pitchDeg() const;
  double rollDeg() const;
  double yawDeg() const;

  // Setters with normalization check
  void setPitch(double radians);
  void setRoll(double radians);
  void setYaw(double radians);

  // Explicit normalization
  AngularCoordinate normalized() const;
  void normalize();

  // Compound operators (override to include normalization check)
  AngularCoordinate& operator+=(const Eigen::MatrixBase<OtherDerived>& other);
  AngularCoordinate& operator-=(const Eigen::MatrixBase<OtherDerived>& other);
  AngularCoordinate& operator*=(double scalar);
  AngularCoordinate& operator/=(double scalar);
};
```

#### Usage Example
```cpp
// Construction
AngularCoordinate orientation{0.0, 0.0, M_PI/4};  // 45° yaw

// Fast accessors (0% overhead, validated by prototypes)
double yaw = orientation.yaw();      // Radians
double yawDeg = orientation.yawDeg(); // Degrees

// Setters with normalization check
orientation.setYaw(3 * M_PI);  // Large value stored
// Normalizes only if exceeds 100π threshold

// Eigen operations preserved
AngularCoordinate delta{0.1, 0.0, 0.0};
orientation += delta;  // Compound operator includes normalization check

// Explicit normalization (if needed)
AngularCoordinate normalized = orientation.normalized();

// Supports std::format
std::cout << std::format("Orientation: {:.2f}", orientation);
// Output: Orientation: (0.00, 0.00, 0.79)
```

#### Axis Convention
ZYX intrinsic rotation convention:
- **pitch**: Rotation around Y-axis (component 0)
- **roll**: Rotation around X-axis (component 1)
- **yaw**: Rotation around Z-axis (component 2)

#### Normalization Strategy
**Deferred normalization** with 100π threshold:
- Normalization check occurs in ALL modifying operations (constructors, assignment, compound operators)
- Values are normalized only when `|value| > 100π` (~50 revolutions)
- Accessors return raw values without normalization check (0% overhead)
- 10x faster than eager-always normalization in typical physics workloads (validated by prototypes P1-P1e)

**Accepted Limitation**: Direct `operator[]` access bypasses normalization. Use semantic accessors (pitch(), roll(), yaw()) for normal use.

#### Thread Safety
**Immutable after creation** — Value semantics make it safe to copy across threads.

#### Error Handling
No exceptions thrown. Normalization handles all input ranges via `std::fmod`.

#### Memory Management
Pure value type inheriting from `Eigen::Vector3d`. Memory footprint: 24 bytes (same as `Eigen::Vector3d`). No dynamic allocation.

#### Dependencies
- `Eigen3` — Base class and linear algebra operations
- `Angle.hpp` — Reference for normalization algorithm (not used internally)
- `Coordinate.hpp` — Pattern for Eigen inheritance and formatting

#### Design Decisions
- **Replaces EulerAngles**: Type-safe separation of orientation (AngularCoordinate) vs rates (AngularRate)
- **Deferred vs Eager**: Prototypes showed 10x speedup with deferred normalization
- **Raw double storage**: 43x faster than `Angle` object storage, 50% smaller memory footprint
- **Explicit duplication**: No shared base class with AngularRate for maximum simplicity and performance

---

### AngularRate

**Location**: `AngularRate.hpp`
**Type**: Header-only, value type
**Diagram**: [`angular-coordinate.puml`](../../../../../docs/msd/msd-sim/Environment/angular-coordinate.puml)
**Introduced**: [Ticket: 0024_angular_coordinate](../../../../../tickets/0024_angular_coordinate.md)

#### Purpose
Represents angular velocity or acceleration without normalization. Inherits from `Eigen::Vector3d` for full matrix operation support. Angular rates can exceed 2π rad/s and should NOT be normalized (e.g., 720°/s is distinct from 0°/s).

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `AngularRate` | `AngularRate.hpp` | Angular velocity/acceleration representation without normalization |

#### Key Interfaces
```cpp
class AngularRate : public Eigen::Vector3d {
public:
  // Construction
  AngularRate();
  AngularRate(double pitchRate, double rollRate, double yawRate);
  AngularRate(const Eigen::Vector3d& vec);
  template <typename OtherDerived>
  AngularRate(const Eigen::MatrixBase<OtherDerived>& other);

  // Semantic accessors (NO normalization)
  double& pitch();
  double& roll();
  double& yaw();
  double pitch() const;
  double roll() const;
  double yaw() const;
};
```

#### Usage Example
```cpp
// Angular rates can exceed 2π
AngularRate angularVelocity{0.0, 0.0, 4 * M_PI};  // 720°/s yaw rate
double yawRate = angularVelocity.yaw();  // Returns 4π (NOT normalized!)

// Eigen operations work normally
AngularRate angularAccel = angularVelocity * 2.0;
double magnitude = angularVelocity.norm();

// Cross product for torque calculation
AngularRate torque = offsetPosition.cross(force);

// Supports std::format
std::cout << std::format("Angular velocity: {:.2f}", angularVelocity);
// Output: Angular velocity: (0.00, 0.00, 12.57)
```

#### Units
- **Angular velocity**: rad/s
- **Angular acceleration**: rad/s²

#### Thread Safety
**Immutable after creation** — Value semantics make it safe to copy across threads.

#### Error Handling
No exceptions thrown. No normalization means no invalid input.

#### Memory Management
Pure value type inheriting from `Eigen::Vector3d`. Memory footprint: 24 bytes (same as `Eigen::Vector3d`). No dynamic allocation.

#### Dependencies
- `Eigen3` — Base class and linear algebra operations
- `Coordinate.hpp` — Pattern for Eigen inheritance and formatting

#### Design Decisions
- **Type Safety**: Separate type from AngularCoordinate prevents accidental assignment of rates to orientations (compile-time error)
- **No Normalization**: Rates should never be normalized - 720°/s is a valid rate distinct from 0°/s
- **Performance**: 0% overhead compared to raw `Eigen::Vector3d` (validated by prototypes)

---

### InertialState

**Location**: `InertialState.hpp`, `InertialState.cpp`
**Type**: Library component with value semantics
**Modified**:
- [Ticket: 0024_angular_coordinate](../../../../../tickets/0024_angular_coordinate.md)
- [Ticket: 0030_lagrangian_quaternion_physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md) — Breaking change to quaternion representation

#### Purpose
Complete kinematic state representation with 6 degrees of freedom. Contains position, velocity, and acceleration for both linear and angular motion. Uses quaternion-based orientation to eliminate gimbal lock singularities.

#### Key Interfaces
```cpp
struct InertialState {
  // Linear components
  Coordinate position;
  Coordinate velocity;
  Coordinate acceleration;

  // Angular components (quaternion-based)
  Eigen::Quaterniond orientation;      // Was: AngularCoordinate
  Eigen::Vector4d quaternionRate;      // New: Q̇ (quaternion time derivative)
  AngularRate angularVelocity;         // ω (angular velocity vector)
  AngularRate angularAcceleration;     // α (angular acceleration vector)

  // Conversion utilities (ticket 0030)
  AngularRate quaternionRateToOmega() const;
  static Eigen::Vector4d omegaToQuaternionRate(const AngularRate& omega,
                                                const Eigen::Quaterniond& Q);

  // Deprecated accessor for backward compatibility
  [[deprecated("Use orientation (Eigen::Quaterniond) instead")]]
  AngularCoordinate getEulerAngles() const;
};
```

#### State Vector Components
The state now uses a 14-component representation:
```
Position-level:  q = [X, Q]ᵀ     (7 components: 3 position + 4 quaternion)
Velocity-level:  q̇ = [Ẋ, Q̇]ᵀ    (7 components: 3 linear vel + 4 quat rate)
```

#### Usage Example
```cpp
InertialState state;
state.position = Coordinate{100.0, 200.0, 300.0};
state.velocity = Coordinate{1.0, 0.0, 0.0};

// Quaternion orientation (no gimbal lock)
state.orientation = Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};  // Identity (w, x, y, z)

// Angular velocity (ω) for physics
state.angularVelocity = AngularRate{0.0, 0.0, 0.5};  // 0.5 rad/s yaw rate

// Quaternion rate (Q̇) for integration
state.quaternionRate = InertialState::omegaToQuaternionRate(
    state.angularVelocity, state.orientation);
```

#### Quaternion ↔ Angular Velocity Conversions
```cpp
// ω → Q̇ conversion (for integration)
AngularRate omega{0.0, 0.0, 1.0};  // 1 rad/s yaw
Eigen::Quaterniond Q{1.0, 0.0, 0.0, 0.0};  // Identity
Eigen::Vector4d Qdot = InertialState::omegaToQuaternionRate(omega, Q);
// Result: Q̇ = ½ * Q ⊗ [0, ω]

// Q̇ → ω conversion (for physics/rendering)
state.quaternionRate = Qdot;
AngularRate omega_recovered = state.quaternionRateToOmega();
// Result: ω = 2 * Q̄ ⊗ Q̇  (where Q̄ is conjugate)
```

#### Migration from Previous Version
Breaking change from ticket 0030_lagrangian_quaternion_physics:
- **orientation**: Type changed from `AngularCoordinate` to `Eigen::Quaterniond`
  - Old: `state.orientation.yaw()`
  - New: `state.orientation.toRotationMatrix()` or use deprecated `state.getEulerAngles().yaw()`
- **quaternionRate**: New member (Eigen::Vector4d) representing Q̇
  - Must be updated during integration: `state.quaternionRate = omegaToQuaternionRate(omega, Q)`
- **State size**: Increased from 13 to 14 components (added quaternionRate)
- **No gimbal lock**: Quaternions have no singularities at any orientation

#### Type Safety Benefits
The quaternion representation prevents gimbal lock:
```cpp
// Euler angles have gimbal lock at pitch = ±90°
AngularCoordinate euler{M_PI/2, 0.0, 0.0};  // Gimbal lock!

// Quaternions have no singularities
Eigen::Quaterniond quat = Eigen::AngleAxisd{M_PI/2, Eigen::Vector3d::UnitX()};
// Remains valid, no numerical instability
```

#### Thread Safety
**Value semantics** — Safe to copy across threads after construction.

#### Memory Management
- 14 component state (3×3 Coordinate + 4 quaternion + 4 quaternion rate + 2×3 AngularRate = 14 doubles)
- 112 bytes total (8 bytes per double)
- Quaternion conversion utilities are static methods (no state)

---

### ReferenceFrame

**Location**: `ReferenceFrame.hpp`, `ReferenceFrame.cpp`
**Type**: Library component
**Modified**:
- [Ticket: 0024_angular_coordinate](../../../../../tickets/0024_angular_coordinate.md)
- [Ticket: 0030_lagrangian_quaternion_physics](../../../../../tickets/0030_lagrangian_quaternion_physics.md) — Breaking change to quaternion storage

#### Purpose
Manages coordinate transformations between reference frames using translation and rotation. Provides efficient batch transformations using Eigen matrix operations. Stores orientation as `Eigen::Quaterniond` to eliminate gimbal lock in transformations.

#### Key Interfaces
```cpp
class ReferenceFrame {
  ReferenceFrame();
  ReferenceFrame(const Coordinate& origin);
  ReferenceFrame(const Coordinate& origin, const Eigen::Quaterniond& quaternion);

  // Deprecated constructor
  [[deprecated("Use ReferenceFrame(origin, Eigen::Quaterniond) instead")]]
  ReferenceFrame(const Coordinate& origin, const AngularCoordinate& angular);

  // Single coordinate transforms
  Coordinate globalToLocal(const Coordinate& globalCoord) const;
  Coordinate localToGlobal(const Coordinate& localCoord) const;
  void globalToLocalInPlace(Coordinate& globalCoord) const;
  void localToGlobalInPlace(Coordinate& localCoord) const;

  // Batch transforms (efficient for large point clouds)
  void globalToLocalBatch(Eigen::Matrix3Xd& globalCoords) const;
  void localToGlobalBatch(Eigen::Matrix3Xd& localCoords) const;

  // Relative transforms (rotation only, for direction vectors)
  Coordinate globalToLocalRelative(const Coordinate& globalVector) const;
  Coordinate localToGlobalRelative(const Coordinate& localVector) const;

  // Setters
  void setOrigin(const Coordinate& origin);
  void setQuaternion(const Eigen::Quaterniond& quaternion);  // New in ticket 0030

  [[deprecated("Use setQuaternion(Eigen::Quaterniond) instead")]]
  void setRotation(const AngularCoordinate& angular);

  // Getters
  Coordinate& getOrigin();
  const Coordinate& getOrigin() const;
  Eigen::Quaterniond getQuaternion() const;  // New in ticket 0030
  const Eigen::Matrix3d& getRotation() const;

  [[deprecated("Use getQuaternion() instead")]]
  AngularCoordinate getAngularCoordinate() const;
};
```

#### Usage Example
```cpp
// Create reference frame with quaternion
ReferenceFrame localFrame;
localFrame.setOrigin(Coordinate{10, 20, 30});

// Set rotation using quaternion (no gimbal lock)
Eigen::Quaterniond rotation = Eigen::AngleAxisd{M_PI/2, Eigen::Vector3d::UnitZ()};
localFrame.setQuaternion(rotation);

// Single coordinate transform
Coordinate globalPoint{100, 200, 300};
Coordinate localPoint = localFrame.globalToLocal(globalPoint);

// Batch transform (10x+ speedup for large point clouds)
Eigen::Matrix3Xd points(3, 1000);  // 1000 points
// ... populate points ...
localFrame.globalToLocalBatch(points);  // In-place transformation

// Access quaternion directly
Eigen::Quaterniond quat = localFrame.getQuaternion();
```

#### Migration from Previous Version
Breaking change from ticket 0030_lagrangian_quaternion_physics:
- **Constructor**: `ReferenceFrame(origin, AngularCoordinate)` → `ReferenceFrame(origin, Eigen::Quaterniond)` (old constructor deprecated)
- **Setter**: `setRotation(AngularCoordinate)` → `setQuaternion(Eigen::Quaterniond)` (old setter deprecated)
- **Getter**: `getAngularCoordinate()` → `getQuaternion()` (old getter deprecated)
- **Internal storage**: `AngularCoordinate angular_` → `Eigen::Quaterniond quaternion_`

Old code:
```cpp
AngularCoordinate angles{0.0, 0.0, M_PI/2};  // Radians
frame.setRotation(angles);
```

New code:
```cpp
Eigen::Quaterniond quat = Eigen::AngleAxisd{M_PI/2, Eigen::Vector3d::UnitZ()};
frame.setQuaternion(quat);
```

#### Transformation Pipeline
```
Object vertices (local frame)
    ↓ localToGlobal (object's ReferenceFrame)
World coordinates (global frame)
    ↓ globalToViewer (camera's ReferenceFrame)
Viewer coordinates (camera frame)
    ↓ projectTo2D (perspective projection)
Screen coordinates (2D pixels)
```

#### Rotation Matrix Update
The internal rotation matrix is lazily computed from the quaternion:
```cpp
void ReferenceFrame::updateRotationMatrix() {
  if (!updated_) {
    rotation_ = quaternion_.toRotationMatrix();  // Eigen's efficient conversion
    updated_ = true;
  }
}
```

#### Thread Safety
**Not thread-safe** — The rotation matrix is lazily computed and cached. Concurrent access to a single instance is not safe.

#### Memory Management
- Stores origin coordinate (24 bytes)
- Stores quaternion (32 bytes: 4 doubles)
- Caches rotation matrix (72 bytes: 9 doubles)
- Caches inverse rotation matrix (72 bytes)
- Total: ~200 bytes per instance

#### Design Notes
- **No gimbal lock**: Quaternion storage eliminates singularities at ±90° pitch
- **Backward compatibility**: Deprecated methods provide Euler angle conversion for legacy code
- **Efficient transforms**: Rotation matrix cached for repeated transformations

---

### Object

**Location**: `Object.hpp`, `Object.cpp`
**Type**: Library component

#### Purpose
Unified object type supporting four distinct simulation roles through a component-based design:

1. **Graphical**: Visual rendering only (no physics or collision)
2. **Inertial**: Full dynamic physics simulation
3. **Environmental**: Static collision objects
4. **Boundary**: Invisible collision-only boundaries

#### Key Interfaces
```cpp
class Object {
  enum class Type : uint8_t { Graphical, Inertial, Environmental, Boundary };

  // Factory methods (preferred construction)
  static Object createGraphical(const msd_assets::Asset& asset,
                                const ReferenceFrame& frame = ReferenceFrame(),
                                float r = 1.0f, float g = 1.0f, float b = 1.0f);

  static Object createInertial(const msd_assets::Asset& asset,
                               const ReferenceFrame& frame,
                               double mass,
                               float r = 1.0f, float g = 1.0f, float b = 1.0f);

  static Object createEnvironmental(const msd_assets::Asset& asset,
                                    const ReferenceFrame& frame = ReferenceFrame(),
                                    float r = 1.0f, float g = 1.0f, float b = 1.0f);

  static Object createBoundary(const ConvexHull& collisionHull,
                               const ReferenceFrame& frame = ReferenceFrame());

  // Component queries
  Type getType() const;
  bool hasVisualGeometry() const;
  bool hasCollision() const;
  bool hasPhysics() const;

  // Component access (throws if not present)
  const ConvexHull& getCollisionHull() const;
  const PhysicsComponent& getPhysics() const;
  PhysicsComponent& getPhysics();

  // Transform access
  ReferenceFrame& getTransform();
  const Coordinate& getPosition() const;
  void setPosition(const Coordinate& position);
};
```

#### Usage Example
```cpp
// Graphical-only object (background decoration)
auto decoration = Object::createGraphical(asset, frame, 0.5f, 0.5f, 0.5f);

// Dynamic physics object (interactable)
auto player = Object::createInertial(asset, frame, 10.0, 1.0f, 0.0f, 0.0f);

// Static collision object (wall, floor)
auto wall = Object::createEnvironmental(asset, frame, 0.3f, 0.3f, 0.3f);

// Invisible boundary
ConvexHull boundaryHull{boundaryPoints};
auto boundary = Object::createBoundary(boundaryHull, frame);
```

#### Thread Safety
**Not thread-safe** — Physics components and transforms are mutable.

#### Memory Management
- **Asset reference**: Optional reference to external asset (not owned)
- **Transform**: Owned ReferenceFrame
- **Optional components**: `std::optional<PhysicsComponent>`, `std::optional<ConvexHull>`

---

### WorldModel

**Location**: `WorldModel.hpp`, `WorldModel.cpp`
**Type**: Library component

#### Purpose
Container and manager for all simulation objects. Uses unified storage with cached index lists for efficient iteration over specific object types.

#### Key Interfaces
```cpp
class WorldModel {
  // Object management
  size_t spawnObject(Object&& object);
  const Object& getObject(size_t index) const;
  Object& getObject(size_t index);
  void removeObject(size_t index);
  size_t getObjectCount() const;
  void clearObjects();

  // Efficient iteration helpers
  const std::vector<size_t>& getPhysicsObjectIndices() const;
  const std::vector<size_t>& getCollisionObjectIndices() const;
  const std::vector<size_t>& getRenderObjectIndices() const;

  // Simulation update
  void update(std::chrono::milliseconds deltaTime);
  std::chrono::milliseconds getTime() const;

  // Legacy platform support
  void addPlatform(Platform&& platform);
  const std::vector<Platform>& getPlatforms() const;
};
```

#### Usage Example
```cpp
WorldModel world;

// Spawn objects
size_t playerIdx = world.spawnObject(Object::createInertial(asset, frame, 10.0));
size_t wallIdx = world.spawnObject(Object::createEnvironmental(wallAsset, wallFrame));

// Efficient iteration patterns
for (size_t idx : world.getPhysicsObjectIndices()) {
  Object& obj = world.getObject(idx);
  // Physics update...
}

for (size_t idx : world.getCollisionObjectIndices()) {
  // Collision detection...
}

for (size_t idx : world.getRenderObjectIndices()) {
  // Render...
}

// Time-stepped simulation
world.update(std::chrono::milliseconds{16});  // ~60 FPS
```

#### Thread Safety
**Not thread-safe** — Single-threaded simulation assumed.

#### Memory Management
- Owns all Objects via `std::vector<Object>`
- Maintains cached index lists for efficient iteration
- Objects are moved into the container (no copies)

---

### Platform

**Location**: `Platform.hpp`, `Platform.cpp`
**Type**: Library component
**Diagram**: [`docs/designs/input-state-management/input-state-management.puml`](../../../../../docs/designs/input-state-management/input-state-management.puml)
**Modified**: [Ticket: 0004_gui_framerate](../../../../../tickets/0004_gui_framerate.md)

#### Purpose
Entity type representing a simulated platform with kinematic state and agent control. Platforms own agents that control their state and can optionally link to Objects in WorldModel for visual representation synchronization. Per AC7 of ticket 0004, agent logic is placed in Platform (not Object). Per AC3, each Platform has its own non-unique internal state.

#### Key Interfaces
```cpp
class Platform {
public:
  explicit Platform(uint32_t id);
  ~Platform();

  // State update with agent logic and visual sync
  void update(const std::chrono::milliseconds& currTime);

  // Agent management
  void setAgent(std::unique_ptr<BaseAgent> agent);
  BaseAgent* getAgent();
  const BaseAgent* getAgent() const;
  bool hasAgent() const;

  // Visual object linking
  void setVisualObject(Object& object);
  bool hasVisualObject() const;
  Object& getVisualObject();
  const Object& getVisualObject() const;

  // State access
  InertialState& getState();
  const InertialState& getState() const;
  uint32_t getId() const;

private:
  InertialState state_;
  std::unique_ptr<BaseAgent> agent_;
  std::optional<std::reference_wrapper<Object>> visualObject_;
  uint32_t id_;
  std::chrono::milliseconds lastUpdateTime_;
};
```

#### Usage Example
```cpp
// Create platform
Platform platform{0};

// Assign agent for control
auto agent = std::make_unique<InputControlAgent>();
platform.setAgent(std::move(agent));

// Link to visual object for rendering
Object& visualObj = worldModel.getObject(objIndex);
platform.setVisualObject(visualObj);

// Update (agent updates state, visual object syncs position)
platform.update(currentTime);
```

#### Update Behavior
The `update()` method performs two operations:
1. If an agent is present, calls `agent->updateState(state_)` to compute new state
2. If a visual object is linked, synchronizes object position and rotation with platform state

This ensures that:
- Agent logic controls the platform's kinematic state
- Visual representation stays synchronized with simulation state
- Platform owns the source of truth for position/orientation

#### Thread Safety
**Not thread-safe** — Mutable state.

#### Memory Management
- **Agent ownership**: Owns agent via `std::unique_ptr<BaseAgent>`
- **Visual object**: Non-owning optional reference (Object owned by WorldModel)
- **State**: Owned InertialState by value
- **RAII**: Agent cleaned up automatically on destruction

#### Design Decisions (Ticket 0004)
- **Platform owns agent** (not Object) per AC7 — separates agent logic from visual representation
- **Non-unique state** per AC3 — each Platform has its own InputCommands state (not singleton)
- **Optional visual link** — Platform can update state without requiring visual representation
- **Synchronization direction** — Platform state drives Object position (not bidirectional)

---

## Design Patterns in Use

### Value Semantics
**Used in**: Coordinate, Angle, EulerAngles, InertialState
**Purpose**: Efficient copying, clear ownership, SIMD-friendly memory layout.

### Lazy Evaluation
**Used in**: Angle normalization, ReferenceFrame rotation matrix
**Purpose**: Reduces redundant computation by deferring expensive operations until needed.

### Factory Pattern
**Used in**: Object creation
**Purpose**: Enforces valid object construction with appropriate components.

### Component-Based Design
**Used in**: Object with optional physics/collision/visual components
**Purpose**: Flexible object composition, efficient iteration by component type.

---

## Cross-Cutting Concerns

### Coordinate System Convention
- **Type**: Right-handed Cartesian
- **Aerospace Convention**:
  - X-axis: Forward (velocity direction)
  - Y-axis: Right (starboard)
  - Z-axis: Up (zenith)

### Error Handling Strategy
- **Exceptions**: Thrown for invalid access (e.g., accessing collision hull when not present)
- **Factory validation**: Object factory methods validate input parameters
- **Optional access**: `has*()` methods allow checking before access

### Memory Management
- **Value semantics**: Coordinate, Angle, EulerAngles, InertialState
- **Unique ownership**: Platform owns agent via `std::unique_ptr`
- **Optional components**: Object uses `std::optional` for optional physics/collision
- **Move semantics**: WorldModel accepts objects via move

### Thread Safety Conventions
- **Value types**: Safe to copy across threads
- **Container types**: Not thread-safe, single-threaded simulation assumed

---

## Build & Configuration

### Build Requirements
- **C++ Standard**: C++20
- **Dependencies**:
  - Eigen3 — Linear algebra
  - msd-assets — Asset reference (for Object)
  - Physics module — PhysicsComponent, ConvexHull (for Object)

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
test/Environment/
├── AngleTest.cpp           # Angle normalization, arithmetic
├── EnvironmentTest.cpp     # Overall environment tests
└── ReferenceFrameTest.cpp  # Transform validation
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
- **Initialization**: Brace initialization `{}` for all object construction
- **Naming**: `PascalCase` for classes, `camelCase` for methods, `snake_case_` for members
- **Return Values**: Prefer returning values/structs over output parameters
- **Memory**: Value semantics for mathematical primitives, `std::unique_ptr` for ownership

See the [root CLAUDE.md](../../../../CLAUDE.md#coding-standards) for complete details and examples.

---

## Getting Help

### For AI Assistants
1. This document provides complete architectural context for the Environment module
2. Review [msd-sim/CLAUDE.md](../../CLAUDE.md) for overall simulation architecture
3. Review Physics module documentation for PhysicsComponent and ConvexHull details
4. Check [root CLAUDE.md](../../../../CLAUDE.md) for project-wide conventions

### For Developers
- **Mathematical primitives**: Start with Coordinate, Angle, EulerAngles
- **Kinematics**: Use InertialState for complete motion representation
- **Transformations**: Use ReferenceFrame for coordinate system conversions
- **Objects**: Use Object factory methods for creating simulation entities
- **Container**: Use WorldModel to manage all objects in the simulation
