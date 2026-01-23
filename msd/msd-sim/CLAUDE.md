# msd-sim Library Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/msd/msd-sim/` for detailed component relationships.

## Library Overview

**msd-sim** is the core simulation library for the MSD project. It provides a comprehensive simulation engine with mathematical primitives, rigid body physics, coordinate transformations, collision detection, and extensible agent control.

**Diagram**: [`msd-sim-core.puml`](../../docs/msd/msd-sim/msd-sim-core.puml)

## Architecture Overview

### High-Level Architecture

The library is organized into distinct modules with clear responsibilities:

```
Engine (Top-level orchestrator)
    ├── AssetRegistry (from msd-assets)
    └── WorldModel (Simulation container)
        └── Object (Unified entity type)
            ├── ReferenceFrame (Position/Orientation)
            ├── ConvexHull (Collision geometry)
            └── PhysicsComponent (Dynamics)
```

### Core Modules

| Module | Location | Purpose | Documentation |
|--------|----------|---------|---------------|
| Engine | `src/` | Top-level simulation orchestrator | This document |
| Agent | `src/Agent/` | Autonomous control interface | [`Agent/CLAUDE.md`](src/Agent/CLAUDE.md) |
| Environment | `src/Environment/` | Mathematical primitives & world model | [`Environment/CLAUDE.md`](src/Environment/CLAUDE.md) |
| Physics | `src/Physics/` | Rigid body dynamics & collision | [`Physics/CLAUDE.md`](src/Physics/CLAUDE.md) |
| Utils | `src/Utils/` | Common helper functions | [`Utils/CLAUDE.md`](src/Utils/CLAUDE.md) |

---

## Module Summary

### Agent Module

**Location**: `src/Agent/`
**Documentation**: [`Agent/CLAUDE.md`](src/Agent/CLAUDE.md)

Abstract interface for autonomous control logic using the Strategy pattern. Enables polymorphic agent behavior where different control algorithms can be swapped at runtime.

**Key Components**:
- `BaseAgent` — Abstract base class with `updateState()` pure virtual method
- Designed for ownership via `std::unique_ptr` by Platform/Object

### Environment Module

**Location**: `src/Environment/`
**Documentation**: [`Environment/CLAUDE.md`](src/Environment/CLAUDE.md)
**Diagrams**: [`docs/msd/msd-sim/Environment/`](../../docs/msd/msd-sim/Environment/)

Core mathematical primitives and simulation entity management. Provides type-safe angle handling, 3D coordinates, kinematic state representation, coordinate transformations, and unified object management.

**Key Components**:
- `Coordinate` — 3D vector wrapper (Eigen-based)
- `Angle` — Type-safe angle with lazy normalization
- `AngularCoordinate` — Orientation angles with deferred normalization
- `AngularRate` — Angular velocity/acceleration without normalization
- `InertialState` — Complete kinematic state (6 DOF)
- `ReferenceFrame` — Coordinate transformations
- `Object` — Unified simulation entity (Graphical/Inertial/Environmental/Boundary)
- `WorldModel` — Container and manager for all simulation objects

### Physics Module

**Location**: `src/Physics/`
**Documentation**: [`Physics/CLAUDE.md`](src/Physics/CLAUDE.md)
**Diagrams**: [`docs/msd/msd-sim/Physics/`](../../docs/msd/msd-sim/Physics/)

Force-based rigid body dynamics and collision detection. Provides convex hull geometry, GJK collision detection, inertia tensor calculation, dynamic state management, and force application with semi-implicit Euler integration.

**Key Components**:
- `ConvexHull` — Convex hull geometry via Qhull
- `PhysicsComponent` — Rigid body physics properties
- `DynamicState` — Velocities and accelerations
- `InertialCalculations` — Inertia tensor computation
- `GJK` — Gilbert-Johnson-Keerthi collision detection with AssetPhysical transform support
- `AssetPhysical` — Combines collision hull with world-space ReferenceFrame transformation
- `AssetInertial` — Dynamic physics object with force accumulation and integration
- `WorldModel` — Physics integration with gravity and semi-implicit Euler

### Utils Module

**Location**: `src/Utils/`
**Documentation**: [`Utils/CLAUDE.md`](src/Utils/CLAUDE.md)

Common helper functions for numerical operations.

**Key Components**:
- `almostEqual()` — Floating-point comparison with tolerance
- `TOLERANCE` — Default comparison tolerance (1e-10)

---

## Engine Component

**Location**: `src/Engine.hpp`, `src/Engine.cpp`

### Purpose
Top-level simulation orchestrator that coordinates asset loading, world management, and simulation updates.

### Key Interfaces
```cpp
class Engine {
  Engine(const std::string& dbPath);

  void update(std::chrono::milliseconds simTime);

  void spawnInertialObject(const std::string assetName,
                           const Coordinate& position,
                           const AngularCoordinate& orientation);

  msd_assets::AssetRegistry& getAssetRegistry();

private:
  msd_assets::AssetRegistry assetRegistry_;
  WorldModel worldModel_;
};
```

### Usage Example
```cpp
#include "msd-sim/src/Engine.hpp"

// Create engine with database path
msd_sim::Engine engine{"assets.db"};

// Spawn objects into simulation
engine.spawnInertialObject("cube",
                           Coordinate{0, 0, 10},
                           AngularCoordinate{});

// Run simulation loop
auto simTime = std::chrono::milliseconds{0};
while (running) {
  engine.update(simTime);
  simTime += std::chrono::milliseconds{16};  // ~60 FPS
}
```

### Thread Safety
**Not thread-safe** — Single-threaded simulation assumed.

### Memory Management
- Owns `AssetRegistry` and `WorldModel` by value
- Objects spawned are owned by WorldModel

### Dependencies
- `msd-assets` — Asset loading and management
- Environment module — WorldModel, Coordinate, AngularCoordinate

---

## Cross-Cutting Concerns

### Coordinate System Convention
- **Type**: Right-handed Cartesian (Aerospace convention)
- **X-axis**: Forward (velocity direction)
- **Y-axis**: Right (starboard)
- **Z-axis**: Up (zenith)

### Units
All quantities use SI units:
- Position: meters [m]
- Mass: kilograms [kg]
- Force: Newtons [N]
- Torque: Newton-meters [N·m]
- Angles: radians [rad]
- Time: seconds [s] or milliseconds [ms]

### Error Handling Strategy
- **Exceptions**: For invalid parameters and degenerate geometry
- **std::optional**: For cache lookups and queries that may fail
- **Validation**: Constructors and factory methods validate inputs

### Memory Management
- **Value semantics**: Mathematical primitives (Coordinate, Angle, etc.)
- **Unique ownership**: `std::unique_ptr` for agents
- **Optional components**: `std::optional` for physics/collision in Object
- **Move semantics**: WorldModel accepts objects via move

### Thread Safety Conventions
- **Value types**: Safe to copy across threads after construction
- **Container types**: Not thread-safe, single-threaded simulation assumed
- **Read-only operations**: Thread-safe for ConvexHull after construction

---

## Build & Configuration

### Build Requirements
- **C++ Standard**: C++20
- **Compiler**: GCC 11+, Clang 14+, or MSVC 2019+
- **Build System**: CMake 3.15+ with Conan 2.x

### Dependencies
- **Eigen3** — Linear algebra
- **Qhull** — Convex hull computation
- **spdlog** — Logging
- **msd-assets** — Asset management (public dependency)

### Building This Library

```bash
# Install dependencies
conan install . --build=missing -s build_type=Debug

# Configure and build
cmake --preset conan-debug
cmake --build --preset debug-sim-only
```

### Build Targets
- `msd_sim` — Main library
- `msd_sim_test` — Test suite

---

## Testing

### Test Organization
```
test/
├── Environment/
│   ├── AngleTest.cpp           # Angle normalization, arithmetic
│   ├── EnvironmentTest.cpp     # Overall environment tests
│   └── ReferenceFrameTest.cpp  # Transform validation
└── Physics/
    └── ConvexHullTest.cpp      # Hull computation, containment, intersection
```

### Running Tests
```bash
cmake --build --preset debug-tests-only
ctest --preset debug
```

---

## Diagrams Index

| Diagram | Description | Location |
|---------|-------------|----------|
| [`msd-sim-core.puml`](../../docs/msd/msd-sim/msd-sim-core.puml) | High-level library architecture | `docs/msd/msd-sim/` |
| [`mathematical-primitives.puml`](../../docs/msd/msd-sim/Environment/mathematical-primitives.puml) | Coordinate, Angle, AngularCoordinate, AngularRate, InertialState | `docs/msd/msd-sim/Environment/` |
| [`angular-coordinate.puml`](../../docs/msd/msd-sim/Environment/angular-coordinate.puml) | AngularCoordinate and AngularRate detailed design | `docs/msd/msd-sim/Environment/` |
| [`reference-frame.puml`](../../docs/msd/msd-sim/Environment/reference-frame.puml) | Coordinate transformations | `docs/msd/msd-sim/Environment/` |
| [`object.puml`](../../docs/msd/msd-sim/Environment/object.puml) | Unified Object entity | `docs/msd/msd-sim/Environment/` |
| [`world-model.puml`](../../docs/msd/msd-sim/Environment/world-model.puml) | WorldModel container | `docs/msd/msd-sim/Environment/` |
| [`physics-core.puml`](../../docs/msd/msd-sim/Physics/physics-core.puml) | Physics module overview | `docs/msd/msd-sim/Physics/` |
| [`convex-hull.puml`](../../docs/msd/msd-sim/Physics/convex-hull.puml) | ConvexHull geometry | `docs/msd/msd-sim/Physics/` |
| [`physics-component.puml`](../../docs/msd/msd-sim/Physics/physics-component.puml) | PhysicsComponent rigid body | `docs/msd/msd-sim/Physics/` |
| [`dynamic-state.puml`](../../docs/msd/msd-sim/Physics/dynamic-state.puml) | DynamicState kinematics | `docs/msd/msd-sim/Physics/` |
| [`gjk-asset-physical.puml`](../../docs/msd/msd-sim/Physics/gjk-asset-physical.puml) | GJK collision detection with AssetPhysical transforms | `docs/msd/msd-sim/Physics/` |
| [`force-application.puml`](../../docs/msd/msd-sim/Physics/force-application.puml) | Force application system with semi-implicit Euler integration | `docs/msd/msd-sim/Physics/` |
| [`mirtich-inertia-tensor.puml`](../../docs/msd/msd-sim/Physics/mirtich-inertia-tensor.puml) | Mirtich algorithm for inertia tensor calculation | `docs/msd/msd-sim/Physics/` |

---

## Recent Architectural Changes

### Mirtich Algorithm for Inertia Tensor Calculation — 2026-01-22
**Ticket**: [0026_mirtich_inertia_tensor](../../tickets/0026_mirtich_inertia_tensor.md)
**Diagram**: [`mirtich-inertia-tensor.puml`](../../docs/msd/msd-sim/Physics/mirtich-inertia-tensor.puml)
**Type**: Algorithm Replacement (Internal Refactor)

Replaced the inaccurate tetrahedron decomposition approach in `InertialCalculations::computeInertiaTensorAboutCentroid()` with Brian Mirtich's mathematically exact algorithm from "Fast and Accurate Computation of Polyhedral Mass Properties" (1996). This eliminates the ~10-15% accuracy error and ad-hoc scaling factors present in the previous implementation.

The Mirtich algorithm uses the divergence theorem to convert volume integrals to surface integrals through three hierarchical layers: projection integrals (2D line integrals over face edges) → face integrals (3D surface integrals) → volume integrals (accumulated across all faces). From the volume integrals, it computes volume, center of mass, and inertia tensor about the origin, then applies the parallel axis theorem to shift to the centroid.

**Accuracy improvements**:
- Unit cube: Machine-perfect precision (< 1e-10 error vs analytical solution)
- Rectangular box: Exact match to analytical formulas within 1e-10
- Regular tetrahedron: Diagonal inertia elements equal within 1e-10
- Previous implementation: ~10-15% error with ad-hoc `/10.0` scaling factor
- New implementation: < 1e-10 relative error (effectively machine precision)

**Key implementation details**:
- **Vertex winding correction**: Added `getWindingCorrectedIndices()` to ensure Qhull's vertex order aligns with facet normals, as required by Mirtich's algorithm
- **Volume validation**: Computed volume as byproduct matches `ConvexHull::getVolume()` within 1e-10
- **No API changes**: Function signature unchanged, internal algorithm replaced
- **Reference implementation**: Cross-validated against Mirtich's public domain `volInt.c`

**Performance**: O(F) complexity where F = number of facets. Higher constant factor than tetrahedron decomposition (~50-100 operations per facet vs ~10-20), but still negligible for typical hulls (10-100 facets) since inertia calculation is one-time at object creation.

**Known limitation resolved**: This fixes the pre-existing NaN bug that prevented angular physics validation in ticket 0023. Angular dynamics now fully operational.

**Key files**:
- `src/Physics/RigidBody/InertialCalculations.cpp` — Complete algorithm replacement with Mirtich implementation
- `src/Physics/RigidBody/InertialCalculations.hpp` — Updated documentation with algorithm reference
- `test/Physics/InertialCalculationsTest.cpp` — 13 test cases validating analytical solutions and edge cases

---

### Force Application System for Rigid Body Physics — 2026-01-21
**Ticket**: [0023_force_application_system](../../tickets/0023_force_application_system.md)
**Diagram**: [`force-application.puml`](../../docs/msd/msd-sim/Physics/force-application.puml)
**Type**: Feature Enhancement

Implemented a complete force application system for rigid body physics, enabling realistic dynamics simulation with gravity, forces at arbitrary points generating torque, and semi-implicit Euler integration. This completes the scaffolding from ticket 0023a and uses the angular types introduced in ticket 0024.

**Key features**:
- **Force accumulation**: `AssetInertial` accumulates forces and torques per frame via `applyForce()`, `applyForceAtPoint()`, and `applyTorque()` methods
- **Torque computation**: Forces applied at offset points generate torque using the cross product `τ = r × F`
- **Semi-implicit Euler integration**: `WorldModel::updatePhysics()` integrates motion using velocity-first integration for better numerical stability
- **Gravity**: Applied as direct acceleration `F = m * g` for efficiency
- **ReferenceFrame synchronization**: Position and orientation synchronized after physics updates to ensure consistency with collision detection and rendering
- **World-space convention**: All forces, torques, and application points use world-space coordinates

**Physics integration order**:
1. Apply gravity to all inertial objects
2. Compute linear acceleration: `a = F_net / m`
3. Update velocity: `v += a * dt` (semi-implicit)
4. Update position: `x += v * dt`
5. Compute angular acceleration: `α = I⁻¹ * τ_net`
6. Update angular velocity: `ω += α * dt` (semi-implicit)
7. Update orientation: `θ += ω * dt`
8. Synchronize ReferenceFrame with InertialState
9. Clear accumulated forces for next frame

**Performance**: O(n) complexity per physics update where n = number of inertial objects. Recommended timestep: 16.67ms (60 FPS).

**Note**: Angular physics integration was initially blocked by a bug in inertia tensor calculation (ticket 0025) that produced NaN values. This was resolved by ticket 0026 which implemented the Mirtich algorithm. Angular dynamics now fully operational.

**Key files**:
- `src/Physics/RigidBody/AssetInertial.hpp`, `AssetInertial.cpp` — Force application and torque computation
- `src/Environment/WorldModel.hpp`, `WorldModel.cpp` — Physics integration with semi-implicit Euler
- `src/Environment/ReferenceFrame.hpp`, `ReferenceFrame.cpp` — Added const overload for getAngularCoordinate()
- `src/Physics/RigidBody/InertialState.hpp` — Kinematic state representation

---

### AngularCoordinate and AngularRate — 2026-01-21
**Ticket**: [0024_angular_coordinate](../../tickets/0024_angular_coordinate.md)
**Diagram**: [`angular-coordinate.puml`](../../docs/msd/msd-sim/Environment/angular-coordinate.puml)
**Type**: Breaking Change

Introduced two type-safe classes for angular quantity representation: `AngularCoordinate` for orientation with deferred normalization, and `AngularRate` for angular velocity/acceleration without normalization. Both inherit from `Eigen::Vector3d` for full matrix operations while providing semantic pitch/roll/yaw accessors.

**Breaking changes**:
- Removed `EulerAngles` class entirely
- `InertialState::orientation` changed from `EulerAngles` to `AngularCoordinate`
- `InertialState::angularVelocity` changed from `Coordinate` to `AngularRate`
- `InertialState::angularAcceleration` changed from `Coordinate` to `AngularRate`
- `ReferenceFrame` constructor, `setRotation()`, and `getAngularCoordinate()` now use `AngularCoordinate`
- Internal `ReferenceFrame::euler_` changed to `ReferenceFrame::angular_`

**Key improvements**:
- **Type safety**: Prevents accidental assignment of rates to orientations (compile-time error)
- **Performance**: Deferred normalization with 100π threshold is 10x faster than eager normalization (validated by prototypes)
- **Semantic clarity**: `orientation.yaw()` vs `angularVelocity.yaw()` makes intent explicit
- **Memory efficiency**: 24 bytes per instance (same as `Eigen::Vector3d`)

**Migration**:
```cpp
// Old (EulerAngles)
state.angularPosition.yaw.getRad()

// New (AngularCoordinate)
state.orientation.yaw()

// Old (Coordinate for rates)
state.angularVelocity.z()

// New (AngularRate)
state.angularVelocity.yaw()
```

**Key files**:
- `src/Environment/AngularCoordinate.hpp` — Orientation with deferred normalization
- `src/Environment/AngularRate.hpp` — Angular velocity/acceleration without normalization
- `src/Physics/RigidBody/InertialState.hpp` — Updated angular field types
- `src/Environment/ReferenceFrame.hpp`, `ReferenceFrame.cpp` — Migrated to AngularCoordinate

---

### GJK AssetPhysical Transform Support — 2026-01-18
**Ticket**: [0022_gjk_asset_physical_transform](../../tickets/0022_gjk_asset_physical_transform.md)
**Diagram**: [`gjk-asset-physical.puml`](../../docs/msd/msd-sim/Physics/gjk-asset-physical.puml)
**Type**: Breaking Change

Refactored GJK collision detection to work exclusively with `AssetPhysical` objects that include `ReferenceFrame` transformations. This enables collision detection between objects with arbitrary positions and orientations in world space by applying transformations on-the-fly during support function computation.

**Breaking changes**:
- Removed `GJK(const ConvexHull&, const ConvexHull&)` constructor
- Removed `gjkIntersects(const ConvexHull&, const ConvexHull&)` convenience function
- Removed `ConvexHull::intersects()` method
- Added `GJK(const AssetPhysical&, const AssetPhysical&)` constructor
- Added `gjkIntersects(const AssetPhysical&, const AssetPhysical&)` convenience function

**Performance**: < 2% overhead compared to identity transform baseline (validated by prototypes).

**Migration**: Wrap `ConvexHull` objects in `AssetPhysical` with identity `ReferenceFrame` for untransformed collision detection.

**Key files**:
- `src/Physics/GJK/GJK.hpp`, `GJK.cpp` — AssetPhysical-based GJK implementation
- `src/Physics/RigidBody/ConvexHull.hpp`, `ConvexHull.cpp` — Removed intersects() method
- `src/Physics/RigidBody/AssetPhysical.hpp` — Documentation updates

---

## Coding Standards

This library follows the project-wide coding standards defined in the [root CLAUDE.md](../../CLAUDE.md#coding-standards).

Key standards applied in this library:
- **Initialization**: Brace initialization `{}`, `NaN` for uninitialized floats
- **Naming**: `PascalCase` for classes, `camelCase` for methods, `snake_case_` for members
- **Return Values**: Prefer returning values/structs over output parameters
- **Memory**: Value semantics for primitives, `std::unique_ptr` for ownership

See the [root CLAUDE.md](../../CLAUDE.md#coding-standards) for complete details and examples.

---

## Getting Help

### For AI Assistants
1. Start with this document for library-level context
2. Review module-specific CLAUDE.md files for detailed component documentation:
   - [`Agent/CLAUDE.md`](src/Agent/CLAUDE.md) — Agent interface
   - [`Environment/CLAUDE.md`](src/Environment/CLAUDE.md) — Mathematical primitives
   - [`Physics/CLAUDE.md`](src/Physics/CLAUDE.md) — Rigid body dynamics
   - [`Utils/CLAUDE.md`](src/Utils/CLAUDE.md) — Helper utilities
3. Reference PlantUML diagrams in `docs/msd/msd-sim/` for visual architecture
4. Check [root CLAUDE.md](../../CLAUDE.md) for project-wide conventions

### For Developers
- **Mathematical primitives**: See Environment module (Coordinate, Angle, etc.)
- **Physics simulation**: See Physics module (ConvexHull, PhysicsComponent)
- **Custom agents**: Extend BaseAgent from Agent module
- **Object creation**: Use Object factory methods (createInertial, createGraphical, etc.)
- **Simulation loop**: Use Engine.update() with time stepping
