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

Force-based rigid body dynamics and collision detection. Provides convex hull geometry, GJK collision detection, inertia tensor calculation, and dynamic state management.

**Key Components**:
- `ConvexHull` — Convex hull geometry via Qhull
- `PhysicsComponent` — Rigid body physics properties
- `DynamicState` — Velocities and accelerations
- `InertialCalculations` — Inertia tensor computation
- `GJK` — Gilbert-Johnson-Keerthi collision detection with AssetPhysical transform support
- `AssetPhysical` — Combines collision hull with world-space ReferenceFrame transformation

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

---

## Recent Architectural Changes

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
