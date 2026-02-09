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
| DataRecorder | `src/DataRecorder/` | Background thread simulation data recording | This document |
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

Force-based rigid body dynamics and collision detection. Provides rigid body representation, convex hull geometry, collision detection and response system, inertia tensor calculation, numerical integration systems, and constraint enforcement.

**Key Components**:
- **RigidBody System** — Rigid body representation, convex hulls, inertial properties, quaternion orientation (see [`src/Physics/RigidBody/CLAUDE.md`](src/Physics/RigidBody/CLAUDE.md))
- **Collision System** — GJK/EPA collision detection and response (see [`src/Physics/Collision/CLAUDE.md`](src/Physics/Collision/CLAUDE.md))
- **Integration** — Numerical integration framework (see [`src/Physics/Integration/CLAUDE.md`](src/Physics/Integration/CLAUDE.md))
- **PotentialEnergy** — Environmental potential energy fields for Lagrangian mechanics (see [`src/Physics/PotentialEnergy/CLAUDE.md`](src/Physics/PotentialEnergy/CLAUDE.md))
- **Constraints** — Lagrange multiplier constraint system (see [`src/Physics/Constraints/CLAUDE.md`](src/Physics/Constraints/CLAUDE.md))
- `WorldModel` — Physics integration with gravity and constraint enforcement

### DataRecorder Module

**Location**: `src/DataRecorder/`
**Diagram**: [`docs/msd/msd-sim/DataRecorder/data-recorder.puml`](../../docs/msd/msd-sim/DataRecorder/data-recorder.puml)

Background thread-based simulation data recording system that captures simulation state to SQLite database. Provides write-only persistence mechanism with minimal impact on simulation performance using cpp_sqlite double-buffered DAOs and periodic transactional flushing.

**Key Components**:
- `DataRecorder` — Background thread orchestrator with configurable flush interval
- Frame-based timestamping via `SimulationFrameRecord` foreign key pattern
- WorldModel integration for opt-in recording

### Utils Module

**Location**: `src/Utils/`
**Documentation**: [`Utils/CLAUDE.md`](src/Utils/CLAUDE.md)

Common helper functions for numerical operations.

**Key Components**:
- `almostEqual()` — Floating-point comparison with tolerance
- `TOLERANCE` — Default comparison tolerance (1e-10)

---

## DataRecorder Component

**Location**: `src/DataRecorder/DataRecorder.hpp`, `src/DataRecorder/DataRecorder.cpp`
**Diagram**: [`data-recorder.puml`](../../docs/msd/msd-sim/DataRecorder/data-recorder.puml)
**Introduced**: [Ticket: 0038_simulation_data_recorder](../../tickets/0038_simulation_data_recorder.md)

### Purpose

Orchestrates background recording of simulation data to SQLite database with minimal impact on simulation thread performance. Uses cpp_sqlite's double-buffered DAOs for thread-safe record submission and periodic transactional flushing on a dedicated background thread.

### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `DataRecorder` | `DataRecorder.hpp` | Background thread orchestrator with configurable flush interval |
| `SimulationFrameRecord` | `msd-transfer/SimulationFrameRecord.hpp` | Frame timestamping record for temporal association |

### Key Interfaces

```cpp
namespace msd_sim {

class DataRecorder {
public:
  struct Config {
    std::chrono::milliseconds flushInterval{100};  // Flush every 100ms
    std::string databasePath;
  };

  explicit DataRecorder(const Config& config);
  ~DataRecorder();

  // Create new simulation frame with timestamp (adds to buffer)
  // Returns the pre-assigned frame ID for FK references in child records
  uint32_t recordFrame(double simulationTime);

  // Get DAO for adding records (thread-safe addToBuffer)
  template<typename T>
  cpp_sqlite::DataAccessObject<T>& getDAO();

  // Explicit flush (e.g., before shutdown)
  void flush();

  // Access database for queries (const only)
  const cpp_sqlite::Database& getDatabase() const;

private:
  void recorderThreadMain(std::stop_token stopToken);

  std::unique_ptr<cpp_sqlite::Database> database_;
  std::chrono::milliseconds flushInterval_;
  std::mutex flushMutex_;
  std::atomic<uint32_t> nextFrameId_{1};
  std::jthread recorderThread_;  // LAST member - ensures initialization order
};

}  // namespace msd_sim
```

### Usage Example

```cpp
#include "msd-sim/src/DataRecorder/DataRecorder.hpp"

// Enable recording in WorldModel
WorldModel worldModel;
worldModel.enableRecording("simulation.db", std::chrono::milliseconds{100});

// Recording happens automatically during worldModel.update()
// ...

// Disable recording (flushes pending records)
worldModel.disableRecording();
```

### Architecture

#### Thread Model

**Simulation Thread**:
1. Calls `recordFrame(simTime)` to create timestamped frame
2. Pre-assigned frame ID returned atomically via `nextFrameId_`
3. Calls `state.toRecord()` for each object
4. Sets `record.frame.id = frameId` for temporal association
5. Calls `getDAO<T>().addToBuffer(record)` (thread-safe, mutex-protected)

**Recorder Thread**:
1. Sleeps for `flushInterval_` in 10ms chunks (responsive shutdown)
2. Wakes and acquires `flushMutex_`
3. Calls `database_->withTransaction([&] { database_->flushAllDAOs(); })`
4. Releases mutex and repeats until `stop_token` signaled
5. Final flush in thread exit before joining

#### Frame-Based Timestamping

Uses foreign key pattern for normalized timestamp storage:
- `SimulationFrameRecord` table has `simulation_time` and `wall_clock_time`
- All per-frame records (e.g., `InertialStateRecord`) include `ForeignKey<SimulationFrameRecord> frame`
- Enables efficient queries ("all states at time T")
- Single source of truth for temporal data

#### DAO Flush Ordering

Frame DAO initialized first in constructor to ensure correct flush order:
```cpp
DataRecorder::DataRecorder(const Config& config) {
  database_ = std::make_unique<cpp_sqlite::Database>(config.databasePath, true);
  database_->getDAO<SimulationFrameRecord>();  // MUST be first for FK integrity
  // ... start thread
}
```

`Database::flushAllDAOs()` flushes DAOs in creation order, ensuring frame records are committed before state records that reference them.

### Thread Safety

- **Constructor/Destructor**: Not thread-safe (single-threaded initialization/teardown)
- **recordFrame()**: Thread-safe (atomic `nextFrameId_` + mutex-protected `addToBuffer()`)
- **getDAO()**: Thread-safe (returns reference to thread-safe DAO)
- **flush()**: Thread-safe (acquires `flushMutex_` to prevent concurrent flushes)
- **getDatabase()**: Thread-safe (const access only)

### Error Handling

- Constructor throws `std::runtime_error` if database path is invalid or cannot be opened
- `getDAO<T>()` creates DAO on first access (no failure mode)
- Database write errors logged but do not crash recorder thread
- Final flush in destructor ensures no data loss on shutdown

### Memory Management

- Owns `cpp_sqlite::Database` via `std::unique_ptr` (exclusive ownership)
- DAOs managed internally by Database (no manual tracking)
- Thread ownership via `std::jthread` (automatic join on destruction)
- Buffered records cleared after each flush (bounded memory usage)

**Member initialization order**: `recorderThread_` declared LAST to ensure all other members (database, mutexes, atomics) are initialized before thread starts and destroyed after thread joins.

### Dependencies

- `cpp_sqlite::Database` — Database connection, DAO management, bulk flushing
- `cpp_sqlite::DataAccessObject<T>` — Thread-safe buffered record insertion
- `cpp_sqlite::Transaction` — RAII transaction management for batch writes
- `msd-transfer::SimulationFrameRecord` — Frame timestamping record
- C++20 threading primitives — `std::jthread`, `std::stop_token`, `std::atomic`, `std::mutex`

### Performance Characteristics

- **Record submission**: < 1μs typical (push to buffer, no I/O)
- **Flush throughput**: > 10,000 records/sec with transactions
- **Memory overhead**: Bounded by flush interval × record rate
- **Simulation impact**: < 5% overhead for typical workloads (60 FPS, 10-100 objects)

### WorldModel Integration

Recording is opt-in via `std::unique_ptr<DataRecorder>`:

```cpp
class WorldModel {
  void enableRecording(const std::string& dbPath,
                       std::chrono::milliseconds flushInterval);
  void disableRecording();

private:
  void recordCurrentFrame();
  std::unique_ptr<DataRecorder> dataRecorder_;  // nullptr = recording disabled
};
```

Backward compatible: existing code unaffected if recording not enabled.

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
| [`epa.puml`](../../docs/msd/msd-sim/Physics/epa.puml) | EPA contact information extraction and CollisionHandler orchestration | `docs/msd/msd-sim/Physics/` |
| [`witness-points.puml`](../../docs/msd/msd-sim/Physics/witness-points.puml) | Witness point tracking for accurate torque calculation | `docs/msd/msd-sim/Physics/` |
| [`force-application.puml`](../../docs/msd/msd-sim/Physics/force-application.puml) | Force application system with semi-implicit Euler integration | `docs/msd/msd-sim/Physics/` |
| [`mirtich-inertia-tensor.puml`](../../docs/msd/msd-sim/Physics/mirtich-inertia-tensor.puml) | Mirtich algorithm for inertia tensor calculation | `docs/msd/msd-sim/Physics/` |
| [`collision-response.puml`](../../docs/msd/msd-sim/Physics/collision-response.puml) | Historical diagram documenting deprecated CollisionResponse namespace (removed 2026-01-31) and its constraint-based replacement | `docs/msd/msd-sim/Physics/` |
| [`0030_lagrangian_quaternion_physics.puml`](../../docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml) | Lagrangian quaternion physics with potential energy and constraints | `docs/designs/0030_lagrangian_quaternion_physics/` |
| [`generalized-constraints.puml`](../../docs/msd/msd-sim/Physics/generalized-constraints.puml) | Generalized Lagrange multiplier constraint system with extensible constraint library | `docs/msd/msd-sim/Physics/` |
| [`two-body-constraints.puml`](../../docs/msd/msd-sim/Physics/two-body-constraints.puml) | Two-body constraint infrastructure with ContactConstraint for collision response | `docs/msd/msd-sim/Physics/` |
| [`0034_active_set_method_contact_solver.puml`](../../docs/designs/0034_active_set_method_contact_solver/0034_active_set_method_contact_solver.puml) | Active Set Method contact solver replacing PGS with exact LCP solution | `docs/designs/0034_active_set_method_contact_solver/` |
| [`0043_constraint_hierarchy_refactor.puml`](../../docs/designs/0043_constraint_hierarchy_refactor/0043_constraint_hierarchy_refactor.puml) | Flattened constraint type hierarchy (2-level design with LambdaBounds) | `docs/designs/0043_constraint_hierarchy_refactor/` |
| [`data-recorder.puml`](../../docs/msd/msd-sim/DataRecorder/data-recorder.puml) | Background thread simulation data recording with frame-based timestamping | `docs/msd/msd-sim/DataRecorder/` |
| [`edge-contact-manifold.puml`](../../docs/msd/msd-sim/Physics/edge-contact-manifold.puml) | Edge-edge contact detection and 2-point contact manifold generation | `docs/msd/msd-sim/Physics/` |

---

## Change History

For architectural change history, design decision rationale, and symbol-level evolution, use the traceability database MCP tools:

- `get_ticket_impact("NNNN")` — All commits, file changes, and decisions for a ticket
- `search_decisions("query")` — Search design decision rationale and trade-offs
- `why_symbol("qualified_name")` — Design decision(s) that created or modified a symbol
- `get_symbol_history("qualified_name")` — Timeline of changes to a symbol across commits
- `get_commit_context("sha")` — Full context for a commit (ticket, phase, file/symbol changes)

See [`scripts/traceability/README.md`](../../scripts/traceability/README.md) for details.

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
