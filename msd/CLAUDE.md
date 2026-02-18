# MSD Libraries Architecture Guide

> High-level orchestration guide for the MSD (Multi-Spacecraft Dynamics) C++ libraries.
> For detailed class and interface documentation, see each library's individual CLAUDE.md file.

## Overview

The MSD system provides a modular framework for spacecraft dynamics simulation and visualization. The libraries are organized in a layered architecture where each layer has a single responsibility and depends only on layers below it.

## Library Dependency Graph

```
                    ┌─────────────┐
                    │   msd-exe   │  Main application
                    └──────┬──────┘
                           │
              ┌────────────┴────────────┐
              │                         │
              ▼                         ▼
       ┌─────────────┐          ┌─────────────┐
       │   msd-gui   │          │  msd-utils  │  Utilities
       └──────┬──────┘          └─────────────┘
              │
              ▼
       ┌─────────────┐
       │   msd-sim   │  Physics simulation
       └──────┬──────┘
              │
              ▼
       ┌─────────────┐
       │ msd-assets  │  Asset management
       └──────┬──────┘
              │
              ▼
       ┌─────────────┐
       │msd-transfer │  Database DTOs
       └─────────────┘


       ┌──────────────┐
       │msd-asset-gen │  Asset generation tool
       └──────┬───────┘
              │
              ▼
       ┌─────────────┐
       │ msd-assets  │
       └─────────────┘
```

## Libraries Summary

| Library | Type | Purpose | Documentation |
|---------|------|---------|---------------|
| [msd-transfer](msd-transfer/) | Header-only | Database transfer objects (DTOs) for asset persistence | [CLAUDE.md](msd-transfer/CLAUDE.md) |
| [msd-assets](msd-assets/) | Static library | Asset management, geometry caching, primitive factories | [CLAUDE.md](msd-assets/CLAUDE.md) |
| [msd-sim](msd-sim/) | Static library | Physics simulation with rigid body dynamics and collision detection | [CLAUDE.md](msd-sim/CLAUDE.md) |
| [msd-gui](msd-gui/) | Static library | SDL3-based GPU rendering and input handling | [CLAUDE.md](msd-gui/CLAUDE.md) |
| [msd-utils](msd-utils/) | Static library | General-purpose utilities (path resolution) | [CLAUDE.md](msd-utils/CLAUDE.md) |
| [msd-exe](msd-exe/) | Executable | Main application entry point | [CLAUDE.md](msd-exe/CLAUDE.md) |
| [msd-asset-gen](msd-asset-gen/) | Executable | Asset database generation tool | [CLAUDE.md](msd-asset-gen/CLAUDE.md) |
| [msd-pybind](msd-pybind/) | pybind11 module | Python bindings for transfer records (auto-generated) and Engine simulation control (manual) | [CLAUDE.md](msd-pybind/CLAUDE.md) |

---

## Library Descriptions

### msd-transfer

**Location**: `msd-transfer/`
**Type**: Header-only interface library
**Diagram**: [msd-transfer-core.puml](../docs/msd/msd-transfer/msd-transfer-core.puml)

Defines lightweight structs representing database records for asset persistence. These DTOs are used by `cpp_sqlite` to automatically generate SQL schema. This library acts as the shared contract between database storage and domain logic.

**External Dependencies**: cpp_sqlite, Boost.Describe

---

### msd-assets

**Location**: `msd-assets/`
**Type**: Static library
**Diagram**: [msd-assets-core.puml](../docs/msd/msd-assets/msd-assets-core.puml)

Manages asset loading, caching, and geometry generation. Provides factories for primitive shapes (pyramids, cubes) and handles STL file loading. The AssetRegistry provides cached access to geometry data.

**Dependencies**: msd-transfer
**External Dependencies**: cpp_sqlite, Eigen3

---

### msd-sim

**Location**: `msd-sim/`
**Type**: Static library
**Diagram**: [msd-sim-core.puml](../docs/msd/msd-sim/msd-sim-core.puml)

Physics simulation engine with rigid body dynamics, collision detection (GJK algorithm), and environmental modeling. Organized into sub-modules:
- **Environment**: Mathematical primitives, reference frames, world model
- **Physics**: Rigid body dynamics, convex hull collision shapes
- **Agent**: Abstract control interface for autonomous entities

**Dependencies**: msd-assets
**External Dependencies**: Eigen3, Qhull

---

### msd-gui

**Location**: `msd-gui/`
**Type**: Static library
**Diagram**: [msd-gui-core.puml](../docs/msd/msd-gui/msd-gui-core.puml)

GPU-accelerated 3D rendering and input handling. Wraps SDL3 with modern C++20 patterns. Key architectural features:
- **Shader Policy System**: Template-based compile-time shader selection
- **Input Management**: Separation of concerns (state tracking, handling, application)
- **GPU Instancing**: Efficient rendering with per-instance data buffers

**Dependencies**: msd-sim, msd-assets
**External Dependencies**: SDL3, SDL3_image, SDL3_mixer, SDL3_ttf, Eigen3

---

### msd-utils

**Location**: `msd-utils/`
**Type**: Static library

General-purpose utilities for path resolution and common operations. Minimal dependencies.

**Dependencies**: None (C++20 standard library only)

---

### msd-exe

**Location**: `msd-exe/`
**Type**: Executable
**Diagram**: [msd-exe.puml](../docs/msd/msd-exe/msd-exe.puml)

Main application entry point. Minimal integration layer that connects msd-gui with msd-sim.

**Dependencies**: msd-gui, msd-sim, msd-utils

---

### msd-asset-gen

**Location**: `msd-asset-gen/`
**Type**: Executable

Command-line tool that generates SQLite databases populated with primitive geometry assets.

**Usage**: `./generate_assets output.db`

**Dependencies**: msd-assets, msd-transfer

---

## Architecture Diagrams

### Library-Level Diagrams

| Diagram | Description |
|---------|-------------|
| [msd-transfer-core.puml](../docs/msd/msd-transfer/msd-transfer-core.puml) | Database record architecture |
| [msd-assets-core.puml](../docs/msd/msd-assets/msd-assets-core.puml) | Asset management architecture |
| [msd-sim-core.puml](../docs/msd/msd-sim/msd-sim-core.puml) | Simulation engine architecture |
| [msd-gui-core.puml](../docs/msd/msd-gui/msd-gui-core.puml) | GUI and rendering architecture |
| [msd-exe.puml](../docs/msd/msd-exe/msd-exe.puml) | Application integration |

### Cross-Cutting Design Documents

| Design | Description | Libraries Affected |
|--------|-------------|-------------------|
| [modularize-gpu-shader-system](../docs/designs/modularize-gpu-shader-system/) | Template-based shader policy system | msd-gui |
| [input-state-management](../docs/designs/input-state-management/) | Input handling separation of concerns | msd-gui |

---

## Build System

All MSD libraries are built together using CMake with Conan for dependency management.

**Quick start:**
```bash
# Install dependencies
conan install . --build=missing -s build_type=Debug

# Build all libraries
cmake --preset conan-debug
cmake --build --preset conan-debug
```

**Library-specific build presets:**
```bash
cmake --build --preset debug-transfer-only   # msd-transfer
cmake --build --preset debug-assets-only     # msd-assets + tests
cmake --build --preset debug-sim-only        # msd-sim + tests
cmake --build --preset debug-gui-only        # msd-gui
cmake --build --preset debug-exe-only        # msd-exe
cmake --build --preset debug-asset-gen-only  # generate_assets
```

See [root CLAUDE.md](../CLAUDE.md#build--configuration) for complete build instructions.

---

## Testing

Each library with testable logic has its own test directory:

```
msd/
├── msd-assets/test/     # Asset management tests
├── msd-sim/test/        # Simulation engine tests
└── msd-utils/test/      # Utility tests
```

```bash
# Run all tests
ctest --preset conan-debug

# Library-specific tests
cmake --build --preset debug-sim-only --target msd_sim_test
./build/Debug/debug/msd_sim_test
```

---

## Getting Help

### For AI Assistants

1. **Start here** for library-level architecture and dependencies
2. **Consult library CLAUDE.md** for class-level details (e.g., [msd-gui/CLAUDE.md](msd-gui/CLAUDE.md))
3. **Reference diagrams** in `docs/msd/{library}/` for component relationships
4. **Check root CLAUDE.md** for [coding standards](../CLAUDE.md#coding-standards) and [build instructions](../CLAUDE.md#build--configuration)

### For Developers

- **Library documentation**: `msd/{library}/CLAUDE.md`
- **Design documents**: `docs/designs/`
- **Feature tickets**: `tickets/`
