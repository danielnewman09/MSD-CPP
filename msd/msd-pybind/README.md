# msd-pybind

Read-only Python bindings for MSD simulation recording databases via [pybind11](https://pybind11.readthedocs.io/).

## Overview

The `msd_reader` Python module provides access to simulation recording databases (`.db` files) produced by the MSD simulation engine. It exposes all transfer record types as Python classes and supports database queries for analysis, visualization, and post-processing workflows.

### Capabilities

- **Record types**: All transfer records (frames, states, energy, collisions, meshes, etc.) as Python objects
- **Database queries**: `select_all`, `select_by_id`, `select_by_frame`, `select_by_body`
- **Geometry deserialization**: Convert vertex BLOBs to Python tuples for plotting/analysis
- **Read-only**: Opens databases without write access — safe for concurrent use

## Prerequisites

- **Python 3.x** with development headers (module component)
- **Conan 2.x** package manager (same as main project)
- All other dependencies are managed via Conan

### macOS (Homebrew)

```bash
# Python 3.12 with headers
brew install python@3.12
```

### Verifying Python development headers

The build requires `Development.Module` from CMake's `FindPython3`. If configure fails with `Could NOT find Python3 (missing: Development)`, ensure your Python installation includes development headers:

```bash
# Check that python3-config exists
python3-config --includes
```

## Building

The pybind module is **opt-in** — it does not build by default and has no impact on the standard C++ build.

### 1. Install dependencies with pybind enabled

```bash
conan install . --build=missing -s build_type=Debug -o "&:enable_pybind=True"
```

### 2. Configure CMake

```bash
cmake --preset conan-debug
```

If CMake picks the wrong Python (e.g., Xcode's Python 3.9 instead of Homebrew's 3.12), specify it explicitly:

```bash
cmake --preset conan-debug -DPython3_EXECUTABLE=$(which python3)
```

### 3. Build the module

```bash
cmake --build --preset debug-pybind-only
```

The shared library is output to `build/Debug/Debug/msd_reader.cpython-3XX-darwin.so` (or equivalent for your platform).

### Switching back to standard C++ build

After building with pybind enabled, CMake caches `ENABLE_PYBIND=True`. To return to the standard build without pybind:

```bash
# Re-run Conan without the pybind option
conan install . --build=missing -s build_type=Debug

# Reconfigure with --fresh to clear the cached ENABLE_PYBIND variable
cmake --preset conan-debug --fresh
```

Without the `--fresh` flag, CMake will keep `ENABLE_PYBIND=ON` from the cache and fail to find pybind11.

## Usage

```python
import sys
sys.path.insert(0, "build/Debug/Debug")  # Adjust to your build output path
import msd_reader

# Open a recording database (read-only)
db = msd_reader.Database("path/to/recording.db")

# Query all simulation frames
frames = db.select_all_frames()
for f in frames:
    print(f"Frame {f.id}: t={f.simulation_time:.4f}s")

# Query inertial states for a specific frame
states = db.select_inertial_states_by_frame(frame_id=1)
for s in states:
    pos = s.position
    print(f"  Body at ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")

# Query energy for a specific body across all frames
energy = db.select_energy_by_body(body_id=1)
for e in energy:
    print(f"  KE={e.linear_ke:.4f}  PE={e.potential_e:.4f}  Total={e.total_e:.4f}")

# Deserialize mesh geometry for visualization
meshes = db.select_all_meshes()
vertices = msd_reader.deserialize_visual_vertices(meshes[0].vertex_data)
# Returns list of (px, py, pz, r, g, b, nx, ny, nz) tuples
```

## Running Tests

### Setting up a virtual environment

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install pytest
```

### Running the test suite

```bash
# From the project root (with the module already built)
python3 -m pytest msd/msd-pybind/test/ -v
```

The tests verify module import, record type availability, database queries, and geometry deserialization.

## Exposed Types

### Tier 1 — Top-level database records

| Python Class | Database Table | Key Fields |
|---|---|---|
| `SimulationFrameRecord` | SimulationFrameRecord | `simulation_time`, `wall_clock_time` |
| `AssetInertialStaticRecord` | AssetInertialStaticRecord | `body_id`, `mass`, `restitution`, `friction` |
| `InertialStateRecord` | InertialStateRecord | `position`, `velocity`, `orientation`, ... |
| `EnergyRecord` | EnergyRecord | `linear_ke`, `rotational_ke`, `potential_e`, `total_e` |
| `SystemEnergyRecord` | SystemEnergyRecord | `total_system_e`, `delta_e`, `collision_active` |
| `CollisionResultRecord` | CollisionResultRecord | `body_a_id`, `body_b_id`, `normal`, `penetrationDepth` |
| `SolverDiagnosticRecord` | SolverDiagnosticRecord | `iterations`, `residual`, `converged` |
| `MeshRecord` | MeshRecord | `vertex_data`, `vertex_count` |
| `ObjectRecord` | ObjectRecord | `name`, `category` |

### Tier 2 — Sub-records (nested in Tier 1 records)

`CoordinateRecord`, `VelocityRecord`, `AccelerationRecord`, `QuaternionDRecord`, `Vector3DRecord`, `Vector4DRecord`, `AngularAccelerationRecord`, `AngularVelocityRecord`, `AngularCoordinateRecord`, `ContactPointRecord`, `ForceVectorRecord`, `TorqueVectorRecord`, `ExternalForceRecord`

### Tier 3 — Extended records (forward compatibility)

`AssetDynamicStateRecord`, `AssetPhysicalStaticRecord`, `AssetPhysicalDynamicRecord`, `MaterialRecord`, `PhysicsTemplateRecord`

## Database Query Methods

| Method | Returns | Description |
|---|---|---|
| `select_all_frames()` | `list[SimulationFrameRecord]` | All simulation frames |
| `select_all_inertial_states()` | `list[InertialStateRecord]` | All kinematic states |
| `select_all_energy()` | `list[EnergyRecord]` | All per-body energy records |
| `select_all_system_energy()` | `list[SystemEnergyRecord]` | All system-level energy records |
| `select_all_collisions()` | `list[CollisionResultRecord]` | All collision events |
| `select_all_meshes()` | `list[MeshRecord]` | All mesh geometries |
| `select_*_by_id(id)` | `Optional[Record]` | Single record by primary key |
| `select_*_by_frame(frame_id)` | `list[Record]` | Records for a simulation frame |
| `select_*_by_body(body_id)` | `list[Record]` | Records for a specific body |
