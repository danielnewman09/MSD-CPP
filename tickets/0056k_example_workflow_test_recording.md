# Ticket 0056k: Example Workflow with Test Recording

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [x] Merged / Complete

**Current Phase**: Merged / Complete
**Type**: Feature
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-12
**Updated**: 2026-02-12
**Generate Tutorial**: No
**Parent Ticket**: [0056_browser_simulation_replay](0056_browser_simulation_replay.md)
**Depends On**: [0056c_python_bindings](0056c_python_bindings.md), [0056d_fastapi_backend](0056d_fastapi_backend.md)

---

## Overview

The FastAPI backend (0056d) and pybind bindings (0056c) are implemented, but there's no way to run them end-to-end because:
1. No test recording database exists
2. The pybind bindings are missing FK ID properties and RepeatedField access
3. The service layer has several bugs (wrong class name, missing field accessors, incorrect vertex format)

This ticket creates an end-to-end example: a C++ program generates a recording database, a Python script demonstrates querying it, and the FastAPI backend can serve it.

---

## Requirements

### R1: Fix pybind11 Bindings — Expose FK IDs and RepeatedField

**File**: `msd/msd-pybind/src/record_bindings.cpp`

Add `def_property_readonly` for ForeignKey `.id` fields and RepeatedField `.data`:

| Record | Property | Lambda |
|--------|----------|--------|
| `InertialStateRecord` | `body_id` | `r.body.id` |
| `InertialStateRecord` | `frame_id` | `r.frame.id` |
| `EnergyRecord` | `body_id` | `r.body.id` |
| `EnergyRecord` | `frame_id` | `r.frame.id` |
| `SystemEnergyRecord` | `frame_id` | `r.frame.id` |
| `CollisionResultRecord` | `frame_id` | `r.frame.id` |
| `CollisionResultRecord` | `contacts` | `r.contacts.data` (returns `vector<ContactPointRecord>`) |
| `SolverDiagnosticRecord` | `frame_id` | `r.frame.id` |

`ForeignKey<T>` has public `uint32_t id` member. `RepeatedFieldTransferObject<T>` has public `std::vector<T> data` member.

### R2: Fix Service Layer Bugs

#### R2a: Wrong class name
**Files**: `replay/replay/services/simulation_service.py`, `replay/replay/services/geometry_service.py`

`msd_reader.RecordingDatabase(...)` → `msd_reader.Database(...)` (the pybind class is called `Database`).

#### R2b: FK field access
**File**: `replay/replay/services/simulation_service.py`

| Line | Bug | Fix |
|------|-----|-----|
| ~176 | `energy.frame_fk` | `energy.frame_id` |
| ~194 | `sys_energy.frame_fk` | `sys_energy.frame_id` |

(`state.body_id` and `collision.contacts` will work after R1 adds the properties.)

#### R2c: Vertex format
**File**: `replay/replay/services/geometry_service.py`

`msd_reader.deserialize_visual_vertices()` returns 9-tuples `(px,py,pz, r,g,b, nx,ny,nz)`. Extract first 3 floats (position) and flatten to `[x,y,z,x,y,z,...]` for Three.js BufferGeometry.

### R3: C++ Recording Generator

**New file**: `replay/tools/generate_test_recording.cpp`
**New CMake**: `replay/tools/CMakeLists.txt` via `add_subdirectory()` from root

Simple C++ program following existing test patterns (e.g., `msd-sim/test/Environment/WorldModelCollisionTest.cpp`):

1. `createCubePoints(double size)` helper (same as tests)
2. `WorldModel` with floor (100×100×100 env cube at z=-50) and 1×1×1 cube at z=2
3. `world.enableRecording(outputPath)`
4. Run 300 timesteps at 10ms intervals (3 seconds: cube falls, bounces, settles)
5. `world.disableRecording()`

CMake links against `msd_sim` (provides all transitive deps). Follows `msd-asset-gen/src/CMakeLists.txt` pattern.

**Note**: Simulation recordings do NOT include MeshRecord data. The `/assets` geometry endpoint requires a combined database. This example focuses on physics data: frames, states, collisions, energies, solver diagnostics.

### R4: Python Example Script

**New file**: `replay/examples/query_recording.py`

Standalone script demonstrating `msd_reader` API:
- List frames and timestamps
- Query body metadata (AssetInertialStaticRecord)
- Get states for a frame (exercises `body_id` FK property)
- Get collisions for a frame (exercises `contacts` RepeatedField property)
- Get system energy timeseries (exercises `frame_id` FK property)

### R5: Update Documentation

**File**: `replay/README.md`

Add Quick Start section: build generator → generate recording → run example → start server.

---

## Files to Create

| File | Purpose |
|------|---------|
| `replay/tools/generate_test_recording.cpp` | C++ recording generator |
| `replay/tools/CMakeLists.txt` | Build config for generator |
| `replay/examples/query_recording.py` | Python example script |
| `replay/recordings/.gitkeep` | Directory placeholder |

## Files to Modify

| File | Change |
|------|--------|
| `msd/msd-pybind/src/record_bindings.cpp` | Add FK ID + RepeatedField properties (R1) |
| `replay/replay/services/simulation_service.py` | Fix class name + field names (R2a, R2b) |
| `replay/replay/services/geometry_service.py` | Fix class name + vertex flattening (R2a, R2c) |
| `replay/README.md` | Add Quick Start section (R5) |
| `CMakeLists.txt` (root) | `add_subdirectory(replay/tools)` (R3) |

---

## Test Plan

```bash
# 1. Build sim + generator
cmake --preset conan-debug
cmake --build --preset conan-debug --target generate_test_recording

# 2. Generate test recording
./build/Debug/debug/generate_test_recording replay/recordings/test_cube_drop.db

# 3. Build pybind module
cmake --build --preset debug-pybind-only

# 4. Run Python example (validates FK properties + RepeatedField access)
cd replay && python examples/query_recording.py

# 5. Start server and test endpoints
cd replay && uvicorn replay.app:app --reload
# GET /api/v1/simulations
# GET /api/v1/simulations/{id}/metadata
# GET /api/v1/simulations/{id}/frames/1/state
# GET /api/v1/simulations/{id}/energy

# 6. Existing C++ tests still pass
./build/Debug/debug/msd_sim_test
```

---

## Acceptance Criteria

1. [x] **AC1**: `generate_test_recording` builds and produces a valid .db file
2. [x] **AC2**: Python example runs successfully, printing frames/states/collisions/energy
3. [x] **AC3**: FK properties (`body_id`, `frame_id`) accessible on all relevant records
4. [x] **AC4**: `collision.contacts` returns list of ContactPointRecord objects
5. [x] **AC5**: FastAPI `/simulations/{id}/frames/{fid}/state` returns valid JSON
6. [x] **AC6**: FastAPI `/simulations/{id}/energy` returns valid JSON
7. [x] **AC7**: Existing C++ tests unaffected

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-12 (Ticket created)
- **Completed**: 2026-02-12
- **Notes**: Ticket created to fix pybind bindings, service layer bugs, and provide end-to-end example workflow. No math design or architectural design required - straightforward implementation of bug fixes and example code. Advanced directly to Ready for Implementation.

### Implementation Phase
- **Started**: 2026-02-12
- **Completed**: 2026-02-12
- **Branch**: 0056k-example-workflow-test-recording
- **PR**: #48
- **Issue**: #47
- **Artifacts**:
  - `msd/msd-pybind/src/record_bindings.cpp` — Added FK ID and RepeatedField properties
  - `replay/replay/services/simulation_service.py` — Fixed Database class name and FK field access
  - `replay/replay/services/geometry_service.py` — Fixed vertex flattening
  - `replay/tools/generate_test_recording.cpp` — C++ test recording generator
  - `replay/tools/CMakeLists.txt` — Build configuration for generator
  - `replay/examples/query_recording.py` — Python example script
  - `replay/recordings/.gitkeep` — Recordings directory placeholder
  - `replay/README.md` — Added Quick Start section
  - `CMakeLists.txt` — Added replay/tools subdirectory
- **Notes**: All requirements implemented.

### Post-Implementation Bug Fixes
- **Date**: 2026-02-12
- **Commit**: ce28674
- **Fixes**:
  1. **Segfault in DataRecorder destructor**: `Database::insert()` for nested ValidTransferObject types lazily creates DAOs during flush, mutating `daoCreationOrder_` while `flushAllDAOs()` iterates it. Fix: pre-create all 20+ DAOs in constructor before starting recorder thread.
  2. **Silent insert failure for CollisionResultRecord, SolverDiagnosticRecord, AssetInertialStaticRecord**: `recordCollisions()`, `recordSolverDiagnostics()`, and `recordStaticAsset()` pre-assigned IDs via `incrementIdCounter()` before `addToBuffer()`. During flush, `DAO::insert()` rejected records because `data.id <= idCounter_` (counter already bumped). Fix: let DAO auto-assign IDs during flush by leaving `id` at `UINT32_MAX`.
  3. **Frozen simulation time in generate_test_recording.cpp**: Passed constant `10ms` to `WorldModel::update()` instead of accumulating absolute time `(i+1)*10ms`. After frame 1, `dt = (10-10)/1000 = 0` so physics never advanced. Fix: pass accumulating time.
  4. **Empty AssetDynamicStateRecord table**: `recordInertialStates()` recorded via `InertialStateRecord` DAO directly instead of via parent `AssetDynamicStateRecord`. Fix: record via `AssetDynamicStateRecord` which auto-inserts nested records.
- **Verified**: All tables populated — 300 frames, 300 states, 158 collisions, 300 solver diagnostics, 1 static asset, 300 energy records, 298 distinct z positions

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
