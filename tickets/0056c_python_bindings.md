# Ticket 0056c: Python Bindings (pybind11)

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Review
**Type**: Feature
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-11
**Updated**: 2026-02-12 22:30
**Generate Tutorial**: No
**Parent Ticket**: [0056_browser_simulation_replay](0056_browser_simulation_replay.md)
**Depends On**: [0056a_collision_force_transfer_records](0056a_collision_force_transfer_records.md), [0056b_collision_pipeline_data_extraction](0056b_collision_pipeline_data_extraction.md), [0056i_static_asset_recording_and_fk](0056i_static_asset_recording_and_fk.md)

---

## Overview

Create a pybind11 module (`msd_reader`) that provides read-only Python access to simulation recording databases and asset databases. This is the bridge between the C++ simulation engine and the Python/web visualization layer.

The module exposes database queries and BLOB deserialization — it does NOT expose simulation control. Tests run in C++; Python only reads the resulting data.

---

## Requirements

### R1: pybind11 Dependency

Add `pybind11` as an optional Conan dependency:

- New Conan option: `enable_pybind` (default `False`)
- New CMake option: `ENABLE_PYBIND` (default `OFF`)
- Conditional `add_subdirectory(msd-pybind)` in `msd/CMakeLists.txt`

### R2: msd-pybind Library

New library at `msd/msd-pybind/` producing a Python module `msd_reader`.

```
msd/msd-pybind/
  CMakeLists.txt
  src/
    msd_bindings.cpp        # Module entry point (PYBIND11_MODULE)
    record_bindings.cpp     # All transfer record type bindings
    database_bindings.cpp   # Database read-only query bindings
    geometry_bindings.cpp   # Vertex BLOB deserialization
```

### R3: Record Type Bindings

Bind all transfer record types as Python classes with read-only properties. Records are organized into three tiers:

#### Tier 1: Top-Level Records (own DB tables, queried directly)

| Record | Key Fields | Notes |
|--------|-----------|-------|
| `SimulationFrameRecord` | id, simulation_time, wall_clock_time | Frame timestamping |
| `AssetInertialStaticRecord` | id, body_id, mass, restitution, friction | Static body data (recorded once at spawn) |
| `InertialStateRecord` | position (sub), velocity (sub), orientation (sub), quaternionRate (sub), angularAcceleration (sub), body FK, frame FK | Per-body per-frame kinematic state |
| `EnergyRecord` | body FK, linear_ke, rotational_ke, potential_e, total_e, frame FK | Per-body per-frame energy |
| `SystemEnergyRecord` | total_linear_ke, total_rotational_ke, total_potential_e, total_system_e, delta_e, energy_injection, collision_active, frame FK | System-level energy summary |
| `CollisionResultRecord` | body_a_id, body_b_id, normal (sub), penetrationDepth, contacts (RepeatedField), frame FK | Per-collision-pair per-frame |
| `SolverDiagnosticRecord` | iterations, residual, converged, num_constraints, num_contacts, frame FK | Per-frame solver stats |
| `MeshRecord` | id, vertex_data (bytes), vertex_count | Asset geometry |
| `ObjectRecord` | id, name, category, mesh FK IDs | Asset definitions |

#### Tier 2: Sub-Records (composed into top-level records)

These are embedded sub-records within top-level records. They appear as nested objects in Python:

| Sub-Record | Fields | Used By |
|-----------|--------|---------|
| `CoordinateRecord` | x, y, z | InertialStateRecord.position, ContactPointRecord.pointA/B |
| `VelocityRecord` | x, y, z | InertialStateRecord.velocity |
| `AccelerationRecord` | x, y, z | InertialStateRecord.acceleration |
| `QuaternionDRecord` | w, x, y, z | InertialStateRecord.orientation |
| `Vector4DRecord` | w, x, y, z | InertialStateRecord.quaternionRate |
| `Vector3DRecord` | x, y, z | CollisionResultRecord.normal |
| `AngularAccelerationRecord` | x, y, z | InertialStateRecord.angularAcceleration |
| `ContactPointRecord` | pointA (sub), pointB (sub), depth | CollisionResultRecord.contacts (RepeatedField) |

#### Tier 3: Extended Records (defined but not yet actively recorded)

These records exist in msd-transfer but are not yet written by the DataRecorder. Bind them for forward compatibility:

| Record | Fields | Notes |
|--------|--------|-------|
| `AssetDynamicStateRecord` | body_id, kinematicState (sub), externalForces (RepeatedField), frame FK | Full per-frame body state |
| `ExternalForceRecord` | force (sub), torque (sub), applicationPoint (sub) | Force/torque sub-record |
| `ForceVectorRecord` | x, y, z | Force component sub-record |
| `TorqueVectorRecord` | x, y, z | Torque component sub-record |
| `AssetPhysicalDynamicRecord` | (check fields) | Physical asset dynamic state |
| `AssetPhysicalStaticRecord` | (check fields) | Physical asset static state |
| `MaterialRecord` | (existing) | Rendering materials |
| `PhysicsTemplateRecord` | (existing) | Physics templates |

### R4: Database Query Bindings

Wrap `cpp_sqlite::Database` with read-only query methods:

```python
db = msd_reader.Database("recording.db")

# Select all records of a type
frames = db.select_all(SimulationFrameRecord)
states = db.select_all(InertialStateRecord)
collisions = db.select_all(CollisionResultRecord)
static_assets = db.select_all(AssetInertialStaticRecord)

# Select by frame FK (custom WHERE query)
states = db.select_by_frame(InertialStateRecord, frame_id)
collisions = db.select_by_frame(CollisionResultRecord, frame_id)
solver = db.select_by_frame(SolverDiagnosticRecord, frame_id)
energy = db.select_by_frame(EnergyRecord, frame_id)
system_energy = db.select_by_frame(SystemEnergyRecord, frame_id)

# Select by body FK
states = db.select_by_body(InertialStateRecord, body_id)
energy = db.select_by_body(EnergyRecord, body_id)

# Select by ID
frame = db.select_by_id(SimulationFrameRecord, 1)
mesh = db.select_by_id(MeshRecord, 1)
asset = db.select_by_id(AssetInertialStaticRecord, 1)
```

**Design note**: Prefer generic `select_all(RecordType)` / `select_by_frame(RecordType, id)` over per-type methods. This keeps the API surface small and automatically supports new record types.

### R5: Geometry Deserialization

Expose functions to decode MeshRecord vertex BLOBs:

```python
# Collision geometry: list of (x, y, z) tuples (doubles)
vertices = msd_reader.deserialize_collision_vertices(mesh.vertex_data)

# Visual geometry: list of (px, py, pz, cx, cy, cz, nx, ny, nz) tuples (floats)
visual = msd_reader.deserialize_visual_vertices(mesh.vertex_data)
```

### R6: Build Presets

Add to `CMakeUserPresets.json`:
```json
{
  "name": "debug-pybind-only",
  "displayName": "Build msd-pybind (Debug)",
  "configurePreset": "conan-debug",
  "targets": ["msd_reader"]
}
```

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `msd/msd-pybind/CMakeLists.txt` | pybind11 module build configuration |
| `msd/msd-pybind/src/msd_bindings.cpp` | PYBIND11_MODULE entry point |
| `msd/msd-pybind/src/record_bindings.cpp` | Transfer record Python classes |
| `msd/msd-pybind/src/database_bindings.cpp` | Database query wrappers |
| `msd/msd-pybind/src/geometry_bindings.cpp` | Vertex BLOB deserialization |

### Modified Files
| File | Change |
|------|--------|
| `conanfile.py` | Add `enable_pybind` option, conditional pybind11 dependency |
| `msd/CMakeLists.txt` | Conditional `add_subdirectory(msd-pybind)` |
| `CMakeUserPresets.json` | Add `debug-pybind-only` preset |

---

## Test Plan

### Python Tests (pytest)

```python
# Verify module imports
def test_import_msd_reader():
    import msd_reader

# Verify database opening
def test_open_database():
    db = msd_reader.Database("test.db")

# Verify record querying
def test_select_all_frames():
    db = msd_reader.Database("test_recording.db")
    frames = db.select_all(msd_reader.SimulationFrameRecord)
    assert len(frames) > 0
    assert hasattr(frames[0], 'simulation_time')

# Verify FK-based queries
def test_select_states_by_frame():
    db = msd_reader.Database("test_recording.db")
    states = db.select_by_frame(msd_reader.InertialStateRecord, 1)
    assert all(hasattr(s, 'position') for s in states)

# Verify body FK queries
def test_select_states_by_body():
    db = msd_reader.Database("test_recording.db")
    assets = db.select_all(msd_reader.AssetInertialStaticRecord)
    assert len(assets) > 0
    body_id = assets[0].id
    states = db.select_by_body(msd_reader.InertialStateRecord, body_id)
    assert len(states) > 0

# Verify collision result with nested contacts
def test_collision_result_contacts():
    db = msd_reader.Database("test_recording.db")
    collisions = db.select_all(msd_reader.CollisionResultRecord)
    if len(collisions) > 0:
        c = collisions[0]
        assert hasattr(c, 'normal')
        assert hasattr(c, 'contacts')  # RepeatedField
        assert hasattr(c.normal, 'x')

# Verify geometry deserialization
def test_deserialize_collision_vertices():
    db = msd_reader.Database("test_assets.db")
    meshes = db.select_all(msd_reader.MeshRecord)
    vertices = msd_reader.deserialize_collision_vertices(meshes[0].vertex_data)
    assert len(vertices) > 0
    assert len(vertices[0]) == 3  # (x, y, z)
```

---

## Acceptance Criteria

1. [x] **AC1**: `import msd_reader` succeeds after building with `ENABLE_PYBIND=ON`
2. [x] **AC2**: Can open recording database and query all Tier 1 record types
3. [x] **AC3**: FK-based queries (select by frame, select by body) return correct records
4. [x] **AC4**: Collision vertex deserialization produces correct (x, y, z) tuples
5. [x] **AC5**: Visual vertex deserialization produces correct 9-element tuples
6. [x] **AC6**: Build preset `debug-pybind-only` works
7. [x] **AC7**: Existing C++ build unaffected when `ENABLE_PYBIND=OFF`
8. [x] **AC8**: Sub-records (CoordinateRecord, ContactPointRecord, etc.) accessible as nested Python objects
9. [x] **AC9**: RepeatedField collections (CollisionResultRecord.contacts) iterable in Python

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-11
- **Completed**: 2026-02-12 21:15
- **Branch**: 0056c-python-bindings
- **PR**: N/A
- **Artifacts**: N/A
- **Notes**: Dependencies (0056a, 0056b, 0056i) all merged to main. Branch rebased and ready for implementation.

### Implementation Phase
- **Started**: 2026-02-12 15:00
- **Completed**: 2026-02-12 22:30
- **Branch**: 0056c-python-bindings
- **PR**: #45 (ready for review)
- **Artifacts**:
  - `conanfile.py` — Added `enable_pybind` option and pybind11 dependency
  - `msd/CMakeLists.txt` — Conditional `add_subdirectory(msd-pybind)`
  - `CMakeUserPresets.json` — Added `debug-pybind-only` build preset
  - `msd/msd-pybind/CMakeLists.txt` — pybind11 module build configuration
  - `msd/msd-pybind/src/msd_bindings.cpp` — PYBIND11_MODULE entry point
  - `msd/msd-pybind/src/record_bindings.cpp` — All transfer record type bindings (Tier 1/2/3)
  - `msd/msd-pybind/src/database_bindings.cpp` — Database query wrappers (select_all, select_by_id, select_by_frame, select_by_body)
  - `msd/msd-pybind/src/geometry_bindings.cpp` — Vertex BLOB deserialization
  - `msd/msd-pybind/test/test_msd_reader.py` — Pytest suite (17 tests, all pass)
  - `msd/msd-pybind/test/pytest.ini` — Pytest configuration
- **Notes**: Implementation complete. All 9 acceptance criteria verified. Python test suite confirms module imports, record types exposed, database queries work, and geometry deserialization functions correctly. Build preset `debug-pybind-only` successfully builds `msd_reader.so`. Existing C++ build unaffected when `ENABLE_PYBIND=OFF`.

---

## Dependency Compatibility Notes (2026-02-12)

Ticket updated to reflect changes from 0056a/0056b1/0056i/0056j:

| Original 0056c Reference | Current Status | Action |
|--------------------------|----------------|--------|
| `ContactRecord` | **Deleted** — replaced by `ContactPointRecord` (sub-record inside `CollisionResultRecord`) | Updated R3 |
| `ConstraintForceRecord` | **Deleted** — redundant per 0056a refactoring | Removed from R3 |
| `AppliedForceRecord` | **Deleted** — redundant per 0056a refactoring | Removed from R3 |
| `BodyMetadataRecord` | **Never existed** — replaced by `AssetInertialStaticRecord` | Updated R3 |
| `EnergyRecord.body_id` (uint32_t) | Now `EnergyRecord.body` (FK to `AssetInertialStaticRecord`) | Updated R3 |
| `InertialStateRecord` | Now has `body` FK to `AssetInertialStaticRecord` | Updated R3 |
| Per-type query methods | Replaced with generic `select_all(Type)` / `select_by_frame(Type, id)` | Updated R4 |
| — | New: `AssetInertialStaticRecord` (static body data) | Added to R3 |
| — | New: `CollisionResultRecord` with `RepeatedField<ContactPointRecord>` | Added to R3 |
| — | New: `select_by_body()` queries via body FK | Added to R4 |
| — | New dependency: 0056i (FK linkage) | Added to Depends On |
