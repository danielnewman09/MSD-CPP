# Ticket 0056c: Python Bindings (pybind11)

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Feature
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-11
**Generate Tutorial**: No
**Parent Ticket**: [0056_browser_simulation_replay](0056_browser_simulation_replay.md)
**Depends On**: [0056a_collision_force_transfer_records](0056a_collision_force_transfer_records.md), [0056b_collision_pipeline_data_extraction](0056b_collision_pipeline_data_extraction.md)

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

Bind all transfer record types as Python classes with read-only properties:

- `SimulationFrameRecord` — id, simulation_time, wall_clock_time
- `InertialStateRecord` — position (tuple), velocity (tuple), orientation (tuple), frame_id
- `EnergyRecord` — body_id, linear_ke, rotational_ke, potential_e, total_e, frame_id
- `SystemEnergyRecord` — totals, delta_e, energy_injection, collision_active, frame_id
- `ContactRecord` — body_a_id, body_b_id, point_a/b (tuples), normal (tuple), depth, frame_id
- `ConstraintForceRecord` — body_id, force/torque (tuples), frame_id
- `AppliedForceRecord` — body_id, force_type, force/torque/point (tuples), frame_id
- `SolverDiagnosticRecord` — iterations, residual, converged, num_constraints, frame_id
- `BodyMetadataRecord` — body_id, asset_id, mass, restitution, friction, is_environment
- `MeshRecord` — id, vertex_data (bytes), vertex_count
- `ObjectRecord` — id, name, category, mesh FK IDs

### R4: Database Query Bindings

Wrap `cpp_sqlite::Database` with read-only query methods:

```python
db = msd_reader.Database("recording.db")

# Select all records of a type
frames = db.select_all_frames()
states = db.select_all_states()
contacts = db.select_all_contacts()
metadata = db.select_all_metadata()

# Select by frame FK (custom WHERE query)
states = db.select_states_by_frame(frame_id)
contacts = db.select_contacts_by_frame(frame_id)
forces = db.select_constraint_forces_by_frame(frame_id)
applied = db.select_applied_forces_by_frame(frame_id)
solver = db.select_solver_diagnostics_by_frame(frame_id)
energy = db.select_energy_by_frame(frame_id)

# Select by ID
frame = db.select_frame_by_id(1)
mesh = db.select_mesh_by_id(1)
```

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
    frames = db.select_all_frames()
    assert len(frames) > 0
    assert hasattr(frames[0], 'simulation_time')

# Verify FK-based queries
def test_select_states_by_frame():
    db = msd_reader.Database("test_recording.db")
    states = db.select_states_by_frame(1)
    assert all(hasattr(s, 'position') for s in states)

# Verify geometry deserialization
def test_deserialize_collision_vertices():
    db = msd_reader.Database("test_assets.db")
    meshes = db.select_all_meshes()
    vertices = msd_reader.deserialize_collision_vertices(meshes[0].vertex_data)
    assert len(vertices) > 0
    assert len(vertices[0]) == 3  # (x, y, z)
```

---

## Acceptance Criteria

1. [ ] **AC1**: `import msd_reader` succeeds after building with `ENABLE_PYBIND=ON`
2. [ ] **AC2**: Can open recording database and query all record types
3. [ ] **AC3**: FK-based queries (select by frame) return correct records
4. [ ] **AC4**: Collision vertex deserialization produces correct (x, y, z) tuples
5. [ ] **AC5**: Visual vertex deserialization produces correct 9-element tuples
6. [ ] **AC6**: Build preset `debug-pybind-only` works
7. [ ] **AC7**: Existing C++ build unaffected when `ENABLE_PYBIND=OFF`

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
