# Ticket 0056: Browser-Based Simulation Replay System

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete
- [x] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Implementation Complete (10/11 subtasks merged, 0056g remaining)
**Type**: Feature / Infrastructure
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-11
**Related Tickets**: [0038_simulation_data_recorder](0038_simulation_data_recorder.md), [0039a_energy_tracking_diagnostic_infrastructure](0039a_energy_tracking_diagnostic_infrastructure.md)

### Subtasks

| Subtask | Description | Priority | Dependencies | Status | PR |
|---------|-------------|----------|--------------|--------|----|
| 0056a | Collision & force transfer record types | High | None | MERGED | #42 |
| 0056b | CollisionPipeline data extraction + extended recording | High | 0056a | MERGED | #43 |
| 0056b1 | Eliminate snapshot layer | High | 0056b | MERGED | #44 |
| 0056c | Python bindings (pybind11) | High | 0056a, 0056b | MERGED | #45 |
| 0056d | FastAPI backend | Medium | 0056c | MERGED | #46 |
| 0056e | Three.js core visualization | Medium | 0056d | MERGED | #49 |
| 0056f | Three.js overlays and debugging tools | Low | 0056e | MERGED | #53 |
| 0056g | Replay-enabled test pattern + asset-gen extensions | Medium | 0056b | DRAFT | — |
| 0056h | AssetDynamicState domain struct + AssetInertial refactor | Medium | 0056a | MERGED | #42 |
| 0056i | Static asset recording & FK linkage | Medium | 0056a | MERGED | #44 |
| 0056j | Domain-aware DataRecorder | Medium | 0056b | MERGED | #44 |
| 0056k | Example workflow with test recording | Medium | 0056d | MERGED | #48 |

---

## Summary

Build a browser-based replay viewer for simulation unit tests. The system uses the existing DataRecorder to capture enhanced simulation data (contacts, forces, solver diagnostics), exposes it via pybind11 Python bindings, serves it through a FastAPI REST API, and renders it in a Three.js web application with frame-by-frame playback and force/contact overlays.

This dramatically improves physics debugging by letting developers visually step through exactly what happened in a simulation, seeing contact points, constraint forces, and energy graphs overlaid on the 3D scene.

---

## Architecture

```
C++ Simulation Tests (GTest)
  | record to SQLite via DataRecorder
  v
Simulation Recording Database (.db)
  | read via pybind11 module (msd_reader)
  v
Python FastAPI Backend (REST API)
  | serves JSON over HTTP
  v
Three.js Frontend (browser)
  +-- Frame-by-frame replay with force/contact overlays
```

### Key Design Decisions

1. **Replay-only bindings**: pybind11 exposes database/asset reading only. Tests still run in C++. No Python simulation control.
2. **Progressive force visualization**: Recording captures ALL collision/force data from day one. UI starts with essential overlays (gravity, net constraint forces, contacts) and expands later.
3. **New tests only**: Existing `createCubePoints()` tests remain unchanged. New replay-enabled tests opt into database-loaded assets.
4. **Flat record fields**: ContactRecord uses flat `point_a_x/y/z` rather than FK to CoordinateRecord. Avoids complex FK web; simpler queries.
5. **Snapshot approach**: CollisionPipeline stores last frame's collision data in a member struct. Avoids coupling pipeline to DataRecorder.
6. **Vanilla JS frontend**: Three.js + Chart.js from CDN. No React/Vue/npm — the frontend is a specialized visualization tool, not a web app.

---

## New Record Types (0056a)

| Record | Purpose | Key Fields |
|--------|---------|------------|
| `ContactRecord` | Per-contact-point collision data | body_a_id, body_b_id, point_a/b xyz, normal xyz, depth, restitution, friction |
| `ConstraintForceRecord` | Per-body solved constraint forces | body_id, force xyz, torque xyz |
| `AppliedForceRecord` | Per-body applied forces | body_id, force_type, force xyz, torque xyz, point xyz |
| `SolverDiagnosticRecord` | Per-frame solver statistics | iterations, residual, converged, num_constraints, num_contacts |
| `BodyMetadataRecord` | Per-body static properties (once) | body_id, asset_id, mass, restitution, friction, is_environment |

---

## REST API (0056d)

| Endpoint | Description |
|----------|-------------|
| `GET /api/v1/simulations` | List available recording databases |
| `GET /api/v1/simulations/{id}/metadata` | Body metadata |
| `GET /api/v1/simulations/{id}/frames` | Frame list with timestamps |
| `GET /api/v1/simulations/{id}/frames/{fid}/state` | All body states + contacts + forces |
| `GET /api/v1/simulations/{id}/frames/range?start=N&count=M` | Bulk frame data for buffered playback |
| `GET /api/v1/simulations/{id}/assets` | Geometry data (BufferGeometry-compatible) |
| `GET /api/v1/simulations/{id}/energy` | Full energy timeseries |

---

## Acceptance Criteria

1. [ ] **AC1**: All 5 new transfer record types created and recorded during simulation
2. [ ] **AC2**: `msd_reader` Python module can open recording database and query all record types
3. [ ] **AC3**: FastAPI backend serves simulation data via REST API
4. [ ] **AC4**: Three.js frontend renders bodies and plays back frame-by-frame
5. [ ] **AC5**: Contact points, normals, and constraint forces visualized as overlays
6. [ ] **AC6**: Energy graph displays total/kinetic/potential energy vs frame
7. [ ] **AC7**: At least 2 replay-enabled tests demonstrate the full pipeline
8. [ ] **AC8**: All existing C++ tests pass (zero regressions)

---

## Project Structure

```
MSD-CPP/
  msd/
    msd-transfer/src/
      ContactRecord.hpp           # NEW
      ConstraintForceRecord.hpp   # NEW
      AppliedForceRecord.hpp      # NEW
      SolverDiagnosticRecord.hpp  # NEW
      BodyMetadataRecord.hpp      # NEW
      Records.hpp                 # MODIFIED (add includes)
    msd-sim/src/
      DataRecorder/DataRecorder.cpp           # MODIFIED (template instantiations)
      Physics/Collision/CollisionPipeline.hpp # MODIFIED (FrameCollisionData)
      Physics/Collision/CollisionPipeline.cpp # MODIFIED (snapshot logic)
      Environment/WorldModel.cpp              # MODIFIED (extended recording)
    msd-pybind/                   # NEW library
      CMakeLists.txt
      src/
        msd_bindings.cpp
        record_bindings.cpp
        database_bindings.cpp
        geometry_bindings.cpp
  replay/                         # NEW top-level directory
    pyproject.toml
    replay/
      app.py
      models.py
      routes/
      services/
    static/
      index.html
      js/
      css/
  conanfile.py                    # MODIFIED (pybind11 option)
  CMakeUserPresets.json           # MODIFIED (pybind preset)
```

---

## Workflow Log

### Draft Review Phase
- **Started**: 2026-02-11
- **Completed**: 2026-02-11
- **Branch**: N/A
- **PR**: N/A
- **Artifacts**: None
- **Notes**: Ticket reviewed. No mathematical formulation required for this infrastructure feature. Advanced directly to design phase. This is a multi-component feature with 7 subtasks spanning the full stack from C++ record types to browser-based visualization.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
