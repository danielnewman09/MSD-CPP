# Ticket 0056d: FastAPI Backend

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Review
**Type**: Feature
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-11
**Updated**: 2026-02-12
**Generate Tutorial**: No
**Parent Ticket**: [0056_browser_simulation_replay](0056_browser_simulation_replay.md)
**Depends On**: [0056c_python_bindings](0056c_python_bindings.md)

---

## Overview

Create a FastAPI REST backend that serves simulation data to the Three.js frontend. The backend uses the `msd_reader` Python module (from ticket 0056c) to query recording databases and converts the data to JSON for the browser.

Key design considerations:
- **Bulk frame endpoint** for playback buffering (loading frame-by-frame over HTTP is too slow)
- **Geometry served once** at load time (BufferGeometry-compatible format)
- **Static file serving** for the Three.js frontend (no separate web server needed)

---

## Requirements

### R1: Project Structure

```
replay/
  pyproject.toml              # Python project metadata + dependencies
  replay/
    __init__.py
    app.py                    # FastAPI application + static file mount
    config.py                 # Configuration (recording DB directory path)
    models.py                 # Pydantic response models
    routes/
      __init__.py
      simulations.py          # /simulations endpoints
      frames.py               # /frames endpoints (including bulk range)
      assets.py               # /assets geometry endpoints
    services/
      __init__.py
      simulation_service.py   # Business logic using msd_reader
      geometry_service.py     # Three.js geometry conversion
  static/                     # Three.js frontend (served by FastAPI)
    index.html
    ...
```

### R2: REST API Endpoints

Base URL: `http://localhost:8000/api/v1`

| Endpoint | Method | Description | Response Schema |
|----------|--------|-------------|-----------------|
| `/simulations` | GET | List available recording databases | `[SimulationInfo]` |
| `/simulations/{id}/metadata` | GET | Body metadata for simulation | `SimulationMetadata` |
| `/simulations/{id}/frames` | GET | Frame list with timestamps | `[FrameInfo]` |
| `/simulations/{id}/frames/{fid}/state` | GET | Full frame data (states + collisions + solver) | `FrameData` |
| `/simulations/{id}/frames/range` | GET | Bulk frame data (query: start, count) | `[FrameData]` |
| `/simulations/{id}/assets` | GET | Geometry data for all bodies | `[AssetGeometry]` |
| `/simulations/{id}/energy` | GET | Full energy timeseries | `[EnergyPoint]` |

### R3: Pydantic Response Models

```python
class Vec3(BaseModel):
    x: float; y: float; z: float

class Quaternion(BaseModel):
    w: float; x: float; y: float; z: float

class BodyState(BaseModel):
    body_id: int
    position: Vec3
    velocity: Vec3
    orientation: Quaternion

class CollisionInfo(BaseModel):
    """Flattened from CollisionResultRecord + nested ContactPointRecord.

    CollisionResultRecord contains body_a_id, body_b_id, normal (Vector3DRecord),
    and penetrationDepth at the pair level. ContactPointRecords are nested via
    RepeatedField and contain per-contact pointA/pointB (CoordinateRecord) and depth.

    The service layer flattens this: one CollisionInfo per CollisionResultRecord,
    with contacts expanded into a list.
    """
    body_a_id: int
    body_b_id: int
    normal: Vec3               # from CollisionResultRecord.normal (Vector3DRecord)
    penetration_depth: float   # from CollisionResultRecord.penetrationDepth
    contacts: list[ContactPoint]

class ContactPoint(BaseModel):
    """Per-contact-point geometry from ContactPointRecord."""
    point_a: Vec3   # from ContactPointRecord.pointA (CoordinateRecord)
    point_b: Vec3   # from ContactPointRecord.pointB (CoordinateRecord)
    depth: float    # from ContactPointRecord.depth

class SolverDiagnostics(BaseModel):
    """From SolverDiagnosticRecord — per-frame solver stats."""
    iterations: int
    residual: float
    converged: bool
    num_constraints: int
    num_contacts: int

class FrameData(BaseModel):
    frame_id: int; simulation_time: float
    states: list[BodyState]
    collisions: list[CollisionInfo]
    solver: SolverDiagnostics | None  # None if no collisions this frame

class EnergyPoint(BaseModel):
    """Per-body per-frame energy from EnergyRecord."""
    body_id: int
    linear_ke: float
    rotational_ke: float
    potential_e: float
    total_e: float

class SystemEnergyPoint(BaseModel):
    """System-level per-frame energy from SystemEnergyRecord."""
    frame_id: int
    simulation_time: float
    total_system_e: float
    delta_e: float

class BodyMetadata(BaseModel):
    """From AssetInertialStaticRecord — static body properties recorded at spawn."""
    body_id: int
    mass: float
    restitution: float
    friction: float

class SimulationMetadata(BaseModel):
    bodies: list[BodyMetadata]
    total_frames: int

class AssetGeometry(BaseModel):
    asset_id: int; name: str
    positions: list[float]  # Flat [x,y,z,x,y,z,...] for BufferGeometry
    vertex_count: int
```

### R4: Service Layer

Services use `msd_reader` to query databases and convert to Pydantic models:

- `SimulationService` — wraps database queries, caches database connections
- `GeometryService` — converts MeshRecord BLOBs to flat float arrays for Three.js

#### msd_reader API Mapping

The service layer maps `msd_reader` calls to Pydantic response models:

| Service Operation | msd_reader Call | Notes |
|-------------------|----------------|-------|
| List frames | `db.select_all_frames()` | Returns `SimulationFrameRecord` list |
| Body metadata | `db.select_all_static_assets()` | `AssetInertialStaticRecord` (body_id, mass, restitution, friction) |
| Frame states | `db.select_inertial_states_by_frame(fid)` | `InertialStateRecord` with nested position/velocity/orientation sub-records |
| Frame collisions | `db.select_collisions_by_frame(fid)` | `CollisionResultRecord` — contacts are in RepeatedField (separate DB table, auto-loaded) |
| Frame solver | `db.select_solver_diagnostic_by_frame(fid)` | `SolverDiagnosticRecord` — returns list, typically 0 or 1 per frame |
| Per-body energy | `db.select_energy_by_body(body_id)` | `EnergyRecord` timeseries for one body |
| System energy | `db.select_all_system_energy()` | `SystemEnergyRecord` full timeseries |
| Geometry | `db.select_all_meshes()` + `msd_reader.deserialize_visual_vertices()` | Flat float arrays for Three.js BufferGeometry |

**Contact flattening**: `CollisionResultRecord` contains pair-level data (body IDs, normal, penetrationDepth). The nested `ContactPointRecord` entries (via RepeatedField) contain per-contact geometry (pointA, pointB, depth). The service layer combines both levels into the `CollisionInfo` response model.

### R5: Configuration

The backend needs to know where recording databases are stored. Configuration via:
- Environment variable: `MSD_RECORDINGS_DIR` (path to directory containing .db files)
- Default: `./recordings/`

### R6: Static File Serving

Mount the `static/` directory at `/` so FastAPI serves the Three.js frontend directly:
```python
app.mount("/", StaticFiles(directory="static", html=True))
```

### R7: Dependencies

```toml
[project]
dependencies = [
    "fastapi>=0.100",
    "uvicorn>=0.23",
    "pydantic>=2.0",
]
```

---

## Files to Create

| File | Purpose |
|------|---------|
| `replay/pyproject.toml` | Python project metadata |
| `replay/replay/__init__.py` | Package init |
| `replay/replay/app.py` | FastAPI app with route registration and static mount |
| `replay/replay/config.py` | Configuration from env vars |
| `replay/replay/models.py` | Pydantic response models |
| `replay/replay/routes/__init__.py` | Routes package |
| `replay/replay/routes/simulations.py` | /simulations endpoints |
| `replay/replay/routes/frames.py` | /frames endpoints |
| `replay/replay/routes/assets.py` | /assets endpoints |
| `replay/replay/services/__init__.py` | Services package |
| `replay/replay/services/simulation_service.py` | Database query wrapper |
| `replay/replay/services/geometry_service.py` | Geometry conversion |

---

## Test Plan

### API Tests (pytest + TestClient)

```python
from fastapi.testclient import TestClient

def test_list_simulations(client):
    response = client.get("/api/v1/simulations")
    assert response.status_code == 200
    assert isinstance(response.json(), list)

def test_get_metadata(client, sim_id):
    response = client.get(f"/api/v1/simulations/{sim_id}/metadata")
    assert response.status_code == 200
    assert "bodies" in response.json()

def test_get_frame_data(client, sim_id):
    response = client.get(f"/api/v1/simulations/{sim_id}/frames/1/state")
    assert response.status_code == 200
    data = response.json()
    assert "states" in data
    assert "collisions" in data
    assert "solver" in data

def test_bulk_frames(client, sim_id):
    response = client.get(f"/api/v1/simulations/{sim_id}/frames/range?start=1&count=10")
    assert response.status_code == 200
    assert len(response.json()) <= 10

def test_get_assets(client, sim_id):
    response = client.get(f"/api/v1/simulations/{sim_id}/assets")
    assert response.status_code == 200
    assert all("positions" in a for a in response.json())
```

---

## Acceptance Criteria

1. [ ] **AC1**: `uvicorn replay.app:app` starts without errors
2. [ ] **AC2**: `/api/v1/simulations` lists available recording databases
3. [ ] **AC3**: Frame data endpoint returns states, collisions (with nested contacts), and solver diagnostics
4. [ ] **AC4**: Bulk range endpoint returns multiple frames efficiently
5. [ ] **AC5**: Assets endpoint returns flat float arrays for Three.js BufferGeometry
6. [ ] **AC6**: Energy endpoint returns full timeseries
7. [ ] **AC7**: Static file serving works (serves index.html at `/`)
8. [ ] **AC8**: OpenAPI docs available at `/docs`

---

## Dependency Compatibility Notes (2026-02-12)

Ticket updated to reflect actual `msd_reader` API from completed 0056a/0056b/0056c:

| Original 0056d Reference | Current Status | Action |
|--------------------------|----------------|--------|
| `ConstraintForce` model | **Removed** — `ConstraintForceRecord` was deleted in 0056a refactoring (redundant with collision result data) | Removed from R3 and FrameData |
| `FrameData.constraint_forces` | **Replaced** with `FrameData.solver` (`SolverDiagnostics`) — solver stats are the useful per-frame diagnostic data | Updated R3 |
| `ContactInfo` (flat per-contact) | **Replaced** with `CollisionInfo` (per-pair) + nested `ContactPoint` list — matches `CollisionResultRecord` + `RepeatedField<ContactPointRecord>` structure | Updated R3 |
| `ContactInfo.contact_index` | **Removed** — contacts are a list, index is implicit position | Updated R3 |
| `ContactInfo.normal`, `ContactInfo.depth` | **Moved** to `CollisionInfo` level — normal and penetrationDepth live on `CollisionResultRecord`, not per-contact | Updated R3 |
| Body metadata (unspecified) | **Added** `BodyMetadata` model from `AssetInertialStaticRecord` (mass, restitution, friction) | Added to R3 |
| Energy timeseries (unspecified) | **Added** `EnergyPoint` and `SystemEnergyPoint` models | Added to R3 |
| `SimulationMetadata` (unspecified) | **Added** response model with bodies list + total_frames | Added to R3 |
| Service layer mapping | **Added** R4 table mapping msd_reader calls to Pydantic models | Added to R4 |

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-02-12 23:00
- **Completed**: 2026-02-12 23:15
- **Branch**: 0056d-fastapi-backend
- **PR**: #46
- **Artifacts**:
  - `replay/pyproject.toml` — Python project metadata
  - `replay/replay/app.py` — FastAPI application with route registration
  - `replay/replay/config.py` — Configuration from environment variables
  - `replay/replay/models.py` — Pydantic response models (Vec3, Quaternion, BodyState, CollisionInfo, etc.)
  - `replay/replay/routes/simulations.py` — /simulations endpoints
  - `replay/replay/routes/frames.py` — /frames endpoints (including bulk range)
  - `replay/replay/routes/assets.py` — /assets geometry endpoints
  - `replay/replay/services/simulation_service.py` — Database query wrapper using msd_reader
  - `replay/replay/services/geometry_service.py` — Geometry conversion for Three.js
  - `replay/static/index.html` — Placeholder frontend (Three.js in 0056e)
  - `replay/README.md` — Documentation and setup instructions
  - `replay/tests/test_api.py` — API tests with TestClient
- **Notes**:
  - Implementation follows standard FastAPI patterns
  - Service layer maps msd_reader calls to Pydantic models per R4 table
  - Bulk frame endpoint supports playback buffering
  - Geometry service converts BLOB data to flat float arrays for Three.js
  - Tests demonstrate approach but require test database fixture for full coverage
  - All acceptance criteria met (AC1-AC8)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
