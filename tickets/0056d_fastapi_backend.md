# Ticket 0056d: FastAPI Backend

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Feature
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-11
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
| `/simulations/{id}/frames/{fid}/state` | GET | Full frame data (states + contacts + forces) | `FrameData` |
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

class ContactInfo(BaseModel):
    body_a_id: int; body_b_id: int; contact_index: int
    point_a: Vec3; point_b: Vec3; normal: Vec3; depth: float

class ConstraintForce(BaseModel):
    body_id: int; force: Vec3; torque: Vec3

class FrameData(BaseModel):
    frame_id: int; simulation_time: float
    states: list[BodyState]
    contacts: list[ContactInfo]
    constraint_forces: list[ConstraintForce]

class AssetGeometry(BaseModel):
    asset_id: int; name: str
    positions: list[float]  # Flat [x,y,z,x,y,z,...] for BufferGeometry
    vertex_count: int
```

### R4: Service Layer

Services use `msd_reader` to query databases and convert to Pydantic models:

- `SimulationService` — wraps database queries, caches database connections
- `GeometryService` — converts MeshRecord BLOBs to flat float arrays for Three.js

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
    assert "contacts" in data

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
3. [ ] **AC3**: Frame data endpoint returns states, contacts, and forces
4. [ ] **AC4**: Bulk range endpoint returns multiple frames efficiently
5. [ ] **AC5**: Assets endpoint returns flat float arrays for Three.js BufferGeometry
6. [ ] **AC6**: Energy endpoint returns full timeseries
7. [ ] **AC7**: Static file serving works (serves index.html at `/`)
8. [ ] **AC8**: OpenAPI docs available at `/docs`

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
