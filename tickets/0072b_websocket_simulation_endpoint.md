# Ticket 0072b: WebSocket Simulation Endpoint

## Status
- [x] Draft
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Feature
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-17
**Generate Tutorial**: No
**Requires Math Design**: No
**Parent Ticket**: [0072_live_browser_simulation](0072_live_browser_simulation.md)
**Depends On**: [0072a_engine_pybind_bindings](0072a_engine_pybind_bindings.md)
**Blocks**: [0072c_live_simulation_frontend](0072c_live_simulation_frontend.md)

---

## Overview

Add a WebSocket endpoint to the FastAPI replay server that accepts spawn configurations, runs the C++ Engine via pybind11, and streams frame data to the browser in real-time. Also adds a REST endpoint for listing available assets.

---

## Requirements

### R1: WebSocket Route

Create `replay/replay/routes/live.py` with a WebSocket endpoint at `/api/v1/live`.

### R2: Wire Protocol

JSON messages over WebSocket:

**Client -> Server:**

```json
{"type": "configure", "objects": [
  {"asset_name": "cube", "position": [0, 0, 5], "orientation": [0, 0, 0],
   "object_type": "inertial", "mass": 10.0, "restitution": 0.8, "friction": 0.5},
  {"asset_name": "large_cube", "position": [0, 0, 0], "orientation": [0, 0, 0],
   "object_type": "environment"}
]}

{"type": "start", "timestep_ms": 16, "duration_s": 30.0}

{"type": "stop"}
```

**Server -> Client:**

```json
{"type": "metadata", "bodies": [
  {"body_id": 1, "asset_id": 3, "asset_name": "cube", "mass": 10.0,
   "restitution": 0.8, "friction": 0.5, "is_environment": false}
], "assets": [
  {"asset_id": 3, "positions": [x1,y1,z1, x2,y2,z2, ...]}
]}

{"type": "frame", "data": {
  "frame_id": 0, "simulation_time": 0.016,
  "states": [...], "collisions": [], "solver": null
}}

{"type": "complete", "total_frames": 1875, "elapsed_s": 30.0}

{"type": "error", "message": "Asset 'nonexistent' not found"}
```

### R3: Simulation Loop

```python
async def run_simulation(engine, websocket, timestep_ms, duration_s):
    sim_time_ms = 0
    max_time_ms = int(duration_s * 1000)
    frame_count = 0
    target_interval = timestep_ms / 1000.0  # real-time pacing

    while sim_time_ms <= max_time_ms:
        t0 = time.monotonic()

        # CPU-bound work in thread pool
        await asyncio.to_thread(engine.update, sim_time_ms)
        frame_dict = await asyncio.to_thread(engine.get_frame_state)
        frame_dict["frame_id"] = frame_count

        await websocket.send_json({"type": "frame", "data": frame_dict})

        sim_time_ms += timestep_ms
        frame_count += 1

        # Real-time pacing
        elapsed = time.monotonic() - t0
        if elapsed < target_interval:
            await asyncio.sleep(target_interval - elapsed)
```

### R4: Error Handling

- Invalid asset name in `configure` -> send `{"type": "error"}` and close
- WebSocket disconnect during simulation -> cancel task, clean up Engine
- Use `try/finally` for Engine cleanup

### R5: REST Asset List Endpoint

Add `GET /api/v1/live/assets` that returns available asset types:

```json
[{"asset_id": 1, "name": "cube"}, {"asset_id": 2, "name": "large_cube"}, ...]
```

This uses the `msd_reader.Engine` or existing `AssetRegistry` to list assets.

### R6: Pydantic Models

Add to `replay/replay/models.py`:

```python
class SpawnObjectConfig(BaseModel):
    asset_name: str
    position: list[float]       # [x, y, z]
    orientation: list[float]    # [pitch, roll, yaw] in radians
    object_type: str            # "inertial" or "environment"
    mass: float = 10.0
    restitution: float = 0.8
    friction: float = 0.5
```

### R7: App Registration

- Register `live_router` in `replay/replay/app.py`
- Add `GET /live` route serving `live.html` (for ticket 0072c)

---

## Constraints
- Engine is not thread-safe — `asyncio.to_thread()` serializes calls through one thread pool thread
- `msd_reader` module must be importable (pybind module in PYTHONPATH)
- Asset database path comes from `config.assets_db_path` (already set by `start_server.sh`)
- Each WebSocket connection creates a new Engine instance (memory-isolated)

## Acceptance Criteria
- [ ] WebSocket connection to `ws://localhost:8000/api/v1/live` succeeds
- [ ] `configure` message spawns objects and returns `metadata` with body info and geometry
- [ ] `start` message begins streaming `frame` messages
- [ ] Frame data matches `FrameData` schema (same structure as replay endpoints)
- [ ] `stop` message or disconnect cleanly terminates simulation
- [ ] `GET /api/v1/live/assets` returns available asset list
- [ ] Invalid asset name returns `error` message
- [ ] Integration test suite passes: `test_live_api.py`

---

## Design Decisions (Human Input)

### Preferred Approaches
- Use `asyncio.to_thread()` for all Engine calls (keep event loop responsive)
- Real-time pacing: sleep to match timestep rate, send as fast as possible if sim is slower
- Include asset geometry in `metadata` message so frontend doesn't need a separate fetch
- Engine constructed with `config.assets_db_path` from existing server config

### Things to Avoid
- Do not use subprocess for simulation (in-process is simpler and sufficient)
- Do not modify existing REST endpoints in `routes/simulations.py`, `routes/frames.py`, `routes/assets.py`
- Do not add persistent state across WebSocket connections

---

## References

### Related Code
- `replay/replay/app.py` — FastAPI app to extend
- `replay/replay/routes/frames.py` — Existing REST pattern for frame data
- `replay/replay/models.py` — Existing Pydantic models (FrameData, BodyState, etc.)
- `replay/replay/services/simulation_service.py` — Service pattern reference
- `replay/replay/config.py` — Config with `assets_db_path`

### Related Tickets
- [0072a_engine_pybind_bindings](0072a_engine_pybind_bindings.md) — Engine Python bindings (prerequisite)
- [0056e_threejs_core_visualization](0056e_threejs_core_visualization.md) — Original Three.js viewer

---

## Workflow Log

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
- **Files Modified**:
- **Notes**:

### Implementation Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

---

## Human Feedback
