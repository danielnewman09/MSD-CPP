# Integration Design: Live Simulation Cleanup (0072e)

**Ticket**: [0072e_live_simulation_cleanup](../../../tickets/0072e_live_simulation_cleanup.md)
**Date**: 2026-02-21
**Status**: Initial Integration Design
**Depends On**: [design.md](./design.md) (APPROVED WITH NOTES)

---

## Purpose

This document defines the cross-language integration contracts between the three layers of the
live simulation stack:

1. **C++ pybind11 layer** — `EngineWrapper::getFrameState()` in `msd/msd-pybind/src/engine_bindings.cpp`
2. **Python FastAPI layer** — `live.py` WebSocket endpoint and `models.py` Pydantic types
3. **JavaScript frontend** — `live-app.js` configure payload and frame handling

The ticket addresses six functional requirements (FR-1 through FR-6) identified in retroactive
design reviews of 0072a, 0072b, and 0072c. All six touch at least one layer boundary, making a
formal integration contract the appropriate next design artifact.

---

## Layer Architecture

```
Browser (JavaScript)
       |  WebSocket (JSON)
       v
Python FastAPI / WebSocket endpoint (live.py)
       |  pybind11 Python API
       v
C++ EngineWrapper (engine_bindings.cpp)
       |  C++ API
       v
msd_sim::Engine / WorldModel
```

Data flows in two directions:
- **Client → Server**: configure and control messages (spawn configs, start/stop)
- **Server → Client**: metadata, frame, complete, and error messages

The pybind11 boundary is internal (Python calls C++); it does not cross a network.

---

## Contract 1: pybind11 Boundary — `EngineWrapper::getFrameState()`

### Location

`msd/msd-pybind/src/engine_bindings.cpp` → called from `live.py` simulation loop via
`asyncio.to_thread(engine.get_frame_state)`.

### Current Return Schema (pre-0072e)

```
{
  "simulation_time": float,
  "states": [
    {
      "body_id":          int,
      "asset_id":         int,
      "position":         {"x": float, "y": float, "z": float},
      "velocity":         {"x": float, "y": float, "z": float},
      "orientation":      {"w": float, "x": float, "y": float, "z": float},
      "angular_velocity": {"x": float, "y": float, "z": float}
      // NOTE: no is_environment field; only inertial bodies included
    }
  ]
}
```

### Post-0072e Return Schema (FR-1)

```
{
  "simulation_time": float,           // seconds (worldModel.getTime() / 1000.0)
  "states": [
    // Inertial (dynamic) bodies — existing fields plus new is_environment flag:
    {
      "body_id":          int,         // asset.getInstanceId()
      "asset_id":         int,         // asset.getAssetId()
      "position":         {"x": float, "y": float, "z": float},
      "velocity":         {"x": float, "y": float, "z": float},
      "orientation":      {"w": float, "x": float, "y": float, "z": float},
      "angular_velocity": {"x": float, "y": float, "z": float},
      "is_environment":   false        // NEW: discriminator field
    },
    // Environment (static) bodies — NEW entries appended after inertial bodies:
    {
      "body_id":          int,         // asset.getInstanceId()
      "asset_id":         int,         // asset.getAssetId()
      "position":         {"x": float, "y": float, "z": float},
      "velocity":         {"x": 0.0, "y": 0.0, "z": 0.0},    // always zero
      "orientation":      {"w": float, "x": float, "y": float, "z": float},
      "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},    // always zero
      "is_environment":   true         // NEW: discriminator field
    }
  ]
}
```

### Key Contract Properties

| Property | Value |
|----------|-------|
| Ordering | Inertial bodies first (spawn order), then environment bodies (spawn order) |
| `body_id` source | `asset.getInstanceId()` — matches IDs in `LiveBodyMetadata` from configure |
| `asset_id` source | `asset.getAssetId()` — matches IDs used in `AssetGeometry` |
| `is_environment` type | Python `bool` (pybind11 maps C++ `true`/`false` to Python `True`/`False`) |
| Environment velocity | Literal `0.0` floats, not NaN (these are wire protocol values, not C++ members) |
| Environment orientation | From `AssetEnvironment::getInertialState().orientation` — quaternion set at spawn time, never mutated |

### Consumption in `live.py`

The simulation loop (Phase 3 of `live_simulation`) consumes this dict without transformation:

```python
frame_dict: dict = await asyncio.to_thread(engine.get_frame_state)
frame_dict["frame_id"] = total_frames  # augmented by Python layer
await websocket.send_json({"type": "frame", "data": frame_dict})
```

The `is_environment` field and new environment entries are forwarded transparently.
The Python layer does not filter or reshape the states list.

### `body_id` Consistency Guarantee

The `configure` phase assigns sequential `body_id` values starting at 1 to each spawned object
(both inertial and environment), stored in `LiveBodyMetadata`. The C++ `getInstanceId()` method
returns the same ID (assigned by the Engine at spawn time). The client can therefore use the
`body_id` in frame entries to look up metadata from the `metadata` message without additional
mapping.

---

## Contract 2: WebSocket Boundary — Client → Server Messages

### Message: `configure`

Sent by `live-app.js::onStartSimulation()`. Received and parsed by `live.py::live_simulation()`.

#### Pre-0072e Wire Format

```json
{
  "type": "configure",
  "objects": [
    {
      "asset_name": "cube",
      "position": [0.0, 0.0, 5.0],
      "orientation": [0.0, 0.0, 0.0],
      "object_type": "inertial",
      "mass": 10.0,
      "restitution": 0.8,
      "friction": 0.5
    },
    {
      "asset_name": "plane",
      "position": [0.0, 0.0, 0.0],
      "orientation": [0.0, 0.0, 0.0],
      "object_type": "environment",
      "mass": 10.0,       // meaningless for environment objects; sent anyway
      "restitution": 0.8,
      "friction": 0.5
    }
  ]
}
```

#### Post-0072e Wire Format (FR-6)

```json
{
  "type": "configure",
  "objects": [
    {
      "asset_name": "cube",
      "position": [0.0, 0.0, 5.0],
      "orientation": [0.0, 0.0, 0.0],
      "object_type": "inertial",
      "mass": 10.0,
      "restitution": 0.8,
      "friction": 0.5
    },
    {
      "asset_name": "plane",
      "position": [0.0, 0.0, 0.0],
      "orientation": [0.0, 0.0, 0.0],
      "object_type": "environment"
      // mass, restitution, friction OMITTED for environment objects
      // Server applies Pydantic defaults: mass=10.0, restitution=0.8, friction=0.5
    }
  ]
}
```

#### `SpawnObjectConfig` Validation (FR-3, FR-4)

The Python layer validates each entry via `SpawnObjectConfig(**obj)` immediately after receiving
the configure message. Post-0072e, validation rejects:

| Violation | Current Behavior | Post-0072e Behavior |
|-----------|-----------------|---------------------|
| `object_type` not in `{"inertial", "environment"}` | Passes validation, may cause runtime error | Pydantic `ValidationError` → `error` WebSocket message |
| `len(position) != 3` | Passes validation, causes C++ argument error | Pydantic `ValidationError` → `error` WebSocket message |
| `len(orientation) != 3` | Passes validation, causes C++ argument error | Pydantic `ValidationError` → `error` WebSocket message |

The WebSocket endpoint catches `ValidationError` exceptions (from the `SpawnObjectConfig(**obj)`
call at the configure phase) and sends:

```json
{"type": "error", "message": "<pydantic validation error text>"}
```

then closes the connection.

### Messages: `start` and `stop`

Unchanged by this ticket. No integration contract changes.

---

## Contract 3: WebSocket Boundary — Server → Client Messages

### Message: `metadata`

Sent once after successful `configure` processing. Unchanged structure, but now includes
`LiveBodyMetadata` entries for environment objects as well as inertial objects (this was already
the case from 0072b/0072c — no change in 0072e).

```json
{
  "type": "metadata",
  "bodies": [
    {
      "body_id": 1,
      "asset_id": 3,
      "asset_name": "cube",
      "mass": 10.0,
      "restitution": 0.8,
      "friction": 0.5,
      "is_environment": false
    },
    {
      "body_id": 2,
      "asset_id": 7,
      "asset_name": "plane",
      "mass": 10.0,
      "restitution": 0.8,
      "friction": 0.5,
      "is_environment": true
    }
  ],
  "assets": [/* AssetGeometry entries */]
}
```

Note: for environment objects where `mass`/`restitution`/`friction` are omitted from the
`configure` payload (FR-6), the `LiveBodyMetadata` records will carry the Pydantic defaults
(`mass=10.0`, `restitution=0.8`, `friction=0.5`). The frontend `SceneManager` does not use
these fields for rendering.

### Message: `frame`

The `frame` message carries the raw dict from `engine.get_frame_state()` with a `frame_id`
field added by the Python layer.

#### Pre-0072e Frame Data

```json
{
  "type": "frame",
  "data": {
    "simulation_time": 0.016,
    "frame_id": 0,
    "states": [
      {
        "body_id": 1,
        "asset_id": 3,
        "position": {"x": 0.0, "y": 0.0, "z": 4.998},
        "velocity": {"x": 0.0, "y": 0.0, "z": -0.157},
        "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0}
        // No is_environment; no environment object entries
      }
    ]
  }
}
```

#### Post-0072e Frame Data (FR-1)

```json
{
  "type": "frame",
  "data": {
    "simulation_time": 0.016,
    "frame_id": 0,
    "states": [
      {
        "body_id": 1,
        "asset_id": 3,
        "position": {"x": 0.0, "y": 0.0, "z": 4.998},
        "velocity": {"x": 0.0, "y": 0.0, "z": -0.157},
        "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "is_environment": false
      },
      {
        "body_id": 2,
        "asset_id": 7,
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "is_environment": true
      }
    ]
  }
}
```

### Frontend Consumption of `frame` (impact of FR-1)

The `handleFrame(data)` function in `live-app.js` passes each entry in `data.states` to
`scene.updateBody(bodyState)`. The `SceneManager` uses `body_id` to look up the Three.js mesh
and updates position/orientation. The `is_environment` field and the new environment entries are
additional data — `SceneManager` ignores unknown fields and already has mesh entries for
environment objects (created during `handleMetadata`). The frontend therefore handles the
extended `frame` data without modification.

### Messages: `complete` and `error`

Unchanged by this ticket.

---

## Contract 4: Dead Code Removal (FR-5)

The `_run_simulation` function in `live.py` (lines 88–134 on the 0072b branch) is removed. It is
never called — the simulation loop is inlined in `live_simulation` (Phase 3). This removal has no
integration impact: no other Python file, no test, and no JavaScript code references this function.

After removal, the top-level definitions in `live.py` are:

| Symbol | Type | Role |
|--------|------|------|
| `_build_asset_geometries` | function | Helper: resolves asset names to geometry |
| `live_simulation` | async function | WebSocket endpoint |
| `_listen_for_stop` | nested function inside `live_simulation` | Background stop-signal listener |
| `list_live_assets` | async function | REST GET /live/assets |

---

## Integration Contract Summary Table

| Boundary | Direction | Change | FR | Backward Compatible |
|----------|-----------|--------|----|---------------------|
| C++ → Python (pybind11) | `getFrameState()` return | Add `is_environment` to all entries; add environment body entries | FR-1 | Yes — additive |
| Python Pydantic validation | Client → configure | Reject invalid `object_type` values | FR-3 | No — intentional breaking for malformed inputs |
| Python Pydantic validation | Client → configure | Reject wrong-length `position`/`orientation` | FR-4 | No — intentional breaking for malformed inputs |
| Python WebSocket endpoint | Internal | Remove dead `_run_simulation` | FR-5 | Yes — no callers |
| JavaScript → Python (WebSocket) | configure payload | Omit `mass`/`restitution`/`friction` for environment objects | FR-6 | Yes — Pydantic defaults fill omitted fields |
| contracts.yaml | Documentation | Formalize `EngineFrameState` schema | FR-2 | N/A — documentation only |

---

## API Contract Document (FR-2)

The `x-pybind11-schemas` section in `docs/api-contracts/contracts.yaml` already contains the
`EngineBodyState` and `EngineFrameState` schemas as committed on the 0072e branch during the
Design phase. The schema correctly reflects the post-FR-1 structure (including `is_environment`
on both inertial and environment entries).

No additional changes to `contracts.yaml` are required beyond what was committed in the Design
phase. The `SpawnObjectConfig` schema in `components/schemas` already reflects the post-FR-3/FR-4
constraints (`minItems: 3`, `maxItems: 3` on position/orientation; `enum: [inertial, environment]`
on `object_type`).

---

## Error Handling at Layer Boundaries

### pybind11 Boundary

Errors in `getFrameState()` (e.g., C++ exception during asset iteration) propagate as
`py::error_already_set` to the Python caller. The `live_simulation` endpoint catches all
`Exception` types and sends an `error` WebSocket message before closing.

### WebSocket configure phase

Pydantic `ValidationError` from `SpawnObjectConfig(**obj)` is not currently caught explicitly
in `live.py` — it propagates to the outer `except Exception as exc` handler, which sends:

```json
{"type": "error", "message": "<pydantic validation error text>"}
```

This behavior is acceptable for the current ticket. A future ticket may choose to catch
`ValidationError` specifically to format a more structured error response.

### WebSocket frame phase

`WebSocketDisconnect` from client disconnect during streaming is caught and handled silently
(client has gone, no response possible). Other exceptions send an `error` message then close.

---

## Sequence Diagram

See: `./0072e_live_simulation_cleanup-sequence.puml`

---

## Test Integration Points

The integration contracts above define the observable boundaries for tests:

### pybind11 boundary tests (`test_engine_bindings.py`)
- Spawn an environment object, call `get_frame_state()`, assert:
  - Environment entry present in `states` with `is_environment: True`
  - Inertial entries have `is_environment: False`
  - Environment velocity/angular_velocity are all zero
  - Environment `body_id` matches `getInstanceId()` (verified by comparing against spawn order)

### WebSocket endpoint tests (`test_live_api.py`)
- Send `object_type: "kinematic"` in configure — assert connection closed with error message
- Send `position: [0, 1]` in configure — assert connection closed with error message
- Send `position: [0, 1, 2, 3]` in configure — assert connection closed with error message
- Send `orientation: [0, 1]` and `orientation: [0, 1, 2, 3]` — same pattern
- Verify `_run_simulation` symbol no longer exists in `live.py` module

### Frontend verification
No automated tests for FR-6. Verified by code inspection: the conditional payload builder
correctly branches on `entry.object_type === 'inertial'`.

---

## Open Questions

None blocking. All integration design decisions are resolved.

### Resolved

1. **FR-6: environment object mass/restitution/friction defaults** — Pydantic defaults
   (`mass=10.0`, `restitution=0.8`, `friction=0.5`) apply when fields are absent. The `metadata`
   message will carry these default values, which is acceptable since the frontend does not
   display them for environment objects.

2. **FR-1: `is_environment` ordering in states list** — Environment entries are appended after all
   inertial entries, consistent with the spawn order within each group. The frontend SceneManager
   already has mesh entries for all body IDs from the `metadata` message; it does not depend on
   ordering within the `states` list.

3. **spawn_inertial_object call site** — The design review (N2) noted that `live.py` passes
   `cfg.position` and `cfg.orientation` as list objects, not as unpacked arguments. The actual
   call at implementation time should use `*cfg.position` and `*cfg.orientation`. FR-4's length
   constraint (exactly 3 elements) ensures that if a list is passed, its length is guaranteed
   correct at the Python/C++ boundary. Verification is required during implementation.
