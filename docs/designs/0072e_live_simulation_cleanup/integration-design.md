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

---

## Integration Design Review

**Reviewer**: Integration Design Reviewer
**Date**: 2026-02-21
**Status**: APPROVED WITH MANDATORY ADDITION
**Iteration**: 0 of 1 (no autonomous revision; mandatory addition escalated to Python Design phase)

---

### Criteria Assessment

#### Contract Completeness

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| All layer boundaries documented | ✓ | Four contracts cover all six FRs across pybind11, Pydantic, WebSocket, and dead code boundaries |
| Wire format before/after documented | ✓ | Both `configure` and `frame` messages show pre/post 0072e JSON |
| Error propagation paths documented | ✓ | pybind11, configure, and frame-phase error paths all specified |
| Backward compatibility assessment | ✓ | Each contract row in summary table includes backward-compat flag with justification |
| Sequence diagram present | ✓ | `0072e_live_simulation_cleanup-sequence.puml` correctly depicts lifecycle |

#### Contract Accuracy

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| pybind11 return schema accurate | ✓ | Matches `engine_bindings.cpp` `getFrameState()` implementation — both inertial fields and the proposed environment extension are correct |
| FR-3 validation contract accurate | ✓ | `SpawnObjectConfig(**obj)` raises `ValidationError` on bad `object_type` — propagates to outer `except Exception` handler as documented |
| FR-4 validation contract accurate | ✓ | `Annotated[list[float], Field(min_length=3, max_length=3)]` on `position`/`orientation` will enforce length at parse time |
| FR-5 dead code removal accurate | ✓ | `_run_simulation` is confirmed present in `live.py` lines 88–134 and never called |
| FR-6 payload omission accurate | ✓ | Environment objects currently send `mass`/`restitution`/`friction`; Pydantic defaults fill absent fields |
| **`body_id` consistency guarantee accurate** | **✗** | **CRITICAL GAP — see Finding I1 below** |

#### Sequence Diagram Accuracy

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Configure phase flow correct | ✗ | Sequence diagram shows `spawn_inertial_object` with `*cfg.position, *cfg.orientation` but actual `live.py` line 205–211 passes `cfg.position` and `cfg.orientation` as list objects (N2 bug confirmed in source) |
| Frame phase flow correct | ✓ | `engine.get_frame_state()` → augment with `frame_id` → `send_json` pattern is correct |
| Error paths complete | ✓ | `ValidationError` → `error` message → close is correctly depicted |
| Stop-signal handling shown | ✓ | `_listen_for_stop` background task and `stop_requested.asyncio.Event` correctly shown |

---

### Critical Finding

#### I1: `body_id` Consistency Guarantee Is Unsupported by Existing Code

**Severity**: Critical (correctness defect — frame state `body_id` values will not match metadata `body_id` values unless the C++ instance ID assignment happens to coincide with Python's `enumerate`)

**Evidence from source code**:

In `replay/replay/routes/live.py` line 202:
```python
for body_id, cfg in enumerate(spawn_configs, start=1):
    ...
    engine.spawn_inertial_object(cfg.asset_name, cfg.position, ...)  # return value DISCARDED
    ...
    body_metadata.append(LiveBodyMetadata(body_id=body_id, ...))  # body_id from enumerate
```

The `spawn_inertial_object` and `spawn_environment_object` pybind11 methods return dicts:
```python
{"instance_id": int, "asset_id": int}
```

These return values are never captured. The `LiveBodyMetadata.body_id` is set from Python's `enumerate`, while `get_frame_state()["states"][i]["body_id"]` is set from C++'s `asset.getInstanceId()` inside `getFrameState()`.

**The integration design's Contract 1 states**:

> The configure phase assigns sequential `body_id` values starting at 1 to each spawned object (both inertial and environment), stored in `LiveBodyMetadata`. The C++ `getInstanceId()` method returns the same ID.

This assertion may coincidentally hold today if the C++ Engine's internal instance ID counter also starts at 1 and increments in spawn order. However:

1. The guarantee is **implementation-dependent and undocumented** — it relies on an assumption about C++ internal state that is not part of any explicit API contract.
2. If a new Engine instance is created (which already happens per WebSocket connection), the counter may reset to 1 — but if any future change initializes C++ state differently (e.g., pre-populates the world model), the IDs will diverge.
3. The sequence diagram shows `body_id = 1` and `body_id = 2` as comments annotated on the spawn calls, suggesting the design author assumed the IDs would be 1 and 2. But the Python code does not capture or verify this.

**Required fix** (to be implemented in the Python Design phase):

Capture the return value of each spawn call and use the returned `instance_id` as the `body_id` in `LiveBodyMetadata`. This makes the consistency guarantee explicit and self-enforcing:

```python
for cfg in spawn_configs:
    asset_id = name_to_id[cfg.asset_name]
    if cfg.object_type == "inertial":
        result = engine.spawn_inertial_object(
            cfg.asset_name,
            *cfg.position,      # FR-4 + N2: unpack 3-element list
            *cfg.orientation,   # FR-4 + N2: unpack 3-element list
            cfg.mass,
            cfg.restitution,
            cfg.friction,
        )
    else:
        result = engine.spawn_environment_object(
            cfg.asset_name,
            *cfg.position,
            *cfg.orientation,
        )
    body_id = result["instance_id"]  # Use C++-assigned ID, not Python enumerate
    body_metadata.append(
        LiveBodyMetadata(
            body_id=body_id,
            asset_id=result["asset_id"],  # Also use C++-returned asset_id
            ...
        )
    )
```

Note this also fixes:
- The N2 issue (list unpacking with `*cfg.position`, `*cfg.orientation`)
- The `asset_id` source — currently using `name_to_id[cfg.asset_name]` (from `list_assets()`) which may differ from `asset.getAssetId()` in C++ if the registry ID assignment differs

**Disposition**: This finding is a **mandatory addition** to the Python Design phase scope. The integration design document's body_id consistency guarantee section must be updated to reflect the correct implementation approach. The Python Design agent must capture `instance_id` from spawn return values.

---

#### I2: N2 Bug Confirmed in Source — `*cfg.position` Unpacking Missing

**Severity**: High (runtime error — currently broken at the Python/C++ pybind11 boundary for any non-mocked test)

**Evidence**: `live.py` lines 205–211 pass `cfg.position` (a `list[float]`) as the `x` positional argument to `spawn_inertial_object`, which expects `double x, double y, double z` as separate scalar arguments. This will raise a `TypeError` at runtime against a real Engine.

The sequence diagram correctly shows `*cfg.position` (list-unpacking) on the spawn calls, but the actual code does not unpack. The fix is the same as I1's required code change above — both issues are resolved simultaneously by capturing the return value and unpacking the position/orientation lists.

**Disposition**: Absorbed into the I1 fix. The Python Design phase must specify `*cfg.position` and `*cfg.orientation` at all spawn call sites.

---

### Items Passing Review

- All six FR contracts are logically sound and complete
- Wire format schemas for `configure`, `metadata`, and `frame` are accurate
- Error propagation at all three layer boundaries is documented
- FR-5 dead code removal has no integration impact (confirmed no callers)
- FR-6 Pydantic defaults correctly fill absent `mass`/`restitution`/`friction` fields
- FR-2 contracts.yaml `x-pybind11-schemas` section is already complete on branch
- `is_environment` discriminated union design is correct for both C++ and Python consumers
- Sequence diagram structure and message ordering are accurate (modulo N2 bug on spawn calls)

---

### Mandatory Addition to Python Design Phase Scope

The Python Design phase MUST include the following as **FR-7** (new requirement discovered during integration design review):

> **FR-7**: `live.py` configure phase shall capture the `instance_id` returned by `engine.spawn_inertial_object()` and `engine.spawn_environment_object()` and use it as the `body_id` in `LiveBodyMetadata`, replacing the current `enumerate`-based assignment. This guarantees that `body_id` values in the `metadata` message are identical to `body_id` values in `get_frame_state()` frame entries, closing the correctness gap identified in Integration Design Review Finding I1.

The fix simultaneously resolves:
- FR-7 (body_id consistency — new)
- N2 from design review (list unpacking — `*cfg.position`, `*cfg.orientation`)

The Python Design agent should also update `contracts.yaml` to document the `instance_id` return type from spawn calls as part of the pybind11 contract.

---

### Summary

The integration design is thorough and correctly documents five of the six FR contracts. The single critical gap is the `body_id` consistency guarantee in Contract 1, which is stated as fact but is not enforced by the existing `live.py` code. The fix — capturing `instance_id` from spawn return values — is straightforward, self-documenting, and simultaneously resolves the N2 list-unpacking bug. This is scoped as a mandatory addition (FR-7) to the Python Design phase rather than a revision to the integration design, since the integration design correctly identifies the desired contract; the gap is in how `live.py` implements it. The integration design is approved to proceed to the Python Design phase with FR-7 added to scope.
