# Retroactive Design Review: 0072b WebSocket Simulation Endpoint

**Date**: 2026-02-20
**Reviewer**: Workflow Orchestrator (retroactive)
**Status**: Merged / Complete — post-hoc review only
**Ticket**: [0072b_websocket_simulation_endpoint](../../tickets/0072b_websocket_simulation_endpoint.md)

---

## Purpose of This Document

Ticket 0072b was implemented before the current workflow template existed. It skipped the Design,
Python Design, and Integration Design phases that the current template requires for a multi-language
Python feature. This document reconstructs what those phases should have produced, evaluates the
actual implementation against those expectations, and records any gaps or follow-up items.

**This review does not change the ticket status.** The feature is shipped and working. The goal is
traceability — ensuring the design rationale is captured for future maintainers and for the
downstream tickets that depend on this wire protocol (0072c, and anything that extends the live
simulation API).

---

## Section 1: What the Design Phase Should Have Produced

### 1.1 Architecture Design Document (design.md)

A proper Design phase for this ticket would have produced
`docs/designs/0072b-websocket-simulation-endpoint/design.md` containing:

**Component Map**

| Component | File | Responsibility |
|-----------|------|----------------|
| `live_router` | `replay/replay/routes/live.py` | WebSocket endpoint + REST asset list |
| `SpawnObjectConfig` | `replay/replay/models.py` | Validates and carries client spawn intent |
| `LiveBodyMetadata` | `replay/replay/models.py` | Per-body response in metadata message |
| `AssetInfo` | `replay/replay/models.py` | REST asset list item |
| `AssetGeometry` | `replay/replay/models.py` | Geometry payload in metadata message (pre-existing, reused) |
| `app.py` | `replay/replay/app.py` | Registers `live_router`; adds `/live` HTML route |

**Lifecycle State Machine**

The WebSocket endpoint has three phases that should have been explicitly modeled:

```
CONNECTED
    |
    v (receive 'configure')
CONFIGURED  <-- error: wrong msg type, empty objects, invalid asset
    |
    v (receive 'start' or 'stop')
RUNNING / STOPPED
    |
    v (duration elapsed or 'stop' received mid-run, or disconnect)
COMPLETE
    |
    v
CLOSED
```

This state machine was implicit in the implementation. It should have been documented explicitly
before coding began, because it governs all error-branch decisions.

**Key Design Decisions That Should Have Been Recorded (DD Blocks)**

The following decisions were made implicitly during implementation. They are reconstructed here.

---

### DD-0072b-001: One Engine instance per WebSocket connection

- **Affects**: `live.py::live_simulation`
- **Rationale**: The C++ Engine holds physics state (spawned bodies, contact manifolds, solver
  warm-start data). Sharing an Engine across connections would require locking and would cause
  cross-connection state corruption. Isolation is the safe default.
- **Alternatives Considered**:
  - Connection pool: rejected — Engine lifecycle is inseparable from the simulation session; pooled
    Engines would carry stale physics state between sessions.
  - Singleton Engine: rejected — serializes all connections, preventing concurrent simulations.
- **Trade-offs**: Memory cost is O(connections * Engine). For a development server with a handful of
  concurrent users this is acceptable. Under high concurrency (hundreds of users), resource
  exhaustion becomes a concern.
- **Status**: active

---

### DD-0072b-002: asyncio.to_thread for all Engine calls

- **Affects**: `live.py::live_simulation`, `live.py::_build_asset_geometries`
- **Rationale**: The C++ Engine (pybind11) holds the GIL during execution and performs CPU-bound
  physics computation. Calling it directly from the async handler would block the FastAPI event loop,
  starving all other WebSocket connections and HTTP requests. `asyncio.to_thread` releases the event
  loop between C++ calls.
- **Alternatives Considered**:
  - `ProcessPoolExecutor`: would avoid GIL contention entirely but adds IPC serialization overhead
    for every frame (frame dicts are non-trivial in size) and complicates Engine lifetime management.
  - Direct call (blocking): unacceptable — blocks the event loop for the duration of each physics
    step.
- **Trade-offs**: `asyncio.to_thread` uses the default `ThreadPoolExecutor`. Under high concurrency,
  thread pool exhaustion is possible. The current design is single-connection-at-a-time per thread;
  scaling to many connections requires tuning the thread pool size.
- **Status**: active

---

### DD-0072b-003: Real-time pacing via asyncio.sleep

- **Affects**: `live.py::live_simulation` (simulation loop)
- **Rationale**: The browser frontend renders at display refresh rate (~60 Hz). Streaming frames
  faster than the timestep rate saturates the WebSocket send buffer and wastes client CPU on
  discarded frames. Pacing to `timestep_ms / 1000.0` seconds per frame aligns server output with
  client consumption.
- **Alternatives Considered**:
  - No pacing (send as fast as possible): produces bursts that exceed client buffer; forces client
    to implement its own rate limiting.
  - Server-side frame dropping: drops physics accuracy from the client's perspective.
- **Trade-offs**: If the physics step takes longer than `timestep_ms`, `sleep_for` becomes negative
  and `asyncio.sleep` is skipped. The simulation runs as fast as physics allows but falls behind
  real time. This is acceptable for development; production use would need a backpressure signal.
- **Status**: active

---

### DD-0072b-004: Stop-signal via asyncio.Event and background listener task

- **Affects**: `live.py::live_simulation` (Phase 3)
- **Rationale**: During the simulation loop the handler is `await`-ing Engine calls and
  `asyncio.sleep`. A concurrent `stop` message from the client arrives on the WebSocket receive
  channel while the handler is not actively receiving. A background `asyncio.Task` monitors the
  receive channel and sets a shared `asyncio.Event`, which the simulation loop checks between frames.
- **Alternatives Considered**:
  - Polling `receive_nowait()`: FastAPI WebSocket does not expose a non-blocking receive; this would
    require restructuring around lower-level Starlette internals.
  - `asyncio.wait` with two awaitables: complex and fragile when one branch is a long CPU-bound
    chain.
- **Trade-offs**: The listener task catches any receive-side exception (including disconnect) and
  sets the event, so disconnects are handled correctly. The inner `finally` cancels the listener to
  prevent task leaks.
- **Status**: active

---

### DD-0072b-005: Asset geometry embedded in metadata message

- **Affects**: `live.py::live_simulation`, `live.py::_build_asset_geometries`
- **Rationale**: The frontend needs geometry to render bodies. Bundling it in the `metadata`
  response (sent immediately after `configure`) eliminates a second round-trip before the first
  frame can be rendered. This matches the replay endpoint pattern (`GET /simulations/{id}/assets`)
  but avoids a separate HTTP request mid-WebSocket session.
- **Alternatives Considered**:
  - Separate REST call: requires the frontend to synchronize two asynchronous operations (WS connect
    + HTTP fetch) before starting playback.
  - Send geometry on demand (per-frame): redundant; geometry does not change within a session.
- **Trade-offs**: The `metadata` message grows proportionally with the number of unique assets
  spawned. For typical scenes (< 10 asset types) this is negligible. For pathological cases with
  many high-poly assets, the message could be large. No chunking mechanism exists today.
- **Status**: active

---

### DD-0072b-006: Temporary Engine for GET /api/v1/live/assets

- **Affects**: `live.py::list_live_assets`
- **Rationale**: The asset list is a read-only query on the assets database. Constructing a
  temporary Engine is the simplest path since `Engine.list_assets()` is already the canonical API
  for this data. No persistent Engine state is needed.
- **Alternatives Considered**:
  - `GeometryService` (already exists): reads geometry but not asset name/id lists in the format
    required. Would require adding a new query method.
  - Direct SQLite query: bypasses pybind11 layer; fragile to schema changes in the assets database.
- **Trade-offs**: A new Engine is constructed and destroyed on every request. Engine construction
  opens the assets SQLite database; for high-frequency polling this is inefficient. An application-
  level cache keyed on the database path modification time would be a future improvement.
- **Status**: active

---

### 1.2 PlantUML Sequence Diagram

A Design phase would have produced a sequence diagram. It is reconstructed here in prose form since
no `.puml` file exists:

```
Client -> Server: WebSocket connect /api/v1/live
Server -> Client: (accept)
Client -> Server: {"type": "configure", "objects": [...]}
Server -> Engine: construct (assets_db_path)
Server -> Engine: list_assets()
Server -> Engine: spawn_inertial_object(...) x N
Server -> Engine: spawn_environment_object(...) x M
Server -> Engine: get_collision_vertices(asset_id) x unique_assets
Server -> Client: {"type": "metadata", "bodies": [...], "assets": [...]}
Client -> Server: {"type": "start", "timestep_ms": 16, "duration_s": 30.0}
loop each frame until duration or stop
    Server -> Engine: update(sim_time_ms)
    Server -> Engine: get_frame_state()
    Server -> Client: {"type": "frame", "data": {...}}
    Server -> Server: asyncio.sleep(remaining_interval)
end
Server -> Client: {"type": "complete", "total_frames": N, "elapsed_s": T}
Server -> Engine: (destructor — engine = None)
```

This diagram was not produced before implementation. The wire protocol in the ticket requirements
(R2) served as its informal substitute.

---

## Section 2: What the Python Design Phase Should Have Produced

### 2.1 Python Module Architecture (python/design.md)

A Python Design phase would have produced
`docs/designs/0072b-websocket-simulation-endpoint/python/design.md` covering:

**Module Boundaries**

| Module | Role | Pattern |
|--------|------|---------|
| `routes/live.py` | Request handling, protocol dispatch, resource lifecycle | FastAPI router — thin handler |
| `models.py` (additions) | Data shapes for wire protocol | Pydantic BaseModel |
| `app.py` (modification) | Router registration | Application composition |

**Patterns Used vs. Established Patterns**

The existing codebase pattern (from `routes/frames.py`, `routes/assets.py`) is:
- Router function calls a Service layer
- Service layer holds query logic
- Models are pure data classes

The live endpoint **deviates** from this pattern: all logic (Engine construction, spawn loop,
simulation loop, geometry building) lives directly in `live.py`. There is no `LiveSimulationService`
class. This was a deliberate simplification (one-off stateful WebSocket vs. stateless REST query),
but it was never documented as an intentional deviation.

**Async Patterns**

The Python design phase would have formally specified:
- All Engine calls must use `asyncio.to_thread` (documented as a constraint, not just an
  implementation choice)
- The stop-signal pattern (background Task + asyncio.Event) is the canonical pattern for
  concurrent receive during an async loop
- `try/finally` with `engine = None` is the correct cleanup idiom for pybind11 objects

**Validation Strategy**

Pydantic validates the `SpawnObjectConfig` shape. Asset-name validation (checking against the
database) is done imperatively in the handler. A design phase would have noted:
- Client-supplied `object_type` is a raw `str`, not an enum. Invalid values (e.g., `"static"`)
  silently fall through the `else` branch and are treated as environment objects.
- `position` and `orientation` are `list[float]` with no length constraint. A 2-element position
  list would pass validation and likely cause a C++ exception downstream.

These are not bugs that were caught by any existing test.

---

## Section 3: Gap Analysis — Design vs. Implementation

### 3.1 What Was Done Well

| Area | Assessment |
|------|------------|
| Wire protocol design | Clean, minimal, versioned under `/api/v1/`. State machine is correct. |
| Resource lifecycle | `engine = None` in `finally` is correct Python idiom for pybind11 cleanup. |
| Stop-signal pattern | Background Task + asyncio.Event is the right tool for concurrent receive. |
| Error handling breadth | Covers: wrong message type, empty config, invalid asset, missing msd_reader, generic exceptions. |
| Test coverage | 16 tests cover all acceptance criteria. Mock engine pattern is sound. |
| Thread safety | `asyncio.to_thread` calls are awaited sequentially; no concurrent Engine calls exist. |
| Code documentation | Module docstring reproduces the full wire protocol. Function docstrings have Args/Returns. |

### 3.2 Gaps and Issues Found

#### GAP-001: Dead function `_run_simulation` (Minor — cosmetic)

**File**: `replay/replay/routes/live.py`, lines 88–134

The `_run_simulation` async function is defined but never called. The simulation loop is inlined in
the `live_simulation` endpoint handler (lines 294–321) with stop-signal support that the standalone
helper does not have. The dead function is misleading: a future developer might try to use it or
wonder why it exists.

**Recommendation**: Remove `_run_simulation` entirely. If a reusable helper is desired later, it
should be designed with the stop-signal parameter from the start.

**Priority**: Low — no runtime impact.

---

#### GAP-002: `object_type` is an unvalidated string — silent fall-through for unknown values (Moderate)

**File**: `replay/replay/models.py`, line 196; `replay/replay/routes/live.py`, lines 204–218

`SpawnObjectConfig.object_type` is typed as `str`. The dispatch in `live_simulation` is:

```python
if cfg.object_type == "inertial":
    engine.spawn_inertial_object(...)
else:
    engine.spawn_environment_object(...)
```

A client sending `"object_type": "kinematic"` or any other string will silently spawn an environment
object. This is unlikely to cause a crash but will produce incorrect simulation results and a
confusing debug experience.

**Recommendation**: Use a `Literal["inertial", "environment"]` type annotation on `object_type`,
or use a `pydantic.field_validator` that raises `ValueError` for unknown values. FastAPI will then
return a 422 Unprocessable Entity with a clear error message before the WebSocket is even accepted.

Note: The WebSocket is already accepted before message parsing, so Pydantic validation errors
on `SpawnObjectConfig(**obj)` (line 182) will raise an unhandled `ValidationError` that falls
through to the generic `except Exception` handler and sends a JSON error message. However, the
error message from a raw `ValidationError` may be verbose and not user-friendly.

**Priority**: Moderate — incorrect behavior for clients that send typos in `object_type`.

---

#### GAP-003: `position` and `orientation` lists have no length validation (Moderate)

**File**: `replay/replay/models.py`, lines 193–194

```python
position: list[float]      # [x, y, z] in metres
orientation: list[float]   # [pitch, roll, yaw] in radians
```

There is no enforcement that these lists are exactly length 3. A client sending `[0, 1]` for
position or `[0, 1, 2, 3]` for orientation will pass Pydantic validation and reach the C++ Engine
call, where the behavior depends on what `msd_reader` does with wrong-length lists (likely raises
an exception that is caught by the generic handler, but the error message will not be clear).

**Recommendation**: Use `Annotated[list[float], Field(min_length=3, max_length=3)]` for both
fields, or use `tuple[float, float, float]` which Pydantic validates strictly.

**Priority**: Moderate — defensive input validation is important for a network endpoint.

---

#### GAP-004: Engine construction in `list_live_assets` is synchronous in an async handler (Low)

**File**: `replay/replay/routes/live.py`, lines 367–368

```python
engine = msd_reader.Engine(str(config.assets_db_path))
assets: list[tuple[int, str]] = engine.list_assets()
```

Both `Engine()` construction and `list_assets()` are synchronous C++ pybind11 calls executed
directly in an `async def` handler. This blocks the event loop for the duration of database open +
query, which is typically fast (< 10 ms) but is inconsistent with the WebSocket endpoint's use of
`asyncio.to_thread` for all Engine calls.

**Recommendation**: Wrap the construction and query in `asyncio.to_thread`:

```python
def _fetch_assets() -> list[tuple[int, str]]:
    eng = msd_reader.Engine(str(config.assets_db_path))
    return eng.list_assets()

assets = await asyncio.to_thread(_fetch_assets)
```

**Priority**: Low — practically fast enough in current use; becomes a correctness concern under load.

---

#### GAP-005: No validation of `duration_s` upper bound (Low)

**File**: `replay/replay/routes/live.py`, lines 263–270

`timestep_ms` is validated to be positive. `duration_s` is not validated at all. A client sending
`duration_s: 86400` (24 hours) would start a simulation that runs indefinitely until the connection
drops. There is no server-side maximum duration guard.

**Recommendation**: Add a maximum duration cap (e.g., 3600 seconds) with a configurable constant
and return an error if exceeded. Alternatively, document this as an intentional "unlimited
duration" feature for the live simulation use case.

**Priority**: Low — relevant only if the endpoint is exposed beyond a local development server.

---

#### GAP-006: No integration design document for the C++/Python boundary (Informational)

This is a multi-language feature (C++ Engine via pybind11, Python FastAPI server). The current
workflow template calls for an Integration Design phase that produces:
- `docs/designs/{feature-name}/integration-design.md`
- A sequence diagram showing C++/Python interactions
- Updates to `docs/api-contracts/contracts.yaml`

None of these exist for 0072b. The pybind11 API contract (method signatures, return types, GIL
behavior) is implicit. This creates risk: if `msd_reader.Engine` API changes in a future ticket,
there is no canonical reference for what 0072b depends on.

**Recommendation**: As a follow-up to ticket 0072a (engine pybind bindings), document the
expected Engine interface in `docs/api-contracts/contracts.yaml` or a dedicated
`docs/designs/0072a_engine_pybind_bindings/api-contract.md`. Tag 0072b as a consumer of that
contract.

**Priority**: Informational — no immediate code change required.

---

#### GAP-007: SpawnObjectConfig has physics defaults for environment objects (Informational)

**File**: `replay/replay/models.py`, lines 197–199

```python
mass: float = 10.0
restitution: float = 0.8
friction: float = 0.5
```

Environment objects are spawned via `spawn_environment_object` which takes no physics parameters.
Yet the model always includes `mass`, `restitution`, and `friction` fields with defaults. This
means the metadata response (which echoes `cfg.mass` etc. in `LiveBodyMetadata`) reports physics
properties for environment bodies that are never used by the Engine.

**Recommendation**: Either:
1. Mark physics fields as `Optional[float] = None` and document that they are ignored for
   environment objects, or
2. Use a discriminated union: `InertialObjectConfig` and `EnvironmentObjectConfig`, with Pydantic's
   `Annotated` + `Literal` discriminator.

The discriminated union approach is more explicit but more complex. For the current use case,
documenting the ignored fields is sufficient.

**Priority**: Informational — no runtime impact, but it can confuse API consumers.

---

### 3.3 Missing Test Cases

The following scenarios are not covered by the current 16 tests:

| Scenario | Risk |
|----------|------|
| `object_type` with an invalid value (e.g., `"kinematic"`) | Silent wrong behavior (spawns as environment) |
| `position` list with wrong length (e.g., `[0, 1]`) | C++ exception, unclear error to client |
| `orientation` list with wrong length | Same |
| `duration_s` of zero | Off-by-one: `while sim_time_ms <= 0` executes once (one frame sent), which may or may not be the intended behavior |
| `timestep_ms` larger than `duration_s * 1000` | One frame sent; probably correct but untested |
| Two simultaneous WebSocket connections | Isolation guarantee never tested at integration level |
| Very large `duration_s` (resource exhaustion) | Not tested; no server-side cap exists |

---

## Section 4: Coding Standards Compliance

The codebase CLAUDE.md applies to C++ code. Python-specific patterns are not formally specified
there. The following evaluation uses the patterns established by existing replay server code.

| Standard | Compliance | Notes |
|----------|------------|-------|
| Type annotations on all parameters and locals | Compliant | All function signatures annotated |
| Docstrings on public callables | Compliant | All exported functions have Google-style docstrings |
| Pydantic models for all wire-protocol types | Compliant | SpawnObjectConfig, LiveBodyMetadata, AssetInfo |
| No shared mutable state across connections | Compliant | `engine` is local to each handler invocation |
| `try/finally` for external resource cleanup | Compliant | Engine released in outer `finally` |
| Consistent import structure | Compliant | Stdlib, then third-party, then local |
| Ticket reference in module header | Compliant | Present in all modified files |

---

## Section 5: Summary and Follow-up Items

### 5.1 What Was Designed Implicitly but Well

The implementation correctly realized the design intent. The wire protocol, lifecycle state machine,
async offloading strategy, and error handling all function as intended. The six design decisions
(DD-0072b-001 through DD-0072b-006) were made correctly even though they were never formally
documented.

### 5.2 Follow-up Items

| ID | Priority | Description |
|----|----------|-------------|
| FU-001 | Low | Remove dead `_run_simulation` function from `live.py` |
| FU-002 | Moderate | Add `Literal["inertial", "environment"]` constraint to `SpawnObjectConfig.object_type` |
| FU-003 | Moderate | Add `min_length=3, max_length=3` constraint to `position` and `orientation` fields |
| FU-004 | Low | Wrap `list_live_assets` Engine calls in `asyncio.to_thread` for consistency |
| FU-005 | Low | Add a configurable maximum `duration_s` cap with a clear error response |
| FU-006 | Informational | Document pybind11 Engine API contract in `docs/api-contracts/` as part of 0072a |
| FU-007 | Informational | Clarify physics fields on `SpawnObjectConfig` for environment objects |
| FU-008 | Low | Add tests for invalid `object_type`, wrong-length position/orientation, zero duration |

FU-001 through FU-003 are candidates for a small follow-up ticket or a targeted PR. FU-004 and
FU-005 are improvements that matter only if the endpoint is used in a multi-user context.

### 5.3 Risk Assessment for Downstream Tickets

**0072c (live simulation frontend)** consumes this wire protocol. GAP-002 and GAP-003 are the
most relevant risks: if the frontend sends malformed `object_type` or wrong-length vectors, the
error messages it receives will be unclear. Fixing FU-002 and FU-003 before or alongside 0072c
would improve the frontend developer experience.

**Future extensions** (additional Engine API methods, additional wire protocol messages) should
produce a formal Python Design document using the current workflow template, now that the template
exists.
