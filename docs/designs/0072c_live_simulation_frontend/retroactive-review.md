# Retroactive Design Review: 0072c Live Simulation Frontend

**Date**: 2026-02-20
**Reviewer**: Workflow Orchestrator (retroactive analysis)
**Ticket**: [0072c_live_simulation_frontend](../../../tickets/0072c_live_simulation_frontend.md)
**Status**: Merged / Complete — this review is informational only; no status changes

---

## Purpose

Ticket 0072c was implemented before the full Design and Frontend Design phases were part of
the workflow template. The implementation went directly from Draft to Ready for Implementation,
skipping:

- **Design Phase** — which would have produced `design.md`, a PlantUML component diagram,
  and recorded explicit design decisions (DD blocks)
- **Frontend Design Phase** — which would have produced `frontend/design.md` covering
  component structure, state machine, API contract verification, and UI interaction flows

This document reconstructs what those phases should have produced, audits the actual
implementation against both the ticket requirements and the 0072b wire protocol, evaluates
SceneManager integration correctness, and records any gaps or follow-up items discovered.

---

## Section 1: What the Design Phase Should Have Produced

### 1.1 Component Architecture

The Design phase should have produced a `design.md` capturing the three-file architecture
and the boundaries between them.

**Reconstructed architecture (from actual implementation):**

```
live.html              — Static shell: DOM structure only, no inline JS
  └── imports live.css (layout, theming, all visual states)
  └── imports live-app.js (type="module")

live-app.js            — Single-file application controller
  ├── imports SceneManager from scene.js (ES module)
  ├── REST client: loadAvailableAssets() → GET /api/v1/live/assets
  ├── Spawn list manager: spawnList[], renderSpawnList(), removeSpawnEntry()
  ├── WebSocket client: onStartSimulation(), handleMetadata(), handleFrame(),
  │                     handleComplete(), handleServerError(), onStopSimulation()
  └── UI state machine: setAppState(), disableControls(), enableControls()

scene.js               — Imported but not modified (owned by ticket 0056e)
  ├── loadBodies(metadata, geometries) — consumed after 'metadata' message
  └── updateFrame(frameData) — consumed on each 'frame' message
```

**What was missing from a formal design pass:**

The ticket requirements already specified this breakdown in R1-R7, but no PlantUML diagram
was produced. A component diagram would have made the SceneManager boundary and the
WebSocket lifecycle explicit before implementation began.

### 1.2 Design Decisions That Should Have Been Explicitly Recorded

The following decisions were made implicitly during implementation and should have been
captured as DD blocks during a design phase.

#### DD-0072C-001: Single-module architecture — no separate state or service files

- **Affects**: `live-app.js`
- **Rationale**: The feature is a contained live viewer with a simple linear
  lifecycle (connect → configure → run → complete). A procedural single-file
  approach matches the complexity and avoids the overhead of a module graph that
  would be harder to follow without a bundler.
- **Alternatives Considered**:
  - Separate `live-state.js` and `live-ws.js`: Rejected — adds import graph
    complexity with no benefit for a feature of this size.
  - Class-based `LiveApp`: Rejected — the design decision in 0056e (SceneManager)
    was to use a class because it owns Three.js resources; `live-app.js` owns no
    long-lived resources beyond the WebSocket socket reference.
- **Trade-offs**: All module state is held as top-level `let`/`const` variables,
  which makes the module non-reusable (one instance per page load). Acceptable here
  since this is a single-page application.
- **Status**: active

#### DD-0072C-002: SceneManager reused without modification

- **Affects**: `live-app.js::handleMetadata`, `live-app.js::handleFrame`
- **Rationale**: The ticket requirement (R5, Constraint) explicitly forbade modifying
  `scene.js`. The wire protocol was designed in 0072b to match the `loadBodies` /
  `updateFrame` field expectations exactly, so no adapter layer is needed.
- **Alternatives Considered**:
  - Fork `scene.js` as `live-scene.js`: Rejected — diverges from the shared renderer
    and creates a maintenance burden.
  - Thin adapter wrapper: Not needed — field names align exactly (see Section 3).
- **Trade-offs**: `live-app.js` silently inherits `SceneManager`'s dependency on
  `window.overlays` (see Section 3, Finding F-02). The overlay update path in
  `updateFrame()` will not find any overlays on the live page and will no-op, which
  is correct, but this coupling is invisible in `live-app.js`.
- **Status**: active

#### DD-0072C-003: ws/wss protocol detection from window.location.protocol

- **Affects**: `live-app.js::onStartSimulation`
- **Rationale**: Allows the same page to work behind HTTPS without a hardcoded URL.
- **Alternatives Considered**:
  - Hardcode `ws://`: Works for local dev but fails in any TLS deployment.
  - Server-side template variable: Adds server coupling to a static file.
- **Trade-offs**: None significant.
- **Status**: active

#### DD-0072C-004: Asset dropdown value is name string, not asset_id integer

- **Affects**: `live-app.js::loadAvailableAssets`, `live-app.js::onAddObject`
- **Rationale**: The wire protocol `configure` message uses `asset_name: str` (not
  `asset_id`). Storing the name as the option value avoids a lookup step.
- **Alternatives Considered**:
  - Store `asset_id` in option value, name in label: Would require a reverse lookup
    when building the configure payload.
- **Trade-offs**: If two assets ever share a name (shouldn't happen per backend
  uniqueness constraint, but is not validated client-side), only one would appear.
- **Status**: active

#### DD-0072C-005: Orientation initialized to Euler zeros, not quaternion identity

- **Affects**: `live-app.js::onAddObject`
- **Rationale**: The `configure` wire message uses `orientation: list[float]`
  interpreted as `[pitch, roll, yaw]` in radians (from 0072b `SpawnObjectConfig`).
  All objects are spawned with no rotation, which is the only sensible default for
  a general-purpose UI that does not expose rotation controls.
- **Alternatives Considered**:
  - Expose rotation inputs in the sidebar: Out of scope for R2.
- **Trade-offs**: Users cannot spawn rotated objects from the UI.
- **Status**: active

---

## Section 2: What the Frontend Design Phase Should Have Produced

### 2.1 UI State Machine

The Frontend Design phase should have formally specified the application state machine.
This was implicit in the implementation. Reconstructed:

```
States: disconnected | connecting | simulating | complete | error

Transitions:
  disconnected  --[click Start, spawnList non-empty]--> connecting
  connecting    --[metadata received + start sent]-----> simulating
  connecting    --[WebSocket close/error]--------------> error
  simulating    --[complete message received]----------> complete
  simulating    --[WebSocket close/error]--------------> error
  simulating    --[click Stop]-------------------------> disconnected
  complete      --[click Start]------------------------> connecting
  error         --[click Start]------------------------> connecting

Side effects per state:
  disconnected: Start visible+enabled (if spawnList non-empty), Stop hidden
  connecting:   Start hidden, Stop visible; all inputs disabled
  simulating:   Start hidden, Stop visible; all inputs disabled
  complete:     Start visible, Stop hidden; inputs re-enabled
  error:        Start visible, Stop hidden; inputs re-enabled; error message shown
```

**Finding**: The implementation correctly encodes all of this in `setAppState()`, but
the `connecting` → `simulating` transition is spread across two functions:
`setAppState('connecting')` is called in `onStartSimulation()`, then `setAppState('simulating')`
is called inside `handleMetadata()` after the `start` message is sent. This is correct
but a design document would have made the two-phase "connecting" period explicit.

### 2.2 WebSocket Lifecycle Sequence

A Frontend Design document should have included a sequence diagram. Reconstructed:

```
Browser                          Server
  |                                |
  |-- WebSocket open ------------->|
  |-- configure {objects} -------->|
  |<-- metadata {bodies, assets} --|
  |   [loadBodies()]               |
  |   [resetCamera()]              |
  |-- start {timestep_ms, dur} --->|
  |<-- frame {data} --------------|  (repeated)
  |   [updateFrame()]              |
  |<-- frame {data} --------------|
  |       ...                      |
  |<-- complete {total_frames} ---|
  |   [socket.close()]             |

OR (stop path):
  |-- stop {} -------------------->|  (during simulating)
  |<-- complete {total_frames} ---|
  |   [socket.close() after 500ms] |

OR (error path):
  |<-- error {message} -----------|
  |   [socket.close()]             |
  |   [showError()]                |
```

### 2.3 REST Dependency

The Frontend Design should have documented the dependency on `GET /api/v1/live/assets`:

- Called once on page load, before any user interaction
- Response shape: `[{asset_id: int, name: str}]`
- Failure mode: Show error in dropdown and status bar; "Add Object" button remains disabled

This was correctly implemented. No issues.

---

## Section 3: SceneManager API Verification

This section audits whether `live-app.js` calls `SceneManager` with the correct
arguments, given the actual API in `scene.js`.

### 3.1 `SceneManager` Constructor

**API in scene.js:**
```js
constructor(canvas)  // Takes an HTMLCanvasElement
```

**Usage in live-app.js (line 62):**
```js
sceneManager = new SceneManager(canvas);
// where canvas = document.getElementById('viewport')
```

**Verdict**: Correct. The canvas element exists in `live.html` at `id="viewport"`.

### 3.2 `loadBodies(metadata, geometries)`

**API in scene.js (lines 83-122):**
```js
loadBodies(metadata, geometries)
// metadata.bodies: Array of {body_id, asset_id, is_environment, ...}
// geometries: Array of {asset_id, positions: numeric array}
```

**Wire protocol metadata message (0072b, live.py):**
```json
{
  "type": "metadata",
  "bodies": [
    {"body_id": 1, "asset_id": 3, "asset_name": "cube", "mass": 10.0,
     "restitution": 0.8, "friction": 0.5, "is_environment": false}
  ],
  "assets": [
    {"asset_id": 3, "name": "cube", "positions": [x,y,z,...], "vertex_count": N}
  ]
}
```

**Usage in live-app.js (lines 339-342):**
```js
const metadata = { bodies: msg.bodies };
const geometries = msg.assets;
sceneManager.loadBodies(metadata, geometries);
```

**Audit:**

| SceneManager expects | Wire provides | Match? |
|---------------------|---------------|--------|
| `metadata.bodies` (array) | `msg.bodies` | Exact |
| `body.body_id` | present | Exact |
| `body.asset_id` | present | Exact |
| `body.is_environment` | present | Exact |
| `geometries[*].asset_id` | `msg.assets[*].asset_id` | Exact |
| `geometries[*].positions` | `msg.assets[*].positions` (flat float array) | Exact |

**Verdict**: Correct. The JSDoc comment in `handleMetadata()` correctly states that
no adapter is needed and the field names match exactly.

**Finding F-01 (Minor): Extra fields in wire format are silently ignored.**
`LiveBodyMetadata` includes `asset_name`, `mass`, `restitution`, `friction` beyond
what `loadBodies()` consumes. `AssetGeometry` includes `name` and `vertex_count`
beyond what `loadBodies()` consumes. These extra fields are benign — `loadBodies()`
reads only what it needs — but a design note would have made this intentional.

### 3.3 `updateFrame(frameData)`

**API in scene.js (lines 128-161):**
```js
updateFrame(frameData)
// frameData.states: Array of {body_id, position: {x,y,z}, orientation: {w,x,y,z}}
// frameData.frame_id: not used internally by SceneManager
// Also checks window.overlays for optional overlay updates
```

**Wire protocol frame message (0072b, live.py line 306):**
```python
await websocket.send_json({"type": "frame", "data": frame_dict})
# frame_dict from engine.get_frame_state() augmented with:
# frame_dict["frame_id"] = total_frames
```

**From test mock (test_live_api.py lines 76-90), `get_frame_state()` returns:**
```json
{
  "simulation_time": 0.016,
  "states": [
    {
      "body_id": 1,
      "position": {"x": 0.0, "y": 0.0, "z": 4.99},
      "velocity": {"x": 0.0, "y": 0.0, "z": -0.16},
      "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
      "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0}
    }
  ],
  "collisions": [],
  "friction_constraints": [],
  "solver": null
}
```

With `frame_id` injected by the server, the message `data` object has all keys
SceneManager reads: `states`, `states[*].body_id`, `states[*].position.{x,y,z}`,
`states[*].orientation.{w,x,y,z}`.

**Usage in live-app.js (line 365):**
```js
sceneManager.updateFrame(frameData);
// where frameData = msg.data (the 'data' field of the frame message)
```

**Verdict**: Correct. `msg.data` is the full frame dict that `updateFrame()` expects.

**Finding F-02 (Minor — design gap): `SceneManager.updateFrame()` checks `window.overlays`.**
The replay viewer (`app.js`) sets `window.overlays` before calling `loadBodies()`,
so that overlay modules receive frame updates. The live page never sets
`window.overlays`. `SceneManager.updateFrame()` guards each overlay access with
`if (window.overlays)`, and `window.overlays` will be `undefined` on the live page,
so the check evaluates to falsy and the overlay update block is skipped entirely.

This is correct runtime behavior, but it represents a hidden coupling: `scene.js` has
a side-effect dependency on a global `window.overlays` that `live-app.js` does not
need to know about but must not accidentally interfere with. The design phase should
have documented this as a known coupling that the live page safely ignores.

**No code change is required.** A comment in `live-app.js` near `sceneManager.updateFrame()`
would be sufficient acknowledgment.

### 3.4 `resetCamera()`

**API in scene.js (lines 69-76):**
```js
resetCamera()
// Resets camera to defaultCameraPosition (10, -10, 5) looking at (0, 0, 1)
```

**Usage in live-app.js:**
- Line 343: `sceneManager.resetCamera()` — called after `loadBodies()` to orient the
  camera before the first frame arrives.
- Line 96: Button event listener — `sceneManager.resetCamera()` wired to Reset Camera button.

**Verdict**: Correct.

---

## Section 4: WebSocket Protocol Conformance

This section audits whether `live-app.js` correctly implements the 0072b wire protocol.

### 4.1 Client → Server Messages

| Message | 0072b Spec | live-app.js Implementation | Match? |
|---------|------------|---------------------------|--------|
| `configure` | `{type, objects: [{asset_name, position, orientation, object_type, mass, restitution, friction}]}` | Built in `onStartSimulation()` lines 254-263; strips internal `id` and `label` fields | Exact |
| `start` | `{type, timestep_ms, duration_s}` | Sent in `handleMetadata()` lines 350-354 | Exact |
| `stop` | `{type}` | Sent in `onStopSimulation()` line 410 | Exact |

**Finding F-03 (Minor — correctness): `configure` sends `mass` for environment objects.**
In `onAddObject()` (line 175), `mass` is always collected from the input regardless of
object type. The configure payload (lines 254-262) always includes `mass`, `restitution`,
and `friction` for all objects. The backend `SpawnObjectConfig` model has `mass` with a
default, so it is accepted without error. The backend `spawn_environment_object()` simply
ignores the `mass` value.

This is functionally correct but wastes a field. A design review would have noted that
the frontend should omit `mass` from the payload when `object_type == "environment"`,
matching the spirit of R2 ("mass shown only for inertial type"). The mass UI input is
correctly hidden for environment objects, but the hidden value is still sent.

**Recommended fix**: In `onStartSimulation()`, when building the configure payload, omit
`mass` (and optionally `restitution`/`friction`) for environment objects. See follow-up
item FU-01.

### 4.2 Server → Client Messages

| Message | 0072b Spec | live-app.js Handler | Match? |
|---------|------------|---------------------|--------|
| `metadata` | `{type, bodies, assets}` | `handleMetadata()` — reads `msg.bodies`, `msg.assets` | Exact |
| `frame` | `{type, data: {frame_id, simulation_time, states, collisions, solver}}` | `handleFrame(msg.data)` — reads `data.frame_id`, `data.simulation_time`, passes full data to `updateFrame()` | Exact |
| `complete` | `{type, total_frames, elapsed_s}` | `handleComplete(msg)` — logs `msg.total_frames`, `msg.elapsed_s` | Exact |
| `error` | `{type, message}` | `handleServerError(msg.message)` | Exact |

**Finding F-04 (Minor): `simulation_time` field may be absent from live frame data.**
The `FrameData` model (models.py line 90-98) includes `simulation_time: float`.
The live backend adds `frame_id` explicitly (live.py line 304) but relies on
`engine.get_frame_state()` to provide `simulation_time`. The test mock (test_live_api.py
line 77) confirms `simulation_time` is present. The `live-app.js` handler guards this
with a `typeof` check (line 369-372):

```js
const simTime = typeof frameData.simulation_time === 'number'
    ? frameData.simulation_time.toFixed(3)
    : '0.000';
```

This defensive guard is correct practice given that the pybind11 binding was implemented
in ticket 0072a, whose exact field names are not audited in this review. The guard
prevents a crash if `simulation_time` is missing or null. **No action required**, but
this demonstrates a design-phase gap: the exact dict keys returned by
`engine.get_frame_state()` should have been verified against the 0072b wire spec during
a design or prototype phase.

### 4.3 Stop Timing Race

**Finding F-05 (Minor — design gap): Stop sends `stop`, then force-closes after 500ms.**

`onStopSimulation()` (lines 408-425):
1. Sends `{type: "stop"}` over WebSocket
2. Sets `appState = 'disconnected'` immediately
3. Calls `enableControls()` immediately
4. Sets a 500ms timeout to force `socket.close()`

The server, on receiving `stop`, finishes the current frame, sends `complete`, then the
connection closes. If `complete` arrives within 500ms (typical for local dev), `handleComplete()`
fires, calls `enableControls()` again (harmless double call), and sets `socket = null`.
Then the timeout fires with `socket` already null, and the inner `if (socket)` guard
prevents a crash.

This is correct but fragile under high-latency conditions: if `complete` arrives after
500ms (e.g., server is under load), the socket is force-closed, the `close` event fires
with `appState === 'disconnected'` (not `'simulating'` or `'connecting'`), so no error
is shown — this is the correct outcome. However, the `complete` message is then discarded
because the socket has already been nulled.

**No functional issue.** The design phase should have specified the stop handshake timing
explicitly and documented the intentional 500ms timeout as a guard against a server that
fails to send `complete`.

---

## Section 5: CSS and Layout Verification

### 5.1 Layout Correctness

The CSS uses a flex column on `body` with `height: 100vh`, a flex row `#layout` with
`height: calc(100vh - 36px)`, and a fixed-height `#status-bar` of `36px`. This correctly
fills the viewport without scrollbars.

The sidebar is `280px` fixed width with `overflow-y: auto`, allowing it to scroll if many
objects are added. The viewport `<canvas>` fills the remaining horizontal space via
`flex: 1`.

**Verdict**: Layout is correct and consistent with `index.html` approach (full-viewport
canvas), adapted for the sidebar configuration panel.

### 5.2 Hidden Utility Class

`live.css` defines `.hidden { display: none !important; }` at line 386. This is also
defined in `style.css` (used by `index.html`). There is no conflict because the two
pages load independent stylesheets. The approach is correct for `live.html`.

### 5.3 Loading Overlay

The overlay is centered absolutely within `#viewport-area`, uses `pointer-events: none`
to avoid blocking OrbitControls, and has `z-index: 10`. This is correct — the Three.js
renderer renders into the canvas, which is behind the overlay.

---

## Section 6: Gaps and Missing Design Artifacts

| Gap | Severity | Description |
|-----|----------|-------------|
| No PlantUML component diagram | Low | A diagram would document the three-file boundary and the SceneManager integration point. Not required retroactively, but useful if the frontend grows. |
| No formal state machine specification | Low | The state machine is implicit in `setAppState()`. A diagram or table would aid future contributors. |
| `window.overlays` coupling undocumented | Low | SceneManager's implicit `window.overlays` dependency should be noted in `live-app.js`. |
| No prototype phase | Low | A brief prototype verifying SceneManager field compatibility would have been appropriate given the integration dependency on 0072b. Since 0072b was developed concurrently, the risk was managed by explicit protocol design instead. |
| No frontend automated tests | Medium | The project has no browser automation infrastructure (Playwright/Cypress). The implementation review correctly notes this is consistent with prior frontend tickets. This is a **project-level gap**, not a 0072c-specific gap. |

---

## Section 7: Issues Summary

### Critical
None.

### Major
None.

### Minor

| ID | Location | Finding | Recommended Action |
|----|----------|---------|-------------------|
| F-01 | `handleMetadata()` | Extra fields in wire format (`asset_name`, `vertex_count`) silently ignored by SceneManager — correct but undocumented | Add brief comment in `handleMetadata()` noting which fields SceneManager consumes |
| F-02 | `handleFrame()` | `window.overlays` coupling in SceneManager silently no-ops on live page | Add comment near `sceneManager.updateFrame()` call noting overlays are not set up on the live page |
| F-03 | `onStartSimulation()` configure payload | `mass`/`restitution`/`friction` always sent for environment objects, even though they are unused by the backend | Omit physics fields for environment objects in the configure payload (see FU-01) |
| F-04 | `handleFrame()` | `simulation_time` guard is defensive and correct, but the field contract was assumed rather than verified | Verify against actual pybind11 binding return type (see FU-02) |
| F-05 | `onStopSimulation()` | 500ms force-close timeout undocumented; behavior under high latency not specified | Add inline comment explaining the timeout rationale |

---

## Section 8: Follow-Up Items

### FU-01: Omit physics fields from environment object configure payloads

**Severity**: Minor / Cosmetic
**File**: `replay/static/js/live-app.js`, `onStartSimulation()` lines 254-262
**Change**: When `entry.object_type === 'environment'`, exclude `mass`, `restitution`,
and `friction` from the payload object. The backend `SpawnObjectConfig` provides defaults,
so omitting them is safe and cleaner.

```js
// Current (sends mass/restitution/friction for all objects):
const objects = spawnList.map(entry => ({
    asset_name:  entry.asset_name,
    position:    entry.position,
    orientation: entry.orientation,
    object_type: entry.object_type,
    mass:        entry.mass,
    restitution: entry.restitution,
    friction:    entry.friction,
}));

// Proposed (omit physics fields for environment objects):
const objects = spawnList.map(entry => {
    const obj = {
        asset_name:  entry.asset_name,
        position:    entry.position,
        orientation: entry.orientation,
        object_type: entry.object_type,
        restitution: entry.restitution,
        friction:    entry.friction,
    };
    if (entry.object_type === 'inertial') {
        obj.mass = entry.mass;
    }
    return obj;
});
```

Note: `restitution` and `friction` may still be meaningful for environment objects
(they affect the collision response from the inertial side). The most conservative
change is to omit only `mass`.

### FU-02: Verify `engine.get_frame_state()` always includes `simulation_time`

**Severity**: Minor / Verification
**File**: `replay/replay/routes/live.py`, integration with pybind11 Engine
**Change**: Confirm via the 0072a ticket or pybind11 binding source that
`get_frame_state()` always returns a `simulation_time` key. If it does not, either
patch `live.py` to inject `simulation_time` (derived from `sim_time_ms / 1000.0`)
alongside `frame_id`, or update `live-app.js` to compute it from `frame_id *
timestep_ms / 1000.0` as a fallback. The defensive guard in `live-app.js` is correct
but the root contract should be verified.

### FU-03: Add inline documentation for window.overlays coupling

**Severity**: Minor / Documentation
**File**: `replay/static/js/live-app.js`, `handleFrame()` (line 365)
**Change**: Add a comment:

```js
// Note: SceneManager.updateFrame() also checks window.overlays for overlay updates.
// The live page does not set up overlays, so those checks are silent no-ops.
sceneManager.updateFrame(frameData);
```

### FU-04: Add comment explaining 500ms stop timeout

**Severity**: Minor / Documentation
**File**: `replay/static/js/live-app.js`, `onStopSimulation()` (line 415)
**Change**: Add a comment:

```js
// Give the server a brief window to send 'complete' before force-closing.
// If 'complete' arrives within this window, handleComplete() runs first and
// nulls socket; the guard below prevents a double-close.
setTimeout(() => {
    if (socket) {
        socket.close();
        socket = null;
    }
}, 500);
```

---

## Section 9: Overall Assessment

**The implementation is correct and complete.**

All R1–R7 requirements are satisfied. The SceneManager integration is verified to use
the correct API with exactly matching field names. The WebSocket wire protocol is
correctly implemented on both the configure and frame paths. The UI state machine
correctly handles all documented states and transitions.

The gaps identified are all minor and fall into two categories:

1. **Documentation gaps**: Things that work correctly but whose rationale is not
   recorded (DD-0072C-001 through DD-0072C-005, F-02, F-05).
2. **Minor payload hygiene**: Sending extra fields for environment objects (F-03) is
   harmless but inconsistent with the UI design intent.

No production defects were found. Follow-up items FU-01 through FU-04 are all cosmetic
or documentation-level and can be addressed in a small cleanup commit on the same branch
or deferred.

**Retroactive verdict: APPROVED with minor follow-up items.**
