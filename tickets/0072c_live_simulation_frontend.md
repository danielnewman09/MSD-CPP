# Ticket 0072c: Live Simulation Frontend

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Merged / Complete

**Current Phase**: Merged / Complete
**Type**: Feature
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-17
**Generate Tutorial**: No
**Requires Math Design**: No
**Parent Ticket**: [0072_live_browser_simulation](0072_live_browser_simulation.md)
**Depends On**: [0072b_websocket_simulation_endpoint](0072b_websocket_simulation_endpoint.md)

---

## Overview

Create a `/live` HTML page with a spawn configuration panel and real-time Three.js visualization. The page connects to the WebSocket endpoint from ticket 0072b, lets users configure initial conditions, and renders the simulation as it streams.

The existing `SceneManager` from `replay/static/js/scene.js` is reused — its `loadBodies()` and `updateFrame()` methods accept the exact data format the WebSocket delivers.

---

## Requirements

### R1: Live Page HTML

Create `replay/static/live.html` with:
- Full-viewport `<canvas>` for Three.js rendering (same setup as `index.html`)
- Configuration sidebar panel
- Status bar at bottom

### R2: Configuration Panel

Sidebar with:
- **Asset type dropdown**: populated from `GET /api/v1/live/assets` on page load
- **Position inputs**: X, Y, Z number fields (defaults: 0, 0, 5)
- **Mass input**: number field (default: 10.0, shown only for inertial type)
- **Restitution input**: number field (default: 0.8, range 0-1)
- **Friction input**: number field (default: 0.5, range 0+)
- **Object type radio**: Inertial / Environment
- **"Add Object" button**: adds to spawn list
- **Spawn list**: shows configured objects with name, position, type, and remove button

### R3: Simulation Controls

- **Timestep dropdown**: 10ms, 16ms (~60fps), 33ms (~30fps)
- **Duration input**: seconds (default: 30)
- **"Start Simulation" button**: disabled until at least one object is added
- **"Stop" button**: visible only during active simulation

### R4: WebSocket Client

Create `replay/static/js/live-app.js`:

1. On page load: fetch `GET /api/v1/live/assets`, populate asset dropdown
2. On "Start Simulation":
   - Open WebSocket to `ws://{host}/api/v1/live`
   - Send `configure` message with spawn list
   - Receive `metadata` response
   - Call `SceneManager.loadBodies(metadata, geometries)` to build 3D scene
   - Send `start` message with timestep and duration
   - On each `frame` message: call `SceneManager.updateFrame(frameData)`
   - On `complete` message: show summary, re-enable controls
3. On "Stop": send `stop` message, close WebSocket
4. On error/disconnect: show error, re-enable controls

### R5: SceneManager Reuse

Import and reuse `SceneManager` from `replay/static/js/scene.js`:
- `loadBodies(metadata, geometries)` — sets up Three.js meshes from metadata
- `updateFrame(frameData)` — updates mesh positions/rotations from frame state

No modifications to `scene.js` should be needed. If the metadata format from the WebSocket `metadata` message differs slightly from the replay metadata, adapt in `live-app.js` before calling `loadBodies()`.

### R6: Status Bar

Bottom bar showing:
- Connection state (Disconnected / Connected / Simulating / Complete)
- Simulation time (seconds)
- Frame count
- Body count

### R7: CSS

Create `replay/static/css/live.css` with styles for:
- Sidebar layout (fixed-width left panel, remaining viewport for canvas)
- Form inputs and buttons
- Spawn list styling
- Status bar

---

## Constraints
- Must not modify `index.html` or any existing JS files in `replay/static/js/`
- `SceneManager` is imported as an ES module — `live.html` must use `<script type="module">`
- Three.js loaded via CDN (same version as `index.html`)
- Asset dropdown depends on `GET /api/v1/live/assets` from ticket 0072b

## Acceptance Criteria
- [ ] `http://localhost:8000/live` loads the live simulation page
- [ ] Asset dropdown is populated with available assets
- [ ] Adding objects shows them in the spawn list with remove buttons
- [ ] Removing objects from spawn list works
- [ ] "Start Simulation" is disabled when spawn list is empty
- [ ] Clicking "Start" opens WebSocket, configures scene, and begins rendering
- [ ] 3D scene shows objects moving under gravity and colliding
- [ ] Status bar updates with simulation time and frame count
- [ ] "Stop" terminates simulation and re-enables controls
- [ ] WebSocket disconnect shows error state
- [ ] Existing replay viewer at `http://localhost:8000/` is completely unaffected

---

## Design Decisions (Human Input)

### Preferred Approaches
- Keep the live page self-contained: `live.html` + `live-app.js` + `live.css`
- Import `SceneManager` as ES module, do not copy/fork it
- Adapt metadata format in `live-app.js` if needed (thin adapter, not SceneManager changes)

### Things to Avoid
- Do not modify existing replay viewer files (`index.html`, `app.js`, `playback.js`, etc.)
- Do not add complex state management — simple procedural flow is sufficient
- Do not add playback controls (this is live, not replay)

---

## References

### Related Code
- `replay/static/index.html` — Existing page structure to reference (not modify)
- `replay/static/js/scene.js` — SceneManager to reuse (loadBodies, updateFrame)
- `replay/static/js/data.js` — DataLoader pattern reference (not reused for live)
- `replay/static/js/app.js` — Existing app entry point (not modified)
- `replay/replay/routes/live.py` — WebSocket endpoint from 0072b

### Related Tickets
- [0072b_websocket_simulation_endpoint](0072b_websocket_simulation_endpoint.md) — WebSocket backend (prerequisite)
- [0056e_threejs_core_visualization](0056e_threejs_core_visualization.md) — Original Three.js viewer implementation

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-02-17 00:00
- **Completed**: 2026-02-17 00:00
- **Branch**: 0072c-live-simulation-frontend
- **PR**: #78 (targeting 0072-live-browser-simulation)
- **Files Created**:
  - `replay/static/live.html`
  - `replay/static/js/live-app.js`
  - `replay/static/css/live.css`
- **Files Modified**: None (index.html and all existing JS files untouched)
- **Notes**: Draft skipped Math Design (not required) and Design phases — requirements fully specified in ticket. Implemented all R1–R7 requirements. SceneManager imported as ES module from scene.js; metadata format from WebSocket aligns exactly with SceneManager.loadBodies() field expectations (body_id, asset_id, is_environment, positions). Thin adapter comment included in handleMetadata() confirming no field mapping needed. Frame data forwarded directly to SceneManager.updateFrame(). WebSocket lifecycle: configure → metadata → start → frame streaming → complete/stop. Controls disabled during simulation to prevent concurrent sessions.

### Quality Gate Phase
- **Completed**: 2026-02-17
- **Result**: PASSED
- **Checks**:
  - All 10 acceptance criteria verified via grep/diff analysis
  - `index.html` unchanged (0 diff lines)
  - All existing JS files unchanged: scene.js, app.js, playback.js, ui.js, data.js
  - SceneManager imported as ES module — field names align exactly, no adapter required
  - WebSocket lifecycle fully implemented: configure → metadata → start → frame → complete/stop
  - Error handling on disconnect: shows error state, re-enables controls

### Implementation Review Phase
- **Started**: 2026-02-17
- **Completed**: 2026-02-17
- **Status**: APPROVED
- **Branch**: 0072c-live-simulation-frontend
- **PR**: #78 (targeting 0072-live-browser-simulation)
- **Artifacts**:
  - `docs/designs/0072c-live-simulation-frontend/implementation-review.md`
- **Reviewer Notes**: All 11 acceptance criteria satisfied. Zero lines changed in existing static files. SceneManager imported as ES module with exact field alignment. WebSocket lifecycle and error handling complete. Two cosmetic minor notes (no action required).

### Documentation Phase
- **Started**: 2026-02-17
- **Completed**: 2026-02-17
- **Branch**: 0072c-live-simulation-frontend
- **PR**: #78 (targeting 0072-live-browser-simulation)
- **Artifacts**:
  - `docs/designs/0072c-live-simulation-frontend/implementation-review.md`
  - `docs/designs/0072c-live-simulation-frontend/doc-sync-summary.md`
- **Files Modified**:
  - `replay/README.md` — Architecture section updated with `live.html`, `live-app.js`, `live.css` entries
- **Notes**: No C++ or CLAUDE.md updates required — pure-frontend feature. No PlantUML diagrams produced. No msd-transfer changes (record layer sync skipped). README already contained API endpoint entries from 0072b; only static file inventory was missing.

---

## Human Feedback
