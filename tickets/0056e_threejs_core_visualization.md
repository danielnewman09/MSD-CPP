# Ticket 0056e: Three.js Core Visualization

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
**Depends On**: [0056d_fastapi_backend](0056d_fastapi_backend.md)

---

## Overview

Build the core Three.js web application that loads simulation recordings and provides frame-by-frame playback of rigid body motion. This ticket covers scene setup, body rendering, transform updates, playback controls, and camera interaction.

**Frontend approach**: Vanilla JavaScript with ES module imports from CDN (Three.js, Chart.js). No React/Vue/npm build tooling. The frontend is a specialized visualization tool, not a general web application.

---

## Requirements

### R1: Scene Setup

- Three.js scene with ambient + directional lighting
- Ground grid (GridHelper) for spatial reference
- Axis helpers (AxesHelper) for orientation reference
- WebGL renderer with antialiasing
- Responsive canvas (resize with window)

### R2: Body Rendering

- Load geometry from `/api/v1/simulations/{id}/assets` endpoint
- Create Three.js `BufferGeometry` from flat position arrays
- Compute vertex normals via `geometry.computeVertexNormals()`
- Distinct materials for dynamic (blue, opaque) vs environment (gray, transparent) bodies
- Bodies stored in a `Map<bodyId, THREE.Mesh>` for efficient per-frame updates

### R3: Frame-by-Frame Transform Updates

- Fetch frame data from REST API
- Update each body's `position` and `quaternion` from frame state
- Handle quaternion convention (w, x, y, z from C++ → Three.js)

### R4: Playback Controls

- **Play/Pause** button
- **Step Forward/Back** buttons (single frame)
- **Timeline scrubber** (slider showing current frame position)
- **Speed control** (0.25x, 0.5x, 1x, 2x, 4x)
- **Frame counter** display (current frame / total frames)
- **Simulation time** display
- Keyboard shortcuts: Space (play/pause), Left/Right arrows (step), +/- (speed)

### R5: Frame Data Buffering

- Prefetch frames ahead of playback position using bulk range endpoint
- Cache fetched frames in memory (`Map<frameId, FrameData>`)
- Buffer size: ~50 frames ahead of current position
- Request new batch when buffer runs low

### R6: Camera Controls

- Three.js OrbitControls (orbit, pan, zoom)
- Reset camera button to return to default view
- Default camera position looking at scene center from above-front

### R7: Simulation Selection

- Dropdown or list to select which recording to load
- Populated from `/api/v1/simulations` endpoint
- Loading indicator while geometry and initial frames load

---

## Files to Create

All files under `replay/static/`:

| File | Purpose |
|------|---------|
| `index.html` | Main HTML page with viewport and controls UI |
| `css/style.css` | Layout and control styling |
| `js/app.js` | Main entry point, initialization |
| `js/scene.js` | SceneManager — Three.js scene, bodies, rendering |
| `js/playback.js` | PlaybackController — timeline, play/pause/step/speed |
| `js/data.js` | DataLoader — REST API client with frame buffering |
| `js/ui.js` | UI controls binding and updates |

---

## Test Plan

### Manual Testing

1. Start FastAPI backend with a recording database
2. Open browser to `http://localhost:8000`
3. Verify: scene renders with grid and axes
4. Verify: bodies appear in correct initial positions
5. Verify: play button advances frames, bodies move
6. Verify: step forward/back moves one frame at a time
7. Verify: timeline scrubber jumps to arbitrary frame
8. Verify: speed control changes playback rate
9. Verify: camera orbit/pan/zoom works
10. Verify: simulation selector loads different recordings

### Automated Testing

Automated browser testing is out of scope for v1. Manual verification is sufficient for a development-only debugging tool.

---

## Acceptance Criteria

1. [ ] **AC1**: Three.js scene renders with lighting, grid, and axes
2. [ ] **AC2**: Bodies loaded from asset endpoint and rendered correctly
3. [ ] **AC3**: Frame playback updates body positions and orientations smoothly
4. [ ] **AC4**: Play/pause/step/scrub controls functional
5. [ ] **AC5**: Speed control works (0.25x to 4x)
6. [ ] **AC6**: Camera controls (orbit, pan, zoom) work
7. [ ] **AC7**: Frame buffering prevents playback stutter
8. [ ] **AC8**: Multiple recordings can be loaded via simulation selector

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
