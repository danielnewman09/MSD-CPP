# Ticket 0072: Live Browser Simulation

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Feature (Parent)
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-17
**Generate Tutorial**: No
**Requires Math Design**: No

---

## Summary

Add a live simulation mode to the replay server where users configure initial conditions (spawn objects with positions and masses) in the browser, start a simulation, and watch it stream in real-time via WebSocket. The existing replay viewer at `/` remains untouched; the new feature lives at `/live`.

## Motivation

The current system only supports post-hoc replay of pre-recorded simulations. Live browser-driven simulation enables rapid experimentation with different configurations without needing to compile C++ executables or manually run command-line tools. This bridges the gap between the C++ simulation engine and interactive exploration.

## Requirements

### Functional Requirements
1. The system shall expose the C++ `Engine` class to Python via pybind11 bindings
2. The system shall provide a WebSocket endpoint for real-time simulation streaming
3. The system shall provide a `/live` browser page with spawn configuration and 3D visualization
4. The system shall reuse the existing `FrameData` wire format for compatibility with Three.js rendering

### Non-Functional Requirements
- **Performance**: Simulation should stream at interactive frame rates (~60 FPS for small scenes)
- **Memory**: Each WebSocket connection creates its own Engine instance; memory-isolated
- **Thread Safety**: Engine is single-threaded; use `asyncio.to_thread()` for non-blocking I/O
- **Backward Compatibility**: Existing replay viewer at `/` must be completely unaffected

## Sub-Tickets

| Ticket | Description | Depends On |
|--------|-------------|------------|
| [0072a](0072a_engine_pybind_bindings.md) | Engine pybind11 bindings | — |
| [0072b](0072b_websocket_simulation_endpoint.md) | WebSocket simulation endpoint | 0072a |
| [0072c](0072c_live_simulation_frontend.md) | Live simulation frontend | 0072b |

## Acceptance Criteria
- [ ] All three sub-tickets completed and merged
- [ ] Existing replay viewer at `/` is unmodified and functional
- [ ] User can navigate to `/live`, configure objects, start simulation, and watch in real-time

---

## Design Decisions (Human Input)

### Preferred Approaches
- **Wrapper with dict returns**: Convert all C++ types (Coordinate, Quaternion, etc.) to Python-native dicts/tuples inside the C++ wrapper. Do not expose Eigen types to Python.
- **asyncio.to_thread()**: Run Engine.update() in thread pool, not subprocess
- **Separate /live page**: Do not modify the existing replay viewer

### Things to Avoid
- Do not expose Eigen::Vector3d or Eigen::Quaterniond directly to Python (fragile type casters)
- Do not modify existing replay REST endpoints or frontend code
- Do not add mid-simulation interactivity (spawn-and-watch only for this ticket)

---

## References

### Related Code
- `msd/msd-sim/src/Engine.hpp` — C++ Engine class to be exposed
- `msd/msd-pybind/src/` — Existing pybind11 bindings (pattern to follow)
- `replay/replay/app.py` — FastAPI application to extend
- `replay/static/js/scene.js` — SceneManager to reuse in live page
- `replay/replay/models.py` — Pydantic models for wire format

### Related Tickets
- [0056_browser_simulation_replay](0056_browser_simulation_replay.md) — Original replay system
- [0056c_python_bindings](0056c_python_bindings.md) — Existing pybind11 module

---

## Workflow Log

### Implementation Phase
- **Started**:
- **Completed**:
- **Notes**: Parent ticket tracks sub-ticket progress

---

## Human Feedback
