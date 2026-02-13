# Ticket 0056f: Three.js Overlays and Debugging Tools

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Review
**Type**: Feature
**Priority**: Low
**Assignee**: TBD
**Created**: 2026-02-11
**Generate Tutorial**: No
**Parent Ticket**: [0056_browser_simulation_replay](0056_browser_simulation_replay.md)
**Depends On**: [0056e_threejs_core_visualization](0056e_threejs_core_visualization.md)

---

## Overview

Add visualization overlays to the Three.js replay viewer for debugging physics behavior. This includes contact point markers, force vector arrows, energy graphs, and body inspection. Each overlay layer can be toggled independently.

This is the ticket that transforms the viewer from a simple body-motion replay into a powerful physics debugging tool.

---

## Requirements

### R1: Contact Point Visualization

- Red spheres at contact point midpoints (midpoint of pointA and pointB)
- Configurable sphere radius (default: 0.02 relative to scene scale)
- Updated each frame from contact data

### R2: Contact Normal Arrows

- Red `ArrowHelper` from contact midpoint in normal direction
- Length proportional to penetration depth (with minimum visibility)
- Updated each frame

### R3: Constraint Force Arrows

- Green `ArrowHelper` from body center-of-mass in force direction
- Logarithmic magnitude scaling: `length = log1p(|F|) * scale`
- Threshold: skip arrows below 0.01 N magnitude
- Updated each frame

### R4: Gravity Force Arrows

- Blue `ArrowHelper` from body center-of-mass downward
- Magnitude proportional to body mass (from metadata)
- Constant per body (does not change per frame)

### R5: Energy Graph Overlay

- 2D chart (Chart.js) overlaid on the 3D viewport
- X-axis: frame number or simulation time
- Y-axis: energy (Joules)
- Lines: total energy, linear KE, rotational KE, potential energy
- Vertical marker showing current frame position
- Collapsible panel (minimize to corner)
- Data from `/api/v1/simulations/{id}/energy` endpoint

### R6: Body Selection and Inspector

- Click on body to select (highlight with wireframe overlay or emissive color)
- Info panel shows selected body properties:
  - Body ID, asset name
  - Mass, restitution, friction coefficient
  - Current position, velocity, angular velocity
  - Current forces (gravity, constraint)
  - Energy (KE, PE, total)
- Raycasting for body picking

### R7: Overlay Toggle Controls

- Checkbox panel for toggling each overlay:
  - [ ] Contact Points
  - [ ] Contact Normals
  - [ ] Constraint Forces
  - [ ] Gravity Arrows
  - [ ] Energy Graph
- All toggles independent
- Default: all off except Energy Graph

### R8: Solver Diagnostics Display

- Small status bar showing per-frame solver info:
  - Iteration count
  - Residual
  - Convergence status
  - Active constraint count
- Turns red on non-convergence frames

---

## Files to Create/Modify

### New Files (under `replay/static/`)
| File | Purpose |
|------|---------|
| `js/overlays/contacts.js` | Contact point and normal visualization |
| `js/overlays/forces.js` | Constraint force and gravity arrow visualization |
| `js/overlays/energy.js` | Energy chart (Chart.js integration) |
| `js/overlays/inspector.js` | Body selection and info panel |
| `js/overlays/solver.js` | Solver diagnostics status bar |

### Modified Files
| File | Change |
|------|--------|
| `replay/static/index.html` | Add overlay toggle panel, Chart.js CDN import |
| `replay/static/js/app.js` | Initialize overlay modules |
| `replay/static/js/scene.js` | Add overlay update calls in render loop |
| `replay/static/css/style.css` | Overlay panel and chart styling |

---

## Test Plan

### Manual Testing

1. Enable contact point overlay → verify red spheres appear at collision locations
2. Enable contact normal overlay → verify arrows point in normal direction
3. Enable constraint force overlay → verify green arrows on bodies during collision
4. Enable gravity overlay → verify blue downward arrows proportional to mass
5. Toggle overlays off → verify all overlay objects removed from scene
6. Verify energy chart updates with playback
7. Click body → verify selection highlight and info panel
8. Verify solver diagnostics update per frame
9. Non-convergence frame → verify red indicator

---

## Acceptance Criteria

1. [ ] **AC1**: Contact points visible as red spheres at correct locations
2. [ ] **AC2**: Contact normals visible as red arrows
3. [ ] **AC3**: Constraint forces visible as green arrows with log scaling
4. [ ] **AC4**: Gravity arrows visible as blue arrows proportional to mass
5. [ ] **AC5**: Energy chart displays all energy components vs frame
6. [ ] **AC6**: Body selection highlights body and shows properties
7. [ ] **AC7**: All overlays can be toggled independently
8. [ ] **AC8**: Solver diagnostics display per-frame stats

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

---

## Workflow Log

### Preparation Phase
- **Started**: 2026-02-13 (orchestrator)
- **Completed**: 2026-02-13
- **Branch**: 0056f-threejs-overlays (exists, based on 0056e)
- **Artifacts**: None yet
- **Notes**: Ticket advanced to Ready for Implementation. The branch already contains the full 0056e implementation (5 commits of proven code). Implementation will ADD overlay modules on top of existing viewer infrastructure.

### Implementation Phase
- **Started**: 2026-02-13 (orchestrator)
- **Completed**: 2026-02-13
- **Branch**: 0056f-threejs-overlays
- **Commit**: 712b404
- **PR**: N/A (sub-ticket, will merge to parent branch)
- **Artifacts**:
  - `replay/static/js/overlays/contacts.js` (159 lines) — Contact points and normals
  - `replay/static/js/overlays/forces.js` (168 lines) — Constraint and gravity forces
  - `replay/static/js/overlays/energy.js` (170 lines) — Chart.js energy graph
  - `replay/static/js/overlays/inspector.js` (181 lines) — Body selection and properties
  - `replay/static/js/overlays/solver.js` (75 lines) — Solver diagnostics display
  - Modified: `index.html`, `style.css`, `app.js`, `scene.js`, `ui.js`
- **Notes**: All 8 overlay requirements (R1-R8) implemented. Each overlay is independently toggleable. Default state: Energy graph enabled. Contact point/normal visualization uses red spheres and arrows. Constraint forces use green arrows with logarithmic scaling. Gravity uses blue arrows proportional to mass. Inspector uses raycasting for body selection. Solver diagnostics highlight non-convergence in red. Chart.js integration for energy graph with frame marker annotation.
