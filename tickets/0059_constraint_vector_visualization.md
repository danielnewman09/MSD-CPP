# Ticket 0059: Constraint Vector Visualization in Replay Viewer

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Feature
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-13
**Generate Tutorial**: No
**Parent Ticket**: [0056f_threejs_overlays](0056f_threejs_overlays.md)
**Depends On**: [0057_contact_tangent_recording](0057_contact_tangent_recording.md)

---

## Overview

Visualize contact normal and friction tangent vectors as colored arrows in the Three.js replay viewer. Ticket 0057 delivered the C++ infrastructure to record constraint state (ContactConstraintRecord, FrictionConstraintRecord) to SQLite via the visitor pattern. This ticket completes the pipeline: Python bindings, REST API exposure, and Three.js arrow rendering.

The end result is three orthogonal arrows at each contact point during collision frames:
- **Red**: Contact normal (from ContactConstraintRecord)
- **Green**: Tangent t1 (from FrictionConstraintRecord)
- **Blue**: Tangent t2 (from FrictionConstraintRecord)

---

## Background

### What 0057 Delivered (C++ Infrastructure)
- `ConstraintRecordVisitor` interface with compile-time type safety (visitor pattern)
- `ContactConstraintRecord` and `FrictionConstraintRecord` transfer records in msd-transfer
- `Constraint::recordState()` virtual method implemented by ContactConstraint and FrictionConstraint
- `CollisionPipeline::recordConstraints()` orchestrates iteration with body ID mapping
- `DataRecorder::recordConstraintStates()` buffers records via `DataRecorderVisitor`
- `WorldModel::recordCurrentFrame()` calls constraint recording in recording sequence

### What This Ticket Adds
- Python bindings exposing constraint records from SQLite
- REST API endpoint returning constraint data per frame
- Three.js arrow rendering at contact points with toggle controls

---

## Requirements

### R1: Python Bindings for Constraint Records
- pybind11 bindings for `FrictionConstraintRecord` and `ContactConstraintRecord`
- Read-only field access (`.body_a_id`, `.normal`, `.tangent1`, etc.)
- `msd_reader` module exposes query methods for constraint records by frame ID

### R2: REST API Constraint Data
- `/api/v1/simulations/{id}/frames/{frame_id}/state` includes `constraints` array
- Each constraint object includes type discriminator (`"contact"` or `"friction"`)
- Friction constraints include `normal`, `tangent1`, `tangent2`, `lever_arm_a`, `lever_arm_b`, `friction_coefficient`, `normal_lambda`
- Contact constraints include `normal`, `lever_arm_a`, `lever_arm_b`, `penetration_depth`, `restitution`
- Backward compatible: older recordings without constraint tables return empty `constraints` array

### R3: Three.js Arrow Rendering
- Three orthogonal `ArrowHelper` instances per friction constraint at contact point
  - Red: normal direction (1.0 unit length, fixed)
  - Green: tangent1 direction (1.0 unit length, fixed)
  - Blue: tangent2 direction (1.0 unit length, fixed)
- Contact point derived from body A position + `lever_arm_a`
- Arrows appear only on frames with active constraints
- Arrows removed when navigating to non-collision frames

### R4: Overlay Toggle
- Single "Contact Vectors" checkbox in overlay panel
- Toggles all three arrow types (normal + t1 + t2) together
- Default: off
- Integrates with existing overlay toggle panel from 0056f

---

## Files to Create/Modify

### Python (replay)
| File | Change |
|------|--------|
| `msd-pybind/src/record_bindings.cpp` | Add `py::class_` for FrictionConstraintRecord, ContactConstraintRecord |
| `replay/replay/models.py` | Add `FrictionConstraintInfo`, `ContactConstraintInfo` Pydantic models |
| `replay/replay/services/simulation_service.py` | Query constraint records by frame, construct response objects |

### Frontend (replay/static)
| File | Change |
|------|--------|
| `replay/static/js/overlays/contacts.js` | Add tangent arrow rendering (extends existing contact overlay) |
| `replay/static/index.html` | Add "Contact Vectors" toggle checkbox |
| `replay/static/js/app.js` | Initialize contact vector overlay |

---

## Test Plan

### Unit Tests
1. Python binding smoke test: read FrictionConstraintRecord fields from test database
2. Python binding smoke test: read ContactConstraintRecord fields from test database
3. REST API: verify `/state` response includes `constraints` array with correct fields
4. REST API: verify older recordings return empty `constraints` array (backward compat)

### Integration Tests
1. Generate recording with friction-enabled collision scenario
2. Query constraint records via Python — verify tangent vectors are unit length and orthogonal to normal
3. Hit REST endpoint — verify JSON constraint objects match database records

### Manual Tests
1. Open replay viewer, navigate to collision frame
2. Enable "Contact Vectors" overlay
3. Verify three orthogonal arrows (red, green, blue) at contact points
4. Navigate to non-collision frame — verify arrows disappear
5. Toggle overlay off — verify arrows removed
6. Open recording without constraint data — verify no errors, no arrows

---

## Acceptance Criteria

1. [ ] **AC1**: Python bindings expose all fields of FrictionConstraintRecord and ContactConstraintRecord
2. [ ] **AC2**: REST API returns constraint data in frame state response
3. [ ] **AC3**: Three.js renders red/green/blue arrows at contact points on collision frames
4. [ ] **AC4**: Arrows disappear on non-collision frames
5. [ ] **AC5**: "Contact Vectors" toggle controls arrow visibility
6. [ ] **AC6**: Older recordings without constraint tables load without error

---

## Implementation Notes

### Contact Point Derivation
Contact world position = body A position + lever_arm_a. The body A position comes from the `objects` array in the frame state (matched by `body_a_id`). Lever arms are stored as `Vector3DRecord` (direction semantics, not point semantics).

### Arrow Length
Fixed at 1.0 units per human preference (ticket 0057 feedback). No force-magnitude scaling.

### Backward Compatibility
cpp_sqlite handles missing tables gracefully. The Python layer should check for table existence or catch query errors, returning an empty list for recordings that predate constraint recording.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
