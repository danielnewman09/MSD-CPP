# Ticket 0062d: Convert Contact Manifold Stability + Edge Contact Tests

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Feature / Testing
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-13
**Generate Tutorial**: No
**Parent Ticket**: [0062_replay_collision_test_suite](0062_replay_collision_test_suite.md)
**Depends On**: [0062a_extend_test_asset_generator](0062a_extend_test_asset_generator.md)

---

## Overview

Convert `ContactManifoldStabilityTest.cpp` (2 tests) and the multi-frame tests in `EdgeContactTest.cpp` (7 of 11 tests) to use the `ReplayEnabledTest` fixture. These test contact stability over long durations (1000 frames) and edge-edge contact geometry — scenarios where replay visualization is most useful for debugging jitter, contact cycling, and instability.

---

## Requirements

### R1: Convert ContactManifoldStabilityTest.cpp (285 LOC, 2 tests)

Tests to convert:
- `ContactManifoldStability_RestingContact_1000Frames` — Cube resting on floor for 1000 frames
- `ContactManifoldStability_StackedCubes_1000Frames` — Two cubes stacked for 1000 frames

These are the longest-running tests in the suite and the most valuable for visual debugging — contact jitter and stability issues are immediately visible in replay.

### R2: Convert EdgeContactTest.cpp Multi-Frame Tests (7 of 11 tests)

Convert only the tests that run multi-frame simulations. Skip the 4 tests that are single-shot edge detection checks (geometric, not temporal).

Multi-frame tests to convert:
- `EdgeContact_CubeOnCubeEdge_Settles`
- `EdgeContact_RotatedCubeOnFloor_Settles`
- `EdgeContact_EdgeToEdge_BounceAndSettle`
- `EdgeContact_CornerDrop_Stabilizes`
- `EdgeContact_TiltedStack_Settles`
- `EdgeContact_RollingEdgeContact_Damping`
- `EdgeContact_GlancingEdge_DeflectionAngle`

Keep as-is (single-shot, no replay value):
- `EdgeContact_ParallelEdges_DetectsContact`
- `EdgeContact_PerpendicularEdges_DetectsContact`
- `EdgeContact_SkewEdges_ClosestPoints`
- `EdgeContact_DegenerateEdge_FallsBackToFace`

---

## Acceptance Criteria

- [ ] AC1: Both ContactManifoldStabilityTest tests pass using ReplayEnabledTest
- [ ] AC2: All 7 multi-frame EdgeContactTest tests pass using ReplayEnabledTest
- [ ] AC3: 4 single-shot EdgeContactTest tests remain unchanged and still pass
- [ ] AC4: Each converted test produces a `.db` recording
- [ ] AC5: 1000-frame stability tests produce viewable recordings showing settling behavior
- [ ] AC6: Zero test regressions

---

## Technical Notes

### Resting Contact Scenarios

These tests place objects on surfaces and verify stability over many frames. The floor in `ReplayEnabledTest` is the Engine's built-in floor (spawned by Engine constructor at z=-60). Tests should spawn cubes high enough to fall and settle on this floor, or use `spawnEnvironment("floor_slab", ...)` to create a custom floor.

### Edge Contact Geometry

Edge-edge contacts require specific cube orientations (rotated 45 degrees, etc.). Use `AngularCoordinate` with the spawn methods or set quaternion orientation post-spawn via `getReferenceFrame().setQuaternion()`.

### Recording Size

1000-frame recordings at 60 FPS produce ~16 seconds of replay data. This is well within reasonable database sizes but should be noted in test documentation.

---

## Workflow Log

| Phase | Date | Agent | Notes |
|-------|------|-------|-------|
| Draft | 2026-02-13 | Human + Claude | Initial ticket creation |
