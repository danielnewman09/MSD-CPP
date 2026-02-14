# Ticket 0062d: Convert Contact Manifold Stability + Edge Contact Tests

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [x] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Approved — Ready to Merge
**Type**: Feature / Testing
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-13
**Generate Tutorial**: No
**Parent Ticket**: [0062_replay_collision_test_suite](0062_replay_collision_test_suite.md)
**Depends On**: [0062a_extend_test_asset_generator](0062a_extend_test_asset_generator.md)

---

## Overview

Convert `ContactManifoldStabilityTest.cpp` (2 tests) and the multi-frame test in `EdgeContactTest.cpp` (1 test) to use the `ReplayEnabledTest` fixture. These test contact stability over long durations (1000 frames) and edge-edge contact geometry — scenarios where replay visualization is most useful for debugging jitter, contact cycling, and instability.

**Note**: The initial ticket specification listed 7 multi-frame EdgeContact tests, but the actual `EdgeContactTest.cpp` file contains only 1 multi-frame test. The file primarily contains single-shot collision detection tests (static geometry checks).

---

## Requirements

### R1: Convert ContactManifoldStabilityTest.cpp (285 LOC, 2 tests)

Tests to convert:
- `ContactManifoldStability_RestingContact_1000Frames` — Cube resting on floor for 1000 frames
- `ContactManifoldStability_StackedCubes_1000Frames` — Two cubes stacked for 1000 frames

These are the longest-running tests in the suite and the most valuable for visual debugging — contact jitter and stability issues are immediately visible in replay.

### R2: Convert EdgeContactTest.cpp Multi-Frame Test (1 test)

**Actual implementation scope**: The `EdgeContactTest.cpp` file contains only 1 multi-frame test with a simulation loop. The remaining 10 tests are single-shot collision detection tests (static geometry checks).

Multi-frame test to convert:
- `EdgeContact_CubeEdgeImpact_InitiatesRotation` — Cube with edge pointing down, dropped onto floor (200-frame simulation)

Single-shot tests (keep as-is, no replay value - 10 tests):
- `ConvexHullEdge_FindClosestEdge_CubeVertex_ReturnsAdjacentEdge`
- `ConvexHullEdge_FindClosestEdge_CubeEdgeMidpoint_ReturnsThatEdge`
- `ConvexHullEdge_FindClosestEdge_CubeFaceCenter_ReturnsNearestEdge`
- `ConvexHullEdge_FindClosestEdge_TetrahedronVertex_ReturnsAdjacentEdge`
- `EdgeContact_CubeEdgeOnFloor_ProducesMultipleContacts`
- `EdgeContact_ContactPoints_HaveGeometricExtent`
- `EdgeContact_LeverArm_CrossNormal_NonZero`
- `EdgeContact_ContactPoints_HavePositiveDepth`
- `EdgeContact_FaceFaceContact_StillProducesMultipleContacts`
- `EdgeContact_SmallPenetration_StillDetected`

---

## Acceptance Criteria

- [x] AC1: Both ContactManifoldStabilityTest tests converted to ReplayEnabledTest (1 passes, 1 is known diagnostic failure per ticket 0047a)
- [x] AC2: EdgeContactTest multi-frame test converted to ReplayEnabledTest (produces recording; test marginal due to minimal rotation from edge contact)
- [x] AC3: 10 single-shot EdgeContactTest tests remain unchanged and still pass
- [x] AC4: Each converted test produces a `.db` recording
- [x] AC5: 1000-frame stability tests produce viewable recordings showing settling behavior
- [x] AC6: Zero new test regressions (D4_MicroJitter pre-existing diagnostic failure, EdgeContact marginal rotation pre-existing)

---

## Technical Notes

### Resting Contact Scenarios

These tests place objects on surfaces and verify stability over many frames. The floor in `ReplayEnabledTest` is the Engine's built-in floor (spawned by Engine constructor at z=-60). Tests should spawn cubes high enough to fall and settle on this floor, or use `spawnEnvironment("floor_slab", ...)` to create a custom floor.

### Edge Contact Geometry

Edge-edge contacts require specific cube orientations (rotated 45 degrees, etc.). Use `AngularCoordinate` with the spawn methods or set quaternion orientation post-spawn via `getReferenceFrame().setQuaternion()`.

### Recording Size

1000-frame recordings at 60 FPS produce ~16 seconds of replay data. This is well within reasonable database sizes but should be noted in test documentation.

### EdgeContact Test Marginal Behavior

The `EdgeContact_CubeEdgeImpact_InitiatesRotation` test exhibits minimal rotation (quaternionRate ~2e-13) due to constraint solver behavior (Baumgarte stabilization, ERP clamping). The test comments acknowledge this:

> "Even with 2 contact points providing geometric extent, the angular response may be very small due to constraint solver behavior... This integration test uses a relaxed threshold."

The test threshold (1e-10) is already relaxed, but actual behavior produces even smaller rotation. The recording is still valuable for visualizing edge contact behavior. The static collision detection tests verify the geometric correctness (2 contact points with non-zero r×n).

---

## Workflow Log

| Phase | Date | Agent | Notes |
|-------|------|-------|-------|
| Draft | 2026-02-13 | Human + Claude | Initial ticket creation |
| Implementation | 2026-02-14 | Claude (Orchestrator) | **Conversions completed**: ContactManifoldStabilityTest.cpp (2 tests), EdgeContactTest.cpp (1 test). **Ticket scope corrected**: EdgeContactTest.cpp contains 1 multi-frame test, not 7 (remaining 10 are single-shot static tests). **Branch**: `0062d-replay-stability-edge-contact-tests`. **PR**: #63 (draft). **Artifacts**: `ContactManifoldStabilityTest.cpp`, `EdgeContactTest.cpp` |
| Quality Gate | 2026-02-14 08:11 | code-quality-gate | **Status**: PASSED. Build verification: PASSED (no warnings with -Werror). Tests: 725/730 passed (5 failures: 2 expected pre-existing from 0062d, 3 unrelated). Static analysis: N/A (test-only changes). Benchmarks: N/A. **Report**: `docs/designs/0062d_replay_stability_edge_contact_tests/quality-gate-report.md` |
| Implementation Review | 2026-02-14 08:13 | implementation-reviewer | **Status**: APPROVED. All 3 multi-frame tests successfully converted to ReplayEnabledTest. Zero new regressions. 2 expected pre-existing failures documented (0047a diagnostic, edge contact marginal rotation). All recordings generated. **Review**: `docs/designs/0062d_replay_stability_edge_contact_tests/implementation-review.md`. **PR**: #63 (ready for review) |
