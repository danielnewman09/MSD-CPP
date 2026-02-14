# Ticket 0062e: Convert Diagnostic + Parameter Isolation Tests

## Status
- [x] Draft
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Implementation
**Type**: Feature / Testing
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-13
**Generate Tutorial**: No
**Parent Ticket**: [0062_replay_collision_test_suite](0062_replay_collision_test_suite.md)
**Depends On**: [0062a_extend_test_asset_generator](0062a_extend_test_asset_generator.md)

---

## Overview

Convert the remaining multi-frame collision tests to use `ReplayEnabledTest`:
- `ParameterIsolationTest.cpp` (989 LOC, 10+ tests) — parameter sensitivity analysis
- `EPAConvergenceDiagnosticTest.cpp` (632 LOC, 7 tests) — EPA algorithm convergence
- `ManifoldDiagnosticTest.cpp` (453 LOC, 5 tests) — contact manifold diagnostics
- `PerContactDepthTest.cpp` (380 LOC, 6 tests) — per-contact penetration tracking
- `CollisionPipelineTest.cpp` (237 LOC, 7 tests) — pipeline integration tests

This is the largest batch (~2,700 LOC) but lower priority since these are diagnostic/analysis tests rather than physics correctness tests.

---

## Requirements

### R1: Convert ParameterIsolationTest.cpp (10+ tests)

These tests sweep collision parameters (restitution, friction, mass ratios) and verify expected behavior at each point. Each test typically:
1. Creates two objects with specific parameters
2. Sets approaching velocities
3. Runs 50-200 frames
4. Checks post-collision state

Convert to fixture, using `spawnInertialWithVelocity()` with parameter overrides.

### R2: Convert EPAConvergenceDiagnosticTest.cpp (7 tests)

These test EPA algorithm convergence under various collision geometries (deep penetration, near-miss, grazing contact). Some are single-step checks that may not benefit from replay — convert only the multi-frame tests and keep single-step tests as-is.

### R3: Convert ManifoldDiagnosticTest.cpp (5 tests)

Contact manifold quality diagnostics — verify manifold point count, normal directions, and penetration depths over multiple frames.

### R4: Convert PerContactDepthTest.cpp (6 tests)

Per-contact penetration depth tracking over multi-frame simulations.

### R5: Convert CollisionPipelineTest.cpp (7 tests)

Integration tests for the full collision pipeline (detection → manifold → constraint → response). These already test the full pipeline and are good candidates for replay.

---

## Acceptance Criteria

- [ ] AC1: All ParameterIsolationTest tests pass using ReplayEnabledTest
- [ ] AC2: Multi-frame EPAConvergenceDiagnosticTest tests converted and passing
- [ ] AC3: All ManifoldDiagnosticTest tests pass using ReplayEnabledTest
- [ ] AC4: All PerContactDepthTest tests pass using ReplayEnabledTest
- [ ] AC5: All CollisionPipelineTest tests pass using ReplayEnabledTest
- [ ] AC6: Each converted test produces a `.db` recording
- [ ] AC7: Zero test regressions

---

## Technical Notes

### Parameter Sweep Pattern

ParameterIsolationTest runs many similar scenarios with different parameters. Consider using GTest parameterized tests (`TEST_P`) with the fixture if the conversion is cleaner that way. Each parameter combination produces its own recording.

### Selective Conversion

Some tests in EPAConvergenceDiagnosticTest are single-step geometric checks. Only convert tests that run `WorldModel::update()` in a loop. Keep single-step tests as-is (they don't benefit from replay).

### Recording Volume

This ticket produces the most recordings (~35+ test cases). Ensure `MSD_KEEP_RECORDINGS=0` cleans up properly to avoid disk bloat in CI.

---

## Workflow Log

| Phase | Date | Agent | Notes |
|-------|------|-------|-------|
| Draft | 2026-02-13 | Human + Claude | Initial ticket creation |
| Implementation (Partial) | 2026-02-14 | Claude Orchestrator | Converted ParameterIsolationTest.cpp (8/10 multi-frame tests). Remaining: EPAConvergenceDiagnosticTest, ManifoldDiagnosticTest, PerContactDepthTest, CollisionPipelineTest. Branch: 0062e-replay-diagnostic-parameter-tests |
