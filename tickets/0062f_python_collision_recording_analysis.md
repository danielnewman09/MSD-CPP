# Ticket 0062f: Python Recording Analysis for Collision Tests

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Feature / Testing
**Priority**: Low
**Assignee**: TBD
**Created**: 2026-02-13
**Generate Tutorial**: No
**Parent Ticket**: [0062_replay_collision_test_suite](0062_replay_collision_test_suite.md)
**Depends On**: [0062b_replay_linear_collision_tests](0062b_replay_linear_collision_tests.md), [0062c_replay_rotational_collision_tests](0062c_replay_rotational_collision_tests.md)

---

## Overview

Create Python pytest tests that analyze the `.db` recordings produced by the converted collision tests (0062b-0062e). These use the `RecordingQuery` API and assertion helpers from the 0060 series to perform post-hoc physics validation on collision recordings.

This completes the two-language test pattern: C++ produces recordings with in-memory assertions, Python validates recordings with post-hoc analysis.

---

## Requirements

### R1: Linear Collision Recording Analysis

Python tests for recordings produced by 0062b:
- Verify momentum conservation across collision frames (pre/post collision momentum delta < tolerance)
- Verify energy conservation for elastic collisions (max energy drift < tolerance)
- Verify energy loss for inelastic collisions (final KE < initial KE)
- Verify frame count matches expected simulation duration

### R2: Rotational Collision Recording Analysis

Python tests for recordings produced by 0062c:
- Verify angular momentum conservation in zero-gravity scenarios
- Verify rotational energy partitioning (translational + rotational = total)
- Verify settling behavior (final speed < threshold after damping)
- Verify rotation damping tests show decreasing angular velocity over time

### R3: Contact Stability Recording Analysis

Python tests for recordings produced by 0062d:
- Verify floor penetration never exceeds threshold over 1000 frames
- Verify position stability (object doesn't drift after settling)
- Verify contact manifold consistency (no sudden jumps in contact count)

### R4: Use Existing Infrastructure

All Python tests should use:
- `RecordingQuery` from 0060b for database queries
- `assert_energy_conserved`, `assert_never_penetrates_below`, `assert_body_comes_to_rest` from 0060c
- `conftest.py` fixtures from 0060c for recording path resolution
- `pytest.mark.skipif` for graceful skip when `msd_reader` unavailable

---

## Acceptance Criteria

- [ ] AC1: Python tests exist for linear collision recordings (momentum, energy)
- [ ] AC2: Python tests exist for rotational collision recordings (angular momentum, settling)
- [ ] AC3: Python tests exist for stability recordings (penetration, drift)
- [ ] AC4: All Python tests skip gracefully when `msd_reader` unavailable
- [ ] AC5: Tests pass when recordings are available and `msd_reader` is built
- [ ] AC6: Tests use the 0060-series infrastructure (RecordingQuery, assertions, conftest)

---

## Technical Notes

### Recording Path Convention

Recordings are at `replay/recordings/{TestSuite}_{TestName}.db`. Python tests use `recording_for("TestSuite", "TestName")` helper from conftest to resolve paths.

### New RecordingQuery Methods

Some analysis may need `RecordingQuery` methods not yet implemented (e.g., angular velocity history, per-body momentum). If so, extend `RecordingQuery` in this ticket or create a follow-on.

### Test Independence

Python tests should not depend on C++ tests running first in the same session. They should check for recording file existence and skip if not found, allowing the test suites to run independently.

---

## Workflow Log

| Phase | Date | Agent | Notes |
|-------|------|-------|-------|
| Draft | 2026-02-13 | Human + Claude | Initial ticket creation |
