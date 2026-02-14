# Ticket 0062b: Convert Linear Collision + Energy Accounting Tests

## Status
- [x] Draft
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Ready for Implementation
**Type**: Feature / Testing
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-13
**Generate Tutorial**: No
**Parent Ticket**: [0062_replay_collision_test_suite](0062_replay_collision_test_suite.md)
**Depends On**: [0062a_extend_test_asset_generator](0062a_extend_test_asset_generator.md)

---

## Overview

Convert `LinearCollisionTest.cpp` (6 tests) and `EnergyAccountingTest.cpp` (4 tests) from direct WorldModel construction to `ReplayEnabledTest` fixture. These are the highest-value conversions because they test fundamental physics invariants (momentum conservation, energy tracking) over multi-frame simulations.

---

## Requirements

### R1: Convert LinearCollisionTest.cpp (473 LOC, 6 tests)

Replace `WorldModel` + `createSpherePoints()` + `ConvexHull` construction with `ReplayEnabledTest` fixture using `unit_sphere` assets.

Tests to convert:
- `LinearCollision_HeadOn_MomentumConserved` — Two equal spheres, head-on, verify momentum
- `LinearCollision_HeadOn_EnergyConserved` — Elastic collision, verify KE conservation
- `LinearCollision_MovingHitsStationary` — One moving, one stationary
- `LinearCollision_AsymmetricMass` — Different masses (use unit_sphere + small_sphere)
- `LinearCollision_GlancingBlow` — Off-axis collision
- `LinearCollision_MultipleCollisions` — Chain of collisions

Each test should:
1. Use `spawnInertialWithVelocity()` to create sphere objects with specific velocities
2. Use `disableGravity()` to isolate collision physics
3. Call `step()` for the same number of frames as the original test
4. Keep the same in-memory assertions (momentum, energy, velocity checks)
5. Produce a `.db` recording automatically via the fixture

### R2: Convert EnergyAccountingTest.cpp (376 LOC, 4 tests)

Tests to convert:
- `EnergyAccounting_ElasticCollision_TotalEnergyConserved`
- `EnergyAccounting_InelasticCollision_EnergyLost`
- `EnergyAccounting_PerfectlyInelastic_MaxEnergyLoss`
- `EnergyAccounting_GravityFreefall_EnergyConserved`

These tests verify the EnergyTracker system during collisions. Convert to use the fixture while preserving all energy tracking assertions.

### R3: Preserve Test Semantics

The converted tests must:
- Test the same physics behaviors as the originals
- Use the same tolerances and frame counts
- Pass with the same success rate (no regressions)
- Delete the old `createSpherePoints()` helper from converted files

---

## Acceptance Criteria

- [ ] AC1: All 6 LinearCollisionTest tests pass using ReplayEnabledTest fixture
- [ ] AC2: All 4 EnergyAccountingTest tests pass using ReplayEnabledTest fixture
- [ ] AC3: Each test produces a `.db` recording in `replay/recordings/`
- [ ] AC4: No `createSpherePoints()` helper remains in converted files
- [ ] AC5: Zero test regressions in the full test suite
- [ ] AC6: Recordings are viewable in the replay viewer (spot-check 2-3)

---

## Technical Notes

### Gravity Isolation

Linear collision tests disable gravity to isolate momentum/energy conservation. Use `disableGravity()` from 0062a in `SetUp` or at the start of each test.

### Mass Configuration

`LinearCollision_AsymmetricMass` uses different masses for the two spheres. If `AssetInertial` mass is set at construction and can't be changed post-spawn, this test may need an Engine-level mass override or a specialized spawn method from 0062a.

### EnergyTracker Access

EnergyAccountingTest uses `EnergyTracker` to compute system energy. The fixture provides `engine()` and `world()` — verify that `EnergyTracker` is accessible through the WorldModel or can be constructed from the world state.

---

## Workflow Log

| Phase | Date | Agent | Notes |
|-------|------|-------|-------|
| Draft | 2026-02-13 | Human + Claude | Initial ticket creation |
| Draft → Ready for Implementation | 2026-02-13 | Orchestrator | No math/design phase needed for test conversion; dependency 0062a complete |
