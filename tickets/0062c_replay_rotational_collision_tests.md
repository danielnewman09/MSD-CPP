# Ticket 0062c: Convert Rotational Collision + Energy + Damping Tests

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Feature / Testing
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-13
**Generate Tutorial**: No
**Parent Ticket**: [0062_replay_collision_test_suite](0062_replay_collision_test_suite.md)
**Depends On**: [0062a_extend_test_asset_generator](0062a_extend_test_asset_generator.md)

---

## Overview

Convert `RotationalCollisionTest.cpp` (5 tests), `RotationalEnergyTest.cpp` (4 tests), and `RotationDampingTest.cpp` (2 tests) to use the `ReplayEnabledTest` fixture. These tests use cubes (not spheres) to generate rotational coupling from off-center impacts, making them especially valuable for replay visualization — angular momentum transfer and damping behavior are difficult to diagnose from numerical assertions alone.

---

## Requirements

### R1: Convert RotationalCollisionTest.cpp (535 LOC, 5 tests)

Tests to convert:
- `RotationalCollision_OffCenterImpact_InducesRotation`
- `RotationalCollision_AngularMomentumConserved`
- `RotationalCollision_CubeEdgeHit_SpinsCorrectly`
- `RotationalCollision_GlancingBlow_RotationOnly`
- `RotationalCollision_SymmetricImpact_NoNetRotation`

These use cube geometry (not spheres) to ensure off-center contacts generate torque. Use `unit_cube` or `large_cube` assets from the test database.

### R2: Convert RotationalEnergyTest.cpp (329 LOC, 4 tests)

Tests to convert:
- `RotationalEnergy_OffCenterHit_TranslationalToRotational`
- `RotationalEnergy_SpinningObject_EnergyConserved`
- `RotationalEnergy_CollisionDissipation_InelasticRotation`
- `RotationalEnergy_TotalEnergy_TranslationalPlusRotational`

These verify energy partitioning between translational and rotational modes.

### R3: Convert RotationDampingTest.cpp (284 LOC, 2 tests)

Tests to convert:
- `RotationDamping_OscillatingCube_DampsToRest`
- `RotationDamping_SpinningOnFloor_SettlesEventually`

These are long-running simulations (500-1000 frames) verifying that oscillating/spinning objects eventually settle. Extremely valuable for replay — settling behavior is a classic visual debugging target.

### R4: Preserve Test Semantics

Same constraints as 0062b — preserve frame counts, tolerances, pass rates. Use cubes (not spheres) since these tests specifically need rotational coupling from non-spherical geometry.

---

## Acceptance Criteria

- [ ] AC1: All 5 RotationalCollisionTest tests pass using ReplayEnabledTest fixture
- [ ] AC2: All 4 RotationalEnergyTest tests pass using ReplayEnabledTest fixture
- [ ] AC3: All 2 RotationDampingTest tests pass using ReplayEnabledTest fixture
- [ ] AC4: Each test produces a `.db` recording in `replay/recordings/`
- [ ] AC5: No `createCubePoints()` or `createSpherePoints()` helpers remain in converted files
- [ ] AC6: Zero test regressions in the full test suite

---

## Technical Notes

### Cube Geometry for Rotation

These tests intentionally use cubes, not spheres. Off-center impacts on flat faces generate torque, while spheres (being symmetric) do not. The existing `unit_cube` asset should work for most tests.

### Initial Angular Velocity

Some tests set initial angular velocity on objects. The fixture's spawn helpers should support this, or tests can access `engine().getWorldModel().getObject(id).getInertialState().angularVelocity` post-spawn.

### Long Simulation Durations

`RotationDampingTest` runs 500-1000 frames. These produce larger recordings but are the most valuable for replay visualization — you can see the object wobbling and settling in the viewer.

---

## Workflow Log

| Phase | Date | Agent | Notes |
|-------|------|-------|-------|
| Draft | 2026-02-13 | Human + Claude | Initial ticket creation |
