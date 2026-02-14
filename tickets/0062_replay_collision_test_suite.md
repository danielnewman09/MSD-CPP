# Ticket 0062: Replay-Enabled Collision Test Suite

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
**Related Tickets**: [0060_replay_integrated_gtest](0060_replay_integrated_gtest.md), [0039b_linear_collision_test_suite](0039b_linear_collision_test_suite.md)

### Subtasks

| Subtask | Description | Priority | Dependencies | Status |
|---------|-------------|----------|--------------|--------|
| 0062a | Extend test asset generator with spheres and parameterized spawning | High | None | PENDING |
| 0062b | Convert LinearCollisionTest + EnergyAccountingTest | High | 0062a | PENDING |
| 0062c | Convert RotationalCollisionTest + RotationalEnergyTest + RotationDampingTest | High | 0062a | PENDING |
| 0062d | Convert ContactManifoldStabilityTest + EdgeContactTest | Medium | 0062a | PENDING |
| 0062e | Convert ParameterIsolationTest + diagnostic tests | Medium | 0062a | PENDING |
| 0062f | Python recording analysis for collision test recordings | Low | 0062b, 0062c | PENDING |

---

## Summary

Convert the existing collision unit test suite (~5,500 LOC across 13 files) from direct `WorldModel` construction with programmatic geometry to `ReplayEnabledTest`-based tests that produce viewable recordings. This enables visual debugging of collision scenarios through the replay viewer and post-hoc analysis via Python.

### Current State

The collision tests currently:
- Construct `ConvexHull` objects from programmatic point clouds (`createSpherePoints()`, `createCubePoints()`)
- Create `AssetInertial` and `AssetEnvironment` objects directly via `WorldModel`
- Run multi-frame simulations (50-1000 frames) checking in-memory assertions
- Produce no persistent output — failures require re-running with print statements to debug

### Target State

After conversion:
- Tests inherit from `ReplayEnabledTest` and use `Engine` with database-backed assets
- Every test run produces a `.db` recording viewable in the browser replay viewer
- Python recording analysis provides post-hoc physics invariant checking
- Complex collision behaviors (settling, energy drift, manifold stability) are visually inspectable

### Conversion Challenges

1. **Geometry**: Current tests use icosphere point clouds (~162 vertices) for rotation-free linear collisions. The test asset generator needs sphere primitives.
2. **Parameters**: Tests set specific mass, restitution, and friction per-object after construction. The fixture's `spawnCube()` uses defaults — tests must configure objects post-spawn.
3. **Gravity**: Some tests disable gravity for isolated collision testing. The fixture always enables gravity — tests may need to spawn objects high enough that gravity is negligible over the test duration, or disable gravity on the WorldModel.

### Files to Convert (by priority)

**Skip** (algorithmic, no time-stepping — keep as-is):
- `GJKTest.cpp`, `EPATest.cpp`, `CollisionHandlerTest.cpp`, `TangentBasisTest.cpp`

**Tier 1** (highest replay value — momentum conservation, energy tracking):
- `LinearCollisionTest.cpp` (473 LOC, 6 tests)
- `EnergyAccountingTest.cpp` (376 LOC, 4 tests)

**Tier 2** (rotational dynamics — angular momentum, damping):
- `RotationalCollisionTest.cpp` (535 LOC, 5 tests)
- `RotationalEnergyTest.cpp` (329 LOC, 4 tests)
- `RotationDampingTest.cpp` (284 LOC, 2 tests)

**Tier 3** (contact stability and edge cases):
- `ContactManifoldStabilityTest.cpp` (285 LOC, 2 tests)
- `EdgeContactTest.cpp` (433 LOC, 11 tests)

**Tier 4** (diagnostics and parameter sweeps):
- `ParameterIsolationTest.cpp` (989 LOC, 10+ tests)
- `EPAConvergenceDiagnosticTest.cpp` (632 LOC, 7 tests)
- `ManifoldDiagnosticTest.cpp` (453 LOC, 5 tests)
- `PerContactDepthTest.cpp` (380 LOC, 6 tests)
- `CollisionPipelineTest.cpp` (237 LOC, 7 tests)

---

## Success Criteria

1. All converted tests pass with no regressions
2. Each test produces a `.db` recording viewable in the replay viewer
3. Recordings capture collision interactions frame-by-frame
4. `MSD_KEEP_RECORDINGS=0` still cleans up after test runs
5. No remaining `createSpherePoints()` / `createCubePoints()` helpers in converted tests
