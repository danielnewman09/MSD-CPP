# Ticket 0060d: Example Replay-Enabled Tests

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
**Parent Ticket**: [0060_replay_integrated_gtest](0060_replay_integrated_gtest.md)
**Depends On**: [0060a_replay_enabled_test_fixture](0060a_replay_enabled_test_fixture.md), [0060b_recording_query_api](0060b_recording_query_api.md), [0060c_replay_matchers](0060c_replay_matchers.md)

---

## Overview

Create 2-3 representative simulation tests that demonstrate the full replay-enabled test pattern. These tests:
- Inherit from `ReplayEnabledTest` (use real asset database, no `createCubePoints()`)
- Run multi-frame simulations with recording enabled
- Use both traditional in-memory assertions AND recording-based assertions via `RecordingQuery` and `ReplayMatchers`
- Produce `.db` files in `replay/recordings/` viewable in the browser

These serve as reference implementations for future test authors migrating from the `createCubePoints()` pattern.

---

## Requirements

### R1: Cube Drop Test

**Scenario**: Single cube dropped from height onto the Engine's default floor.

**Physics tested**: Gravity, resting contact, energy dissipation, settling.

```cpp
class ReplayDropTest : public ReplayEnabledTest {};

TEST_F(ReplayDropTest, CubeDropsAndSettles) {
  const auto& cube = spawnCube("unit_cube", Coordinate{0.0, 0.0, 5.0});
  uint32_t cubeId = cube.getInstanceId();

  step(300);  // ~5 seconds

  // Traditional assertion
  EXPECT_GT(world().getObject(cubeId).getInertialState().position.z(), -60.0);

  // Recording-based assertions
  auto q = query();
  EXPECT_GT(q.frameCount(), 290);
  EXPECT_THAT(recordingPath().string(), NeverPenetratesBelow(cubeId, -11.0));
  EXPECT_THAT(recordingPath().string(), BodyComesToRest(cubeId, 0.5));
}
```

### R2: Two-Body Collision Test

**Scenario**: Two cubes collide head-on, high above the floor (z=50) to isolate collision from floor interaction.

**Physics tested**: Collision detection, impulse response, contact events.

```cpp
class ReplayCollisionTest : public ReplayEnabledTest {};

TEST_F(ReplayCollisionTest, TwoCubesCollide) {
  auto& cubeA = spawnCube("unit_cube", Coordinate{0.0, 0.0, 50.0});
  auto& cubeB = spawnCube("unit_cube", Coordinate{0.9, 0.0, 50.0});

  uint32_t idA = cubeA.getInstanceId();
  uint32_t idB = cubeB.getInstanceId();

  world().getObject(idA).setCoefficientOfRestitution(1.0);
  world().getObject(idB).setCoefficientOfRestitution(1.0);
  world().getObject(idA).getInertialState().velocity = Coordinate{2.0, 0.0, 0.0};
  world().getObject(idB).getInertialState().velocity = Coordinate{-2.0, 0.0, 0.0};

  step(10);

  // Traditional assertion
  double vAxFinal = world().getObject(idA).getInertialState().velocity.x();
  double vBxFinal = world().getObject(idB).getInertialState().velocity.x();
  EXPECT_NE(2.0, vAxFinal);
  EXPECT_NE(-2.0, vBxFinal);

  // Recording-based assertions
  auto q = query();
  EXPECT_GT(q.totalContactFrames(), 0);
  EXPECT_GT(q.contactFramesBetween(idA, idB), 0);
}
```

### R3: Recording Validity Test

**Scenario**: Verify the recording database produced by a replay-enabled test is self-contained and contains all expected record types.

```cpp
TEST_F(ReplayDropTest, RecordingContainsAllRecordTypes) {
  spawnCube("unit_cube", Coordinate{0.0, 0.0, 5.0});
  step(50);

  auto q = query();

  // Geometry present (from fixture setup)
  // Verified by opening DB and checking MeshRecord table
  cpp_sqlite::Database db{recordingPath().string(), false};
  auto& meshDAO = db.getDAO<msd_transfer::MeshRecord>();
  EXPECT_GT(meshDAO.selectAll().size(), 0);

  // State present (from recording)
  EXPECT_GT(q.frameCount(), 0);

  // Verify recording is viewable (has both geometry and state tables)
  auto& frameDAO = db.getDAO<msd_transfer::SimulationFrameRecord>();
  EXPECT_GT(frameDAO.selectAll().size(), 0);
}
```

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `msd/msd-sim/test/Replay/DropTest.cpp` | Cube drop test with recording assertions |
| `msd/msd-sim/test/Replay/CollisionTest.cpp` | Two-body collision test with recording assertions |

### Modified Files
| File | Change |
|------|--------|
| `msd/msd-sim/CMakeLists.txt` | Add `test/Replay/DropTest.cpp` and `test/Replay/CollisionTest.cpp` to test sources |

---

## Test Plan

### Automated Tests

All tests in R1-R3 above serve as both the deliverable and the test plan. Additionally:

```cpp
// Verify tests produce output files
TEST_F(ReplayDropTest, Recording_FileExists)
// After step(), verify recordingPath() points to an existing .db file

// Verify recordings are isolated (one per test)
// Run both DropTest and CollisionTest, verify two separate .db files exist
```

### Manual Verification

1. Build and run: `./build/Debug/debug/msd_sim_test --gtest_filter="Replay*"`
2. Check output: `ls replay/recordings/Replay*.db`
3. Start viewer: `cd replay && uvicorn replay.app:app --reload`
4. Open http://localhost:8000 — verify recordings appear in simulation list
5. Select a recording — verify geometry renders and frames play back

---

## Acceptance Criteria

1. [ ] **AC1**: `ReplayDropTest::CubeDropsAndSettles` passes with both traditional and recording assertions
2. [ ] **AC2**: `ReplayCollisionTest::TwoCubesCollide` passes and detects contact events
3. [ ] **AC3**: Recording databases contain both geometry (MeshRecord) and state (SimulationFrameRecord) tables
4. [ ] **AC4**: Recording files appear in `replay/recordings/` with correct naming
5. [ ] **AC5**: Recordings load in the FastAPI viewer without modification
6. [ ] **AC6**: Existing tests unaffected (`--gtest_filter="-Replay*"` all pass)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
