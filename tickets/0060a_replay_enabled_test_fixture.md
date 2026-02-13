# Ticket 0060a: ReplayEnabledTest Fixture

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Review
**Type**: Feature / Testing
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-13
**Generate Tutorial**: No
**Parent Ticket**: [0060_replay_integrated_gtest](0060_replay_integrated_gtest.md)
**Depends On**: None
**Note**: Recording-based query/assertion functionality is handled by the Python layer (0060b, 0060c). This ticket covers the C++ fixture that produces `.db` files only.

---

## Overview

Create a GTest fixture base class and a build-time asset generation executable that together manage the full lifecycle of a replay-enabled simulation test. A standalone `generate_test_assets` executable creates a shared test asset database at build time. The `ReplayEnabledTest` fixture copies this pre-built database per-test, initializes Engine, enables recording, and cleans up. Tests inheriting from this fixture get real asset-registry-backed geometry and produce self-contained SQLite recordings viewable in the browser.

---

## Requirements

### R1: Test Asset Generator Executable

A standalone C++ executable that creates a SQLite database populated with standard test primitives. Runs once at build time via CMake `add_custom_command`.

| Asset Name | Factory Call | Purpose |
|------------|-------------|---------|
| `"unit_cube"` | `GeometryFactory::createCube(1.0)` | Standard 1x1x1 dynamic object |
| `"large_cube"` | `GeometryFactory::createCube(2.0)` | Larger dynamic/environment object |
| `"floor_slab"` | `GeometryFactory::createCube(100.0)` | Floor/wall environment objects |

Each asset gets both a visual MeshRecord and a collision MeshRecord, plus an ObjectRecord with FK references to both. Follows the proven pattern from `msd/msd-asset-gen/src/generate_assets.cpp`.

Usage: `generate_test_assets <output.db>`

Links against `msd_assets` only (lightweight — no `msd_sim` dependency).

### R2: CMake Build-Time Integration

The executable runs as a custom command during build:

```cmake
# In replay/tools/CMakeLists.txt
add_executable(generate_test_assets generate_test_assets.cpp)
target_link_libraries(generate_test_assets PRIVATE msd_assets)

set(TEST_ASSETS_DB "${CMAKE_BINARY_DIR}/test_assets.db")
add_custom_command(
  OUTPUT "${TEST_ASSETS_DB}"
  COMMAND generate_test_assets "${TEST_ASSETS_DB}"
  DEPENDS generate_test_assets
  COMMENT "Generating test asset database"
  VERBATIM
)
add_custom_target(test_assets_db ALL DEPENDS "${TEST_ASSETS_DB}")
```

The test executable depends on this target and receives paths via compile definitions:

```cmake
# In msd/msd-sim/test/CMakeLists.txt
target_compile_definitions(${MSD_SIM_TEST_NAME} PRIVATE
  MSD_TEST_ASSETS_DB="${CMAKE_BINARY_DIR}/test_assets.db"
  MSD_RECORDINGS_DIR="${CMAKE_SOURCE_DIR}/replay/recordings"
)
add_dependencies(${MSD_SIM_TEST_NAME} test_assets_db)
```

### R3: Fixture SetUp — Copy and Initialize

The fixture's `SetUp()` does NOT create assets from scratch. Instead:

1. Get test name from `::testing::UnitTest::GetInstance()->current_test_info()`
2. Build per-test path: `{MSD_RECORDINGS_DIR}/{TestSuite}_{TestName}.db`
3. Create recordings directory: `fs::create_directories(MSD_RECORDINGS_DIR)`
4. Copy pre-built asset DB: `fs::copy_file(MSD_TEST_ASSETS_DB, dbPath_, overwrite_existing)`
5. Create `Engine{dbPath_}` — reads geometry via `AssetRegistry` (read-only)
6. Call `engine_->getWorldModel().enableRecording(dbPath_)` — `DataRecorder` opens same DB read-write, adds state tables via `CREATE TABLE IF NOT EXISTS`

This preserves the single-database design: each test gets a self-contained `.db` with both geometry tables (from the copy) and state tables (from recording).

### R4: Fixture TearDown

1. Destroy Engine — RAII triggers `DataRecorder` flush and thread join
2. If `MSD_KEEP_RECORDINGS` env var is `"0"`, delete the `.db` file; otherwise preserve it for the replay viewer

### R5: Spawn Helpers

Convenience methods that delegate to `Engine::spawnInertialObject()` and `Engine::spawnEnvironmentObject()`:

```cpp
const AssetInertial& spawnCube(const std::string& assetName, const Coordinate& position);
const AssetEnvironment& spawnEnvironment(const std::string& assetName, const Coordinate& position);
```

These pass a default `AngularCoordinate{}` for orientation.

### R6: Simulation Stepping

```cpp
void step(int frames = 1, std::chrono::milliseconds dt = std::chrono::milliseconds{16});
```

Tracks cumulative simulation time internally. Each call advances `currentTime_ += dt` per frame and calls `engine_->update(currentTime_)`.

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `replay/tools/generate_test_assets.cpp` | Standalone executable that creates test asset database |
| `msd/msd-sim/test/Replay/ReplayEnabledTest.hpp` | Fixture header |
| `msd/msd-sim/test/Replay/ReplayEnabledTest.cpp` | Fixture implementation |
| `msd/msd-sim/test/Replay/CMakeLists.txt` | Add Replay sources to msd_sim_test |

### Modified Files
| File | Change |
|------|--------|
| `replay/tools/CMakeLists.txt` | Add `generate_test_assets` target + `add_custom_command` + `add_custom_target(test_assets_db)` |
| `msd/msd-sim/test/CMakeLists.txt` | Add `add_subdirectory(Replay)`, `MSD_TEST_ASSETS_DB` and `MSD_RECORDINGS_DIR` compile definitions, `add_dependencies` on `test_assets_db` |

---

## Key Reusable Components

| Component | Location | Reused For |
|-----------|----------|------------|
| `generate_assets.cpp` | `msd/msd-asset-gen/src/generate_assets.cpp` | Pattern for `generate_test_assets.cpp` (GeometryFactory + DAO insertion) |
| `GeometryFactory::createCube()` | `msd/msd-assets/src/GeometryFactory.hpp` | Creating MeshRecords for test geometry |
| `Engine` | `msd/msd-sim/src/Engine.hpp` | Orchestrates AssetRegistry + WorldModel |
| `Engine::spawnInertialObject()` | `msd/msd-sim/src/Engine.cpp:46` | Spawn helpers delegate here |
| `Engine::spawnEnvironmentObject()` | `msd/msd-sim/src/Engine.cpp:77` | Spawn helpers delegate here |
| `WorldModel::enableRecording()` | `msd/msd-sim/src/Environment/WorldModel.cpp:283` | Enables DataRecorder on existing DB |

---

## Interface

```cpp
class ReplayEnabledTest : public ::testing::Test {
protected:
  void SetUp() override;
  void TearDown() override;

  const AssetInertial& spawnCube(const std::string& assetName, const Coordinate& position);
  const AssetEnvironment& spawnEnvironment(const std::string& assetName, const Coordinate& position);

  void step(int frames = 1, std::chrono::milliseconds dt = std::chrono::milliseconds{16});

  Engine& engine();
  WorldModel& world();
  std::filesystem::path recordingPath() const;

private:
  std::filesystem::path dbPath_;
  std::unique_ptr<Engine> engine_;
  std::chrono::milliseconds currentTime_{0};
};
```

---

## Test Plan

### Unit Tests

```cpp
TEST_F(ReplayEnabledTest, SetUp_CreatesDatabase)
// Verify .db file exists at expected path after SetUp

TEST_F(ReplayEnabledTest, SetUp_DatabaseContainsGeometry)
// Open .db independently, verify MeshRecord and ObjectRecord tables populated

TEST_F(ReplayEnabledTest, SpawnCube_CreatesInertialAsset)
// Verify object added to WorldModel

TEST_F(ReplayEnabledTest, Step_AdvancesSimulationTime)
// Verify world().getTime() increases

TEST_F(ReplayEnabledTest, TearDown_ProducesRecordingWithFrames)
// Run a few steps, verify recording DB contains SimulationFrameRecords
```

---

## Acceptance Criteria

1. [ ] **AC1**: `generate_test_assets` executable creates SQLite database with MeshRecord/ObjectRecord for unit_cube, large_cube, floor_slab
2. [ ] **AC2**: CMake runs `generate_test_assets` at build time, producing `build/{type}/test_assets.db`
3. [ ] **AC3**: Fixture copies pre-built asset DB to per-test recording path
4. [ ] **AC4**: Engine successfully reads geometry from the copied database
5. [ ] **AC5**: DataRecorder successfully writes state tables to the same database
6. [ ] **AC6**: Recording output path follows `{MSD_RECORDINGS_DIR}/{Suite}_{Test}.db` convention
7. [ ] **AC7**: `MSD_KEEP_RECORDINGS=0` removes recording on TearDown; default preserves it
8. [ ] **AC8**: `step()` advances simulation and produces recorded frames
9. [ ] **AC9**: Existing tests compile and pass without modification

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-02-13 14:00
- **Completed**: 2026-02-13 14:30
- **Branch**: 0060a-replay-enabled-test-fixture
- **PR**: #54
- **Artifacts**:
  - `replay/tools/generate_test_assets.cpp` (108 LOC)
  - `msd/msd-sim/test/Replay/ReplayEnabledTest.hpp` (118 LOC)
  - `msd/msd-sim/test/Replay/ReplayEnabledTest.cpp` (84 LOC)
  - `msd/msd-sim/test/Replay/ReplayEnabledTestTests.cpp` (122 LOC)
  - `msd/msd-sim/test/Replay/CMakeLists.txt`
  - Modified `replay/tools/CMakeLists.txt`, `msd/msd-sim/test/CMakeLists.txt`
  - `docs/designs/0060a_replay_enabled_test_fixture/iteration-log.md`
- **Notes**: Single-iteration implementation. All 6 new tests pass (100% success rate). Pre-built asset database pattern works perfectly. Self-contained recordings produced. Zero regressions (793/797 total, +6 new tests).

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
