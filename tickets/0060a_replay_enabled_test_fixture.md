# Ticket 0060a: ReplayEnabledTest Fixture

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
**Parent Ticket**: [0060_replay_integrated_gtest](0060_replay_integrated_gtest.md)
**Depends On**: None
**Note**: Recording-based query/assertion functionality is handled by the Python layer (0060b, 0060c). This ticket covers the C++ fixture that produces `.db` files only.

---

## Overview

Create a GTest fixture base class that manages the full lifecycle of a replay-enabled simulation test: asset database creation, Engine initialization, recording enablement, and cleanup. Tests inheriting from this fixture get real asset-registry-backed geometry and produce self-contained SQLite recordings viewable in the browser.

---

## Requirements

### R1: Asset Database Creation

The fixture's `SetUp()` creates a SQLite database populated with standard test primitives:

| Asset Name | Geometry | Purpose |
|------------|----------|---------|
| `"unit_cube"` | 1x1x1 cube | Standard dynamic object |
| `"large_cube"` | 2x2x2 cube | Larger dynamic/environment object |
| `"floor_slab"` | 100x100x100 cube | Floor/wall environment objects |

Uses `GeometryFactory::createCube()` to produce `MeshRecord` and writes `ObjectRecord` entries via cpp_sqlite DAOs. This follows the proven pattern from `EngineIntegrationTest::createTestAssets()`.

### R2: Engine Lifecycle

After creating the asset database:
1. Create `Engine{dbPath}` — reads geometry via `AssetRegistry` (read-only connection)
2. Call `engine_.getWorldModel().enableRecording(dbPath)` — `DataRecorder` opens same DB read-write, adds state tables

On `TearDown()`:
1. Destroy Engine — RAII triggers `DataRecorder` flush and thread join
2. If `MSD_KEEP_RECORDINGS` env var is `"0"`, delete the `.db` file

### R3: Recording Output Path

Recordings are written to `{MSD_RECORDINGS_DIR}/{TestSuite}_{TestName}.db` where `MSD_RECORDINGS_DIR` is a CMake compile-time define:

```cmake
target_compile_definitions(msd_sim_test PRIVATE
    MSD_RECORDINGS_DIR="${CMAKE_SOURCE_DIR}/replay/recordings"
)
```

The test suite and test name are obtained from `::testing::UnitTest::GetInstance()->current_test_info()`.

### R4: Spawn Helpers

Convenience methods that delegate to `Engine::spawnInertialObject()` and `Engine::spawnEnvironmentObject()`:

```cpp
const AssetInertial& spawnCube(const std::string& assetName, const Coordinate& position);
const AssetEnvironment& spawnEnvironment(const std::string& assetName, const Coordinate& position);
```

### R5: Simulation Stepping

```cpp
void step(int frames = 1, std::chrono::milliseconds dt = std::chrono::milliseconds{16});
```

Tracks cumulative simulation time internally. Each call advances `currentTime_ += dt` per frame and calls `engine_->update(currentTime_)`.

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `msd/msd-sim/test/Replay/ReplayEnabledTest.hpp` | Fixture header |
| `msd/msd-sim/test/Replay/ReplayEnabledTest.cpp` | Fixture implementation |

### Modified Files
| File | Change |
|------|--------|
| `msd/msd-sim/CMakeLists.txt` | Add `test/Replay/ReplayEnabledTest.cpp` to test sources; add `MSD_RECORDINGS_DIR` compile definition |

---

## Key Reusable Components

| Component | Location | Reused For |
|-----------|----------|------------|
| `GeometryFactory::createCube()` | `msd/msd-assets/src/GeometryFactory.hpp:43` | Creating MeshRecords for test geometry |
| `EngineIntegrationTest::createTestAssets()` | `msd/msd-sim/test/EngineIntegrationTest.cpp:89` | Pattern for writing geometry to test DB via DAOs |
| `Engine` | `msd/msd-sim/src/Engine.hpp` | Orchestrates AssetRegistry + WorldModel |
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
  void createAssetDatabase();

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

1. [ ] **AC1**: Fixture creates SQLite database with MeshRecord/ObjectRecord for standard primitives
2. [ ] **AC2**: Engine successfully reads geometry from the same database
3. [ ] **AC3**: DataRecorder successfully writes state tables to the same database
4. [ ] **AC4**: Recording output path follows `{MSD_RECORDINGS_DIR}/{Suite}_{Test}.db` convention
5. [ ] **AC5**: `MSD_KEEP_RECORDINGS=0` removes recording on TearDown; default preserves it
6. [ ] **AC6**: `step()` advances simulation and produces recorded frames
7. [ ] **AC7**: Existing tests compile and pass without modification

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
