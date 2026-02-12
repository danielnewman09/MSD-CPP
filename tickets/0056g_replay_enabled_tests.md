# Ticket 0056g: Replay-Enabled Test Pattern & Asset-Gen Extensions

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Feature / Testing
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-11
**Generate Tutorial**: No
**Parent Ticket**: [0056_browser_simulation_replay](0056_browser_simulation_replay.md)
**Depends On**: [0056b_collision_pipeline_data_extraction](0056b_collision_pipeline_data_extraction.md)

---

## Overview

Establish a test pattern where unit tests produce self-contained simulation recordings that can be replayed in the browser viewer. This involves:

1. Using `msd-asset-gen` / `GeometryFactory` to create test asset databases (rather than `createCubePoints()` helpers)
2. Loading geometry via `AssetRegistry` so the replay viewer can load the same shapes
3. Enabling recording during test execution to produce viewable recordings

**Existing tests remain unchanged.** The `createCubePoints()` pattern continues to work. This ticket adds a *new* pattern for tests that want replay capability.

---

## Requirements

### R1: Replay-Enabled Test Base Class

Create a GTest fixture providing common setup for replay-enabled tests:

```cpp
class ReplayEnabledTest : public ::testing::Test
{
protected:
  void SetUp() override;
  void TearDown() override;

  // Create asset database with standard primitives
  void createAssetDatabase();

  // Load a ConvexHull from the asset database by name
  ConvexHull& loadHull(const std::string& assetName);

  // Access the world model (with recording already enabled)
  WorldModel& world();

  // Path to the recording database (for verification or replay)
  std::filesystem::path recordingPath() const;

private:
  std::filesystem::path assetDbPath_;
  std::filesystem::path recordingDbPath_;
  std::unique_ptr<msd_assets::AssetRegistry> registry_;
  std::map<std::string, std::unique_ptr<ConvexHull>> hulls_;
  WorldModel worldModel_;
};
```

### R2: Asset Database Creation in Tests

The `createAssetDatabase()` method uses `GeometryFactory` to populate a test asset database:

- Creates temporary SQLite database
- Inserts standard primitives: cube (1.0), cube (100.0 for floor), pyramid
- Each primitive has both visual and collision MeshRecord + ObjectRecord
- `BodyMetadataRecord` links body_id to asset_id for replay

### R3: Extend msd-asset-gen with Additional Primitives

Add common test primitives to `GeometryFactory` and `generate_assets`:

- **Icosphere** — used in sphere collision tests (B-series)
- **Rectangular prism** — for asymmetric collision tests

### R4: Representative Replay-Enabled Tests

Port 2-3 representative collision tests to the new pattern:

1. **Simple drop test** — cube dropped onto floor (demonstrates gravity + resting contact)
2. **Two-body collision** — cube-on-cube impact (demonstrates collision forces + contact points)
3. **Resting stack** — two cubes stacked (demonstrates constraint solver convergence)

These tests should:
- Use `loadHull()` instead of `createCubePoints()`
- Enable recording via the fixture
- Produce `.db` files that the replay viewer can load

### R5: Recording Output Convention

Tests write recording databases to a configurable output directory:
- Default: `build/Debug/test_recordings/`
- Naming: `{TestSuiteName}_{TestName}.db`
- Cleaned up on `TearDown()` unless `MSD_KEEP_RECORDINGS=1` environment variable set

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `msd-sim/test/Replay/ReplayEnabledTest.hpp` | Base test fixture |
| `msd-sim/test/Replay/ReplayEnabledTest.cpp` | Fixture implementation |
| `msd-sim/test/Replay/DropTest.cpp` | Simple drop test (replay-enabled) |
| `msd-sim/test/Replay/CollisionTest.cpp` | Two-body collision test (replay-enabled) |
| `msd-sim/test/Replay/StackTest.cpp` | Resting stack test (replay-enabled) |

### Modified Files
| File | Change |
|------|--------|
| `msd-assets/src/GeometryFactory.hpp` | Add `createIcosphere()`, `createRectangularPrism()` |
| `msd-assets/src/GeometryFactory.cpp` | Implement new primitives |
| `msd-asset-gen/src/generate_assets.cpp` | Include new primitives in generated DB |
| `msd-sim/CMakeLists.txt` | Add Replay test directory |

---

## Test Plan

### Unit Tests

```cpp
// Verify fixture setup
TEST_F(ReplayEnabledTest, SetUp_CreatesAssetDatabase)
TEST_F(ReplayEnabledTest, SetUp_EnablesRecording)
TEST_F(ReplayEnabledTest, LoadHull_ReturnsCubeGeometry)

// Verify new geometry primitives
TEST(GeometryFactory, CreateIcosphere_ProducesValidMeshRecord)
TEST(GeometryFactory, CreateRectangularPrism_ProducesValidMeshRecord)
```

### Integration Tests

```cpp
// Verify replay-enabled tests produce valid recordings
TEST_F(ReplayDropTest, Recording_ContainsFrames)
TEST_F(ReplayDropTest, Recording_ContainsContactRecords)
TEST_F(ReplayDropTest, Recording_ContainsBodyMetadata)
```

---

## Acceptance Criteria

1. [ ] **AC1**: `ReplayEnabledTest` fixture creates asset database and enables recording
2. [ ] **AC2**: `loadHull()` returns valid ConvexHull from asset database
3. [ ] **AC3**: Icosphere and rectangular prism added to GeometryFactory
4. [ ] **AC4**: At least 2 replay-enabled tests produce loadable recording databases
5. [ ] **AC5**: Recording databases contain all record types (state, energy, contacts, forces, metadata)
6. [ ] **AC6**: Recordings can be loaded by the replay viewer (manual verification)
7. [ ] **AC7**: Existing `createCubePoints()` tests unaffected
8. [ ] **AC8**: `MSD_KEEP_RECORDINGS=1` preserves recording files for replay

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
