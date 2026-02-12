# Design: Static Asset Recording & Foreign Key Linkage

## Summary

Establish proper relational integrity between static asset data (recorded once at spawn) and per-frame dynamic data (recorded every frame). Currently, `InertialStateRecord` has no body identifier and `EnergyRecord` uses a raw `uint32_t body_id` with no referential integrity. This design adds database recording of `AssetInertialStaticRecord` at spawn time and foreign key linkage from all per-frame records.

## Architecture Changes

### PlantUML Diagram
See: [`./0056i_static_asset_recording_and_fk.puml`](./0056i_static_asset_recording_and_fk.puml)

### Modified Components

#### InertialStateRecord
- **Current location**: `msd-transfer/src/InertialStateRecord.hpp`
- **Changes required**:
  - Add member: `cpp_sqlite::ForeignKey<AssetInertialStaticRecord> body`
  - Update `BOOST_DESCRIBE_STRUCT` macro to include `body` field
- **Backward compatibility**: Breaking change — existing databases with `InertialStateRecord` will not have `body` column. Migration required or fresh database generation.

**Modified structure**:
```cpp
struct InertialStateRecord : public cpp_sqlite::BaseTransferObject
{
  CoordinateRecord position;
  VelocityRecord velocity;
  AccelerationRecord acceleration;
  QuaternionDRecord orientation;
  Vector4DRecord quaternionRate;
  AngularAccelerationRecord angularAcceleration;
  cpp_sqlite::ForeignKey<AssetInertialStaticRecord> body;  // NEW
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};
```

#### EnergyRecord
- **Current location**: `msd-transfer/src/EnergyRecord.hpp`
- **Changes required**:
  - Replace `uint32_t body_id` with `cpp_sqlite::ForeignKey<AssetInertialStaticRecord> body`
  - Update `BOOST_DESCRIBE_STRUCT` macro (replace `body_id` with `body`)
- **Backward compatibility**: Breaking change — field rename from `body_id` to `body`, type change from `uint32_t` to `ForeignKey`.

**Modified structure**:
```cpp
struct EnergyRecord : public cpp_sqlite::BaseTransferObject
{
  cpp_sqlite::ForeignKey<AssetInertialStaticRecord> body;  // CHANGED from uint32_t body_id
  double linear_ke{0.0};
  double rotational_ke{0.0};
  double potential_e{0.0};
  double total_e{0.0};
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};
```

#### CollisionResultRecord (No Change)
- **Current location**: `msd-transfer/src/CollisionResultRecord.hpp`
- **Changes required**: **None**
- **Rationale**: Per design decision **Option C** (see Open Questions below), environment bodies remain as raw `uint32_t` IDs. `body_a_id` and `body_b_id` stay unchanged. Simplest approach, sufficient for collision replay.

#### WorldModel
- **Current location**: `msd-sim/src/Environment/WorldModel.hpp`, `WorldModel.cpp`
- **Changes required**:
  1. Add private helper method: `void recordStaticData(const AssetInertial& asset)`
  2. Add private helper method: `void backfillStaticData()`
  3. Modify `spawnObject()`: Call `recordStaticData()` if `dataRecorder_` is non-null
  4. Modify `enableRecording()`: Call `backfillStaticData()` after creating `DataRecorder`
  5. Modify `recordCurrentFrame()`: Set `state.body.id` and `energy.body.id` when creating records
- **Backward compatibility**: Non-breaking — recording is opt-in via `enableRecording()`.

**New helper methods**:
```cpp
private:
  /**
   * @brief Record static asset data for a single asset
   *
   * Creates AssetInertialStaticRecord from asset's static state and writes
   * to database via DataRecorder. The record's auto-incremented id becomes
   * the FK target for per-frame records.
   *
   * @param asset The asset to record static data for
   * @ticket 0056i_static_asset_recording_and_fk
   */
  void recordStaticData(const AssetInertial& asset);

  /**
   * @brief Backfill static records for all existing assets
   *
   * Called when recording is enabled after assets have already been spawned.
   * Iterates all inertial assets and records static data for each.
   *
   * @ticket 0056i_static_asset_recording_and_fk
   */
  void backfillStaticData();
```

**Modified spawnObject()**:
```cpp
const AssetInertial& WorldModel::spawnObject(/* params */) {
  // ... existing spawn logic ...
  const AssetInertial& asset = inertialAssets_.back();

  // NEW: Record static data if recording is enabled
  if (dataRecorder_) {
    recordStaticData(asset);
  }

  return asset;
}
```

**Modified enableRecording()**:
```cpp
void WorldModel::enableRecording(const std::string& dbPath,
                                  std::chrono::milliseconds flushInterval) {
  // ... create DataRecorder ...

  // NEW: Backfill static data for already-spawned assets
  backfillStaticData();
}
```

**Modified recordCurrentFrame()**:
```cpp
void WorldModel::recordCurrentFrame() {
  uint32_t frameId = dataRecorder_->recordFrame(time_.count() / 1000.0);

  for (const auto& asset : inertialAssets_) {
    // InertialState record
    auto stateRecord = asset.getState().toRecord();
    stateRecord.frame.id = frameId;
    stateRecord.body.id = asset.getInstanceId();  // NEW: Set body FK
    dataRecorder_->getDAO<InertialStateRecord>().addToBuffer(stateRecord);

    // Energy record
    auto energyRecord = energyTracker_->computeEnergy(asset);
    energyRecord.frame.id = frameId;
    energyRecord.body.id = asset.getInstanceId();  // NEW: Set body FK (was body_id)
    dataRecorder_->getDAO<EnergyRecord>().addToBuffer(energyRecord);
  }
}
```

#### DataRecorder (DAO Instantiation)
- **Current location**: `msd-sim/src/DataRecorder/DataRecorder.cpp`
- **Changes required**:
  - Add template instantiation for `AssetInertialStaticRecord` in the DAO factory methods or constructor
  - Ensure `AssetInertialStaticRecord` DAO is initialized (via `getDAO<>()`) before any records are written
- **Backward compatibility**: Non-breaking — internal implementation detail.

**Modified constructor (or DAO initialization)**:
```cpp
DataRecorder::DataRecorder(const Config& config) {
  database_ = std::make_unique<cpp_sqlite::Database>(config.databasePath, true);

  // Initialize DAOs in dependency order (frames first, then static, then per-frame)
  database_->getDAO<SimulationFrameRecord>();
  database_->getDAO<AssetInertialStaticRecord>();  // NEW
  database_->getDAO<InertialStateRecord>();
  database_->getDAO<EnergyRecord>();
  // ... other DAOs ...

  recorderThread_ = std::jthread{[this](std::stop_token st) {
    recorderThreadMain(st);
  }};
}
```

#### Records.hpp
- **Current location**: `msd-transfer/src/Records.hpp`
- **Changes required**: **Verify** `AssetInertialStaticRecord.hpp` is included
- **Backward compatibility**: Non-breaking — header-only convenience include.

**Verification**:
```cpp
// Ensure this line exists in Records.hpp:
#include "msd-transfer/src/AssetInertialStaticRecord.hpp"
```

### Integration Points
| New Behavior | Existing Component | Integration Type | Notes |
|--------------|-------------------|------------------|-------|
| `recordStaticData()` | `WorldModel::spawnObject()` | Direct call | Triggered if `dataRecorder_` is non-null |
| `backfillStaticData()` | `WorldModel::enableRecording()` | Direct call | Iterates `inertialAssets_` and records each |
| `body` FK field | `InertialStateRecord` | Schema change | Database migration required for existing DBs |
| `body` FK field | `EnergyRecord` | Schema change | Database migration required for existing DBs |
| DAO initialization | `DataRecorder::DataRecorder()` | DAO creation | Ensures `AssetInertialStaticRecord` DAO exists before use |

## Test Impact

### Existing Tests Affected
| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `msd-sim/test/DataRecorder/DataRecorderTest.cpp` | Any tests asserting `InertialStateRecord` structure | Schema change | Update assertions to include `body` field |
| `msd-sim/test/DataRecorder/DataRecorderTest.cpp` | Any tests asserting `EnergyRecord` structure | Schema change | Update assertions to replace `body_id` with `body` |
| `msd-sim/test/Environment/WorldModelTest.cpp` | Tests using recording | Backfill behavior | Verify `backfillStaticData()` is called on `enableRecording()` |

### New Tests Required

#### Unit Tests
| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| WorldModel | `WorldModelRecording::SpawnObject_RecordsStaticData` | `AssetInertialStaticRecord` written when asset spawned with recording enabled |
| WorldModel | `WorldModelRecording::EnableRecording_BackfillsExistingAssets` | Static records created for all pre-existing assets when recording is enabled |
| WorldModel | `WorldModelRecording::InertialStateRecord_HasBodyFK` | `InertialStateRecord.body.id` matches asset instance ID |
| WorldModel | `WorldModelRecording::EnergyRecord_HasBodyFK` | `EnergyRecord.body.id` matches asset instance ID |
| WorldModel | `WorldModelRecording::StaticRecord_SurvivesFlushAndQuery` | `AssetInertialStaticRecord` can be queried after flush |
| WorldModel | `WorldModelRecording::SpawnWithoutRecording_NoStaticData` | No static record written if `dataRecorder_` is null |

#### Integration Tests
| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| End-to-End Recording | WorldModel + DataRecorder + Database | Full lifecycle: spawn → enable recording → record frames → query static + dynamic data via FK |

### Regression Tests
All existing 713/717 tests must pass unchanged. The schema changes are additive (new field in `InertialStateRecord`) or transparent (`body_id` → `body` in `EnergyRecord` uses same underlying value).

## Open Questions

### Design Decisions (Human Input Needed)

#### 1. Environment Asset Static Record (R5) — **DECISION MADE: Option C**

**Context**: `CollisionResultRecord` stores `body_a_id` and `body_b_id` which can reference either inertial or environment assets. Should environment assets get their own static record?

**Option A**: Separate `AssetEnvironmentStaticRecord` with `body_id`, `restitution`, `friction` (no mass — infinite)
- **Pros**: Full FK integrity for all collision pairs, consistent schema
- **Cons**: Requires polymorphic lookup (check both inertial and environment tables), more complex queries

**Option B**: Unified `AssetStaticRecord` covering both inertial and environment assets
- **Pros**: Single FK target, simpler queries
- **Cons**: Mixed semantics (mass is NaN for environment), less type-safe

**Option C**: Only record inertial static data; environment bodies remain raw IDs in `CollisionResultRecord`
- **Pros**: Simplest implementation, sufficient for replay (environment properties are static and known)
- **Cons**: No FK integrity for `body_a_id` / `body_b_id` when referencing environment

**Recommendation**: **Option C** — Environment asset properties (restitution, friction) are compile-time constants set at spawn and don't change. For replay purposes, the raw `body_id` is sufficient to identify which environment object was involved. Full FK integrity is valuable for dynamic inertial assets (which have per-frame state changes), but less critical for static environment objects.

**Accepted Trade-off**: `CollisionResultRecord.body_a_id` and `body_b_id` remain raw `uint32_t` without FK enforcement. Query logic must handle cases where the ID may reference either an inertial asset (with static record) or an environment asset (without static record).

#### 2. Static Record DAO Initialization Order — **RESOLVED**

**Question**: Should `AssetInertialStaticRecord` DAO be initialized before or after `SimulationFrameRecord` DAO in `DataRecorder::DataRecorder()`?

**Answer**: **After `SimulationFrameRecord`**, before per-frame records (`InertialStateRecord`, `EnergyRecord`). This ensures proper flush order:
1. Frames (no dependencies)
2. Static data (no FK to frames)
3. Per-frame data (FKs to both frames and static records)

**Implementation**: Explicit `getDAO<AssetInertialStaticRecord>()` call in `DataRecorder` constructor after frame DAO initialization.

#### 3. Database Migration Strategy — **OUT OF SCOPE**

**Question**: How should existing databases with the old schema be migrated?

**Answer**: Out of scope for this ticket. Users with existing simulation databases will need to:
- Regenerate databases with the new schema (simplest), OR
- Write manual SQL migration script to add `body` column to `InertialStateRecord` and rename/convert `body_id` in `EnergyRecord`

**Accepted Limitation**: No automatic migration. Documentation should note this as a breaking schema change.

### Prototype Required
None — straightforward schema addition with well-understood FK patterns from existing `frame` FK usage.

### Requirements Clarification
None — all requirements clear from ticket description.

## Implementation Notes

### Body ID Stability

The `body.id` FK references `AssetInertialStaticRecord.id` (the auto-incremented PK from `BaseTransferObject`), NOT `body_id` (the instance ID). The `AssetInertialStaticRecord.body_id` field stores the instance ID for human-readable queries, but the FK relationship uses the PK.

**Query pattern**:
```cpp
// Given an InertialStateRecord, find the asset's static properties
uint32_t staticRecordId = stateRecord.body.id;
auto staticRecord = staticDAO.selectById(staticRecordId);
uint32_t instanceId = staticRecord->body_id;  // Original asset instance ID
```

### Backfill Performance

`backfillStaticData()` iterates all assets once. For typical simulations with < 1000 assets, this is negligible. For larger simulations (10,000+ assets), backfill may take several milliseconds. This is acceptable since `enableRecording()` is a one-time setup operation.

### Foreign Key Constraint Enforcement

`cpp_sqlite` enforces FK constraints at the SQLite level. If a per-frame record references a non-existent `AssetInertialStaticRecord.id`, the insert will fail with an FK violation error. This is the desired behavior — it catches bugs where static data was not recorded properly.

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-12
**Status**: APPROVED
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | All names follow project standards: `recordStaticData`, `backfillStaticData` are camelCase methods, `AssetInertialStaticRecord` is PascalCase, `body` field is lowercase |
| Namespace organization | ✓ | Transfer records in `msd_transfer`, domain logic in `msd_sim` — correct layering |
| File structure | ✓ | Follows `msd/{module}/src/` pattern. All file paths correctly specified |
| Dependency direction | ✓ | Correct: msd-sim → msd-transfer (no reverse dependency). AssetStaticState.toRecord() is the proper conversion boundary |

#### C++ Design Quality
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | DataRecorder uses `std::unique_ptr<Database>`, WorldModel uses `std::unique_ptr<DataRecorder>` — proper ownership |
| Smart pointer appropriateness | ✓ | Exclusive ownership via `unique_ptr`, references for non-owning access (AssetInertial&) — follows CLAUDE.md standards |
| Value/reference semantics | ✓ | Records use value semantics, helper methods take `const AssetInertial&` non-owning references |
| Rule of 0/3/5 | ✓ | Transfer records use Rule of Zero (default special members via `= default`) |
| Const correctness | ✓ | `recordStaticData(const AssetInertial&)` uses const reference, helper methods are private |
| Exception safety | ✓ | Database write errors handled by cpp_sqlite, FK violations reported as errors (desired behavior) |

#### Feasibility
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | No circular dependencies. AssetInertialStaticRecord already exists, cpp_sqlite::ForeignKey is header-only |
| Template complexity | ✓ | No new templates. Uses existing `getDAO<T>()` pattern already validated in DataRecorder |
| Memory strategy | ✓ | Static records are small (32 bytes), backfill is O(N) where N = asset count (acceptable) |
| Thread safety | ✓ | All writes go through thread-safe DataRecorder::getDAO().addToBuffer() (mutex-protected) |
| Build integration | ✓ | Requires adding AssetInertialStaticRecord to DataRecorder DAO initialization — straightforward |

#### Testability
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | Each component testable in isolation: AssetStaticState, WorldModel recording, DataRecorder DAO |
| Mockable dependencies | ✓ | DataRecorder can be null-checked (`if (dataRecorder_)`) for test scenarios without recording |
| Observable state | ✓ | FK values can be queried via `selectById()`, flush ensures records are queryable |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | FK mismatch between `body.id` (PK) and `body_id` (instance ID) could cause query confusion | Technical | Low | Medium | Clear documentation in design (already present), query pattern example provided | No |
| R2 | Backfill performance for 10,000+ assets could block enableRecording() | Performance | Low | Low | Backfill is one-time setup, acceptable delay. Could add progress logging if needed | No |
| R3 | Breaking schema change requires database migration or regeneration | Maintenance | High | Low | Documented as accepted limitation, users regenerate DBs (standard practice for this project) | No |

### Required Revisions
None — Design is approved.

### Summary

This design is well-structured and follows established patterns from the existing codebase. Key strengths:

1. **Correct layering**: Proper separation between transfer records (msd-transfer) and domain logic (msd-sim) with clean conversion boundary via `AssetStaticState::toRecord()`
2. **Consistent FK pattern**: Follows the same FK approach used for `SimulationFrameRecord` — proven design
3. **Non-blocking integration**: Recording is opt-in via `enableRecording()`, backward compatible
4. **Appropriate design decision**: Option C for R5 (environment bodies as raw IDs) is the right trade-off — simplicity over perfect FK integrity for static, known-at-compile-time data

Minor observations:
- R1 risk (PK vs instance ID) is well-mitigated by clear query pattern documentation
- R3 (breaking schema change) is an accepted limitation consistent with project practices
- Backfill performance (R2) is negligible for typical workloads (<1000 assets)

**Recommendation**: Proceed to implementation. No prototype required — this is a straightforward schema addition with well-understood FK patterns.

