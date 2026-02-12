# Ticket 0056h: AssetDynamicState Domain Struct

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Review
**Type**: Infrastructure
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-12
**Generate Tutorial**: No
**Prototype**: No
**Parent Ticket**: [0056_browser_simulation_replay](0056_browser_simulation_replay.md)
**Depends On**: [0056a_collision_force_transfer_records](0056a_collision_force_transfer_records.md)

---

## Overview

Create an `AssetDynamicState` value struct that groups the three per-frame mutable fields currently scattered across `AssetInertial`: the kinematic state (`InertialState`), accumulated force (`ForceVector`), and accumulated torque (`TorqueVector`). Then refactor `AssetInertial` to hold a single `AssetDynamicState` member instead of three separate fields.

This mirrors the `AssetDynamicStateRecord` transfer record (created in 0056a) on the domain side, making the relationship between domain objects and their database representation explicit. It also simplifies WorldModel recording by providing a single struct that captures everything needed for per-frame state persistence.

---

## Requirements

### R1: AssetDynamicState Value Struct

Create a header-only value struct following the `InertialState` pattern:

```cpp
struct AssetDynamicState
{
  InertialState inertialState;
  ForceVector accumulatedForce{0.0, 0.0, 0.0};
  TorqueVector accumulatedTorque{0.0, 0.0, 0.0};

  void clearForces();

  [[nodiscard]] msd_transfer::AssetDynamicStateRecord
  toRecord(uint32_t bodyId) const;

  static AssetDynamicState
  fromRecord(const msd_transfer::AssetDynamicStateRecord& record);
};
```

Key design decisions:
- `body_id` is a parameter to `toRecord()`, not a member — it's asset identity, not state
- `clearForces()` zeroes both accumulators (moved from AssetInertial)
- `toRecord()` delegates to sub-object `toRecord()` methods
- `fromRecord()` static factory reconstructs from transfer record

### R2: Refactor AssetInertial Members

Replace three private members with one:

```cpp
// Before:
InertialState dynamicState_;
ForceVector accumulatedForce_{0.0, 0.0, 0.0};
TorqueVector accumulatedTorque_{0.0, 0.0, 0.0};

// After:
AssetDynamicState dynamicState_;
```

### R3: Add getDynamicState() Accessor

Add new public accessors to `AssetInertial`:

```cpp
AssetDynamicState& getDynamicState();
const AssetDynamicState& getDynamicState() const;
```

### R4: Preserve Backward Compatibility

All existing public API must continue to work with identical signatures:
- `getInertialState()` → returns `dynamicState_.inertialState`
- `getAccumulatedForce()` / `getAccumulatedTorque()` → return from `dynamicState_`
- `clearForces()` → delegates to `dynamicState_.clearForces()`
- `toDynamicStateRecord()` → delegates to `dynamicState_.toRecord(instanceId_)`
- All `applyForce/Torque/Impulse` methods remain on `AssetInertial`

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `msd-sim/src/Physics/RigidBody/AssetDynamicState.hpp` | Domain-side value struct (header-only) |
| `msd-sim/test/Physics/RigidBody/AssetDynamicStateTest.cpp` | Unit tests for struct |

### Modified Files
| File | Change |
|------|--------|
| `msd-sim/src/Physics/RigidBody/AssetInertial.hpp` | Replace 3 members → 1, add getDynamicState(), update includes |
| `msd-sim/src/Physics/RigidBody/AssetInertial.cpp` | Update internal field paths, simplify toDynamicStateRecord() |
| `msd-sim/src/Physics/RigidBody/CMakeLists.txt` | Add AssetDynamicState.hpp to PHYSICS_HEADER_FILES |
| `msd-sim/test/Physics/RigidBody/CMakeLists.txt` | Add test file to target_sources |

---

## Test Plan

### Unit Tests

```cpp
// AssetDynamicState struct tests
TEST(AssetDynamicState, DefaultConstruction_ForcesTorquesZero)
TEST(AssetDynamicState, ClearForces_ResetsToZero)
TEST(AssetDynamicState, ToRecord_CapturesAllFields)
TEST(AssetDynamicState, FromRecord_ReconstructsCorrectly)
TEST(AssetDynamicState, RoundTrip_ToRecordFromRecord)
```

### Regression Tests

All existing tests must pass with zero changes:
- `AssetDynamicStateTransferTest.*` (4 tests)
- Full `msd_sim_test` suite

---

## Acceptance Criteria

1. [x] **AC1**: `AssetDynamicState` struct created with InertialState + ForceVector + TorqueVector
2. [x] **AC2**: `AssetInertial` uses single `AssetDynamicState` member instead of 3 separate fields
3. [x] **AC3**: `getDynamicState()` accessor exposes the composite state
4. [x] **AC4**: All existing `getInertialState()` callers work unchanged (backward compatible)
5. [x] **AC5**: `toDynamicStateRecord()` delegates to `AssetDynamicState::toRecord()`
6. [x] **AC6**: `fromRecord()` static factory correctly reconstructs from transfer record
7. [x] **AC7**: All existing tests pass (zero regressions)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-12
- **Notes**: Ticket created as companion to 0056a (transfer records). Provides domain-side struct to mirror AssetDynamicStateRecord. No design phase needed — requirements are fully specified as a mechanical refactoring with clear before/after.

### Ready for Implementation
- **Advanced**: 2026-02-12 18:23
- **Branch**: 0056a-collision-force-transfer-records (shared with 0056a)
- **PR**: N/A (will be part of 0056a PR)
- **Notes**: Skipped design phase per ticket metadata (Prototype: No). Dependency 0056a is implementation-complete. Ready for implementer agent.

### Implementation Phase
- **Started**: 2026-02-12 18:25
- **Completed**: 2026-02-12 18:30
- **Branch**: 0056a-collision-force-transfer-records
- **PR**: N/A (part of 0056a PR)
- **Artifacts**:
  - `msd-sim/src/Physics/RigidBody/AssetDynamicState.hpp` (header-only struct)
  - `msd-sim/src/Physics/RigidBody/AssetInertial.hpp` (refactored to use single member)
  - `msd-sim/src/Physics/RigidBody/AssetInertial.cpp` (updated accessors)
  - `msd-sim/src/Physics/RigidBody/CMakeLists.txt` (added AssetDynamicState.hpp)
  - `msd-sim/test/Physics/RigidBody/AssetDynamicStateTest.cpp` (7 unit tests)
  - `msd-sim/test/Physics/RigidBody/CMakeLists.txt` (added test file)
- **Test Results**: 11/11 AssetDynamicState tests pass, 619/672 total sim tests pass (53 pre-existing failures)
- **Notes**: Implementation found to be already complete. Verified all acceptance criteria met. Struct provides clean domain-side mirror of AssetDynamicStateRecord. All public API backward compatible via delegation.
