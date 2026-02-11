# Ticket 0056a: Collision & Force Transfer Record Types

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Review
**Type**: Infrastructure
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-11
**Generate Tutorial**: No
**Parent Ticket**: [0056_browser_simulation_replay](0056_browser_simulation_replay.md)
**Depends On**: None

---

## Overview

Create five new transfer record types in `msd-transfer` to capture collision, force, and solver data that the DataRecorder currently does not persist. These records are the foundation for the entire replay system — without them, there is nothing to visualize beyond kinematic state and energy.

All records follow the established pattern: inherit `cpp_sqlite::BaseTransferObject`, use `BOOST_DESCRIBE_STRUCT`, include `ForeignKey<SimulationFrameRecord>` for temporal association.

---

## Requirements

### R1: ContactRecord

Per-contact-point data for collision visualization. Up to 4 contacts per collision pair per frame.

```cpp
struct ContactRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_a_id{0};     // Instance ID of body A
  uint32_t body_b_id{0};     // Instance ID of body B
  uint32_t contact_index{0}; // Contact index within manifold [0,3]

  double point_a_x{std::numeric_limits<double>::quiet_NaN()};
  double point_a_y{std::numeric_limits<double>::quiet_NaN()};
  double point_a_z{std::numeric_limits<double>::quiet_NaN()};

  double point_b_x{std::numeric_limits<double>::quiet_NaN()};
  double point_b_y{std::numeric_limits<double>::quiet_NaN()};
  double point_b_z{std::numeric_limits<double>::quiet_NaN()};

  double normal_x{std::numeric_limits<double>::quiet_NaN()};
  double normal_y{std::numeric_limits<double>::quiet_NaN()};
  double normal_z{std::numeric_limits<double>::quiet_NaN()};

  double depth{std::numeric_limits<double>::quiet_NaN()};
  double restitution{std::numeric_limits<double>::quiet_NaN()};
  double friction{std::numeric_limits<double>::quiet_NaN()};

  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};
```

**Design choice**: Flat fields (e.g. `point_a_x`, `point_a_y`, `point_a_z`) rather than FK to CoordinateRecord. Rationale: multiple 3D vectors per record (pointA, pointB, normal) would create a complex FK web. Flat fields are simpler to query and each row is self-contained.

### R2: ConstraintForceRecord

Per-body solved constraint forces (output of the constraint solver).

```cpp
struct ConstraintForceRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_id{0};

  double force_x{0.0};
  double force_y{0.0};
  double force_z{0.0};

  double torque_x{0.0};
  double torque_y{0.0};
  double torque_z{0.0};

  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};
```

### R3: AppliedForceRecord

Per-body applied forces (gravity, external forces, potential energy fields).

```cpp
struct AppliedForceRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_id{0};
  uint32_t force_type{0}; // 0=gravity, 1=external, 2=potential

  double force_x{0.0};
  double force_y{0.0};
  double force_z{0.0};

  double torque_x{0.0};
  double torque_y{0.0};
  double torque_z{0.0};

  double point_x{std::numeric_limits<double>::quiet_NaN()};
  double point_y{std::numeric_limits<double>::quiet_NaN()};
  double point_z{std::numeric_limits<double>::quiet_NaN()};

  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};
```

### R4: SolverDiagnosticRecord

Per-frame solver statistics for debugging convergence issues.

```cpp
struct SolverDiagnosticRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t iterations{0};
  double residual{std::numeric_limits<double>::quiet_NaN()};
  uint32_t converged{0};      // Boolean as uint32_t for SQLite
  uint32_t num_constraints{0};
  uint32_t num_contacts{0};

  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};
```

### R5: BodyMetadataRecord

Per-body static properties, recorded once at spawn time (not per frame).

```cpp
struct BodyMetadataRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_id{0};
  uint32_t asset_id{0};
  double mass{std::numeric_limits<double>::quiet_NaN()};
  double restitution{std::numeric_limits<double>::quiet_NaN()};
  double friction{std::numeric_limits<double>::quiet_NaN()};
  uint32_t is_environment{0}; // Boolean as uint32_t for SQLite

  // No frame FK — recorded once, not per-frame
};
```

**Note**: BodyMetadataRecord has no `ForeignKey<SimulationFrameRecord>` because it is recorded once at spawn, not per frame.

### R6: Records.hpp Update

Add includes for all 5 new headers to `msd/msd-transfer/src/Records.hpp`.

### R7: DataRecorder Template Instantiations

Add explicit template instantiations in `msd/msd-sim/src/DataRecorder/DataRecorder.cpp` for all 5 new record types.

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `msd-transfer/src/ContactRecord.hpp` | Per-contact collision data |
| `msd-transfer/src/ConstraintForceRecord.hpp` | Per-body constraint forces |
| `msd-transfer/src/AppliedForceRecord.hpp` | Per-body applied forces |
| `msd-transfer/src/SolverDiagnosticRecord.hpp` | Per-frame solver stats |
| `msd-transfer/src/BodyMetadataRecord.hpp` | Per-body static metadata |

### Modified Files
| File | Change |
|------|--------|
| `msd-transfer/src/Records.hpp` | Add 5 new includes |
| `msd-sim/src/DataRecorder/DataRecorder.cpp` | Add 5 explicit template instantiations |

---

## Test Plan

### Unit Tests

```cpp
// Verify record creation and field initialization
TEST(ContactRecord, DefaultConstructor_FieldsInitializedToNaN)
TEST(ContactRecord, BoostDescribe_AllFieldsDescribed)

TEST(ConstraintForceRecord, DefaultConstructor_ForcesZero)
TEST(AppliedForceRecord, DefaultConstructor_ForceTypeZero)
TEST(SolverDiagnosticRecord, DefaultConstructor_IterationsZero)
TEST(BodyMetadataRecord, DefaultConstructor_MassNaN)
```

### Integration Tests

```cpp
// Verify records can be inserted and queried via cpp_sqlite
TEST(RecordIntegration, ContactRecord_InsertAndSelectAll)
TEST(RecordIntegration, ContactRecord_ForeignKeyResolution)
TEST(RecordIntegration, BodyMetadataRecord_NoFrameFK)

// Verify DataRecorder template instantiations compile
TEST(DataRecorder, GetDAO_ContactRecord_ReturnsValidDAO)
TEST(DataRecorder, GetDAO_ConstraintForceRecord_ReturnsValidDAO)
TEST(DataRecorder, GetDAO_AppliedForceRecord_ReturnsValidDAO)
TEST(DataRecorder, GetDAO_SolverDiagnosticRecord_ReturnsValidDAO)
TEST(DataRecorder, GetDAO_BodyMetadataRecord_ReturnsValidDAO)
```

---

## Acceptance Criteria

1. [ ] **AC1**: All 5 record types compile with BOOST_DESCRIBE_STRUCT
2. [ ] **AC2**: cpp_sqlite auto-generates correct SQLite schema for each record
3. [ ] **AC3**: Records can be inserted and queried via DAO pattern
4. [ ] **AC4**: ForeignKey<SimulationFrameRecord> resolves correctly on 4 per-frame records
5. [ ] **AC5**: BodyMetadataRecord works without frame FK
6. [ ] **AC6**: DataRecorder::getDAO<T>() works for all 5 types
7. [ ] **AC7**: Records.hpp includes all new headers
8. [ ] **AC8**: All existing tests pass (zero regressions)

---

## Workflow Log

### Draft → Ready for Implementation
- **Started**: 2026-02-11
- **Completed**: 2026-02-11
- **Branch**: Not yet created
- **PR**: N/A
- **Artifacts**: None (status advancement only)
- **Notes**: Infrastructure ticket with well-defined requirements. No design phase needed - requirements specify exact struct layouts and integration points. Advancing directly to implementation.

### Implementation Phase
- **Started**: 2026-02-11
- **Completed**: 2026-02-11
- **Branch**: `0056a-collision-force-transfer-records`
- **PR**: #42 (https://github.com/danielnewman09/MSD-CPP/pull/42)
- **Artifacts**:
  - `msd-transfer/src/ContactRecord.hpp` — Per-contact collision geometry
  - `msd-transfer/src/ConstraintForceRecord.hpp` — Per-body constraint forces
  - `msd-transfer/src/AppliedForceRecord.hpp` — Per-body applied forces
  - `msd-transfer/src/SolverDiagnosticRecord.hpp` — Per-frame solver diagnostics
  - `msd-transfer/src/BodyMetadataRecord.hpp` — Per-body static metadata
  - `msd-transfer/src/Records.hpp` — Updated with 5 new includes
  - `msd-sim/src/DataRecorder/DataRecorder.cpp` — Added 5 template instantiations
- **Notes**: All records follow established pattern (BaseTransferObject + BOOST_DESCRIBE_STRUCT). BodyMetadataRecord omits frame FK as intended (spawn-time data). Build successful, 0 test regressions (657/661 passing, 4 pre-existing failures). Commit: 7b4bb91.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
