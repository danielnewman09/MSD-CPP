# Ticket 0056b: CollisionPipeline Data Extraction & Extended Recording

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype (SKIPPED - no prototype needed)
- [x] Prototype Complete — Awaiting Review (SKIPPED)
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Quality Gate
**Type**: Infrastructure
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-11
**Generate Tutorial**: No
**Parent Ticket**: [0056_browser_simulation_replay](0056_browser_simulation_replay.md)
**Depends On**: [0056a_collision_force_transfer_records](0056a_collision_force_transfer_records.md)

---

## Overview

Collision data (contact points, normals, constraint forces, solver diagnostics) is ephemeral — it is created and destroyed within `CollisionPipeline::execute()` each frame. This ticket adds a snapshot mechanism to capture that data before it is cleared, then extends `WorldModel::recordCurrentFrame()` to persist it.

The key design principle is **separation of concerns**: CollisionPipeline captures data into a struct but knows nothing about the DataRecorder. WorldModel reads the struct and writes records.

---

## Requirements

### R1: FrameCollisionData Struct

Add a data struct to `CollisionPipeline` that captures all visualization-relevant collision data for a single frame.

```cpp
struct FrameCollisionData
{
  struct ContactData
  {
    uint32_t bodyAId;
    uint32_t bodyBId;
    Coordinate pointA;
    Coordinate pointB;
    Coordinate normal;
    double depth;
    double restitution;
    double friction;
    uint32_t contactIndex;
  };
  std::vector<ContactData> contacts;

  struct BodyForceData
  {
    uint32_t bodyId;
    Vector3D linearForce;
    Vector3D angularTorque;
  };
  std::vector<BodyForceData> constraintForces;

  struct SolverData
  {
    int iterations{0};
    double residual{0.0};
    bool converged{false};
    size_t numConstraints{0};
    size_t numContacts{0};
  };
  SolverData solverData;
};
```

### R2: Snapshot in CollisionPipeline::execute()

After Phase 5 (force application) and before `clearFrameData()`:
1. Iterate `collisions_` to extract contact data into `lastFrameData_.contacts`
2. Extract per-body constraint forces from solver result into `lastFrameData_.constraintForces`
3. Capture solver diagnostics (iterations, residual, convergence) into `lastFrameData_.solverData`

Add const accessor:
```cpp
const FrameCollisionData& getLastFrameData() const;
```

### R3: Extended WorldModel::recordCurrentFrame()

Extend the existing recording logic to also persist:

1. **ContactRecord** — from `collisionPipeline_.getLastFrameData().contacts`
2. **ConstraintForceRecord** — from `collisionPipeline_.getLastFrameData().constraintForces`
3. **AppliedForceRecord** — gravity forces computed from `potentialEnergies_` for each body
4. **SolverDiagnosticRecord** — from `collisionPipeline_.getLastFrameData().solverData`

### R4: BodyMetadataRecord on Spawn

Modify `WorldModel::spawnObject()` and `WorldModel::spawnEnvironmentObject()` to record `BodyMetadataRecord` when `dataRecorder_` is active. Metadata is static (mass, restitution, friction, asset_id, is_environment) and only needs to be recorded once per body.

---

## Files to Modify

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Collision/CollisionPipeline.hpp` | Add `FrameCollisionData` struct, `lastFrameData_` member, `getLastFrameData()` accessor |
| `msd-sim/src/Physics/Collision/CollisionPipeline.cpp` | Snapshot logic after Phase 5 in `execute()` |
| `msd-sim/src/Environment/WorldModel.hpp` | Add method signatures if needed |
| `msd-sim/src/Environment/WorldModel.cpp` | Extended `recordCurrentFrame()`, metadata recording in spawn methods |

---

## Test Plan

### Unit Tests

```cpp
// Verify snapshot captures contact data correctly
TEST(CollisionPipeline, GetLastFrameData_ReturnsContactPoints)
TEST(CollisionPipeline, GetLastFrameData_ReturnsCorrectBodyIds)
TEST(CollisionPipeline, GetLastFrameData_ReturnsNormals)
TEST(CollisionPipeline, GetLastFrameData_ReturnsPenetrationDepth)

// Verify constraint forces captured
TEST(CollisionPipeline, GetLastFrameData_ReturnsConstraintForces)

// Verify solver diagnostics captured
TEST(CollisionPipeline, GetLastFrameData_ReturnsSolverDiagnostics)

// Verify no data when no collisions
TEST(CollisionPipeline, GetLastFrameData_EmptyWhenNoCollisions)
```

### Integration Tests

```cpp
// Verify extended recording writes all new record types
TEST(WorldModel, RecordCurrentFrame_WritesContactRecords)
TEST(WorldModel, RecordCurrentFrame_WritesConstraintForceRecords)
TEST(WorldModel, RecordCurrentFrame_WritesAppliedForceRecords)
TEST(WorldModel, RecordCurrentFrame_WritesSolverDiagnosticRecords)

// Verify metadata recorded on spawn
TEST(WorldModel, SpawnObject_RecordsBodyMetadata)
TEST(WorldModel, SpawnEnvironmentObject_RecordsBodyMetadata)
TEST(WorldModel, SpawnObject_MetadataRecordedOnce)

// End-to-end: run simulation with collision, verify all records in DB
TEST(ReplayRecording, CollisionSimulation_AllRecordTypesPopulated)
```

---

## Acceptance Criteria

1. [ ] **AC1**: `FrameCollisionData` struct captures all contact, force, and solver data
2. [ ] **AC2**: `getLastFrameData()` returns correct data after `execute()`
3. [ ] **AC3**: Snapshot preserves data across `clearFrameData()` call
4. [ ] **AC4**: `recordCurrentFrame()` writes ContactRecord for each contact point
5. [ ] **AC5**: `recordCurrentFrame()` writes ConstraintForceRecord for each body with forces
6. [ ] **AC6**: `recordCurrentFrame()` writes AppliedForceRecord for gravity on each body
7. [ ] **AC7**: `recordCurrentFrame()` writes SolverDiagnosticRecord per frame
8. [ ] **AC8**: BodyMetadataRecord recorded once per body at spawn
9. [ ] **AC9**: All existing tests pass (zero regressions)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

---

## Workflow Log

### Draft → Ready for Design
- **Completed**: 2026-02-11
- **Notes**: Ticket created with detailed requirements. No math design required. Ready for architectural design phase.

### Design Phase
- **Started**: 2026-02-11
- **Completed**: 2026-02-11
- **Branch**: `0056b-collision-pipeline-data-extraction`
- **PR**: #43 (https://github.com/danielnewman09/MSD-CPP/pull/43)
- **Artifacts**:
  - `docs/designs/0056b_collision_pipeline_data_extraction/design.md` — Detailed design document
  - `docs/designs/0056b_collision_pipeline_data_extraction/0056b_collision_pipeline_data_extraction.puml` — Architecture diagram
- **Notes**: Design covers FrameCollisionData snapshot struct, CollisionPipeline integration, WorldModel recording extensions, and ConstraintSolver force extraction. Three open questions flagged: force extraction granularity, snapshot preservation timing, and BodyMetadataRecord uniqueness handling. Fully backward compatible design—all changes are additive.

### Design Review
- **Completed**: 2026-02-11
- **Reviewer**: Human (via orchestrator feedback)
- **Key Findings**:
  1. ConstraintSolver::SolveResult ALREADY has bodyForces with linearForce and angularTorque — skip ConstraintSolver changes entirely
  2. Force extraction: Per-body net forces (Option A, already available)
  3. Body ID reuse: Accept duplicates (no uniqueness constraint)
  4. Snapshot timing: Confirmed — recordCurrentFrame() called in update() after execute()
- **Decision**: Approved for implementation, skip prototype phase (no unknowns)

### Implementation Phase
- **Started**: 2026-02-11
- **Completed**: 2026-02-11
- **Branch**: `0056b-collision-pipeline-data-extraction`
- **PR**: #43 (https://github.com/danielnewman09/MSD-CPP/pull/43) — marked ready for review
- **Artifacts**:
  - `docs/designs/0056b_collision_pipeline_data_extraction/implementation-notes.md` — Implementation summary, deviations, test plan
  - `docs/designs/0056b_collision_pipeline_data_extraction/iteration-log.md` — 2 iterations, 0 regressions
- **Files Modified** (~455 LOC added):
  - `msd/msd-sim/src/Physics/Collision/CollisionPipeline.{hpp,cpp}` — FrameCollisionData struct, snapshotFrameData(), getLastFrameData()
  - `msd/msd-sim/src/Environment/WorldModel.{hpp,cpp}` — 5 recording helpers, recordCurrentFrame() extension, spawn metadata
- **Test Results**: 657/661 passing (baseline maintained, 0 regressions)
- **Notes**: Implementation complete per design. ConstraintSolver changes skipped (bodyForces already existed). All 5 record types implemented. Metadata recorded at spawn, collision data recorded per frame. Awaiting quality gate (unit/integration tests).
