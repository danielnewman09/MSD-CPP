# Ticket 0056b: CollisionPipeline Data Extraction & Extended Recording

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype (SKIPPED - no prototype needed)
- [x] Prototype Complete — Awaiting Review (SKIPPED)
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Merged / Complete

**Current Phase**: Merged / Complete
**Type**: Infrastructure
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-11
**Generate Tutorial**: No
**Parent Ticket**: [0056_browser_simulation_replay](0056_browser_simulation_replay.md)
**Depends On**: [0056a_collision_force_transfer_records](0056a_collision_force_transfer_records.md)

---

## Overview

Collision data (contact points, normals, constraint forces, solver diagnostics) is ephemeral — it is created and destroyed within `CollisionPipeline::execute()` each frame. This ticket makes that data accessible for recording after `execute()` completes.

**Architectural evolution**: The original design used a `FrameCollisionData` snapshot struct copied at end-of-execute. This was superseded by the `0056b1-eliminate-snapshot-layer` refactoring which recognized that `collisions_` (value-owned `CollisionResult` objects) and `solverData_` naturally survive between frames. The snapshot layer was eliminated — `CollisionPipeline` now exposes `getCollisions()` and `getSolverData()` directly, and `clearEphemeralState()` only clears reference/pointer vectors (not value-owned data).

**Recording responsibility** was further refined by:
- **0056i**: Static asset recording at spawn with FK linkage
- **0056j**: Domain-aware DataRecorder (all recording logic moved from WorldModel to DataRecorder)

---

## Requirements

### R1: Collision Data Accessible After execute() — IMPLEMENTED

`CollisionPipeline` exposes collision and solver data via direct accessors:

```cpp
// Value-owned data survives between frames
const std::vector<CollisionPair>& getCollisions() const;
const SolverData& getSolverData() const;
```

`CollisionPair` contains body instance IDs and the `CollisionResult` (normal, depth, contact manifold). `SolverData` contains iterations, residual, convergence, constraint/contact counts.

### R2: Data Lifecycle — IMPLEMENTED

`clearEphemeralState()` clears only reference/pointer vectors (states_, constraintPtrs_, etc.) that would dangle between frames. Value-owned `collisions_` and `solverData_` persist from end of `execute()` until start of next `execute()`, giving WorldModel a safe window to read them.

### R3: Extended WorldModel::recordCurrentFrame() — IMPLEMENTED (via 0056j)

Recording delegates to DataRecorder domain-aware methods:
- `dataRecorder_->recordCollisions(frameId, collisionPipeline_)` — persists collision results
- `dataRecorder_->recordSolverDiagnostics(frameId, collisionPipeline_)` — persists solver data

### R4: Static Asset Recording on Spawn — IMPLEMENTED (via 0056i)

`WorldModel::spawnObject()` calls `dataRecorder_->recordStaticAsset(asset)` when recording is enabled. Per-frame records reference static data via `ForeignKey<AssetInertialStaticRecord>`.

---

## Files Modified

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Collision/CollisionPipeline.hpp` | Eliminated `FrameCollisionData`, added `CollisionPair`/`SolverData` public structs, `getCollisions()`/`getSolverData()`, `clearEphemeralState()` |
| `msd-sim/src/Physics/Collision/CollisionPipeline.cpp` | Removed `snapshotFrameData()`/`clearFrameData()`, inline solver data capture, `clearEphemeralState()` |
| `msd-sim/src/Environment/WorldModel.hpp` | Simplified — recording helpers removed (moved to DataRecorder via 0056j) |
| `msd-sim/src/Environment/WorldModel.cpp` | Thin orchestrator `recordCurrentFrame()`, spawn-time recording via DataRecorder |
| `msd-sim/src/DataRecorder/DataRecorder.hpp` | Domain-aware recording methods (via 0056j) |
| `msd-sim/src/DataRecorder/DataRecorder.cpp` | Recording logic implementation (via 0056j) |

---

## Test Plan

### Regression Tests

All existing 713/717 tests must pass unchanged. The refactoring is structural — identical records are written to the database.

---

## Acceptance Criteria

1. [x] **AC1**: Collision data (contacts, normals, depths, body IDs) accessible after `execute()` via `getCollisions()`
2. [x] **AC2**: Solver diagnostics (iterations, residual, convergence) accessible via `getSolverData()`
3. [x] **AC3**: Data persists between frames without snapshot copy (value-owned members)
4. [x] **AC4**: `recordCurrentFrame()` writes collision results per frame (via DataRecorder)
5. [x] **AC5**: ~~ConstraintForceRecord~~ (removed — redundant with collision result data, see 0056a refactoring)
6. [x] **AC6**: ~~AppliedForceRecord~~ (removed — redundant, see 0056a refactoring)
7. [x] **AC7**: `recordCurrentFrame()` writes SolverDiagnosticRecord per frame (via DataRecorder)
8. [x] **AC8**: Static asset data recorded once per body at spawn (via 0056i)
9. [x] **AC9**: All existing tests pass (713/717, zero regressions)

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

### Implementation Phase (Original — 0056b branch)
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

### Quality Gate Phase (Original — FAILED)
- **Started**: 2026-02-12
- **Status**: FAILED
- **Branch**: `0056b-collision-pipeline-data-extraction`
- **PR**: #43
- **Failure**: 2 unused parameter errors in `snapshotFrameData()` (Release build -Werror)

### Refactoring Phase (0056b1 — Eliminate Snapshot Layer)
- **Started**: 2026-02-12
- **Completed**: 2026-02-12
- **Branch**: `0056b1-eliminate-snapshot-layer`
- **PR**: #44 (draft)
- **Commit**: `174c295`
- **Key Changes**:
  - Eliminated `FrameCollisionData` intermediate struct entirely
  - Removed `snapshotFrameData()` and `getLastFrameData()`
  - `collisions_` and `solverData_` are value-owned members that persist between frames
  - Added `getCollisions()` and `getSolverData()` direct accessors
  - Renamed `clearFrameData()` to `clearEphemeralState()` (only clears reference/pointer vectors)
  - WorldModel updated to use new accessors
- **Rationale**: Snapshot layer was unnecessary — `collisions_` holds `CollisionResult` by value (no dangling reference risk). Reduces data copies from 3 to 2 per frame.
- **Test Results**: 713/717 passing (zero regressions)

### Subsequent Tickets on Same Branch
- **0056i** (commit `26db50a`): Static asset recording at spawn + FK linkage — APPROVED
- **0056j** (commit `8ebc119`): Domain-aware DataRecorder — APPROVED
- Both tickets passed implementation review on `0056b1-eliminate-snapshot-layer` branch

### Quality Gate Phase (0056b1 — PASSED)
- **Started**: 2026-02-12 14:47
- **Completed**: 2026-02-12 14:47
- **Branch**: `0056b1-eliminate-snapshot-layer`
- **PR**: #44 (draft)
- **Status**: PASSED
- **Artifacts**:
  - `docs/designs/0056b_collision_pipeline_data_extraction/quality-gate-report.md` — Quality gate report
- **Results**:
  - Build: PASSED (zero warnings, zero errors)
  - Tests: PASSED (793/797, 4 pre-existing failures, 0 regressions)
  - Static Analysis: PASSED (28 stylistic warnings, 0 errors)
  - Benchmarks: N/A (not applicable per design)
- **Notes**: All gates passed. Snapshot layer elimination resolved original build failures. Zero regressions introduced. Ready for implementation review.

### Implementation Review Phase (0056b1 — APPROVED)
- **Started**: 2026-02-12 14:50
- **Completed**: 2026-02-12 14:50
- **Branch**: `0056b1-eliminate-snapshot-layer`
- **PR**: #44 (draft → ready for review)
- **Reviewer**: Workflow Orchestrator
- **Status**: APPROVED
- **Artifacts**:
  - `docs/designs/0056b_collision_pipeline_data_extraction/implementation-review.md` — Implementation review report
- **Key Findings**:
  - **Design Conformance**: EXCELLENT — Refactor improves upon original design
  - **Code Quality**: EXCELLENT — Proper RAII, value semantics, type safety
  - **Test Coverage**: PASSING — 793/797 tests pass (0 regressions)
  - **Code Style**: CONFORMANT — Follows project standards
- **Strengths**:
  1. Architectural improvement (reduces data copies 3→2 per frame)
  2. Simplifies API (direct accessors vs struct unwrapping)
  3. Zero regressions (validates refactoring safety)
  4. Clean separation of concerns (CollisionPipeline/DataRecorder/WorldModel)
  5. Proper resource management throughout
- **Recommendations** (non-blocking):
  - Add [[nodiscard]] attributes to getCollisions()/getSolverData()
  - Document data validity window in Doxygen comments
- **Decision**: Approved for merge. Ready for human review and merge.
