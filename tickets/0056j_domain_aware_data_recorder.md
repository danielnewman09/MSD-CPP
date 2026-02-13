# Ticket 0056j: Domain-Aware DataRecorder

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Merged / Complete

**Current Phase**: Merged / Complete
**Type**: Refactoring
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-12
**Generate Tutorial**: No
**Prototype**: No
**Parent Ticket**: [0056_browser_simulation_replay](0056_browser_simulation_replay.md)
**Depends On**: [0056i_static_asset_recording_and_fk](0056i_static_asset_recording_and_fk.md)

---

## Overview

Move all `record*()` functions from `WorldModel` to `DataRecorder`, making DataRecorder domain-aware. Currently WorldModel contains ~100 lines of recording logic that iterates domain objects, calls `toRecord()`, sets FK fields, and buffers records via DAOs. This logic belongs in DataRecorder — WorldModel should only be responsible for calling `dataRecorder_->recordFrame(...)` with the relevant domain data.

DataRecorder already lives in `msd-sim` (not `msd-transfer`), so it is already coupled to the simulation layer. Making it domain-aware is a natural progression.

---

## Requirements

### R1: Move Per-Frame Recording to DataRecorder

Create domain-aware recording methods on DataRecorder:

```cpp
void recordInertialStates(uint32_t frameId,
                          std::span<const AssetInertial> assets);

void recordBodyEnergies(uint32_t frameId,
                        std::span<const AssetInertial> assets,
                        std::span<const std::unique_ptr<PotentialEnergy>> potentials);

void recordSystemEnergy(uint32_t frameId,
                        const EnergyTracker::SystemEnergy& energy,
                        double previousTotal,
                        bool collisionActive);

void recordCollisions(uint32_t frameId,
                      const std::vector<CollisionPipeline::CollisionPair>& pairs);

void recordSolverDiagnostics(uint32_t frameId,
                             const CollisionPipeline::SolverData& solver);
```

Each method encapsulates the iteration, `toRecord()` calls, FK assignment, and DAO buffering that currently lives in WorldModel.

### R2: Move Static Asset Recording to DataRecorder

```cpp
void recordStaticAsset(const AssetInertial& asset);
```

Replaces `WorldModel::recordStaticData()`.

### R3: Simplify WorldModel Recording

`WorldModel::recordCurrentFrame()` becomes a thin orchestrator:
- Calls `dataRecorder_->recordFrame(simulationTime)` for the frame ID
- Passes domain objects to DataRecorder methods
- Retains only the `previousSystemEnergy_` tracking (WorldModel state)

`WorldModel::enableRecording()` calls `dataRecorder_->recordStaticAsset()` for backfill instead of `backfillStaticData()`.

### R4: Remove Recording Methods from WorldModel

Delete from WorldModel:
- `recordCollisions(uint32_t frameId)`
- `recordSolverDiagnostics(uint32_t frameId)`
- `recordStaticData(const AssetInertial&)`
- `backfillStaticData()`

`recordCurrentFrame()` stays but becomes significantly shorter.

### R5: Preserve Thread Safety

All new DataRecorder methods must remain thread-safe via the existing mutex-protected DAO `addToBuffer()` pattern. No new synchronization required — the existing model (simulation thread writes to buffer, recorder thread flushes) is preserved.

---

## Files to Modify

| File | Change |
|------|--------|
| `msd-sim/src/DataRecorder/DataRecorder.hpp` | Add domain-aware recording methods, add includes for domain types |
| `msd-sim/src/DataRecorder/DataRecorder.cpp` | Implement recording methods (logic moved from WorldModel) |
| `msd-sim/src/Environment/WorldModel.hpp` | Remove recording helper declarations |
| `msd-sim/src/Environment/WorldModel.cpp` | Simplify recordCurrentFrame(), remove helper implementations, update enableRecording() |

---

## Test Plan

### Regression Tests

All existing 713/717 tests must pass unchanged. The refactoring is purely structural — identical records are written to the database.

---

## Acceptance Criteria

1. [ ] **AC1**: All `record*()` logic moved from WorldModel to DataRecorder
2. [ ] **AC2**: WorldModel::recordCurrentFrame() is a thin orchestrator (<30 lines)
3. [ ] **AC3**: DataRecorder accepts domain types (AssetInertial, CollisionPair, SolverData, etc.)
4. [ ] **AC4**: Static asset recording at spawn delegated to DataRecorder
5. [ ] **AC5**: All existing tests pass (zero regressions)
6. [ ] **AC6**: No new public WorldModel methods for recording (only DataRecorder)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-12
- **Notes**: Ticket created from discussion about moving recording responsibility from WorldModel to DataRecorder. Option A (domain-aware DataRecorder) chosen over Option B (record-level). DataRecorder already in msd-sim, coupling to domain types is natural.

### Design Phase
- **Started**: 2026-02-12
- **Completed**: 2026-02-12
- **Branch**: 0056b1-eliminate-snapshot-layer
- **PR**: #44 (draft)
- **Notes**: Skipped formal design phase — this is a mechanical refactoring with clear requirements. The ticket specifies exact method signatures and logic to move. No architectural decisions needed.

### Implementation Phase
- **Started**: 2026-02-12
- **Completed**: 2026-02-12
- **Branch**: 0056b1-eliminate-snapshot-layer
- **PR**: #44 (draft)
- **Commit**: 8ebc119
- **Artifacts**:
  - `msd-sim/src/DataRecorder/DataRecorder.hpp` — Added 6 domain-aware recording methods
  - `msd-sim/src/DataRecorder/DataRecorder.cpp` — Implemented recording logic (moved from WorldModel)
  - `msd-sim/src/Environment/WorldModel.hpp` — Removed recording helper declarations
  - `msd-sim/src/Environment/WorldModel.cpp` — Simplified recordCurrentFrame() to 24 lines
- **Test Results**: 713/717 passing (baseline maintained, zero regressions)
- **Notes**:
  - DataRecorder.hpp includes EnergyTracker.hpp for nested SystemEnergy struct in signature
  - recordCollisions/recordSolverDiagnostics take CollisionPipeline by const ref (nested structs)
  - WorldModel::recordCurrentFrame() now a thin orchestrator delegating to DataRecorder
  - All recording logic successfully moved from WorldModel to DataRecorder
  - previousSystemEnergy_ tracking remains in WorldModel (simulation state)

### Implementation Review Phase
- **Started**: 2026-02-12
- **Completed**: 2026-02-12
- **Branch**: 0056b1-eliminate-snapshot-layer
- **PR**: #44 (draft)
- **Commit**: f59ec90
- **Artifacts**:
  - `docs/designs/0056j_domain_aware_data_recorder/implementation-review.md` — APPROVED
- **Test Results**: 713/717 passing (zero regressions)
- **Notes**:
  - All 6 acceptance criteria met (AC1-AC6)
  - Design conformance: PASS — All ticket requirements satisfied
  - Code quality: PASS — Thread safety preserved, project standards followed
  - Test coverage: PASS — Baseline maintained, zero regressions
  - recordCurrentFrame() is 24 lines (requirement: <30 lines)
  - Review summary posted to PR #44

### Documentation Phase
- **Started**: 2026-02-12
- **Completed**: 2026-02-12
- **Branch**: 0056b1-eliminate-snapshot-layer
- **PR**: #44 (draft)
- **Commit**: 6955023
- **Artifacts**:
  - `msd/msd-sim/CLAUDE.md` — Updated DataRecorder documentation with domain-aware methods
  - `docs/designs/0056j_domain_aware_data_recorder/doc-sync-summary.md` — Documentation sync summary
- **Notes**:
  - Updated DataRecorder Purpose section to explain domain-aware recording
  - Added 6 domain-aware methods to Key Interfaces
  - Updated Thread Model to reflect WorldModel delegation pattern
  - Added WorldModel Integration "Recording Flow" subsection
  - No new diagrams needed (mechanical refactoring)
  - Generate Tutorial: No — skipping tutorial phase per ticket metadata
