# Iteration Log — 0056b_collision_pipeline_data_extraction

> **Purpose**: Track every build-test cycle during implementation or investigation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/designs/0056b_collision_pipeline_data_extraction/iteration-log.md`
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0056b_collision_pipeline_data_extraction
**Branch**: 0056b-collision-pipeline-data-extraction
**Baseline**: 657/661 tests passing at start

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 1 — 2026-02-11 14:30
**Commit**: 4fa59c7
**Hypothesis**: Add FrameCollisionData struct and snapshot logic to CollisionPipeline to capture ephemeral collision data before clearFrameData()
**Changes**:
- `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp`: Add FrameCollisionData nested struct with ContactData, BodyForceData, SolverData; add getLastFrameData() accessor; add snapshotFrameData() private method signature
- `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp`: Implement getLastFrameData() accessor; implement snapshotFrameData() to extract contacts, forces, solver diagnostics; call snapshotFrameData() after Phase 6; clear lastFrameData_ at start of execute()
**Build Result**: PASS
**Test Result**: 657/661 — Same as baseline (no regressions)
**Impact vs Previous**: 0 change (baseline maintained)
**Assessment**: CollisionPipeline changes complete. Snapshot captures all required data. No ConstraintSolver changes needed (bodyForces already exists). Next: Extend WorldModel::recordCurrentFrame() to persist the snapshot data.

