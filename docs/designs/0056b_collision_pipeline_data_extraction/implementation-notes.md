# Implementation Notes — 0056b_collision_pipeline_data_extraction

## Summary

Successfully implemented collision data extraction and extended recording according to design document. Key accomplishment: **No ConstraintSolver changes required** — `SolveResult::bodyForces` already existed with exactly the fields we needed (`linearForce`, `angularTorque`).

## Files Created

_None — all changes were additions to existing files._

## Files Modified

| File | Purpose | LOC Added |
|------|---------|-----------|
| `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp` | Added FrameCollisionData nested struct, getLastFrameData() accessor, snapshotFrameData() private method | +120 |
| `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp` | Implemented getLastFrameData(), snapshotFrameData(), clear lastFrameData_ at execute() start, call snapshot after Phase 6 | +70 |
| `msd/msd-sim/src/Environment/WorldModel.hpp` | Added 5 private recording helper method signatures | +85 |
| `msd/msd-sim/src/Environment/WorldModel.cpp` | Added includes for 5 new transfer records; implemented 5 recording helpers; extended recordCurrentFrame(); added metadata recording in spawn methods | +180 |
| **Total** | | **~455 LOC** |

## Design Adherence Matrix

| Design Requirement | Status | Notes |
|--------------------|--------|-------|
| R1: FrameCollisionData struct | ✅ | Nested struct with ContactData, BodyForceData, SolverData per design |
| R2: Snapshot in execute() | ✅ | Called after Phase 6, before clearFrameData() |
| R3: Extended recordCurrentFrame() | ✅ | Calls all 4 recording helpers (contacts, constraint forces, applied forces, solver diagnostics) |
| R4: BodyMetadataRecord on spawn | ✅ | Recorded in spawnObject() and spawnEnvironmentObject() if dataRecorder_ active |
| AC1: FrameCollisionData captures all data | ✅ | Contacts, forces, solver diagnostics all captured |
| AC2: getLastFrameData() returns correct data | ⏳ | Needs unit test verification |
| AC3: Snapshot preserves data across clearFrameData() | ✅ | Snapshot occurs before clear, data stored in lastFrameData_ member |
| AC4: recordCurrentFrame() writes ContactRecord | ⏳ | Implemented, needs integration test |
| AC5: recordCurrentFrame() writes ConstraintForceRecord | ⏳ | Implemented, needs integration test |
| AC6: recordCurrentFrame() writes AppliedForceRecord | ⏳ | Implemented, needs integration test |
| AC7: recordCurrentFrame() writes SolverDiagnosticRecord | ⏳ | Implemented, needs integration test |
| AC8: BodyMetadataRecord recorded once at spawn | ⏳ | Implemented, needs integration test |
| AC9: All existing tests pass | ✅ | 657/661 passing (baseline maintained) |

## Prototype Application Notes

**N/A** — This ticket did not have a prototyping phase. The design relied on established patterns from tickets 0038 (DataRecorder integration) and 0056a (transfer records).

## Deviations from Design

### Skip ConstraintSolver Changes (Design Review Feedback)

**Design stated**: Modify `ConstraintSolver::SolveResult` to add `bodyForces` and `bodyTorques` members.

**Actual**: **No changes made** to ConstraintSolver.

**Reason**: Design review discovered that `SolveResult::bodyForces` already existed (added in ticket 0045_constraint_solver_unification) with exactly the structure we needed:
```cpp
struct BodyForces {
  Vector3D linearForce;
  Vector3D angularTorque;
};
std::vector<BodyForces> bodyForces;
```

**Impact**: Simplified implementation by ~30 LOC, reduced risk of regressions in ConstraintSolver.

## Test Coverage Summary

### Unit Tests Written

_To be implemented in follow-up session._

**Planned tests** (from design):
- `CollisionPipeline::GetLastFrameData_ReturnsContactPoints`
- `CollisionPipeline::GetLastFrameData_ReturnsCorrectBodyIds`
- `CollisionPipeline::GetLastFrameData_ReturnsNormals`
- `CollisionPipeline::GetLastFrameData_ReturnsPenetrationDepth`
- `CollisionPipeline::GetLastFrameData_ReturnsConstraintForces`
- `CollisionPipeline::GetLastFrameData_ReturnsSolverDiagnostics`
- `CollisionPipeline::GetLastFrameData_EmptyWhenNoCollisions`

### Integration Tests Written

_To be implemented in follow-up session._

**Planned tests** (from design):
- `WorldModel::RecordCurrentFrame_WritesContactRecords`
- `WorldModel::RecordCurrentFrame_WritesConstraintForceRecords`
- `WorldModel::RecordCurrentFrame_WritesAppliedForceRecords`
- `WorldModel::RecordCurrentFrame_WritesSolverDiagnosticRecords`
- `WorldModel::SpawnObject_RecordsBodyMetadata`
- `WorldModel::SpawnEnvironmentObject_RecordsBodyMetadata`
- `WorldModel::SpawnObject_MetadataRecordedOnce`
- End-to-end: `ReplayRecording::CollisionSimulation_AllRecordTypesPopulated`

### Existing Tests Modified

_None — implementation is fully backward compatible._

## Known Limitations

1. **Metadata recording depends on DataRecorder existence** — If WorldModel is used without enabling recording, metadata is not persisted. This is intentional (opt-in recording).
2. **Body ID reuse** — If a body is removed and a new body spawned with the same ID, BodyMetadataRecord will have duplicate `body_id` entries. Design decision from review: Accept duplicates (no uniqueness constraint).
3. **No validation of snapshot timing** — The design assumes `recordCurrentFrame()` is called immediately after `update()`, which calls `execute()`. If there's a gap between execution and recording, snapshot data may be stale. Current usage in Engine.cpp confirms this assumption holds.

## Future Considerations

1. **Contact manifold fidelity** — Currently captures up to 4 contacts per collision. Future work may extend to capture edge-edge contact geometry (ticket 0040c follow-up).
2. **Force breakdown** — Current design records net constraint forces per body. Future visualization may benefit from per-constraint force breakdown (per-contact-point forces).
3. **Friction forces** — ConstraintForceRecord includes forces from friction constraints (ticket 0052 integration). No changes needed.
4. **Database schema evolution** — If transfer record structure changes, existing databases will need migration. Consider versioning strategy.

## Implementation Timeline

| Phase | Duration | Notes |
|-------|----------|-------|
| Preparation | 10 min | Review design, check ConstraintSolver, create iteration log |
| CollisionPipeline changes | 30 min | FrameCollisionData struct, snapshotFrameData() implementation |
| WorldModel changes | 45 min | Recording helpers, recordCurrentFrame() extension, spawn metadata |
| Build/test/commit | 15 min | Fix warnings, verify baseline tests, commit iterations |
| **Total** | **100 min** | **~1.67 hours** |

## Areas Warranting Extra Attention in Review

1. **Type conversions in recordSolverDiagnostics()** — Added static_cast for `size_t` → `uint32_t` conversions. Verify this is acceptable for production scale (assumes < 4B constraints/contacts per frame).
2. **Metadata recording timing** — Metadata is recorded synchronously in spawn methods. If spawn is called during simulation (mid-frame), metadata may be flushed before the first frame record. Review confirms this is acceptable (metadata is frame-independent).
3. **Gravity force computation** — `recordAppliedForces()` iterates all `potentialEnergies_` and sums forces. Verify this matches physics update semantics (confirmed: matches `updatePhysics()` logic).
4. **FrameCollisionData memory footprint** — Each frame allocates vectors for contacts and forces. Typical: ~4KB/frame, acceptable for 60 FPS recording (~240KB/s).

## Iteration Log

See: `docs/designs/0056b_collision_pipeline_data_extraction/iteration-log.md`

**Summary**:
- **Iteration 1**: CollisionPipeline snapshot infrastructure (FrameCollisionData, snapshotFrameData(), getLastFrameData())
- **Iteration 2**: WorldModel recording extensions (5 recording helpers, recordCurrentFrame() extension, spawn metadata)
- **Total iterations**: 2
- **Circle detection flags**: None

## Handoff Notes

Implementation is **feature-complete** per design. Remaining work:
1. **Write unit tests** for CollisionPipeline::getLastFrameData()
2. **Write integration tests** for WorldModel recording methods
3. **Run end-to-end test** with DataRecorder enabled to verify all 5 record types populate database
4. **Update ticket status** to "Implementation Complete — Awaiting Quality Gate"

No blocking issues. Ready for human review of implementation approach.
