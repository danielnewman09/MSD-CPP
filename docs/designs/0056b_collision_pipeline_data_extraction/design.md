# Design: CollisionPipeline Data Extraction & Extended Recording

## Summary

Collision data (contact points, normals, constraint forces, solver diagnostics) is ephemeral—created and destroyed within `CollisionPipeline::execute()` each frame. This design adds a snapshot mechanism to capture that data before it is cleared, then extends `WorldModel::recordCurrentFrame()` to persist it using the 5 new transfer record types from ticket 0056a.

The key design principle is **separation of concerns**: CollisionPipeline captures data into a struct but knows nothing about DataRecorder. WorldModel reads the struct and writes records.

## Architecture Changes

### PlantUML Diagram
See: `./0056b_collision_pipeline_data_extraction.puml`

### New Components

#### FrameCollisionData (Nested Structs)

- **Purpose**: Snapshot struct to capture ephemeral collision pipeline output for a single frame
- **Header location**: `msd-sim/src/Physics/Collision/CollisionPipeline.hpp` (nested within CollisionPipeline)
- **Key interfaces**:
  ```cpp
  struct FrameCollisionData
  {
    struct ContactData
    {
      uint32_t bodyAId{0};
      uint32_t bodyBId{0};
      Coordinate pointA{};
      Coordinate pointB{};
      Coordinate normal{};
      double depth{std::numeric_limits<double>::quiet_NaN()};
      double restitution{std::numeric_limits<double>::quiet_NaN()};
      double friction{std::numeric_limits<double>::quiet_NaN()};
      uint32_t contactIndex{0};
    };
    std::vector<ContactData> contacts;

    struct BodyForceData
    {
      uint32_t bodyId{0};
      Vector3D linearForce{};
      Vector3D angularTorque{};
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
- **Dependencies**:
  - `Coordinate` (from Environment module) — for contact point positions and normal vectors
  - `Vector3D` (Eigen) — for force/torque vectors
  - `std::vector` — for variable-length contact and force data
- **Thread safety**: Not thread-safe (single-threaded simulation)
- **Error handling**: No validation (data is snapshot of internal state)
- **Memory management**: Value type with STL containers, no dynamic allocation beyond vector storage
- **Ownership**: Owned by CollisionPipeline as member variable `lastFrameData_`

#### WorldModel Private Methods (Recording Helpers)

- **Purpose**: Decompose `recordCurrentFrame()` into focused recording methods for each record type
- **Header location**: `msd-sim/src/Environment/WorldModel.hpp`
- **Source location**: `msd-sim/src/Environment/WorldModel.cpp`
- **Key interfaces**:
  ```cpp
  class WorldModel {
  private:
    void recordBodyMetadata(uint32_t bodyId,
                            uint32_t assetId,
                            double mass,
                            double restitution,
                            double friction,
                            bool isEnvironment);

    void recordContacts(uint32_t frameId,
                        const FrameCollisionData& frameData);

    void recordConstraintForces(uint32_t frameId,
                                const FrameCollisionData& frameData);

    void recordAppliedForces(uint32_t frameId);

    void recordSolverDiagnostics(uint32_t frameId,
                                 const FrameCollisionData& frameData);
  };
  ```
- **Dependencies**:
  - `DataRecorder` — for `getDAO<T>()` access
  - `FrameCollisionData` — for collision/force snapshot data
  - `PotentialEnergy` — for computing gravity forces
  - Transfer record types (ContactRecord, etc.)
- **Thread safety**: Not thread-safe (single-threaded simulation)
- **Error handling**: None (assumes DataRecorder handles DB errors)

### Modified Components

#### CollisionPipeline

- **Current location**: `msd-sim/src/Physics/Collision/CollisionPipeline.hpp/.cpp`
- **Changes required**:
  1. Add `FrameCollisionData lastFrameData_` member variable
  2. Add `const FrameCollisionData& getLastFrameData() const` accessor
  3. In `execute()`, after Phase 5 (force application) and before `clearFrameData()`:
     - Snapshot `collisions_` data into `lastFrameData_.contacts`
     - Extract per-body constraint forces from `SolveResult` into `lastFrameData_.constraintForces`
     - Capture solver diagnostics into `lastFrameData_.solverData`
  4. Clear `lastFrameData_` at the start of `execute()` (before collision detection)
- **Backward compatibility**: Fully backward compatible—new accessor is opt-in, no changes to existing interfaces

#### ConstraintSolver::SolveResult

- **Current location**: `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`
- **Changes required**:
  1. Add `std::vector<Vector3D> bodyForces` member (per-body net linear force)
  2. Add `std::vector<Vector3D> bodyTorques` member (per-body net angular torque)
  3. Modify `ConstraintSolver::solve()` to populate these fields during constraint force accumulation
- **Backward compatibility**: Fully backward compatible—existing code ignores the new fields

#### WorldModel::recordCurrentFrame

- **Current location**: `msd-sim/src/Environment/WorldModel.cpp`
- **Changes required**:
  1. After recording `InertialStateRecord` for each asset, read `collisionPipeline_.getLastFrameData()`
  2. Call `recordContacts(frameId, frameData)`
  3. Call `recordConstraintForces(frameId, frameData)`
  4. Call `recordAppliedForces(frameId)`
  5. Call `recordSolverDiagnostics(frameId, frameData)`
- **Backward compatibility**: Fully backward compatible—extended functionality, no changes to existing recording logic

#### WorldModel::spawnObject / spawnEnvironmentObject

- **Current location**: `msd-sim/src/Environment/WorldModel.cpp`
- **Changes required**:
  1. After creating `AssetInertial` or `AssetEnvironment`, check `if (dataRecorder_)`
  2. If recording active, call `recordBodyMetadata()` with body properties
- **Backward compatibility**: Fully backward compatible—metadata recording only occurs if DataRecorder is enabled

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| `FrameCollisionData` | `CollisionPipeline` | Member variable | Owned by pipeline, populated in `execute()`, cleared each frame |
| `FrameCollisionData` | `WorldModel` | Read-only access | `getLastFrameData()` accessor provides snapshot for recording |
| `FrameCollisionData::ContactData` | `CollisionPipeline::CollisionPair` | Data extraction | Iterate `collisions_` to populate `contacts` vector |
| `FrameCollisionData::BodyForceData` | `ConstraintSolver::SolveResult` | Data extraction | Extract per-body forces from solver result |
| `FrameCollisionData::SolverData` | `ConstraintSolver::SolveResult` | Data extraction | Copy solver diagnostics (iterations, residual, convergence) |
| `WorldModel::recordContacts` | `DataRecorder::getDAO<ContactRecord>()` | DAO write | One record per contact point |
| `WorldModel::recordConstraintForces` | `DataRecorder::getDAO<ConstraintForceRecord>()` | DAO write | One record per body with non-zero constraint forces |
| `WorldModel::recordAppliedForces` | `DataRecorder::getDAO<AppliedForceRecord>()` | DAO write | One record per body (gravity from `potentialEnergies_`) |
| `WorldModel::recordSolverDiagnostics` | `DataRecorder::getDAO<SolverDiagnosticRecord>()` | DAO write | One record per frame |
| `WorldModel::recordBodyMetadata` | `DataRecorder::getDAO<BodyMetadataRecord>()` | DAO write | One record per body at spawn time |

## Implementation Details

### Snapshot Timing in CollisionPipeline::execute()

The snapshot must occur **after Phase 5 (force application)** and **before clearFrameData()**:

```cpp
void CollisionPipeline::execute(...) {
  clearFrameData();  // Start of frame: clear previous snapshot

  // Phase 1-4: Collision detection, constraint creation, solver input, solving
  // ...

  // Phase 5: Force application
  applyForces(inertialAssets, solveResult);

  // NEW: Phase 5.5: Snapshot before clearFrameData()
  snapshotFrameData(solveResult);

  clearFrameData();  // Clear internal collision/constraint data
}
```

**Rationale**: After force application, all collision data (contacts, forces, solver state) is valid but about to be cleared. The snapshot preserves it for WorldModel to record.

### ConstraintSolver Force Extraction

The solver must accumulate per-body net forces during constraint solving:

```cpp
ConstraintSolver::SolveResult ConstraintSolver::solve(...) {
  SolveResult result;
  result.bodyForces.resize(numBodies, Vector3D{0, 0, 0});
  result.bodyTorques.resize(numBodies, Vector3D{0, 0, 0});

  // During constraint force accumulation:
  for (each constraint) {
    Vector3D force = lambda * constraint.getJacobian(...);
    result.bodyForces[bodyIndex] += force;
    result.bodyTorques[bodyIndex] += torque;
  }

  return result;
}
```

### AppliedForceRecord Gravity Extraction

Gravity forces are computed from `potentialEnergies_` member:

```cpp
void WorldModel::recordAppliedForces(uint32_t frameId) {
  for (const auto& asset : inertialAssets_) {
    Vector3D gravityForce{0, 0, 0};
    Vector3D gravityTorque{0, 0, 0};

    for (const auto& potential : potentialEnergies_) {
      gravityForce += potential->computeForce(asset.getState(), asset.getMass());
      gravityTorque += potential->computeTorque(asset.getState(), asset.getInertiaTensor());
    }

    AppliedForceRecord record{};
    record.body_id = asset.getInstanceId();
    record.force_type = 0;  // Gravity
    record.force_x = gravityForce.x();
    record.force_y = gravityForce.y();
    record.force_z = gravityForce.z();
    record.torque_x = gravityTorque.x();
    record.torque_y = gravityTorque.y();
    record.torque_z = gravityTorque.z();
    // point_x/y/z remains NaN (gravity acts at center of mass)
    record.frame.id = frameId;

    dataRecorder_->getDAO<AppliedForceRecord>().addToBuffer(record);
  }
}
```

### BodyMetadataRecord Spawn-Time Recording

Metadata is recorded once at spawn, not per frame:

```cpp
const AssetInertial& WorldModel::spawnObject(...) {
  // ... existing spawn logic ...
  uint32_t instanceId = asset.getInstanceId();

  // NEW: Record metadata if recording active
  if (dataRecorder_) {
    recordBodyMetadata(instanceId, assetId, mass, restitution, friction, false);
  }

  return asset;
}

void WorldModel::recordBodyMetadata(uint32_t bodyId, ...) {
  BodyMetadataRecord record{};
  record.body_id = bodyId;
  record.asset_id = assetId;
  record.mass = mass;
  record.restitution = restitution;
  record.friction = friction;
  record.is_environment = isEnvironment ? 1 : 0;
  // No frame FK—recorded once, not per-frame

  dataRecorder_->getDAO<BodyMetadataRecord>().addToBuffer(record);
}
```

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `CollisionPipelineTest.cpp` | All existing tests | None | No changes (new accessor is unused) |
| `ConstraintSolverTest.cpp` | Tests of `solve()` return value | None | Ignore new `bodyForces`/`bodyTorques` fields |
| `WorldModelTest.cpp` | Tests of recording | None | Extended functionality, existing tests unaffected |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| `CollisionPipeline` | `GetLastFrameData_ReturnsContactPoints` | Snapshot captures contact point positions |
| `CollisionPipeline` | `GetLastFrameData_ReturnsCorrectBodyIds` | Body IDs match collision pair |
| `CollisionPipeline` | `GetLastFrameData_ReturnsNormals` | Contact normals captured correctly |
| `CollisionPipeline` | `GetLastFrameData_ReturnsPenetrationDepth` | Penetration depth matches EPA result |
| `CollisionPipeline` | `GetLastFrameData_ReturnsConstraintForces` | Per-body forces extracted from solver |
| `CollisionPipeline` | `GetLastFrameData_ReturnsSolverDiagnostics` | Iterations, residual, convergence captured |
| `CollisionPipeline` | `GetLastFrameData_EmptyWhenNoCollisions` | No data when no collisions occurred |
| `CollisionPipeline` | `GetLastFrameData_ClearedNextFrame` | Snapshot cleared at start of next `execute()` |
| `ConstraintSolver` | `SolveResult_BodyForcesPopulated` | `bodyForces` vector has correct size and values |
| `ConstraintSolver` | `SolveResult_BodyTorquesPopulated` | `bodyTorques` vector has correct size and values |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `WorldModel::RecordCurrentFrame_WritesContactRecords` | WorldModel, DataRecorder, CollisionPipeline | ContactRecord written for each contact point |
| `WorldModel::RecordCurrentFrame_WritesConstraintForceRecords` | WorldModel, DataRecorder, CollisionPipeline | ConstraintForceRecord written for each body with forces |
| `WorldModel::RecordCurrentFrame_WritesAppliedForceRecords` | WorldModel, DataRecorder | AppliedForceRecord written for gravity on each body |
| `WorldModel::RecordCurrentFrame_WritesSolverDiagnosticRecords` | WorldModel, DataRecorder, CollisionPipeline | SolverDiagnosticRecord written per frame |
| `WorldModel::SpawnObject_RecordsBodyMetadata` | WorldModel, DataRecorder | BodyMetadataRecord written on spawn |
| `WorldModel::SpawnEnvironmentObject_RecordsBodyMetadata` | WorldModel, DataRecorder | BodyMetadataRecord written for environment objects |
| `WorldModel::SpawnObject_MetadataRecordedOnce` | WorldModel, DataRecorder | Metadata NOT duplicated per frame |
| `ReplayRecording::CollisionSimulation_AllRecordTypesPopulated` | End-to-end | All 5 record types present in DB after simulation |

#### Benchmark Tests

Not applicable—this feature adds data capture, not performance-critical operations.

## Open Questions

### Design Decisions (Human Input Needed)

1. **Force extraction granularity**
   - Option A: Per-body net forces only (current design) — Simpler, sufficient for net force arrows in UI
   - Option B: Per-constraint forces with body pair tracking — Enables force arrow per contact point, more complex
   - Recommendation: **Option A** for initial implementation. Option B can be added later if needed for advanced visualization.

2. **AppliedForceRecord force_type field**
   - Option A: Use enum with named constants (GRAVITY=0, EXTERNAL=1, POTENTIAL=2) in transfer record
   - Option B: Use raw uint32_t, document values in transfer record comment
   - Recommendation: **Option B** for consistency with existing transfer record pattern (BodyMetadataRecord uses raw uint32_t for booleans). Frontend can define enum from values.

3. **Contact point storage format**
   - Option A: Flat fields (point_a_x/y/z, point_b_x/y/z, normal_x/y/z) as designed
   - Option B: Foreign key to separate CoordinateRecord table (normalized)
   - Recommendation: **Option A** (already in 0056a design). Each ContactRecord has 3 vectors (pointA, pointB, normal)—FK approach would create 3 FK references per contact, complex queries.

### Prototype Required

None—design follows established patterns (DataRecorder DAO usage, transfer record structure).

### Requirements Clarification

1. **Snapshot preservation across frames**
   - Current design: `lastFrameData_` is cleared at start of each `execute()`, so data is valid from end of frame N until start of frame N+1
   - Question: Is this sufficient, or should `lastFrameData_` persist until explicitly overwritten (no clear)?
   - Impact: If WorldModel calls `collisionPipeline_.execute()` then immediately `getLastFrameData()`, current design works. If there's a gap between execution and recording, data may be stale.
   - Clarification needed: Confirm `recordCurrentFrame()` is called immediately after `update()` (which calls `execute()`).

2. **BodyMetadataRecord uniqueness**
   - Current design: Metadata recorded once at spawn
   - Question: What happens if a body is removed and a new body spawned with the same ID?
   - Impact: Database would have duplicate `body_id` entries if IDs are reused
   - Clarification needed: Does `body_id` monotonically increase, or can IDs be recycled?
   - Mitigation: Add `PRIMARY KEY(body_id)` constraint to BodyMetadataRecord table? Or accept duplicates?

## Performance Considerations

### Snapshot Cost

- **Contact data**: 9 doubles + 4 uint32_t per contact × 4 contacts/collision × N collisions
  - Typical: 10 collisions/frame → ~1.5 KB snapshot data
- **Force data**: 6 doubles per body × N bodies
  - Typical: 50 bodies → 2.4 KB snapshot data
- **Total**: ~4 KB/frame typical, negligible memory footprint

### Recording Cost

- **DAO buffering**: < 1 μs per record (in-memory append)
- **Periodic flush**: ~100ms every 100ms (background thread, non-blocking)
- **Simulation impact**: < 5% overhead typical (validated by ticket 0038 prototypes)

### Scalability

- **Contact records**: Up to 4 contacts/collision × N collisions/frame
  - Example: 100 collisions/frame = 400 ContactRecords/frame @ 60 FPS = 24,000 records/sec
  - Database flush throughput: > 10,000 records/sec (ticket 0038)
  - Mitigation: Reduce flush interval if needed, or batch fewer frames per flush

## Coding Standards Applied

- **Initialization**: All struct members use brace initialization with NaN for uninitialized floats
- **Naming**: `PascalCase` for structs, `camelCase` for methods, `snake_case_` for transfer record fields
- **Memory**: Value semantics for snapshot structs, no dynamic allocation except STL containers
- **Return values**: `getLastFrameData()` returns const reference (read-only access)
- **Separation of concerns**: CollisionPipeline captures, WorldModel records—no coupling to DataRecorder in pipeline

## Revision History

None—initial design.
