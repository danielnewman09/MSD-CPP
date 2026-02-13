# Design: Contact Tangent Vector Recording

## Summary

Extend the simulation recording system to capture and persist contact tangent vectors (t1, t2) from the constraint solver, enabling visualization of the tangent basis alongside collision normals in the Three.js replay viewer. This design adds two `Vector3DRecord` fields to `CollisionResultRecord`, populates them by extracting tangents from `FrictionConstraint` instances after constraint creation, and threads the data through the REST API and frontend rendering pipeline.

## Architecture Changes

### PlantUML Diagram
See: `./contact-tangent-recording.puml`

### New Components

None. This design extends existing components to expose and persist tangent data already computed by `FrictionConstraint`.

### Modified Components

#### CollisionResultRecord (msd-transfer)
- **Current location**: `msd/msd-transfer/src/CollisionResultRecord.hpp`
- **Changes required**:
  - Add `Vector3DRecord tangent1` field
  - Add `Vector3DRecord tangent2` field
  - Update `BOOST_DESCRIBE_STRUCT` macro to include new fields
- **Backward compatibility**:
  - Fields are optional in pybind (default to zero vectors if missing)
  - Older recordings without tangent fields load without error
  - No schema migration needed (cpp_sqlite handles missing columns gracefully)

```cpp
struct CollisionResultRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_a_id{0};
  uint32_t body_b_id{0};

  Vector3DRecord normal;
  double penetrationDepth{std::numeric_limits<double>::quiet_NaN()};

  // NEW: Tangent basis for friction visualization
  Vector3DRecord tangent1;  // First tangent direction (unit length)
  Vector3DRecord tangent2;  // Second tangent direction (unit length)

  cpp_sqlite::RepeatedFieldTransferObject<ContactPointRecord> contacts;
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

BOOST_DESCRIBE_STRUCT(CollisionResultRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_a_id,
                       body_b_id,
                       normal,
                       penetrationDepth,
                       tangent1,  // NEW
                       tangent2,  // NEW
                       contacts,
                       frame));
```

#### CollisionResult (msd-sim)
- **Current location**: `msd/msd-sim/src/Physics/Collision/CollisionResult.hpp`
- **Changes required**:
  - Add `Coordinate tangent1` member (default: zero vector)
  - Add `Coordinate tangent2` member (default: zero vector)
  - Update `toRecord()` to serialize tangent fields
  - Update `fromRecord()` to deserialize tangent fields (with fallback to zero if missing)
- **Backward compatibility**:
  - Default-initialized tangents remain zero vectors if not populated
  - Existing collision detection code (GJK/EPA) unaffected
  - Only collision responses with friction will populate tangents

```cpp
struct CollisionResult
{
  Coordinate normal;
  double penetrationDepth{std::numeric_limits<double>::quiet_NaN()};

  std::array<ContactPoint, 4> contacts;
  size_t contactCount{0};

  // NEW: Tangent basis extracted from FrictionConstraint (if friction enabled)
  Coordinate tangent1{};  // First tangent direction (zero if no friction)
  Coordinate tangent2{};  // Second tangent direction (zero if no friction)

  [[nodiscard]] msd_transfer::CollisionResultRecord toRecord(
    uint32_t bodyAId,
    uint32_t bodyBId) const
  {
    msd_transfer::CollisionResultRecord record;
    record.body_a_id = bodyAId;
    record.body_b_id = bodyBId;
    record.normal = Vector3D{normal.x(), normal.y(), normal.z()}.toRecord();
    record.penetrationDepth = penetrationDepth;

    // NEW: Serialize tangent basis
    record.tangent1 = Vector3D{tangent1.x(), tangent1.y(), tangent1.z()}.toRecord();
    record.tangent2 = Vector3D{tangent2.x(), tangent2.y(), tangent2.z()}.toRecord();

    for (size_t i = 0; i < contactCount; ++i)
    {
      record.contacts.data.push_back(contacts[i].toRecord());
    }
    return record;
  }
};
```

#### CollisionPipeline (msd-sim)
- **Current location**: `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp/.cpp`
- **Changes required**:
  - Add private method `extractTangentsFromConstraints()` called after `createConstraints()`
  - Iterate `frictionConstraints_` vector, match each constraint to corresponding `CollisionPair` by body indices
  - Populate `CollisionPair::result.tangent1` and `tangent2` from `FrictionConstraint::getTangent1/2()`
- **Backward compatibility**:
  - Frictionless collisions leave tangents as zero vectors
  - Existing collision pipeline phases unaffected
  - No performance impact (tangent extraction is O(n) where n = collision count)

```cpp
class CollisionPipeline
{
protected:
  void createConstraints(std::span<AssetInertial> inertialAssets,
                         std::span<const AssetEnvironment> environmentalAssets,
                         double dt);

private:
  // NEW: Extract tangent basis from friction constraints for recording
  void extractTangentsFromConstraints();

  std::vector<CollisionPair> collisions_;
  std::vector<std::unique_ptr<FrictionConstraint>> frictionConstraints_;
};
```

Implementation in `CollisionPipeline::execute()`:
```cpp
void CollisionPipeline::execute(/* ... */) {
  // ... existing phases ...
  createConstraints(inertialAssets, environmentalAssets, dt);

  // NEW: Extract tangent data from friction constraints for recording
  extractTangentsFromConstraints();

  // ... solver, forces, position correction ...
}

void CollisionPipeline::extractTangentsFromConstraints() {
  // For each friction constraint, find the matching collision pair
  // Friction constraints are created in the same order as collisions
  // Each collision produces one friction constraint (if friction > 0)

  size_t frictionIdx = 0;
  for (auto& collision : collisions_) {
    if (collision.frictionCoefficient > 0.0 && frictionIdx < frictionConstraints_.size()) {
      const auto& constraint = frictionConstraints_[frictionIdx];

      // Verify body indices match (safety check)
      if (constraint->bodyAIndex() == collision.bodyAIndex &&
          constraint->bodyBIndex() == collision.bodyBIndex) {
        collision.result.tangent1 = constraint->getTangent1();
        collision.result.tangent2 = constraint->getTangent2();
      }

      ++frictionIdx;
    }
  }
}
```

**Design rationale**:
- Extract tangents AFTER constraint creation to record the actual solver state (even if future changes make tangent selection non-deterministic)
- Match constraints to collisions by body indices for safety (handles reordering if broadphase changes)
- Zero tangents for frictionless contacts are valid (visualized as zero-length arrows, effectively invisible)

#### DataRecorder (msd-sim)
- **Current location**: `msd/msd-sim/src/DataRecorder/DataRecorder.cpp`
- **Changes required**: None (automatically picks up new `CollisionResult::toRecord()` behavior)
- **Backward compatibility**: `recordCollisions()` already calls `pair.result.toRecord()`; new tangent fields serialize automatically

#### Python Bindings (msd-pybind)
- **Current location**: `msd/msd-pybind/src/record_bindings.cpp`
- **Changes required**:
  - Add `.def_readwrite("tangent1", &CollisionResultRecord::tangent1, py::return_value_policy::reference_internal)`
  - Add `.def_readwrite("tangent2", &CollisionResultRecord::tangent2, py::return_value_policy::reference_internal)`
- **Backward compatibility**: Fields are optional (older recordings return default Vector3DRecord with zeros)

```cpp
py::class_<msd_transfer::CollisionResultRecord, msd_transfer::BaseTransferObject>(
  m, "CollisionResultRecord")
  .def(py::init<>())
  .def_readwrite("body_a_id", &msd_transfer::CollisionResultRecord::body_a_id)
  .def_readwrite("body_b_id", &msd_transfer::CollisionResultRecord::body_b_id)
  .def_readwrite("normal", &msd_transfer::CollisionResultRecord::normal)
  .def_readwrite("penetrationDepth", &msd_transfer::CollisionResultRecord::penetrationDepth)
  // NEW
  .def_readwrite("tangent1", &msd_transfer::CollisionResultRecord::tangent1,
                 py::return_value_policy::reference_internal)
  .def_readwrite("tangent2", &msd_transfer::CollisionResultRecord::tangent2,
                 py::return_value_policy::reference_internal)
  // ...
```

#### Replay Python Models (replay)
- **Current location**: `replay/replay/models.py`
- **Changes required**:
  - Add `tangent1: Optional[Vector3D] = None` to `Collision` model
  - Add `tangent2: Optional[Vector3D] = None` to `Collision` model
- **Backward compatibility**: Optional fields default to `None` for older recordings

```python
class Collision(BaseModel):
    body_a_id: int
    body_b_id: int
    normal: Vector3D
    penetration_depth: float
    contacts: List[ContactPoint]

    # NEW: Tangent basis for friction visualization
    tangent1: Optional[Vector3D] = None
    tangent2: Optional[Vector3D] = None
```

#### Replay SimulationService (replay)
- **Current location**: `replay/replay/services/simulation_service.py`
- **Changes required**:
  - Read `tangent1` and `tangent2` from `CollisionResultRecord` in `_build_collision_from_record()`
  - Pass tangent data to `Collision` model constructor
- **Backward compatibility**: Check if fields exist before reading (graceful fallback to `None`)

```python
def _build_collision_from_record(self, record: CollisionResultRecord) -> Collision:
    normal = Vector3D(x=record.normal.x, y=record.normal.y, z=record.normal.z)

    # NEW: Read tangent basis (backward compatible)
    tangent1 = None
    tangent2 = None
    if hasattr(record, 'tangent1') and record.tangent1:
        tangent1 = Vector3D(x=record.tangent1.x, y=record.tangent1.y, z=record.tangent1.z)
    if hasattr(record, 'tangent2') and record.tangent2:
        tangent2 = Vector3D(x=record.tangent2.x, y=record.tangent2.y, z=record.tangent2.z)

    contacts = [self._build_contact_from_record(c) for c in record.contacts.data]

    return Collision(
        body_a_id=record.body_a_id,
        body_b_id=record.body_b_id,
        normal=normal,
        penetration_depth=record.penetrationDepth,
        contacts=contacts,
        tangent1=tangent1,
        tangent2=tangent2
    )
```

#### Three.js ContactOverlay (replay/static)
- **Current location**: `replay/static/js/overlays/contacts.js` (will be created in ticket 0056f)
- **Changes required**:
  - Add `tangent1Arrows_` and `tangent2Arrows_` maps (similar to `normalArrows_`)
  - In `update()`, render tangent arrows if `collision.tangent1` and `tangent2` are present
  - Use green for tangent1, blue for tangent2 (red already reserved for normal)
  - Render at contact midpoint (same as normal arrow)
- **Backward compatibility**: Check for tangent field existence before rendering

```javascript
class ContactOverlay {
    constructor(scene) {
        this.scene = scene;
        this.normalArrows = new Map();   // Red arrows (existing)
        this.tangent1Arrows = new Map(); // Green arrows (NEW)
        this.tangent2Arrows = new Map(); // Blue arrows (NEW)
        this.enabled = false;
    }

    update(collisions) {
        if (!this.enabled) {
            this.clear();
            return;
        }

        // Clear previous frame arrows
        this.clear();

        collisions.forEach((collision, idx) => {
            const contactMidpoint = this.computeContactMidpoint(collision.contacts);

            // Render normal (red)
            const normalArrow = new THREE.ArrowHelper(
                new THREE.Vector3(collision.normal.x, collision.normal.y, collision.normal.z),
                contactMidpoint,
                1.0, // length
                0xff0000 // red
            );
            this.scene.add(normalArrow);
            this.normalArrows.set(`collision_${idx}`, normalArrow);

            // NEW: Render tangent1 (green) if present
            if (collision.tangent1) {
                const t1Arrow = new THREE.ArrowHelper(
                    new THREE.Vector3(collision.tangent1.x, collision.tangent1.y, collision.tangent1.z),
                    contactMidpoint,
                    1.0,
                    0x00ff00 // green
                );
                this.scene.add(t1Arrow);
                this.tangent1Arrows.set(`collision_${idx}_t1`, t1Arrow);
            }

            // NEW: Render tangent2 (blue) if present
            if (collision.tangent2) {
                const t2Arrow = new THREE.ArrowHelper(
                    new THREE.Vector3(collision.tangent2.x, collision.tangent2.y, collision.tangent2.z),
                    contactMidpoint,
                    1.0,
                    0x0000ff // blue
                );
                this.scene.add(t2Arrow);
                this.tangent2Arrows.set(`collision_${idx}_t2`, t2Arrow);
            }
        });
    }

    clear() {
        this.normalArrows.forEach(arrow => this.scene.remove(arrow));
        this.tangent1Arrows.forEach(arrow => this.scene.remove(arrow));
        this.tangent2Arrows.forEach(arrow => this.scene.remove(arrow));

        this.normalArrows.clear();
        this.tangent1Arrows.clear();
        this.tangent2Arrows.clear();
    }
}
```

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| CollisionResult.tangent1/tangent2 | CollisionResultRecord | Serialization | toRecord() populates transfer fields |
| CollisionPipeline::extractTangentsFromConstraints() | FrictionConstraint | Data extraction | Reads getTangent1/2() after constraint creation |
| DataRecorder::recordCollisions() | CollisionResult::toRecord() | Automatic | No changes needed, uses updated toRecord() |
| Python Collision model | CollisionResultRecord | Binding | Optional fields with None fallback |
| ContactOverlay | Three.js scene | Rendering | Conditional arrow creation if tangents present |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `msd-sim/test/Physics/Collision/CollisionResultTest.cpp` | toRecord/fromRecord round-trip | Updated assertions | Add assertions for tangent1/tangent2 fields |
| `replay/tests/test_simulation_service.py` | Frame state serialization | Schema validation | Verify tangent fields present in JSON response |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| `CollisionResult` | `toRecord_WithTangents_SerializesProperly` | Tangent vectors serialize to Vector3DRecord correctly |
| `CollisionResult` | `fromRecord_WithoutTangents_DefaultsToZero` | Backward compatibility: missing tangents default to zero |
| `CollisionPipeline` | `extractTangentsFromConstraints_MatchesByBodyIndices` | Tangent data correctly associated with collision pairs |
| `CollisionPipeline` | `extractTangentsFromConstraints_NoFriction_LeavesZeros` | Frictionless collisions keep zero tangents |
| `TangentBasis` | `computeTangentBasis_Determinism` | Same normal produces same tangent basis every time |
| `TangentBasis` | `computeTangentBasis_Orthogonality` | t1·t2 = 0, t1·n = 0, t2·n = 0 within tolerance |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `generate_test_recording` with friction | DataRecorder, CollisionPipeline, CollisionResult | Tangent vectors present in SQLite recording |
| REST API `/api/v1/simulations/{id}/frames/{frame_id}/state` | SimulationService, Python bindings | Tangent data appears in JSON response |
| Tangent vector orthogonality check | CollisionResult | Tangents are unit length, mutually orthogonal, perpendicular to normal |

#### Manual Tests

| Test Case | Steps | Expected Behavior |
|-----------|-------|-------------------|
| Replay viewer tangent arrows | 1. Generate recording with friction scenario<br>2. Open replay viewer<br>3. Navigate to collision frame<br>4. Enable contact overlay | Three orthogonal arrows visible: red (normal), green (t1), blue (t2) |
| Tangent arrows disappear on separation | 1. Navigate to frame after collision ends<br>2. Contact overlay still enabled | No arrows visible (objects separated) |
| Backward compatibility | 1. Open recording from before tangent fields existed<br>2. Navigate to collision frame | Normal arrows render, no errors (tangents absent) |

## Open Questions

### Design Decisions (Human Input Needed)

1. **Tangent arrow length scaling**
   - Option A: Fixed length (e.g., 1.0 units) — Consistent visual size, independent of collision magnitude
   - Option B: Scale by friction force magnitude — Shows friction force strength visually
   - Recommendation: Option A for MVP, Option B as enhancement (requires recording lambda values)

2. **Tangent visualization toggle**
   - Option A: Shared toggle with normal arrows — Single "Contact Overlay" button shows all 3 vectors
   - Option B: Separate toggles — "Normal", "Tangent1", "Tangent2" independently controlled
   - Recommendation: Option A for simplicity, Option B if users need to declutter view

### Prototype Required

None. All components are straightforward extensions of existing patterns:
- Transfer record extension follows `ContactPointRecord` precedent
- Tangent extraction mirrors existing constraint iteration patterns
- Three.js arrow rendering uses same `ArrowHelper` as normal arrows (ticket 0056f)

### Requirements Clarification

1. **Tangent data for environment collisions**: Should tangent vectors be recorded for inertial-environment collisions (where one body is static)?
   - Current assumption: Yes (FrictionConstraint is created for both inertial-inertial and inertial-environment pairs)

2. **Tangent data granularity**: Should each contact point in a manifold have its own tangent basis, or share one per collision?
   - Current assumption: Shared per collision (all contacts share the collision normal, hence same tangent basis)
   - Alternative: Per-contact tangents if future changes introduce varying normals within a manifold

## Implementation Notes

### Tangent Basis Determinism

The `TangentBasis::computeTangentBasis()` algorithm (Duff et al. 2017) guarantees:
- **Determinism**: Same normal always produces same t1, t2
- **Continuity**: Small change in normal produces small change in tangents
- **Orthogonality**: ||t1|| = ||t2|| = 1, t1·t2 = 0, t1·n = 0, t2·n = 0

This means tangents could be recomputed from the normal at read time rather than persisting them. However, persisting tangents:
- Records the actual solver state (if future changes make tangent selection non-deterministic)
- Avoids recomputation in Python/JavaScript (minor performance benefit)
- Simplifies debugging (tangents visible in raw database)

### SQLite Schema Evolution

cpp_sqlite handles missing columns gracefully:
- New `tangent1` and `tangent2` columns added to `CollisionResultRecord` table on first write
- Older recordings without these columns return default-initialized Vector3DRecord (all zeros)
- No manual schema migration needed

### Performance Impact

- **C++ overhead**: ~O(n) where n = collision count (one tangent extraction per collision)
- **Database overhead**: +48 bytes per collision record (2 × Vector3DRecord with 3 doubles each)
- **Rendering overhead**: +2 ArrowHelper instances per collision (minimal GPU impact)

Estimated total overhead: < 1% for typical scenarios (10-100 collisions per frame)

---

## Design Review -- Initial Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-02-12
**Status**: REVISION_REQUESTED
**Iteration**: 0 of 1

### Issues Requiring Revision

| ID | Issue | Category | Required Change |
|----|-------|----------|-----------------|
| I1 | `extractTangentsFromConstraints()` incorrectly assumes 1 friction constraint per collision | Feasibility | Fix matching logic to account for per-contact-point friction constraints |
| I2 | Python bindings use `def_readwrite` but existing record bindings use `def_readonly` | Architectural Fit | Use `def_readonly` to match existing pattern |
| I3 | Python model references `Vector3D` type but actual model class is `Vec3` | Architectural Fit | Use `Vec3` to match existing models.py |
| I4 | SimulationService design introduces a new `_build_collision_from_record()` method that does not exist | Architectural Fit | Show changes inline in the existing `get_frame_data()` method |
| I5 | `CollisionResult::fromRecord()` not updated to deserialize tangent fields | C++ Quality | Add tangent deserialization in `fromRecord()` |

### Revision Instructions for Architect

The following changes must be made before final review:

1. **Issue I1 -- Friction constraint per-contact-point matching logic is incorrect**: The design's `extractTangentsFromConstraints()` assumes one friction constraint per collision pair. However, examining `CollisionPipeline::createConstraints()` (lines 233-292 of `CollisionPipeline.cpp`), the code creates one `FrictionConstraint` per **contact point** in the manifold -- not per collision. The `constraints_` and `frictionConstraints_` vectors have a 1:1 correspondence (one friction constraint for each contact constraint). Since all friction constraints for a given collision share the same normal and thus the same tangent basis, the implementation should use `pairRanges_` (which tracks the constraint index range per collision pair) to extract tangents from the first friction constraint of each pair. The loop structure should be:

   ```cpp
   void CollisionPipeline::extractTangentsFromConstraints() {
     if (frictionConstraints_.empty()) return;
     for (const auto& range : pairRanges_) {
       auto& collision = collisions_[range.pairIdx];
       // All friction constraints for this pair share the same tangent basis
       // (same normal), so take from the first one in the range
       const auto& fc = frictionConstraints_[range.startIdx];
       collision.result.tangent1 = fc->getTangent1();
       collision.result.tangent2 = fc->getTangent2();
     }
   }
   ```

   Update the design document to reflect this corrected logic, including the prose description that says "Each collision produces one friction constraint" -- it should say "Each contact point produces one friction constraint; all share the same tangent basis per collision."

2. **Issue I2 -- Python binding access mode**: The existing `CollisionResultRecord` bindings in `record_bindings.cpp` (line 180-196) use `def_readonly` for all fields. The design proposes `def_readwrite` for tangent1/tangent2. Use `def_readonly` to be consistent with the existing pattern. Also, `py::return_value_policy::reference_internal` is not needed with `def_readonly` for value-type members (Vector3DRecord is a value type, not a pointer/reference). Remove the return_value_policy parameter.

3. **Issue I3 -- Python model type name mismatch**: The design references `Vector3D` and `Collision` in the Python model code, but the actual model classes in `replay/replay/models.py` are named `Vec3` and `CollisionInfo`. Update the design to use the correct class names: `tangent1: Vec3 | None = None` and `tangent2: Vec3 | None = None` on the `CollisionInfo` model.

4. **Issue I4 -- SimulationService integration approach**: The design shows a new `_build_collision_from_record()` method that does not exist in the current `SimulationService`. The actual collision building happens inline in `get_frame_data()` (lines 109-138 of `simulation_service.py`). Revise to show the tangent extraction within the existing inline list comprehension or as a minimal modification to the existing code block, not as a new helper method.

5. **Issue I5 -- `fromRecord()` tangent deserialization missing**: The design mentions updating `fromRecord()` but does not show the implementation. Add the deserialization code in `CollisionResult::fromRecord()` to read tangent1 and tangent2 from the record, with graceful fallback to zero if the fields contain NaN (indicating an older recording):

   ```cpp
   // In fromRecord():
   result.tangent1 = Coordinate{record.tangent1.x, record.tangent1.y, record.tangent1.z};
   result.tangent2 = Coordinate{record.tangent2.x, record.tangent2.y, record.tangent2.z};
   ```

### Items Passing Review (No Changes Needed)

The following aspects of the design are well-done and should not be modified:

- **CollisionResultRecord extension**: Adding `Vector3DRecord tangent1` and `tangent2` fields with BOOST_DESCRIBE_STRUCT is the correct approach, follows `ContactPointRecord` precedent, and maintains backward compatibility through cpp_sqlite's missing column handling.
- **CollisionResult tangent member placement**: Adding tangent fields to `CollisionResult` (geometry-level) rather than `CollisionPair` (pipeline-level) is architecturally sound -- it keeps serialization self-contained via `toRecord()`.
- **Placement in `execute()` flow**: Extracting tangents after `createConstraints()` but before `clearEphemeralState()` is correct timing, as friction constraints are destroyed during `clearEphemeralState()`.
- **DataRecorder zero-change approach**: The design correctly identifies that `DataRecorder::recordCollisions()` automatically picks up the new fields via `pair.result.toRecord()` with no code changes required.
- **Three.js ContactOverlay design**: Color scheme (red/green/blue for normal/t1/t2), fixed 1.0 unit length, single toggle approach, and conditional rendering are all appropriate.
- **Backward compatibility strategy**: Zero-vector defaults for missing tangents, optional Python fields, and conditional frontend rendering form a sound backward compatibility chain.
- **Test plan**: Coverage of serialization round-trip, pipeline extraction, determinism, and orthogonality is thorough.
- **Decision to persist rather than recompute**: Recording actual solver state is the right trade-off for debuggability and forward compatibility.

---
