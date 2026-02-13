# Design: Constraint State Recording System

## Summary

Design a general-purpose constraint state recording system where each concrete constraint type can persist its own solver state to the simulation database. This enables recording of friction tangent vectors for visualization (the immediate goal) while establishing an extensible pattern for future constraint types (joints, motors, limits). Each constraint type defines its own transfer record and implements state serialization, decoupling constraint logic from recording infrastructure.

## Architecture Changes

### PlantUML Diagram
See: `./contact-tangent-recording.puml`

### New Components

#### ConstraintRecord (msd-transfer)
- **Location**: `msd/msd-transfer/src/ConstraintRecord.hpp`
- **Purpose**: Base transfer record for constraint polymorphic persistence
- **Type**: Struct inheriting `BaseTransferObject`
- **Key interfaces**:
  ```cpp
  struct ConstraintRecord : public cpp_sqlite::BaseTransferObject
  {
    uint32_t body_a_id{0};
    uint32_t body_b_id{0};
    std::string constraint_type;  // Type identifier for polymorphic reconstruction
    cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
  };

  BOOST_DESCRIBE_STRUCT(ConstraintRecord,
                        (cpp_sqlite::BaseTransferObject),
                        (body_a_id, body_b_id, constraint_type, frame));
  ```
- **Design rationale**: Provides common FK fields (frame, body IDs) and type identifier for polymorphic deserialization. Future constraint records will inherit from this or follow the same pattern (separate table per constraint type for schema flexibility).

#### FrictionConstraintRecord (msd-transfer)
- **Location**: `msd/msd-transfer/src/FrictionConstraintRecord.hpp`
- **Purpose**: Friction constraint state persistence (tangents, lambda values, geometry)
- **Type**: Struct inheriting `BaseTransferObject`
- **Key interfaces**:
  ```cpp
  struct FrictionConstraintRecord : public cpp_sqlite::BaseTransferObject
  {
    uint32_t body_a_id{0};
    uint32_t body_b_id{0};

    // Contact geometry
    Vector3DRecord normal;       // Contact normal (world space)
    Vector3DRecord tangent1;     // First tangent direction (world space, unit length)
    Vector3DRecord tangent2;     // Second tangent direction (world space, unit length)
    Vector3DRecord lever_arm_a;  // Lever arm from CoM to contact (body A)
    Vector3DRecord lever_arm_b;  // Lever arm from CoM to contact (body B)

    // Constraint parameters
    double friction_coefficient{std::numeric_limits<double>::quiet_NaN()};
    double normal_lambda{std::numeric_limits<double>::quiet_NaN()};  // Normal force magnitude

    cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
  };

  BOOST_DESCRIBE_STRUCT(FrictionConstraintRecord,
                        (cpp_sqlite::BaseTransferObject),
                        (body_a_id, body_b_id,
                         normal, tangent1, tangent2,
                         lever_arm_a, lever_arm_b,
                         friction_coefficient, normal_lambda,
                         frame));
  ```
- **Design rationale**:
  - Stores complete friction constraint state for debugging and visualization
  - `tangent1` and `tangent2` are `Vector3DRecord` (not `Coordinate`) — they are directions, not points (avoids `globalToLocal` overload bug)
  - Includes `normal_lambda` for force-magnitude scaling in future visualization enhancements
  - Separate table from `ContactConstraintRecord` for schema flexibility (each constraint type has different state)
  - All FK relationships use existing `SimulationFrameRecord` pattern

#### ContactConstraintRecord (msd-transfer)
- **Location**: `msd/msd-transfer/src/ContactConstraintRecord.hpp`
- **Purpose**: Contact constraint state persistence (normal, penetration, restitution)
- **Type**: Struct inheriting `BaseTransferObject`
- **Key interfaces**:
  ```cpp
  struct ContactConstraintRecord : public cpp_sqlite::BaseTransferObject
  {
    uint32_t body_a_id{0};
    uint32_t body_b_id{0};

    // Contact geometry
    Vector3DRecord normal;      // Contact normal (world space)
    Vector3DRecord lever_arm_a; // Lever arm from CoM to contact (body A)
    Vector3DRecord lever_arm_b; // Lever arm from CoM to contact (body B)
    double penetration_depth{std::numeric_limits<double>::quiet_NaN()};

    // Constraint parameters
    double restitution{std::numeric_limits<double>::quiet_NaN()};
    double pre_impact_rel_vel_normal{std::numeric_limits<double>::quiet_NaN()};

    cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
  };

  BOOST_DESCRIBE_STRUCT(ContactConstraintRecord,
                        (cpp_sqlite::BaseTransferObject),
                        (body_a_id, body_b_id,
                         normal, lever_arm_a, lever_arm_b, penetration_depth,
                         restitution, pre_impact_rel_vel_normal,
                         frame));
  ```
- **Design rationale**: Separate table from `FrictionConstraintRecord` for schema flexibility. Contact constraints have different state (restitution, penetration) than friction constraints (tangents, friction coefficient).

### Modified Components

#### Constraint (msd-sim)
- **Current location**: `msd/msd-sim/src/Physics/Constraints/Constraint.hpp`
- **Changes required**:
  - Add pure virtual method `toRecord(uint32_t bodyAId, uint32_t bodyBId) const` returning `std::any`
  - Each concrete constraint implements `toRecord()` to return its specific record type wrapped in `std::any`
- **Backward compatibility**: New virtual method added (non-breaking — existing constraints must implement it)
- **Design rationale**:
  - `std::any` enables type-erased return (caller uses `std::any_cast<T>` to extract specific record type)
  - Alternative considered: visitor pattern (rejected as more intrusive — requires visitor interface on each constraint)
  - Alternative considered: template method pattern (rejected — incompatible with polymorphic base class)

```cpp
// In Constraint base class (Constraint.hpp)
class Constraint {
public:
  // NEW: Serialize constraint state to transfer record
  [[nodiscard]] virtual std::any toRecord(uint32_t bodyAId, uint32_t bodyBId) const = 0;

  // ... existing virtual methods ...
};
```

#### FrictionConstraint (msd-sim)
- **Current location**: `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.hpp`
- **Changes required**: Implement `toRecord()` method
- **Backward compatibility**: No existing call sites affected (new method implementation)

```cpp
// In FrictionConstraint (FrictionConstraint.hpp)
class FrictionConstraint : public Constraint {
public:
  [[nodiscard]] std::any toRecord(uint32_t bodyAId, uint32_t bodyBId) const override;

  // ... existing methods ...
};

// Implementation (FrictionConstraint.cpp)
std::any FrictionConstraint::toRecord(uint32_t bodyAId, uint32_t bodyBId) const {
  msd_transfer::FrictionConstraintRecord record;
  record.body_a_id = bodyAId;
  record.body_b_id = bodyBId;

  // Serialize geometry (convert Coordinate to Vector3D to avoid point semantics)
  record.normal = Vector3D{contact_normal_.x(), contact_normal_.y(), contact_normal_.z()}.toRecord();
  record.tangent1 = Vector3D{tangent1_.x(), tangent1_.y(), tangent1_.z()}.toRecord();
  record.tangent2 = Vector3D{tangent2_.x(), tangent2_.y(), tangent2_.z()}.toRecord();
  record.lever_arm_a = Vector3D{lever_arm_a_.x(), lever_arm_a_.y(), lever_arm_a_.z()}.toRecord();
  record.lever_arm_b = Vector3D{lever_arm_b_.x(), lever_arm_b_.y(), lever_arm_b_.z()}.toRecord();

  // Serialize parameters
  record.friction_coefficient = friction_coefficient_;
  record.normal_lambda = normal_lambda_;

  return record;
}
```

#### ContactConstraint (msd-sim)
- **Current location**: `msd/msd-sim/src/Physics/Constraints/ContactConstraint.hpp`
- **Changes required**: Implement `toRecord()` method
- **Backward compatibility**: No existing call sites affected (new method implementation)

```cpp
// In ContactConstraint (ContactConstraint.hpp)
class ContactConstraint : public Constraint {
public:
  [[nodiscard]] std::any toRecord(uint32_t bodyAId, uint32_t bodyBId) const override;

  // ... existing methods ...
};

// Implementation (ContactConstraint.cpp)
std::any ContactConstraint::toRecord(uint32_t bodyAId, uint32_t bodyBId) const {
  msd_transfer::ContactConstraintRecord record;
  record.body_a_id = bodyAId;
  record.body_b_id = bodyBId;

  // Serialize geometry (convert Coordinate to Vector3D)
  record.normal = Vector3D{contact_normal_.x(), contact_normal_.y(), contact_normal_.z()}.toRecord();
  record.lever_arm_a = Vector3D{lever_arm_a_.x(), lever_arm_a_.y(), lever_arm_a_.z()}.toRecord();
  record.lever_arm_b = Vector3D{lever_arm_b_.x(), lever_arm_b_.y(), lever_arm_b_.z()}.toRecord();
  record.penetration_depth = penetration_depth_;

  // Serialize parameters
  record.restitution = restitution_;
  record.pre_impact_rel_vel_normal = pre_impact_rel_vel_normal_;

  return record;
}
```

#### DataRecorder (msd-sim)
- **Current location**: `msd/msd-sim/src/DataRecorder/DataRecorder.hpp/.cpp`
- **Changes required**:
  - Add `recordConstraintStates(uint32_t frameId, const CollisionPipeline& pipeline)` method
  - Iterate `pipeline.getContactConstraints()` and `pipeline.getFrictionConstraints()`, call `toRecord()`, set FK, buffer
- **Backward compatibility**: New method, no existing call sites affected

```cpp
// In DataRecorder.hpp (new method)
void recordConstraintStates(uint32_t frameId, const CollisionPipeline& pipeline);

// Implementation (DataRecorder.cpp)
void DataRecorder::recordConstraintStates(uint32_t frameId, const CollisionPipeline& pipeline) {
  // Record contact constraints
  for (const auto& constraint : pipeline.getContactConstraints()) {
    auto recordAny = constraint->toRecord(
      pipeline.getBodyAId(constraint.get()),
      pipeline.getBodyBId(constraint.get())
    );
    auto record = std::any_cast<msd_transfer::ContactConstraintRecord>(recordAny);
    record.frame.id = frameId;
    getDAO<msd_transfer::ContactConstraintRecord>().addToBuffer(record);
  }

  // Record friction constraints
  for (const auto& constraint : pipeline.getFrictionConstraints()) {
    auto recordAny = constraint->toRecord(
      pipeline.getBodyAId(constraint.get()),
      pipeline.getBodyBId(constraint.get())
    );
    auto record = std::any_cast<msd_transfer::FrictionConstraintRecord>(recordAny);
    record.frame.id = frameId;
    getDAO<msd_transfer::FrictionConstraintRecord>().addToBuffer(record);
  }
}
```

**Design questions**:
1. **Body ID retrieval**: `CollisionPipeline` needs accessor methods `getBodyAId(const Constraint*)` and `getBodyBId(const Constraint*)` to map constraint pointer to body IDs. Alternative: constraints already store `bodyAIndex()` and `bodyBIndex()`, but those are indices into the solver's body list, not persistent IDs. The pipeline must provide the mapping from index to ID.
2. **Constraint iteration**: Assumes `CollisionPipeline` exposes `getContactConstraints()` and `getFrictionConstraints()` (currently these are private members). If not exposed, the pipeline must provide a `recordConstraints(DataRecorder&, uint32_t frameId)` method that handles iteration internally.

#### CollisionPipeline (msd-sim)
- **Current location**: `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp/.cpp`
- **Changes required**:
  - **Option A** (minimal): Add accessor methods `getContactConstraints()` and `getFrictionConstraints()` to expose constraint vectors for iteration by `DataRecorder`
  - **Option B** (encapsulated): Keep constraints private, add `recordConstraints(DataRecorder& recorder, uint32_t frameId)` method that iterates constraints and calls recorder
  - Add method to map constraint pointer to body IDs: `getBodyAId(const Constraint*)`, `getBodyBId(const Constraint*)`
- **Backward compatibility**: New methods, no breaking changes
- **Recommendation**: **Option B** — keeps constraint ownership encapsulated, follows existing `CollisionPipeline` pattern of exposing high-level operations rather than raw data

```cpp
// Option B: CollisionPipeline.hpp (new method)
class CollisionPipeline {
public:
  void recordConstraints(DataRecorder& recorder, uint32_t frameId) const;

private:
  std::vector<std::unique_ptr<ContactConstraint>> constraints_;
  std::vector<std::unique_ptr<FrictionConstraint>> frictionConstraints_;
  std::vector<CollisionPair> collisions_;  // Maps collision pairs to body IDs
};

// Implementation (CollisionPipeline.cpp)
void CollisionPipeline::recordConstraints(DataRecorder& recorder, uint32_t frameId) const {
  // Iterate contact constraints (one per contact point, mapped by pairRanges_)
  for (size_t i = 0; i < constraints_.size(); ++i) {
    const auto& constraint = constraints_[i];

    // Find collision pair for this constraint (via pairRanges_ lookup)
    const auto& pair = findPairForConstraintIndex(i);  // Helper method

    auto recordAny = constraint->toRecord(pair.bodyAId, pair.bodyBId);
    auto record = std::any_cast<msd_transfer::ContactConstraintRecord>(recordAny);
    record.frame.id = frameId;
    recorder.getDAO<msd_transfer::ContactConstraintRecord>().addToBuffer(record);
  }

  // Iterate friction constraints (one per contact point, same indexing as contact constraints)
  for (size_t i = 0; i < frictionConstraints_.size(); ++i) {
    const auto& constraint = frictionConstraints_[i];

    // Find collision pair for this constraint (via pairRanges_ lookup)
    const auto& pair = findPairForConstraintIndex(i);  // Helper method

    auto recordAny = constraint->toRecord(pair.bodyAId, pair.bodyBId);
    auto record = std::any_cast<msd_transfer::FrictionConstraintRecord>(recordAny);
    record.frame.id = frameId;
    recorder.getDAO<msd_transfer::FrictionConstraintRecord>().addToBuffer(record);
  }
}
```

#### WorldModel (msd-sim)
- **Current location**: `msd/msd-sim/src/Environment/WorldModel.hpp/.cpp`
- **Changes required**: Call `dataRecorder_->recordConstraintStates(frameId, collisionPipeline_)` in `recordCurrentFrame()`
- **Backward compatibility**: Existing recording flow unaffected (new method added to existing recording sequence)

```cpp
// In WorldModel::recordCurrentFrame()
void WorldModel::recordCurrentFrame() {
  if (!dataRecorder_) return;

  uint32_t frameId = dataRecorder_->recordFrame(time_);
  dataRecorder_->recordInertialStates(frameId, inertialAssets_);
  dataRecorder_->recordBodyEnergies(frameId, inertialAssets_, potentialEnergies_);
  dataRecorder_->recordSystemEnergy(frameId, systemEnergy, previousSystemEnergy_, collisionActive);
  dataRecorder_->recordCollisions(frameId, collisionPipeline_);
  dataRecorder_->recordSolverDiagnostics(frameId, collisionPipeline_);

  // NEW: Record constraint states
  collisionPipeline_.recordConstraints(*dataRecorder_, frameId);

  previousSystemEnergy_ = systemEnergy.total;
}
```

### Python Bindings (msd-pybind)

#### record_bindings.cpp
- **Current location**: `msd/msd-pybind/src/record_bindings.cpp`
- **Changes required**:
  - Add `py::class_<FrictionConstraintRecord>` with `.def_readonly()` for all fields
  - Add `py::class_<ContactConstraintRecord>` with `.def_readonly()` for all fields
- **Backward compatibility**: New bindings, no existing code affected

```cpp
py::class_<msd_transfer::FrictionConstraintRecord>(m, "FrictionConstraintRecord")
  .def(py::init<>())
  .def_readonly("id", &msd_transfer::FrictionConstraintRecord::id)
  .def_readonly("body_a_id", &msd_transfer::FrictionConstraintRecord::body_a_id)
  .def_readonly("body_b_id", &msd_transfer::FrictionConstraintRecord::body_b_id)
  .def_readonly("normal", &msd_transfer::FrictionConstraintRecord::normal)
  .def_readonly("tangent1", &msd_transfer::FrictionConstraintRecord::tangent1)
  .def_readonly("tangent2", &msd_transfer::FrictionConstraintRecord::tangent2)
  .def_readonly("lever_arm_a", &msd_transfer::FrictionConstraintRecord::lever_arm_a)
  .def_readonly("lever_arm_b", &msd_transfer::FrictionConstraintRecord::lever_arm_b)
  .def_readonly("friction_coefficient", &msd_transfer::FrictionConstraintRecord::friction_coefficient)
  .def_readonly("normal_lambda", &msd_transfer::FrictionConstraintRecord::normal_lambda)
  .def_readonly("frame_id", &msd_transfer::FrictionConstraintRecord::frame.id);

py::class_<msd_transfer::ContactConstraintRecord>(m, "ContactConstraintRecord")
  .def(py::init<>())
  .def_readonly("id", &msd_transfer::ContactConstraintRecord::id)
  .def_readonly("body_a_id", &msd_transfer::ContactConstraintRecord::body_a_id)
  .def_readonly("body_b_id", &msd_transfer::ContactConstraintRecord::body_b_id)
  .def_readonly("normal", &msd_transfer::ContactConstraintRecord::normal)
  .def_readonly("lever_arm_a", &msd_transfer::ContactConstraintRecord::lever_arm_a)
  .def_readonly("lever_arm_b", &msd_transfer::ContactConstraintRecord::lever_arm_b)
  .def_readonly("penetration_depth", &msd_transfer::ContactConstraintRecord::penetration_depth)
  .def_readonly("restitution", &msd_transfer::ContactConstraintRecord::restitution)
  .def_readonly("pre_impact_rel_vel_normal", &msd_transfer::ContactConstraintRecord::pre_impact_rel_vel_normal)
  .def_readonly("frame_id", &msd_transfer::ContactConstraintRecord::frame.id);
```

### Replay Python Models (replay)

#### models.py
- **Current location**: `replay/replay/models.py`
- **Changes required**:
  - Add `FrictionConstraintInfo` model
  - Add `ContactConstraintInfo` model
  - Add `constraints` field to `FrameState` (list of constraint info objects)
- **Backward compatibility**: Optional fields default to empty list for older recordings

```python
class FrictionConstraintInfo(BaseModel):
    body_a_id: int
    body_b_id: int
    normal: Vec3
    tangent1: Vec3  # First tangent direction
    tangent2: Vec3  # Second tangent direction
    lever_arm_a: Vec3
    lever_arm_b: Vec3
    friction_coefficient: float
    normal_lambda: float  # Normal force magnitude

class ContactConstraintInfo(BaseModel):
    body_a_id: int
    body_b_id: int
    normal: Vec3
    lever_arm_a: Vec3
    lever_arm_b: Vec3
    penetration_depth: float
    restitution: float
    pre_impact_rel_vel_normal: float

class FrameState(BaseModel):
    time: float
    objects: list[ObjectState]
    collisions: list[CollisionInfo]
    constraints: list[FrictionConstraintInfo | ContactConstraintInfo] = []  # NEW
```

#### simulation_service.py
- **Current location**: `replay/replay/services/simulation_service.py`
- **Changes required**:
  - Add constraint record retrieval in `get_frame_data()`
  - Construct `FrictionConstraintInfo` and `ContactConstraintInfo` from records
- **Backward compatibility**: Check if constraint tables exist before querying

```python
def get_frame_data(self, simulation_id: int, frame_id: int) -> FrameState:
    # ... existing object and collision retrieval ...

    # NEW: Retrieve constraint states
    constraints = []

    # Friction constraints
    friction_records = self.db.select_friction_constraints_by_frame(frame_id)
    for fc in friction_records:
        constraints.append(FrictionConstraintInfo(
            body_a_id=fc.body_a_id,
            body_b_id=fc.body_b_id,
            normal=Vec3(x=fc.normal.x, y=fc.normal.y, z=fc.normal.z),
            tangent1=Vec3(x=fc.tangent1.x, y=fc.tangent1.y, z=fc.tangent1.z),
            tangent2=Vec3(x=fc.tangent2.x, y=fc.tangent2.y, z=fc.tangent2.z),
            lever_arm_a=Vec3(x=fc.lever_arm_a.x, y=fc.lever_arm_a.y, z=fc.lever_arm_a.z),
            lever_arm_b=Vec3(x=fc.lever_arm_b.x, y=fc.lever_arm_b.y, z=fc.lever_arm_b.z),
            friction_coefficient=fc.friction_coefficient,
            normal_lambda=fc.normal_lambda
        ))

    # Contact constraints
    contact_records = self.db.select_contact_constraints_by_frame(frame_id)
    for cc in contact_records:
        constraints.append(ContactConstraintInfo(
            body_a_id=cc.body_a_id,
            body_b_id=cc.body_b_id,
            normal=Vec3(x=cc.normal.x, y=cc.normal.y, z=cc.normal.z),
            lever_arm_a=Vec3(x=cc.lever_arm_a.x, y=cc.lever_arm_a.y, z=cc.lever_arm_a.z),
            lever_arm_b=Vec3(x=cc.lever_arm_b.x, y=cc.lever_arm_b.y, z=cc.lever_arm_b.z),
            penetration_depth=cc.penetration_depth,
            restitution=cc.restitution,
            pre_impact_rel_vel_normal=cc.pre_impact_rel_vel_normal
        ))

    return FrameState(
        time=frame_record.simulation_time,
        objects=objects,
        collisions=collisions,
        constraints=constraints  # NEW
    )
```

### Frontend (replay/static)

#### contacts.js (from ticket 0056f)
- **Current location**: `replay/static/js/overlays/contacts.js` (created in ticket 0056f)
- **Changes required**:
  - Add tangent arrow rendering for `FrictionConstraintInfo` entries in frame state
  - Use green for tangent1, blue for tangent2 (red already reserved for normal)
  - Render at contact point (derive from `lever_arm_a` + body A position)
- **Backward compatibility**: Check for constraint list existence before rendering

```javascript
class ContactOverlay {
    constructor(scene) {
        this.scene = scene;
        this.normalArrows = new Map();   // Red arrows (from 0056f)
        this.tangent1Arrows = new Map(); // Green arrows (NEW)
        this.tangent2Arrows = new Map(); // Blue arrows (NEW)
        this.enabled = false;
    }

    update(frameState, objects) {
        if (!this.enabled) {
            this.clear();
            return;
        }

        this.clear();

        // Render friction constraint tangent arrows
        if (frameState.constraints) {
            frameState.constraints.forEach((constraint, idx) => {
                if (constraint.tangent1 && constraint.tangent2) {
                    // Find body A position to compute contact point world location
                    const bodyA = objects.find(o => o.id === constraint.body_a_id);
                    if (!bodyA) return;

                    // Contact point = body A position + lever_arm_a
                    const contactPoint = new THREE.Vector3(
                        bodyA.position.x + constraint.lever_arm_a.x,
                        bodyA.position.y + constraint.lever_arm_a.y,
                        bodyA.position.z + constraint.lever_arm_a.z
                    );

                    // Render tangent1 (green)
                    const t1Arrow = new THREE.ArrowHelper(
                        new THREE.Vector3(constraint.tangent1.x, constraint.tangent1.y, constraint.tangent1.z),
                        contactPoint,
                        1.0,  // Fixed length (1.0 units)
                        0x00ff00  // Green
                    );
                    this.scene.add(t1Arrow);
                    this.tangent1Arrows.set(`constraint_${idx}_t1`, t1Arrow);

                    // Render tangent2 (blue)
                    const t2Arrow = new THREE.ArrowHelper(
                        new THREE.Vector3(constraint.tangent2.x, constraint.tangent2.y, constraint.tangent2.z),
                        contactPoint,
                        1.0,
                        0x0000ff  // Blue
                    );
                    this.scene.add(t2Arrow);
                    this.tangent2Arrows.set(`constraint_${idx}_t2`, t2Arrow);

                    // Render normal (red) - already implemented in 0056f
                    const normalArrow = new THREE.ArrowHelper(
                        new THREE.Vector3(constraint.normal.x, constraint.normal.y, constraint.normal.z),
                        contactPoint,
                        1.0,
                        0xff0000  // Red
                    );
                    this.scene.add(normalArrow);
                    this.normalArrows.set(`constraint_${idx}_normal`, normalArrow);
                }
            });
        }
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
| FrictionConstraintRecord | FrictionConstraint::toRecord() | Serialization | Constraint calls .toRecord() on member fields (Coordinate -> Vector3D) |
| ContactConstraintRecord | ContactConstraint::toRecord() | Serialization | Similar pattern to FrictionConstraint |
| DataRecorder::recordConstraintStates() | CollisionPipeline | Iteration delegation | Pipeline exposes recordConstraints() method encapsulating iteration |
| WorldModel::recordCurrentFrame() | DataRecorder::recordConstraintStates() | Recording orchestration | Called alongside existing recordCollisions() |
| Python FrictionConstraintInfo | FrictionConstraintRecord | Binding | pybind11 exposes fields via .def_readonly() |
| Three.js ContactOverlay | FrameState.constraints | Rendering | Iterates constraint list and renders arrows |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| N/A | N/A | None | No existing tests modified (new functionality only) |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| `FrictionConstraint` | `toRecord_SerializesAllFields` | All member variables correctly serialized to FrictionConstraintRecord |
| `FrictionConstraint` | `toRecord_TangentsAreVector3D` | tangent1/tangent2 are Vector3DRecord (not Coordinate semantics) |
| `ContactConstraint` | `toRecord_SerializesAllFields` | All member variables correctly serialized to ContactConstraintRecord |
| `DataRecorder` | `recordConstraintStates_WritesRecords` | Constraint records buffered correctly via DAO |
| `CollisionPipeline` | `recordConstraints_MapsBodyIDs` | Constraint-to-body-ID mapping correct via pairRanges_ |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `generate_test_recording` with friction | DataRecorder, CollisionPipeline, FrictionConstraint | FrictionConstraintRecord present in SQLite recording |
| REST API `/api/v1/simulations/{id}/frames/{frame_id}/state` | SimulationService, Python bindings | Constraint data appears in JSON response |
| Tangent vector orthogonality check | FrictionConstraintRecord | Tangents are unit length, mutually orthogonal, perpendicular to normal |

#### Manual Tests

| Test Case | Steps | Expected Behavior |
|-----------|-------|-------------------|
| Replay viewer tangent arrows | 1. Generate recording with friction scenario<br>2. Open replay viewer<br>3. Navigate to collision frame<br>4. Enable contact overlay | Three orthogonal arrows visible at contact point: red (normal), green (t1), blue (t2) |
| Tangent arrows disappear on separation | 1. Navigate to frame after collision ends<br>2. Contact overlay still enabled | No arrows visible (objects separated, no constraints active) |
| Backward compatibility | 1. Open recording from before constraint recording existed<br>2. Navigate to frames | No errors (constraint list empty or absent) |

## Open Questions

### Design Decisions (Human Input Needed)

1. **CollisionPipeline constraint access pattern**
   - Option A: Expose `getContactConstraints()` and `getFrictionConstraints()` accessors — DataRecorder iterates constraints directly
   - Option B: Keep constraints private, pipeline provides `recordConstraints(DataRecorder&, uint32_t frameId)` method — Pipeline handles iteration and body ID mapping internally
   - Recommendation: **Option B** — Maintains encapsulation, follows existing CollisionPipeline pattern of exposing operations rather than data

2. **Constraint-to-body-ID mapping**
   - Currently constraints store `bodyAIndex()` and `bodyBIndex()` (indices into solver body list)
   - Recording needs persistent body IDs (uint32_t from AssetInertial::getId())
   - CollisionPipeline owns the mapping via `collisions_` vector (bodyAId, bodyBId per collision)
   - **Design**: CollisionPipeline must map constraint index to collision pair via `pairRanges_`, then extract body IDs

3. **Polymorphic constraint recording pattern** — How to extend to future constraint types?
   - Each new constraint type (HingeConstraint, MotorConstraint) adds:
     - New transfer record (`HingeConstraintRecord`) in msd-transfer
     - `toRecord()` implementation returning its record type wrapped in `std::any`
     - New iteration branch in `CollisionPipeline::recordConstraints()` or new recording method in DataRecorder
   - **Open question**: Is the `std::any` + `any_cast` pattern acceptable, or should we use a different polymorphic approach?
   - **Alternative**: Template-based dispatch (requires compile-time knowledge of all types)
   - **Alternative**: Visitor pattern (more intrusive, requires visitor interface on each constraint)

### Prototype Required

None. All components are straightforward extensions of existing patterns:
- Transfer record extension follows `ContactPointRecord` precedent
- Virtual method addition to `Constraint` base class is standard polymorphism
- DataRecorder domain-aware methods follow existing `recordCollisions()` pattern
- Python bindings follow existing `CollisionResultRecord` pattern
- Three.js arrow rendering uses same `ArrowHelper` as normal arrows (ticket 0056f)

### Requirements Clarification

1. **Constraint recording granularity**: Should we record ALL constraints (including single-body UnitQuaternionConstraint, DistanceConstraint) or only contact-related constraints?
   - Current assumption: Only contact-related constraints (ContactConstraint, FrictionConstraint) are recorded for visualization
   - Rationale: Single-body constraints have no geometric representation to visualize

2. **Constraint state vs. solver output**: Should we record pre-solve constraint state (geometry, parameters) or post-solve state (lambda values)?
   - Current assumption: Both — geometry/parameters for visualization, lambda values for debugging force magnitudes
   - FrictionConstraint already exposes `normal_lambda` after solve; ContactConstraint could expose its lambda similarly

## Implementation Notes

### Type Safety: Vector3D vs. Coordinate

**Critical**: Tangent vectors MUST use `Vector3D` type, not `Coordinate`:
- `Coordinate` inherits from `Vector3D` and represents a **point in space**
- `Vector3D` represents a **direction vector**
- The `globalToLocal` overload bug occurs when passing a `Coordinate` where a `Vector3D` is expected — the `Coordinate` overload applies translation, corrupting direction vectors
- **Solution**: Explicitly construct `Vector3D` from `Coordinate` member values when serializing tangents:
  ```cpp
  record.tangent1 = Vector3D{tangent1_.x(), tangent1_.y(), tangent1_.z()}.toRecord();
  ```

### Constraint Ownership and Lifetime

- Constraints owned by `CollisionPipeline` via `std::unique_ptr<Constraint>` vectors
- Constraints destroyed after each physics step in `CollisionPipeline::clearEphemeralState()`
- Recording must happen BEFORE `clearEphemeralState()` is called
- Current flow: `WorldModel::update()` → collision solve → **record constraints** → `clearEphemeralState()`

### SQLite Schema Evolution

cpp_sqlite handles missing tables/columns gracefully:
- New `FrictionConstraintRecord` and `ContactConstraintRecord` tables added on first write
- Older recordings without constraint tables load without error (empty constraint list in Python)
- No manual schema migration needed

### Performance Impact

- **C++ overhead**: O(n_constraints) record creation (one per contact point, typically 1-20 per frame)
- **Database overhead**: +~150 bytes per friction constraint (7 × Vector3DRecord + 2 doubles + FK)
- **Rendering overhead**: +2 ArrowHelper instances per friction constraint (minimal GPU impact)

Estimated total overhead: < 2% for typical scenarios (10-100 constraints per frame)

---

## Design Rationale Summary

This design establishes a general-purpose constraint state recording pattern while achieving the immediate goal of tangent vector visualization. Key architectural decisions:

1. **Separate transfer records per constraint type** — Schema flexibility for evolving constraint state (FrictionConstraint has tangents, ContactConstraint has restitution, future types have their own state)
2. **Virtual toRecord() method on Constraint base** — Polymorphic serialization without tight coupling to DataRecorder
3. **std::any for type-erased return** — Enables heterogeneous constraint iteration without template metaprogramming or visitor pattern complexity
4. **CollisionPipeline encapsulation** — Pipeline handles constraint-to-body-ID mapping and iteration, DataRecorder just buffers records
5. **Vector3D for tangent vectors** — Type-safe direction semantics, avoids `globalToLocal` overload bug

The pattern is extensible: adding a `HingeConstraint` requires only:
- Define `HingeConstraintRecord` struct
- Implement `toRecord()` on `HingeConstraint`
- Add iteration branch in `CollisionPipeline::recordConstraints()` or DataRecorder

Future constraint types integrate seamlessly without modifying the base `Constraint` interface or recording infrastructure.
