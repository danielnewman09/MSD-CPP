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

#### ConstraintRecordVisitor (msd-transfer)
- **Location**: `msd/msd-transfer/src/ConstraintRecordVisitor.hpp` (new interface)
- **Purpose**: Visitor interface for type-safe constraint record dispatching
- **Type**: Abstract interface
- **Key interfaces**:
  ```cpp
  class ConstraintRecordVisitor {
  public:
    virtual ~ConstraintRecordVisitor() = default;

    virtual void visit(const ContactConstraintRecord& record) = 0;
    virtual void visit(const FrictionConstraintRecord& record) = 0;
    // Future constraint types add new visit() overloads
  };
  ```
- **Design rationale**:
  - **Type safety**: Compiler enforces handling of all constraint types (missing visit() overload = compile error)
  - **No dynamic_cast**: Constraints dispatch via virtual `recordState()` call, not runtime type inspection
  - **No std::any_cast**: Concrete visitor receives typed record directly
  - **Extensibility**: New constraint types extend interface with new visit() overload, compiler catches unhandled cases

#### Constraint (msd-sim)
- **Current location**: `msd/msd-sim/src/Physics/Constraints/Constraint.hpp`
- **Changes required**:
  - Add pure virtual method `recordState(ConstraintRecordVisitor& visitor, uint32_t bodyAId, uint32_t bodyBId) const`
  - Each concrete constraint implements `recordState()` by building its specific record and calling `visitor.visit(record)`
- **Backward compatibility**: New virtual method added (non-breaking — existing constraints must implement it)
- **Design rationale**:
  - **Visitor pattern**: Constraints build typed records and dispatch to visitor via overload resolution (no type erasure)
  - **Separation of concerns**: Constraint builds record, visitor decides what to do with it (buffer to DAO, serialize to JSON, etc.)
  - **vs. std::any**: Compile-time type safety instead of runtime type inspection
  - **vs. template method**: Compatible with polymorphic base class (visitor is runtime polymorphic)

```cpp
// In Constraint base class (Constraint.hpp)
class Constraint {
public:
  // NEW: Serialize constraint state via visitor pattern
  virtual void recordState(ConstraintRecordVisitor& visitor,
                           uint32_t bodyAId,
                           uint32_t bodyBId) const = 0;

  // ... existing virtual methods ...
};
```

#### FrictionConstraint (msd-sim)
- **Current location**: `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.hpp`
- **Changes required**: Implement `recordState()` method
- **Backward compatibility**: No existing call sites affected (new method implementation)

```cpp
// In FrictionConstraint (FrictionConstraint.hpp)
class FrictionConstraint : public Constraint {
public:
  void recordState(ConstraintRecordVisitor& visitor,
                   uint32_t bodyAId,
                   uint32_t bodyBId) const override;

  // ... existing methods ...
};

// Implementation (FrictionConstraint.cpp)
void FrictionConstraint::recordState(ConstraintRecordVisitor& visitor,
                                      uint32_t bodyAId,
                                      uint32_t bodyBId) const {
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

  // Dispatch to visitor via overload resolution
  visitor.visit(record);
}
```

#### ContactConstraint (msd-sim)
- **Current location**: `msd/msd-sim/src/Physics/Constraints/ContactConstraint.hpp`
- **Changes required**: Implement `recordState()` method
- **Backward compatibility**: No existing call sites affected (new method implementation)

```cpp
// In ContactConstraint (ContactConstraint.hpp)
class ContactConstraint : public Constraint {
public:
  void recordState(ConstraintRecordVisitor& visitor,
                   uint32_t bodyAId,
                   uint32_t bodyBId) const override;

  // ... existing methods ...
};

// Implementation (ContactConstraint.cpp)
void ContactConstraint::recordState(ConstraintRecordVisitor& visitor,
                                     uint32_t bodyAId,
                                     uint32_t bodyBId) const {
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

  // Dispatch to visitor via overload resolution
  visitor.visit(record);
}
```

#### DataRecorderVisitor (msd-sim)
- **Location**: `msd/msd-sim/src/DataRecorder/DataRecorderVisitor.hpp` (new concrete visitor)
- **Purpose**: Concrete visitor that buffers constraint records to DataRecorder DAOs
- **Type**: Concrete implementation of `ConstraintRecordVisitor`
- **Key interfaces**:
  ```cpp
  class DataRecorderVisitor : public ConstraintRecordVisitor {
  public:
    DataRecorderVisitor(DataRecorder& recorder, uint32_t frameId)
      : recorder_{recorder}, frameId_{frameId} {}

    void visit(const ContactConstraintRecord& record) override {
      auto recordCopy = record;
      recordCopy.frame.id = frameId_;
      recorder_.getDAO<ContactConstraintRecord>().addToBuffer(recordCopy);
    }

    void visit(const FrictionConstraintRecord& record) override {
      auto recordCopy = record;
      recordCopy.frame.id = frameId_;
      recorder_.getDAO<FrictionConstraintRecord>().addToBuffer(recordCopy);
    }

  private:
    DataRecorder& recorder_;
    uint32_t frameId_;
  };
  ```
- **Design rationale**:
  - **Single responsibility**: Visitor handles DAO buffering logic, constraints handle record building
  - **Type safety**: Each visit() overload receives typed record, no casting required
  - **Extensibility**: New constraint types add new visit() overload — compiler enforces completeness

#### DataRecorder (msd-sim)
- **Current location**: `msd/msd-sim/src/DataRecorder/DataRecorder.hpp/.cpp`
- **Changes required**: No changes (visitor pattern removes need for constraint-specific recording methods)
- **Backward compatibility**: No changes required (CollisionPipeline uses visitor internally)

#### CollisionPipeline (msd-sim)
- **Current location**: `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp/.cpp`
- **Current state** ([Ticket 0058](../../tickets/0058_constraint_ownership_cleanup.md)): Single owning container `allConstraints_` with interleaved [CC, FC, CC, FC, ...] storage for friction-enabled contacts. Typed views generated on-demand via `buildSolverView()` and `buildContactView()`.
- **Changes required**:
  - Add `recordConstraints(DataRecorder& recorder, uint32_t frameId)` method
  - Method creates `DataRecorderVisitor`, iterates `allConstraints_`, calls `constraint->recordState(visitor, bodyAId, bodyBId)` for each
  - Lookup collision pair via `pairRanges_` to map constraint index → collision pair → (bodyAId, bodyBId)
  - Constraints remain private (no accessors exposed)
- **Backward compatibility**: New method, no breaking changes
- **Design rationale**:
  - **Option B from design discussion**: Pipeline owns the recording method, keeps constraints encapsulated
  - **Visitor pattern**: No dynamic_cast or std::any_cast — constraints dispatch via virtual call
  - **Separation of concerns**: Pipeline handles iteration and body ID mapping, constraints handle record building, visitor handles DAO buffering

```cpp
// CollisionPipeline.hpp (new method)
class CollisionPipeline {
public:
  void recordConstraints(DataRecorder& recorder, uint32_t frameId) const;

private:
  std::vector<std::unique_ptr<Constraint>> allConstraints_;  // Single owning container (ticket 0058)
  std::vector<CollisionPair> collisions_;  // Maps collision pairs to body IDs
  std::vector<PairConstraintRange> pairRanges_;  // Maps constraint index → collision pair

  // Helper: Map constraint index to collision pair index
  size_t findPairIndexForConstraint(size_t constraintIdx) const;
};

// Implementation (CollisionPipeline.cpp)
void CollisionPipeline::recordConstraints(DataRecorder& recorder, uint32_t frameId) const {
  // Create visitor wrapping DataRecorder
  DataRecorderVisitor visitor{recorder, frameId};

  // Iterate all constraints (interleaved [CC, FC, CC, FC, ...] or all CC if no friction)
  for (size_t i = 0; i < allConstraints_.size(); ++i) {
    const auto& constraint = allConstraints_[i];

    // Find collision pair for this constraint (via pairRanges_ lookup)
    size_t pairIdx = findPairIndexForConstraint(i);
    const auto& pair = collisions_[pairIdx];

    // Constraint builds record and dispatches to visitor (no casting here)
    constraint->recordState(visitor, pair.bodyAId, pair.bodyBId);
  }
}

// Helper: Map constraint index to collision pair index
size_t CollisionPipeline::findPairIndexForConstraint(size_t constraintIdx) const {
  // Iterate pairRanges_ to find which pair owns this constraint index
  for (const auto& range : pairRanges_) {
    if (constraintIdx >= range.startIdx && constraintIdx < range.startIdx + range.count) {
      return range.pairIdx;
    }
  }
  // Should never reach here if pairRanges_ is correct
  throw std::logic_error("Constraint index not found in pairRanges_");
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
| ConstraintRecordVisitor | Constraint::recordState() | Visitor pattern | Abstract interface for type-safe dispatching |
| DataRecorderVisitor | ConstraintRecordVisitor | Concrete visitor | Buffers records to DataRecorder DAOs |
| FrictionConstraintRecord | FrictionConstraint::recordState() | Serialization | Constraint calls .toRecord() on member fields (Coordinate -> Vector3D), dispatches to visitor |
| ContactConstraintRecord | ContactConstraint::recordState() | Serialization | Similar pattern to FrictionConstraint |
| CollisionPipeline::recordConstraints() | DataRecorderVisitor | Iteration orchestration | Pipeline creates visitor, iterates constraints, dispatches recordState() calls |
| WorldModel::recordCurrentFrame() | CollisionPipeline::recordConstraints() | Recording orchestration | Called alongside existing recordCollisions() |
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

1. **Constraint-to-body-ID mapping**
   - ✅ **Resolved** (Ticket 0058): CollisionPipeline owns the mapping via `collisions_` vector (bodyAId, bodyBId per collision)
   - Pipeline maps constraint index → collision pair via `pairRanges_` → extract body IDs

2. **Polymorphic constraint recording pattern**
   - ✅ **Resolved** (Human feedback): Use visitor pattern instead of std::any
   - Each new constraint type (HingeConstraint, MotorConstraint) adds:
     - New transfer record (`HingeConstraintRecord`) in msd-transfer
     - `recordState(visitor, bodyAId, bodyBId)` implementation that builds record and calls `visitor.visit(record)`
     - New `visit(const HingeConstraintRecord&)` overload on `ConstraintRecordVisitor` interface
   - **Benefits**: Compile-time type safety, no dynamic_cast, no std::any_cast, compiler enforces handling all constraint types

3. **Constraint iteration and access**
   - ✅ **Resolved** (Human feedback): Option B — `CollisionPipeline::recordConstraints(DataRecorder&, frameId)` method
   - Pipeline keeps constraints private, handles iteration and body ID mapping internally
   - No constraint accessors exposed (encapsulation maintained)

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
2. **Visitor pattern for polymorphic dispatching** — Type-safe constraint recording without dynamic_cast or std::any_cast
3. **Virtual recordState() method on Constraint base** — Each constraint builds its record and dispatches to visitor via overload resolution
4. **CollisionPipeline encapsulation (Option B)** — Pipeline owns `recordConstraints()` method, keeps constraints private, handles iteration and body ID mapping internally
5. **DataRecorderVisitor concrete implementation** — Wraps DataRecorder, buffers records to appropriate DAOs based on type
6. **Vector3D for tangent vectors** — Type-safe direction semantics, avoids `globalToLocal` overload bug

The pattern is extensible: adding a `HingeConstraint` requires only:
- Define `HingeConstraintRecord` struct
- Add `visit(const HingeConstraintRecord&)` overload to `ConstraintRecordVisitor` interface
- Implement `recordState()` on `HingeConstraint` that builds record and calls `visitor.visit(record)`
- **Compiler enforces completeness** — Missing visit() overload = compile error

Future constraint types integrate seamlessly with compile-time type safety. The visitor pattern provides:
- **No runtime type inspection** (vs. dynamic_cast approach)
- **No type erasure** (vs. std::any approach)
- **Compiler-enforced exhaustiveness** (missing case = build failure)
- **Single Responsibility** (constraints build records, visitor decides what to do with them)
