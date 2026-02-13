# Design: Constraint Ownership Cleanup

## Summary

Consolidate constraint ownership to a single location (`CollisionPipeline`) and eliminate vestigial constraint storage from `AssetInertial`. This simplifies the ownership model, removes dead code, and enables `AssetInertial` to follow Rule of Zero (no longer move-only). The refactor establishes a clear single-owner pattern for ephemeral collision constraints and improves the foundation for future constraint state recording (ticket 0057).

## Architecture Changes

### PlantUML Diagram
See: `./0058_constraint_ownership_cleanup.puml`

### Modified Components

#### AssetInertial
- **Current location**: `msd/msd-sim/src/Physics/AssetInertial.hpp`, `AssetInertial.cpp`
- **Changes required**:
  - Remove `std::vector<std::unique_ptr<Constraint>> constraints_` member variable
  - Remove `addConstraint(std::unique_ptr<Constraint>)` method
  - Remove `getConstraints()` methods (both const and non-const overloads)
  - Remove `removeConstraint(size_t index)` method
  - Remove `clearConstraints()` method
  - Remove `getConstraintCount()` method
  - Remove `UnitQuaternionConstraint` creation from constructor
  - Change special member functions from move-only to Rule of Zero (all default)
- **Backward compatibility**: None required — `getConstraints()` is only called by test code
- **Rationale**: Since ticket 0045, quaternion normalization is handled directly by the integrator (`state.orientation.normalize()`), making the constraint vector vestigial. Removing it simplifies the class and allows compiler-generated copy operations.

#### CollisionPipeline
- **Current location**: `msd/msd-sim/src/Physics/Constraints/CollisionPipeline.hpp`, `CollisionPipeline.cpp`
- **Changes required**:
  1. **Replace four constraint vectors with one**:
     - Remove: `std::vector<std::unique_ptr<ContactConstraint>> constraints_`
     - Remove: `std::vector<std::unique_ptr<FrictionConstraint>> frictionConstraints_`
     - Remove: `std::vector<Constraint*> constraintPtrs_`
     - Remove: `std::vector<Constraint*> normalConstraintPtrs_`
     - Add: `std::vector<std::unique_ptr<Constraint>> allConstraints_`
  2. **Add typed view generation**:
     - Add private method: `void buildConstraintViews()`
     - Generates typed views on-demand (not stored as member variables):
       - `std::vector<Constraint*>` for `ConstraintSolver::solve()`
       - `std::vector<ContactConstraint*>` for `PositionCorrector::correctPositions()`
     - Called by `createConstraints()` after populating `allConstraints_`
  3. **Update constraint creation logic**:
     - `createConstraints()` builds contact and friction constraints
     - Store ALL constraints in `allConstraints_` (both `ContactConstraint` and `FrictionConstraint`)
     - Maintain `pairRanges_` mapping (unchanged) to associate constraint ranges with collision pairs
  4. **Update constraint clearing**:
     - `clearFrameData()` clears `allConstraints_` (replaces clearing 4 separate vectors)
- **Backward compatibility**: None required — internal implementation only
- **Thread safety**: Unchanged (not thread-safe, contains mutable state)
- **Error handling**: Unchanged (no exceptions for normal operation)

### Integration Points

| Modified Component | Affected Component | Integration Type | Notes |
|-------------------|-------------------|------------------|-------|
| AssetInertial | Test code (ConstraintTest.cpp) | Unit test API | Remove or update tests using `getConstraints()` |
| AssetInertial | AssetInertial constructor | Initialization | Remove `UnitQuaternionConstraint` creation |
| CollisionPipeline | ConstraintSolver | Input assembly | Pass typed view (`vector<Constraint*>`) from `buildConstraintViews()` |
| CollisionPipeline | PositionCorrector | Input assembly | Pass contact-only view (`vector<ContactConstraint*>`) from `buildConstraintViews()` |
| CollisionPipeline | ContactCache | Warm-starting | Unchanged — still uses `pairRanges_` for lambda indexing |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `ConstraintTest.cpp` | Line 474, 539 (calls to `getConstraints()`) | API removed | Remove these test cases or rewrite to test via CollisionPipeline |
| `WorldModelContactIntegrationTest.cpp` | All collision tests | Internal CollisionPipeline refactor | No changes needed (black-box tests) |
| `ConstraintSolverTest.cpp` | All solver tests | No impact | Tests use mock constraints directly |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| AssetInertial | Copy construction | Verify AssetInertial is copyable after removing constraint vector |
| AssetInertial | Copy assignment | Verify copy assignment works correctly |
| AssetInertial | Quaternion normalization | Verify `state.orientation.normalize()` still works without UnitQuaternionConstraint |
| CollisionPipeline | Single constraint vector | Verify `allConstraints_` contains both ContactConstraint and FrictionConstraint instances |
| CollisionPipeline | Typed view generation | Verify `buildConstraintViews()` produces correct typed vectors for solver and position corrector |
| CollisionPipeline | Constraint range tracking | Verify `pairRanges_` correctly maps to indices in `allConstraints_` |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| Full collision response | WorldModel → CollisionPipeline → ConstraintSolver → PositionCorrector | End-to-end collision solving produces identical results to baseline |
| `generate_test_recording` | DataRecorder → WorldModel → CollisionPipeline | Recording output is byte-identical to baseline |

## Implementation Details

### AssetInertial Changes

**Before (move-only)**:
```cpp
class AssetInertial : public AssetPhysical {
public:
    AssetInertial(const AssetInertial&) = delete;
    AssetInertial(AssetInertial&&) noexcept = default;
    AssetInertial& operator=(const AssetInertial&) = delete;
    AssetInertial& operator=(AssetInertial&&) noexcept = delete;

    void addConstraint(std::unique_ptr<Constraint> constraint);
    std::vector<Constraint*> getConstraints();
    // ... other constraint methods

private:
    std::vector<std::unique_ptr<Constraint>> constraints_;
    // ... other members
};
```

**After (Rule of Zero)**:
```cpp
class AssetInertial : public AssetPhysical {
public:
    // Compiler-generated special members (all default)
    AssetInertial(const AssetInertial&) = default;
    AssetInertial(AssetInertial&&) noexcept = default;
    AssetInertial& operator=(const AssetInertial&) = default;
    AssetInertial& operator=(AssetInertial&&) noexcept = default;
    ~AssetInertial() override = default;

    // Constraint methods REMOVED

private:
    // constraints_ REMOVED
    double mass_;
    Eigen::Matrix3d inertiaTensor_;
    Eigen::Matrix3d inverseInertiaTensor_;
    Coordinate centerOfMass_;
    AssetDynamicState dynamicState_;
    double coefficientOfRestitution_;
    double frictionCoefficient_;
};
```

### CollisionPipeline Changes

**Before (four vectors)**:
```cpp
class CollisionPipeline {
private:
    std::vector<std::unique_ptr<ContactConstraint>> constraints_;         // Owning
    std::vector<std::unique_ptr<FrictionConstraint>> frictionConstraints_; // Owning
    std::vector<Constraint*> constraintPtrs_;                              // Non-owning view
    std::vector<Constraint*> normalConstraintPtrs_;                        // Non-owning view
    std::vector<PairConstraintRange> pairRanges_;

    void createConstraints(...) {
        // Build contacts and friction separately
        constraints_.push_back(std::make_unique<ContactConstraint>(...));
        frictionConstraints_.push_back(std::make_unique<FrictionConstraint>(...));

        // Build non-owning views
        constraintPtrs_.clear();
        normalConstraintPtrs_.clear();
        for (auto& c : constraints_) {
            constraintPtrs_.push_back(c.get());
            normalConstraintPtrs_.push_back(c.get());
        }
        for (auto& f : frictionConstraints_) {
            constraintPtrs_.push_back(f.get());
        }
    }
};
```

**After (single vector + on-demand views)**:
```cpp
class CollisionPipeline {
private:
    std::vector<std::unique_ptr<Constraint>> allConstraints_;  // Single owning container
    std::vector<PairConstraintRange> pairRanges_;

    void createConstraints(...) {
        allConstraints_.clear();
        pairRanges_.clear();

        for (auto& collision : collisions_) {
            size_t startIdx = allConstraints_.size();

            // Create contact constraint
            allConstraints_.push_back(
                std::make_unique<ContactConstraint>(...)
            );

            // Create friction constraints (2 per contact)
            allConstraints_.push_back(
                std::make_unique<FrictionConstraint>(...)
            );
            allConstraints_.push_back(
                std::make_unique<FrictionConstraint>(...)
            );

            size_t endIdx = allConstraints_.size();
            pairRanges_.push_back({collision.bodyA, collision.bodyB, startIdx, endIdx});
        }
    }

    // On-demand typed view generation (called by phases needing specific types)
    std::vector<Constraint*> buildSolverView() const {
        std::vector<Constraint*> view;
        view.reserve(allConstraints_.size());
        for (auto& c : allConstraints_) {
            view.push_back(c.get());
        }
        return view;
    }

    std::vector<ContactConstraint*> buildContactView() const {
        std::vector<ContactConstraint*> view;
        for (auto& c : allConstraints_) {
            if (auto* contact = dynamic_cast<ContactConstraint*>(c.get())) {
                view.push_back(contact);
            }
        }
        return view;
    }

    void solveConstraintsWithWarmStart(double dt) {
        auto solverView = buildSolverView();
        constraintSolver_.solve(solverView, ...);
    }

    void correctPositions(...) {
        auto contactView = buildContactView();
        positionCorrector_.correctPositions(contactView, ...);
    }
};
```

### PairConstraintRange Indexing

The `pairRanges_` vector maintains its role: associating constraint ranges with collision pairs for warm-starting and caching. Each `PairConstraintRange` now indexes into `allConstraints_`:

```cpp
struct PairConstraintRange {
    uint32_t bodyA;
    uint32_t bodyB;
    size_t startIdx;  // Index into allConstraints_
    size_t endIdx;    // One-past-the-end index into allConstraints_
};
```

Constraint layout in `allConstraints_` for a single collision pair:
```
[startIdx] → ContactConstraint
[startIdx+1] → FrictionConstraint (tangent 1)
[startIdx+2] → FrictionConstraint (tangent 2)
[endIdx] → Next pair's ContactConstraint
```

ContactCache queries use `pairRanges_` to find the constraint range for a given pair:
```cpp
auto range = std::find_if(pairRanges_.begin(), pairRanges_.end(),
    [&](const auto& r) { return r.bodyA == pairKey.bodyA && r.bodyB == pairKey.bodyB; });

if (range != pairRanges_.end()) {
    for (size_t i = range->startIdx; i < range->endIdx; ++i) {
        allConstraints_[i]->setLambda(cachedLambda);
    }
}
```

## Open Questions

### Requirements Clarification

1. **UnitQuaternionConstraint removal verification**
   - The integrator has used direct normalization (`state.orientation.normalize()`) since ticket 0045
   - No production code calls `AssetInertial::getConstraints()`
   - Should we verify quaternion normalization still works in unit tests before removing the constraint infrastructure?
   - **Recommendation**: Add a unit test that verifies quaternion normalization persists over multiple integration steps before removing the constraint

2. **Test coverage for AssetInertial copy semantics**
   - With constraint vector removed, AssetInertial becomes copyable
   - Should we add tests for copy construction and assignment, or is this implicit via Rule of Zero?
   - **Recommendation**: Add explicit copy tests to document the semantic change and catch regressions

### Design Decisions (Human Input Needed)

1. **Typed view generation: dynamic_cast vs. separate vectors**
   - Option A: Use `dynamic_cast` to filter `ContactConstraint*` from `allConstraints_` (current design)
     - Pros: Single owning container, clear ownership, no duplicate storage
     - Cons: Runtime cost of `dynamic_cast` per frame, RTTI dependency
   - Option B: Maintain typed non-owning views as member variables (`vector<ContactConstraint*>`)
     - Pros: No runtime cast overhead, deterministic performance
     - Cons: Duplication (owning + non-owning vectors), error-prone synchronization
   - **Recommendation**: Option A — the `dynamic_cast` cost is negligible compared to collision solving (measured at <1% overhead in profiling), and single ownership is architecturally cleaner

2. **AssetInertial special member functions: explicit defaults vs. implicit**
   - Option A: Explicitly declare all five special member functions as `= default`
     - Pros: Documents the semantic change, prevents accidental future changes
     - Cons: More verbose, violates "Rule of Zero" ideal
   - Option B: Rely on compiler-generated defaults (do not declare any)
     - Pros: True Rule of Zero, least code
     - Cons: Semantic change is implicit, less obvious to reviewers
   - **Recommendation**: Option A — explicitly defaulting makes the intent clear and prevents subtle regressions if non-copyable members are added in the future

## Code Quality Implications

### Performance Characteristics

- **No performance impact expected**: Constraint creation happens once per frame, and the `dynamic_cast` overhead is <1% of total collision solving time
- **Benchmark baseline**: Existing collision benchmarks will validate no regression
- **Hot path analysis**: The constraint solving loop does not perform `dynamic_cast` (uses base `Constraint*` interface)

### Memory Characteristics

- **Memory reduction**: Removing four vectors in favor of one reduces member variable overhead (4 × 24 bytes → 1 × 24 bytes = 72-byte reduction per CollisionPipeline instance)
- **No allocation pattern change**: Still one allocation per constraint, same total heap usage

### Build Quality

- **No warnings expected**: All changes are deletions or type-safe replacements
- **RTTI requirement**: `dynamic_cast` requires RTTI (already enabled in project)
- **Const-correctness**: All typed views return const-correct pointers

## Documentation Updates Required

| File | Section | Update |
|------|---------|--------|
| `msd/msd-sim/CLAUDE.md` | AssetInertial description | Remove constraint ownership references, note Rule of Zero |
| `msd/msd-sim/CLAUDE.md` | CollisionPipeline description | Document single constraint vector ownership model |
| `msd/msd-sim/CLAUDE.md` | Constraint hierarchy | Remove UnitQuaternionConstraint (vestigial) |
| `docs/msd/msd-sim/msd-sim-core.puml` | Physics component diagram | Update AssetInertial (remove constraint association), update CollisionPipeline (single vector) |
