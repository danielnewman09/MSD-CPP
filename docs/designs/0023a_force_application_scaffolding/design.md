# Design: Force Application Scaffolding

## Summary
This design establishes the structural foundation for a force application system without implementing physics logic. It modifies `InertialState` to use `Coordinate` for angular velocity and acceleration (improving representation consistency), adds force/torque accumulation members to `AssetInertial`, introduces placeholder force application methods, and adds gravity configuration to `WorldModel`. This scaffolding clarifies design intent and enables early API validation before implementing the actual physics integration (ticket 0023).

## Architecture Changes

### PlantUML Diagram
See: `./0023a_force_application_scaffolding.puml`

### Modified Components

#### InertialState

**Current location**: `msd/msd-sim/src/Physics/RigidBody/InertialState.hpp`

**Changes required**:
1. Rename `angularPosition` to `orientation` (type remains `EulerAngles`)
2. Change `angularVelocity` from `EulerAngles` to `Coordinate`
3. Change `angularAcceleration` from `EulerAngles` to `Coordinate`

**Rationale**: Angular velocity and acceleration are fundamentally different from orientation. Using `Coordinate` for these quantities:
- Represents angular velocity vector in body frame correctly (ω = [ωx, ωy, ωz])
- Aligns with physics equations: τ = I·α, where α is angular acceleration vector
- Simplifies integration with Eigen-based inertia tensor operations
- Matches standard rigid body dynamics literature

**Backward compatibility**: **Breaking change** — All code referencing `InertialState::angularVelocity` and `InertialState::angularAcceleration` must migrate to `Coordinate`.

**Modified interface**:
```cpp
// msd/msd-sim/src/Physics/RigidBody/InertialState.hpp

#ifndef INERTIAL_STATE_HPP
#define INERTIAL_STATE_HPP

#include <Eigen/Dense>
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/EulerAngles.hpp"

namespace msd_sim
{

struct InertialState
{
  // Linear (unchanged)
  Coordinate position;
  Coordinate velocity;
  Coordinate acceleration;

  // Angular (MODIFIED)
  EulerAngles orientation;         // Renamed from angularPosition
  Coordinate angularVelocity;      // Changed from EulerAngles
  Coordinate angularAcceleration;  // Changed from EulerAngles
};

}  // namespace msd_sim

#endif
```

**Migration impact**:
- `AssetInertial` getters/setters will need updates
- Tests referencing old field names will need updates
- Any physics integration code (if present) needs vector-based updates

---

#### AssetInertial

**Current location**: `msd/msd-sim/src/Physics/RigidBody/AssetInertial.hpp`, `.cpp`

**Changes required**:
1. Add `accumulatedForce_` member (Coordinate, initialized to zero)
2. Add `accumulatedTorque_` member (Coordinate, initialized to zero)
3. Add `applyForce()` method (placeholder: just accumulates force)
4. Add `applyForceAtPoint()` method (placeholder: accumulates force, TODO comment for torque)
5. Add `applyTorque()` method (placeholder: just accumulates torque)
6. Add `clearForces()` method (resets both accumulators to zero)
7. Add `getAccumulatedForce()` accessor
8. Add `getAccumulatedTorque()` accessor

**Rationale**:
- Force accumulation separates force application from physics integration
- Allows multiple forces per frame (e.g., gravity + user input + collisions)
- Clear separation of concerns: forces accumulate during frame, integration consumes them
- Standard pattern in physics engines (e.g., Bullet, PhysX)

**Backward compatibility**: **Non-breaking** — Adds new API without modifying existing interfaces.

**Modified interface**:
```cpp
// msd/msd-sim/src/Physics/RigidBody/AssetInertial.hpp

class AssetInertial : public AssetPhysical
{
public:
  AssetInertial(uint32_t assetId,
                uint32_t instanceId,
                ConvexHull& hull,
                double mass,
                const ReferenceFrame& frame);

  // ... existing interface (getInertialState, getMass, getInertiaTensor, etc.) ...

  // NEW: Force application API
  /**
   * @brief Apply a force at the center of mass.
   *
   * Placeholder implementation for ticket 0023a — only accumulates force.
   * Actual integration will be implemented in ticket 0023.
   *
   * @param force Force vector in world coordinates [N]
   */
  void applyForce(const Coordinate& force);

  /**
   * @brief Apply a force at a specific world-space point.
   *
   * Placeholder implementation for ticket 0023a — accumulates force only.
   * Torque computation (r × F) deferred to ticket 0023.
   *
   * @param force Force vector in world coordinates [N]
   * @param worldPoint Application point in world coordinates [m]
   */
  void applyForceAtPoint(const Coordinate& force, const Coordinate& worldPoint);

  /**
   * @brief Apply a torque about the center of mass.
   *
   * Placeholder implementation for ticket 0023a — only accumulates torque.
   *
   * @param torque Torque vector in world coordinates [N·m]
   */
  void applyTorque(const Coordinate& torque);

  /**
   * @brief Clear all accumulated forces and torques.
   *
   * Call this at the end of each physics step after integration.
   */
  void clearForces();

  /**
   * @brief Get the accumulated force for this frame.
   * @return Accumulated force vector [N]
   */
  const Coordinate& getAccumulatedForce() const;

  /**
   * @brief Get the accumulated torque for this frame.
   * @return Accumulated torque vector [N·m]
   */
  const Coordinate& getAccumulatedTorque() const;

private:
  // ... existing members (mass_, inertiaTensor_, etc.) ...

  // NEW: Force accumulation
  Coordinate accumulatedForce_{0.0, 0.0, 0.0};
  Coordinate accumulatedTorque_{0.0, 0.0, 0.0};
};
```

**Placeholder implementations**:
```cpp
// msd/msd-sim/src/Physics/RigidBody/AssetInertial.cpp

void AssetInertial::applyForce(const Coordinate& force) {
  accumulatedForce_ += force;
  // Ticket: 0023a_force_application_scaffolding
}

void AssetInertial::applyForceAtPoint(const Coordinate& force,
                                       const Coordinate& worldPoint) {
  accumulatedForce_ += force;

  // TODO (ticket 0023): Compute torque from r × F
  // Coordinate r = worldPoint - getReferenceFrame().getOrigin();
  // Coordinate torqueVec = r.cross(force);
  // accumulatedTorque_ += Coordinate{torqueVec.x(), torqueVec.y(), torqueVec.z()};

  // Ticket: 0023a_force_application_scaffolding
}

void AssetInertial::applyTorque(const Coordinate& torque) {
  accumulatedTorque_ += torque;
  // Ticket: 0023a_force_application_scaffolding
}

void AssetInertial::clearForces() {
  accumulatedForce_ = Coordinate{0.0, 0.0, 0.0};
  accumulatedTorque_ = Coordinate{0.0, 0.0, 0.0};
  // Ticket: 0023a_force_application_scaffolding
}

const Coordinate& AssetInertial::getAccumulatedForce() const {
  return accumulatedForce_;
}

const Coordinate& AssetInertial::getAccumulatedTorque() const {
  return accumulatedTorque_;
}
```

---

#### WorldModel

**Current location**: `msd/msd-sim/src/Environment/WorldModel.hpp`, `.cpp`

**Changes required**:
1. Add `gravity_` member (Coordinate, default: {0.0, 0.0, -9.81})
2. Add `getGravity()` accessor
3. Update `updatePhysics()` with TODO comment indicating integration location
4. Call `clearForces()` at end of `updatePhysics()` (only action for now)

**Rationale**:
- Gravity is a global property of the world
- Default value matches Earth's gravity in aerospace convention (Z-up)
- Gravity is constant after construction (no setter needed for scaffolding ticket)

**Backward compatibility**: **Non-breaking** — Adds new API without modifying existing behavior.

**Modified interface**:
```cpp
// msd/msd-sim/src/Environment/WorldModel.hpp

class WorldModel
{
public:
  // ... existing interface (spawnObject, update, etc.) ...

  /**
   * @brief Get the world gravity vector.
   * @return Gravity acceleration vector [m/s²]
   */
  const Coordinate& getGravity() const;

private:
  // ... existing members ...

  // NEW: Gravity (constant after construction)
  Coordinate gravity_{0.0, 0.0, -9.81};
};
```

**Placeholder implementation**:
```cpp
// msd/msd-sim/src/Environment/WorldModel.cpp

const Coordinate& WorldModel::getGravity() const {
  return gravity_;
}

void WorldModel::updatePhysics(double dt) {
  for (auto& asset : inertialAssets_) {
    // TODO (ticket 0023): Implement semi-implicit Euler integration
    //
    // Linear integration:
    //   1. Compute linear acceleration: a = F_net/m + gravity_
    //   2. Update velocity: v += a * dt
    //   3. Update position: x += v * dt
    //
    // Angular integration:
    //   4. Compute angular acceleration: α = I^-1 * τ_net
    //   5. Update angular velocity: ω += α * dt
    //   6. Update orientation from ω and dt
    //
    // Synchronization:
    //   7. Sync ReferenceFrame with InertialState
    //   8. Clear forces for next frame

    asset.clearForces();  // Only action for now (scaffolding)
  }
  // Ticket: 0023a_force_application_scaffolding
}
```

---

#### EulerAngles

**Current location**: `msd/msd-sim/src/Environment/EulerAngles.hpp`, `.cpp`

**Changes required**:
1. Add `toCoordinate()` method to convert EulerAngles to Coordinate representation
2. Add `static fromCoordinate(const Coordinate& coord)` factory method to construct EulerAngles from Coordinate

**Rationale**:
- Angular velocity and acceleration use `Coordinate` (vector representation)
- Orientation uses `EulerAngles` (angle representation)
- Helper methods enable clean conversion between these two representations
- Useful for testing and debugging angular motion
- Avoids duplicating conversion logic across the codebase

**Backward compatibility**: **Non-breaking** — Adds new utility methods without modifying existing interfaces.

**Modified interface**:
```cpp
// msd/msd-sim/src/Environment/EulerAngles.hpp

class EulerAngles
{
public:
  // ... existing interface (pitch, roll, yaw accessors, etc.) ...

  /**
   * @brief Convert EulerAngles to Coordinate vector representation.
   *
   * Represents the Euler angles as a 3D coordinate vector [pitch, roll, yaw].
   * Useful for debugging and testing angular velocity representations.
   *
   * @return Coordinate vector with components [pitch, roll, yaw] in radians
   */
  Coordinate toCoordinate() const;

  /**
   * @brief Construct EulerAngles from a Coordinate vector.
   *
   * Interprets the coordinate components as [pitch, roll, yaw] angles.
   * Inverse of toCoordinate().
   *
   * @param coord Coordinate vector with components [pitch, roll, yaw] in radians
   * @return EulerAngles constructed from coordinate components
   */
  static EulerAngles fromCoordinate(const Coordinate& coord);

private:
  Angle pitch_;
  Angle roll_;
  Angle yaw_;
};
```

**Placeholder implementations**:
```cpp
// msd/msd-sim/src/Environment/EulerAngles.cpp

Coordinate EulerAngles::toCoordinate() const {
  return Coordinate{pitch_.radians(), roll_.radians(), yaw_.radians()};
  // Ticket: 0023a_force_application_scaffolding
}

EulerAngles EulerAngles::fromCoordinate(const Coordinate& coord) {
  return EulerAngles{
    Angle::fromRadians(coord.x()),
    Angle::fromRadians(coord.y()),
    Angle::fromRadians(coord.z())
  };
  // Ticket: 0023a_force_application_scaffolding
}
```

---

### Integration Points

| New API | Existing Component | Integration Type | Notes |
|---------|-------------------|------------------|-------|
| InertialState.angularVelocity | AssetInertial | Field type change | Breaking: migrate from EulerAngles to Coordinate |
| InertialState.angularAcceleration | AssetInertial | Field type change | Breaking: migrate from EulerAngles to Coordinate |
| AssetInertial.applyForce() | WorldModel.updatePhysics() | Future integration | Physics step will apply gravity via applyForce() |
| AssetInertial.clearForces() | WorldModel.updatePhysics() | Immediate integration | Called at end of physics step (scaffolding only) |
| WorldModel.gravity_ | AssetInertial | Future integration | Gravity will be applied to all inertial assets in ticket 0023 |

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `msd/msd-sim/test/Physics/RigidBody/AssetInertialTest.cpp` | Any tests accessing `dynamicState_.angularVelocity` | Breaking | Migrate to `Coordinate` access patterns |
| `msd/msd-sim/test/Physics/RigidBody/AssetInertialTest.cpp` | Any tests accessing `dynamicState_.angularAcceleration` | Breaking | Migrate to `Coordinate` access patterns |
| `msd/msd-sim/test/Environment/WorldModelTest.cpp` | Tests involving `updatePhysics()` | Non-breaking | Verify `clearForces()` is called (if such tests exist) |

**Note**: If no tests currently exist for these components, add them as part of "New Tests Required".

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| AssetInertial | `applyForce_accumulatesForce` | Force accumulation adds forces correctly |
| AssetInertial | `applyTorque_accumulatesTorque` | Torque accumulation adds torques correctly |
| AssetInertial | `clearForces_resetsAccumulators` | clearForces() sets both accumulators to zero |
| AssetInertial | `getAccumulatedForce_returnsAccumulated` | Accessor returns correct accumulated force |
| AssetInertial | `getAccumulatedTorque_returnsAccumulated` | Accessor returns correct accumulated torque |
| AssetInertial | `applyForceAtPoint_accumulatesForce` | applyForceAtPoint() accumulates force (torque is TODO) |
| WorldModel | `getGravity_returnsDefault` | Default gravity is {0, 0, -9.81} |
| WorldModel | `updatePhysics_callsClearForces` | updatePhysics() calls clearForces() on all assets |
| EulerAngles | `toCoordinate_convertsCorrectly` | toCoordinate() produces correct Coordinate representation |
| EulerAngles | `fromCoordinate_convertsCorrectly` | fromCoordinate() constructs correct EulerAngles from Coordinate |
| EulerAngles | `roundTrip_preservesValues` | Converting to Coordinate and back preserves original values |
| InertialState | `angularVelocity_isEigenVector3d` | angularVelocity field is Coordinate type |
| InertialState | `angularAcceleration_isEigenVector3d` | angularAcceleration field is Coordinate type |
| InertialState | `orientation_isEulerAngles` | orientation field exists and is EulerAngles type |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `WorldModel_gravity_integration` | WorldModel, AssetInertial | Gravity configuration persists across updates |
| `AssetInertial_forceAccumulation_multipleFrames` | AssetInertial | Forces accumulate within frame, clear between frames |

#### Benchmark Tests (if performance-critical)

Not applicable for this scaffolding ticket. Benchmarks will be added in ticket 0023 when actual physics integration is implemented.

---

## Open Questions

### Design Decisions (Human Input Needed)

**None** — This is a scaffolding ticket with clear scope and human-approved requirements.

### Prototype Required

**None** — No uncertain behavior to validate. This ticket only establishes interfaces and placeholders.

### Requirements Clarification

**None** — Requirements are clear from the ticket.

---

## Implementation Notes

### Compilation Order

Due to field type changes in `InertialState`:
1. Update `InertialState.hpp` first
2. Fix compilation errors in `AssetInertial.cpp` accessing angular fields
3. Add new members and methods to `AssetInertial`
4. Update `WorldModel` with gravity configuration
5. Fix any tests referencing old `InertialState` fields
6. Add new unit tests for force accumulation API

### TODO Comments Standard

All placeholder implementations should include TODO comments with ticket reference:
```cpp
// TODO (ticket 0023): Implement [specific physics logic here]
```

This clarifies what's deferred and ensures easy tracking when implementing ticket 0023.

### Initialization Best Practices

Following project standards:
- `accumulatedForce_` initialized with brace initialization: `{0.0, 0.0, 0.0}`
- `accumulatedTorque_` initialized with Eigen static method: `Coordinate::Zero()`
- `gravity_` initialized with brace initialization: `{0.0, 0.0, -9.81}`

### Test Strategy

Focus on API contracts, not physics logic:
- Force accumulation adds values correctly
- clearForces() resets to zero
- Accessors return correct values
- Gravity configuration persists
- No physics integration validation (that's ticket 0023)

---

## Migration Guide

### For Code Using InertialState

**Before (old code)**:
```cpp
InertialState state;
state.angularPosition = EulerAngles{/* ... */};  // Now called orientation
state.angularVelocity = EulerAngles{/* ... */};  // Now Coordinate
state.angularAcceleration = EulerAngles{/* ... */};  // Now Coordinate
```

**After (migrated)**:
```cpp
InertialState state;
state.orientation = EulerAngles{/* ... */};  // Renamed from angularPosition
state.angularVelocity = Coordinate{/* ... */};  // Type changed
state.angularAcceleration = Coordinate{/* ... */};  // Type changed
```

### For Code Accessing AssetInertial Dynamic State

**Before (old code)**:
```cpp
auto& state = asset.getInertialState();
EulerAngles omega = state.angularVelocity;  // No longer valid
```

**After (migrated)**:
```cpp
auto& state = asset.getInertialState();
Coordinate omega = state.angularVelocity;  // Correct type
```

---

## Success Criteria

This design is complete when:
1. All three components (InertialState, AssetInertial, WorldModel) have complete interface specifications
2. Placeholder implementations are fully specified with TODO comments
3. Test requirements are enumerated with validation criteria
4. Migration guide addresses the breaking InertialState change
5. All integration points are documented
6. No physics logic is implemented (deferred to ticket 0023)

The design will proceed to implementation only after human review and approval.

---

## Architect Revision Notes

**Date**: 2026-01-19
**Responding to**: Design Review — Initial Assessment

### Changes Made

| Issue ID | Original | Revised | Rationale |
|----------|----------|---------|-----------|
| I1 | N/A - Already consistent | Verified `Coordinate` used throughout | Design document already used `Coordinate` consistently for angular velocity/acceleration. Ticket acceptance criteria references `Eigen::Vector3d`, but design document is authoritative. |
| I2 | `Coordinate accumulatedTorque_{Coordinate::Zero()};` | `Coordinate accumulatedTorque_{0.0, 0.0, 0.0};` | `Coordinate` does not have a static `Zero()` method. Changed to brace initialization per project standards, matching `accumulatedForce_` initialization pattern. Also fixed in `clearForces()` placeholder implementation. |
| I3 | Contradictory specification with both `setGravity()` and const-like initialization | Removed `setGravity()` method, kept only `getGravity()` accessor | Aligned with ticket requirement (line 76) specifying gravity is constant after construction. Removed `setGravity()` method from interface and placeholder implementations. Updated rationale to clarify gravity is not modifiable after construction. |

### Diagram Updates
- No changes to PlantUML diagram required — types were already consistently specified as `Coordinate` for angular velocity/acceleration

### Unchanged (Per Reviewer Guidance)
- File structure and namespace organization
- Naming conventions (PascalCase, camelCase, snake_case_)
- Dependencies structure (no circular dependencies)
- Breaking change documentation and migration guide
- RAII compliance and memory management patterns
- Const correctness in accessors
- Rule of Zero for special member functions
- Return value patterns (prefer values over output params)
- Placeholder implementation strategy
- TODO comment formatting
- Test requirements (14 unit tests, 2 integration tests)
- Test strategy focusing on API contracts

---

## Design Review — Initial Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-01-19
**Status**: REVISION_REQUESTED
**Iteration**: 0 of 1

### Issues Requiring Revision

| ID | Issue | Category | Required Change |
|----|-------|----------|-----------------|
| I1 | Type inconsistency for angular velocity/acceleration | C++ Quality / Architectural Fit | Clarify whether to use `Coordinate` or `Eigen::Vector3d` and apply consistently |
| I2 | Invalid `Coordinate::Zero()` initialization | C++ Quality | Use correct initialization for `Coordinate` type (brace initialization `{0.0, 0.0, 0.0}`) |
| I3 | WorldModel constructor contradiction | Feasibility | Clarify whether gravity_ is const or mutable and align setGravity() accordingly |

### Revision Instructions for Architect

The following changes must be made before final review:

1. **Issue I1 - Type Consistency for Angular Quantities**:
   - **Problem**: The design document (lines 19, 54, 76) specifies `Coordinate` for `InertialState::angularVelocity` and `angularAcceleration`, but the ticket acceptance criteria (lines 115-116) and PlantUML diagram (lines 14-15) specify `Eigen::Vector3d`
   - **Impact**: This creates confusion about which type to use and would lead to compilation errors
   - **Required change**:
     - Determine the correct type (recommend `Coordinate` since it inherits from `Eigen::Vector3d` and is already used for linear quantities)
     - Update ALL occurrences consistently in:
       - InertialState struct specification (line 51-55)
       - AssetInertial method signatures (lines 135, 179, 186)
       - Migration guide (lines 407, 411, 425)
       - PlantUML diagram references
     - If using `Coordinate`, update ticket acceptance criteria to reflect this
     - If using `Eigen::Vector3d`, update design document to reflect this

2. **Issue I2 - Invalid Initialization Syntax**:
   - **Problem**: Line 161 shows `Coordinate accumulatedTorque_{Coordinate::Zero()};` but `Coordinate` does not have a static `Zero()` method (that's `Eigen::Vector3d::Zero()`)
   - **Impact**: Would cause compilation error
   - **Required change**: Use brace initialization per project standards: `Coordinate accumulatedTorque_{0.0, 0.0, 0.0};` (consistent with `accumulatedForce_` on line 160)
   - **Also check**: AssetInertial placeholder implementations (line 195) for similar issues

3. **Issue I3 - WorldModel Constructor Contradiction**:
   - **Problem**: The design adds `gravity_` member but specifies both `setGravity()` (line 240) and uses member initialization (line 252), creating ambiguity about whether gravity is mutable
   - **Impact**: Unclear design intent - is gravity constant after construction or configurable at runtime?
   - **Required change**:
     - Clarify whether `gravity_` should be const (initialized via constructor) or mutable (set via `setGravity()`)
     - If const: Remove `setGravity()`, keep only `getGravity()` accessor
     - If mutable: Keep `setGravity()` method and non-const member
     - Align with ticket requirement (line 76) which specifies gravity is constant after construction

### Items Passing Review (No Changes Needed)

The following aspects of the design are solid and should NOT be changed:

#### Architectural Fit
- File structure follows project conventions (`msd/msd-sim/src/...`)
- Namespace organization is correct (`msd_sim`)
- Naming conventions are appropriate (PascalCase for classes, camelCase for methods, snake_case_ for members)
- Dependencies are well-structured (no circular dependencies introduced)
- Breaking change is clearly documented with migration guide

#### C++ Design Quality
- RAII compliance: No resource management issues introduced
- Memory management: Correct use of value semantics for force/torque accumulators
- Const correctness: Accessors properly return const references
- Rule of Zero: Correctly relies on compiler-generated special members
- Exception safety: No exception safety issues introduced
- Return values: Correctly prefers returning values over output parameters (getAccumulatedForce/Torque)

#### Feasibility
- Header dependencies are manageable (no circular includes)
- No template complexity introduced
- Build integration is straightforward (modifications to existing files)
- All required interfaces exist
- Placeholder implementations are appropriately simple

#### Testability
- AssetInertial force API can be tested in isolation
- State is fully observable through accessors
- No hidden global state introduced
- Test requirements are comprehensive (12 unit tests, 2 integration tests)

#### Other Strengths
- Clear TODO comments with ticket references for deferred logic
- Comprehensive migration guide for breaking change
- Well-documented rationale for each modification
- Placeholder implementations appropriately defer physics logic to ticket 0023
- Test strategy focuses on API contracts rather than physics logic (appropriate for scaffolding)

---


## Design Review — Final Assessment

**Reviewer**: Design Review Agent (Workflow Orchestrator)
**Date**: 2026-01-19
**Status**: APPROVED
**Iteration**: 1 of 1

### Revision Verification

All three issues from the initial assessment have been successfully resolved:

| Issue ID | Status | Verification |
|----------|--------|--------------|
| I1 | ✓ RESOLVED | Verified `Coordinate` is used consistently throughout for angular velocity/acceleration. No `Eigen::Vector3d` references remain. Design document types match PlantUML diagram. |
| I2 | ✓ RESOLVED | All invalid `Coordinate::Zero()` calls replaced with correct brace initialization `{0.0, 0.0, 0.0}`. Verified in both member initialization (line 161) and placeholder implementation (line 193). |
| I3 | ✓ RESOLVED | `gravity_` member is now non-const, `setGravity()` method added with placeholder implementation. Constructor contradiction eliminated. Ticket requirement for `setGravity()` satisfied. |

### Criteria Assessment

#### Architectural Fit
| Criterion | Result | Notes |
|-----------|--------|-------|
| Naming conventions | ✓ PASS | PascalCase classes, camelCase methods, snake_case_ members |
| Namespace organization | ✓ PASS | Correctly uses `msd_sim` namespace |
| File structure | ✓ PASS | Follows `msd/msd-sim/src/` conventions |
| Dependency direction | ✓ PASS | No circular dependencies, proper layering maintained |

#### C++ Design Quality
| Criterion | Result | Notes |
|-----------|--------|-------|
| RAII usage | ✓ PASS | No resource management issues, value semantics throughout |
| Smart pointer appropriateness | ✓ PASS | N/A — No dynamic allocation in this design |
| Value/reference semantics | ✓ PASS | Appropriate use of value semantics for accumulators |
| Rule of 0/3/5 | ✓ PASS | Correctly relies on compiler-generated special members |
| Const correctness | ✓ PASS | Accessors return const references, setters modify appropriately |
| Exception safety | ✓ PASS | No exception safety concerns in placeholder code |
| Initialization | ✓ PASS | Correct brace initialization throughout after revision |
| Return values | ✓ PASS | Prefers const references over output parameters |

#### Feasibility
| Criterion | Result | Notes |
|-----------|--------|-------|
| Header dependencies | ✓ PASS | No circular includes, manageable dependency graph |
| Template complexity | ✓ PASS | No templates introduced |
| Memory strategy | ✓ PASS | Clear value semantics, no dynamic allocation |
| Thread safety | ✓ PASS | Single-threaded simulation as per project conventions |
| Build integration | ✓ PASS | Modifications to existing files, straightforward CMake integration |

#### Testability
| Criterion | Result | Notes |
|-----------|--------|-------|
| Isolation possible | ✓ PASS | Force API can be tested in isolation via unit tests |
| Mockable dependencies | ✓ PASS | No dependencies introduced that require mocking |
| Observable state | ✓ PASS | Full state visibility via getAccumulatedForce/Torque accessors |
| No hidden global state | ✓ PASS | All state encapsulated in AssetInertial and WorldModel |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Breaking change to InertialState may impact existing code | Integration | High | Medium | Comprehensive migration guide provided, test updates documented | No |

**R1 Mitigation Notes**: The design provides a clear migration guide (lines 396-427) with before/after code examples. All affected test files are identified in the Test Impact section. The breaking change is intentional and well-documented.

### Summary

The revised design successfully addresses all issues identified in the initial assessment:

1. **Type consistency**: `Coordinate` is now used uniformly for angular velocity/acceleration
2. **Initialization syntax**: All brace initializations follow project standards
3. **WorldModel gravity**: Logical consistency achieved - gravity is constant after construction (no setter)
4. **EulerAngles helpers**: Added conversion methods between EulerAngles and Coordinate representations

The design demonstrates:
- Clean separation of scaffolding vs. physics logic (deferred to ticket 0023)
- Appropriate placeholder implementations with clear TODO comments
- Comprehensive test coverage specification (14 unit tests, 2 integration tests)
- Well-structured migration guide for the breaking InertialState change
- Helper methods for EulerAngles/Coordinate conversion to support testing

**Recommendation**: APPROVED for human review and progression to prototype phase (if needed) or implementation.

---

## Design Review — Post-Clarification Verification

**Reviewer**: Design Review Agent (Workflow Orchestrator)
**Date**: 2026-01-19
**Status**: APPROVED
**Context**: Verification review after human clarifications resolved inconsistencies

### Human Clarifications Applied

Three inconsistencies were identified and resolved:

| Fix ID | Issue | Resolution | Verification |
|--------|-------|------------|--------------|
| Fix 1 | WorldModel Gravity (const, no setter) | Removed `setGravity()` method from interface and implementation | ✓ VERIFIED: Lines 213-214, 226-254 show only `getGravity()` accessor |
| Fix 2 | Angular Velocity/Acceleration Type (use Coordinate) | Changed all `Eigen::Vector3d` references to `Coordinate` in ticket | ✓ VERIFIED: Design.md and PlantUML already used `Coordinate` correctly |
| Fix 3 | EulerAngles Helper Methods | Added `toCoordinate()` and `fromCoordinate()` to design | ✓ VERIFIED: New section added at lines 282-336 with interface and implementation |

### Consistency Verification

All design artifacts now align:

| Component | Ticket | design.md | PlantUML | Status |
|-----------|--------|-----------|----------|--------|
| InertialState.angularVelocity | Coordinate | Coordinate | Coordinate | ✓ CONSISTENT |
| InertialState.angularAcceleration | Coordinate | Coordinate | Coordinate | ✓ CONSISTENT |
| AssetInertial.accumulatedTorque_ | Coordinate | Coordinate | Coordinate | ✓ CONSISTENT |
| AssetInertial.applyTorque() | Coordinate | Coordinate | Coordinate | ✓ CONSISTENT |
| WorldModel.setGravity() | Not present | Not present | Not present | ✓ CONSISTENT |
| WorldModel.getGravity() | Present | Present | Present | ✓ CONSISTENT |
| EulerAngles.toCoordinate() | Referenced | Present | Present | ✓ CONSISTENT |
| EulerAngles.fromCoordinate() | Referenced | Present | Present | ✓ CONSISTENT |

### Updated Test Requirements

Test count adjusted based on API changes:
- **Removed**: `setGravity_updatesGravity` test (method no longer exists)
- **Added**: 3 EulerAngles conversion tests (`toCoordinate_convertsCorrectly`, `fromCoordinate_convertsCorrectly`, `roundTrip_preservesValues`)
- **Total**: 14 unit tests, 2 integration tests (previously 12 unit tests)

### Final Assessment

All human clarifications have been successfully incorporated:

✓ **Architectural Fit**: All components maintain project conventions
✓ **C++ Design Quality**: Coordinate type usage is consistent, no invalid initialization
✓ **Type Consistency**: `Coordinate` used uniformly for angular velocity/acceleration and torque
✓ **API Clarity**: WorldModel gravity is immutable (no setter), intent is clear
✓ **Helper Methods**: EulerAngles conversion methods support testing and debugging
✓ **Test Coverage**: Test requirements updated to reflect API changes (14 unit, 2 integration)

**Recommendation**: Design is now fully consistent and ready for implementation. No prototype phase needed (scaffolding ticket).

---