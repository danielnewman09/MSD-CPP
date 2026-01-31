# Ticket 0032a: Two-Body Constraint Infrastructure

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [ ] Documentation Complete — Awaiting Tutorial
- [ ] Merged / Complete

**Current Phase**: Approved — Ready to Merge (code live on branch `0034-active_set_method`)
**Assignee**: N/A
**Created**: 2026-01-29
**Generate Tutorial**: No
**Parent Ticket**: [0032_contact_constraint_refactor](0032_contact_constraint_refactor.md)

---

## Summary

Introduce the foundational classes for constraint-based contact response: a `TwoBodyConstraint` abstract interface for multi-body constraints, a `ContactConstraint` concrete implementation for non-penetration enforcement, a `ContactConstraintFactory` utility for constructing constraints from collision results, and minor extensions to `AssetEnvironment` and `AssetInertial` to support a unified solver path. This ticket covers only the data model and math layer — no solver or integration changes.

---

## Motivation

The existing constraint framework (Ticket 0031) operates on single-body state vectors. Contact constraints couple two bodies and require a two-body interface. This ticket establishes the foundational classes that the PGS solver (0032b) and WorldModel integration (0032c) will consume.

Separating this layer allows:
1. Independent review of the mathematical formulation (Jacobian, Baumgarte, restitution)
2. Unit testing of constraint evaluation without solver or integration dependencies
3. Clean merge that adds new code without changing any existing behavior

---

## Technical Approach

### New Components

#### TwoBodyConstraint (Abstract)

- **Location**: `msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp/.cpp`
- Extends `UnilateralConstraint` with two-body evaluation methods
- Stores body indices (`body_a_index_`, `body_b_index_`) for solver dispatch
- Single-body `evaluate()`/`jacobian()`/`isActive()` throw `std::logic_error` (misuse guard)
- See design document Section "TwoBodyConstraint" for full interface

#### ContactConstraint (Concrete)

- **Location**: `msd-sim/src/Physics/Constraints/ContactConstraint.hpp/.cpp`
- `dimension() = 1` (one constraint row per contact point)
- Constraint function: `C(q) = (x_B - x_A) . n + d >= 0`
- Jacobian (1x12): `J = [-n^T, -(r_A x n)^T, n^T, (r_B x n)^T]`
- Pre-computed lever arms at construction (transient per-frame lifetime)
- Baumgarte: ERP=0.2 velocity-level formulation (per P1 prototype findings)
- Restitution: stores `pre_impact_rel_vel_normal_` (per P2 prototype findings)
- Validates inputs: unit normal, penetration >= 0, restitution in [0, 1]

#### ContactConstraintFactory (Utility Namespace)

- **Location**: `msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp/.cpp`
- `createFromCollision()`: one `ContactConstraint` per contact point in manifold
- `combineRestitution(eA, eB)`: geometric mean `sqrt(eA * eB)`
- `computeRelativeNormalVelocity()`: includes angular contribution `(v_B + w_B x r_B - v_A - w_A x r_A) . n`
- `kRestVelocityThreshold = 0.5` m/s (disables restitution for slow contacts)

### Modified Components

#### AssetEnvironment

- Add `getInverseMass()` returning `0.0` (infinite mass for static bodies)
- Add `getInverseInertiaTensor()` returning `Eigen::Matrix3d::Zero()`
- Add `getInertialState()` returning static zero-velocity state
- Add `getCoefficientOfRestitution()` / `setCoefficientOfRestitution(double e)` with validation

#### AssetInertial

- Add `getInverseMass()` returning `1.0 / mass_` (convenience method)

---

## Requirements

### Functional Requirements

1. **FR-1**: `TwoBodyConstraint` abstract class extends `UnilateralConstraint` with two-body evaluation
2. **FR-2**: `ContactConstraint` implements non-penetration constraint with correct Jacobian
3. **FR-3**: `ContactConstraintFactory` creates one constraint per contact point from `CollisionResult`
4. **FR-4**: `AssetEnvironment` provides inverse mass = 0, inverse inertia = Zero for unified solver
5. **FR-5**: `AssetInertial` provides `getInverseMass()` convenience method

### Non-Functional Requirements

1. **NFR-1**: `ContactConstraint` immutable after construction (thread-safe reads)
2. **NFR-2**: `ContactConstraintFactory` stateless (thread-safe)
3. **NFR-3**: No changes to existing `Constraint`, `BilateralConstraint`, or `UnilateralConstraint` interfaces
4. **NFR-4**: No changes to existing `ConstraintSolver` behavior

---

## Acceptance Criteria

1. [x] AC1: `ContactConstraint` class implements `TwoBodyConstraint` with correct `evaluateTwoBody()` and `jacobianTwoBody()`
2. [x] AC2: Jacobian matches finite-difference numerical verification (within 1e-5 tolerance)
3. [x] AC3: `isActiveTwoBody()` returns true for penetrating pairs, false for separated pairs
4. [x] AC4: `ContactConstraintFactory::createFromCollision()` produces one constraint per contact point
5. [x] AC5: `combineRestitution()` returns geometric mean
6. [x] AC6: `computeRelativeNormalVelocity()` includes angular velocity contribution
7. [x] AC7: `AssetEnvironment::getInverseMass()` returns 0.0
8. [x] AC8: `AssetEnvironment::getInverseInertiaTensor()` returns zero matrix
9. [x] AC9: All existing constraint tests pass without modification
10. [x] AC10: Unit tests cover all new classes (33 test cases)

---

## Implementation Status

### Code Complete ✓
- `TwoBodyConstraint.hpp/.cpp`
- `ContactConstraint.hpp/.cpp`
- `ContactConstraintFactory.hpp/.cpp`
- `AssetEnvironment.hpp/.cpp` modifications
- `AssetInertial.hpp/.cpp` modifications
- `CMakeLists.txt` updated with new sources

### Tests Complete ✓
- `test/Physics/Constraints/ContactConstraintTest.cpp` — 17 ContactConstraint unit tests
- `test/Physics/Constraints/ContactConstraintFactoryTest.cpp` — 8 factory unit tests
- `test/Physics/RigidBody/AssetEnvironmentTest.cpp` — 7 AssetEnvironment tests
- `test/Physics/Constraints/TwoBodyConstraintTest.cpp` — 1 body index accessor test
- **Total**: 33 tests, all passing

---

## Dependencies

- **Ticket 0031**: Generalized Lagrange Multiplier Constraint System (prerequisite — provides Constraint hierarchy)
- **Blocks**: [0032b](0032b_pgs_solver_extension.md), [0032c](0032c_worldmodel_contact_integration.md), [0032d](0032d_collision_response_cleanup.md)

---

## Files

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp` | Abstract two-body constraint interface |
| `msd-sim/src/Physics/Constraints/TwoBodyConstraint.cpp` | Single-body redirect implementations |
| `msd-sim/src/Physics/Constraints/ContactConstraint.hpp` | Contact constraint header |
| `msd-sim/src/Physics/Constraints/ContactConstraint.cpp` | Evaluate, Jacobian, isActive implementations |
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp` | Factory header |
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` | Factory implementations |
| `msd-sim/test/Physics/Constraints/ContactConstraintTest.cpp` | Unit tests |
| `msd-sim/test/Physics/Constraints/ContactConstraintFactoryTest.cpp` | Factory tests |

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp` | Add inverse mass, inverse inertia, state, restitution |
| `msd-sim/src/Physics/RigidBody/AssetEnvironment.cpp` | Implement new methods |
| `msd-sim/src/Physics/RigidBody/AssetInertial.hpp` | Add `getInverseMass()` |
| `msd-sim/src/Physics/RigidBody/AssetInertial.cpp` | Implement `getInverseMass()` |
| `msd-sim/CMakeLists.txt` | Add new source files |

---

## References

- **Design document**: `docs/designs/0032_contact_constraint_refactor/design.md`
- **Math formulation**: `docs/designs/0032_contact_constraint_refactor/math-formulation.md`
- **P1 Debug Findings**: `prototypes/0032_contact_constraint_refactor/p1_pgs_convergence/Debug_Findings.md` (Baumgarte ERP=0.2)
- **P2 Debug Findings**: `prototypes/0032_contact_constraint_refactor/p2_energy_conservation/Debug_Findings.md` (restitution formula)

---

## Workflow Log

### Design Phase
- **Completed**: 2026-01-29
- **Notes**: Design shared with parent ticket 0032. See `docs/designs/0032_contact_constraint_refactor/design.md`.

### Prototype Phase
- **Completed**: 2026-01-29
- **Notes**: Prototypes P1 and P2 validated mathematical formulation. See parent ticket for details.

### Implementation Phase
- **Started**: 2026-01-29
- **Completed**: 2026-01-29
- **Artifacts**:
  - `test/Physics/Constraints/ContactConstraintTest.cpp`
  - `test/Physics/Constraints/ContactConstraintFactoryTest.cpp`
  - `test/Physics/Constraints/TwoBodyConstraintTest.cpp`
  - `test/Physics/RigidBody/AssetEnvironmentTest.cpp`
  - `test/Physics/RigidBody/CMakeLists.txt`
  - Updated `test/Physics/CMakeLists.txt`
  - Updated `test/Physics/Constraints/CMakeLists.txt`
- **Test Results**: 33/33 tests passing (100% pass rate)
- **Notes**: Implementation complete with comprehensive unit test coverage. All acceptance criteria met.

### Quality Gate Phase
- **Started**: 2026-01-29 15:30
- **Completed**: 2026-01-29 16:30
- **Artifacts**:
  - `docs/designs/0032_contact_constraint_refactor/quality-gate-report.md`
- **Status**: PASSED ✓
- **Notes**:
  - Initial run FAILED due to pre-existing technical debt (deprecated functions from ticket 0030)
  - Human decision: Fix technical debt before proceeding (Option 1)
  - Fixed 7 files with deprecated API usage:
    - MotionController.cpp: setRotation() → setQuaternion()
    - GravityPotential.cpp: Unused parameter warnings
    - ConstraintSolver.cpp: Sign conversion warnings
    - Test files: Deprecated API and unused functions
  - Release build: PASSED (all warnings resolved)
  - Debug tests: 379/381 passing (2 pre-existing failures unrelated to ticket)
  - All 33 new tests from ticket 0032a passing (100% pass rate)

---

## Human Feedback

**Quality Gate Blocker (2026-01-29)** ✓ RESOLVED:
Release build failed due to pre-existing deprecated function call in `msd/msd-sim/src/Environment/MotionController.cpp:88`. This file is NOT modified by ticket 0032a. The deprecation was introduced by ticket 0030, but calling code from ticket 0005 was not updated.

**Decision**: Option 1 - Fix unrelated technical debt before proceeding
**Resolution**: All technical debt fixed, quality gate re-run successful, proceeding to Implementation Review phase.
