# Design: Constraint Solver Unification

## Summary

Ticket 0043 unified the `Constraint` base class interface, eliminating the `TwoBodyConstraint` intermediate class. However, `ConstraintSolver` still maintains two completely separate solver paths from the pre-refactor era: a single-body position-level path for `UnitQuaternionConstraint` and a multi-body velocity-level path for contact constraints. This design eliminates the redundant single-body path, renames the multi-body methods to drop the "Contact" prefix, and removes the integrator's dependency on constraint solving since quaternion normalization via `state.orientation.normalize()` already handles drift correction.

## Architecture Changes

### PlantUML Diagram
See: `./0045_constraint_solver_unification.puml`

### Modified Components

#### ConstraintSolver
- **Current location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`, `.cpp`
- **Changes required**:
  1. **Delete single-body solver path**:
     - Remove static `solve()` method (7-DOF position-level)
     - Remove `SolveResult` struct (single-body version)
     - Remove `assembleConstraintMatrix()` helper
     - Remove single-body `assembleRHS()` overload
     - Remove `extractConstraintForces()` helper
     - Remove `kNumStates` constant
  2. **Rename multi-body methods** (drop "Contact" prefix):
     - `solveWithContacts()` → `solve()`
     - `MultiBodySolveResult` → `SolveResult`
     - `assembleContactJacobians()` → `assembleJacobians()`
     - `assembleContactEffectiveMass()` → `assembleEffectiveMass()`
     - `assembleContactRHS()` → `assembleRHS()`
     - `extractContactBodyForces()` → `extractBodyForces()`
  3. **No signature changes**: The renamed `solve()` method retains the same parameters as the old `solveWithContacts()` (multi-body velocity-level formulation)
  4. **Keep unchanged**: `solveActiveSet()`, `solveWithECOS()`, `buildFrictionConeSpec()`, solver configuration methods, member variables
- **Backward compatibility**: Breaking change — callers using `solveWithContacts()` or `MultiBodySolveResult` must update to renamed versions

#### Integrator (base class)
- **Current location**: `msd/msd-sim/src/Physics/Integration/Integrator.hpp`
- **Changes required**:
  1. Remove `const std::vector<Constraint*>& constraints` parameter from pure virtual `step()` method
  2. Update signature to: `virtual InertialState step(const InertialState& state, const Coordinate& externalForce, const Coordinate& externalTorque, double dt) = 0;`
- **Backward compatibility**: Breaking change — all integrator implementations must update

#### SemiImplicitEulerIntegrator
- **Current location**: `msd/msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp`, `.cpp`
- **Changes required**:
  1. Remove `ConstraintSolver solver_` member variable
  2. Remove `SemiImplicitEulerIntegrator(ConstraintSolver)` constructor
  3. Remove constraints parameter from `step()` override
  4. Delete constraint solving block (lines 33-53 in `.cpp`)
  5. Keep quaternion normalization at line 78 (this becomes the sole drift correction mechanism)
- **Backward compatibility**: Breaking change — callers must remove constraints argument

#### AssetInertial
- **Current location**: `msd/msd-sim/src/Physics/RigidBody/AssetInertial.hpp`, `.cpp`
- **Changes required**:
  1. Remove the two `constraints_.push_back(std::make_unique<UnitQuaternionConstraint>(...))` calls from constructors (lines 45-46 and 91-92 in `.cpp`)
  2. Remove `#include "UnitQuaternionConstraint.hpp"` from `.cpp`
  3. Keep `constraints_` member (users can still add custom constraints)
  4. Keep `UnitQuaternionConstraint` class itself (used in standalone tests)
- **Backward compatibility**: Behavior change — `AssetInertial` now defaults to empty constraint list instead of including `UnitQuaternionConstraint`

#### WorldModel
- **Current location**: `msd/msd-sim/src/Environment/WorldModel.hpp`, `.cpp`
- **Changes required**:
  1. Remove "Step 2: Gather Constraints" block from `updatePhysics()` (line 169 in `.cpp`)
  2. Remove `constraints` argument from `integrator_->step()` call (line 174)
- **Backward compatibility**: No interface changes to public API

#### CollisionPipeline
- **Current location**: `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp`, `.cpp`
- **Changes required**:
  1. Rename all references to `solveWithContacts()` → `solve()`
  2. Rename all references to `MultiBodySolveResult` → `SolveResult`
- **Backward compatibility**: Internal implementation change, no public API impact

### Integration Points

| Modified Component | Modified Component | Integration Type | Notes |
|-------------------|-------------------|------------------|-------|
| CollisionPipeline | ConstraintSolver | Method call | Updates to `solve()` and `SolveResult` names |
| WorldModel | Integrator | Method call | Removes constraints argument from `step()` |
| WorldModel | AssetInertial | Data access | No longer gathers constraints for integrator |
| SemiImplicitEulerIntegrator | ConstraintSolver | Dependency removal | No longer owns or uses solver instance |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `ConstraintTest.cpp` | `EmptyConstraintSet_ReturnsZeroForces` | Uses deleted single-body `solve()` | Delete test |
| `ConstraintTest.cpp` | `SingleQuaternionConstraint_Converges` | Uses deleted single-body `solve()` | Delete test |
| `ConstraintTest.cpp` | `MultipleConstraints_Converges` | Uses deleted single-body `solve()` | Delete test |
| `ConstraintTest.cpp` | `ConditionNumber_WellConditioned` | Uses deleted single-body `solve()` | Delete test |
| `ConstraintTest.cpp` | `DefaultConstraint_IsUnitQuaternion` | Expects `UnitQuaternionConstraint` in default list | Update to expect empty constraint list |
| `ConstraintTest.cpp` | Any test asserting `typeName() == "UnitQuaternionConstraint"` | Default constraints changed | Update assertions |
| `ConstraintSolverContactTest.cpp` | All 28 tests | Use `solveWithContacts()` and `MultiBodySolveResult` | Rename to `solve()` and `SolveResult` |
| `ConstraintSolverASMTest.cpp` | All 12 tests | Use `solveWithContacts()` and `MultiBodySolveResult` | Rename to `solve()` and `SolveResult` |
| `JacobianLinearTest.cpp` | All 6 tests | Use `solveWithContacts()` and `MultiBodySolveResult` | Rename to `solve()` and `SolveResult` |
| `SplitImpulseTest.cpp` | All 2 tests | Use `solveWithContacts()` and `MultiBodySolveResult` | Rename to `solve()` and `SolveResult` |

### New Tests Required

No new tests are required. This is pure refactoring (dead code removal + renames).

#### Unit Tests
Not applicable — existing test coverage is sufficient.

#### Integration Tests
Not applicable — existing collision tests validate the unified solver path.

#### Benchmark Tests
Not applicable — no performance changes expected.

## Risk Assessment

### Behavioral Changes

1. **Quaternion drift correction**:
   - **Before**: `UnitQuaternionConstraint` computed Lagrange multipliers to enforce `||Q|| = 1`, resulting in small angular velocity corrections each frame. Then `state.orientation.normalize()` immediately re-normalized the quaternion.
   - **After**: Only `state.orientation.normalize()` runs (position-level correction). No constraint forces.
   - **Expected impact**: At typical timesteps (16.67ms), position-level normalization is sufficient. Long-running integration tests may show sub-epsilon numerical differences due to the removal of velocity-level correction.
   - **Mitigation**: If quaternion drift becomes observable in production, add explicit damping to angular velocity or reduce timestep. The removed constraint path was redundant with normalization.

2. **Empty default constraints**:
   - **Before**: `AssetInertial` constructors populated `constraints_` with `UnitQuaternionConstraint`.
   - **After**: `constraints_` defaults to empty vector.
   - **Expected impact**: Users who relied on the default constraint will now see empty lists. Since the constraint was redundant with normalization, this should cause no behavioral change.
   - **Mitigation**: Document that quaternion normalization is handled by the integrator, not constraints.

### Breaking Changes

1. **ConstraintSolver API**:
   - Methods `solveWithContacts()` and `MultiBodySolveResult` are renamed.
   - Single-body `solve()` is deleted.
   - **Impact**: All test files referencing the old names must be updated (58 test cases across 5 files).

2. **Integrator API**:
   - `step()` signature no longer accepts `constraints` parameter.
   - **Impact**: `WorldModel` and any custom integrator implementations must update.

3. **AssetInertial default behavior**:
   - Default constraint list changes from `[UnitQuaternionConstraint]` to `[]`.
   - **Impact**: Tests asserting on default constraints must update expectations.

### Complexity Reduction

- **Deleted code**: ~250 lines (single-body solver path + helpers)
- **Simplified integrator**: Removes constraint solving logic, making `SemiImplicitEulerIntegrator` a pure integrator
- **Unified naming**: Single `solve()` method and `SolveResult` type, eliminating confusion between solver paths

## Open Questions

### Design Decisions (Human Input Needed)
None — this design follows the ticket's proposal directly.

### Prototype Required
None — this is straightforward refactoring with no algorithmic changes.

### Requirements Clarification
None — acceptance criteria are clear from the ticket.

## Implementation Notes

### Order of Changes

To minimize intermediate build failures, implement changes in this order:

1. **Phase 1 — Create renamed aliases** (temporary backwards compatibility):
   - Add `using SolveResult = MultiBodySolveResult;` to create temporary alias
   - Add `solve()` method that delegates to `solveWithContacts()`
   - This allows incremental migration of callers

2. **Phase 2 — Update all callers**:
   - Update all test files to use `solve()` and `SolveResult`
   - Update `CollisionPipeline` to use renamed methods
   - Update `WorldModel` and integrator callers

3. **Phase 3 — Delete old single-body path**:
   - Remove old `solve()` (single-body), `SolveResult` (single-body), and helpers
   - Remove constraints from integrator interface and implementation
   - Remove default `UnitQuaternionConstraint` from `AssetInertial`

4. **Phase 4 — Finalize renames**:
   - Rename `solveWithContacts()` → `solve()` (remove delegation)
   - Rename `MultiBodySolveResult` → `SolveResult` (remove alias)
   - Rename all helper methods to drop "Contact" prefix
   - Update documentation strings

This phased approach allows running tests between each step to catch issues early.

### File Modification Checklist

| File | Lines to Modify | Notes |
|------|----------------|-------|
| `ConstraintSolver.hpp` | 110-116, 63-86, 335-372, 495 | Delete single-body path, rename multi-body |
| `ConstraintSolver.cpp` | 37-282 | Delete single-body implementations |
| `Integrator.hpp` | `step()` signature | Remove constraints parameter |
| `SemiImplicitEulerIntegrator.hpp` | Constructor, `step()` signature, `solver_` member | Remove constraint solving |
| `SemiImplicitEulerIntegrator.cpp` | 33-53 | Delete constraint solving block |
| `AssetInertial.cpp` | 45-46, 91-92 | Remove default `UnitQuaternionConstraint` |
| `WorldModel.cpp` | 169, 174 | Remove constraint gathering |
| `CollisionPipeline.hpp` | Type references | Rename `MultiBodySolveResult` → `SolveResult` |
| `CollisionPipeline.cpp` | Method calls | Rename `solveWithContacts()` → `solve()` |
| `ConstraintTest.cpp` | 4 tests + assertions | Delete solver tests, update default constraint tests |
| `ConstraintSolverContactTest.cpp` | 28 tests | Rename types/methods |
| `ConstraintSolverASMTest.cpp` | 12 tests | Rename types/methods |
| `JacobianLinearTest.cpp` | 6 tests | Rename types/methods |
| `SplitImpulseTest.cpp` | 2 tests | Rename types/methods |

### Testing Strategy

1. **Incremental validation**: Run full test suite after each phase
2. **Behavioral regression check**: Compare integration test outputs before/after (expect sub-epsilon differences only)
3. **Collision test coverage**: Existing contact constraint tests validate the unified solver path
4. **Quaternion drift monitoring**: Run long-duration integration tests to verify normalization suffices

## Acceptance Criteria Verification

| Criterion | How Verified |
|-----------|--------------|
| Single public `solve()` method | Inspect `ConstraintSolver.hpp` — no `solveWithContacts()` overload |
| Single `SolveResult` type | Grep for `MultiBodySolveResult` returns zero matches |
| Helper methods use generic names | Grep for `assembleContact*` returns zero matches |
| `step()` has no constraints parameter | Inspect `Integrator.hpp` and `SemiImplicitEulerIntegrator.hpp` |
| `AssetInertial` defaults to empty constraints | Inspect `AssetInertial.cpp` constructors |
| Quaternion normalization works | Existing integration tests pass |
| All tests pass (minus deleted ones) | Full test suite passes |
| Zero collision behavioral change | Collision test outputs unchanged (bit-exact) |
