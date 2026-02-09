# Ticket 0045: Constraint Solver Unification

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Documentation Complete
- [x] Merged / Complete

**Current Phase**: Complete
**Assignee**: TBD
**Created**: 2026-02-08
**Generate Tutorial**: No
**Predecessor**: [0043_constraint_hierarchy_refactor](0043_constraint_hierarchy_refactor.md), [0044_collision_pipeline_integration](0044_collision_pipeline_integration.md)
**Related Tickets**: [0031_generalized_lagrange_constraints](0031_generalized_lagrange_constraints.md), [0032_contact_constraint_refactor](0032_contact_constraint_refactor.md), [0034_active_set_method_contact_solver](0034_active_set_method_contact_solver.md)
**Type**: Refactoring

---

## Problem Statement

Ticket 0043 refactored the `Constraint` base class to have a unified two-body interface (`evaluate(stateA, stateB, time)`, `bodyAIndex()`, `bodyBIndex()`, `bodyCount()`), eliminating the `TwoBodyConstraint` intermediate class. However, `ConstraintSolver` still retains two completely separate solver paths from the pre-refactor era:

### Dual-Path Architecture

**Path 1 — Single-body bilateral solve (`solve()` static method)**:
- 7-DOF position-level formulation (X(3), Q(4))
- LLT direct solve for unconstrained λ
- Helpers: `assembleConstraintMatrix()`, `assembleRHS()`, `extractConstraintForces()`
- Returns `SolveResult` (single-body forces + condition number)
- **Sole production caller**: `SemiImplicitEulerIntegrator::step()` for `UnitQuaternionConstraint`

**Path 2 — Multi-body contact solve (`solveWithContacts()`)**:
- 12-DOF velocity-level formulation (v_A(3), ω_A(3), v_B(3), ω_B(3))
- ASM/ECOS for λ ≥ 0 / box-constrained
- Helpers: `assembleContactJacobians()`, `assembleContactEffectiveMass()`, `assembleContactRHS()`, `extractContactBodyForces()`
- Returns `MultiBodySolveResult` (per-body forces + iterations)
- **Production caller**: `CollisionPipeline::solveConstraintsWithWarmStart()`

### Specific Problems

1. **Redundant constraint solving**: The integrator already normalizes the quaternion via `state.orientation.normalize()` (line 78 of `SemiImplicitEulerIntegrator.cpp`), making `UnitQuaternionConstraint`'s Lagrange multiplier forces redundant. The single-body solver path exists solely to service this redundant computation.

2. **Misleading "Contact" prefix**: The multi-body helpers (`assembleContactJacobians`, etc.) now take `Constraint*` (not the deleted `TwoBodyConstraint*`), yet still carry the "Contact" prefix from their original narrow purpose.

3. **Duplicate result types**: `SolveResult` and `MultiBodySolveResult` serve the same conceptual role (constraint solve output) with different structures.

4. **Unnecessary integrator complexity**: `SemiImplicitEulerIntegrator` owns a `ConstraintSolver` instance and accepts a `constraints` parameter solely for the single-body path that produces negligible effect.

---

## Proposed Solution: Eliminate Single-Body Path, Rename Multi-Body Methods

### 1. Remove constraint solving from integrator

**Files**: `Integrator.hpp`, `SemiImplicitEulerIntegrator.hpp`, `SemiImplicitEulerIntegrator.cpp`

- Remove `const std::vector<Constraint*>& constraints` parameter from `Integrator::step()` and `SemiImplicitEulerIntegrator::step()`
- Remove `ConstraintSolver solver_` member from `SemiImplicitEulerIntegrator`
- Remove `SemiImplicitEulerIntegrator(ConstraintSolver)` constructor
- Remove the `ConstraintSolver::solve()` call and constraint force computation (lines 33-53 of `.cpp`)
- Keep quaternion normalization at line 78 (already handles Q drift)

### 2. Remove UnitQuaternionConstraint from AssetInertial defaults

**File**: `AssetInertial.cpp`

- Remove the two `constraints_.push_back(std::make_unique<UnitQuaternionConstraint>(...))` calls (lines 45-46 and 91-92)
- Remove `#include UnitQuaternionConstraint.hpp` from `AssetInertial.cpp`
- Keep `UnitQuaternionConstraint` class itself (tests exercise it as a standalone constraint)

### 3. Update WorldModel to not pass constraints to integrator

**File**: `WorldModel.cpp`

- Remove "Step 2: Gather Constraints" block (line 169)
- Remove `constraints` argument from the `integrator_->step()` call (line 174)

### 4. Remove single-body solver path from ConstraintSolver

**Files**: `ConstraintSolver.hpp`, `ConstraintSolver.cpp`

Remove (only used by the deleted `solve()` path):
- `solve()` static method (hpp:110-116, cpp:37-103)
- `SolveResult` struct (hpp:63-86)
- `assembleConstraintMatrix()` (hpp:335-340, cpp:105-153)
- `assembleRHS()` — the single-body version (hpp:350-358, cpp:155-242)
- `extractConstraintForces()` (hpp:368-372, cpp:244-282)
- `kNumStates` constant (hpp:495)

### 5. Rename multi-body methods and result type

**Files**: `ConstraintSolver.hpp`, `ConstraintSolver.cpp`

| Old Name | New Name |
|----------|----------|
| `solveWithContacts()` | `solve()` |
| `MultiBodySolveResult` | `SolveResult` |
| `assembleContactJacobians()` | `assembleJacobians()` |
| `assembleContactEffectiveMass()` | `assembleEffectiveMass()` |
| `assembleContactRHS()` | `assembleRHS()` |
| `extractContactBodyForces()` | `extractBodyForces()` |

### 6. Update callers of renamed methods/types

**Files** referencing `solveWithContacts` or `MultiBodySolveResult`:
- `CollisionPipeline.hpp`, `CollisionPipeline.cpp`
- `ConstraintSolverContactTest.cpp` (28 tests)
- `ConstraintSolverASMTest.cpp` (12 tests)
- `JacobianLinearTest.cpp` (6 tests)
- `SplitImpulseTest.cpp` (2 tests)

### 7. Update/delete affected tests

**File**: `ConstraintTest.cpp`

- **Delete** 4 `ConstraintSolverTest` tests that call the old `solve()`:
  - `EmptyConstraintSet_ReturnsZeroForces`
  - `SingleQuaternionConstraint_Converges`
  - `MultipleConstraints_Converges`
  - `ConditionNumber_WellConditioned`
- **Update** `DefaultConstraint_IsUnitQuaternion` test to expect empty constraint list
- **Update** any test asserting `constraints[0]->typeName() == "UnitQuaternionConstraint"`

---

## What Stays Unchanged

- `UnitQuaternionConstraint` class — kept for standalone unit testing
- `DistanceConstraint` class — kept for standalone unit testing (test-only, never goes through solver in production)
- `PositionCorrector` — already uses `Constraint*`, no changes needed
- `ContactConstraint`, `FrictionConstraint` — no interface changes
- `ActiveSetResult`, `solveActiveSet()`, `solveWithECOS()`, `buildFrictionConeSpec()` — internal solver methods, unchanged
- Quaternion normalization in integrator (line 78) — kept as-is, becomes the sole Q normalization mechanism

---

## Files Modified (Summary)

| File | Change |
|------|--------|
| `src/Physics/Integration/Integrator.hpp` | Remove constraints param from `step()` |
| `src/Physics/Integration/SemiImplicitEulerIntegrator.hpp` | Remove solver member, custom constructor |
| `src/Physics/Integration/SemiImplicitEulerIntegrator.cpp` | Remove constraint solving, simplify to pure integration |
| `src/Physics/RigidBody/AssetInertial.cpp` | Remove default UnitQuaternionConstraint |
| `src/Environment/WorldModel.cpp` | Remove constraint gathering for integrator |
| `src/Physics/Constraints/ConstraintSolver.hpp` | Remove single-body path, rename multi-body |
| `src/Physics/Constraints/ConstraintSolver.cpp` | Remove single-body path, rename multi-body |
| `src/Physics/Collision/CollisionPipeline.hpp` | Rename types |
| `src/Physics/Collision/CollisionPipeline.cpp` | Rename types/methods |
| `test/Physics/Constraints/ConstraintTest.cpp` | Delete 4 solver tests, update default constraint tests |
| `test/Physics/Constraints/ConstraintSolverContactTest.cpp` | Rename `solveWithContacts` → `solve`, `MultiBodySolveResult` → `SolveResult` |
| `test/Physics/Constraints/ConstraintSolverASMTest.cpp` | Rename types/methods |
| `test/Physics/Constraints/JacobianLinearTest.cpp` | Rename types/methods |
| `test/Physics/Constraints/SplitImpulseTest.cpp` | Rename types/methods |

---

## Acceptance Criteria

1. `ConstraintSolver` has a single public solve method (`solve()`) — no `solveWithContacts()` overload
2. No `SolveResult` / `MultiBodySolveResult` naming duplication — single result type named `SolveResult`
3. Helper methods use generic names without "Contact" prefix
4. `SemiImplicitEulerIntegrator::step()` has no `constraints` parameter and no `ConstraintSolver` dependency
5. `AssetInertial` defaults to an empty constraint list (no `UnitQuaternionConstraint`)
6. Quaternion normalization still works via direct `normalize()` in integrator
7. All existing tests pass (minus deleted single-body solver tests)
8. Zero behavioral change to collision response (pure rename + dead code removal)

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-08 (ticket creation)
- **Completed**: 2026-02-08
- **Branch**: N/A
- **PR**: N/A
- **Artifacts**: N/A
- **Notes**: Ticket created with full problem statement, proposed solution, and acceptance criteria. Ticket does not require math design — advancing directly to architectural design.

### Design Phase
- **Started**: 2026-02-08
- **Completed**: 2026-02-08
- **Branch**: 0045-constraint-solver-unification
- **PR**: #16
- **Artifacts**:
  - `docs/designs/0045_constraint_solver_unification/design.md`
  - `docs/designs/0045_constraint_solver_unification/0045_constraint_solver_unification.puml`
- **Notes**: Created architectural design for eliminating redundant single-body solver path and unifying ConstraintSolver API. Key decisions: (1) Delete single-body solve() method and helpers (~250 lines), (2) Rename multi-body methods to drop "Contact" prefix, (3) Remove constraint solving from integrator since quaternion normalization via normalize() already handles drift. PlantUML diagram shows modified components with strikethrough for deleted methods. Design includes phased implementation strategy to minimize intermediate build failures. Zero behavioral change expected for collision response — pure refactoring.

### Design Review Phase
- **Started**: 2026-02-08
- **Completed**: 2026-02-08
- **Branch**: 0045-constraint-solver-unification
- **PR**: #16
- **Artifacts**: Design review appended to `docs/designs/0045_constraint_solver_unification/design.md`
- **Notes**: Design review APPROVED on first pass (iteration 0 of 1, no revision needed). All criteria passed: architectural fit, C++ quality, feasibility, and testability. Key findings: (1) Exemplary technical debt reduction removing ~250 lines of provably redundant code, (2) Improves dependency structure by eliminating integrator ↔ solver coupling, (3) Zero behavioral risk since removed UnitQuaternionConstraint forces were mathematically redundant with state.orientation.normalize(). No prototype required — proceed directly to implementation using phased strategy. Review posted to PR #16.

### Prototype Phase
- **Started**: 2026-02-08
- **Completed**: 2026-02-08 (skipped)
- **Branch**: 0045-constraint-solver-unification
- **PR**: #16
- **Artifacts**: N/A
- **Notes**: Prototype phase skipped per design review recommendation. Design represents straightforward refactoring (dead code removal + renames) with zero algorithmic changes. Behavioral equivalence is provable: removed constraint forces were immediately overwritten by quaternion normalization. Proceeding directly to implementation.

---

## Risk Assessment

- **Low risk**: This is dead code removal + renames. The single-body solver path produces forces that are immediately superseded by quaternion normalization.
- **Behavioral change**: Removing UnitQuaternionConstraint's correction forces from the integrator eliminates a small angular velocity correction each frame. Since `normalize()` already handles the position-level drift, this should produce no observable change at typical timesteps (16.67ms). May cause sub-epsilon numerical differences in long-running integration tests.
- **Test impact**: 4 tests deleted (exercised the removed single-body `solve()` path), ~50 tests updated (renames only).
