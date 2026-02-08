# Ticket 0043: Constraint Hierarchy Refactor

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
**Predecessor**: [0031_generalized_lagrange_constraints](0031_generalized_lagrange_constraints.md), [0032_contact_constraint_refactor](0032_contact_constraint_refactor.md), [0035a_tangent_basis_and_friction_constraint](0035a_tangent_basis_and_friction_constraint.md)
**Related Tickets**: [0034_active_set_method_contact_solver](0034_active_set_method_contact_solver.md), [0036_collision_pipeline_extraction](0036_collision_pipeline_extraction.md), [0042_collision_numerical_stability](0042_collision_numerical_stability.md)
**Type**: Refactoring

---

## Problem Statement

The constraint type hierarchy has grown organically through tickets 0031, 0032, and 0035a into a 4-level deep inheritance chain that exhibits several design smells:

### Current Hierarchy

```
Constraint (abstract: dimension, evaluate, jacobian, alpha, beta, typeName)
├── BilateralConstraint (empty semantic marker — zero new methods/data)
│   ├── UnitQuaternionConstraint (single-body, equality)
│   └── DistanceConstraint (single-body, equality)
└── UnilateralConstraint (adds only isActive() — inequality C(q,t) ≥ 0)
    └── TwoBodyConstraint (adds evaluateTwoBody, jacobianTwoBody, isActiveTwoBody, body indices)
        │                  (throws on inherited evaluate/jacobian/isActive)
        ├── ContactConstraint (two-body, inequality)
        └── FrictionConstraint (two-body, box-constrained)
```

### Specific Problems

1. **Excessive inheritance depth**: ContactConstraint/FrictionConstraint require 4 levels of inheritance (ContactConstraint → TwoBodyConstraint → UnilateralConstraint → Constraint)

2. **Empty marker class**: `BilateralConstraint` contributes zero methods and zero data members — it exists purely as a "semantic marker" for future solver optimizations that were never implemented

3. **Trivial intermediate class**: `UnilateralConstraint` adds only `isActive()` — a single pure virtual method does not justify an entire inheritance layer

4. **Liskov Substitution Principle violation**: `TwoBodyConstraint` overrides `evaluate()`, `jacobian()`, and `isActive()` from the base to throw `std::logic_error("Single-body methods not supported")`. Callers cannot treat a `TwoBodyConstraint*` as a `Constraint*` without risking exceptions

5. **Two orthogonal concerns conflated**: The hierarchy conflates two independent axes:
   - **Multiplier bounds**: bilateral (λ unrestricted) vs unilateral (λ ≥ 0) vs box-constrained (-bound ≤ λ ≤ bound)
   - **Body count**: single-body (1 state) vs two-body (2 states + body indices)

   These should be independently composable, not forced into a single inheritance chain

6. **Heavy dynamic_cast usage**: The `ConstraintSolver` uses 6+ `dynamic_cast` calls to recover type information that the hierarchy is supposed to provide:
   - `dynamic_cast<TwoBodyConstraint*>` for two-body dispatch
   - `dynamic_cast<ContactConstraint*>` for contact-specific data (normal, penetration, restitution)
   - `dynamic_cast<FrictionConstraint*>` for friction-specific data (bounds, tangent basis)

---

## Proposed Solution: Flatten to Single-Level Hierarchy

### Design Goals

1. **Maximum 2 levels of inheritance**: Base `Constraint` + concrete implementations
2. **Eliminate empty/trivial intermediate classes**: Remove BilateralConstraint, UnilateralConstraint, TwoBodyConstraint
3. **No LSP violations**: Every concrete constraint is a proper substitutable Constraint
4. **Minimize dynamic_cast usage**: The solver should be able to operate through the base interface as much as possible
5. **Encode multiplier bounds in the interface**: Bilateral/unilateral/box-constrained distinction accessible without type-casting

### Key Design Decisions to Make

1. **Unified evaluation signature**: Should `evaluate()`/`jacobian()` always take two body states (single-body ignores the second), or should we use a different mechanism (e.g., a span of states)?

2. **Multiplier bounds representation**: An enum (`Bilateral`, `Unilateral`, `BoxConstrained`), or a virtual `getLambdaBounds() → (lower, upper)` returning (-∞,+∞), (0,+∞), or (-μλn, +μλn)?

3. **Body index storage**: Move `bodyAIndex`/`bodyBIndex` into the base class, or make them part of the evaluation interface?

4. **Contact/friction-specific accessors**: Keep these on the concrete classes (requiring targeted casts in specific solver paths), or abstract them into the base?

### Concrete Types in New Hierarchy

```
Constraint (base: unified interface)
├── UnitQuaternionConstraint (single-body, bilateral)
├── DistanceConstraint (single-body, bilateral)
├── ContactConstraint (two-body, unilateral)
└── FrictionConstraint (two-body, box-constrained)
```

---

## Scope

### In Scope
- Flatten the constraint inheritance hierarchy
- Update ConstraintSolver to use the new interface (reduce dynamic_cast usage)
- Update PositionCorrector to use the new interface
- Update WorldModel constraint handling
- Update ContactConstraintFactory
- Update all constraint tests
- Update ContactCache if affected

### Out of Scope
- Adding new constraint types (joints, limits, motors)
- Changing the PGS solver algorithm itself
- Modifying the collision detection pipeline
- Performance optimization of the constraint solver

---

## Files Affected

### Primary (constraint definitions)

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Constraints/Constraint.hpp` | Redesign base interface |
| `msd-sim/src/Physics/Constraints/BilateralConstraint.hpp` | Remove |
| `msd-sim/src/Physics/Constraints/UnilateralConstraint.hpp` | Remove |
| `msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp` | Remove |
| `msd-sim/src/Physics/Constraints/TwoBodyConstraint.cpp` | Remove |
| `msd-sim/src/Physics/Constraints/ContactConstraint.hpp` | Update base class |
| `msd-sim/src/Physics/Constraints/ContactConstraint.cpp` | Update implementation |
| `msd-sim/src/Physics/Constraints/FrictionConstraint.hpp` | Update base class |
| `msd-sim/src/Physics/Constraints/FrictionConstraint.cpp` | Update implementation |
| `msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.hpp` | Update base class |
| `msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.cpp` | Update implementation |
| `msd-sim/src/Physics/Constraints/DistanceConstraint.hpp` | Update base class |
| `msd-sim/src/Physics/Constraints/DistanceConstraint.cpp` | Update implementation |

### Secondary (consumers)

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Update dispatch logic |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Remove dynamic_cast dispatch, use new interface |
| `msd-sim/src/Physics/Constraints/PositionCorrector.cpp` | Update constraint access |
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp` | Update return types |
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` | Update factory methods |
| `msd-sim/src/Environment/WorldModel.cpp` | Update constraint handling |
| `msd-sim/src/Physics/RigidBody/AssetInertial.hpp` | Verify unique_ptr<Constraint> still works |

### Tests

| File | Change |
|------|--------|
| `msd-sim/test/Physics/Constraints/ConstraintSolverContactTest.cpp` | Update for new API |
| `msd-sim/test/Physics/Constraints/ContactConstraintFactoryTest.cpp` | Update for new API |
| `msd-sim/test/Physics/Constraints/JacobianLinearTest.cpp` | Update for new API |
| `msd-sim/test/Physics/Constraints/SplitImpulseTest.cpp` | Update for new API |
| All constraint unit tests | Update include paths, base class references |

### Documentation

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Constraints/CLAUDE.md` | Update hierarchy documentation |
| `msd-sim/CLAUDE.md` | Update constraint system documentation |
| PlantUML diagrams | Update class diagrams |

---

## Acceptance Criteria

1. [ ] **AC1**: All concrete constraints inherit directly from `Constraint` (max 2 levels)
2. [ ] **AC2**: `BilateralConstraint`, `UnilateralConstraint`, and `TwoBodyConstraint` classes are removed
3. [ ] **AC3**: No Liskov Substitution Principle violations — all Constraint* operations are valid on any concrete constraint
4. [ ] **AC4**: `dynamic_cast` usage in ConstraintSolver reduced from 6+ to ≤ 2 (targeted contact/friction-specific accessor casts are acceptable)
5. [ ] **AC5**: All existing tests pass with no regressions (678 tests baseline)
6. [ ] **AC6**: No behavioral change — the physics simulation produces identical results before and after refactoring
7. [ ] **AC7**: Updated PlantUML diagrams and CLAUDE.md documentation reflect the new hierarchy

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Behavioral regression from interface change | Medium | High | Run full test suite after each incremental change; compare simulation output |
| Breaking AssetInertial's unique_ptr<Constraint> ownership | Low | Medium | unique_ptr<Constraint> is unaffected by flattening — still polymorphic |
| Missing a consumer of the removed intermediate classes | Low | Medium | Grep for all #include and usage of removed classes before deletion |
| Performance regression from virtual dispatch changes | Low | Low | Profile before/after; vtable layout changes are typically negligible |

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-08
- **Notes**: Ticket created to address architectural debt in the constraint hierarchy. The current 4-level inheritance chain with LSP violations, empty marker classes, and heavy dynamic_cast usage evolved organically through tickets 0031, 0032, and 0035a. Goal is to flatten to a clean single-level hierarchy where all concrete constraints inherit directly from a unified Constraint base.

### Status Advancement
- **Timestamp**: 2026-02-08
- **From**: Draft
- **To**: Ready for Design
- **Notes**: Refactoring ticket does not require Math Design phase. Advanced directly to architectural design phase. Next step: Execute cpp-architect agent to design the flattened constraint hierarchy.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
