# Ticket 0075a: Phase 1 — Data Structure Unification

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Implementation
- [x] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Implementation Complete
**Type**: Implementation
**Priority**: High
**Created**: 2026-02-22
**Generate Tutorial**: No
**Parent Ticket**: [0075](0075_unified_contact_constraint.md)
**Related Tickets**: [0032](0032_contact_constraint_refactor.md) (contact constraint), [0035a](0035a_tangent_basis_and_friction_constraint.md) (friction constraint)

---

## Summary

Merge `FrictionConstraint` data fields into `ContactConstraint` and update the transfer record layer. This is the data structure foundation that Phase 2 (Block PGS solver, ticket 0075b) builds upon.

---

## Scope

### In Scope
1. Create `UnifiedContactConstraintRecord` in `msd-transfer`
2. Update `ConstraintRecordVisitor` to use new record type
3. Extend `ContactConstraint` with friction fields (constructor, dimension, jacobian, accessors)
4. Update `ContactConstraintFactory` to create unified constraints with friction coefficient
5. Update `CollisionPipeline::createConstraints()` to use unified factory (remove FC creation)
6. Update `ContactCache` to use `Vec3` impulse storage
7. Update all includes and references
8. Update replay server generated models and pybind11 bindings for new record type

### Out of Scope
- Block PGS solver implementation (ticket 0075b)
- NLopt removal (ticket 0075c)
- New test assets (30-degree ramp — deferred to 0075b)

---

## Validation Gate

Build without errors. Existing tests pass. Friction behavior may regress temporarily during this phase as NLopt no longer receives correct input format — this is accepted and will be resolved in 0075b.

---

## Design Reference

Full design: [`docs/designs/0075_unified_contact_constraint/design.md`](../docs/designs/0075_unified_contact_constraint/design.md), Phase 1 section.

---

## Workflow Log

### Draft
- **Date**: 2026-02-22
- **Notes**: Split from parent ticket 0075 per PR #94 review feedback.

### Implementation Phase
- **Started**: 2026-02-22
- **Completed**: 2026-02-23
- **Branch**: `0075-unified-contact-constraint`
- **PR**: Pending
- **Artifacts**:
  - `msd/msd-transfer/src/UnifiedContactConstraintRecord.hpp` (new)
  - `msd/msd-transfer/src/ConstraintRecordVisitor.hpp`
  - `msd/msd-transfer/src/Records.hpp`
  - `msd/msd-sim/src/Physics/Constraints/ContactConstraint.hpp/.cpp`
  - `msd/msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp/.cpp`
  - `msd/msd-sim/src/Physics/Constraints/ContactCache.hpp/.cpp`
  - `msd/msd-sim/src/Physics/Constraints/ConstraintPool.hpp/.cpp`
  - `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.cpp`
  - `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp/.cpp`
  - `msd/msd-sim/src/DataRecorder/DataRecorder.cpp`
  - `msd/msd-sim/src/DataRecorder/DataRecorderVisitor.hpp/.cpp`
  - `msd/msd-pybind/src/record_bindings.cpp`
  - `replay/replay/generated_models.py`
  - `msd/msd-sim/test/Physics/Constraints/ContactCacheTest.cpp`
  - `docs/designs/0075_unified_contact_constraint/iteration-log.md`
- **Notes**: Build successful. 729/733 tests pass. 4 friction-behavior tests regress as expected per Phase 1 design spec (see iteration-log.md). Validation gate condition "friction behavior may regress in Phase 1" is satisfied. Phase 2 (Block PGS, ticket 0075b) will resolve the regressions.
