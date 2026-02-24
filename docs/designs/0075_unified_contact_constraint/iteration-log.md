# Iteration Log: 0075a Unified Constraint Data Structure (Phase 1)

## Iteration 1 — 2026-02-22/23

### Objective
Implement Phase 1 of ticket 0075 (data structure unification): merge FrictionConstraint data fields into ContactConstraint, create UnifiedContactConstraintRecord, and update all referencing code.

### Approach
Per the design in `docs/designs/0075_unified_contact_constraint/design.md`, Phase 1 section:

1. Created `msd/msd-transfer/src/UnifiedContactConstraintRecord.hpp` — new transfer record superseding ContactConstraintRecord + FrictionConstraintRecord. Includes all contact geometry + friction direction + solved lambda fields.

2. Updated `msd/msd-transfer/src/Records.hpp` — added include for UnifiedContactConstraintRecord.

3. Updated `msd/msd-transfer/src/ConstraintRecordVisitor.hpp` — replaced two visit overloads (ContactConstraintRecord, FrictionConstraintRecord) with single `visit(UnifiedContactConstraintRecord&)`.

4. Extended `msd/msd-sim/src/Physics/Constraints/ContactConstraint.hpp/.cpp`:
   - Added `frictionCoefficient` parameter (default 0.0 for backward compat)
   - `dimension()` returns 1 (frictionless) or 3 (friction active)
   - `jacobian()` returns 1x12 or 3x12
   - Added friction accessors: `hasFriction()`, `getFrictionCoefficient()`, `getTangent1()`, `getTangent2()`, `isSlidingMode()`
   - Added mutation methods: `setSlidingMode()`, `setTangentLambdas()`, `setNormalLambda()`
   - `recordState()` now dispatches `UnifiedContactConstraintRecord`

5. Updated `ContactConstraintFactory` — added `frictionCoefficient` parameter.

6. Updated `CollisionPipeline.cpp` — removed all FrictionConstraint creation. Friction coefficient passed directly to `allocateContact()`. Stride simplified to 1. `extractContactPoints`, `propagateSolvedLambdas`, `buildContactView`, `findPairIndexForConstraint` all simplified.

7. Updated `ContactCache.hpp/.cpp` — replaced `std::vector<double> lambdas` with `std::vector<Eigen::Vector3d> impulses`, providing `getWarmStart3()` / `update3()` API. Each Vec3 encodes `{lambda_n, lambda_t1, lambda_t2}`. Updated CollisionPipeline warm-start and cache-update paths.

8. Updated `ConstraintPool.hpp/.cpp` — `allocateFriction()` deprecated with doc comment (not removed to maintain backward compat).

9. Updated `FrictionConstraint.cpp` — `recordState()` updated to build `UnifiedContactConstraintRecord` instead of `FrictionConstraintRecord` (deprecated but must compile).

10. Updated `DataRecorder.cpp` / `DataRecorderVisitor.hpp/.cpp` — replaced ContactConstraintRecord + FrictionConstraintRecord instantiations with UnifiedContactConstraintRecord.

11. Updated `msd/msd-pybind/src/record_bindings.cpp` — replaced old record bindings with `UnifiedContactConstraintRecord` binding.

12. Updated `replay/replay/generated_models.py` — added `UnifiedContactConstraint` Pydantic model.

13. Updated `ContactCacheTest.cpp` — all tests converted from old scalar `update()`/`getWarmStart()` API to new `update3()`/`getWarmStart3()` Vec3 API. Added new `FrictionImpulse_StoredAndRetrieved` test.

### Build Result
Compilation successful. 729/733 tests pass.

### Test Results
4 friction-behavior tests regress as expected per design spec:
- `FrictionDirectionTest.SlidingCube_FrictionOpposesTangentialVelocity`
- `FrictionConeSolverTest.SlidingCubeOnFloor_FrictionSaturatesAtConeLimit`
- `ReplayEnabledTest.LinearCollisionTest_A4_EqualMassElastic_VelocitySwap`
- `ReplayEnabledTest.RotationalEnergyTest_F4b_ZeroGravity_RotationalEnergyTransfer_Conserved`

These regressions are expected per the Phase 1 design (see `design.md` Phase 1 Known Limitations section). The unified `ContactConstraint.lambdaBounds()` returns a single `LambdaBounds` for all 3 rows; the solver cannot express mixed unilateral (normal) + box (tangent) bounds through the current interface. Full friction solving requires Phase 2 (Block PGS, ticket 0075b).

### Validation Gate
- Build: PASS
- Tests: 729/733 PASS (4 expected friction regressions)
- Gate condition: "friction behavior may regress in Phase 1" — SATISFIED
