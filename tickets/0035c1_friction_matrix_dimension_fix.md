# Ticket 0035c1: Friction Constraint Matrix Dimension Fix

## Status
- [x] Draft
- [x] Investigation
- [x] Fix Implemented
- [x] Tests Passing
- [x] Complete

**Current Phase**: Complete
**Assignee**: N/A
**Created**: 2026-02-01
**Type**: Bug Fix (Regression from 0035c)
**Parent Ticket**: [0035c_friction_pipeline_integration](0035c_friction_pipeline_integration.md)
**Generate Tutorial**: No

---

## Summary

Ticket 0035c introduced 20 test regressions in collision-related tests. All fail with the same error:

```
ECOSProblemBuilder::build: A matrix dimension N does not match 3*numContacts = M
```

The root cause is that `ConstraintSolver::assembleContactEffectiveMass()` and `assembleContactRHS()` assume every constraint object contributes exactly 1 row (dimension 1), but `FrictionConstraint` has dimension 2 (two tangent directions). When friction constraints are merged into the constraint list, the assembled A matrix has the wrong dimensions for the ECOS solver.

---

## Root Cause Analysis

### Chain of Events

1. Ticket 0035c added `frictionCoefficient_` with **default μ=0.5** to `AssetInertial` and `AssetEnvironment`
2. Every collision now creates `FrictionConstraint` objects (since `√(0.5 × 0.5) = 0.5 > 0`)
3. Normal + friction constraints are merged into a flat `vector<TwoBodyConstraint*>` and passed to `solveWithContacts()`
4. `assembleContactEffectiveMass()` builds A as **C×C** where C = number of constraint *objects* (e.g., 8 for 4 contacts: 4 normal + 4 friction)
5. `ECOSProblemBuilder::build()` validates A as **3C'×3C'** where C' = number of *contact points* (e.g., 12 for 4 contacts: 3 rows per contact)
6. Dimension mismatch: 8 ≠ 12 → exception thrown

### Specific Code Locations

| File | Line(s) | Issue |
|------|---------|-------|
| `ConstraintSolver.cpp` | 385 | `A = MatrixXd::Zero(C, C)` — uses constraint count, not total row count |
| `ConstraintSolver.cpp` | 392-393 | `J_i.block<1, 6>(...)` — hardcodes 1×6 but FrictionConstraint Jacobian is 2×6 |
| `ConstraintSolver.cpp` | 400-401 | Same 1×6 hardcoding for inner loop |
| `ECOSProblemBuilder.cpp` | 32-37 | Validates `A.rows() == 3*numContacts`, which is correct for the ECOS formulation |

### Why Only ECOS Path Is Affected

The non-friction Active Set Method path (line 325) still works because:
- When no `FrictionConstraint` is present, `hasFriction = false`
- The ASM solver receives only `ContactConstraint` objects (all dim=1)
- A is correctly C×C where C = number of contacts

The bug only manifests when friction constraints are present, which is now **every collision** due to the default μ=0.5.

---

## Failing Tests (20 regressions)

### EngineIntegrationTest (7 failures)
**File**: `msd/msd-sim/test/EngineIntegrationTest.cpp`

| Test Name | Line |
|-----------|------|
| `Engine_OverlappingObjects_VelocitiesChange` | 273 |
| `ObjectRestsOnfloor` | 354 |
| `Engine_MomentumConserved_ElasticCollision` | 393 |
| `Engine_InelasticCollision_EnergyLost` | 437 |
| `Engine_PositionCorrection_ObjectsSeparated` | 485 |
| `Engine_InertialBouncesOffStaticEnvironment` | 527 |
| `Engine_StaticEnvironmentUnaffectedByCollision` | 567 |

### WorldModelCollisionTest (4 failures)
**File**: `msd/msd-sim/test/Environment/WorldModelCollisionTest.cpp`

| Test Name | Line |
|-----------|------|
| `updateCollisions_OverlappingObjects_ImpulseApplied` | 94 |
| `updateCollisions_PositionCorrection_ObjectsSeparated` | 139 |
| `updateCollisions_InelasticCollision_VelocityReduced` | 179 |
| `updateCollisions_ElasticCollision_MomentumConserved` | 223 |

### WorldModelStaticCollisionTest (3 failures)
**File**: `msd/msd-sim/test/Environment/WorldModelCollisionTest.cpp`

| Test Name | Line |
|-----------|------|
| `inertialVsEnvironment_ImpulseApplied` | 291 |
| `inertialVsEnvironment_PositionCorrected` | 327 |
| `inertialVsEnvironment_StaticUnchanged` | 358 |

### WorldModelContactIntegrationTest (6 failures)
**File**: `msd/msd-sim/test/Environment/WorldModelContactIntegrationTest.cpp`

| Test Name | Line |
|-----------|------|
| `HeadOnElasticCollision_SwapsVelocities` | 49 |
| `Collision_ConservesMomentum` | 104 |
| `RestingContact_StableFor1000Frames` | 160 |
| `GlancingCollision_ProducesAngularVelocity` | 214 |
| `DynamicStaticCollision_StaticUnmoved` | 268 |
| `MultipleSimultaneousContacts_ResolvedCorrectly` | 313 |

### NOT a regression (1 pre-existing failure)
| `GeometryDatabaseTest.VisualGeometry_CreateAndStore_Cube` | Vertex struct size mismatch — unrelated |

---

## Required Fix

The constraint solver's matrix assembly functions need to be generalized to handle multi-dimensional constraints:

### 1. `assembleContactJacobians()` (ConstraintSolver.cpp)
- Currently returns `vector<MatrixXd>` where each entry is assumed 1×12
- Must use `constraint->dimension()` to determine row count per constraint
- Each Jacobian entry should be `dim×12` (1×12 for normal, 2×12 for friction)

### 2. `assembleContactEffectiveMass()` (ConstraintSolver.cpp:364-427)
- Currently builds A as C×C with scalar `a_ij` entries
- Must compute total row count: `totalRows = Σ constraint[i].dimension()`
- A becomes `totalRows × totalRows`
- Each block `A[i,j]` becomes `dim_i × dim_j` (not 1×1)
- Jacobian extraction must use `dim_i × 6` blocks (not 1×6)

### 3. `assembleContactRHS()` (ConstraintSolver.cpp)
- Must similarly account for multi-dimensional constraints
- RHS vector length = totalRows (not C)

### 4. `solveWithContacts()` ECOS dispatch (ConstraintSolver.cpp:296-320)
- A and b already have correct total dimensions after fix above
- `numContacts` counting (line 307-314) is correct (counts only ContactConstraint)
- ECOS expects 3C×3C where C = contact points — this matches totalRows = C×1 + C×2 = 3C

---

## Acceptance Criteria

- [x] **AC1**: All 20 regression tests pass
- [x] **AC2**: All existing friction tests (FrictionConeSpec, FrictionConstraint, ECOSFrictionValidationTest) still pass
- [x] **AC3**: No new warnings with `-Werror`
- [x] **AC4**: Matrix dimensions: A is `totalRows × totalRows` where `totalRows = Σ dim_i` across all constraints

---

## Files

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Generalize `assembleContactEffectiveMass()`, `assembleContactRHS()`, `assembleContactJacobians()` to handle multi-dimensional constraints |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Update signatures if needed |

---

## References

- **Parent ticket**: [0035c_friction_pipeline_integration](0035c_friction_pipeline_integration.md)
- **Bug location**: `ConstraintSolver.cpp:364-427` (`assembleContactEffectiveMass`)
- **Validation**: `ECOSProblemBuilder.cpp:32-37` (dimension check that catches the mismatch)
- **FrictionConstraint dimension**: `FrictionConstraint::dimension()` returns 2

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-01
- **Notes**: Follow-on bug fix ticket for 20 test regressions introduced by 0035c. Root cause identified: constraint solver matrix assembly hardcodes dimension=1 per constraint but FrictionConstraint has dimension=2. Fix requires generalizing assembly to use `constraint->dimension()`.

### Investigation Phase
- **Started**: 2026-02-01 14:19
- **Completed**: 2026-02-01 14:25
- **Findings**: Root cause confirmed via code inspection. Three functions in ConstraintSolver.cpp hardcode dimension=1: `assembleContactEffectiveMass()`, `assembleContactRHS()`, and `extractContactBodyForces()`. `assembleContactJacobians()` is already correct (uses jacobianTwoBody() which returns appropriate dimension).
- **Debug session**: `.debug-sessions/debug_20260201_141918.md`

### Fix Implementation Phase
- **Started**: 2026-02-01 14:25
- **Completed**: 2026-02-01 14:30
- **Artifacts Modified**:
  - `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`
    - `assembleContactEffectiveMass()`: Generalized to compute totalRows and use variable-size Jacobian blocks
    - `assembleContactRHS()`: Generalized to build b as VectorXd(totalRows) with multi-row entries
    - `extractContactBodyForces()`: Generalized to use constraint->dimension() for lambda extraction
- **Build**: Clean build completed with no warnings
- **Notes**: No header changes required. All functions now correctly use `constraint->dimension()` to handle both ContactConstraint (dim=1) and FrictionConstraint (dim=2).

### Testing Phase
- **Started**: 2026-02-01 14:30
- **Completed**: 2026-02-01 14:32
- **Results**: 590/591 tests pass (99.8%)
  - All 20 regression tests now PASS
  - All 41 friction tests still PASS
  - 1 pre-existing failure (GeometryDatabaseTest - unrelated)
- **Verification**: Matrix dimension fix confirmed - A is now totalRows×totalRows where totalRows = Σ dim_i

### Completion
- **Date**: 2026-02-01 14:32
- **Status**: All acceptance criteria met
- **Next Steps**: Ready for commit (branch: 0035b5-validation_tests)
