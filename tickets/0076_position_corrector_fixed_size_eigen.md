# Ticket 0076: PositionCorrector Fixed-Size Eigen Matrices

## Status
- [x] Draft
- [ ] Investigation Complete
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Implementation
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Investigation / Performance
**Priority**: Medium
**Created**: 2026-02-21
**Generate Tutorial**: No
**Parent Ticket**: [0071_collision_pipeline_profiling](0071_collision_pipeline_profiling.md)
**Related Tickets**: [0071c_eigen_fixed_size_matrices](0071c_eigen_fixed_size_matrices.md) (same pattern applied to ConstraintSolver)

---

## Summary

`PositionCorrector` uses heap-allocated `Eigen::MatrixXd` / `Eigen::VectorXd` for its workspace members and per-constraint Jacobian storage. Since all ContactConstraint Jacobians are 1x12 (two-body, velocity-level), several of these can potentially be converted to fixed-size Eigen types, eliminating per-frame heap allocations — the same pattern successfully applied in ticket 0071c for `ConstraintSolver`.

This ticket covers **investigation** to determine which workspace members can be converted and which must remain dynamic due to runtime-sized dimensions.

---

## Problem

### Current Workspace Members (PositionCorrector.hpp:83-94)

| Member | Current Type | Size | Fixed-Size Candidate? |
|--------|-------------|------|----------------------|
| `bPos_` | `Eigen::VectorXd` | c (num contacts) | Likely NO — runtime-sized |
| `effectiveMass_` | `Eigen::MatrixXd` | c x c | Likely NO — runtime-sized |
| `lambdaPos_` | `Eigen::VectorXd` | c | Likely NO — runtime-sized |
| `jacobians_` | `std::vector<Eigen::MatrixXd>` | c elements, each 1x12 | **YES** — same pattern as 0071c |
| `asmAw_` | `Eigen::MatrixXd` | activeSize x activeSize | Likely NO — varies per ASM iteration |
| `asmBw_` | `Eigen::VectorXd` | activeSize | Likely NO — varies per ASM iteration |
| `asmW_` | `Eigen::VectorXd` | c | Likely NO — runtime-sized |

### Known Conversion

`jacobians_` stores per-constraint Jacobians returned by `constraint->jacobian()`. All ContactConstraint Jacobians are 1x12. This is the same pattern converted in 0071c for `ConstraintSolver::assembleJacobians()`. Converting `std::vector<Eigen::MatrixXd>` to `std::vector<Eigen::Matrix<double, 1, 12>>` eliminates a heap allocation per contact per frame.

### Investigation Questions

1. **`jacobians_`**: Confirm all callers pass only ContactConstraint (1x12). Any path that passes other constraint types would break a fixed-size assumption.
2. **ASM workspace (`asmAw_`, `asmBw_`)**: The active set size varies per iteration. Could a reasonable upper bound allow fixed-size with `Eigen::Matrix<double, MaxN, MaxN>`? What's the typical and maximum active set size in practice?
3. **System matrices (`bPos_`, `effectiveMass_`, `lambdaPos_`, `asmW_`)**: These are `c x c` or `c`-sized where `c` is the number of contacts. Is there a practical upper bound on contacts per island that would make fixed-size feasible, or are these truly unbounded?
4. **Block extraction**: Lines 124-125, 291-292 already use `block<1,6>()` on the dynamic `jacobians_[i]`. With a fixed-size source, these become compile-time optimizable.

---

## Test Coverage

### Existing Tests

`PositionCorrector` is tested in `msd/msd-sim/test/Physics/Constraints/SplitImpulseTest.cpp` under the `SplitImpulse` test suite (12 tests):

| Test | What it verifies |
|------|-----------------|
| `VelocityRHS_RestitutionOnly` | Velocity-level RHS assembly |
| `PositionCorrection_ReducesPenetration` | Core position correction reduces penetration depth |
| `PositionCorrection_DoesNotChangeVelocity` | Pseudo-velocities discarded (no KE injection) |
| `PositionCorrection_DoesNotChangeAngularVelocity` | Angular velocity unaffected by correction |
| `Slop_NoCorrectionBelowThreshold` | No correction when penetration < slop |
| `Slop_PartialCorrectionAboveThreshold` | Partial correction for penetration above slop |
| `EnergyConservation_RestingContact` | No energy injection from position correction |
| `EnvironmentBody_NotMoved` | Static (infinite mass) bodies remain fixed |
| `OrientationCorrection_TiltedCube` | Quaternion orientation correction path |
| `BetaZero_NoCorrection` | Beta=0 config disables correction |
| `SlopZero_FullCorrection` | Slop=0 config corrects full penetration |
| `MultiContact_IndependentCorrection` | Multiple simultaneous contacts solved together |

All 12 tests exercise `correctPositions()` end-to-end, including the Jacobian assembly (step 2), effective mass build (step 3), ASM solve (step 4), and position application (step 5). This provides solid regression coverage for the `jacobians_` fixed-size conversion.

### Test Hygiene: Rename `SplitImpulse` → `PositionCorrector`

The test suite is named `SplitImpulse` (after the algorithm) rather than `PositionCorrector` (the class under test). Project convention is to name test suites after the class they test. As part of this ticket, rename:
- Test suite prefix: `SplitImpulse` → `PositionCorrector`
- File: `SplitImpulseTest.cpp` → `PositionCorrectorTest.cpp`

---

## Success Criteria

- Investigation documents which members can be converted and which must remain dynamic
- If conversions are identified, implementation converts them with no test regressions
- `Constraint` virtual interface unchanged
- Test suite renamed from `SplitImpulse` to `PositionCorrector` (file and test prefix)

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-21
- **Notes**: Created from observation that PositionCorrector workspace uses dynamic Eigen types where some dimensions are compile-time known. Follows the same pattern as 0071c (ConstraintSolver fixed-size Jacobian rows).

### Investigation Phase
- **Started**: 2026-02-21
- **Notes**:
  - Confirmed existing test coverage: 12 tests in `SplitImpulseTest.cpp` exercise all code paths in `correctPositions()`
  - Identified test hygiene item: rename `SplitImpulse` suite to `PositionCorrector` to match class-under-test convention
  - `jacobians_` confirmed as primary conversion candidate — all callers pass only ContactConstraint (1x12 Jacobians)
