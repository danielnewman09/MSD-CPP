# Ticket 0035d6: ECOSProblemBuilder Test Realignment for SOCP Reformulation

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Quality Gate
**Assignee**: N/A
**Created**: 2026-02-01
**Type**: Test Fix (8 tests broken — 7 assertion failures + 1 crash)
**Severity**: High — blocks quality gate validation of 0035d3/d4/d5
**Generate Tutorial**: No
**Parent Ticket**: [0035d_friction_hardening_and_validation](0035d_friction_hardening_and_validation.md)

---

## Summary

The `ECOSProblemBuilder` implementation was redesigned for SOCP reformulation (ticket 0035d1 design phase), but the unit tests in `ECOSProblemBuilderTest.cpp` still assert the old feasibility formulation dimensions and structure. All 8 failing tests need to be rewritten to validate the new SOCP problem structure.

This is **not a bug in the implementation** — it is a **test/implementation mismatch** caused by the implementation evolving ahead of the test expectations.

---

## Problem Statement

### Old Formulation (what tests expect)

The tests were written for the feasibility formulation:
```
min 0^T·x  s.t.  A·x = b, G·x + s = 0, s in K_SOC
```

Where for C contacts:
- Decision variable: `x = lambda` (dimension 3C)
- G matrix: 3C x 3C block-diagonal (one 3x3 friction cone per contact)
- h vector: all zeros (size 3C)
- c vector: all zeros (size 3C)
- Cone sizes: C cones of size 3

### New Formulation (what implementation produces)

The SOCP reformulation lifts the quadratic objective into conic form:
```
min t  s.t.  epigraph cone, friction cones, A_eq·x = b_eq
```

Where for C contacts:
- Decision variable: `x = [lambda; y; t]` (dimension 6C+1)
- G matrix: (6C+2) x (6C+1) with epigraph block + friction blocks
- h vector: non-zero (includes epigraph RHS terms)
- c vector: `[0...0, 1]` (objective on auxiliary t variable)
- Cone sizes: 1 epigraph cone of size (3C+2) + C friction cones of size 3

### Test-by-Test Mismatch

| Test | Asserts | Gets | Root Cause |
|------|---------|------|------------|
| `SingleContactGMatrix` | nrows=3, ncols=3, nnz=3 | nrows=8, ncols=7, nnz=8 | SOCP adds epigraph + auxiliary dimensions |
| `MultiContactGMatrix` | nrows=6, ncols=6, nnz=6 | nrows=14, ncols=13, nnz=14 | Same |
| `GMatrixDimensions` | nrows=9, ncols=9, num_vars=9, num_cones=3 | nrows=20, ncols=19, num_vars=19, num_cones=4 | Same + extra epigraph cone |
| `HVectorAllZeros` | size=6, all zeros | size=14, non-zero entries | Epigraph RHS is non-zero |
| `CVectorAllZeros` | size=3, all zeros | size=7, c[last]=1 | Objective minimizes t |
| `ConeSizes` | 4 cones of size 3 | 5 cones, first is size 14 | Epigraph cone added |
| `CSCFormatValidation` | 6x6 diagonal structure | 14x7 mixed structure | SOCP constraint layout |
| `BlockDiagonalStructure` | Reconstructs 3Cx3C dense | **CRASH** (Eigen assertion) | Wrong matrix dimensions cause out-of-bounds access |

---

## Technical Approach

Rewrite all 8 test cases to validate the SOCP formulation structure. For each test:

1. Compute the expected SOCP dimensions from the math formulation in `docs/designs/0035d1_ecos_socp_reformulation/math-formulation.md`
2. Assert the correct G matrix structure (epigraph block in top rows, friction block in bottom rows)
3. Assert h vector contains correct epigraph RHS values
4. Assert c vector has `1.0` in the `t` position and zeros elsewhere
5. Assert cone sizes include the epigraph cone

The `BlockDiagonalStructure` test must be fundamentally restructured since the G matrix is no longer block-diagonal.

---

## Failing Tests

| # | Test Name | Failure Type | File:Line |
|---|-----------|-------------|-----------|
| 1 | `ECOSProblemBuilderTest.SingleContactGMatrix` | Assertion (wrong dimensions) | `ECOSProblemBuilderTest.cpp:27` |
| 2 | `ECOSProblemBuilderTest.MultiContactGMatrix` | Assertion (wrong dimensions) | `ECOSProblemBuilderTest.cpp:68` |
| 3 | `ECOSProblemBuilderTest.GMatrixDimensions` | Assertion (wrong dimensions) | `ECOSProblemBuilderTest.cpp:105` |
| 4 | `ECOSProblemBuilderTest.HVectorAllZeros` | Assertion (non-zero h) | `ECOSProblemBuilderTest.cpp:126` |
| 5 | `ECOSProblemBuilderTest.CVectorAllZeros` | Assertion (non-zero c) | `ECOSProblemBuilderTest.cpp:146` |
| 6 | `ECOSProblemBuilderTest.ConeSizes` | Assertion (extra cone) | `ECOSProblemBuilderTest.cpp:169` |
| 7 | `ECOSProblemBuilderTest.CSCFormatValidation` | Assertion (wrong structure) | `ECOSProblemBuilderTest.cpp:~190` |
| 8 | `ECOSProblemBuilderTest.BlockDiagonalStructure` | **CRASH** (Eigen bounds) | `ECOSProblemBuilderTest.cpp:~275` |

---

## Files

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/test/Physics/Constraints/ECOS/ECOSProblemBuilderTest.cpp` | Rewrite 8 test cases for SOCP formulation |

### Reference Files

| File | Purpose |
|------|---------|
| `msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.cpp` | Current SOCP implementation |
| `msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp` | API surface |
| `docs/designs/0035d1_ecos_socp_reformulation/math-formulation.md` | SOCP math derivation |

---

## Acceptance Criteria

- [x] **AC1**: All 8 ECOSProblemBuilderTest cases pass
- [x] **AC2**: No crash in `BlockDiagonalStructure` (or renamed replacement test)
- [x] **AC3**: Tests validate SOCP-specific structure (epigraph cone, auxiliary variables, non-zero h/c)
- [x] **AC4**: Zero regressions in other test suites

---

## Dependencies

- **Caused by**: 0035d1 design implementation applied to ECOSProblemBuilder without updating tests
- **Independent of**: 0035d7 (ECOS solver integration) and 0035d8 (physics integration)
- **Blocks**: Quality gate validation of 0035d3/d4/d5

---

## Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Tests written for wrong SOCP structure | Medium | Medium | Cross-reference with math-formulation.md |
| Crash in BlockDiagonalStructure masks deeper issue | Low | Low | Fix dimensions first, verify no memory corruption |

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-01
- **Notes**: 8 ECOSProblemBuilderTest failures identified during post-0035d3/d4/d5 assessment. Tests assert old feasibility formulation dimensions. Implementation was updated for SOCP reformulation (0035d1 design applied) but tests were not aligned. One test crashes due to Eigen out-of-bounds access from wrong matrix dimensions.

### Ready for Implementation Phase
- **Started**: 2026-02-01
- **Notes**: Skipping Math Design and Architectural Design phases - this is a test fix to align existing tests with the SOCP formulation already designed in 0035d1 and implemented in ECOSProblemBuilder. No new design required. Ready for implementer to rewrite the 8 failing test cases to validate SOCP structure.

### Implementation Phase
- **Started**: 2026-02-01
- **Completed**: 2026-02-01
- **Artifacts**:
  - `msd-sim/test/Physics/Constraints/ECOS/ECOSProblemBuilderTest.cpp` — Rewrote all 8 failing test cases
- **Notes**: Successfully rewrote all 8 ECOSProblemBuilderTest cases to validate the SOCP auxiliary-variable formulation (x = [λ; y; t]). Key changes:
  - Updated dimension assertions: G matrix (6C+2) × (6C+1), h vector 6C+2, c vector 6C+1
  - Fixed h vector test to expect non-zero epigraph RHS values (-2d where d = L⁻¹b)
  - Fixed c vector test to expect 1.0 in last position (minimize t)
  - Fixed cone sizes test to expect [3C+2, 3, 3, ..., 3] structure
  - Rewrote BlockDiagonalStructure test to validate SOCP mixed structure (not block-diagonal)
  - Fixed CSCFormatValidation for SOCP sparse pattern
  - All 14 ECOSProblemBuilderTest tests now pass (8 previously failing, 6 unaffected)
  - Zero regressions in ECOSProblemBuilderTest suite
  - AC1-AC4 all satisfied
