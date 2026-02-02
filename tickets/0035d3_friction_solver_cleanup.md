# Ticket 0035d3: Friction Solver Cleanup (Ordering Bug + Debug Removal)

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
**Type**: Bug Fix + Cleanup
**Requires Math Design**: No
**Generate Tutorial**: No
**Parent Ticket**: [0035d_friction_hardening_and_validation](0035d_friction_hardening_and_validation.md)

---

## Summary

Three independent fixes in `ConstraintSolver.cpp` that are prerequisites for the centroid reduction work (0035d4/0035d5). No behavioral change for the current codebase since the ordering bug only manifests with specific constraint orderings that don't occur yet — but it would cause incorrect ECOS solutions once 0035d5 changes the constraint layout.

---

## Problem Statement

### Bug 1: `buildFrictionConeSpec()` assumes interleaved constraint ordering

**File**: `ConstraintSolver.cpp:744-773`

The function uses `normalIdx = 3 * contactIdx` to find the row index of the normal constraint for each friction cone. This assumes interleaved ordering `[λ_n0, λ_t1_0, λ_t2_0, λ_n1, ...]`, but the actual ordering from WorldModel is normals-first: `[CC0, CC1, ..., CCN, FC0, FC1, ..., FCN]`.

For the current code this happens to work when there is exactly 1 contact (1 normal + 1 friction = 3 rows, `normalIdx = 0` is correct). With the centroid reduction (0035d5), the ordering will be `[CentroidCC(row 0), CentroidFC(rows 1-2)]` which also works by coincidence. However, the assumption is wrong and should be fixed for correctness.

### Bug 2: `solveWithECOS()` hardcodes `n = 3 * numContacts`

**File**: `ConstraintSolver.cpp:781`

The variable count `n` is used to extract the lambda vector from the ECOS solution. It assumes each contact contributes exactly 3 rows (1 normal + 2 tangential). This is fragile and should be computed from the actual A matrix dimension.

### Cleanup: Debug output in `solveWithECOS()`

**File**: `ConstraintSolver.cpp:787-878`

Temporary `std::cerr` debug output from ticket 0035d1 debugging. Should be removed along with the `#include <iostream>` at line 19.

---

## Technical Approach

### Fix 1: `buildFrictionConeSpec()` — use actual row offsets

Replace `normalIdx = 3 * contactIdx` with a two-pass approach:
1. First pass: scan constraints, record row offset for each `ContactConstraint`
2. Second pass: match each `FrictionConstraint` to its corresponding normal by position (i-th friction → i-th normal)

### Fix 2: `solveWithECOS()` — compute n from A matrix

Replace `const int n = 3 * numContacts` with `const int n = static_cast<int>(A.rows())`. The A matrix dimension already reflects the total Jacobian row count.

### Fix 3: Remove debug output

Delete lines 787-878 (all `std::cerr` blocks) and `#include <iostream>` at line 19.

---

## Files

| File | Changes |
|------|---------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Fix `buildFrictionConeSpec()`, fix `solveWithECOS()` n, remove debug output |

---

## Acceptance Criteria

- [ ] **AC1**: `buildFrictionConeSpec()` uses actual row offsets, not `3*contactIdx`
- [ ] **AC2**: `solveWithECOS()` computes n from `A.rows()`, not `3*numContacts`
- [ ] **AC3**: No `std::cerr` debug output in `solveWithECOS()`
- [ ] **AC4**: No `#include <iostream>` in ConstraintSolver.cpp
- [ ] **AC5**: All existing tests pass (zero regressions)

---

## Dependencies

- **Requires**: [0035b4](0035b4_ecos_solve_integration.md) — ECOS solve integration (the code being cleaned up)
- **Blocks**: [0035d5](0035d5_two_phase_friction_solve.md) — Two-phase solve relies on correct ordering

---

## Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Row offset computation incorrect for edge cases | Low | Medium | Unit test with mixed constraint dimensions |

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-01
- **Notes**: Split from centroid reduction plan. Contains correctness fixes and cleanup that are prerequisites for 0035d4/0035d5.

### Implementation Phase
- **Started**: 2026-02-01
- **Completed**: 2026-02-01
- **Artifacts**:
  - Modified: `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`
- **Changes**:
  - **AC1**: Fixed `buildFrictionConeSpec()` to use actual row offsets via two-pass approach (lines 744-781)
  - **AC2**: Fixed `solveWithECOS()` to compute `n` from `A.rows()` instead of `3*numContacts` (line 789)
  - **AC3**: Removed all `std::cerr` debug output blocks (deleted 92 lines)
  - **AC4**: Removed `#include <iostream>` from includes (line 19)
  - **AC5**: All existing tests pass at same baseline as before changes (20 pre-existing failures unrelated to this ticket)
- **Notes**:
  - Test failures (20 total) are pre-existing bugs documented in DEBUG_0035d_friction_energy_injection.md
  - Failures relate to incorrect ECOS problem formulation (H5), not constraint ordering
  - Our changes address prerequisite correctness issues for future centroid reduction work
  - Zero regressions introduced
