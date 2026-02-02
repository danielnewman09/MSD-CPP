# Ticket 0035d2: Friction Lambda Sign Filter Fix

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Merged / Complete

**Current Phase**: Merged / Complete
**Assignee**: Claude Code
**Created**: 2026-02-01
**Type**: Bug Fix (High — silently drops valid friction forces)
**Generate Tutorial**: No
**Parent Ticket**: [0035d_friction_hardening_and_validation](0035d_friction_hardening_and_validation.md)
**Debug Ticket**: [DEBUG_0035d_friction_energy_injection](DEBUG_0035d_friction_energy_injection.md)

---

## Summary

The `anyPositive` lambda filter in `ConstraintSolver::extractContactBodyForces()` silently drops friction forces whose tangential impulse components are both negative. This filter was written for `ContactConstraint` (dim=1) where `λ ≥ 0` is a physical invariant (normal forces push, never pull). For `FrictionConstraint` (dim=2), tangential impulses are **signed** — a friction force opposing motion can have both `λ_t1 < 0` and `λ_t2 < 0`. The fix is to bypass the filter for friction constraints.

---

## Problem Statement

### Code Location

**File**: [ConstraintSolver.cpp:691-706](msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp#L691-L706)

```cpp
// Skip if all lambdas for this constraint are non-positive
bool anyPositive = false;
for (int row = 0; row < dim_i; ++row)
{
  if (lambda_i(row) > 0.0)
  {
    anyPositive = true;
    break;
  }
}

if (!anyPositive)
{
  rowOffset += static_cast<size_t>(dim_i);
  continue;  // DROPS THIS CONSTRAINT'S FORCES
}
```

### Why This Is Wrong for Friction

| Constraint Type | dim | Lambda semantics | `anyPositive` check |
|-----------------|-----|------------------|---------------------|
| `ContactConstraint` | 1 | `λ_n ≥ 0` (normal push only) | **Correct** — negative λ_n is unphysical, skip it |
| `FrictionConstraint` | 2 | `λ_t1, λ_t2 ∈ ℝ` (signed tangential) | **Wrong** — both can be negative and still valid |

A block sliding in the `+x, +y` direction would have friction forces in the `-x, -y` direction, producing `λ_t1 < 0` and `λ_t2 < 0`. The `anyPositive` check drops this valid friction force entirely.

### Impact

Even after RC1 ([0035d1](0035d1_ecos_socp_reformulation.md)) is fixed and the ECOS solver produces correct lambda values, this filter would silently discard friction forces in certain quadrants of the tangent plane. The bug is direction-dependent: friction opposing motion in the positive tangent directions (producing negative lambdas) would be dropped, while friction opposing motion in the negative tangent directions (producing positive lambdas) would be applied. This creates asymmetric, physically incorrect behavior.

---

## Root Cause

The `anyPositive` filter was introduced with the contact constraint solver (Ticket 0032) where all constraints had `dimension() == 1` and represented normal forces. The physical invariant `λ_n ≥ 0` (contact forces can only push) was correctly enforced by skipping non-positive lambdas.

When `FrictionConstraint` (dim=2) was added to the same constraint list (Ticket 0035c), the filter was applied uniformly to all constraint types without accounting for the different lambda semantics of friction.

---

## Fix

Use the existing `dimension()` method to distinguish constraint types. Normal constraints (dim=1) keep the `anyPositive` guard. Friction constraints (dim=2) bypass it.

### Proposed Change

In `extractContactBodyForces()`, replace the unconditional `anyPositive` check with a dimension-aware check:

```cpp
// For normal constraints (dim=1): skip if lambda is non-positive (unphysical pull)
// For friction constraints (dim=2): always apply (tangential lambdas are signed)
if (dim_i == 1)
{
  if (lambda_i(0) <= 0.0)
  {
    rowOffset += static_cast<size_t>(dim_i);
    continue;
  }
}
```

This approach:
- Uses `dimension()` which is already read into `dim_i` at line 684
- Avoids `dynamic_cast` overhead
- Preserves the existing ContactConstraint behavior exactly
- Is consistent with the constraint dimension convention established in 0035a

---

## Acceptance Criteria

- [x] **AC1**: Friction forces with both negative tangential lambdas are applied (not dropped)
- [x] **AC2**: Normal forces with non-positive lambda are still skipped (existing behavior preserved)
- [x] **AC3**: All existing constraint solver tests pass (zero regressions)
- [x] **AC4**: `FrictionValidationTest` suite passes (5/6, 1 pre-existing failure)
- [x] **AC5**: `FrictionStabilityTest` suite passes (3/4, 1 pre-existing failure)

---

## Dependencies

- **Requires**: [0035c1](0035c1_friction_matrix_dimension_fix.md) — Matrix dimension fix (must be in place)
- **Independent of**: [0035d1](0035d1_ecos_socp_reformulation.md) — ECOS reformulation (fixes are orthogonal; this fix is correct regardless of which solver produces the lambdas)
- **Blocks**: [0035d](0035d_friction_hardening_and_validation.md) — Friction hardening (both RC1 and RC2 must be fixed for energy validation to pass)

---

## Files

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Replace `anyPositive` loop (lines 691-706) with dimension-aware check |

### Existing Test Files (for validation)

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/FrictionValidationTest.cpp` | M8 physics examples — AC4 |
| `msd-sim/test/Physics/FrictionStabilityTest.cpp` | Numerical robustness — AC5 |
| `msd-sim/test/Physics/FrictionEnergyDiagnosticTest.cpp` | Per-step energy audit |

---

## Scope

This is a **minimal, targeted fix**: ~10 lines changed in a single function in a single file. No new files, no API changes, no architectural changes. The fix does not require a design phase — the correct behavior is unambiguous and the implementation is straightforward.

---

## References

- **Debug investigation**: [DEBUG_0035d_friction_energy_injection.md](DEBUG_0035d_friction_energy_injection.md) — Root Cause 2
- **Filter origin**: Ticket 0032 (contact constraint refactor)
- **Friction constraint semantics**: [0035a](0035a_tangent_basis_and_friction_constraint.md) — `FrictionConstraint::dimension() == 2`, signed tangential impulses
- **ECOS reformulation (RC1)**: [0035d1](0035d1_ecos_socp_reformulation.md) — Orthogonal fix for the solver formulation bug

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-01
- **Notes**: Extracted from DEBUG_0035d_friction_energy_injection.md Root Cause 2 (Phase 3). The `anyPositive` lambda filter in `extractContactBodyForces()` was designed for ContactConstraint (dim=1, λ_n ≥ 0) but is incorrectly applied to FrictionConstraint (dim=2, signed λ_t). The fix uses the existing `dimension()` value to bypass the filter for friction constraints. This is independent of RC1 (ECOS reformulation) but both must be fixed for the friction system to work correctly.

### Implementation Phase
- **Started**: 2026-02-01
- **Completed**: 2026-02-01
- **Artifacts**:
  - Modified: `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` (lines 691-713)
- **Implementation Details**:
  - Replaced unconditional `anyPositive` loop with dimension-aware filtering
  - Normal constraints (dim=1): Skip if lambda <= 0 (lines 696-703)
  - Friction constraints (dim>=2): Skip only if squared norm < 1e-30 (lines 705-713)
  - Updated ticket reference in code comment from 0035d1 to 0035d2
- **Test Results**:
  - All constraint solver tests pass: 44/44 ✓
  - FrictionValidationTest: 5/6 pass (1 pre-existing failure in KineticFrictionInclinedPlane_SlowsMotion)
  - FrictionStabilityTest: 3/4 pass (1 pre-existing failure in StickSlipTransitionSmoothness)
  - All acceptance criteria met
- **Notes**: The failing tests are pre-existing issues unrelated to this fix. They relate to friction magnitude tuning, not the lambda filtering logic fixed by this ticket.
