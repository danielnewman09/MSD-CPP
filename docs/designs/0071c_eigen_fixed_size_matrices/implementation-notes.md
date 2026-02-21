# Implementation Notes — 0071c: Eigen Fixed-Size Matrices

## Summary

Converted the no-friction solver path's Jacobian storage from dynamic `Eigen::MatrixXd` to fixed-size `Eigen::Matrix<double, 1, 12>` in `ConstraintSolver`. The friction/flat path (`FlattenedConstraints::jacobianRows`) was already using `Matrix<double, 1, 12>` from a prior ticket.

## Files Modified

| File | Change | Purpose |
|------|--------|---------|
| `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Added `JacobianRow` type alias; updated 4 private method signatures | Type boundary declaration |
| `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Converted 4 methods to use `JacobianRow` | Eliminate dynamic Jacobian rows |

## Design Adherence

All conversions are strictly within the scope defined in the ticket:

| Ticket Conversion Point | Status | Notes |
|------------------------|--------|-------|
| `assembleJacobians()` return → `vector<Matrix<double,1,12>>` | Done | Type alias `JacobianRow` used |
| `assembleEffectiveMass()` jacobians param | Done | Uses `leftCols<6>()` / `rightCols<6>()` |
| `assembleRHS()` jacobians param | Done | Uses `dot()` for compile-time scalar product |
| `extractBodyForces()` jacobians param | Done | Uses `leftCols<6>()` / `rightCols<6>()` |
| `FlattenedConstraints::jacobianRows` | Already done | Pre-existing `Matrix<double,1,12>` |
| `assembleFlatEffectiveMass()` | Already done | Pre-existing fixed-size block ops |

## Virtual Boundary

The `Constraint::jacobian()` virtual method still returns `Eigen::MatrixXd` (unchanged — per ticket scope, which explicitly defers interface changes to 0071d). The narrowing copy happens in `assembleJacobians()`:

```cpp
const Eigen::MatrixXd j = contact->jacobian(stateA, stateB, 0.0);
jacobians.push_back(j.row(0));  // Copy row 0 to JacobianRow (Matrix<double,1,12>)
```

This is safe because `ContactConstraint::jacobian()` always returns a 1x12 matrix.

## Test Coverage

- All 718 existing tests pass after changes (no regressions).
- No new test cases required — the change is purely a storage type optimization with identical numerical behavior. Correctness is validated by the existing constraint solver test suite.

## Known Limitations

- The virtual `Constraint::jacobian()` boundary still allocates a temporary `MatrixXd` per constraint call. Eliminating this requires changes to the virtual interface (tracked in ticket 0071d).
- The `assembleJacobians` path is used only for the no-friction (contact-only, small island) case. Large islands go through PGS; friction goes through the flat path. Both of those already used fixed-size types.

## Future Considerations

- Ticket 0071d will address the virtual interface itself, potentially eliminating the `MatrixXd` allocation at the `jacobian()` call site.
