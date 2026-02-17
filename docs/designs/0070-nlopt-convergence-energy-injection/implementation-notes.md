# Implementation Notes — 0070 NLopt Convergence Energy Injection

**Status**: Partial Implementation - Escalation Required
**Branch**: 0070-nlopt-convergence-energy-injection
**Date**: 2026-02-17

---

## Summary

Implemented decoupled normal-then-friction solver as specified in ticket 0070. The normal solve works correctly and fixes F4 (tumbling cube energy injection). However, the per-contact independent friction solve causes regressions due to ignoring multi-contact coupling.

---

## What Was Implemented

### 1. Decoupled Solve Structure

Replaced the single coupled QP with sequential solves:
1. **Normal-only solve**: Extract normal rows, solve with ASM (existing, proven)
2. **Velocity update**: Compute `Jv_post_normal = -b + A * lambda_normal`
3. **Per-contact friction**: Analytic ball projection for each contact independently

### 2. Removed Obsolete Functions

- `solveWithFriction()`: No longer called
- `clampImpulseEnergy()`: No longer needed with corrected formulation
- `clampPositiveWorkFriction()`: No longer needed with corrected formulation

### 3. Key Implementation Details

**Normal solve extraction**:
```cpp
// Extract normal-only subproblem
std::vector<int> normalIndices;
for (size_t i = 0; i < flat.rowTypes.size(); ++i)
{
  if (flat.rowTypes[i] == RowType::Normal)
  {
    normalIndices.push_back(static_cast<int>(i));
  }
}

// Build normal-only A and b matrices
// ... (extract rows/columns)

// Solve with ASM
auto normalResult = solveActiveSet(aNormal, bNormal, numNormals, std::nullopt);
```

**Velocity update** (CRITICAL FIX):
```cpp
// CORRECT: Jv_new = Jv_old + A*lambda = -b + A*lambda (for tangents where b = -Jv_old)
Eigen::VectorXd const jvPostNormal = -b + a * lambdaNormal;

// WRONG (initial implementation): postNormalVelocity = b + a * lambdaNormal
// This gives the NEGATIVE of the correct velocity
```

**Per-contact friction** (flattened structure: [n_0, t1_0, t2_0, n_1, t1_1, t2_1, ...]):
```cpp
for (int c = 0; c < numContacts; ++c)
{
  int const normalIdx = 3 * c;
  int const tangent1Idx = 3 * c + 1;
  int const tangent2Idx = 3 * c + 2;

  // Extract 2x2 tangent block from A
  Eigen::Matrix2d att;
  att(0, 0) = a(tangent1Idx, tangent1Idx);
  att(0, 1) = a(tangent1Idx, tangent2Idx);
  // ...

  // Friction QP RHS: drive tangent velocity to zero
  Eigen::Vector2d bt;
  bt(0) = -jvPostNormal(tangent1Idx);
  bt(1) = -jvPostNormal(tangent2Idx);

  // Unconstrained optimum
  Eigen::Vector2d lambdaTangentStar = att.ldlt().solve(bt);

  // Ball projection
  double const radiusSquared = mu * mu * normalResult.lambda(c) * normalResult.lambda(c);
  if (lambdaTangentStar.squaredNorm() > radiusSquared)
  {
    lambdaTangentStar *= std::sqrt(radiusSquared / lambdaTangentStar.squaredNorm());
  }

  lambdaFriction(tangent1Idx) = lambdaTangentStar(0);
  lambdaFriction(tangent2Idx) = lambdaTangentStar(1);
}
```

---

## Test Results

### Iteration 1
- **Build**: PASS
- **Tests**: 685/697 (baseline: 690/697)
- **Impact**: F4 fixed (+1), 5 regressions (-5)

### Failures Analysis

**Fixed**:
- F4 (RotationalEnergyTest_F4_RotationEnergyTransfer_EnergyConserved): Energy injection eliminated with decoupled normal solve

**Regressions** (12 total failures, 7 baseline = 5 new):
- A4 (LinearCollisionTest_A4_EqualMassElastic_VelocitySwap)
- A6 (LinearCollisionTest_A6_GlancingCollision_MomentumAndEnergyConserved): WORSE than baseline (1.39J vs 0.286J claimed in ticket)
- D4 (ContactManifoldStabilityTest_D4_MicroJitter_DampsOut)
- H5 (ParameterIsolation_H5_ContactPointCount_EvolutionDiagnostic)
- H6 (ParameterIsolation_H6_ZeroGravity_RestingContact_Stable)
- Friction tests (FrictionDirectionTest x2, FrictionConeSolverTest)

---

## Root Cause of Regressions

### Per-Contact Independent Solve is Too Approximate

The per-contact friction solve treats each contact independently:
```cpp
A_tt_contact_c * lambda_t_c = b_t_c
```

This ignores coupling between contacts through the A matrix. For contacts on the same body:
```
ΔKE = lambda^T * Jv + 0.5 * lambda^T * A * lambda
```

The off-diagonal blocks `A[contact_i, contact_j]` (i ≠ j) represent how friction at contact i affects tangent velocity at contact j (through shared body motion). Solving independently sets these to zero, which is physically incorrect and violates energy conservation.

**Example**: A6 glancing collision
- Two bodies with off-center contact
- Friction at contact induces rotation
- Rotation couples back to tangent velocity through lever arm
- Per-contact solve misses this coupling → energy injection

---

## Next Steps (Requires Human Decision)

### Option 1: Gauss-Seidel Iteration

Iterate the per-contact solve to convergence:
```cpp
for (int iter = 0; iter < maxIter; ++iter)
{
  for (int c = 0; c < numContacts; ++c)
  {
    // Recompute b_t_c using current lambda for all other contacts
    Eigen::Vector2d bt = -jvPostNormal.segment<2>(3*c + 1)
                         - (A.block<2, 3*numContacts>(3*c+1, 0) * lambdaFriction).segment<2>(0);
    // Solve contact c with updated RHS
    // ...
  }
  if (converged) break;
}
```

**Pros**: Handles multi-contact coupling, simple to implement
**Cons**: Requires iteration, may converge slowly for stiff problems

### Option 2: Joint Tangent Solve

Solve the full tangent system as a single QP:
```cpp
min 0.5 * lambda_t^T * A_tt * lambda_t + b_t^T * lambda_t
s.t. ||lambda_t[2*c:2*c+2]|| ≤ mu_c * lambda_n[c]  for each contact c
```

This is a SOCP with per-contact ball constraints (same as original NLopt formulation, but AFTER normal solve).

**Pros**: Exact solution, handles coupling correctly
**Cons**: Still requires NLopt or similar SOCP solver

### Option 3: Hybrid Approach

Use Gauss-Seidel for most frames, fall back to joint solve when iteration doesn't converge.

**Pros**: Fast common path, robust fallback
**Cons**: More complex, two code paths to maintain

---

## Files Modified

| File | Change |
|------|--------|
| `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Decoupled solve, removed clamps, velocity update fix |
| `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Removed obsolete method declarations |

---

## Recommendations

1. **Short term**: Implement Gauss-Seidel iteration (Option 1) as a quick fix for regressions
2. **Long term**: Consider joint tangent solve (Option 2) for robustness, or profile to see if Gauss-Seidel is sufficient
3. **Test specifically**: A6 glancing collision should be the litmus test - it MUST conserve energy with this fix

---

## Iteration Log

See `docs/designs/0070-nlopt-convergence-energy-injection/iteration-log.md` for full build-test history.
