# Design: ECOS SOCP Problem Reformulation (As-Implemented)

## Summary

This design documents the ECOS friction contact problem reformulation from an over-constrained feasibility problem to a proper Second-Order Cone Program (SOCP) using epigraph lifting with an auxiliary-variable formulation. The implementation converts the Quadratic Program (QP) `min (1/2)*lambda^T*A*lambda - b^T*lambda` subject to friction cone constraints into ECOS standard form by introducing auxiliary variables `y = L^T*lambda` and an epigraph variable `t`. This avoids backward substitution through potentially ill-conditioned upper triangular matrices and delegates the QP objective to a rotated second-order cone constraint. This document replaces the original approved design which described a simpler `x = [lambda; t]` formulation that was never implemented.

**Mathematical Foundation**: See `math-formulation.md` for the original QP derivation and epigraph lifting technique. Note that `math-formulation.md` uses `L*lambda` and `L^{-T}*b` in the epigraph constraint, while the implementation correctly uses `L^T*lambda` and `L^{-1}*b` (see "Mathematical Correctness Proof" section below for the verification).

---

## Architecture Changes

### PlantUML Diagram

See: `./0035d1_ecos_socp_reformulation.puml`

### Mathematical Correctness Proof

The QP objective is:

```
f(lambda) = (1/2)*lambda^T*A*lambda - b^T*lambda
```

With `A = L*L^T` (Cholesky), let `u = L^T*lambda`. Then `||u||^2 = lambda^T*L*L^T*lambda = lambda^T*A*lambda` and `b^T*lambda = b^T*(L^T)^{-1}*u = (L^{-1}*b)^T*u`. Completing the square:

```
f = (1/2)*||u||^2 - (L^{-1}*b)^T*u
  = (1/2)*||u - L^{-1}*b||^2 - (1/2)*||L^{-1}*b||^2
```

Define `d = L^{-1}*b` (computed via forward substitution, which is numerically stable since L is lower triangular). Then:

```
f = (1/2)*||L^T*lambda - d||^2 - const
```

The standard epigraph reformulation encodes `t >= ||L^T*lambda - d||^2` as:

```
||(2*(L^T*lambda - d), t-1)||_2 <= t+1
```

which expands to `4*||L^T*lambda - d||^2 + (t-1)^2 <= (t+1)^2`, simplifying to `||L^T*lambda - d||^2 <= t`.

The auxiliary-variable formulation introduces `y = L^T*lambda` as equality constraints, so the epigraph operates on `y` directly:

```
||(2*(y - d), t-1)||_2 <= t+1,   L^T*lambda - y = 0
```

Minimizing `t` is equivalent to minimizing `f(lambda)` because `t >= 2*f(lambda) + const` and the constant does not affect the argmin.

**Important correction**: The `math-formulation.md` document uses `L*lambda` and `L^{-T}*b` in the epigraph constraint (Section 2, Step 5). This is a transpositional error. The correct forms are `L^T*lambda` and `L^{-1}*b`, as verified above and as the implementation uses. The error in the math doc does not affect the implementation since the implementation was coded correctly.

### Design Rationale: Why Auxiliary Variables?

The original approved design (never implemented) used `x = [lambda; t]` with the epigraph constraint containing `2*L*lambda` in the G matrix. This would require computing `L^{-T}*b` via backward substitution through an upper triangular matrix, which can amplify errors when L is ill-conditioned.

The auxiliary-variable formulation avoids this by:
1. Computing `d = L^{-1}*b` via **forward** substitution (numerically stable for lower triangular L)
2. Introducing `y = L^T*lambda` as equality constraints (exact, no inversion needed)
3. Having the epigraph cone operate on `y` variables directly with `d` as a constant in `h`

The trade-off is increased problem size (6C+1 variables vs 3C+1, plus 3C equality constraints), but ECOS handles this efficiently via its interior-point method.

---

### Modified Components

#### ECOSProblemBuilder

**Current location**: `msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp`, `.cpp`

**Changes from original (0035b3) implementation**:
- **Removed**: Direct equality constraint `A*lambda = b` (the old over-constrained feasibility problem)
- **Added**: Cholesky factorization `A = L*L^T` with regularization fallback
- **Added**: Auxiliary-variable formulation with `y = L^T*lambda` as equality constraints
- **Added**: Epigraph cone constraint encoding `t >= ||y - d||^2` where `d = L^{-1}*b`
- **Modified**: Problem dimensions completely changed (see table below)

**Dimension comparison**:

| Quantity | Old (0035b3) | Original Design (never impl.) | Actual Implementation |
|----------|-------------|-------------------------------|----------------------|
| Variables | 3C | 3C+1 | **6C+1** `[lambda; y; t]` |
| Equality constraints | 3C (`A*lambda = b`) | 0 | **3C** (`L^T*lambda - y = 0`) |
| G matrix rows | 3C | 6C+1 | **6C+2** |
| G matrix cols | 3C | 3C+1 | **6C+1** |
| Epigraph cone size | N/A | 3C+1 | **3C+2** |
| Friction cone sizes | [3, 3, ..., 3] | [3, 3, ..., 3] | [3, 3, ..., 3] |
| Cone sizes array | [3, ..., 3] | [3C+1, 3, ..., 3] | **[3C+2, 3, ..., 3]** |
| c vector | [0, ..., 0] | [0, ..., 0, 1] | **[0, ..., 0, 1]** (6C+1 elements) |
| h vector | [0, ..., 0] | [1, 2*L^{-T}*b, -1, 0, ..., 0] | **[1, -2*d, -1, 0, ..., 0]** (6C+2 elements) |

**Variable layout**: `x = [lambda_n1, lambda_t1_1, lambda_t2_1, ..., lambda_nC, lambda_t1_C, lambda_t2_C, y_1, ..., y_3C, t]`

**Internal methods (all static, private)**:

```cpp
// Cholesky factorization with regularization (epsilon = 1e-8)
static Eigen::MatrixXd computeCholeskyFactor(const Eigen::MatrixXd& A);

// Precompute epigraph terms: 2*L, 2*L^{-1}*b, cone size = 3C+2
static EpigraphConeData buildEpigraphCone(
    const Eigen::MatrixXd& L,
    const Eigen::VectorXd& b,
    int numContacts);

// Build G matrix (6C+2) x (6C+1) in CSC format
static ECOSSparseMatrix buildExtendedGMatrix(
    const EpigraphConeData& epigraphData,
    const FrictionConeSpec& coneSpec,
    int numContacts);

// Build h vector (6C+2 elements)
static std::vector<pfloat> buildExtendedHVector(
    const EpigraphConeData& epigraphData,
    int numContacts);

// Build c vector (6C+1 elements): [0, ..., 0, 1]
static std::vector<pfloat> buildExtendedCVector(int numContacts);
```

**Internal data structure**:

```cpp
struct EpigraphConeData {
    Eigen::MatrixXd L_times_2;           // 2*L (3C x 3C) — stored for equality constraint construction
    Eigen::VectorXd L_inv_T_b_times_2;   // 2*d = 2*L^{-1}*b (3C x 1) — FIELD NAME IS MISLEADING
    idxint epigraph_cone_size;            // 3C+2 (NOT 3C+1 as the field comment claims)

    EpigraphConeData() = default;
};
```

**G matrix block structure** (6C+2 rows x 6C+1 cols):

```
Epigraph cone block (3C+2 rows):
  Row 0:         [0..0 (lambda cols), 0..0 (y cols), -1 (t col)]
  Rows 1..3C:    [0..0 (lambda cols), -2*I  (y cols),  0 (t col)]
  Row 3C+1:      [0..0 (lambda cols), 0..0 (y cols), -1 (t col)]

Friction cone block (3C rows, on lambda variables):
  Per contact i:
    Row 3C+2+3i:     [-mu_i at col 3i, rest 0]
    Row 3C+2+3i+1:   [-1 at col 3i+1, rest 0]
    Row 3C+2+3i+2:   [-1 at col 3i+2, rest 0]
```

ECOS interprets `s = h - G*x`:
- `s_0 = 1 - (-1)*t = 1 + t` (epigraph bound, must be >= 0)
- `s_{1..3C} = -2*d_i - (-2)*y_i = 2*(y_i - d_i)` (epigraph norm components)
- `s_{3C+1} = -1 - (-1)*t = t - 1` (epigraph norm component)
- Friction: `s = [mu*lambda_n, lambda_t1, lambda_t2]` (standard friction cone)

Cone constraint: `||s_{1..3C+1}|| <= s_0` gives `||(2*(y-d), t-1)|| <= t+1`.

**h vector** (6C+2 elements):
```
h[0] = 1
h[1..3C] = -2*d_i  where d = L^{-1}*b
h[3C+1] = -1
h[3C+2..6C+1] = 0  (friction cone RHS)
```

**Equality constraint matrices** (3C rows x (6C+1) cols):

```
A_eq = [L^T | -I | 0]   (3C x 6C+1)
b_eq = [0, ..., 0]       (3C x 1)
```

This encodes `L^T*lambda - y = 0`, i.e., `y = L^T*lambda`.

The `A_eq` matrix is built in CSC format:
- Lambda columns (0..3C-1): Upper triangular entries from L^T (since L^T(row,col) = L(col,row), nonzeros at rows 0..col for each column)
- y columns (3C..6C-1): Diagonal entries of -1
- t column (6C): No entries

**build() method flow**:
1. Validate inputs (A square, dimensions match, numContacts > 0)
2. Compute Cholesky factor L with regularization fallback (epsilon = 1e-8)
3. Precompute epigraph data: `2*L`, `2*L^{-1}*b`, cone size `3C+2`
4. Build extended G matrix (6C+2 x 6C+1) in CSC format
5. Build extended h vector (6C+2 elements)
6. Build extended c vector (6C+1 elements)
7. Build cone sizes: [3C+2, 3, 3, ..., 3]
8. Build equality constraint A_eq (3C x 6C+1) in CSC format
9. Build equality b_eq (3C zeros)
10. Create ECOSData with 6C+1 variables, 6C+2 inequality rows, C+1 cones, 3C equality constraints

**Backward compatibility**: The `build()` method signature is unchanged. Callers pass the same `(A, b, coneSpec)` arguments and receive an `ECOSData` ready for `setup()`. The increased problem size is transparent to callers.

---

#### ConstraintSolver

**Current location**: `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`, `.cpp`

**Modified method** (`solveWithECOS()`):

The `solveWithECOS()` method at line 820 of `ConstraintSolver.cpp` correctly extracts lambda from the extended solution vector. The solution vector `workspace->x` has dimension 6C+1 = `[lambda(3C); y(3C); t(1)]`. The extraction uses `n = A.rows() = 3C`:

```cpp
Eigen::VectorXd lambda = Eigen::Map<Eigen::VectorXd>(workspace->x, n);
```

This correctly takes the first 3C elements, discarding auxiliary `y` and epigraph `t`.

**Stale comments to fix**: Line 830 says "Problem now has 3C+1 variables [lambda; t], no equality constraints" but the actual problem has 6C+1 variables and 3C equality constraints. Line 849 says "Solution vector has dimension 3C+1" but it has dimension 6C+1.

**No functional changes needed**: The extraction logic is correct.

---

#### ECOSData

**Current location**: `msd-sim/src/Physics/Constraints/ECOS/ECOSData.hpp`, `.cpp`

**Changes from original**: The ECOSData constructor now receives 4 parameters including `numEquality`:

```cpp
ECOSData data{static_cast<idxint>(numVars), numInequalityRows, numCones,
              static_cast<idxint>(numEq)};
```

Where `numVars = 6C+1`, `numInequalityRows = 6C+2`, `numCones = C+1`, `numEq = 3C`.

**Stale comments to fix in ECOSData.hpp**:
- Line 61: Says "Decision variables (3C+1 after epigraph reformulation)" — should be "6C+1"
- Line 64: Says "Number of equality constraints (0 after ticket 0035d1)" — should be "3C"

The `setup()` method correctly handles equality constraints via the conditional pointer passing (lines 141-144), which was already implemented in ticket 0035b4.

---

#### ECOSProblemBuilder.hpp (Header Documentation)

**Current location**: `msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp`

The header documentation is deeply stale and contradicts the implementation in multiple places. The following corrections are needed:

| Line | Current (Incorrect) | Correct |
|------|-------------------|---------|
| 30 | `EpigraphConeData::epigraph_cone_size: Always 3C+1` | `Always 3C+2` |
| 29 | `L_inv_T_b_times_2: 2*L^{-T}*b (3C x 1)` | `2*L^{-1}*b (3C x 1)` — field name is misleading |
| 44 | `||(2*L*lambda - 2*L^{-T}*b, t-1)|| <= t+1` | `||(2*(y - d), t-1)|| <= t+1 where y = L^T*lambda, d = L^{-1}*b` |
| 64 | `Extended variable vector: x = [lambda; t] (dimension 3C+1)` | `x = [lambda; y; t] (dimension 6C+1)` |
| 65 | `Extended G matrix: (6C+1) x (3C+1)` | `(6C+2) x (6C+1)` |
| 66 | `Cone structure: [3C+1, 3, 3, ..., 3]` | `[3C+2, 3, 3, ..., 3]` |
| 86 | `Postcondition: The equality constraint ... is removed` | `Equality constraints: L^T*lambda - y = 0 (3C equations)` |
| 96 | `G matrix is (6C+1) x (3C+1)` | `G matrix is (6C+2) x (6C+1)` |
| 97 | `h vector has dimension 6C+1 with ... [1; 2*L^{-T}*b; -1; 0; ...]` | `h vector has dimension 6C+2 with [1; -2*L^{-1}*b; -1; 0; ...]` |
| 98 | `c vector has dimension 3C+1 with [0; ...; 0; 1]` | `c vector has dimension 6C+1` |
| 99 | `cone_sizes is [3C+1, 3, 3, ..., 3]` | `[3C+2, 3, 3, ..., 3]` |
| 100 | `num_equality = 0 (NO equality constraints)` | `num_equality = 3C` |
| 148-149 | `L_inv_T_b_times_2: Vector 2*L^{-T}*b for RHS constant` / `epigraph_cone_size: Always 3C+1` | `2*L^{-1}*b` / `Always 3C+2` |
| 166-167 | `G matrix (6C+1 rows x 3C+1 cols)` | `G matrix (6C+2 rows x 6C+1 cols)` |
| 189 | `h vector (6C+1 elements): ... [1; 2*L^{-T}*b; -1; 0; ...]` | `h vector (6C+2 elements): ... [1; -2*L^{-1}*b; -1; 0; ...]` |
| 207-208 | `c vector (3C+1 elements)` | `c vector (6C+1 elements)` |

---

### Identified Issues in Current Implementation

After thorough analysis, the SOCP formulation itself is **mathematically correct**. The auxiliary-variable approach correctly encodes the QP objective as an epigraph constraint. The G matrix signs, h vector values, equality constraints, and cone structure are all consistent with the ECOS standard form.

The remaining test failures (0035d7 and 0035d8) are likely caused by:

#### Issue 1: Stale Header Documentation Creates Maintenance Hazard

The header file `ECOSProblemBuilder.hpp` documents dimensions, formulations, and postconditions that are completely wrong relative to the implementation. Any developer or test writer relying on the header documentation would write incorrect tests or integration code.

**Severity**: High (maintenance/correctness hazard)
**Fix**: Update all header documentation to match the implementation.

#### Issue 2: Misleading Field Name in EpigraphConeData

The field `L_inv_T_b_times_2` is named to suggest it stores `2*L^{-T}*b` but actually stores `2*L^{-1}*b`. The comment on line 76 of `ECOSProblemBuilder.cpp` acknowledges this: "Reusing field name; actually stores 2*L^{-1}*b".

**Severity**: Medium (code clarity, no functional impact)
**Fix**: Rename field to `d_times_2` or `L_inv_b_times_2`.

#### Issue 3: Stale Comments in ConstraintSolver.cpp

Lines 830 and 849 describe the problem as having "3C+1 variables [lambda; t], no equality constraints" when the implementation produces 6C+1 variables with 3C equality constraints.

**Severity**: Low (comments only, no functional impact)
**Fix**: Update comments.

#### Issue 4: Test Expectations May Not Account for Auxiliary-Variable Formulation

The failing tests in `ECOSFrictionValidationTest` and `ECOSSolveTest` were written before the auxiliary-variable formulation was implemented. They check:
- `result.lambda.size() == 3` (correct since extraction takes first 3C elements)
- `result.lambda(0) == 10.0` (depends on solver producing correct optimal)
- Convergence (`result.converged == true`)

If the tests are failing on convergence or solution quality, the root cause is likely **not** in the SOCP formulation itself but in one of:
1. The ECOS solver struggling with the larger problem size (6C+1 vs 3C+1 variables)
2. Numerical conditioning of the equality constraints
3. ECOS tolerance settings being too tight for the auxiliary-variable formulation
4. An integration-level issue in how the solver results are applied to bodies

**Severity**: Critical (blocks friction validation)
**Recommended Investigation**: Run the stick-regime test (`A = I, b = [10, 1, 1]`) and print the full ECOS workspace state (exit flag, primal/dual residuals, gap, solution vector) to determine if ECOS reaches optimality.

---

### Integration Points

| Component | Existing Component | Integration Type | Notes |
|-----------|-------------------|------------------|-------|
| `computeCholeskyFactor()` | ConstraintSolver regularization | Algorithm reuse | Same regularization pattern as `applyRegularizationFallback()` |
| `buildExtendedGMatrix()` | ECOS C API | CSC sparse matrix | 6C+2 rows for epigraph + friction cones |
| `build()` → `ECOSData` | `ECOSData::setup()` | Problem construction | Creates ECOSData with 6C+1 vars, 3C equality constraints |
| `solveWithECOS()` | `ECOSProblemBuilder::build()` | Solution extraction | Extracts first 3C elements from 6C+1 solution vector |
| Equality constraints | ECOS C API `ECOS_setup()` | A_eq/b_eq parameters | 3C rows linking y to L^T*lambda |

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `ECOSProblemBuilderTest.cpp` | All 14 tests | Tests rewritten by 0035d6 for SOCP structure | Verify all pass with current formulation |
| `ECOSDataTest.cpp` | `ECOSProblemBuilderPopulatesEqualityConstraints` | Verifies A_eq and b_eq | Must test for 3C equality constraints (not 0) |
| `ECOSFrictionValidationTest.cpp` | `StickRegime_InteriorSolution` | End-to-end solver test | Verify ECOS produces correct lambda |
| `ECOSFrictionValidationTest.cpp` | `SlipRegime_ConeBoundarySolution` | End-to-end solver test | Verify lambda on cone boundary |
| `ECOSFrictionValidationTest.cpp` | `StickSlipTransition_ContinuousTransition` | Sweep test | Verify continuous transition |
| `ECOSSolveTest.cpp` | `SingleContactStickRegime` | Solver integration test | Verify lambda = [10, 1, 1] for A=I, b=[10,1,1] |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| `ECOSProblemBuilder` | `AuxiliaryVariableDimensions` | numVars = 6C+1, numEq = 3C, G rows = 6C+2 |
| `ECOSProblemBuilder` | `EqualityConstraintStructure` | A_eq = [L^T; -I; 0] with correct sparsity pattern |
| `ECOSProblemBuilder` | `EpigraphConeSize` | cone_sizes[0] = 3C+2 |
| `ECOSProblemBuilder` | `ForwardSubstitutionCorrectness` | d = L^{-1}*b matches analytical for diagonal A |
| `ECOSProblemBuilder` | `HVectorSignConvention` | h[0]=1, h[1..3C]=-2*d, h[3C+1]=-1 |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `SticKRegimeIdentityMass` | ECOSProblemBuilder + ECOS solver | lambda = b for A=I, b inside cone |
| `SlipRegimeClampsToCone` | ECOSProblemBuilder + ECOS solver | ||lambda_t|| = mu*lambda_n when b outside cone |
| `SlidingBlockPhysics` | Full pipeline | Energy monotonically decreasing for friction-only scenario |

---

## Header Documentation Update Plan

The following files require documentation updates (no functional changes):

### ECOSProblemBuilder.hpp

Replace the class-level docstring (lines 36-73) with documentation that accurately describes:
- Variable layout: `x = [lambda(3C); y(3C); t(1)]`, dimension 6C+1
- Equality constraints: `L^T*lambda - y = 0` (3C equations)
- Epigraph cone: `||(2*(y - d), t-1)|| <= t+1`, dimension 3C+2
- Friction cones: `||[lambda_t1, lambda_t2]|| <= mu*lambda_n`, dimension 3 each
- G matrix: (6C+2) x (6C+1)
- h vector: 6C+2 elements
- c vector: 6C+1 elements
- Cone sizes: [3C+2, 3, ..., 3]
- num_equality: 3C

Replace the `EpigraphConeData` struct documentation (lines 26-33) with:
- `L_times_2`: 2*L (stored for equality constraint A_eq construction)
- `L_inv_T_b_times_2`: Actually stores 2*d = 2*L^{-1}*b (field name is a legacy artifact)
- `epigraph_cone_size`: 3C+2

Replace the `build()` method docstring (lines 78-115) with accurate postconditions reflecting the auxiliary-variable formulation.

Replace each private method docstring to reflect actual dimensions and computations.

### ECOSData.hpp

Update dimension comments on lines 61-64 to reflect:
- `num_variables_`: 6C+1 after auxiliary-variable reformulation
- `num_equality_`: 3C after auxiliary-variable reformulation (not 0)

### ConstraintSolver.cpp

Update stale comments on lines 830 and 849 to reflect 6C+1 variables and 3C equality constraints.

---

## Open Questions

### Design Decisions (Human Input Needed)

1. **Rename `EpigraphConeData::L_inv_T_b_times_2` to `d_times_2`?**
   - Option A: Rename to `d_times_2` — Pros: Accurate, concise. Cons: Breaks any external references to the field.
   - Option B: Rename to `L_inv_b_times_2` — Pros: Follows original naming pattern. Cons: Still partially misleading.
   - Option C: Keep current name, update comment only — Pros: No code changes. Cons: Name remains misleading.
   - Recommendation: Option A. The struct is internal (private to ECOSProblemBuilder) so no external references exist.

2. **Should the math-formulation.md transpositional error be corrected?**
   - Option A: Correct `L*lambda` to `L^T*lambda` and `L^{-T}*b` to `L^{-1}*b` — Pros: Document becomes accurate. Cons: Changes an approved document.
   - Option B: Add a correction note referencing this design doc — Pros: Preserves review history. Cons: Reader must cross-reference.
   - Recommendation: Option A with a revision note at the top of the document.

3. **Are the 4 ECOS solver integration test failures (0035d7) caused by the SOCP formulation or by something else?**
   - The SOCP formulation has been verified mathematically correct in this analysis.
   - The test failures should be investigated by running ECOS in verbose mode on a simple test case (A=I, b=[10,1,1], mu=0.5) and examining the ECOS exit flag, primal/dual residuals, and solution vector.
   - If ECOS returns `ECOS_OPTIMAL` with correct lambda, the issue is in the integration pipeline (force extraction, body force application).
   - If ECOS does not return `ECOS_OPTIMAL`, there may be a numerical issue with the problem conditioning or a bug in the CSC matrix construction that unit tests do not catch (e.g., column pointer off-by-one in the A_eq construction).

### Prototype Required

1. **ECOS solver diagnostic run**: Execute the stick-regime test case (`A = I_{3x3}, b = [10, 1, 1], mu = 0.5`) through the full `ECOSProblemBuilder::build()` -> `ECOSData::setup()` -> `ECOS_solve()` pipeline, printing all intermediate matrices (G, h, c, A_eq, b_eq, cone_sizes) and the ECOS workspace output (exit flag, solution vector, residuals). This will definitively identify whether the test failures are in the SOCP formulation or downstream.

### Requirements Clarification

1. **Ticket 0035d7 test `ECOSProblemBuilderPopulatesEqualityConstraints`**: This test name suggests it verifies equality constraints exist. The current implementation does have equality constraints (3C of them), so this test should pass unless it expects a specific structure that does not match. Need to verify the test expectations against the actual A_eq structure.

---

## Appendix: Complete Variable and Matrix Layout for C=1 (Single Contact)

For a single contact (C=1, 3 physical variables):

**Variables** (7 total): `x = [lambda_n, lambda_t1, lambda_t2, y_1, y_2, y_3, t]`

**Objective**: `c = [0, 0, 0, 0, 0, 0, 1]` (minimize t)

**Equality constraints** (3 equations):
```
A_eq * x = b_eq

A_eq = [L^T(0,0)  L^T(0,1)  L^T(0,2)  -1   0   0   0]   = [L(0,0)     0         0       -1  0  0  0]
       [L^T(1,0)  L^T(1,1)  L^T(1,2)   0  -1   0   0]     [L(0,1)  L(1,1)       0        0 -1  0  0]
       [L^T(2,0)  L^T(2,1)  L^T(2,2)   0   0  -1   0]     [L(0,2)  L(1,2)    L(2,2)      0  0 -1  0]

b_eq = [0, 0, 0]
```

(Since L is lower triangular, `L^T(i,j) = L(j,i)`, nonzero only when j >= i.)

**Inequality constraint G matrix** (8 rows x 7 cols):
```
Epigraph cone (5 rows = 3+2):
Row 0: [0  0  0   0   0   0  -1]     s_0 = 1 + t
Row 1: [0  0  0  -2   0   0   0]     s_1 = -2*d_1 + 2*y_1 = 2*(y_1 - d_1)
Row 2: [0  0  0   0  -2   0   0]     s_2 = -2*d_2 + 2*y_2 = 2*(y_2 - d_2)
Row 3: [0  0  0   0   0  -2   0]     s_3 = -2*d_3 + 2*y_3 = 2*(y_3 - d_3)
Row 4: [0  0  0   0   0   0  -1]     s_4 = -1 + t = t - 1

Friction cone (3 rows):
Row 5: [-mu  0   0   0   0   0   0]  s_5 = mu*lambda_n
Row 6: [ 0  -1   0   0   0   0   0]  s_6 = lambda_t1
Row 7: [ 0   0  -1   0   0   0   0]  s_7 = lambda_t2
```

**h vector** (8 elements): `[1, -2*d_1, -2*d_2, -2*d_3, -1, 0, 0, 0]`

**Cone sizes**: `[5, 3]` (epigraph Q^5, friction Q^3)

**ECOS parameters**: `n=7, m=8, p=3, l=0, ncones=2, q=[5,3]`

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-01
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | Pass | Class names (ECOSProblemBuilder, EpigraphConeData), method names (buildEpigraphCone, computeCholeskyFactor), and member variables (L_times_2, epigraph_cone_size) follow project PascalCase/camelCase/snake_case_ conventions per CLAUDE.md. |
| Namespace organization | Pass | All components remain within `msd_sim` namespace. No new namespaces introduced. |
| File structure | Pass | All modifications are within existing files under `msd-sim/src/Physics/Constraints/ECOS/`. No new files needed for the implementation (design doc and math formulation are documentation-only additions). |
| Dependency direction | Pass | ECOSProblemBuilder depends only on ECOSData, FrictionConeSpec, and Eigen. No new external dependencies. ConstraintSolver depends on ECOSProblemBuilder (existing relationship). No circular dependencies. |

#### C++ Design Quality
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | Pass | ECOSData continues to manage ECOS workspace via unique_ptr with custom deleter. The equilibration constraint is correctly handled via custom move assignment and member declaration ordering. |
| Smart pointer appropriateness | Pass | ECOSWorkspacePtr (unique_ptr) for exclusive ECOS workspace ownership. No shared_ptr usage. Non-owning references used where appropriate. |
| Value/reference semantics | Pass | EpigraphConeData uses value semantics (Eigen matrices by value). ECOSData is move-only (correct for RAII wrapper). ECOSProblemBuilder::build() returns ECOSData by value (move). |
| Rule of 0/3/5 | Pass | EpigraphConeData uses Rule of Zero with `= default` constructor. ECOSData correctly implements Rule of Five with custom move operations (justified by ECOS equilibration constraint). |
| Const correctness | Pass | solveWithECOS() is const. Static private methods in ECOSProblemBuilder are appropriately stateless. Input parameters to build() are const references. |
| Exception safety | Pass | computeCholeskyFactor() provides strong guarantee (throws on failure, no state modified). build() validates inputs before mutation. ECOSData::setup() provides strong guarantee (workspace_ unchanged on failure). |

#### Feasibility
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | Pass | No new headers required. Existing Eigen/Dense and Eigen/Cholesky are sufficient. No circular header risks. |
| Template complexity | Pass | No templates used. All methods are concrete static functions or member functions. |
| Memory strategy | Pass | Problem size scales linearly with contacts (6C+1 variables, 6C+2 inequality rows, 3C equality rows). All matrices stored in CSC sparse format, minimizing memory for the typical block-sparse structure. |
| Thread safety | Pass | ECOSProblemBuilder methods are all static with no shared state (thread-safe). solveWithECOS() creates a local ECOSData per call (thread-safe). |
| Build integration | Pass | No new build targets or dependencies. Changes are within existing source files. |

#### Testability
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | Pass | ECOSProblemBuilder::build() is a pure function (static, no state). Can be tested in complete isolation with synthetic A, b, coneSpec inputs. Individual private methods can be tested indirectly through build(). |
| Mockable dependencies | Pass | FrictionConeSpec is a simple data class, trivially constructible for tests. Eigen matrices are value types. No external services or singletons. |
| Observable state | Pass | ECOSData exposes all problem matrices (G_, h_, c_, A_eq_, b_eq_, cone_sizes_) for direct inspection. ECOS workspace provides solution vector, exit flag, and residuals. Design includes comprehensive dimension assertions. |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | The math-formulation.md uses `L*lambda` and `L^{-T}*b` in the epigraph (Section 2), while the implementation correctly uses `L^T*lambda` and `L^{-1}*b`. Header documentation also reflects the incorrect `L*lambda` convention. Developers reading the math doc or header may implement the wrong thing. | Maintenance | High | Medium | Design correctly identifies the discrepancy. Header update plan in Section "Header Documentation Update Plan" covers all lines. Math-formulation.md correction is Open Question 2. | No |
| R2 | Misleading field name `L_inv_T_b_times_2` suggests `L^{-T}*b` but stores `L^{-1}*b`. Future developers could misinterpret what the field contains. | Maintenance | Medium | Low | Design recommends renaming to `d_times_2` (Open Question 1, Option A). Field is private to ECOSProblemBuilder, limiting blast radius. | No |
| R3 | The 8 failing tests (4 ECOS integration in 0035d7, 4 physics in 0035d8) may not all resolve from documentation-only changes. The design concludes the SOCP formulation is mathematically correct but does not provide definitive evidence that ECOS_solve() returns ECOS_OPTIMAL for the test inputs. | Technical | Medium | High | Design recommends a diagnostic run (Open Question 3 / "Prototype Required" section) to print ECOS workspace state for stick-regime test case. This is the right approach. | Yes |
| R4 | Acceptance Criterion AC8 in the ticket states "No equality constraints in ECOS problem (A_eq removed from formulation)" but the actual implementation HAS 3C equality constraints. The ticket AC is stale relative to the implementation. | Maintenance | High | Low | The ticket was written before the auxiliary-variable approach was implemented. AC8 should be updated to "3C equality constraints encoding y = L^T*lambda". No functional risk since implementation is correct. | No |
| R5 | For mu=0, the friction cone `||[lambda_t1, lambda_t2]|| <= 0` implies `lambda_n >= 0` only vacuously. Normal force positivity is implicitly encoded via the cone constraint `s_0 = mu*lambda_n >= 0`. When mu=0, s_0=0 regardless of lambda_n sign. ECOS could return lambda_n < 0 in degenerate cases. | Technical | Low | Medium | The math-formulation.md Section "Frictionless Contact" correctly notes this and recommends routing mu=0 contacts to the ASM solver. The ConstraintSolver already dispatches to ASM when no FrictionConstraint instances are detected, so pure mu=0 scenarios bypass ECOS. For mixed mu>0/mu=0 contacts in the same solve, the epigraph QP objective should drive lambda_n >= 0 since negative lambda_n increases the quadratic cost. | No |

### Prototype Guidance

#### Prototype P1: ECOS Solver Diagnostic Run

**Risk addressed**: R3
**Question to answer**: Does `ECOS_solve()` return `ECOS_OPTIMAL` for the stick-regime test case (A=I, b=[10,1,1], mu=0.5) with the auxiliary-variable formulation, and does the extracted lambda match the expected solution [10, 1, 1]?

**Success criteria**:
- ECOS exit flag == ECOS_OPTIMAL (0)
- Extracted lambda (first 3C elements) matches expected [10.0, 1.0, 1.0] within tolerance 1e-4
- Primal residual < 1e-6
- Dual residual < 1e-6
- Duality gap < 1e-6

**Prototype approach**:
```
Location: prototypes/0035d1_ecos_socp_reformulation/p1_solver_diagnostic/
Type: Catch2 test harness (can be a standalone test or added to existing test file)

Steps:
1. Construct A = I_3x3, b = [10, 1, 1], FrictionConeSpec with mu=0.5
2. Call ECOSProblemBuilder::build(A, b, coneSpec)
3. Call ECOSData::setup()
4. Print all problem matrices: G (data, row_indices, col_ptrs), h, c, A_eq, b_eq, cone_sizes
5. Call ECOS_solve(workspace)
6. Print: exit_flag, solution vector x (all 6C+1 elements), pres, dres, gap
7. Verify lambda = x[0:3] matches [10, 1, 1]
8. Repeat with b = [10, 5, 5] (should saturate friction cone)
9. Verify lambda_n > 0 and ||[lambda_t1, lambda_t2]|| == mu * lambda_n
```

**Time box**: 1 hour

**If prototype fails**:
- If ECOS returns non-optimal exit code: Inspect problem matrices for correctness against the C=1 appendix in the design document. Check for CSC format errors (column pointer off-by-one, missing entries, wrong signs).
- If ECOS returns optimal but wrong lambda: The epigraph formulation may need the factor-of-2 scaling corrected. Verify that minimizing t produces the same argmin as minimizing f(lambda).
- If ECOS returns optimal and correct lambda but tests still fail: The issue is downstream in the integration pipeline (force extraction, body application), not in the SOCP formulation. Investigate 0035d7/0035d8 separately.

### Notes for Implementer

#### Note 1: Stale Ticket Acceptance Criterion
Ticket 0035d1 Acceptance Criterion AC8 ("No equality constraints in ECOS problem") is stale. The auxiliary-variable formulation requires 3C equality constraints (`L^T*lambda - y = 0`). This is correct behavior. AC8 should be updated to reflect the actual implementation before the ticket transitions to "Implementation Complete."

#### Note 2: Mathematical Correctness Verification
The design's mathematical correctness proof in the "Mathematical Correctness Proof" section has been verified by this review. The key identity chain is:

1. `A = L*L^T` (Cholesky)
2. Let `u = L^T*lambda`, then `||u||^2 = lambda^T*A*lambda` (correct)
3. `b^T*lambda = (L^{-1}*b)^T * (L^T*lambda) = d^T * u` where `d = L^{-1}*b` (correct: `b^T*lambda = b^T*(L^T)^{-1}*L^T*lambda = (L^{-1}*b)^T*u`)
4. Completing the square: `(1/2)||u||^2 - d^T*u = (1/2)||u - d||^2 - (1/2)||d||^2` (correct)
5. Therefore `f(lambda) = (1/2)||L^T*lambda - d||^2 - const` (correct)
6. Epigraph: `||(2*(y-d), t-1)|| <= t+1` encodes `t >= ||y-d||^2` (correct: expanding gives `4||y-d||^2 + t^2 - 2t + 1 <= t^2 + 2t + 1`, simplifying to `||y-d||^2 <= t`)
7. With `y = L^T*lambda` (equality constraint), this gives `t >= ||L^T*lambda - d||^2 = 2*f(lambda) + const` (correct)
8. Minimizing `t` minimizes `f(lambda)` since the scaling factor does not affect the argmin (correct)

The `L^T*lambda` vs `L*lambda` distinction is critical and the implementation is correct. The design clearly identifies the transpositional error in math-formulation.md.

#### Note 3: G Matrix Sign Convention
The design correctly documents the ECOS convention `s = h - G*x` with `s` in the cone. The G matrix uses negative entries (`-1`, `-2`, `-mu`) so that:
- `s_0 = h_0 - G_0*x = 1 - (-1)*t = 1 + t` (non-negative for t >= -1, which is always true at optimum)
- `s_i = h_i - G_i*x = -2*d_i - (-2)*y_i = 2*(y_i - d_i)` (correct norm components)
- `s_{3C+1} = h_{3C+1} - G_{3C+1}*x = -1 - (-1)*t = t - 1` (correct)

The cone constraint `||s_{1..3C+1}|| <= s_0` gives `||(2*(y-d), t-1)|| <= t+1`, which matches the epigraph formulation.

For friction cones: `s = h - G*x = 0 - (-mu, -1, -1) * (lambda_n, lambda_t1, lambda_t2) = (mu*lambda_n, lambda_t1, lambda_t2)`. The cone `||s_{1:2}|| <= s_0` gives `||[lambda_t1, lambda_t2]|| <= mu*lambda_n`. Correct.

#### Note 4: CSC Matrix Column Pointer Counts
Verified by manual trace for C=1:
- G matrix: 7 columns need 8 col_ptrs. Code produces 3 (lambda) + 3 (y) + 1 (t) + 1 (final) = 8. Correct.
- A_eq matrix: 7 columns need 8 col_ptrs. Code produces 3 (lambda) + 3 (y) + 1 (t) + 1 (final) = 8. Correct.

#### Note 5: h Vector Sign Convention
The design documents `h[1..3C] = -2*d` where `d = L^{-1}*b`. The implementation at line 208 of ECOSProblemBuilder.cpp uses `-epigraphData.L_inv_T_b_times_2(i)` where `L_inv_T_b_times_2` stores `2*d`. So `h[i+1] = -(2*d_i) = -2*d_i`. This is correct and consistent with the sign convention needed for `s_i = -2*d_i - (-2)*y_i = 2*(y_i - d_i)`.

### Summary

The design is thorough, well-structured, and accurately documents the as-implemented auxiliary-variable SOCP formulation. The mathematical correctness proof is verified. The design correctly identifies all stale documentation and proposes concrete fixes. The main outstanding uncertainty is whether the ECOS solver actually returns correct results for the test inputs (Risk R3), which requires the recommended diagnostic prototype (P1, estimated 1 hour). The four notes above highlight areas requiring attention during implementation: updating stale ticket ACs, the verified math chain, sign convention correctness, and CSC pointer counts. The design is ready for human review and, pending prototype validation, for implementation of the documentation updates and investigation of test failures.
