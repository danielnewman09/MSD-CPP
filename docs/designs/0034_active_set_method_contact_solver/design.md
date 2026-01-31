# Design: Active Set Method Contact Solver

## Summary

This design replaces the Projected Gauss-Seidel (PGS) iterative solver within `ConstraintSolver::solveWithContacts()` with an Active Set Method (ASM) for solving the contact constraint Linear Complementarity Problem (LCP). The ASM partitions contacts into active (compressive, lambda > 0) and inactive (separating, lambda = 0) sets, solving an equality-constrained subproblem at each step via direct LLT decomposition, and iterates by adding/removing constraints until all Karush-Kuhn-Tucker (KKT) conditions are satisfied. This provides finite convergence to the exact solution, eliminating PGS's sensitivity to iteration count, constraint ordering, and high mass ratios.

**Ticket**: [0034_active_set_method_contact_solver](../../../tickets/0034_active_set_method_contact_solver.md)
**Related**: [0032_contact_constraint_refactor](../../../tickets/0032_contact_constraint_refactor.md), [0033_constraint_solver_contact_tests](../../../tickets/0033_constraint_solver_contact_tests.md)

---

## Architecture Changes

### PlantUML Diagram
See: `./0034_active_set_method_contact_solver.puml`

### Overview of Change Scope

This is a **targeted internal replacement** within `ConstraintSolver`. The public interface (`solveWithContacts()`, `MultiBodySolveResult`, `BodyForces`) is unchanged. The Jacobian assembly, effective mass matrix assembly, RHS assembly, and force extraction pipelines are all unchanged. Only the solver kernel -- the method that takes the assembled `A` matrix and `b` vector and produces the lambda vector -- is replaced.

| Category | Count | Description |
|----------|-------|-------------|
| New components | 0 | No new classes or files |
| Modified components | 1 | `ConstraintSolver` (internal method replacement) |
| Unchanged components | 8+ | All other constraint, physics, and environment components |
| New test files | 1 | `ConstraintSolverASMTest.cpp` |

---

### Modified Components

#### ConstraintSolver

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`, `ConstraintSolver.cpp`
- **Changes required**:
  1. Replace `PGSResult` private struct with `ActiveSetResult` private struct
  2. Replace `solvePGS()` private method with `solveActiveSet()` private method
  3. Rename `max_iterations_` to `max_safety_iterations_` with updated default value computation
  4. Repurpose `convergence_tolerance_` for inactive constraint violation check
  5. Update `setMaxIterations()` doc comment (semantic change from iteration budget to safety cap)
  6. Update `setConvergenceTolerance()` doc comment (semantic change from PGS convergence to violation threshold)
  7. Update `solveWithContacts()` to call `solveActiveSet()` instead of `solvePGS()` and pass contact count for safety cap computation
- **Backward compatibility**: The public interface (`solveWithContacts()` signature, `MultiBodySolveResult`, `BodyForces`) is **completely unchanged**. The `solve()` bilateral method is **completely unchanged**. Callers of `setMaxIterations()` and `setConvergenceTolerance()` will continue to work but with updated semantics.

##### Replaced Struct: PGSResult -> ActiveSetResult

**Current** (removed):
```cpp
struct PGSResult
{
    Eigen::VectorXd lambda;
    bool converged{false};
    int iterations{0};
};
```

**New** (replacement):
```cpp
/**
 * @brief Result of Active Set Method solve
 *
 * Contains the solution lambda vector, convergence status, iteration count
 * (number of active set changes), and the size of the final active set.
 */
struct ActiveSetResult
{
    Eigen::VectorXd lambda;         // Lagrange multipliers (all >= 0)
    bool converged{false};          // True if all KKT conditions satisfied
    int iterations{0};              // Number of active set changes performed
    int active_set_size{0};         // Number of contacts in final active set

    ActiveSetResult() = default;
};
```

##### Replaced Method: solvePGS -> solveActiveSet

**Current** (removed):
```cpp
PGSResult solvePGS(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) const;
```

**New** (replacement):
```cpp
/**
 * @brief Solve contact LCP using Active Set Method
 *
 * Partitions contacts into active (compressive) and inactive (separating)
 * sets, solving an equality subproblem at each iteration via LLT decomposition
 * until all KKT conditions are satisfied.
 *
 * Algorithm:
 * 1. Initialize working set W = {0, 1, ..., C-1} (all contacts active)
 * 2. Solve equality subproblem: A_W * lambda_W = b_W via LLT
 * 3. If any lambda_W[i] < 0: remove most negative from W (Bland's rule for ties)
 * 4. If all lambda_W >= 0: check inactive constraints for violation
 * 5. If violated inactive constraint found: add most violated to W (Bland's rule)
 * 6. If no violations: KKT conditions satisfied, return exact solution
 *
 * Convergence: Finite, typically <= C iterations for non-degenerate systems.
 * Safety cap: 2*C iterations to prevent cycling in degenerate cases.
 *
 * @param A Effective mass matrix (C x C), symmetric positive semi-definite
 * @param b RHS vector (C x 1) with restitution and Baumgarte terms
 * @param numContacts Number of contacts (used for safety iteration cap = 2*C).
 *        Note: numContacts == b.size() by construction (one constraint row per contact).
 *        Passed explicitly rather than derived from b.size() to document intent and
 *        decouple the safety cap computation from the Eigen vector internals.
 * @return ActiveSetResult with lambda vector, convergence info, and active set size
 *
 * @ticket 0034_active_set_method_contact_solver
 */
ActiveSetResult solveActiveSet(const Eigen::MatrixXd& A,
                               const Eigen::VectorXd& b,
                               int numContacts) const;
```

##### Renamed Member: max_iterations_ -> max_safety_iterations_

**Current**:
```cpp
int max_iterations_{10};
```

**New**:
```cpp
int max_safety_iterations_{100};  // Safety cap; actual limit computed as min(2*C, max_safety_iterations_)
```

The default value of 100 is a global upper bound. The per-solve effective limit is `std::min(2 * numContacts, max_safety_iterations_)`. For typical contact counts (1-10), the effective limit is 2-20, well below the global cap. The global cap protects against pathological inputs (e.g., a caller passing 10000 contacts).

##### Updated Member Semantics: convergence_tolerance_

**Current semantics**: PGS convergence check (`maxDelta < convergence_tolerance_`).

**New semantics**: Inactive constraint violation threshold. A constraint `i` not in the active set is considered violated if `w_i = (A * lambda - b)_i < -convergence_tolerance_`. The tolerance prevents numerical noise from triggering spurious active set changes.

```cpp
double convergence_tolerance_{1e-6};  // Updated default: tighter for direct solve
```

The default is changed from `1e-4` to `1e-6` because the ASM uses a direct LLT solve (not iterative), so tighter tolerance is achievable without additional cost. This tighter default improves solution accuracy.

##### Updated solveWithContacts() Internal Call

The `solveWithContacts()` method body changes only at the solver call site:

**Current** (line 279 of ConstraintSolver.cpp):
```cpp
auto [lambda, converged, iterations] = solvePGS(A, b);
```

**New**:
```cpp
const int numContacts = static_cast<int>(contactConstraints.size());
auto [lambda, converged, iterations, activeSetSize] = solveActiveSet(A, b, numContacts);
```

Uses structured bindings consistent with the existing codebase pattern for `solvePGS`. The move semantics of structured bindings avoid copying the `Eigen::VectorXd lambda`. All downstream code (force extraction, result assembly, residual computation) remains identical.

##### Updated Public Method Documentation

```cpp
/**
 * @brief Set maximum safety iteration cap for Active Set Method
 *
 * The effective iteration limit per solve is min(2*C, maxIter) where
 * C is the number of contacts. This safety cap prevents infinite loops
 * for degenerate inputs. For typical use, the default (100) should not
 * need adjustment.
 *
 * @param maxIter Maximum safety iterations (default: 100)
 * @ticket 0034_active_set_method_contact_solver
 */
void setMaxIterations(int maxIter) { max_safety_iterations_ = maxIter; }

/**
 * @brief Set constraint violation tolerance for Active Set Method
 *
 * Inactive constraints with violation magnitude below this tolerance
 * are considered satisfied. Tighter tolerance improves accuracy but
 * may cause extra iterations for nearly-active constraints.
 *
 * @param tol Violation tolerance (default: 1e-6)
 * @ticket 0034_active_set_method_contact_solver
 */
void setConvergenceTolerance(double tol) { convergence_tolerance_ = tol; }
```

---

### Unchanged Components

The following components are **completely unchanged** by this design:

| Component | File(s) | Reason |
|-----------|---------|--------|
| `solveWithContacts()` public signature | `ConstraintSolver.hpp` | Public API stability (FR-4) |
| `MultiBodySolveResult` struct | `ConstraintSolver.hpp` | Public return type, consumed by WorldModel |
| `BodyForces` struct | `ConstraintSolver.hpp` | Public return type |
| `solve()` bilateral method | `ConstraintSolver.hpp/.cpp` | Separate single-body solver, no contact involvement |
| `assembleContactJacobians()` | `ConstraintSolver.cpp` | Input assembly, solver-agnostic |
| `assembleContactEffectiveMass()` | `ConstraintSolver.cpp` | Matrix A assembly, solver-agnostic |
| `assembleContactRHS()` | `ConstraintSolver.cpp` | Vector b assembly, solver-agnostic |
| `extractContactBodyForces()` | `ConstraintSolver.cpp` | Output extraction from lambda, solver-agnostic |
| `kRegularizationEpsilon` | `ConstraintSolver.hpp` | Diagonal regularization preserved for numerical stability |
| `TwoBodyConstraint` | `TwoBodyConstraint.hpp/.cpp` | Abstract interface, no solver dependency |
| `ContactConstraint` | `ContactConstraint.hpp/.cpp` | Concrete constraint, no solver dependency |
| `ContactConstraintFactory` | `ContactConstraintFactory.hpp/.cpp` | Factory, no solver dependency |
| `Constraint` hierarchy | Various | Base constraint classes, no solver dependency |
| `WorldModel` | `WorldModel.hpp/.cpp` | Calls `solveWithContacts()` only, interface unchanged |
| `SemiImplicitEulerIntegrator` | `SemiImplicitEulerIntegrator.hpp/.cpp` | Uses bilateral `solve()`, no contact involvement |
| All bilateral constraints | Various | `solve()` method path unchanged |
| All existing test files | Various | Expected to pass without modification |

### Semantic Change: MultiBodySolveResult::residual

The existing code computes `result.residual = (A * lambda - b).norm()` after solving. Under PGS, this norm converges toward zero as PGS iterates — it serves as a convergence metric. Under ASM, the residual `w = A*lambda - b` has `w_i = 0` for active contacts (by construction of the equality subproblem) and `w_i >= 0` for inactive contacts (separating). The norm will be non-zero whenever any contact is separating, which is physically correct (not an error).

The `residual` field retains its type (`double`) and computation (`(A*lambda - b).norm()`), but its interpretation changes from "solver convergence metric" to "dual residual norm (complementarity measure)". No existing tests or downstream code inspect this field, so this is a documentation-only semantic change. The field remains useful as a diagnostic: a zero residual means all contacts are active; a non-zero residual indicates the magnitude of the separating constraint residuals.

---

## Algorithm Description

### Active Set Method for Contact LCP

The contact constraint problem after assembly is a convex Quadratic Program (QP) with non-negativity constraints:

```
minimize    (1/2) * lambda^T * A * lambda - b^T * lambda
subject to  lambda >= 0
```

Where `A = J * M^{-1} * J^T` is the symmetric positive semi-definite effective mass matrix (C x C), `b` is the RHS vector encoding restitution and Baumgarte terms, and `lambda` is the vector of contact impulse magnitudes.

This is equivalent to the LCP: find lambda such that `w = A*lambda - b >= 0`, `lambda >= 0`, `lambda^T * w = 0`.

### Pseudocode

```
ActiveSetSolve(A, b, C):
  // Initialize: assume all contacts active (compressive)
  W = {0, 1, ..., C-1}     // Working set (active constraint indices)
  lambda = zeros(C)          // Initialize lambda vector
  max_iter = min(2*C, max_safety_iterations_)

  for iter = 1 to max_iter:

    // === Step 1: Solve equality subproblem for active set ===
    if W is empty:
      lambda = zeros(C)
    else:
      A_W = A[W, W]           // Extract rows/columns for active indices
      b_W = b[W]              // Extract RHS for active indices
      lambda_W = LLT_solve(A_W, b_W)  // Direct Cholesky solve

    // Assign solution
    for i in W: lambda[i] = lambda_W[index_of(i)]
    for i not in W: lambda[i] = 0

    // === Step 2: Check primal feasibility (lambda >= 0 for active set) ===
    min_lambda = +infinity
    min_index = -1
    for i in W:
      if lambda[i] < min_lambda:
        min_lambda = lambda[i]
        min_index = i
      else if lambda[i] == min_lambda and i < min_index:
        min_index = i          // Bland's rule: smallest index for ties

    if min_lambda < 0:
      // Remove most negative lambda from active set
      W = W \ {min_index}
      continue

    // === Step 3: Check dual feasibility (w >= 0 for inactive set) ===
    w = A * lambda - b         // Constraint residual for all contacts
    max_violation = 0
    max_violation_index = -1
    for i not in W:
      if w[i] < -convergence_tolerance_:
        if w[i] < max_violation or
           (w[i] == max_violation and i < max_violation_index):
          max_violation = w[i]
          max_violation_index = i  // Bland's rule: smallest index for ties

    if max_violation_index == -1:
      // All KKT conditions satisfied
      return ActiveSetResult{lambda, converged=true, iter, |W|}

    // === Step 4: Add most violated constraint to active set ===
    W = W + {max_violation_index}

  // Safety cap reached
  return ActiveSetResult{lambda, converged=false, max_iter, |W|}
```

### Initialization Strategy

The algorithm initializes with all contacts in the active set (W = {0, 1, ..., C-1}). This is optimal for the common case of resting contacts where most or all contacts are compressive. For separating contacts, the algorithm removes them from the active set within the first few iterations (one removal per separating contact).

**Rationale**: Starting with a full active set means the first iteration solves the full unconstrained system. If all lambdas are non-negative, the algorithm terminates in a single iteration (optimal for resting contacts). Starting with an empty active set would require C iterations to add all active contacts one-by-one.

### Anti-Cycling: Bland's Rule

To prevent cycling (the same constraint repeatedly added and removed), the algorithm uses Bland's rule: when multiple constraints are tied (equal lambda values for removal, or equal violation magnitudes for addition), the constraint with the **smallest index** is selected. This guarantees finite termination for all non-degenerate inputs.

The cycling risk is inherently low for contact LCPs because the effective mass matrix A is symmetric positive semi-definite with regularization (positive definite), which makes the QP strictly convex. Cycling can only occur in degenerate cases where multiple constraints have identical geometry.

### Subproblem Extraction

Each iteration requires extracting the active submatrix `A_W` and subvector `b_W` from the full system. This is implemented using Eigen indexing:

```cpp
// Extract active indices into dense subproblem
Eigen::MatrixXd A_W(activeSize, activeSize);
Eigen::VectorXd b_W(activeSize);

for (int i = 0; i < activeSize; ++i)
{
    b_W(i) = b(activeIndices[i]);
    for (int j = 0; j < activeSize; ++j)
    {
        A_W(i, j) = A(activeIndices[i], activeIndices[j]);
    }
}
```

This extraction is O(|W|^2) per iteration. For typical contact counts (|W| < 20), this is negligible.

### LLT Solve for Subproblem

The subproblem `A_W * lambda_W = b_W` is solved using Eigen's `LLT` Cholesky decomposition, the same decomposition already used by the bilateral `solve()` method. The regularization epsilon (`kRegularizationEpsilon = 1e-8`) on the diagonal of the full A matrix ensures that A_W is positive definite for any subset of active contacts, making LLT decomposition valid.

If LLT decomposition fails (e.g., due to a truly singular subproblem), the algorithm handles this by returning `converged = false` with the current best lambda estimate. This is a degenerate edge case that should not occur with proper regularization.

---

## Mathematical Foundation

### KKT Conditions for Contact LCP

The Active Set Method converges when all Karush-Kuhn-Tucker conditions hold simultaneously:

1. **Primal feasibility**: `lambda_i >= 0` for all i (contacts can only push, not pull)
2. **Dual feasibility**: `w_i = (A * lambda - b)_i >= 0` for all i (no constraint violation)
3. **Complementarity**: `lambda_i * w_i = 0` for all i (either force is zero or constraint is exactly satisfied)

The active set partition enforces complementarity by construction:
- Active set (W): `lambda_i > 0` and `w_i = 0` (compressive contacts, equality subproblem forces w=0)
- Inactive set (~W): `lambda_i = 0` and `w_i >= 0` (separating contacts, checked in Step 3)

### Convergence Proof Sketch

For a strictly convex QP (which is guaranteed by the regularized positive definite A matrix):

1. **Finite working sets**: There are at most 2^C possible working sets (subsets of {0, ..., C-1}).
2. **Strict decrease**: Each iteration either decreases the objective function or changes the working set. Bland's rule prevents revisiting the same working set.
3. **Termination**: Since the number of working sets is finite and no working set is visited twice, the algorithm terminates in at most 2^C iterations.
4. **Practical bound**: For non-degenerate problems, convergence typically occurs in O(C) iterations because each iteration resolves one constraint's status (active vs. inactive).

**Reference**: Nocedal & Wright, "Numerical Optimization", Chapter 16, Theorem 16.4.

### QP Equivalence to LCP

The contact LCP `w = A*lambda - b >= 0, lambda >= 0, lambda^T * w = 0` is the KKT system of the QP:

```
minimize    (1/2) * lambda^T * A * lambda - b^T * lambda
subject to  lambda >= 0
```

Since A is symmetric positive definite (after regularization), the QP is strictly convex and has a unique global minimum. The ASM finds this minimum exactly.

---

## Implementation Details

### Data Structures

**Active set representation**: The active set W is represented as a `std::vector<int>` containing the sorted indices of active contacts. This provides:
- O(1) size query via `size()`
- O(|W|) iteration for subproblem extraction
- O(|W|) insertion/removal (acceptable for |W| < 50)

Alternative considered: `std::vector<bool>` with |W| = number of true entries. Rejected because extracting active indices for subproblem construction requires a full scan. The sorted index vector is more direct.

**Subproblem matrices**: Dense `Eigen::MatrixXd` and `Eigen::VectorXd` allocated locally per iteration. For typical active set sizes (< 20), the allocation cost is negligible. Stack allocation via `Eigen::Matrix<double, Dynamic, Dynamic, 0, 20, 20>` could be used as a future optimization for compile-time bounded sizes.

### Safety Iteration Cap

The effective safety cap per solve is:

```cpp
const int effectiveMaxIter = std::min(2 * numContacts, max_safety_iterations_);
```

- `2 * numContacts`: Theoretical upper bound for non-degenerate problems. The factor of 2 provides margin for add/remove oscillation near the boundary.
- `max_safety_iterations_`: Global upper bound (default: 100) to protect against pathological inputs.

For typical contact counts:
- 1 contact: cap = 2
- 5 contacts: cap = 10
- 10 contacts: cap = 20
- 50 contacts: cap = 100 (global cap)

### Empty Active Set Handling

When the active set becomes empty (all contacts determined to be separating):
- `lambda` is set to zero vector
- The dual feasibility check (Step 3) determines whether any inactive constraint is violated
- If no violations, the zero solution is exact (all contacts truly separating)
- If a violation exists, the most violated constraint is added and the algorithm continues

### LLT Decomposition Failure Handling

If `Eigen::LLT` reports `info() != Eigen::Success` for a subproblem:

```cpp
Eigen::LLT<Eigen::MatrixXd> llt{A_W};
if (llt.info() != Eigen::Success)
{
    // Subproblem is singular despite regularization (extremely rare)
    // Return current best estimate with converged = false
    return ActiveSetResult{lambda, false, iter, static_cast<int>(activeIndices.size())};
}
```

This should not occur in practice because `kRegularizationEpsilon` (1e-8) on the diagonal ensures positive definiteness. The check is a defensive measure.

---

## Performance Analysis

### Complexity Comparison

| Metric | PGS (current) | Active Set Method |
|--------|---------------|-------------------|
| Per-iteration cost | O(C^2) | O(\|W\|^3) for LLT solve + O(\|W\|^2) extraction |
| Total iterations | Fixed at `max_iterations_` (10 default) | Variable, typically <= C |
| Worst case total | O(10 * C^2) = O(C^2) | O(2^C * C^3) (theoretical, never observed) |
| Practical worst case | O(10 * C^2) | O(C * C^3) = O(C^4) |
| Solution quality | Approximate | Exact (to machine precision) |
| Mass ratio sensitivity | Degrades above 100:1 | Robust to 1e12:1 (LLT condition number limit) |
| Order dependence | Yes (Gauss-Seidel sequential updates) | No (deterministic for given A, b) |

### Practical Performance for Typical Contact Counts

| Contacts (C) | PGS (10 iter, O(C^2)) | ASM (C iter, O(C^3)) | ASM/PGS Ratio |
|---|---|---|---|
| 1 | ~10 ops | ~1 op | 0.1x (faster) |
| 2 | ~40 ops | ~16 ops | 0.4x (faster) |
| 5 | ~250 ops | ~625 ops | 2.5x (slower per op, but exact) |
| 10 | ~1000 ops | ~10000 ops | 10x (slower per op, but exact) |
| 20 | ~4000 ops | ~160000 ops | 40x |

**Note**: The ASM "operations" are dominated by dense LLT solve which has excellent cache behavior and BLAS optimization in Eigen. The PGS "operations" are scalar updates with poor vectorization. In wall-clock time, the gap is narrower than the operation count suggests.

**For this project's typical use case (1-10 contacts)**, the ASM provides exact solutions at comparable or better wall-clock time than PGS. For large contact counts (> 20), the cubic per-iteration cost dominates, but this project does not target such scenarios.

### Memory Overhead

Per `solveActiveSet()` call:
- `std::vector<int> activeIndices`: C entries maximum (24 bytes for C=3)
- `Eigen::MatrixXd A_W`: |W|^2 doubles (800 bytes for |W|=10)
- `Eigen::VectorXd b_W`: |W| doubles (80 bytes for |W|=10)
- `Eigen::VectorXd lambda_W`: |W| doubles (80 bytes for |W|=10)
- `Eigen::VectorXd w`: C doubles (80 bytes for C=10)

Total: < 2 KB for C=10. All stack-allocated or locally scoped. No persistent heap allocation.

---

## Error Handling

### Degenerate Inputs

| Condition | Behavior | Justification |
|-----------|----------|---------------|
| Zero contacts (C=0) | Handled by `solveWithContacts()` early return before calling `solveActiveSet()` | Existing behavior preserved |
| Both bodies static (inverseMass=0 for both) | A matrix has zeros on diagonal, regularization provides small positive values. ASM may compute non-zero lambda but body forces have no effect (F * inverseMass = 0). Returns converged. | Matches existing PGS behavior (see `BothBodiesStatic_AllLambdasZero_0033` test) |
| Redundant contacts (identical geometry) | A_W becomes ill-conditioned. Regularization epsilon prevents singularity. LLT succeeds. Lambda distributes among redundant contacts. | Matches ticket requirement (NFR-6) |
| Safety cap reached | Returns `converged = false` with best current lambda estimate | Matches ticket requirement (FR-6) |
| LLT failure on subproblem | Returns `converged = false` with current lambda | Defensive; should not occur with regularization |
| Negative b entries (all contacts separating) | All lambdas clamped to zero. Algorithm terminates in 1-C iterations as contacts are removed from active set. | Correct behavior for separating contacts |

### Numerical Considerations

- **Regularization**: The existing `kRegularizationEpsilon = 1e-8` added to the diagonal of A (in `assembleContactEffectiveMass()`) ensures positive definiteness of all subproblems. This is unchanged.
- **Tolerance**: The `convergence_tolerance_` (default 1e-6) prevents numerical noise in the residual vector `w` from triggering spurious active set changes. Values below machine epsilon (~1e-16) would cause infinite oscillation.
- **Floating-point comparison**: The algorithm uses strict inequality (`lambda[i] < 0`, `w[i] < -tolerance`) rather than equality tests, avoiding floating-point equality pitfalls.

---

## Interface Changes Summary

### Header File Changes (ConstraintSolver.hpp)

| Line(s) | Current | New | Type |
|---------|---------|-----|------|
| 133 | `int iterations{0};  // PGS iterations used` | `int iterations{0};  // Active set changes performed` | Comment update |
| 139 | `Solve contact ... using Projected Gauss-Seidel (PGS)` | `Solve contact ... using Active Set Method (ASM)` | Doxygen update |
| 243-248 | `struct PGSResult { ... }` | `struct ActiveSetResult { ... int active_set_size{0}; }` | Struct replacement |
| 290-295 | `PGSResult solvePGS(...)` | `ActiveSetResult solveActiveSet(..., int numContacts)` | Method replacement |
| 311-312 | `// PGS configuration` | `// Active Set Method configuration` | Comment update |
| 312 | `int max_iterations_{10}` | `int max_safety_iterations_{100}` | Rename + default change |
| 313 | `double convergence_tolerance_{1e-4}` | `double convergence_tolerance_{1e-6}` | Default change |
| 175 | `void setMaxIterations(int maxIter) { max_iterations_ = maxIter; }` | `void setMaxIterations(int maxIter) { max_safety_iterations_ = maxIter; }` | Internal rename |

### Source File Changes (ConstraintSolver.cpp)

| Section | Current | New | Type |
|---------|---------|-----|------|
| Line 279 | `auto [lambda, converged, iterations] = solvePGS(A, b);` | `auto [lambda, converged, iterations, activeSetSize] = solveActiveSet(A, b, numContacts);` | Call site update |
| Lines 437-475 | `ConstraintSolver::PGSResult ConstraintSolver::solvePGS(...)` | `ConstraintSolver::ActiveSetResult ConstraintSolver::solveActiveSet(...)` | Full method replacement |

---

## Integration Points

| Modified Code | Existing Component | Integration Type | Notes |
|---|---|---|---|
| `solveActiveSet()` | `assembleContactEffectiveMass()` | Consumes A matrix | Same matrix, different solver |
| `solveActiveSet()` | `assembleContactRHS()` | Consumes b vector | Same vector, different solver |
| `solveWithContacts()` | `solveActiveSet()` | Internal call (was `solvePGS()`) | Same call pattern |
| `solveWithContacts()` | `extractContactBodyForces()` | Passes lambda | Same lambda format |
| `MultiBodySolveResult` | WorldModel | Return type | Unchanged struct, `iterations` field now counts ASM iterations |

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|-----------------|
| `test/Physics/Constraints/ConstraintSolverContactTest.cpp` | 23 of 24 tests | **Behavioral equivalence expected for 23 tests** | 1 test (`MaxIterationsReached`) requires scenario modification; see Notes on Existing Test Compatibility below |
| `test/Physics/Constraints/ConstraintTest.cpp` | All 30 tests | **Unchanged** | Bilateral solver untouched |
| `test/Physics/Integration/QuaternionPhysicsTest.cpp` | All 16 tests | **Unchanged** | Integration uses bilateral solver |

#### Notes on Existing Test Compatibility

The existing `ConstraintSolverContactTest` tests were designed with PGS-specific tolerances (e.g., `1e-6` for lambda checks, `1e-4` for zero-penetration checks). The ASM produces exact solutions, so all existing tolerances are satisfied with significant margin. Specific considerations:

- **`MaxIterationsReached_ReportsNotConverged_0033`**: **KNOWN BREAKAGE — requires test modification.** This test sets `maxIterations=1` and expects `converged=false`. Under ASM, the 2-contact resting scenario (zero relative velocity, positive Baumgarte bias) is solved exactly in 1 iteration: the full active set W={0,1} produces positive lambdas via LLT, and since the active set equals the full set, there are no inactive constraints to check dual feasibility. All KKT conditions are satisfied, so ASM returns `converged=true` in 1 iteration, causing `EXPECT_FALSE(result.converged)` to fail.

  **Mitigation**: Modify this test to construct a scenario that genuinely requires multiple active set changes. Replace the two compressive-only contacts with a mixed scenario: one contact with positive Baumgarte bias (compressive) and one with a separating relative velocity that initially appears active but becomes inactive after the first solve (negative lambda). This forces at least 2 active set iterations. The test should then set `max_safety_iterations_=1` and verify that ASM returns `converged=false` because the safety cap prevents the required second iteration. This test file (`ConstraintSolverContactTest.cpp`) is added to the "Modified Files" table below.

- **`SetMaxIterations_Respected_0033`**: Sets `maxIterations=5`. With ASM and C=1, effective cap = `min(2, 5) = 2`. Single contact should converge in 1 iteration. Test checks `EXPECT_LE(result.iterations, 5)`, which passes.

- **`SetConvergenceTolerance_EarlyExit_0033`**: Compares tight (1e-8) vs loose (1e-2) tolerance. With ASM, tolerance affects the violation check threshold, not convergence speed. Both solvers should converge in the same number of iterations for a single contact (1 iteration). The test checks `EXPECT_GE(resultTight.iterations, resultLoose.iterations)`, which passes (equal iterations).

- **`SingleContact_Converges_0033`**: Checks `EXPECT_GT(result.iterations, 0)` and `EXPECT_LT(result.iterations, 10)`. ASM solves a single contact in 1 iteration: full active set -> solve -> all KKT satisfied. `iterations=1` satisfies both bounds.

### New Tests Required

#### Unit Tests (ConstraintSolverASMTest.cpp)

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| ActiveSetResult | `ActiveSetResult_DefaultConstruction_Zeroed` | Default constructor initializes fields correctly |
| solveActiveSet | `SingleContact_ExactSolution_MatchesAnalytical` | Single contact lambda matches `b/A` within 1e-12 |
| solveActiveSet | `OrderIndependence_ShuffledContacts_IdenticalLambdas` | Two contact orderings produce identical lambda within 1e-12 |
| solveActiveSet | `HighMassRatio_1e6_Converges` | Mass ratio 1e6:1 converges successfully |
| solveActiveSet | `AllSeparating_EmptyActiveSet` | All contacts separating -> all lambda = 0, active_set_size = 0 |
| solveActiveSet | `AllCompressive_FullActiveSet` | All contacts compressive -> all lambda > 0, active_set_size = C |
| solveActiveSet | `MixedActiveInactive_CorrectPartition` | Some active, some inactive -- verify correct partition |
| solveActiveSet | `RedundantContacts_RegularizationPreventsFailure` | Duplicate contacts at same point do not crash |
| solveActiveSet | `SafetyCapReached_ReportsNotConverged` | Artificial degenerate input hits safety cap, returns converged=false |
| solveActiveSet | `KKTConditions_VerifiedPostSolve` | Post-solve: lambda >= 0, A*lambda-b >= -tol, lambda^T*(A*lambda-b) < tol |
| solveActiveSet | `IterationCount_WithinTwoCBound` | Verify iterations <= 2*C for non-degenerate systems |
| solveActiveSet | `ActiveSetSize_ReportedCorrectly` | active_set_size matches number of non-zero lambdas |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `ExistingTestSuite_23of24Pass` | ConstraintSolver, ContactConstraint | 23 of 24 existing tests pass without modification; `MaxIterationsReached_ReportsNotConverged_0033` requires scenario update (see I1 mitigation) |
| `HeadOn_EqualMass_AnalyticalMatch` | ConstraintSolver, ContactConstraint | Lambda matches analytical result within 1e-10 (tighter than PGS's 1e-6) (AC4) |

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Active set cycling (same constraint added/removed) | Very Low | Medium | Bland's rule anti-cycling; regularization ensures strict convexity |
| Higher per-iteration cost for large contact counts (> 20) | Medium | Low | Typical contact count < 10; safety cap limits total work; PGS could be retained as fallback option in future |
| Degenerate configurations (redundant contacts) | Medium | Low | Regularization epsilon prevents singular subproblems; defensive LLT failure check |
| Existing test behavior change | **Confirmed** (1 test) | Medium | `MaxIterationsReached_ReportsNotConverged_0033` requires test scenario modification; ASM converges faster than PGS for trivial cases. All other 23 tests pass unchanged. See I1 mitigation above. |
| Default parameter changes break caller expectations | Low | Low | `setMaxIterations(10)` was PGS-specific; callers that set custom values may need to update. `setConvergenceTolerance(1e-4)` is replaced by `1e-6` default -- callers using default are unaffected; callers using custom values continue to work |
| Performance regression for simple cases (1-2 contacts) | Very Low | Low | ASM solves 1-contact system in 1 iteration vs PGS's 1-10 iterations; ASM is faster for trivial cases |

---

## Open Questions

### Design Decisions (Human Input Needed)

1. **Default `convergence_tolerance_` value change (1e-4 -> 1e-6)**
   - Option A: Keep 1e-4 for backward compatibility with any callers using `setConvergenceTolerance()`
   - Option B: Change to 1e-6 (recommended) since ASM can achieve tighter tolerance at no additional cost
   - Recommendation: Option B. The tolerance semantics change (PGS convergence check vs. ASM violation threshold) makes the old default value meaningless in the new context. A tighter default is more appropriate for a direct solver.

2. **Default `max_safety_iterations_` value (was 10, now 100)**
   - Option A: Keep default at 10 -- backward compatible but may cause premature safety-cap termination for 6+ contacts (2*6=12 > 10)
   - Option B: Change to 100 (recommended) -- provides ample headroom for typical scenes
   - Option C: Change to `INT_MAX` -- rely entirely on 2*C formula
   - Recommendation: Option B. The value 100 handles up to 50 contacts without hitting the global cap, which covers all foreseeable use cases. Setting `INT_MAX` removes the last safety net against pathological inputs.

### Prototype Required

None. The Active Set Method is a well-understood algorithm with established convergence theory. The existing prototype infrastructure (P1 PGS convergence, P2 energy conservation) validates the mathematical correctness of the assembly pipeline (A, b, force extraction). The only change is the solver kernel, which can be validated through unit tests.

### Requirements Clarification

None. The ticket is comprehensive and all requirements are clear.

---

## File Summary

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Replace `PGSResult` with `ActiveSetResult`, replace `solvePGS()` with `solveActiveSet()`, rename `max_iterations_` to `max_safety_iterations_`, update defaults and doc comments |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Implement `solveActiveSet()`, remove `solvePGS()`, update call site in `solveWithContacts()` |
| `msd-sim/test/Physics/Constraints/ConstraintSolverContactTest.cpp` | Modify `MaxIterationsReached_ReportsNotConverged_0033` test scenario to use mixed compressive/separating contacts requiring multiple active set changes (see Test Impact section, I1 mitigation) |

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/Constraints/ConstraintSolverASMTest.cpp` | Active Set Method-specific unit tests (12 tests) |

### Unchanged Files

All files outside of `ConstraintSolver.hpp` and `ConstraintSolver.cpp` are unchanged. Specifically:
- `TwoBodyConstraint.hpp/.cpp`
- `ContactConstraint.hpp/.cpp`
- `ContactConstraintFactory.hpp/.cpp`
- `Constraint.hpp/.cpp`
- `BilateralConstraint.hpp`
- `UnilateralConstraint.hpp`
- `UnitQuaternionConstraint.hpp/.cpp`
- `DistanceConstraint.hpp/.cpp`
- `WorldModel.hpp/.cpp`
- `SemiImplicitEulerIntegrator.hpp/.cpp`
- `InertialState.hpp`
- `AssetInertial.hpp/.cpp`
- `AssetEnvironment.hpp/.cpp`
- `CMakeLists.txt` (only if new test file needs adding to test target)

### Files Requiring CMakeLists.txt Update

| File | Change |
|------|--------|
| `msd-sim/test/Physics/Constraints/CMakeLists.txt` | Add `ConstraintSolverASMTest.cpp` to test sources |

---

## Design Review -- Initial Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-01-31
**Status**: REVISION_REQUESTED
**Iteration**: 0 of 1

### Issues Requiring Revision

| ID | Issue | Category | Required Change |
|----|-------|----------|-----------------|
| I1 | `MaxIterationsReached_ReportsNotConverged_0033` test will FAIL | Backward Compatibility | Revise the test compatibility analysis for this test; acknowledge the breakage and propose a concrete mitigation strategy |
| I2 | Bland's rule specification inconsistent with standard definition | Algorithm Correctness | Clarify that the removal step selects the constraint with the most negative lambda (not the smallest index among all negative), and explicitly state why this is safe under strict convexity |
| I3 | `ActiveSetResult` struct uses `int active_set_size{0}` instead of `std::numeric_limits<int>::quiet_NaN()` pattern; however, int has no NaN | C++ Quality | Minor: document that 0 is the correct default (empty active set), not an uninitialized sentinel. No code change needed but add clarifying comment |
| I4 | Residual semantics change not documented | Interface Stability | Document that `MultiBodySolveResult::residual` changes meaning (from PGS convergence metric to complementarity residual norm) |
| I5 | Design uses structured bindings extraction instead of direct structured binding | C++ Quality | The proposed `solveWithContacts` call site extracts fields manually (`auto asmResult = ...; auto lambda = asmResult.lambda;`). Use structured bindings consistently with the existing codebase pattern |
| I6 | `solveActiveSet` signature differs between ticket and design | Consistency | Ticket pseudocode shows `solveActiveSet(A, b)` without `numContacts` parameter; design adds `numContacts`. Reconcile by documenting why the extra parameter is needed |

### Revision Instructions for Architect

The following changes must be made before final review:

1. **Issue I1 -- Test `MaxIterationsReached_ReportsNotConverged_0033` will FAIL**:

   The existing test (lines 183-236 of `ConstraintSolverContactTest.cpp`) sets `solver.setMaxIterations(1)` with 2 contacts (both penetrating with Baumgarte bias producing positive RHS). Under ASM:
   - `max_safety_iterations_ = 1`
   - Effective cap = `min(2*2, 1) = 1`
   - ASM iteration 1: Full active set W={0,1}, LLT solve produces lambda with both entries positive (b is positive from Baumgarte, A is well-conditioned PD). Primal feasibility passes. Active set is the full set, so no inactive constraints to check. KKT conditions satisfied.
   - ASM returns `converged=true, iterations=1`
   - Test asserts `EXPECT_FALSE(result.converged)` -- **FAILS**

   The design document states "This test should still report non-convergence because 1 iteration is insufficient to verify KKT conditions for a 2-contact system." This analysis is incorrect. For a 2-contact system where both contacts are compressive (positive Baumgarte bias, zero relative velocity), the ASM finds the exact solution in 1 iteration with all KKT conditions satisfied.

   **Required action**: Update the test compatibility analysis in the design document to:
   (a) Acknowledge that this test WILL break under ASM
   (b) Propose one of these mitigations:
     - Option A: Update the test to use a scenario where ASM cannot converge in 1 iteration (e.g., a mixed separating/compressive setup that requires active set changes)
     - Option B: Accept the test modification as a known behavioral difference and document it as a test change required by the ticket
   (c) Add the test modification to the "Modified Files" table

2. **Issue I4 -- Residual semantics change**:

   After `solveActiveSet()`, the existing code computes:
   ```cpp
   Eigen::VectorXd residualVec = A * lambda - b;
   result.residual = residualVec.norm();
   ```
   For PGS, this norm converges toward zero as PGS iterates. For ASM, `w = A*lambda - b` has `w_i = 0` for active contacts and `w_i >= 0` for inactive (separating) contacts. The norm will be non-zero for any solution with inactive contacts. This is not a convergence metric for ASM -- it is the dual residual.

   **Required action**: Add a note in the "Interface Changes Summary" or "Unchanged Components" section explaining the semantic change in `result.residual`. If existing tests or downstream code check this field, flag it. (Current tests do not check `residual`, so this is documentation-only.)

3. **Issue I5 -- Call site extraction style**:

   The current code uses structured bindings:
   ```cpp
   auto [lambda, converged, iterations] = solvePGS(A, b);
   ```

   The proposed replacement uses manual extraction:
   ```cpp
   auto asmResult = solveActiveSet(A, b, numContacts);
   auto lambda = asmResult.lambda;
   auto converged = asmResult.converged;
   auto iterations = asmResult.iterations;
   ```

   This creates unnecessary copies of `lambda` (an `Eigen::VectorXd`). Either:
   (a) Use structured bindings: `auto [lambda, converged, iterations, activeSetSize] = solveActiveSet(A, b, numContacts);`
   (b) Or use `std::move`: `auto lambda = std::move(asmResult.lambda);`

   **Required action**: Update the call site example in the design to avoid the unnecessary copy.

4. **Issue I6 -- Parameter reconciliation**:

   The ticket pseudocode defines `ActiveSetSolve(A, b)` without a `numContacts` parameter, computing the safety cap internally from `b.size()`. The design adds an explicit `numContacts` parameter. Since `numContacts == b.size()` by construction (one constraint row per contact), the parameter is redundant but documents intent.

   **Required action**: Add a brief note explaining why `numContacts` is passed explicitly rather than derived from `b.size()`, or simplify the interface to derive it. Either approach is acceptable; just make the rationale explicit.

### Items Passing Review (No Changes Needed)

The following aspects of the design are well-executed and should NOT be modified:

- **Architectural fit**: Targeted internal replacement with unchanged public interface. Excellent scoping.
- **File structure**: No new classes or files beyond the test file. Follows existing `msd/{component}/src/` pattern.
- **Namespace organization**: Stays within `msd_sim` namespace. Correct.
- **Dependency direction**: No new dependencies introduced. Correct.
- **RAII and smart pointer usage**: No ownership changes. ActiveSetResult uses value semantics. Correct.
- **Rule of Zero**: ActiveSetResult with `= default` constructor. Correct.
- **Const correctness**: `solveActiveSet` is `const` method, matching `solvePGS`. Correct.
- **Brace initialization**: Code samples use brace initialization (`Eigen::LLT<Eigen::MatrixXd> llt{A_W}`). Correct per CLAUDE.md.
- **Data structure choices**: `std::vector<int>` for active indices with rationale for rejecting `std::vector<bool>`. Well-reasoned.
- **Performance analysis**: Honest comparison showing ASM is slower per-iteration for large contact counts but exact. The project's typical use case (1-10 contacts) is well within ASM's efficient regime.
- **Error handling**: Defensive LLT failure check with `converged=false` fallback. Correct.
- **Testability**: New tests cover all critical paths (exact solve, ordering independence, mass ratio, empty/full active sets, KKT verification).
- **PlantUML diagram**: Accurately reflects the scope of changes. Modified, removed, and unchanged components clearly marked.
- **Algorithm description**: The pseudocode is correct for the primal active set method. Mathematical foundation section with KKT conditions is rigorous.
- **Initialization strategy**: Starting with full active set is optimal for the common case (resting contacts). Well-justified.
- **Open Questions**: Recommendations for both default parameter changes (1e-6 tolerance, 100 safety cap) are well-reasoned.

---

## Design Review -- Final Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-01-31
**Status**: APPROVED WITH NOTES
**Iteration**: 1 of 1

### Revision Verification

All 4 issues from the initial assessment have been addressed. Verification follows:

| ID | Issue | Resolution | Verdict |
|----|-------|------------|---------|
| I1 | `MaxIterationsReached` test will FAIL | Acknowledged as "KNOWN BREAKAGE" with concrete mitigation: modify test to use mixed compressive/separating scenario requiring 2+ active set changes. Test file added to Modified Files table. Risk table updated to "Confirmed" status. | Resolved |
| I4 | Residual semantics change not documented | New "Semantic Change: MultiBodySolveResult::residual" section (lines 216-220) explains that `w = A*lambda - b` will be non-zero for inactive contacts and that no existing tests or downstream code inspect this field. | Resolved |
| I5 | Unnecessary copy in call site | Call site updated to structured bindings: `auto [lambda, converged, iterations, activeSetSize] = solveActiveSet(A, b, numContacts);` (line 156), consistent with existing codebase pattern. | Resolved |
| I6 | `numContacts` parameter redundancy | Doxygen comment for `solveActiveSet` (lines 105-109) now documents rationale: "Passed explicitly rather than derived from b.size() to document intent and decouple the safety cap computation from the Eigen vector internals." | Resolved |

### Criteria Assessment

#### Architectural Fit
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | Pass | `ActiveSetResult`, `solveActiveSet`, `max_safety_iterations_`, `active_set_size` all follow project conventions |
| Namespace organization | Pass | Stays within `msd_sim` namespace, no new namespaces introduced |
| File structure | Pass | No new source files; new test file follows `test/Physics/Constraints/` pattern |
| Dependency direction | Pass | No new dependencies; internal replacement within `ConstraintSolver` |

#### C++ Design Quality
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | Pass | No ownership changes; subproblem matrices are locally scoped |
| Smart pointer appropriateness | Pass | No pointer usage in new code; value semantics throughout |
| Value/reference semantics | Pass | `ActiveSetResult` returned by value; `A` and `b` passed by const reference |
| Rule of 0/3/5 | Pass | `ActiveSetResult` uses `= default` constructor with zero-initialized members |
| Const correctness | Pass | `solveActiveSet` is `const` method, matching `solvePGS` |
| Exception safety | Pass | Defensive LLT failure check returns `converged=false`; no exceptions thrown |
| Brace initialization | Pass | Code samples use brace initialization (`Eigen::LLT<Eigen::MatrixXd> llt{A_W}`) |
| NaN initialization | Pass | `int` members use `{0}` defaults (appropriate; `int` has no NaN representation) |

#### Feasibility
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | Pass | No new includes required |
| Template complexity | Pass | No templates introduced |
| Memory strategy | Pass | All allocations are local/stack; < 2 KB for C=10 |
| Thread safety | Pass | `solveActiveSet` is `const` with only local state; thread-safe for different inputs |
| Build integration | Pass | Only `CMakeLists.txt` update for new test file |

#### Testability
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | Pass | `solveActiveSet` takes `A`, `b` matrices directly; can be tested with synthetic inputs |
| Mockable dependencies | Pass | No external dependencies to mock; pure mathematical function |
| Observable state | Pass | `ActiveSetResult` exposes all diagnostic fields (lambda, converged, iterations, active_set_size) |

### Notes

#### Note 1: Interface Changes Summary Table Inconsistency (Minor)

The "Source File Changes" table (line 515) still describes the call site as:
```
auto asmResult = solveActiveSet(A, b, numContacts); + extract fields
```
This contradicts the corrected call site in the "Updated solveWithContacts() Internal Call" section (line 156), which correctly uses structured bindings. The implementer should follow the detailed section (line 156), not the summary table. This is a documentation-only inconsistency that does not affect implementation correctness.

#### Note 2: Ticket AC2 Conflict Requires Human Decision

Ticket AC2 states: "All 24 existing `ConstraintSolverContactTest` tests pass **without modification**." The design correctly identifies that `MaxIterationsReached_ReportsNotConverged_0033` will break and proposes modifying the test. This creates a conflict between the ticket acceptance criteria and the design.

**Human action needed**: Either (a) update ticket AC2 to "23 of 24 tests pass without modification; 1 test requires scenario update", or (b) accept that this AC is technically violated but the modification is justified by the solver behavioral change. This is a documentation/process decision, not a technical issue.

#### Note 3: Existing Tests Table Still Says "All 24 Tests" (Minor)

The "Existing Tests Affected" table (line 538) states:
```
| ConstraintSolverContactTest.cpp | All 24 tests | Behavioral equivalence expected | Verify all pass without modification |
```
The "Action Required" column says "Verify all pass without modification," which contradicts the detailed analysis below it that identifies 1 test requiring modification. The implementer should treat the detailed analysis (lines 546-548) as authoritative.

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Active set cycling in degenerate configurations | Technical | Very Low | Medium | Bland's rule + regularization ensures strict convexity | No |
| R2 | Cubic per-iteration cost for large contact counts | Performance | Medium | Low | Project targets < 10 contacts; safety cap limits total work | No |
| R3 | Test scenario for `MaxIterationsReached` may be non-trivial to construct | Technical | Low | Low | Design proposes mixed compressive/separating scenario; implementer should verify negative lambda is produced by the chosen contact geometry | No |

### Prototype Guidance

No prototypes required. The Active Set Method is well-established in numerical optimization literature. The algorithm operates on the same assembled `A` and `b` matrices already validated by the P1 and P2 prototypes from ticket 0032b. The 12 new unit tests provide sufficient validation coverage for the solver kernel in isolation.

### Summary

All 4 issues from the initial review have been adequately resolved. The design document now correctly acknowledges the `MaxIterationsReached` test breakage with a concrete mitigation strategy, documents the residual semantics change, uses structured bindings at the call site, and explains the `numContacts` parameter rationale. The only remaining items requiring human attention are (1) reconciling ticket AC2 with the acknowledged test modification and (2) minor documentation inconsistencies in summary tables that do not affect implementation correctness. The design is ready for human review and, upon approval, can proceed directly to implementation without a prototype phase.

---
