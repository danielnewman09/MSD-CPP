# Design: Constraint Interface Static Dispatch Investigation

**Ticket**: 0071d_constraint_interface_static_dispatch
**Type**: Performance / Investigation
**Date**: 2026-02-21
**Status**: Investigation Complete — Recommendation: Do Not Proceed

---

## Summary

This document investigates whether replacing the `Constraint` virtual interface
with static polymorphism (CRTP, `std::variant`, hybrid fixed-size accessors, or
output buffers) is warranted given the profiling data established by ticket 0071
and the direct fixed-size conversions delivered by ticket 0071c.

The investigation concludes that architectural changes to the `Constraint`
virtual interface are **not justified** at this time. The remaining heap
allocation cost (one `MatrixXd` per virtual `jacobian()` call in
`flattenConstraints` and `PositionCorrector`) is small relative to the dominant
costs (Eigen BLAS for LLT/matrix-multiply, `solveActiveSet` pivoting, and
`correctPositions` iterative solve). The architectural cost of any static
dispatch approach — closed type set, significant solver refactor, loss of
extensibility — outweighs the performance benefit.

---

## Context: What 0071c Delivered

Ticket 0071c converted solver-internal data structures to fixed-size Eigen types
at the *boundary* between the virtual interface and the solver internals:

- `FlattenedConstraints::jacobianRows`: `vector<RowVectorXd>` → `vector<Matrix<double, 1, 12>>`
- `assembleJacobians()` return: `vector<MatrixXd>` → `vector<Matrix<double, 1, 12>>`
- All downstream callers (`assembleEffectiveMass`, `assembleRHS`,
  `extractBodyForces`) now use compile-time block operations

Key insight from 0071c: the solver already decouples from the virtual interface
after the initial jacobian extraction. The virtual `jacobian()` call returns a
`MatrixXd`, but the result is immediately narrowed into a fixed-size
`Matrix<double, 1, 12>` storage type. All subsequent computation (effective mass
assembly, RHS, force extraction) uses fixed-size arithmetic with zero heap
allocation.

---

## Remaining Virtual Interface Costs After 0071c

The virtual `jacobian()` call still returns `MatrixXd` at two locations:

### 1. `flattenConstraints()` — friction path

```cpp
// ConstraintSolver.cpp line ~687
auto j = constraint->jacobian(states[bodyA].get(), states[bodyB].get(), 0.0);
// j is MatrixXd — one heap allocation per constraint
for (int row = 0; row < dim; ++row)
{
  flat.jacobianRows.push_back(j.row(row));  // copies into Matrix<double,1,12>
}
```

Cost per constraint: one `MatrixXd` construction (small, 1x12 or 2x12), one
vector push_back (no allocation if reserved), one copy into fixed-size type.
For 150 constraints with friction, this is approximately 150 `MatrixXd`
allocations per frame.

### 2. `PositionCorrector::correctPositions()` — jacobians workspace

```cpp
// PositionCorrector.cpp line ~102
jacobians_[i] = constraint->jacobian(*states[bodyA], *states[bodyB], 0.0);
// jacobians_ is vector<MatrixXd> — workspace reused across frames
```

The workspace vector is a member of `PositionCorrector` (ticket 0053f) and is
resized but not re-allocated each call if size is stable. The assignment
`jacobians_[i] = MatrixXd` uses the Eigen assignment operator, which
reallocates if the dimensions change (they do not across frames for a stable
constraint set). In practice this is a copy, not an allocation, after the first
frame. **The remaining cost in `correctPositions` is the virtual call overhead
itself, not the heap allocation.**

### Quantified Residual Cost

From the ClusterDrop/32 profiling (18,146 total samples, ~5 seconds):

| Source | Samples | % Total | Notes |
|--------|---------|---------|-------|
| Memory allocator (_xzm_free + malloc) | ~80 | ~0.4% | All sources combined |
| `flattenConstraints` | 24 | 0.13% | Includes virtual calls + any alloc |
| `PositionCorrector::correctPositions` | 120 | 0.66% | Full function, not just virtual calls |

The 80 allocator samples span all sources (constraint construction, vector
resizing, Eigen temporaries from BLAS, DataRecorder). Attributing even half
(40 samples, 0.22%) to the remaining virtual `jacobian()` `MatrixXd`
allocations in `flattenConstraints` is generous. The realistic contribution from
the two virtual interface call sites is **less than 0.2% of total CPU**.

For comparison, the dominant costs are:

| Source | Samples | % Total |
|--------|---------|---------|
| Eigen BLAS (gebp_kernel, triangular_solve, LLT) | ~650 | ~3.6% |
| `ConstraintSolver::solveActiveSet` | 90 | 0.5% |
| `ConstraintSolver::solve` | 86 | 0.5% |
| `PositionCorrector::correctPositions` | 120 | 0.7% |
| `assembleFlatEffectiveMass` | 56 | 0.3% |

The Eigen BLAS operations (LLT Cholesky, matrix multiply) are **18× larger**
than the estimated remaining virtual interface allocation cost. These BLAS
operations are inherent to the O(n³) dense LCP solve and cannot be eliminated
by interface changes — only by switching to a fundamentally different algorithm
(already addressed for large islands by ticket 0073 PGS).

---

## Option Evaluation

### Option A: CRTP

Replace virtual dispatch with `template<typename Derived, int Dim, int Cols>
class ConstraintBase`.

**Analysis**:
- Eliminates virtual dispatch AND heap allocation for `jacobian()` / `evaluate()`
- **Breaking change**: Cannot store `ContactConstraint` and `FrictionConstraint`
  in a single `vector<Constraint*>`. The entire `ConstraintSolver::solve()`,
  `flattenConstraints()`, `PositionCorrector::correctPositions()`, and
  `CollisionPipeline` interface must be refactored to use `std::variant`,
  `std::tuple<vector<ContactConstraint>, vector<FrictionConstraint>>`, or
  separate typed loops
- The future work noted in CLAUDE.md explicitly mentions joints (hinges,
  ball-socket). Adding a `HingeConstraint` to a CRTP hierarchy requires
  modifying every call site that iterates the constraint collection
- The `recordState()` visitor pattern on the existing `Constraint` interface
  is incompatible with CRTP without a parallel virtual dispatch mechanism
- **Verdict: Cost too high, benefit too small**

### Option B: std::variant-Based Dispatch

Replace `vector<unique_ptr<Constraint>>` with
`vector<variant<ContactConstraint, FrictionConstraint>>`.

**Analysis**:
- No virtual dispatch; `std::visit` handles dispatch
- For N=2 types, `std::visit` performance is comparable to virtual dispatch
  (branch prediction typically succeeds on homogeneous type lists)
- Value semantics: constraints stored inline, no `unique_ptr` overhead
- **Closed type set**: Adding `HingeConstraint` requires modifying the variant
  alias, `std::visit` handlers in all three components, the `ConstraintRecordVisitor`
  pattern, and the `CollisionPipeline` factory
- `FrictionConstraint::setNormalLambda()` is called by the solver per-iteration.
  With variant storage, this becomes a `std::get<FrictionConstraint>(v).setNormalLambda()`
  pattern requiring type knowledge at call sites — equivalent complexity to
  the current `dynamic_cast` pattern
- Performance gain vs CRTP: eliminates virtual dispatch overhead but still
  pays `std::visit` overhead. For N=2 types, the difference from virtual
  dispatch is negligible (both are indirect calls with branch prediction)
- **Verdict: Extensibility regression not justified by the <0.2% gain**

### Option C: Hybrid — Keep Virtual Interface, Add Fixed-Size Accessors

Add non-virtual fixed-size methods alongside existing virtual methods:

```cpp
// On ContactConstraint (non-virtual):
Eigen::Matrix<double, 1, 12> jacobianFixed(...) const;

// On FrictionConstraint (non-virtual):
Eigen::Matrix<double, 2, 12> jacobianFixed(...) const;
```

Hot paths that know the concrete type (e.g., after a `dynamic_cast` or
using typed containers) call the fixed-size accessors directly.

**Analysis**:
- Additive change — virtual interface unchanged, extensibility preserved
- `flattenConstraints()` already uses `dynamic_cast<const ContactConstraint*>`
  to check row type. Adding a parallel `dynamic_cast` for the jacobian call
  is feasible but duplicates business logic
- `PositionCorrector` uses only `ContactConstraint` (the `jacobian` workspace
  is all contact constraints). Adding a `dynamic_cast` + `jacobianFixed()`
  call path is straightforward
- Maintenance burden: Two `jacobian()` implementations per concrete type that
  must remain in sync
- The fixed-size methods do not eliminate the `dynamic_cast` pattern; they
  require it. The virtual call is already O(1); the issue is the `MatrixXd`
  heap allocation on return. The `jacobianFixed()` approach eliminates the
  allocation without eliminating the dispatch overhead
- However: the `PositionCorrector` workspace `jacobians_` is already reused
  across calls (ticket 0053f). After the first frame, no allocation occurs —
  only virtual call + copy. The call overhead for correctPositions' 120 samples
  is predominantly the iterative ASM solve, not the virtual calls
- **Verdict: Not recommended.** The benefit is narrow and the maintenance cost
  of two parallel implementations is ongoing. The allocation-from-`MatrixXd`
  return is already eliminated in all downstream compute paths by 0071c; the
  only remaining cost is the `MatrixXd` temporary in `flattenConstraints()`,
  which is bounded by the constraint count and small in absolute terms

### Option D: Output Buffer Pattern

Change virtual interface to write into caller-provided buffers:

```cpp
virtual void jacobianInto(double* output, int rows, int cols) const = 0;
```

**Analysis**:
- Eliminates heap allocation (caller provides stack buffer)
- Still has virtual dispatch overhead
- Breaks the ergonomic Eigen return type; callers must pre-allocate and know
  dimensions before calling — the caller already knows this (it's always 1x12
  for ContactConstraint or 2x12 for FrictionConstraint), so this is viable
  but verbose
- The `Eigen::Map<Eigen::Matrix<double, 1, 12>>` pattern can wrap the output
  buffer for ergonomic downstream use
- The extensibility impact is significant: any new constraint type must have
  its callers know the correct `rows` and `cols` before the call, eliminating
  the "generic treatment of heterogeneous constraints" that the current interface
  provides
- For the `dimension()` virtual call still needed to know `rows`, this does
  not eliminate the virtual call, only the return allocation
- **Verdict: Not recommended.** Breaks interface ergonomics and extensibility
  for a gain that is smaller than the noise floor in profiling

---

## Cost-Benefit Summary

| Option | Estimated CPU Gain | Architectural Cost | Extensibility Impact | Recommendation |
|--------|-------------------|--------------------|----------------------|----------------|
| A: CRTP | <0.2% | Very High — full refactor of solver + pipeline + visitor | Closed type set | Do Not Proceed |
| B: std::variant | <0.2% | High — variant alias + visit handlers in 3 components | Closed type set | Do Not Proceed |
| C: Hybrid fixed-size accessors | <0.1% | Medium — duplicate methods on 2 types, dynamic_cast at call sites | No regression | Not Worth It |
| D: Output buffer | <0.1% | Medium — breaks ergonomic API, callers must know dimensions | Degraded | Not Worth It |

---

## The 0071c Boundary Pattern Is Sufficient

The key architectural insight from 0071c is that the solver can be partitioned
into two zones:

1. **Interface Zone** (virtual calls, `MatrixXd` returns): `flattenConstraints()`,
   `assembleJacobians()`, `PositionCorrector::correctPositions()`. These are
   narrow call sites executed once per constraint per solver invocation.

2. **Compute Zone** (fixed-size operations, zero virtual calls): All downstream
   matrix assembly (`assembleFlatEffectiveMass`, `assembleEffectiveMass`), solve
   (`solveActiveSet`, Gauss-Seidel friction), and force extraction. These execute
   many operations per constraint and are now fully fixed-size.

The profiling shows that the Compute Zone (Eigen BLAS: 3.6%) dominates. The
Interface Zone is a small fraction of the 0.4% memory allocator cost. Changing
the interface contract to eliminate the Interface Zone costs would require
significant architectural work for a sub-0.2% gain against a 3.6% BLAS
baseline that is not addressable by interface changes.

---

## Real Optimization Opportunities

If further constraint solver performance is needed, the evidence points to:

1. **Reduce constraint system size** (0071a, already in progress for large islands
   via ticket 0073 PGS): The O(n³) LLT Cholesky on the full dense effective mass
   matrix is the dominant cost (gebp_kernel + triangular_solve = 2.3%). For
   small-to-medium islands (≤20 rows), the current ASM is appropriate. For large
   islands, ticket 0073's PGS path is already routing correctly.

2. **PositionCorrector::correctPositions workspace conversion** (Medium Priority):
   The `jacobians_` workspace in `PositionCorrector` stores `MatrixXd`. For the
   contact-only view it receives (all `ContactConstraint`, dimension=1, columns=12),
   this could be converted to `vector<Matrix<double, 1, 12>>` following the same
   pattern 0071c applied to `assembleJacobians()`. This is a non-architectural
   change (no virtual interface modification) with moderate benefit. Estimated
   gain: small allocation reduction in `correctPositions` (currently 120 samples,
   0.7% of total). **This is the most attractive remaining optimization in the
   constraint solver and does not require any interface change.**

3. **PositionCorrector fixed-size Jacobian workspace** (see above): The
   `PositionCorrector::jacobians_` member is `vector<Eigen::MatrixXd>`. After
   ticket 0053f made it a reused workspace, the allocation cost is one-time per
   resize event. Converting to `vector<Matrix<double, 1, 12>>` eliminates the
   per-element copy from `MatrixXd` to fixed-size and enables `leftCols<6>()` /
   `rightCols<6>()` in the effective mass assembly inside
   `correctPositions`. This is scoped entirely within `PositionCorrector` with no
   interface changes. **Recommend creating a follow-on ticket for this.**

---

## Recommendation

**Do not proceed with any architectural change to the `Constraint` virtual
interface.** The virtual interface with `Eigen::MatrixXd` / `VectorXd` return
types should be retained as-is.

The 0071c boundary pattern (fixed-size storage in solver internals, `MatrixXd`
at the interface boundary) achieves the available benefit from fixed-size Eigen
types without any extensibility regression.

The next highest-value optimization in the constraint solver is converting
`PositionCorrector::jacobians_` from `vector<MatrixXd>` to
`vector<Matrix<double, 1, 12>>`, which is a non-architectural change entirely
within `PositionCorrector.hpp/.cpp`. This should be tracked as a separate
small ticket (suggested: 0071e or 0071b continuation) if the 0.7%
`correctPositions` cost warrants further work.

---

## Ticket Status Advancement

Based on this investigation, the 0071d ticket should advance to
"Investigation Complete" with the following disposition:

- **Finding**: Architectural static dispatch changes are not warranted
- **Action**: Close 0071d as "No Implementation Required — Investigation Only"
- **Follow-on**: Optional ticket for `PositionCorrector` fixed-size Jacobian
  workspace (non-architectural, additive)

---

## References

- [0071_collision_pipeline_profiling.md](../../tickets/0071_collision_pipeline_profiling.md)
  — Profiling data and multi-body benchmark results
- [0071c_eigen_fixed_size_matrices.md](../../tickets/0071c_eigen_fixed_size_matrices.md)
  — Delivered boundary pattern and implementation notes
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` — `flattenConstraints()`,
  `assembleJacobians()`, Ticket 0071c annotations
- `msd/msd-sim/src/Physics/Constraints/PositionCorrector.cpp` — `jacobians_`
  workspace, Ticket 0053f annotations
- `msd/msd-sim/src/Physics/Constraints/Constraint.hpp` — Virtual interface definition
