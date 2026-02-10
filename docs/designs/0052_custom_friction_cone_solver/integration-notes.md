# Integration Notes: Custom Friction Cone Solver (Ticket 0052)

> Codebase analysis for replacing ECOS SOCP solver with a custom Newton-based friction cone solver.
> Produced by codebase-researcher agent.

---

## 1. Current ECOS Dispatch Path

### Entry Point: `ConstraintSolver::solve()`

**File**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp:52-144`

The top-level `solve()` method dispatches between two solver paths based on friction presence:

```
solve()
  ├── hasFriction = scan constraints for lambdaBounds().isBoxConstrained()
  ├── assembleJacobians()     → per-constraint Jacobian matrices
  ├── assembleEffectiveMass() → A = J M⁻¹ Jᵀ (C×C or 3C×3C)
  ├── assembleRHS()           → b = -(1+e) * J*v
  │
  ├── if hasFriction:
  │   ├── count numContacts (unilateral constraints)
  │   ├── buildFrictionConeSpec()  → FrictionConeSpec with μ per contact
  │   └── solveWithECOS(A, b, coneSpec, numContacts)
  │       ├── ECOSProblemBuilder::build(A, b, coneSpec) → ECOSData
  │       ├── ECOSData::setup()     → ECOS_setup()
  │       ├── ECOS_solve()          → raw solution
  │       └── extract lambda, diagnostics
  │
  └── if !hasFriction:
      └── solveActiveSet(A, b, numContacts, initialLambda)
          └── Active Set Method with warm-starting
```

### Key Observations

1. **Friction detection**: The solver scans for any constraint with `lambdaBounds().isBoxConstrained()`. If found, the *entire* system is routed to ECOS (line 101).

2. **buildFrictionConeSpec()** (line 540-569): Scans the flat constraint list for `FrictionConstraint*` via `dynamic_cast`. For each friction constraint, records μ and the normal constraint index (3*contactIdx).

3. **solveWithECOS()** (line 571-625): Creates ECOSData, sets up workspace, calls ECOS_solve(), extracts lambda. Result is wrapped in `ActiveSetResult` with ECOS-specific diagnostics.

4. **Current friction path is not used in production**: `CollisionPipeline` only creates `ContactConstraint` objects, never `FrictionConstraint`. The ECOS path was implemented for validation/testing but is not integrated into the live simulation pipeline.

---

## 2. System Matrix Assembly

### `assembleJacobians()` (line 148-165)

- Stores one `Eigen::MatrixXd` per constraint (size varies: 1×12 for ContactConstraint, 2×12 for FrictionConstraint)
- Does NOT flatten multi-row Jacobians into separate rows

### `assembleEffectiveMass()` (line 167-242)

**Critical limitation**: Uses `block<1, 6>(0, 0)` and `block<1, 6>(0, 6)` to extract Jacobian sub-blocks. This **only reads row 0** of each Jacobian matrix. For FrictionConstraint (dim=2), row 1 is ignored.

- Builds per-body 6×6 inverse mass matrices: `diag(m⁻¹ I₃, I⁻¹)`
- Assembles `A_ij = Σ_shared_bodies J_i_k M_k⁻¹ J_j_kᵀ`
- Adds diagonal regularization `ε = 1e-8`
- Result is `C × C` where C = number of constraint entries (NOT rows)

**What changes for 3C×3C**: The custom solver needs to expand multi-row constraints so that A is indexed by constraint *rows* (3 per contact: 1 normal + 2 friction). Two approaches:
  1. **Flatten**: Expand FrictionConstraint(dim=2) into two separate 1×12 rows before assembly
  2. **Block-aware**: Modify assembly to handle variable-dimension constraints

Recommendation: **Flatten approach** is simpler and matches the math formulation. The custom solver receives a flat 3C×3C system.

### `assembleRHS()` (line 244-306)

- Computes `b_i = -(1+e) * (J_i * v)` for ContactConstraints
- Falls back to `b_i = -jv` for non-contact constraints (generic)
- Uses `dynamic_cast<ContactConstraint*>` to check for restitution
- Also uses `block<1,6>(0,0)` assumption implicitly via `(jacobians[i] * v)(0)` -- this works for 1×12 but for 2×12 would produce a 2×1 vector, and only `(0)` is taken

**What changes**: For friction rows, restitution = 0 (friction doesn't have restitution, it dampens tangential velocity). The RHS for friction rows is simply `b_t = -J_t * v` (drive tangential velocity to zero for static friction).

### `extractBodyForces()` (line 484-531)

- Computes `F_k = Σ J_i_kᵀ * λ_i / dt` for each body k
- Only uses row 0 via `block<1,6>(0,0)` and `block<1,6>(0,6)`
- Skips `λ ≤ 0` entries (but friction lambda can be negative!)

**Critical change needed**: `extractBodyForces()` skips entries where `lambda(i) <= 0.0` (line 502). Friction forces can be negative (pushing in negative tangent direction). This skip must be removed or conditioned on constraint type for the friction path.

---

## 3. ContactCache Adaptation

### Current Format

**File**: `msd/msd-sim/src/Physics/Constraints/ContactCache.hpp/cpp`

```cpp
struct CachedContact {
  uint32_t bodyA_id;
  uint32_t bodyB_id;
  Vector3D normal;
  std::vector<double> lambdas;     // Per-contact lambda values
  std::vector<Coordinate> points;  // Contact midpoints
  uint32_t age{0};
};
```

- `lambdas` is a `std::vector<double>` -- variable size, one entry per contact point in the manifold
- Currently stores **1 lambda per contact** (normal force only)
- Matching done by nearest-neighbor point matching (2cm radius threshold)
- Invalidation on normal rotation > 15 degrees

### Required Changes for 3-Component Lambda

With friction, each contact produces 3 lambda values: `[λ_n, λ_t1, λ_t2]`.

**Option A (Flat vector)**: Store `lambdas` as a flat vector of size `3 * numContacts`. Contact i's lambdas are at indices `[3i, 3i+1, 3i+2]`. The `getWarmStart()` matching logic operates on contact *groups* rather than individual entries.

**Option B (Struct per contact)**: Change to `std::vector<ContactLambda>` where:
```cpp
struct ContactLambda {
  double normal;
  double tangent1;
  double tangent2;
};
```

**Recommendation**: Option A (flat vector) for minimal structural change. The cache already stores `lambdas` as `vector<double>` and the matching logic pairs contacts 1:1. Just change the pairing to map 3 values per contact instead of 1.

### `getWarmStart()` Changes

Current: returns `vector<double>` with one entry per current contact point.
New: returns `vector<double>` with 3 entries per current contact point (flat).

The nearest-neighbor matching (line 49-75 of ContactCache.cpp) currently copies a single `cached.lambdas[*bestIdx]` per matched contact. It would need to copy 3 values: `cached.lambdas[3*bestIdx]`, `cached.lambdas[3*bestIdx+1]`, `cached.lambdas[3*bestIdx+2]`.

### `CollisionPipeline` Impact

`CollisionPipeline::solveConstraintsWithWarmStart()` builds an `initialLambda` vector of size `totalConstraints` (one per Constraint* entry). With friction, the lambda vector will be size 3C (3 per contact). The cache update (line 308-327) will need to extract 3 values per contact from `solveResult.lambdas`.

---

## 4. FrictionConstraint Interface

### Tangent Basis

**File**: `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.cpp:12-40`

- Computed once at construction via `TangentBasis::computeTangentBasis(normal)`
- Returns `TangentFrame{t1, t2}` -- two unit vectors orthogonal to normal and each other
- Stored as `tangent1_`, `tangent2_` members

### 2×12 Jacobian Structure

```
Row 0 (t1): [t1ᵀ, (rA×t1)ᵀ, -t1ᵀ, -(rB×t1)ᵀ]
Row 1 (t2): [t2ᵀ, (rA×t2)ᵀ, -t2ᵀ, -(rB×t2)ᵀ]
```

Block layout: `[v_A(3), ω_A(3), v_B(3), ω_B(3)]` -- same as ContactConstraint.

### `setNormalLambda()` and `getFrictionBounds()`

- `setNormalLambda(λ_n)` stores the current normal force (updated each solver iteration)
- `getFrictionBounds()` returns `±(μ/√2) * λ_n` (box constraint approximation of circular cone)
- The `√2` factor is the inscribed-square approximation

**For the custom solver**: We do NOT use box constraint approximation. We enforce the full circular cone `||[λ_t1, λ_t2]|| ≤ μ * λ_n` directly. Therefore `getFrictionBounds()` is not needed. The custom solver uses `getFrictionCoefficient()` directly.

### `isActive()` Check

Returns true when `μ > 0` AND `λ_n > 0`. The custom solver will decide activation internally based on the normal force during the Newton iteration.

---

## 5. Files to Remove (ECOS)

All files in `msd/msd-sim/src/Physics/Constraints/ECOS/`:

| File | Purpose | External Dependencies |
|------|---------|----------------------|
| `ECOSData.hpp` | RAII wrapper for ECOS workspace | `<ecos/ecos.h>` |
| `ECOSData.cpp` | Implementation of setup/cleanup | `<ecos/ecos.h>`, `<ecos/glblopts.h>` |
| `ECOSProblemBuilder.hpp` | LCP → SOCP problem conversion | FrictionConeSpec |
| `ECOSProblemBuilder.cpp` | G matrix construction, problem building | ECOSData, FrictionConeSpec |
| `ECOSSparseMatrix.hpp` | Eigen → CSC format conversion | `<ecos/ecos.h>`, `<Eigen/Sparse>` |
| `ECOSSparseMatrix.cpp` | fromDense(), fromSparse() | `<ecos/glblopts.h>` |
| `FrictionConeSpec.hpp` | Friction cone specification (μ, indices) | `<ecos/ecos.h>` (for `idxint`) |
| `FrictionConeSpec.cpp` | Implementation | `<ecos/glblopts.h>` (for `idxint`) |
| `CLAUDE.md` | Documentation | — |
| `CMakeLists.txt` | Build config | — |

### External Dependency Confirmation

- **FrictionConeSpec** currently depends on `<ecos/ecos.h>` for the `idxint` type (used in `getConeSizes()` return type). However, FrictionConeSpec's core data (μ per contact, normal indices) has no ECOS-specific logic. The custom solver may want a similar data structure, but without the ECOS type dependency.

- **No other code outside `ECOS/` and `ConstraintSolver` depends on ECOS files** -- confirmed by grep. The only references to FrictionConeSpec outside ECOS/ are in `ConstraintSolver.hpp` (include + function signatures) and `ConstraintSolver.cpp` (include + usage).

### Test Files to Remove

| File | Tests |
|------|-------|
| `test/Physics/Constraints/ECOS/FrictionConeSpecTest.cpp` | 11 tests |
| `test/Physics/Constraints/ECOS/ECOSProblemBuilderTest.cpp` | ~15 tests |
| `test/Physics/Constraints/ECOS/ECOSSolveTest.cpp` | ~11 tests |
| `test/Physics/Constraints/ECOS/ECOSFrictionValidationTest.cpp` | ~12 tests |
| `test/Physics/Constraints/ECOS/CMakeLists.txt` | Build config |

---

## 6. Files to Modify

### `ConstraintSolver.hpp`

- Remove `#include "ECOS/FrictionConeSpec.hpp"` (line 23)
- Remove `solveWithECOS()` declaration (lines 262-265)
- Remove `buildFrictionConeSpec()` declaration (lines 397-399)
- Remove ECOS configuration members: `ecos_abs_tol_`, `ecos_rel_tol_`, `ecos_max_iters_` (lines 408-411)
- Remove ECOS configuration methods: `setECOSTolerance()`, `setECOSMaxIterations()`, `getECOSTolerance()`, `getECOSMaxIterations()` (lines 179-215)
- Remove ECOS diagnostic fields from `ActiveSetResult`: `solver_type`, `ecos_exit_flag`, `primal_residual`, `dual_residual`, `gap` (lines 237-243)
- Add: `solveWithFriction()` method declaration for the new solver
- Modify `assembleJacobians()`, `assembleEffectiveMass()`, `assembleRHS()`, `extractBodyForces()` to handle 3-row constraint groups (or add friction-specific variants)

### `ConstraintSolver.cpp`

- Remove `#include <ecos/ecos.h>` and `#include <ecos/glblopts.h>` (lines 17-18)
- Remove `#include "ECOS/ECOSData.hpp"`, `#include "ECOS/ECOSProblemBuilder.hpp"`, `#include "ECOS/FrictionConeSpec.hpp"` (lines 30-32)
- Remove `buildFrictionConeSpec()` implementation (lines 540-569)
- Remove `solveWithECOS()` implementation (lines 571-625)
- Replace ECOS dispatch in `solve()` (lines 101-123) with custom solver dispatch
- Fix `extractBodyForces()` to not skip negative lambda (friction forces)

### `msd/msd-sim/CMakeLists.txt`

- Remove `find_package(ecos REQUIRED CONFIG)` (line 16)
- Remove `ecos::ecos` from `target_link_libraries` (line 33)

### `msd/msd-sim/src/Physics/Constraints/CMakeLists.txt`

- Remove `add_subdirectory(ECOS)` (line 1)

### `conanfile.py`

- Remove `self.requires("ecos/2.0.10")` (line 99)

### `ContactCache.hpp/cpp`

- Modify `getWarmStart()` to return 3 values per contact instead of 1
- Modify `update()` to accept 3 values per contact
- Or: keep interface as-is and let the caller handle flattening/expanding

### `CollisionPipeline.hpp/cpp`

- Modify `createConstraints()` to also create `FrictionConstraint` objects alongside `ContactConstraint`
- Currently stores `vector<unique_ptr<ContactConstraint>>` -- change to `vector<unique_ptr<Constraint>>` to hold both types
- Modify `solveConstraintsWithWarmStart()` to handle 3C-dimensional lambda
- Modify cache update to store/retrieve 3 lambdas per contact
- Add friction coefficient as a property of AssetInertial (or passed to factory)

### `ContactConstraintFactory.hpp/cpp`

- Add factory method to create paired `ContactConstraint` + `FrictionConstraint` per contact point
- Or: create a new `FrictionContactFactory` that produces both constraint types together

---

## 7. New Files

| File | Purpose |
|------|---------|
| `ConeProjection.hpp/cpp` | Implements projection onto friction cone: `proj_K(λ) = argmin_{x ∈ K} ||x - λ||` |
| `FrictionConeSolver.hpp/cpp` | Newton-based LCP solver with cone constraints: `A*λ + b ∈ K*`, `λ ∈ K` |

### ConeProjection

The cone projection function projects a 3-vector `(λ_n, λ_t1, λ_t2)` onto the second-order cone `{x : ||(x_1, x_2)|| ≤ μ * x_0}`:

- If already inside cone: return as-is
- If in dual cone (fully separated): return zero
- Otherwise: project onto cone boundary

### FrictionConeSolver

Newton solver for the mixed complementarity problem:
```
Find λ such that:
  Aλ + b = w
  For each contact: (λ_i, w_i) satisfy cone complementarity
```

Uses Fischer-Burmeister NCP function adapted for SOC constraints.

---

## 8. Gotchas and Risks

### 1. Jacobian Row Counting

The current `assembleEffectiveMass()`, `assembleRHS()`, and `extractBodyForces()` all assume each entry in `contactConstraints` produces exactly 1 Jacobian row (using `block<1,6>`). This assumption breaks with FrictionConstraint (dim=2).

**Mitigation**: Either:
- Expand constraints into individual rows before passing to assembly functions
- Rewrite assembly functions to handle variable-dimension constraints

The **expansion approach** (flatten into 1 row per entry) is safest and consistent with the math formulation where A is 3C×3C with rows indexed by [n₁, t1₁, t2₁, n₂, t1₂, t2₂, ...].

### 2. Negative Lambda Filtering

`extractBodyForces()` (line 502) skips `lambda(i) <= 0.0`. Friction lambdas are unconstrained in sign (can push in either tangent direction). This filter must be removed or restricted to the normal component only.

### 3. FrictionConstraint Construction Requires CoM

FrictionConstraint needs `contactPointA`, `contactPointB`, `comA`, `comB` -- same as ContactConstraint. The factory already computes these, so FrictionConstraint creation can happen alongside ContactConstraint in `createConstraints()`.

### 4. Constraint Ordering Convention

ECOS used ordering `[n₀, t1₀, t2₀, n₁, t1₁, t2₁, ...]` (interleaved per contact). The custom solver must maintain this ordering for the cone projection to identify which 3-tuples form a contact.

### 5. ContactCache Symmetry

`ContactCache::makeKey()` produces symmetric keys: `(min(a,b), max(a,b))`. The normal direction must be consistent with body ordering. This is already handled by `CollisionHandler` (normal points A→B).

### 6. Environment Bodies Have Zero Friction

`AssetEnvironment` currently has no friction coefficient property. A default friction coefficient will need to be defined (e.g., 0.5) or made configurable.

### 7. Warm-Starting Tangential Lambda

Warm-starting tangential lambda is more sensitive than normal lambda because tangent basis can rotate between frames (even with the same contact normal, if the body rotates in-plane). Consider zeroing tangential warm-start when the tangent basis changes significantly.

### 8. PositionCorrector Only Uses Normal Constraints

`PositionCorrector::correctPositions()` operates on `constraintPtrs_` which will now include FrictionConstraint entries. Position correction should only use normal (ContactConstraint) rows. The corrector may need filtering.

### 9. ECOS Test Coverage

Removing ECOS removes ~50 test cases. The custom solver validation suite (ticket 0052e) must cover equivalent scenarios to avoid regression.

---

## 9. FrictionConeSpec Reuse

### Analysis

`FrictionConeSpec` stores:
- `numContacts: int`
- `frictionCoefficients: vector<double>`
- `normalIndices: vector<int>`

Its useful data is the per-contact friction coefficient and contact-normal index mapping. However:

1. It has `#include <ecos/ecos.h>` for the `idxint` type (only used in `getConeSizes()`)
2. `getConeSizes()` returns `vector<idxint>` -- ECOS-specific
3. `normalIndices` stores the index into the lambda vector where each contact's normal force lives -- this is useful for any cone solver

### Recommendation

**Do not reuse FrictionConeSpec directly**. Instead, create a lightweight replacement without ECOS dependency:

```cpp
struct FrictionSpec {
  int numContacts;
  std::vector<double> frictionCoefficients;  // μ per contact
};
```

The normal index mapping (3i for contact i) is implicit in the interleaved ordering convention and does not need to be stored explicitly.

---

## Summary of Integration Effort

| Area | Complexity | Notes |
|------|-----------|-------|
| Remove ECOS files | Low | Delete 10 source files, 4 test files, update CMakeLists |
| Remove ECOS dependency | Low | conanfile.py + CMakeLists.txt |
| New ConeProjection | Medium | Self-contained math, well-defined interface |
| New FrictionConeSolver | High | Core algorithmic work, Newton solver + convergence |
| Modify ConstraintSolver | Medium | Replace dispatch, fix extractBodyForces, handle multi-row J |
| Modify CollisionPipeline | Medium | Add FrictionConstraint creation, expand cache |
| Modify ContactCache | Low | 1→3 values per contact |
| Add friction to assets | Low | New property on AssetInertial/AssetEnvironment |
| Validation suite | Medium | Replace ECOS test coverage |
