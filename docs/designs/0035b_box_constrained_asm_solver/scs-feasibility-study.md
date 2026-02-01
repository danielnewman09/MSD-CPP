# SOCP Solver Feasibility Study: Exact Coulomb Cone Friction Solver

**Date**: 2026-01-31 (updated: 2026-01-31)
**Ticket**: 0035b_box_constrained_asm_solver
**Purpose**: Evaluate feasibility of using SOCP solvers (SCS and ECOS) for exact Coulomb cone friction constraints vs box-LCP approximation

---

## Executive Summary

**Question**: Should we use an SOCP solver (SCS or ECOS) to solve the exact Coulomb friction cone as a second-order cone program, or use the box-LCP approximation with the existing Active Set Method?

**Quick comparison**:

| Criterion | SCS (SOCP) | ECOS (SOCP) | Box-LCP (ASM) |
|-----------|------------|-------------|---------------|
| **Accuracy** | Exact (0% error) | Exact (0% error) | 29% under-approximation (per M3) |
| **Algorithm** | ADMM (first-order) | Interior-point (second-order) | Active set pivot |
| **Dependency** | External (C, ~15K LOC) | External (C, ~4K LOC) | In-house (C++, extends existing ASM) |
| **Performance** | 10-30 ADMM iterations | 5-15 IP iterations (superlinear) | 2-5 outer × 5-20 inner ASM iterations |
| **Determinism** | Non-deterministic | Near-deterministic (IP path-following) | Deterministic (finite ASM termination) |
| **Small problems** | Overhead from ADMM setup | Efficient (designed for embedded) | Very efficient (dense LLT) |
| **Licensing** | MIT | GPLv3 | N/A (in-house) |
| **Build system** | Requires CMake/Conan | Requires CMake/Conan | Existing codebase |

**Recommendation**: See "Recommendation" section below.

---

## Background

### Coulomb Friction Cone

The exact Coulomb friction constraint is a **second-order cone constraint**:
$$
\|\boldsymbol{\lambda}_t\| \leq \mu \lambda_n
$$

where:
- $\boldsymbol{\lambda}_t = [\lambda_{t_1}, \lambda_{t_2}]^\top$ is the tangential friction force (2D)
- $\lambda_n$ is the normal force
- $\mu$ is the friction coefficient
- $\|\cdot\|$ is the Euclidean norm

This is a **non-linear** constraint (quadratic cone), which cannot be solved directly by linear complementarity problem (LCP) solvers like our Active Set Method.

### Current Approach (Box-LCP)

The current design (M5 Option A) approximates the circular cone with a **square box**:
$$
|\lambda_{t_1}| \leq \frac{\mu}{\sqrt{2}} \lambda_n \quad \text{and} \quad |\lambda_{t_2}| \leq \frac{\mu}{\sqrt{2}} \lambda_n
$$

This is a **polyhedral approximation** (linear inequalities) solvable by LCP solvers. The scaling factor $1/\sqrt{2}$ makes the box inscribe the circle, producing a **conservative approximation** (under-estimates friction by up to 29% along diagonal directions per M3).

### Alternative Approach (SCS SOCP)

Use a **second-order cone programming (SOCP)** solver to handle the exact Coulomb cone. SCS (Splitting Conic Solver) is a state-of-the-art open-source SOCP solver specifically designed for this class of problems.

---

## SCS Library Overview

### What is SCS?

**Splitting Conic Solver (SCS)** is a first-order convex optimization solver that solves problems with:
- Linear objectives
- Linear equality and inequality constraints
- **Conic constraints**: second-order cone (Lorentz cone), positive semidefinite cone, exponential cone

**Algorithm**: Operator splitting (Douglas-Rachford splitting, alternating direction method of multipliers ADMM variant)

**Performance**: Designed for large-scale problems (thousands to millions of variables), prioritizes speed over high precision.

**Project**: https://github.com/cvxgrp/scs
**Authors**: Brendan O'Donoghue, Eric Chu, Neal Parikh, Stephen Boyd (Stanford)
**License**: MIT (permissive, commercial-friendly)
**Language**: C (with Python, MATLAB, Julia, R bindings)
**Maintenance**: Actively maintained (latest release 2024)

### Key Properties

| Property | Value |
|----------|-------|
| **LOC** | ~15,000 lines of C code |
| **Dependencies** | BLAS/LAPACK (linear algebra), optional: GPU acceleration (CUDA) |
| **Platforms** | Linux, macOS, Windows |
| **Precision** | Configurable tolerance (default: 1e-4 relative, 1e-4 absolute) |
| **Determinism** | Non-deterministic (iteration count varies with conditioning) |
| **Convergence** | Guaranteed for convex problems (Coulomb friction is convex LCP) |

---

## ECOS Library Overview

### What is ECOS?

**Embedded Conic Solver (ECOS)** is an interior-point solver for second-order cone programs (SOCPs), designed specifically for embedded and real-time applications. Unlike SCS (first-order ADMM), ECOS uses a **second-order interior-point method** with Nesterov-Todd scaling, providing superlinear convergence.

**Algorithm**: Primal-dual interior-point method with Mehrotra predictor-corrector steps
**Performance**: Optimized for small-to-medium problems (up to hundreds of variables), superlinear convergence
**Project**: https://github.com/embotech/ecos
**Authors**: Alexander Domahidi, Eric Chu, Stephen Boyd (Stanford/embotech)
**License**: GPLv3 (copyleft — **licensing implications for commercial use**)
**Language**: C (with Python, MATLAB, Julia bindings)
**Maintenance**: Mature, stable (latest release 2022, limited recent activity)

### Key Properties

| Property | Value |
|----------|-------|
| **LOC** | ~4,000 lines of C code (much smaller than SCS) |
| **Dependencies** | None beyond standard math library (self-contained) |
| **Platforms** | Linux, macOS, Windows, embedded ARM |
| **Precision** | High accuracy (interior-point: superlinear convergence to 1e-8+) |
| **Determinism** | Near-deterministic (interior-point path is predictable, iteration count stable) |
| **Convergence** | Superlinear for convex SOCP problems, typically 5-15 iterations |
| **Memory** | Pre-allocated, no dynamic allocation after setup (embedded-friendly) |

### ECOS vs SCS: Algorithmic Differences

| Aspect | SCS (ADMM) | ECOS (Interior-Point) |
|--------|------------|----------------------|
| **Order** | First-order (gradient) | Second-order (Hessian) |
| **Convergence rate** | O(1/k) sublinear | Superlinear (quadratic near solution) |
| **Per-iteration cost** | Low (matrix-vector products) | Higher (factorization) |
| **Iteration count** | 10-1000+ (depends on tolerance) | 5-15 (nearly constant) |
| **Accuracy at convergence** | Medium (1e-4 typical) | High (1e-8+ achievable) |
| **Warm starting** | Supported (reduces iterations) | Not supported (IP always starts from center) |
| **Problem scale** | Large-scale (1000+ variables) | Small-to-medium (<500 variables) |
| **Memory model** | Dynamic allocation | Pre-allocated (embedded-safe) |

**Key insight for this project**: ECOS is better suited to our problem scale (3-60 variables for 1-20 contacts) because interior-point methods dominate first-order methods for small dense problems. SCS's advantage (scalability to thousands of variables) is irrelevant here.

### Licensing Consideration

**ECOS uses GPLv3**, which is copyleft:
- Any software linking ECOS must also be distributed under GPLv3
- This may be a **blocking constraint** depending on the MSD-CPP project's licensing model
- If the project is open-source under a GPL-compatible license: no issue
- If the project requires permissive/commercial licensing: **ECOS cannot be used without a commercial license from embotech**
- embotech offers commercial licensing for ECOS — terms would need to be evaluated

**Mitigation**: If GPLv3 is problematic, SCS (MIT license) remains the alternative SOCP solver with no licensing restrictions.

---

## Problem Formulation

### SOCP Formulation for Friction

To solve friction with SCS, we reformulate the contact constraint system as a second-order cone program:

**Decision variables**: $\boldsymbol{\lambda} = [\lambda_{n_1}, \lambda_{t_1}^{(1)}, \lambda_{t_2}^{(1)}, \lambda_{n_2}, \lambda_{t_1}^{(2)}, \lambda_{t_2}^{(2)}, \ldots]^\top$ (3 per contact)

**Objective**:
$$
\min_{\boldsymbol{\lambda}} \quad \frac{1}{2} \boldsymbol{\lambda}^\top \mathbf{A} \boldsymbol{\lambda} - \mathbf{b}^\top \boldsymbol{\lambda}
$$

where:
- $\mathbf{A} = \mathbf{J} \mathbf{M}^{-1} \mathbf{J}^\top$ is the effective mass matrix (symmetric positive semidefinite)
- $\mathbf{b}$ is the RHS with restitution and Baumgarte terms

**Constraints** (per contact $i$):
1. **Normal force non-negativity**: $\lambda_{n_i} \geq 0$ (linear inequality)
2. **Friction cone**: $\|[\lambda_{t_1}^{(i)}, \lambda_{t_2}^{(i)}]^\top\| \leq \mu_i \lambda_{n_i}$ (second-order cone)

**SCS standard form**:
$$
\begin{aligned}
\min_x \quad & c^\top x \\
\text{s.t.} \quad & Ax + s = b \\
& s \in \mathcal{K}
\end{aligned}
$$

where $\mathcal{K}$ is a Cartesian product of cones (zero cone for equality, non-negative cone for inequality, second-order cone for friction).

**Conversion**: Our quadratic objective requires conversion to SCS's linear form via epigraph reformulation:
1. Introduce auxiliary variable $t$
2. Minimize $t$ subject to $\frac{1}{2} \boldsymbol{\lambda}^\top \mathbf{A} \boldsymbol{\lambda} - \mathbf{b}^\top \boldsymbol{\lambda} \leq t$
3. Reformulate quadratic constraint as rotated second-order cone

This is **standard SOCP preprocessing** but adds complexity to the formulation.

---

## Integration Analysis

### Build System Integration

**Requirements**:
1. Add SCS as Conan dependency in `conanfile.py`
2. Check if SCS is available in ConanCenter (it is: `scs/3.2.4` available)
3. Link against SCS library in `msd-sim/CMakeLists.txt`

**CMakeLists.txt changes**:
```cmake
find_package(scs REQUIRED)
target_link_libraries(msd-sim PRIVATE scs::scs)
```

**Conanfile.py changes**:
```python
def requirements(self):
    # ... existing dependencies ...
    self.requires("scs/3.2.4")
```

**Complexity**: **Low** — SCS is available in ConanCenter, standard CMake integration

**Build time impact**: SCS is a compiled library (~15K LOC C code), adds ~5-10 seconds to clean build on modern hardware.

---

### API Integration

**SCS API** (simplified):
```c
// Problem data
ScsData data = {
    .m = num_constraints,
    .n = num_variables,
    .A = constraint_matrix,  // Sparse CSC format
    .b = rhs_vector,
    .c = objective_vector
};

// Cone specification
ScsCone cone = {
    .q = second_order_cone_sizes,  // Array of cone dimensions
    .qsize = num_soc_constraints,
    .l = num_linear_inequalities
};

// Settings
ScsSettings settings = {
    .eps_abs = 1e-6,
    .eps_rel = 1e-6,
    .max_iters = 10000
};

// Solve
ScsSolution *sol = NULL;
ScsInfo info;
scs(&data, &cone, &settings, &sol, &info);

// Extract solution
double *lambda = sol->x;
int status = info.status;  // SCS_SOLVED, SCS_INFEASIBLE, etc.
int iterations = info.iter;

scs_free_solution(sol);
```

**Complexity**: **Medium** — Requires:
1. Convert Eigen matrices to SCS sparse CSC format
2. Formulate cone constraints (cone sizes, ordering)
3. Handle epigraph reformulation for quadratic objective
4. Extract solution and check convergence status

**Code estimate**: ~200-300 lines of glue code to interface SCS with existing ConstraintSolver

---

### Data Structure Conversion

**Challenge**: SCS expects sparse compressed sparse column (CSC) matrices, but our ASM uses dense Eigen matrices.

**Current workflow** (ASM):
```cpp
Eigen::MatrixXd A = assembleContactEffectiveMass(...);  // Dense C x C
Eigen::VectorXd b = assembleContactRHS(...);
Eigen::VectorXd lambda = solveActiveSet(A, b, numContacts).lambda;
```

**SCS workflow** (required):
```cpp
Eigen::MatrixXd A_dense = assembleContactEffectiveMass(...);
Eigen::VectorXd b = assembleContactRHS(...);

// Convert to sparse
Eigen::SparseMatrix<double> A_sparse = A_dense.sparseView();

// Convert to SCS CSC format
ScsMatrix A_scs = eigenSparseToSCS(A_sparse);

// Build cone specification
ScsCone cone = buildFrictionCone(numContacts, frictionCoefficients);

// Solve
ScsData data = {A_scs, b.data(), objectiveVector.data(), ...};
ScsSolution *sol = scs(&data, &cone, &settings);

Eigen::VectorXd lambda = Eigen::Map<Eigen::VectorXd>(sol->x, sol->n);
```

**Complexity**: **Medium** — Requires sparse matrix conversion utilities, cone specification builder

**Performance overhead**: For small problems (C < 20 contacts), dense-to-sparse conversion overhead likely dominates solve time. SCS is designed for large-scale sparse problems (1000+ variables), not small dense systems.

---

## Performance Comparison

### Theoretical Complexity

| Solver | Algorithm | Complexity per Iteration | Expected Iterations | Total Complexity |
|--------|-----------|-------------------------|---------------------|------------------|
| **Box-LCP (ASM)** | Active Set Method | O((3C)³) LLT decomposition | 5-20 inner (per outer), 2-5 outer | O(C³) × 100-200 |
| **SCS (SOCP)** | Douglas-Rachford splitting | O(nnz(A) × C) matrix-vector product | 10-30 (typical), 100-1000 (worst-case) | O(C⁴) × 10-30 (dense A) |
| **ECOS (SOCP)** | Interior-point (Mehrotra) | O((3C)³) factorization | 5-15 (nearly constant) | O(C³) × 5-15 |

**Key observations**:
- **Small problems (C < 10)**: ASM likely fastest due to tight LLT decomposition, no conversion overhead. ECOS competitive (similar factorization cost, fewer iterations than SCS)
- **Large problems (C > 50)**: SCS likely faster if A is sparse (contact Jacobians are sparse), but this project targets C < 20
- **Dense systems**: ASM exploits dense structure via BLAS3. ECOS also handles dense well (interior-point with dense factorization). SCS treats as generic sparse (slower)
- **ECOS sweet spot**: Small-to-medium dense problems (exactly our use case) — fewest iterations with superlinear convergence

### Iteration Count Sensitivity

**ASM (box-LCP)**:
- **Inner iterations**: Bounded by $2 \times 3C$ active set changes (Cottle theorem), typical 5-20
- **Outer iterations**: Empirical (needs prototyping), expected 2-5 based on physics engine experience
- **Total**: ~10-100 constraint solves (LLT decompositions)

**SCS (SOCP)**:
- **Typical**: 10-30 iterations for medium-accuracy (1e-4 tolerance)
- **High-accuracy**: 50-200 iterations for tight tolerance (1e-6)
- **Ill-conditioned**: 100-1000 iterations for high mass ratios, stiff contacts
- **Non-deterministic**: Iteration count varies with problem conditioning, initial guess

**ECOS (SOCP)**:
- **Typical**: 5-15 iterations (nearly constant across problem sizes)
- **High-accuracy**: Same 5-15 iterations (interior-point achieves 1e-8+ without additional iterations)
- **Ill-conditioned**: 10-25 iterations (interior-point handles conditioning better than first-order methods)
- **Near-deterministic**: Iteration count varies by 1-3 across similar problems (path-following is stable)

**Predictability**: ASM is fully deterministic (finite termination). ECOS is near-deterministic (interior-point iteration count is stable). SCS is least predictable (may require iteration cap for real-time use).

### Benchmark Estimates (Projected)

Based on published SCS benchmarks and ASM performance from ticket 0034:

| Scenario | ASM (box-LCP) | SCS (SOCP) | ECOS (SOCP) | Winner |
|----------|---------------|------------|-------------|--------|
| **1 contact** | ~10 μs (1 outer × 5 inner × 2 μs LLT) | ~50 μs (20 iters × 2.5 μs ADMM step) | ~15 μs (8 iters × 2 μs factorization) | ASM ≈ ECOS |
| **5 contacts** | ~200 μs (3 outer × 10 inner × 6 μs LLT) | ~300 μs (20 iters × 15 μs) | ~80 μs (10 iters × 8 μs) | ECOS (2.5x faster than ASM) |
| **10 contacts** | ~800 μs (4 outer × 15 inner × 13 μs LLT) | ~1000 μs (25 iters × 40 μs) | ~200 μs (12 iters × 17 μs) | ECOS (4x faster than ASM) |
| **20 contacts** | ~5 ms (5 outer × 20 inner × 50 μs LLT) | ~4 ms (30 iters × 130 μs) | ~800 μs (14 iters × 57 μs) | ECOS (5-6x faster) |

**Caveat**: These are **rough estimates**. Actual performance requires benchmarking. ECOS estimates are based on published interior-point benchmarks for similar problem sizes.

**Verdict**: For this project's target scale (C < 10 typical, C < 20 max):
- **ECOS is potentially the fastest** for multi-contact scenarios (5+ contacts) — interior-point with superlinear convergence means fewer total factorizations than ASM's outer×inner iteration product
- **ASM is competitive for 1-2 contacts** — minimal overhead, no conversion cost
- **SCS is the slowest for small problems** — ADMM overhead and high iteration count for this problem scale
- **All estimates need empirical validation** — the prototype phase should benchmark all three

---

## Accuracy Comparison

### Coulomb Cone Approximation Error

| Approach | Max Error | Error Type | Physical Interpretation |
|----------|-----------|------------|-------------------------|
| **Box-LCP** | 29% | Under-approximation | Friction force limited to 71% of true limit along diagonal directions |
| **SCS (exact cone)** | 0% (up to solver tolerance ~1e-4) | None | Exact Coulomb cone enforcement |

**Visual comparison** (friction cone cross-section):
- **Exact cone**: Circle (radius $\mu \lambda_n$)
- **Box approximation**: Square inscribed in circle (side length $\sqrt{2} \mu \lambda_n$)
- **Box approximation (scaled)**: Square with side $\sqrt{2} \mu \lambda_n / \sqrt{2} = \mu \lambda_n$, inscribed circle has radius $\mu \lambda_n / \sqrt{2}$

**Impact on simulation**:
- **Box-LCP**: Objects may slip slightly more than physically correct (friction under-estimated)
- **SCS**: Physically accurate friction (no spurious slipping)

**Practical significance**:
- For **games/interactive simulation**: 29% error often acceptable (visual plausibility more important than precision)
- For **engineering analysis**: Exact cone preferable (precision matters for torque calculations, stability analysis)
- For **robotics/manipulation**: Exact cone may be required (grasp force calculations depend on accurate friction)

**Upgrade path**: Box-LCP can be upgraded to pyramidal (4 coupled constraints) or octagonal (8 constraints, <2% error) without changing solver framework. SCS is the "ultimate" accuracy.

---

## Determinism and Convergence

### ASM (Box-LCP)

**Convergence guarantee**: **Finite termination** for monotone LCP (positive semidefinite effective mass matrix)
- **Proof**: Cottle, Pang, Stone theorem (active set method terminates in finite iterations for monotone LCP)
- **Iteration bound**: $\leq 2 \times 3C$ active set changes (empirically much lower, 5-20 typical)

**Determinism**: **Fully deterministic**
- Same input → same output
- Same iteration count
- No randomness, no initial guess sensitivity (beyond numerical precision)

**Outer iteration**: **Empirical convergence** (not formally proven, but physically motivated)
- Friction bounds converge as $\lambda_n$ stabilizes
- Expected 2-5 iterations based on physics engine experience (needs validation)

### SCS (SOCP)

**Convergence guarantee**: **Asymptotic convergence** for convex problems
- **Proof**: Douglas-Rachford splitting converges for convex optimization (Eckstein & Bertsekas, 1992)
- **Rate**: O(1/k) convergence (residual decreases inversely with iteration count)

**Determinism**: **Non-deterministic**
- Iteration count varies with problem conditioning
- May require different iteration counts for similar inputs with slight perturbations
- Initial guess affects convergence speed (warm starting helps)

**Iteration cap**: **Required for real-time use**
- Must set `max_iters` (e.g., 100-1000) to prevent unbounded solve time
- If cap reached, solution is approximate (may violate constraints slightly)

**Trade-off**: SCS provides flexibility (trade accuracy for speed) but loses determinism.

### ECOS (SOCP)

**Convergence guarantee**: **Polynomial-time convergence** for SOCP problems
- **Proof**: Primal-dual interior-point methods converge in $O(\sqrt{n} \log(1/\epsilon))$ iterations (Nesterov & Nemirovski, 1994)
- **Rate**: Superlinear (quadratic near solution due to Mehrotra predictor-corrector)
- **Practical**: 5-15 iterations for problems up to hundreds of variables

**Determinism**: **Near-deterministic**
- Interior-point path-following produces consistent iteration counts (±1-3 across similar problems)
- No randomness in algorithm (unlike some ADMM variants)
- Slight sensitivity to numerical conditioning (but much more stable than SCS)

**Warm starting**: **Not supported** (interior-point methods always start from analytic center)
- This is a limitation vs SCS (which supports warm starting)
- For friction, this means no benefit from previous timestep's solution
- However, the low iteration count (5-15) makes warm starting less critical

**Memory model**: **Pre-allocated** — no dynamic allocation after initial setup
- All memory allocated during `ECOS_setup()`
- Solve phase uses only pre-allocated workspace
- Suitable for real-time and embedded systems

**Trade-off**: ECOS provides high accuracy and near-determinism with excellent small-problem performance, but GPLv3 licensing may be a constraint.

---

## Code Complexity Comparison

### Box-LCP (ASM Extension)

**New code required**:
1. `solveActiveSetWithBoxBounds()` — ~150 lines (extend existing ASM logic to three-state tracking)
2. `computeFrictionBounds()` — ~30 lines (extract $\lambda_n$, compute bounds)
3. `checkOuterConvergence()` — ~20 lines (compare normal forces)
4. Outer iteration loop in `solveWithContacts()` — ~50 lines
5. `BoxBounds` struct — ~40 lines (bound storage, setters)
6. `BoundState` enum — ~5 lines

**Total**: ~300 lines of C++ in existing ConstraintSolver class

**Dependencies**: None (uses existing Eigen)

**Maintainability**: Medium complexity (three-state logic is subtle, needs careful testing)

### SCS (SOCP)

**New code required**:
1. SCS data structure conversion — ~100 lines (Eigen → SCS CSC format)
2. Cone specification builder — ~80 lines (map friction constraints to SOC cone sizes)
3. Epigraph reformulation for quadratic objective — ~60 lines (augment problem with auxiliary variable)
4. SCS solve wrapper — ~50 lines (call SCS API, extract solution)
5. Error handling and convergence checking — ~40 lines (check SCS status codes)

**Total**: ~330 lines of C++ glue code + 15K LOC external SCS library

**Dependencies**: SCS library (external C code, ~15K LOC)

**Maintainability**: Medium complexity (formulation is tricky, but encapsulated in conversion layer)

**Learning curve**: Requires understanding SOCP formulation, conic constraints, SCS API

### ECOS (SOCP)

**New code required**:
1. ECOS data structure conversion — ~80 lines (Eigen → ECOS sparse format, similar to SCS)
2. Cone specification builder — ~60 lines (simpler API than SCS — cone sizes array)
3. ECOS solve wrapper — ~40 lines (call ECOS_setup/ECOS_solve/ECOS_cleanup)
4. Error handling — ~30 lines (check exit codes: ECOS_OPTIMAL, ECOS_INFEASIBLE, etc.)

**Total**: ~210 lines of C++ glue code + ~4K LOC external ECOS library

**Dependencies**: ECOS library (external C code, ~4K LOC — much smaller than SCS)

**Maintainability**: Lower complexity than SCS (simpler API, no epigraph reformulation for QP — ECOS handles SOCP directly)

**Learning curve**: Similar to SCS (SOCP formulation, cone constraints) but simpler API

**Advantage over SCS**: ECOS's API is more straightforward — it directly accepts second-order cone specifications without requiring the epigraph reformulation that SCS needs for quadratic objectives

---

## Licensing and Distribution

### SCS License

**License**: MIT License
- **Permissive**: Commercial use allowed, no copyleft restrictions
- **Attribution**: Must include copyright notice in distributions
- **Compatible**: With MSD-CPP project (assuming permissive licensing)

**Full text**: https://github.com/cvxgrp/scs/blob/master/LICENSE

**Verdict**: **No licensing concerns** for commercial or open-source use.

### Distribution Impact

**With SCS**:
- Project gains a new binary dependency (~500 KB SCS shared library)
- Users must install SCS via Conan (transparent if using Conan workflow)
- Slightly larger build times (~5-10 seconds for SCS compilation)

**Without SCS**:
- No external dependencies beyond existing (Eigen, Catch2, etc.)
- Fully self-contained constraint solver

**Verdict**: SCS dependency is **manageable** but adds external complexity.

---

## Risk Analysis

| Risk | Box-LCP (ASM) | SCS (SOCP) | ECOS (SOCP) |
|------|---------------|------------|-------------|
| **Doesn't converge** | Low (finite termination for monotone LCP) | Medium (may need iteration cap, convergence rate varies) | Low (polynomial-time convergence guarantee) |
| **Too slow for real-time** | Low (O(C³), 10-100 solves) | Medium (10-30 ADMM iterations, may be slower for small C) | Low (5-15 IP iterations, designed for embedded) |
| **Maintenance burden** | Low (in-house code, ~300 lines) | Medium (external library, API changes, version updates) | Low-Medium (~4K LOC, stable but less actively maintained) |
| **Accuracy insufficient** | Medium (29% error, may need upgrade to pyramid/octagon) | Low (exact cone, solver tolerance only limit) | Low (exact cone, interior-point achieves 1e-8+) |
| **Integration complexity** | Low (natural ASM extension) | Medium (sparse conversion, cone formulation, epigraph reformulation) | Low-Medium (sparse conversion, cone formulation, simpler API) |
| **Dependency fragility** | None (no external deps) | Low (SCS is mature, actively maintained, but external) | Medium (mature but limited recent activity since 2022) |
| **Licensing** | None (in-house) | None (MIT — permissive) | **High (GPLv3 — copyleft, may require commercial license)** |

---

## Decision Criteria

### When to Choose Box-LCP (ASM)

**Favor ASM if**:
1. **Performance is critical** and contact count is low (C < 10 typical)
2. **Determinism is required** (same input → same output, same iteration count)
3. **No external dependencies** is a project constraint
4. **29% friction error is acceptable** (games, interactive simulation, qualitative visualization)
5. **Prototyping/MVP**: Want to ship friction support quickly, can upgrade accuracy later

**Risks**:
- Under-approximated friction may cause slight over-slipping (visually noticeable in slow-motion)
- May need to upgrade to pyramidal/octagonal approximation later (adds complexity)

### When to Choose SCS (SOCP)

**Favor SCS if**:
1. **Accuracy is critical** (engineering analysis, robotics, quantitative validation)
2. **Contact count may grow** (C > 20), where SCS's O(nnz) scaling helps if Jacobians are sparse
3. **External dependencies are acceptable** (Conan workflow already in use)
4. **Non-determinism is tolerable** (iteration count variation okay)
5. **Future extensibility**: Want to support other conic constraints (PSD cones, exponential cones)

**Risks**:
- SCS may be slower for small problems (C < 10) due to overhead
- Non-deterministic iteration count complicates real-time guarantees
- External dependency adds maintenance burden (API changes, version updates)

### When to Choose ECOS (SOCP)

**Favor ECOS if**:
1. **Small-to-medium problem scale** (C < 20 contacts) — interior-point excels here
2. **High accuracy needed** with predictable performance — superlinear convergence in 5-15 iterations
3. **Near-deterministic behavior** is valued — IP iteration count is stable
4. **GPLv3 licensing is acceptable** — or commercial license can be obtained from embotech
5. **Embedded/real-time** considerations — pre-allocated memory, no dynamic allocation during solve

**Risks**:
- **GPLv3 licensing** is the primary concern — copyleft may be incompatible with project requirements
- Limited recent maintenance (last release 2022) — though the codebase is mature and stable
- No warm starting — every solve starts fresh (mitigated by low iteration count)
- Smaller community than SCS

---

## Recommendation

### Recommended Approach: **Staged Implementation**

**Phase 1: Implement Box-LCP (ASM) — Immediate**
- **Rationale**:
  - Fastest path to working friction (natural extension of existing ASM)
  - Zero external dependencies (project already uses Eigen)
  - Deterministic, predictable performance for real-time use
  - 29% error acceptable for MVP (games/interactive simulation)
  - Provides baseline for SCS comparison
- **Timeline**: 2-3 weeks (design → prototype → implement → test)
- **Risk**: Low (proven approach, physics engines use box-LCP successfully)

**Phase 2: Prototype SOCP Solvers (SCS + ECOS) — Validation**
- **Rationale**:
  - Measure actual performance vs theoretical estimates for both SOCP solvers
  - Validate accuracy improvement (exact cone vs 29% error)
  - Assess integration complexity (sparse conversion, formulation)
  - Compare wall-clock time for representative workloads (1, 5, 10 contacts)
  - Determine if ECOS licensing is compatible with project requirements
- **Timeline**: 1-2 weeks (integrate both, benchmark, compare)
- **Deliverable**: Benchmark report comparing ASM vs SCS vs ECOS on:
  - Single contact (stick/slip transition)
  - Multi-contact (5, 10, 20 contacts)
  - High mass ratio (1000:1)
  - Iteration count distribution (SCS vs ECOS convergence behavior)
  - Wall-clock time (including setup/conversion overhead)
  - Accuracy (friction force error vs ground truth)
  - Memory usage (pre-allocated ECOS vs dynamic SCS)

**Phase 3: Decision Point**
- **If ECOS excels and GPLv3 is acceptable**:
  - Adopt ECOS as production solver (best accuracy + best small-problem performance)
  - Keep ASM as dependency-free fallback
- **If ECOS licensing is prohibitive but SCS performs well**:
  - Adopt SCS as production solver (exact accuracy, MIT license)
  - Keep ASM as fallback for edge cases
- **If both SOCP solvers are too slow or too complex**:
  - Keep ASM, document 29% error as known limitation
  - Plan future upgrade to pyramidal/octagonal approximation (4-8 constraints per contact, <2% error)

**If neither SOCP solver meets requirements**:
- Keep ASM box-LCP, document limitation
- Future work: Pyramidal (4 constraints, 29% error) → Octagonal (8 constraints, 2% error) → SOCP (exact)

### Why Not SOCP Immediately?

1. **Unproven performance for this scale**: SCS is designed for large-scale sparse problems (1000+ variables). ECOS is designed for small-medium problems but still needs validation on our specific problem structure. ASM may be faster for 1-2 contacts.

2. **Integration risk**: Sparse conversion, cone formulation add complexity. Better to have working friction (box-LCP) first, then upgrade if accuracy demands it.

3. **Determinism value**: ASM provides deterministic, finite termination. Valuable for debugging, reproducibility, and real-time guarantees. ECOS is near-deterministic. SCS is non-deterministic.

4. **Licensing uncertainty**: ECOS's GPLv3 may be incompatible with project requirements. This needs resolution before committing to ECOS as the production solver.

5. **Incremental accuracy path**: Box (29% error) → Pyramid (29% error, 4 constraints) → Octagon (2% error, 8 constraints) → SOCP (exact). Can stop at "good enough" accuracy without SOCP complexity.

### Counterargument: Why SOCP First?

**If the project prioritizes**:
- **Accuracy over speed**: Engineering analysis, quantitative validation require exact cone
- **Future extensibility**: Plan to add other conic constraints (e.g., torque limits, polytope approximations)
- **One-time implementation**: Don't want to revisit solver design later

**Then**: Implement ECOS (if licensing allows) or SCS directly, skip box-LCP MVP.

**ECOS is the stronger candidate** for SOCP-first because:
- Interior-point method is better suited to our problem scale
- Near-deterministic iteration count
- Pre-allocated memory model
- Potentially faster than ASM for 5+ contacts

**However**: This requires accepting:
- GPLv3 licensing constraint (ECOS) or non-determinism (SCS)
- External dependency maintenance
- No warm starting (ECOS)

---

## Proposed Next Steps

### Option A: Box-LCP First + SOCP Prototype (Recommended)

1. **Proceed with current design** (`design.md`, box-LCP approach)
2. **Implement and validate** box-LCP (ticket 0035b as designed)
3. **Measure accuracy** in practice (does 29% error cause visible issues?)
4. **Prototype both SCS and ECOS** in parallel (separate branch, measure performance)
5. **Resolve ECOS licensing** (GPLv3 compatibility with project)
6. **Compare** at end of ticket 0035b:
   - If ECOS excels and licensing is clear → Adopt ECOS
   - If SCS is better or ECOS licensing is blocked → Evaluate SCS
   - If ASM is sufficient → Keep ASM, close SOCP investigation

**Risk**: Low (working friction delivered regardless of SOCP outcome)

### Option B: ECOS First (If Licensing Permits)

1. **Resolve ECOS GPLv3 licensing** immediately (blocking question)
2. **If clear**: Revise design to use ECOS instead of box-LCP
3. **Integrate ECOS** (Conan, CMake, API wrapper — simpler than SCS)
4. **Formulate SOCP** (cone constraints, sparse conversion)
5. **Implement and validate** ECOS solver
6. **Keep ASM as fallback** if ECOS is too slow for 1-2 contact cases

**Risk**: Medium (licensing may block, performance unvalidated)

### Option C: SCS First (License-Safe SOCP)

1. **Revise design** to use SCS instead of box-LCP
2. **Integrate SCS** (Conan, CMake, API wrapper — MIT license, no restrictions)
3. **Formulate SOCP** (cone constraints, epigraph, sparse conversion)
4. **Implement and validate** SCS solver
5. **Measure performance** (may need fallback to ASM if too slow for small problems)

**Risk**: Medium (may discover SCS is too slow for C < 10, requiring fallback to ASM)

### Option D: Parallel Implementation with All Three

1. **Implement all three** (ASM, SCS, ECOS) behind a solver interface
2. **Provide solver selection** at runtime (`--solver=asm`, `--solver=scs`, `--solver=ecos`)
3. **Compare** on real workloads across all dimensions
4. **Choose default** based on benchmarks

**Risk**: Low (hedged bet) but **resource-intensive** (3x implementation work)

---

## Conclusion

**For ticket 0035b**, recommend:

**Immediate action**: Proceed with **box-LCP (ASM) implementation** as designed
- Natural extension of existing ASM solver
- Deterministic, proven approach
- No external dependencies
- 29% error acceptable for MVP

**Parallel investigation**: Prototype both SCS and ECOS in separate branch
- Measure actual performance vs theoretical estimates for both SOCP solvers
- Validate accuracy improvement (exact cone vs 29% box approximation)
- Assess integration complexity for each
- **Resolve ECOS GPLv3 licensing** — this is a potential blocker

**ECOS is the more promising SOCP candidate** for this project because:
- Interior-point method matches our problem scale (small dense systems)
- Near-deterministic iteration count (5-15 iterations)
- Potentially faster than ASM for 5+ contacts
- Smaller codebase (~4K LOC vs ~15K LOC)
- Pre-allocated memory model (embedded-friendly)

**But GPLv3 licensing is the key risk** — if copyleft is incompatible, SCS (MIT) is the fallback SOCP option.

**Decision point**: After box-LCP implementation and SOCP prototypes
- If ECOS excels and licensing is clear → Adopt ECOS
- If ECOS licensing blocks but SCS performs well → Adopt SCS
- If ASM is sufficient (error tolerable, performance better) → Keep ASM
- If neither SOCP solver meets needs → Consider pyramidal/octagonal approximation (middle ground)

**Rationale**: De-risk by delivering working friction (ASM) while investigating higher-accuracy alternatives (SCS + ECOS) in parallel. Avoid committing to SOCP without performance validation and licensing resolution.

---

## Appendix A: SCS API Example

**Complete friction solver with SCS** (simplified pseudocode):

```cpp
#include <scs.h>

// Convert Eigen problem to SCS format
ScsData* createSCSData(const Eigen::MatrixXd& A,
                       const Eigen::VectorXd& b,
                       int numContacts,
                       const std::vector<double>& frictionCoefficients) {
    // Epigraph reformulation: min t s.t. 1/2 λ^T A λ - b^T λ <= t
    // Becomes: min [0, ..., 0, 1]^T [λ, t] s.t. rotated SOC constraint

    int n = 3 * numContacts;  // Variables: λ_n, λ_t1, λ_t2 per contact
    int m = ... ;  // Constraints: equality + inequality + cone

    // Build constraint matrix (sparse CSC)
    Eigen::SparseMatrix<double, Eigen::ColMajor> A_sparse = ...;

    // Allocate SCS structures
    ScsData *data = (ScsData*)calloc(1, sizeof(ScsData));
    data->m = m;
    data->n = n;
    data->A = convertEigenToSCSMatrix(A_sparse);
    data->b = convertEigenToArray(b);
    data->c = ... ;  // Objective vector

    return data;
}

// Specify cone constraints
ScsCone* createFrictionCone(int numContacts,
                            const std::vector<double>& mu) {
    ScsCone *cone = (ScsCone*)calloc(1, sizeof(ScsCone));

    // Second-order cone sizes: one per contact (3D: [λ_n, λ_t1, λ_t2])
    cone->qsize = numContacts;
    cone->q = (scs_int*)malloc(numContacts * sizeof(scs_int));
    for (int i = 0; i < numContacts; ++i) {
        cone->q[i] = 3;  // Each friction cone is 3D (1 normal + 2 tangential)
    }

    return cone;
}

// Solve friction LCP with SCS
Eigen::VectorXd solveFrictionSCS(const Eigen::MatrixXd& A,
                                  const Eigen::VectorXd& b,
                                  int numContacts,
                                  const std::vector<double>& mu) {
    // Convert problem
    ScsData *data = createSCSData(A, b, numContacts, mu);
    ScsCone *cone = createFrictionCone(numContacts, mu);

    // Configure solver
    ScsSettings settings;
    scs_set_default_settings(&settings);
    settings.eps_abs = 1e-6;
    settings.eps_rel = 1e-6;
    settings.max_iters = 1000;

    // Solve
    ScsSolution *sol = (ScsSolution*)calloc(1, sizeof(ScsSolution));
    ScsInfo info;
    scs(data, cone, &settings, sol, &info);

    // Check convergence
    if (info.status != SCS_SOLVED) {
        // Handle failure: SCS_INFEASIBLE, SCS_UNBOUNDED, SCS_INDETERMINATE, SCS_FAILED
        throw std::runtime_error("SCS failed to solve friction LCP");
    }

    // Extract solution
    Eigen::VectorXd lambda = Eigen::Map<Eigen::VectorXd>(sol->x, sol->n);

    // Cleanup
    scs_free_data_cone(data, cone);
    scs_free_solution(sol);

    return lambda;
}
```

**Complexity**: ~200-300 lines total (conversion utilities + solve wrapper)

---

## Appendix B: References

### SCS Documentation
- **GitHub**: https://github.com/cvxgrp/scs
- **Paper**: O'Donoghue, Chu, Parikh, Boyd (2016), "Conic Optimization via Operator Splitting and Homogeneous Self-Dual Embedding"
- **API docs**: https://www.cvxgrp.org/scs/

### ECOS Documentation
- **GitHub**: https://github.com/embotech/ecos
- **Paper**: Domahidi, Chu, Boyd (2013), "ECOS: An SOCP Solver for Embedded Systems"
- **License**: GPLv3 (https://github.com/embotech/ecos/blob/master/COPYING)
- **Commercial licensing**: Available from embotech (https://embotech.com)

### SOCP Theory
- Boyd & Vandenberghe (2004), *Convex Optimization*, Chapter 4 (Second-Order Cone Programming)
- Lobo, Vandenberghe, Boyd, Lebret (1998), "Applications of Second-Order Cone Programming"
- Nesterov & Nemirovski (1994), *Interior-Point Polynomial Algorithms in Convex Programming* (convergence theory for ECOS)

### Friction Formulation
- Anitescu & Potra (1997), "Formulating Dynamic Multi-Rigid-Body Contact Problems with Friction as Solvable Linear Complementarity Problems"
- Stewart & Trinkle (1996), "An Implicit Time-Stepping Scheme for Rigid Body Dynamics with Coulomb Friction"

### Box-LCP References (Comparison)
- Catto (2009), Box2D friction solver documentation
- Bullet Physics: btSequentialImpulseConstraintSolver source code
- Cottle, Pang, Stone (2009), *The Linear Complementarity Problem* (Chapter 11: Box-constrained LCP)

---

**End of Feasibility Study**
