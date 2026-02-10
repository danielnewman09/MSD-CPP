# Friction Contact Solver Research

## Context

The MSD-CPP project currently uses ECOS for solving friction contact constraints formulated as an SOCP. The problem involves per-contact 3D Coulomb friction cones with typically 1-50 contacts in spacecraft docking scenarios. As documented in ticket `0035d1`, the current ECOS formulation is incorrectly set up as a feasibility problem rather than a proper QP-in-SOCP-form. This document evaluates solver alternatives before committing to fixing the ECOS path or replacing it entirely.

### Problem Formulation

The friction contact problem is naturally a **Quadratic Program with Second-Order Cone constraints**:

```
min  (1/2) λᵀ A λ - bᵀ λ
s.t. ‖λ_tᵢ‖ ≤ μᵢ λ_nᵢ   (friction cones)
     λ_nᵢ ≥ 0              (normal bounds)
```

This distinction matters because different solvers handle different subsets of this formulation natively.

---

## Solver Evaluations

### 1. ECOS (Current Solver)

- **Type**: Interior-point method (Mehrotra predictor-corrector with Nesterov-Todd scaling)
- **Language**: ANSI C (~4,000 LOC)
- **License**: GPLv3 (copyleft)

| Criterion | Assessment |
|-----------|-----------|
| 3D Coulomb friction cones | Via epigraph lifting only — cannot handle quadratic objectives natively |
| Small problem performance | Excellent (5-15 iterations, superlinear convergence) |
| C++ integration | Good (C API, already integrated via Conan) |
| License | **GPLv3 — copyleft concern** |
| Numerical robustness | Good for well-posed problems |
| Maintenance | **Effectively unmaintained.** CVXPY dropped it as default in v1.5 (2024) and removed it as a dependency in v1.6 |
| Warm starting | **Not supported** — interior-point always starts from analytic center |

**Assessment**: Poor long-term choice. GPLv3 licensing, no warm starting, unmaintained, and its inability to handle quadratic objectives natively forces error-prone epigraph lifting (which is the root cause of the current formulation bug).

---

### 2. Clarabel

- **Type**: Interior-point method (novel homogeneous embedding)
- **Language**: Rust (primary), with C/C++ interface using Eigen
- **License**: Apache 2.0
- **Website**: [clarabel.org](https://clarabel.org/)

| Criterion | Assessment |
|-----------|-----------|
| 3D Coulomb friction cones | **Native QP + SOC support — no epigraph lifting needed** |
| Small problem performance | Very good (interior-point, superlinear convergence) |
| C++ integration | Good (C/C++ interface accepts Eigen matrices directly) |
| License | **Apache 2.0 — fully permissive** |
| Numerical robustness | Excellent (homogeneous embedding with infeasibility detection) |
| Maintenance | **Very actively maintained** (Oxford Control group, CVXPY's chosen ECOS replacement) |
| Warm starting | Limited (interior-point fundamental limitation) |

**Assessment**: Strongest external solver replacement for ECOS. Handles the exact problem formulation natively, permissive license, actively maintained. Main drawback is requiring Rust toolchain in the build system.

---

### 3. OSQP

- **Type**: ADMM-based QP solver (first-order)
- **Language**: C
- **License**: Apache 2.0
- **Website**: [osqp.org](https://osqp.org/)

| Criterion | Assessment |
|-----------|-----------|
| 3D Coulomb friction cones | **Cannot handle SOC constraints natively** — requires polyhedral approximation (8-sided ≈ 5% error, box ≈ 29% error) |
| Small problem performance | Good for QPs |
| C++ integration | Excellent (pure C, well-documented API) |
| License | **Apache 2.0** |
| Numerical robustness | Good |
| Maintenance | Actively maintained (used in Tesla Autopilot, numerous robotics labs) |
| Warm starting | **Supported** — primal and dual, very effective for time-stepping |

**Assessment**: Good choice if polyhedral friction cone approximation is acceptable. Cannot solve exact Coulomb cones. Warm starting is a significant advantage for simulation stepping.

---

### 4. SCS (Splitting Conic Solver)

- **Type**: ADMM-based operator splitting (first-order)
- **Language**: C (~15,000 LOC)
- **License**: MIT
- **Website**: [github.com/cvxgrp/scs](https://github.com/cvxgrp/scs)

| Criterion | Assessment |
|-----------|-----------|
| 3D Coulomb friction cones | Via epigraph lifting (SCS 3.x added partial quadratic support) |
| Small problem performance | **Poor** — designed for large-scale problems, ADMM overhead dominates at small scale |
| C++ integration | Good (C API, requires BLAS/LAPACK) |
| License | **MIT** |
| Numerical robustness | Moderate (sensitive to conditioning, non-deterministic iteration counts) |
| Maintenance | Actively maintained (Stanford CVX group) |
| Warm starting | **Supported** |

**Assessment**: Poor fit for this problem scale. Strength is large-scale problems (thousands+ variables), which is irrelevant for 1-50 contacts.

---

### 5. MOSEK

- **Type**: Commercial interior-point solver
- **Language**: C API with C++ (Fusion) interface
- **License**: **Commercial (proprietary)**

| Criterion | Assessment |
|-----------|-----------|
| 3D Coulomb friction cones | Excellent native SOCP support |
| Small problem performance | Very good |
| Numerical robustness | Best-in-class (gold standard for conic optimization) |
| Maintenance | Actively maintained (full-time team) |

**Assessment**: Technically excellent but **commercially licensed — not viable for this project**. Useful as a benchmark reference for validating other solvers.

---

### 6. Custom PGS (Projected Gauss-Seidel)

- **Type**: Iterative splitting method (first-order)
- **Language**: Custom C++ implementation (~200-300 LOC)
- **License**: N/A (no external dependency)

This is the approach used by Bullet, PhysX, ODE, Box2D, and Roblox.

| Criterion | Assessment |
|-----------|-----------|
| 3D Coulomb friction cones | Via per-iteration projection onto exact cone |
| Small problem performance | **Fastest** — no matrix factorization, just scalar updates. Sub-10 μs for 1-5 contacts |
| C++ integration | **Best** — no external dependency |
| Numerical robustness | Moderate (first-order convergence, 10-30 iterations typical) |
| Warm starting | **Excellent** — previous timestep impulses as initial guess |

**Assessment**: Simplest to implement, fastest for small problems, best warm starting. Tradeoff is first-order convergence — may exhibit "viscous" contact behavior if iteration count is too low. Sensitive to high mass ratios.

---

### 7. Custom Convex Solver (Newton-based, MuJoCo/Drake-style)

- **Type**: Newton method for convex QP with cone constraints
- **Language**: Custom C++ implementation (~400-600 LOC)
- **License**: N/A (no external dependency)

This is what MuJoCo and Drake (SAP solver) use in production.

| Criterion | Assessment |
|-----------|-----------|
| 3D Coulomb friction cones | **Exact Coulomb cones natively** (QP + SOC) |
| Small problem performance | Excellent (3-8 Newton iterations, quadratic convergence) |
| C++ integration | **Best** — no external dependency |
| Numerical robustness | Very good (requires Hessian factorization and cone projection) |
| Warm starting | Good (Newton converges faster from nearby initial point) |

**Assessment**: Highest quality solution. Handles the exact problem formulation, no external dependencies, proven in production (MuJoCo, Drake). Medium-high implementation effort. Key references: Todorov 2014, Castro et al. 2022 (SAP).

---

### 8. Lemke/Pivoting LCP Solvers

- **Type**: Pivoting (active set) methods
- **Language**: Various C++ implementations (LCPSolve, Siconos)
- **License**: Apache 2.0 (Siconos)

| Criterion | Assessment |
|-----------|-----------|
| 3D Coulomb friction cones | **Cannot handle exact cones** — requires linearization |
| Small problem performance | Excellent for very small problems |
| Numerical robustness | Known issues with standard Lemke on ill-conditioned LCPs |
| Warm starting | Limited |

**Assessment**: Fast and deterministic but shares the linearization limitation with OSQP. Siconos provides a comprehensive friction-contact solver framework (Apache 2.0).

---

### 9. PATH Solver

- **Type**: Stabilized Newton for Mixed Complementarity Problems
- **License**: **Proprietary** (free for ≤300 variables)

**Assessment**: Technically excellent for MCP/LCP but proprietary licensing makes it unsuitable for this project.

---

## Comparative Summary

| Solver | Exact Coulomb Cone | Small Perf | C++ Integration | License | Robust | Maintained | Warm Start |
|--------|-------------------|-----------|----------------|---------|--------|------------|------------|
| ECOS (current) | Via epigraph | Excellent | Good | **GPLv3** | Good | **No** | **No** |
| Clarabel | **Yes (native)** | Very Good | Good (Eigen) | Apache 2.0 | Excellent | Yes | Limited |
| OSQP | No (linearize) | Good | Excellent | Apache 2.0 | Good | Yes | **Yes** |
| SCS | Via epigraph | Poor | Good | MIT | Moderate | Yes | Yes |
| Custom PGS | Via projection | **Best** | **Best** | N/A | Moderate | N/A | **Best** |
| Custom Newton | **Yes (exact)** | Excellent | **Best** | N/A | Very Good | N/A | Good |
| Lemke/Siconos | No (linearize) | Excellent | Good | Apache 2.0 | Moderate | Siconos: Yes | Limited |

---

## What Established Physics Engines Use

### MuJoCo — Custom Newton Solver
Relaxes strict complementarity and solves a convex QP with exact elliptic friction cones. Newton method (default), CG, or PGS available. Convex formulation guarantees unique solution and polynomial-time complexity. Tradeoff is "soft contact" via compliance regularization.

### Drake — SAP (Semi-Analytical Primal)
Unconstrained convex formulation that analytically eliminates contact constraints. Warm-starts effectively, second-order accuracy. Published in IEEE Transactions on Robotics (2022). Also supports Clarabel for SOCP and previously used SCS/MOSEK.

### Bullet — Sequential Impulse (PGS)
`btSequentialImpulseConstraintSolver` with 10 iterations default. Pyramidal friction cone approximation. Speed over accuracy for real-time guarantees.

### PhysX — PGS and TGS
Both PGS and Temporal Gauss-Seidel (sub-stepping instead of more iterations). TGS becoming the default for improved stability.

### DART — LCP (Dantzig/PGS)
Linearized friction cones. Best friction accuracy in comparative benchmarks but poor scalability.

---

## Recommendations for MSD-CPP

Ranked by suitability for the project's requirements (1-50 contacts, near-real-time, C++20, permissive license, exact Coulomb cones desired):

### 1. Custom Convex Solver (Recommended)

Implement a Newton-based solver for the friction contact QP with exact cone constraints. This is the approach MuJoCo and Drake's SAP solver use in production.

**Advantages**:
- No external dependencies (eliminates ECOS GPLv3 concern entirely)
- Handles exact Coulomb cones natively
- 3-8 Newton iterations with quadratic convergence
- Supports warm starting from previous timestep
- Best performance for 1-50 contacts
- Proven approach in production physics engines

**Effort**: ~400-600 lines of C++. Requires cone projection and Newton step with Hessian factorization.

**Key references**:
- Todorov, "Convex and analytically-invertible dynamics with contacts and constraints" (2014)
- Castro et al., "An unconstrained convex formulation of compliant contact" (2022) — Drake SAP

### 2. Clarabel (Best External Solver)

Replace ECOS with Clarabel as the conic solver.

**Advantages**:
- Handles QP + SOC natively (no epigraph lifting)
- Apache 2.0 license
- Official ECOS successor (CVXPY's choice)
- Actively maintained, excellent robustness

**Effort**: Requires Rust toolchain in build system, Conan recipe, and API migration from ECOS to Clarabel. C++ interface accepts Eigen types.

### 3. Custom PGS with Cone Projection (Simplest)

Implement PGS with per-iteration projection onto exact friction cones.

**Advantages**:
- Simplest to implement (~200-300 lines)
- Fastest for small contact counts
- Best warm starting support
- No external dependencies

**Tradeoff**: First-order convergence, may need 10-30 iterations, less accurate than Newton-based approaches.

### 4. OSQP with Polyhedral Friction (Pragmatic)

Use OSQP with 8-12 sided polyhedral approximation of friction cones.

**Advantages**:
- Well-maintained, Apache 2.0, excellent warm starting
- 8-sided polygon gives <5% friction error
- Simpler formulation (pure QP)

**Tradeoff**: Not exact Coulomb cones. Additional linear constraints per contact.

### Not Recommended

- **ECOS**: GPLv3, unmaintained, no warm starting, requires error-prone epigraph lifting
- **SCS**: Too slow for small problems, poor tail convergence
- **MOSEK**: Commercial license incompatible with open-source
- **PATH**: Proprietary

---

## References

- Todorov, E. "Convex and analytically-invertible dynamics with contacts and constraints." ICRA 2014.
- Castro, A. et al. "An unconstrained convex formulation of compliant contact." IEEE T-RO 2022. [arXiv:2110.10107](https://arxiv.org/abs/2110.10107)
- Goulart, P. and Chen, Y. "Clarabel: An interior-point solver for conic programs with quadratic objectives." 2024. [arXiv:2405.12762](https://arxiv.org/html/2405.12762v1)
- Domahidi, A. et al. "ECOS: An SOCP solver for embedded systems." ECC 2013.
- [CVXPY ECOS-to-Clarabel migration discussion](https://github.com/cvxpy/cvxpy/discussions/2178)
- [Clarabel C/C++ interface documentation](https://clarabel.org/stable/user_guide_c_cpp/)
- [MuJoCo computation documentation](https://mujoco.readthedocs.io/en/stable/computation/index.html)
- [Drake contact model documentation](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html)
- [PhysX rigid body dynamics (PGS/TGS)](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/RigidBodyDynamics.html)
- [Box2D Solver2D analysis (2024)](https://box2d.org/posts/2024/02/solver2d/)
- [Siconos friction-contact solvers](https://nonsmooth.gricad-pages.univ-grenoble-alpes.fr/siconos/users_guide/problems_and_solvers/friction_contact.html)
- [SimBenchmark physics engine comparison](https://leggedrobotics.github.io/SimBenchmark/)
