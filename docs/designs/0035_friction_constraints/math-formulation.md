# Mathematical Formulation: Friction Constraints

## Summary

This document establishes the rigorous mathematical foundation for Coulomb friction constraints in the contact constraint system. Each contact point currently produces a single normal (non-penetration) constraint. This formulation extends each contact to include two orthogonal tangential friction constraint rows, coupled to the normal constraint via the Coulomb friction cone: $\|\boldsymbol{\lambda}_t\| \leq \mu \lambda_n$.

The formulation is organized into eight sections (M1-M8), each in its own document. This overview describes the problem, defines shared notation and assumptions, and shows how the sections interconnect.

---

## Section Index

| Section | Document | Topic |
|---------|----------|-------|
| **M1** | [M1-tangent-basis.md](M1-tangent-basis.md) | Tangential Contact Basis Construction |
| **M2** | [M2-friction-jacobian.md](M2-friction-jacobian.md) | Friction Constraint Jacobian Derivation |
| **M3** | [M3-coulomb-cone.md](M3-coulomb-cone.md) | Coulomb Friction Cone Formulation |
| **M4** | [M4-complementarity.md](M4-complementarity.md) | Complementarity Conditions for Friction |
| **M5** | [M5-solver-extension.md](M5-solver-extension.md) | Solver Extension Analysis |
| **M6** | [M6-energy-dissipation.md](M6-energy-dissipation.md) | Energy Dissipation Analysis |
| **M7** | [M7-numerical-stability.md](M7-numerical-stability.md) | Numerical Stability Analysis |
| **M8** | [M8-numerical-examples.md](M8-numerical-examples.md) | Numerical Examples with GTest Mappings |

---

## Section Dependency Graph

The sections build on each other in a directed acyclic graph. Read top-to-bottom; each section depends on those above it that have connecting arrows.

```
    M1 (Tangent Basis)
      │
      ▼
    M2 (Friction Jacobian)
      │
      ├──────────────┐
      ▼              ▼
    M3 (Coulomb    M4 (Complementarity)
     Cone)           │
      │    ┌─────────┤
      │    │         │
      ▼    ▼         ▼
    M5 (Solver     M6 (Energy
     Extension)     Dissipation)
      │              │
      └──────┬───────┘
             ▼
    M7 (Numerical Stability)
             │
             ▼
    M8 (Numerical Examples)
```

### Detailed Dependencies

| Section | Depends On | Provides To |
|---------|-----------|-------------|
| **M1** | _(standalone)_ | M2, M8 |
| **M2** | M1 | M3, M4, M5, M7, M8 |
| **M3** | M2 | M4, M5, M6, M8 |
| **M4** | M2, M3 | M5, M6 |
| **M5** | M2, M3, M4 | M7, M8 |
| **M6** | M3, M4 | M7 |
| **M7** | M2, M5, M6 | M8 |
| **M8** | M1, M2, M3, M4, M5, M7 | _(terminal — GTest templates)_ |

### Reading Order

For a **sequential read**, follow: M1 → M2 → M3 → M4 → M5 → M6 → M7 → M8.

For a **focused read**:
- **Basis and Jacobians**: M1 → M2
- **Friction model**: M3 → M4
- **Solver design**: M5 (after M2-M4)
- **Safety proofs**: M6 → M7
- **Test cases**: M8 (requires all above)

---

## Problem Statement

### Physical Context

When two rigid bodies make contact, physical interactions occur in three orthogonal directions at the contact point: one normal direction (preventing interpenetration) and two tangential directions (resisting sliding). The Coulomb friction model describes the relationship between normal and tangential forces at the contact interface.

**Current system** (Ticket 0032, 0034): Contact constraints enforce only normal non-penetration, resulting in frictionless sliding.

**Target system**: Each contact point enforces:
1. **Normal constraint**: Non-penetration with restitution ($v_n \geq 0$, $\lambda_n \geq 0$, $\lambda_n \cdot v_n = 0$)
2. **Friction constraints**: Two tangential constraints coupled to normal force via Coulomb cone

### Mathematical Objective

Given:
- Contact normal $\mathbf{n} \in \mathbb{R}^3$ (unit vector)
- Contact point positions $\mathbf{p}_A, \mathbf{p}_B \in \mathbb{R}^3$ on bodies A and B
- Two-body generalized coordinates $\mathbf{q} = [\mathbf{x}_A^\top, \mathbf{Q}_A^\top, \mathbf{x}_B^\top, \mathbf{Q}_B^\top]^\top \in \mathbb{R}^{14}$
- Coefficient of friction $\mu \geq 0$

Compute:
1. Orthonormal tangent basis $\{\mathbf{t}_1, \mathbf{t}_2\}$ spanning the contact tangent plane → **[M1](M1-tangent-basis.md)**
2. Friction constraint Jacobians $\mathbf{J}_{t_1}, \mathbf{J}_{t_2} \in \mathbb{R}^{1 \times 12}$ → **[M2](M2-friction-jacobian.md)**
3. Tangential constraint forces $\boldsymbol{\lambda}_t = [\lambda_{t_1}, \lambda_{t_2}]^\top$ satisfying Coulomb cone → **[M3](M3-coulomb-cone.md)**, **[M4](M4-complementarity.md)**
4. Solver algorithm producing these forces → **[M5](M5-solver-extension.md)**

Such that:
- Energy is dissipated (never injected) by friction → **[M6](M6-energy-dissipation.md)**
- System is numerically stable → **[M7](M7-numerical-stability.md)**
- Friction forces obey Newton's third law between bodies → **[M8](M8-numerical-examples.md)**

---

## Mathematical Framework

### Definitions and Notation

| Symbol | Type | Definition | Units |
|--------|------|------------|-------|
| $\mathbf{n}$ | $\mathbb{R}^3$ | Contact normal (points from B to A, unit vector) | — |
| $\mathbf{t}_1, \mathbf{t}_2$ | $\mathbb{R}^3$ | Orthonormal tangent basis vectors | — |
| $\mathbf{p}$ | $\mathbb{R}^3$ | Contact point position (world frame) | m |
| $\mathbf{r}_A = \mathbf{p} - \mathbf{x}_A$ | $\mathbb{R}^3$ | Contact point relative to body A center of mass | m |
| $\mathbf{r}_B = \mathbf{p} - \mathbf{x}_B$ | $\mathbb{R}^3$ | Contact point relative to body B center of mass | m |
| $\mathbf{q}$ | $\mathbb{R}^{14}$ | Two-body generalized coordinates: $[\mathbf{x}_A^\top, \mathbf{Q}_A^\top, \mathbf{x}_B^\top, \mathbf{Q}_B^\top]^\top$ | mixed |
| $\dot{\mathbf{q}}$ | $\mathbb{R}^{14}$ | Two-body generalized velocities | mixed |
| $\mathbf{v}_n$ | $\mathbb{R}$ | Normal relative velocity at contact | m/s |
| $\mathbf{v}_{t_i}$ | $\mathbb{R}$ | Tangential relative velocity (direction $i \in \{1, 2\}$) | m/s |
| $\mathbf{v}_t$ | $\mathbb{R}^2$ | Tangential velocity vector: $[\mathbf{v}_{t_1}, \mathbf{v}_{t_2}]^\top$ | m/s |
| $\lambda_n$ | $\mathbb{R}_{\geq 0}$ | Normal constraint force magnitude | N |
| $\lambda_{t_i}$ | $\mathbb{R}$ | Tangential constraint force (direction $i$) | N |
| $\boldsymbol{\lambda}_t$ | $\mathbb{R}^2$ | Tangential force vector: $[\lambda_{t_1}, \lambda_{t_2}]^\top$ | N |
| $\mu$ | $\mathbb{R}_{\geq 0}$ | Coefficient of friction (combined) | — |
| $\mathbf{J}_n$ | $\mathbb{R}^{1 \times 14}$ | Normal constraint Jacobian | — |
| $\mathbf{J}_{t_i}$ | $\mathbb{R}^{1 \times 14}$ | Tangential constraint Jacobian (direction $i$) | — |
| $\mathbf{M}$ | $\mathbb{R}^{14 \times 14}$ | Block-diagonal mass matrix | kg, kg·m² |
| $\mathbf{A}$ | $\mathbb{R}^{C \times C}$ | Effective mass matrix: $\mathbf{J}\mathbf{M}^{-1}\mathbf{J}^\top$ | kg⁻¹ |
| $C$ | $\mathbb{N}$ | Total number of constraint rows (= 3 × number of contacts) | — |

### Coordinate Systems

**World Frame**: Inertial reference frame in which all positions, velocities, and forces are expressed.

**Contact Frame**: Orthonormal basis $\{\mathbf{t}_1, \mathbf{t}_2, \mathbf{n}\}$ at the contact point, with:
- $\mathbf{n}$ — normal direction (defined by collision detection, points from B to A)
- $\mathbf{t}_1, \mathbf{t}_2$ — tangent directions (constructed per **[M1](M1-tangent-basis.md)**)

**Body Frame**: Attached to each rigid body's center of mass. Not directly used in constraint formulation (all constraints expressed in world frame).

### Assumptions

1. **Rigid body kinematics**: Bodies are perfectly rigid (no deformation), positions fully determined by generalized coordinates $\mathbf{q}$.

2. **Instantaneous contact**: Contact occurs at discrete points (or averaged point in contact manifold), no extended contact patches.

3. **Isotropic Coulomb friction**: Friction coefficient $\mu$ is independent of sliding direction in the tangent plane (rotationally symmetric friction cone).

4. **No rolling resistance**: Friction acts only on sliding velocity, not on rolling contact (spin about normal axis not resisted).

5. **Quasi-static friction coefficient**: Static and kinetic friction coefficients assumed equal ($\mu_s = \mu_k = \mu$). Transition between stick and slip is instantaneous.

6. **Contact normal continuity**: Contact normal $\mathbf{n}$ varies smoothly with configuration (no discontinuous flips).

7. **Small timestep**: Integration timestep $\Delta t$ small enough that contact geometry does not change significantly within one step.

---

## Key Recommendations

These are the primary recommendations from the detailed analysis sections:

| Decision | Recommendation | Section |
|----------|---------------|---------|
| **Tangent basis method** | Duff et al. (2017) — continuous, deterministic, branch-free per case | [M1](M1-tangent-basis.md) |
| **Jacobian formulation** | Direct angular velocity (12-DOF), not quaternion (14-DOF) | [M2](M2-friction-jacobian.md) |
| **Friction cone approximation** | Scaled per-axis box ($\|\lambda_{t_i}\| \leq \mu/\sqrt{2} \cdot \lambda_n$), 29% error, conservative | [M3](M3-coulomb-cone.md) |
| **Solver approach** | Box-constrained LCP with Active Set Method (extend existing ASM) | [M5](M5-solver-extension.md) |
| **Baumgarte for friction** | $\alpha = \beta = 0$ (no stabilization on tangential rows) | [M6](M6-energy-dissipation.md) |
| **Regularization** | None initially; add $\epsilon \sim 10^{-10}$ if LLT fails | [M7](M7-numerical-stability.md) |
| **Velocity threshold** | $v_{\text{rest}} = 0.01$ m/s (same as restitution threshold) | [M7](M7-numerical-stability.md) |

---

## Open Questions

### Mathematical Decisions (Human Input Needed)

1. **Friction cone approximation selection**:
   - **Option A**: Scaled box approximation ($|\lambda_{t_i}| \leq \mu \lambda_n / \sqrt{2}$) — Simplest, decoupled, 29% error
   - **Option B**: Pyramidal approximation ($|\lambda_{t_1}| + |\lambda_{t_2}| \leq \mu \lambda_n$) — Coupled, 29% error, 4 constraints
   - **Option C**: Octagonal approximation (8-sided polygon) — High accuracy (<2% error), 8 constraints
   - **Option D**: Exact SOCP with circular cone — Zero error, requires external SOCP solver library

   **Recommendation**: Start with **Option A** (scaled box) for initial implementation. Validate accuracy with Example 5 (friction cone saturation). If error acceptable, ship it. If not, upgrade to Option C (octagonal) or Option D (SOCP).

   See [M3](M3-coulomb-cone.md) for full analysis.

2. **Outer iteration strategy for variable friction bounds**:
   - **Option A**: Single-pass ASM with fixed bounds ($\mu_{\text{max}}$) — Fastest, approximate
   - **Option B**: Outer iteration until $\|\Delta \lambda_n\| < \epsilon$ — Exact for box LCP, slower

   **Recommendation**: Prototype both. Measure convergence rate and accuracy. If single-pass is within 5% error, use it (faster). Otherwise, use outer iteration with 10-iteration cap.

   See [M5](M5-solver-extension.md) for full analysis.

3. **Velocity threshold for stick-slip transition**:
   - Current restitution rest velocity: $v_{\text{rest}} = 0.01$ m/s (from Ticket 0032)
   - Should friction use the same threshold, or a different value?

   **Recommendation**: Use same threshold ($0.01$ m/s) for consistency. If jitter observed in tests, increase to $0.05$ m/s.

   See [M7](M7-numerical-stability.md) for analysis.

### Clarifications Needed

1. **Friction coefficient combination rule**: When two materials with different friction coefficients $\mu_A, \mu_B$ make contact, how is the combined friction coefficient computed?
   - **Option A**: Geometric mean: $\mu = \sqrt{\mu_A \cdot \mu_B}$ (matches restitution combination in Ticket 0032)
   - **Option B**: Harmonic mean: $\mu = 2 \mu_A \mu_B / (\mu_A + \mu_B)$
   - **Option C**: Minimum: $\mu = \min(\mu_A, \mu_B)$

   **Recommendation**: **Geometric mean** for consistency with restitution (AC9 specifies this).

2. **Static vs kinetic friction**: Should the formulation distinguish between static ($\mu_s$) and kinetic ($\mu_k$) friction coefficients?
   - **Current assumption**: $\mu_s = \mu_k = \mu$ (single coefficient)
   - **Alternative**: Two coefficients with velocity-dependent transition

   **Recommendation**: Single coefficient for simplicity (Assumption 5). Future work can add velocity-dependent $\mu(\|\mathbf{v}_t\|)$ if needed.

---

## Beyond Scope

1. **Rolling resistance**: Friction about the contact normal (spin resistance) not modeled. This would require a third friction direction perpendicular to $\mathbf{t}_1, \mathbf{t}_2$ (aligned with $\mathbf{n}$).

2. **Anisotropic friction**: Direction-dependent friction coefficients (e.g., higher friction along grain direction). Requires replacing scalar $\mu$ with friction tensor $\boldsymbol{\mu}$.

3. **Velocity-dependent friction**: $\mu$ varies with sliding speed (Stribeck curve). Common in lubricated contacts.

4. **Joint friction**: Friction in revolute/prismatic joints (damping torques). Different mathematical model than contact friction.

5. **Warm-starting**: Using previous timestep's $\boldsymbol{\lambda}$ as initial guess for Active Set Method to reduce iterations.

6. **Contact islands**: Decomposing disconnected contact groups into independent LCPs to reduce solver dimension (block-diagonal $\mathbf{A}$).

---

## References

### Academic Literature

- **Stewart, D. & Trinkle, J. (1996)**. "An Implicit Time-Stepping Scheme for Rigid Body Dynamics with Coulomb Friction". *International Journal for Numerical Methods in Engineering*.
  - Establishes LCP formulation for Coulomb friction with complementarity conditions.

- **Anitescu, M. & Potra, F. (1997)**. "Formulating Dynamic Multi-Rigid-Body Contact Problems with Friction as Solvable Linear Complementarity Problems". *Nonlinear Dynamics*, 14(3), 231-247.
  - Derives time-stepping scheme for friction constraints using polyhedral cone approximations.

- **Erleben, K. (2007)**. "Velocity-based shock propagation for multibody dynamics animation". *ACM Transactions on Graphics (SIGGRAPH)*, 26(2).
  - Analyzes velocity-level formulation for contact and friction suitable for real-time simulation.

- **Todorov, E. (2014)**. "Convex and analytically-invertible dynamics with contacts and constraints: Theory and implementation in MuJoCo". *IEEE International Conference on Robotics and Automation (ICRA)*.
  - Describes convex optimization approach for Coulomb friction using cone constraints.

- **Catto, E. (2009)**. "Modeling and Solving Constraints". *Game Developers Conference (GDC)*.
  - Practical approach to friction constraints in Box2D physics engine using sequential impulse method.

- **Duff, T., Burgess, J., Christensen, P., Hery, C., Kensler, A., Liani, M., & Villemin, R. (2017)**. "Building an Orthonormal Basis, Revisited". *Journal of Computer Graphics Techniques (JCGT)*, 6(1), 1-8.
  - Robust method for constructing orthonormal tangent basis from a single normal vector (used in [M1](M1-tangent-basis.md)).

- **Cottle, R. W., Pang, J.-S., & Stone, R. E. (2009)**. *The Linear Complementarity Problem*. SIAM.
  - Comprehensive reference for LCP theory, including Active Set Method convergence proofs.

### Existing Codebase

- **`msd/msd-sim/src/Physics/Constraints/ContactConstraint.hpp`** — Normal constraint Jacobian and evaluation (foundation for friction extension)
- **`msd/msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp`** — Factory to extend with friction constraint creation
- **`msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`** — Active Set Method solver to extend with box-constrained LCP
- **`docs/designs/0032_contact_constraint_refactor/math-formulation.md`** — Normal constraint mathematical foundation (Jacobian derivation, complementarity conditions)
- **`docs/designs/0034_active_set_method_contact_solver/design.md`** — ASM solver architecture and convergence analysis
