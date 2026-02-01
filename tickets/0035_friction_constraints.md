# Ticket 0035: Friction Constraints

## Status
- [x] Draft
- [x] Ready for Math Formulation
- [x] Math Formulation Complete — Awaiting Review
- [ ] Math Review Approved — Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Math Review
**Assignee**: math-reviewer
**Created**: 2026-01-31
**Generate Tutorial**: Yes
**Related Tickets**: [0032_contact_constraint_refactor](0032_contact_constraint_refactor.md), [0034_active_set_method_contact_solver](0034_active_set_method_contact_solver.md), [0031_generalized_lagrange_constraints](0031_generalized_lagrange_constraints.md)

### Subtasks

| Subtask | Description | Dependencies |
|---------|-------------|--------------|
| [0035a](0035a_tangent_basis_and_friction_constraint.md) | Tangent basis + FrictionConstraint class | 0032a |
| [0035b](0035b_box_constrained_asm_solver.md) | Box-constrained ASM solver extension | 0035a, 0034 |
| [0035c](0035c_friction_pipeline_integration.md) | Pipeline integration + material properties | 0035a, 0035b |
| [0035d](0035d_friction_hardening_and_validation.md) | Hardening + full validation suite | 0035a, 0035b, 0035c |

---

## Summary

Add Coulomb friction constraints to the contact constraint system. Each contact point currently produces a single normal (non-penetration) constraint. This ticket extends each contact to include two tangential friction constraint rows, enabling realistic sliding resistance, torque from glancing collisions, and static stacking stability. The friction model couples tangential impulses to normal impulses via the Coulomb friction cone: $\|\boldsymbol{\lambda}_t\| \leq \mu \lambda_n$.

---

## Motivation

### Current Limitation

The constraint-based collision response system (Tickets 0032, 0034) enforces only normal non-penetration constraints. This means:

1. **No sliding resistance**: Objects slide frictionlessly on surfaces. A sphere on an inclined plane accelerates without bound.
2. **No torque from tangential contact**: Glancing collisions cannot generate angular velocity through tangential forces. This was identified as a known limitation in Ticket 0032c (AC4 test modified due to missing friction).
3. **Unrealistic stacking**: Stacked objects rely solely on normal forces for stability. Without friction, lateral perturbations cause immediate sliding.
4. **No rolling resistance**: Objects cannot transition from sliding to rolling, or maintain rolling contact.

### What Friction Enables

1. **Realistic contact dynamics**: Objects decelerate on surfaces, roll instead of slide, and stack stably under lateral perturbations.
2. **Glancing collision torque**: Off-center impacts generate angular velocity through tangential impulse transfer (resolves 0032c AC4 limitation).
3. **Static equilibrium**: Objects on inclined surfaces below the friction angle remain stationary.
4. **Coupled normal-tangential response**: The Coulomb cone couples normal and tangential impulses, producing physically correct force distributions.

### Use Cases

- **Resting on inclined surfaces**: Object on a 30° slope with μ = 0.6 remains static (friction angle = arctan(0.6) ≈ 31°)
- **Sliding deceleration**: Object with initial horizontal velocity decelerates due to kinetic friction
- **Spin from glancing impact**: Off-center collision produces angular velocity proportional to tangential impulse
- **Stack stability under perturbation**: Horizontal force on stacked object resisted by friction up to μ·N

---

## Mathematical Analysis Requirements

This ticket requires a rigorous mathematical formulation phase **before** architectural design. The math formulation must establish:

### M1. Tangential Contact Basis Construction

Derive a robust method for constructing an orthonormal tangent basis $\{\mathbf{t}_1, \mathbf{t}_2\}$ from the contact normal $\mathbf{n}$:
- Must be deterministic and continuous (no discontinuities as $\mathbf{n}$ varies)
- Must handle degenerate normals (e.g., $\mathbf{n}$ aligned with coordinate axes)
- Preferred: Duff et al. (2017) or Pixar's method for building orthonormal basis from a single vector

### M2. Friction Constraint Jacobian Derivation

Derive the velocity-level friction constraint Jacobians for a two-body contact:
- Tangential relative velocity: $v_{t_i} = \mathbf{J}_{t_i} \dot{\mathbf{q}}$ for $i \in \{1, 2\}$
- Each friction Jacobian row: $\mathbf{J}_{t_i} = [-\mathbf{t}_i^\top, -(\mathbf{r}_A \times \mathbf{t}_i)^\top, \mathbf{t}_i^\top, (\mathbf{r}_B \times \mathbf{t}_i)^\top]$
- Combined 3×12 Jacobian per contact: one normal row + two tangential rows
- Verify orthogonality: $\mathbf{J}_n \mathbf{M}^{-1} \mathbf{J}_{t_i}^\top$ coupling terms

### M3. Coulomb Friction Cone Formulation

Formulate the Coulomb friction model within the LCP/complementarity framework:
- **Friction cone constraint**: $\sqrt{\lambda_{t_1}^2 + \lambda_{t_2}^2} \leq \mu \lambda_n$
- **Linearized (pyramidal) approximation**: $|\lambda_{t_1}| + |\lambda_{t_2}| \leq \mu \lambda_n$ or $|\lambda_{t_i}| \leq \frac{\mu}{\sqrt{2}} \lambda_n$
- **Box friction approximation**: $|\lambda_{t_i}| \leq \mu \lambda_n$ (per-axis, decoupled)
- Analysis of approximation quality vs. solver complexity for each
- Recommendation with justification

### M4. Complementarity Conditions for Friction

Derive the full complementarity system combining normal and friction constraints:
- **Normal**: $\lambda_n \geq 0$, $v_n \geq 0$, $\lambda_n v_n = 0$
- **Stick condition**: $\|\mathbf{v}_t\| = 0$ when $\|\boldsymbol{\lambda}_t\| < \mu \lambda_n$
- **Slide condition**: $\mathbf{v}_t = -\gamma \boldsymbol{\lambda}_t$ (sliding direction opposes friction force) when $\|\boldsymbol{\lambda}_t\| = \mu \lambda_n$
- Maximum dissipation principle: friction maximizes energy dissipation rate
- Show this is a Mixed Complementarity Problem (MCP), not a standard LCP

### M5. Solver Extension Analysis

Analyze how the existing Active Set Method (ASM) solver extends to handle friction:
- **Option A**: Box-constrained LCP — extend ASM with per-constraint bounds $\lambda_{t_i} \in [-\mu \lambda_n, +\mu \lambda_n]$
- **Option B**: Cone-constrained QP — project onto second-order cone (SOCP)
- **Option C**: Splitting method — decompose into normal solve + friction projection (Gauss-Seidel style per-contact)
- **Option D**: NCP (Nonlinear Complementarity) reformulation — Fischer-Burmeister or min-map functions
- Convergence analysis for each approach
- Interaction with existing bilateral constraint solve path (Phase 1 / Phase 2 architecture)
- Recommendation with justification

### M6. Energy Dissipation Analysis

Prove or verify that the friction formulation:
- Dissipates energy (friction power $P_f = \boldsymbol{\lambda}_t \cdot \mathbf{v}_t \leq 0$)
- Does not inject energy into the system
- Correctly transitions between static friction (zero sliding velocity) and kinetic friction (sliding)
- Analyze interaction with Baumgarte stabilization (ERP) — can friction + ERP inject energy?

### M7. Numerical Stability Analysis

Analyze conditioning and stability:
- Effect of friction on effective mass matrix $\mathbf{A}$ conditioning
- 3×3 block structure per contact (vs current 1×1) — sparsity pattern
- Mass ratio sensitivity with friction (does friction worsen or improve conditioning?)
- Regularization strategy for friction rows
- Degenerate cases: zero normal force with nonzero tangential velocity, sliding reversal

### M8. Numerical Examples with GTest Mappings

Provide concrete worked examples:

1. **Block on inclined plane (static friction)**: μ = 0.6, slope = 20°. Verify block remains stationary, friction force = mg·sin(20°) < μ·mg·cos(20°).
2. **Block on inclined plane (kinetic friction)**: μ = 0.3, slope = 45°. Verify block accelerates at g·(sin(45°) - μ·cos(45°)).
3. **Sliding deceleration on flat surface**: Horizontal velocity v₀, μ = 0.5. Verify deceleration = μ·g, stopping distance = v₀²/(2μg).
4. **Glancing collision with spin**: Off-center impact producing angular velocity. Verify tangential impulse generates correct torque.
5. **Friction cone saturation**: Applied tangential load exceeds μ·N, verify transition from stick to slide.
6. **Two-body friction**: Both bodies dynamic, verify Newton's third law for tangential forces.

---

## Technical Approach (High-Level)

The detailed technical approach will be determined during the Design phase after the mathematical formulation is complete. The following outlines the expected architecture:

### Expected Component Changes

1. **New: `FrictionConstraint`** — Two tangential constraint rows per contact point, coupled to normal constraint via friction coefficient
2. **Modified: `ContactConstraintFactory`** — Create friction constraints alongside normal constraints; compute tangent basis
3. **Modified: `ConstraintSolver::solveWithContacts()`** — Extend ASM to handle friction bounds (box-LCP or cone projection)
4. **Modified: `WorldModel::update()`** — Pass friction coefficients through contact pipeline
5. **New/Modified: Material properties** — Per-body friction coefficient (static and kinetic)

### Constraint Dimension Growth

- **Current**: 1 constraint row per contact point (normal only)
- **After**: 3 constraint rows per contact point (1 normal + 2 tangential)
- For k contact points: system grows from k×k to 3k×3k effective mass matrix

### Solver Modification Scope

The solver extension depends on the math formulation recommendation (M5). Possible approaches:
- **Minimal**: Box-friction approximation within existing ASM framework (add per-constraint bounds)
- **Moderate**: Pyramidal friction cone with split normal/tangential solving
- **Full**: Second-order cone projection for true Coulomb friction

---

## Acceptance Criteria

### Mathematical Formulation Phase

- [x] **MF1**: Tangent basis construction derived with continuity proof
- [x] **MF2**: Friction Jacobian derivation complete with dimensional verification (2×12 per tangent direction, 3×12 combined)
- [x] **MF3**: Coulomb friction cone formulation with linearization analysis and recommendation
- [x] **MF4**: Full complementarity conditions (normal + friction) derived
- [x] **MF5**: Solver extension analysis with convergence properties and recommendation
- [x] **MF6**: Energy dissipation proof (friction never injects energy)
- [x] **MF7**: Numerical stability analysis with conditioning bounds
- [x] **MF8**: All 6 numerical examples worked through with hand-verified results

### Implementation Phase (After Design)

- [ ] **AC1**: `FrictionConstraint` class passes unit tests (Jacobian, evaluation, activation)
- [ ] **AC2**: Tangent basis construction is deterministic and continuous
- [ ] **AC3**: Block on inclined plane (below friction angle) remains static for 1000 frames
- [ ] **AC4**: Sliding object decelerates at μ·g (within 5% tolerance)
- [ ] **AC5**: Glancing collision produces angular velocity (resolves 0032c AC4 limitation)
- [ ] **AC6**: Friction force obeys Coulomb cone: $\|\boldsymbol{\lambda}_t\| \leq \mu \lambda_n + \epsilon$
- [ ] **AC7**: Energy is monotonically non-increasing for friction-only scenarios (no external forces)
- [ ] **AC8**: All existing normal contact tests pass (79 constraint tests, zero regressions)
- [ ] **AC9**: Friction coefficient combinable: $\mu_{combined} = \sqrt{\mu_A \cdot \mu_B}$ (geometric mean, matching restitution pattern)
- [ ] **AC10**: Performance: 10 contacts with friction solve in < 2× wall time of 10 normal-only contacts

---

## Dependencies

### Required (Must Exist)

| Dependency | Status | Notes |
|------------|--------|-------|
| Ticket 0032 (Contact Constraint Refactor) | Complete | Provides `ContactConstraint`, `TwoBodyConstraint`, `ContactConstraintFactory` |
| Ticket 0034 (Active Set Method Solver) | Complete | Provides ASM solver to extend with friction bounds |
| Ticket 0031 (Generalized Constraints) | Complete | Provides `Constraint`, `UnilateralConstraint` framework |

### Blocked By

None — all dependencies are complete.

---

## Risks and Mitigations

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Coulomb cone makes problem non-convex (NCP) | High | Medium | Use box/pyramidal approximation (convex); analyze error bounds |
| Friction + Baumgarte energy injection | Medium | Medium | Energy audit in math formulation; damping terms if needed |
| 3× constraint growth degrades ASM performance | Medium | Low | Profile; consider contact islands or constraint reduction |
| Static-to-kinetic friction transition causes jitter | Medium | Medium | Velocity threshold (like rest velocity threshold for restitution) |
| Solver convergence with coupled normal-friction | High | Low | Prototype validation before implementation |

---

## References

### Academic

- Stewart, D. & Trinkle, J. (1996). "An Implicit Time-Stepping Scheme for Rigid Body Dynamics with Coulomb Friction"
- Anitescu, M. & Potra, F. (1997). "Formulating Dynamic Multi-Rigid-Body Contact Problems with Friction as Solvable Linear Complementarity Problems"
- Erleben, K. (2007). "Velocity-based shock propagation for multibody dynamics animation"
- Todorov, E. (2014). "Convex and analytically-invertible dynamics with contacts and constraints" (MuJoCo)
- Catto, E. (2009). "Modeling and Solving Constraints" (GDC presentation, Box2D friction model)
- Duff, T. et al. (2017). "Building an Orthonormal Basis, Revisited" (JCGT, tangent basis construction)

### Existing Codebase

- `msd/msd-sim/src/Physics/Constraints/ContactConstraint.hpp` — Normal constraint to extend
- `msd/msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp` — Factory to extend
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` — Solver to extend
- `docs/designs/0032_contact_constraint_refactor/math-formulation.md` — Normal constraint math foundation

---

## Workflow Log

### Math Formulation Phase
- **Started**: 2026-01-31 15:00
- **Completed**: 2026-01-31 15:45
- **Assignee**: math-designer
- **Artifacts**:
  - `docs/designs/0035_friction_constraints/math-formulation.md`
- **Notes**: Produced comprehensive mathematical formulation covering all M1-M8 requirements:
  - M1: Duff et al. (2017) orthonormal basis construction with continuity proof
  - M2: Friction Jacobian derivation (1×12 per tangent direction, 3×12 combined per contact)
  - M3: Coulomb friction cone formulation with 4 approximation options analyzed; recommended scaled box approximation
  - M4: Full complementarity conditions for stick/slip regimes with maximum dissipation principle
  - M5: Solver extension analysis comparing 4 approaches; recommended box-constrained LCP with Active Set Method
  - M6: Energy dissipation proof showing friction power ≤ 0 in all regimes
  - M7: Numerical stability analysis with conditioning bounds, mass ratio sensitivity, degenerate case handling
  - M8: Six concrete numerical examples with hand computations and GTest templates
  - **Open questions**: Friction cone approximation selection, outer iteration strategy, friction coefficient combination rule

### Math Review Phase
- **Started**: 2026-01-31 15:45
- **Status**: Awaiting math-reviewer
- **Assignee**: math-reviewer
- **Notes**: Mathematical formulation ready for rigorous review and validation
