# M5. Solver Extension Analysis

> **Parent**: [math-formulation.md](math-formulation.md)
> **Depends on**: [M2 (Friction Jacobian)](M2-friction-jacobian.md), [M3 (Coulomb Cone)](M3-coulomb-cone.md), [M4 (Complementarity)](M4-complementarity.md)
> **Required by**: [M7 (Numerical Stability)](M7-numerical-stability.md), [M8 (Numerical Examples)](M8-numerical-examples.md)

---

**Objective**: Analyze how the existing Active Set Method (ASM) solver can be extended to handle friction constraints, comparing solver strategies and providing a recommendation.

## Existing Solver: Active Set Method for Normal Constraints (Ticket 0034)

The current solver handles **unilateral normal constraints** using the Active Set Method:
- **Input**: Normal constraint LCP: $\mathbf{A} \boldsymbol{\lambda}_n = \mathbf{b}$ with $\boldsymbol{\lambda}_n \geq 0$
- **Algorithm**: Iteratively adjust active set (contacts with $\lambda_n > 0$) until KKT conditions satisfied
- **Output**: Exact LCP solution satisfying all complementarity conditions

**Complexity**: Finite iterations (typically $\leq C$ for $C$ contacts), each requiring LLT decomposition of active constraint submatrix.

## Extension Options

### Option A: Box-Constrained LCP with Active Set Method

**Approach**: Extend ASM to handle **box-constrained variables** $\lambda_{t_i} \in [-\mu \lambda_n / \sqrt{2}, +\mu \lambda_n / \sqrt{2}]$.

**Algorithm modifications**:
1. **Constraint system**: Stacked Jacobian with 3 rows per contact (1 normal + 2 tangential)
2. **Variable bounds**:
   - Normal: $\lambda_n \geq 0$ (lower bound only)
   - Tangential: $\lambda_{t_i} \in [-\mu \lambda_n / \sqrt{2}, +\mu \lambda_n / \sqrt{2}]$ (box bounds)
3. **Working set tracking**: For each constraint, track whether it's at lower bound, upper bound, or inactive (interior)
4. **KKT conditions**:
   - **Primal feasibility**: $\lambda_{t_i}$ respects bounds
   - **Dual feasibility**: Gradient of Lagrangian w.r.t. inactive constraints is zero
   - **Complementarity**: Active constraints have non-zero dual variables
5. **Iteration**:
   - Solve active subproblem (constraints at bounds)
   - If any dual variable violates KKT, move constraint between active/inactive sets
   - If any primal variable violates bounds, project to nearest bound

**Coupling challenge**: Friction bounds depend on $\lambda_n$, which is itself a variable. This requires:
- **Inner-outer iteration**:
  - **Outer loop**: Fix $\lambda_n$ from previous iteration to define friction bounds
  - **Inner loop**: Solve box-constrained LCP for $\boldsymbol{\lambda} = [\lambda_n, \lambda_{t_1}, \lambda_{t_2}]^\top$
  - **Update**: Recompute friction bounds based on new $\lambda_n$, repeat until convergence
- **Convergence**: Guaranteed for monotone LCP (effective mass matrix $\mathbf{A}$ is positive semidefinite)

**Pros**:
- Direct extension of existing ASM solver
- Exact solution for box friction approximation
- Deterministic (no iteration count tuning)

**Cons**:
- Variable bounds require outer iteration loop (not single-pass ASM)
- 3x constraint dimension increases LLT decomposition cost ($O((3C)^3)$ vs $O(C^3)$)
- Friction cone approximation error (29%)

**Recommended**: Yes, for initial implementation due to direct extension of proven ASM framework.

### Option B: Second-Order Cone Programming (SOCP)

**Approach**: Formulate exact Coulomb cone as second-order cone constraint and solve with SOCP solver.

**Algorithm**: Use specialized SOCP solver (e.g., ECOS, SCS, MOSEK) to solve:
$$
\begin{aligned}
\min_{\boldsymbol{\lambda}} \quad & \frac{1}{2} \boldsymbol{\lambda}^\top \mathbf{A} \boldsymbol{\lambda} - \mathbf{b}^\top \boldsymbol{\lambda} \\
\text{subject to} \quad & \|\boldsymbol{\lambda}_t\| \leq \mu \lambda_n \quad \text{(second-order cone)} \\
& \lambda_n \geq 0
\end{aligned}
$$

**Pros**:
- Exact Coulomb cone (no approximation error)
- Proven convergence for convex SOCP

**Cons**:
- Requires external SOCP library (ECOS, SCS) — new dependency
- Higher computational cost than LCP (interior-point methods are $O(n^3)$ per iteration, 10-30 iterations typical)
- Not deterministic (iteration count varies)

**Recommended**: Future work after box-friction implementation validated. SOCP provides engineering-grade accuracy but at higher cost.

### Option C: Gauss-Seidel with Cone Projection

**Approach**: Iterate over contacts sequentially, projecting friction forces onto friction cone after each update.

**Algorithm**:
1. Initialize $\boldsymbol{\lambda} = \mathbf{0}$
2. For each contact $i$:
   - Solve local LCP for $\lambda_n^{(i)}$ (normal constraint)
   - Solve local linear system for $\boldsymbol{\lambda}_t^{(i)}$ (tangential constraints, treating other contacts as fixed)
   - Project $\boldsymbol{\lambda}_t^{(i)}$ onto friction cone: $\boldsymbol{\lambda}_t^{(i)} \gets \min(\|\boldsymbol{\lambda}_t^{(i)}\|, \mu \lambda_n^{(i)}) \frac{\boldsymbol{\lambda}_t^{(i)}}{\|\boldsymbol{\lambda}_t^{(i)}\|}$
3. Repeat until convergence (residual below tolerance) or max iterations reached

**Pros**:
- Handles exact Coulomb cone (projection onto circular cone)
- Simple to implement
- Low memory (no global matrix decomposition)

**Cons**:
- **Approximate solution** (no convergence guarantee for friction, only for normal contacts)
- **Iteration count sensitivity**: Convergence depends on contact ordering, mass ratios, friction coefficients
- **Slower than ASM**: Typically requires 50-200 iterations vs 5-20 ASM active set changes
- **Non-deterministic**: Results vary with iteration count and ordering

**Recommended**: No. Ticket 0034 replaced Projected Gauss-Seidel with Active Set Method for normal constraints specifically to eliminate iteration count sensitivity and achieve exact solutions. Reintroducing PGS for friction would regress on these quality goals.

### Option D: NCP Reformulation with Fischer-Burmeister

**Approach**: Reformulate friction complementarity using NCP function and solve with Newton's method.

**Fischer-Burmeister function**:
$$
\phi(a, b) = a + b - \sqrt{a^2 + b^2}
$$

Complementarity condition $0 \leq a \perp b \geq 0$ is equivalent to $\phi(a, b) = 0$.

**Algorithm**: Apply Newton's method to the system of equations $\mathbf{\Phi}(\boldsymbol{\lambda}) = \mathbf{0}$ where $\mathbf{\Phi}$ stacks Fischer-Burmeister functions for all complementarity pairs.

**Pros**:
- Mathematically elegant
- Can handle exact Coulomb cone with appropriate reformulation

**Cons**:
- **Jacobian singularity**: $\nabla \phi$ is singular when $a = b = 0$ (common in friction: stick regime)
- **Convergence issues**: Newton's method unreliable for ill-conditioned contact systems (high mass ratios)
- **Computational cost**: Each Newton iteration requires full Jacobian factorization

**Recommended**: No. Too fragile for game/simulation use cases with diverse contact configurations.

## Recommendation

**Recommended approach**: **Option A — Box-Constrained LCP with Active Set Method**

**Implementation strategy**:
1. **Phase 1: Single-pass ASM with fixed bounds** (prototype):
   - Use $\mu_{\text{max}}$ (maximum friction coefficient in scene) to define static bounds $\lambda_{t_i} \in [-\mu_{\text{max}} \lambda_n^{\text{guess}} / \sqrt{2}, +\mu_{\text{max}} \lambda_n^{\text{guess}} / \sqrt{2}]$
   - Solve box-constrained LCP in single ASM pass (no outer iteration)
   - Assess accuracy: Do friction forces stay within correct per-contact cones?

2. **Phase 2: Outer iteration for variable bounds** (if Phase 1 accuracy insufficient):
   - Iterate: Solve box LCP → update friction bounds based on $\lambda_n$ → re-solve
   - Convergence criterion: $\|\lambda_n^{(k+1)} - \lambda_n^{(k)}\| < \epsilon$
   - Typical convergence: 2-5 outer iterations for well-conditioned systems

3. **Phase 3: Upgrade to pyramidal/octagonal cone** (future work):
   - Replace box with 4-sided pyramid (4 coupled constraints per contact)
   - Or 8-sided polygon (8 constraints per contact, <2% error)
   - Still solvable with ASM (polyhedral constraints remain linear inequalities)

## Interaction with Bilateral Constraint Solve (Phase 1 / Phase 2)

The existing constraint solver (Ticket 0031) uses a two-phase approach:
- **Phase 1**: Solve bilateral constraints (e.g., UnitQuaternionConstraint, DistanceConstraint) using direct LLT
- **Phase 2**: Solve unilateral contact constraints using Active Set Method

**Integration**: Friction constraints are part of **Phase 2** (contact solving). The bilateral solve is unchanged. Friction extends the contact constraint system from $C \times C$ (normal only) to $3C \times 3C$ (normal + 2 tangential per contact).

## Convergence Analysis

- **LCP monotonicity**: Effective mass matrix $\mathbf{A} = \mathbf{J}\mathbf{M}^{-1}\mathbf{J}^\top$ is positive semidefinite (symmetric, all eigenvalues $\geq 0$)
- **ASM convergence**: Guaranteed for monotone LCP (Cottle, Pang, Stone theorem)
- **Outer iteration convergence**: Monotone convergence (friction bounds tighten or stay constant with each iteration)
- **Termination**: Finite iterations (bounded by number of constraint bound activation changes, typically $\ll 3C$)
