# M7. Numerical Stability Analysis

> **Parent**: [math-formulation.md](math-formulation.md)
> **Depends on**: [M2 (Friction Jacobian)](M2-friction-jacobian.md), [M5 (Solver Extension)](M5-solver-extension.md), [M6 (Energy Dissipation)](M6-energy-dissipation.md)
> **Required by**: [M8 (Numerical Examples)](M8-numerical-examples.md)

---

**Objective**: Analyze conditioning, stability, sparsity, mass ratio sensitivity, regularization strategies, and degenerate cases.

## Effect of Friction on Effective Mass Matrix Conditioning

**Effective mass matrix** (without friction, $C$ contacts):
$$
\mathbf{A}_{\text{normal}} = \mathbf{J}_n \mathbf{M}^{-1} \mathbf{J}_n^\top \in \mathbb{R}^{C \times C}
$$

**With friction** ($C$ contacts, 3 rows per contact):
$$
\mathbf{A}_{\text{friction}} = \mathbf{J} \mathbf{M}^{-1} \mathbf{J}^\top \in \mathbb{R}^{3C \times 3C}
$$

where $\mathbf{J}$ stacks normal and tangential Jacobians:
$$
\mathbf{J} =
\begin{bmatrix}
\mathbf{J}_{n,1} \\
\mathbf{J}_{t_1,1} \\
\mathbf{J}_{t_2,1} \\
\vdots \\
\mathbf{J}_{n,C} \\
\mathbf{J}_{t_1,C} \\
\mathbf{J}_{t_2,C}
\end{bmatrix}
\in \mathbb{R}^{3C \times 12}
$$

**Sparsity pattern**: For two-body contacts, each contact interacts with only 2 bodies (12 DOFs). Global Jacobian is sparse (each row has at most 12 non-zero entries). However, after forming $\mathbf{A} = \mathbf{J}\mathbf{M}^{-1}\mathbf{J}^\top$, the matrix is **dense** within each 3x3 block (normal and tangential constraints for the same contact couple together) and has **off-diagonal blocks** for contacts involving the same body.

### Conditioning Analysis

**Condition number** $\kappa(\mathbf{A}) = \|\mathbf{A}\| \|\mathbf{A}^{-1}\| = \lambda_{\max} / \lambda_{\min}$ where $\lambda_{\max}, \lambda_{\min}$ are largest and smallest eigenvalues.

**Factors affecting conditioning**:
1. **Mass ratio**: If two bodies in contact have mass ratio $m_A / m_B = r$, then effective mass in normal direction is:
   $$
   m_{\text{eff}} = \frac{m_A m_B}{m_A + m_B} = \frac{m_B}{1 + 1/r}
   $$
   For $r \gg 1$ (heavy object on light object), $m_{\text{eff}} \approx m_B$ (small).
   For $r \ll 1$ (light object on heavy object), $m_{\text{eff}} \approx m_A$ (small).

   Extreme mass ratios produce small eigenvalues in $\mathbf{A}$, increasing condition number.

2. **Constraint coupling**: Normal and tangential constraints for the same contact share the same contact point geometry. The cross-coupling terms $\mathbf{J}_n \mathbf{M}^{-1} \mathbf{J}_{t_i}^\top$ are generally non-zero (unless $\mathbf{r} \times \mathbf{n}$ is perfectly aligned with $\mathbf{r} \times \mathbf{t}_i$, which is rare).

3. **Friction coefficient**: High $\mu$ allows large tangential forces, but does not directly affect $\mathbf{A}$ (friction bounds are imposed after solving, not in the matrix). However, friction forces couple to normal force, which can affect iterative convergence in outer loop (for variable bounds).

**Empirical estimate** (from Ticket 0034 normal constraint analysis):
- Mass ratio $10^6:1$ produces condition number $\kappa(\mathbf{A}) \approx 10^{12}$ (near double-precision limit $10^{16}$)
- With friction, $3 \times 3$ blocks per contact couple normal and tangential directions
- Off-diagonal blocks couple contacts on the same body

**Expected conditioning with friction**:
- **Best case** (well-separated contacts, similar masses): $\kappa(\mathbf{A}) \approx \kappa(\mathbf{A}_{\text{normal}})$ (friction does not significantly worsen conditioning)
- **Worst case** (nearly parallel tangent directions, extreme mass ratios): $\kappa(\mathbf{A}) \approx 3 \times \kappa(\mathbf{A}_{\text{normal}})$ (factor of 3 from dimension increase, cross-coupling)

**Conclusion**: Friction increases constraint dimension (3x rows) but does not fundamentally change conditioning. LLT decomposition remains viable up to mass ratios $\sim 10^6:1$ (same as normal constraints).

## Mass Ratio Sensitivity

**Test scenario**: Body A (mass $m_A = 1$ kg) contacts Body B (mass $m_B = 10^6$ kg).

**Normal effective mass**:
$$
m_{\text{eff},n} = \frac{m_A m_B}{m_A + m_B} \approx m_A = 1 \text{ kg}
$$

**Tangential effective mass** (assuming $\mathbf{r}_A, \mathbf{r}_B$ have similar magnitudes):
$$
m_{\text{eff},t} \approx m_{\text{eff},n} = 1 \text{ kg}
$$

**Observation**: Friction does not worsen mass ratio sensitivity compared to normal constraints. Both normal and tangential constraints see the same effective mass (determined by contact geometry and mass distribution).

**Mitigation**: Existing LLT solver (Eigen::LLT) handles condition numbers up to $10^{12}$ (Ticket 0034 validation). No additional regularization needed for friction beyond what's already applied to normal constraints.

## Regularization Strategy

**Current approach** (Ticket 0032/0034): No explicit regularization (Baumgarte stabilization provides implicit regularization via $\alpha, \beta$ terms in RHS).

**For friction constraints**:
- **No Baumgarte stabilization**: $\alpha = \beta = 0$ for tangential constraints (friction is dissipative, no position error to stabilize)
- **Possible regularization**: Add small diagonal term to $\mathbf{A}$ to improve conditioning:
  $$
  \mathbf{A}_{\text{reg}} = \mathbf{A} + \epsilon \mathbf{I}
  $$
  where $\epsilon \approx 10^{-8}$ is a small regularization parameter.

**Trade-off**: Regularization improves numerical stability but introduces compliance (artificial "softness" in friction constraints).

**Recommendation**: Start without regularization. If LLT decomposition fails (non-positive-definite $\mathbf{A}$), add minimal regularization $\epsilon \sim 10^{-10}$ (much smaller than physical constraint tolerances).

## Degenerate Cases

### Case 1: Zero Normal Force with Nonzero Tangential Velocity

**Scenario**: Contact breaks ($\lambda_n = 0$) but tangential velocity $\mathbf{v}_t \neq \mathbf{0}$ remains.

**Friction cone constraint**: $\|\boldsymbol{\lambda}_t\| \leq \mu \lambda_n = 0 \Rightarrow \boldsymbol{\lambda}_t = \mathbf{0}$

**Expected behavior**: Friction forces vanish instantly when contact breaks (no "sticky" friction after liftoff).

**Numerical handling**: Active Set Method naturally handles this:
- When $\lambda_n \to 0$, friction bounds shrink to zero
- Friction constraints become inactive (zero force)
- No special-case code needed

### Case 2: Sliding Reversal (Direction Change)

**Scenario**: Tangential velocity changes sign: $v_{t_1}(t) > 0$ at time $t$, then $v_{t_1}(t + \Delta t) < 0$ after timestep.

**Physical expectation**: Friction opposes motion in both directions (always dissipates energy).

**Numerical challenge**: Box-constrained LCP allows $\lambda_{t_1}$ to change sign without restriction. Does this produce correct reversal behavior?

**Analysis**:
- **Before reversal**: $v_{t_1} > 0 \Rightarrow \lambda_{t_1} = -\mu \lambda_n / \sqrt{2}$ (lower bound active, friction opposes positive velocity)
- **After reversal**: $v_{t_1} < 0 \Rightarrow \lambda_{t_1} = +\mu \lambda_n / \sqrt{2}$ (upper bound active, friction opposes negative velocity)
- **Transition**: Active Set Method switches active bound when velocity crosses zero

**Conclusion**: Box friction correctly handles reversal (friction force sign follows velocity sign to maintain dissipation).

### Case 3: Nearly Parallel Tangent Directions (Degenerate Contact Geometry)

**Scenario**: Two contacts on the same body have nearly identical contact normals $\mathbf{n}_1 \approx \mathbf{n}_2$.

**Issue**: Tangent bases $\{\mathbf{t}_{1,1}, \mathbf{t}_{2,1}\}$ and $\{\mathbf{t}_{1,2}, \mathbf{t}_{2,2}\}$ span nearly the same tangent plane. This creates near-linear dependence in the Jacobian:
$$
\mathbf{J}_{t_1,1} \approx \mathbf{J}_{t_1,2} \quad \text{and} \quad \mathbf{J}_{t_2,1} \approx \mathbf{J}_{t_2,2}
$$

**Effect on conditioning**: $\mathbf{A} = \mathbf{J}\mathbf{M}^{-1}\mathbf{J}^\top$ has nearly rank-deficient rows, increasing condition number.

**Mitigation**: Contact manifold reduction (Ticket 0032) already addresses this by merging nearby contacts into a single representative contact. For friction, the same contact reduction applies (merged contact has single normal and single tangent basis).

**Numerical handling**: If two contacts remain after reduction, their tangent bases are constructed independently (potentially different orientations within the same tangent plane). LLT decomposition remains stable as long as $\mathbf{A}$ is positive definite (guaranteed for contacts with positive normal forces).

### Case 4: Contact at Zero Velocity (Stick-Slip Transition Jitter)

**Scenario**: Object on inclined plane with friction. At equilibrium, tangential velocity $\mathbf{v}_t = \mathbf{0}$ and friction force $\boldsymbol{\lambda}_t = m \mathbf{g}_{\text{tangent}}$ (balances gravity component). Small numerical error in $\mathbf{v}_t$ can cause oscillation between stick and slip.

**Issue**: Complementarity solver might "chatter" between stick (interior solution) and slip (boundary solution) if velocity is near zero but not exactly zero.

**Mitigation**: **Velocity threshold** (similar to restitution rest velocity threshold in Ticket 0032):
- If $\|\mathbf{v}_t\| < v_{\text{rest}} \approx 0.01$ m/s, clamp $\mathbf{v}_t \gets \mathbf{0}$ (force stick regime)
- This prevents jitter when object is "nearly stationary"

**Recommendation**: Implement velocity threshold as post-processing step after LCP solve. If $\|\mathbf{v}_t\| < v_{\text{rest}}$, set $\mathbf{v}_t = \mathbf{0}$ and allow $\boldsymbol{\lambda}_t$ to be interior solution (balancing forces without sliding).

## Special Cases

### Zero Friction Coefficient ($\mu = 0$)

**Condition**: $\mu = 0$ (frictionless contact)

**Mathematical Treatment**:
$$
\|\boldsymbol{\lambda}_t\| \leq 0 \Rightarrow \boldsymbol{\lambda}_t = \mathbf{0}
$$

Friction constraints become **inactive** (zero force). System reduces to normal-only constraints (current implementation).

**Numerical Handling**:
- Skip friction constraint rows when $\mu = 0$ (do not add to Jacobian)
- Constraint system remains $C \times C$ instead of $3C \times 3C$

### Infinite Friction Coefficient ($\mu \to \infty$)

**Condition**: $\mu \to \infty$ (infinite friction, no sliding allowed)

**Mathematical Treatment**:

Friction cone becomes infinitely wide. Any tangential force is admissible, so friction can prevent all sliding:
$$
\mathbf{v}_t = \mathbf{0} \quad \text{(stick always, never slip)}
$$

This is equivalent to a **bilateral tangential constraint** (equality constraint $\mathbf{v}_t = \mathbf{0}$).

**Numerical Handling**:
- For $\mu > \mu_{\text{large}}$ (e.g., $\mu > 100$), treat as bilateral constraint:
  - Remove bounds on $\lambda_{t_i}$ (allow any real value)
  - Enforce $v_{t_i} = 0$ exactly (LCP becomes standard linear system)
- Alternatively, cap friction bounds at large value: $\lambda_{t_i} \in [-M, +M]$ where $M = \mu_{\text{cap}} \lambda_n$ with $\mu_{\text{cap}} = 100$

### Grazing Contact (Tangential Velocity Parallel to Normal)

**Condition**: Relative velocity at contact is nearly parallel to normal: $\mathbf{v}_{\text{rel}} \approx v_n \mathbf{n}$, so $\mathbf{v}_t \approx \mathbf{0}$ (minimal sliding).

**Risk**: Division by zero in slip direction computation $\mathbf{v}_t / \|\mathbf{v}_t\|$ if $\|\mathbf{v}_t\| \to 0$.

**Mitigation**: Maximum dissipation principle automatically handles this:
- If $\|\mathbf{v}_t\| < v_{\text{rest}}$, clamp to stick regime ($\mathbf{v}_t = \mathbf{0}$, $\boldsymbol{\lambda}_t$ interior to cone)
- No division by zero occurs (stick solution does not require velocity direction)

## Condition Number Analysis

**Key matrix**: Effective mass matrix $\mathbf{A} = \mathbf{J}\mathbf{M}^{-1}\mathbf{J}^\top \in \mathbb{R}^{3C \times 3C}$

**Condition number estimate**:
$$
\kappa(\mathbf{A}) \approx \frac{m_{\max}}{m_{\min}} \times \frac{I_{\max}}{I_{\min}}
$$
where $m_{\max}, m_{\min}$ are largest and smallest body masses, and $I_{\max}, I_{\min}$ are largest and smallest principal moments of inertia.

**Typical values**:
- Mass ratio: $10^3:1$ (spacecraft $10^3$ kg vs debris $1$ kg) → $\kappa \sim 10^6$
- Mass ratio: $10^6:1$ (asteroid vs small object) → $\kappa \sim 10^{12}$ (near double-precision limit)

**With friction**: Condition number increases by factor of at most 3 (from 3x3 block coupling), so:
$$
\kappa(\mathbf{A}_{\text{friction}}) \lesssim 3 \times \kappa(\mathbf{A}_{\text{normal}})
$$

**Solver stability**: Eigen::LLT (Cholesky decomposition) stable for $\kappa < 10^{14}$ (double precision), so mass ratios up to $10^6:1$ are supported.

## Potential Instabilities

| Operation | Risk | Mitigation |
|-----------|------|------------|
| Division by $\|\mathbf{v}_t\|$ in slip direction | Zero tangential velocity | Velocity threshold: if $\|\mathbf{v}_t\| < v_{\text{rest}}$, use stick regime (no division) |
| LLT decomposition failure (non-positive-definite $\mathbf{A}$) | Nearly parallel constraints, extreme mass ratios | Regularization: $\mathbf{A} + \epsilon \mathbf{I}$ with $\epsilon \sim 10^{-10}$ |
| Friction bounds $\mu \lambda_n$ with $\lambda_n \approx 0$ | Contact at threshold of breaking | Allow $\lambda_{t_i} = 0$ when $\lambda_n < \lambda_{\text{min}} \sim 10^{-6}$ N |
| Outer iteration divergence (friction bounds oscillate) | Ill-conditioned contact configuration | Iteration limit (max 10 iterations) + fallback to last valid solution |
| Tangent basis discontinuity at $\|n_x\| = \|n_y\| = \|n_z\|$ | Contact normal exactly at branch boundary | Duff et al. method is continuous at boundaries (cross product varies smoothly) |

## Precision Requirements

| Computation | Recommended Precision | Rationale |
|-------------|----------------------|-----------|
| Tangent basis construction | `float` (32-bit) | Orthonormality maintained to $\sim 10^{-6}$, sufficient for physics simulation |
| Friction Jacobian | `double` (64-bit) | Cross product and matrix multiply accumulate errors; double precision avoids accumulation in large systems |
| Effective mass matrix $\mathbf{A}$ | `double` (64-bit) | Condition number up to $10^{12}$ requires double precision for stable LLT |
| LLT decomposition | `double` (64-bit) | Cholesky factorization sensitive to rounding errors for ill-conditioned matrices |
| Friction force $\boldsymbol{\lambda}_t$ | `double` (64-bit) | Multiplied by Jacobian transpose (accumulated errors), needs double precision |
| Velocity threshold $v_{\text{rest}}$ | `float` (32-bit) | Threshold comparison tolerant to single precision |

**Recommendation**: Use `double` throughout constraint solver (existing implementation already uses `Eigen::VectorXd`, `Eigen::MatrixXd` for constraints).

## Tolerances

| Comparison | Tolerance | Rationale |
|------------|-----------|-----------|
| Tangent orthogonality $\|\mathbf{t}_1 \cdot \mathbf{t}_2\|$ | $10^{-6}$ | Ensures tangent basis is orthonormal to 6 decimal places (sufficient for physics) |
| Stick velocity threshold $\|\mathbf{v}_t\| < v_{\text{rest}}$ | $0.01$ m/s | Objects moving slower than 1 cm/s treated as stationary (avoids stick-slip jitter) |
| LCP convergence (Active Set Method) | $10^{-6}$ | KKT residual tolerance (dual feasibility check) |
| Friction cone violation $\|\boldsymbol{\lambda}_t\| - \mu \lambda_n$ | $10^{-4}$ N | Allow small violation (numerical error in box approximation) |
| Outer iteration convergence $\|\lambda_n^{(k+1)} - \lambda_n^{(k)}\|$ | $10^{-6}$ N | Normal force change between iterations (friction bounds stabilized) |

## Iterative Methods

| Algorithm | Convergence Criterion | Max Iterations | Fallback |
|-----------|----------------------|----------------|----------|
| Active Set Method (inner LCP solve) | KKT conditions satisfied ($\|\mathbf{w}\| < 10^{-6}$) | $\min(2 \times 3C, 100)$ | Return last feasible solution (warn) |
| Outer iteration (variable friction bounds) | $\|\Delta \lambda_n\| < 10^{-6}$ | 10 | Use last iteration (bounds approximate) |
| LLT decomposition | N/A (direct method) | 1 (single factorization) | Return non-converged flag if decomposition fails |

**Notes**:
- **Active Set Method**: Typically converges in $< C$ iterations (changes to working set), but safety cap at $2 \times 3C$ prevents infinite loops
- **Outer iteration**: Empirically converges in 2-5 iterations for most contact configurations
- **No warm-starting**: Each timestep solves from scratch (previous timestep's $\boldsymbol{\lambda}$ not reused as initial guess). Future optimization could use warm-starting to reduce iterations.
