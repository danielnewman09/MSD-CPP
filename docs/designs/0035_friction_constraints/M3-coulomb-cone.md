# M3. Coulomb Friction Cone Formulation

> **Parent**: [math-formulation.md](math-formulation.md)
> **Depends on**: [M2 (Friction Jacobian)](M2-friction-jacobian.md) — provides $\mathbf{J}_{t_1}, \mathbf{J}_{t_2}$
> **Required by**: [M4 (Complementarity)](M4-complementarity.md), [M5 (Solver Extension)](M5-solver-extension.md), [M6 (Energy Dissipation)](M6-energy-dissipation.md), [M8 (Numerical Examples)](M8-numerical-examples.md)

---

**Objective**: Formulate the Coulomb friction model within the LCP/complementarity framework, comparing exact cone and linearized approximations.

## Exact Coulomb Friction Cone

The Coulomb friction law states that the magnitude of the tangential friction force is bounded by the normal force:
$$
\|\boldsymbol{\lambda}_t\| \leq \mu \lambda_n
$$

where $\|\boldsymbol{\lambda}_t\| = \sqrt{\lambda_{t_1}^2 + \lambda_{t_2}^2}$ is the Euclidean norm.

**Geometric interpretation**: The admissible friction forces lie within a circular cone in 3D force space $(\lambda_n, \lambda_{t_1}, \lambda_{t_2})$.

**Issue**: This is a **second-order cone constraint** (also called Lorentz cone or ice-cream cone), which is **non-polyhedral** and **non-linear**. Standard LCP solvers (including the Active Set Method) require **linear inequality constraints**, so the exact cone cannot be solved directly without specialized second-order cone programming (SOCP) solvers.

## Linearized Approximations

To enable solution within an LCP framework, we approximate the circular friction cone with a **polyhedral cone** using linear inequalities.

### Option 1: Pyramidal Approximation (4-sided)

Approximate the circular cone with a square pyramid using $L^1$ norm:
$$
|\lambda_{t_1}| + |\lambda_{t_2}| \leq \mu \lambda_n
$$

**Equivalent linear inequalities** (4 faces):
$$
\begin{aligned}
\lambda_{t_1} + \lambda_{t_2} &\leq \mu \lambda_n \\
\lambda_{t_1} - \lambda_{t_2} &\leq \mu \lambda_n \\
-\lambda_{t_1} + \lambda_{t_2} &\leq \mu \lambda_n \\
-\lambda_{t_1} - \lambda_{t_2} &\leq \mu \lambda_n
\end{aligned}
$$

**Approximation quality**:
- **Inscribed circle**: Largest circle fitting inside pyramid has radius $r_{\text{in}} = \frac{\mu \lambda_n}{\sqrt{2}}$
- **Circumscribed circle**: Smallest circle containing pyramid has radius $r_{\text{out}} = \mu \lambda_n$
- **Error**: Under-approximates friction by factor of $\sqrt{2}$ along diagonal directions ($45°$ from axes)

**Accuracy**: Maximum error is $\approx 29\%$ (when friction acts along $\lambda_{t_1} = \lambda_{t_2}$ direction, pyramid allows force $\mu \lambda_n / \sqrt{2} \approx 0.71 \mu \lambda_n$ instead of $\mu \lambda_n$).

### Option 2: Box Approximation (Per-Axis Decoupled)

Approximate with a rectangular box using $L^\infty$ norm:
$$
|\lambda_{t_1}| \leq \mu \lambda_n \quad \text{and} \quad |\lambda_{t_2}| \leq \mu \lambda_n
$$

**Equivalent linear inequalities** (4 faces):
$$
\begin{aligned}
\lambda_{t_1} &\leq \mu \lambda_n \\
-\lambda_{t_1} &\leq \mu \lambda_n \\
\lambda_{t_2} &\leq \mu \lambda_n \\
-\lambda_{t_2} &\leq \mu \lambda_n
\end{aligned}
$$

**Approximation quality**:
- **Inscribed circle**: radius $r_{\text{in}} = \mu \lambda_n$
- **Circumscribed circle**: radius $r_{\text{out}} = \sqrt{2} \mu \lambda_n$
- **Error**: Over-approximates friction by factor of $\sqrt{2}$ along diagonal directions

**Accuracy**: Maximum error is $\approx 41\%$ (when friction acts along diagonal, box allows force $\sqrt{2} \mu \lambda_n \approx 1.41 \mu \lambda_n$ instead of $\mu \lambda_n$).

**Benefit**: Decouples $\lambda_{t_1}$ and $\lambda_{t_2}$, allowing independent bounds per axis. Simplest to implement in Active Set Method (each friction direction treated as independent constraint with bounds $[-\mu \lambda_n, +\mu \lambda_n]$).

### Option 3: Scaled Per-Axis Bounds

Compensate for box over-approximation by scaling bounds:
$$
|\lambda_{t_1}| \leq \frac{\mu}{\sqrt{2}} \lambda_n \quad \text{and} \quad |\lambda_{t_2}| \leq \frac{\mu}{\sqrt{2}} \lambda_n
$$

**Approximation quality**:
- **Inscribed circle**: radius $r_{\text{in}} = \frac{\mu}{\sqrt{2}} \lambda_n$
- **Circumscribed circle**: radius $r_{\text{out}} = \mu \lambda_n$
- **Error**: Under-approximates friction by factor of $\sqrt{2}$ along axis-aligned directions, exact along diagonal

**Accuracy**: Maximum error is $\approx 29\%$ (when friction acts along $\lambda_{t_1}$ or $\lambda_{t_2}$ alone, box allows force $\mu / \sqrt{2} \approx 0.71 \mu$ instead of $\mu$).

This is **equivalent to the pyramidal approximation** but formulated with per-axis bounds.

### Option 4: Polyhedral Cone with N Faces

Use a regular $N$-sided polygon approximation with $N$ linear inequalities. As $N \to \infty$, the approximation converges to the exact circular cone.

For $N = 8$ (octagonal approximation):
$$
\cos\left(\frac{\pi}{N}\right) \sqrt{\lambda_{t_1}^2 + \lambda_{t_2}^2} \leq \mu \lambda_n
$$
approximated with 8 linear inequalities in the form:
$$
\lambda_{t_1} \cos\theta_k + \lambda_{t_2} \sin\theta_k \leq \frac{\mu \lambda_n}{\cos(\pi/N)} \quad \text{for } k = 0, 1, \ldots, 7
$$
where $\theta_k = 2\pi k / N$.

**Approximation quality** ($N=8$):
- Error: $< 2\%$ (inscribed circle radius is $\mu \lambda_n \cos(\pi/8) \approx 0.9239 \mu \lambda_n$)

**Solver complexity**: Requires 8 inequality constraints per contact (vs 4 for pyramid/box), increasing Active Set iterations.

## Comparison and Recommendation

| Approximation | Linear Inequalities | Max Error | Solver Complexity | Energy Conservation |
|---------------|---------------------|-----------|-------------------|---------------------|
| Exact Cone (SOCP) | 1 second-order cone | 0% | SOCP solver required | Exact |
| Pyramidal (4-sided) | 4 per contact | 29% under-approx | Low (4 constraints) | Conservative (safe) |
| Box (per-axis) | 4 per contact | 41% over-approx | Lowest (decoupled) | Non-conservative (may inject energy) |
| Scaled Box | 4 per contact | 29% under-approx | Lowest (decoupled) | Conservative (safe) |
| Octagonal (8-sided) | 8 per contact | 2% under-approx | Moderate (8 constraints) | Nearly exact |

**Recommendation**: **Scaled per-axis box approximation** (Option 3) for initial implementation:
- **Simplest integration**: Maps directly to box-constrained LCP with bounds $\lambda_{t_i} \in [-\mu \lambda_n / \sqrt{2}, +\mu \lambda_n / \sqrt{2}]$
- **Decoupled constraints**: Each friction direction is independent, fitting naturally into Active Set Method
- **Conservative approximation**: Under-approximates friction (safer than over-approximation which can inject energy)
- **Adequate accuracy**: 29% error acceptable for game/simulation (not critical for engineering analysis)
- **Upgrade path**: Can later extend to pyramidal (4 coupled constraints) or octagonal (8 constraints) for higher accuracy

**Future work**: True SOCP solution with exact Coulomb cone using specialized cone solvers (e.g., ECOS, SCS) for engineering-grade accuracy.
