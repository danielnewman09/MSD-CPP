# M4. Complementarity Conditions for Friction

> **Parent**: [math-formulation.md](math-formulation.md)
> **Depends on**: [M2 (Friction Jacobian)](M2-friction-jacobian.md), [M3 (Coulomb Cone)](M3-coulomb-cone.md)
> **Required by**: [M5 (Solver Extension)](M5-solver-extension.md), [M6 (Energy Dissipation)](M6-energy-dissipation.md)

---

**Objective**: Derive the full complementarity system combining normal and friction constraints, formulating stick/slip conditions within the LCP framework.

## Normal Constraint Complementarity (Signorini Conditions)

Non-penetration constraint (from Ticket 0032):
$$
\begin{aligned}
\lambda_n &\geq 0 \quad \text{(no pulling forces)} \\
v_n &\geq 0 \quad \text{(no penetration)} \\
\lambda_n \cdot v_n &= 0 \quad \text{(complementarity: force is zero unless in contact)}
\end{aligned}
$$

These are standard **unilateral constraint complementarity conditions** for a Linear Complementarity Problem (LCP).

## Friction Complementarity Conditions

Friction obeys two distinct regimes:

**Regime 1: Stick (Static Friction)**
- Condition: Tangential velocity is zero: $\mathbf{v}_t = \mathbf{0}$ (no sliding)
- Constraint: Friction force lies strictly inside friction cone: $\|\boldsymbol{\lambda}_t\| < \mu \lambda_n$
- Physical interpretation: Friction exactly opposes applied tangential forces up to the friction limit

**Regime 2: Slip (Kinetic Friction)**
- Condition: Friction force saturates the cone: $\|\boldsymbol{\lambda}_t\| = \mu \lambda_n$
- Constraint: Tangential velocity opposes friction force: $\mathbf{v}_t = -\gamma \frac{\boldsymbol{\lambda}_t}{\|\boldsymbol{\lambda}_t\|}$ for some $\gamma \geq 0$
- Physical interpretation: Friction force reaches maximum magnitude and acts opposite to sliding direction

## Maximum Dissipation Principle

The friction force $\boldsymbol{\lambda}_t$ is chosen to **maximize energy dissipation** subject to the friction cone constraint:
$$
\boldsymbol{\lambda}_t = \arg\max_{\|\boldsymbol{\lambda}\| \leq \mu \lambda_n} \boldsymbol{\lambda}^\top \mathbf{v}_t
$$

This produces:
$$
\boldsymbol{\lambda}_t =
\begin{cases}
-\mu \lambda_n \frac{\mathbf{v}_t}{\|\mathbf{v}_t\|} & \text{if } \|\mathbf{v}_t\| > 0 \quad \text{(slip: friction opposes velocity)} \\
\text{any } \boldsymbol{\lambda} \text{ with } \|\boldsymbol{\lambda}\| \leq \mu \lambda_n & \text{if } \mathbf{v}_t = \mathbf{0} \quad \text{(stick: friction balances applied forces)}
\end{cases}
$$

## Combined Complementarity Formulation

The complete system is a **Mixed Complementarity Problem (MCP)**, not a pure LCP, due to the coupling between $\lambda_n$ and bounds on $\boldsymbol{\lambda}_t$.

For the **box friction approximation** (scaled per-axis bounds):
$$
\begin{aligned}
&\text{Normal constraint:} \\
&\quad 0 \leq \lambda_n \perp (v_n + \text{restitution/stabilization terms}) \geq 0 \\[8pt]
&\text{Friction constraints (per direction } i \in \{1, 2\}\text{):} \\
&\quad -\frac{\mu}{\sqrt{2}} \lambda_n \leq \lambda_{t_i} \leq \frac{\mu}{\sqrt{2}} \lambda_n \\
&\quad v_{t_i} = \mathbf{J}_{t_i} \dot{\mathbf{q}} \\[8pt]
&\text{Complementarity for each friction direction:} \\
&\quad \text{If } \lambda_{t_i} = -\frac{\mu}{\sqrt{2}} \lambda_n, \text{ then } v_{t_i} \leq 0 \quad \text{(lower bound active)} \\
&\quad \text{If } \lambda_{t_i} = +\frac{\mu}{\sqrt{2}} \lambda_n, \text{ then } v_{t_i} \geq 0 \quad \text{(upper bound active)} \\
&\quad \text{If } -\frac{\mu}{\sqrt{2}} \lambda_n < \lambda_{t_i} < +\frac{\mu}{\sqrt{2}} \lambda_n, \text{ then } v_{t_i} = 0 \quad \text{(stick)}
\end{aligned}
$$

This is a **box-constrained LCP** where each tangential constraint has variable bounds that depend on $\lambda_n$.

## Reformulation as Nonlinear Complementarity Problem (NCP)

An alternative formulation uses NCP functions (e.g., Fischer-Burmeister function) to encode the friction cone constraint. Define:
$$
\phi(\lambda_n, \lambda_{t_1}, \lambda_{t_2}) = \sqrt{\lambda_{t_1}^2 + \lambda_{t_2}^2 + \lambda_n^2} - \left(\mu \lambda_n + \sqrt{\lambda_{t_1}^2 + \lambda_{t_2}^2}\right)
$$

The complementarity condition is:
$$
\phi(\lambda_n, \lambda_{t_1}, \lambda_{t_2}) = 0
$$

This formulation is **non-linear** and requires specialized NCP solvers (e.g., PATH solver).

**Not recommended** for game/simulation due to computational cost and convergence challenges with ill-conditioned contact systems.
