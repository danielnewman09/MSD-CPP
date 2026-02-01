# M6. Energy Dissipation Analysis

> **Parent**: [math-formulation.md](math-formulation.md)
> **Depends on**: [M3 (Coulomb Cone)](M3-coulomb-cone.md), [M4 (Complementarity)](M4-complementarity.md)
> **Required by**: [M7 (Numerical Stability)](M7-numerical-stability.md)

---

**Objective**: Prove that the friction formulation dissipates energy (never injects energy into the system).

## Friction Power

The instantaneous power dissipated by friction at a contact point is:
$$
P_f = \boldsymbol{\lambda}_t^\top \mathbf{v}_t = \lambda_{t_1} v_{t_1} + \lambda_{t_2} v_{t_2}
$$

For energy dissipation (not injection), we require:
$$
P_f \leq 0 \quad \text{(friction removes kinetic energy or is neutral)}
$$

## Stick Regime Energy Analysis

**Condition**: $\mathbf{v}_t = \mathbf{0}$ (zero tangential velocity)

**Power**:
$$
P_f = \boldsymbol{\lambda}_t^\top \mathbf{0} = 0
$$

**Conclusion**: No energy transfer (conservative constraint).

## Slip Regime Energy Analysis

**Condition**: $\mathbf{v}_t \neq \mathbf{0}$, friction saturates cone: $\|\boldsymbol{\lambda}_t\| = \mu \lambda_n$

**Maximum dissipation principle** gives:
$$
\boldsymbol{\lambda}_t = -\mu \lambda_n \frac{\mathbf{v}_t}{\|\mathbf{v}_t\|}
$$

**Power**:
$$
P_f = \boldsymbol{\lambda}_t^\top \mathbf{v}_t = \left(-\mu \lambda_n \frac{\mathbf{v}_t}{\|\mathbf{v}_t\|}\right)^\top \mathbf{v}_t = -\mu \lambda_n \frac{\mathbf{v}_t^\top \mathbf{v}_t}{\|\mathbf{v}_t\|} = -\mu \lambda_n \|\mathbf{v}_t\|
$$

Since $\mu \geq 0$, $\lambda_n \geq 0$ (no pulling), and $\|\mathbf{v}_t\| \geq 0$:
$$
P_f = -\mu \lambda_n \|\mathbf{v}_t\| \leq 0
$$

**Conclusion**: Energy is strictly dissipated when sliding ($\mathbf{v}_t \neq \mathbf{0}$).

## Energy Dissipation over Timestep

Integrating power over a timestep $\Delta t$:
$$
\Delta E = \int_0^{\Delta t} P_f \, dt = \int_0^{\Delta t} \boldsymbol{\lambda}_t^\top \mathbf{v}_t \, dt
$$

For stick ($\mathbf{v}_t = \mathbf{0}$):
$$
\Delta E = 0
$$

For slip ($\boldsymbol{\lambda}_t = -\mu \lambda_n \mathbf{v}_t / \|\mathbf{v}_t\|$):
$$
\Delta E = -\mu \lambda_n \int_0^{\Delta t} \|\mathbf{v}_t\| \, dt \leq 0
$$

**Global energy balance**: Summing over all contacts:
$$
\Delta E_{\text{total}} = \sum_{i=1}^{k} \Delta E_i \leq 0
$$

**Conclusion**: Total kinetic energy is monotonically non-increasing due to friction (barring external forces).

## Interaction with Baumgarte Stabilization

**Baumgarte terms** (from Ticket 0032): Normal constraint includes position and velocity error feedback:
$$
b_n = -\mathbf{J}_n \mathbf{M}^{-1} \mathbf{F}_{\text{ext}} - \alpha C_n - \beta \dot{C}_n + \text{restitution terms}
$$

where $\alpha, \beta > 0$ are stabilization gains.

**Concern**: Can Baumgarte stabilization combined with friction inject energy?

**Analysis**:
- **Baumgarte for normal constraint**: Adds restoring force proportional to penetration depth ($\alpha C_n$) and penetration velocity ($\beta \dot{C}_n$). This can inject energy if restitution $e > 1$ (super-elastic), but for $e \leq 1$, energy is dissipated or conserved.
- **Baumgarte for friction constraints**: Friction constraints do not have position-level errors (no "tangential penetration"). Only velocity-level constraint: $v_{t_i} = 0$ (stick) or $v_{t_i} \neq 0$ (slip). Therefore, no Baumgarte position term for friction.
- **Velocity stabilization**: If tangential velocity constraint includes $\beta \dot{C}_{t_i}$ term, this could inject energy if $\beta < 0$ or if combined with restitution in tangential direction. **However**, friction constraints should NOT include restitution (friction does not "bounce" tangentially). Therefore, $\beta = 0$ for friction constraints.

**Recommendation**:
- **Normal constraints**: Use existing Baumgarte stabilization with $\alpha, \beta > 0$ and restitution $e \in [0, 1]$
- **Friction constraints**: Set $\alpha = \beta = 0$ (no Baumgarte stabilization for tangential direction, friction is purely dissipative)

**Energy injection risk**: Friction with $\alpha = \beta = 0$ is guaranteed non-injective. Normal constraints with $e \leq 1$ are non-injective. Combined system dissipates energy.

## Numerical Energy Drift

**Discrete timestep effects**: Symplectic integrators (semi-implicit Euler, used in codebase) conserve energy over long timescales for Hamiltonian systems. However, constraint violation can introduce artificial energy.

**Mitigation**: Active Set Method produces exact LCP solution satisfying complementarity conditions to machine precision. This minimizes spurious energy injection compared to approximate solvers (e.g., PGS with finite iterations).

**Validation**: Energy monitoring in tests (AC7) verifies monotonic energy decrease.
