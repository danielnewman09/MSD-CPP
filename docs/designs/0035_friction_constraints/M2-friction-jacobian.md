# M2. Friction Constraint Jacobian Derivation

> **Parent**: [math-formulation.md](math-formulation.md)
> **Depends on**: [M1 (Tangent Basis)](M1-tangent-basis.md) — provides $\{\mathbf{t}_1, \mathbf{t}_2\}$
> **Required by**: [M3 (Coulomb Cone)](M3-coulomb-cone.md), [M4 (Complementarity)](M4-complementarity.md), [M5 (Solver Extension)](M5-solver-extension.md), [M7 (Numerical Stability)](M7-numerical-stability.md), [M8 (Numerical Examples)](M8-numerical-examples.md)

---

**Objective**: Derive the Jacobian matrices $\mathbf{J}_{t_1}, \mathbf{J}_{t_2} \in \mathbb{R}^{1 \times 12}$ relating generalized velocities $\dot{\mathbf{q}}$ to tangential relative velocities $v_{t_1}, v_{t_2}$.

## Relative Velocity at Contact Point

For a contact point on rigid body $A$ at world position $\mathbf{p}$:
$$
\mathbf{v}_A^{\text{contact}} = \dot{\mathbf{x}}_A + \boldsymbol{\omega}_A \times \mathbf{r}_A
$$
where:
- $\dot{\mathbf{x}}_A \in \mathbb{R}^3$ is linear velocity of body A's center of mass
- $\boldsymbol{\omega}_A \in \mathbb{R}^3$ is angular velocity of body A
- $\mathbf{r}_A = \mathbf{p} - \mathbf{x}_A$ is contact point position relative to center of mass

Similarly for body B:
$$
\mathbf{v}_B^{\text{contact}} = \dot{\mathbf{x}}_B + \boldsymbol{\omega}_B \times \mathbf{r}_B
$$

**Relative contact velocity** (velocity of A relative to B at contact point):
$$
\mathbf{v}_{\text{rel}} = \mathbf{v}_A^{\text{contact}} - \mathbf{v}_B^{\text{contact}} = (\dot{\mathbf{x}}_A + \boldsymbol{\omega}_A \times \mathbf{r}_A) - (\dot{\mathbf{x}}_B + \boldsymbol{\omega}_B \times \mathbf{r}_B)
$$

## Tangential Velocity Components

Project relative velocity onto tangent directions:
$$
v_{t_1} = \mathbf{t}_1 \cdot \mathbf{v}_{\text{rel}} = \mathbf{t}_1^\top \mathbf{v}_{\text{rel}}
$$
$$
v_{t_2} = \mathbf{t}_2 \cdot \mathbf{v}_{\text{rel}} = \mathbf{t}_2^\top \mathbf{v}_{\text{rel}}
$$

## Jacobian Construction

Expand $v_{t_1}$:
$$
v_{t_1} = \mathbf{t}_1^\top [(\dot{\mathbf{x}}_A + \boldsymbol{\omega}_A \times \mathbf{r}_A) - (\dot{\mathbf{x}}_B + \boldsymbol{\omega}_B \times \mathbf{r}_B)]
$$

Using the vector triple product identity $\mathbf{a} \cdot (\mathbf{b} \times \mathbf{c}) = \mathbf{c} \cdot (\mathbf{a} \times \mathbf{b})$:
$$
\mathbf{t}_1^\top (\boldsymbol{\omega}_A \times \mathbf{r}_A) = \mathbf{r}_A \cdot (\mathbf{t}_1 \times \boldsymbol{\omega}_A) = -\boldsymbol{\omega}_A \cdot (\mathbf{r}_A \times \mathbf{t}_1) = (\mathbf{r}_A \times \mathbf{t}_1)^\top \boldsymbol{\omega}_A
$$

Therefore:
$$
v_{t_1} = \mathbf{t}_1^\top \dot{\mathbf{x}}_A + (\mathbf{r}_A \times \mathbf{t}_1)^\top \boldsymbol{\omega}_A - \mathbf{t}_1^\top \dot{\mathbf{x}}_B - (\mathbf{r}_B \times \mathbf{t}_1)^\top \boldsymbol{\omega}_B
$$

### Quaternion Kinematics (Full 14-DOF Formulation)

Recalling that generalized velocities include quaternion derivatives $\dot{\mathbf{Q}}$ (4 components per body), but angular velocity is the relevant quantity for constraint formulation. The relationship is:
$$
\dot{\mathbf{Q}}_A = \frac{1}{2}
\begin{bmatrix}
-q_1 & -q_2 & -q_3 \\
q_0 & -q_3 & q_2 \\
q_3 & q_0 & -q_1 \\
-q_2 & q_1 & q_0
\end{bmatrix}
\boldsymbol{\omega}_A
= \frac{1}{2} \mathbf{G}(\mathbf{Q}_A)^\top \boldsymbol{\omega}_A
$$

where $\mathbf{G}(\mathbf{Q}) \in \mathbb{R}^{3 \times 4}$ is the quaternion-to-angular-velocity matrix.

Inverting (for unit quaternions):
$$
\boldsymbol{\omega}_A = 2 \mathbf{G}(\mathbf{Q}_A) \dot{\mathbf{Q}}_A
$$

**However**, for constraint Jacobian formulation with respect to generalized velocities $\dot{\mathbf{q}} = [\dot{\mathbf{x}}_A^\top, \dot{\mathbf{Q}}_A^\top, \dot{\mathbf{x}}_B^\top, \dot{\mathbf{Q}}_B^\top]^\top$, the standard approach in constraint-based physics engines is to use the generalized velocity including quaternion derivatives directly.

The constraint velocity is:
$$
v_{t_1} = \mathbf{J}_{t_1} \dot{\mathbf{q}}
$$

where $\mathbf{J}_{t_1} \in \mathbb{R}^{1 \times 14}$ has the structure:
$$
\mathbf{J}_{t_1} =
\begin{bmatrix}
\mathbf{t}_1^\top & (\mathbf{r}_A \times \mathbf{t}_1)^\top \mathbf{G}(\mathbf{Q}_A) & -\mathbf{t}_1^\top & -(\mathbf{r}_B \times \mathbf{t}_1)^\top \mathbf{G}(\mathbf{Q}_B)
\end{bmatrix}
$$

This is a $1 \times 14$ row vector with blocks:
- **Block 1** (columns 1-3): $\mathbf{t}_1^\top$ — body A linear velocity contribution
- **Block 2** (columns 4-7): $(\mathbf{r}_A \times \mathbf{t}_1)^\top \mathbf{G}(\mathbf{Q}_A)$ — body A angular velocity contribution (transformed via quaternion kinematics)
- **Block 3** (columns 8-10): $-\mathbf{t}_1^\top$ — body B linear velocity contribution
- **Block 4** (columns 11-14): $-(\mathbf{r}_B \times \mathbf{t}_1)^\top \mathbf{G}(\mathbf{Q}_B)$ — body B angular velocity contribution

## Simplified Jacobian (Direct Angular Velocity)

In the existing codebase (per Ticket 0032), the constraint Jacobian is formulated directly with respect to angular velocities rather than quaternion derivatives. This simplifies the Jacobian to:

$$
\mathbf{J}_{t_1} =
\begin{bmatrix}
\mathbf{t}_1^\top & (\mathbf{r}_A \times \mathbf{t}_1)^\top & -\mathbf{t}_1^\top & -(\mathbf{r}_B \times \mathbf{t}_1)^\top
\end{bmatrix}
\in \mathbb{R}^{1 \times 12}
$$

where the generalized velocity is $\dot{\mathbf{q}} = [\dot{\mathbf{x}}_A^\top, \boldsymbol{\omega}_A^\top, \dot{\mathbf{x}}_B^\top, \boldsymbol{\omega}_B^\top]^\top \in \mathbb{R}^{12}$.

This formulation is computationally simpler and avoids the quaternion kinematics matrix $\mathbf{G}(\mathbf{Q})$ in the Jacobian. The quaternion update is handled separately during numerical integration.

## Statement (Direct Angular Velocity Formulation)

$$
\mathbf{J}_{t_1} =
\begin{bmatrix}
\mathbf{t}_1^\top & (\mathbf{r}_A \times \mathbf{t}_1)^\top & -\mathbf{t}_1^\top & -(\mathbf{r}_B \times \mathbf{t}_1)^\top
\end{bmatrix}
\in \mathbb{R}^{1 \times 12}
$$

$$
\mathbf{J}_{t_2} =
\begin{bmatrix}
\mathbf{t}_2^\top & (\mathbf{r}_A \times \mathbf{t}_2)^\top & -\mathbf{t}_2^\top & -(\mathbf{r}_B \times \mathbf{t}_2)^\top
\end{bmatrix}
\in \mathbb{R}^{1 \times 12}
$$

Combined 3-row Jacobian per contact:
$$
\mathbf{J}_{\text{contact}} =
\begin{bmatrix}
\mathbf{J}_n \\
\mathbf{J}_{t_1} \\
\mathbf{J}_{t_2}
\end{bmatrix}
\in \mathbb{R}^{3 \times 12}
$$

## Dimensional Verification

- Each Jacobian row: $1 \times 12$
- Per contact: 3 rows (1 normal + 2 tangential)
- For $k$ contacts: $3k \times 12$ stacked Jacobian matrix
- For $n$ bodies: generalized velocity dimension increases to $6n$ (3 linear + 3 angular per body)

## Physical Interpretation

The friction Jacobian $\mathbf{J}_{t_i}$ computes the relative tangential velocity at the contact point when the two bodies move with generalized velocities $\dot{\mathbf{q}}$. The positive blocks correspond to body A's contribution, and the negative blocks correspond to body B's contribution (relative velocity = A velocity - B velocity).

The term $\mathbf{r} \times \mathbf{t}$ transforms angular velocity into linear velocity at the contact point offset from the center of mass.
