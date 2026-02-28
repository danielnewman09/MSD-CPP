# Mathematical Formulation: K_nt Coupling in Block PGS Solver

## Summary

This document derives the 3x3 effective mass matrix K for frictional contact constraints in a Block Projected Gauss-Seidel (PGS) solver, analyzes the structure of its off-diagonal K_nt coupling terms, and establishes a mathematical criterion for distinguishing energy-injecting coupling (oblique sliding) from physically correct coupling (axis-aligned tipping). The formulation addresses a fundamental incompatibility: the same K_nt terms that cause spurious normal impulse growth in oblique contacts also provide the correct per-contact force decomposition needed for tipping torque in axis-aligned contacts.

## Problem Statement

### Physical/Computational Context

A rigid body simulator uses a Block PGS solver to enforce frictional contact constraints. Each contact is represented as a 3-row constraint (1 normal + 2 tangent), and the solver iterates over contacts applying impulse corrections computed from the 3x3 inverse effective mass matrix K^{-1}. The solver operates in two phases: Phase A applies restitution bounce impulses (normal only), and Phase B iteratively drives contact-space velocity errors to zero subject to Coulomb cone constraints.

### Mathematical Objective

1. Derive the 3x3 K matrix from first principles and characterize its off-diagonal structure.
2. Quantify the K_nt coupling magnitude for axis-aligned vs oblique sliding geometries with multiple contact points.
3. Establish when K_nt coupling produces a net energy injection (the bug) vs a physically correct force redistribution (the feature).
4. Derive a mathematical criterion that can separate these two cases, enabling a solver modification that eliminates the bug without breaking correct physics.

## Mathematical Framework

### Definitions and Notation

| Symbol | Type | Definition | Units |
|--------|------|------------|-------|
| $\mathbf{n}$ | $\mathbb{R}^3$ | Contact normal (unit, A to B) | -- |
| $\mathbf{t}_1$ | $\mathbb{R}^3$ | First tangent direction (unit) | -- |
| $\mathbf{t}_2$ | $\mathbb{R}^3$ | Second tangent direction (unit) | -- |
| $\mathbf{r}_A^{(i)}$ | $\mathbb{R}^3$ | Lever arm from COM of body A to contact point $i$ | m |
| $\mathbf{r}_B^{(i)}$ | $\mathbb{R}^3$ | Lever arm from COM of body B to contact point $i$ | m |
| $w_A$ | $\mathbb{R}^+$ | Inverse mass of body A ($1/m_A$) | 1/kg |
| $w_B$ | $\mathbb{R}^+$ | Inverse mass of body B ($1/m_B$) | 1/kg |
| $\mathbf{I}_A^{-1}$ | $\mathbb{R}^{3\times3}$ | Inverse inertia tensor of body A | 1/(kg m^2) |
| $\mathbf{I}_B^{-1}$ | $\mathbb{R}^{3\times3}$ | Inverse inertia tensor of body B | 1/(kg m^2) |
| $K$ | $\mathbb{R}^{3\times3}$ | Effective mass matrix for a single contact | 1/kg (generalized) |
| $\lambda$ | $\mathbb{R}^3$ | Impulse vector $(\lambda_n, \lambda_{t1}, \lambda_{t2})$ | N s |
| $\mathbf{v}_\text{err}$ | $\mathbb{R}^3$ | Constraint-space velocity error | m/s |
| $\mu$ | $\mathbb{R}^+$ | Coulomb friction coefficient | -- |
| $N_c$ | $\mathbb{Z}^+$ | Number of contact points | -- |

### Coordinate Systems

Right-handed Cartesian. For a cube on a horizontal floor:
- Z-axis points upward (normal direction for a horizontal floor)
- X-axis and Y-axis are tangential to the floor
- Contact normal $\mathbf{n} = [0, 0, 1]^T$ (floor to cube, i.e., A to B where A = cube, B = floor)

Note on sign convention: In this codebase, $\mathbf{n}$ points from body A to body B. For a cube (A) resting on a floor (B) with normal pointing downward from A to B, $\mathbf{n} = [0, 0, -1]^T$. However, the contact manifold generator in the codebase may flip the normal depending on the collision detection order. For clarity of exposition, we use $\mathbf{n} = [0, 0, 1]^T$ (upward) in all examples, noting that the mathematical structure is invariant under sign flip of $\mathbf{n}$.

### Assumptions

1. **Rigid bodies**: No deformation. Contact impulses propagate instantaneously.
2. **Small timestep**: Velocity-level formulation is adequate (no position-level drift within a single solve).
3. **Symmetric inertia tensors**: $\mathbf{I}_A^{-1}$ and $\mathbf{I}_B^{-1}$ are symmetric positive semi-definite.
4. **Orthonormal contact frame**: $\{\mathbf{n}, \mathbf{t}_1, \mathbf{t}_2\}$ form an orthonormal basis.
5. **Static floor**: Body B (floor) has $w_B = 0$ and $\mathbf{I}_B^{-1} = \mathbf{0}$ (infinite mass), simplifying $K$ to contributions from body A only.
6. **Unit cube geometry**: Side length $s = 1$ m, mass $m = 1$ kg, uniform density. Inertia tensor: $\mathbf{I} = \frac{m s^2}{6} \mathbf{I}_3 = \frac{1}{6}\mathbf{I}_3$.

## Core Equations

### Section 1: Derivation of the 3x3 Effective Mass Matrix K

**Statement**: For a single contact point with contact frame $\{\mathbf{n}, \mathbf{t}_1, \mathbf{t}_2\}$, the effective mass matrix is:

$$
K = J \, M^{-1} \, J^T
$$

where $J$ is the 3x12 Jacobian and $M^{-1}$ is the 12x12 block-diagonal inverse generalized mass matrix.

**Derivation**:

The constraint Jacobian maps generalized velocities to constraint-space velocities. For a two-body contact, the generalized velocity vector is:

$$
\mathbf{q} = [\mathbf{v}_A, \boldsymbol{\omega}_A, \mathbf{v}_B, \boldsymbol{\omega}_B]^T \in \mathbb{R}^{12}
$$

The Jacobian rows correspond to the normal and two tangent constraint directions. Using the codebase sign convention:

$$
J = \begin{bmatrix}
-\mathbf{n}^T & -(\mathbf{r}_A \times \mathbf{n})^T & \mathbf{n}^T & (\mathbf{r}_B \times \mathbf{n})^T \\
\mathbf{t}_1^T & (\mathbf{r}_A \times \mathbf{t}_1)^T & -\mathbf{t}_1^T & -(\mathbf{r}_B \times \mathbf{t}_1)^T \\
\mathbf{t}_2^T & (\mathbf{r}_A \times \mathbf{t}_2)^T & -\mathbf{t}_2^T & -(\mathbf{r}_B \times \mathbf{t}_2)^T
\end{bmatrix}
$$

The inverse generalized mass matrix is:

$$
M^{-1} = \text{diag}(w_A \mathbf{I}_3, \, \mathbf{I}_A^{-1}, \, w_B \mathbf{I}_3, \, \mathbf{I}_B^{-1})
$$

Splitting the Jacobian into body A and body B blocks ($J_A \in \mathbb{R}^{3\times6}$, $J_B \in \mathbb{R}^{3\times6}$):

$$
K = J_A \, M_A^{-1} \, J_A^T + J_B \, M_B^{-1} \, J_B^T
$$

where

$$
J_A = \begin{bmatrix}
-\mathbf{n}^T & -(\mathbf{r}_A \times \mathbf{n})^T \\
\mathbf{t}_1^T & (\mathbf{r}_A \times \mathbf{t}_1)^T \\
\mathbf{t}_2^T & (\mathbf{r}_A \times \mathbf{t}_2)^T
\end{bmatrix}, \quad
M_A^{-1} = \begin{bmatrix}
w_A \mathbf{I}_3 & \mathbf{0} \\
\mathbf{0} & \mathbf{I}_A^{-1}
\end{bmatrix}
$$

Expanding:

$$
K_A = w_A (J_A^{\text{lin}})(J_A^{\text{lin}})^T + (J_A^{\text{ang}})\, \mathbf{I}_A^{-1} \, (J_A^{\text{ang}})^T
$$

The linear block contribution $J_A^{\text{lin}} (J_A^{\text{lin}})^T$:

The linear columns of $J_A$ are $[-\mathbf{n}, \mathbf{t}_1, \mathbf{t}_2]^T$ (each row's first 3 entries). Since $\{\mathbf{n}, \mathbf{t}_1, \mathbf{t}_2\}$ is orthonormal:

$$
(J_A^{\text{lin}})(J_A^{\text{lin}})^T = \begin{bmatrix}
\mathbf{n} \cdot \mathbf{n} & -\mathbf{n} \cdot \mathbf{t}_1 & -\mathbf{n} \cdot \mathbf{t}_2 \\
-\mathbf{t}_1 \cdot \mathbf{n} & \mathbf{t}_1 \cdot \mathbf{t}_1 & \mathbf{t}_1 \cdot \mathbf{t}_2 \\
-\mathbf{t}_2 \cdot \mathbf{n} & \mathbf{t}_2 \cdot \mathbf{t}_1 & \mathbf{t}_2 \cdot \mathbf{t}_2
\end{bmatrix} = \mathbf{I}_3
$$

**Key result**: The linear (translational) contribution to K is always diagonal: $w_A \mathbf{I}_3 + w_B \mathbf{I}_3$. All off-diagonal K_nt coupling comes exclusively from the angular (rotational) contribution.

The angular block contribution gives the off-diagonal terms. Define:

$$
\mathbf{a}_n = \mathbf{r}_A \times \mathbf{n}, \quad
\mathbf{a}_1 = \mathbf{r}_A \times \mathbf{t}_1, \quad
\mathbf{a}_2 = \mathbf{r}_A \times \mathbf{t}_2
$$

Then (noting the sign on the normal row):

$$
J_A^{\text{ang}} = \begin{bmatrix}
-\mathbf{a}_n^T \\
\mathbf{a}_1^T \\
\mathbf{a}_2^T
\end{bmatrix}
$$

The angular contribution to K from body A:

$$
K_A^{\text{ang}} = \begin{bmatrix}
\mathbf{a}_n^T \mathbf{I}_A^{-1} \mathbf{a}_n & -\mathbf{a}_n^T \mathbf{I}_A^{-1} \mathbf{a}_1 & -\mathbf{a}_n^T \mathbf{I}_A^{-1} \mathbf{a}_2 \\
-\mathbf{a}_1^T \mathbf{I}_A^{-1} \mathbf{a}_n & \mathbf{a}_1^T \mathbf{I}_A^{-1} \mathbf{a}_1 & \mathbf{a}_1^T \mathbf{I}_A^{-1} \mathbf{a}_2 \\
-\mathbf{a}_2^T \mathbf{I}_A^{-1} \mathbf{a}_n & \mathbf{a}_2^T \mathbf{I}_A^{-1} \mathbf{a}_1 & \mathbf{a}_2^T \mathbf{I}_A^{-1} \mathbf{a}_2
\end{bmatrix}
$$

The off-diagonal K_nt terms are:

$$
K(0,1) = K_{n,t_1} = -\mathbf{a}_n^T \mathbf{I}_A^{-1} \mathbf{a}_1 = -(\mathbf{r}_A \times \mathbf{n})^T \mathbf{I}_A^{-1} (\mathbf{r}_A \times \mathbf{t}_1)
$$

$$
K(0,2) = K_{n,t_2} = -(\mathbf{r}_A \times \mathbf{n})^T \mathbf{I}_A^{-1} (\mathbf{r}_A \times \mathbf{t}_2)
$$

The negative sign arises from the Jacobian sign convention: the normal row has $-\mathbf{n}$ for body A's linear part and $-(\mathbf{r}_A \times \mathbf{n})$ for the angular part, while tangent rows have $+\mathbf{t}_k$ and $+(\mathbf{r}_A \times \mathbf{t}_k)$.

**Physical interpretation**: $K_{n,t_1}$ quantifies how a unit impulse in the $\mathbf{t}_1$ direction changes the normal relative velocity at the contact point, through the rotational coupling mediated by the lever arm $\mathbf{r}_A$. It is non-zero whenever $\mathbf{r}_A \times \mathbf{n}$ and $\mathbf{r}_A \times \mathbf{t}_1$ are not $\mathbf{I}_A^{-1}$-orthogonal.

### Section 2: K_nt Structure for Specific Geometries

We now evaluate K_nt for a unit cube (A) on a static floor (B) with 4 corner contact points. The cube has its center of mass at the origin of the cube's body frame, so the four bottom-face corners are at:

$$
\mathbf{r}_A^{(i)} \in \left\{
\begin{bmatrix} \pm 0.5 \\ \pm 0.5 \\ -0.5 \end{bmatrix}
\right\}
$$

(lever arms from COM to the four bottom corners, with the cube's bottom face at $z = -0.5$ relative to COM).

For the unit cube: $\mathbf{I}_A^{-1} = 6 \, \mathbf{I}_3$ (inverse of $\frac{1}{6}\mathbf{I}_3$).

Let $\mathbf{n} = [0, 0, 1]^T$. We consider two tangent basis choices:

**Case A (axis-aligned sliding, $\mathbf{t}_1 = [1,0,0]^T$, $\mathbf{t}_2 = [0,1,0]^T$)**:

For corner $\mathbf{r}_A = [r_x, r_y, -0.5]^T$:

$$
\mathbf{r}_A \times \mathbf{n} = \begin{bmatrix} r_x \\ r_y \\ -0.5 \end{bmatrix} \times \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} = \begin{bmatrix} r_y \\ -r_x \\ 0 \end{bmatrix}
$$

$$
\mathbf{r}_A \times \mathbf{t}_1 = \begin{bmatrix} r_x \\ r_y \\ -0.5 \end{bmatrix} \times \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix} = \begin{bmatrix} 0 \\ -0.5 \\ -r_y \end{bmatrix}
$$

Since $\mathbf{I}_A^{-1} = 6 \mathbf{I}_3$:

$$
K_{n,t_1}^{(i)} = -(\mathbf{r}_A \times \mathbf{n})^T \cdot 6 \cdot (\mathbf{r}_A \times \mathbf{t}_1) = -6 \begin{bmatrix} r_y \\ -r_x \\ 0 \end{bmatrix}^T \begin{bmatrix} 0 \\ -0.5 \\ -r_y \end{bmatrix}
$$

$$
= -6 (0 + 0.5 r_x + 0) = -3 r_x
$$

For the four corners:

| Corner | $r_x$ | $r_y$ | $K_{n,t_1}^{(i)}$ |
|--------|--------|--------|--------------------|
| $(+0.5, +0.5, -0.5)$ | $+0.5$ | $+0.5$ | $-1.5$ |
| $(-0.5, +0.5, -0.5)$ | $-0.5$ | $+0.5$ | $+1.5$ |
| $(+0.5, -0.5, -0.5)$ | $+0.5$ | $-0.5$ | $-1.5$ |
| $(-0.5, -0.5, -0.5)$ | $-0.5$ | $-0.5$ | $+1.5$ |

**Key observation**: The K_nt values for the four corners are $\{-1.5, +1.5, -1.5, +1.5\}$. They are individually non-zero but sum to zero:

$$
\sum_{i=1}^{4} K_{n,t_1}^{(i)} = 0
$$

Similarly, by symmetry of $r_y$ values, $\sum_i K_{n,t_2}^{(i)} = 0$.

**Case B (oblique sliding, tangent basis rotated by 45 degrees)**:

For oblique sliding at 45 degrees in the XY-plane, the `setSlidingMode()` function aligns $\mathbf{t}_1$ with the sliding direction. For sliding in $[1/\sqrt{2}, 1/\sqrt{2}, 0]^T$:

$$
\mathbf{t}_1 = \frac{1}{\sqrt{2}}[-1, -1, 0]^T \quad \text{(opposing sliding direction)}
$$

$$
\mathbf{t}_2 = \mathbf{n} \times \mathbf{t}_1 = [0,0,1]^T \times \frac{1}{\sqrt{2}}[-1, -1, 0]^T = \frac{1}{\sqrt{2}}[1, -1, 0]^T
$$

For corner $\mathbf{r}_A = [r_x, r_y, -0.5]^T$:

$$
\mathbf{r}_A \times \mathbf{t}_1 = \begin{bmatrix} r_x \\ r_y \\ -0.5 \end{bmatrix} \times \frac{1}{\sqrt{2}} \begin{bmatrix} -1 \\ -1 \\ 0 \end{bmatrix} = \frac{1}{\sqrt{2}} \begin{bmatrix} -0.5 \\ 0.5 \\ -(r_x - r_y) \end{bmatrix}
$$

Wait, let me compute this correctly:

$$
\mathbf{r}_A \times \mathbf{t}_1 = \frac{1}{\sqrt{2}} \begin{bmatrix} r_y \cdot 0 - (-0.5)(-1) \\ (-0.5)(-1) - r_x \cdot 0 \\ r_x(-1) - r_y(-1) \end{bmatrix} = \frac{1}{\sqrt{2}} \begin{bmatrix} -0.5 \\ 0.5 \\ -r_x + r_y \end{bmatrix}
$$

Then:

$$
K_{n,t_1}^{(i)} = -6 \begin{bmatrix} r_y \\ -r_x \\ 0 \end{bmatrix}^T \cdot \frac{1}{\sqrt{2}} \begin{bmatrix} -0.5 \\ 0.5 \\ -r_x + r_y \end{bmatrix}
$$

$$
= \frac{-6}{\sqrt{2}} \left( -0.5 r_y - 0.5 r_x \right) = \frac{3}{\sqrt{2}} (r_x + r_y) = \frac{3(r_x + r_y)}{\sqrt{2}}
$$

For the four corners:

| Corner | $r_x$ | $r_y$ | $r_x + r_y$ | $K_{n,t_1}^{(i)}$ |
|--------|--------|--------|--------------|---------------------|
| $(+0.5, +0.5, -0.5)$ | $+0.5$ | $+0.5$ | $+1.0$ | $+3/\sqrt{2} \approx +2.12$ |
| $(-0.5, +0.5, -0.5)$ | $-0.5$ | $+0.5$ | $0.0$ | $0$ |
| $(+0.5, -0.5, -0.5)$ | $+0.5$ | $-0.5$ | $0.0$ | $0$ |
| $(-0.5, -0.5, -0.5)$ | $-0.5$ | $-0.5$ | $-1.0$ | $-3/\sqrt{2} \approx -2.12$ |

**Sum**: $K_{n,t_1}^{(1)} + K_{n,t_1}^{(2)} + K_{n,t_1}^{(3)} + K_{n,t_1}^{(4)} = 2.12 + 0 + 0 - 2.12 = 0$.

Interesting -- the K_nt values still sum to zero per contact. But the critical difference is in how these values interact with the velocity errors at each contact point.

### Section 3: The Energy Injection Mechanism

The per-contact normal impulse correction in the coupled solve is:

$$
\Delta\lambda_n^{(i)} = \left[ K^{-1,(i)} \cdot (-\mathbf{v}_\text{err}^{(i)}) \right]_0
$$

For a non-penetrating sliding contact ($v_\text{err,n}^{(i)} \approx 0$), this reduces to:

$$
\Delta\lambda_n^{(i),\text{pre}} \approx -K^{-1}_{01}(i) \, v_\text{err,t1}^{(i)} - K^{-1}_{02}(i) \, v_\text{err,t2}^{(i)}
$$

where the superscript "pre" denotes the value before Coulomb cone projection.

**Key result (linear regime)**: The pre-projection net normal impulse across all contacts is identically zero for any rigid-body angular velocity:

$$
\Delta\Lambda_n^{\text{pre}} = \sum_{i=1}^{N_c} \Delta\lambda_n^{(i),\text{pre}} = 0
$$

**Proof**: The velocity error at contact point $i$ is:

$$
v_\text{err,t1}^{(i)} = \mathbf{t}_1 \cdot \left(\mathbf{v}_\text{COM} + \boldsymbol{\omega} \times \mathbf{r}_A^{(i)}\right)
$$

Splitting into common and differential parts:

$$
\Delta\Lambda_n^{\text{pre}} = -\sum_i K^{-1}_{01}(i) \left[ v_{t1,\text{COM}} + \mathbf{t}_1 \cdot (\boldsymbol{\omega} \times \mathbf{r}_A^{(i)}) \right] - \sum_i K^{-1}_{02}(i) (\ldots)
$$

For the oblique case, contacts with non-zero K_nt1 are corners 1 ($r_x + r_y = +1$, $K_{n,t1} = +3/\sqrt{2}$) and 4 ($r_x + r_y = -1$, $K_{n,t1} = -3/\sqrt{2}$). Their position vectors satisfy $\mathbf{r}_A^{(4)} = -\mathbf{r}_A^{(1)}$ in the XY plane (same $r_z$). Therefore $\boldsymbol{\omega} \times \mathbf{r}_A^{(4)}$ and $\boldsymbol{\omega} \times \mathbf{r}_A^{(1)}$ have equal tangential components for any $\boldsymbol{\omega}$ (the XY antisymmetry of cross products under sign flip of $r_{xy}$ maps to equal tangential projections when $K_{nt}$ also changes sign). Thus $K_{nt}^{(1)} \cdot v_{err,t1}^{(1)} + K_{nt}^{(4)} \cdot v_{err,t1}^{(4)} = 0$, and contacts 2, 3 contribute zero (their $K_{nt} = 0$). The sum is zero regardless of $\boldsymbol{\omega}$.

**The actual energy injection mechanism: Nonlinear Coulomb cone projection**.

The K_nt coupling is necessary but not sufficient for energy injection. The essential nonlinearity is the Coulomb cone projection step:

$$
\lambda^{(i)} \leftarrow \text{Proj}_{\text{cone}}\left(\lambda_{\text{old}}^{(i)} + \Delta\lambda^{(i),\text{pre}}\right)
$$

where the Coulomb cone constrains $\|\lambda_t^{(i)}\| \leq \mu \lambda_n^{(i)}$.

When a contact is in sliding friction (i.e., $\|\lambda_t\|$ would exceed $\mu \lambda_n$ without clipping), the projection clips the tangential impulse:

$$
\lambda_t^{(i),\text{proj}} = \lambda_t^{(i),\text{pre}} \cdot \frac{\mu \lambda_n^{(i)}}{\|\lambda_t^{(i),\text{pre}}\|}
$$

This clipping modifies the actual realized $\Delta\lambda_n^{(i)}$ (the post-projection change relative to $\lambda_{\text{old}}^{(i)}$), and critically, **different contacts are clipped by different amounts** because:

1. Each contact has a different unconstrained $\lambda_t^{(i),\text{pre}}$ (from different velocity errors).
2. Each contact has a different $\lambda_n^{(i)}$ (from accumulated K_nt-driven impulses plus warm-start).
3. The ratio $\|\lambda_t^{(i),\text{pre}}\| / (\mu \lambda_n^{(i)})$ differs per contact.

After projection, the post-projection net normal impulse change per body is:

$$
\Delta\Lambda_n^{\text{post}} = \sum_{i=1}^{N_c} \left(\lambda_n^{(i),\text{proj}} - \lambda_{n,\text{old}}^{(i)}\right)
$$

Because the projection is nonlinear, the pre-projection identity $\Delta\Lambda_n^{\text{pre}} = 0$ no longer holds post-projection. The asymmetric clipping creates $\Delta\Lambda_n^{\text{post}} \neq 0$, which is the actual energy injection.

**The feedback loop**: A positive $\Delta\Lambda_n^{\text{post}}$ increases $\lambda_n$ at some contacts, expanding their cone radius $\mu \lambda_n$, which in turn allows larger tangential impulses in subsequent sweeps, which produce different velocity errors, which produce different pre-projection impulses, which are again asymmetrically clipped. This is a positive feedback loop that grows without bound for oblique sliding geometries.

**Why oblique but not axis-aligned?** For axis-aligned sliding with 4 symmetric corners, the K_nt pattern is $\{-1.5, +1.5, -1.5, +1.5\}$ for $K_{n,t_1}$ (Section 2). The pre-projection $\Delta\lambda_n^{(i),\text{pre}}$ values tend to cancel by this sign symmetry. Even after cone projection, the asymmetric clipping is small because the tangential velocity errors are nearly equal across contacts (translational sliding with small angular perturbation). For oblique sliding, the K_nt pattern $\{+2.121, 0, 0, -2.121\}$ concentrates the normal impulse coupling on two corners. The iterative PGS process builds up differential angular velocities at these corners, which get cone-clipped asymmetrically, creating persistent net normal growth.

### Section 4: Why Per-Contact Fixes Fail

All four prototype approaches (P2, P3, P4, P5) failed because they attempted to modify the per-contact K_nt behavior without accounting for the system-level summation property.

**Theorem (Per-Contact Indistinguishability)**: For any single contact point $i$ in isolation, the K_nt coupling term $K^{-1}_{0j}(i) \cdot v_{\text{err},j}^{(i)}$ is structurally identical regardless of whether the sliding direction is axis-aligned or oblique. The distinction between energy-injecting and energy-neutral coupling exists only at the system level, after the nonlinear Coulomb cone projection is applied across multiple contacts.

**Proof sketch**: For a single contact, the K matrix depends only on $\mathbf{r}_A^{(i)}$, $\mathbf{n}$, $\mathbf{t}_1$, $\mathbf{t}_2$, $\mathbf{I}_A^{-1}$, and $w_A$. None of these encode information about other contacts or about whether the sliding direction aligns with a geometric symmetry axis of the multi-contact system. The pre-projection K_nt contribution $\Delta\lambda_n^{(i),\text{pre}}$ at any single contact may be non-zero in both axis-aligned and oblique cases. The cancellation in the linear (pre-projection) regime comes from summing across all contacts — a system-level property. Furthermore, even after observing the post-projection $\Delta\lambda_n^{(i),\text{proj}}$ at a single contact, one cannot determine whether the net across all contacts is positive (energy-injecting) or zero (energy-neutral), because this depends on how the other contacts are cone-clipped. Therefore, no per-contact criterion based on $K^{(i)}$, $\mathbf{v}_\text{err}^{(i)}$, or their post-projection impulses can distinguish the two cases.

**Corollary**: Any correct fix must either:
1. Operate at the system level (aggregating post-projection impulse changes across contacts sharing a body), or
2. Reformulate the per-contact solve to avoid the problematic K_nt coupling entirely while preserving its beneficial effects through alternative means.

### Section 5: The Correct Normal Impulse for Oblique Sliding

For a non-penetrating contact ($v_\text{err,n} = 0$), the physically correct normal impulse correction should be:

$$
\Delta\lambda_n = 0
$$

Any non-zero $\Delta\lambda_n$ when $v_\text{err,n} = 0$ is an artifact of the K_nt coupling in the coupled solve. The correct normal impulse for the system is:

$$
\lambda_n^{\text{total}} = \lambda_n^{\text{gravity}} + \lambda_n^{\text{restitution}}
$$

where $\lambda_n^{\text{gravity}} = m g \Delta t$ (weight support) and $\lambda_n^{\text{restitution}}$ comes from Phase A. In steady-state sliding without penetration, $\Delta\lambda_n = 0$ is exact.

However, the K_nt coupling in the axis-aligned case provides a correct per-contact redistribution of normal force that sums to zero but individually provides the correct torque arm. The per-contact normal impulses are:

$$
\lambda_n^{(i)} = \frac{mg\Delta t}{4} + \delta\lambda_n^{(i)}
$$

where $\delta\lambda_n^{(i)}$ are the K_nt-driven redistributions satisfying $\sum_i \delta\lambda_n^{(i)} = 0$. These redistributions are essential for correct tipping torque because they adjust how much of the normal force passes through each corner, creating the moment arm differential that produces angular acceleration.

### Section 6: A System-Level Correction Criterion

As established in Section 3, the energy injection arises from the nonlinear Coulomb cone projection generating asymmetric post-projection $\Delta\lambda_n$ across contacts sharing a body. The correct criterion must operate on **post-projection** quantities, not pre-projection K_nt-weighted velocity errors (which are identically zero by the symmetry argument in Section 3).

**Definition**: The **net post-projection normal impulse change** over one PGS sweep for a body $b$ is:

$$
\Delta\Lambda_n^{(b)} = \sum_{i \in \mathcal{C}(b)} \left( \lambda_n^{(i),\text{after}} - \lambda_n^{(i),\text{before}} \right)
$$

where $\mathcal{C}(b)$ is the set of all contact indices involving body $b$, and "before/after" refer to the accumulated $\lambda_n$ values at the start and end of the PGS sweep, after the full coupled K_inv solve and Coulomb cone projection for each contact.

**Criterion**: The cone projection has injected energy when $\Delta\Lambda_n^{(b)} > \epsilon$ for a body with no penetrating contacts (all $v_{\text{err,n}}^{(i)} \geq 0$). In the absence of penetration, any positive net normal impulse growth across a body's contacts is spurious — it is not warranted by contact geometry and represents energy injection.

**Proposed algorithm (Post-Sweep Net-Zero Redistribution)**:

After each PGS sweep, for each body $b$ that has no penetrating contacts ($v_{\text{err,n}}^{(i)} \geq 0$ for all $i \in \mathcal{C}(b)$):

1. Compute $\Delta\Lambda_n^{(b)}$ (the post-projection net normal impulse change).
2. If $|\Delta\Lambda_n^{(b)}| > \epsilon$, subtract the mean from each contact's $\lambda_n$:

$$
\lambda_n^{(i)} \leftarrow \lambda_n^{(i)} - \frac{\Delta\Lambda_n^{(b)}}{|\mathcal{C}(b)|}
$$

3. Re-project each modified contact onto the Coulomb cone to ensure $\|\lambda_t^{(i)}\| \leq \mu \lambda_n^{(i)}$.

This redistribution:
- Zeros the net normal impulse injection (prevents spurious upward momentum).
- Preserves the per-contact differential $\lambda_n^{(i)} - \lambda_n^{(j)}$ (preserves tipping torque distribution).
- Re-projection ensures cone feasibility after the redistribution.

**Why operate on post-projection quantities?**: The pre-projection K_nt contribution $\sum_i K^{-1}_{0j}(i) v_{\text{err},j}^{(i)}$ is identically zero for any angular velocity (proven in Section 3). Using it as a criterion would yield a zero signal and provide no information about actual energy injection. The post-projection $\Delta\Lambda_n^{(b)}$ captures the actual asymmetric clipping effect that breaks the pre-projection cancellation and drives energy injection.

**Alternative algorithm (Split-Impulse for Normal Row)**:

Decompose the normal row computation into two parts:

1. **Self-term**: $\Delta\lambda_n^{\text{self}} = K^{-1}_{00} \cdot (-v_{\text{err,n}})$ — the direct normal correction from penetration velocity.
2. **K_nt-term**: $\Delta\lambda_n^{\text{K_{nt}}} = K^{-1}_{01} \cdot (-v_{\text{err,t1}}) + K^{-1}_{02} \cdot (-v_{\text{err,t2}})$ — the coupling correction from tangential velocity.

Apply self-terms directly (these correct actual penetration). Collect K_nt-terms for all contacts sharing a body, enforce mean-zero redistribution, then apply. This operates on pre-projection quantities but exploits the known cancellation structure to achieve net-zero. Note that after this redistribution, the Coulomb cone re-projection must still be applied, which can re-introduce asymmetry. This approach is less complete than the post-projection criterion above, but simpler to implement in a single-pass sweep.

### Section 7: Schur Complement Relationship

The relationship between the full 3x3 inverse and the partitioned (decoupled) inverse is governed by the Schur complement.

Partition K as:

$$
K = \begin{bmatrix}
K_{nn} & \mathbf{k}_{nt}^T \\
\mathbf{k}_{nt} & K_{tt}
\end{bmatrix}
$$

where $K_{nn} \in \mathbb{R}$, $\mathbf{k}_{nt} \in \mathbb{R}^2$, $K_{tt} \in \mathbb{R}^{2\times2}$.

The $(0,0)$ entry of $K^{-1}$ is:

$$
K^{-1}_{00} = \frac{1}{K_{nn} - \mathbf{k}_{nt}^T K_{tt}^{-1} \mathbf{k}_{nt}}
$$

Since $K_{tt}$ is positive definite, $\mathbf{k}_{nt}^T K_{tt}^{-1} \mathbf{k}_{nt} \geq 0$, so:

$$
K^{-1}_{00} \geq \frac{1}{K_{nn}}
$$

with equality if and only if $\mathbf{k}_{nt} = \mathbf{0}$.

**This is why P4 (asymmetric decoupling) failed**: Using $1/K_{nn}$ instead of $K^{-1}_{00}$ gives a LARGER normal impulse per unit $v_\text{err,n}$. This changes the Coulomb cone bound $\mu \cdot \lambda_n$ and disrupts the angular impulse balance across 4 contacts.

**Quantitative example**: For corner $(+0.5, +0.5, -0.5)$ of the unit cube with $\mathbf{n} = [0,0,1]^T$, $\mathbf{t}_1 = [1,0,0]^T$, $\mathbf{t}_2 = [0,1,0]^T$:

$$
K_{nn} = w_A + 6(r_y^2 + r_x^2) = 1 + 6(0.25 + 0.25) = 4.0
$$

$$
\mathbf{k}_{nt} = \begin{bmatrix} K(1,0) \\ K(2,0) \end{bmatrix} = \begin{bmatrix} -3 r_x \\ -3 r_y \end{bmatrix} = \begin{bmatrix} -1.5 \\ -1.5 \end{bmatrix}
$$

$$
K_{tt} = \begin{bmatrix}
w_A + 6(0.25 + r_y^2) & 6 \cdot r_x \cdot r_y \\
6 \cdot r_x \cdot r_y & w_A + 6(0.25 + r_x^2)
\end{bmatrix} = \begin{bmatrix}
3.5 & 1.5 \\
1.5 & 3.5
\end{bmatrix}
$$

Wait, let me recompute $K_{tt}$ more carefully.

$K_{tt}(0,0) = w_A (\mathbf{t}_1 \cdot \mathbf{t}_1) + (\mathbf{r}_A \times \mathbf{t}_1)^T \mathbf{I}_A^{-1} (\mathbf{r}_A \times \mathbf{t}_1)$

$\mathbf{r}_A \times \mathbf{t}_1 = [r_x, r_y, -0.5]^T \times [1,0,0]^T = [0, -0.5, -r_y]^T$

$(\mathbf{r}_A \times \mathbf{t}_1)^T \cdot 6 \cdot (\mathbf{r}_A \times \mathbf{t}_1) = 6(0.25 + r_y^2)$

So $K_{tt}(0,0) = 1 + 6(0.25 + 0.25) = 4.0$

$K_{tt}(1,1)$ similarly $= 1 + 6(0.25 + r_x^2) = 4.0$

$K_{tt}(0,1) = -w_A(\mathbf{t}_1 \cdot \mathbf{t}_2) + (\mathbf{r}_A \times \mathbf{t}_1)^T \cdot 6 \cdot (\mathbf{r}_A \times \mathbf{t}_2)$

Wait, the sign. The tangent rows of $J_A$ have $+\mathbf{t}_k$ for linear and $+(\mathbf{r}_A \times \mathbf{t}_k)$ for angular, so:

$K_{tt}(0,1) = w_A (\mathbf{t}_1 \cdot \mathbf{t}_2) + (\mathbf{r}_A \times \mathbf{t}_1)^T \mathbf{I}_A^{-1} (\mathbf{r}_A \times \mathbf{t}_2)$

$\mathbf{t}_1 \cdot \mathbf{t}_2 = 0$

$\mathbf{r}_A \times \mathbf{t}_2 = [r_x, r_y, -0.5]^T \times [0,1,0]^T = [0.5, 0, r_x]^T$

$(\mathbf{r}_A \times \mathbf{t}_1)^T \cdot 6 \cdot (\mathbf{r}_A \times \mathbf{t}_2) = 6 [0, -0.5, -r_y] \cdot [0.5, 0, r_x] = 6(0 + 0 - r_x r_y) = -6 r_x r_y$

For corner $(+0.5, +0.5, -0.5)$: $K_{tt}(0,1) = -6(0.25) = -1.5$

So:

$$
K = \begin{bmatrix} 4.0 & -1.5 & -1.5 \\ -1.5 & 4.0 & -1.5 \\ -1.5 & -1.5 & 4.0 \end{bmatrix} + \epsilon_{\text{CFM}} \mathbf{I}_3
$$

Ignoring CFM ($\epsilon = 10^{-8}$):

Schur complement: $S = K_{nn} - \mathbf{k}_{nt}^T K_{tt}^{-1} \mathbf{k}_{nt}$

$K_{tt} = \begin{bmatrix} 4.0 & -1.5 \\ -1.5 & 4.0 \end{bmatrix}$

$K_{tt}^{-1} = \frac{1}{4^2 - 1.5^2} \begin{bmatrix} 4.0 & 1.5 \\ 1.5 & 4.0 \end{bmatrix} = \frac{1}{13.75} \begin{bmatrix} 4.0 & 1.5 \\ 1.5 & 4.0 \end{bmatrix}$

$\mathbf{k}_{nt}^T K_{tt}^{-1} \mathbf{k}_{nt} = [-1.5, -1.5] \cdot \frac{1}{13.75} \begin{bmatrix} 4.0 & 1.5 \\ 1.5 & 4.0 \end{bmatrix} \begin{bmatrix} -1.5 \\ -1.5 \end{bmatrix}$

$= \frac{1}{13.75} [-1.5, -1.5] \begin{bmatrix} -6 - 2.25 \\ -2.25 - 6 \end{bmatrix} = \frac{1}{13.75} [-1.5, -1.5] \begin{bmatrix} -8.25 \\ -8.25 \end{bmatrix}$

$= \frac{1}{13.75} (12.375 + 12.375) = \frac{24.75}{13.75} = 1.8$

$S = 4.0 - 1.8 = 2.2$

$K^{-1}_{00} = 1/S = 1/2.2 \approx 0.4545$

$1/K_{nn} = 1/4.0 = 0.25$

**Ratio**: $K^{-1}_{00} / (1/K_{nn}) = 0.4545 / 0.25 = 1.818$

The coupled solve gives an 82% larger effective normal compliance than the decoupled solve. This is why P4's scalar normal solve produced different cone bounds and broke tipping torque.

## Special Cases and Edge Conditions

### Degenerate Case 1: Coplanar Lever Arm

**Condition**: When $\mathbf{r}_A$ lies entirely in the contact plane ($\mathbf{r}_A \cdot \mathbf{n} = 0$), i.e., the contact point is at the same height as the COM.

**Mathematical Treatment**: The cross products simplify but K_nt remains non-zero in general. For example, if $\mathbf{r}_A = [r_x, r_y, 0]$ with $\mathbf{n} = [0,0,1]$:

$\mathbf{r}_A \times \mathbf{n} = [r_y, -r_x, 0]^T$, $\mathbf{r}_A \times \mathbf{t}_1 = [0, 0, -r_y]^T$ (for $\mathbf{t}_1 = [1,0,0]$)

$K_{n,t_1} = -6 [r_y, -r_x, 0] \cdot [0, 0, -r_y] = 0$

So K_nt = 0 when the lever arm has no normal component. This makes physical sense: if the contact point is at COM height, a tangential impulse produces no torque about the normal direction.

### Degenerate Case 2: Single Contact Point

**Condition**: $N_c = 1$. Only one contact point (e.g., sphere-on-floor).

**Risk**: With a single contact, there is no cancellation mechanism. The K_nt term at that contact directly drives $\Delta\lambda_n$.

**Mitigation**: For a sphere, the lever arm points from COM radially to the contact point, which is parallel to $\mathbf{n}$. Therefore $\mathbf{r}_A \times \mathbf{n} = \mathbf{0}$, and K_nt = 0 identically. More generally, K_nt = 0 whenever the lever arm is parallel to the normal.

### Degenerate Case 3: Singular K Matrix

**Condition**: K has a zero or near-zero eigenvalue (extreme mass ratio or degenerate geometry).

**Risk**: $K^{-1}$ has very large entries, amplifying the K_nt coupling effect.

**Mitigation**: CFM regularization ($K \leftarrow K + \epsilon \mathbf{I}$) with $\epsilon = 10^{-8}$ prevents true singularity. The LDLT decomposition fallback to zero on failure provides a safe default.

## Numerical Considerations

### Numerical Stability

#### Condition Number Analysis

The condition number of K for the unit cube corner contact computed above:

$K = \begin{bmatrix} 4.0 & -1.5 & -1.5 \\ -1.5 & 4.0 & -1.5 \\ -1.5 & -1.5 & 4.0 \end{bmatrix}$

Eigenvalues: $\lambda_1 = 4.0 + 1.5 + 1.5 = 7.0$ (eigenvector $[-1, -1, -1]/\sqrt{3}$ -- but let me compute properly)

For a matrix of the form $a\mathbf{I} + b\mathbf{1}\mathbf{1}^T$ where $\mathbf{1} = [-1, -1, -1]^T/\sqrt{3}$... Actually, this matrix has the form $4\mathbf{I} - 1.5(\mathbf{J} - \mathbf{I}) = 5.5\mathbf{I} - 1.5\mathbf{J}$ where $\mathbf{J}$ is all-ones. So eigenvalues are $5.5 - 1.5 \cdot 3 = 1.0$ (multiplicity 1, eigenvector $[1,1,1]^T$) and $5.5 - 1.5 \cdot 0 = 5.5$ (multiplicity 2, eigenvectors orthogonal to $[1,1,1]^T$).

$\kappa(K) = 5.5 / 1.0 = 5.5$

This is well-conditioned. LDLT decomposition is numerically safe.

#### Potential Instabilities

| Operation | Risk | Mitigation |
|-----------|------|------------|
| K^{-1} computation via LDLT | Ill-conditioned K at extreme mass ratios | CFM regularization ($10^{-8}$) |
| Division by $K_{nn}$ in Phase A | $K_{nn} \to 0$ for degenerate contacts | Guard: skip if $K_{nn} < 10^{-12}$ |
| Coulomb cone projection (tangent scaling) | Division by $\|\lambda_t\|$ when $\|\lambda_t\| \to 0$ | Only scale when $\|\lambda_t\| > \mu \lambda_n$ (never divides by zero because scaling only happens when norm exceeds threshold) |
| Net K_nt summation across contacts | Catastrophic cancellation when terms nearly cancel | Use Kahan summation or compute in double precision (already double) |

### Precision Requirements

| Computation | Recommended Precision | Rationale |
|-------------|----------------------|-----------|
| K matrix construction | double (64-bit) | Cross products of lever arms involve subtraction of similar-magnitude terms |
| K^{-1} via LDLT | double (64-bit) | Schur complement subtraction requires full precision |
| Velocity error computation | double (64-bit) | Contact point velocities involve $\omega \times r$ where magnitudes can differ by 10x |
| Net K_nt summation | double (64-bit) | Terms sum to near-zero; precision is essential |

### Tolerances

| Comparison | Tolerance | Rationale |
|------------|-----------|-----------|
| $K_{nn} > 0$ guard | $10^{-12}$ | Below this, the contact is effectively degenerate |
| $v_\text{err,n} \geq 0$ (non-penetrating) | 0.0 (exact) | Any negative $v_\text{err,n}$ indicates approach |
| Net K_nt injection threshold | $10^{-6}$ N$\cdot$s | Below measurement noise for the solver convergence tolerance |
| Energy injection per frame | 0.05 J | Test threshold from FrictionSlidingTest; allows tipping transient |

## Validation Examples

### Example 1: Nominal Case -- Axis-Aligned Sliding K_nt Cancellation

**Scenario**: Unit cube sliding in +X on static floor. Four corner contacts. Verify that K_nt contributions sum to zero across all contacts.

**Inputs**:
```
Body A: unit cube, mass = 1.0 kg, I_A^{-1} = 6 * I_3
Body B: static floor, w_B = 0, I_B^{-1} = 0
n = [0, 0, 1]
t1 = [1, 0, 0]
t2 = [0, 1, 0]

Contact 1: r_A = [+0.5, +0.5, -0.5]
Contact 2: r_A = [-0.5, +0.5, -0.5]
Contact 3: r_A = [+0.5, -0.5, -0.5]
Contact 4: r_A = [-0.5, -0.5, -0.5]

Sliding velocity: v_COM = [2.0, 0.0, 0.0] m/s
Angular velocity: omega = [0, 0, 0] rad/s (initially)
```

**Hand Computation**:

Step 1: K matrix for contact 1 ($r_A = [0.5, 0.5, -0.5]$):

$r_A \times n = [0.5, -0.5, 0]$
$r_A \times t_1 = [0, -0.5, -0.5]$
$r_A \times t_2 = [0.5, 0, 0.5]$

$K_{nn} = 1 + 6(0.5^2 + 0.5^2) = 1 + 3 = 4.0$
$K_{n,t1} = -6 \cdot [0.5, -0.5, 0] \cdot [0, -0.5, -0.5] = -6(0 + 0.25 + 0) = -1.5$
$K_{n,t2} = -6 \cdot [0.5, -0.5, 0] \cdot [0.5, 0, 0.5] = -6(0.25 + 0 + 0) = -1.5$

Step 2: K_{n,t1} for all four contacts:

| Contact | $r_x$ | $K_{n,t1} = -3r_x$ |
|---------|--------|---------------------|
| 1 | +0.5 | -1.5 |
| 2 | -0.5 | +1.5 |
| 3 | +0.5 | -1.5 |
| 4 | -0.5 | +1.5 |

Sum = $-1.5 + 1.5 - 1.5 + 1.5 = 0.0$

Step 3: Velocity errors (with zero angular velocity, all contacts have the same vErr):

$v_\text{err,n} = n \cdot (v_B - v_A) = [0,0,1] \cdot ([0,0,0] - [2,0,0]) = 0$
$v_\text{err,t1} = t_1 \cdot (v_A - v_B) = [1,0,0] \cdot [2,0,0] = 2.0$
$v_\text{err,t2} = t_2 \cdot (v_A - v_B) = [0,1,0] \cdot [2,0,0] = 0.0$

Step 4: Per-contact K_nt normal impulse correction (from $K^{-1}$, but proportional analysis):

Since all contacts have the same $v_\text{err}$ and $K_{n,t1}$ sums to zero, the net normal impulse from K_nt coupling is zero. (Full K^{-1} computation confirms this by symmetry.)

**Expected Output**:
```
sum_K_nt1 = 0.0
sum_K_nt2 = 0.0
net_delta_Lambda_n (from K_nt coupling) = 0.0
```

**GTest Template**:
```cpp
TEST(KntCoupling, AxisAligned_KntSumsToZero) {
  // Build K matrices for all 4 corner contacts of unit cube on floor
  // n = [0,0,1], t1 = [1,0,0], t2 = [0,1,0]
  // r_A corners: [+/-0.5, +/-0.5, -0.5]

  const Eigen::Vector3d n{0, 0, 1};
  const Eigen::Vector3d t1{1, 0, 0};
  const Eigen::Vector3d t2{0, 1, 0};
  const Eigen::Matrix3d I_inv = 6.0 * Eigen::Matrix3d::Identity();
  constexpr double wA = 1.0;

  double sum_Knt1 = 0.0;
  double sum_Knt2 = 0.0;

  for (double rx : {-0.5, 0.5}) {
    for (double ry : {-0.5, 0.5}) {
      const Eigen::Vector3d rA{rx, ry, -0.5};
      const Eigen::Vector3d an = rA.cross(n);
      const Eigen::Vector3d a1 = rA.cross(t1);
      const Eigen::Vector3d a2 = rA.cross(t2);

      const double Knt1 = -(an.transpose() * I_inv * a1);
      const double Knt2 = -(an.transpose() * I_inv * a2);

      sum_Knt1 += Knt1;
      sum_Knt2 += Knt2;
    }
  }

  constexpr double kTolerance = 1e-12;
  EXPECT_NEAR(sum_Knt1, 0.0, kTolerance);
  EXPECT_NEAR(sum_Knt2, 0.0, kTolerance);
}
```

### Example 2: Edge Case -- Oblique Sliding Cone-Projection Asymmetry

**Scenario**: Unit cube sliding at 45 degrees. Demonstrate that the pre-projection net normal impulse from K_nt coupling is zero, but after Coulomb cone projection the net becomes non-zero due to asymmetric clipping.

**Inputs**:
```
Body A: unit cube, mass = 1.0 kg, I_A^{-1} = 6 * I_3
Body B: static floor, w_B = 0, I_B^{-1} = 0
n = [0, 0, 1]
t1 = [-1/sqrt(2), -1/sqrt(2), 0]  (opposing 45-degree sliding direction)
t2 = [1/sqrt(2), -1/sqrt(2), 0]   (n x t1, corrected sign)
mu = 0.5  (friction coefficient)

Contact 1: r_A = [+0.5, +0.5, -0.5],  lambda_old = [1.0, -0.2, -0.1]
Contact 2: r_A = [-0.5, +0.5, -0.5],  lambda_old = [1.0, -0.1, -0.1]
Contact 3: r_A = [+0.5, -0.5, -0.5],  lambda_old = [1.0, -0.1, -0.1]
Contact 4: r_A = [-0.5, -0.5, -0.5],  lambda_old = [1.0, -0.2, -0.1]

(lambda_old represents current accumulated impulses: lambda_n = 1.0 Ns for weight support,
with small tangential components from prior friction.)

Sliding velocity: v_COM = [2/sqrt(2), 2/sqrt(2), 0.0] m/s
Angular velocity: omega = [0, 0, 0] rad/s (simplified for hand computation)
```

**Hand Computation**:

Step 1: Pre-projection unconstrained delta-lambda from K_nt coupling.

For each contact, the unconstrained normal correction is:
$\Delta\lambda_n^{(i),\text{pre}} = K^{-1}_{01}(i) \cdot (-v_{\text{err,t1}}^{(i)}) + K^{-1}_{02}(i) \cdot (-v_{\text{err,t2}}^{(i)})$

With $\omega = 0$, all contacts have the same $v_\text{err}$:

$v_{\text{err,t1}} = \mathbf{t}_1 \cdot v_\text{COM} = (-1/\sqrt{2})(2/\sqrt{2}) + (-1/\sqrt{2})(2/\sqrt{2}) = -1 - 1 = -2.0$ m/s

So $(-v_{\text{err,t1}}) = +2.0$ m/s for all contacts.

Since $K^{-1}_{01}$ sums to zero (by symmetry of K_nt values from Section 2):

$\Delta\Lambda_n^{\text{pre}} = \sum_i K^{-1}_{01}(i) \cdot 2.0 = 2.0 \sum_i K^{-1}_{01}(i) = 0$

**Pre-projection net = 0. Confirmed.**

Step 2: Compute accumulated lambda_temp before cone projection.

$K^{-1}_{01}$ values are proportional to $K_{n,t1}$ values (same sign pattern): contacts 1 and 4 get a positive $\Delta\lambda_n^{\text{pre}} \sim +\epsilon$ while contacts 2 and 3 get $0$ (since K_nt1 = 0). However, numerically the contact-1 and contact-4 contributions cancel exactly, so $\Delta\lambda_n^{(i),\text{pre}} \approx 0$ for all.

For this example, assume representative pre-projection accumulated impulses:

```
Contact 1: lambda_temp = lambda_old + delta_pre = [1.0, -0.35, -0.18]  (|lambda_t| = 0.391)
Contact 2: lambda_temp = [1.0, -0.20, -0.10]                           (|lambda_t| = 0.224)
Contact 3: lambda_temp = [1.0, -0.20, -0.10]                           (|lambda_t| = 0.224)
Contact 4: lambda_temp = [1.0, -0.35, -0.18]                           (|lambda_t| = 0.391)
```

(Contacts 1 and 4 have larger tangential impulses because their K_nt drives a larger tangential correction than contacts 2 and 3.)

Step 3: Apply Coulomb cone projection with $\mu = 0.5$.

Cone radius = $\mu \lambda_n = 0.5 \times 1.0 = 0.5$ Ns for all contacts.

| Contact | $\|\lambda_t^{\text{pre}}\|$ | Inside cone? | Clipping factor | $\lambda_n^{\text{proj}}$ |
|---------|-----------------------------|--------------|-----------------|--------------------------|
| 1 | 0.391 | Yes (0.391 < 0.5) | 1.0 | 1.0 |
| 2 | 0.224 | Yes (0.224 < 0.5) | 1.0 | 1.0 |
| 3 | 0.224 | Yes (0.224 < 0.5) | 1.0 | 1.0 |
| 4 | 0.391 | Yes (0.391 < 0.5) | 1.0 | 1.0 |

In this symmetric case all contacts are inside the cone, so no clipping occurs and $\Delta\Lambda_n^{\text{post}} = 0$.

Step 4: Asymmetric case — contacts at different cone radii.

Now suppose contact 1 has accumulated extra normal impulse from prior iterations: $\lambda_n^{(1)} = 1.2$, $\lambda_n^{(4)} = 0.8$ (sum = 2.0, same total weight support). Cone radii: $\mu\lambda_n^{(1)} = 0.6$, $\mu\lambda_n^{(4)} = 0.4$.

Pre-projection tangential magnitudes (same as before): contacts 1 and 4 both have $\|\lambda_t^{\text{pre}}\| = 0.391$.

| Contact | $\lambda_n^{(i)}$ | $\mu\lambda_n$ | $\|\lambda_t^{\text{pre}}\|$ | Clipped? | $\|\lambda_t^{\text{proj}}\|$ | $\lambda_n^{\text{proj}}$ |
|---------|--------------------|----------------|------------------------------|----------|-------------------------------|--------------------------|
| 1 | 1.2 | 0.6 | 0.391 | No | 0.391 | 1.2 |
| 4 | 0.8 | 0.4 | 0.391 | **Yes** | 0.4 | 0.8 |

Contact 4 is clipped but contact 1 is not. After projection, both $\lambda_n$ values remain unchanged (cone projection only clips $\lambda_t$, $\lambda_n$ is not modified by the projection itself). However, in the next PGS sweep, the velocity error at contact 4 will differ from contact 1 because its tangential impulse was clipped. This asymmetric clipping produces different velocity residuals, which in subsequent iterations lead to different $\Delta\lambda_n$ corrections — and because the K_nt values at contacts 1 and 4 have opposite signs (+2.121 vs -2.121), the asymmetric velocity residuals generate a non-zero net $\Delta\Lambda_n$ in the next sweep.

**Mechanism summary**:
- Pre-projection: $\sum_i K_{nt}^{(i)} v_{err}^{(i)} = 0$ (exact, proven above)
- After clipping: contact 4 has reduced $\|\lambda_t\|$, so its velocity error in the next sweep is different from contact 1's
- Next sweep: contacts 1 and 4 see different $v_\text{err}$ → their K_nt contributions no longer cancel
- Net $\Delta\Lambda_n^{\text{post}} \neq 0$: positive energy injection begins

This is the actual energy injection mechanism. It requires at least two PGS sweeps (one to generate the asymmetric state via clipping, one to convert it to net normal impulse), explaining why the feedback is iterative.

**Expected Output**:
```
sum_K_nt1_oblique_pre = 0.0  (K_nt values sum to zero: proven analytically)
After asymmetric cone clipping: net delta_Lambda_n != 0 in subsequent sweeps
```

**GTest Template**:
```cpp
TEST(KntCoupling, Oblique_PreProjectionNetIsZero) {
  // Pre-projection: K_nt-weighted velocity errors sum to zero for any angular velocity
  const Eigen::Vector3d n{0, 0, 1};
  const Eigen::Vector3d t1 = Eigen::Vector3d{-1, -1, 0}.normalized();
  const Eigen::Vector3d t2 = n.cross(t1);  // [1/sqrt(2), -1/sqrt(2), 0]
  const Eigen::Matrix3d I_inv = 6.0 * Eigen::Matrix3d::Identity();
  constexpr double wA = 1.0;

  // Build K matrices for all 4 contacts and compute K^{-1}_{01}
  // Use a representative angular velocity to verify sum = 0
  const Eigen::Vector3d omega{0.1, 0.0, 0.0};
  const Eigen::Vector3d vCOM = Eigen::Vector3d{1, 1, 0} / std::sqrt(2.0) * 2.0;

  double sumNetNormal = 0.0;

  for (double rx : {-0.5, 0.5}) {
    for (double ry : {-0.5, 0.5}) {
      const Eigen::Vector3d rA{rx, ry, -0.5};
      const Eigen::Vector3d an = rA.cross(n);
      const Eigen::Vector3d a1 = rA.cross(t1);
      const Eigen::Vector3d a2 = rA.cross(t2);

      // Build 3x3 K
      Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
      K(0,0) = wA + an.dot(I_inv * an);
      K(0,1) = -(an.dot(I_inv * a1));   K(1,0) = K(0,1);
      K(0,2) = -(an.dot(I_inv * a2));   K(2,0) = K(0,2);
      K(1,1) = wA + a1.dot(I_inv * a1);
      K(1,2) = a1.dot(I_inv * a2);      K(2,1) = K(1,2);
      K(2,2) = wA + a2.dot(I_inv * a2);

      const Eigen::Matrix3d Kinv = K.inverse();

      // Velocity error at this contact
      const Eigen::Vector3d vContact = vCOM + omega.cross(rA);
      Eigen::Vector3d vErr;
      vErr(0) = -n.dot(vContact);
      vErr(1) = t1.dot(vContact);
      vErr(2) = t2.dot(vContact);

      // Pre-projection normal correction
      const double deltaN = (Kinv.row(0) * (-vErr))(0);
      sumNetNormal += deltaN;
    }
  }

  // Pre-projection net must be zero
  constexpr double kTolerance = 1e-10;
  EXPECT_NEAR(sumNetNormal, 0.0, kTolerance)
    << "Pre-projection K_nt net is identically zero; energy injection "
       "requires nonlinear cone projection to break cancellation";
}
```

### Example 3: Degenerate Case -- Sphere Contact (K_nt = 0)

**Scenario**: Sphere with lever arm parallel to normal. K_nt should be exactly zero.

**Why This Is Degenerate**: When $\mathbf{r}_A \parallel \mathbf{n}$, the cross product $\mathbf{r}_A \times \mathbf{n} = \mathbf{0}$, making K_nt identically zero regardless of tangent basis.

**Inputs**:
```
Body A: sphere, radius 0.5 m, mass = 1.0 kg
  I_A = (2/5) m r^2 I_3 = 0.1 I_3, I_A^{-1} = 10 I_3
Body B: static floor
n = [0, 0, 1]
t1 = [1, 0, 0], t2 = [0, 1, 0]
r_A = [0, 0, -0.5]  (bottom of sphere, directly below COM)
```

**Hand Computation**:

$\mathbf{r}_A \times \mathbf{n} = [0, 0, -0.5] \times [0, 0, 1] = [0 \cdot 1 - (-0.5) \cdot 0, \; (-0.5) \cdot 0 - 0 \cdot 1, \; 0 \cdot 0 - 0 \cdot 0] = [0, 0, 0]$

Since $\mathbf{r}_A \times \mathbf{n} = \mathbf{0}$:

$K_{n,t1} = -(\mathbf{r}_A \times \mathbf{n})^T \mathbf{I}_A^{-1} (\mathbf{r}_A \times \mathbf{t}_1) = 0$
$K_{n,t2} = 0$

The K matrix is:

$K_{nn} = w_A + 0 = 1.0$

$\mathbf{r}_A \times \mathbf{t}_1 = [0, 0, -0.5] \times [1, 0, 0] = [0, -0.5, 0]$
$K_{t1,t1} = 1.0 + 10 \cdot 0.25 = 3.5$

$\mathbf{r}_A \times \mathbf{t}_2 = [0, 0, -0.5] \times [0, 1, 0] = [0.5, 0, 0]$
$K_{t2,t2} = 1.0 + 10 \cdot 0.25 = 3.5$

$K_{t1,t2} = 10 \cdot [0, -0.5, 0] \cdot [0.5, 0, 0] = 0$

$$
K = \begin{bmatrix} 1.0 & 0 & 0 \\ 0 & 3.5 & 0 \\ 0 & 0 & 3.5 \end{bmatrix}
$$

K is diagonal. $K^{-1}$ is also diagonal. The coupled and decoupled solves are identical.

**Expected Output**:
```
K(0,1) = 0.0
K(0,2) = 0.0
K^{-1}(0,1) = 0.0
K^{-1}(0,2) = 0.0
```

**Expected Behavior**: No K_nt coupling exists for sphere contacts. The coupled and decoupled solves produce identical results. No energy injection is possible through K_nt terms.

**GTest Template**:
```cpp
TEST(KntCoupling, SphereContact_KntIsZero) {
  // Sphere with r_A parallel to n: K_nt must be zero
  const Eigen::Vector3d n{0, 0, 1};
  const Eigen::Vector3d t1{1, 0, 0};
  const Eigen::Vector3d t2{0, 1, 0};
  const Eigen::Matrix3d I_inv = 10.0 * Eigen::Matrix3d::Identity();
  constexpr double wA = 1.0;

  const Eigen::Vector3d rA{0, 0, -0.5};  // directly below COM

  const Eigen::Vector3d an = rA.cross(n);   // [0, 0, 0]
  const Eigen::Vector3d a1 = rA.cross(t1);
  const Eigen::Vector3d a2 = rA.cross(t2);

  // K_nt terms
  const double Knt1 = -(an.transpose() * I_inv * a1);
  const double Knt2 = -(an.transpose() * I_inv * a2);

  constexpr double kTolerance = 1e-15;
  EXPECT_NEAR(Knt1, 0.0, kTolerance);
  EXPECT_NEAR(Knt2, 0.0, kTolerance);

  // Build full K matrix
  Eigen::Matrix3d K;
  K(0,0) = wA + an.transpose() * I_inv * an;
  K(0,1) = Knt1;  K(1,0) = Knt1;
  K(0,2) = Knt2;  K(2,0) = Knt2;
  K(1,1) = wA + a1.transpose() * I_inv * a1;
  K(1,2) = a1.transpose() * I_inv * a2;
  K(2,1) = K(1,2);
  K(2,2) = wA + a2.transpose() * I_inv * a2;

  // K should be diagonal
  EXPECT_NEAR(K(0,0), 1.0, kTolerance);
  EXPECT_NEAR(K(1,1), 3.5, kTolerance);
  EXPECT_NEAR(K(2,2), 3.5, kTolerance);
  EXPECT_NEAR(K(0,1), 0.0, kTolerance);
  EXPECT_NEAR(K(0,2), 0.0, kTolerance);
  EXPECT_NEAR(K(1,2), 0.0, kTolerance);
}
```

### Example 4: Numerical Verification -- Schur Complement Ratio

**Scenario**: Verify the Schur complement relationship $K^{-1}_{00} = 1/(K_{nn} - \mathbf{k}_{nt}^T K_{tt}^{-1} \mathbf{k}_{nt})$ for the unit cube corner contact, and confirm the 1.818 ratio between $K^{-1}_{00}$ and $1/K_{nn}$.

**Inputs**:
```
K = [[4.0, -1.5, -1.5],
     [-1.5, 4.0, -1.5],
     [-1.5, -1.5, 4.0]]
```

(Unit cube corner $(+0.5, +0.5, -0.5)$ with $\mathbf{n} = [0,0,1]$, $\mathbf{t}_1 = [1,0,0]$, $\mathbf{t}_2 = [0,1,0]$, ignoring CFM.)

**Hand Computation**:

$K_{nn} = 4.0$, $\mathbf{k}_{nt} = [-1.5, -1.5]^T$, $K_{tt} = \begin{bmatrix} 4.0 & -1.5 \\ -1.5 & 4.0 \end{bmatrix}$

$\det(K_{tt}) = 16 - 2.25 = 13.75$

$K_{tt}^{-1} = \frac{1}{13.75} \begin{bmatrix} 4.0 & 1.5 \\ 1.5 & 4.0 \end{bmatrix}$

$K_{tt}^{-1} \mathbf{k}_{nt} = \frac{1}{13.75} \begin{bmatrix} 4.0 & 1.5 \\ 1.5 & 4.0 \end{bmatrix} \begin{bmatrix} -1.5 \\ -1.5 \end{bmatrix} = \frac{1}{13.75} \begin{bmatrix} -8.25 \\ -8.25 \end{bmatrix}$

$\mathbf{k}_{nt}^T K_{tt}^{-1} \mathbf{k}_{nt} = [-1.5, -1.5] \cdot \frac{1}{13.75} [-8.25, -8.25]^T = \frac{24.75}{13.75} = 1.8$

$S = K_{nn} - 1.8 = 4.0 - 1.8 = 2.2$

$K^{-1}_{00} = 1/S = 1/2.2 = 0.45\overline{45}$

$1/K_{nn} = 1/4.0 = 0.25$

$\text{ratio} = K^{-1}_{00} \cdot K_{nn} = 0.4545 \times 4.0 = 1.818$

Verification via direct K^{-1}:

$\det(K) = 4(16 - 2.25) - (-1.5)(-6 - 2.25) + (-1.5)(2.25 + 6)$
$= 4(13.75) + 1.5(-8.25) - 1.5(8.25)$
$= 55 - 12.375 - 12.375 = 30.25$

$K^{-1}_{00} = \text{cofactor}(0,0) / \det(K) = (16 - 2.25) / 30.25 = 13.75 / 30.25 = 0.4545$

Confirmed.

**Expected Output**:
```
K_inv_00 = 0.4545 (4 s.f.)
1_over_Knn = 0.25
ratio = 1.818
Schur_complement = 1.8
```

**GTest Template**:
```cpp
TEST(KntCoupling, SchurComplementRatio) {
  Eigen::Matrix3d K;
  K << 4.0, -1.5, -1.5,
       -1.5, 4.0, -1.5,
       -1.5, -1.5, 4.0;

  const Eigen::Matrix3d K_inv = K.inverse();

  constexpr double kTolerance = 1e-10;

  // K^{-1}(0,0) from Schur complement
  const double K_nn = K(0, 0);
  const Eigen::Vector2d k_nt = K.block<2, 1>(1, 0);
  const Eigen::Matrix2d K_tt = K.block<2, 2>(1, 1);
  const double schur = k_nt.transpose() * K_tt.inverse() * k_nt;
  const double K_inv_00_schur = 1.0 / (K_nn - schur);

  EXPECT_NEAR(K_inv(0, 0), K_inv_00_schur, kTolerance);
  EXPECT_NEAR(K_inv(0, 0), 0.45454545, 1e-6);
  EXPECT_NEAR(1.0 / K_nn, 0.25, kTolerance);

  // The ratio shows why 1/K_nn != K_inv(0,0)
  const double ratio = K_inv(0, 0) * K_nn;
  EXPECT_NEAR(ratio, 1.818, 0.001)
    << "Coupled solve gives 82% larger effective normal compliance than "
       "decoupled scalar";

  // Schur complement term
  EXPECT_NEAR(schur, 1.8, kTolerance);
}
```

## References

- Erleben, K. (2007). "Velocity-based shock propagation for multibody dynamics animation." ACM Transactions on Graphics.
- Catto, E. (2005). "Iterative Dynamics with Temporal Coherence." Game Developer Conference. (Block PGS formulation.)
- Baraff, D. (1994). "Fast Contact Force Computation for Nonpenetrating Rigid Bodies." SIGGRAPH. (LCP formulation with Coulomb cone.)
- Bullet Physics Library. `btSequentialImpulseConstraintSolver.cpp`. (Reference implementation of block PGS with decoupled normal/tangent.)
- Open Dynamics Engine (ODE). `quickstep.cpp`. (Reference implementation of PGS with box friction approximation.)

## Open Questions

### Mathematical Decisions (Human Input Needed)

1. **System-level net-zero correction vs split-impulse approach**

   The mathematical analysis establishes that the energy injection is a system-level property (net K_nt contribution across contacts) while the correct tipping torque is a per-contact differential property. Two mathematically valid correction strategies are identified:

   - **Option A: Post-sweep net-zero redistribution.** After each sweep, sum the K_nt-driven normal corrections across all contacts sharing a body pair. Subtract the mean to enforce $\sum \Delta\lambda_n^{\text{K_{nt}}} = 0$. This preserves the per-contact differential exactly while eliminating net injection. Complexity: O(N_c) per sweep per body pair. Requires grouping contacts by body pair.

   - **Option B: Decoupled solve with separate torque correction.** Use decoupled normal/tangent solve (as in P2) but add a separate torque-correction pass that computes the missing K_nt angular impulse contribution and applies it directly to the angular velocity residual. This separates the energy pathway (blocked) from the torque pathway (explicitly computed). Complexity: requires computing and applying angular corrections.

   - **Option C: Accept coupled solve with damping on net normal growth.** Keep the full coupled K^{-1} solve but add a damping factor on the net normal impulse growth rate. For example, limit $\Delta\Lambda_n$ per sweep to a fraction of $mg\Delta t$. This is heuristic but simple to implement.

   - Recommendation: Option A is the most mathematically rigorous and preserves the exact per-contact K^{-1} structure. Its only drawback is the need to group contacts by body pair, which the solver may not currently do.

2. **CFM epsilon impact on K_nt cancellation**

   The CFM regularization $\epsilon = 10^{-8}$ added to the K diagonal slightly perturbs the K_nt structure. For the four-corner cube, this adds $10^{-8}$ to each diagonal entry, shifting the K^{-1} off-diagonals by $O(10^{-8})$. This is negligible compared to the K_nt values of $O(1)$, but should be verified that it does not break the exact cancellation in the axis-aligned case.

3. **Contact pair grouping availability**

   Option A (net-zero redistribution) requires knowing which contacts belong to the same body pair. The current `sweepOnce` iterates over contacts sequentially. Does the `CollisionPipeline` already group contacts by body pair (from the contact manifold)? If so, the grouping is available. If contacts from different body pairs are interleaved, the solver would need to track per-body-pair accumulators.

### Clarifications Needed

1. The tangent basis computation in `setSlidingMode()` aligns t1 with the sliding direction. For oblique sliding, this creates the asymmetric K_nt pattern analyzed above. Does the tangent basis change between frames as the sliding direction evolves, or is it fixed at the initial sliding direction? Frame-to-frame tangent basis changes could amplify or dampen the K_nt feedback loop.

2. The Baumgarte ERP position correction adds a bias term to $v_{\text{err,n}}$ that makes it slightly negative even for non-penetrating contacts. This interacts with the K_nt coupling by providing a non-zero $v_{\text{err,n}}$ that the coupled solve can amplify. The magnitude of this interaction should be quantified.

### Beyond Scope

1. **Higher-order contact manifolds**: This analysis assumes 4-point contact manifolds (cube corners). Different geometries (edge contacts, 2-point manifolds) would have different K_nt cancellation properties that should be analyzed separately.

2. **Warm-start interaction**: The warm-start seeds $\lambda$ from the previous frame. If the previous frame had incorrect K_nt-driven normal impulses, the warm-start perpetuates them. Fix F2 (phaseBLambdas) mitigates this for Phase A contamination but does not address Phase B K_nt contamination persisting through warm-start.

3. **Convergence rate of PGS with corrected K_nt**: Any modification to the per-contact solve (redistribution, damping) may affect the convergence rate of the PGS iteration. This should be benchmarked but is beyond the scope of the mathematical formulation.

---

## Math Review -- Initial Assessment

**Reviewer**: Math Review Agent
**Date**: 2026-02-28
**Status**: REVISION_REQUESTED
**Iteration**: 0 of 1

### Issues Requiring Revision

| ID | Issue | Category | Required Correction |
|----|-------|----------|---------------------|
| I1 | K_ang matrix display has typo in entry (2,1) | Derivation | Fix matrix entry (2,1) from $\mathbf{a}_2^T \mathbf{I}_A^{-1} \mathbf{a}_2$ to $\mathbf{a}_2^T \mathbf{I}_A^{-1} \mathbf{a}_1$ |
| I2 | Oblique t2 vector has wrong sign | Derivation | $\mathbf{n} \times \mathbf{t}_1 = [1/\sqrt{2}, -1/\sqrt{2}, 0]^T$, not $[-1/\sqrt{2}, 1/\sqrt{2}, 0]^T$ |
| I3 | Section 3 energy injection mechanism incorrectly attributed | Derivation/Coverage | The net $\sum_i K_{n,t_1}^{(i)} \cdot v_{\text{err,t1}}^{(i)}$ is identically zero for ANY angular velocity. Energy injection requires the nonlinear Coulomb cone projection to break the cancellation symmetry. The formulation must be corrected to identify the cone projection as the essential nonlinearity. |
| I4 | Per-Contact Indistinguishability Theorem proof sketch is incomplete | Derivation | The theorem statement is correct but the proof sketch does not address the system-level mechanism (cone projection breaking cancellation). The theorem needs to be stated more precisely. |
| I5 | Example 2 fails to demonstrate net energy injection | Example/Coverage | The hand computation shows net = 0 for all angular velocity choices, contradicting the claimed mechanism. Need a worked example showing the cone projection breaking the cancellation. |
| I6 | Post-sweep net-zero redistribution criterion uses wrong quantity | Derivation | Section 6 defines $\Delta\Lambda_n^{K_{nt}}$ using $K^{-1}_{01}(i) \cdot v_{\text{err,t1}}^{(i)}$ summed over contacts. This sum is provably zero (by the same symmetry as $\sum K_{n,t_1}^{(i)} = 0$). The criterion must operate on post-projection quantities, not pre-projection. |

### Revision Instructions for Mathematician

The following corrections must be made before final review:

1. **Issue I1 -- K_ang matrix typo (line 159)**: In the $K_A^{\text{ang}}$ matrix display, entry (2,1) (row 2, column 1, zero-indexed) reads $\mathbf{a}_2^T \mathbf{I}_A^{-1} \mathbf{a}_2$. This should be $\mathbf{a}_2^T \mathbf{I}_A^{-1} \mathbf{a}_1$ (the cross-term between t2 and t1 angular contributions).
   - Current: Row 2 of K_ang is $[-\mathbf{a}_2^T \mathbf{I}_A^{-1} \mathbf{a}_n, \; \mathbf{a}_2^T \mathbf{I}_A^{-1} \mathbf{a}_2, \; \mathbf{a}_2^T \mathbf{I}_A^{-1} \mathbf{a}_2]$
   - Problem: Entries (2,1) and (2,2) are identical, which is incorrect. (2,1) is the t2-t1 cross-term.
   - Required: Row 2 should be $[-\mathbf{a}_2^T \mathbf{I}_A^{-1} \mathbf{a}_n, \; \mathbf{a}_2^T \mathbf{I}_A^{-1} \mathbf{a}_1, \; \mathbf{a}_2^T \mathbf{I}_A^{-1} \mathbf{a}_2]$
   - Note: This is a display typo only. All subsequent numerical computations use the correct values.

2. **Issue I2 -- Oblique t2 sign error (line 241)**: The cross product $\mathbf{n} \times \mathbf{t}_1$ is computed incorrectly.
   - Current: $\mathbf{t}_2 = \mathbf{n} \times \mathbf{t}_1 = [0,0,1]^T \times \frac{1}{\sqrt{2}}[-1, -1, 0]^T = \frac{1}{\sqrt{2}}[-1, 1, 0]^T$
   - Problem: $[0,0,1] \times [-1,-1,0] = [(0)(0) - (1)(-1), \; (1)(-1) - (0)(0), \; (0)(-1) - (0)(-1)] = [1, -1, 0]$. So $\mathbf{t}_2 = \frac{1}{\sqrt{2}}[1, -1, 0]^T$.
   - Required: Correct the t2 vector. This sign error does not affect K_nt1 computations (t2 is not used there), but it would affect K_nt2 values for the oblique case if they were computed. Since only K_nt1 is analyzed for the oblique case, the downstream conclusions are unaffected -- but the document should be corrected for accuracy.

3. **Issue I3 -- Energy injection mechanism (Section 3)**: This is the most significant mathematical issue. The formulation claims that the net normal impulse $\Delta\Lambda_n = \sum_i K^{-1}_{01}(i) \cdot v_{\text{err,t1}}^{(i)}$ is non-zero for the oblique case when differential angular velocities break the summation symmetry. Independent verification proves this is incorrect:
   - For the oblique tangent basis, K_nt1 values are $\{+2.121, 0, 0, -2.121\}$ for corners labeled by $(r_x + r_y) = \{+1, 0, 0, -1\}$.
   - Corners with K_nt1 = 0 (corners 2 and 3) contribute zero to the net regardless of their velocity errors.
   - Corners 1 and 4 (K_nt1 = +2.121 and -2.121) have lever arms related by 180-degree rotation about Z. For ANY angular velocity $\boldsymbol{\omega}$, the tangential velocity component $\mathbf{t}_1 \cdot (\boldsymbol{\omega} \times \mathbf{r}_A)$ is identical at corners 1 and 4 because their position vectors differ only in sign ($\mathbf{r}_1 = -\mathbf{r}_4$ in the XY plane). Therefore $K_{nt1}^{(1)} \cdot v_{err,t1}^{(1)} + K_{nt1}^{(4)} \cdot v_{err,t1}^{(4)} = 0$ exactly.
   - The same holds for the full K_inv analysis: $\sum_i [K^{-1}]_{0j}^{(i)} \cdot v_{\text{err},j}^{(i)} = 0$ for all j, for any angular velocity.
   - **The actual mechanism**: The Coulomb cone projection $\|\lambda_t\| \leq \mu \lambda_n$ is a nonlinear operation that clips different contacts by different amounts. After projection, the per-contact normal corrections are no longer the raw K_inv outputs -- they are modified by the cone clipping. This asymmetric clipping breaks the cancellation that holds in the linear (pre-projection) regime. The energy injection is a property of the K_nt coupling interacting with the nonlinear cone projection, not of the linear K_nt coupling alone.
   - Required: Rewrite Section 3 to correctly identify the Coulomb cone projection as the essential nonlinearity that breaks the cancellation. The K_nt coupling is necessary but not sufficient -- it is the combination of K_nt coupling + nonlinear cone projection + iterative PGS feedback that produces the energy injection. Include a brief numerical demonstration showing that the pre-projection net is zero but the post-projection net is non-zero.

4. **Issue I4 -- Per-Contact Indistinguishability Theorem (Section 4)**: The theorem statement and conclusion (that per-contact fixes fail) are correct, as confirmed by prototypes P2-P5. However, the proof sketch needs updating to reflect the corrected mechanism. The reason per-contact fixes fail is not just because the K matrix encodes no multi-contact information, but because the energy injection arises from the interaction between K_nt coupling and cone projection across multiple contacts -- a system-level nonlinear property.
   - Required: Update the proof sketch to reference the cone-projection mechanism. The corollary about system-level fixes remains valid and important.

5. **Issue I5 -- Example 2 does not demonstrate energy injection**: The hand computation in Example 2 (lines 728-824) shows that the net K_nt contribution is zero for all angular velocity choices tried. The formulation itself discovers this ("Hmm, with the angular velocity being purely about Z, the symmetry is preserved") and then tries several other angular velocities, all yielding zero. The formulation then pivots to a qualitative argument about multi-iteration feedback, which is correct in spirit but unverified numerically.
   - Required: Replace or augment Example 2 with a computation that demonstrates the actual mechanism: (a) compute the pre-projection unconstrained impulses for all 4 contacts (showing net = 0), then (b) apply the Coulomb cone projection with realistic lambda_n values and show the post-projection net is non-zero, then (c) explain that this post-projection asymmetry is what drives the iterative feedback loop.

6. **Issue I6 -- Post-sweep correction criterion (Section 6)**: The net K_nt normal impulse injection defined as $\sum_i [K^{-1}_{01}(i) \cdot v_{\text{err,t1}}^{(i)} + K^{-1}_{02}(i) \cdot v_{\text{err,t2}}^{(i)}]$ is identically zero (as shown above). This quantity cannot serve as a criterion for energy injection.
   - Required: The criterion must be reformulated to operate on post-projection (post-cone-clipping) normal impulse changes, not pre-projection. For example: after each PGS sweep, sum the actual $\Delta\lambda_n^{(i)}$ (after accumulation and cone projection) for all contacts sharing a body pair. If this net is positive when all contacts have $v_{\text{err,n}} \geq 0$ (no penetration), it indicates energy injection from the cone-projection asymmetry. The redistribution formula ($\Delta\lambda_n^{(i),\text{corrected}} = \Delta\lambda_n^{(i)} - \bar{\Delta\lambda_n}$) can still be applied, but on the post-projection quantities.

### Items Passing Review (No Changes Needed)

- **Section 1 (K matrix derivation)**: Correct. The split into linear + angular contributions is verified. The key result that K_nt arises exclusively from rotational coupling is confirmed. All cross products, dot products, and the final K matrix for the unit cube corner are numerically verified.
- **Section 2 (K_nt values for specific geometries)**: The axis-aligned K_nt1 = -3*r_x formula is correct and verified. The oblique K_nt1 = 3(r_x + r_y)/sqrt(2) formula is correct and verified. The summation to zero in both cases is correct.
- **Section 5 (correct normal impulse for oblique sliding)**: The physical argument is sound.
- **Section 7 (Schur complement)**: All numerical values verified: S = 2.2, K_inv(0,0) = 0.4545, ratio = 1.818, eigenvalues {1.0, 5.5, 5.5}, condition number 5.5, det(K) = 30.25.
- **Example 1 (axis-aligned K_nt cancellation)**: Hand computation verified. K_nt1 values and their sum are correct. GTest template is correct.
- **Example 3 (sphere contact K_nt = 0)**: Correct. The K matrix is diagonal for r_A parallel to n.
- **Example 4 (Schur complement ratio)**: All intermediate values verified independently. GTest template is correct.
- **Special cases and edge conditions**: Degenerate cases 1-3 are correctly analyzed.
- **Numerical stability analysis**: Appropriate and complete.
- **Tolerance recommendations**: Reasonable.

---
