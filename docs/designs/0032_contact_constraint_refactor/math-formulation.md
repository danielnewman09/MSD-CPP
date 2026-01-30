# Mathematical Formulation: Contact Constraint System

> Rigorous derivation of constraint-based collision response for Ticket 0032.
> This document provides the mathematical foundation for implementing `ContactConstraint` within the existing Lagrangian constraint framework.

**Ticket**: [0032_contact_constraint_refactor](../../../tickets/0032_contact_constraint_refactor.md)
**Design**: [design.md](./design.md) (to be created)
**PlantUML**: [0032_contact_constraint_refactor.puml](./0032_contact_constraint_refactor.puml)

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Contact Constraint Formulation](#2-contact-constraint-formulation)
3. [Constraint Jacobian Derivation](#3-constraint-jacobian-derivation)
4. [Multi-Body Lagrange Multiplier System](#4-multi-body-lagrange-multiplier-system)
5. [Complementarity Conditions](#5-complementarity-conditions)
6. [Restitution Handling](#6-restitution-handling)
7. [Projected Gauss-Seidel Solver](#7-projected-gauss-seidel-solver)
8. [Baumgarte Stabilization](#8-baumgarte-stabilization)
9. [Numerical Stability Analysis](#9-numerical-stability-analysis)
10. [Special Cases](#10-special-cases)
11. [Numerical Examples](#11-numerical-examples)
12. [GTest Templates](#12-gtest-templates)
13. [References](#13-references)

---

## 1. Introduction

### 1.1 Motivation

The current collision response system (Ticket 0027) uses a standalone `CollisionResponse` namespace that computes impulses directly. While functional, this approach operates independently from the Lagrangian constraint framework (Ticket 0031). Unifying these systems provides:

- **Consistency**: All constraint forces computed through the same solver infrastructure
- **Accuracy**: Contact constraints benefit from Baumgarte stabilization for drift correction
- **Physical correctness**: Lagrangian formulation ensures energy-consistent constraint enforcement
- **Extensibility**: Future friction constraints integrate naturally as additional constraint rows

### 1.2 Scope

This document derives the mathematical formulation for:

1. **Position-level constraint**: Non-penetration condition $C(\mathbf{q}) \geq 0$
2. **Velocity-level constraint**: Relative normal velocity condition
3. **Two-body Jacobian**: Partial derivatives involving lever arms
4. **Constrained dynamics**: Multi-body system $\mathbf{J} \mathbf{M}^{-1} \mathbf{J}^\top \boldsymbol{\lambda} = \mathbf{b}$
5. **Inequality handling**: Complementarity conditions and projection
6. **Numerical methods**: Projected Gauss-Seidel with convergence analysis

### 1.3 Notation

| Symbol | Description | Units |
|--------|-------------|-------|
| $\mathbf{q}$ | Generalized coordinates (position + quaternion) | m, - |
| $\dot{\mathbf{q}}$ | Generalized velocities | m/s, 1/s |
| $C(\mathbf{q})$ | Constraint function (scalar for single contact) | m |
| $\mathbf{J}$ | Constraint Jacobian $(\partial C / \partial \mathbf{q})$ | - |
| $\mathbf{M}$ | Generalized mass matrix (block diagonal) | kg, kg$\cdot$m$^2$ |
| $\lambda$ | Lagrange multiplier (constraint force magnitude) | N |
| $\mathbf{n}$ | Contact normal (unit vector, A $\to$ B) | - |
| $\mathbf{r}_A, \mathbf{r}_B$ | Lever arms from center of mass to contact point | m |
| $\mathbf{v}_A, \mathbf{v}_B$ | Linear velocities of bodies A and B | m/s |
| $\boldsymbol{\omega}_A, \boldsymbol{\omega}_B$ | Angular velocities of bodies A and B | rad/s |
| $e$ | Coefficient of restitution | - |
| $\alpha, \beta$ | Baumgarte stabilization parameters | 1/s$^2$, 1/s |
| $\Delta t$ | Timestep | s |

---

## 2. Contact Constraint Formulation

### 2.1 Position-Level Constraint (Unilateral)

For two rigid bodies A and B in contact, the non-penetration constraint is:

$$
C(\mathbf{q}) = (\mathbf{x}_B^c - \mathbf{x}_A^c) \cdot \mathbf{n} \geq 0
$$

Where:
- $\mathbf{x}_A^c$ is the contact point on body A's surface (world coordinates)
- $\mathbf{x}_B^c$ is the contact point on body B's surface (world coordinates)
- $\mathbf{n}$ is the contact normal pointing from A toward B

**Interpretation**:
- $C > 0$: Bodies are separated (constraint inactive)
- $C = 0$: Bodies are in contact (constraint active)
- $C < 0$: Bodies are penetrating (constraint violated)

### 2.2 Contact Point Expression

The contact points are expressed in terms of body center of mass positions and lever arms:

$$
\mathbf{x}_A^c = \mathbf{x}_A + \mathbf{r}_A
$$

$$
\mathbf{x}_B^c = \mathbf{x}_B + \mathbf{r}_B
$$

Where:
- $\mathbf{x}_A, \mathbf{x}_B$ are center of mass positions
- $\mathbf{r}_A, \mathbf{r}_B$ are lever arms from center of mass to contact point (world frame)

For rotating bodies, the lever arms transform with orientation:

$$
\mathbf{r}_A = \mathbf{R}_A \mathbf{r}_A^{\text{body}}
$$

$$
\mathbf{r}_B = \mathbf{R}_B \mathbf{r}_B^{\text{body}}
$$

Where $\mathbf{R}_A, \mathbf{R}_B$ are rotation matrices derived from quaternions $\mathbf{Q}_A, \mathbf{Q}_B$.

### 2.3 Signed Penetration Depth

When bodies penetrate, we define the signed penetration depth $d$ as:

$$
d = -C(\mathbf{q}) = (\mathbf{x}_A^c - \mathbf{x}_B^c) \cdot \mathbf{n}
$$

For penetrating configurations, $d > 0$ represents the overlap distance along the contact normal.

### 2.4 Constraint Function with Penetration

The complete constraint function incorporating penetration depth from collision detection:

$$
C(\mathbf{q}) = (\mathbf{x}_B - \mathbf{x}_A) \cdot \mathbf{n} + (\mathbf{r}_B - \mathbf{r}_A) \cdot \mathbf{n}
$$

$$
= (\mathbf{x}_B - \mathbf{x}_A) \cdot \mathbf{n} + \Delta\mathbf{r} \cdot \mathbf{n}
$$

Where $\Delta\mathbf{r} = \mathbf{r}_B - \mathbf{r}_A$ is the difference in lever arms.

For the simplified case where contact occurs at the same world point (single contact approximation):

$$
C(\mathbf{q}) = (\mathbf{x}_B + \mathbf{r}_B - \mathbf{x}_A - \mathbf{r}_A) \cdot \mathbf{n}
$$

### 2.5 Generalization to Multiple Contact Points

The formulation above describes a single contact point between bodies A and B. In practice, collision detection may produce a **contact manifold** with $k$ contact points (e.g., face-face collisions produce up to 4 contact points in the existing `CollisionResult` structure).

**Each contact point becomes an independent constraint row.** For $k$ contact points between the same body pair:

$$
C_i(\mathbf{q}) = (\mathbf{x}_B + \mathbf{r}_{B,i} - \mathbf{x}_A - \mathbf{r}_{A,i}) \cdot \mathbf{n}_i \geq 0 \quad \text{for } i = 1, \ldots, k
$$

Each constraint $C_i$ has its own:
- Contact normal $\mathbf{n}_i$ (may differ across points for curved surfaces; identical for face-face contacts)
- Lever arms $\mathbf{r}_{A,i}$, $\mathbf{r}_{B,i}$ from each body's center of mass to the $i$-th contact point
- Lagrange multiplier $\lambda_i \geq 0$

The constraint Jacobian becomes a $k \times 12$ matrix (stacking $k$ rows of the $(1 \times 12)$ Jacobian from Section 3.6):

$$
\mathbf{J} = \begin{bmatrix} \mathbf{J}_1 \\ \vdots \\ \mathbf{J}_k \end{bmatrix} \quad (k \times 12)
$$

The effective mass matrix becomes $k \times k$:

$$
\mathbf{A} = \mathbf{J} \mathbf{M}^{-1} \mathbf{J}^\top \quad (k \times k)
$$

Off-diagonal entries $A_{ij}$ couple contact points through the shared mass matrix of bodies A and B (see Section 10.3 for the explicit coupling formula). The system $\mathbf{A}\boldsymbol{\lambda} = \mathbf{b}$ with $\boldsymbol{\lambda} \geq 0$ is solved by the Projected Gauss-Seidel algorithm (Section 7).

**Relationship to existing code**: The current `CollisionResult` manifold stores up to 4 `ContactPoint` entries. The centroid approach (averaging contacts into a single representative point) reduces $k$ constraints to 1, which is a valid approximation. However, using individual contact points as separate constraint rows provides better stability for resting contacts (e.g., an object resting on a flat surface with 4 support points) because the solver can independently balance forces at each contact location.

---

## 3. Constraint Jacobian Derivation

### 3.1 Single-Body Jacobian Review

From the existing constraint framework (Ticket 0031), single-body constraints have Jacobian:

$$
\mathbf{J} = \frac{\partial C}{\partial \mathbf{q}} = \left[ \frac{\partial C}{\partial \mathbf{x}}, \frac{\partial C}{\partial \mathbf{Q}} \right]
$$

For a 7-DOF state (position $\mathbf{x} \in \mathbb{R}^3$, quaternion $\mathbf{Q} \in \mathbb{R}^4$), $\mathbf{J}$ is a $(\text{dim} \times 7)$ matrix.

### 3.2 Two-Body Jacobian Structure

For contact constraints involving two bodies, the Jacobian spans both state vectors:

$$
\mathbf{J} = \begin{bmatrix} \mathbf{J}_A & \mathbf{J}_B \end{bmatrix}
$$

$$
\mathbf{J}_A = \left[ \frac{\partial C}{\partial \mathbf{x}_A}, \frac{\partial C}{\partial \mathbf{Q}_A} \right] \quad (\text{dim} \times 7)
$$

$$
\mathbf{J}_B = \left[ \frac{\partial C}{\partial \mathbf{x}_B}, \frac{\partial C}{\partial \mathbf{Q}_B} \right] \quad (\text{dim} \times 7)
$$

The combined Jacobian is $(\text{dim} \times 14)$ for the stacked state vector $[\mathbf{q}_A; \mathbf{q}_B]$.

### 3.3 Linear Component Derivation

The constraint function depends on positions:

$$
C = (\mathbf{x}_B + \mathbf{r}_B - \mathbf{x}_A - \mathbf{r}_A) \cdot \mathbf{n}
$$

Taking derivatives with respect to center of mass positions. **Key observation**: the lever arm $\mathbf{r}_A = \mathbf{R}(\mathbf{Q}_A)\mathbf{r}_A^{\text{body}}$ depends on body A's *orientation* $\mathbf{Q}_A$, not on its *position* $\mathbf{x}_A$. Therefore when differentiating $C$ with respect to $\mathbf{x}_A$, the lever arm terms $\mathbf{r}_A$ and $\mathbf{r}_B$ (and the constant $\mathbf{n}$) drop out:

$$
\frac{\partial C}{\partial \mathbf{x}_A} = \frac{\partial}{\partial \mathbf{x}_A}\left[(\mathbf{x}_B + \mathbf{r}_B - \mathbf{x}_A - \mathbf{r}_A) \cdot \mathbf{n}\right] = \frac{\partial}{\partial \mathbf{x}_A}\left[-\mathbf{x}_A \cdot \mathbf{n}\right] = -\mathbf{n}^\top \quad (1 \times 3 \text{ row vector})
$$

$$
\frac{\partial C}{\partial \mathbf{x}_B} = \frac{\partial}{\partial \mathbf{x}_B}\left[(\mathbf{x}_B + \mathbf{r}_B - \mathbf{x}_A - \mathbf{r}_A) \cdot \mathbf{n}\right] = \frac{\partial}{\partial \mathbf{x}_B}\left[\mathbf{x}_B \cdot \mathbf{n}\right] = +\mathbf{n}^\top \quad (1 \times 3 \text{ row vector})
$$

**Physical intuition**: If body A's center of mass is translated by a small displacement $\delta\mathbf{x}_A$ (without rotating), the contact point on A moves rigidly with it. The gap $C$ changes by $-\delta\mathbf{x}_A \cdot \mathbf{n}$: moving A toward B (i.e., in the direction of $\mathbf{n}$) *closes* the gap, hence the negative sign. Conversely, moving B away from A (in the direction of $\mathbf{n}$) *opens* the gap, hence the positive sign for $\partial C / \partial \mathbf{x}_B$.

### 3.4 Angular Component Derivation

**Why angular velocity appears**: The constraint $C$ depends on body orientations through the lever arms $\mathbf{r}_A = \mathbf{R}(\mathbf{Q}_A)\mathbf{r}_A^{\text{body}}$. For constrained dynamics, we need the **velocity-level** Jacobian that maps generalized velocities $\dot{\mathbf{q}}_v = [\mathbf{v}, \boldsymbol{\omega}]$ to the constraint rate $\dot{C}$. This is distinct from the position-level Jacobian $\partial C / \partial \mathbf{q}_p$ where $\mathbf{q}_p = [\mathbf{x}, \mathbf{Q}]$.

**How $\boldsymbol{\omega}$ enters**: The time derivative of the lever arm is given by the rigid body kinematics identity:

$$
\dot{\mathbf{r}}_A = \boldsymbol{\omega}_A \times \mathbf{r}_A
$$

By the chain rule, the constraint rate expands as:

$$
\dot{C} = \frac{\partial C}{\partial \mathbf{x}_A} \cdot \mathbf{v}_A + \frac{\partial C}{\partial \mathbf{r}_A} \cdot \dot{\mathbf{r}}_A + \frac{\partial C}{\partial \mathbf{x}_B} \cdot \mathbf{v}_B + \frac{\partial C}{\partial \mathbf{r}_B} \cdot \dot{\mathbf{r}}_B
$$

Substituting $\dot{\mathbf{r}}_A = \boldsymbol{\omega}_A \times \mathbf{r}_A$ and $\dot{\mathbf{r}}_B = \boldsymbol{\omega}_B \times \mathbf{r}_B$ naturally introduces $\boldsymbol{\omega}_A$ and $\boldsymbol{\omega}_B$ into the expression. We then collect terms to identify the effective Jacobian with respect to $\boldsymbol{\omega}$. Working in $\boldsymbol{\omega}$-space (rather than $\dot{\mathbf{Q}}$-space) is standard practice because it avoids the redundancy of 4 quaternion components for 3 rotational degrees of freedom; the conversion to $\dot{\mathbf{Q}}$-space is handled later in Section 3.5.

**Formal derivation**: The lever arms depend on orientation through the rotation matrices:

$$
\mathbf{r}_A = \mathbf{R}(\mathbf{Q}_A) \mathbf{r}_A^{\text{body}}
$$

Taking the derivative:

$$
\frac{\partial \mathbf{r}_A}{\partial \mathbf{Q}_A} = \frac{\partial \mathbf{R}}{\partial \mathbf{Q}_A} \mathbf{r}_A^{\text{body}}
$$

Using the quaternion-rotation relationship, for a rotation of point $\mathbf{p}$:

$$
\frac{\partial (\mathbf{R}(\mathbf{Q}) \mathbf{p})}{\partial \boldsymbol{\omega}} = -[\mathbf{R}(\mathbf{Q}) \mathbf{p}]_\times
$$

Where $[\cdot]_\times$ denotes the skew-symmetric cross-product matrix.

The constraint derivative with respect to angular velocity:

$$
\frac{\partial C}{\partial \boldsymbol{\omega}_A} = -\mathbf{n}^\top \frac{\partial \mathbf{r}_A}{\partial \boldsymbol{\omega}_A} = -\mathbf{n}^\top (-[\mathbf{r}_A]_\times) = \mathbf{n}^\top [\mathbf{r}_A]_\times = -(\mathbf{r}_A \times \mathbf{n})^\top
$$

Similarly:

$$
\frac{\partial C}{\partial \boldsymbol{\omega}_B} = (\mathbf{r}_B \times \mathbf{n})^\top
$$

### 3.5 Complete Jacobian Assembly

The velocity-level constraint is:

$$
\dot{C} = \mathbf{J}_A \dot{\mathbf{q}}_A + \mathbf{J}_B \dot{\mathbf{q}}_B
$$

Where $\dot{\mathbf{q}}$ includes both linear velocity $\mathbf{v}$ and angular velocity $\boldsymbol{\omega}$ (or equivalently, quaternion rate $\dot{\mathbf{Q}}$).

**Linear-angular Jacobian (6-DOF per body)**:

$$
\mathbf{J}_A^{(6)} = \begin{bmatrix} -\mathbf{n}^\top & -(\mathbf{r}_A \times \mathbf{n})^\top \end{bmatrix} \quad (1 \times 6)
$$

$$
\mathbf{J}_B^{(6)} = \begin{bmatrix} +\mathbf{n}^\top & +(\mathbf{r}_B \times \mathbf{n})^\top \end{bmatrix} \quad (1 \times 6)
$$

**Expanded form for existing framework (7-DOF per body)**:

The current framework uses $\dot{\mathbf{q}} = [v_x, v_y, v_z, \dot{Q}_w, \dot{Q}_x, \dot{Q}_y, \dot{Q}_z]$.

The angular Jacobian must be transformed from $\boldsymbol{\omega}$-space to $\dot{\mathbf{Q}}$-space:

$$
\boldsymbol{\omega} = 2 \bar{\mathbf{Q}} \otimes \dot{\mathbf{Q}} \quad \text{(quaternion multiplication)}
$$

Therefore:

$$
\mathbf{J}_Q = \mathbf{J}_\omega \cdot 2 \mathbf{E}(\mathbf{Q})^\top
$$

Where $\mathbf{E}(\mathbf{Q})$ is the $3 \times 4$ matrix relating $\dot{\mathbf{Q}}$ to $\boldsymbol{\omega}$.

### 3.6 Practical Implementation

For implementation, we work with the 6-DOF angular velocity formulation and convert to constraint forces:

$$
\mathbf{J} = \begin{bmatrix} -\mathbf{n}^\top & -(\mathbf{r}_A \times \mathbf{n})^\top & +\mathbf{n}^\top & +(\mathbf{r}_B \times \mathbf{n})^\top \end{bmatrix}
$$

This is $(1 \times 12)$ for $[\mathbf{v}_A, \boldsymbol{\omega}_A, \mathbf{v}_B, \boldsymbol{\omega}_B]$.

> **Interface Note**: The existing `Constraint::jacobian()` method takes a single `InertialState` and returns a single-body Jacobian. The two-body $(1 \times 12)$ Jacobian derived here does not map directly to this interface. However, it decomposes naturally into two $(1 \times 6)$ blocks -- one per body:
>
> $$\mathbf{J}_A^{(6)} = \begin{bmatrix} -\mathbf{n}^\top & -(\mathbf{r}_A \times \mathbf{n})^\top \end{bmatrix}, \quad \mathbf{J}_B^{(6)} = \begin{bmatrix} +\mathbf{n}^\top & +(\mathbf{r}_B \times \mathbf{n})^\top \end{bmatrix}$$
>
> The `ContactConstraint` implementation must store references or indices for both bodies and assemble the full Jacobian by combining per-body blocks. This requires extending the existing `Constraint` interface (or introducing a `TwoBodyConstraint` subclass) to accept two body states. The constraint solver must correspondingly be extended to handle multi-body state vectors when computing the effective mass and applying impulses. This interface extension is an architectural concern to be addressed in the design document.

The constraint force (impulse) in generalized coordinates:

$$
\mathbf{F}_{\text{constraint}} = \mathbf{J}^\top \lambda = \begin{bmatrix} -\mathbf{n} \lambda \\ -(\mathbf{r}_A \times \mathbf{n}) \lambda \\ +\mathbf{n} \lambda \\ +(\mathbf{r}_B \times \mathbf{n}) \lambda \end{bmatrix}
$$

---

## 4. Multi-Body Lagrange Multiplier System

### 4.1 Constrained Dynamics Equation

For a system of rigid bodies with constraints, the equations of motion are:

$$
\mathbf{M} \ddot{\mathbf{q}} = \mathbf{F}_{\text{ext}} + \mathbf{J}^\top \boldsymbol{\lambda}
$$

Where:
- $\mathbf{M}$ is the generalized mass matrix (block diagonal)
- $\ddot{\mathbf{q}}$ is the generalized acceleration
- $\mathbf{F}_{\text{ext}}$ is the external force
- $\mathbf{J}^\top \boldsymbol{\lambda}$ is the constraint force

### 4.2 Mass Matrix Structure

For a two-body system, the mass matrix is block diagonal:

$$
\mathbf{M} = \begin{bmatrix} \mathbf{M}_A & \mathbf{0} \\ \mathbf{0} & \mathbf{M}_B \end{bmatrix}
$$

Each body's mass matrix $(6 \times 6)$:

$$
\mathbf{M}_i = \begin{bmatrix} m_i \mathbf{I}_3 & \mathbf{0} \\ \mathbf{0} & \mathbf{I}_i \end{bmatrix}
$$

Where:
- $m_i$ is the mass
- $\mathbf{I}_i$ is the $3 \times 3$ inertia tensor (in world frame)

The inverse mass matrix:

$$
\mathbf{M}_i^{-1} = \begin{bmatrix} m_i^{-1} \mathbf{I}_3 & \mathbf{0} \\ \mathbf{0} & \mathbf{I}_i^{-1} \end{bmatrix}
$$

### 4.3 Lagrange Multiplier Equation

At the velocity level, we require:

$$
\dot{C}^+ = \mathbf{J} \dot{\mathbf{q}}^+ = b_{\text{target}}
$$

Where $\dot{C}^+$ is the post-constraint velocity and $b_{\text{target}}$ encodes the desired behavior (typically 0 for maintaining contact, or $-e \dot{C}^-$ for restitution).

Using the velocity update:

$$
\dot{\mathbf{q}}^+ = \dot{\mathbf{q}}^- + \mathbf{M}^{-1} \mathbf{J}^\top \boldsymbol{\lambda}
$$

Substituting:

$$
\mathbf{J} \left( \dot{\mathbf{q}}^- + \mathbf{M}^{-1} \mathbf{J}^\top \boldsymbol{\lambda} \right) = b_{\text{target}}
$$

$$
\mathbf{J} \dot{\mathbf{q}}^- + \mathbf{J} \mathbf{M}^{-1} \mathbf{J}^\top \boldsymbol{\lambda} = b_{\text{target}}
$$

Rearranging:

$$
\mathbf{A} \boldsymbol{\lambda} = \mathbf{b}
$$

Where:

$$
\mathbf{A} = \mathbf{J} \mathbf{M}^{-1} \mathbf{J}^\top \quad \text{(effective mass matrix)}
$$

$$
\mathbf{b} = b_{\text{target}} - \mathbf{J} \dot{\mathbf{q}}^- \quad \text{(right-hand side)}
$$

### 4.4 Effective Mass Expansion

For a single contact constraint with the Jacobian:

$$
\mathbf{J} = \begin{bmatrix} -\mathbf{n}^\top & -(\mathbf{r}_A \times \mathbf{n})^\top & +\mathbf{n}^\top & +(\mathbf{r}_B \times \mathbf{n})^\top \end{bmatrix}
$$

The effective mass (scalar for 1D constraint):

$$
A = \mathbf{J} \mathbf{M}^{-1} \mathbf{J}^\top
$$

$$
= \mathbf{n}^\top m_A^{-1} \mathbf{n} + (\mathbf{r}_A \times \mathbf{n})^\top \mathbf{I}_A^{-1} (\mathbf{r}_A \times \mathbf{n}) + \mathbf{n}^\top m_B^{-1} \mathbf{n} + (\mathbf{r}_B \times \mathbf{n})^\top \mathbf{I}_B^{-1} (\mathbf{r}_B \times \mathbf{n})
$$

Simplifying (since $\mathbf{n}$ is a unit vector):

$$
A = \left( \frac{1}{m_A} + \frac{1}{m_B} \right) + (\mathbf{r}_A \times \mathbf{n})^\top \mathbf{I}_A^{-1} (\mathbf{r}_A \times \mathbf{n}) + (\mathbf{r}_B \times \mathbf{n})^\top \mathbf{I}_B^{-1} (\mathbf{r}_B \times \mathbf{n})
$$

### 4.5 Relationship to Existing Implementation

The current `CollisionResponse::computeLagrangeMultiplier()` computes:

```cpp
double effectiveMass = invMassSum + angularContribA + angularContribB;
double lambda = (1.0 + restitution) * vRelNormal / effectiveMass;
```

This is equivalent to solving $A \lambda = b$ with:
- $A = \text{effectiveMass}$
- $b = (1 + e) \, v_{\text{rel}} \cdot \mathbf{n}$

The constraint formulation provides the same result with additional structure for multi-constraint systems.

---

## 5. Complementarity Conditions

### 5.1 Unilateral Constraint Properties

Contact constraints are unilateral (inequality constraints) with the following conditions:

1. **Non-penetration**: $C \geq 0$
2. **Compression only**: $\lambda \geq 0$ (contact forces push, never pull)
3. **Complementarity**: $\lambda \cdot C = 0$ (force is zero when not in contact)

### 5.2 Velocity-Level Complementarity

At the velocity level:

1. **Non-separation**: $\dot{C} \geq 0$ (relative velocity along normal is non-negative)
2. **Compression only**: $\lambda \geq 0$
3. **Complementarity**: $\lambda \cdot \dot{C} = 0$

### 5.3 Linear Complementarity Problem (LCP)

The system forms a Linear Complementarity Problem:

$$
\text{Find } \boldsymbol{\lambda} \text{ such that:}
$$

$$
\mathbf{w} = \mathbf{A} \boldsymbol{\lambda} + \mathbf{b}
$$

$$
\boldsymbol{\lambda} \geq 0
$$

$$
\mathbf{w} \geq 0
$$

$$
\boldsymbol{\lambda} \cdot \mathbf{w} = 0
$$

Where $\mathbf{w} = \dot{C}^+$ is the post-solve constraint velocity.

### 5.4 Solution Methods

**Direct methods** (Lemke's algorithm): Exact solution for small systems, $O(n^3)$ complexity with pivoting.

**Iterative methods** (Projected Gauss-Seidel): Approximate solution with clamping, $O(n \cdot k)$ per iteration, robust for large systems.

**Recommendation**: Projected Gauss-Seidel is industry standard (Bullet, ODE, Box2D) due to robustness and simplicity.

---

## 6. Restitution Handling

### 6.1 Coefficient of Restitution

The coefficient of restitution $e$ relates pre-collision and post-collision normal velocities:

$$
v_{\text{rel},n}^+ = -e \cdot v_{\text{rel},n}^-
$$

Where:
- $v_{\text{rel},n}^-$ is the pre-collision relative normal velocity (approaching)
- $v_{\text{rel},n}^+$ is the post-collision relative normal velocity (separating)
- $e \in [0, 1]$ where $e=0$ is perfectly inelastic, $e=1$ is perfectly elastic

### 6.2 Modified Constraint Target

The restitution modifies the constraint right-hand side:

$$
b_{\text{target}} = -e \cdot \dot{C}^- \quad \text{(when } \dot{C}^- > 0 \text{, approaching)}
$$

$$
b_{\text{target}} = 0 \quad \text{(when } \dot{C}^- \leq 0 \text{, separating)}
$$

The constraint equation becomes:

$$
A \lambda = -e \cdot (\mathbf{J} \dot{\mathbf{q}}^-) - (\mathbf{J} \dot{\mathbf{q}}^-)
$$

$$
= -(1 + e) \cdot (\mathbf{J} \dot{\mathbf{q}}^-)
$$

For approaching contacts ($\mathbf{J} \dot{\mathbf{q}}^- < 0$, equivalently $v_{\text{rel},n} > 0$ since $\dot{C} = -v_{\text{rel},n}$ with our Jacobian convention):

$$
\lambda = \frac{-(1 + e) \cdot (\mathbf{J} \dot{\mathbf{q}}^-)}{A} = \frac{(1 + e) \cdot v_{\text{rel},n}}{A} > 0
$$

> **Implementation Note (from Prototype P2 validation):** The formulas above define the **constraint RHS** for the PGS solver system $A\lambda = b$, where $b = -(1+e) \cdot \mathbf{J}\dot{\mathbf{q}}^-$. This must not be confused with the **target velocity** formulation used in direct velocity updates: $v_{\text{target}} = -e \cdot v_{\text{pre}}$. Both produce the same impulse, but they operate through different code paths. Using the RHS formula $-(1+e) \cdot v$ as a target velocity (without the solver) would double-count the pre-velocity contribution, resulting in energy injection instead of dissipation. See `prototypes/0032_contact_constraint_refactor/p2_energy_conservation/Debug_Findings.md` for the detailed root cause analysis.

### 6.3 Combined Restitution

For two bodies with different restitution coefficients $e_A$ and $e_B$, use geometric mean:

$$
e = \sqrt{e_A \cdot e_B}
$$

This ensures:
- $e(A,B) = e(B,A)$ (symmetric)
- $e = 0$ if either body is fully inelastic
- $e = 1$ only if both bodies are fully elastic

### 6.4 Velocity Threshold

For low relative velocities, disable restitution to prevent jitter:

$$
\text{if } |v_{\text{rel},n}| < \text{threshold}: \quad e_{\text{effective}} = 0
$$

$$
\text{else}: \quad e_{\text{effective}} = e
$$

Typical threshold: 0.5 - 1.0 m/s (see `CollisionResponse::kRestVelocityThreshold = 0.0001` m/s in current implementation, which is very conservative).

---

## 7. Projected Gauss-Seidel Solver

### 7.1 Algorithm Overview

Projected Gauss-Seidel (PGS) solves the LCP iteratively:

```
for iter = 1 to maxIterations:
    for i = 1 to numConstraints:
        // Compute unconstrained update
        delta_lambda_i = (b_i - sum_{j!=i} A_ij * lambda_j) / A_ii - lambda_i

        // Project to feasible region
        lambda_i = max(0, lambda_i + delta_lambda_i)
```

### 7.2 Detailed Algorithm

**Input**:
- $\mathbf{A}$: Effective mass matrix $(n \times n)$
- $\mathbf{b}$: Right-hand side vector $(n \times 1)$
- $\boldsymbol{\lambda}_0$: Initial guess $(n \times 1)$, typically zeros or warm start

**Output**:
- $\boldsymbol{\lambda}$: Lagrange multipliers satisfying complementarity

**Algorithm**: `Projected_Gauss_Seidel`$(\mathbf{A}, \mathbf{b}, \boldsymbol{\lambda}_0, \text{maxIter}, \text{tolerance})$

1. $\boldsymbol{\lambda} \leftarrow \boldsymbol{\lambda}_0$
2. **for** $\text{iter} = 1$ **to** $\text{maxIter}$:
3. $\quad \boldsymbol{\lambda}_{\text{old}} \leftarrow \boldsymbol{\lambda}$
4. $\quad$ **for** $i = 1$ **to** $n$:
5. $\quad\quad$ // Compute residual for constraint $i$
6. $\quad\quad$ $\text{residual} \leftarrow b_i$
7. $\quad\quad$ **for** $j = 1$ **to** $n$:
8. $\quad\quad\quad$ **if** $j \neq i$:
9. $\quad\quad\quad\quad$ $\text{residual} \leftarrow \text{residual} - A_{ij} \lambda_j$
10. $\quad\quad$ // Update $\lambda_i$ (Gauss-Seidel step)
11. $\quad\quad$ $\lambda_i^{\text{unconstrained}} \leftarrow \text{residual} / A_{ii}$
12. $\quad\quad$ // Project to $\lambda \geq 0$ (unilateral constraint)
13. $\quad\quad$ $\lambda_i \leftarrow \max(0, \lambda_i^{\text{unconstrained}})$
14. $\quad$ // Check convergence
15. $\quad$ **if** $\|\boldsymbol{\lambda} - \boldsymbol{\lambda}_{\text{old}}\| < \text{tolerance}$:
16. $\quad\quad$ **return** $\boldsymbol{\lambda}$
17. **return** $\boldsymbol{\lambda}$

### 7.3 Diagonal Dominance Preconditioning

For better convergence, precondition by scaling constraints to unit diagonal:

$$
\mathbf{A}_{\text{scaled}} = \mathbf{D}^{-1/2} \mathbf{A} \mathbf{D}^{-1/2}
$$

$$
\mathbf{b}_{\text{scaled}} = \mathbf{D}^{-1/2} \mathbf{b}
$$

Where $\mathbf{D} = \text{diag}(\mathbf{A})$.

After solving, recover:

$$
\boldsymbol{\lambda} = \mathbf{D}^{-1/2} \boldsymbol{\lambda}_{\text{scaled}}
$$

### 7.4 Warm Starting

Use previous frame's $\boldsymbol{\lambda}$ as initial guess:

$$
\boldsymbol{\lambda}_0 = \alpha \cdot \boldsymbol{\lambda}_{\text{previous}}
$$

Where $\alpha \in [0.5, 1.0]$ accounts for configuration changes. Typical value: $\alpha = 0.9$.

Benefits:
- Faster convergence (2-3x fewer iterations)
- Reduced jitter for stable contacts
- Requires contact matching across frames

### 7.5 Convergence Analysis

**Convergence rate**: Linear convergence for well-conditioned systems.

**Spectral radius**: $\rho(\mathbf{I} - \mathbf{D}^{-1} \mathbf{A})$ determines convergence rate.
- $\rho < 1$: Converges
- $\rho \approx 1$: Slow convergence
- $\rho > 1$: Diverges

**Sufficient condition**: $\mathbf{A}$ is symmetric positive semi-definite (guaranteed for physical mass matrices).

---

## 8. Baumgarte Stabilization

### 8.1 Position Drift Problem

Numerical integration causes constraint drift over time:
- Each timestep, position errors accumulate
- Penetration depth can grow unbounded
- Visual artifacts and instability result

### 8.2 Baumgarte Formulation

Add feedback terms to the constraint equation to correct drift:

$$
\ddot{C} + \beta \dot{C} + \alpha C = 0
$$

This is a critically damped oscillator equation with:
- $\alpha$: Position error gain $[1/\text{s}^2]$
- $\beta$: Velocity error gain $[1/\text{s}]$

Critical damping: $\beta = 2\sqrt{\alpha}$

### 8.3 Modified Right-Hand Side

The Baumgarte-stabilized constraint equation:

$$
\mathbf{A} \boldsymbol{\lambda} = \mathbf{b} - \alpha C - \beta \dot{C}
$$

Where:
- $\mathbf{b} = -(1 + e) \mathbf{J} \dot{\mathbf{q}}^-$ (restitution target)
- $C$ = penetration depth (negative when penetrating)
- $\dot{C} = \mathbf{J} \dot{\mathbf{q}}$ (relative normal velocity)

### 8.4 Parameter Selection

**Theoretical values** (critical damping):

$$
\omega_n = \sqrt{\alpha} \quad \text{(natural frequency)}
$$

$$
\zeta = \frac{\beta}{2\omega_n} \quad \text{(damping ratio)}
$$

For critical damping ($\zeta = 1$):

$$
\beta = 2\sqrt{\alpha}
$$

**Practical values** depend on timestep $\Delta t$:

$$
\alpha = \frac{4}{\Delta t^2} \quad \text{(stiff correction)}
$$

$$
\beta = \frac{4}{\Delta t} \quad \text{(critically damped)}
$$

For $\Delta t = 0.016$ s (60 FPS):

$$
\alpha \approx 15625 \; [1/\text{s}^2]
$$

$$
\beta \approx 250 \; [1/\text{s}]
$$

**Default values in existing framework** (Ticket 0031):

$$
\alpha = 10.0 \; [1/\text{s}^2]
$$

$$
\beta = 10.0 \; [1/\text{s}]
$$

These are much softer values, suitable for bilateral constraints with small violations. For contact constraints with larger penetrations, higher values may be needed.

### 8.5 Energy Considerations

**Problem**: Baumgarte stabilization can add energy to the system.

**Mitigation strategies**:
1. Use soft (low) stabilization parameters
2. Apply stabilization only for penetration correction, not velocity
3. Use split impulse method (separate position and velocity corrections)

### 8.6 Split Impulse Method

Separate the correction into position and velocity components:

**Velocity solve** (standard):

$$
\lambda_v \text{ such that } \dot{C}^+ = -e \cdot \dot{C}^-
$$

**Position solve** (bias):

$$
\lambda_p \text{ such that position error is corrected}
$$

Apply position correction as pseudo-velocity:

$$
\mathbf{x}^+ = \mathbf{x}^- + \Delta t \cdot (\mathbf{v} + \mathbf{v}_{\text{bias}})
$$

$$
\mathbf{v}_{\text{bias}} = \frac{\alpha}{\Delta t} \cdot C \cdot \mathbf{n}
$$

This prevents energy injection while still correcting penetration.

---

## 9. Numerical Stability Analysis

### 9.1 Condition Number Analysis

The effective mass matrix $\mathbf{A} = \mathbf{J} \mathbf{M}^{-1} \mathbf{J}^\top$ condition number:

$$
\kappa(\mathbf{A}) = \frac{\sigma_{\max}}{\sigma_{\min}}
$$

Where $\sigma_{\max}, \sigma_{\min}$ are largest and smallest singular values.

**Ill-conditioning sources**:
1. Large mass ratios ($m_A \gg m_B$)
2. Small lever arms ($\mathbf{r} \approx 0$)
3. Nearly-parallel constraints
4. Redundant constraints

### 9.2 Mass Ratio Sensitivity

For two bodies with masses $m_A$ and $m_B$:

$$
\kappa \sim \frac{\max(m_A, m_B)}{\min(m_A, m_B)}
$$

**Rule of thumb**: Mass ratios $> 100:1$ cause stability issues.

**Mitigation**: Scale constraint equations by effective mass.

### 9.3 Convergence Criteria

**Residual-based**:

$$
\|\mathbf{A} \boldsymbol{\lambda} - \mathbf{b}\|_\infty < \epsilon
$$

**Change-based**:

$$
\|\boldsymbol{\lambda}^{k+1} - \boldsymbol{\lambda}^k\|_\infty < \epsilon
$$

**Typical tolerance**: $\epsilon = 10^{-4}$ to $10^{-6}$

### 9.4 Iteration Limits

**Empirical guidelines**:
- Simple contacts: 4-10 iterations
- Stacked objects: 10-20 iterations
- Complex configurations: 20-50 iterations

**Adaptive iteration**: Increase iterations when residual is high.

### 9.5 Regularization

Add small diagonal term for numerical stability:

$$
\mathbf{A}_{\text{reg}} = \mathbf{A} + \epsilon \mathbf{I}
$$

Where $\epsilon = 10^{-7}$ to $10^{-10}$.

**Effect**: Adds small compliance to constraints, preventing singularity.

### 9.6 Clamping and Limiting

**Impulse clamping**:

$$
\lambda = \text{clamp}(\lambda, 0, \lambda_{\max})
$$

Where $\lambda_{\max}$ prevents unrealistic impulses. Typical: $\lambda_{\max} = 10 \cdot m \cdot g \cdot \Delta t$.

**Position correction clamping**:

$$
\text{correction} = \text{clamp}(\text{penetration} - \text{slop}, 0, \text{max\_correction})
$$

Typical values:
- slop $= 0.01$ m (1 cm)
- max\_correction $= 0.2$ m per frame

---

## 10. Special Cases

### 10.1 Resting Contact

**Characteristics**:
- Relative normal velocity $\approx 0$
- Persistent contact over multiple frames
- Gravity must be balanced by contact force

**Handling**:
1. Disable restitution when $|v_{\text{rel},n}| < \text{threshold}$
2. Include gravity in constraint RHS:

$$
\mathbf{b} = -\mathbf{J} \mathbf{M}^{-1} \mathbf{F}_{\text{gravity}} \Delta t
$$

3. Use warm starting for stability

**Expected lambda**:

$$
\lambda \approx m \cdot g \cdot \cos\theta \cdot \Delta t
$$

Where $\theta$ is the angle between contact normal and gravity.

### 10.2 Grazing Contact

**Characteristics**:
- Contact normal nearly perpendicular to relative velocity
- $v_{\text{rel},n} \approx 0$ but $|v_{\text{rel},t}| \gg 0$
- Tangential motion dominates

**Handling**:
1. Use velocity threshold to activate/deactivate constraint
2. Avoid restitution for grazing contacts
3. Angular effects may dominate linear effects

**Jacobian sensitivity**:
The cross product $\mathbf{r} \times \mathbf{n}$ becomes important:
- If $\mathbf{r}$ parallel to $\mathbf{v}_{\text{rel}}$: angular effect dominates
- If $\mathbf{r}$ perpendicular to $\mathbf{v}_{\text{rel}}$: linear effect dominates

### 10.3 Multiple Simultaneous Contacts

**Characteristics**:
- Single object touching multiple surfaces
- Constraints are coupled through shared body
- Order of processing affects result (for sequential solvers)

**Handling**:
1. Assemble coupled constraint matrix:

$$
\mathbf{A} = \begin{bmatrix} A_{11} & A_{12} & \cdots \\ A_{21} & A_{22} & \cdots \\ \vdots & \vdots & \ddots \end{bmatrix}
$$

2. Off-diagonal terms couple constraints
3. Use iterative solver with multiple passes

**Coupling terms**:
For constraints $i$ and $j$ sharing body $k$:

$$
A_{ij} = \mathbf{J}_i^k \mathbf{M}_k^{-1} (\mathbf{J}_j^k)^\top
$$

**Explicit expansion**: For two contacts $i$ and $j$ on the same body pair (A, B), the coupling term expands to:

$$
A_{ij} = \mathbf{n}_i^\top m_A^{-1} \mathbf{n}_j + (\mathbf{r}_{A,i} \times \mathbf{n}_i)^\top \mathbf{I}_A^{-1} (\mathbf{r}_{A,j} \times \mathbf{n}_j) + \mathbf{n}_i^\top m_B^{-1} \mathbf{n}_j + (\mathbf{r}_{B,i} \times \mathbf{n}_i)^\top \mathbf{I}_B^{-1} (\mathbf{r}_{B,j} \times \mathbf{n}_j)
$$

When two contacts share the same normal ($\mathbf{n}_i = \mathbf{n}_j$), the linear terms in $A_{ij}$ equal those in $A_{ii}$, making the matrix singular without the angular contributions. In practice, distinct lever arms ($\mathbf{r}_{A,i} \neq \mathbf{r}_{A,j}$) provide the angular differentiation needed for regularity. For nearly-degenerate configurations, regularization (Section 9.5) may be necessary.

The coupled system is solved by the Projected Gauss-Seidel algorithm (Section 7), which handles the off-diagonal coupling naturally through its iterative updates.

### 10.4 Static-Dynamic Collision

**Characteristics**:
- One body has infinite mass (environment)
- Only dynamic body moves
- Simplified Jacobian and effective mass

**Handling**:
Set $m_{\text{static}} = \infty$, $\mathbf{I}_{\text{static}} = \infty$:

$$
\mathbf{M}_{\text{static}}^{-1} = \mathbf{0}
$$

Effective mass simplifies to:

$$
A = \frac{1}{m_{\text{dynamic}}} + (\mathbf{r}_{\text{dynamic}} \times \mathbf{n})^\top \mathbf{I}_{\text{dynamic}}^{-1} (\mathbf{r}_{\text{dynamic}} \times \mathbf{n})
$$

**Current implementation**: `CollisionResponse::computeLagrangeMultiplierStatic()` handles this case separately from the two-body path (`computeLagrangeMultiplier()`), resulting in duplicate code paths.

> **Implementation Note**: To enable a unified constraint solver code path (eliminating separate static/dynamic functions), `AssetEnvironment` should be extended with explicit inverse mass properties:
>
> - `inverseMass_ = 0.0`
> - `inverseInertia_ = Eigen::Matrix3d::Zero()`
>
> With these properties, the constraint solver treats environment objects identically to inertial objects without special-case branching. The effective mass formula (Section 4.4) naturally handles this: when $m_B^{-1} = 0$ and $\mathbf{I}_B^{-1} = \mathbf{0}$, body B's contribution to the effective mass vanishes, and body B receives zero velocity change from the constraint impulse.
>
> **Design options** (to be captured in the design document):
> - **Option A**: Add `inverseMass_` and `inverseInertia_` members directly to `AssetEnvironment`, initialized to zero. Minimal change.
> - **Option B**: Create a common `MassProperties` struct (with a static factory for infinite mass) used by both `AssetInertial` and `AssetEnvironment`.
> - **Recommendation**: Option A for minimal disruption; Option B if additional mass property consumers are anticipated.

### 10.5 Edge-Edge Contact

**Characteristics**:
- Contact between edges (not faces)
- Contact normal may be ambiguous
- Lever arms computed to closest points

**Handling**:
1. Contact normal = normalized cross product of edge directions
2. Contact point = midpoint of closest points on edges
3. Penetration depth requires special computation (EPA handles this)

### 10.6 Vertex-Face Contact

**Characteristics**:
- Single contact point
- Well-defined contact normal (face normal)
- Simple lever arm computation

**Handling**:
Standard formulation applies directly:
- Contact point = vertex location
- Contact normal = face outward normal
- Lever arms = vertex position - body center of mass

---

## 11. Numerical Examples

### 11.1 Example 1: Head-On Collision (1D)

**Setup**:
- Two identical spheres ($m = 1$ kg, $I = 0.4$ kg$\cdot$m$^2$)
- Body A at origin, velocity $\mathbf{v}_A = (2, 0, 0)$ m/s
- Body B at $(3, 0, 0)$ m, velocity $\mathbf{v}_B = (-1, 0, 0)$ m/s
- Contact normal $\mathbf{n} = (1, 0, 0)$
- Contact point at $(1.5, 0, 0)$ m
- Lever arms: $\mathbf{r}_A = (1.5, 0, 0)$ m, $\mathbf{r}_B = (-1.5, 0, 0)$ m
- Restitution $e = 1.0$ (elastic)

**Step 1: Compute relative velocity**

$$
\mathbf{v}_{\text{rel}} = \mathbf{v}_A + \boldsymbol{\omega}_A \times \mathbf{r}_A - \mathbf{v}_B - \boldsymbol{\omega}_B \times \mathbf{r}_B
$$

$$
= (2, 0, 0) + (0, 0, 0) \times (1.5, 0, 0) - (-1, 0, 0) - (0, 0, 0) \times (-1.5, 0, 0)
$$

$$
= (2, 0, 0) - (-1, 0, 0) = (3, 0, 0) \text{ m/s}
$$

$$
v_{\text{rel},n} = \mathbf{v}_{\text{rel}} \cdot \mathbf{n} = 3 \text{ m/s}
$$

**Step 2: Compute effective mass**

$$
A = \frac{1}{m_A} + \frac{1}{m_B} + (\mathbf{r}_A \times \mathbf{n})^\top \mathbf{I}_A^{-1} (\mathbf{r}_A \times \mathbf{n}) + (\mathbf{r}_B \times \mathbf{n})^\top \mathbf{I}_B^{-1} (\mathbf{r}_B \times \mathbf{n})
$$

$$
= \frac{1}{1} + \frac{1}{1} + 0 + 0 = 2 \text{ kg}^{-1}
$$

Note: $\mathbf{r}_A \times \mathbf{n} = \mathbf{0}$ since $\mathbf{r}_A$ is parallel to $\mathbf{n}$.

**Step 3: Compute Lagrange multiplier**

$$
b = (1 + e) \cdot v_{\text{rel},n} = (1 + 1) \cdot 3 = 6 \text{ m/s}
$$

$$
\lambda = \frac{b}{A} = \frac{6}{2} = 3 \text{ N}\cdot\text{s}
$$

**Step 4: Apply velocity changes**

$$
\Delta \mathbf{v}_A = -\frac{\lambda \mathbf{n}}{m_A} = -\frac{3 \cdot (1, 0, 0)}{1} = (-3, 0, 0) \text{ m/s}
$$

$$
\Delta \mathbf{v}_B = +\frac{\lambda \mathbf{n}}{m_B} = +\frac{3 \cdot (1, 0, 0)}{1} = (+3, 0, 0) \text{ m/s}
$$

$$
\mathbf{v}_A^+ = (2, 0, 0) + (-3, 0, 0) = (-1, 0, 0) \text{ m/s}
$$

$$
\mathbf{v}_B^+ = (-1, 0, 0) + (3, 0, 0) = (2, 0, 0) \text{ m/s}
$$

**Verification**:
- Velocities swapped (correct for elastic collision of equal masses)
- Momentum conserved: $1 \cdot 2 + 1 \cdot (-1) = 1$ kg$\cdot$m/s before
- Momentum conserved: $1 \cdot (-1) + 1 \cdot 2 = 1$ kg$\cdot$m/s after
- Kinetic energy conserved: $0.5 \cdot 1 \cdot 4 + 0.5 \cdot 1 \cdot 1 = 2.5$ J before and after

---

### 11.2 Example 2: Glancing Collision with Angular Effects

**Setup**:
- Cube A ($m = 2$ kg, $\mathbf{I} = \text{diag}(0.333, 0.333, 0.333)$ kg$\cdot$m$^2$, edge = 1 m)
- Cube B ($m = 1$ kg, $\mathbf{I} = \text{diag}(0.167, 0.167, 0.167)$ kg$\cdot$m$^2$, edge = 1 m)
- Body A at origin, velocity $\mathbf{v}_A = (1, 0, 0)$ m/s
- Body B at $(2, 0, 0)$ m, velocity $\mathbf{v}_B = (0, 0, 0)$ m/s
- Contact normal $\mathbf{n} = (1, 0, 0)$
- Contact point at $(0.5, 0.5, 0)$ m (edge contact)
- Lever arm $\mathbf{r}_A = (0.5, 0.5, 0)$ m, $\mathbf{r}_B = (-0.5, 0.5, 0)$ m
- Restitution $e = 0.5$

**Step 1: Compute cross products**

$$
\mathbf{r}_A \times \mathbf{n} = (0.5, 0.5, 0) \times (1, 0, 0) = (0, 0, -0.5) \text{ rad}
$$

$$
\mathbf{r}_B \times \mathbf{n} = (-0.5, 0.5, 0) \times (1, 0, 0) = (0 \cdot 0 - 0 \cdot 0, \; 0 \cdot 1 - (-0.5) \cdot 0, \; (-0.5) \cdot 0 - 0.5 \cdot 1) = (0, 0, -0.5) \text{ rad}
$$

**Step 2: Compute angular contribution**

$$
\text{angular}_A = (\mathbf{r}_A \times \mathbf{n})^\top \mathbf{I}_A^{-1} (\mathbf{r}_A \times \mathbf{n})
$$

$$
= (0, 0, -0.5) \cdot \left( \frac{1}{0.333} \cdot (0, 0, -0.5) \right) = (0, 0, -0.5) \cdot (0, 0, -1.5) = 0.75 \text{ kg}^{-1}
$$

$$
\text{angular}_B = (\mathbf{r}_B \times \mathbf{n})^\top \mathbf{I}_B^{-1} (\mathbf{r}_B \times \mathbf{n})
$$

$$
= (0, 0, -0.5) \cdot \left( \frac{1}{0.167} \cdot (0, 0, -0.5) \right) = (0, 0, -0.5) \cdot (0, 0, -3.0) = 1.5 \text{ kg}^{-1}
$$

**Step 3: Compute effective mass**

$$
A = \frac{1}{m_A} + \frac{1}{m_B} + \text{angular}_A + \text{angular}_B
$$

$$
= \frac{1}{2} + \frac{1}{1} + 0.75 + 1.5 = 0.5 + 1.0 + 0.75 + 1.5 = 3.75 \text{ kg}^{-1}
$$

**Step 4: Compute Lagrange multiplier**

$$
v_{\text{rel},n} = \mathbf{v}_A \cdot \mathbf{n} - \mathbf{v}_B \cdot \mathbf{n} = 1 - 0 = 1 \text{ m/s}
$$

$$
b = (1 + e) \cdot v_{\text{rel},n} = 1.5 \cdot 1 = 1.5 \text{ m/s}
$$

$$
\lambda = \frac{b}{A} = \frac{1.5}{3.75} = 0.4 \text{ N}\cdot\text{s}
$$

**Step 5: Apply velocity changes**

Linear:

$$
\Delta \mathbf{v}_A = -\frac{0.4 \cdot (1, 0, 0)}{2} = (-0.2, 0, 0) \text{ m/s}
$$

$$
\Delta \mathbf{v}_B = +\frac{0.4 \cdot (1, 0, 0)}{1} = (+0.4, 0, 0) \text{ m/s}
$$

Angular:

$$
\Delta \boldsymbol{\omega}_A = -\mathbf{I}_A^{-1} \left( \mathbf{r}_A \times (\lambda \mathbf{n}) \right) = -3.0 \cdot (0.5, 0.5, 0) \times (0.4, 0, 0)
$$

$$
= -3.0 \cdot (0, 0, -0.2) = (0, 0, 0.6) \text{ rad/s}
$$

$$
\Delta \boldsymbol{\omega}_B = +\mathbf{I}_B^{-1} \left( \mathbf{r}_B \times (\lambda \mathbf{n}) \right) = +6.0 \cdot (-0.5, 0.5, 0) \times (0.4, 0, 0)
$$

$$
= +6.0 \cdot (0, 0, -0.2) = (0, 0, -1.2) \text{ rad/s}
$$

> **Physical intuition**: The impulse $\lambda \mathbf{n} = (0.4, 0, 0)$ pushes body B's contact point (which is above its center of mass, at relative position $(-0.5, 0.5, 0)$) in the $+x$ direction. This creates a clockwise torque about the $z$-axis when viewed from above, hence the negative $\omega_z$.

**Final velocities**:

$$
\mathbf{v}_A^+ = (1, 0, 0) + (-0.2, 0, 0) = (0.8, 0, 0) \text{ m/s}
$$

$$
\mathbf{v}_B^+ = (0, 0, 0) + (0.4, 0, 0) = (0.4, 0, 0) \text{ m/s}
$$

$$
\boldsymbol{\omega}_A^+ = (0, 0, 0) + (0, 0, 0.6) = (0, 0, 0.6) \text{ rad/s}
$$

$$
\boldsymbol{\omega}_B^+ = (0, 0, 0) + (0, 0, -1.2) = (0, 0, -1.2) \text{ rad/s}
$$

**Verification**:
- Total linear momentum before: $2 \cdot 1 + 1 \cdot 0 = 2$ kg$\cdot$m/s
- Total linear momentum after: $2 \cdot 0.8 + 1 \cdot 0.4 = 2$ kg$\cdot$m/s (conserved)

---

### 11.3 Example 3: Resting Contact with Gravity

**Setup**:
- Cube A resting on infinite plane B
- $m_A = 5$ kg, $\mathbf{I}_A = \text{diag}(0.833, 0.833, 0.833)$ kg$\cdot$m$^2$ (1 m cube)
- Gravity $\mathbf{g} = (0, 0, -9.81)$ m/s$^2$
- Contact normal $\mathbf{n} = (0, 0, 1)$
- Contact point at $(0, 0, 0)$ (cube's bottom center)
- Lever arm $\mathbf{r}_A = (0, 0, -0.5)$ m
- Restitution $e = 0$ (inelastic, resting)
- Timestep $\Delta t = 0.016$ s (60 FPS)

**Step 1: Compute gravitational velocity change (unconstrained)**

$$
\Delta \mathbf{v}_{\text{gravity}} = \mathbf{g} \cdot \Delta t = (0, 0, -9.81) \cdot 0.016 = (0, 0, -0.157) \text{ m/s}
$$

**Step 2: Compute effective mass**

$$
\mathbf{r}_A \times \mathbf{n} = (0, 0, -0.5) \times (0, 0, 1) = (0, 0, 0) \quad \text{(parallel vectors)}
$$

$$
\text{angular}_A = 0
$$

$$
A = \frac{1}{m_A} = \frac{1}{5} = 0.2 \text{ kg}^{-1}
$$

**Step 3: Compute required constraint impulse**

$$
v_{\text{rel},n} = \Delta \mathbf{v}_{\text{gravity}} \cdot \mathbf{n} = -0.157 \text{ m/s} \quad \text{(penetrating velocity)}
$$

$$
b = -v_{\text{rel},n} = 0.157 \text{ m/s} \quad \text{(cancel penetrating velocity, } e=0 \text{)}
$$

$$
\lambda = \frac{b}{A} = \frac{0.157}{0.2} = 0.785 \text{ N}\cdot\text{s}
$$

**Step 4: Verify force balance**

$$
\text{Constraint force} = \frac{\lambda}{\Delta t} = \frac{0.785}{0.016} = 49.05 \text{ N}
$$

$$
\text{Gravitational force} = m \cdot g = 5 \cdot 9.81 = 49.05 \text{ N}
$$

Forces balance exactly (as expected for static equilibrium).

**Continuous force interpretation**:
In the constraint framework, $\lambda$ represents impulse. The equivalent continuous force:

$$
F_{\text{contact}} = \frac{\lambda}{\Delta t} = m \cdot g \cdot \cos(0) = 49.05 \text{ N}
$$

---

## 12. GTest Templates

### 12.1 Test: Head-On Elastic Collision

```cpp
// File: msd-sim/test/Physics/Constraints/ContactConstraintTest.cpp

TEST_CASE("ContactConstraint: Head-on elastic collision swaps velocities")
{
  // Setup: Two identical 1kg spheres
  const double mass = 1.0;
  const double restitution = 1.0;

  // Create simplified inertial states for testing
  InertialState stateA;
  stateA.position = Coordinate{0.0, 0.0, 0.0};
  stateA.velocity = Coordinate{2.0, 0.0, 0.0};  // Moving right
  stateA.orientation = Eigen::Quaterniond::Identity();
  stateA.setAngularVelocity(AngularRate{0.0, 0.0, 0.0});

  InertialState stateB;
  stateB.position = Coordinate{3.0, 0.0, 0.0};
  stateB.velocity = Coordinate{-1.0, 0.0, 0.0};  // Moving left
  stateB.orientation = Eigen::Quaterniond::Identity();
  stateB.setAngularVelocity(AngularRate{0.0, 0.0, 0.0});

  // Contact information
  Coordinate contactNormal{1.0, 0.0, 0.0};  // A -> B
  Coordinate contactPoint{1.5, 0.0, 0.0};
  Coordinate leverArmA = contactPoint - stateA.position;  // (1.5, 0, 0)
  Coordinate leverArmB = contactPoint - stateB.position;  // (-1.5, 0, 0)

  // Inertia tensors (spheres, I = 0.4 * m * r^2, r=1)
  Eigen::Matrix3d inertiaA = 0.4 * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d inertiaB = 0.4 * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d invInertiaA = inertiaA.inverse();
  Eigen::Matrix3d invInertiaB = inertiaB.inverse();

  // Compute relative velocity
  AngularRate omegaA = stateA.getAngularVelocity();
  AngularRate omegaB = stateB.getAngularVelocity();
  Coordinate vContactA = stateA.velocity + omegaA.cross(leverArmA);
  Coordinate vContactB = stateB.velocity + omegaB.cross(leverArmB);
  Coordinate vRel = vContactA - vContactB;
  double vRelNormal = vRel.dot(contactNormal);

  REQUIRE(vRelNormal == Catch::Approx(3.0).epsilon(1e-10));  // Approaching at 3 m/s

  // Compute effective mass
  Coordinate rAxn = leverArmA.cross(contactNormal);
  Coordinate rBxn = leverArmB.cross(contactNormal);
  double angularA = rAxn.dot(invInertiaA * rAxn);
  double angularB = rBxn.dot(invInertiaB * rBxn);
  double effectiveMass = (1.0/mass) + (1.0/mass) + angularA + angularB;

  // r parallel to n, so cross product is zero
  REQUIRE(rAxn.norm() == Catch::Approx(0.0).margin(1e-10));
  REQUIRE(effectiveMass == Catch::Approx(2.0).epsilon(1e-10));

  // Compute lambda
  double lambda = (1.0 + restitution) * vRelNormal / effectiveMass;
  REQUIRE(lambda == Catch::Approx(3.0).epsilon(1e-10));

  // Apply impulses
  Coordinate impulse = contactNormal * lambda;
  Coordinate vA_new = stateA.velocity - impulse / mass;
  Coordinate vB_new = stateB.velocity + impulse / mass;

  // Verify velocity swap
  REQUIRE(vA_new.x() == Catch::Approx(-1.0).epsilon(1e-10));
  REQUIRE(vB_new.x() == Catch::Approx(2.0).epsilon(1e-10));

  // Verify momentum conservation
  Coordinate pBefore = mass * stateA.velocity + mass * stateB.velocity;
  Coordinate pAfter = mass * vA_new + mass * vB_new;
  REQUIRE((pAfter - pBefore).norm() < 1e-10);

  // Verify energy conservation (elastic)
  double keBefore = 0.5 * mass * stateA.velocity.squaredNorm()
                  + 0.5 * mass * stateB.velocity.squaredNorm();
  double keAfter = 0.5 * mass * vA_new.squaredNorm()
                 + 0.5 * mass * vB_new.squaredNorm();
  REQUIRE(keAfter == Catch::Approx(keBefore).epsilon(1e-10));
}
```

### 12.2 Test: Glancing Collision with Angular Effects

```cpp
TEST_CASE("ContactConstraint: Glancing collision generates angular velocity")
{
  // Setup: Two cubes with off-center contact
  const double massA = 2.0;
  const double massB = 1.0;
  const double restitution = 0.5;

  InertialState stateA;
  stateA.position = Coordinate{0.0, 0.0, 0.0};
  stateA.velocity = Coordinate{1.0, 0.0, 0.0};
  stateA.orientation = Eigen::Quaterniond::Identity();
  stateA.setAngularVelocity(AngularRate{0.0, 0.0, 0.0});

  InertialState stateB;
  stateB.position = Coordinate{2.0, 0.0, 0.0};
  stateB.velocity = Coordinate{0.0, 0.0, 0.0};
  stateB.orientation = Eigen::Quaterniond::Identity();
  stateB.setAngularVelocity(AngularRate{0.0, 0.0, 0.0});

  // Contact at edge (off-center)
  // Each body's lever arm is from its own center of mass to its surface contact point.
  // Body A (center at origin, 1m cube): surface contact at (0.5, 0.5, 0) -> r_A = (0.5, 0.5, 0)
  // Body B (center at (2,0,0), 1m cube): surface contact at (1.5, 0.5, 0) -> r_B = (-0.5, 0.5, 0)
  Coordinate contactNormal{1.0, 0.0, 0.0};
  Coordinate leverArmA{0.5, 0.5, 0.0};   // r_A from Example 11.2
  Coordinate leverArmB{-0.5, 0.5, 0.0};  // r_B from Example 11.2

  // Cube inertias: I = (1/6) * m * s^2 for s=1
  Eigen::Matrix3d inertiaA = (1.0/6.0) * massA * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d inertiaB = (1.0/6.0) * massB * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d invInertiaA = inertiaA.inverse();
  Eigen::Matrix3d invInertiaB = inertiaB.inverse();

  // Compute cross products
  Coordinate rAxn = leverArmA.cross(contactNormal);  // (0, 0, -0.5)
  Coordinate rBxn = leverArmB.cross(contactNormal);  // (0, 0, -0.5)

  REQUIRE(rAxn.z() == Catch::Approx(-0.5).epsilon(1e-10));
  REQUIRE(rBxn.z() == Catch::Approx(-0.5).epsilon(1e-10));

  // Compute angular contributions
  double angularA = rAxn.dot(invInertiaA * rAxn);
  double angularB = rBxn.dot(invInertiaB * rBxn);

  // I_zz = (1/6) * m, so I_zz^{-1} = 6/m
  REQUIRE(angularA == Catch::Approx(0.25 * 6.0 / massA).epsilon(1e-10));  // 0.75
  REQUIRE(angularB == Catch::Approx(0.25 * 6.0 / massB).epsilon(1e-10));  // 1.5

  // Effective mass
  double effectiveMass = (1.0/massA) + (1.0/massB) + angularA + angularB;
  REQUIRE(effectiveMass == Catch::Approx(3.75).epsilon(1e-10));

  // Relative velocity
  double vRelNormal = 1.0;  // Only A moving

  // Lambda
  double lambda = (1.0 + restitution) * vRelNormal / effectiveMass;
  REQUIRE(lambda == Catch::Approx(0.4).epsilon(1e-10));

  // Angular velocity changes
  Coordinate impulse = contactNormal * lambda;
  AngularRate deltaOmegaA = invInertiaA * leverArmA.cross(-impulse);
  AngularRate deltaOmegaB = invInertiaB * leverArmB.cross(impulse);

  // Body A spins CCW (positive z), Body B spins CW (negative z)
  // because the impulse pushes B's above-center contact point in +x direction
  REQUIRE(deltaOmegaA.z() > 0);   // CCW from above
  REQUIRE(deltaOmegaB.z() < 0);   // CW from above
  REQUIRE(deltaOmegaA.z() == Catch::Approx(0.6).epsilon(1e-10));
  REQUIRE(deltaOmegaB.z() == Catch::Approx(-1.2).epsilon(1e-10));

  // Verify momentum conservation
  Coordinate vA_new = stateA.velocity - impulse / massA;
  Coordinate vB_new = stateB.velocity + impulse / massB;
  Coordinate pBefore = massA * stateA.velocity + massB * stateB.velocity;
  Coordinate pAfter = massA * vA_new + massB * vB_new;
  REQUIRE((pAfter - pBefore).norm() < 1e-10);
}
```

### 12.3 Test: Resting Contact Balances Gravity

```cpp
TEST_CASE("ContactConstraint: Resting contact balances gravitational force")
{
  // Setup: Cube resting on infinite plane
  const double mass = 5.0;
  const double dt = 0.016;  // 60 FPS
  const Coordinate gravity{0.0, 0.0, -9.81};

  InertialState state;
  state.position = Coordinate{0.0, 0.0, 0.5};  // Center at 0.5m (1m cube)
  state.velocity = Coordinate{0.0, 0.0, 0.0};
  state.orientation = Eigen::Quaterniond::Identity();
  state.setAngularVelocity(AngularRate{0.0, 0.0, 0.0});

  // Contact with ground plane
  Coordinate contactNormal{0.0, 0.0, 1.0};  // Up from ground
  Coordinate contactPoint{0.0, 0.0, 0.0};   // Bottom of cube
  Coordinate leverArm = contactPoint - state.position;  // (0, 0, -0.5)

  // Gravity-induced velocity change
  Coordinate deltaVGravity = gravity * dt;  // (0, 0, -0.157)
  double vRelNormal = deltaVGravity.dot(contactNormal);  // -0.157 (penetrating)

  // For resting contact (e=0), we need to cancel the penetrating velocity
  double restitution = 0.0;

  // Effective mass (static body has infinite mass)
  Coordinate rxn = leverArm.cross(contactNormal);  // Zero (parallel)
  REQUIRE(rxn.norm() == Catch::Approx(0.0).margin(1e-10));
  double effectiveMass = 1.0 / mass;  // Only dynamic body contributes

  // Lambda to cancel gravity
  double b = -vRelNormal;  // Target: zero normal velocity
  double lambda = b / effectiveMass;

  // Verify lambda corresponds to gravitational force
  double contactForce = lambda / dt;
  double gravityForce = mass * std::abs(gravity.z());
  REQUIRE(contactForce == Catch::Approx(gravityForce).epsilon(1e-6));

  // Apply impulse
  Coordinate vNew = state.velocity + deltaVGravity + (contactNormal * lambda / mass);

  // Final velocity should be zero (resting)
  REQUIRE(vNew.norm() < 1e-10);
}
```

### 12.4 Test: Projected Gauss-Seidel Converges

```cpp
TEST_CASE("ContactConstraint: PGS solver converges for simple contact")
{
  // Simple 2-contact system with known solution
  // Two constraints, both active

  // Effective mass matrix (symmetric positive definite)
  Eigen::Matrix2d A;
  A << 2.0, 0.5,
       0.5, 3.0;

  // Right-hand side
  Eigen::Vector2d b{1.0, 1.5};

  // Expected solution (computed analytically): A * lambda = b
  Eigen::Vector2d lambdaExpected = A.llt().solve(b);

  // Both should be positive (active constraints)
  REQUIRE(lambdaExpected(0) > 0);
  REQUIRE(lambdaExpected(1) > 0);

  // PGS iteration
  const int maxIterations = 100;
  const double tolerance = 1e-8;
  Eigen::Vector2d lambda = Eigen::Vector2d::Zero();

  for (int iter = 0; iter < maxIterations; ++iter)
  {
    Eigen::Vector2d lambdaOld = lambda;

    // Update lambda_0
    double residual0 = b(0) - A(0, 1) * lambda(1);
    lambda(0) = std::max(0.0, residual0 / A(0, 0));

    // Update lambda_1
    double residual1 = b(1) - A(1, 0) * lambda(0);
    lambda(1) = std::max(0.0, residual1 / A(1, 1));

    // Check convergence
    if ((lambda - lambdaOld).norm() < tolerance)
    {
      INFO("Converged in " << iter + 1 << " iterations");
      break;
    }
  }

  // Verify solution
  REQUIRE(lambda(0) == Catch::Approx(lambdaExpected(0)).epsilon(1e-6));
  REQUIRE(lambda(1) == Catch::Approx(lambdaExpected(1)).epsilon(1e-6));

  // Verify constraint satisfaction
  Eigen::Vector2d w = A * lambda - b;
  REQUIRE(w.norm() < 1e-6);
}
```

### 12.5 Test: Complementarity Conditions Satisfied

```cpp
TEST_CASE("ContactConstraint: Inactive constraint has zero lambda")
{
  // One constraint should be inactive (separating contact)

  InertialState stateA;
  stateA.position = Coordinate{0.0, 0.0, 0.0};
  stateA.velocity = Coordinate{-1.0, 0.0, 0.0};  // Moving away

  InertialState stateB;
  stateB.position = Coordinate{2.0, 0.0, 0.0};
  stateB.velocity = Coordinate{1.0, 0.0, 0.0};   // Moving away

  Coordinate contactNormal{1.0, 0.0, 0.0};  // A -> B

  // Relative velocity
  Coordinate vRel = stateA.velocity - stateB.velocity;  // (-2, 0, 0)
  double vRelNormal = vRel.dot(contactNormal);  // -2 (separating)

  // Contact is separating, constraint should be inactive
  REQUIRE(vRelNormal < 0);

  // Lambda should be zero (constraint inactive)
  double lambda = 0.0;  // Complementarity: separating -> no force
  if (vRelNormal >= 0)
  {
    // Would compute lambda here
    lambda = (1.0 + 1.0) * vRelNormal / 2.0;  // Hypothetical
  }

  REQUIRE(lambda == 0.0);

  // Verify complementarity: lambda * C_dot = 0
  // Here lambda = 0, so condition is satisfied regardless of C_dot
  double complementarity = lambda * vRelNormal;
  REQUIRE(complementarity == 0.0);
}
```

### 12.6 Test: Multiple Contact Coupling

> **Note**: This test uses a simplified model that ignores angular contributions.
> When two contacts share the same normal on the same body and angular terms are
> absent, the effective mass matrix $\mathbf{A}$ is singular (see Section 10.3
> coupling formula). The test adds small diagonal regularization ($\epsilon = 10^{-6}$)
> to make $\mathbf{A}$ positive definite; this is consistent with the regularization
> strategy described in Section 9.5. In a full simulation, distinct lever arms
> would provide angular differentiation that naturally resolves the singularity.

```cpp
TEST_CASE("ContactConstraint: Multiple contacts couple correctly")
{
  // Object resting on two supports (simplified 2D)
  const double mass = 10.0;
  const double dt = 0.016;
  const Coordinate gravity{0.0, 0.0, -9.81};

  // Two contact points, both pushing up
  Coordinate n1{0.0, 0.0, 1.0};
  Coordinate n2{0.0, 0.0, 1.0};

  // Lever arms (symmetric about center)
  Coordinate r1{-0.5, 0.0, -0.5};  // Left support
  Coordinate r2{+0.5, 0.0, -0.5};  // Right support

  // For resting contact, need to balance gravity
  Coordinate deltaVGravity = gravity * dt;

  // Effective mass matrix (2x2 for 2 contacts)
  // A_11 = J_1 * M^{-1} * J_1^T = 1/m + angular_1
  // A_12 = J_1 * M^{-1} * J_2^T (coupling through shared body)

  // Simplified (ignoring angular for clarity):
  double invMass = 1.0 / mass;
  Eigen::Matrix2d A;
  A << invMass, invMass,   // Both contacts affect same body linearly
       invMass, invMass;

  // Add small regularization for positive definiteness
  A(0, 0) += 1e-6;
  A(1, 1) += 1e-6;

  // RHS: cancel gravity (distributed to both contacts)
  double vRelNormal = deltaVGravity.dot(n1);  // Same for both
  Eigen::Vector2d b{-vRelNormal, -vRelNormal};

  // Solve (expect equal distribution due to symmetry)
  Eigen::Vector2d lambda = A.llt().solve(b);

  // Each contact should carry half the load (symmetric setup)
  double expectedLambda = mass * std::abs(gravity.z()) * dt / 2.0;
  REQUIRE(lambda(0) == Catch::Approx(expectedLambda).epsilon(0.01));
  REQUIRE(lambda(1) == Catch::Approx(expectedLambda).epsilon(0.01));

  // Total force should balance gravity
  double totalForce = (lambda(0) + lambda(1)) / dt;
  double gravityForce = mass * std::abs(gravity.z());
  REQUIRE(totalForce == Catch::Approx(gravityForce).epsilon(0.01));
}
```

---

## 13. References

### Academic Literature

1. **Baraff, D.** (1994). "Fast Contact Force Computation for Nonpenetrating Rigid Bodies." SIGGRAPH '94.

2. **Catto, E.** (2005). "Iterative Dynamics with Temporal Coherence." Game Developers Conference.

3. **Stewart, D. & Trinkle, J.** (1996). "An Implicit Time-Stepping Scheme for Rigid Body Dynamics with Coulomb Friction." International Journal for Numerical Methods in Engineering.

4. **Baumgarte, J.** (1972). "Stabilization of Constraints and Integrals of Motion in Dynamical Systems." Computer Methods in Applied Mechanics and Engineering.

5. **Mirtich, B.** (1996). "Impulse-based Dynamic Simulation of Rigid Body Systems." PhD thesis, UC Berkeley.

### Physics Engine Implementations

- **Bullet Physics**: `btSequentialImpulseConstraintSolver` - Reference PGS implementation
- **ODE** (Open Dynamics Engine): Contact constraint handling with LCP
- **Box2D**: Position and velocity constraint solver (Erin Catto)
- **PhysX**: Multi-threaded PGS with warm starting

### Project References

- [Ticket 0031: Generalized Lagrange Constraints](../../../tickets/0031_generalized_lagrange_constraints.md)
- [Ticket 0027: Collision Response System](../../../tickets/0027_collision_response_system.md)
- [Physics/Constraints/CLAUDE.md](../../../msd/msd-sim/src/Physics/Constraints/CLAUDE.md)
- [CollisionResponse.cpp](../../../msd/msd-sim/src/Physics/CollisionResponse.cpp) - Current implementation to be replaced

---

## Appendix A: Derivation of Quaternion Rate Jacobian

For completeness, the transformation from angular velocity $\boldsymbol{\omega}$ to quaternion rate $\dot{\mathbf{Q}}$ is:

$$
\dot{\mathbf{Q}} = \frac{1}{2} \mathbf{Q} \otimes [0, \boldsymbol{\omega}]
$$

Where $\otimes$ denotes quaternion multiplication and $[0, \boldsymbol{\omega}]$ is a pure quaternion.

In matrix form:

$$
\dot{\mathbf{Q}} = \frac{1}{2} \mathbf{E}(\mathbf{Q}) \boldsymbol{\omega}
$$

Where $\mathbf{E}(\mathbf{Q})$ is the $4 \times 3$ matrix:

$$
\mathbf{E}(\mathbf{Q}) = \begin{bmatrix} -q_x & -q_y & -q_z \\ q_w & -q_z & q_y \\ q_z & q_w & -q_x \\ -q_y & q_x & q_w \end{bmatrix}
$$

And $\mathbf{Q} = [q_w, q_x, q_y, q_z]$.

The inverse relation:

$$
\boldsymbol{\omega} = 2 \mathbf{E}(\mathbf{Q})^\top \dot{\mathbf{Q}}
$$

For the Jacobian transformation:

$$
\mathbf{J}_Q = \mathbf{J}_\omega \cdot (2 \mathbf{E}(\mathbf{Q})^\top)
$$

Where $\mathbf{J}_\omega$ is the $(\text{dim} \times 3)$ Jacobian with respect to angular velocity.

---

## Appendix B: Condition Number Bounds

For the effective mass matrix $\mathbf{A} = \mathbf{J} \mathbf{M}^{-1} \mathbf{J}^\top$:

**Lower bound on minimum eigenvalue**:

$$
\sigma_{\min}(\mathbf{A}) \geq \sigma_{\min}(\mathbf{J})^2 \cdot \sigma_{\min}(\mathbf{M}^{-1})
$$

**Upper bound on maximum eigenvalue**:

$$
\sigma_{\max}(\mathbf{A}) \leq \sigma_{\max}(\mathbf{J})^2 \cdot \sigma_{\max}(\mathbf{M}^{-1})
$$

**Condition number bound**:

$$
\kappa(\mathbf{A}) \leq \kappa(\mathbf{J})^2 \cdot \kappa(\mathbf{M}^{-1})
$$

For diagonal $\mathbf{M}$, $\kappa(\mathbf{M}^{-1}) = m_{\max} / m_{\min}$.

For contact Jacobians with unit normals, $\|\mathbf{J}\| \approx \sqrt{2}$ per constraint (linear + angular components).

---

## Appendix C: Energy Analysis

### Kinetic Energy Change

For an impulse $\lambda$ applied along contact normal $\mathbf{n}$:

**Linear kinetic energy change**:

$$
\Delta \text{KE}_{\text{linear}} = \frac{1}{2} m (\mathbf{v}^+)^2 - \frac{1}{2} m (\mathbf{v}^-)^2 = \lambda \mathbf{n} \cdot \mathbf{v}_{\text{avg}}
$$

Where $\mathbf{v}_{\text{avg}} = (\mathbf{v}^+ + \mathbf{v}^-) / 2$.

**Angular kinetic energy change**:

$$
\Delta \text{KE}_{\text{angular}} = \frac{1}{2} (\boldsymbol{\omega}^+)^\top \mathbf{I} \boldsymbol{\omega}^+ - \frac{1}{2} (\boldsymbol{\omega}^-)^\top \mathbf{I} \boldsymbol{\omega}^-
$$

### Energy Conservation (Elastic Collision)

For $e = 1$:

$$
\text{KE}_{\text{total}}^+ = \text{KE}_{\text{total}}^-
$$

This requires both linear and angular contributions to balance.

### Energy Dissipation (Inelastic Collision)

**General formula**: For any collision (including angular effects), the kinetic energy change is determined by the Lagrange multiplier and effective mass:

$$
\Delta\text{KE} = -\frac{1}{2}(1 - e^2) \frac{\lambda^2}{A_{\text{eff}}}
$$

where $A_{\text{eff}} = \mathbf{J}\mathbf{M}^{-1}\mathbf{J}^\top$ is the effective mass (Section 4.4) and $\lambda$ is the constraint impulse. This formula accounts for both translational and rotational energy transfer and is valid for all contact geometries.

**Simplified case (translational motion only, no angular effects)**: For purely translational collisions with $e < 1$:

$$
\text{KE}_{\text{total}}^+ = e^2 \cdot \text{KE}_{\text{normal}}^- + \text{KE}_{\text{tangential}}^-
$$

The normal component is reduced by factor $e^2$. This approximation does not hold for general collisions with angular momentum transfer, where the energy dissipation depends on the full effective mass including rotational terms.

---

## Mathematical Formulation Review

**Date**: 2026-01-29
**Reviewer**: Architect (design-architect agent)
**Scope**: Full review of mathematical derivations, numerical examples, stability analysis, coverage, and human feedback

---

### Review Summary

The mathematical formulation is thorough and provides a solid foundation for implementing the `ContactConstraint` within the existing Lagrangian framework. The core derivations (constraint function, Jacobian structure, Lagrange multiplier system, PGS algorithm, Baumgarte stabilization) are correct. However, several issues require revision before the formulation can serve as an implementation reference.

**Overall Assessment**: REVISION REQUESTED

| Category | Status | Details |
|----------|--------|---------|
| Core Derivations (Sec 2-4) | PASS with minor issues | Sign convention inconsistency in Sec 6.2; derivations otherwise correct |
| Numerical Example 11.1 | PASS | Head-on collision verified by hand; all quantities correct |
| Numerical Example 11.2 | FAIL | Cross product sign error in $\mathbf{r}_B \times \mathbf{n}$; angular velocity of B has wrong sign |
| Numerical Example 11.3 | PASS | Resting contact verified; force balance correct |
| GTest Template 12.2 | FAIL | Inherits errors from Example 11.2; expected values incorrect |
| Stability Analysis (Sec 9) | PASS | Standard treatment; adequate for implementation |
| Coverage | INCOMPLETE | Missing multi-contact assembly example; interface extension gap |
| Human Feedback Items | NOT ADDRESSED | Four items from ticket require explicit revision |

---

### Issue I1: Sign Convention Inconsistency in Section 6.2

**Severity**: Medium
**Location**: Section 6.2, final equations

The derivation in Section 6.2 correctly arrives at:

$$
A\lambda = -(1 + e) \cdot (\mathbf{J} \dot{\mathbf{q}}^-)
$$

Then states: "For approaching contacts ($\mathbf{J} \dot{\mathbf{q}}^- > 0$): $\lambda = \frac{(1 + e) \cdot (\mathbf{J} \dot{\mathbf{q}}^-)}{A}$"

This is inconsistent. With the Jacobian convention $\mathbf{J} = [-\mathbf{n}^\top, ..., +\mathbf{n}^\top, ...]$, the constraint rate is:

$$
\dot{C} = \mathbf{J}\dot{\mathbf{q}} = -\mathbf{n}^\top \mathbf{v}_A + \mathbf{n}^\top \mathbf{v}_B + \text{angular terms} = -v_{\text{rel},n}
$$

For approaching contacts, $v_{\text{rel},n} > 0$ (using the code convention $\mathbf{v}_{\text{rel}} = \mathbf{v}_A - \mathbf{v}_B$), which means $\dot{C} = -v_{\text{rel},n} < 0$.

**Required revision**: Change to: "For approaching contacts ($\mathbf{J}\dot{\mathbf{q}}^- < 0$, equivalently $v_{\text{rel},n} > 0$):"

$$
\lambda = \frac{-(1 + e) \cdot (\mathbf{J}\dot{\mathbf{q}}^-)}{A} = \frac{(1 + e) \cdot v_{\text{rel},n}}{A} > 0
$$

This makes the sign convention explicit and matches the existing code.

---

### Issue I2: Cross Product Sign Error in Example 11.2

**Severity**: High
**Location**: Section 11.2 Step 1

The document states:

$$
\mathbf{r}_B \times \mathbf{n} = (-0.5, 0.5, 0) \times (1, 0, 0) = (0, 0, 0.5)
$$

The correct computation:

$$
\mathbf{r}_B \times \mathbf{n} = (-0.5, 0.5, 0) \times (1, 0, 0)
$$

$$
= \begin{vmatrix} \mathbf{i} & \mathbf{j} & \mathbf{k} \\ -0.5 & 0.5 & 0 \\ 1 & 0 & 0 \end{vmatrix}
= (0 - 0, 0 - 0, 0 - 0.5) = (0, 0, -0.5)
$$

The sign is negative, not positive. This error propagates to $\Delta\boldsymbol{\omega}_B$.

**Impact on effective mass**: None. The angular contribution $(\mathbf{r}_B \times \mathbf{n})^\top \mathbf{I}_B^{-1} (\mathbf{r}_B \times \mathbf{n})$ is a quadratic form, so sign cancels: $(0,0,-0.5)^\top \cdot 6.0 \cdot (0,0,-0.5) = 1.5$. All scalar quantities ($A$, $\lambda$, linear velocities) remain correct.

**Impact on angular velocity of B**: The correct angular velocity change for B is:

$$
\Delta\boldsymbol{\omega}_B = \mathbf{I}_B^{-1} (\mathbf{r}_B \times (\lambda\mathbf{n}))
$$

$$
= 6.0 \cdot ((-0.5, 0.5, 0) \times (0.4, 0, 0)) = 6.0 \cdot (0, 0, -0.2) = (0, 0, -1.2) \text{ rad/s}
$$

The document claims $(0, 0, +1.2)$ which is incorrect. Body B should spin clockwise (negative z-axis) because the impulse pushes its contact point (above center) in the $+x$ direction, creating a clockwise torque.

**Required revision**: Fix $\mathbf{r}_B \times \mathbf{n}$ to $(0, 0, -0.5)$ and $\boldsymbol{\omega}_B^+$ to $(0, 0, -1.2)$ rad/s throughout Section 11.2.

---

### Issue I3: GTest Template 12.2 Inherits Example 11.2 Errors

**Severity**: High
**Location**: Section 12.2

The GTest template inherits two problems:

1. **Lever arm inconsistency**: The test uses `leverArmB = contactPoint - stateB.position = (-1.5, 0.5, 0)` (computed from a single contact point), but the narrative example uses $\mathbf{r}_B = (-0.5, 0.5, 0)$ (computed from B's actual surface contact point). These are different physical scenarios. If the test is meant to validate Example 11.2, the lever arms should match.

2. **Wrong expected cross product sign**: The comment `// (0, 0, 0.5)` and assertion `REQUIRE(rBxn.z() == Catch::Approx(0.5))` are incorrect regardless of which lever arm is used. For $(-1.5, 0.5, 0) \times (1, 0, 0) = (0, 0, -0.5)$ and for $(-0.5, 0.5, 0) \times (1, 0, 0) = (0, 0, -0.5)$.

3. **Wrong angular velocity sign assertion**: `REQUIRE(deltaOmegaB.z() > 0)` should be `REQUIRE(deltaOmegaB.z() < 0)`.

**Required revision**: Fix all expected values in GTest 12.2 to match corrected derivation. Decide whether to use single-contact-point lever arms (code convention) or per-body surface point lever arms (physical convention) and be consistent between narrative and test.

---

### Issue I4: Human Feedback -- Section 2.2 Multiple Contact Points (MUST ADDRESS)

**Severity**: High (human feedback)
**Location**: Section 2.2

The human asks: "Is there a reason to constrain this to one contact point compared to if we have multiple contact points (e.g. two faces colliding)? Or does this generalize to multiple contact points?"

The formulation presents a single contact pair but does not explain the generalization. Section 10.3 touches on multiple simultaneous contacts but only for contacts involving different body pairs.

**Required revision**: Add a subsection to Section 2.2 (or a new Section 2.5) explicitly addressing multiple contact points for the same body pair:

- For $k$ contact points between bodies A and B, create $k$ independent scalar constraints $C_i \geq 0$, each with its own contact normal $\mathbf{n}_i$, lever arms $\mathbf{r}_{A,i}$, $\mathbf{r}_{B,i}$, and Lagrange multiplier $\lambda_i$.
- The system becomes $k$ rows in the constraint Jacobian, producing a $k \times k$ effective mass matrix with coupling between contact points through the shared mass matrix.
- This is exactly how the `CollisionResult` manifold (up to 4 `ContactPoint` entries) maps to constraint rows.
- The current code's centroid approach (averaging contacts into a single point) is an approximation that reduces $k$ constraints to 1. The constraint formulation supports both approaches; individual contact points provide better stability for resting contacts.

---

### Issue I5: Human Feedback -- Section 3.3 Derivative Intuition (MUST ADDRESS)

**Severity**: Medium (human feedback)
**Location**: Section 3.3

The human asks: "Can you elaborate on how $\partial C / \partial \mathbf{x}_A = -\mathbf{n}^\top$?"

The document states the result without explaining why $\mathbf{r}_A$ is independent of $\mathbf{x}_A$.

**Required revision**: Add an explanatory paragraph to Section 3.3:

1. **Key observation**: The lever arm $\mathbf{r}_A = \mathbf{R}(\mathbf{Q}_A) \mathbf{r}_A^{\text{body}}$ depends on orientation $\mathbf{Q}_A$, not on position $\mathbf{x}_A$. When differentiating $C$ with respect to $\mathbf{x}_A$, the lever arm terms are constant.

2. **Explicit differentiation**: $\frac{\partial C}{\partial \mathbf{x}_A} = \frac{\partial}{\partial \mathbf{x}_A}[(\mathbf{x}_B + \mathbf{r}_B - \mathbf{x}_A - \mathbf{r}_A) \cdot \mathbf{n}] = \frac{\partial}{\partial \mathbf{x}_A}[-\mathbf{x}_A \cdot \mathbf{n}] = -\mathbf{n}^\top$ since all other terms are independent of $\mathbf{x}_A$.

3. **Physical intuition**: If you translate body A's center of mass by a small displacement $\delta\mathbf{x}_A$ (without rotating), the contact point on A moves rigidly with it. The gap $C$ changes by $-\delta\mathbf{x}_A \cdot \mathbf{n}$: moving A toward B (along $\mathbf{n}$) closes the gap, hence the negative sign.

---

### Issue I6: Human Feedback -- Section 3.4 Angular Velocity vs Position (MUST ADDRESS)

**Severity**: Medium (human feedback)
**Location**: Section 3.4

The human asks: "It's unclear to me where $\partial\omega$ comes from, considering we're using the angular positions, not rates."

The document jumps from $\frac{\partial \mathbf{r}_A}{\partial \mathbf{Q}_A}$ directly to $\frac{\partial C}{\partial \boldsymbol{\omega}_A}$ without explaining the conceptual shift from position-level to velocity-level Jacobians.

**Required revision**: Add a motivating paragraph at the beginning of Section 3.4 explaining:

1. **Why velocity-level**: For constrained dynamics, we need the velocity-level Jacobian that maps generalized velocities to constraint rates: $\dot{C} = \mathbf{J}_v \dot{\mathbf{q}}_v$ where $\dot{\mathbf{q}}_v = [\mathbf{v}, \boldsymbol{\omega}]$. This is distinct from the position-level Jacobian $\partial C / \partial \mathbf{q}_p$ where $\mathbf{q}_p = [\mathbf{x}, \mathbf{Q}]$.

2. **How $\boldsymbol{\omega}$ enters**: The lever arm $\mathbf{r}_A = \mathbf{R}(\mathbf{Q}_A) \mathbf{r}_A^{\text{body}}$ depends on orientation. Its time derivative is $\dot{\mathbf{r}}_A = \boldsymbol{\omega}_A \times \mathbf{r}_A$ (from rigid body kinematics). By the chain rule: $\dot{C} = \frac{\partial C}{\partial \mathbf{x}_A} \cdot \mathbf{v}_A + \frac{\partial C}{\partial \mathbf{r}_A} \cdot \dot{\mathbf{r}}_A + ...$ Substituting $\dot{\mathbf{r}}_A = \boldsymbol{\omega}_A \times \mathbf{r}_A$ naturally introduces $\boldsymbol{\omega}_A$ into the expression. We then collect terms to identify the "effective" Jacobian with respect to $\boldsymbol{\omega}_A$.

3. **Connection to quaternion Jacobian**: The velocity-level Jacobian in $\boldsymbol{\omega}$-space relates to the position-level quaternion Jacobian via the $\mathbf{E}(\mathbf{Q})$ matrix (as shown in Section 3.5). Working in $\boldsymbol{\omega}$-space first is standard practice because it avoids the redundancy of 4 quaternion components for 3 rotational DOF.

---

### Issue I7: Human Feedback -- AssetEnvironment Mass Property (MUST ADDRESS)

**Severity**: Medium (human feedback / architectural)
**Location**: Section 10.4

The human notes: "I suspect we'll want to slightly modify the AssetEnvironment such that it has a mass that can drive the inverse mass to zero."

This is an architectural insight that impacts the design document (not just the math formulation). The current `AssetEnvironment` has no mass, and the code handles static collisions with separate functions (`computeLagrangeMultiplierStatic`, `applyConstraintResponseStatic`). This creates the dual-path architecture that Ticket 0032 aims to eliminate.

**Required addition to Section 10.4**: Add a design note:

> **Implementation Note**: To enable a unified constraint solver code path, `AssetEnvironment` should be extended with a mass property set to a very large value (e.g., `std::numeric_limits<double>::max()`) or carry an explicit `inverseMass = 0.0` and `inverseInertia = Matrix3d::Zero()`. This allows the constraint solver to treat environment objects identically to inertial objects without special-case branching. The effective mass formula (Section 4.4) naturally handles this: when $m_B^{-1} = 0$ and $\mathbf{I}_B^{-1} = \mathbf{0}$, body B's contribution vanishes.
>
> This change is architectural and should be captured in the design document. Options:
> - **Option A**: Add `inverseMass_` and `inverseInertia_` members to `AssetEnvironment` (both zero)
> - **Option B**: Create a common `MassProperties` struct used by both `AssetInertial` and `AssetEnvironment`, with a static factory for infinite mass
> - **Recommendation**: Option A (minimal change, avoids new abstractions)

---

### Issue I8: Missing Multi-Contact Assembly Example

**Severity**: Low
**Location**: Coverage gap

The formulation describes multi-contact coupling in Section 10.3 but does not provide a worked numerical example showing how to assemble the system matrix when multiple contacts share a body. A brief example (e.g., an object resting on two supports with two contact points) would strengthen the formulation.

Section 12.6 provides a GTest for this case, but the GTest uses a highly simplified model (ignoring angular terms, adding artificial regularization) that doesn't match the full formulation. The effective mass matrix for two contacts sharing a body is:

$$
A_{ij} = \mathbf{J}_i \mathbf{M}^{-1} \mathbf{J}_j^\top
$$

For two contacts with the same normal $\mathbf{n}$ on the same body, the off-diagonal terms are non-zero and equal to the diagonal terms (minus angular contributions), which makes the matrix singular without angular contributions. The GTest handles this with regularization ($\epsilon = 10^{-6}$), which is a valid approach but should be explicitly motivated in the formulation.

**Required revision**: Either add a brief worked example in Section 10.3, or add a note in Section 12.6 explaining why regularization is needed for this test case and how the angular terms would resolve singularity in practice.

---

### Issue I9: Appendix C Energy Formula Imprecision

**Severity**: Low
**Location**: Appendix C, final equation

The formula:

$$
\text{KE}_{\text{total}}^+ = e^2 \cdot \text{KE}_{\text{normal}}^- + \text{KE}_{\text{tangential}}^-
$$

is an approximation valid only for center-of-mass collisions without angular effects. For general collisions with angular momentum transfer, the energy dissipation depends on the full effective mass (including rotational terms) and the formula does not hold exactly.

**Required revision**: Add a qualifier: "For the simplified case of purely translational motion (no angular effects):" before the equation, or derive the general energy change formula in terms of the Lagrange multiplier: $\Delta\text{KE} = -\frac{1}{2}(1 - e^2) \lambda^2 / A_{\text{eff}}$ which is valid for all cases.

---

### Issue I10: Existing Constraint Interface Compatibility

**Severity**: Medium (coverage gap)
**Location**: Not addressed in formulation

The existing `Constraint::evaluate()` and `Constraint::jacobian()` take a single `InertialState`. The contact constraint formulation requires two body states. The formulation derives the two-body Jacobian (Section 3.2) but does not discuss how to reconcile this with the existing single-body interface.

This is primarily a design document concern rather than a math formulation concern, but the formulation should acknowledge the interface gap. Section 4.1 and the ticket mention "Option C: Constraint References Bodies by ID" but the math formulation does not show how the stacked state vector or body-index approach maps to the Jacobian structure derived here.

**Required revision**: Add a brief note in Section 3.6 or Section 4 explaining the interface mapping: the $(1 \times 12)$ Jacobian decomposes into two $(1 \times 6)$ blocks, one per body. The constraint implementation stores body references/indices and assembles the full Jacobian by combining per-body blocks. The solver must be extended to handle multi-body state vectors.

---

### Items Passing Review (No Changes Needed)

| Section | Assessment |
|---------|------------|
| Section 1 (Introduction) | Clear motivation and notation table |
| Section 2.1 (Position-Level Constraint) | Correct formulation with proper interpretation |
| Section 2.3-2.4 (Penetration Depth) | Consistent with Section 2.1 |
| Section 3.1-3.2 (Jacobian Structure) | Correct two-body structure |
| Section 3.5-3.6 (Complete Jacobian) | Correct assembly and force extraction |
| Section 4.1-4.5 (Lagrange System) | Correct derivation; effective mass matches code |
| Section 5 (Complementarity) | Standard LCP formulation |
| Section 7 (PGS Solver) | Correct algorithm with convergence analysis |
| Section 8 (Baumgarte) | Correct stabilization; practical parameter values provided |
| Section 9 (Stability) | Adequate treatment for implementation |
| Section 10.1-10.2, 10.5-10.6 (Special Cases) | Correct handling |
| Section 11.1 (Example 1) | Verified correct by hand |
| Section 11.3 (Example 3) | Verified correct by hand |
| Section 12.1, 12.3-12.5 (GTests) | Correct expected values |
| Appendix A (Quaternion Rate) | Correct $\mathbf{E}(\mathbf{Q})$ matrix |
| Appendix B (Condition Bounds) | Standard eigenvalue bounds |

---

### Revision Priority

| Priority | Issue | Impact |
|----------|-------|--------|
| **HIGH** | I2: Example 11.2 cross product sign error | Incorrect reference values for implementation |
| **HIGH** | I3: GTest 12.2 wrong expected values | Tests would fail if implemented as-is |
| **HIGH** | I4: Multiple contact points (human feedback) | Missing generalization needed for implementation |
| **MEDIUM** | I1: Section 6.2 sign convention | Confusing but numerical examples are correct |
| **MEDIUM** | I5: Section 3.3 intuition (human feedback) | Clarity for implementer |
| **MEDIUM** | I6: Section 3.4 omega motivation (human feedback) | Clarity for implementer |
| **MEDIUM** | I7: AssetEnvironment mass (human feedback) | Architectural implication |
| **MEDIUM** | I10: Interface compatibility gap | Design-math bridge |
| **LOW** | I8: Multi-contact assembly example | Completeness |
| **LOW** | I9: Appendix C energy formula | Precision |

---

### Recommendations for Next Steps

1. **Fix Examples**: Correct the sign errors in Example 11.2 and GTest 12.2 before any implementation begins.
2. **Address Human Feedback**: Add explanatory paragraphs for Sections 2.2, 3.3, 3.4, and 10.4 as specified in Issues I4-I7.
3. **Bridge to Design**: Add interface mapping notes (Issue I10) to connect the mathematical formulation to the existing codebase.
4. **Proceed to Design Document**: After revisions, the corrected formulation provides sufficient mathematical foundation for the `ContactConstraint` design document.

---

## Architect Revision Notes

**Date**: 2026-01-29
**Responding to**: Math Review -- Initial Assessment

### Changes Made

| Issue ID | Severity | Section | Original | Revised | Rationale |
|----------|----------|---------|----------|---------|-----------|
| I1 | MEDIUM | 6.2 | Condition stated as $\mathbf{J}\dot{\mathbf{q}}^- > 0$ for approaching contacts | Changed to $\mathbf{J}\dot{\mathbf{q}}^- < 0$ with explicit equivalence $\dot{C} = -v_{\text{rel},n}$; lambda formula shows negation yielding positive result | Aligns sign convention with Jacobian structure $[-\mathbf{n}^\top, ..., +\mathbf{n}^\top, ...]$ where approaching contacts have negative constraint rate |
| I2 | HIGH | 11.2 | $\mathbf{r}_B \times \mathbf{n} = (0, 0, 0.5)$; $\Delta\boldsymbol{\omega}_B = (0, 0, 1.2)$; $\boldsymbol{\omega}_B^+ = (0, 0, 1.2)$ | $\mathbf{r}_B \times \mathbf{n} = (0, 0, -0.5)$; $\Delta\boldsymbol{\omega}_B = (0, 0, -1.2)$; $\boldsymbol{\omega}_B^+ = (0, 0, -1.2)$; added determinant expansion and physical intuition | Correct cross product: $(-0.5)(0) - (0.5)(1) = -0.5$ for z-component. Body B spins clockwise (negative z) because impulse acts above its center of mass. Effective mass unchanged (quadratic form). |
| I3 | HIGH | 12.2 | `leverArmB = contactPoint - stateB.position` = $(-1.5, 0.5, 0)$; `rBxn.z()` expected $0.5$; `deltaOmegaB.z() > 0` | `leverArmB = {-0.5, 0.5, 0}` (per-body surface point); `rBxn.z()` expected $-0.5$; `deltaOmegaB.z() < 0` and $\approx -1.2$; added explicit numeric checks | Lever arm now matches Example 11.2 narrative. Added comments explaining per-body surface point convention. |
| I4 | HIGH | 2.5 (new) | No discussion of multiple contact points for same body pair | Added Section 2.5: $k$ contacts become $k$ independent constraint rows with $k \times k$ effective mass matrix; explains coupling through shared mass matrix; relates to `CollisionResult` manifold | Essential generalization for face-face collisions; explains how centroid approximation relates to full formulation |
| I5 | MEDIUM | 3.3 | Derivatives stated without explanation | Added key observation (lever arms depend on orientation not position), explicit term-by-term differentiation, and physical intuition (moving A toward B closes gap) | Addresses human question about why $\partial C / \partial \mathbf{x}_A = -\mathbf{n}^\top$ |
| I6 | MEDIUM | 3.4 | Jumped from $\partial\mathbf{r}/\partial\mathbf{Q}$ to $\partial C / \partial\boldsymbol{\omega}$ without motivation | Added motivating paragraphs: why velocity-level Jacobian, how $\boldsymbol{\omega}$ enters via $\dot{\mathbf{r}} = \boldsymbol{\omega} \times \mathbf{r}$, chain rule expansion of $\dot{C}$, connection to quaternion Jacobian | Addresses human question about where $\partial\omega$ comes from |
| I7 | MEDIUM | 10.4 | Mentioned $\mathbf{M}_{\text{static}}^{-1} = \mathbf{0}$ without implementation guidance | Added implementation note: `AssetEnvironment` needs `inverseMass_ = 0.0` and `inverseInertia_ = Zero` for unified solver path; presented Option A (add members) and Option B (common struct); recommended Option A | Bridges mathematical formulation to architectural design; eliminates dual static/dynamic code paths |
| I8 | LOW | 10.3, 12.6 | Missing cross-reference to PGS solver; no explicit coupling formula; no regularization motivation in GTest | Added explicit expansion of $A_{ij}$ coupling term; added cross-reference to Section 7 (PGS); added note before GTest 12.6 explaining regularization need and connection to Section 9.5 | Completes the multi-contact discussion with actionable formulas |
| I9 | LOW | Appendix C | Energy dissipation formula presented without qualifier | Added general formula $\Delta\text{KE} = -\frac{1}{2}(1-e^2)\lambda^2/A_{\text{eff}}$ valid for all cases; qualified original formula as translational-only approximation | General formula valid for angular effects; simplified formula now clearly scoped |
| I10 | MEDIUM | 3.6 | No mention of existing single-body `Constraint::jacobian()` interface incompatibility | Added interface note: two-body Jacobian decomposes into per-body blocks; `ContactConstraint` needs interface extension or `TwoBodyConstraint` subclass; solver must handle multi-body state vectors | Bridges mathematical formulation to existing codebase; flags design document requirement |

### Diagram Updates
- No changes to PlantUML diagram (issues were limited to mathematical text, numerical examples, and explanatory content).

### Verification of Corrected Numerical Examples

**Example 11.2 re-verification (corrected values)**:

Step 1 cross products:
- $\mathbf{r}_A \times \mathbf{n} = (0.5, 0.5, 0) \times (1, 0, 0) = (0, 0, -0.5)$ -- unchanged, was already correct
- $\mathbf{r}_B \times \mathbf{n} = (-0.5, 0.5, 0) \times (1, 0, 0) = (0, 0, -0.5)$ -- CORRECTED from $(0, 0, 0.5)$

Step 2 angular contributions (quadratic forms, sign-independent):
- $\text{angular}_A = (0, 0, -0.5) \cdot 3.0 \cdot (0, 0, -0.5) = 0.75$ -- unchanged
- $\text{angular}_B = (0, 0, -0.5) \cdot 6.0 \cdot (0, 0, -0.5) = 1.5$ -- unchanged (sign cancels)

Steps 3-4 ($A = 3.75$, $\lambda = 0.4$, linear velocities) -- unchanged

Step 5 angular velocities:
- $\Delta\boldsymbol{\omega}_A = -3.0 \cdot (0.5, 0.5, 0) \times (0.4, 0, 0) = -3.0 \cdot (0, 0, -0.2) = (0, 0, 0.6)$ -- unchanged
- $\Delta\boldsymbol{\omega}_B = +6.0 \cdot (-0.5, 0.5, 0) \times (0.4, 0, 0) = +6.0 \cdot (0, 0, -0.2) = (0, 0, -1.2)$ -- CORRECTED from $(0, 0, +1.2)$

Angular momentum conservation check (about z-axis):
- Before: $L_z = I_{A,zz} \cdot \omega_{A,z} + I_{B,zz} \cdot \omega_{B,z} + m_A (x_A v_{A,y} - y_A v_{A,x}) + m_B (x_B v_{B,y} - y_B v_{B,x}) = 0$ (all angular velocities zero, all y-velocities zero)
- After: $L_z^{\text{angular}} = 0.333 \cdot 0.6 + 0.167 \cdot (-1.2) = 0.2 - 0.2 = 0$. The angular momentum about the z-axis is conserved, confirming the corrected signs are consistent.

### Unchanged (Per Reviewer Guidance)

The following sections passed review and were not modified:
- Section 1 (Introduction): Clear motivation and notation table
- Section 2.1 (Position-Level Constraint): Correct formulation
- Sections 2.3-2.4 (Penetration Depth): Consistent with Section 2.1
- Sections 3.1-3.2 (Jacobian Structure): Correct two-body structure
- Section 3.5 (Complete Jacobian): Correct assembly
- Sections 4.1-4.5 (Lagrange Multiplier System): Correct derivation
- Section 5 (Complementarity Conditions): Standard LCP formulation
- Section 7 (PGS Solver): Correct algorithm
- Section 8 (Baumgarte Stabilization): Correct with practical values
- Section 9 (Numerical Stability): Adequate treatment
- Sections 10.1-10.2, 10.5-10.6 (Special Cases): Correct handling
- Section 11.1 (Example 1: Head-On Collision): Verified correct
- Section 11.3 (Example 3: Resting Contact): Verified correct
- Sections 12.1, 12.3-12.5 (GTest Templates): Correct expected values
- Appendix A (Quaternion Rate Jacobian): Correct
- Appendix B (Condition Number Bounds): Standard bounds

---

## Math Review -- Final Assessment

**Date**: 2026-01-29
**Reviewer**: Architect (design-architect agent)
**Scope**: Final review pass verifying all 10 issues (I1--I10) from the initial assessment have been properly addressed
**Iteration**: 1 of 1

---

### Overall Assessment: APPROVED WITH NOTES

The mathematician has addressed all 10 issues from the initial review. The revised formulation is mathematically correct and ready to serve as the implementation reference for the `ContactConstraint` class. All human feedback from the ticket has been incorporated. One minor notational inconsistency remains (see Note N1 below) which does not affect the correctness of any implementation-facing formula.

---

### Issue Resolution Verification

| Issue | Severity | Status | Verification |
|-------|----------|--------|-------------|
| I1: Section 6.2 sign convention | MEDIUM | RESOLVED | Lines 559--563 now correctly state approaching contacts have $\mathbf{J}\dot{\mathbf{q}}^- < 0$ with explicit equivalence to $v_{\text{rel},n} > 0$. Lambda formula shows negation yielding positive result. Implementation formulas are correct. |
| I2: Example 11.2 cross product | HIGH | RESOLVED | $\mathbf{r}_B \times \mathbf{n}$ corrected to $(0, 0, -0.5)$ with full determinant expansion. $\Delta\boldsymbol{\omega}_B$ corrected to $(0, 0, -1.2)$. Physical intuition added. Re-verified by hand (see below). |
| I3: GTest 12.2 errors | HIGH | RESOLVED | Lever arm corrected to $(-0.5, 0.5, 0)$ matching Example 11.2. Cross product assertion corrected to $-0.5$. Angular velocity assertion corrected to negative $z$. All expected values verified against Example 11.2. |
| I4: Multiple contact points | HIGH | RESOLVED | New Section 2.5 correctly generalizes to $k$ contact points with $k \times 12$ Jacobian, $k \times k$ effective mass matrix, and coupling through shared mass matrix. References Section 10.3 and connects to existing `CollisionResult` manifold. |
| I5: Section 3.3 intuition | MEDIUM | RESOLVED | Added key observation (lever arms depend on orientation not position), explicit term-by-term differentiation, and physical intuition (moving A toward B closes gap). Mathematically accurate. |
| I6: Section 3.4 omega motivation | MEDIUM | RESOLVED | Added clear explanation of velocity-level vs position-level Jacobians, how $\boldsymbol{\omega}$ enters via $\dot{\mathbf{r}} = \boldsymbol{\omega} \times \mathbf{r}$, chain rule expansion, and connection to quaternion Jacobian. Mathematically accurate. |
| I7: AssetEnvironment mass | MEDIUM | RESOLVED | Implementation note in Section 10.4 specifies `inverseMass_ = 0.0` and `inverseInertia_ = Zero`. Correctly explains that body B's contribution to effective mass vanishes and body B receives zero velocity change. Presents two design options with recommendation. |
| I8: Multi-contact assembly | LOW | RESOLVED | Section 10.3 now includes explicit $A_{ij}$ coupling formula. GTest 12.6 has explanatory note about regularization need and connection to Section 9.5. Cross-reference to PGS solver added. |
| I9: Appendix C energy formula | LOW | RESOLVED | General formula $\Delta\text{KE} = -\frac{1}{2}(1-e^2)\lambda^2/A_{\text{eff}}$ added as the primary result. Original formula clearly scoped as translational-only approximation. |
| I10: Interface compatibility | MEDIUM | RESOLVED | Section 3.6 interface note explains $(1 \times 12)$ decomposition into per-body $(1 \times 6)$ blocks, need for `TwoBodyConstraint` subclass or interface extension, and solver extension requirement. |

---

### Hand Verification of Corrected Example 11.2

**Setup**: Cube A ($m=2$ kg) at origin with $\mathbf{v}_A = (1,0,0)$, Cube B ($m=1$ kg) at $(2,0,0)$ stationary. Contact at edge with $\mathbf{n} = (1,0,0)$, $\mathbf{r}_A = (0.5, 0.5, 0)$, $\mathbf{r}_B = (-0.5, 0.5, 0)$, $e = 0.5$.

**Cross products** (determinant expansion):

$$\mathbf{r}_A \times \mathbf{n} = (0.5, 0.5, 0) \times (1, 0, 0) = (0 \cdot 0 - 0 \cdot 0,\; 0 \cdot 1 - 0.5 \cdot 0,\; 0.5 \cdot 0 - 0.5 \cdot 1) = (0, 0, -0.5)$$

$$\mathbf{r}_B \times \mathbf{n} = (-0.5, 0.5, 0) \times (1, 0, 0) = (0.5 \cdot 0 - 0 \cdot 0,\; 0 \cdot 1 - (-0.5) \cdot 0,\; (-0.5) \cdot 0 - 0.5 \cdot 1) = (0, 0, -0.5)$$

Both yield $(0, 0, -0.5)$. Matches document.

**Angular contributions** (quadratic forms):

$$\text{angular}_A = (0, 0, -0.5) \cdot 3.0 \cdot (0, 0, -0.5) = 0.75$$
$$\text{angular}_B = (0, 0, -0.5) \cdot 6.0 \cdot (0, 0, -0.5) = 1.5$$

**Effective mass**: $A = 0.5 + 1.0 + 0.75 + 1.5 = 3.75$. **Lambda**: $\lambda = 1.5/3.75 = 0.4$.

**Linear velocities**: $\mathbf{v}_A^+ = (0.8, 0, 0)$, $\mathbf{v}_B^+ = (0.4, 0, 0)$.

**Angular velocities**:

$$\Delta\boldsymbol{\omega}_A = -3.0 \cdot ((0.5, 0.5, 0) \times (0.4, 0, 0)) = -3.0 \cdot (0, 0, -0.2) = (0, 0, 0.6)$$
$$\Delta\boldsymbol{\omega}_B = +6.0 \cdot ((-0.5, 0.5, 0) \times (0.4, 0, 0)) = +6.0 \cdot (0, 0, -0.2) = (0, 0, -1.2)$$

**Conservation checks**:
- Linear momentum: $2 \cdot 1 + 1 \cdot 0 = 2$ before; $2 \cdot 0.8 + 1 \cdot 0.4 = 2$ after. Conserved.
- Angular momentum (z-axis about origin): $I_{A,zz} \cdot \omega_{A,z} + I_{B,zz} \cdot \omega_{B,z} = \frac{1}{3}(0.6) + \frac{1}{6}(-1.2) = 0.2 - 0.2 = 0$ after. Was $0$ before. Conserved.
- Physical sense: Body B's contact point is above its center of mass. Impulse in $+x$ creates clockwise torque $\Rightarrow$ negative $\omega_z$. Correct.

**All values verified. Example 11.2 is correct.**

---

### Human Feedback Verification

All four feedback items from the ticket `Human Feedback > Feedback on Design` section have been addressed:

| Feedback Item | Section | Resolution |
|---------------|---------|------------|
| Multiple contact points generalization (Section 2.2) | New Section 2.5 | $k$ contacts as independent constraint rows with full coupling analysis |
| Elaborate on $\partial C / \partial \mathbf{x}_A = -\mathbf{n}^\top$ (Section 3.3) | Expanded Section 3.3 | Key observation + explicit differentiation + physical intuition |
| Where does $\partial\omega$ come from (Section 3.4) | Expanded Section 3.4 | Velocity-level vs position-level distinction + chain rule + rigid body kinematics |
| AssetEnvironment with inverse mass zero (Section 10.4) | Expanded Section 10.4 | Implementation note with `inverseMass_ = 0.0` and design options A/B |

---

### Notes for Implementation

**N1 (Minor): Section 6.2 preamble sign convention**

Lines 541--542 state:

> $b_{\text{target}} = -e \cdot \dot{C}^-$ (when $\dot{C}^- > 0$, approaching)

Under the Jacobian convention established in Section 3.6, approaching contacts have $\dot{C}^- = -v_{\text{rel},n} < 0$, not $> 0$. The condition should read "when $\dot{C}^- < 0$, approaching" to be consistent with the Jacobian sign convention.

However, this only affects the descriptive text at lines 541--542. All implementation-facing formulas (lines 549--563) correctly handle the sign, explicitly noting "$\mathbf{J}\dot{\mathbf{q}}^- < 0$, equivalently $v_{\text{rel},n} > 0$." The numerical examples (11.1, 11.2) also use the correct signs.

**Recommendation**: Fix on next edit pass for editorial consistency, but this does not block implementation.

**N2 (Informational): GTest 12.6 regularization approach**

The regularization in GTest 12.6 ($\epsilon = 10^{-6}$) is appropriate for the simplified test case but the note correctly warns that full simulations with angular terms would not need it. The implementer should be aware that the regularization strategy in Section 9.5 ($\epsilon = 10^{-7}$ to $10^{-10}$) may need tuning based on the condition number analysis in Appendix B.

---

### Conclusion

The revised mathematical formulation is **approved** for use as the implementation reference. All 10 issues have been properly resolved, all human feedback has been incorporated, and the corrected numerical examples (verified by hand) provide reliable reference values for test implementation. The single remaining notational inconsistency (N1) is purely editorial and does not affect any formula that would be used in code.

**Status**: APPROVED WITH NOTES

**Next Step**: Proceed to architectural design document creation using this formulation as the mathematical foundation.
