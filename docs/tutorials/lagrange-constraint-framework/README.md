# Lagrange Multiplier Constraint Framework

## Overview

This tutorial explains how to implement constraint-based physics using **Lagrange multipliers** with **Baumgarte stabilization**. We'll build a constraint solver from first principles that can enforce arbitrary constraints (quaternion normalization, fixed distances, joints) in rigid body simulations.

## Prerequisites

- **Linear algebra**: Matrix operations, eigenvalues, Cholesky decomposition
- **Calculus**: Partial derivatives, gradients, Jacobians
- **Physics**: Newton's laws, rigid body dynamics basics
- **C++**: Templates, virtual functions, Eigen library familiarity

## The Problem

Consider a rigid body simulation where we need to enforce certain relationships:

1. **Quaternion normalization**: Unit quaternions represent rotation, so `|Q| = 1` must hold
2. **Fixed distance**: An object tethered to a point at distance `d`
3. **Joints**: Two objects connected at a pivot point

Naive approaches (like rescaling after each timestep) introduce energy drift and instability. **Lagrange multipliers** provide a mathematically rigorous solution that:

- Computes exact constraint forces
- Preserves energy (symplectic integration)
- Handles multiple constraints simultaneously
- Extends naturally to new constraint types

## Mathematical Foundation

### Constraint Formulation

A **holonomic constraint** is expressed as:

$$C(q, t) = 0$$

where $q$ is the generalized coordinate vector (position, orientation) and $t$ is time.

For a rigid body with position $X \in \mathbb{R}^3$ and quaternion orientation $Q \in \mathbb{R}^4$, the state vector is:

$$q = [X_x, X_y, X_z, Q_w, Q_x, Q_y, Q_z]^T \in \mathbb{R}^7$$

### Constraint Examples

**Unit Quaternion Constraint:**
$$C(Q) = Q^T Q - 1 = Q_w^2 + Q_x^2 + Q_y^2 + Q_z^2 - 1 = 0$$

**Distance from Origin:**
$$C(X) = |X|^2 - d^2 = X_x^2 + X_y^2 + X_z^2 - d^2 = 0$$

### The Constraint Jacobian

The **Jacobian** $J$ captures how the constraint changes with configuration:

$$J = \frac{\partial C}{\partial q}$$

For a scalar constraint on a 7-DOF system, $J$ is a $1 \times 7$ row vector.

**Unit Quaternion Jacobian:**
$$J = [0, 0, 0, 2Q_w, 2Q_x, 2Q_y, 2Q_z]$$

The first three components are zero because the quaternion constraint doesn't depend on position.

**Distance Constraint Jacobian:**
$$J = [2X_x, 2X_y, 2X_z, 0, 0, 0, 0]$$

The quaternion components are zero because distance doesn't depend on orientation.

### Lagrange Multiplier Formulation

We want to find a **constraint force** $F_c = J^T \lambda$ that keeps the system on the constraint manifold without doing work in the unconstrained directions.

The Lagrange multiplier $\lambda$ satisfies:

$$J M^{-1} J^T \lambda = b$$

where:
- $M^{-1}$ is the inverse mass matrix (block diagonal: $[\frac{1}{m}I_3, I^{-1}]$)
- $b$ is the right-hand side including external forces and stabilization terms

### Baumgarte Stabilization

Numerical integration causes constraint drift over time. **Baumgarte stabilization** adds feedback terms:

$$b = -J M^{-1} F_{ext} - \dot{J}\dot{q} - \alpha C - \beta \dot{C}$$

where:
- $\alpha$ (position gain): Corrects position-level constraint violation
- $\beta$ (velocity gain): Corrects velocity-level constraint violation
- Typical values: $\alpha = \beta = 10.0$ (for timesteps around 16ms)

The term $-\alpha C$ acts like a spring pulling the system back to the constraint surface.
The term $-\beta \dot{C}$ acts like a damper preventing oscillation.

### Complete Algorithm

1. **Assemble Jacobian** $J$ by stacking all constraint Jacobians
2. **Build mass matrix** $M^{-1}$ (block diagonal)
3. **Form constraint matrix** $A = J M^{-1} J^T$
4. **Compute RHS** $b = -J M^{-1} F_{ext} - \alpha C - \beta \dot{C}$
5. **Solve** $A \lambda = b$ using Cholesky decomposition (LLT)
6. **Extract forces** $F_c = J^T \lambda$
7. **Apply** constraint forces alongside external forces in integration

## Implementation Notes

### Production vs Tutorial Code

| Production (`msd-sim`) | Tutorial (`example.cpp`) |
|------------------------|--------------------------|
| Eigen matrices with templates | Simple dense matrices |
| Multiple inheritance (Bilateral/Unilateral) | Single Constraint class |
| `std::unique_ptr` ownership | Value semantics |
| AssetInertial integration | Standalone state struct |
| Thread-safe const methods | Single-threaded |

### Why Direct Solve (LLT)?

For small constraint counts ($n < 100$), direct factorization is optimal:
- $O(n^3)$ complexity is acceptable for $n < 10$
- Guarantees exact solution (up to numerical precision)
- Avoids iteration tuning

For large constraint counts (contact-heavy scenes), iterative methods like **Projected Gauss-Seidel** become necessary.

### State Vector Layout

The production code uses a 7-DOF position state:

```
q = [X_x, X_y, X_z, Q_w, Q_x, Q_y, Q_z]
    ├──────────────┤ ├────────────────┤
        Position        Quaternion
```

Jacobians are always $\text{dim} \times 7$ matrices where:
- Columns 0-2: Position derivatives
- Columns 3-6: Quaternion derivatives

### Condition Number Monitoring

The condition number of $A = J M^{-1} J^T$ indicates solver health:
- $\kappa < 100$: Well-conditioned (typical)
- $\kappa > 10^6$: Nearly singular (constraint redundancy)
- $\kappa = \infty$: Singular (degenerate constraints)

The production code reports condition number in `SolveResult` for diagnostics.

## Algorithm Walkthrough

### Step 1: Define Constraint Interface

Every constraint must provide:

```cpp
class Constraint {
public:
    // Number of scalar equations
    virtual int dimension() const = 0;

    // Evaluate C(q, t)
    virtual VectorXd evaluate(const State& state, double time) const = 0;

    // Compute J = dC/dq
    virtual MatrixXd jacobian(const State& state, double time) const = 0;

    // Baumgarte gains
    virtual double alpha() const { return 10.0; }
    virtual double beta() const { return 10.0; }
};
```

### Step 2: Implement Specific Constraints

**Unit Quaternion:**
```cpp
VectorXd evaluate(const State& state, double time) const override {
    const Quaterniond& Q = state.orientation;
    VectorXd C(1);
    C(0) = Q.squaredNorm() - 1.0;
    return C;
}

MatrixXd jacobian(const State& state, double time) const override {
    const Quaterniond& Q = state.orientation;
    MatrixXd J(1, 7);
    J.setZero();
    J(0, 3) = 2.0 * Q.w();
    J(0, 4) = 2.0 * Q.x();
    J(0, 5) = 2.0 * Q.y();
    J(0, 6) = 2.0 * Q.z();
    return J;
}
```

### Step 3: Solve for Lagrange Multipliers

```cpp
SolveResult solve(const vector<Constraint*>& constraints,
                  const State& state,
                  const Vector3d& externalForce,
                  double mass,
                  const Matrix3d& inverseInertia,
                  double dt) {
    // Assemble stacked Jacobian
    MatrixXd J = assembleJacobian(constraints, state);

    // Build inverse mass matrix
    MatrixXd M_inv = buildMassInverse(mass, inverseInertia);

    // Form A = J * M_inv * J^T
    MatrixXd A = J * M_inv * J.transpose();

    // Compute RHS with Baumgarte stabilization
    VectorXd b = computeRHS(constraints, state, externalForce, M_inv, J);

    // Solve A * lambda = b
    LLT<MatrixXd> llt(A);
    VectorXd lambda = llt.solve(b);

    // Extract constraint forces
    VectorXd F_c = J.transpose() * lambda;

    return {lambda, F_c, llt.info() == Success};
}
```

### Step 4: Integrate with Physics Loop

```cpp
void physicsStep(State& state, double dt) {
    // 1. Compute external forces
    Vector3d F_ext = computeGravity(state, mass);

    // 2. Solve constraints
    auto result = solver.solve(constraints, state, F_ext, mass, I_inv, dt);

    // 3. Combine forces
    Vector3d F_total = F_ext + result.linearForce;
    Vector3d tau_total = externalTorque + result.angularTorque;

    // 4. Semi-implicit Euler integration
    state.velocity += (F_total / mass) * dt;
    state.position += state.velocity * dt;

    // 5. Quaternion integration
    state.quaternionRate = updateQuaternionRate(state, tau_total, I_inv);
    integrateQuaternion(state, dt);
}
```

## References

- Baraff, D. (1996). "Linear-Time Dynamics using Lagrange Multipliers"
- Cline, M. B. (2002). "Rigid Body Simulation with Contact and Constraints"
- Baumgarte, J. (1972). "Stabilization of constraints and integrals of motion in dynamical systems"
- Goldstein, H. "Classical Mechanics" (Chapter 2: Lagrangian Mechanics)

## See Also

- `msd-sim/src/Physics/Constraints/Constraint.hpp` — Abstract constraint interface
- `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` — Production solver
- `msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.hpp` — Quaternion constraint
- `msd-sim/src/Physics/Constraints/DistanceConstraint.hpp` — Distance constraint
- `docs/designs/0031_generalized_lagrange_constraints/design.md` — Design document
