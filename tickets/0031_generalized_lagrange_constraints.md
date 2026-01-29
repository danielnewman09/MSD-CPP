# Ticket 0031: Generalized Lagrange Multiplier Constraint System

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Documentation Complete — Awaiting Tutorial
- [x] Tutorial Complete — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Tutorial Complete — Ready to Merge
**Assignee**: Workflow Orchestrator
**Created**: 2026-01-28
**Generate Tutorial**: Yes

---

## Summary

Develop a generalized constraint framework using Lagrange multipliers that allows users to define arbitrary constraints by providing constraint function data. The system should enable building a constraint library where each constraint type provides the mathematical information (constraint function, Jacobian, etc.) and the solver handles enforcement uniformly.

---

## Motivation

The current `QuaternionConstraint` (Ticket 0030) successfully demonstrates Lagrange multiplier constraint enforcement with Baumgarte stabilization. However, it is a specialized, hard-coded implementation. As the physics engine grows, we need:

1. **Extensibility**: Easy addition of new constraint types (joints, contacts, limits)
2. **Reusability**: Common solver infrastructure shared across constraints
3. **Composability**: Multiple constraints on the same body working together
4. **Maintainability**: Single constraint solving algorithm, multiple constraint definitions

### Use Cases
- **Joint constraints**: Revolute, prismatic, spherical, universal joints
- **Contact constraints**: Non-penetration, friction (from collision response)
- **Limit constraints**: Position limits, velocity limits, joint angle limits
- **Holonomic constraints**: Fixed distance, surface constraints
- **Non-holonomic constraints**: Rolling without slipping, knife-edge

---

## Technical Approach

### Constraint Mathematical Framework

A constraint is defined by:

1. **Constraint function**: `C(q, t) = 0` (holonomic) or `C(q, q̇, t) = 0` (non-holonomic)
2. **Constraint Jacobian**: `J = ∂C/∂q` (how constraint changes with configuration)
3. **Constraint time derivative**: `Ċ = J·q̇ + ∂C/∂t`
4. **Baumgarte stabilization terms**: `α` (position gain), `β` (velocity gain)

The Lagrange multiplier λ is computed to satisfy:
```
J·M⁻¹·Jᵀ·λ = -J·M⁻¹·F_ext - J̇·q̇ - α·C - β·Ċ
```

The constraint force applied is:
```
F_constraint = Jᵀ·λ
```

### Proposed Architecture

#### 1. Constraint Interface

```cpp
// Abstract constraint base class
class Constraint {
public:
    virtual ~Constraint() = default;

    // Number of scalar constraint equations
    virtual int dimension() const = 0;

    // Evaluate constraint function C(q, t)
    // Returns vector of constraint violations
    virtual Eigen::VectorXd evaluate(
        const InertialState& state,
        double time) const = 0;

    // Compute constraint Jacobian J = ∂C/∂q
    // Returns (dimension x state_dim) matrix
    virtual Eigen::MatrixXd jacobian(
        const InertialState& state,
        double time) const = 0;

    // Compute constraint time derivative ∂C/∂t (optional)
    virtual Eigen::VectorXd partialTimeDerivative(
        const InertialState& state,
        double time) const;

    // Baumgarte stabilization parameters
    virtual double alpha() const { return 10.0; }
    virtual double beta() const { return 10.0; }

    // Constraint type for debugging/logging
    virtual std::string typeName() const = 0;
};
```

#### 2. Bilateral vs Unilateral Constraints

```cpp
// Bilateral: Equality constraint (C = 0)
class BilateralConstraint : public Constraint {
    // Standard Lagrange multiplier (λ unrestricted)
};

// Unilateral: Inequality constraint (C ≥ 0)
class UnilateralConstraint : public Constraint {
    // Complementarity condition: λ ≥ 0, C ≥ 0, λ·C = 0
    virtual bool isActive(const InertialState& state, double time) const;
};
```

#### 3. Constraint Solver

```cpp
class ConstraintSolver {
public:
    // Solve for Lagrange multipliers given system state
    struct SolveResult {
        Eigen::VectorXd lambdas;           // Lagrange multipliers
        Eigen::VectorXd constraintForces;  // Generalized forces
        bool converged;
        int iterations;
    };

    SolveResult solve(
        const std::vector<Constraint*>& constraints,
        const InertialState& state,
        const Eigen::VectorXd& externalForces,
        const Eigen::MatrixXd& massMatrix,
        double dt);
};
```

#### 4. Constraint Library (Example Implementations)

```cpp
// Unit quaternion constraint (migrate from current QuaternionConstraint)
class UnitQuaternionConstraint : public BilateralConstraint {
    // C(q) = |Q|² - 1 = 0
    // J = 2·Qᵀ (w.r.t. quaternion components)
};

// Fixed distance constraint (rod/spring)
class DistanceConstraint : public BilateralConstraint {
    // C(q) = |p₁ - p₂|² - d² = 0
    // J = 2·(p₁ - p₂)ᵀ
};

// Ball-socket joint (spherical joint)
class BallSocketConstraint : public BilateralConstraint {
    // C(q) = p₁ + R₁·r₁ - p₂ - R₂·r₂ = 0  (3 constraints)
};

// Hinge/revolute joint
class RevoluteConstraint : public BilateralConstraint {
    // 5 constraints: 3 position + 2 orientation
};

// Contact non-penetration
class ContactConstraint : public UnilateralConstraint {
    // C(q) = n·(p₁ - p₂) - d ≥ 0
    // Active only when bodies in contact
};
```

---

## Requirements

### Functional Requirements

1. **FR-1**: Define abstract `Constraint` interface with evaluation, Jacobian, and stabilization methods
2. **FR-2**: Implement `ConstraintSolver` that computes Lagrange multipliers for arbitrary constraints
3. **FR-3**: Migrate existing `QuaternionConstraint` to use new framework
4. **FR-4**: Support both bilateral (equality) and unilateral (inequality) constraints
5. **FR-5**: Enable multiple constraints per body with proper force accumulation
6. **FR-6**: Integrate with existing `Integrator` abstraction

### Non-Functional Requirements

1. **NFR-1**: Constraint evaluation and Jacobian computation must be efficient (called every timestep)
2. **NFR-2**: Solver should handle ill-conditioned constraint matrices gracefully
3. **NFR-3**: Memory allocation should be minimized in hot paths (pre-allocate matrices)
4. **NFR-4**: Clear separation between constraint definition and solving algorithm

---

## Acceptance Criteria

1. [ ] `Constraint` abstract base class defined with evaluation and Jacobian interface
2. [ ] `BilateralConstraint` and `UnilateralConstraint` subclasses implemented
3. [ ] `ConstraintSolver` computes correct Lagrange multipliers for test constraints
4. [ ] `UnitQuaternionConstraint` reimplemented using new framework, passes existing tests
5. [ ] At least one additional constraint type implemented (e.g., `DistanceConstraint`)
6. [ ] Multiple constraints on same body correctly accumulate forces
7. [ ] Integration with `SemiImplicitEulerIntegrator` maintains existing behavior
8. [ ] Baumgarte stabilization parameters configurable per-constraint
9. [ ] Unit tests cover constraint evaluation, Jacobian correctness, and solver convergence
10. [ ] Documentation updated in CLAUDE.md with constraint library architecture

---

## Design Considerations

### Solver Algorithm Options

1. **Direct solve**: Gaussian elimination on J·M⁻¹·Jᵀ (small constraint counts)
2. **Iterative solve**: Projected Gauss-Seidel (large constraint counts, inequality constraints)
3. **Hybrid**: Direct for bilateral, iterative for unilateral

### Integration with Physics Pipeline

Two main approaches:

**Option A: Post-correction (current approach)**
- Integrator updates state ignoring constraints
- Constraint solver projects state back to constraint manifold
- Pros: Simple, decoupled
- Cons: Can cause energy drift

**Option B: Constraint forces during integration**
- Compute constraint forces before integration step
- Add to external forces
- Pros: More physically accurate, better energy conservation
- Cons: More coupled, requires modified integrator interface

**Recommendation**: Start with Option A (matches current architecture), with Option B as future enhancement.

### Jacobian Computation

- Analytical Jacobians preferred for efficiency and accuracy
- Numerical Jacobians (finite differences) as fallback for complex constraints
- Consider automatic differentiation for future work

---

## Dependencies

- **Ticket 0030**: Lagrangian quaternion physics (provides foundation and existing constraint)
- Eigen library for matrix operations

---

## Risks

1. **Numerical stability**: Constraint Jacobian matrices can become singular
   - Mitigation: Regularization, pseudo-inverse, or iterative solvers
2. **Performance**: Matrix operations per constraint per timestep
   - Mitigation: Pre-allocation, sparse matrices, constraint batching
3. **Complexity**: General framework may be over-engineered for current needs
   - Mitigation: Start with minimal interface, extend as needed

---

## Future Extensions

- **Friction constraints**: Coulomb friction via complementarity
- **Motor/actuator constraints**: Constraints with target velocities
- **Soft constraints**: Spring-damper approximations
- **Constraint graphs**: Hierarchical constraint dependencies
- **Warm starting**: Use previous λ as initial guess for iterative solvers

---

## References

- Baraff, D. (1996). "Linear-Time Dynamics using Lagrange Multipliers"
- Cline, M. B. (2002). "Rigid Body Simulation with Contact and Constraints"
- Bullet Physics Library: btSequentialImpulseConstraintSolver
- ODE (Open Dynamics Engine): Constraint solving architecture

---

## Workflow Log

### Design Phase
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Artifacts**:
  - `docs/designs/0031_generalized_lagrange_constraints/design.md`
  - `docs/designs/0031_generalized_lagrange_constraints/0031_generalized_lagrange_constraints.puml`
- **Notes**:
  - Designed extensible constraint framework with abstract `Constraint` interface
  - Created `BilateralConstraint` and `UnilateralConstraint` subclasses for equality/inequality constraints
  - Implemented `ConstraintSolver` using direct solve (Gaussian elimination on J·M⁻¹·Jᵀ)
  - Designed `UnitQuaternionConstraint` as migration from existing `QuaternionConstraint`
  - Added `DistanceConstraint` as example implementation
  - Modified `Integrator` interface to accept `std::vector<Constraint*>` instead of single constraint
  - Modified `SemiImplicitEulerIntegrator` to use `ConstraintSolver`
  - Modified `AssetInertial` to own constraint vector with default `UnitQuaternionConstraint`
  - Modified `WorldModel::updatePhysics()` to gather constraints from assets
  - Deferred multi-object constraints, unilateral solver, and actuator constraints to future tickets
  - Design includes 4 open questions requiring human input before proceeding to prototype
  - Breaking changes: `Integrator::step()` signature, `AssetInertial` member type
  - Backward compatibility preserved via automatic `UnitQuaternionConstraint` addition

### Design Review Phase
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Status**: Approved with notes
- **Notes**: Design approved, proceed to prototype phase

### Prototype Phase
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Artifacts**:
  - `docs/designs/0031_generalized_lagrange_constraints/prototype-results.md`
  - `prototypes/0031_generalized_lagrange_constraints/p1_conditioning/`
  - `prototypes/0031_generalized_lagrange_constraints/p2_overhead/`
- **Notes**:
  - P1 validated LLT decomposition with condition number monitoring
  - P2 validated virtual dispatch overhead < 1%
  - Both prototypes passed validation

### Implementation Phase
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Artifacts**:
  - `msd/msd-sim/src/Physics/Constraints/Constraint.hpp`
  - `msd/msd-sim/src/Physics/Constraints/Constraint.cpp`
  - `msd/msd-sim/src/Physics/Constraints/BilateralConstraint.hpp`
  - `msd/msd-sim/src/Physics/Constraints/UnilateralConstraint.hpp`
  - `msd/msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.hpp`
  - `msd/msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.cpp`
  - `msd/msd-sim/src/Physics/Constraints/DistanceConstraint.hpp`
  - `msd/msd-sim/src/Physics/Constraints/DistanceConstraint.cpp`
  - `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`
  - `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`
  - Modified: `msd/msd-sim/src/Physics/Integration/Integrator.hpp`
  - Modified: `msd/msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp`
  - Modified: `msd/msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.cpp`
- **Notes**:
  - All core constraint framework components implemented
  - Integration with existing integrator complete

### Quality Gate Phase
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Artifacts**:
  - `docs/designs/0031_generalized_lagrange_constraints/quality-gate-report.md`
- **Notes**:
  - Build: PASSED
  - Tests: PASSED (8/8 constraint tests, 398/400 overall)
  - 2 pre-existing test failures unrelated to ticket

### Implementation Review Phase (Iteration 1)
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Artifacts**:
  - `docs/designs/0031_generalized_lagrange_constraints/implementation-review.md`
- **Status**: CHANGES REQUESTED
- **Critical Issues**:
  - C2: Missing unit tests for constraint framework components
- **Notes**:
  - Design conformance: PASS
  - Code quality: PASS
  - Test coverage: NEEDS IMPROVEMENT (13 unit tests specified but not implemented)

### Test Implementation Phase
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Artifacts**:
  - `msd/msd-sim/test/Physics/Constraints/ConstraintTest.cpp`
- **Notes**:
  - Created 30 unit tests covering all specified test cases
  - UnitQuaternionConstraint: 9 tests
  - DistanceConstraint: 9 tests
  - ConstraintSolver: 4 tests
  - AssetInertial integration: 5 tests
  - Integration tests: 3 tests
  - All 30 tests passing (428/430 overall, 2 pre-existing failures)

### Implementation Review Phase (Iteration 2)
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Artifacts**:
  - `docs/designs/0031_generalized_lagrange_constraints/implementation-review-iteration2.md`
- **Status**: APPROVED (with minor items for follow-up)
- **Resolution**:
  - Critical issue C2 (missing unit tests) FULLY RESOLVED
  - Test coverage upgraded from NEEDS IMPROVEMENT to PASS
  - 30/13 required tests implemented (231% coverage)
  - All constraint tests passing at 100% rate
- **Non-blocking items**:
  - M1: Deprecate QuaternionConstraint (can be done pre-merge)
  - M3: Update CLAUDE.md (deferred to documentation phase per workflow)
  - Minor items m1, m2, m3 acknowledged (cosmetic only)
- **Next Steps**: Proceed to documentation phase

### Documentation Phase
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Artifacts**:
  - `docs/msd/msd-sim/Physics/generalized-constraints.puml`
  - Updated: `msd/msd-sim/src/Physics/CLAUDE.md`
  - Updated: `msd/msd-sim/CLAUDE.md`
  - `docs/designs/0031_generalized_lagrange_constraints/doc-sync-summary.md`
- **Notes**:
  - Copied design diagram to library documentation folder
  - Added 7 new constraint framework components to Physics module Core Components table
  - Created comprehensive "Generalized Constraint Framework" section in Physics/CLAUDE.md
  - Added diagram to msd-sim diagrams index
  - Added "Recent Architectural Changes" entry
  - Documented breaking changes, memory management, thread safety
  - Created documentation sync summary
- **Resolution of M3**: CLAUDE.md updates complete

### Tutorial Phase
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Artifacts**:
  - `docs/tutorials/lagrange-constraint-framework/README.md`
  - `docs/tutorials/lagrange-constraint-framework/example.cpp`
  - `docs/tutorials/lagrange-constraint-framework/CMakeLists.txt`
  - `docs/tutorials/lagrange-constraint-framework/presentation.html`
  - Updated: `docs/tutorials/TUTORIAL_STATE.md`
- **Notes**:
  - Created comprehensive tutorial explaining Lagrange multiplier constraint framework
  - README.md covers mathematical foundation (constraint formulation, Jacobians, Baumgarte stabilization)
  - example.cpp is standalone C++ implementation (~700 lines) with no external dependencies
  - Implements Constraint interface, UnitQuaternionConstraint, DistanceConstraint, and LLT solver
  - presentation.html provides 22 Reveal.js slides covering theory and implementation
  - Compiles and runs successfully, demonstrates constraint enforcement with Baumgarte stabilization
  - Tutorial shows expected drift without full angular integration (educational design choice)
  - Codebase mappings added to TUTORIAL_STATE.md

---

## Human Feedback

### Implementation Review Iteration 1 - Addressed
- **Issue C2**: Missing unit tests for constraint framework ✓
  - **Resolution**: Created `test/Physics/Constraints/ConstraintTest.cpp` with 30 comprehensive unit tests
  - All 13 specified test categories covered
  - All tests passing
