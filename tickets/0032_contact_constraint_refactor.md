# Ticket 0032: Contact Constraint Refactor

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete — Awaiting Tutorial
- [ ] Tutorial Complete — Ready to Merge (if Generate Tutorial: Yes)
- [ ] Merged / Complete

**Current Phase**: Implementation Review
**Assignee**: implementation-reviewer
**Created**: 2026-01-29
**Generate Tutorial**: Yes

---

## Sub-Tickets

This ticket has been decomposed into the following sub-tickets:

| Sub-Ticket | Description | Status | Depends On |
|------------|-------------|--------|------------|
| [0032a](0032a_two_body_constraint_infrastructure.md) | Two-Body Constraint Infrastructure (classes + unit tests) | Code complete, tests pending | 0031 |
| [0032b](0032b_pgs_solver_extension.md) | Projected Gauss-Seidel Solver Extension | Not started | 0032a |
| [0032c](0032c_worldmodel_contact_integration.md) | WorldModel Contact Integration + integration tests | Not started | 0032b |
| [0032d](0032d_collision_response_cleanup.md) | CollisionResponse Namespace Cleanup | Not started | 0032c |

**Dependency chain**: `0032a → 0032b → 0032c → 0032d`

---

## Summary

Refactor the collision response system to use the Constraint framework from Ticket 0031. This involves creating a `ContactConstraint` class that implements the `UnilateralConstraint` interface, integrating collision detection with constraint-based physics, and removing the standalone `CollisionResponse` namespace utilities.

---

## Motivation

The current collision response system (Ticket 0027) uses a standalone `CollisionResponse` namespace with impulse-based physics that operates separately from the Lagrangian constraint framework (Ticket 0031). This creates two parallel systems for applying constraint forces:

1. **Constraint System**: `Constraint` interface with `ConstraintSolver` computing Lagrange multipliers
2. **Collision System**: `CollisionResponse` namespace with ad-hoc impulse calculations

Unifying these systems provides:

1. **Consistency**: All constraint forces (joints, contacts, quaternion normalization) computed through the same solver infrastructure
2. **Accuracy**: Contact constraints benefit from Baumgarte stabilization for drift correction
3. **Extensibility**: Friction can be added as additional constraint rows in the future
4. **Maintainability**: Single constraint solving algorithm instead of two separate force calculation methods
5. **Physical Correctness**: Lagrangian formulation provides energy-consistent constraint enforcement

### Use Cases

- **Resting contacts**: Objects stacked on each other remain stable without drift
- **Impact collisions**: High-velocity impacts resolved with proper momentum transfer
- **Multiple simultaneous contacts**: Object touching multiple surfaces handled uniformly
- **Future friction**: Tangential constraint forces added as additional rows (deferred)

---

## Technical Approach

### Contact Constraint Formulation

A contact constraint enforces non-penetration between two colliding bodies:

**Constraint Function (Unilateral)**:
```
C(q) = (x_B - x_A) · n + d ≥ 0
```
Where:
- `x_A`, `x_B` are contact points on bodies A and B
- `n` is the contact normal (A → B)
- `d` is the signed penetration depth (negative when penetrating)

**Constraint Jacobian**:
```
J = [∂C/∂x_A, ∂C/∂Q_A, ∂C/∂x_B, ∂C/∂Q_B]
  = [-n^T, -(r_A × n)^T, n^T, (r_B × n)^T]
```
Where `r_A`, `r_B` are lever arms from center of mass to contact point.

**Velocity-Level Constraint**:
```
Ċ = v_rel · n ≥ 0  (for separating or sliding contacts)
```

**Complementarity Conditions** (unilateral):
```
C ≥ 0    (non-penetration)
λ ≥ 0    (force only pushes, never pulls)
λ · C = 0  (force is zero when not in contact)
```

### Restitution Handling

Coefficient of restitution modifies the velocity-level constraint target:
```
Ċ_target = -e · Ċ_impact
```
Where `e` is the combined coefficient of restitution.

This can be incorporated into the constraint RHS:
```
b = -e · (J · q̇)_impact
```

### Proposed Architecture

#### 1. ContactConstraint Class

```cpp
class ContactConstraint : public UnilateralConstraint {
public:
    ContactConstraint(
        const CollisionResult& collision,
        const AssetInertial& bodyA,
        const AssetInertial& bodyB,
        double restitution);

    // Constraint interface
    int dimension() const override;  // 1 per contact point
    Eigen::VectorXd evaluate(const InertialState& stateA,
                             const InertialState& stateB,
                             double time) const override;
    Eigen::MatrixXd jacobian(const InertialState& stateA,
                             const InertialState& stateB,
                             double time) const override;
    bool isActive(const InertialState& stateA,
                  const InertialState& stateB,
                  double time) const override;

    std::string typeName() const override { return "ContactConstraint"; }

private:
    Coordinate contactNormal_;           // A → B
    Coordinate contactPointA_;           // World-space contact on A
    Coordinate contactPointB_;           // World-space contact on B
    double penetrationDepth_;
    double restitution_;
    double impactVelocity_;              // For restitution calculation
};
```

#### 2. Multi-Body Constraint Extension

The current `Constraint` interface operates on single-object `InertialState`. Contact constraints involve two bodies. Options:

**Option A: Extend Constraint Interface**
```cpp
// Add multi-body variant methods
virtual Eigen::MatrixXd jacobianMultiBody(
    const std::vector<InertialState*>& states,
    double time) const;
```

**Option B: Stacked State Vector**
```cpp
// Treat two-body system as single state vector [stateA; stateB]
// Constraint operates on 28-component vector (14 + 14)
```

**Option C: Constraint References Bodies by ID**
```cpp
// Constraint stores body indices, solver manages state assembly
class ContactConstraint {
    size_t bodyAIndex_;
    size_t bodyBIndex_;
};
```

**Recommendation**: Option C provides cleanest separation and enables future N-body constraint graphs.

#### 3. Modified Constraint Solver

The `ConstraintSolver` needs extension to handle:
- Multi-body constraints (two or more bodies per constraint)
- Unilateral constraints with complementarity (λ ≥ 0)
- Warm starting for iterative convergence

```cpp
class ConstraintSolver {
public:
    // Extended solve for contact constraints
    SolveResult solveWithContacts(
        const std::vector<Constraint*>& bilateralConstraints,
        const std::vector<ContactConstraint*>& contactConstraints,
        const std::vector<AssetInertial*>& bodies,
        double dt);

private:
    // Projected Gauss-Seidel for inequality constraints
    void solveUnilateral(/* ... */);
};
```

#### 4. WorldModel Integration

```cpp
void WorldModel::update(double dt) {
    // 1. Detect collisions
    auto contacts = detectCollisions();

    // 2. Create contact constraints
    std::vector<std::unique_ptr<ContactConstraint>> contactConstraints;
    for (const auto& contact : contacts) {
        contactConstraints.push_back(
            std::make_unique<ContactConstraint>(contact, ...));
    }

    // 3. Gather all constraints (bilateral + contacts)
    std::vector<Constraint*> allConstraints;
    // ... add object constraints
    // ... add contact constraints

    // 4. Solve unified constraint system
    constraintSolver_.solve(allConstraints, bodies, dt);

    // 5. Integrate motion
    integrator_->step(...);
}
```

---

## Requirements

### Functional Requirements

1. **FR-1**: Create `ContactConstraint` class implementing `UnilateralConstraint` interface
2. **FR-2**: Extend `ConstraintSolver` to handle multi-body constraints
3. **FR-3**: Implement projected Gauss-Seidel solver for unilateral constraints (λ ≥ 0)
4. **FR-4**: Support coefficient of restitution in contact constraint formulation
5. **FR-5**: Integrate contact constraints into `WorldModel::update()` pipeline
6. **FR-6**: Remove `CollisionResponse` namespace after migration complete
7. **FR-7**: Maintain existing collision detection (GJK/EPA) as input to contact constraints

### Non-Functional Requirements

1. **NFR-1**: Performance must match or exceed current impulse-based approach
2. **NFR-2**: Memory allocation minimized in collision/constraint hot path
3. **NFR-3**: Contact constraints must be transient (created/destroyed per frame)
4. **NFR-4**: Clear separation between collision detection and constraint solving
5. **NFR-5**: Existing bilateral constraints (quaternion, distance) unaffected

---

## Acceptance Criteria

1. [ ] AC1: `ContactConstraint` class implements `UnilateralConstraint` with correct `evaluate()` and `jacobian()`
2. [ ] AC2: `ConstraintSolver` handles multi-body constraints correctly
3. [ ] AC3: Projected Gauss-Seidel solver enforces λ ≥ 0 for contact constraints
4. [ ] AC4: Head-on collision with e=1.0 swaps velocities (matches current behavior)
5. [ ] AC5: Total momentum conserved before/after collision (within 1e-6 tolerance)
6. [ ] AC6: Stacked objects remain stable for 1000 frames without drift
7. [ ] AC7: Glancing collision produces appropriate angular velocity
8. [ ] AC8: `CollisionResponse.hpp` and `CollisionResponse.cpp` removed
9. [ ] AC9: All existing constraint tests continue to pass
10. [ ] AC10: Unit tests cover contact constraint evaluation, Jacobian, and solver
11. [ ] AC11: Integration tests cover collision scenarios from Ticket 0027

---

## Design Considerations

### Solver Algorithm for Unilateral Constraints

The current `ConstraintSolver` uses direct LLT solve suitable for bilateral constraints. Unilateral constraints require:

**Option A: Projected Gauss-Seidel (PGS)**
- Iterative solver with clamping: `λ = max(0, λ_unclamped)`
- Industry standard (Bullet, ODE, Box2D)
- Handles inequality and friction naturally
- Pros: Simple, robust, handles friction
- Cons: Slower convergence, requires iteration count tuning

**Option B: Lemke's Algorithm**
- Direct solver for Linear Complementarity Problems (LCP)
- Pros: Exact solution for small systems
- Cons: Complex implementation, pivoting issues

**Option C: Hybrid**
- Direct solve for bilateral, PGS for unilateral
- Solve bilateral first, then project contacts
- Pros: Leverages existing solver, good performance
- Cons: Coupling between bilateral and unilateral

**Recommendation**: Option A (PGS) as primary approach, matching industry practice.

### Position Correction Strategy

Two approaches for preventing penetration drift:

**Option A: Baumgarte Stabilization Only**
- Use existing α·C term in constraint RHS
- Pros: Clean, unified with other constraints
- Cons: Can add energy, requires tuning

**Option B: Split Impulse (Velocity Bias)**
- Separate position and velocity correction
- Pros: Better energy conservation
- Cons: More complex implementation

**Recommendation**: Start with Option A (matches existing framework), consider Option B as enhancement.

### Transient vs Persistent Contacts

**Option A: Transient (Per-Frame)**
- Create contact constraints fresh each frame
- Pros: Simple, no state management
- Cons: No warm starting, potential jitter

**Option B: Persistent (Contact Caching)**
- Track contacts across frames, update positions
- Pros: Warm starting improves convergence
- Cons: Complex contact matching logic

**Recommendation**: Start with Option A (simpler), add persistence as future enhancement.

---

## Dependencies

- **Ticket 0031**: Generalized Lagrange Multiplier Constraint System (prerequisite)
- **Ticket 0027**: Collision Response System (being replaced)
- **Ticket 0027a**: EPA Algorithm (provides collision information)
- **Ticket 0029**: Contact Manifold Generation (provides multi-point contacts)

---

## Risks

1. **Performance Regression**: Iterative solver may be slower than direct impulse
   - Mitigation: Benchmark against current system, tune iteration count
2. **Numerical Stability**: PGS convergence depends on constraint conditioning
   - Mitigation: Regularization, warm starting, iteration limits
3. **Energy Drift**: Baumgarte stabilization can add energy
   - Mitigation: Careful parameter tuning, consider split impulse
4. **Complexity**: Multi-body constraint handling adds complexity to solver
   - Mitigation: Clear abstractions, thorough testing

---

## Files to Modify

### New Files
- `msd-sim/src/Physics/Constraints/ContactConstraint.hpp`
- `msd-sim/src/Physics/Constraints/ContactConstraint.cpp`
- `msd-sim/test/Physics/Constraints/ContactConstraintTest.cpp`

### Modified Files
- `msd-sim/src/Physics/Constraints/Constraint.hpp` — Multi-body support (if needed)
- `msd-sim/src/Physics/Constraints/UnilateralConstraint.hpp` — Complementarity interface
- `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` — PGS solver
- `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` — PGS implementation
- `msd-sim/src/Environment/WorldModel.hpp` — Contact constraint integration
- `msd-sim/src/Environment/WorldModel.cpp` — Updated collision pipeline
- `msd-sim/CMakeLists.txt` — Add new source files

### Files to Remove
- `msd-sim/src/Physics/CollisionResponse.hpp`
- `msd-sim/src/Physics/CollisionResponse.cpp`
- `msd-sim/test/Physics/CollisionResponseTest.cpp` (migrate tests to ContactConstraintTest)

---

## Future Extensions

- **Friction Constraints**: Add tangential constraint rows with Coulomb friction cone
- **Persistent Contacts**: Contact caching and warm starting for stability
- **Contact Graphs**: Handle contact islands independently for parallelization
- **Continuous Collision**: Time-of-impact constraints for tunneling prevention
- **Soft Contacts**: Compliance/regularization for softer collision response

---

## References

### Academic Literature
- Catto, E. (2005). "Iterative Dynamics with Temporal Coherence" (Box2D)
- Baraff, D. (1994). "Fast Contact Force Computation for Nonpenetrating Rigid Bodies"
- Stewart, D. & Trinkle, J. (1996). "An Implicit Time-Stepping Scheme for Rigid Body Dynamics with Coulomb Friction"

### Physics Engine Implementations
- Bullet Physics: `btSequentialImpulseConstraintSolver`
- ODE (Open Dynamics Engine): Contact constraint handling
- Box2D: Position and velocity constraint solver

### Related Code
- `msd-sim/src/Physics/Constraints/Constraint.hpp` — Constraint interface to implement
- `msd-sim/src/Physics/Constraints/UnilateralConstraint.hpp` — Base class for contact
- `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` — Solver to extend
- `msd-sim/src/Physics/CollisionResponse.hpp` — Code to be replaced
- `msd-sim/src/Physics/CollisionHandler.hpp` — Collision detection (retained)
- `msd-sim/src/Physics/CollisionResult.hpp` — Contact manifold input

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-01-29 14:00
- **Completed**: 2026-01-29 14:20
- **Artifacts**:
  - `docs/designs/0032_contact_constraint_refactor/math-formulation.md`
  - `docs/designs/0032_contact_constraint_refactor/design.md`
  - `docs/designs/0032_contact_constraint_refactor/0032_contact_constraint_refactor.puml`
- **Notes**: Design completed by cpp-architect agent. Math formulation includes detailed constraint derivation, Baumgarte stabilization, and restitution handling. Architectural design includes two-body constraint interface, ContactConstraint class, PGS solver, and WorldModel integration.

### Design Review Phase
- **Started**: 2026-01-29
- **Completed**: 2026-01-29
- **Status**: APPROVED WITH NOTES
- **Iteration**: 0 of 1 (no revision needed)
- **Artifacts**:
  - Design review appended to `docs/designs/0032_contact_constraint_refactor/design.md`
- **Notes**: Design is architecturally sound and ready for implementation. All criteria pass. Two critical prototype bugs (Baumgarte parameter mismatch, restitution formula) were identified and resolved with clear implementation guidance. Recommended design document updates before implementation: (1) Use ERP=0.2 for Baumgarte (not α=100 directly), (2) Clarify restitution formula (v_target = -e*v_pre). Prototype validation confirms numerical stability and correctness. No architectural changes required.

### Prototype Phase
- **Started**: 2026-01-29 14:30
- **Completed**: 2026-01-29 15:50
- **Artifacts**:
  - `prototypes/0032_contact_constraint_refactor/p1_pgs_convergence/` (PASSED)
  - `prototypes/0032_contact_constraint_refactor/p2_energy_conservation/` (PASSED after fix)
  - `prototypes/0032_contact_constraint_refactor/p1_pgs_convergence/Debug_Findings.md`
  - `prototypes/0032_contact_constraint_refactor/p2_energy_conservation/Debug_Findings.md`
- **Notes**: All three prototypes completed and validated. P1 confirmed PGS convergence with ERP=0.2 (stacked objects stable for 10,000 frames, drift < 0.001m). P2 confirmed energy conservation with correct restitution formula (v_target = -e*v_pre). Two critical bugs identified and resolved: (1) Baumgarte parameter unit mismatch - use ERP=0.2 not α=100 directly, (2) Restitution formula bug - single-line fix validated. Design document updated with prototype findings. Ready for implementation.

### Implementation Phase
- **Started**:
- **Completed**:
- **Artifacts**:
- **Notes**:

### Quality Gate Phase
- **Started**:
- **Completed**:
- **Artifacts**:
- **Notes**:

### Implementation Review Phase
- **Started**: 2026-01-29 18:45
- **Completed**: 2026-01-29 19:00
- **Status**: APPROVED
- **Artifacts**:
  - `docs/designs/0032_contact_constraint_refactor/implementation-review.md`
- **Notes**: Implementation of ticket 0032a (Two-Body Constraint Infrastructure) is production-ready and approved for merge. All components match the design specification exactly with zero deviations. Prototype learnings (ERP=0.2, restitution formula v_target=-e*v_pre) correctly applied. Code quality is exemplary with comprehensive documentation, proper validation, and efficient implementation. Test coverage comprehensive with 33 new tests, all passing. No critical, major, or minor issues identified. Ready to proceed to sub-ticket 0032b (PGS Solver Extension).

### Documentation Phase
- **Started**:
- **Completed**:
- **Artifacts**:
- **Notes**:

### Tutorial Phase (if Generate Tutorial: Yes)
- **Started**:
- **Completed**:
- **Artifacts**:
- **Notes**:

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design

~~In the math design section 2.2, you have one contact point per object. Is there a reason to constrain this to one contact point compared to if we have multiple contact points (e.g. two faces colliding)? Or does this generalize to multiple contact points?~~ ✓ **ADDRESSED**: Math formulation Section 2.5 added, explains generalization to k contact points with separate constraint rows.

~~In 3.3, can you elaborate on how $\frac{\partial{C}}{\partial{X_{A}}} = -n^T$? I think I can get there, but the intuition isn't immediately clear to me.~~ ✓ **ADDRESSED**: Math formulation Section 3.3 expanded with physical intuition explanation.

~~Can you elaborate on section 3.4? It's unclear to me where $\partial{\omega}$ comes from, considering we're using the angular positions, not rates.~~ ✓ **ADDRESSED**: Math formulation Section 3.4 expanded with "Why angular velocity appears" subsection explaining rigid body kinematics.

~~Noting the special case with the environment, I suspect we'll want to slightly modify the AssetEnvironment such that it has a mass that can drive the inverse mass to zero.~~ ✓ **ADDRESSED**: Design Section on AssetEnvironment adds getInverseMass() returning 0.0 and getInverseInertiaTensor() returning Zero matrix for unified solver path.

### Feedback on Prototypes

~~Prototypes 1 and 2 have `Debug_Findings.md` files whereing the debug agent performed a root-cause analysis of the errors that were reported. Please reference them when addressing issues.~~ ✓ **REVIEWED**: Both debug findings incorporated into design review:
- **P1 Baumgarte mismatch**: Use ERP=0.2 instead of α=100 directly (conversion formula required)
- **P2 Restitution bug**: Use v_target = -e*v_pre (single-line fix validated)

### Feedback on Implementation
{Your comments on the implementation}

### Feedback on Tutorial (if Generate Tutorial: Yes)
{Your comments on the tutorial content, concepts to cover, or presentation style}
