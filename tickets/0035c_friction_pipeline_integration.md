# Ticket 0035c: Friction Pipeline Integration

## Status
- [x] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Draft
**Assignee**: N/A
**Created**: 2026-01-31
**Generate Tutorial**: No
**Parent Ticket**: [0035_friction_constraints](0035_friction_constraints.md)

---

## Summary

Integrate friction constraints into the existing contact pipeline. Extend `ContactConstraintFactory` to create `FrictionConstraint` objects alongside normal `ContactConstraint` objects, add friction coefficient to rigid body material properties, implement the friction coefficient combination rule, and wire friction through `WorldModel::update()`. This subtask connects the foundation (0035a) and solver (0035b) to produce end-to-end friction behavior.

---

## Motivation

After 0035a (FrictionConstraint class) and 0035b (solver extension), the pieces exist but aren't connected. Currently:
- Collision detection produces contact points with normals
- `ContactConstraintFactory` creates normal constraints only
- `WorldModel::update()` passes normal constraints to the solver
- Bodies have no friction coefficient

This subtask:
- Adds friction coefficient ($\mu$) to body/material properties
- Extends the factory to create friction constraints alongside normal constraints
- Implements the combination rule ($\mu = \sqrt{\mu_A \cdot \mu_B}$)
- Passes friction constraints through the solver pipeline
- Provides the first end-to-end integration tests (inclined plane, sliding deceleration)

---

## Technical Approach

### Modified Components

| Component | Changes |
|-----------|---------|
| `RigidBody` or material system | Add `frictionCoefficient` property (default $\mu = 0.5$) |
| `ContactConstraintFactory` | Create `FrictionConstraint` alongside `ContactConstraint` for each contact point. Compute tangent basis. Combine friction coefficients. |
| `WorldModel::update()` | Pass friction constraints to `ConstraintSolver::solveWithContacts()` |
| `CollisionResponse` | Propagate friction coefficient through contact data |

### Friction Coefficient Combination

Per AC9 of parent ticket, use geometric mean:
$$
\mu_{\text{combined}} = \sqrt{\mu_A \cdot \mu_B}
$$

This matches the restitution combination pattern from Ticket 0032.

### Optimization: Skip Friction When $\mu = 0$

Per M7 analysis, when $\mu = 0$ (frictionless contact), skip friction constraint creation entirely. The constraint system remains $C \times C$ instead of $3C \times 3C$, preserving performance for frictionless scenarios.

---

## Requirements

### Functional Requirements

1. **FR-1**: Bodies have configurable friction coefficient $\mu \geq 0$
2. **FR-2**: `ContactConstraintFactory` creates 3 constraint rows per contact (1 normal + 2 friction) when $\mu > 0$
3. **FR-3**: Factory computes tangent basis using `TangentBasis` from 0035a
4. **FR-4**: Friction coefficient combined via geometric mean
5. **FR-5**: `WorldModel::update()` produces correct friction forces in simulation loop
6. **FR-6**: When $\mu = 0$, no friction constraints created (backward compatible)

### Non-Functional Requirements

1. **NFR-1**: All existing normal-contact tests pass with $\mu = 0$ (zero regressions)
2. **NFR-2**: Default friction coefficient is non-zero ($\mu = 0.5$) for new bodies
3. **NFR-3**: No API-breaking changes to existing `ContactConstraintFactory` interface

---

## Acceptance Criteria

- [ ] **AC1**: Block on 20° inclined plane with $\mu = 0.6$ remains stationary for 1000 timesteps (static friction, M8 Example 1)
- [ ] **AC2**: Block on 45° inclined plane with $\mu = 0.3$ accelerates at $g(\sin\theta - \mu\cos\theta) \pm 5\%$ (kinetic friction, M8 Example 2)
- [ ] **AC3**: Sliding block with $v_0 = 10$ m/s, $\mu = 0.5$ decelerates at $\mu g \pm 5\%$ (M8 Example 3)
- [ ] **AC4**: Glancing collision produces angular velocity (M8 Example 4, resolves 0032c AC4 limitation)
- [ ] **AC5**: Friction coefficient combination: $\mu_A = 0.3$, $\mu_B = 0.8$ produces $\mu = \sqrt{0.24} \approx 0.49$
- [ ] **AC6**: Frictionless contacts ($\mu = 0$) behave identically to current system
- [ ] **AC7**: All existing constraint tests pass (zero regressions)

---

## Dependencies

- **Ticket 0035a**: FrictionConstraint class (prerequisite — provides constraint data model)
- **Ticket 0035b**: Box-constrained ASM solver (prerequisite — provides solver capability)
- **Blocks**: [0035d](0035d_friction_hardening_and_validation.md) (hardening needs working pipeline)

---

## Files

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp` | Create friction constraints, compute tangent basis, combine $\mu$ |
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` | Implementation |
| `msd-sim/src/Physics/CollisionResponse.hpp` | Propagate friction coefficient |
| `msd-sim/src/Physics/CollisionResponse.cpp` | Implementation |
| `msd-sim/src/Physics/WorldModel.hpp` | Pass friction through update loop |
| `msd-sim/src/Physics/WorldModel.cpp` | Implementation |
| Body/material class (TBD) | Add friction coefficient property |
| `msd-sim/test/Physics/CMakeLists.txt` | Add integration test files |

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/FrictionIntegrationTest.cpp` | End-to-end friction tests (inclined plane, sliding, glancing collision) |

---

## References

- **Math formulation**: [M8-numerical-examples.md](../docs/designs/0035_friction_constraints/M8-numerical-examples.md) (integration test scenarios)
- **Existing factory**: `msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp`
- **Existing pipeline**: `msd-sim/src/Physics/WorldModel.hpp`

---

## Workflow Log

### Draft Phase
- **Created**: 2026-01-31
- **Notes**: Integration subtask connecting 0035a and 0035b to the simulation pipeline. First subtask that produces user-visible friction behavior. Math formulation already complete in parent.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
