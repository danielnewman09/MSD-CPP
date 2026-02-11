# Ticket 0055: Tilted Cube Friction Direction Investigation

## Status
- [ ] Draft
- [ ] Investigation Complete — Root Cause Identified
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Debug Investigation (Parent)
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-10
**Branch**: TBD
**GitHub Issue**: TBD
**Related Tickets**: [0035_friction_constraints](0035_friction_constraints.md), [0035a_tangent_basis_friction_constraint](0035a_tangent_basis_friction_constraint.md), [0052_custom_friction_cone_solver](0052_custom_friction_cone_solver.md), [0039_collision_energy_stabilization_debug](0039_collision_energy_stabilization_debug.md)

---

## Subtickets

| Subticket | Title | Type | Dependencies | Status |
|-----------|-------|------|--------------|--------|
| [0055a](0055a_tilted_cube_trajectory_test_suite.md) | Tilted Cube Trajectory Test Suite | Test Suite | None | Draft |
| [0055b](0055b_friction_direction_root_cause.md) | Friction Direction Root Cause Investigation | Investigation | 0055a | Investigation Complete |
| [0055c](0055c_friction_direction_fix.md) | Friction Direction Fix and Regression Testing | Implementation | 0055b | Draft |

### Dependency Graph

```
0055a (Trajectory Tests)
    │
    ▼
0055b (Root Cause Investigation)
    │
    ▼
0055c (Fix + Regression Testing)
```

### Subticket Summary

- **0055a**: Extends tilted cube tests to cover multiple tilt orientations and asserts trajectory correctness.
- **0055b**: Uses failing tests to diagnose why friction pushes in incorrect directions for certain tilt angles.
- **0055c**: Implements the fix and verifies all trajectory tests pass without regressions.

---

## Problem Statement

When a cube is dropped onto the floor with an initial tilt, the resulting rotation and bounce direction are visibly incorrect for certain tilt orientations. Specifically, a cube tilted by `(0.1, 0.1, 0)` radians in the `(x, y, z)` frame produces rotation in unexpected directions upon contact with the floor.

### Observed Behavior

- A tilted cube contacting the floor should bounce/rotate in a direction consistent with the contact geometry — e.g., a cube tilted to the right should rock rightward, not leftward or in an arbitrary direction.
- For small tilts purely about X, the behavior may be approximately correct.
- For compound tilts (X+Y, or negative angles), the post-contact trajectory is visibly wrong — the cube rotates or translates in a direction inconsistent with its tilt.

### Suspected Root Cause

The frictional force direction appears incorrect in certain cases. Possible sources:

1. **EPA contact normal inaccuracy**: For near-flat contacts with small tilt, EPA may return imprecise normals that misalign the tangent basis.
2. **Tangent basis discontinuity**: The Duff et al. (2017) tangent basis is deterministic but the axis selection switches at threshold values of the normal. A near-vertical normal with small tilt may produce tangent vectors that don't align with the physical sliding direction.
3. **Contact point offset error**: If the EPA contact point is inaccurate for tilted configurations, the lever arm `r = contactPoint - CoM` will be wrong, producing incorrect torque even if the friction force magnitude is correct.
4. **Constraint solver friction direction**: The FrictionConeSolver operates in the tangent basis frame. If the basis doesn't correspond to the physical sliding plane, the friction impulse will be projected incorrectly back to world coordinates.

### What Correct Behavior Looks Like

For a unit cube dropped from rest onto a horizontal floor with initial tilt:

| Tilt | Expected Bounce/Roll Direction |
|------|-------------------------------|
| `+θ` about X | Rock toward +Y (cube leans in +Y direction, tips that way) |
| `-θ` about X | Rock toward -Y (mirror of above) |
| `+θ` about Y | Rock toward -X |
| `-θ` about Y | Rock toward +X |
| `+θ` about X, `+θ` about Y | Rock toward diagonal (+Y, -X) |
| `+θ` about X, `-θ` about Y | Rock toward diagonal (-Y, -X) |

The sign/direction must be physically consistent — negating the tilt angle should mirror the trajectory.

---

## Existing Test Coverage

### Tilted Cube Tests (Existing)

| Test | File | What It Tests | Tilt |
|------|------|---------------|------|
| `H8_TiltedCube_FeedbackLoop` | `ParameterIsolationTest.cpp:731` | Energy growth from tilt feedback loop | 2° about X |
| `C2_RockingCube_AmplitudeDecreases` | `RotationDampingTest.cpp:71` | Rocking amplitude decreases | 15° about X |
| `C3_TiltedCubeSettles_ToFlatFace` | `RotationDampingTest.cpp:180` | Cube settles to flat face | 30° about X |
| `TiltedCube_AsymmetricDepths` | `PerContactDepthTest.cpp:159` | Asymmetric contact depths | 3° about Y |
| `OrientationCorrection_TiltedCube` | `SplitImpulseTest.cpp:493` | Position corrector orientation fix | Various |

### Gap

All existing tilted cube tests use **single-axis tilt about X only** (or occasionally Y). No test validates:
- Compound tilts (X+Y)
- Negative tilts (symmetry)
- Trajectory direction correctness (only energy/stability checked)
- Friction force direction specifically

---

## Scope

### In Scope

- New test suite with multiple tilt angle variations
- Trajectory direction assertions (bounce direction matches tilt geometry)
- Symmetry assertions (negating tilt mirrors trajectory)
- Root cause investigation of incorrect friction direction
- Fix implementation with full regression testing

### Out of Scope

- Friction coefficient tuning or new friction models
- Performance optimization of friction solver
- Tilted floor surfaces (this ticket focuses on tilted body, flat floor)
- Stacking or multi-body friction scenarios

---

## Acceptance Criteria

1. Test suite covers at minimum: `+X`, `-X`, `+Y`, `-Y`, `+X+Y`, `+X-Y`, `-X+Y`, `-X-Y` tilt orientations
2. Tests assert that post-contact lateral displacement direction is consistent with tilt geometry
3. Tests assert symmetry: negating tilt produces mirrored trajectory
4. All new tests pass after fix implementation
5. No regressions in existing test suite (currently 693 tests)

---

## Technical Context

### Friction Pipeline (Current State)

```
EPA contact detection
  → ContactConstraint (normal direction, 1 row)
  → FrictionConstraint (2 tangent directions, 2 rows)
  → ConstraintSolver dispatches to FrictionConeSolver
  → FrictionConeSolver enforces Coulomb cone: ||λ_t|| ≤ μ·λ_n
```

### Key Files

| File | Purpose |
|------|---------|
| `Physics/Collision/TangentBasis.hpp` | Duff et al. tangent basis from contact normal |
| `Physics/Constraints/FrictionConstraint.hpp/.cpp` | 2×12 Jacobian for friction |
| `Physics/Constraints/FrictionConeSolver.hpp/.cpp` | Newton solver for Coulomb cone |
| `Physics/Constraints/ConstraintSolver.cpp` | Friction path dispatch and flattening |
| `Physics/Collision/CollisionPipeline.cpp` | Friction coefficient combination and constraint creation |
| `Physics/Collision/EPA.hpp/.cpp` | Contact normal and point extraction |
| `Physics/Collision/CollisionHandler.cpp` | SAT fallback for contact manifold |
