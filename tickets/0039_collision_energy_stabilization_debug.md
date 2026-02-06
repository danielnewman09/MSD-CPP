# Ticket 0038: Collision Energy Stabilization Debug Investigation

## Status
- [x] Draft
- [ ] Ready for Investigation
- [ ] Investigation Complete — Root Cause Identified
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Draft
**Assignee**: TBD
**Created**: 2026-02-05
**Generate Tutorial**: No
**Related Tickets**: [0032_contact_constraint_refactor](0032_contact_constraint_refactor.md), [0034_active_set_method_contact_solver](0034_active_set_method_contact_solver.md), [0035_friction_constraints](0035_friction_constraints.md), [0036_collision_pipeline_extraction](0036_collision_pipeline_extraction.md)
**Type**: Debug Investigation (Parent)

---

## Subtickets

This investigation has been split into sequential subtickets:

| Subticket | Title | Type | Dependencies | Status |
|-----------|-------|------|--------------|--------|
| [0038a](0038a_energy_tracking_diagnostic_infrastructure.md) | Energy Tracking Diagnostic Infrastructure | Infrastructure | None | Draft |
| [0038b](0038b_linear_collision_test_suite.md) | Linear Collision Test Suite | Test Suite | 0038a | Draft |
| [0038c](0038c_rotational_coupling_test_suite.md) | Rotational Coupling Test Suite | Test Suite | 0038b | Draft |
| [0038d](0038d_parameter_isolation_root_cause.md) | Parameter Isolation and Root Cause Identification | Investigation | 0038a, 0038c | Draft |
| [0038e](0038e_fix_implementation_regression_testing.md) | Fix Implementation and Regression Testing | Implementation | 0038d | Draft |

### Dependency Graph

```
0038a (Diagnostics)
    │
    ├──► 0038b (Linear Tests)
    │        │
    │        ▼
    │    0038c (Rotation Tests)
    │        │
    └────────┼──► 0038d (Root Cause)
             │        │
             │        ▼
             └──► 0038e (Fix)
```

### Subticket Summary

- **0038a**: Creates `EnergyTracker` utility and logging infrastructure. Must be done first.
- **0038b**: Tests pure translational collisions (no rotation). Establishes baseline correctness.
- **0038c**: Tests rotational collisions. This is where the bug should manifest.
- **0038d**: Uses diagnostics and failing tests to systematically identify root cause.
- **0038e**: Implements the fix and verifies all tests pass.

---

## Problem Statement

When dropping a cube onto the world floor at an angle, the cube does not come to rest as expected.

### Physics Clarification

#### Contact Geometry: Axis-Aligned vs. Edge/Corner

**Axis-aligned contact** (cube face parallel to floor, rotation only about vertical axis):
- Contact normal is purely vertical (aligned with gravity)
- Linear velocity is purely vertical
- No lever arm couples linear motion to rotation
- Collision is effectively 1D
- **Result**: Each bounce loses energy by factor e². Vertical KE → 0 as bounces → ∞. Complete rest achieved.

**Edge/corner contact** (cube tilted so edge or corner contacts first):
- Contact point is offset from center of mass
- Lever arm `r = contact_point - CoM` is non-zero
- Collision couples linear and rotational motion: `τ = r × F`
- Energy transfers between translational and rotational modes
- **Result**: Energy "sloshes" between modes but total energy must still decrease monotonically with e < 1. Rest takes longer but is still achieved.

#### Friction Effects

**Without friction (μ = 0)**:
1. Impact and bounce
2. For edge/corner contact: convert some linear KE to rotational KE via off-center impulse
3. Lose **normal-direction** energy on each bounce due to coefficient of restitution (e < 1)
4. **Normal energy approaches zero**; tangential/rotational energy remains constant (or converts between modes) but **does not grow**
5. Tangential sliding and rotation about any axis may persist indefinitely

**With friction (μ > 0)**:
1. All of the above, plus:
2. Tangential velocity damped by friction forces
3. Rotational energy about all axes eventually dissipated
4. Complete rest achieved (for e < 1)

### Observed Behavior

The cube does not stabilize. Specifically:
- **Jitter**: High-frequency, low-amplitude vibration at rest (suggests Baumgarte or mass matrix issues)
- **Energy gain**: Bounce height or velocity increases over time (suggests integration order or restitution logic errors)
- **Explosion**: Immediate large velocities or NaN (suggests Jacobian sign error or division by zero)

*(Classify which behavior is observed during investigation)*

### Expected Behavior

**Primary requirement**: Total mechanical energy should monotonically decrease (or stay constant for e = 1) until the system reaches equilibrium.

**Normal-direction stabilization**: Kinetic energy in the contact-normal direction must approach zero for e < 1.

**Energy conservation bound**: Energy must **never increase** across collision frames.

**Key invariant** (both axis-aligned and edge/corner): Regardless of contact geometry, each collision with e < 1 must remove energy from the system. The *rate* of energy loss varies with geometry (axis-aligned converges faster), but the *direction* (decreasing) must be preserved.

---

## Hypothesis Space

This section catalogs potential root causes organized by collision pipeline component, from detection through response.

### H1: Collision Detection Issues

| ID | Hypothesis | Component | Likelihood | Impact |
|----|-----------|-----------|------------|--------|
| H1.1 | GJK/EPA returns incorrect penetration depth | `GJK.cpp`, `EPA.cpp` | Low | High |
| H1.2 | Contact normal direction inconsistent (flips frame-to-frame) | `EPA.cpp`, `CollisionHandler.cpp` | Medium | High |
| H1.3 | Witness points not stable for corner/edge contacts | `EPA.cpp` | Medium | Medium |
| H1.4 | Contact manifold generation misses contact points | `ContactManifold.cpp` | Low | Medium |

### H2: Constraint Assembly Issues

| ID | Hypothesis | Component | Likelihood | Impact |
|----|-----------|-----------|------------|--------|
| H2.1 | Lever arm computation incorrect for rotated bodies | `ContactConstraintFactory.cpp` | Medium | High |
| H2.2 | Pre-impact velocity captured at wrong time | `ContactConstraintFactory.cpp` | Low | High |
| H2.3 | Angular velocity from quaternion rate incorrect | `InertialState.cpp` | Low | High |
| H2.4 | Jacobian sign convention error in angular terms | `ContactConstraint.cpp` | Low | High |

### H3: Solver Issues

| ID | Hypothesis | Component | Likelihood | Impact |
|----|-----------|-----------|------------|--------|
| H3.1 | Effective mass matrix assembly incorrect for two-body rotation | `ConstraintSolver.cpp` | Medium | High |
| H3.2 | RHS restitution term sign or magnitude incorrect | `ConstraintSolver.cpp` | Low | High |
| H3.3 | Baumgarte stabilization injects energy | `ConstraintSolver.cpp`, `ContactConstraint.cpp` | Medium | High |
| H3.4 | Active Set Method converges to wrong partition | `ConstraintSolver.cpp` | Low | Medium |
| H3.5 | Regularization epsilon too large | `ConstraintSolver.cpp` | Low | Low |

### H4: Force Application Issues

| ID | Hypothesis | Component | Likelihood | Impact |
|----|-----------|-----------|------------|--------|
| H4.1 | Impulse-to-force conversion timing incorrect | `CollisionPipeline.cpp` | Medium | High |
| H4.2 | Torque applied in wrong frame (body vs world) | `AssetInertial.cpp` | Low | High |
| H4.3 | Force/torque double-applied or partially applied | `CollisionPipeline.cpp` | Low | High |
| H4.4 | Inertia tensor not properly transformed to world frame | `AssetInertial.cpp` | Low | High |

### H5: Time Integration Issues

| ID | Hypothesis | Component | Likelihood | Impact |
|----|-----------|-----------|------------|--------|
| H5.1 | Collision forces applied at wrong integration phase | `WorldModel.cpp` | Medium | High |
| H5.2 | Quaternion integration drift not properly constrained | `QuaternionConstraint.cpp` | Low | Medium |
| H5.3 | Semi-implicit Euler causes energy drift | `WorldModel.cpp` | Low | Medium |

### H6: Multi-Contact Stability Issues

| ID | Hypothesis | Component | Likelihood | Impact |
|----|-----------|-----------|------------|--------|
| H6.1 | Contact manifold flickers (contacts appear/disappear) | `ContactManifold.cpp` | Medium | High |
| H6.2 | Constraint activation threshold (0.01m) too large/small | `ContactConstraint.cpp` | Low | Medium |
| H6.3 | Rest velocity threshold (0.5 m/s) triggers incorrectly | `ContactConstraintFactory.cpp` | Low | Medium |

### H7: Solver State and Warm-Starting Issues

| ID | Hypothesis | Component | Likelihood | Impact |
|----|-----------|-----------|------------|--------|
| H7.1 | Warm-start impulses not cleared when contact breaks or changes orientation | `ConstraintSolver.cpp` | Medium | High |
| H7.2 | Warm-start impulses applied before collision check, affecting penetration | `CollisionPipeline.cpp` | Low | High |
| H7.3 | Solver iteration count too low, leaving residual error for Baumgarte (spring oscillation) | `ConstraintSolver.cpp` | Low | Medium |
| H7.4 | Constraint ordering bias causes artificial drift/rotation | `ConstraintSolver.cpp` | Low | Low |

### H8: Restitution Threshold Issues

| ID | Hypothesis | Component | Likelihood | Impact |
|----|-----------|-----------|------------|--------|
| H8.1 | No velocity threshold for restitution clamping (micro-bounces from gravity) | `ContactConstraintFactory.cpp` | Medium | High |
| H8.2 | Restitution threshold (0.5 m/s) set incorrectly relative to gravity×dt | `ContactConstraintFactory.cpp` | Low | Medium |

---

## Investigation Plan

### Phase 1: Instrumentation and Data Collection

Create diagnostic tools to capture per-frame collision pipeline data:

1. **Energy tracking**: Total kinetic energy (linear + rotational) per body per frame
   - **CRITICAL**: Rotational KE must use world-frame inertia tensor: `KE_rot = ½ ω^T (R I_body R^T) ω`
   - Using body-frame tensor with world angular velocity will produce chaotic-looking energy even if physics are correct
2. **Constraint state logging**: For each contact: normal, penetration depth, pre/post velocities, lambda value
3. **Contact persistence tracking**: Which contacts persist frame-to-frame vs. flicker
4. **Force audit trail**: Track forces from solver output through to applied torques
5. **Visual debugging** (recommended): Draw contact normals (red), applied impulses (green, scaled), contact points (blue spheres)
   - Normal flicker of even 5 degrees between frames is immediately visible in visualization but easy to miss in logs

### Phase 2: Systematic Test Scenarios

Design unit tests that isolate each pipeline component using first-principles physics.

#### Scenario Category A: Linear Collision (No Rotation)

These scenarios validate the collision system with pure translational motion, eliminating rotational coupling.

| Test ID | Scenario | Expected Outcome | Validates |
|---------|----------|------------------|-----------|
| A1 | Sphere drops vertically onto horizontal plane | Bounces with decreasing height until rest | Basic restitution, Baumgarte |
| A2 | Sphere with e=0 drops vertically | Single impact, immediate rest | Inelastic collision handling |
| A3 | Sphere with e=1 drops vertically | Perpetual bouncing at constant height | Energy conservation for elastic |
| A4 | Two spheres, head-on, equal mass, e=1 | Velocity swap | Momentum conservation |
| A5 | Two spheres, head-on, unequal mass (10:1), e=1 | Classical mechanics velocity formulas | Mass ratio handling |

#### Scenario Category B: Rotation Initiation (Angled Impact)

These scenarios validate that rotational energy is correctly initiated by off-center contacts.

| Test ID | Scenario | Expected Outcome | Validates |
|---------|----------|------------------|-----------|
| B1 | Cube corner impact on floor at 45-degree | Rotation initiated, translates and spins | Lever arm coupling |
| B2 | Cube edge impact (one edge parallel to floor) | Predictable rotation axis | Edge contact handling |
| B3 | Sphere with offset COM drops vertically | No rotation (symmetric contact) | Lever arm = 0 for symmetric |
| B4 | Rod (elongated cube) falls flat | No rotation (contact point at COM level) | Torque arm validation |

#### Scenario Category C: Rotation Damping (Energy Dissipation)

These scenarios validate that rotational energy is properly dissipated.

| Test ID | Scenario | Expected Outcome | Validates |
|---------|----------|------------------|-----------|
| C1 | Spinning cube on flat floor, initial ω, e=0 | Eventually stops rotating | Friction or contact damping |
| C2 | Rocking cube (corner pivot), e=0.5 | Rocking amplitude decreases | Rotational restitution |
| C3 | Tilted cube released from rest on floor | Settles to flat face | Multi-contact stability |
| C4 | Cube dropped flat with initial horizontal v | Slides then stops (with friction) or perpetual slide (without) | Friction integration |

#### Scenario Category D: Contact Manifold Stability

These scenarios validate contact persistence and manifold stability.

| Test ID | Scenario | Expected Outcome | Validates |
|---------|----------|------------------|-----------|
| D1 | Cube resting flat on floor for 1000 frames | Position drift < 0.001m | Resting contact stability |
| D2 | Two stacked cubes resting for 1000 frames | Both stable, no interpenetration | Contact stacking |
| D3 | Cube in corner (floor + wall) resting | Stable with 2+ contacts | Multi-surface contacts |
| D4 | Cube with small jitter velocity on floor | Settles quickly, no amplification | Damping of micro-motions |

#### Scenario Category E: Edge Cases and Degenerate Configurations

These scenarios test robustness of the collision pipeline.

| Test ID | Scenario | Expected Outcome | Validates |
|---------|----------|------------------|-----------|
| E1 | Two cubes touching face-to-face (zero penetration) | No explosion, stable | Zero-penetration handling |
| E2 | High-velocity impact (100 m/s) | No numerical explosion, energy bounded | Numerical stability |
| E3 | Very light object (0.001 kg) on heavy floor | Proper bouncing, no instability | Mass ratio extremes |
| E4 | Cube with e=1.0 in enclosed box | Perpetual bouncing, energy constant | Energy conservation limit |
| E5 | Simultaneous 4-corner contact (cube lands flat) | All 4 contacts active, stable | Multi-contact consistency |

#### Scenario Category F: Energy Accounting

These scenarios explicitly verify energy conservation/dissipation properties.

| Test ID | Scenario | Expected Outcome | Validates |
|---------|----------|------------------|-----------|
| F1 | Free-falling cube (no collision) for 100 frames | KE + PE = constant | Integration baseline |
| F2 | Elastic bounce (e=1) on floor | Post-bounce KE = Pre-bounce KE | Elastic energy conservation |
| F3 | Inelastic bounce (e=0.5) on floor | Post-bounce KE = e^2 * Pre-bounce KE | Inelastic energy loss formula |
| F4 | Collision with rotation transfer | Total KE (linear + rotational) conserved/reduced | Rotational energy accounting |
| F5 | Multi-bounce sequence (e=0.8, 10 bounces) | Energy decreases monotonically | No energy injection |

#### Scenario Category G: Warm-Start Validation

These scenarios validate solver state management across contact changes.

| Test ID | Scenario | Expected Outcome | Validates |
|---------|----------|------------------|-----------|
| G1 | Cube bounces, teleport high mid-air, drop again | No "jump" or violent reaction on next impact | Cached impulse invalidation |
| G2 | Contact breaks and reforms with different normal | No energy spike on contact reformation | Normal-change handling |

#### Scenario Category H: Parameter Isolation

These scenarios isolate specific parameters to identify energy injection sources.

| Test ID | Scenario | Expected Outcome | Validates |
|---------|----------|------------------|-----------|
| H1 | Set Baumgarte bias (β) to 0.0 | Object sinks but energy doesn't increase | Baumgarte as energy source |
| H2 | Set restitution (e) to 0.0 | Immediate stabilization on contact | Restitution logic correctness |
| H3 | Set ERP to 0.0 (disable position correction) | Energy stable (object may penetrate) | Position correction as energy source |

---

## Diagnostic Tests to Implement

### Unit Tests for Component Isolation

These tests check individual components in isolation before integration.

```cpp
// Test Category: Jacobian Correctness
TEST(ContactConstraintJacobian, LinearComponentsSymmetric)
TEST(ContactConstraintJacobian, AngularComponentsCrossProduct)
TEST(ContactConstraintJacobian, FiniteDifferenceValidation)  // J * v_dot ≈ (C(q+ε*v) - C(q)) / ε

// Test Category: Lever Arm Computation
TEST(LeverArm, CubeCornerContact_CorrectArm)
TEST(LeverArm, EdgeContact_CorrectArm)
TEST(LeverArm, FaceContact_ZeroTorqueArm)

// Test Category: Effective Mass Matrix
TEST(EffectiveMass, SingleBodyDiagonal_MatchesInverseMass)
TEST(EffectiveMass, TwoBody_SymmetricPositiveDefinite)
TEST(EffectiveMass, AngularCoupling_OffDiagonalCorrect)

// Test Category: RHS Assembly
TEST(ContactRHS, RestitutionTerm_CorrectSign)
TEST(ContactRHS, BaumgarteTerm_PenetrationProportional)
TEST(ContactRHS, ZeroVelocity_OnlyBaumgarte)

// Test Category: Solver Output
TEST(ASMSolver, SingleContact_AnalyticalLambda)
TEST(ASMSolver, TwoBodyRotation_MomentumConserved)
TEST(ASMSolver, EnergyNotIncreased_PostSolve)

// Test Category: Force Application
TEST(ForceApplication, LinearForce_NewtonSecondLaw)
TEST(ForceApplication, Torque_EulerEquation)
TEST(ForceApplication, ImpulseToVelocityChange_Correct)
```

### Integration Tests for End-to-End Validation

```cpp
// Test Category: Energy Tracking
TEST(EnergyIntegration, ElasticCollision_KE_Conserved)
TEST(EnergyIntegration, InelasticSequence_KE_Monotonic_Decrease)
TEST(EnergyIntegration, RestingContact_NoEnergyInjection)

// Test Category: Rotation Damping
TEST(RotationDamping, AngleDrop_EventualRest)
TEST(RotationDamping, InitialSpin_DampedToZero)
TEST(RotationDamping, RockingCube_AmplitudeDecreases)
```

---

## Systematic Elimination Protocol

### Step 1: Baseline Energy Measurement

1. Create `EnergyTracker` utility that computes total mechanical energy:
   - Linear KE: `½ m v²`
   - Rotational KE: `½ ω^T I ω`
   - Gravitational PE: `m g h`
2. Log energy at start and end of each `WorldModel::update()` call
3. Identify frames where energy increases (energy injection)

### Step 2: Isolate Collision vs. Integration

1. Disable collision pipeline entirely → verify integration-only energy conservation
2. Re-enable collision → identify if energy injection correlates with collision frames

### Step 3: Bisect Collision Pipeline Phases

For each collision frame:
1. Log state before Phase 1 (detection)
2. Log constraint parameters after Phase 2 (constraint creation)
3. Log solver output after Phase 4 (solve)
4. Log state after Phase 5 (force application)
5. Identify which phase introduces energy discrepancy

### Step 4: Component-Level Debugging

Based on Step 3 findings, drill into specific component:
- If detection: Add EPA visualization, check normal stability
- If constraint assembly: Validate Jacobian with finite differences
- If solver: Verify KKT conditions post-solve
- If force application: Verify impulse matches expected delta-v

---

## Files to Investigate

### Primary Suspects (ordered by investigation priority)

| Priority | File | Reason |
|----------|------|--------|
| 1 | `msd-sim/src/Physics/Collision/CollisionPipeline.cpp` | Orchestrates all phases; force application timing |
| 2 | `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | RHS assembly, effective mass, force extraction |
| 3 | `msd-sim/src/Physics/Constraints/ContactConstraint.cpp` | Jacobian computation, evaluation |
| 4 | `msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` | Pre-impact velocity, lever arms |
| 5 | `msd-sim/src/Environment/WorldModel.cpp` | Integration order, constraint timing |
| 6 | `msd-sim/src/Physics/RigidBody/AssetInertial.cpp` | Force/torque application, inertia tensor |

### Secondary (check if primary investigation inconclusive)

| File | Reason |
|------|--------|
| `msd-sim/src/Physics/RigidBody/InertialState.cpp` | Angular velocity from quaternion |
| `msd-sim/src/Physics/Collision/EPA.cpp` | Penetration depth, normal stability |
| `msd-sim/src/Physics/Collision/ContactManifold.cpp` | Multi-point contact generation |

---

## Acceptance Criteria

1. [ ] **AC1**: Root cause(s) identified and documented with evidence
2. [ ] **AC2**: At least one failing test case that reproduces the observed behavior
3. [ ] **AC3**: Energy tracking utility implemented and integrated
4. [ ] **AC4**: All Scenario Category A tests pass (linear collision baseline)
5. [ ] **AC5**: All Scenario Category F tests pass (energy accounting)
6. [ ] **AC6**: Fix implemented that causes angled cube drop to eventually stabilize
7. [ ] **AC7**: No regression in existing collision tests (`WorldModelContactIntegrationTest`, `ConstraintSolverContactTest`, `ConstraintSolverASMTest`)
8. [ ] **AC8**: Energy injection cases eliminated (energy never increases across collision frames for e ≤ 1)

---

## Deliverables

1. **Debug Report**: Document findings from systematic elimination
2. **Energy Tracking Utility**: Reusable diagnostic tool for future physics debugging
3. **Test Suite**: Unit tests from Scenarios A-F that pass post-fix
4. **Fix Implementation**: Code changes that resolve the root cause
5. **Regression Tests**: Tests that prevent recurrence of the bug

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Multiple interacting bugs | Medium | High | Systematic isolation via test scenarios |
| Fix causes regression elsewhere | Medium | Medium | Run full test suite before/after |
| Root cause in fundamental algorithm | Low | High | May require architectural changes; document trade-offs |
| Performance degradation from fix | Low | Low | Benchmark before/after |

---

## Priority Investigation Order

Based on common physics engine bugs and the specific symptom (angled cube not stabilizing), investigate in this order:

### Priority 1: Baumgarte Stabilization (H3.3)
**Action**: Set Baumgarte bias/ERP to 0.0 temporarily. If energy stabilizes (even if object sinks), tune the bias or clamp the correction impulse.
**Rationale**: This fixes ~60% of energy injection bugs in physics engines.

### Priority 2: Integration Phase Order (H5.1)
**Action**: Verify the update loop order matches:
- **Good**: `Integrate External Forces → Detect Collision → Solve Constraints → Integrate Position`
- **Bad**: `Detect → Integrate → Solve` or `Solve → Integrate → Detect`

### Priority 3: Jacobian Cross Product Sign (H2.1 / H4.2)
**Action**: Verify:
- `r × n` direction is correct in Jacobian
- `ApplyImpulse` computes `torque += cross(r, impulse)` with correct sign
- A sign flip causes object to spin *into* the floor, creating feedback loop

### Priority 4: EPA Witness Point Stability (H1.3)
**Action**: If contact point jitters along cube edge, lever arm changes rapidly, pumping energy into rotation.
**Check**: Log contact point position frame-to-frame for variance.

### Priority 5: Pre-Impact Velocity Timing (H2.2)
**Action**: Verify that when body rotates, the velocity at contact point (`v + ω × r`) uses consistent state.
**Check**: If position/rotation updated *before* collision detection, but *old* velocities used for constraints (or vice versa), this creates a time-step mismatch that acts like negative damping (adds energy).

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-05
- **Notes**: Ticket created to investigate unexpected collision behavior where cube dropped at angle does not stabilize. Comprehensive hypothesis space and test scenarios defined.

### Gemini Review (2026-02-05)
- **Rating**: 9/10 (Excellent)
- **Status**: Amendments incorporated
- **Key feedback incorporated**:
  1. Clarified frictionless vs frictional expected behavior in Problem Statement
  2. Added H7 (Warm-Starting) and H8 (Restitution Threshold) hypothesis categories
  3. Added Scenario Categories G (Warm-Start Validation) and H (Parameter Isolation)
  4. Added world-frame inertia tensor note for energy calculation
  5. Added visual debugging recommendation
  6. Added Priority Investigation Order based on common physics engine bugs
  7. Clarified that horizontal/rotational energy persists without friction

### Physics Clarification Update (2026-02-05)
- Added distinction between axis-aligned and edge/corner contact scenarios
- Clarified that axis-aligned contacts are effectively 1D and converge faster
- Added "Key invariant" emphasizing energy must decrease regardless of contact geometry

### Subticket Split (2026-02-05)
- Split into 5 sequential subtickets (0038a-0038e) for manageable scope
- 0038a: Diagnostic infrastructure (EnergyTracker, logging)
- 0038b: Linear collision test suite (baseline validation)
- 0038c: Rotational coupling test suite (bug reproduction)
- 0038d: Parameter isolation and root cause identification
- 0038e: Fix implementation and regression testing

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

