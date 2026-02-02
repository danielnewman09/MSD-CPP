# Ticket 0035d5: Two-Phase Friction Solve (ASM Normals + ECOS Centroid Friction)

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Quality Gate
**Assignee**: Claude Opus 4.5
**Created**: 2026-02-01
**Type**: Refactor (Architecture change)
**Requires Math Design**: No
**Generate Tutorial**: No
**Parent Ticket**: [0035d_friction_hardening_and_validation](0035d_friction_hardening_and_validation.md)

---

## Summary

Restructure the friction contact solving pipeline to split into two phases:

1. **Phase A (ASM)**: Solve N normal constraints per collision pair using the Active Set Method. This enforces non-penetration at all contact manifold points and handles rank deficiency gracefully.

2. **Phase B (ECOS)**: Solve 1 centroid normal + 1 centroid friction per collision pair using ECOS. This produces a well-conditioned 3×3 system that reliably computes friction forces within the Coulomb cone.

Forces from both phases are combined, with the centroid normal force excluded to avoid double-counting with the ASM normal forces.

---

## Problem Statement

### Current Architecture

`WorldModel::updateCollisions()` creates a single flat constraint list (N normals + N frictions per collision pair) and passes it all to `solveWithContacts()`. When friction is detected, the entire system (including all normals) is routed to ECOS. For 4-contact collisions, ECOS receives a 12×12 ill-conditioned system.

### Target Architecture

```
WorldModel creates:
  normalConstraints:  [CC0, CC1, CC2, CC3]           → ASM (4×4, well-conditioned)
  frictionPairs:      [CentroidCC, CentroidFC]        → ECOS (3×3, well-conditioned)

Results combined:
  bodyForces = ASM_normalForces + ECOS_frictionOnlyForces
```

---

## Technical Approach

### Step 1: Add `extractFrictionOnlyBodyForces()` to ConstraintSolver

**File**: `ConstraintSolver.hpp/.cpp`

New public method that extracts body forces from a solved lambda vector, but zeroes out any `ContactConstraint` lambdas before extraction. This way only `FrictionConstraint` forces contribute to the output.

```cpp
std::vector<BodyForces> extractFrictionOnlyBodyForces(
    const std::vector<TwoBodyConstraint*>& constraints,
    const std::vector<std::reference_wrapper<const InertialState>>& states,
    const Eigen::VectorXd& lambda,
    size_t numBodies,
    double dt) const;
```

**Implementation:**
1. Call `assembleContactJacobians(constraints, states)` to get Jacobians
2. Clone the lambda vector
3. Zero out entries corresponding to `ContactConstraint` instances (identified via `dynamic_cast`)
4. Call `extractContactBodyForces(constraints, jacobians, modifiedLambda, numBodies, dt)`
5. Return the result

### Step 2: Restructure WorldModel Phase 2 — Constraint Creation

**File**: `WorldModel.cpp` (lines 258-309)

Replace the current single-list creation with two lists:

```cpp
std::vector<std::unique_ptr<TwoBodyConstraint>> normalConstraints;
std::vector<std::unique_ptr<TwoBodyConstraint>> frictionPairs;

for (const auto& pair : collisions) {
  // ... (state/COM lookup unchanged) ...

  // N normal constraints for ASM
  auto contacts = ContactConstraintFactory::createFromCollision(
      pair.bodyAIndex, pair.bodyBIndex, pair.result,
      stateA, stateB, comA, comB, pair.restitution);
  for (auto& c : contacts) {
    normalConstraints.push_back(std::move(c));
  }

  // 1 centroid normal + 1 centroid friction for ECOS (if μ > 0)
  auto centroidNormal = ContactConstraintFactory::createCentroidContactConstraint(
      pair.bodyAIndex, pair.bodyBIndex, pair.result,
      stateA, stateB, comA, comB, pair.restitution);
  auto centroidFriction = ContactConstraintFactory::createCentroidFrictionConstraint(
      pair.bodyAIndex, pair.bodyBIndex, pair.result,
      comA, comB, pair.frictionCoefficientA, pair.frictionCoefficientB);

  if (centroidNormal && centroidFriction) {
    frictionPairs.push_back(std::move(centroidNormal));   // Must be first (ECOS cone ordering)
    frictionPairs.push_back(std::move(centroidFriction));
  }
}
```

### Step 3: Restructure WorldModel Phase 4 — Two-Phase Solve

**Phase 4a**: Solve normals via ASM
```cpp
// Build normalPtrs from normalConstraints
auto normalResult = contactSolver_.solveWithContacts(
    normalPtrs, states, inverseMasses, inverseInertias, numBodies, dt);
// No friction detected → dispatches to ASM automatically
```

**Phase 4b**: Solve friction via ECOS
```cpp
if (!frictionPairs.empty()) {
  // Build frictionPtrs from frictionPairs
  auto frictionResult = contactSolver_.solveWithContacts(
      frictionPtrs, states, inverseMasses, inverseInertias, numBodies, dt);
  // FrictionConstraint detected → dispatches to ECOS

  // Extract friction-only body forces (zeroes centroid normal contribution)
  auto frictionBodyForces = contactSolver_.extractFrictionOnlyBodyForces(
      frictionPtrs, states, frictionResult.lambdas, numBodies, dt);
}
```

### Step 4: Restructure WorldModel Phase 5 — Force Application

```cpp
for (size_t k = 0; k < numInertial; ++k) {
  Coordinate totalLinear{0.0, 0.0, 0.0};
  Coordinate totalAngular{0.0, 0.0, 0.0};

  // Add ASM normal forces
  totalLinear += normalResult.bodyForces[k].linearForce;
  totalAngular += normalResult.bodyForces[k].angularTorque;

  // Add ECOS friction-only forces
  if (!frictionPairs.empty()) {
    totalLinear += frictionBodyForces[k].linearForce;
    totalAngular += frictionBodyForces[k].angularTorque;
  }

  if (totalLinear.norm() > 1e-12 || totalAngular.norm() > 1e-12) {
    inertialAssets_[k].applyForce(totalLinear);
    inertialAssets_[k].applyTorque(totalAngular);
  }
}
```

### Key Constraint: Ordering for ECOS

The friction pair list must have `CentroidCC` before `CentroidFC` for each collision pair. This ensures `buildFrictionConeSpec()` (fixed in 0035d3) correctly maps `normalIdx = 0` for the first friction cone.

For multiple collision pairs:
```
frictionPairs = [CentroidCC_pair0, CentroidFC_pair0, CentroidCC_pair1, CentroidFC_pair1, ...]
```

Lambda rows: `[λ_n0, λ_t1_0, λ_t2_0, λ_n1, λ_t1_1, λ_t2_1, ...]` — naturally interleaved since each pair contributes [1 normal, 2 tangential].

---

## Files

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Add `extractFrictionOnlyBodyForces()` declaration |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Implement `extractFrictionOnlyBodyForces()` |
| `msd-sim/src/Environment/WorldModel.cpp` | Restructure Phase 2 (two lists), Phase 4 (two-phase solve), Phase 5 (combine forces) |

### Test Files (Validation)

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/FrictionEnergyTest.cpp` | Energy monotonicity — AC1 |
| `msd-sim/test/Physics/FrictionValidationTest.cpp` | Physics correctness — AC3 |
| `msd-sim/test/Physics/FrictionStabilityTest.cpp` | Numerical robustness — AC4 |

---

## Acceptance Criteria

- [ ] **AC1**: `FrictionEnergyTest::EnergyMonotonicDecreaseForSliding` passes (energy does not inject)
- [ ] **AC2**: Cube sliding on plane decelerates and stays on surface (no launch)
- [ ] **AC3**: Friction impulse opposes tangential velocity (`dv_y ≈ 0`, `dv_z ≈ 0` for horizontal sliding)
- [ ] **AC4**: ECOS solver reports `ECOS_OPTIMAL` for standard friction scenarios
- [ ] **AC5**: 3×3 effective mass matrix condition number < 1e6
- [ ] **AC6**: All existing non-friction tests pass (zero regressions on ASM path)
- [ ] **AC7**: All `FrictionValidationTest` and `FrictionStabilityTest` cases pass

---

## Dependencies

- **Requires**: [0035d3](0035d3_friction_solver_cleanup.md) — Ordering bug fix (must be in place before ECOS receives centroid constraints)
- **Requires**: [0035d4](0035d4_centroid_friction_factory.md) — Centroid factory methods (consumed by WorldModel)
- **Blocks**: [0035d](0035d_friction_hardening_and_validation.md) AC1 (energy monotonicity)

---

## Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Two-phase solve produces different normal forces than single solve | Medium | Medium | Normal forces from ASM are the same as current (ASM path unchanged). Friction is additive. |
| Centroid normal in ECOS produces different λ_n than sum of ASM normals | Medium | Low | Acceptable — ECOS λ_n only bounds friction cone, not applied as body force. |
| Multiple collision pairs produce larger ECOS system (3K×3K) | Low | Low | K collision pairs × 3 rows each. Typical K < 5, so 15×15 max — still well-conditioned. |
| ASM and ECOS solve interference (forces from one affect the other) | Low | Medium | Solves are independent. In practice, normal and friction forces are nearly orthogonal (normal ⊥ tangent), so no interference. |

---

## Physical Justification

The two-phase approach is physically sound:

1. **Normal forces (ASM)**: Prevent interpenetration at each contact point independently. The full N-point manifold ensures torque stability (e.g., prevents a resting cube from rocking).

2. **Friction force (ECOS)**: Opposes tangential sliding at the aggregate contact patch center. A single centroid friction is a good approximation because:
   - All N contacts share the same normal and friction coefficient
   - The friction force direction depends on the tangential velocity at the contact, which is nearly identical across a small patch
   - The magnitude is bounded by μ times the centroid normal force, which approximates μ times the total normal force

3. **Force combination**: Normal forces from ASM act perpendicular to the contact surface. Friction forces from ECOS act parallel. They are nearly orthogonal and additive.

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-01
- **Notes**: Core integration ticket for the centroid reduction plan. Depends on 0035d3 (ordering fix) and 0035d4 (factory methods). This is the main architectural change that resolves the ill-conditioned mass matrix problem.

### Implementation Phase
- **Started**: 2026-02-01
- **Completed**: 2026-02-01
- **Artifacts**:
  - Modified: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` — Added extractFrictionOnlyBodyForces() public method
  - Modified: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` — Implemented extractFrictionOnlyBodyForces() with lambda zeroing
  - Modified: `msd/msd-sim/src/Environment/WorldModel.cpp` — Restructured Phase 2 (two lists), Phase 4 (two-phase solve), Phase 5 (force combination)
- **Changes**:
  - **Step 1**: Added `extractFrictionOnlyBodyForces()` to ConstraintSolver that zeroes ContactConstraint lambdas before force extraction
  - **Step 2**: Restructured WorldModel Phase 2 to create two separate constraint lists:
    - `normalConstraints`: N normals per collision pair (for ASM)
    - `frictionPairs`: 1 centroid normal + 1 centroid friction per pair (for ECOS)
  - **Step 3**: Restructured WorldModel Phase 4 into two-phase solve:
    - Phase 4a: ASM solves normal constraints (N per collision)
    - Phase 4b: ECOS solves friction pairs (if present), extracts friction-only forces
  - **Step 4**: Restructured WorldModel Phase 5 to combine forces:
    - Accumulates ASM normal forces + ECOS friction-only forces per body
    - Applies combined forces to inertial assets
  - **Ordering**: frictionPairs maintains ECOS cone ordering (centroid normal before centroid friction)
- **Test Results**:
  - Contact tests: 84/86 passing (2 pre-existing ECOS failures unrelated to this ticket)
  - Friction tests: 60/65 passing (5 pre-existing failures documented in DEBUG_0035d_friction_energy_injection.md)
  - Zero new regressions introduced
  - Acceptance criteria validation deferred to quality gate phase
- **Notes**:
  - Implementation follows ticket Technical Approach exactly
  - Dependencies (0035d3, 0035d4) are implemented and available
  - Two-phase architecture cleanly separates ASM normal solve from ECOS friction solve
  - extractFrictionOnlyBodyForces() enables combining forces from independent solves without double-counting normals
