# Ticket 0040b: Split Impulse Position Correction

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Implementation
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Design Complete — Awaiting Review
**Assignee**: TBD
**Created**: 2026-02-07
**Generate Tutorial**: No
**Parent Ticket**: [0040_collision_stabilization_phase2](0040_collision_stabilization_phase2.md)
**Dependencies**: [0040a](0040a_per_contact_penetration_depth.md)
**Blocks**: [0040d](0040d_contact_persistence_warm_starting.md)
**Type**: Implementation

---

## Overview

Replace Baumgarte stabilization with split impulse (pseudo-velocity) position correction. This is the highest-impact change in the 0040 series, targeting ~9 of the 11 remaining test failures.

Baumgarte stabilization mixes position correction with velocity correction by adding a bias term `(ERP/dt) * penetration` to the constraint's right-hand side. This correction velocity becomes real kinetic energy that persists after contact separation, creating the primary source of energy injection in the current system.

Split impulse decouples position correction from velocity correction by solving them in separate passes, preventing position correction from injecting energy into the velocity state.

---

## Problem

### Current Behavior (Baumgarte)

The constraint solver assembles a single RHS that combines restitution and position correction:

```
b = -(1 + e) * v_rel  +  (ERP / dt) * penetrationDepth
     └── restitution ──┘  └── Baumgarte bias ──────────┘
```

The solver computes `λ` from `A·λ = b`, then applies `λ` as a velocity change:

```
Δv = M⁻¹ · Jᵀ · λ
```

The Baumgarte component of `λ` becomes a **real velocity change** that persists as kinetic energy. For resting contacts or rocking objects, this creates a positive energy feedback loop.

### Desired Behavior (Split Impulse)

Separate the solve into two passes:

**Pass 1 — Velocity solve** (affects real velocity state):
```
b_vel = -(1 + e) * v_rel
λ_vel = solve(A, b_vel)
Δv = M⁻¹ · Jᵀ · λ_vel
```

**Pass 2 — Position correction** (uses pseudo-velocities, does NOT affect real velocity):
```
b_pos = (β / dt) * max(penetration - slop, 0)
λ_pos = solve(A, b_pos)
Δx = M⁻¹ · Jᵀ · λ_pos * dt   // Apply as position change, not velocity
```

The pseudo-velocity from position correction is discarded after the position update and never becomes kinetic energy.

---

## Requirements

### R1: Remove Baumgarte Bias from Velocity RHS

Modify `ConstraintSolver` to assemble the velocity RHS without the Baumgarte term:

```cpp
// BEFORE (Baumgarte):
b[i] = -(1 + e) * v_rel + (ERP / dt) * depth;

// AFTER (Split Impulse):
b[i] = -(1 + e) * v_rel;  // Velocity solve only
```

### R2: Implement Position Correction Pass

Create a `PositionCorrector` class that:

1. Takes the current contact manifold and body states
2. Computes position-level constraint violations (penetration depth per contact from 0040a)
3. Solves for position correction impulses using the same effective mass matrix
4. Applies corrections directly to positions (and orientations for angular correction) without modifying velocities
5. Uses a slop parameter (e.g., 0.005m) to allow small penetrations without correction (prevents jitter)
6. Uses a position correction factor β (e.g., 0.2) to avoid overcorrection

### R3: Integrate Position Correction into WorldModel

Call position correction after the velocity solve in `WorldModel::updateCollisions()`:

```
1. Detect collisions
2. Build constraints
3. Velocity solve (restitution only, no Baumgarte)
4. Apply velocity changes
5. Position correction pass (pseudo-velocity, affects positions only)
```

### R4: Penetration Slop Parameter

Add a configurable slop parameter to avoid correcting micro-penetrations:

```cpp
float positionSlop = 0.005f;  // Allow 5mm penetration without correction
float correctedDepth = std::max(depth - positionSlop, 0.0f);
```

This prevents the position corrector from fighting numerical noise and creating jitter for resting contacts.

### R5: Energy Conservation Guarantee

After split impulse implementation, the strict energy invariant should hold:

```
E_post ≤ E_pre  (for e ≤ 1)
```

The position correction pass does not inject energy because pseudo-velocities are discarded. The velocity solve only includes restitution, which by definition does not add energy for `e ≤ 1`.

---

## Affected Tests

This change directly targets:

| Test | How Split Impulse Fixes It |
|------|---------------------------|
| H1_DisableRestitution_RestingCube | No Baumgarte = no energy injection at rest |
| H8_TiltedCube_FeedbackLoop | Position correction doesn't create real angular velocity |
| C2_RockingCube_AmplitudeDecreases | No energy injection through offset contacts |
| C3_TiltedCubeSettles_ToFlatFace | Position correction pushes to flat without adding KE |
| D1_RestingCube_1000Frames | No compounding energy from position correction |
| D4_SmallJitter_NoAmplification | Slop parameter absorbs micro-penetrations |
| F4_RotationEnergyTransfer | Per-contact depth (0040a) + no energy injection |
| F4b_ZeroGravity_RotationalEnergyTransfer | Same mechanism without gravity |
| B1_CubeCornerImpact | No energy injection through corner lever arm |

---

## Implementation Approach

### Step 1: Modify ConstraintSolver RHS Assembly

Remove the Baumgarte bias term from the RHS assembly in `ConstraintSolver.cpp`. The RHS should only contain the restitution term.

### Step 2: Create PositionCorrector

```cpp
// msd-sim/src/Physics/Constraints/PositionCorrector.hpp

namespace msd_sim {

class PositionCorrector {
public:
  struct Config {
    float beta{0.2f};         // Position correction factor
    float slop{0.005f};       // Penetration slop [m]
    int maxIterations{4};     // Position correction iterations
  };

  /// Correct positions to resolve penetration without affecting velocities
  /// @param contacts Active contact constraints with per-contact depth
  /// @param bodies Mutable body states for position/orientation updates
  /// @param config Position correction parameters
  static void correctPositions(
    const std::vector<ContactConstraintData>& contacts,
    std::span<AssetInertial> bodies,
    const Config& config = {});
};

} // namespace msd_sim
```

### Step 3: Integrate into WorldModel

In `WorldModel::updateCollisions()`, after the velocity solve:

```cpp
void WorldModel::updateCollisions(double dt) {
  // 1. Detect collisions (existing)
  // 2. Build constraints (existing)
  // 3. Velocity solve — restitution only, no Baumgarte (MODIFIED)
  solver_.solve(constraints, dt);
  // 4. Apply velocity changes (existing)

  // 5. Position correction pass (NEW)
  PositionCorrector::correctPositions(contactData, inertialAssets_);
}
```

### Step 4: Remove ERP Clamp

The ERP velocity clamp added in 0039e (`max 5.0 m/s`) can be removed since Baumgarte is no longer in the velocity RHS. The position correction pass handles penetration resolution.

---

## Test Plan

### Unit Tests

```cpp
// Verify velocity RHS has no Baumgarte bias
TEST(SplitImpulse, VelocityRHS_NoBaumgarteBias)
// RHS for resting contact (v_rel ≈ 0) should be ≈ 0, not (ERP/dt)*depth

TEST(SplitImpulse, PositionCorrection_ReducesPenetration)
// After correction, penetration depth decreases

TEST(SplitImpulse, PositionCorrection_DoesNotChangeVelocity)
// Velocity state unchanged after position correction pass

TEST(SplitImpulse, Slop_NoCorrectionBelowThreshold)
// Penetration < slop → no position correction applied

TEST(SplitImpulse, EnergyConservation_RestingContact)
// Resting contact maintains E_post ≤ E_pre across 100 frames

TEST(SplitImpulse, EnergyConservation_RockingCube)
// Rocking cube energy monotonically decreases
```

### Integration Tests

All 0039b/0039c/0039d diagnostic tests should be re-run:

| Category | Expected Result |
|----------|----------------|
| A1-A6 (Linear) | All pass (no regression) |
| F1-F5 (Energy) | All pass (no regression) |
| B1 (Corner impact) | Pass (energy within 5% tolerance) |
| C2, C3 (Rocking/settling) | Pass (amplitude decreases, cube settles) |
| D1, D4 (Stability) | Pass (no drift, no amplification) |
| F4, F4b (Rotation energy) | Pass (energy conserved/decreased) |
| H1, H8 (Parameter isolation) | Pass (no energy injection) |

### Regression Tests

- All existing collision tests (`WorldModelContactIntegrationTest`, `ConstraintSolverContactTest`, `ConstraintSolverASMTest`)
- Stacking stability: 3-5 cube vertical stack stable for 500 frames (position correction must push objects apart without sinking)

---

## Acceptance Criteria

1. [ ] **AC1**: Baumgarte bias removed from velocity RHS in `ConstraintSolver`
2. [ ] **AC2**: `PositionCorrector` class implemented with slop and beta parameters
3. [ ] **AC3**: Position correction integrated into `WorldModel::updateCollisions()`
4. [ ] **AC4**: E_post ≤ E_pre holds for all test scenarios with e ≤ 1
5. [ ] **AC5**: Resting cube energy stable for 1000 frames (D1 passes)
6. [ ] **AC6**: Rocking cube amplitude decreases (C2 passes)
7. [ ] **AC7**: Tilted cube settles to flat face (C3 passes)
8. [ ] **AC8**: No regression in existing collision tests
9. [ ] **AC9**: Stacking test passes (no sinking)

---

## Files to Modify

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Remove Baumgarte bias from velocity RHS |
| `msd-sim/src/Environment/WorldModel.cpp` | Add position correction call after velocity solve |

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/src/Physics/Constraints/PositionCorrector.hpp` | Position correction class declaration |
| `msd-sim/src/Physics/Constraints/PositionCorrector.cpp` | Position correction implementation |
| `msd-sim/test/Physics/Constraints/SplitImpulseTest.cpp` | Unit tests for split impulse |

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Objects sink through floor (correction too weak) | Medium | High | Tune β and slop parameters; stacking regression test |
| Jitter at rest (correction oscillates) | Medium | Medium | Slop parameter absorbs micro-penetrations |
| Solver convergence changes | Low | Medium | Same effective mass matrix, only RHS changes |
| Performance regression from second pass | Low | Low | Position correction is lightweight (reuses mass matrix) |

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-07
- **Notes**: Highest-impact subticket of 0040. Replaces Baumgarte stabilization (identified as primary energy injection source in 0039d) with split impulse approach. Referenced in 0039e as recommended follow-on.

### Design Phase
- **Started**: 2026-02-07
- **Completed**: 2026-02-07
- **Artifacts**:
  - `docs/designs/0040b-split-impulse-position-correction/design.md`
  - `docs/designs/0040b-split-impulse-position-correction/0040b-split-impulse-position-correction.puml`
- **Notes**: Thorough design covering three key changes: (1) Remove Baumgarte bias from ConstraintSolver::assembleContactRHS(), (2) New PositionCorrector class with beta/slop/maxIterations config, (3) Phase 6 position correction in WorldModel::updateCollisions(). Design includes energy conservation proof, parameter selection rationale, and 14 unit tests. ERP clamp from 0039e is removed. PositionCorrector reimplements Jacobian/effective mass assembly (same algorithm as ConstraintSolver) to maintain encapsulation.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
