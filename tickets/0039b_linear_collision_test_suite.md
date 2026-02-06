# Ticket 0038b: Linear Collision Test Suite (No Rotation)

## Status
- [x] Draft
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
**Parent Ticket**: [0038_collision_energy_stabilization_debug](0038_collision_energy_stabilization_debug.md)
**Dependencies**: [0038a_energy_tracking_diagnostic_infrastructure](0038a_energy_tracking_diagnostic_infrastructure.md)
**Type**: Test Suite

---

## Overview

This ticket creates a comprehensive test suite for **pure translational collisions** (no rotation). By eliminating rotational coupling, we establish a baseline for collision correctness. If these tests fail, the bug is fundamental to the collision system. If they pass, we can focus subsequent investigation on rotation-specific issues.

---

## Requirements

### R1: Scenario Category A — Linear Collision (No Rotation)

These tests validate the collision system with pure translational motion.

**IMPORTANT**: All tests in this suite MUST set **Friction (μ) = 0.0** to ensure pure linear behavior.

| Test ID | Scenario | Expected Outcome | Validates |
|---------|----------|------------------|-----------|
| A1 | Sphere drops vertically onto horizontal plane | Bounces with decreasing height until rest | Basic restitution, Baumgarte |
| A2 | Sphere with e=0 drops vertically | Velocity < 1e-4 after 2 frames | Inelastic collision handling |
| A3 | Sphere with e=1 drops vertically | Perpetual bouncing at constant height | Energy conservation (elastic) |
| A4 | Two spheres, head-on, equal mass, e=1 | Velocity swap | Momentum conservation |
| A5 | Two spheres, head-on, unequal mass (10:1), e=1 | Classical mechanics velocity formulas | Mass ratio handling |
| A6 | Two spheres, glancing collision at 45° angle, e=1 | 90° separation angle post-collision | Normal vector construction |

### R2: Scenario Category F — Energy Accounting (Linear Cases)

These tests explicitly verify energy conservation/dissipation for linear collisions.

| Test ID | Scenario | Expected Outcome | Validates |
|---------|----------|------------------|-----------|
| F1 | Free-falling sphere (no collision) for 100 frames | KE + PE = constant | Integration baseline |
| F2 | Elastic bounce (e=1) on floor | Post-bounce KE = Pre-bounce KE | Elastic energy conservation |
| F3 | Inelastic bounce (e=0.5) on floor | Post-bounce KE = e² × Pre-bounce KE | Inelastic energy loss formula |
| F5 | Multi-bounce sequence (e=0.8, 10 bounces) | Energy decreases monotonically | No energy injection |

### R3: Component Isolation Unit Tests

These tests check individual collision components in isolation.

#### R3.1: Jacobian Linear Components
```cpp
TEST(ContactConstraintJacobian, LinearComponentsSymmetric)
// Verify J_linear = [-n^T, n^T] for two-body contact
// Body A gets -n, Body B gets +n

TEST(ContactConstraintJacobian, LinearComponentsUnitNormal)
// Verify linear Jacobian components have unit magnitude
```

#### R3.2: Effective Mass Matrix (Linear Only)
```cpp
TEST(EffectiveMass, SingleBody_InverseMassOnly)
// For single body with no rotation: A = m_eff^{-1} = 1/m

TEST(EffectiveMass, TwoBody_EqualMass_HalfEffective)
// For two equal masses: m_eff = m/2
```

#### R3.3: RHS Assembly (Linear)
```cpp
TEST(ContactRHS, RestitutionTerm_CorrectSign)
// Verify b = -(1+e) * v_rel_normal

TEST(ContactRHS, ZeroVelocity_OnlyBaumgarte)
// When v_rel = 0, only Baumgarte term contributes
```

---

## Test Implementation Details

### A1: Sphere Drops Vertically (Inelastic Settling)

**Setup**:
- Sphere: mass = 1.0 kg, radius = 0.5 m, e = 0.7
- Initial position: (0, 0, 5) m
- Initial velocity: (0, 0, 0) m/s
- Floor: z = 0 plane

**Verification**:
1. Track bounce apex heights: h₀ = 5, h₁, h₂, ...
2. Each apex should satisfy: hₙ₊₁ ≈ e² × hₙ
3. After N bounces (N ≈ 20-30), sphere should be at rest (z ≈ 0.5 m)
4. Energy should decrease monotonically

**Pass Criteria**:
- Final position z within [0.499, 0.501] m (tightened per Gemini review)
- No energy increase detected across any frame
- Sphere comes to rest (velocity < 0.01 m/s) within 500 frames

### A2: Perfectly Inelastic (e=0)

**Setup**:
- Sphere: mass = 1.0 kg, radius = 0.5 m, e = 0.0
- Initial position: (0, 0, 2) m
- Initial velocity: (0, 0, 0) m/s

**Verification**:
1. First contact should result in near-zero rebound velocity
2. Sphere should remain at rest on floor

**Pass Criteria** (clarified per Gemini review):
- Post-collision velocity < 1e-4 m/s **after 2 frames** (allows for gravity re-application after resolve)
- Position stable at z = 0.5 m for 100 frames
- No jitter or micro-bounces

**Note**: "Immediate rest" is physically ideal but computationally difficult in discrete simulations due to integration order (gravity re-applied after velocity resolve).

### A3: Perfectly Elastic (e=1)

**Setup**:
- Sphere: mass = 1.0 kg, radius = 0.5 m, e = 1.0
- Initial position: (0, 0, 2) m
- Initial velocity: (0, 0, 0) m/s

**Verification**:
1. Track bounce apex heights over 50+ bounces
2. All apex heights should equal initial height (within tolerance)

**Pass Criteria**:
- Apex height variance < 1% of initial height
- Total energy variance < 0.1% over 1000 frames

### A4: Two Spheres, Equal Mass, Elastic (Velocity Swap)

**Setup**:
- Sphere A: mass = 1.0 kg, position = (-2, 0, 0.5), velocity = (2, 0, 0)
- Sphere B: mass = 1.0 kg, position = (0, 0, 0.5), velocity = (0, 0, 0)
- Both e = 1.0

**Verification**:
Classical 1D elastic collision:
- Post-collision: v_A = 0, v_B = 2 m/s

**Pass Criteria**:
- Post-collision v_A magnitude < 0.01 m/s
- Post-collision v_B.x within [1.99, 2.01] m/s
- Total momentum conserved within 0.1%
- Total KE conserved within 0.1%

### A5: Two Spheres, Unequal Mass (10:1)

**Setup**:
- Sphere A: mass = 10.0 kg, position = (-2, 0, 0.5), velocity = (1, 0, 0)
- Sphere B: mass = 1.0 kg, position = (0, 0, 0.5), velocity = (0, 0, 0)
- Both e = 1.0

**Verification**:
Classical formulas:
```
v_A' = ((m_A - m_B) / (m_A + m_B)) * v_A = (9/11) * 1 ≈ 0.818 m/s
v_B' = (2 * m_A / (m_A + m_B)) * v_A = (20/11) * 1 ≈ 1.818 m/s
```

**Pass Criteria**:
- v_A' within [0.81, 0.83] m/s
- v_B' within [1.81, 1.83] m/s
- Total momentum conserved within 0.1%
- Total KE conserved within 0.1%

### F1: Free Fall Energy Conservation

**Setup**:
- Sphere: mass = 1.0 kg, position = (0, 0, 10), velocity = (0, 0, 0)
- No floor (or floor at z = -100 to avoid collision)
- Gravity = 9.81 m/s²

**Verification**:
```
E_total = KE + PE = ½mv² + mgh
```
At t=0: E = 0 + 1.0 × 9.81 × 10 = 98.1 J
At any t: E should remain 98.1 J

**Pass Criteria**:
- Energy variance < 0.01% over 100 frames
- Validates that integration alone doesn't introduce energy error

### F2/F3: Elastic vs Inelastic Energy

**F2 Setup** (Elastic):
- Drop sphere from h = 2 m, e = 1.0
- Measure KE just before and just after collision

**F2 Pass Criteria**:
- KE_post / KE_pre within [0.99, 1.01]

**F3 Setup** (Inelastic):
- Drop sphere from h = 2 m, e = 0.5
- Measure KE just before and just after collision

**F3 Pass Criteria**:
- KE_post / KE_pre within [0.249, 0.251] (e² = 0.25, tightened per Gemini review)

### F5: Multi-Bounce Monotonic Decrease

**Setup**:
- Sphere: mass = 1.0 kg, e = 0.8
- Drop from h = 5 m
- Track energy for 10+ bounces

**Pass Criteria**:
- E(frame N+1) ≤ E(frame N) for all N
- No single frame shows energy increase > 0.001 J
- Final energy < 1% of initial energy

### A6: Glancing Collision (45° Angle) — NEW from Gemini review

**Setup**:
- Sphere A: mass = 1.0 kg, position = (-2, 0, 0.5), velocity = (2, 0, 0)
- Sphere B: mass = 1.0 kg, position = (0, 0.5, 0.5), velocity = (0, 0, 0)
  - Offset by 0.5m in Y to create glancing (off-center) impact
- Both e = 1.0, μ = 0.0

**Verification**:
- Impulse acts along line connecting sphere centers (contact normal)
- For equal mass elastic glancing collision: post-collision velocity vectors are perpendicular

**Pass Criteria**:
- Angle between v_A' and v_B' within [89°, 91°]
- Total momentum conserved within 0.1%
- Total KE conserved within 0.1%

**Rationale**: Validates that the Jacobian constructs the normal vector correctly, not just the impulse magnitude.

---

## Acceptance Criteria

1. [ ] **AC1**: All Scenario A tests (A1-A5) implemented and passing
2. [ ] **AC2**: All Scenario F linear tests (F1-F3, F5) implemented and passing
3. [ ] **AC3**: Component isolation tests (Jacobian linear, effective mass) passing
4. [ ] **AC4**: Test coverage for edge cases (e=0, e=1, mass ratios)
5. [ ] **AC5**: No energy increase detected in any linear collision test
6. [ ] **AC6**: Tests use EnergyTracker from 0038a for verification

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/Collision/LinearCollisionTest.cpp` | Scenario A tests |
| `msd-sim/test/Physics/Collision/EnergyAccountingTest.cpp` | Scenario F tests |
| `msd-sim/test/Physics/Constraints/JacobianLinearTest.cpp` | Jacobian unit tests |

### Modified Files
| File | Change |
|------|--------|
| `msd-sim/test/CMakeLists.txt` | Add new test files |

---

## Test Infrastructure Notes

### Using EnergyTracker

All tests should use the `EnergyTracker` utility from 0038a:

```cpp
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"

TEST(LinearCollision, A1_SphereDrop_SettlesToRest)
{
  // Setup scene...

  double prevEnergy = EnergyTracker::computeSystemEnergy(bodies);

  for (int frame = 0; frame < 500; ++frame)
  {
    worldModel.update(dt);

    double currentEnergy = EnergyTracker::computeSystemEnergy(bodies);
    EXPECT_LE(currentEnergy, prevEnergy + 1e-9)
      << "Energy increased at frame " << frame;
    prevEnergy = currentEnergy;
  }

  // Verify rest state...
}
```

### Sphere vs Cube for Linear Tests

**Use spheres** for linear collision tests because:
- Symmetric contact point (always at pole)
- No lever arm = no rotational coupling
- Cleaner isolation of linear collision mechanics

Cubes introduce edge/corner contact ambiguity even when "axis-aligned."

---

## Estimated Effort

- Scenario A tests: ~4 hours
- Scenario F tests: ~2 hours
- Component isolation tests: ~2 hours
- Integration with EnergyTracker: ~1 hour

**Total**: ~9 hours

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-05
- **Notes**: Subticket split from parent 0038. Establishes linear collision baseline before investigating rotational issues.

### Gemini Review (2026-02-05)
**Status: Approved with Modifications**

Key changes incorporated:
1. **R1**: Added explicit requirement for **Friction (μ) = 0.0** for all tests
2. **A2**: Clarified "immediate rest" → "velocity < 1e-4 after 2 frames"
3. **A6**: Added glancing/oblique collision test (45° impact angle)
4. **A1**: Tightened position tolerance from [0.49, 0.51] to [0.499, 0.501]
5. **F3**: Tightened energy ratio tolerance from [0.24, 0.26] to [0.249, 0.251]

Additional recommendations noted (may implement in future):
- Simultaneous collision test (Newton's Cradle setup)
- High-speed tunneling test (CCD validation)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

