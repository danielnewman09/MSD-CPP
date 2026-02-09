# Ticket 0046: Slop Correction Evaluation

## Status
- [x] Draft
- [x] Ready for Investigation
- [x] Investigation Complete — Findings Documented
- [x] Ready for Implementation
- [x] Merged / Complete

**Current Phase**: Complete
**Assignee**: TBD
**Created**: 2026-02-08
**Generate Tutorial**: No
**Related Tickets**: [0039d_parameter_isolation_root_cause](0039d_parameter_isolation_root_cause.md), [0039e_fix_implementation_regression_testing](0039e_fix_implementation_regression_testing.md), [0040b_split_impulse_position_correction](0040b_split_impulse_position_correction.md), [0045_constraint_solver_unification](0045_constraint_solver_unification.md)
**Type**: Investigation

---

## Problem Statement

The `ConstraintSolver::assembleRHS()` method currently includes a velocity-level slop correction term in the `b` vector of the constraint system `A*lambda = b`:

```cpp
// ConstraintSolver.cpp:320
b(i) = -(1.0 + e) * jv + slopCorrection;
```

where:
```cpp
slopCorrection = 0.2 * (penetration - 0.005) / dt;  // capped at 1.0 m/s
```

This term converts a **position error** (penetration depth) into a **velocity demand** that the solver fulfills through impulse forces. Because those forces produce real velocity changes integrated into the body state, the slop correction **injects kinetic energy** into the system every frame that a contact has penetration exceeding the slop threshold.

### Why This Is a Problem

The solver produces `lambda` values that create forces `F = J^T * lambda / dt`. These forces are applied by the integrator as velocity changes. The `-(1+e)*jv` term is physically correct (it reverses approach velocity with restitution), but `slopCorrection` adds an artificial separating velocity that has no physical origin. This velocity becomes real kinetic energy.

For resting contacts, this creates a pump cycle:
1. Gravity pushes body into floor -> small penetration
2. `jv ~ 0` (nearly at rest), so restitution term is near zero
3. `slopCorrection > 0` because penetration > 0.005m
4. Solver produces lambda that creates separating velocity
5. Body gains KE it didn't have before
6. Next frame: gravity pulls back, repeat

This was identified during 0039d investigation and 0042b work. Tests D1, H1, H5 (resting contacts) fail with slop enabled due to energy injection. Test B1 (polyhedral corner bouncing) fails without slop because bodies dwell in contact with imprecise normals.

### Existing Position Correction

`PositionCorrector` (ticket 0040b) already resolves penetration via **pseudo-velocities** that are applied directly to positions and then discarded — they never enter the velocity state, so no kinetic energy is injected. This is the architecturally correct mechanism for position-level correction.

---

## Objective

Perform a thorough evaluation of what happens when the slop correction is removed from the velocity-level RHS. Document every test that changes status, what the actual failure mode is, and categorize the failures to inform what replacement mechanism (if any) is needed.

---

## Investigation Plan

### Phase 1: Baseline Capture

Run the full `msd_sim_test` suite on the current codebase and record:
- Total pass/fail count
- Explicit list of all currently failing tests (if any)

### Phase 2: Remove Slop Correction

Make a single, minimal change to `ConstraintSolver::assembleRHS()`:

```cpp
// BEFORE (lines 298-318):
double slopCorrection = 0.0;
constexpr double kSlop = 0.005;
if (penetration > kSlop)
{
  double const correctionVel = (penetration - kSlop) / dt;
  slopCorrection = 0.2 * correctionVel;
  slopCorrection = std::min(slopCorrection, 1.0);
  double const approachSpeed = std::abs(jv);
  if (approachSpeed > 0.5)
  {
    slopCorrection = 0.0;
  }
}

b(static_cast<Eigen::Index>(i)) = -(1.0 + e) * jv + slopCorrection;

// AFTER:
b(static_cast<Eigen::Index>(i)) = -(1.0 + e) * jv;
```

No other changes. This isolates the effect of the slop correction alone.

### Phase 3: Full Test Suite Run

Run the full `msd_sim_test` suite with slop removed and record:
- Total pass/fail count
- Every test that changed status (pass->fail or fail->pass)

### Phase 4: Categorize Each Change

For every test that changed status, document:

| Test Name | Old Status | New Status | Failure Detail | Category |
|-----------|-----------|-----------|----------------|----------|
| ... | PASS/FAIL | PASS/FAIL | Actual vs expected | See below |

**Categories:**
- **A — Energy conservation improved**: Test was failing due to energy injection and now passes
- **B — Resting contact sinking**: Body sinks into floor because penetration isn't corrected at velocity level
- **C — Low-speed bounce behavior**: Body behavior changes during low-speed/grazing contacts
- **D — Polyhedral normal dwell**: Body lingers in contact with imprecise polyhedral normals, causing energy growth or incorrect behavior
- **E — Other**: Unexpected failure mode not covered above

### Phase 5: Analysis Report

Produce a summary documenting:
1. Net test count change (gained vs lost)
2. Breakdown by category
3. For each category, assessment of whether the failure is:
   - A **real physics deficiency** that needs a new mechanism
   - A **test tolerance issue** that should be adjusted
   - An **artifact of polyhedral geometry** that will improve with better manifold generation
4. Recommendation for next steps

---

## Key Test Files to Monitor

These test files contain the most relevant collision physics tests:

| File | Description | Key Tests |
|------|-------------|-----------|
| `LinearCollisionTest.cpp` | Head-on linear collisions (A1-A6, F1-F5) | Energy conservation, momentum conservation |
| `RotationalCollisionTest.cpp` | Off-center impacts (B1-B5) | Rotation initiation, energy bounds |
| `ParameterIsolationTest.cpp` | Parameter knockout tests (H1-H6) | Resting contact energy, damping |
| `EnergyAccountingTest.cpp` | Per-frame energy tracking (D1) | Resting contact energy stability |
| `RotationalEnergyTest.cpp` | Rotational energy tests | Energy in rotational collisions |
| `RotationDampingTest.cpp` | Post-collision damping behavior | Energy dissipation |
| `ContactManifoldStabilityTest.cpp` | Contact stability over time | Multi-frame contact behavior |
| `EdgeContactTest.cpp` | Edge-on contact scenarios | Edge manifold handling |
| `PerContactDepthTest.cpp` | Per-contact penetration depth | Depth computation accuracy |
| `CollisionPipelineTest.cpp` | Pipeline integration tests | End-to-end collision flow |
| `ConstraintSolverContactTest.cpp` | Solver unit tests | RHS computation, lambda values |
| `SplitImpulseTest.cpp` | Position correction tests | PositionCorrector behavior |

---

## Acceptance Criteria

1. **AC1**: Baseline test results captured before any changes
2. **AC2**: Slop correction removed with minimal, isolated change
3. **AC3**: Full test suite results captured after removal
4. **AC4**: Every test status change documented with failure detail and category
5. **AC5**: Analysis report produced with net impact assessment and recommendations

---

## Expected Findings (Hypotheses)

Based on prior investigation (0039d, 0042b):

- **Tests likely to improve (pass)**: D1, H1, H5 and other resting-contact energy tests that currently fail due to energy injection
- **Tests likely to regress (fail)**: B1 (polyhedral corner bounce) and possibly other low-speed polyhedral contact scenarios where slop was masking normal imprecision
- **Tests likely unchanged**: High-speed linear collisions (A1-A6) where `approachSpeed > 0.5` already disables slop, friction tests, manifold tests

The investigation will confirm or refute these hypotheses with data.

---

## Deliverables

### D1: Test Comparison Table
Complete pass/fail comparison: baseline vs slop-removed, with failure details.

### D2: Category Breakdown
Count and analysis of each failure category (A through E).

### D3: Recommendations
Concrete recommendation for:
- Whether to permanently remove the slop correction
- What replacement mechanism (if any) is needed for the scenarios that regress
- Whether any test tolerances should be adjusted

---

## Investigation Results

### Phase 1: Baseline

**684 tests total: 676 pass, 8 fail**

Baseline failures (pre-existing):

| # | Test | Failure Summary |
|---|------|----------------|
| 1 | `ContactManifoldStabilityTest.D1_RestingCube_StableFor1000Frames` | Position drift=10.44m, maxVel=2.88m/s, maxOmega=12.88rad/s, energy grew 4.905→5.42J |
| 2 | `ContactManifoldStabilityTest.D4_MicroJitter_DampsOut` | Micro-jitter amplified: maxVel=2.14m/s after 0.017m/s perturbation |
| 3 | `ParameterIsolation.H1_DisableRestitution_RestingCube` | Energy grew 262% (49J→178J) even with e=0, rotKE=114.6J |
| 4 | `ParameterIsolation.H5_ContactPointCount_EvolutionDiagnostic` | Single-contact frames (43) dominate multi-contact (31), max 3 contacts |
| 5 | `ParameterIsolation.H6_ZeroGravity_RestingContact_Stable` | EPA convergence exception |
| 6 | `RotationalCollisionTest.B2_CubeEdgeImpact_PredictableRotationAxis` | No rotation from edge impact (omega=5.8e-8 rad/s) |
| 7 | `RotationalCollisionTest.B3_SphereDrop_NoRotation` | Sphere gained rotation: maxOmega=0.87 (threshold 0.5) |
| 8 | `RotationalCollisionTest.B5_LShapeDrop_RotationFromAsymmetricCOM` | No rotation from asymmetric hull (omega=1.4e-6 rad/s) |

### Phase 2: Change Applied

Single change in `ConstraintSolver::assembleRHS()`: removed the entire `slopCorrection` computation block (lines 296-318), replacing:
```cpp
b(i) = -(1.0 + e) * jv + slopCorrection;
```
with:
```cpp
b(i) = -(1.0 + e) * jv;
```

### Phase 3: Post-Removal Results

**684 tests total: 675 pass, 9 fail**

Net: **-1 passing test** (gained 2 passes, lost 3 passes)

### Phase 4: Test Status Changes

#### Tests That Now PASS (were failing) — 2 tests

| Test | Baseline Failure | Post-Removal | Category |
|------|-----------------|--------------|----------|
| `ParameterIsolation.H5_ContactPointCount_EvolutionDiagnostic` | Single-contact frames dominated (43 single vs 31 multi), max 3 contacts | **PASSES** — contact point distribution now meets expectations | **A — Energy conservation improved** (indirectly: without slop-induced rotation, the cube stays axis-aligned, so EPA produces stable face contacts) |
| `RotationalCollisionTest.B3_SphereDrop_NoRotation` | Sphere acquired omega=0.87 rad/s from symmetric drop | **PASSES** — sphere stays non-rotating as expected | **A — Energy conservation improved** (slop was injecting asymmetric impulses through the single EPA contact point, causing spurious rotation) |

#### Tests That Now FAIL (were passing) — 3 tests

| Test | Post-Removal Failure | Category |
|------|---------------------|----------|
| `ConstraintSolverContactTest.SlopCorrection_CappedToApproachVelocity_0033` | Asserts `lambda > 0` for resting contact with penetration — no lambda produced because no slop correction exists | **E — Test asserts removed feature** |
| `ContactRHS.SlopCorrectionGatedByApproachVelocity` | Asserts lambda matches slop correction formula — lambda is now 0 | **E — Test asserts removed feature** |
| `SplitImpulse.VelocityRHS_NoBaumgarteBias` | Asserts resting contact gets "gentle slop correction" — no correction applied | **E — Test asserts removed feature** |

#### Tests Still Failing (in both baseline and post-removal) — 6 tests

| Test | Baseline | Post-Removal | Change |
|------|----------|-------------|--------|
| `D1_RestingCube_StableFor1000Frames` | drift=10.44m, omega=12.88, energy +10.5% | drift=4.04m, omega=1.79, **energy check now PASSES** (2 of 4 assertions now pass vs 0 of 4) | **Significantly improved** |
| `D4_MicroJitter_DampsOut` | maxVel=2.14m/s, vel@50=0.34 | maxVel=0.46m/s, vel@50=0.16 | **Improved** (amplification 4.6x better, but still fails threshold) |
| `H1_DisableRestitution_RestingCube` | Energy grew 262% (49→178J), rotKE=114.6J | Energy grew 10.6% (49→54J), rotKE=2.66J | **Dramatically improved** (26x less energy injection, 43x less rotKE) |
| `H6_ZeroGravity_RestingContact_Stable` | EPA convergence exception | EPA convergence exception | **Unchanged** (unrelated EPA bug) |
| `B2_CubeEdgeImpact_PredictableRotationAxis` | omega=5.8e-8 | omega=7.3e-8 | **Unchanged** (edge impact rotation not initiated — separate issue) |
| `B5_LShapeDrop_RotationFromAsymmetricCOM` | omega=1.4e-6 | omega=3.4e-15 | **Unchanged** (convex hull fills L-shape — separate geometric issue) |

#### Notable: B1 Did NOT Regress

`RotationalCollisionTest.B1_CubeCornerImpact_RotationInitiated` — **still passes**. The prior hypothesis that slop removal would break B1 was **not confirmed**. The corner impact scenario generates sufficient approach velocity that restitution alone handles it correctly.

### Phase 5: Analysis

#### Summary by Category

| Category | Count | Tests |
|----------|-------|-------|
| **A — Energy conservation improved** | 2 gained | H5 (now passes), B3 (now passes) |
| **E — Test asserts removed feature** | 3 lost | SlopCorrection_Capped, SlopCorrectionGated, VelocityRHS_NoBaumgarte |
| **Improved but still failing** | 3 | D1, D4, H1 (all dramatically better) |
| **Unchanged failures** | 3 | H6, B2, B5 (unrelated root causes) |

#### Net Assessment

The 3 new failures are all **tests that explicitly assert the slop correction feature exists**. These are not physics regressions — they are tests of the removed code path. If slop correction is permanently removed, these 3 tests should be deleted.

Adjusting for this: **net +2 passing physics tests**, with 3 additional tests showing dramatic improvement in their failure metrics even though they don't yet fully pass.

#### D1 Deep Dive: Still Failing But Much Better

| Metric | Baseline | Post-Removal | Improvement |
|--------|----------|-------------|-------------|
| Position drift | 10.44m | 4.04m | 2.6x better |
| Max omega | 12.88 rad/s | 1.79 rad/s | 7.2x better |
| Max velocity | 2.88 m/s | (passes <1.0 check) | Now passes |
| Energy growth | 10.5% | Passes ≤1.01 check | Now passes |

D1 now passes 2 of its 4 assertions (velocity and energy). The remaining failures (position drift and omega) are from a different root cause: EPA producing a single offset contact point for face-on-face contacts, creating an unbalanced torque even without slop. This is the manifold generation issue identified in 0039d.

#### H1 Deep Dive: 26x Reduction in Energy Injection

With slop removed, H1 still shows 10.6% energy growth (down from 262%). The residual growth comes from:
- PositionCorrector's position-level corrections can interact with the velocity solver across frames
- EPA single-contact-point creating asymmetric angular impulses from restitution term itself (even with e=0, the `-(1+0)*jv` term with `jv` from a single offset contact point can create small torques)

This residual is likely addressable by improving manifold generation (multiple contact points for face contacts).

#### Hypothesis Outcomes

| Hypothesis | Result |
|-----------|--------|
| D1, H1, H5 improve | **Confirmed**: H5 now passes, D1 and H1 dramatically improved |
| B1 regresses | **Refuted**: B1 still passes |
| High-speed collisions unchanged | **Confirmed**: A1-A6, F1-F5 all unaffected |
| B3 would be unaffected | **Pleasantly wrong**: B3 now passes (slop was causing spurious sphere rotation) |

---

## Recommendations

### R1: Permanently Remove Slop Correction

The evidence strongly supports permanent removal:
- **+2 net physics passes** (H5, B3)
- **3 tests dramatically improved** (D1, D4, H1)
- **0 physics regressions** (B1 still passes)
- **3 "new failures" are tests of the removed feature, not physics bugs**
- The remaining failures all have root causes in EPA/manifold generation, not in the absence of slop

### R2: Delete 3 Slop-Specific Tests

These tests assert the existence of slop correction and should be deleted:
- `ConstraintSolverContactTest.SlopCorrection_CappedToApproachVelocity_0033`
- `ContactRHS.SlopCorrectionGatedByApproachVelocity`
- `SplitImpulse.VelocityRHS_NoBaumgarteBias`

### R3: No Replacement Mechanism Needed for Velocity-Level RHS

`PositionCorrector` already handles penetration correction without energy injection. The velocity-level RHS should contain **only** the physically correct restitution term `-(1+e)*jv`. No velocity-level position correction is needed.

### R4: Future Work — Manifold Generation

The residual failures (D1 position drift/omega, H1 residual energy growth, D4 jitter amplification) all trace to EPA producing a single contact point for face-on-face contacts. Improving manifold generation to produce 4 contact points for face contacts would likely resolve these remaining issues. This is a separate ticket scope.

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-08 (ticket creation)
- **Completed**: 2026-02-08
- **Notes**: Ticket created to systematically evaluate the impact of removing velocity-level slop correction from the constraint solver RHS. Motivated by analysis showing slop correction injects kinetic energy into the system, conflicting with the energy-conservative PositionCorrector mechanism.

### Investigation Phase
- **Started**: 2026-02-08
- **Completed**: 2026-02-08
- **Branch**: 0045-constraint-solver-unification
- **Notes**: Executed all 5 phases. Baseline: 676/684 pass. After slop removal: 675/684 pass. Net -1 but all 3 new failures are tests of the removed feature. Physics-only: +2 passes (H5, B3), 0 regressions, 3 tests dramatically improved. B1 did NOT regress (contrary to hypothesis). Recommendation: permanently remove slop, delete 3 feature-specific tests.

### Implementation Phase
- **Started**: 2026-02-08
- **Completed**: 2026-02-08
- **Branch**: 0045-constraint-solver-unification
- **Notes**: Applied all recommendations from investigation:
  1. Permanently removed slop correction from `ConstraintSolver::assembleRHS()` — RHS is now purely `-(1+e)*jv`
  2. Deleted 3 slop-specific tests (SlopCorrection_CappedToApproachVelocity, SlopCorrectionGatedByApproachVelocity, VelocityRHS_NoBaumgarteBias)
  3. Updated stale Baumgarte/ERP/slop comments in ConstraintSolver.hpp and ConstraintSolver.cpp
  4. Updated stale comments in JacobianLinearTest.cpp
  - Final result: 681 tests, 675 pass, 6 fail (all 6 are pre-existing failures with unrelated root causes)
  - vs baseline: 684 tests, 676 pass, 8 fail → net +2 physics passes, -3 deleted tests

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
