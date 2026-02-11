# Investigation Log: 0055b Friction Direction Root Cause

**Created**: 2026-02-10
**Investigator**: workflow-orchestrator → diagnostic analysis
**Objective**: Systematically identify why friction force direction is incorrect for certain tilt orientations

---

## Phase 1: Characterize the Failure Pattern

### Test Suite Status (from 0055a)

Running the TiltedCubeTrajectoryTest suite to characterize failures:

**Command**:
```bash
cd /Users/danielnewman/Documents/GitHub/MSD-CPP
cmake --build --preset conan-debug --target msd_sim_test
./build/Debug/debug/msd_sim_test --gtest_filter="TiltedCubeTrajectory*"
```

### Phase 1 Questions

| Question | Status | Findings |
|----------|--------|----------|
| Which tilt orientations fail? | ANSWERED | All T1-T8 pass; failures are in compound-tilt sliding tests |
| Is failure in direction or magnitude? | ANSWERED | Both: spurious cross-axis motion AND 21x energy injection |
| Do symmetry tests fail? | ANSWERED | No — symmetry is preserved |
| Does failure depend on friction? | ANSWERED | **YES** — frictionless compound case is perfectly stable |
| Does failure depend on tilt magnitude? | ANSWERED | Yes — 0.01 rad perturbation sufficient to trigger |

### Test Execution Results

**Date**: 2026-02-10
**Command**: `./build/Debug/debug/msd_sim_test --gtest_filter="TiltedCubeTrajectory*"`
**Result**: 16 PASS, 3 FAIL

#### Passing Tests (16)
- T1-T8 (all tilt orientation tests) — **Interesting: no NaN, but failures are in magnitude/cross-axis tests**
- Symmetry tests (4) — All pass
- Several sliding/compound tests pass

#### Failing Tests (3)

1. **Compound_SuperpositionMagnitude**
   - Compound displacement at only **26.6%** of superposition prediction
   - Single-X mag = 0.0417m, Single-Y mag = 0.0417m
   - Superposition prediction = 0.0589m
   - **Actual compound = 0.0157m** (suppressed)

2. **Sliding_ThrownCube_SDLAppConfig** (CRITICAL)
   - Config: tilt = `(π/3, 0.01, 0)`, velocity = `(5, 0, 0)`, friction = 0.5
   - **Spurious Y displacement = 20.7m** (should be ~0)
   - Total X displacement = 57.4m
   - Y/X ratio = 36.1% (massive cross-axis motion)
   - Peak vy = 4.27 m/s

3. **Sliding_PurePitch_vs_CompoundTilt_SpuriousY**
   - Pure pitch (π/3, 0, 0): X = 2.72m, Y ≈ 0
   - Compound (π/3, 0.01, 0): X = 57.4m, Y = -20.7m
   - **Compound travels 21x farther than pure pitch**
   - Massive energy injection from tiny 0.01 rad perturbation

### Key Findings

1. **The bug is NOT in basic tilt handling** — T1-T8 all pass without NaN
2. **The bug IS in cross-axis coupling when velocity + tilt break symmetry**
3. **Tiny perturbations (0.01 rad) cause massive energy injection** (21x amplification)
4. **Spurious Y displacement of 20.7m from 0.01 rad perturbation** — this is unreasonable
5. **Symmetry is preserved** — suggests the bug is deterministic, not a numerical instability

---

## Phase 2: Isolate the Source

### Phase 2a: EPA Contact Normal Accuracy — ELIMINATED

**Hypothesis (H1)**: EPA extracts a contact normal with small lateral components from the Minkowski difference for tilted cubes, which rotates the tangent basis and causes friction to push in the wrong direction.

**Diagnostic**: `Diag_EPANormal_PurePitch_vs_Compound` test with `runInstrumentedSimulation()`.

**Result: H1 COMPLETELY RULED OUT**
- EPA normal is **perfectly (0, 0, -1)** for both pure pitch and compound cases
- Lateral component is literally 0.0 for compound, ~1e-9 for pure pitch
- Tangent basis is perfect: t1=(0,1,0), t2=(1,0,0)
- Normal is stable across all 500 frames (no flips, no drift)
- Normal time series test confirms zero lateral component throughout

### Phase 2b: Contact Manifold Quality — ROOT CAUSE FOUND

**Hypothesis (H2)**: Contact point asymmetry between pure pitch and compound tilt cases.

**Diagnostic**: `Diag_ContactCount_And_LeverArm` test.

**Result: CRITICAL DIFFERENCE FOUND**

| Metric | Pure Pitch | Compound |
|--------|-----------|----------|
| 1-point contacts | **0%** | **89%** |
| 2-point contacts | 12% | 11% |
| 4-point contacts | **88%** | **0%** |

Pure pitch cube rests on an **edge** → naturally produces 2-4 contact points.
Compound-tilted cube rests on a **vertex** → geometrically correct single contact point.

**Contact Point Drift**: Compound contact Y-coordinate jumps wildly between corners:
```
f5   cpA_Y = -0.495
f6   cpA_Y = +0.492   (flip!)
f75  cpA_Y = -0.153
f92  cpA_Y = -0.989
f97  cpA_Y = -0.435
```
The contact point sweeps across the entire cube bottom face each bounce period.

### Phase 2c: Energy Balance — FRICTION IS THE SOLE CAUSE

**Diagnostic**: `Diag_EnergyBalance_CompoundVsPurePitch` test.

**Result: Energy injection requires friction and only occurs with single-point contacts**

| Case | KE at f=0 | Peak KE | KE at f=1000 | Status |
|------|-----------|---------|-------------|--------|
| Pure pitch, μ=0.5 | 125 J | 125 J | 0.048 J | CORRECT (dissipates) |
| Compound, μ=0.5 | 125 J | **134.7 J** | 119.9 J | **ENERGY INJECTION** |
| Compound, μ=0.0 | 125 J | 125.0 J | 125.0 J | PERFECT (conserves) |

Key observations:
- Compound with friction reaches **134.7 J peak** — exceeds initial 125 J by 7.7%
- Without friction, compound case is perfectly energy-conserving at 125.048 J for 1000 frames
- Pure pitch correctly dissipates from 125 J → 0.048 J
- **The normal constraint does NOT inject energy** — only friction does

### Phase 2d: Yaw Coupling Mechanism — CONFIRMED

**Hypothesis**: Single-point friction creates uncompensated yaw torque via the friction Jacobian's angular coupling term `rA × t`.

**Diagnostic**: `Diag_YawCoupling_SinglePointFriction` test.

**Result: CONFIRMED — single-point friction creates 4.8 million times more yaw than multi-point**

| Case | Peak |omega_z| | Ratio vs Pure |
|------|---------------------|-----------------|
| Pure pitch, μ=0.5 | 4.3e-7 rad/s | 1x |
| Compound, μ=0.5 | **2.06 rad/s** | **4,800,000x** |
| Compound, μ=0.0 | 0.0018 rad/s | 4,000x |

The compound+friction case has massive yaw rotation that grows over time (cumulative yaw reaches 1.12 radians by frame 500). The frictionless case has negligible yaw (0.008 rad).

---

## Phase 3: Root Cause Documentation

### Root Cause Summary

**Status**: IDENTIFIED AND CONFIRMED

**Classification**: Single-point friction energy injection via uncompensated yaw torque

### Root Cause Chain

1. **Compound tilt** (tiltX=0.01, tiltY=π/3) → cube rests on a **vertex**, not an edge
2. **Vertex-face contact** → EPA/SAT correctly returns **1 contact point** (89% of frames)
3. **Single contact at offset lever arm** → friction Jacobian has angular coupling: `J_ω = rA × t`
4. **Friction impulse at offset point** → creates **yaw torque** about the contact normal (Z-axis)
5. **With multi-point contacts**, opposing yaw torques from different corners cancel; with 1 point, **yaw is uncompensated**
6. **Yaw rotation** changes which vertex is lowest → contact point **jumps to a new corner**
7. **New contact point = new lever arm** → friction pushes in a **different direction**
8. **Oscillating friction direction** does net positive work → **energy injection**
9. **Energy injection** → growing velocity → growing penetration → self-reinforcing instability

### Mathematical Explanation

For a friction constraint at contact point `p` with lever arm `rA = p - CoM`:

```
J_friction = [t^T, (rA × t)^T, -t^T, -(rB × t)^T]
```

The angular coupling `rA × t` creates a torque when friction impulse λ_t is applied. For tangent `t = (0, 1, 0)` and lever arm `rA = (rX, rY, rZ)`:

```
rA × t = (-rZ, 0, rX)
```

The Z-component `rX` creates yaw. With a single contact at corner position `rX ≈ ±0.5m`, this produces a yaw torque of `0.5 * λ_t` each frame.

With 4 contact points at all corners, the yaw contributions cancel:
```
Σ(rA_i × t) = (rX₁ + rX₂ + rX₃ + rX₄) * λ_t ≈ 0
```

With 1 contact point, the yaw torque is fully uncompensated.

### Code Location(s)

- **Contact manifold generation**: `CollisionHandler::checkCollision()` — produces single-point manifolds for vertex-face contacts
- **SAT fallback**: `CollisionHandler::buildSATContact()` — responsible for contact manifold construction when SAT is used
- **Friction Jacobian**: `FrictionConstraint::jacobian()` — angular coupling `rA × t` creates yaw from offset contact
- **Lever arm**: `FrictionConstraint` constructor computes `lever_arm_a_ = contactPointA - comA`

The bug is NOT in any individual component — each works correctly in isolation. The issue is that **single-point contact manifolds** combined with **friction Jacobian angular coupling** create an unstable feedback loop.

### Proposed Fix Strategy

**Approach 1: Multi-point manifold generation** (RECOMMENDED)
- For vertex-face contacts, clip the face plane against the opposing geometry to generate 3-4 contact points
- This is the standard approach in Box2D and Bullet
- Ensures yaw torques from friction cancel even for corner-resting cubes
- Benefits: physically correct, eliminates root cause

**Approach 2: Contact point temporal coherence**
- Cache contact point positions across frames
- Smooth/filter contact point locations to prevent frame-to-frame jumps
- Benefits: simpler to implement
- Risks: may mask other issues, doesn't fix the fundamental under-constraint problem

**Approach 3: Friction impulse at centroid**
- Apply friction at the geometric center of the contact patch rather than at the detected contact point
- Eliminates yaw coupling entirely for single contacts
- Benefits: simple, eliminates yaw
- Risks: changes friction behavior for legitimate off-center contacts

**Approach 4: Energy-limited friction**
- Clamp friction impulse to never increase total kinetic energy
- Post-solve check: if KE increased, scale friction impulse down
- Benefits: prevents energy injection regardless of cause
- Risks: may over-damp legitimate friction scenarios

---

## Iteration Log

### Iteration 1: Phase 1 Execution and Initial Findings
**Date**: 2026-02-10
**Actions**:
- Created investigation artifacts directory and initialized log
- Ran full TiltedCubeTrajectory test suite (19 tests)
- Analyzed failure pattern
- Reviewed tangent basis construction algorithm (Duff et al. 2017)
- Reviewed EPA/SAT collision detection architecture
- Created diagnostic program skeleton for EPA normal extraction

**Results**:
- 16/19 tests PASS
- 3/19 tests FAIL with massive energy injection (21x amplification from 0.01 rad perturbation)
- Bug is NOT in basic tilt handling (all orientation tests pass)
- Bug IS in cross-axis coupling when velocity + asymmetric tilt break symmetry
- Symmetry is preserved (deterministic bug, not numerical instability)

**Key Finding**: Tiny 0.01 rad perturbation causes:
- 20.7m spurious Y displacement (should be ~0)
- 21x travel distance vs pure pitch baseline
- Peak Y velocity of 4.27 m/s from ~0 initial

**Hypothesis Ranking**: H1 (EPA Normal Lateral Perturbation) is MOST LIKELY

### Iteration 2: Phase 2 — Full Diagnostic Suite and Root Cause Identification
**Date**: 2026-02-10
**Actions**:
- Built `runInstrumentedSimulation()` helper in TiltedCubeTrajectoryTest.cpp
- Created 5 diagnostic tests:
  1. `Diag_EPANormal_PurePitch_vs_Compound` — EPA normal comparison
  2. `Diag_ContactCount_And_LeverArm` — Contact manifold quality
  3. `Diag_EnergyBalance_CompoundVsPurePitch` — Energy conservation analysis
  4. `Diag_EPANormal_NormalTimeSeries` — Normal stability over time
  5. `Diag_YawCoupling_SinglePointFriction` — Yaw torque mechanism confirmation
- Examined FrictionConstraint implementation (lever arm computation, Jacobian structure)

**Results**:
- **H1 ELIMINATED**: EPA normals are perfect (0, 0, -1) with zero lateral component
- **H2 CONFIRMED**: Contact manifold quality is the root cause
  - Pure pitch: 0% single-point, 88% four-point (stable edge contacts)
  - Compound: 89% single-point, 0% four-point (unstable vertex contacts)
- **Energy injection REQUIRES friction**: Frictionless compound = perfect energy conservation
- **Yaw coupling CONFIRMED**: Compound+friction has 4.8 million times more yaw than pure pitch
- Root cause chain fully documented (see Phase 3)

**Root Cause**: Single-point vertex-face contacts with friction create uncompensated yaw torque through the friction Jacobian angular coupling `rA × t`. The yaw rotates the cube, changing the contact vertex, causing the contact point to jump. Oscillating friction direction does net positive work, injecting energy.

**Recommended Fix**: Multi-point manifold generation for vertex-face contacts (standard Box2D/Bullet approach).
