# Ticket 0067: Contact Phase Energy Injection During Tumbling

## Status
- [x] Draft
- [x] Ready for Investigation
- [x] Root Cause Identified
- [ ] Fix Implemented
- [ ] Verified
- [ ] Merged / Complete

**Current Phase**: Root Cause Identified
**Type**: Debug
**Priority**: High
**Created**: 2026-02-16
**Related Tickets**: [0066](0066_friction_cone_solver_saturation_bug.md) (friction direction fix), [0042b](0042b_rotational_coupling_accuracy.md) (rotational coupling), [0046](0046_slop_correction_removal.md) (slop removal)

---

## Summary

The F4 test (tilted cube dropped with e=1.0, mu=0.5) exhibits sustained energy injection during extended contact phases where the cube tumbles along the floor. While the overall simulation dissipates energy (28.1 J initial -> 4.9 J final), there are phases of 50-100 frames where energy grows by 0.03-0.07 J/frame, accumulating up to +2.4 J in a single contact episode. This violates energy conservation: constraint forces should do zero net work, and friction should only dissipate energy.

---

## Observed Behavior

**Recording**: `ReplayEnabledTest_RotationalEnergyTest_F4_RotationEnergyTransfer_EnergyConserved.db`
**Body**: 1 (1 kg cube, e=1.0, mu=0.5, tilted 45 deg on two axes)

### Energy Profile Summary

| Phase | Frames | Time (s) | Behavior | Energy Change |
|-------|--------|----------|----------|---------------|
| Free fall | 1-40 | 0.0-0.64 | Constant energy loss (integrator) | -0.012 J/frame |
| First bounce | 41 | 0.656 | Impact dissipation | -9.56 J |
| Second bounce | 93 | 1.488 | Impact dissipation | -3.69 J |
| Third bounce | 126 | 2.016 | Impact dissipation | -2.31 J |
| Fourth bounce | 156 | 2.496 | Impact dissipation | -1.92 J |
| **Contact episode 1** | **173-184** | **2.77-2.94** | **Energy injection** | **+0.16 J total** |
| Fifth bounce | 185 | 2.96 | Impact dissipation | -0.50 J |
| Contact/bounces | 206-224 | 3.3-3.6 | Mixed injection/dissipation | oscillating |
| **Contact episode 2** | **226-256** | **3.6-4.1** | **Sustained energy injection** | **+1.05 J total** |
| Bounces | 264-284 | 4.2-4.5 | Impact dissipation series | -1.8 J total |
| **Contact episode 3** | **285-332** | **4.6-5.3** | **Major sustained injection** | **+2.39 J total** |
| Impact sequence | 333-356 | 5.3-5.7 | Dissipation | -2.1 J |
| **Contact episode 4** | **370-378** | **5.9-6.0** | **Energy injection** | **+0.46 J total** |
| Final impact | 379 | 6.064 | Major dissipation | -1.59 J |

### Key Characteristics of Energy Injection Phases

1. **Injection rate grows over time within an episode**: e.g., episode 3 starts at +0.008 J/frame and peaks at +0.069 J/frame
2. **Multi-contact is present**: 2 contact points during injection phases
3. **Friction is near or at saturation**: At frame 300 (mid-episode 3):
   - Contact 0: |lambda_t| = 0.038, cone limit = 0.038 (100% saturated)
   - Contact 1: |lambda_t| = 0.016, cone limit = 0.032 (50% interior)
4. **High angular velocity during injection**: omega ~ (1.36, 0.94, 0.17) rad/s at frame 300
5. **Significant lever arms**: ~0.5-0.7 m from CoM to contact points

### Contact State at Frame 300 (Peak Injection)

```
Position:  (-0.36, -2.05, 0.61)
Velocity:  (+0.61, +0.82, -0.50)   <- still moving downward (vz < 0)
Angular:   (-1.36, +0.94, -0.17)

Contact 0: lever_a = (+0.26, -0.49, -0.66), lambda_n = 0.076, |lambda_t| = 0.038
Contact 1: lever_a = (-0.53, +0.12, -0.67), lambda_n = 0.063, |lambda_t| = 0.016
```

---

## Initial Hypotheses

### H1: Position Corrector Pseudo-Velocity Energy Injection

**Theory**: The PositionCorrector applies pseudo-velocity corrections to resolve penetration. During tumbling contact with large angular velocity, the pseudo-velocities may interact with the constraint forces to do net positive work.

**Evidence For**:
- Energy injection correlates with sustained contact (when PositionCorrector is active)
- Free-flight phases show clean energy behavior (only integrator drift)

**Evidence Against**:
- PositionCorrector was specifically designed to avoid KE injection (ticket 0042a)
- Pseudo-velocities are separate from the velocity solve

**Test**: Disable PositionCorrector in a diagnostic test; compare energy profile.

### H2: Constraint Force Work via Lever Arm Torque

**Theory**: The normal contact force (lambda_n) acts at a lever arm from the CoM. For a tumbling body, this force+lever creates a torque. If the angular velocity aligns with this torque, the constraint force does positive rotational work: `P = tau . omega = (r x F) . omega`. This should be zero for a true rigid contact, but numerical integration error means the body penetrates slightly, then the constraint force accelerates it outward.

**Evidence For**:
- Large lever arms (~0.5-0.7 m) at contact points
- High angular velocity during injection phases
- Injection rate grows as angular velocity increases within an episode

**Evidence Against**:
- Normal force should only push along normal (z-axis for floor contact)
- `(r x F_normal) . omega` should average to zero over a rotation cycle

**Test**: Log `(r x lambda_n * normal) . omega` at each contact point per frame. Check if it correlates with delta_e.

### H3: Friction Force Doing Positive Work (Accelerating Instead of Opposing)

**Theory**: The friction force direction, even after the sliding-direction projection fix (ticket 0066), may still not perfectly oppose the contact point velocity. With 2 contacts and coupled rotational dynamics, the solver may find a friction solution that is self-consistent for the linearized system but does net positive work on the actual nonlinear motion.

**Evidence For**:
- `clampPositiveWorkFriction` only checks work against the *pre-solve* sliding velocity (RHS vector b). Post-solve velocity may differ.
- Both saturated and interior friction present simultaneously

**Evidence Against**:
- The clamp should catch the worst cases
- Friction magnitude is small relative to normal forces

**Test**: Compute `lambda_t . v_contact_tangential` using post-solve velocities. Check for positive work.

### H4: Semi-Implicit Euler Rotational Energy Drift

**Theory**: Semi-implicit Euler is not symplectic for rotational dynamics with constraints. The constraint impulse is computed using pre-step velocities but applied to update the state. For rapidly rotating bodies, this time-integration mismatch causes systematic energy drift.

**Evidence For**:
- Energy injection rate correlates with angular velocity magnitude
- The effect is not present during linear-only motion
- Known limitation of semi-implicit Euler for constrained rotational systems

**Evidence Against**:
- Other tests with rotation (B-series) don't show this magnitude of injection
- The effect seems specific to sustained multi-contact tumbling

**Test**: Halve the timestep and check if energy injection scales quadratically (would confirm integration error).

### H5: Contact Point Migration Work

**Theory**: As the cube tumbles, the contact point slides along both surfaces. The constraint Jacobian is computed for a specific contact point, but the actual contact point moves between frames. The force applied at the old contact location does work as the contact migrates, effectively pumping energy into the system.

**Evidence For**:
- Multiple bounces show contact at different positions on the cube
- Lever arms change significantly between frames during tumbling

**Evidence Against**:
- EPA recomputes contact points each frame
- This should only matter for persistent contacts across frames

**Test**: Compare contact point positions between consecutive frames during injection. Check if migration distance correlates with energy injection.

---

## Investigation Plan

### Phase 1: Characterize the Energy Source

**Goal**: Determine whether the injected energy appears as linear KE, rotational KE, or potential energy.

1. Use body energy timeseries to decompose total energy into components during injection phase (frames 285-332)
2. Check if the normal force is doing work: `F_n . v_contact_point` at each contact
3. Check if friction force is doing work: `F_t . v_contact_tangential` at each contact

### Phase 2: Test Hypotheses via Diagnostic Experiments

**Goal**: Narrow down the root cause.

1. **H1 test**: Run F4 with PositionCorrector disabled. If injection disappears, H1 is confirmed.
2. **H2 test**: Log constraint torque and angular velocity dot product. Correlate with delta_e.
3. **H3 test**: Compute post-solve friction work at each contact point.
4. **H4 test**: Run F4 at dt=0.008 (half timestep). If injection quarters, H4 is confirmed.
5. **H5 test**: Track contact point positions across consecutive frames.

### Phase 3: Targeted Fix

Based on confirmed hypothesis, implement and verify a targeted fix.

---

## Investigation Findings

### Phase 1 Results: Energy Source Identified

**The energy injection comes from the constraint solver returning non-converged solutions.**

#### Energy Decomposition (Episode 3, frames 285-332)

| Component | Frame 285 | Frame 332 | Change |
|-----------|-----------|-----------|--------|
| Linear KE | 0.17 J | 3.00 J | +2.83 J |
| Rotational KE | 0.28 J | 1.26 J | +0.98 J |
| Potential E | 5.99 J | 4.56 J | -1.43 J |
| **Total** | **6.44 J** | **8.82 J** | **+2.38 J** |

The body lowers by ~0.15 m during this phase (PE decreases 1.43 J), but KE grows by 3.81 J. The excess 2.38 J is injected by the constraint solver.

#### Solver Non-Convergence Correlates with Energy Injection

| Frame Range | Converged? | Residual Range | Energy per Frame |
|-------------|-----------|----------------|-----------------|
| 285-286 | Yes (1-4 iter) | ~2.3-2.6 | +0.008-0.011 J |
| 287-302 | **No (50 iter)** | 0.03-1.78 | +0.013-0.056 J |
| 303-310 | Mixed | 1e-11 to 0.13 | +0.058-0.069 J |
| 311-332 | **No (13-50 iter)** | 0.0002-0.93 | +0.034-0.069 J |

**During the entire 47-frame injection episode, the FrictionConeSolver fails to converge on ~80% of frames**, hitting the 50-iteration limit with residuals orders of magnitude above tolerance. The non-converged lambda produces constraint forces that do net positive work.

#### Why the Solver Fails to Converge

The system has:
- 1 contact with 2 contact points → 4 constraints (2 normal + 2 friction = `num_constraints: 4`)
- Friction is saturated (cone surface contact) for at least 1 contact
- The cube is tumbling with angular velocity ~1-2 rad/s

For cone-surface contacts, the projected Newton method's Hessian becomes rank-deficient (the projection Jacobian `J_proj` is rank-2 for a 3x3 system), making the reduced Hessian `H_r = J^T A J` poorly conditioned. The Newton step direction is poor, and the line search (Armijo backtracking) can't find sufficient decrease, so the solver bounces around without converging.

This is the same core issue identified in ticket 0066 (friction cone solver saturation bug), but manifesting as energy injection rather than direction reversal.

### Hypothesis Conclusions

| Hypothesis | Status | Notes |
|-----------|--------|-------|
| H1: PositionCorrector | **Not primary** | May contribute but solver non-convergence is the dominant effect |
| H2: Constraint torque work | **Contributing** | Large lever arms + wrong lambda = large torque error |
| H3: Friction positive work | **Confirmed** | Non-converged lambda has wrong friction direction/magnitude |
| H4: Semi-implicit Euler | **Contributing** | Integration mismatch amplified by non-converged solver output |
| H5: Contact migration | **Not tested** | Superseded by solver convergence finding |

### Root Cause Summary

**The FrictionConeSolver's projected Newton method fails to converge for sustained tumbling contacts where friction is saturated. The non-converged lambda (constraint impulse) does net positive work, injecting energy at 0.03-0.07 J/frame. Over 50+ frame contact episodes, this accumulates to 1-2.4 J of spurious energy.**

The existing mitigations (projectVectorWithSliding, clampPositiveWorkFriction) help with direction but don't address the fundamental convergence failure. The solver needs either:
1. A better convergence strategy for cone-surface contacts (e.g., ADMM, pivoting)
2. A post-solve energy clamp that limits the constraint force to do zero net work
3. ~~Increased iteration budget or adaptive tolerance~~ **RULED OUT**: Tested 200 iterations (4x increase) — residuals barely improved (0.187 → 0.142 at frame 300), energy injection unchanged (+0.051 → +0.043 J/frame), runtime 2.6x worse. The solver is fundamentally stuck on the rank-deficient cone surface, not iteration-starved.

---

## Files of Interest

| File | Relevance |
|------|-----------|
| `msd-sim/src/Environment/WorldModel.cpp` | `update()` orchestration, PositionCorrector call |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Friction solve path, `clampPositiveWorkFriction` |
| `msd-sim/src/Physics/Constraints/FrictionConeSolver.cpp` | Projected Newton solver with sliding projection |
| `msd-sim/src/Physics/Collision/CollisionPipeline.cpp` | Contact detection and manifold generation |
| `msd-sim/src/Physics/Constraints/PositionCorrector.cpp` | Penetration resolution via pseudo-velocities |
| `msd-sim/src/Physics/Integration/SemiImplicitEuler.cpp` | Velocity/position integration |
| `msd-sim/test/Physics/Collision/RotationalEnergyTest.cpp` | F4 test definition |

---

## Reproduction

```bash
# Build and run F4 test to generate recording
cmake --build --preset debug-sim-only
./build/Debug/debug/msd_sim_test --gtest_filter="*F4_RotationEnergyTransfer*"

# Load recording and examine energy injection phase
# Using replay MCP server:
load_recording("ReplayEnabledTest_RotationalEnergyTest_F4_RotationEnergyTransfer_EnergyConserved.db")
get_system_energy(start_frame=285, end_frame=332)  # Major injection episode
get_contacts_for_body(body_id=1, frame_id=300)      # Contact state during injection
get_body_state(body_id=1, frame_id=300)              # Kinematic state during injection
```

---

## Acceptance Criteria

1. Energy injection during sustained contact phases is eliminated or reduced below 0.01 J/frame
2. F4 test energy drift stays below 1% of initial energy (currently passes, should remain passing)
3. No regression in existing physics tests (734/741 baseline on branch 0066)
4. Root cause is documented with clear explanation of the energy injection mechanism

---

## Notes

- The F4 test currently **passes** its 1% energy drift threshold because the net effect over 500 frames is energy loss. The energy injection is masked by larger dissipation at impacts.
- This ticket builds on the friction direction fixes from ticket 0066 (projectVectorWithSliding + clampPositiveWorkFriction), which addressed a related but distinct problem (friction *direction* vs energy *injection*).
- The issue is most visible in the F4 test because it creates sustained tumbling contact — a scenario that stresses the coupling between rotational dynamics, friction, and constraint forces.
