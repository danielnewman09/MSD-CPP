# Ticket 0055b: Friction Direction Root Cause Investigation

## Status
- [x] Draft
- [x] Investigation In Progress
- [x] Investigation Complete — Root Cause Identified
- [ ] Merged / Complete

**Current Phase**: Investigation Complete — Root Cause Identified
**Type**: Investigation
**Priority**: High
**Assignee**: workflow-orchestrator
**Created**: 2026-02-10
**Branch**: 0055b-friction-direction-root-cause
**GitHub Issue**: #39
**GitHub PR**: #40 (draft)
**Parent Ticket**: [0055_tilted_cube_friction_direction](0055_tilted_cube_friction_direction.md)
**Dependencies**: [0055a](0055a_tilted_cube_trajectory_test_suite.md)
**Generate Tutorial**: No

---

## Objective

Using the failing tests from 0055a as diagnostic evidence, systematically identify why the friction force direction is incorrect for certain tilt orientations. Produce a documented root cause analysis with specific code locations.

---

## Root Cause Summary

**Classification**: Single-point friction energy injection via uncompensated yaw torque

### Root Cause Chain

1. **Compound tilt** (tiltX=0.01, tiltY=π/3) → cube rests on a **vertex**, not an edge
2. **Vertex-face contact** → EPA/SAT correctly returns **1 contact point** (89% of frames vs 0% for pure pitch)
3. **Single contact at offset lever arm** → friction Jacobian angular coupling `rA × t` creates **yaw torque**
4. **With multi-point contacts**, opposing yaw torques cancel; with 1 point, **yaw is uncompensated**
5. **Yaw rotation** changes which vertex is lowest → contact point **jumps** to a new corner
6. **Oscillating friction direction** does net positive work → **energy injection** (KE exceeds initial by 7.7%)
7. Self-reinforcing: growing velocity → growing penetration → accelerating instability

### Key Evidence

| Diagnostic | Finding |
|-----------|---------|
| EPA Normal | Perfect (0,0,-1), zero lateral — H1 ELIMINATED |
| Contact Count | Pure pitch: 0% single-point, 88% four-point. Compound: **89% single-point**, 0% four-point |
| Energy Balance | Frictionless compound: perfectly conserves 125J. With friction: **peaks at 134.7J** (exceeds initial) |
| Yaw Coupling | Compound+friction: peak omega_z = **2.06 rad/s** (4.8 million× more than pure pitch) |

### Code Locations

- Contact manifold generation: `CollisionHandler::checkCollision()`, `buildSATContact()`
- Friction lever arm coupling: `FrictionConstraint::jacobian()` — angular term `rA × t`
- The bug is NOT in any individual component — each works correctly in isolation

### Proposed Fix for 0055c

**Recommended**: Multi-point manifold generation for vertex-face contacts (standard Box2D/Bullet approach). Clip the contact face against the opposing geometry to generate 3-4 contact points, ensuring yaw torques from friction cancel.

---

## Investigation Strategy

### Phase 1: Characterize the Failure Pattern — COMPLETE

| Question | Answer | Evidence |
|----------|--------|----------|
| Which tilt orientations fail? | Compound tilts with velocity | All T1-T8 pass; sliding tests fail |
| Is failure in direction or magnitude? | Both | Spurious Y=20.7m, 21x energy injection |
| Do symmetry tests fail? | No | All 4 symmetry tests pass |
| Does failure depend on friction? | **YES** | Frictionless compound: KE=125J constant |
| Does failure depend on tilt magnitude? | Yes | 0.01 rad perturbation sufficient |

### Phase 2: Isolate the Source — COMPLETE

- **2a: EPA Normal** — ELIMINATED. Perfect (0,0,-1) for all cases.
- **2b: Tangent Basis** — ELIMINATED. t1=(0,1,0), t2=(1,0,0) — perfectly aligned.
- **2c: Contact Manifold Quality** — ROOT CAUSE. 89% single-point vs 0% for pure pitch.
- **2d: Yaw Coupling** — CONFIRMED. 4.8M× more yaw with single-point friction.
- 2e: Not needed (root cause found at 2c/2d).

### Phase 3: Root Cause Documentation — COMPLETE

Full investigation log: `docs/investigations/0055b_friction_direction_root_cause/investigation-log.md`

---

## Key Hypotheses (Final Status)

| Hypothesis | Status | Evidence |
|-----------|--------|----------|
| H1: EPA Normal Lateral Perturbation | **ELIMINATED** | Normal is exactly (0,0,-1) with zero lateral |
| H2: Contact Point Asymmetry | **CONFIRMED** (reframed as contact manifold quality) | 89% single-point vs 0% for pure pitch |
| H3: ReferenceFrame Overload Bug | Not investigated (root cause found) | |
| H4: Sign Convention Inconsistency | **ELIMINATED** | Symmetry tests pass |

---

## Acceptance Criteria

1. [x] Root cause identified with specific file and line number references
2. [x] Failure pattern fully characterized (which orientations fail and why)
3. [x] At least one hypothesis confirmed or ruled out with evidence
4. [x] Proposed fix strategy documented
5. [x] Investigation findings recorded in ticket for 0055c to implement

---

## Workflow Log

### Investigation In Progress Phase
- **Started**: 2026-02-10
- **Branch**: 0055b-friction-direction-root-cause
- **GitHub Issue**: #39
- **GitHub PR**: #40 (draft)
- **Artifacts**:
  - `docs/investigations/0055b_friction_direction_root_cause/investigation-log.md`
- **Progress**:
  - Phase 1 COMPLETE: Characterized failure pattern (16/19 tests pass, 3 fail with 21x energy injection)
  - Phase 2a COMPLETE: EPA normals are perfect — H1 eliminated
  - Phase 2b COMPLETE: Tangent basis is perfect
  - Phase 2c COMPLETE: Contact manifold quality is the root cause (89% single-point)
  - Phase 2d COMPLETE: Yaw coupling confirmed (4.8M× more with friction)
  - Phase 3 COMPLETE: Root cause documented with fix strategy

### Investigation Complete Phase
- **Completed**: 2026-02-10
- **Diagnostic tests added**: 5 tests in TiltedCubeTrajectoryTest.cpp
  1. `Diag_EPANormal_PurePitch_vs_Compound`
  2. `Diag_ContactCount_And_LeverArm`
  3. `Diag_EnergyBalance_CompoundVsPurePitch`
  4. `Diag_EPANormal_NormalTimeSeries`
  5. `Diag_YawCoupling_SinglePointFriction`
- **Root cause**: Single-point vertex-face contacts + friction Jacobian angular coupling = uncompensated yaw torque → energy injection
- **Recommended fix**: Multi-point manifold generation for vertex-face contacts
