# Ticket 0047: Face Contact Manifold Multi-Point Generation

## Status
- [x] Draft
- [x] Ready for Investigation
- [ ] Investigation Complete
- [ ] Ready for Implementation
- [ ] Merged / Complete

**Current Phase**: Ready for Investigation
**Assignee**: TBD
**Created**: 2026-02-09
**Generate Tutorial**: No
**Related Tickets**: [0046_slop_correction_evaluation](0046_slop_correction_evaluation.md), [0040c_edge_contact_manifold](0040c_edge_contact_manifold.md), [0027a_expanding_polytope_algorithm](0027a_expanding_polytope_algorithm.md)
**Type**: Investigation / Feature Enhancement

---

## Problem Statement

EPA's `extractContactManifold()` produces a **single contact point** for face-on-face contact scenarios. When two axis-aligned cube faces are in contact (e.g., a cube resting on a flat floor), the manifold should produce 4 contact points (one at each corner of the contact region), but instead produces 1 point at an offset location. This single offset contact point creates a non-zero lever arm `r x n != 0`, generating spurious torque that destabilizes resting contacts.

### Affected Tests (3 tests)

| Test | Suite | Failure Mode | Current Values |
|------|-------|-------------|----------------|
| `D1_RestingCube_StableFor1000Frames` | ContactManifoldStabilityTest | Position drift = 4.04m (threshold 0.1m), omega = 1.79 rad/s (threshold 1.0) | Energy and velocity assertions now pass (post-0046) |
| `D4_MicroJitter_DampsOut` | ContactManifoldStabilityTest | Micro-jitter amplified 26.6x (threshold 10x), vel@50 = 0.16 (threshold 0.087) | Perturbation of 0.017 m/s amplifies instead of damping |
| `H1_DisableRestitution_RestingCube` | ParameterIsolation | Energy grows 10.6% with e=0 (should be stable), rotKE = 2.66 J (threshold 0.01) | Energy growth 26x better than pre-0046 baseline but still present |

### Root Cause Analysis

The Sutherland-Hodgman clipping algorithm in `EPA::extractContactManifold()` is intended to produce multi-point contact manifolds for face-face contacts. However, for axis-aligned cube-on-plane scenarios, the algorithm falls through to the degenerate-case branch (< 3 clipped points) and produces a single contact point. This single point is positioned at the EPA witness point, which is typically offset from the center of mass, creating a lever arm that converts normal force into torque.

**Why this matters**: With 4 contact points symmetrically positioned around the face, the torques from normal forces cancel out (sum of `r_i x F_i = 0` for symmetric `r_i`). With 1 offset contact point, every frame of resting contact injects rotational energy into the system.

**Evidence from 0046 investigation**: After removing slop correction, D1/H1 dramatically improved (26x less energy injection for H1) but still fail because the residual energy injection comes from the single-contact-point torque mechanism, not from slop.

---

## Investigation Plan

### Phase 1: Diagnose Manifold Generation Path

1. Add diagnostic output to `EPA::extractContactManifold()` to trace the clipping path for axis-aligned cube-on-plane contacts
2. Identify why Sutherland-Hodgman clipping falls to the degenerate branch for face-face contacts
3. Determine if the issue is in facet alignment detection, reference face selection, or clipping arithmetic

### Phase 2: Study Reference Implementations

1. Research how production physics engines (Bullet, Box2D, ODE) generate face-face contact manifolds
2. Identify the standard approach: most engines use a dedicated face-clipping algorithm that selects the reference face and incident face, clips the incident face polygon against the reference face edges, and projects surviving points onto the reference face plane
3. Document the algorithm with equations and edge cases

### Phase 3: Prototype Fix

1. Implement improved face-face clipping that produces 4 contact points for cube-face contacts
2. Validate with diagnostic tests:
   - Axis-aligned cube on plane should produce 4 contact points
   - Tilted cube on plane should produce 1-2 edge contact points (as currently handled by 0040c)
   - Two overlapping cubes should produce appropriate manifold
3. Verify D1, D4, H1 assertions pass with improved manifold

### Phase 4: Integration

1. Integrate improved manifold generation into EPA
2. Run full test suite to verify no regressions
3. Verify resting contact stability over extended simulation periods

---

## Key Files

| File | Relevance |
|------|-----------|
| `msd-sim/src/Physics/Collision/EPA.hpp`, `EPA.cpp` | Contact manifold extraction (Sutherland-Hodgman clipping) |
| `msd-sim/src/Physics/Collision/CollisionResult.hpp` | CollisionResult with contactPoints vector |
| `msd-sim/src/Physics/Collision/CollisionPipeline.cpp` | Creates ContactConstraints from manifold points |
| `msd-sim/src/Physics/Constraints/ContactConstraint.cpp` | Computes lever arms `r_A`, `r_B` for torque |
| `msd-sim/test/Physics/Collision/ContactManifoldStabilityTest.cpp` | D1, D4 tests |
| `msd-sim/test/Physics/Collision/ParameterIsolationTest.cpp` | H1 test |

---

## Acceptance Criteria

1. **AC1**: EPA produces 4 contact points for axis-aligned face-on-face contacts (cube resting flat on plane)
2. **AC2**: `D1_RestingCube_StableFor1000Frames` passes all 4 assertions (position drift < 0.1m, velocity < 1.0 m/s, omega < 1.0 rad/s, energy growth <= 1%)
3. **AC3**: `D4_MicroJitter_DampsOut` passes (jitter amplification < 10x, velocity at frame 50 < 5x perturbation)
4. **AC4**: `H1_DisableRestitution_RestingCube` passes (energy stable with e=0, rotational KE < 0.01 J)
5. **AC5**: No regressions in existing passing tests (675/681 baseline)
6. **AC6**: Edge contact manifolds (ticket 0040c) continue to work correctly for tilted/edge-on contacts

---

## Deliverables

### D1: Root Cause Diagnosis
Document the exact code path in EPA where face-face contacts fall to the degenerate branch and why.

### D2: Improved Manifold Algorithm
Face-face clipping that produces up to 4 contact points for face contacts, integrated into EPA.

### D3: Test Results
Full test suite results showing D1, D4, H1 passing with no regressions.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

---

## Workflow Log

### Draft → Ready for Investigation
- **Timestamp**: 2026-02-09 (workflow orchestrator)
- **Action**: Ticket advanced to Ready for Investigation
- **Notes**: Investigation ticket ready for prototyping agent to execute investigation plan (Phases 1-4)
