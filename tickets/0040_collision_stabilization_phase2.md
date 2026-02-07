# Ticket 0040: Collision System Stabilization Phase 2

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
**Created**: 2026-02-07
**Generate Tutorial**: No
**Predecessor**: [0039_collision_energy_stabilization_debug](0039_collision_energy_stabilization_debug.md)
**Type**: Implementation (Parent)

---

## Subtickets

This ticket tracks the remaining 11 test failures from the 0039 collision debug investigation. These failures expose deeper architectural issues in the collision constraint pipeline that require dedicated fixes.

| Subticket | Title | Type | Dependencies | Status |
|-----------|-------|------|--------------|--------|
| [0040a](0040a_per_contact_penetration_depth.md) | Per-Contact Penetration Depth | Implementation | None | Draft |
| [0040b](0040b_split_impulse_position_correction.md) | Split Impulse Position Correction | Implementation | 0040a | Draft |
| [0040c](0040c_edge_contact_manifold.md) | Edge Contact Manifold | Implementation | None | Draft |
| [0040d](0040d_contact_persistence_warm_starting.md) | Contact Persistence and Warm-Starting | Implementation | 0040b | Draft |

### Dependency Graph

```
0040a (Per-Contact Depth)
    │
    └──► 0040b (Split Impulse) ──► 0040d (Warm-Starting)

0040c (Edge Contacts)          [independent, can be done anytime]
```

### Subticket Summary

- **0040a**: Add per-contact penetration depth to `ContactPoint`. Currently all contacts share a single `penetrationDepth`, causing N× overcorrection with multi-contact manifolds.
- **0040b**: Replace Baumgarte stabilization with split impulse (pseudo-velocity) position correction. Eliminates the primary source of energy injection.
- **0040c**: Fix edge-edge contact manifold generation. Currently produces a single degenerate point with `r × n = 0`, preventing torque generation.
- **0040d**: Add contact persistence cache and warm-starting. Reduces frame-to-frame jitter and improves solver convergence.

---

## Background

### What 0039 Accomplished

Ticket 0039 diagnosed and partially fixed collision energy injection:

1. **EPA normal-space mismatch** (0039d/0039e): `getFacetsAlignedWith()` received world-space normal but hull facets are in local space. Fixed by transforming normal to hull-local space.
2. **Baumgarte ERP velocity amplification** (0039d/0039e): Unclamped `(ERP/dt) * penetration` injected excessive correction velocity. Fixed with 5.0 m/s clamp.

**Result**: 612/624 → 613/624 passing tests.

### Remaining 11 Failures

All remaining failures are diagnostic tests from 0039c/0039d that probe deeper architectural issues:

#### Category 1: Baumgarte Energy Injection (~7 failures)

Baumgarte correction `(ERP/dt) * penetration` adds velocity to resolve positional drift, but this velocity becomes real kinetic energy. With rotated/rocking objects and multi-contact points sharing the same `penetrationDepth`, the overcorrection creates a positive feedback loop.

| Test | What Fails | Root Cause |
|------|-----------|------------|
| H1_DisableRestitution_RestingCube | Energy grows 675,753% even with e=0 | Baumgarte injects energy independent of restitution |
| H8_TiltedCube_FeedbackLoop | Angular velocity grows 20+ consecutive frames | Tilt → asymmetric penetration → torque → more tilt |
| C2_RockingCube_AmplitudeDecreases | Rocking amplitude increases | Baumgarte correction through offset contact creates torque |
| C3_TiltedCubeSettles_ToFlatFace | Cube doesn't settle | Energy injection prevents equilibrium |
| D1_RestingCube_1000Frames | Position drifts over 1000 frames | Compounding Baumgarte energy |
| D4_SmallJitter_NoAmplification | Micro-motions amplify | Baumgarte amplifies instead of damping |
| F4_RotationEnergyTransfer | Energy grows during rotational collision | Multi-contact shares single penetrationDepth → N× overcorrection |

#### Category 2: Edge Contact Geometry (1 failure)

| Test | What Fails | Root Cause |
|------|-----------|------------|
| B2_CubeEdgeImpact | No rotation from edge impact | `buildPolygonFromFacets()` returns <3 vertices → single-point fallback at normal axis → `r × n = 0` → zero torque |

#### Category 3: Convex Hull Limitation (1 failure — not fixable)

| Test | What Fails | Root Cause |
|------|-----------|------------|
| B5_LShapeDrop | No asymmetric rotation | Convex hull fills L-shape concavity, making geometry symmetric. Requires compound shapes (out of scope). |

#### Category 4: Zero-Gravity Rotational Transfer (1 failure)

| Test | What Fails | Root Cause |
|------|-----------|------------|
| F4b_ZeroGravity_RotationalEnergyTransfer | Energy grows without gravity | Multi-contact sharing single penetrationDepth causes overcorrection even without gravity feedback |

#### Category 5: Rotational Initiation Energy (1 failure)

| Test | What Fails | Root Cause |
|------|-----------|------------|
| B1_CubeCornerImpact | Energy growth > 5% tolerance | Baumgarte correction through corner contact lever arm injects energy |

---

## Known Limitation: B5 L-Shape Convex Hull

B5's failure is a fundamental convex hull limitation — the L-shape concavity is filled by the convex hull, making the geometry symmetric. The center of mass is at the geometric center, not offset as needed for asymmetric rotation. Fixing this requires compound shapes or mesh collision, which is out of scope for 0040. Document as a known limitation in the test.

---

## Acceptance Criteria

1. [ ] **AC1**: All Baumgarte energy injection tests pass (H1, H8, C2, C3, D1, D4, F4, F4b, B1)
2. [ ] **AC2**: Edge contact test passes (B2)
3. [ ] **AC3**: No regression in existing collision tests or 0039 linear collision tests
4. [ ] **AC4**: B5 documented as known limitation
5. [ ] **AC5**: Total passing tests ≥ 623/624 (all except B5)

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-07
- **Notes**: Follow-on from 0039 collision debug investigation. Addresses remaining 11 test failures with 4 subtickets targeting per-contact depth, split impulse, edge contacts, and warm-starting.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
