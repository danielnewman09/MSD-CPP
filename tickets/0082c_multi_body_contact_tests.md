# Ticket 0082c: Multi-Body Contact Tests

## Status
- [x] Draft
- [x] Ready for Implementation
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Implementation
**Type**: Testing
**Priority**: Medium
**Created**: 2026-02-26
**Generate Tutorial**: No
**Parent Ticket**: [0082](0082_collision_test_restructure.md)
**Blocked By**: None (can proceed in parallel with 0082a, 0082b)
**Related Tickets**: [0071a](0071a_constraint_solver_scalability.md) (island builder), [0073](0073_hybrid_pgs_large_islands.md) (hybrid PGS)

---

## Summary

Add multi-body contact scenario tests. The current test suite only covers two-body interactions (one body pair, or one body against floor). No tests exist for stacking, cascading collisions, or pile stability — all core simulation scenarios that exercise island decomposition, solver convergence under coupling, and contact persistence.

---

## Problem

### Missing Scenarios

Every collision test involves at most 2 bodies (one inertial + one environment, or two inertial). Real simulations involve:

- **Stacking**: Object A rests on B which rests on floor. The contact forces at B-floor must support both A and B. This requires the solver to handle coupled constraints across multiple contacts.
- **Cascading impacts**: Object A hits B, which then hits C. The island builder must correctly identify the chain and the solver must propagate impulses.
- **Simultaneous multi-contact**: Multiple objects resting on or colliding with the same surface simultaneously. The island builder should decompose into independent islands (bodies connected only through environment don't form one island — already tested in ConstraintIslandBuilderTest).
- **Pile stability**: Multiple objects at rest in a configuration that requires sustained normal forces across many contacts. Tests long-term stability.

### Why This Matters

Multi-body scenarios stress:
1. **Island decomposition correctness** — ConstraintIslandBuilder unit tests exist, but no integration test verifies correct island building from actual collision detection
2. **Solver convergence with many contacts** — PGS convergence degrades with contact count; Block PGS coupling may help or hurt
3. **Contact cache coherence** — warm-starting from frame N to N+1 with evolving contact topology
4. **Position correction accumulation** — Baumgarte stabilization across multiple penetrating pairs

---

## Scope

### Test File
`msd/msd-sim/test/Physics/Collision/MultiBodyContactTest.cpp` (ReplayEnabledTest fixture)

### Required Tests

#### Stacking
1. **TwoBoxStack_StableFor500Frames** — Box A (1kg) rests on Box B (1kg) which rests on floor. Both remain stable (drift < 0.05m) for 500 frames. Verify A stays above B, B stays above floor.
2. **TwoBoxStack_TopBoxNormalForce** — In the stack, the floor contact normal force should support ~2mg (both boxes). Verify via replay recording contact forces.
3. **ThreeBoxStack_StableFor200Frames** — A on B on C on floor. All three remain stable. This is a harder test for solver convergence — 3 coupled contact pairs.

#### Cascading Impacts
4. **ContactCascade_AHitsBHitsC** — Box A moving horizontally hits stationary B, which then hits stationary C. Verify momentum transfer through the chain: A slows, B briefly moves, C accelerates.
5. **DropOntoStack_PerturbsLowerBox** — Drop box A onto resting box B on floor. Verify B absorbs impact without falling through floor, and A comes to rest on top of B.

#### Simultaneous Multi-Contact
6. **TwoBoxesOnFloor_IndependentIslands** — Two separated boxes dropped on floor simultaneously. They should not influence each other. Verify each box's behavior is identical to the single-box drop case.
7. **TwoBoxesCollide_OnFloor** — Two boxes sliding toward each other on a floor. Three-way contact: box-box and each box-floor. Verify boxes stop after collision without penetrating floor.

#### Stability
8. **FiveBoxPile_SettlesWithinBounds** — Drop 5 boxes from small heights onto floor in a loose pile. After 500 frames, all should be at rest (speed < threshold) and no box should have fallen through the floor.

---

## Acceptance Criteria

1. >= 6 multi-body contact tests
2. At least one 3-body stack test
3. At least one cascade/chain collision test
4. All tests use ReplayEnabledTest for recording
5. All test names are descriptive without ticket references
6. No tests depend on specific asset shapes beyond unit_cube and floor_slab (available in test DB)

---

## Notes

- The test database contains `unit_cube` (1x1x1m) and `floor_slab` (100x100x100m) assets. All multi-body tests should use these.
- Stack stability is sensitive to the gravity-timestep oscillation floor (~g*dt). The "stable" criterion should be drift < 0.05m, not zero drift.
- Island decomposition for environment-connected bodies is already tested at the unit level (ConstraintIslandBuilderTest::environmentDoesNotConnect). These integration tests verify the full pipeline.
- Pile tests with 5+ objects may require more than the default solver iteration count to converge. Tests should document expected iteration counts.
