# Ticket 0075b: Phase 2 — Block PGS Solver

## Status
- [x] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Implementation
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Implementation
**Priority**: High
**Created**: 2026-02-22
**Generate Tutorial**: No
**Parent Ticket**: [0075](0075_unified_contact_constraint.md)
**Blocked By**: [0075a](0075a_unified_constraint_data_structure.md)
**Related Tickets**: [0067](0067_contact_phase_energy_injection.md) (energy injection), [0069](0069_friction_velocity_reversal.md) (sliding mode), [0070](0070_nlopt_convergence_energy_injection.md) (decoupled solve), [0073](0073_hybrid_pgs_large_islands.md) (hybrid PGS)

---

## Summary

Implement the two-phase `BlockPGSSolver` (Phase A: restitution pre-solve, Phase B: dissipative block PGS), wire it into `ConstraintSolver`, remove `FrictionConstraint`, and validate with replay-enabled integration tests.

---

## Scope

### In Scope
1. Implement `BlockPGSSolver` with two-phase structure
   - `applyRestitutionPreSolve()` (Phase A)
   - `sweepOnce()` (Phase B)
   - `projectCoulombCone()`, `buildBlockK()`, `updateVResNormalOnly()`, `computeBlockVelocityError()`
2. Wire `BlockPGSSolver` into `ConstraintSolver::solve()` for the friction dispatch path
3. Remove `FrictionConstraint` class and all references
4. Run investigation I1 (K_nt analysis) on existing recordings to validate coupling assumption
5. Run replay-enabled integration tests (ramp slide, energy conservation, Phase A isolation tests)
6. Add 30-degree ramp asset to `generate_test_assets` and regenerate test `assets.db`

### Out of Scope
- NLopt removal (ticket 0075c — kept behind runtime toggle)
- Benchmark regression tests (benchmark infrastructure not functional)

---

## Test Requirements

All integration tests must be **replay-enabled** (recording to `.db` via `DataRecorder`) with strict, thorough assertions on per-frame impulse magnitudes, energy deltas, and convergence metrics.

See design document test plan for full unit test and integration test listings.

---

## Design Reference

Full design: [`docs/designs/0075_unified_contact_constraint/design.md`](../docs/designs/0075_unified_contact_constraint/design.md), Phase 2 section.

---

## Workflow Log

### Draft
- **Date**: 2026-02-22
- **Notes**: Split from parent ticket 0075 per PR #94 review feedback.
