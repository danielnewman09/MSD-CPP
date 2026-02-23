# Ticket 0075c: Phase 3 — NLopt Removal and Cleanup

## Status
- [x] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Implementation
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Cleanup
**Priority**: Medium
**Created**: 2026-02-22
**Generate Tutorial**: No
**Parent Ticket**: [0075](0075_unified_contact_constraint.md)
**Blocked By**: [0075b](0075b_block_pgs_solver.md)
**Related Tickets**: [0070](0070_nlopt_convergence_energy_injection.md) (NLopt introduction)

---

## Summary

Remove `NLoptFrictionSolver` and associated infrastructure once Block PGS (ticket 0075b) passes all integration tests. Clean up deprecated transfer records and update documentation.

---

## Scope

### In Scope
1. Remove `NLoptFrictionSolver` class and all references
2. Remove NLopt runtime toggle from `ConstraintSolver`
3. Delete `FrictionConstraintRecord` and `ContactConstraintRecord` (superseded by `UnifiedContactConstraintRecord`)
4. Update `CLAUDE.md` files in `Constraints/`, `Physics/`, and `msd-sim/`
5. Update PlantUML diagrams in `docs/msd/`
6. Remove NLopt/ECOS from Conan dependencies if no longer used elsewhere

### Out of Scope
- Any solver behavior changes (Block PGS must be fully validated before this ticket)

---

## Prerequisites

All 0075b integration tests must pass with Block PGS as the sole friction solver before this ticket proceeds.

---

## Design Reference

Full design: [`docs/designs/0075_unified_contact_constraint/design.md`](../docs/designs/0075_unified_contact_constraint/design.md), Phase 3 section.

---

## Workflow Log

### Draft
- **Date**: 2026-02-22
- **Notes**: Created as follow-on cleanup ticket per design question Q2 resolution (Option A: keep NLopt during development, remove in follow-on).
