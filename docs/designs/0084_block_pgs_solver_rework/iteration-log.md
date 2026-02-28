# Iteration Log: Block PGS Solver Rework

**Ticket**: [0084_block_pgs_solver_rework](../../../tickets/0084_block_pgs_solver_rework.md)

---

## Iteration 1 — Prototype P1: Warm-Start Disable Diagnostic

**Date**: 2026-02-28
**Phase**: Prototype
**Change**: Set `const bool hasWarmStart = false` in `BlockPGSSolver::solve()`
**Goal**: Confirm that warm-start contamination is the root cause of all 12 failures

### Result: NEGATIVE

All 12 tests still fail with identical failure values:

| Test | Z-Vel / KE ratio before | After warm-start disable | Change |
|------|------------------------|--------------------------|--------|
| Oblique45 | 21.1 m/s | 21.1 m/s | None |
| HighSpeedOblique | 43.4 m/s | 43.4 m/s | None |
| Oblique45_Medium | 10.0 m/s | 10.0 m/s | None |
| Oblique45_Slow | 3.66 m/s | 3.66 m/s | None |
| InelasticBounce | 0.057 ratio | 0.057 ratio | None |
| EqualMassElastic | omegaZ=3.14 | omegaZ=3.14 | None |

### Finding

The Z-velocity injection occurs within the first 10 simulation frames, independent of
warm-start state. This is a per-frame Phase B problem. The K_nt off-diagonal coupling
in `K_inv` maps tangential velocity error (`vErr(1,2) != 0`) to a normal impulse
correction (`unconstrained(0) != 0`) even when `vErr(0) = 0`.

### Next Step

Design revision required. The warm-start fix (F1) does not address the actual root
cause. Need to investigate:
1. Zero the normal correction when `vErr(0) >= 0` (contact not approaching)
2. Or use a decoupled normal/tangent solve

---

## Iteration 2 — Design Revision: Decoupled Normal/Tangent Solve

**Date**: 2026-02-28
**Phase**: Design Revision
**Change**: Revised design document and PlantUML diagram. New root cause identified:
K_nt off-diagonal coupling in 3x3 block solve drives normal impulse from tangential
velocity error on every frame. Fix: decouple normal row (scalar K_nn) from tangent
rows (2x2 K_tt subblock) in Phase B `sweepOnce`.

### Human Decision

The human reviewed the prototype P1 findings and approved the following approach:
- **Full decoupled solve**: scalar K_nn for normal row, 2x2 K_tt for tangent rows independently
- **Do NOT use Hypothesis B** (zeroing `unconstrained(0)` when `vErr(0) >= 0`) — rejected as
  a band-aid
- **Preserve two-phase architecture** (Phase A restitution + Phase B dissipative)
- **Prototype P2 required** to validate decoupled solve before full implementation

### Design Changes from Revision 1

| What changed | Old approach | New approach |
|---|---|---|
| Primary fix | F1: Phase B-only cache storage (phaseBLambdas) | F1 (revised): Decoupled normal/tangent solve in sweepOnce |
| Root cause | Warm-start contamination (Phase A bounce in cache) | K_nt coupling in 3x3 block solve drives normal impulse from tangential vErr |
| Files changed for primary fix | BlockPGSSolver.hpp, BlockPGSSolver.cpp, ConstraintSolver, CollisionPipeline | BlockPGSSolver.cpp only (sweepOnce function) |
| Secondary fix | N/A | phaseBLambdas cache split retained as secondary correctness improvement |
| Previous blockKInvs | Precomputed 3x3 K^{-1}, passed to sweepOnce | Removed; use blockKs directly with per-contact scalar/2x2 LDLT |

### Prototype P2 Target

Before full implementation, prototype P2 must validate:
- Replace `blockKInvs[ci] * (-vErr)` with decoupled scalar/2x2 solve in sweepOnce
- All 5 oblique sliding tests must pass (Z-velocity < 2.0 m/s)
- At least 9 of 12 total failing tests must pass
- Zero regression in a sample of currently-passing tests
