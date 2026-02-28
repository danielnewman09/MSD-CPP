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
