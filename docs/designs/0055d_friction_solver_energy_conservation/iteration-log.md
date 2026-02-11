# Iteration Log — 0055d_friction_solver_energy_conservation

> **Purpose**: Track every build-test cycle during implementation or investigation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/designs/0055d_friction_solver_energy_conservation/iteration-log.md`
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0055d_friction_solver_energy_conservation
**Branch**: 0055c-friction-direction-fix (shared with 0055c parent)
**Baseline**: 690/699 tests passing at start (branch state from 0055c iteration 12b)
**Starting Point**: git tag `bookmark/0055c-iter12-visual-improvement` (commit ee80766)

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 1 — 2026-02-11 14:15
**Commit**: 2c8cc60
**Hypothesis**: The 1e-10 absolute threshold is too aggressive — it clamps all inflation, including legitimate increases needed for elastic bounces with friction. Use a 2.0× multiplicative threshold to allow moderate inflation while catching the extreme 3.75× tilted cube case.
**Changes**:
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`: Changed capping threshold from `coupledN > cleanN + 1e-10` to `coupledN > kInflationThreshold * cleanN && coupledN > cleanN + 1e-10` with `kInflationThreshold = 2.0`
**Build Result**: PASS (1 warning: unused `solveFrictionPGS`)
**Test Result**: 691/699 — 8 failures:
- Pre-existing (3): H3_TimestepSensitivity, B2_CubeEdgeImpact, B5_LShapeDrop
- Regressions vs baseline (5): A3_PerfectlyElastic, F2_ElasticBounce, F3_InelasticBounce, D4_MicroJitter, Sliding_PurePitch_vs_CompoundTilt_SpuriousY
- **Fixed**: A4_EqualMassElastic (was failing at 690/699)
**Impact vs Previous**: +1 pass (A4), -0 new regressions (all 5 regressions were present in 690/699 baseline)
**Assessment**: POSITIVE PROGRESS. The 2.0× threshold fixed A4 (elastic head-on collision energy conservation) without introducing new regressions. A3, F2, F3 still fail — these likely need a lower threshold (they're more sensitive to small inflation). D4 regression persists (resting contact stability), and the tilted cube SpuriousY test still fails. Next: try reducing threshold to 1.5× to catch more moderate inflation cases.
