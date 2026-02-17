# Iteration Log — 0070-nlopt-convergence-energy-injection

> **Purpose**: Track every build-test cycle during implementation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/designs/0070-nlopt-convergence-energy-injection/iteration-log.md`
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0070_nlopt_convergence_energy_injection
**Branch**: 0070-nlopt-convergence-energy-injection
**Baseline**: 690/697 tests passing at start (from ticket)

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 1 — 2026-02-17 12:50
**Commit**: 3ad6a78
**Hypothesis**: Decouple normal and friction solve. Per-contact independent friction with analytic ball projection will eliminate energy injection from friction cone constraint.
**Changes**:
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`: Replaced coupled NLopt solver with decoupled approach (normal ASM, then per-contact friction with ball projection)
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`: Removed solveWithFriction, clamp function declarations
- Fixed velocity update formula: `Jv_new = -b + A*lambda` (not `b + A*lambda`)
**Build Result**: PASS
**Test Result**: 685/697 — F4 fixed (+1), A4/A6/D4/H5/H6 regressed (-5), friction tests failing
**Impact vs Previous**: +1 pass (F4 energy conservation), -5 regressions
**Assessment**: Per-contact independent friction solve is too approximate. Multi-contact coupling through A matrix ignored. A6 worse (1.39J vs 0.286J ticket claim). Need Gauss-Seidel iteration or joint tangent solve.

### Iteration 2 — 2026-02-17 13:10
**Commit**: e3fd5bf
**Hypothesis**: Gauss-Seidel iteration will handle multi-contact coupling and fix A6 energy conservation. Iterate per-contact solves with RHS updated from current friction impulses.
**Changes**:
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`: Added Gauss-Seidel outer loop (max 10 iterations, convergence tolerance 1e-6), per-contact RHS updated with current friction from other contacts: `jvCurrent = jvPostNormal + A * lambdaFriction`
**Build Result**: PASS
**Test Result**: 688/697 — A6 fixed (+1), A4 still failing, D4/H3/H5/H6/B2/F4b/FrictionConeTest still failing
**Impact vs Previous**: +3 passes (A6 glancing collision now conserves energy, 2 others), -2 regressions vs baseline (690)
**Assessment**: Significant improvement! A6 litmus test passes (energy conserved < 0.1J). Only 2 failures away from baseline. Remaining failures may be pre-existing or edge cases. Check if baseline failures list is available.
