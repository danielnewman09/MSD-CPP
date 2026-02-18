# Iteration Log — 0071a_constraint_solver_scalability

> **Purpose**: Track every build-test cycle during implementation or investigation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/designs/0071a_constraint_solver_scalability/iteration-log.md`
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0071a_constraint_solver_scalability
**Branch**: 0071a-constraint-solver-scalability
**Baseline**: 697/697 tests passing at start

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 1 — 2026-02-17 00:00
**Commit**: (pending)
**Hypothesis**: Implement ConstraintIslandBuilder + integrate island decomposition into CollisionPipeline::solveConstraintsWithWarmStart() and correctPositions(). This is the core O(n) partitioning step that reduces solver complexity from O((3C)³) global to O(Σᵢ (3Cᵢ)³) per-island.
**Changes**:
- `msd/msd-sim/src/Physics/Constraints/ConstraintIslandBuilder.hpp`: New static utility class with Island struct and buildIslands() static method
- `msd/msd-sim/src/Physics/Constraints/ConstraintIslandBuilder.cpp`: Union-find implementation with path compression and union-by-rank; environment bodies excluded from connectivity
- `msd/msd-sim/src/Physics/Constraints/CMakeLists.txt`: Added ConstraintIslandBuilder.cpp to target_sources
- `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp`: Added ConstraintIslandBuilder.hpp include
- `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp`: Replaced global solve in solveConstraintsWithWarmStart() with per-island solve loop; added island decomposition to correctPositions(); added unordered_map/unordered_set/algorithm includes
- `msd/msd-sim/test/Physics/Constraints/ConstraintIslandBuilderTest.cpp`: 11 unit tests covering empty, single pair, independent pairs, chain, star, environment non-connectivity, mixed connectivity, body index correctness, multi-contact, constraint order preservation
- `msd/msd-sim/test/Physics/Constraints/CMakeLists.txt`: Added ConstraintIslandBuilderTest.cpp
- `docs/designs/0071a_constraint_solver_scalability/iteration-log.md`: This file (created)
**Build Result**: PASS (no errors, no warnings)
**Test Result**: 708/708 — 11 new tests added and passing; all 697 prior tests pass
**Impact vs Previous**: +11 passes, 0 regressions, net +11
**Assessment**: Implementation complete in first iteration. All unit tests pass. Physics tests fully preserved — island decomposition produces identical forces to global solve as expected (single-island cases are mathematically equivalent). The critical correctness condition (environment bodies excluded from connectivity) is validated by buildIslands_environmentDoesNotConnect test.

