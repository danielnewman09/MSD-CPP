# Iteration Log — 0062b_replay_linear_collision_tests

> **Purpose**: Track every build-test cycle during implementation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/designs/0062b_replay_linear_collision_tests/iteration-log.md`
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0062b_replay_linear_collision_tests
**Branch**: 0062b-replay-linear-collision-tests
**Baseline**: 728/732 tests passing at start (4 pre-existing failures)

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 1 — 2026-02-13 23:25
**Commit**: c62f13d
**Hypothesis**: Convert all 10 tests by replacing WorldModel+createSpherePoints() with ReplayEnabledTest fixture using small_sphere assets (0.5m radius to match original geometry).
**Changes**:
- `msd/msd-sim/test/Physics/Collision/LinearCollisionTest.cpp`: Converted 6 tests (A1-A6), deleted createSpherePoints()/createCubePoints() helpers
- `msd/msd-sim/test/Physics/Collision/EnergyAccountingTest.cpp`: Converted 4 tests (F1,F2,F3,F5), kept computeSystemEnergy() helper
- Used `small_sphere` asset (0.5m radius) to match original `createSpherePoints(0.5)` geometry
- Used `spawnInertial()` and `spawnInertialWithVelocity()` from 0062a fixture
- Used `disableGravity()` for zero-gravity collision tests
- Each test produces `.db` recording automatically
**Build Result**: PASS (5 warnings for unused variables — cosmetic)
**Test Result**: 728/732 passing (all 10 converted tests passing, 4 pre-existing failures)
**Impact vs Previous**: +0 passes, -0 regressions, net 0 change (conversion successful)
**Assessment**: Full success. All requirements met. Original test semantics preserved. Recordings produced in `replay/recordings/`. Ready for final handoff.

