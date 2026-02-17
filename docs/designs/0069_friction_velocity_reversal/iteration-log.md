# Iteration Log — 0069_friction_velocity_reversal

> **Purpose**: Track every build-test cycle during implementation or investigation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/designs/0069_friction_velocity_reversal/iteration-log.md`
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0069_friction_velocity_reversal
**Branch**: 0069-friction-velocity-reversal
**Baseline**: 691/697 tests passing at start

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 1 — 2026-02-17 11:28
**Commit**: ea52275
**Hypothesis**: Add sliding state tracking to ContactCache to detect sustained sliding contacts
**Changes**:
- `msd/msd-sim/src/Physics/Constraints/ContactCache.hpp`: Added `slidingDirection` (optional Vector3D) and `slidingFrameCount` (int) to `CachedContact` struct
- `msd/msd-sim/src/Physics/Constraints/ContactCache.hpp`: Added `updateSlidingState()` and `getSlidingState()` methods
- `msd/msd-sim/src/Physics/Constraints/ContactCache.cpp`: Implemented sliding state tracking logic (velocity threshold check, direction normalization, frame count increment/reset)
**Build Result**: PASS
**Test Result**: 691/697 (baseline)
**Impact vs Previous**: No change (baseline maintained)
**Assessment**: Infrastructure for sliding state tracking in place. Next: add `setSlidingMode()` to `FrictionConstraint` to override tangent basis.

### Iteration 2 — 2026-02-17 11:35
**Commit**: 76c8d4a
**Hypothesis**: Add `setSlidingMode()` to FrictionConstraint to override tangent basis when sliding detected
**Changes**:
- `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.hpp`: Added `setSlidingMode(Vector3D)` method declaration, `is_sliding_mode_` flag, `isSlidingMode()` query, included Vector3D header
- `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.cpp`: Implemented `setSlidingMode()` to align t1 with -slidingDirection, compute t2 = normal × t1, set flag
**Build Result**: PASS
**Test Result**: 691/697 (baseline)
**Impact vs Previous**: No change (baseline maintained)
**Assessment**: FrictionConstraint can now override its tangent basis for sliding mode. Next: modify NLoptFrictionSolver to support unilateral tangent bounds (lambda_t1 >= 0).
