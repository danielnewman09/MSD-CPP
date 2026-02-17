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

### Iteration 3 — 2026-02-17 11:42
**Commit**: 44c2f6b
**Hypothesis**: Add tangent1 lower bounds parameter to NLoptFrictionSolver to enforce unilateral friction in sliding mode
**Changes**:
- `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.hpp`: Added `tangent1LowerBounds` parameter to `solve()` signature
- `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.cpp`: Updated lower bounds setup to apply per-contact tangent1 bounds when provided (lambda_t1_i >= tangent1LowerBounds[i])
**Build Result**: PASS
**Test Result**: 691/697 (baseline)
**Impact vs Previous**: No change (baseline maintained)
**Assessment**: NLopt solver can now enforce unilateral friction bounds. Next: wire everything together in CollisionPipeline — query ContactCache for sliding state, call setSlidingMode() on FrictionConstraint, pass tangent1LowerBounds to solver.

### Iteration 4 — 2026-02-17 11:52
**Commit**: 426de2e
**Hypothesis**: Complete integration by wiring sliding mode detection and enforcement through CollisionPipeline
**Changes**:
- `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp`: Query `getSlidingState()` when creating FrictionConstraints, call `setSlidingMode()` if active, compute tangent velocity and call `updateSlidingState()` after solving
- `msd/msd-sim/src/Physics/Constraints/FrictionSpec.hpp`: Added `tangent1LowerBounds` vector field
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`: Updated `buildFrictionSpec()` to populate tangent1LowerBounds (0.0 for sliding mode, -∞ for bilateral), pass to NLoptFrictionSolver
**Build Result**: PASS
**Test Result**: 691/697 (baseline)
**Impact vs Previous**: No change (baseline maintained), F4 tests still passing
**Assessment**: Full sliding friction mode implementation complete. All components integrated: ContactCache tracks sliding state, FrictionConstraint aligns tangent basis, NLoptFrictionSolver enforces unilateral bounds, CollisionPipeline orchestrates the workflow. Ready for final testing on F4 settling behavior.
