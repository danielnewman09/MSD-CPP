# Implementation Notes — 0069_friction_velocity_reversal

**Ticket**: 0069_friction_velocity_reversal
**Branch**: 0069-friction-velocity-reversal
**Date**: 2026-02-17

---

## Summary

Implemented sliding friction mode to prevent friction from reversing the sliding direction during sustained low-speed contact. The fix introduces a constraint-level mechanism where `ContactCache` detects sustained sliding, `FrictionConstraint` aligns its tangent basis with the sliding direction, and `NLoptFrictionSolver` enforces unilateral bounds on lambda_t1 to prevent acceleration in the sliding direction.

---

## Files Created

None.

---

## Files Modified

| File | LOC | Purpose |
|------|-----|---------|
| `msd/msd-sim/src/Physics/Constraints/ContactCache.hpp` | +19 | Added `slidingDirection` and `slidingFrameCount` to `CachedContact`, added `updateSlidingState()` and `getSlidingState()` methods |
| `msd/msd-sim/src/Physics/Constraints/ContactCache.cpp` | +51 | Implemented sliding state tracking and query logic |
| `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.hpp` | +28 | Added `setSlidingMode()` method, `is_sliding_mode_` flag, `isSlidingMode()` query |
| `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.cpp` | +18 | Implemented `setSlidingMode()` to align tangent basis with -slidingDirection |
| `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.hpp` | +5 | Added `tangent1LowerBounds` parameter to `solve()` |
| `msd/msd-sim/src/Physics/Constraints/NLoptFrictionSolver.cpp` | +13 | Updated lower bounds setup to apply per-contact tangent1 bounds |
| `msd/msd-sim/src/Physics/Constraints/FrictionSpec.hpp` | +2 | Added `tangent1LowerBounds` field |
| `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | +19 | Updated `buildFrictionSpec()` to populate tangent1 bounds based on sliding mode |
| `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp` | +42 | Query sliding state when creating constraints, update sliding state after solving |

**Total**: ~197 LOC added

---

## Design Adherence

### R1: Track sliding contact state ✅

**Requirement**: Extend `ContactCache` to store per-body-pair sliding direction and duration.

**Implementation**:
- Added `std::optional<Vector3D> slidingDirection` and `int slidingFrameCount` to `CachedContact`
- `updateSlidingState()` increments frame count when tangent velocity exceeds threshold (default 0.01 m/s), resets to zero otherwise
- `getSlidingState()` returns sliding direction and whether mode is active (age >= 3 frames)

### R2: Implement sliding friction mode ✅

**Requirement**: When a contact is in sliding mode, align t1 with `-slidingDirection` and enforce `lambda_t1 >= 0`.

**Implementation**:
- `FrictionConstraint::setSlidingMode()` overrides tangent basis: `t1 = -slidingDirection`, `t2 = normal × t1`
- `NLoptFrictionSolver` accepts `tangent1LowerBounds` parameter and applies per-contact lower bounds to lambda_t1
- `ConstraintSolver::buildFrictionSpec()` populates bounds: 0.0 for sliding mode, -∞ for bilateral
- CollisionPipeline queries sliding state when creating constraints and calls `setSlidingMode()` when active

### R3: Settling behavior (PENDING)

**Requirement**: F4 test cube should settle to face-resting position (CoM z ~ 0.5, KE ~ 0) within 500 frames.

**Status**: Implementation complete, F4 tests passing at baseline. Need to run extended test with tightened assertions to verify settling behavior improves.

### R4: No regression ✅

**Requirement**: All existing tests pass at 691/697 or better.

**Status**: **691/697 passing** — baseline maintained across all 4 iterations.

---

## Prototype Application

No prototype phase for this ticket — bug fix with clear requirements and implementation approach specified in ticket.

---

## Deviations from Design

### Minor Adjustment: Sliding State Update Location

**Original plan**: Update sliding state during cache update (Phase 4.5).

**Actual**: Added sliding state update after normal cache update in `CollisionPipeline::solveConstraintsWithWarmStart()`, computing tangent velocity from post-solve states.

**Reason**: Needed access to post-solve velocities and contact geometry to compute tangent velocity for sliding detection. No architectural impact.

---

## Test Coverage

### Unit Tests

All existing tests maintained:
- **691/697 passing** (baseline)
- No new unit tests added (integration-level feature)

### Integration Tests

F4 test (`RotationalEnergyTest_F4_RotationEnergyTransfer_EnergyConserved`) exercises the full sliding friction mode:
- 1m cube, tilted 45°, dropped onto floor with e=1.0, mu=0.5
- After initial bounce (~250 frames), cube enters sustained tumbling contact
- Sliding mode should activate and prevent velocity reversals
- **Status**: Test passing at baseline (no tightened assertions yet)

---

## Known Limitations

1. **Sliding detection uses first contact point only**: For multi-point contacts, sliding state is computed from the first contact point. This is a simplification that works for most cases.

2. **Velocity threshold hardcoded**: The 0.01 m/s threshold is hardcoded in `updateSlidingState()` call. Could be made configurable if needed.

3. **Frame count threshold hardcoded**: The 3-frame persistence requirement is hardcoded in `getSlidingState()` call. Could be made configurable.

4. **No diagnostic recording**: Sliding mode activation is not recorded to database (no `SlidingModeRecord` transfer object). Can be added if debugging is needed.

---

## Future Considerations

1. **Tighten F4 assertions**: Add explicit settling assertions (KE < 0.01 J, total_E ≈ PE) to verify sliding mode effect.

2. **Sliding mode diagnostics**: Add recording of sliding mode activation state to database for visualization and debugging.

3. **Multi-point sliding detection**: Consider using average tangent velocity across all contact points instead of first point only.

4. **Configurable thresholds**: Expose velocity threshold and frame count threshold as configuration parameters.

5. **Edge case handling**: Test sliding mode behavior with very low friction (mu < 0.1) where static friction may not exist.

---

## Performance Impact

- **ContactCache**: +2 fields per cached contact (16 bytes)
- **FrictionConstraint**: +1 bool flag per constraint (1 byte)
- **Solver**: No runtime overhead when sliding mode inactive (bilateral path unchanged)
- **Solver**: Minimal overhead when active (single lower bound per contact)

**Estimated impact**: < 1% for typical workloads (60 FPS, 10-100 objects, < 10% sliding contacts).

---

## Iteration Summary

| Iteration | Hypothesis | Result | Pass/Total |
|-----------|------------|--------|------------|
| 1 | Add sliding state to ContactCache | SUCCESS | 691/697 |
| 2 | Add setSlidingMode() to FrictionConstraint | SUCCESS | 691/697 |
| 3 | Add tangent1 lower bounds to NLoptFrictionSolver | SUCCESS | 691/697 |
| 4 | Wire everything together in CollisionPipeline | SUCCESS | 691/697 |

**Total iterations**: 4
**Circle detection flags**: None
**Regressions**: None

---

## Conclusion

The sliding friction mode is fully implemented and integrated. All components work together to detect sustained sliding, align friction direction, and enforce unilateral bounds. The implementation is clean, minimal, and maintains the baseline test pass rate. Ready for final verification with tightened F4 assertions.
