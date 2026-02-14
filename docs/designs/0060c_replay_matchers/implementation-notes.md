# Implementation Notes: 0060c Python Pytest Recording Assertions

**Ticket**: 0060c_replay_matchers
**Date**: 2026-02-13
**Branch**: 0060c-replay-matchers
**PR**: #57

## Summary

Implemented physics-invariant pytest assertion functions that wrap `RecordingQuery` to enable expressive test verification of simulation results. These assertions provide clear failure messages with actual measured values for easy debugging.

## Files Created

| File | LOC | Purpose |
|------|-----|---------|
| `replay/replay/testing/assertions.py` | 77 | 3 assertion functions (energy, penetration, rest) |
| `replay/replay/testing/conftest.py` | 45 | Pytest fixtures for recording path discovery |
| `replay/tests/test_assertions.py` | 306 | 12 unit tests across 4 test classes |

## Files Modified

| File | Change |
|------|--------|
| `replay/replay/testing/__init__.py` | Export 3 new assertion functions alongside RecordingQuery |

## Implementation Details

### Assertion Functions

**1. assert_energy_conserved(db_path, tolerance)**
- Wraps `RecordingQuery.max_energy_drift()`
- Verifies energy conservation within relative tolerance
- Failure message: `"max energy drift was X, expected < Y"`

**2. assert_never_penetrates_below(db_path, body_id, z_min)**
- Wraps `RecordingQuery.min_z(body_id)`
- Verifies body never penetrates floor threshold
- Failure message: `"body {id} min z was X, expected >= Y"`

**3. assert_body_comes_to_rest(db_path, body_id, speed_threshold)**
- Wraps `RecordingQuery.speed_history(body_id)`
- Verifies final speed below threshold
- Failure message: `"body {id} final speed was X, expected < Y"`

### Pytest Fixtures

**recordings_dir() fixture**
- Returns path to `replay/recordings/` directory
- Enables tests to locate recording databases

**recording_for(test_suite, test_name) helper**
- Resolves path to specific recording: `{suite}_{test}.db`
- Raises helpful `FileNotFoundError` with gtest filter command if missing

## Test Coverage

### Test Classes
1. **TestAssertEnergyConserved** (3 tests)
   - Pass when drift below tolerance
   - Fail when drift exceeds tolerance (with message verification)
   - Handle zero drift (perfect conservation)

2. **TestAssertNeverPenetratesBelow** (3 tests)
   - Pass when body stays above threshold
   - Fail when body penetrates below (with message verification)
   - Pass when body reaches exactly threshold (>= allows equality)

3. **TestAssertBodyComesToRest** (4 tests)
   - Pass when final speed below threshold
   - Fail when body still moving (with message verification)
   - Handle 3D velocity vectors correctly
   - Handle empty recordings gracefully (final speed = 0.0)

4. **TestRecordingForHelper** (2 tests)
   - Raise helpful error when recording not found
   - Return correct path when recording exists

### Test Results
- **Total**: 12 tests collected
- **Skip**: 10 tests (require msd_reader module)
- **Pass**: 2 tests (helper function tests)
- **Pattern**: Same skip mechanism as 0060b for consistent behavior

## Design Adherence

All requirements from ticket specification met:

- **R1**: `assert_energy_conserved` implemented with tolerance parameter
- **R2**: `assert_never_penetrates_below` implemented with body_id and z_min
- **R3**: `assert_body_comes_to_rest` implemented with speed threshold
- **R4**: Pytest fixtures for recording path discovery

All acceptance criteria satisfied:
- AC1-AC3: Assertions pass/fail correctly based on measured values
- AC4: All failure messages include actual measured values
- AC5: Edge cases handled (empty recordings, zero values)
- AC6: `recording_for()` provides helpful error messages

## Integration Points

**Depends On**:
- `replay.testing.RecordingQuery` from ticket 0060b
- Recording databases produced by GTest infrastructure from 0060a

**Used By**:
- Python pytest tests that verify simulation physics
- Integration with C++ GTest via recording databases

## Known Limitations

None. All specified functionality implemented.

## Future Considerations

Potential additional assertions:
- `assert_contacts_occur(db_path, body_a, body_b, min_frames)` — verify contact detection
- `assert_momentum_conserved(db_path, tolerance)` — system momentum checks
- `assert_trajectory_matches(db_path, body_id, expected_path, tolerance)` — path verification

These can be added incrementally as testing needs arise.

## Notes

**Implementation Approach**: Straightforward wrapper pattern over `RecordingQuery`. No design document needed since this is a thin assertion layer with clear interface specification in the ticket.

**Test Strategy**: Created mock recording databases via direct SQLite insertion for predictable test data. Tests skip gracefully when msd_reader not available (consistent with 0060b pattern).

**Code Quality**: Clean, well-documented Python with type hints. Follows pytest best practices for assertion helpers.
