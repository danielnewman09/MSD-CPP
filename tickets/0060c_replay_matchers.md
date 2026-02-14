# Ticket 0060c: Python Pytest Recording Assertions

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Review
**Type**: Feature / Testing
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-13
**Generate Tutorial**: No
**Parent Ticket**: [0060_replay_integrated_gtest](0060_replay_integrated_gtest.md)
**Depends On**: [0060b_recording_query_api](0060b_recording_query_api.md)

---

## Overview

Create Python pytest assertion helpers that wrap `RecordingQuery` to provide physics-invariant checks with clear failure messages. These enable expressive test assertions like:

```python
assert_energy_conserved(recording_path, tolerance=0.05)
assert_never_penetrates_below(recording_path, body_id=1, z_min=-0.6)
assert_body_comes_to_rest(recording_path, body_id=1, speed_threshold=0.1)
```

On failure, assertions report the actual measured value (e.g., `"max energy drift was 0.12, expected < 0.05"`) for easy debugging.

---

## Requirements

### R1: assert_energy_conserved

```python
def assert_energy_conserved(db_path: str | Path, tolerance: float) -> None:
```

- Takes `tolerance` (float) — maximum acceptable relative energy drift
- Creates `RecordingQuery`, calls `max_energy_drift()`
- Passes if `max_energy_drift() < tolerance`
- On failure: raises `AssertionError` with message `"max energy drift was X, expected < Y"`

### R2: assert_never_penetrates_below

```python
def assert_never_penetrates_below(db_path: str | Path, body_id: int, z_min: float) -> None:
```

- Creates `RecordingQuery`, calls `min_z(body_id)`
- Passes if `min_z(body_id) >= z_min`
- On failure: raises `AssertionError` with message `"body {body_id} min z was X, expected >= Y"`

### R3: assert_body_comes_to_rest

```python
def assert_body_comes_to_rest(db_path: str | Path, body_id: int, speed_threshold: float) -> None:
```

- Creates `RecordingQuery`, calls `speed_history(body_id)`
- Passes if the final speed value is below `speed_threshold`
- On failure: raises `AssertionError` with message `"body {body_id} final speed was X, expected < Y"`

### R4: Pytest Fixture for Recording Path Discovery

```python
@pytest.fixture
def recording_path(request) -> Path:
```

A pytest fixture that resolves recording paths from the `replay/recordings/` directory. Can be parameterized with test suite/test name patterns to locate specific recording `.db` files produced by C++ GTest.

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `replay/replay/testing/assertions.py` | Physics-invariant assertion functions |
| `replay/replay/testing/conftest.py` | Pytest fixtures for recording path discovery |

### Modified Files
| File | Change |
|------|--------|
| `replay/replay/testing/__init__.py` | Export assertion functions alongside RecordingQuery |

---

## Interface

```python
from pathlib import Path
from .recording_query import RecordingQuery


def assert_energy_conserved(db_path: str | Path, tolerance: float) -> None:
    """Assert that system energy is conserved within the given relative tolerance.

    Raises AssertionError with actual drift value on failure.
    """
    q = RecordingQuery(db_path)
    drift = q.max_energy_drift()
    assert drift < tolerance, f"max energy drift was {drift:.6f}, expected < {tolerance}"


def assert_never_penetrates_below(
    db_path: str | Path, body_id: int, z_min: float
) -> None:
    """Assert that a body's z-position never drops below the given threshold.

    Raises AssertionError with actual min z on failure.
    """
    q = RecordingQuery(db_path)
    actual_min_z = q.min_z(body_id)
    assert actual_min_z >= z_min, (
        f"body {body_id} min z was {actual_min_z:.6f}, expected >= {z_min}"
    )


def assert_body_comes_to_rest(
    db_path: str | Path, body_id: int, speed_threshold: float
) -> None:
    """Assert that a body's final speed is below the given threshold.

    Raises AssertionError with actual final speed on failure.
    """
    q = RecordingQuery(db_path)
    speeds = q.speed_history(body_id)
    final_speed = speeds[-1] if speeds else 0.0
    assert final_speed < speed_threshold, (
        f"body {body_id} final speed was {final_speed:.6f}, expected < {speed_threshold}"
    )
```

### Pytest Fixture

```python
# replay/replay/testing/conftest.py
import pytest
from pathlib import Path

RECORDINGS_DIR = Path(__file__).parent.parent.parent / "recordings"


@pytest.fixture
def recordings_dir() -> Path:
    """Path to the recordings directory."""
    return RECORDINGS_DIR


def recording_for(test_suite: str, test_name: str) -> Path:
    """Resolve path to a specific recording produced by C++ GTest."""
    path = RECORDINGS_DIR / f"{test_suite}_{test_name}.db"
    if not path.exists():
        raise FileNotFoundError(
            f"Recording not found: {path}\n"
            f"Run C++ tests first: ./build/Debug/debug/msd_sim_test --gtest_filter=\"{test_suite}*\""
        )
    return path
```

---

## Test Plan

### Unit Tests

```python
def test_assert_energy_conserved_passes_when_drift_below_tolerance(sample_recording):
    """Verify no assertion raised when energy drift is within tolerance."""

def test_assert_energy_conserved_fails_when_drift_exceeds_tolerance(sample_recording):
    """Verify AssertionError raised with actual drift in message."""

def test_assert_never_penetrates_below_passes_when_above_threshold(sample_recording):
    """Verify no assertion raised when body stays above z threshold."""

def test_assert_never_penetrates_below_fails_when_below_threshold(sample_recording):
    """Verify AssertionError raised with actual min z in message."""

def test_assert_body_comes_to_rest_passes_when_final_speed_below_threshold(sample_recording):
    """Verify no assertion raised when body has settled."""

def test_assert_body_comes_to_rest_fails_when_still_moving(sample_recording):
    """Verify AssertionError raised with actual final speed in message."""

def test_recording_for_raises_when_file_missing():
    """Verify helpful error message when recording not found."""
```

---

## Acceptance Criteria

1. [x] **AC1**: `assert_energy_conserved` passes/fails correctly based on energy drift
2. [x] **AC2**: `assert_never_penetrates_below` passes/fails correctly based on min z position
3. [x] **AC3**: `assert_body_comes_to_rest` passes/fails correctly based on final speed
4. [x] **AC4**: All assertions report the actual measured value in failure messages
5. [x] **AC5**: Assertions handle edge cases (empty recordings, zero initial energy)
6. [x] **AC6**: `recording_for()` helper provides clear error when recording `.db` not found

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-02-13 18:30
- **Completed**: 2026-02-13 18:45
- **Branch**: 0060c-replay-matchers
- **PR**: #57
- **Artifacts**:
  - `replay/replay/testing/assertions.py` (77 LOC) — 3 assertion functions
  - `replay/replay/testing/conftest.py` (45 LOC) — pytest fixtures
  - `replay/tests/test_assertions.py` (306 LOC) — 12 unit tests
  - `replay/replay/testing/__init__.py` (modified) — export assertions
- **Notes**:
  - All 3 assertion functions implemented as specified
  - Tests skip gracefully when msd_reader not available (same pattern as 0060b)
  - Helper function tests pass without msd_reader dependency
  - Clear error messages with actual measured values on assertion failures
  - All acceptance criteria met

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
