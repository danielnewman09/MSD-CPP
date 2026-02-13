# Ticket 0060b: RecordingQuery Python Query API

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Feature / Testing
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-13
**Generate Tutorial**: No
**Parent Ticket**: [0060_replay_integrated_gtest](0060_replay_integrated_gtest.md)
**Depends On**: [0060a_replay_enabled_test_fixture](0060a_replay_enabled_test_fixture.md)

---

## Overview

Create a Python query module over recording databases that enables pytest tests to interrogate simulation results after C++ GTest execution. `RecordingQuery` opens a recording `.db` (read-only) via the existing `msd_reader` pybind11 module and provides typed queries for position/velocity timeseries, energy tracking, and contact event analysis.

This leverages the same `msd_reader` module already used by the FastAPI replay viewer's `SimulationService`, avoiding any duplication of database reading logic in C++.

---

## Requirements

### R1: Frame Queries

```python
def frame_count(self) -> int: ...
def total_simulation_time(self) -> float: ...
```

Read `SimulationFrameRecord` table via `msd_reader`. `total_simulation_time()` returns the `simulation_time` of the last frame.

### R2: Per-Body Position/Velocity Timeseries

```python
def position_history(self, body_id: int) -> list[tuple[float, float, float]]: ...
def velocity_history(self, body_id: int) -> list[tuple[float, float, float]]: ...
def speed_history(self, body_id: int) -> list[float]: ...
```

Read `InertialStateRecord` table, filter by `body_id`, ordered by frame ID. Return one entry per recorded frame. `speed_history()` returns velocity magnitude per frame.

### R3: Per-Body Scalar Aggregates

```python
def position_at_frame(self, body_id: int, frame_id: int) -> tuple[float, float, float]: ...
def min_z(self, body_id: int) -> float: ...
def max_z(self, body_id: int) -> float: ...
def max_speed(self, body_id: int) -> float: ...
```

Computed from the timeseries data. `min_z()` returns the minimum z-component of position across all frames — useful for "object never fell through the floor" assertions.

### R4: System Energy Queries

```python
def system_energy_history(self) -> list[float]: ...
def max_energy_drift(self) -> float: ...
```

Read `SystemEnergyRecord` table. `system_energy_history()` returns `total_system_e` per frame. `max_energy_drift()` computes `max |E(t) - E(0)| / |E(0)|` across all frames — the relative energy error.

### R5: Contact Event Queries

```python
def total_contact_frames(self) -> int: ...
def contact_frames_between(self, body_a_id: int, body_b_id: int) -> int: ...
```

Read `CollisionResultRecord` table. `total_contact_frames()` counts distinct frames with at least one collision record. `contact_frames_between()` counts frames where the specific body pair has a collision record.

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `replay/replay/testing/recording_query.py` | RecordingQuery class using `msd_reader` |
| `replay/replay/testing/__init__.py` | Package init, exports RecordingQuery |
| `replay/tests/test_recording_query.py` | Unit tests for RecordingQuery |

### Modified Files
None — this is a new Python module within the existing `replay/` package.

---

## Key Reusable Components

| Component | Location | Reused For |
|-----------|----------|------------|
| `msd_reader.Database` | pybind11 module (built from `msd/msd-reader/`) | Read-only DB connection |
| `msd_reader.Database.select_all_frames()` | pybind11 binding | Frame enumeration |
| `msd_reader.Database.select_all_states()` | pybind11 binding | Position/velocity queries |
| `msd_reader.Database.select_all_system_energies()` | pybind11 binding | Energy timeseries |
| `msd_reader.Database.select_all_collisions()` | pybind11 binding | Contact event queries |
| `replay.services.simulation_service` | `replay/replay/services/simulation_service.py` | Reference pattern for using `msd_reader` |

---

## Interface

```python
import msd_reader
from pathlib import Path
import math


class RecordingQuery:
    """Query wrapper over a recording database for test assertions."""

    def __init__(self, db_path: str | Path) -> None:
        self._db = msd_reader.Database(str(db_path))

    # Frame queries
    def frame_count(self) -> int: ...
    def total_simulation_time(self) -> float: ...

    # Per-body timeseries
    def position_history(self, body_id: int) -> list[tuple[float, float, float]]: ...
    def velocity_history(self, body_id: int) -> list[tuple[float, float, float]]: ...
    def speed_history(self, body_id: int) -> list[float]: ...

    # Per-body aggregates
    def position_at_frame(self, body_id: int, frame_id: int) -> tuple[float, float, float]: ...
    def min_z(self, body_id: int) -> float: ...
    def max_z(self, body_id: int) -> float: ...
    def max_speed(self, body_id: int) -> float: ...

    # System energy
    def system_energy_history(self) -> list[float]: ...
    def max_energy_drift(self) -> float: ...

    # Contact events
    def total_contact_frames(self) -> int: ...
    def contact_frames_between(self, body_a_id: int, body_b_id: int) -> int: ...
```

---

## Implementation Notes

### Database Access Pattern

Follow the pattern established by `replay/replay/services/simulation_service.py`:

```python
class RecordingQuery:
    def __init__(self, db_path: str | Path) -> None:
        self._db = msd_reader.Database(str(db_path))

    def frame_count(self) -> int:
        return len(self._db.select_all_frames())

    def total_simulation_time(self) -> float:
        frames = self._db.select_all_frames()
        return frames[-1].simulation_time if frames else 0.0
```

### Energy Drift Calculation

```python
def max_energy_drift(self) -> float:
    energies = self.system_energy_history()
    if not energies or abs(energies[0]) < 1e-12:
        return 0.0

    e0 = energies[0]
    return max(abs(e - e0) / abs(e0) for e in energies)
```

### Speed Calculation

```python
def speed_history(self, body_id: int) -> list[float]:
    velocities = self.velocity_history(body_id)
    return [math.sqrt(vx**2 + vy**2 + vz**2) for vx, vy, vz in velocities]
```

---

## Test Plan

### Unit Tests

```python
def test_frame_count_returns_correct_count(sample_recording):
    """Create a recording DB with known number of frames, verify frame_count()."""

def test_position_history_returns_correct_positions(sample_recording):
    """Create recording with known positions, verify position_history() output."""

def test_min_z_returns_lowest_position(sample_recording):
    """Create recording where body moves through known z range, verify min_z()."""

def test_max_energy_drift_computes_correct_relative_drift(sample_recording):
    """Create recording with known energy values, verify drift calculation."""

def test_total_contact_frames_counts_distinct_frames(sample_recording):
    """Create recording with collision records, verify contact frame count."""

def test_empty_recording_returns_zero(empty_recording):
    """Verify all queries handle empty recordings gracefully."""
```

These tests use pytest fixtures that create temporary recording databases with known data via the C++ test infrastructure or by running a short simulation.

---

## Acceptance Criteria

1. [ ] **AC1**: `RecordingQuery` opens recording databases via `msd_reader` (read-only)
2. [ ] **AC2**: Frame queries return correct counts and timestamps
3. [ ] **AC3**: Position/velocity timeseries return per-frame data ordered by frame ID
4. [ ] **AC4**: `min_z()`/`max_z()`/`max_speed()` compute correct aggregates
5. [ ] **AC5**: `max_energy_drift()` computes relative energy error correctly
6. [ ] **AC6**: Contact queries count distinct frames with collision records
7. [ ] **AC7**: All queries handle empty recordings gracefully (zero frames, no contacts)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
