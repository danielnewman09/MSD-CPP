# Ticket 0060b: RecordingQuery Database Query API

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

Create a C++ query wrapper over recording databases that enables tests to interrogate simulation results after execution. `RecordingQuery` opens a recording `.db` (read-only) and provides typed queries for position/velocity timeseries, energy tracking, and contact event analysis using existing `msd_transfer` record types and `cpp_sqlite` DAOs.

This is the core "read from the data recorder for easier interrogation of results" capability.

---

## Requirements

### R1: Frame Queries

```cpp
size_t frameCount() const;
double totalSimulationTime() const;
```

Read `SimulationFrameRecord` table. `totalSimulationTime()` returns the `simulation_time` of the last frame.

### R2: Per-Body Position/Velocity Timeseries

```cpp
std::vector<Coordinate> positionHistory(uint32_t bodyId) const;
std::vector<Coordinate> velocityHistory(uint32_t bodyId) const;
std::vector<double> speedHistory(uint32_t bodyId) const;
```

Read `InertialStateRecord` table, filter by `body_id`, ordered by `frame.id`. Return one entry per recorded frame. `speedHistory()` returns `velocity.norm()` per frame.

### R3: Per-Body Scalar Aggregates

```cpp
Coordinate positionAtFrame(uint32_t bodyId, uint32_t frameId) const;
double minZ(uint32_t bodyId) const;
double maxZ(uint32_t bodyId) const;
double maxSpeed(uint32_t bodyId) const;
```

Computed from the timeseries data. `minZ()` returns the minimum z-component of position across all frames — useful for "object never fell through the floor" assertions.

### R4: System Energy Queries

```cpp
std::vector<double> systemEnergyHistory() const;
double maxEnergyDrift() const;
```

Read `SystemEnergyRecord` table. `systemEnergyHistory()` returns `total_system_e` per frame. `maxEnergyDrift()` computes `max |E(t) - E(0)| / |E(0)|` across all frames — the relative energy error.

### R5: Contact Event Queries

```cpp
size_t totalContactFrames() const;
size_t contactFramesBetween(uint32_t bodyAId, uint32_t bodyBId) const;
```

Read `CollisionResultRecord` table. `totalContactFrames()` counts distinct frames with at least one collision record. `contactFramesBetween()` counts frames where the specific body pair has a collision record.

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `msd/msd-sim/test/Replay/RecordingQuery.hpp` | Query API header |
| `msd/msd-sim/test/Replay/RecordingQuery.cpp` | Implementation |

### Modified Files
| File | Change |
|------|--------|
| `msd/msd-sim/CMakeLists.txt` | Add `test/Replay/RecordingQuery.cpp` to test sources |

---

## Key Reusable Components

| Component | Location | Reused For |
|-----------|----------|------------|
| `cpp_sqlite::Database` | External dependency | Read-only DB connection |
| `msd_transfer::SimulationFrameRecord` | `msd/msd-transfer/src/SimulationFrameRecord.hpp` | Frame enumeration |
| `msd_transfer::InertialStateRecord` | `msd/msd-transfer/src/InertialStateRecord.hpp` | Position/velocity queries |
| `msd_transfer::SystemEnergyRecord` | `msd/msd-transfer/src/SystemEnergyRecord.hpp` | Energy timeseries |
| `msd_transfer::EnergyRecord` | `msd/msd-transfer/src/EnergyRecord.hpp` | Per-body energy |
| `msd_transfer::CollisionResultRecord` | `msd/msd-transfer/src/CollisionResultRecord.hpp` | Contact event queries |

---

## Interface

```cpp
class RecordingQuery {
public:
  explicit RecordingQuery(const std::string& dbPath);

  // Frame queries
  size_t frameCount() const;
  double totalSimulationTime() const;

  // Per-body timeseries
  std::vector<Coordinate> positionHistory(uint32_t bodyId) const;
  std::vector<Coordinate> velocityHistory(uint32_t bodyId) const;
  std::vector<double> speedHistory(uint32_t bodyId) const;

  // Per-body aggregates
  Coordinate positionAtFrame(uint32_t bodyId, uint32_t frameId) const;
  double minZ(uint32_t bodyId) const;
  double maxZ(uint32_t bodyId) const;
  double maxSpeed(uint32_t bodyId) const;

  // System energy
  std::vector<double> systemEnergyHistory() const;
  double maxEnergyDrift() const;

  // Contact events
  size_t totalContactFrames() const;
  size_t contactFramesBetween(uint32_t bodyAId, uint32_t bodyBId) const;

private:
  std::unique_ptr<cpp_sqlite::Database> db_;
};
```

---

## Implementation Notes

### Database Access Pattern

Open the recording database read-only:
```cpp
RecordingQuery::RecordingQuery(const std::string& dbPath)
  : db_{std::make_unique<cpp_sqlite::Database>(dbPath, false)}  // read-only
{}
```

Use `selectAll()` on the appropriate DAO, then filter/aggregate in C++. For large recordings, consider SQL WHERE clauses if `cpp_sqlite` supports parameterized queries — but for test databases (typically 100-1000 frames), in-memory filtering is fast enough.

### Energy Drift Calculation

```cpp
double RecordingQuery::maxEnergyDrift() const {
  auto energies = systemEnergyHistory();
  if (energies.empty() || std::abs(energies[0]) < 1e-12) return 0.0;

  double e0 = energies[0];
  double maxDrift = 0.0;
  for (double e : energies) {
    maxDrift = std::max(maxDrift, std::abs(e - e0) / std::abs(e0));
  }
  return maxDrift;
}
```

---

## Test Plan

### Unit Tests

```cpp
TEST(RecordingQuery, FrameCount_ReturnsCorrectCount)
// Create a recording DB with known number of frames, verify frameCount()

TEST(RecordingQuery, PositionHistory_ReturnsCorrectPositions)
// Create recording with known positions, verify positionHistory() output

TEST(RecordingQuery, MinZ_ReturnsLowestPosition)
// Create recording where body moves through known z range, verify minZ()

TEST(RecordingQuery, MaxEnergyDrift_ComputesCorrectRelativeDrift)
// Create recording with known energy values, verify drift calculation

TEST(RecordingQuery, TotalContactFrames_CountsDistinctFrames)
// Create recording with collision records, verify contact frame count
```

These tests can use the `DataRecorderTest` pattern (create temp DB, write known records, verify queries).

---

## Acceptance Criteria

1. [ ] **AC1**: `RecordingQuery` opens recording databases read-only
2. [ ] **AC2**: Frame queries return correct counts and timestamps
3. [ ] **AC3**: Position/velocity timeseries return per-frame data ordered by frame ID
4. [ ] **AC4**: `minZ()`/`maxZ()`/`maxSpeed()` compute correct aggregates
5. [ ] **AC5**: `maxEnergyDrift()` computes relative energy error correctly
6. [ ] **AC6**: Contact queries count distinct frames with collision records
7. [ ] **AC7**: All queries handle empty recordings gracefully (zero frames, no contacts)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
