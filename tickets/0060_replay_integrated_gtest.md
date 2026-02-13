# Ticket 0060: Replay-Integrated GTest Infrastructure

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Feature / Testing / Infrastructure
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-13
**Generate Tutorial**: No
**Related Tickets**: [0056_browser_simulation_replay](0056_browser_simulation_replay.md), [0056g_replay_enabled_tests](0056g_replay_enabled_tests.md), [0038_simulation_data_recorder](0038_simulation_data_recorder.md)

### Subtasks

| Subtask | Description | Priority | Dependencies | Status |
|---------|-------------|----------|--------------|--------|
| 0060a | ReplayEnabledTest fixture | High | None | PENDING |
| 0060b | RecordingQuery Python query API | High | 0060a | PENDING |
| 0060c | Python pytest recording assertions | Medium | 0060b | PENDING |
| 0060d | Example replay-enabled tests | Medium | 0060a, 0060b, 0060c | PENDING |

---

## Summary

Build a two-language test infrastructure that unifies simulation unit tests with the browser-based replay viewer. C++ GTest fixtures use real asset databases (replacing inline `createCubePoints()` helpers), record simulation state to SQLite via DataRecorder, and produce self-contained `.db` files viewable in the existing FastAPI replay viewer. A Python query layer over the recording database — using the existing `msd_reader` pybind11 module — enables recording-based assertions (energy conservation, floor penetration, settling behavior) via pytest, while C++ GTest handles traditional in-memory assertions.

This supersedes [0056g](0056g_replay_enabled_tests.md) with a more complete design that includes post-simulation query infrastructure and Python-based physics invariant checks.

---

## Architecture

```
C++ Layer (GTest):
  ReplayEnabledTest (GTest fixture)
      |
      |-- SetUp: creates single .db with geometry (MeshRecord/ObjectRecord)
      |           then creates Engine (reads geometry via AssetRegistry)
      |           then enables recording (DataRecorder adds state tables to same .db)
      |-- TearDown: destroys Engine (RAII flushes DataRecorder)
      |
      |-- Traditional in-memory assertions (EXPECT_GT, EXPECT_NE, etc.)
      |
      Output: replay/recordings/{TestSuite}_{TestName}.db
              (self-contained: geometry tables + simulation state tables)

Python Layer (pytest):
  recording_query (Python module using msd_reader pybind11)
      |
      |-- RecordingQuery class (post-simulation analysis)
      |     |-- Timeseries: position_history(), velocity_history(), system_energy_history()
      |     |-- Aggregates: min_z(), max_speed(), max_energy_drift()
      |
      |-- pytest assertion helpers
            |-- assert_energy_conserved(db_path, tolerance)
            |-- assert_never_penetrates_below(db_path, body_id, z_min)
            |-- assert_body_comes_to_rest(db_path, body_id, speed_threshold)

Both layers read: replay/recordings/{TestSuite}_{TestName}.db
                   (immediately viewable via existing FastAPI viewer)
```

### Single-Database Design

The replay viewer's GeometryService reads mesh data from the same `.db` as simulation state. For self-contained recordings:

1. Fixture creates DB at `replay/recordings/{name}.db` with `cpp_sqlite::Database(path, true)`
2. Writes `MeshRecord` + `ObjectRecord` (geometry tables) using existing DAO pattern
3. Closes initial connection
4. Creates `Engine{dbPath}` — opens same DB read-only via `AssetRegistry(dbPath, false)`
5. Calls `world.enableRecording(dbPath)` — `DataRecorder` opens same DB read-write, adds state tables via `CREATE TABLE IF NOT EXISTS`
6. Result: single `.db` with both geometry and state — viewer works without modification

This is safe because SQLite supports concurrent read-only + read-write connections, which is already proven by `EngineIntegrationTest`.

### Migration Strategy

- **Existing tests unchanged** — `createCubePoints()` pattern continues to work
- **Gradual migration** — Existing tests can be converted to the replay pattern as they are touched
- **New tests** — Should prefer the replay-enabled pattern

---

## Key Design Decisions

1. **Single database**: Geometry + recording state in one `.db` — viewer needs no modification
2. **Engine-based**: Fixture uses `Engine` (wraps AssetRegistry + WorldModel) rather than raw WorldModel — proven pattern from `EngineIntegrationTest`
3. **Compile-time recording path**: `MSD_RECORDINGS_DIR` CMake define points to `${CMAKE_SOURCE_DIR}/replay/recordings` — works regardless of CWD
4. **Two-language split**: C++ GTest produces `.db` files with traditional in-memory assertions; Python pytest reads `.db` files for recording-based physics invariant checks. This leverages the existing `msd_reader` pybind11 module and avoids duplicating database reading logic in C++.
5. **Python query layer**: `RecordingQuery` is a Python class using `msd_reader` — same module already powering the FastAPI replay viewer. This keeps all database reading in one language and benefits from Python's rapid iteration for analysis code.

---

## Acceptance Criteria

1. [ ] **AC1**: `ReplayEnabledTest` fixture creates self-contained `.db` with geometry + recording
2. [ ] **AC2**: Python `RecordingQuery` provides timeseries and aggregate queries over recording databases via `msd_reader`
3. [ ] **AC3**: Python pytest assertion helpers validate physics invariants from recorded data
4. [ ] **AC4**: At least 2 example C++ tests produce recordings viewable in the FastAPI replay viewer
5. [ ] **AC5**: At least 2 example Python tests validate recording data with physics invariant assertions
6. [ ] **AC6**: Recordings written to `replay/recordings/` are immediately served by `uvicorn replay.app:app`
7. [ ] **AC7**: Existing C++ tests unaffected (`--gtest_filter="-Replay*"` all pass)
8. [ ] **AC8**: `MSD_KEEP_RECORDINGS` env var controls recording cleanup (default: keep)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
