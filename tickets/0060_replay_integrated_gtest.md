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
| 0060b | RecordingQuery database query API | High | 0060a | PENDING |
| 0060c | ReplayMatchers — GTest custom matchers | Medium | 0060b | PENDING |
| 0060d | Example replay-enabled tests | Medium | 0060a, 0060b, 0060c | PENDING |

---

## Summary

Build a GTest infrastructure that unifies simulation unit tests with the browser-based replay viewer. Tests use real asset databases (replacing inline `createCubePoints()` helpers), record simulation state to SQLite via DataRecorder, and produce self-contained `.db` files viewable in the existing FastAPI replay viewer. A query API over the recording database enables recording-based assertions (energy conservation, floor penetration, settling behavior) alongside traditional in-memory assertions.

This supersedes [0056g](0056g_replay_enabled_tests.md) with a more complete design that includes post-simulation query infrastructure and GTest custom matchers.

---

## Architecture

```
ReplayEnabledTest (GTest fixture)
    |
    |-- SetUp: creates single .db with geometry (MeshRecord/ObjectRecord)
    |           then creates Engine (reads geometry via AssetRegistry)
    |           then enables recording (DataRecorder adds state tables to same .db)
    |-- TearDown: destroys Engine (RAII flushes DataRecorder)
    |
    +-- RecordingQuery (post-simulation analysis, wraps cpp_sqlite)
    |     |-- Timeseries: positionHistory(), velocityHistory(), systemEnergyHistory()
    |     |-- Aggregates: minZ(), maxSpeed(), maxEnergyDrift()
    |
    +-- ReplayMatchers (GTest custom matchers using RecordingQuery)
          |-- EnergyConservedWithin(tolerance)
          |-- NeverPenetratesBelow(bodyId, zMin)
          |-- BodyComesToRest(bodyId, speedThreshold)

Output: replay/recordings/{TestSuite}_{TestName}.db
        (self-contained: geometry tables + simulation state tables)
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
4. **Lazy query creation**: `RecordingQuery` created on first `query()` call after simulation completes (recording must be flushed first)
5. **Matchers take path string**: GTest matchers accept `std::string` path, create `RecordingQuery` internally — clean `EXPECT_THAT` syntax

---

## Acceptance Criteria

1. [ ] **AC1**: `ReplayEnabledTest` fixture creates self-contained `.db` with geometry + recording
2. [ ] **AC2**: `RecordingQuery` provides timeseries and aggregate queries over recording databases
3. [ ] **AC3**: Custom GTest matchers assert physics invariants from recorded data
4. [ ] **AC4**: At least 2 example tests produce recordings viewable in the FastAPI replay viewer
5. [ ] **AC5**: Recordings written to `replay/recordings/` are immediately served by `uvicorn replay.app:app`
6. [ ] **AC6**: Existing tests unaffected (`--gtest_filter="-Replay*"` all pass)
7. [ ] **AC7**: `MSD_KEEP_RECORDINGS` env var controls recording cleanup (default: keep)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
