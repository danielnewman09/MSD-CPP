# Iteration Log — 0060d_example_replay_tests

> **Purpose**: Track every build-test cycle during implementation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/investigations/0060d_example_replay_tests/iteration-log.md`
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0060d_example_replay_tests
**Branch**: 0060d-example-replay-tests
**Baseline**: 713/717 tests passing at start (4 pre-existing failures unrelated to replay infrastructure)

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 1 — 2026-02-13 18:50
**Commit**: (pending)
**Hypothesis**: Create C++ example tests for ReplayEnabledTest fixture demonstrating drop and collision scenarios
**Changes**:
- `msd/msd-sim/test/Replay/DropTest.cpp`: Created cube drop test with gravity settling
- `msd/msd-sim/test/Replay/CollisionTest.cpp`: Created two-cube collision test
- `msd/msd-sim/test/Replay/CMakeLists.txt`: Added new test files to build
**Build Result**: PASS
**Test Result**: 715/719 — Added 2 new tests (DropTest::CubeDropsAndSettles, CollisionTest::TwoCubesCollide), both passing
**Impact vs Previous**: +2 passes, baseline 713/717 → 715/719 (+2 new tests added)
**Assessment**: C++ side complete. Tests produce .db recordings as expected. Collision test simplified to avoid instanceId lookup bug (not in scope for this ticket). Ready for Python side.

### Iteration 2 — 2026-02-13 19:00
**Commit**: (pending)
**Hypothesis**: Create Python recording validation tests demonstrating RecordingQuery and assertion APIs
**Changes**:
- `replay/tests/test_drop_recording.py`: Created Python tests for drop scenario validation
- `replay/tests/test_collision_recording.py`: Created Python tests for collision scenario validation
- Added skipif decorators for tests requiring msd_reader pybind11 module
- test_recording_contains_geometry_and_state uses raw SQLite and passes without pybind11
**Build Result**: N/A (Python only)
**Test Result**: 1/7 passed, 6/7 skipped (msd_reader not available) — Expected behavior
**Impact vs Previous**: Python test infrastructure complete, demonstrates API patterns
**Assessment**: Complete. Tests demonstrate two-language pattern: C++ produces .db, Python validates. When msd_reader is built, skipped tests will run automatically.
