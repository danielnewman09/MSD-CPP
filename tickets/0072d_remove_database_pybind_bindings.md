# Feature Ticket: Remove Database Pybind Bindings

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [ ] Ready for Integration Design
- [ ] Integration Design Complete — Awaiting Review
- [ ] Integration Design Approved
- [ ] Ready for Python Design
- [ ] Python Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Test Writing
- [ ] Test Writing Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-02-20
- **Author**: Daniel Newman
- **Priority**: Medium
- **Estimated Complexity**: Medium
- **Target Component(s)**: msd-pybind, replay
- **Languages**: C++, Python
- **Generate Tutorial**: No
- **Requires Math Design**: No
- **GitHub Issue**: #79

---

## Summary
Remove the `database_bindings.cpp` pybind11 module that exposes C++ database query operations (`DatabaseWrapper`) to Python. Python can more naturally construct its own SQL queries using the native `sqlite3` module, making the C++ wrapper unnecessary overhead. The Python replay framework (services, MCP server, testing) must be adjusted to use direct SQLite queries instead of the `msd_reader.Database` API.

## Motivation
The current `database_bindings.cpp` wraps `cpp_sqlite::Database` and exposes ~30 methods (`select_all_*`, `select_by_id_*`, `select_by_frame_*`, `select_by_body_*`) through pybind11. This creates several problems:

1. **Maintenance burden**: Every new transfer record requires adding new binding methods in C++ — duplicating work already handled by the auto-generated `record_bindings.cpp` and Pydantic models.
2. **Unnecessary complexity**: Python has excellent native SQLite support via `sqlite3`. Queries can be written more naturally and flexibly in Python than through rigid C++ template wrappers.
3. **Build coupling**: The database bindings pull in `cpp_sqlite` headers and link against SQLite through pybind11, adding build complexity for functionality Python already provides natively.
4. **Replay framework impact**: The replay server, MCP server, and test assertions currently use `msd_reader.Database` — these must be migrated to native Python SQLite queries, which will be simpler and more maintainable.

## Requirements

### Functional Requirements
1. The system shall remove `database_bindings.cpp` from `msd-pybind/src/` and its CMakeLists.txt entry
2. The system shall remove the `bind_database()` call from `msd_bindings.cpp`
3. The system shall update `replay/replay/services/simulation_service.py` to use native `sqlite3` queries instead of `msd_reader.Database`
4. The system shall update `replay/replay/testing/recording_query.py` to use native `sqlite3` queries instead of `msd_reader.Database`
5. The system shall update `replay/mcp_server/server.py` to use native `sqlite3` queries instead of `msd_reader.Database`
6. The system shall update `replay/replay/services/geometry_service.py` if it uses `msd_reader.Database`
7. The system shall update any test files that import or use `msd_reader.Database` (e.g., `test_assertions.py`, `test_recording_query.py`, `test_api.py`)
8. The system shall ensure all existing replay server endpoints continue to function correctly after migration
9. The system shall update `msd/msd-pybind/CLAUDE.md` to remove references to `database_bindings.cpp` and the `DatabaseWrapper`

### Non-Functional Requirements
- **Performance**: Native Python SQLite queries should have equivalent or better performance than the pybind11 wrapper path
- **Memory**: No change expected — Python sqlite3 module handles memory natively
- **Thread Safety**: No change — replay server already uses per-request database connections
- **Backward Compatibility**: The `msd_reader` module will no longer expose `Database` class — all Python consumers must be migrated

## Constraints
- The `record_bindings.cpp` (auto-generated), `engine_bindings.cpp`, `geometry_bindings.cpp`, and `asset_registry_bindings.cpp` must remain untouched
- The Pydantic `generated_models.py` already provides Python-native record types that can be used for deserializing SQLite query results
- The replay MCP server must continue to pass all existing tests

## Acceptance Criteria
- [ ] `database_bindings.cpp` is removed from the codebase and CMakeLists.txt
- [ ] `msd_reader.Database` is no longer exposed in the Python module
- [ ] All replay server endpoints (`/simulations`, `/frames`, etc.) return correct data using native SQLite
- [ ] All replay test suites pass (`test_api.py`, `test_recording_query.py`, `test_assertions.py`)
- [ ] MCP replay server continues to function (load recordings, query frames, energy, contacts)
- [ ] `msd-pybind/CLAUDE.md` updated to reflect removal
- [ ] No remaining imports of `msd_reader.Database` anywhere in the Python codebase

---

## Design Decisions (Human Input)

{Use this section to provide guidance to the Design agent, or to record decisions made during review}

### Preferred Approaches
- Use Python's built-in `sqlite3` module for all database access
- Consider creating a lightweight Python helper class (e.g., `RecordingDB`) that wraps `sqlite3` connections with typed query methods, using the existing Pydantic models for deserialization
- The existing `RecordingQuery` class in `replay/replay/testing/recording_query.py` is a good candidate for refactoring into the primary database access layer

### Things to Avoid
- Do not replace with SQLAlchemy or other heavy ORMs — keep it simple with raw `sqlite3`
- Do not modify `record_bindings.cpp` or the auto-generated Pydantic models
- Do not change the Engine bindings (`engine_bindings.cpp`) or asset registry bindings

### Open Questions
- Should the Python SQLite helper live in `replay/replay/services/` or a new shared utility location?
- Should we create a shared base class for recording database access, or keep service-specific query methods?

---

## References

### Related Code
- `msd/msd-pybind/src/database_bindings.cpp` — The file being removed
- `msd/msd-pybind/src/msd_bindings.cpp` — Module definition that calls `bind_database()`
- `msd/msd-pybind/CMakeLists.txt` — Build configuration to update
- `replay/replay/services/simulation_service.py` — Primary consumer of `msd_reader.Database`
- `replay/replay/testing/recording_query.py` — Test assertion query layer using `msd_reader.Database`
- `replay/mcp_server/server.py` — MCP server using recording queries
- `replay/replay/services/geometry_service.py` — Geometry service using `msd_reader`
- `replay/replay/generated_models.py` — Pydantic models available for Python-native deserialization

### Related Documentation
- `msd/msd-pybind/CLAUDE.md` — pybind module documentation (needs update)

### Related Tickets
- [0072](0072_live_browser_simulation.md) — Parent: Live Browser Simulation
- [0072a](0072a_engine_pybind_bindings.md) — Engine pybind bindings (unaffected)
- [0072b](0072b_websocket_simulation_endpoint.md) — WebSocket endpoint (may be affected)
- [0072c](0072c_live_simulation_frontend.md) — Live simulation frontend (unaffected)
- [0056c](0056c_python_bindings.md) — Original Python bindings ticket that created `database_bindings.cpp`

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-02-20 00:00
- **Completed**: 2026-02-20 00:00
- **Branch**: 0072d-remove-database-pybind-bindings
- **PR**: N/A (pending creation after push)
- **Artifacts**:
  - `docs/designs/0072d_remove_database_pybind_bindings/design.md`
  - `docs/designs/0072d_remove_database_pybind_bindings/0072d_remove_database_pybind_bindings.puml`
- **Design Decisions**:
  - DD-0072d-001: Place `RecordingDB` in `replay/replay/db/` subpackage (active)
  - DD-0072d-002: `RecordingDB` returns plain dicts, not Pydantic models (active)
  - DD-0072d-003: Single persistent SQLite connection per `RecordingDB` instance (active)
- **Notes**: Design introduces `RecordingDB` as a pure Python sqlite3 wrapper replacing `msd_reader.Database`. Two open questions for human: disposition of `query_recording.py` example and database tests in `test_msd_reader.py`.

### Design Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Integration Design Phase
- **Started**:
- **Completed**:
- **Artifacts**:
  - `docs/designs/0072d_remove_database_pybind_bindings/integration-design.md`
  - `docs/designs/0072d_remove_database_pybind_bindings/0072d_remove_database_pybind_bindings-sequence.puml`
  - Updates to `docs/api-contracts/contracts.yaml`
- **Notes**:

### Integration Design Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Python Design Phase
- **Started**:
- **Completed**:
- **Artifacts**:
  - `docs/designs/0072d_remove_database_pybind_bindings/python/design.md`
- **Notes**:

### Prototype Phase
- **Started**:
- **Completed**:
- **Prototypes**:
  - P1: {name} — {result}
- **Artifacts**:
  - `docs/designs/0072d_remove_database_pybind_bindings/prototype-results.md`
- **Notes**:

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
- **Files Modified**:
- **Artifacts**:
  - `docs/designs/0072d_remove_database_pybind_bindings/implementation-notes.md`
- **Notes**:

### Test Writing Phase
- **Started**:
- **Completed**:
- **Test Files Created**:
- **Test Coverage Summary**:
- **Test Failures Documented for Implementer**:
- **Notes**:

### Implementation Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Documentation Update Phase
- **Started**:
- **Completed**:
- **CLAUDE.md Updates**:
- **Diagrams Indexed**:
- **Notes**:

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Integration Design
{Your comments on the integration design, API contracts, wire protocols}

### Feedback on Python Design
{Your comments on the Python architecture}

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}

### Feedback on Tests
{Your comments on test coverage, test quality, or missing test scenarios}
