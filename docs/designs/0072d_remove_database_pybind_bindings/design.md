# Design: Remove Database Pybind Bindings

## Summary

This design removes `database_bindings.cpp` from `msd-pybind` — the pybind11 wrapper that exposes C++ SQLite query operations (`DatabaseWrapper`) to Python as `msd_reader.Database`. Python consumers (the replay server, MCP server, and test layer) are migrated to use Python's native `sqlite3` module directly. To provide a clean, reusable access layer, a new `RecordingDB` class is introduced in the replay package and the existing `RecordingQuery` class is refactored to use it. The change eliminates a maintenance burden, removes build coupling to `cpp_sqlite` headers through pybind11, and makes Python database access more natural and flexible.

## Architecture Changes

### PlantUML Diagram

See: `./0072d_remove_database_pybind_bindings.puml`

### Removed Components

#### `DatabaseWrapper` (in `msd-pybind/src/database_bindings.cpp`)
- **Removed from**: C++ pybind11 module (`msd_reader`)
- **Impact**: `msd_reader.Database` class no longer exists in Python
- **Consumers that must migrate**: `SimulationService`, `RecordingQuery`, `test_msd_reader.py`

### New Components

#### `RecordingDB` (Python)
- **Purpose**: Lightweight Python wrapper around `sqlite3` that provides typed, reusable query methods for replay recording databases. Serves as the single database access layer for the entire replay package — replacing `msd_reader.Database` everywhere.
- **File location**: `replay/replay/db/recording_db.py`
- **Package init**: `replay/replay/db/__init__.py` (exports `RecordingDB`)
- **Key interfaces**:
  ```python
  class RecordingDB:
      """Read-only SQLite access layer for MSD recording databases.

      Replaces msd_reader.Database with native Python sqlite3 queries.
      Uses a single persistent connection in WAL mode for read efficiency.
      """

      def __init__(self, db_path: str | Path) -> None:
          """Open the recording database (read-only)."""

      def close(self) -> None:
          """Close the database connection."""

      # Frame queries
      def select_all_frames(self) -> list[dict]:
          """Returns list of {id, simulation_time} dicts."""

      # Collision queries
      def select_collisions_by_frame(self, frame_id: int) -> list[dict]:
          """Returns collision records with nested contacts for a frame."""

      # Solver queries
      def select_solver_diagnostic_by_frame(self, frame_id: int) -> list[dict]:
          """Returns solver diagnostic records for a frame."""

      # Friction constraint queries
      def select_friction_constraints_by_frame(self, frame_id: int) -> list[dict]:
          """Returns friction constraint records for a frame.
          Returns empty list if table does not exist (older databases).
          """

      # Energy queries
      def select_energy_by_body(self, body_id: int) -> list[dict]:
          """Returns EnergyRecord rows for a body (all frames)."""

      def select_all_system_energy(self) -> list[dict]:
          """Returns all SystemEnergyRecord rows."""

      # Static asset queries
      def select_all_static_assets(self) -> list[dict]:
          """Returns all AssetInertialStaticRecord rows."""

      # Body state queries (JOIN queries replacing select_all_inertial_states)
      def select_body_states_by_frame(self, frame_id: int) -> list[dict]:
          """Returns body states for a frame using SQL JOIN.

          Joins AssetDynamicStateRecord -> InertialStateRecord -> sub-records.
          Returns list of {body_id, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,
                           qw, qx, qy, qz, qdx, qdy, qdz, qdw} dicts.
          """

      def select_body_states_all_frames(self) -> list[dict]:
          """Returns body states across all frames (for timeseries queries).

          Returns list of {frame_id, body_id, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z}
          ordered by frame_id, body_id.
          """
  ```
- **Dependencies**: Python standard library `sqlite3` only
- **Thread safety**: Not thread-safe. Each `SimulationService` instance holds one `RecordingDB`. The replay server already uses per-request or per-session service instances — no change required.
- **Error handling**: `sqlite3.OperationalError` raised if database does not exist or table is missing. Callers that need to handle optional tables (e.g., `FrictionConstraintRecord` in older databases) must catch `sqlite3.OperationalError`.
- **Connection strategy**: Single persistent connection opened in the constructor, closed on `close()`. Opened with `check_same_thread=False` since the replay server is async and accesses the DB from a single thread per service instance.

### Modified Components

#### `SimulationService` (`replay/replay/services/simulation_service.py`)
- **Changes required**:
  - Remove `import msd_reader`
  - Add `from ..db import RecordingDB`
  - Replace `self.db = msd_reader.Database(str(db_path))` with `self.db = RecordingDB(db_path)`
  - Replace all `self.db.select_*()` calls with equivalent `RecordingDB` method calls
  - The internal `sqlite3.connect()` calls for JOIN queries are absorbed into `RecordingDB.select_body_states_by_frame()` and `select_body_states_all_frames()` — remove the inline `conn = sqlite3.connect()` blocks
  - The inline `sqlite3.connect()` for `AssetPhysicalStaticRecord` in `get_metadata()` is also moved to a `RecordingDB` method

- **Backward compatibility**: No change to `SimulationService`'s public API (method signatures unchanged)

#### `RecordingQuery` (`replay/replay/testing/recording_query.py`)
- **Changes required**:
  - Remove `import msd_reader`
  - Add `from ..db import RecordingDB`
  - Replace `self._db = msd_reader.Database(str(db_path))` with `self._db = RecordingDB(db_path)`
  - Replace `self._db.select_all_frames()` with `RecordingDB.select_all_frames()`
  - Replace `self._db.select_all_inertial_states()` — this C++ method performed a flat JOIN and returned objects with `body_id`, `frame_id`, `position.x`, etc. Replace with `RecordingDB.select_body_states_all_frames()` which returns equivalent dict rows. Update attribute access patterns (`.position.x` → `["pos_x"]`) throughout `RecordingQuery`.
  - Replace `self._db.select_all_system_energy()` with `RecordingDB.select_all_system_energy()`
  - Replace `self._db.select_all_collisions()` with a new `RecordingDB.select_all_collisions()` method

- **Backward compatibility**: `RecordingQuery`'s public method signatures are unchanged. The refactoring is internal.

#### `msd_bindings.cpp` (`msd/msd-pybind/src/msd_bindings.cpp`)
- **Changes required**:
  - Remove forward declaration `void bind_database(py::module_& m);`
  - Remove call `bind_database(m);`
  - Update module docstring to remove mention of `Database` class

#### `CMakeLists.txt` (`msd/msd-pybind/CMakeLists.txt`)
- **Changes required**:
  - Remove `src/database_bindings.cpp` from `pybind11_add_module(msd_reader ...)` source list

#### `msd-pybind/CLAUDE.md`
- **Changes required**:
  - Remove `database_bindings.cpp` from the file structure table
  - Remove `DatabaseWrapper` description from the database bindings section
  - Update the file list to reflect the removal

### New Module: `replay/replay/db/`

The `RecordingDB` class is placed in a new `db/` subpackage within the replay package. This provides a clear separation between the database access layer and the service layer.

```
replay/
└── replay/
    ├── db/                        # NEW: Database access layer
    │   ├── __init__.py            # Exports RecordingDB
    │   └── recording_db.py        # RecordingDB implementation
    ├── services/
    │   ├── simulation_service.py  # MODIFIED: uses RecordingDB
    │   └── geometry_service.py    # UNCHANGED (uses msd_reader.AssetRegistry)
    └── testing/
        └── recording_query.py     # MODIFIED: uses RecordingDB
```

### Integration Points

| Modified Component | Existing Component | Integration Type | Notes |
|-------------------|-------------------|------------------|-------|
| `SimulationService` | `RecordingDB` | Composition (owns one instance) | Replaces `msd_reader.Database` |
| `RecordingQuery` | `RecordingDB` | Composition (owns one instance) | Replaces `msd_reader.Database` |
| `msd_bindings.cpp` | `database_bindings.cpp` | Removal | `bind_database()` call removed |
| `CMakeLists.txt` | `database_bindings.cpp` | Removal | Source file removed from build |

### Design Decision: `select_all_inertial_states()` Replacement

The C++ `DatabaseWrapper::select_all_inertial_states()` returned a flat list of objects with `.body_id`, `.frame_id`, `.position.x/.y/.z`, `.velocity.x/.y/.z`. This flat representation required the C++ ORM to follow foreign key chains: `InertialStateRecord` → `CoordinateRecord` (position), `VelocityRecord` (velocity).

Looking at how `RecordingQuery` uses this method — it always:
1. Filters by `body_id`
2. Sorts by `frame_id`
3. Extracts position or velocity tuples

The `SimulationService` already uses the correct JOIN query via raw `sqlite3.connect()`. The `RecordingDB.select_body_states_all_frames()` method provides the same flat result directly from SQL, eliminating the in-Python filter loop and matching the normalized schema.

### DD-0072d-001: Place `RecordingDB` in `replay/replay/db/` Subpackage
- **Affects**: `replay.db.RecordingDB`, `replay.services.simulation_service.SimulationService`, `replay.testing.recording_query.RecordingQuery`
- **Rationale**: A dedicated `db/` subpackage creates a clear separation of concerns. The service layer (`services/`) handles business logic; the database access layer (`db/`) handles SQL. This matches common Python application architecture patterns and makes the layer explicit without adding external dependencies.
- **Alternatives Considered**:
  - Place in `replay/replay/services/recording_db.py`: Blurs the service/data-access boundary — services would peer into each other's modules.
  - Place in `replay/replay/testing/recording_db.py`: Wrong — `RecordingDB` is not test-only; `SimulationService` also needs it.
  - Place at `replay/replay/recording_db.py` (top level of package): Acceptable, but a subpackage is cleaner as the DB module grows.
- **Trade-offs**: Adds one new directory. All import paths must be updated.
- **Status**: active

### DD-0072d-002: `RecordingDB` Returns Plain Dicts, Not Pydantic Models
- **Affects**: `replay.db.RecordingDB`
- **Rationale**: `RecordingDB` is a low-level data access layer. Returning plain dicts keeps it framework-agnostic and avoids coupling the DB layer to Pydantic. The conversion to Pydantic/domain models happens in the service layer (`SimulationService`) exactly as it does today — the service translates raw rows into `FrameData`, `BodyState`, etc.
- **Alternatives Considered**:
  - Return Pydantic models from `RecordingDB`: Couples the DB layer to the REST API models; makes `RecordingDB` harder to use in test contexts.
  - Return `namedtuple`: More structured than dicts but adds a type definition per query. Dicts are sufficient for this use case.
- **Trade-offs**: Callers must know column names. This is mitigated by clear method docstrings documenting the returned dict structure.
- **Status**: active

### DD-0072d-003: Single Persistent SQLite Connection Per `RecordingDB` Instance
- **Affects**: `replay.db.RecordingDB`
- **Rationale**: Opening a new `sqlite3.connect()` per query (as done in the original `SimulationService` inline queries) is wasteful for read-heavy workloads. A single persistent read-only connection is more efficient. `sqlite3` WAL mode allows concurrent reads without locking.
- **Alternatives Considered**:
  - Per-query connections: Simple but adds overhead on every call; this was the original inline approach.
  - Connection pool: Overkill — the replay server has one `SimulationService` per loaded recording, so one connection per service is sufficient.
- **Trade-offs**: The connection must be explicitly closed. `RecordingDB` provides a `close()` method. `SimulationService` can call `close()` in a `__del__` or when a new recording is loaded.
- **Status**: active

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|-----------------|
| `msd/msd-pybind/test/test_msd_reader.py` | Database-related tests (lines 94–154) | `msd_reader.Database` no longer exists | Remove or update tests that use `msd_reader.Database` — these tests validated the C++ bindings, which are being deleted |
| `replay/tests/test_recording_query.py` | All tests using `query._db.select_*` | Internal `_db` attribute changes from `msd_reader.Database` to `RecordingDB` | Update tests that access `query._db` directly to use `RecordingDB` methods; public `RecordingQuery` API is unchanged |
| `replay/tests/test_assertions.py` | All assertion tests | `import msd_reader` at top (line 10) | Remove `import msd_reader` if unused; verify tests still pass |
| `replay/examples/query_recording.py` | Example script | Uses `msd_reader.Database` directly | Update to use `RecordingDB` directly, or remove as superseded |

### New Tests Required

#### Unit Tests for `RecordingDB`

| Component | Test File | Test Case | What It Validates |
|-----------|-----------|-----------|-------------------|
| `RecordingDB` | `replay/tests/test_recording_db.py` | `test_select_all_frames` | Returns correct frame count and simulation_time values |
| `RecordingDB` | `replay/tests/test_recording_db.py` | `test_select_body_states_by_frame` | Returns correct body states for a given frame_id |
| `RecordingDB` | `replay/tests/test_recording_db.py` | `test_select_body_states_all_frames` | Returns states across all frames in correct order |
| `RecordingDB` | `replay/tests/test_recording_db.py` | `test_select_collisions_by_frame` | Returns collision records with nested contacts |
| `RecordingDB` | `replay/tests/test_recording_db.py` | `test_select_friction_constraints_by_frame` | Returns friction constraints; returns empty list on older DBs |
| `RecordingDB` | `replay/tests/test_recording_db.py` | `test_select_energy_by_body` | Returns energy timeseries for a specific body |
| `RecordingDB` | `replay/tests/test_recording_db.py` | `test_select_all_system_energy` | Returns system energy records |
| `RecordingDB` | `replay/tests/test_recording_db.py` | `test_missing_table_graceful` | `select_friction_constraints_by_frame` returns [] on older DB |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| Existing `test_api.py` (all endpoints) | `SimulationService` + `RecordingDB` + FastAPI | All REST endpoints return correct data post-migration |
| Existing `test_recording_query.py` | `RecordingQuery` + `RecordingDB` | `RecordingQuery` public API still works after internal refactor |

## Open Questions

### Design Decisions (Human Input Needed)

1. **`query_recording.py` example file disposition**
   - Option A: Update `replay/examples/query_recording.py` to use `RecordingDB` — keeps the example current
   - Option B: Remove the example file — the MCP server is now the canonical query interface
   - Recommendation: Option A — update the example; it serves as documentation for external users

2. **`test_msd_reader.py` database tests disposition**
   - Option A: Remove only the `msd_reader.Database` test cases; keep the rest of the file (record type tests)
   - Option B: Keep the file but skip/stub the database tests with a comment explaining removal
   - Recommendation: Option A — cleanly remove the deleted functionality's tests

### Requirements Clarification

1. **`select_all_inertial_states()` in `RecordingQuery`**: The current C++ method returns a flat object with `body_id` and `frame_id`. However, `InertialStateRecord` in the database schema does NOT have a `body_id` column directly — it lives in `AssetDynamicStateRecord`. The `DatabaseWrapper` implementation joins these implicitly through the C++ ORM. The new `RecordingDB.select_body_states_all_frames()` method must replicate this JOIN. The existing SQL JOIN in `SimulationService._get_states_by_frame()` confirms the correct join path.
