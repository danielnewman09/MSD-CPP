# Ticket 0038: Simulation Data Recorder

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Documentation Complete
**Assignee**: Unassigned
**Created**: 2026-02-06
**Priority**: Medium
**Complexity**: Medium-Large
**Target Component**: msd-sim (new DataRecorder subsystem), msd-transfer (records)
**Generate Tutorial**: No

---

## Summary

Design and implement a write-only data recording mechanism that captures simulation state to a SQLite database using `cpp_sqlite`. The recorder should operate on a dedicated background thread to minimize impact on simulation performance, leveraging the existing `msd-transfer` record types (e.g., `InertialStateRecord`) via `toRecord()` methods on domain objects.

---

## Motivation

Currently, the simulation engine (`msd-sim`) has no persistence mechanism for recording simulation state over time. This limits the ability to:

1. **Post-simulation analysis**: Replay or analyze simulation results after execution
2. **Debugging**: Capture state history for diagnosing physics anomalies or bugs
3. **Validation**: Compare simulation outputs against analytical solutions or reference data
4. **Data export**: Provide simulation data to external tools for visualization or further processing

The `msd-transfer` library already provides a robust foundation with:
- Database transfer objects (DTOs) for core types (`InertialStateRecord`, `CoordinateRecord`, etc.)
- `cpp_sqlite` ORM integration with automatic schema generation via `BOOST_DESCRIBE_STRUCT`
- Existing `toRecord()` methods on domain objects like `InertialState`

Building a dedicated recorder that operates on its own thread will:
- Decouple I/O latency from simulation timing
- Enable recording at high simulation rates without frame drops
- Provide a clean separation of concerns (simulation vs. persistence)

**User impact**: Enables post-hoc analysis of simulation runs, facilitates debugging of physics issues, and establishes infrastructure for future playback/replay features.

---

## Requirements

### Functional Requirements

1. **FR-1**: The system shall provide a `DataRecorder` class that writes simulation records to a SQLite database
2. **FR-2**: The recorder shall be write-only (no read/query functionality in this ticket)
3. **FR-3**: The recorder shall accept any type that has a `toRecord()` method returning a `cpp_sqlite::BaseTransferObject`-derived type
4. **FR-4**: The recorder shall operate on a dedicated background thread, decoupled from the main simulation thread
5. **FR-5**: The system shall provide a thread-safe queue for passing records from the simulation thread to the recorder thread
6. **FR-6**: The recorder shall support timestamping records (simulation time, not wall-clock time)
7. **FR-7**: The recorder shall batch writes for efficiency (configurable batch size)
8. **FR-8**: The recorder shall provide graceful shutdown, flushing all pending records before closing
9. **FR-9**: The system shall support recording multiple object types in a single database (e.g., `InertialStateRecord`, `CoordinateRecord`)

### Non-Functional Requirements

1. **NFR-1**: Record submission from simulation thread must be lock-free or minimally blocking (< 1μs typical)
2. **NFR-2**: The recorder thread shall not block the simulation thread waiting for I/O
3. **NFR-3**: Memory usage for the record queue shall be bounded (configurable maximum queue depth)
4. **NFR-4**: The recorder shall handle queue overflow gracefully (drop oldest records with warning, not crash)
5. **NFR-5**: Database writes shall be transactional (batch commits for performance)

---

## Design Questions

The design phase should address these questions:

### Q1: Thread Communication Mechanism

**Options:**
- **(a)** Lock-free SPSC (single-producer, single-consumer) queue using `std::atomic`
- **(b)** `std::mutex`-protected `std::queue` with condition variable
- **(c)** Third-party lock-free queue (e.g., moodycamel::ConcurrentQueue)
- **(d)** Use `std::jthread` with `std::stop_token` and a simple protected queue
- **(e)** Leverage existing `cpp_sqlite::DataAccessObject` double-buffer mechanism

**Considerations**: The simulation is single-threaded, so SPSC is sufficient. Lock-free is ideal for NFR-1 but adds complexity. The queue only needs to handle record submission and shutdown signaling.

**cpp_sqlite Review Finding**: The `DataAccessObject<T>` class already implements a **double-buffered write system**:
- `writeBuffer_` / `flushBuffer_` with mutex protection
- Thread-safe `addToBuffer(const T&)` method for adding records from any thread
- `insert()` (no args) method that atomically swaps buffers and flushes to DB
- `DAOBase` interface with virtual `insert()` for type-erased batch flushing

**Recommendation**: **(e)** — Use existing `cpp_sqlite` double-buffer mechanism. The recorder thread calls `dao.insert()` periodically to flush buffers. Simulation thread calls `dao.addToBuffer(record)`. This is already thread-safe and proven.

### Q2: Record Type Abstraction

**Options:**
- **(a)** Type-erased queue (store `std::any` or custom type-erased wrapper)
- **(b)** Separate queues per record type (requires knowing types at compile time)
- **(c)** Variant-based queue (`std::variant<InertialStateRecord, CoordinateRecord, ...>`)
- **(d)** Template the entire recorder on a single record type (one recorder per type)
- **(e)** Use `DAOBase*` polymorphism — each DAO has its own typed buffer, recorder iterates over registered DAOs

**Considerations**: Multiple record types need to be interleaved (e.g., record inertial state and contact events in same simulation step). Type erasure (a) is most flexible but has overhead. Variant (c) requires closed set of types.

**cpp_sqlite Review Finding**: The `DAOBase` abstract class provides:
- Virtual `insert()` — flush buffered data to database
- Virtual `clearBuffer()` — clear internal buffer
- Each `DataAccessObject<T>` maintains its own typed `writeBuffer_`

**Recommendation**: **(e)** — The recorder holds `std::vector<DAOBase*>` (or references) to all registered DAOs. On flush, it iterates and calls `dao->insert()` for each. No type erasure needed — each DAO handles its own type internally. This aligns with Human Recommendation.

**Human Recommendation**: cpp_sqlite has `Transaction` mechanisms and buffers. We should just be able to use some of the mechanisms from that library and not worry too much about record type abstraction. once we create the DAOs for our objects, we should be able to use the SQL transaction mechanism and/or the buffer mechanism

**Response**: Confirmed. The existing `DataAccessObject<T>::addToBuffer()` and `insert()` methods provide exactly this. Transaction support is NOT currently in `cpp_sqlite` but SQLite WAL mode + batch inserts should suffice. If explicit transactions are needed, we can add `beginTransaction()`/`commit()` to `cpp_sqlite::Database` as an enhancement.

### Q3: Timestamp Association

**Options:**
- **(a)** Add `simulation_time` field to each record type (requires modifying all `*Record` structs)
- **(b)** Create a wrapper record with timestamp + payload reference
- **(c)** Store timestamp in a separate table with foreign key to record
- **(d)** Use database row insertion order as implicit time ordering (no explicit timestamp)
- **(e)** Create a `SimulationFrame` record with timestamp; all per-frame records have FK to frame

**Considerations**: (a) is cleanest but invasive. (b) adds complexity. (c) is normalized but complex. (d) loses precise timing information.

**Human Recommendation**: If everything is synchronized once per world model update, the timestamps can directly be associated with the world model. This is essentially a version of (c), where the World Model is the separate table.

**Response**: Agreed. This is essentially **(e)**. We create a `SimulationFrameRecord` with:
- `id` (primary key)
- `simulation_time` (double, simulation seconds)
- `wall_clock_time` (optional, for diagnostics)

All per-step records (`InertialStateRecord`, etc.) include a `ForeignKey<SimulationFrameRecord> frame` field. This:
- Avoids modifying all existing records (just add one FK field)
- Normalizes timestamp storage (single source of truth)
- Enables efficient queries ("all states at time T")
- Aligns with WorldModel's single-update-per-step semantics

### Q4: Database Schema Strategy

**Options:**
- **(a)** One table per record type (standard cpp_sqlite pattern)
- **(b)** Single denormalized table with JSON/BLOB payload column
- **(c)** Hybrid: typed tables for high-frequency records, generic table for rare events

**Recommendation**: (a) — Leverage existing `cpp_sqlite` ORM patterns. Each record type maps to its own table.  **human response** agreed. Please leverage what exists in cpp_sqlite and raise design issues when this is found to be unsatisfactory. We can make changes to this library.

### Q5: Recorder Lifecycle Management

**Options:**
- **(a)** RAII-based: Recorder owns thread, destructor joins and flushes
- **(b)** Explicit start()/stop() methods with manual lifecycle control
- **(c)** Static singleton recorder (global access)

**Recommendation**: (a) — RAII is idiomatic and prevents resource leaks. Constructor starts thread, destructor flushes and joins.

---

## Technical Risks

1. **Queue contention under high load**: If records are produced faster than they can be written, the queue will grow or overflow. Mitigation: bounded queue with configurable size and overflow policy.

2. **Database write latency spikes**: SQLite can have variable write latency, especially on first write or when flushing to disk. Mitigation: batch commits, WAL mode for SQLite.

3. **Record serialization overhead**: `toRecord()` conversion happens on the simulation thread. If serialization is expensive, this could impact simulation performance. Mitigation: keep records lightweight (they already are), profile if needed.

   **Human comment**: Can this conversion occur on the Data Recorder thread?

   **Response**: Yes, this is possible with a different architecture:
   - **Current approach**: Simulation thread calls `toRecord()`, then `dao.addToBuffer(record)`. Record conversion on sim thread.
   - **Alternative approach**: Pass raw domain object references/copies to recorder thread, which calls `toRecord()`. This requires:
     - Either copying domain objects (memory overhead) or
     - Ensuring domain objects remain valid until recorder processes them (synchronization complexity)

   **Trade-off analysis**:
   - `toRecord()` for `InertialState` is ~20 field copies (floats/doubles) — very cheap (~100ns estimated)
   - Copying domain objects to defer `toRecord()` would be similar cost plus allocation overhead
   - Recommendation: Keep `toRecord()` on simulation thread for simplicity. Profile if needed; optimize later if proven bottleneck.

   However, if WorldModel integration is designed such that the recorder thread *reads* state directly (see soft constraint below), then `toRecord()` naturally moves to the recorder thread.

4. **Thread shutdown ordering**: If simulation ends abruptly, pending records may be lost. Mitigation: explicit flush() method, RAII destructor that blocks until queue is drained.

5. **Type erasure overhead**: If using `std::any` or similar, there's runtime overhead for type recovery. Mitigation: measure and optimize if needed; consider variant if type set is small.

6. **Foreign key relationships**: Records with foreign keys (e.g., `InertialStateRecord` contains `CoordinateRecord`) need careful handling. Current `cpp_sqlite` pattern embeds nested records. Mitigation: verify nested record insertion works correctly.

---

## Constraints

- Must use existing `cpp_sqlite` library for database access (no new database dependencies)
- **cpp_sqlite Enhancement Policy**: If during the design phase a deficiency in `cpp_sqlite` is identified (e.g., missing transaction support), create a separate ticket for that enhancement. Assume such tickets will be directly passed to development on that library — do not let missing `cpp_sqlite` features block the DataRecorder design. Document the required enhancement as a dependency and design against the expected interface.
- Must leverage existing `msd-transfer` record types and `toRecord()` pattern
- Recorder thread must not require modifications to `WorldModel` physics loop internals

  **Human Comment**: Consider this a soft constraint. If this constraint negatively affects simplicity or our ability to accomplish the multithreading part of this effort, consider the performance gain as higher priority.

  **Response**: Acknowledged. Design phase should evaluate two approaches:
  1. **Non-invasive**: Recorder is standalone; simulation code explicitly calls `recorder.record(state)` at appropriate points
  2. **Integrated**: WorldModel owns/knows about recorder; calls recording automatically after each physics step

  Option 2 is cleaner for users and naturally moves `toRecord()` to a point where simulation state is consistent. Will evaluate during design.
- No new third-party threading libraries (use C++ standard library threading)
- Should not require changes to existing `*Record` structs (unless timestamp is added per Q3)

---

## Acceptance Criteria

- [ ] **AC1**: `DataRecorder` class exists with RAII-based lifecycle (constructor opens DB + starts thread, destructor flushes + joins)
- [ ] **AC2**: Records can be submitted from simulation thread without blocking (< 1μs typical submission time)

  **Human comment**: Please consider if it's possible to submit records from the datarecorder thread.

  **Response**: Yes, two viable approaches:
  1. **Push model** (current): Simulation thread pushes records via `dao.addToBuffer()` — thread-safe, proven in cpp_sqlite
  2. **Pull model**: Recorder thread periodically reads WorldModel state directly, calls `toRecord()` on recorder thread

  Pull model advantages:
  - Zero overhead on simulation thread (no `addToBuffer()` calls)
  - `toRecord()` conversion happens on recorder thread
  - Natural synchronization point (recorder reads after simulation step completes)

  Pull model challenges:
  - Requires synchronization to ensure consistent state read
  - WorldModel must expose state for reading (may need accessor methods)

  **Recommendation**: Design should explore pull model as primary approach if WorldModel integration is allowed (per soft constraint relaxation).
- [ ] **AC3**: `InertialStateRecord` can be recorded successfully via `InertialState::toRecord()`
- [ ] **AC4**: Multiple record types can be recorded to the same database in interleaved fashion
- [ ] **AC5**: Graceful shutdown: all pending records flushed to database before destructor returns
- [ ] **AC6**: Queue overflow handled without crash (oldest records dropped with logged warning)
- [ ] **AC7**: Batch commits work correctly (configurable batch size, default ~100 records)

  **Human comment**: Please review the cpp_sqlite library for existing batching mechanisms.

  **cpp_sqlite Review Finding**: The library already has batching via double-buffered DAOs:
  - `addToBuffer()` accumulates records in `writeBuffer_`
  - `insert()` (no args) swaps `writeBuffer_` ↔ `flushBuffer_`, then inserts all buffered records
  - Buffer swap is protected by mutex; actual DB writes happen without lock held

  **Note**: The library does NOT currently have explicit SQL transaction support (`BEGIN TRANSACTION`/`COMMIT`). Each `insert()` is auto-committed. For high-volume recording, we may want to add transaction support to `cpp_sqlite::Database` to wrap batch inserts in a single transaction (significant performance improvement for SQLite).
- [ ] **AC8**: Database contains expected records after simulation run (verified via SQL query in test)
- [ ] **AC9**: Unit tests cover: normal operation, shutdown flush, queue overflow, multi-type recording

---

## Scope Boundaries

### In Scope
- `DataRecorder` class with background thread
- Thread-safe record queue
- Write path: domain object → `toRecord()` → queue → database
- Basic configuration (database path, batch size, queue depth)
- Unit tests for recorder functionality

### Out of Scope (Future Work)
- Read/playback functionality (separate ticket)
- Query API for recorded data
- Data compression or archival
- Real-time streaming to external systems
- Integration with `WorldModel` (this ticket provides the tool; integration is separate)
- UI for viewing recorded data

---

## References

### Existing Infrastructure
- [msd-transfer/CLAUDE.md](msd/msd-transfer/CLAUDE.md) — Transfer object patterns and cpp_sqlite usage
- [InertialState.hpp](msd/msd-sim/src/Physics/RigidBody/InertialState.hpp) — Example domain object with `toRecord()`
- [InertialStateRecord.hpp](msd/msd-transfer/src/InertialStateRecord.hpp) — Example transfer record
- [Records.hpp](msd/msd-transfer/src/Records.hpp) — All available record types

### cpp_sqlite Patterns
- Database creation: `cpp_sqlite::Database db{"path.db", true};`
- DAO access: `auto& dao = db.getDAO<RecordType>();`
- Direct insert: `dao.insert(record);` — auto-commits
- Buffered insert: `dao.addToBuffer(record);` + `dao.insert();` — thread-safe double-buffer
- Type-erased flush: `DAOBase* base = &dao; base->insert();` — flushes buffer via virtual call
- **Note**: Explicit `beginTransaction()`/`commit()` NOT currently implemented (potential enhancement)

### Related Tickets
- `0030_lagrangian_quaternion_physics` — Added `toRecord()/fromRecord()` to InertialState
- `0037_datatype_simplification` — Refactoring datatypes (in progress, may affect record types)

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-06
- **Completed**: 2026-02-06
- **Artifacts**: Initial ticket created
- **Notes**: Ticket created to establish write-only simulation data recording mechanism. Leverages existing msd-transfer records and cpp_sqlite ORM. Key design decisions pending: thread communication (Q1), type abstraction (Q2), timestamp strategy (Q3). Background thread approach chosen to decouple I/O from simulation performance.

### Draft Review (Human + cpp_sqlite Library Review)
- **Completed**: 2026-02-06
- **Key Findings**:

  1. **cpp_sqlite already has double-buffered DAOs** — `DataAccessObject<T>` provides thread-safe `addToBuffer()` and batch `insert()` via `DAOBase` virtual interface. No need to build custom queue/buffer mechanism.

  2. **Type abstraction solved** — `DAOBase*` polymorphism allows recorder to hold vector of DAOs and flush all via `dao->insert()`. Each DAO manages its own typed buffer internally.

  3. **Timestamp strategy agreed** — Create `SimulationFrameRecord` table with timestamp; all per-frame records have FK to frame. Aligns with WorldModel single-update semantics.

  4. **Pull model viable** — If WorldModel integration is allowed (soft constraint relaxed), recorder thread can read state directly and call `toRecord()` on its own thread, achieving zero simulation-thread overhead.

  5. **Transaction support needed** — cpp_sqlite lacks explicit `BEGIN TRANSACTION`/`COMMIT`. May need to add this for high-volume recording performance. Flag for potential cpp_sqlite enhancement.

  6. **WorldModel integration preferred** — Relaxed constraint opens cleaner design where WorldModel owns recorder and triggers recording after each physics step.

- **Status**: Ready for Design phase

### Design Phase
- **Started**: 2026-02-06
- **Completed**: 2026-02-06
- **Artifacts**:
  - `docs/designs/0038_simulation_data_recorder/design.md` — Architectural design document
  - `docs/designs/0038_simulation_data_recorder/0038_simulation_data_recorder.puml` — PlantUML diagram
- **Notes**: Design leverages cpp_sqlite double-buffered DAOs as recommended in draft review. DataRecorder owns background thread that wakes periodically to flush all registered DAOs. SimulationFrameRecord provides frame-based timestamping via FK pattern. WorldModel integration is opt-in (std::optional<DataRecorder>) to minimize impact. Push model chosen (simulation thread calls addToBuffer) over pull model for simplicity and proven performance. Open questions documented for flush interval tuning, transaction support prototyping, and record type selection for MVP.

### Design Review
- **Started**: 2026-02-06
- **Completed**: 2026-02-06
- **Reviewer**: design-reviewer agent
- **Iterations**: 2 (initial review → revision → final approval)
- **Issues Addressed**:
  1. **I1**: Added `std::atomic<uint32_t> nextFrameId_` for pre-assigning frame IDs before buffering
  2. **I2**: Specified `#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>` for InertialStateRecord
  3. **I3**: Changed `std::optional<DataRecorder>` to `std::unique_ptr<DataRecorder>` (non-movable type)
  4. **I4**: Added `std::mutex flushMutex_` to protect concurrent flush operations
  5. **I5**: Added explicit code example showing `record.frame.id = frameId` assignment
- **Outcome**: APPROVED — Design ready for prototype phase

### Prototype Phase
- **Started**: 2026-02-06
- **Completed**: 2026-02-06
- **Time Invested**: 2 hours (within time box)
- **Artifacts**:
  - `prototypes/0038_simulation_data_recorder/p1_transaction_perf/` — Transaction performance benchmark
  - `prototypes/0038_simulation_data_recorder/p2_flush_interval/` — Flush interval sensitivity analysis
  - `docs/designs/0038_simulation_data_recorder/prototype-results.md` — Comprehensive results and implementation ticket
- **Key Findings**:
  1. **P1 Transaction Performance**: VALIDATED — Transactions provide 74-358x speedup for batch sizes 100-1000 (far exceeding 2x target)
  2. **P2 Flush Interval**: VALIDATED — All tested intervals (10-500ms) meet criteria; worst-case buffer only 640KB (15x under 10MB limit) with <8ms flush latency (125x under 1s limit)
  3. **Recommended defaults**: 100ms flush interval, always use transactions
- **Outcome**: Both prototypes VALIDATED. Design assumptions confirmed. Ready for implementation.

### Implementation Phase
- **Started**: 2026-02-06
- **Completed**: 2026-02-06
- **Artifacts Created**:

  **Phase 1: Transfer Objects (msd-transfer)**
  - `msd/msd-transfer/src/SimulationFrameRecord.hpp` — NEW: Frame timestamping record
  - `msd/msd-transfer/src/InertialStateRecord.hpp` — MODIFIED: Added FK to SimulationFrameRecord
  - `msd/msd-transfer/src/Records.hpp` — MODIFIED: Includes SimulationFrameRecord

  **Phase 2: DataRecorder Core (msd-sim/src/DataRecorder/)**
  - `msd/msd-sim/src/DataRecorder/DataRecorder.hpp` — NEW: DataRecorder class with Config, jthread, atomic nextFrameId_
  - `msd/msd-sim/src/DataRecorder/DataRecorder.cpp` — NEW: Implementation with chunked sleep for responsive shutdown
  - `msd/msd-sim/src/DataRecorder/CMakeLists.txt` — NEW: Build integration

  **Phase 3: WorldModel Integration**
  - `msd/msd-sim/src/Environment/WorldModel.hpp` — MODIFIED: Added enableRecording/disableRecording/recordCurrentFrame
  - `msd/msd-sim/src/Environment/WorldModel.cpp` — MODIFIED: Implementation with FK assignment pattern

  **Phase 4: Testing**
  - `msd/msd-sim/test/DataRecorder/DataRecorderTest.cpp` — NEW: 10 unit tests covering construction, frame recording, DAO access, flush, background thread, and integration
  - `msd/msd-sim/test/DataRecorder/CMakeLists.txt` — NEW: Test build integration

- **Key Implementation Decisions**:
  1. Used chunked sleep (10ms intervals) for responsive shutdown instead of condition_variable (simpler, avoids mutex ordering issues)
  2. Member variable order: `recorderThread_` declared LAST to ensure all other members initialized before thread starts
  3. Final flush performed in thread exit rather than destructor to avoid race conditions
  4. `BOOST_DESCRIBE_STRUCT` must be inside namespace for cpp_sqlite ORM compatibility
  5. WorldModel destructor defined in .cpp (not header) for unique_ptr forward-declaration pattern

- **Tests**: 10 new tests, all passing (569 total tests in msd_sim_test)
- **Build**: Full project builds without warnings
- **Outcome**: Implementation complete. Ready for quality gate review.

### Quality Gate Review
- **Started**: 2026-02-06
- **Completed**: 2026-02-06
- **Reviewer**: cpp-code-reviewer agent
- **Verdict**: **PASS WITH COMMENTS**

**Executive Summary**: The implementation demonstrates excellent adherence to modern C++ best practices and project standards. Code is production-ready.

**Strengths Identified**:
1. Exemplary use of C++20 threading primitives (jthread, stop_token, atomic)
2. Correct member initialization order preventing race conditions
3. Proper use of standard library containers and algorithms
4. Clean separation of concerns
5. Comprehensive test coverage (10 tests covering all key scenarios)

**Minor Issues** (non-blocking) — **ALL ADDRESSED**:
1. ✅ `DataRecorder.hpp:61` — Documentation clarified: "if database path is invalid or cannot be opened"
2. ✅ `DataRecorder.cpp` — Template definition moved before explicit instantiations (standard ordering)
3. ✅ `WorldModel.cpp` — Used `std::chrono::duration<double>(time_).count()` for type-safe conversion
4. ✅ `DataRecorderTest.cpp:305` — Fixed sign-conversion warning with `size_t` loop variable

**CLAUDE.md Compliance**: All coding standards verified ✅
- Brace initialization used throughout
- Rule of Five correctly applied (all deleted for thread ownership)
- Proper memory management (unique_ptr, no raw pointers)
- Naming conventions followed

**Outcome**: APPROVED for merge. All minor issues addressed.

### Documentation Phase
- **Started**: 2026-02-06
- **Completed**: 2026-02-06
- **Artifacts Created**:
  - `docs/msd/msd-sim/DataRecorder/data-recorder.puml` — Library diagram adapted from design (removed feature highlighting)
  - `docs/designs/0038_simulation_data_recorder/doc-sync-summary.md` — Documentation sync summary

  **msd-sim CLAUDE.md Updates**:
  - Added DataRecorder to Core Modules table
  - Added DataRecorder Module summary section
  - Added complete DataRecorder Component section with all required subsections (Purpose, Key Classes, Key Interfaces, Usage Example, Architecture, Thread Safety, Error Handling, Memory Management, Dependencies, Performance, WorldModel Integration)
  - Added diagram to Diagrams Index table
  - Added "Simulation Data Recorder — 2026-02-06" to Recent Architectural Changes with comprehensive details

  **msd-transfer CLAUDE.md Updates**:
  - Added SimulationFrameRecord and InertialStateRecord to file structure listing
  - Added both records to Key Classes table
  - Added both records to Database Schema table

- **Key Documentation Decisions**:
  1. Single comprehensive diagram (data-recorder.puml) covers entire subsystem — no need for separate core vs component diagrams
  2. Adapted design diagram for library context by removing feature highlighting and changing package labels
  3. Documented member initialization order (recorderThread_ last) as critical safety pattern
  4. Comprehensive thread safety documentation per method (atomic operations, mutex protection)
  5. Cross-library impact documented in both msd-sim and msd-transfer CLAUDE.md files

- **Outcome**: Documentation complete. All diagram links verified, formatting consistent, no broken references.

---

## Human Feedback

*(Space for reviewer comments at any point in the workflow)*

