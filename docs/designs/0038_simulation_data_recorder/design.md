# Design: Simulation Data Recorder

## Summary

This design implements a write-only data recording mechanism that captures simulation state to a SQLite database on a dedicated background thread. The recorder leverages existing `cpp_sqlite` double-buffered DAOs for thread-safe record submission and periodic batch flushing. A new `SimulationFrameRecord` table provides frame-based timestamping, allowing all per-frame records to reference a single simulation time via foreign keys. The recorder operates autonomously on its own thread, waking periodically to flush buffered records without blocking the simulation thread.

## Architecture Changes

### PlantUML Diagram
See: `./0038_simulation_data_recorder.puml`

### New Components

#### DataRecorder

- **Purpose**: Orchestrates background recording of simulation data to SQLite database
- **Header location**: `msd/msd-sim/src/DataRecorder/DataRecorder.hpp`
- **Source location**: `msd/msd-sim/src/DataRecorder/DataRecorder.cpp`
- **Key interfaces**:
  ```cpp
  class DataRecorder {
  public:
      // Configuration for flush behavior
      struct Config {
          std::chrono::milliseconds flushInterval{100};  // Flush every 100ms
          std::string databasePath;
      };

      explicit DataRecorder(const Config& config);
      ~DataRecorder();

      // Delete copy/move (thread ownership)
      DataRecorder(const DataRecorder&) = delete;
      DataRecorder& operator=(const DataRecorder&) = delete;
      DataRecorder(DataRecorder&&) = delete;
      DataRecorder& operator=(DataRecorder&&) = delete;

      // Create new simulation frame with timestamp (adds to buffer)
      // Returns the pre-assigned frame ID for FK references in child records
      uint32_t recordFrame(double simulationTime);

      // Get DAO for adding records (thread-safe addToBuffer)
      template<typename T>
      cpp_sqlite::DataAccessObject<T>& getDAO();

      // Explicit flush (e.g., before shutdown)
      // Thread-safe: uses internal mutex to prevent concurrent flushes
      void flush();

      // Access database for queries (const only)
      const cpp_sqlite::Database& getDatabase() const;

  private:
      void recorderThreadMain(std::stop_token stopToken);

      std::unique_ptr<cpp_sqlite::Database> database_;
      std::jthread recorderThread_;
      std::chrono::milliseconds flushInterval_;
      std::mutex flushMutex_;                      // Protects flush operations
      std::atomic<uint32_t> nextFrameId_{1};       // Pre-assigns frame IDs before buffering
  };
  ```

- **Dependencies**:
  - `cpp_sqlite::Database` — Database connection, DAO management, and bulk flushing via `flushAllDAOs()`
  - `cpp_sqlite::DataAccessObject<T>` — Thread-safe buffered record insertion
  - `cpp_sqlite::Transaction` — RAII transaction management for batch writes
  - `SimulationFrameRecord` — Frame timestamping record
  - `<thread>`, `<stop_token>` — C++20 threading primitives

- **Thread safety**:
  - **Constructor/Destructor**: Not thread-safe, single-threaded initialization/teardown
  - **recordFrame()**: Thread-safe, uses atomic `nextFrameId_` for ID assignment and mutex-protected `addToBuffer()`
  - **getDAO()**: Thread-safe, returns reference to thread-safe DAO (managed by Database)
  - **flush()**: Thread-safe, acquires `flushMutex_` to prevent concurrent flushes with recorder thread
  - **getDatabase()**: Thread-safe (const access only)

- **Error handling**:
  - Constructor throws if database path is invalid or database creation fails
  - getDAO<T>() delegates to `database_->getDAO<T>()` which creates DAO on first access
  - Database write errors logged but do not crash recorder thread

- **cpp_sqlite Features Used**:
  - `Database::flushAllDAOs()` — Flushes all registered DAOs in creation order (no manual DAO tracking needed)
  - `Database::withTransaction()` — Wraps batch inserts in transaction for performance
  - `DataAccessObject<T>::addToBuffer()` — Thread-safe record buffering

#### SimulationFrameRecord

- **Purpose**: Database record representing a single simulation frame with timestamp
- **Header location**: `msd/msd-transfer/src/SimulationFrameRecord.hpp`
- **Key interfaces**:
  ```cpp
  namespace msd_transfer {

  /**
   * @brief Database record for simulation frame timestamping
   *
   * Each simulation frame has exactly one SimulationFrameRecord.
   * Per-frame records (InertialStateRecord, etc.) reference this
   * frame via ForeignKey<SimulationFrameRecord> for temporal association.
   *
   * @see DataRecorder::recordFrame()
   */
  struct SimulationFrameRecord : public cpp_sqlite::BaseTransferObject {
      using Id = cpp_sqlite::PrimaryKey<SimulationFrameRecord>;

      double simulation_time{0.0};    // Simulation time [seconds]
      double wall_clock_time{0.0};    // Wall clock time [seconds since epoch]
  };

  }  // namespace msd_transfer

  // Register with Boost.Describe for cpp_sqlite ORM
  BOOST_DESCRIBE_STRUCT(msd_transfer::SimulationFrameRecord,
                        (cpp_sqlite::BaseTransferObject),
                        (simulation_time, wall_clock_time));
  ```

- **Dependencies**: `cpp_sqlite::BaseTransferObject`, `Boost.Describe`
- **Thread safety**: Immutable after creation (value semantics)
- **Error handling**: None (pure data struct)

### Modified Components

#### WorldModel

- **Current location**: `msd/msd-sim/src/Environment/WorldModel.hpp`
- **Changes required**:
  - Add `std::unique_ptr<DataRecorder> dataRecorder_` member (not `std::optional` — DataRecorder is non-movable due to thread ownership)
  - Add `enableRecording(const std::string& dbPath, std::chrono::milliseconds flushInterval)` method
    - Implementation: `dataRecorder_ = std::make_unique<DataRecorder>(config);`
  - Add `disableRecording()` method
    - Implementation: `dataRecorder_.reset();` (triggers flush and thread join via RAII)
  - Modify `update()` to call `recordCurrentFrame()` after physics step if recorder enabled
    - Check: `if (dataRecorder_) { recordCurrentFrame(); }`
  - Add private `recordCurrentFrame()` method that:
    1. Gets frame ID via `auto frameId = dataRecorder_->recordFrame(simTime);`
    2. Iterates over all inertial assets
    3. For each asset: calls `state.toRecord()`, sets `record.frame.id = frameId`, then calls `addToBuffer(record)`

- **Backward compatibility**: Recording is opt-in. Existing code unaffected if `enableRecording()` not called.

#### InertialStateRecord

- **Current location**: `msd/msd-transfer/src/InertialStateRecord.hpp`
- **Changes required**:
  - Add `#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>` header
  - Add forward declaration or include for `SimulationFrameRecord`
  - Add `cpp_sqlite::ForeignKey<SimulationFrameRecord> frame` member
  - Update `BOOST_DESCRIBE_STRUCT` to include `frame` field

- **Backward compatibility**: Adding FK field is additive. Existing database queries unaffected (FK field will be NULL for old records if database migration not enforced).

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| DataRecorder | cpp_sqlite::Database | Composition | DataRecorder owns Database instance; uses `flushAllDAOs()` for bulk flush |
| DataRecorder | cpp_sqlite::Transaction | Usage | Uses `withTransaction()` for batch writes |
| DataRecorder | std::jthread | Composition | Owns background thread, joins on destruction |
| SimulationFrameRecord | BaseTransferObject | Inheritance | Standard cpp_sqlite record pattern |
| WorldModel | DataRecorder | Composition | Optional ownership via std::unique_ptr (non-movable type) |
| InertialStateRecord | SimulationFrameRecord | Foreign key | FK relationship for timestamping |

## Implementation Strategy

### Phase 1: Transfer Objects (msd-transfer)
1. Create `SimulationFrameRecord.hpp` with timestamp fields
2. Modify `InertialStateRecord.hpp` to add FK to SimulationFrameRecord
3. Update `Records.hpp` to include SimulationFrameRecord

### Phase 2: DataRecorder Core (msd-sim/src/DataRecorder/)
1. Create `DataRecorder.hpp` with class definition
2. Implement constructor:
   - Open database via `cpp_sqlite::Database`
   - Initialize DAO for `SimulationFrameRecord` via `database_->getDAO<SimulationFrameRecord>()` — **MUST be first DAO created** to ensure correct flush ordering for FK integrity
   - Initialize `nextFrameId_` atomic counter to 1
   - Start recorder thread via `std::jthread`
3. Implement `getDAO<T>()`:
   - Delegate to `database_->getDAO<T>()` (DAOs created on first access, managed by Database)
4. Implement `recordFrame(simTime) -> uint32_t`:
   - Pre-assign frame ID: `uint32_t frameId = nextFrameId_.fetch_add(1);`
   - Create SimulationFrameRecord with `id = frameId`, simulation time, and wall-clock time
   - Call `database_->getDAO<SimulationFrameRecord>().addToBuffer(record)` (buffered insert per human preference)
   - Return `frameId` for use in child record FK assignment
5. Implement `recorderThreadMain(stopToken)`:
   - Loop while `!stopToken.stop_requested()`
   - Sleep for `flushInterval_` milliseconds
   - Acquire `flushMutex_` lock
   - Call `database_->withTransaction([&]() { database_->flushAllDAOs(); })` — flush all DAOs in single transaction
   - Release lock
6. Implement destructor:
   - Call `flush()` to ensure all pending records written
   - `std::jthread` automatically joins on destruction
7. Implement `flush()`:
   - Acquire `flushMutex_` lock (prevents concurrent flush with recorder thread)
   - Call `database_->withTransaction([&]() { database_->flushAllDAOs(); })`
   - Release lock

### Phase 3: WorldModel Integration
1. Add `std::unique_ptr<DataRecorder> dataRecorder_` member (not `std::optional` — DataRecorder is non-movable)
2. Implement `enableRecording(dbPath, flushInterval)`:
   - Construct `DataRecorder` with config via `std::make_unique<DataRecorder>(config)`
   - Store in `dataRecorder_`
   - (DAOs created automatically on first `getDAO<T>()` call — no explicit registration needed)
3. Implement `disableRecording()`:
   - Call `dataRecorder_.reset()` (triggers flush and thread join via RAII destructor)
4. Modify `update()`:
   - After physics step, if `dataRecorder_` is active, call `recordCurrentFrame()`
   - Check: `if (dataRecorder_) { recordCurrentFrame(); }`
5. Implement `recordCurrentFrame()`:
   ```cpp
   void WorldModel::recordCurrentFrame() {
       // Get pre-assigned frame ID from recorder
       uint32_t frameId = dataRecorder_->recordFrame(currentSimulationTime_);

       // Record all inertial states with FK to this frame
       auto& stateDAO = dataRecorder_->getDAO<InertialStateRecord>();
       for (const auto& asset : inertialAssets_) {
           auto record = asset.getState().toRecord();
           record.frame.id = frameId;  // Explicit FK assignment
           stateDAO.addToBuffer(record);
       }
   }
   ```

### Phase 4: Testing
1. Unit tests for DataRecorder:
   - Test thread creation/destruction
   - Test DAO registration
   - Test frame creation
   - Test flush behavior
2. Integration tests for WorldModel:
   - Test recording enable/disable
   - Test frame timestamping
   - Test multi-object recording
   - Verify database contents via SQL query

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| None | N/A | No breaking changes | No action |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| DataRecorder | Constructor_CreatesDatabase_AndStartsThread | Database created, thread started |
| DataRecorder | Destructor_FlushesAndJoinsThread | All pending records written before destruction |
| DataRecorder | GetDAO_CreatesDAOOnFirstAccess | DAO created via database_->getDAO<T>() |
| DataRecorder | RecordFrame_CreatesFrameRecord | Frame record created with correct timestamp |
| DataRecorder | RecorderThread_FlushesDAOsPeriodically | Background thread calls flushAllDAOs() on interval |
| DataRecorder | Flush_UsesTransaction | Explicit flush wraps flushAllDAOs() in transaction |
| SimulationFrameRecord | DatabaseSchema_CreatesCorrectTable | cpp_sqlite generates correct schema |
| InertialStateRecord | ForeignKey_ReferencesFrame | FK relationship works |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| WorldModel_EnableRecording_StartsRecorder | WorldModel, DataRecorder | Recording can be enabled during runtime |
| WorldModel_DisableRecording_StopsRecorder | WorldModel, DataRecorder | Recording can be disabled during runtime |
| WorldModel_RecordFrame_CreatesFrameAndStates | WorldModel, DataRecorder, InertialStateRecord | Frame and state records created correctly |
| WorldModel_MultiObjectRecording_AllStatesRecorded | WorldModel, DataRecorder | Multiple objects recorded in single frame |
| WorldModel_DatabaseQuery_VerifiesRecordedData | WorldModel, DataRecorder, Database | Recorded data can be queried from database |
| WorldModel_LongRun_NoMemoryLeak | WorldModel, DataRecorder | Recorder thread does not leak memory over time |
| WorldModel_FlushOnShutdown_AllRecordsWritten | WorldModel, DataRecorder | Graceful shutdown writes all pending records |

#### Benchmark Tests

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| DataRecorder | RecordSubmission_Latency | Time to call addToBuffer() | < 1 μs (NFR-1) |
| DataRecorder | FlushThroughput | Records flushed per second | > 10,000 records/sec |
| WorldModel | Update_WithRecording_Overhead | Simulation frame time with recording enabled | < 5% overhead vs no recording |

## Open Questions

### Design Decisions (Human Input Needed)

1. **Flush Interval Default**
   - Option A: 100ms — Provides ~10 flushes/sec, balances I/O amortization with memory usage
   - Option B: 1000ms (1 sec) — Reduces I/O frequency, higher memory usage for buffered records
   - Option C: Configurable with 100ms default — Maximum flexibility
   - **Recommendation**: Option C — Allows tuning based on simulation characteristics **Response** agreed

2. **WorldModel Integration Approach**
   - Option A: Non-invasive — Recorder is optional member, recording is explicit opt-in
   - Option B: Integrated — WorldModel always owns recorder, enabled by default if dbPath provided
   - **Recommendation**: Option A — Minimizes impact, follows soft constraint preference from ticket **response** yes, this is fine for now

3. **Frame Record Creation Timing**
   - Option A: Direct insert on simulation thread — Ensures frame ID available immediately for FK references
   - Option B: Buffered insert on recorder thread — Consistent with other records, but requires FK back-reference
   - **Recommendation**: Option A — Simpler FK association, direct insert is fast (< 1ms), not a bottleneck **Response** I think I would prefer option B. Saying "This DB Operation is fast so it's not a big deal" is a slippery slope when we're talking about recording all of the data in the simulation environment, which will very quickly become a lot.

4. **InertialStateRecord FK Field Name**
   - Option A: `frame` — Concise, matches typical ORM naming
   - Option B: `simulation_frame` — More explicit
   - **Recommendation**: Option A — Follows existing msd-transfer naming conventions **Response** agreed.

### Prototype Required

1. **Transaction Performance Assessment**
   - **Question**: Does wrapping `flushAllDAOs()` in `withTransaction()` provide meaningful performance improvement over auto-commit?
   - **Prototype**: Measure flush throughput with and without explicit transactions using existing cpp_sqlite `Transaction` class
   - **Decision criteria**: If > 2x speedup, always use transactions; otherwise make configurable
   - **Note**: cpp_sqlite now has full transaction support (`Transaction` class, `withTransaction()` helper, nested transactions via savepoints)

2. **Flush Interval Sensitivity Analysis**
   - **Question**: What is the optimal flush interval for typical simulation workloads (10-100 objects at 60 FPS)?
   - **Prototype**: Benchmark flush intervals from 10ms to 1000ms, measure memory usage and I/O latency
   - **Decision criteria**: Choose interval that keeps memory usage < 100MB and 99th percentile I/O latency < 10ms

### Requirements Clarification

1. **Record Selection for Initial Implementation**
   - **Question**: Which record types should be registered for recording in MVP? Just InertialStateRecord, or also CoordinateRecord, Vector3DRecord, etc.?
   - **Clarification needed**: Define minimal viable record set for first implementation
   - **Recommendation**: Start with InertialStateRecord only, add others in follow-on work

2. **Wall Clock Time Precision**
   - **Question**: Should wall_clock_time use `std::chrono::system_clock` (absolute time) or `std::chrono::steady_clock` (monotonic, not wall-clock)?
   - **Clarification needed**: Is absolute timestamp needed, or just monotonic elapsed time?
   - **Recommendation**: Use `std::chrono::system_clock::now().time_since_epoch().count()` for absolute wall-clock time (enables correlation with external events)

3. **Database Path Validation**
   - **Question**: Should DataRecorder validate that database path is writable before starting thread?
   - **Clarification needed**: Is it acceptable to fail during first write, or should constructor validate?
   - **Recommendation**: Constructor validates path is writable (fail-fast principle)

## Technical Risks and Mitigations

### Risk 1: Database Write Latency Spikes
**Risk**: SQLite can have variable write latency, especially on first write or WAL checkpoint.
**Impact**: Recorder thread blocked during flush, buffered records accumulate.
**Mitigation**:
- Enable WAL mode for SQLite (reduces checkpoint latency)
- Use configurable flush interval to tune I/O frequency
- Bounded buffer size in cpp_sqlite (overflow handled gracefully)

### Risk 2: Memory Growth from Buffered Records
**Risk**: If simulation produces records faster than recorder flushes, memory usage grows unbounded.
**Impact**: Out-of-memory crash.
**Mitigation**:
- Flush interval tuned to handle expected record rate
- cpp_sqlite DAOs have bounded buffers with overflow policy
- Monitoring of buffer sizes in debug builds

### Risk 3: Thread Shutdown Ordering
**Risk**: If WorldModel destructor is called before DataRecorder, pending records may be lost.
**Impact**: Last ~100ms of simulation not recorded.
**Mitigation**:
- `std::unique_ptr<DataRecorder>` ensures RAII destruction order (DataRecorder destroyed when unique_ptr is reset or WorldModel destroyed)
- DataRecorder destructor explicitly calls `flush()` before thread join
- Test coverage for graceful shutdown scenario

### Risk 4: Foreign Key Constraint Violations
**Risk**: If frame record insert fails, subsequent state records have invalid FK.
**Impact**: Database constraint violation, state records rejected.
**Mitigation**:
- Constructor validates database schema supports FK constraints
- Unit test coverage for FK validation

### Risk 5: DAO Flush Ordering with Buffered Frame Records
**Risk**: With Option B (buffered frame inserts), child records (InertialStateRecord) reference a Frame ID that may not be committed yet.
**Impact**: FK constraint violation if State records are flushed before Frame records.
**Mitigation**:
- `cpp_sqlite::Database::flushAllDAOs()` flushes DAOs in **creation order**
- **INVARIANT**: `SimulationFrameRecord` DAO must be accessed (via `getDAO<SimulationFrameRecord>()`) before any per-frame record DAOs (e.g., `InertialStateRecord`)
- DataRecorder constructor initializes frame DAO first: `database_->getDAO<SimulationFrameRecord>()`
- This ensures frame records are always flushed before state records within the same transaction
- Unit test required: verify flush order matches creation order

## Design Complexity Sanity Checks

### No Red Flags Detected

- **Combinatorial Overloads**: None — Single DataRecorder class, template methods for type safety
- **Smart Pointer for Optional Ownership**: `std::unique_ptr<DataRecorder>` in WorldModel is appropriate — DataRecorder is non-movable (owns thread), and unique_ptr provides nullable semantics for opt-in recording
- **Modified Components**: 2 modified vs 2 new — Balanced ratio, modifications are additive
- **Conditional Logic**: Minimal — Only `if (dataRecorder_)` check in WorldModel::update()

### Justification for std::unique_ptr<DataRecorder>

The use of `std::unique_ptr<DataRecorder>` (rather than `std::optional`) in WorldModel is required because:
1. DataRecorder is non-copyable and non-movable (owns `std::jthread`)
2. `std::optional<T>` requires T to be movable for `emplace()` and value assignment
3. `std::unique_ptr<T>` provides nullable semantics without requiring T to be movable
4. Opt-in pattern via `make_unique` / `reset()` is idiomatic for optional owned resources
5. Recording is genuinely optional functionality (not all simulations need persistence)

## Code Quality Gates Awareness

### Build Quality Requirements
- All code compiles without warnings under `-Wall -Wextra -Wpedantic -Werror`
- clang-tidy checks pass:
  - `modernize-*` (C++20 usage)
  - `performance-*` (thread safety, move semantics)
  - `readability-*` (naming conventions)

### Performance Considerations
- **Benchmark regression detection**: Record submission latency benchmarked (target < 1 μs per NFR-1)
- **Hot path impact**: WorldModel::update() with recording enabled should add < 5% overhead
- **Flush throughput**: Background thread should handle > 10,000 records/sec

### Test Infrastructure Requirements
- **Testability**: DataRecorder injectable via WorldModel::enableRecording()
- **Observable state**: Database contents queryable via SQL for verification
- **Mockable interfaces**: cpp_sqlite DAOs allow test doubles for unit testing
- **Thread safety testing**: TSAN (Thread Sanitizer) enabled for integration tests

## Notes

This design prioritizes simplicity by leveraging existing cpp_sqlite infrastructure:
- **Double-buffered DAOs**: Thread-safe `addToBuffer()` for record submission
- **`Database::flushAllDAOs()`**: Bulk flush all DAOs in creation order — no manual DAO tracking needed
- **`Database::withTransaction()`**: RAII transaction wrapper for batch writes
- **Nested transaction support**: Via savepoints for complex scenarios

The ticket's "Human Recommendation" to use cpp_sqlite buffers and transactions is fully addressed. The DataRecorder simply owns the Database and delegates all DAO management to it.

The pull model (recorder thread reads WorldModel state) was considered but rejected due to synchronization complexity. The push model (simulation thread calls `addToBuffer()`) is proven in cpp_sqlite and aligns with existing asset loading patterns. The `toRecord()` conversion happens on the simulation thread, which is acceptable given conversion cost is ~100ns for InertialState (20 field copies).

Frame-based timestamping (SimulationFrameRecord with FK references) provides normalized time storage and efficient queries ("all states at time T"). This aligns with WorldModel's single-update-per-step semantics and avoids adding timestamp fields to every record type.

## Human Questions

### Q1: DAO Storage in DataRecorder (RESOLVED)

> I want to review the way the DAOs are stored in the data recorder. The cpp_sqlite library provides a mechanism for automatically generating DAOs on request:
>
> ```cpp
> // Create DataAccessObject for our test TransferObject
> auto& productDAO = db.getDAO<TestProduct>();
> ```
>
> My expectation is that, if we're storing a unique pointer to the database, we won't need to specifically manage the DAOs individually as well. We just retrieve them from the database.

**Resolution**: Confirmed. The design has been updated to remove manual DAO tracking. cpp_sqlite now provides:

1. **`Database::flushAllDAOs()`** — Flushes all DAOs in creation order. DataRecorder calls this instead of tracking `std::vector<DAOBase*>`.
2. **`Database::withTransaction()`** — RAII transaction wrapper for batch writes.

The simplified DataRecorder design:
- Owns `std::unique_ptr<cpp_sqlite::Database>`
- Delegates `getDAO<T>()` to the database (DAOs created on first access)
- Calls `database_->withTransaction([&]() { database_->flushAllDAOs(); })` for periodic flush
- No manual DAO bookkeeping required

---

## Design Review — Initial Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-02-06
**Status**: REVISION_REQUESTED
**Iteration**: 0 of 1

### Issues Requiring Revision

| ID | Issue | Category | Required Change |
|----|-------|----------|-----------------|
| I1 | FK ID assignment mechanism not specified | C++ Quality / Feasibility | Clarify how InertialStateRecord obtains the frame ID for its FK field when using buffered inserts |
| I2 | Missing include for ForeignKey in InertialStateRecord | Architectural | Add `#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>` to InertialStateRecord.hpp |
| I3 | std::optional<DataRecorder> invalid for non-movable type | C++ Quality | Change to std::unique_ptr<DataRecorder> since DataRecorder is move-deleted |
| I4 | flush() thread-safety claim requires verification | Feasibility | Clarify synchronization between flush() called from main thread and recorder thread's periodic flush |
| I5 | InertialState::toRecord() FK assignment location unclear | Architectural | Specify where frame FK is set — toRecord() cannot know frame ID |

### Revision Instructions for Architect

The following changes must be made before final review:

1. **Issue I1 — FK ID Assignment Mechanism**

   The design states that frame records are buffered (Option B per human preference), but does not explain how child records (InertialStateRecord) obtain the frame ID for their FK field. The frame record is created via `addToBuffer()` but its ID is not assigned until database insertion.

   **Required clarification**: Specify one of these approaches:
   - (a) Pre-assign frame IDs using an atomic counter managed by DataRecorder (before buffering)
   - (b) Use cpp_sqlite's `PrimaryKey<T>` auto-increment and defer FK assignment to flush time
   - (c) Document that cpp_sqlite assigns IDs during `addToBuffer()` (verify this is actually the case)

   Recommend approach (a) — DataRecorder maintains `std::atomic<uint32_t> nextFrameId_` and assigns IDs before buffering. This allows frame ID to be returned from `recordFrame()` for use in subsequent FK assignments.

2. **Issue I2 — Missing ForeignKey Include**

   The modification to InertialStateRecord.hpp adding `ForeignKey<SimulationFrameRecord> frame` requires including the ForeignKey header. Existing pattern in the codebase (see MeshRecord.hpp line 10):
   ```cpp
   #include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>
   ```

   Update the "Changes required" section for InertialStateRecord to include this header.

3. **Issue I3 — std::optional vs std::unique_ptr for DataRecorder**

   The design specifies `std::optional<DataRecorder> dataRecorder_` in WorldModel, but DataRecorder explicitly deletes move operations:
   ```cpp
   DataRecorder(DataRecorder&&) = delete;
   DataRecorder& operator=(DataRecorder&&) = delete;
   ```

   `std::optional<T>` requires T to be either copyable or movable to support `emplace()` and `reset()`. Since DataRecorder is neither, `std::optional<DataRecorder>` will not compile.

   **Required change**: Replace `std::optional<DataRecorder>` with `std::unique_ptr<DataRecorder>`:
   ```cpp
   std::unique_ptr<DataRecorder> dataRecorder_;
   ```

   Update:
   - WorldModel member declaration
   - `enableRecording()` to use `std::make_unique<DataRecorder>(config)`
   - `disableRecording()` to use `dataRecorder_.reset()`
   - Conditional check to use `if (dataRecorder_)` (unchanged, works for both)

4. **Issue I4 — flush() Thread-Safety Clarification**

   The design claims `flush()` is thread-safe and can be called from any thread, but there is potential for race conditions:
   - Main thread calls `flush()` which calls `withTransaction([&] { flushAllDAOs(); })`
   - Recorder thread wakes from sleep and also calls `withTransaction([&] { flushAllDAOs(); })`

   Both threads would be calling `flushAllDAOs()` potentially concurrently.

   **Required clarification**: Specify synchronization approach:
   - (a) Add mutex in DataRecorder protecting flush operations
   - (b) Document that cpp_sqlite::Database is internally thread-safe for these operations (verify)
   - (c) Remove thread-safety claim for flush() and document that it should only be called when recorder thread is stopped

   Recommend approach (a) — Add `std::mutex flushMutex_` to DataRecorder and acquire lock in both `flush()` and `recorderThreadMain()` before flushing.

5. **Issue I5 — toRecord() FK Assignment Location**

   The implementation strategy (Phase 3, step 5) states:
   > "For each asset: call `asset.getState().toRecord()`, call `dataRecorder_->getDAO<InertialStateRecord>().addToBuffer(record)`"

   However, `InertialState::toRecord()` currently returns `InertialStateRecord` with no knowledge of the frame FK. The FK must be set separately.

   **Required clarification**: Update implementation strategy to show explicit FK assignment:
   ```cpp
   // In WorldModel::recordCurrentFrame():
   auto frameId = dataRecorder_->recordFrame(simTime);  // Returns assigned ID
   for (const auto& asset : inertialAssets_) {
     auto record = asset.getState().toRecord();
     record.frame.id = frameId;  // Explicit FK assignment
     dataRecorder_->getDAO<InertialStateRecord>().addToBuffer(record);
   }
   ```

   This requires `recordFrame()` to return the assigned frame ID, which ties back to Issue I1.

### Items Passing Review (No Changes Needed)

The following aspects of the design are well-conceived and should not be modified:

1. **cpp_sqlite Integration Strategy**: Leveraging existing `flushAllDAOs()`, `withTransaction()`, and `addToBuffer()` mechanisms is correct and aligns with the library's design.

2. **DAO Creation Order for FK Integrity**: The invariant that SimulationFrameRecord DAO must be created first (via constructor initialization) is correct and ensures proper flush ordering.

3. **RAII Thread Lifecycle**: Using `std::jthread` with `stop_token` for clean shutdown is idiomatic C++20.

4. **SimulationFrameRecord Design**: The record structure with `simulation_time` and `wall_clock_time` fields is appropriate.

5. **Test Strategy**: The proposed unit and integration tests provide adequate coverage.

6. **Risk Mitigations**: The identified risks and mitigations for FK constraints, memory growth, and thread shutdown are appropriate.

7. **PlantUML Diagram**: Accurately represents the component relationships and thread interactions.

8. **Namespace Organization**: Placing DataRecorder in `msd-sim/src/DataRecorder/` follows project conventions.

9. **Coding Standards Compliance**: Design follows project conventions for naming, initialization, and memory management.

---

## Design Revision — Addressing Review Feedback

**Date**: 2026-02-06
**Status**: REVISED — Ready for Final Review

### Changes Made

| Issue ID | Change Summary |
|----------|----------------|
| **I1** | Added `std::atomic<uint32_t> nextFrameId_` to DataRecorder for pre-assigning frame IDs before buffering. `recordFrame()` now returns `uint32_t` frame ID. |
| **I2** | Updated InertialStateRecord changes to include `#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>` and forward declaration for SimulationFrameRecord. |
| **I3** | Changed `std::optional<DataRecorder>` to `std::unique_ptr<DataRecorder>` in WorldModel. Updated `enableRecording()` to use `make_unique`, `disableRecording()` to use `reset()`. |
| **I4** | Added `std::mutex flushMutex_` to DataRecorder. Both `flush()` and `recorderThreadMain()` acquire this lock before flushing to prevent concurrent database operations. |
| **I5** | Updated Phase 3 implementation strategy with explicit code showing FK assignment: `record.frame.id = frameId;` after calling `toRecord()`. |

### Detailed Changes by Section

1. **DataRecorder Key Interfaces** (lines 19-62):
   - Added `flushMutex_` and `nextFrameId_` members
   - Changed `recordFrame()` return type from `void` to `uint32_t`
   - Updated thread safety documentation for `flush()`

2. **WorldModel Changes Required** (lines 125-140):
   - Changed from `std::optional<DataRecorder>` to `std::unique_ptr<DataRecorder>`
   - Added implementation details for `enableRecording()` and `disableRecording()`
   - Updated `recordCurrentFrame()` description with FK assignment

3. **InertialStateRecord Changes Required** (lines 143-152):
   - Added required include for ForeignKey header
   - Added forward declaration requirement

4. **Integration Points Table** (line 159):
   - Updated to reflect `std::unique_ptr` instead of `std::optional`

5. **Phase 2 Implementation Strategy** (lines 170-195):
   - Added frame ID pre-assignment via atomic counter
   - Added flush mutex acquisition in `recorderThreadMain()` and `flush()`
   - Documented DAO creation order invariant for FK integrity

6. **Phase 3 Implementation Strategy** (lines 197-220):
   - Changed to `std::unique_ptr` pattern
   - Added explicit code example for `recordCurrentFrame()` with FK assignment

7. **Design Complexity Sanity Checks** (lines 370-385):
   - Updated justification from `std::optional` to `std::unique_ptr`
   - Explained why unique_ptr is required (non-movable type)

8. **Risk 3 Mitigation** (lines 345-350):
   - Updated to reference `std::unique_ptr` instead of `std::optional`

---

## Design Review — Final Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-02-06
**Status**: APPROVED
**Iteration**: 1 of 1

### Issue Resolution Verification

All five issues identified in the initial review have been adequately addressed:

| Issue ID | Original Issue | Resolution | Verified |
|----------|----------------|------------|----------|
| I1 | FK ID assignment mechanism not specified | Added `std::atomic<uint32_t> nextFrameId_{1}` with `fetch_add(1)` for pre-assignment; `recordFrame()` returns assigned ID | Yes |
| I2 | Missing ForeignKey include in InertialStateRecord | Added `#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>` to required changes | Yes |
| I3 | `std::optional<DataRecorder>` invalid for non-movable type | Changed to `std::unique_ptr<DataRecorder>` throughout; added justification section | Yes |
| I4 | `flush()` thread-safety claim requires verification | Added `std::mutex flushMutex_` protecting both `flush()` and `recorderThreadMain()` | Yes |
| I5 | `toRecord()` FK assignment location unclear | Added explicit code example showing `record.frame.id = frameId` assignment in `recordCurrentFrame()` | Yes |

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | Pass | Classes use PascalCase, members use snake_case_, functions use camelCase |
| Namespace organization | Pass | `msd_transfer` for records, DataRecorder in `msd-sim` |
| File structure | Pass | `msd/msd-sim/src/DataRecorder/DataRecorder.hpp` follows project conventions |
| Dependency direction | Pass | msd-transfer has no deps on msd-sim; DataRecorder depends on cpp_sqlite |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | Pass | `std::jthread` for thread lifecycle, `std::unique_ptr` for database ownership |
| Smart pointer appropriateness | Pass | `unique_ptr<Database>` for ownership, `unique_ptr<DataRecorder>` in WorldModel |
| Value/reference semantics | Pass | Records use value semantics, DAOs accessed via reference |
| Rule of 0/3/5 | Pass | DataRecorder explicitly deletes copy/move, declares all five special members |
| Const correctness | Pass | `getDatabase()` returns const reference |
| Exception safety | Pass | Constructor throws on failure; thread errors logged but don't crash |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | Pass | ForeignKey include now specified; forward declarations documented |
| Template complexity | Pass | Single template method `getDAO<T>()` delegates to cpp_sqlite |
| Memory strategy | Pass | Buffered records with periodic flush; bounded by flush interval |
| Thread safety | Pass | Atomic for ID assignment; mutex for flush synchronization |
| Build integration | Pass | No new dependencies; uses existing cpp_sqlite |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | Pass | DataRecorder can be constructed standalone with test database path |
| Mockable dependencies | Pass | cpp_sqlite DAOs support test doubles |
| Observable state | Pass | Database contents queryable via SQL; `getDatabase()` provides access |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | DAO flush ordering depends on creation order invariant | Technical | Low | High | DataRecorder constructor initializes frame DAO first; unit test verifies order | No |
| R2 | Transaction performance vs auto-commit | Performance | Med | Low | Design already specifies prototype for this | Yes (existing) |
| R3 | Optimal flush interval unknown | Performance | Med | Med | Design already specifies prototype for this | Yes (existing) |

### Prototype Guidance

The design document already specifies appropriate prototypes in the "Prototype Required" section:

1. **Transaction Performance Assessment** — Measures `withTransaction()` speedup (already documented)
2. **Flush Interval Sensitivity Analysis** — Measures optimal interval for memory/latency tradeoff (already documented)

No additional prototypes required. The existing prototype guidance is well-specified with clear success criteria and time boxes.

### Summary

The revised design adequately addresses all issues identified in the initial review. The architecture is sound, with proper thread safety mechanisms (`flushMutex_`, `atomic nextFrameId_`), correct smart pointer usage (`unique_ptr` for non-movable DataRecorder), and explicit FK assignment in the recorded code examples. The design follows project coding standards and leverages existing cpp_sqlite infrastructure appropriately.

The design is ready for human review and subsequent prototype validation of transaction performance and flush interval tuning.

---
