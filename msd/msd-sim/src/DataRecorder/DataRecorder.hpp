// Ticket: 0038_simulation_data_recorder
// Design: docs/designs/0038_simulation_data_recorder/design.md

#ifndef MSD_SIM_DATA_RECORDER_HPP
#define MSD_SIM_DATA_RECORDER_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <stop_token>
#include <thread>

#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>

namespace msd_transfer
{
struct SimulationFrameRecord;
}

namespace msd_sim
{

/**
 * @brief Orchestrates background recording of simulation data to SQLite
 * database
 *
 * DataRecorder operates on a dedicated background thread, periodically flushing
 * buffered records to disk. Record submission from the simulation thread is
 * thread-safe via cpp_sqlite's double-buffered DAOs.
 *
 * Architecture:
 * - Simulation thread calls recordFrame() to create timestamped frames
 * - Simulation thread calls getDAO<T>().addToBuffer(record) for per-frame data
 * - Recorder thread wakes periodically to flush all buffers via transaction
 * - Destructor ensures all pending records are written before shutdown
 *
 * @see
 * docs/designs/0038_simulation_data_recorder/0038_simulation_data_recorder.puml
 * @ticket 0038_simulation_data_recorder
 */
class DataRecorder
{
public:
  /**
   * @brief Configuration for DataRecorder behavior
   */
  struct Config
  {
    std::chrono::milliseconds flushInterval{
      100};                    // Flush every 100ms (default)
    std::string databasePath;  // Path to SQLite database file
  };

  /**
   * @brief Construct DataRecorder and start background thread
   *
   * Opens database, initializes SimulationFrameRecord DAO first (for FK
   * integrity), and starts recorder thread.
   *
   * @param config Configuration for flush behavior and database path
   * @throws std::runtime_error if database path is invalid or cannot be opened
   *
   * @ticket 0038_simulation_data_recorder
   */
  explicit DataRecorder(const Config& config);

  /**
   * @brief Destructor flushes all pending records and joins thread
   *
   * Calls flush() explicitly before allowing jthread to join automatically.
   * Ensures no data loss on shutdown.
   *
   * @ticket 0038_simulation_data_recorder
   */
  ~DataRecorder();

  // Delete copy/move (thread ownership)
  DataRecorder(const DataRecorder&) = delete;
  DataRecorder& operator=(const DataRecorder&) = delete;
  DataRecorder(DataRecorder&&) = delete;
  DataRecorder& operator=(DataRecorder&&) = delete;

  /**
   * @brief Create new simulation frame with timestamp
   *
   * Pre-assigns frame ID atomically, creates SimulationFrameRecord with
   * simulation time and wall-clock time, adds to buffer. Returns the
   * pre-assigned frame ID for FK references in child records.
   *
   * Thread-safe: Uses atomic nextFrameId_ for ID assignment and mutex-protected
   * addToBuffer().
   *
   * @param simulationTime Current simulation time [seconds]
   * @return Pre-assigned frame ID for FK references
   *
   * @ticket 0038_simulation_data_recorder
   */
  uint32_t recordFrame(double simulationTime);

  /**
   * @brief Get DAO for adding records (thread-safe addToBuffer)
   *
   * Delegates to database_->getDAO<T>(). DAOs are created on first access
   * and managed by Database. All DAOs support thread-safe addToBuffer().
   *
   * @tparam T Record type (must inherit from BaseTransferObject)
   * @return Reference to DAO for type T
   *
   * @ticket 0038_simulation_data_recorder
   */
  template <typename T>
  cpp_sqlite::DataAccessObject<T>& getDAO();

  /**
   * @brief Explicit flush of all pending records
   *
   * Thread-safe: Acquires flushMutex_ to prevent concurrent flushes with
   * recorder thread. Wraps database_->flushAllDAOs() in transaction for
   * performance.
   *
   * @ticket 0038_simulation_data_recorder
   */
  void flush();

  /**
   * @brief Access database for queries (const only)
   *
   * Provides read-only access to database for verification/testing.
   * Write access should only occur through recorder mechanisms.
   *
   * @return Const reference to database
   *
   * @ticket 0038_simulation_data_recorder
   */
  const cpp_sqlite::Database& getDatabase() const;

private:
  /**
   * @brief Recorder thread main loop
   *
   * Sleeps for flushInterval_, then acquires flushMutex_ and flushes all DAOs
   * in a transaction. Continues until stop_token is requested.
   *
   * @param stopToken Token for clean shutdown signaling
   *
   * @ticket 0038_simulation_data_recorder
   */
  void recorderThreadMain(std::stop_token stopToken);

  std::unique_ptr<cpp_sqlite::Database> database_;
  std::chrono::milliseconds flushInterval_;
  std::mutex flushMutex_;  // Protects flush operations
  std::atomic<uint32_t> nextFrameId_{
    1};  // Pre-assigns frame IDs before buffering
  // IMPORTANT: recorderThread_ must be declared LAST to ensure all other
  // members are initialized before thread starts and destroyed after thread
  // joins
  std::jthread recorderThread_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_DATA_RECORDER_HPP
