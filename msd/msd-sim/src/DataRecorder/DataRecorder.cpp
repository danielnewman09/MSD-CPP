// Ticket: 0038_simulation_data_recorder
// Design: docs/designs/0038_simulation_data_recorder/design.md

#include <chrono>

#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include "msd-sim/src/DataRecorder/DataRecorder.hpp"
#include "msd-transfer/src/AppliedForceRecord.hpp"
#include "msd-transfer/src/AssetDynamicStateRecord.hpp"
#include "msd-transfer/src/BodyMetadataRecord.hpp"
#include "msd-transfer/src/ConstraintForceRecord.hpp"
#include "msd-transfer/src/ContactRecord.hpp"
#include "msd-transfer/src/EnergyRecord.hpp"
#include "msd-transfer/src/InertialStateRecord.hpp"
#include "msd-transfer/src/SimulationFrameRecord.hpp"
#include "msd-transfer/src/SolverDiagnosticRecord.hpp"
#include "msd-transfer/src/SystemEnergyRecord.hpp"

namespace msd_sim
{

DataRecorder::DataRecorder(const Config& config)
  : flushInterval_{config.flushInterval}
{
  // Open database
  database_ = std::make_unique<cpp_sqlite::Database>(config.databasePath, true);

  // Note: We don't initialize SimulationFrameRecord DAO explicitly here due to
  // a cpp_sqlite emplace bug with recent Clang versions. The DAO will be
  // created automatically on first recordFrame() call. The DAO creation order
  // (frame before state) is maintained because recordFrame() is always called
  // before state recording. Ticket: 0038_simulation_data_recorder

  // Start recorder thread
  recorderThread_ = std::jthread{[this](std::stop_token st)
                                 { recorderThreadMain(std::move(st)); }};
}

DataRecorder::~DataRecorder()
{
  // Request stop on the jthread (triggers stop_token)
  // The thread will check this on next loop iteration
  recorderThread_.request_stop();

  // jthread joins automatically on destruction, waiting for thread to finish

  // Note: No explicit flush() here because recorderThreadMain does a final
  // flush before exiting when stop is requested
}

uint32_t DataRecorder::recordFrame(double simulationTime)
{
  // Pre-assign frame ID atomically
  const uint32_t frameId = nextFrameId_.fetch_add(1);

  // Create SimulationFrameRecord with pre-assigned ID
  msd_transfer::SimulationFrameRecord record{};
  record.id = frameId;
  record.simulation_time = simulationTime;

  // Get wall-clock time in seconds since epoch
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  record.wall_clock_time =
    std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();

  // Buffer the frame record (thread-safe)
  database_->getDAO<msd_transfer::SimulationFrameRecord>().addToBuffer(record);

  return frameId;
}

void DataRecorder::flush()
{
  // Acquire mutex to prevent concurrent flush with recorder thread
  std::scoped_lock lock{flushMutex_};

  // Flush all DAOs in creation order within a transaction
  database_->withTransaction([this]() { database_->flushAllDAOs(); });
}

void DataRecorder::recorderThreadMain(std::stop_token stopToken)
{
  // Use smaller sleep intervals for responsive shutdown
  constexpr auto kSleepChunk = std::chrono::milliseconds{10};

  while (!stopToken.stop_requested())
  {
    // Sleep in small chunks to allow responsive shutdown
    auto remaining = flushInterval_;
    while (remaining > std::chrono::milliseconds{0} &&
           !stopToken.stop_requested())
    {
      auto sleepTime = std::min(remaining, kSleepChunk);
      std::this_thread::sleep_for(sleepTime);
      remaining -= sleepTime;
    }

    // Check if we were woken for shutdown
    if (stopToken.stop_requested())
    {
      break;
    }

    // Acquire flush mutex and flush all DAOs
    std::scoped_lock lock{flushMutex_};
    database_->withTransaction([this]() { database_->flushAllDAOs(); });
  }

  // Final flush before thread exits
  std::scoped_lock lock{flushMutex_};
  database_->withTransaction([this]() { database_->flushAllDAOs(); });
}

const cpp_sqlite::Database& DataRecorder::getDatabase() const
{
  return *database_;
}

// Template definition
template <typename T>
cpp_sqlite::DataAccessObject<T>& DataRecorder::getDAO()
{
  return database_->getDAO<T>();
}

// Explicit template instantiations for commonly used types
template cpp_sqlite::DataAccessObject<msd_transfer::SimulationFrameRecord>&
DataRecorder::getDAO<msd_transfer::SimulationFrameRecord>();

template cpp_sqlite::DataAccessObject<msd_transfer::InertialStateRecord>&
DataRecorder::getDAO<msd_transfer::InertialStateRecord>();

// Ticket: 0039a_energy_tracking_diagnostic_infrastructure
template cpp_sqlite::DataAccessObject<msd_transfer::EnergyRecord>&
DataRecorder::getDAO<msd_transfer::EnergyRecord>();

template cpp_sqlite::DataAccessObject<msd_transfer::SystemEnergyRecord>&
DataRecorder::getDAO<msd_transfer::SystemEnergyRecord>();

// Ticket: 0056a_collision_force_transfer_records
template cpp_sqlite::DataAccessObject<msd_transfer::ContactRecord>&
DataRecorder::getDAO<msd_transfer::ContactRecord>();

template cpp_sqlite::DataAccessObject<msd_transfer::ConstraintForceRecord>&
DataRecorder::getDAO<msd_transfer::ConstraintForceRecord>();

template cpp_sqlite::DataAccessObject<msd_transfer::AppliedForceRecord>&
DataRecorder::getDAO<msd_transfer::AppliedForceRecord>();

template cpp_sqlite::DataAccessObject<msd_transfer::SolverDiagnosticRecord>&
DataRecorder::getDAO<msd_transfer::SolverDiagnosticRecord>();

template cpp_sqlite::DataAccessObject<msd_transfer::BodyMetadataRecord>&
DataRecorder::getDAO<msd_transfer::BodyMetadataRecord>();

template cpp_sqlite::DataAccessObject<msd_transfer::AssetDynamicStateRecord>&
DataRecorder::getDAO<msd_transfer::AssetDynamicStateRecord>();

}  // namespace msd_sim
