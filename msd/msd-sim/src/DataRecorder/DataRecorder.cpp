// Ticket: 0038_simulation_data_recorder
// Design: docs/designs/0038_simulation_data_recorder/design.md
// Ticket: 0056j_domain_aware_data_recorder

#include <chrono>

#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include "msd-sim/src/DataRecorder/DataRecorder.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/src/Physics/Collision/CollisionPipeline.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-transfer/src/AccelerationRecord.hpp"
#include "msd-transfer/src/AngularAccelerationRecord.hpp"
#include "msd-transfer/src/AssetDynamicStateRecord.hpp"
#include "msd-transfer/src/AssetInertialStaticRecord.hpp"
#include "msd-transfer/src/AssetPhysicalDynamicRecord.hpp"
#include "msd-transfer/src/AssetPhysicalStaticRecord.hpp"
#include "msd-transfer/src/CollisionResultRecord.hpp"
#include "msd-transfer/src/ContactPointRecord.hpp"
#include "msd-transfer/src/CoordinateRecord.hpp"
#include "msd-transfer/src/EnergyRecord.hpp"
#include "msd-transfer/src/ExternalForceRecord.hpp"
#include "msd-transfer/src/ForceVectorRecord.hpp"
#include "msd-transfer/src/InertialStateRecord.hpp"
#include "msd-transfer/src/QuaternionDRecord.hpp"
#include "msd-transfer/src/SimulationFrameRecord.hpp"
#include "msd-transfer/src/SolverDiagnosticRecord.hpp"
#include "msd-transfer/src/SystemEnergyRecord.hpp"
#include "msd-transfer/src/TorqueVectorRecord.hpp"
#include "msd-transfer/src/Vector3DRecord.hpp"
#include "msd-transfer/src/Vector4DRecord.hpp"
#include "msd-transfer/src/VelocityRecord.hpp"

namespace msd_sim
{

DataRecorder::DataRecorder(const Config& config)
  : flushInterval_{config.flushInterval}
{
  // Open database
  database_ = std::make_unique<cpp_sqlite::Database>(config.databasePath, true);

  // Pre-create ALL DAOs before starting the recorder thread. This is critical
  // because Database::insert() for types with nested ValidTransferObject members
  // (e.g., CollisionResultRecord → Vector3DRecord, ContactPointRecord →
  // CoordinateRecord) lazily calls getDAO<NestedType>() during flush, which
  // modifies daoCreationOrder_ while flushAllDAOs() iterates it — causing
  // iterator invalidation and a segfault. Pre-creating ensures the vector is
  // stable before the thread starts iterating it.

  // Frame record (must be first for FK integrity)
  database_->getDAO<msd_transfer::SimulationFrameRecord>();

  // Leaf sub-record types (nested ValidTransferObjects within top-level records)
  database_->getDAO<msd_transfer::CoordinateRecord>();
  database_->getDAO<msd_transfer::VelocityRecord>();
  database_->getDAO<msd_transfer::AccelerationRecord>();
  database_->getDAO<msd_transfer::QuaternionDRecord>();
  database_->getDAO<msd_transfer::Vector4DRecord>();
  database_->getDAO<msd_transfer::AngularAccelerationRecord>();
  database_->getDAO<msd_transfer::Vector3DRecord>();
  database_->getDAO<msd_transfer::ContactPointRecord>();
  database_->getDAO<msd_transfer::ExternalForceRecord>();
  database_->getDAO<msd_transfer::ForceVectorRecord>();
  database_->getDAO<msd_transfer::TorqueVectorRecord>();

  // Top-level record types used by domain-aware recording methods
  database_->getDAO<msd_transfer::InertialStateRecord>();
  database_->getDAO<msd_transfer::EnergyRecord>();
  database_->getDAO<msd_transfer::SystemEnergyRecord>();
  database_->getDAO<msd_transfer::CollisionResultRecord>();
  database_->getDAO<msd_transfer::SolverDiagnosticRecord>();
  database_->getDAO<msd_transfer::AssetInertialStaticRecord>();
  database_->getDAO<msd_transfer::AssetPhysicalStaticRecord>();
  database_->getDAO<msd_transfer::AssetPhysicalDynamicRecord>();
  database_->getDAO<msd_transfer::AssetDynamicStateRecord>();

  // Start recorder thread (daoCreationOrder_ is now fully populated)
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
template cpp_sqlite::DataAccessObject<msd_transfer::CollisionResultRecord>&
DataRecorder::getDAO<msd_transfer::CollisionResultRecord>();

template cpp_sqlite::DataAccessObject<msd_transfer::SolverDiagnosticRecord>&
DataRecorder::getDAO<msd_transfer::SolverDiagnosticRecord>();

template cpp_sqlite::DataAccessObject<msd_transfer::AssetPhysicalStaticRecord>&
DataRecorder::getDAO<msd_transfer::AssetPhysicalStaticRecord>();

template cpp_sqlite::DataAccessObject<msd_transfer::AssetPhysicalDynamicRecord>&
DataRecorder::getDAO<msd_transfer::AssetPhysicalDynamicRecord>();

template cpp_sqlite::DataAccessObject<msd_transfer::AssetInertialStaticRecord>&
DataRecorder::getDAO<msd_transfer::AssetInertialStaticRecord>();

template cpp_sqlite::DataAccessObject<msd_transfer::AssetDynamicStateRecord>&
DataRecorder::getDAO<msd_transfer::AssetDynamicStateRecord>();

// ========== Domain-Aware Recording Methods ==========
// Ticket: 0056j_domain_aware_data_recorder

void DataRecorder::recordInertialStates(uint32_t frameId,
                                        std::span<const AssetInertial> assets)
{
  auto& dynamicDAO = getDAO<msd_transfer::AssetDynamicStateRecord>();
  for (const auto& asset : assets)
  {
    auto record = asset.toDynamicStateRecord();
    record.frame.id = frameId;
    dynamicDAO.addToBuffer(record);
  }
}

void DataRecorder::recordBodyEnergies(
  uint32_t frameId,
  std::span<const AssetInertial> assets,
  std::span<const std::unique_ptr<PotentialEnergy>> potentials)
{
  auto& energyDAO = getDAO<msd_transfer::EnergyRecord>();
  for (const auto& asset : assets)
  {
    auto bodyEnergy = EnergyTracker::computeBodyEnergy(
      asset.getInertialState(), asset.getMass(), asset.getInertiaTensor(),
      potentials);
    auto energyRecord = bodyEnergy.toRecord(frameId, asset.getInstanceId());
    energyDAO.addToBuffer(energyRecord);
  }
}

void DataRecorder::recordSystemEnergy(
  uint32_t frameId,
  const EnergyTracker::SystemEnergy& energy,
  double previousTotal,
  bool collisionActive)
{
  auto& sysEnergyDAO = getDAO<msd_transfer::SystemEnergyRecord>();
  auto sysRecord = energy.toRecord(frameId, previousTotal, collisionActive);
  sysEnergyDAO.addToBuffer(sysRecord);
}

void DataRecorder::recordCollisions(uint32_t frameId,
                                    const CollisionPipeline& pipeline)
{
  auto& collisionDAO = getDAO<msd_transfer::CollisionResultRecord>();
  for (const auto& pair : pipeline.getCollisions())
  {
    auto record = pair.result.toRecord(pair.bodyAId, pair.bodyBId);
    record.frame.id = frameId;
    collisionDAO.addToBuffer(record);
  }
}

void DataRecorder::recordSolverDiagnostics(uint32_t frameId,
                                           const CollisionPipeline& pipeline)
{
  const auto& solver = pipeline.getSolverData();
  auto& diagDAO = getDAO<msd_transfer::SolverDiagnosticRecord>();

  msd_transfer::SolverDiagnosticRecord record{};
  record.iterations = static_cast<uint32_t>(solver.iterations);
  record.residual = solver.residual;
  record.converged = solver.converged ? 1 : 0;
  record.num_constraints = static_cast<uint32_t>(solver.numConstraints);
  record.num_contacts = static_cast<uint32_t>(solver.numContacts);
  record.frame.id = frameId;

  diagDAO.addToBuffer(record);
}

void DataRecorder::recordStaticAsset(const AssetInertial& asset)
{
  // Record inertial properties (mass, restitution, friction)
  auto& staticDAO = getDAO<msd_transfer::AssetInertialStaticRecord>();

  msd_transfer::AssetInertialStaticRecord record{};
  record.body_id = asset.getInstanceId();
  record.mass = asset.getMass();
  record.restitution = asset.getCoefficientOfRestitution();
  record.friction = asset.getFrictionCoefficient();
  staticDAO.addToBuffer(record);

  // Record physical properties (asset_id for geometry lookup)
  // Ticket: 0056e_threejs_core_visualization (R0a)
  auto& physicalDAO = getDAO<msd_transfer::AssetPhysicalStaticRecord>();
  // Call base class method explicitly
  const AssetPhysical& physicalBase = asset;
  auto physicalRecord = physicalBase.toStaticRecord(false);  // false = not environment
  physicalDAO.addToBuffer(physicalRecord);
}

}  // namespace msd_sim
