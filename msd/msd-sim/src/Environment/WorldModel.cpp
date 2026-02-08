
#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "msd-sim/src/DataRecorder/DataRecorder.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp"
#include "msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"
#include "msd-transfer/src/EnergyRecord.hpp"
#include "msd-transfer/src/InertialStateRecord.hpp"
#include "msd-transfer/src/SystemEnergyRecord.hpp"

namespace msd_sim
{

// Constructor initializes default integrator and gravity potential
WorldModel::WorldModel()
{
  // Initialize default integrator (semi-implicit Euler)
  integrator_ = std::make_unique<SemiImplicitEulerIntegrator>();

  // Initialize default gravity potential
  potentialEnergies_.push_back(
    std::make_unique<GravityPotential>(Coordinate{0.0, 0.0, -9.81}));
  // Ticket: 0030_lagrangian_quaternion_physics
}

// Destructor defined in .cpp where DataRecorder type is complete
// Required for std::unique_ptr<DataRecorder> forward declaration pattern
WorldModel::~WorldModel() = default;


// ========== Object Management ==========

const AssetInertial& WorldModel::spawnObject(uint32_t assetId,
                                             ConvexHull& hull,
                                             const ReferenceFrame& origin)
{
  auto instanceId = getInertialAssetId();
  inertialAssets_.emplace_back(assetId, instanceId, hull, 10.0, origin);
  return inertialAssets_.back();
}

const AssetInertial& WorldModel::spawnObject(uint32_t assetId,
                                             ConvexHull& hull,
                                             double mass,
                                             const ReferenceFrame& origin)
{
  auto instanceId = getInertialAssetId();
  inertialAssets_.emplace_back(assetId, instanceId, hull, mass, origin);
  return inertialAssets_.back();
}

const AssetEnvironment& WorldModel::spawnEnvironmentObject(
  uint32_t assetId,
  ConvexHull& hull,
  const ReferenceFrame& origin)
{
  auto instanceId = ++environmentAssetIdCounter_;
  environmentalAssets_.emplace_back(assetId, instanceId, hull, origin);
  return environmentalAssets_.back();
}

const AssetInertial& WorldModel::getObject(uint32_t instanceId) const
{
  auto it =
    std::ranges::find_if(inertialAssets_,

                         [instanceId](const AssetInertial& asset)
                         { return asset.getInstanceId() == instanceId; });
  if (it == inertialAssets_.end())
  {
    throw std::out_of_range("Object with instanceId not found: " +
                            std::to_string(instanceId));
  }
  return *it;
}

AssetInertial& WorldModel::getObject(uint32_t instanceId)
{
  auto it =
    std::ranges::find_if(inertialAssets_,

                         [instanceId](const AssetInertial& asset)
                         { return asset.getInstanceId() == instanceId; });
  if (it == inertialAssets_.end())
  {
    throw std::out_of_range("Object with instanceId not found: " +
                            std::to_string(instanceId));
  }
  return *it;
}

void WorldModel::clearObjects()
{
  inertialAssets_.clear();
}

// ========== Platform Management (Legacy) ==========

void WorldModel::addPlatform(Platform&& platform)
{
  platforms_.push_back(std::move(platform));
}

// ========== Simulation Update ==========

void WorldModel::update(std::chrono::milliseconds simTime)
{
  // Convert to seconds for physics calculations
  double const dt =
    static_cast<double>(simTime.count() - time_.count()) / 1000.0;

  // Update all platforms (agent logic + visual sync)
  for (auto& platform : platforms_)
  {
    platform.update(simTime);
  }

  // IMPORTANT: Contact constraint forces BEFORE physics integration
  // Order matters: constraint forces must be accumulated before velocity
  // integration so they are included in the force-based integration step
  // Ticket: 0032_contact_constraint_refactor
  updateCollisions(dt);

  // Update physics for all dynamic objects
  updatePhysics(dt);

  // Update simulation time
  time_ = simTime;

  // Record current frame if recording is enabled
  // Ticket: 0038_simulation_data_recorder
  if (dataRecorder_)
  {
    recordCurrentFrame();
  }
}

// ========== Private Methods ==========

void WorldModel::updatePhysics(double dt)
{
  for (auto& asset : inertialAssets_)
  {
    // ===== Step 1: Compute Generalized Forces from Potential Energies =====
    Coordinate netForce{0.0, 0.0, 0.0};
    Coordinate netTorque{0.0, 0.0, 0.0};

    InertialState& state = asset.getInertialState();
    double const mass = asset.getMass();
    const Eigen::Matrix3d& inertiaTensor = asset.getInertiaTensor();

    for (const auto& potential : potentialEnergies_)
    {
      netForce += potential->computeForce(state, mass);
      netTorque += potential->computeTorque(state, inertiaTensor);
    }

    // Add accumulated forces (from collisions, thrusters, etc.)
    netForce += asset.getAccumulatedForce();
    netTorque += asset.getAccumulatedTorque();

    // ===== Step 2: Gather Constraints =====
    // Get all constraints attached to this asset
    std::vector<Constraint*> const constraints = asset.getConstraints();

    // ===== Step 3: Delegate Integration to Integrator =====
    // Integrator handles: velocity update, position update, constraint
    // enforcement
    integrator_->step(state,
                      netForce,
                      netTorque,
                      mass,
                      asset.getInverseInertiaTensorWorld(),
                      constraints,
                      dt);

    // ===== Step 4: Synchronize ReferenceFrame =====
    // ReferenceFrame must match InertialState for collision detection and
    // rendering
    ReferenceFrame& frame = asset.getReferenceFrame();
    frame.setOrigin(state.position);
    frame.setQuaternion(state.orientation);

    // ===== Step 5: Clear Forces for Next Frame =====
    asset.clearForces();
  }
  // Ticket: 0030_lagrangian_quaternion_physics
}

void WorldModel::updateCollisions(double dt)
{
  // Ticket: 0044_collision_pipeline_integration
  // Constraint-based collision response using Active Set Method (ASM)
  //
  // Phase 0: Cache lifecycle management
  collisionPipeline_.advanceFrame();
  collisionPipeline_.expireOldEntries();  // Default maxAge = 10

  // Phase 1-6: Full collision response pipeline
  collisionPipeline_.execute(inertialAssets_, environmentalAssets_, dt);

  // Update energy tracking flag
  collisionActiveThisFrame_ = collisionPipeline_.hadCollisions();
}

uint32_t WorldModel::getInertialAssetId()
{
  return ++inertialAssetIdCounter_;
}

// ========== Potential Energy Configuration (ticket 0030) ==========

void WorldModel::addPotentialEnergy(std::unique_ptr<PotentialEnergy> energy)
{
  potentialEnergies_.push_back(std::move(energy));
}

void WorldModel::clearPotentialEnergies()
{
  potentialEnergies_.clear();
}

// ========== Integrator Configuration (ticket 0030) ==========

void WorldModel::setIntegrator(std::unique_ptr<Integrator> integrator)
{
  integrator_ = std::move(integrator);
}

// ========== Gravity Configuration (deprecated, ticket 0023a) ==========

const Coordinate& WorldModel::getGravity() const
{
  return gravity_;
}

// ========== Data Recording (ticket 0038) ==========

void WorldModel::enableRecording(const std::string& dbPath,
                                 std::chrono::milliseconds flushInterval)
{
  DataRecorder::Config config{};
  config.databasePath = dbPath;
  config.flushInterval = flushInterval;
  dataRecorder_ = std::make_unique<DataRecorder>(config);
}

void WorldModel::disableRecording()
{
  dataRecorder_.reset();  // Triggers flush and thread join via RAII
}

void WorldModel::recordCurrentFrame()
{
  // Get pre-assigned frame ID from recorder
  double const simulationTime = std::chrono::duration<double>(time_).count();
  const uint32_t frameId = dataRecorder_->recordFrame(simulationTime);

  // Record all inertial states with FK to this frame
  auto& stateDAO = dataRecorder_->getDAO<msd_transfer::InertialStateRecord>();
  for (const auto& asset : inertialAssets_)
  {
    auto record = asset.getInertialState().toRecord();
    record.frame.id = frameId;  // Explicit FK assignment
    stateDAO.addToBuffer(record);
  }

  // Ticket: 0039a_energy_tracking_diagnostic_infrastructure
  // Compute and record per-body energy
  auto& energyDAO = dataRecorder_->getDAO<msd_transfer::EnergyRecord>();
  for (const auto& asset : inertialAssets_)
  {
    auto bodyEnergy = EnergyTracker::computeBodyEnergy(asset.getInertialState(),
                                                       asset.getMass(),
                                                       asset.getInertiaTensor(),
                                                       potentialEnergies_);
    auto energyRecord = bodyEnergy.toRecord(frameId, asset.getInstanceId());
    energyDAO.addToBuffer(energyRecord);
  }

  // Compute and record system energy summary
  auto systemEnergy =
    EnergyTracker::computeSystemEnergy(inertialAssets_, potentialEnergies_);
  auto& sysEnergyDAO =
    dataRecorder_->getDAO<msd_transfer::SystemEnergyRecord>();
  auto sysRecord = systemEnergy.toRecord(
    frameId, previousSystemEnergy_, collisionActiveThisFrame_);
  sysEnergyDAO.addToBuffer(sysRecord);
  previousSystemEnergy_ = systemEnergy.total();
}

}  // namespace msd_sim
