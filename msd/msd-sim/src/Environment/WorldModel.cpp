
#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "msd-sim/src/DataRecorder/DataRecorder.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp"
#include "msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"
#include "msd-transfer/src/AssetInertialStaticRecord.hpp"
#include "msd-transfer/src/CollisionResultRecord.hpp"
#include "msd-transfer/src/EnergyRecord.hpp"
#include "msd-transfer/src/InertialStateRecord.hpp"
#include "msd-transfer/src/SolverDiagnosticRecord.hpp"
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
  inertialAssets_.emplace_back(
    assetId, instanceId, hull, 10.0, origin, 0.5, 0.5);

  const AssetInertial& asset = inertialAssets_.back();

  // Ticket: 0056j_domain_aware_data_recorder
  // Record static data if recording is enabled
  if (dataRecorder_)
  {
    dataRecorder_->recordStaticAsset(asset);
  }

  return asset;
}

const AssetInertial& WorldModel::spawnObject(uint32_t assetId,
                                             ConvexHull& hull,
                                             double mass,
                                             const ReferenceFrame& origin)
{
  auto instanceId = getInertialAssetId();
  inertialAssets_.emplace_back(assetId, instanceId, hull, mass, origin);

  const AssetInertial& asset = inertialAssets_.back();

  // Ticket: 0056j_domain_aware_data_recorder
  // Record static data if recording is enabled
  if (dataRecorder_)
  {
    dataRecorder_->recordStaticAsset(asset);
  }

  return asset;
}

const AssetEnvironment& WorldModel::spawnEnvironmentObject(
  uint32_t assetId,
  ConvexHull& hull,
  const ReferenceFrame& origin)
{
  auto instanceId = ++environmentAssetIdCounter_;
  environmentalAssets_.emplace_back(
    assetId, instanceId, hull, origin, 0.5, 0.5);

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

  // Ticket: 0047a_revert_gravity_preapply
  //
  // Gravity pre-apply REMOVED (was: ticket
  // 0047_face_contact_manifold_generation).
  //
  // Investigation goal: Characterize the resting contact problem without
  // velocity mutation, then implement a cleaner approach that doesn't couple
  // restitution with gravity.
  //
  // With this revert, the constraint solver sees v (not v+g*dt), and gravity
  // is applied in updatePhysics() along with all other forces.

  // Contact constraint forces BEFORE physics integration
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
    // Ticket: 0047a_revert_gravity_preapply
    //
    // Gravity application RESTORED to updatePhysics (was: split in ticket
    // 0047).
    //
    // All forces (gravity + contact + external) now integrated in a single
    // pass. No velocity mutation before collision solving — the constraint
    // solver sees the actual velocity v, not an augmented v+g*dt.
    Coordinate netForce = asset.getAccumulatedForce();
    Coordinate netTorque{0.0, 0.0, 0.0};

    InertialState& state = asset.getInertialState();
    const Eigen::Matrix3d& inertiaTensor = asset.getInertiaTensor();
    double const mass = asset.getMass();

    // Apply ALL potential energy forces and torques
    for (const auto& potential : potentialEnergies_)
    {
      netForce += potential->computeForce(state, mass);
      netTorque += potential->computeTorque(state, inertiaTensor);
    }

    netTorque += asset.getAccumulatedTorque();

    // ===== Step 2: Delegate Integration to Integrator =====
    // Integrator handles: velocity update, position update, quaternion
    // normalization
    // Ticket 0045: Removed constraint gathering - quaternion normalization
    // now handled via state.orientation.normalize()
    integrator_->step(state,
                      netForce,
                      netTorque,
                      mass,
                      asset.getInverseInertiaTensorWorld(),
                      dt);

    // ===== Step 3: Synchronize ReferenceFrame =====
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

  // Ticket: 0056j_domain_aware_data_recorder
  // Backfill static data for already-spawned assets
  for (const auto& asset : inertialAssets_)
  {
    dataRecorder_->recordStaticAsset(asset);
  }
}

void WorldModel::disableRecording()
{
  dataRecorder_.reset();  // Triggers flush and thread join via RAII
}

void WorldModel::recordCurrentFrame()
{
  // Ticket: 0056j_domain_aware_data_recorder
  // Thin orchestrator — delegates to DataRecorder domain-aware methods

  // Get pre-assigned frame ID from recorder
  double const simulationTime = std::chrono::duration<double>(time_).count();
  const uint32_t frameId = dataRecorder_->recordFrame(simulationTime);

  // Delegate recording to DataRecorder
  dataRecorder_->recordInertialStates(frameId, inertialAssets_);
  dataRecorder_->recordBodyEnergies(frameId, inertialAssets_,
                                    potentialEnergies_);

  // Compute system energy (WorldModel state tracking)
  auto systemEnergy =
    EnergyTracker::computeSystemEnergy(inertialAssets_, potentialEnergies_);
  dataRecorder_->recordSystemEnergy(frameId, systemEnergy,
                                    previousSystemEnergy_,
                                    collisionActiveThisFrame_);
  previousSystemEnergy_ = systemEnergy.total();

  // Record collision results and solver diagnostics
  dataRecorder_->recordCollisions(frameId, collisionPipeline_);
  dataRecorder_->recordSolverDiagnostics(frameId, collisionPipeline_);

  // Record constraint states (tangents, forces, geometry)
  // Ticket: 0057_contact_tangent_recording
  collisionPipeline_.recordConstraints(*dataRecorder_, frameId);
}

}  // namespace msd_sim
