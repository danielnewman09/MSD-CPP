
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

  // Ticket: 0047_face_contact_manifold_generation
  //
  // Pre-apply external forces (gravity) to velocities before collision solving.
  // This is the standard Box2D/Bullet approach: the constraint solver sees
  // gravity-augmented velocities and can produce non-zero support force even
  // when bodies are at rest (v≈0 → v_temp = g*dt → solver counteracts it).
  //
  // Without this, a resting cube with v=0 produces RHS b = -(1+e)*J*v = 0
  // → λ = 0 → no support force → micro-bounce oscillation every 2 frames.
  //
  // Note: This couples gravity with restitution in the solver's RHS:
  //   b = -(1+e) * J * (v + g*dt)
  // The extra e*J*g*dt term is bounded (≈ e*9.81*0.016 ≈ 0.08 m/s) and
  // acceptable for the simulation fidelity required.
  for (auto& asset : inertialAssets_)
  {
    InertialState& state = asset.getInertialState();
    double const mass = asset.getMass();

    for (const auto& potential : potentialEnergies_)
    {
      Coordinate const force = potential->computeForce(state, mass);
      state.velocity += force / mass * dt;
    }
  }

  // IMPORTANT: Contact constraint forces BEFORE physics integration
  // Solver sees gravity-augmented velocities from the pre-apply above
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
    // Ticket: 0047_face_contact_manifold_generation
    //
    // Potential energy FORCES (gravity) already applied to velocities in
    // update() before collision solving. Only accumulated forces (from
    // collisions, thrusters, etc.) remain to be integrated here.
    //
    // Potential energy TORQUES are still applied here since they were NOT
    // pre-applied (gravity torque = 0 for uniform fields, so this is a
    // no-op currently, but correct for future non-uniform potentials).
    Coordinate netForce = asset.getAccumulatedForce();
    Coordinate netTorque{0.0, 0.0, 0.0};

    InertialState& state = asset.getInertialState();
    const Eigen::Matrix3d& inertiaTensor = asset.getInertiaTensor();

    for (const auto& potential : potentialEnergies_)
    {
      netTorque += potential->computeTorque(state, inertiaTensor);
    }

    netTorque += asset.getAccumulatedTorque();
    double const mass = asset.getMass();

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
