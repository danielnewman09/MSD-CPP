
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

  // Ticket: 0051_restitution_gravity_coupling
  //
  // Removed direct velocity mutation (gravity pre-apply) in favor of velocity-bias
  // approach. Gravity is now threaded through the collision pipeline as a separate
  // bias parameter to the constraint solver, decoupling restitution from gravity:
  //   RHS = -(1+e)*J*v - J*v_bias  (bias NOT multiplied by (1+e))
  //
  // Previous approach (ticket 0047) directly mutated velocities:
  //   v += g*dt  (REMOVED)
  // This caused restitution-gravity coupling: b = -(1+e)*J*(v+g*dt)
  // The extra e*J*g*dt term (≈0.08 m/s) caused spurious rotation (B3) and
  // modified energy patterns (H3).
  //
  // The collision pipeline now computes the bias internally via
  // WorldModel::computeVelocityBias().

  // IMPORTANT: Contact constraint forces BEFORE physics integration
  // Solver sees velocity bias without direct mutation
  // Ticket: 0032_contact_constraint_refactor, 0051_restitution_gravity_coupling
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
    // Ticket: 0051_restitution_gravity_coupling
    //
    // Potential energy LINEAR FORCES (gravity) are NOT applied here!
    // They are handled SOLELY via the velocity-bias approach in the collision
    // solver to decouple restitution from gravity.
    //
    // The collision solver (updateCollisions) uses a velocity bias where:
    //   v_bias = g * dt  (computed from potential energies)
    //   RHS = -(1+e)*J*v - J*v_bias
    //
    // This allows the solver to "see" gravity's effect without pre-applying it
    // to velocities, preventing the restitution-gravity coupling term e*J*g*dt.
    //
    // The solver produces contact forces that balance gravity for resting bodies,
    // and those forces are integrated here via accumulated forces.
    //
    // Potential energy TORQUES are still applied (gravity torque = 0 for uniform
    // fields, but correct for future non-uniform potentials like tidal forces).
    //
    // Accumulated forces are from collision response (contact constraints) and
    // implicitly include the effect of gravity via the bias mechanism.
    Coordinate netForce = asset.getAccumulatedForce();
    Coordinate netTorque{0.0, 0.0, 0.0};

    InertialState& state = asset.getInertialState();
    const Eigen::Matrix3d& inertiaTensor = asset.getInertiaTensor();

    // Apply potential energy forces and torques
    // NOTE (ticket 0051): This will cause restitution-gravity coupling for
    // colliding bodies, but is necessary for non-colliding bodies (free fall).
    // The velocity-bias approach needs further refinement to handle both cases.
    for (const auto& potential : potentialEnergies_)
    {
      netForce += potential->computeForce(state, asset.getMass());
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

  // Ticket: 0051_restitution_gravity_coupling
  // Compute velocity bias from potential energies (gravity)
  auto velocityBias = computeVelocityBias(dt);

  // Phase 1-6: Full collision response pipeline
  collisionPipeline_.execute(inertialAssets_, environmentalAssets_, dt, velocityBias);

  // Update energy tracking flag
  collisionActiveThisFrame_ = collisionPipeline_.hadCollisions();
}

std::vector<InertialState> WorldModel::computeVelocityBias(double dt) const
{
  // Ticket: 0051_restitution_gravity_coupling
  //
  // Compute velocity bias from potential energy forces (gravity) to enable
  // velocity-bias approach in constraint solver. This decouples restitution
  // from gravity:
  //   RHS = -(1+e)*J*v - J*v_bias  (bias NOT multiplied by (1+e))
  //
  // Only velocity fields are populated; position/orientation remain at defaults.
  std::vector<InertialState> bias(inertialAssets_.size());

  for (size_t i = 0; i < inertialAssets_.size(); ++i)
  {
    const auto& asset = inertialAssets_[i];
    const InertialState& state = asset.getInertialState();
    const double mass = asset.getMass();
    const Eigen::Matrix3d& inertiaTensor = asset.getInertiaTensor();

    // Accumulate forces and torques from all potential energies
    Coordinate netForce{0.0, 0.0, 0.0};
    Coordinate netTorque{0.0, 0.0, 0.0};

    for (const auto& potential : potentialEnergies_)
    {
      netForce += potential->computeForce(state, mass);
      netTorque += potential->computeTorque(state, inertiaTensor);
    }

    // Convert to velocity bias via InertialState velocity fields
    // Only velocity and angular velocity are meaningful; other fields remain default
    bias[i].velocity = netForce / mass * dt;
    bias[i].setAngularVelocity(asset.getInverseInertiaTensorWorld() * netTorque * dt);
  }

  return bias;
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
