
#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "msd-sim/src/DataRecorder/DataRecorder.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp"
#include "msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"
#include "msd-transfer/src/AppliedForceRecord.hpp"
#include "msd-transfer/src/BodyMetadataRecord.hpp"
#include "msd-transfer/src/ConstraintForceRecord.hpp"
#include "msd-transfer/src/ContactRecord.hpp"
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

  // Ticket: 0056b_collision_pipeline_data_extraction
  // Record body metadata if recording is enabled
  if (dataRecorder_)
  {
    recordBodyMetadata(instanceId, assetId, 10.0, 0.5, 0.5, false);
  }

  return inertialAssets_.back();
}

const AssetInertial& WorldModel::spawnObject(uint32_t assetId,
                                             ConvexHull& hull,
                                             double mass,
                                             const ReferenceFrame& origin)
{
  auto instanceId = getInertialAssetId();
  inertialAssets_.emplace_back(assetId, instanceId, hull, mass, origin);

  // Ticket: 0056b_collision_pipeline_data_extraction
  // Record body metadata if recording is enabled
  if (dataRecorder_)
  {
    const auto& asset = inertialAssets_.back();
    recordBodyMetadata(instanceId,
                       assetId,
                       mass,
                       asset.getCoefficientOfRestitution(),
                       asset.getFrictionCoefficient(),
                       false);
  }

  return inertialAssets_.back();
}

const AssetEnvironment& WorldModel::spawnEnvironmentObject(
  uint32_t assetId,
  ConvexHull& hull,
  const ReferenceFrame& origin)
{
  auto instanceId = ++environmentAssetIdCounter_;
  environmentalAssets_.emplace_back(
    assetId, instanceId, hull, origin, 0.5, 0.5);

  // Ticket: 0056b_collision_pipeline_data_extraction
  // Record body metadata if recording is enabled
  if (dataRecorder_)
  {
    const auto& asset = environmentalAssets_.back();
    recordBodyMetadata(instanceId,
                       assetId,
                       0.0,  // Environment assets are static (zero mass)
                       asset.getCoefficientOfRestitution(),
                       asset.getFrictionCoefficient(),
                       true);
  }

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

  // Ticket: 0056b_collision_pipeline_data_extraction
  // Record collision-related data from pipeline snapshot
  const auto& frameData = collisionPipeline_.getLastFrameData();
  recordContacts(frameId, frameData);
  recordConstraintForces(frameId, frameData);
  recordAppliedForces(frameId);
  recordSolverDiagnostics(frameId, frameData);
}

void WorldModel::recordBodyMetadata(uint32_t bodyId,
                                     uint32_t assetId,
                                     double mass,
                                     double restitution,
                                     double friction,
                                     bool isEnvironment)
{
  msd_transfer::BodyMetadataRecord record{};
  record.id = dataRecorder_->getDAO<msd_transfer::BodyMetadataRecord>()
                .incrementIdCounter();
  record.body_id = bodyId;
  record.asset_id = assetId;
  record.mass = mass;
  record.restitution = restitution;
  record.friction = friction;
  record.is_environment = isEnvironment ? 1 : 0;

  dataRecorder_->getDAO<msd_transfer::BodyMetadataRecord>().addToBuffer(record);
}

void WorldModel::recordContacts(
  uint32_t frameId,
  const CollisionPipeline::FrameCollisionData& frameData)
{
  auto& contactDAO = dataRecorder_->getDAO<msd_transfer::ContactRecord>();
  for (const auto& contact : frameData.contacts)
  {
    msd_transfer::ContactRecord record{};
    record.id = contactDAO.incrementIdCounter();
    record.body_a_id = contact.bodyAId;
    record.body_b_id = contact.bodyBId;
    record.contact_index = contact.contactIndex;

    record.point_a_x = contact.pointA.x();
    record.point_a_y = contact.pointA.y();
    record.point_a_z = contact.pointA.z();

    record.point_b_x = contact.pointB.x();
    record.point_b_y = contact.pointB.y();
    record.point_b_z = contact.pointB.z();

    record.normal_x = contact.normal.x();
    record.normal_y = contact.normal.y();
    record.normal_z = contact.normal.z();

    record.depth = contact.depth;
    record.restitution = contact.restitution;
    record.friction = contact.friction;
    record.frame.id = frameId;

    contactDAO.addToBuffer(record);
  }
}

void WorldModel::recordConstraintForces(
  uint32_t frameId,
  const CollisionPipeline::FrameCollisionData& frameData)
{
  auto& forceDAO = dataRecorder_->getDAO<msd_transfer::ConstraintForceRecord>();
  for (const auto& bodyForce : frameData.constraintForces)
  {
    msd_transfer::ConstraintForceRecord record{};
    record.id = forceDAO.incrementIdCounter();
    record.body_id = bodyForce.bodyId;

    record.force_x = bodyForce.linearForce.x();
    record.force_y = bodyForce.linearForce.y();
    record.force_z = bodyForce.linearForce.z();

    record.torque_x = bodyForce.angularTorque.x();
    record.torque_y = bodyForce.angularTorque.y();
    record.torque_z = bodyForce.angularTorque.z();

    record.frame.id = frameId;

    forceDAO.addToBuffer(record);
  }
}

void WorldModel::recordAppliedForces(uint32_t frameId)
{
  auto& forceDAO = dataRecorder_->getDAO<msd_transfer::AppliedForceRecord>();
  for (const auto& asset : inertialAssets_)
  {
    Vector3D gravityForce{0, 0, 0};
    Vector3D gravityTorque{0, 0, 0};

    // Sum forces from all potential energy fields
    for (const auto& potential : potentialEnergies_)
    {
      gravityForce += potential->computeForce(asset.getInertialState(),
                                              asset.getMass());
      gravityTorque += potential->computeTorque(asset.getInertialState(),
                                                asset.getInertiaTensor());
    }

    msd_transfer::AppliedForceRecord record{};
    record.id = forceDAO.incrementIdCounter();
    record.body_id = asset.getInstanceId();
    record.force_type = 0;  // Gravity

    record.force_x = gravityForce.x();
    record.force_y = gravityForce.y();
    record.force_z = gravityForce.z();

    record.torque_x = gravityTorque.x();
    record.torque_y = gravityTorque.y();
    record.torque_z = gravityTorque.z();

    // point_x/y/z remains NaN (gravity acts at center of mass)
    record.frame.id = frameId;

    forceDAO.addToBuffer(record);
  }
}

void WorldModel::recordSolverDiagnostics(
  uint32_t frameId,
  const CollisionPipeline::FrameCollisionData& frameData)
{
  auto& diagDAO =
    dataRecorder_->getDAO<msd_transfer::SolverDiagnosticRecord>();

  msd_transfer::SolverDiagnosticRecord record{};
  record.id = diagDAO.incrementIdCounter();
  record.iterations = static_cast<uint32_t>(frameData.solverData.iterations);
  record.residual = frameData.solverData.residual;
  record.converged = frameData.solverData.converged ? 1 : 0;
  record.num_constraints =
    static_cast<uint32_t>(frameData.solverData.numConstraints);
  record.num_contacts = static_cast<uint32_t>(frameData.solverData.numContacts);
  record.frame.id = frameId;

  diagDAO.addToBuffer(record);
}

}  // namespace msd_sim
