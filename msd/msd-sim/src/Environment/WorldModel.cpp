
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
  // Constraint-based collision response using Active Set Method (ASM)
  //
  // Solver body indexing:
  //   [0 .. numInertial-1]                   = inertial bodies
  //   [numInertial .. numInertial+numEnv-1]   = environment bodies (infinite
  //   mass)
  //
  // Ticket: 0032_contact_constraint_refactor
  // Ticket: 0040d_contact_persistence_warm_starting

  // Ticket: 0039a_energy_tracking_diagnostic_infrastructure
  collisionActiveThisFrame_ = false;

  const size_t numInertial = inertialAssets_.size();
  const size_t numEnvironment = environmentalAssets_.size();
  const size_t numBodies = numInertial + numEnvironment;

  // Ticket 0040d: Advance cache age and expire stale entries each frame
  contactCache_.advanceFrame();
  contactCache_.expireOldEntries();

  if (numBodies == 0 || dt <= 0.0)
  {
    return;
  }

  // ===== Phase 1: Collision Detection =====
  // Collect all collision pairs with their body indices

  struct CollisionPair
  {
    size_t bodyAIndex;  // Solver body index
    size_t bodyBIndex;  // Solver body index
    uint32_t bodyAId;   // Unique body ID for cache keying
    uint32_t bodyBId;   // Unique body ID for cache keying
    CollisionResult result;
    double restitution;
  };

  // Ticket 0040d: Helper to encode unique body IDs.
  // Inertial instance IDs are used directly.
  // Environment IDs get high bit set to avoid collision with inertial IDs.
  constexpr uint32_t kEnvIdFlag = 0x80000000u;

  std::vector<CollisionPair> collisions;

  // Inertial vs Inertial (O(n²) pairwise)
  for (size_t i = 0; i < numInertial; ++i)
  {
    for (size_t j = i + 1; j < numInertial; ++j)
    {
      auto result = collisionHandler_.checkCollision(inertialAssets_[i],
                                                     inertialAssets_[j]);
      if (!result)
      {
        continue;
      }

      double const combinedE = contact_constraint_factory::combineRestitution(
        inertialAssets_[i].getCoefficientOfRestitution(),
        inertialAssets_[j].getCoefficientOfRestitution());

      collisions.push_back({i,
                            j,
                            inertialAssets_[i].getInstanceId(),
                            inertialAssets_[j].getInstanceId(),
                            *result,
                            combinedE});
    }
  }

  // Inertial vs Environment
  for (size_t i = 0; i < numInertial; ++i)
  {
    for (size_t e = 0; e < numEnvironment; ++e)
    {
      auto result = collisionHandler_.checkCollision(inertialAssets_[i],
                                                     environmentalAssets_[e]);
      if (!result)
      {
        continue;
      }

      double const combinedE = contact_constraint_factory::combineRestitution(
        inertialAssets_[i].getCoefficientOfRestitution(),
        environmentalAssets_[e].getCoefficientOfRestitution());

      // Environment body index offset by numInertial
      collisions.push_back(
        {i,
         numInertial + e,
         inertialAssets_[i].getInstanceId(),
         environmentalAssets_[e].getInstanceId() | kEnvIdFlag,
         *result,
         combinedE});
    }
  }

  if (collisions.empty())
  {
    return;
  }

  // Ticket: 0039a_energy_tracking_diagnostic_infrastructure
  collisionActiveThisFrame_ = true;

  // ===== Phase 2: Create Contact Constraints =====
  // Track constraint range per collision pair for cache mapping
  struct PairConstraintRange
  {
    size_t startIdx;
    size_t count;
    size_t pairIdx;
  };
  std::vector<PairConstraintRange> pairRanges;
  std::vector<std::unique_ptr<ContactConstraint>> allConstraints;

  for (size_t p = 0; p < collisions.size(); ++p)
  {
    const auto& pair = collisions[p];
    size_t const rangeStart = allConstraints.size();

    const InertialState& stateA =
      (pair.bodyAIndex < numInertial)
        ? inertialAssets_[pair.bodyAIndex].getInertialState()
        : environmentalAssets_[pair.bodyAIndex - numInertial]
            .getInertialState();

    const InertialState& stateB =
      (pair.bodyBIndex < numInertial)
        ? inertialAssets_[pair.bodyBIndex].getInertialState()
        : environmentalAssets_[pair.bodyBIndex - numInertial]
            .getInertialState();

    const Coordinate& comA = stateA.position;
    const Coordinate& comB = stateB.position;

    auto constraints =
      contact_constraint_factory::createFromCollision(pair.bodyAIndex,
                                                      pair.bodyBIndex,
                                                      pair.result,
                                                      stateA,
                                                      stateB,
                                                      comA,
                                                      comB,
                                                      pair.restitution);

    for (auto& c : constraints)
    {
      allConstraints.push_back(std::move(c));
    }

    pairRanges.push_back({rangeStart, allConstraints.size() - rangeStart, p});
  }

  if (allConstraints.empty())
  {
    return;
  }

  // ===== Phase 3: Build Solver Input Arrays =====
  std::vector<std::reference_wrapper<const InertialState>> states;
  std::vector<double> inverseMasses;
  std::vector<Eigen::Matrix3d> inverseInertias;

  states.reserve(numBodies);
  inverseMasses.reserve(numBodies);
  inverseInertias.reserve(numBodies);

  for (auto& asset : inertialAssets_)
  {
    states.push_back(std::cref(asset.getInertialState()));
    inverseMasses.push_back(asset.getInverseMass());
    inverseInertias.push_back(asset.getInverseInertiaTensorWorld());
  }

  for (auto& envAsset : environmentalAssets_)
  {
    states.push_back(std::cref(envAsset.getInertialState()));
    inverseMasses.push_back(
      msd_sim::AssetEnvironment::getInverseMass());  // 0.0
    inverseInertias.push_back(
      msd_sim::AssetEnvironment::getInverseInertiaTensor());  // Zero matrix
  }

  // Build non-owning TwoBodyConstraint* vector for solver
  std::vector<Constraint*> constraintPtrs;
  constraintPtrs.reserve(allConstraints.size());
  for (auto& c : allConstraints)
  {
    constraintPtrs.push_back(c.get());
  }

  // ===== Phase 3.5: Warm-Start from Contact Cache (Ticket 0040d) =====
  const size_t totalConstraints = allConstraints.size();
  Eigen::VectorXd initialLambda =
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(totalConstraints));

  // Helper: extract contact world-space points for a collision pair's
  // constraint range.  Contact point ≈ comA + leverArmA.
  auto extractContactPoints =
    [&](const PairConstraintRange& range,
        const CollisionPair& pair) -> std::vector<Coordinate>
  {
    std::vector<Coordinate> points;
    points.reserve(range.count);
    for (size_t ci = 0; ci < range.count; ++ci)
    {
      const auto& contact = allConstraints[range.startIdx + ci];
      const InertialState& stateA =
        (pair.bodyAIndex < numInertial)
          ? inertialAssets_[pair.bodyAIndex].getInertialState()
          : environmentalAssets_[pair.bodyAIndex - numInertial]
              .getInertialState();
      points.push_back(Coordinate{stateA.position + contact->getLeverArmA()});
    }
    return points;
  };

  for (const auto& range : pairRanges)
  {
    const auto& pair = collisions[range.pairIdx];

    auto currentPoints = extractContactPoints(range, pair);

    // Look up cache
    Vector3D const normalVec{
      pair.result.normal.x(), pair.result.normal.y(), pair.result.normal.z()};
    auto cachedLambdas = contactCache_.getWarmStart(
      pair.bodyAId, pair.bodyBId, normalVec, currentPoints);

    if (!cachedLambdas.empty() && cachedLambdas.size() == range.count)
    {
      for (size_t ci = 0; ci < range.count; ++ci)
      {
        initialLambda(static_cast<Eigen::Index>(range.startIdx + ci)) =
          cachedLambdas[ci];
      }
    }
  }

  // ===== Phase 4: Solve with warm-starting =====
  auto solveResult = contactSolver_.solveWithContacts(constraintPtrs,
                                                      states,
                                                      inverseMasses,
                                                      inverseInertias,
                                                      numBodies,
                                                      dt,
                                                      initialLambda);

  // ===== Phase 4.5: Update Contact Cache (Ticket 0040d) =====
  for (const auto& range : pairRanges)
  {
    const auto& pair = collisions[range.pairIdx];

    std::vector<double> solvedLambdas;
    solvedLambdas.reserve(range.count);
    for (size_t ci = 0; ci < range.count; ++ci)
    {
      solvedLambdas.push_back(
        solveResult.lambdas(static_cast<Eigen::Index>(range.startIdx + ci)));
    }

    auto contactPoints = extractContactPoints(range, pair);

    Vector3D const normalVec{
      pair.result.normal.x(), pair.result.normal.y(), pair.result.normal.z()};
    contactCache_.update(
      pair.bodyAId, pair.bodyBId, normalVec, solvedLambdas, contactPoints);
  }

  // ===== Phase 5: Apply Constraint Forces to Inertial Bodies =====
  // Environment bodies (indices >= numInertial) are skipped because they
  // have infinite mass and cannot be moved.
  for (size_t k = 0; k < numInertial; ++k)
  {
    const auto& forces = solveResult.bodyForces[k];

    // Skip bodies with no constraint force
    if (forces.linearForce.norm() < 1e-12 &&
        forces.angularTorque.norm() < 1e-12)
    {
      continue;
    }

    inertialAssets_[k].applyForce(forces.linearForce);
    inertialAssets_[k].applyTorque(forces.angularTorque);
  }
  // ===== Phase 6: Position Correction (pseudo-velocity, positions only) =====
  // Ticket: 0040b_split_impulse_position_correction
  // Build mutable state pointer list for position correction
  std::vector<InertialState*> mutableStates;
  mutableStates.reserve(numBodies);
  for (auto& asset : inertialAssets_)
  {
    mutableStates.push_back(&asset.getInertialState());
  }
  for (auto& envAsset : environmentalAssets_)
  {
    // Environment states are conceptually immutable, but position corrector
    // handles this via zero inverse mass (no position change applied)
    mutableStates.push_back(
      const_cast<InertialState*>(&envAsset.getInertialState()));
  }

  positionCorrector_.correctPositions(constraintPtrs,
                                      mutableStates,
                                      inverseMasses,
                                      inverseInertias,
                                      numBodies,
                                      numInertial,
                                      dt);
  // Ticket: 0032_contact_constraint_refactor
  // Constraint forces accumulated here are integrated in updatePhysics()
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
