#include "msd-sim/src/Environment/WorldModel.hpp"
#include <iostream>
#include <stdexcept>
#include "msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp"
#include "msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"

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


// ========== Object Management ==========

const AssetInertial& WorldModel::spawnObject(uint32_t assetId,
                                             ConvexHull& hull,
                                             const ReferenceFrame& origin)
{
  auto instanceId = getInertialAssetId();
  inertialAssets_.emplace_back(assetId, instanceId, hull, 10.0, origin);
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
  auto it = std::find_if(inertialAssets_.begin(),
                         inertialAssets_.end(),
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
  auto it = std::find_if(inertialAssets_.begin(),
                         inertialAssets_.end(),
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
  double dt = (simTime.count() - time_.count()) / 1000.0;

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
    double mass = asset.getMass();
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
    std::vector<Constraint*> constraints = asset.getConstraints();

    // ===== Step 3: Delegate Integration to Integrator =====
    // Integrator handles: velocity update, position update, constraint enforcement
    integrator_->step(
      state,
      netForce,
      netTorque,
      mass,
      asset.getInverseInertiaTensor(),
      constraints,
      dt);

    // ===== Step 4: Synchronize ReferenceFrame =====
    // ReferenceFrame must match InertialState for collision detection and rendering
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
  // Constraint-based collision response using Projected Gauss-Seidel (PGS)
  // Replaces per-pair impulse-based CollisionResponse with a unified solver.
  //
  // Solver body indexing:
  //   [0 .. numInertial-1]                   = inertial bodies
  //   [numInertial .. numInertial+numEnv-1]   = environment bodies (infinite mass)
  //
  // Ticket: 0032_contact_constraint_refactor

  const size_t numInertial = inertialAssets_.size();
  const size_t numEnvironment = environmentalAssets_.size();
  const size_t numBodies = numInertial + numEnvironment;

  if (numBodies == 0 || dt <= 0.0)
  {
    return;
  }

  // ===== Phase 1: Collision Detection =====
  // Collect all collision pairs with their body indices

  struct CollisionPair
  {
    size_t bodyAIndex;
    size_t bodyBIndex;
    CollisionResult result;
    double restitution;
  };

  std::vector<CollisionPair> collisions;

  // Inertial vs Inertial (O(n²) pairwise)
  for (size_t i = 0; i < numInertial; ++i)
  {
    for (size_t j = i + 1; j < numInertial; ++j)
    {
      auto result = collisionHandler_.checkCollision(
          inertialAssets_[i], inertialAssets_[j]);
      if (!result)
      {
        continue;
      }

      double combinedE = ContactConstraintFactory::combineRestitution(
          inertialAssets_[i].getCoefficientOfRestitution(),
          inertialAssets_[j].getCoefficientOfRestitution());

      collisions.push_back({i, j, *result, combinedE});
    }
  }

  // Inertial vs Environment
  for (size_t i = 0; i < numInertial; ++i)
  {
    for (size_t e = 0; e < numEnvironment; ++e)
    {
      auto result = collisionHandler_.checkCollision(
          inertialAssets_[i], environmentalAssets_[e]);
      if (!result)
      {
        continue;
      }

      double combinedE = ContactConstraintFactory::combineRestitution(
          inertialAssets_[i].getCoefficientOfRestitution(),
          environmentalAssets_[e].getCoefficientOfRestitution());

      // Environment body index offset by numInertial
      collisions.push_back({i, numInertial + e, *result, combinedE});
    }
  }

  if (collisions.empty())
  {
    return;
  }

  // ===== Phase 2: Create Contact Constraints =====
  std::vector<std::unique_ptr<ContactConstraint>> allConstraints;

  for (const auto& pair : collisions)
  {
    const InertialState& stateA = (pair.bodyAIndex < numInertial)
        ? inertialAssets_[pair.bodyAIndex].getInertialState()
        : environmentalAssets_[pair.bodyAIndex - numInertial].getInertialState();

    const InertialState& stateB = (pair.bodyBIndex < numInertial)
        ? inertialAssets_[pair.bodyBIndex].getInertialState()
        : environmentalAssets_[pair.bodyBIndex - numInertial].getInertialState();

    const Coordinate& comA = stateA.position;
    const Coordinate& comB = stateB.position;

    auto constraints = ContactConstraintFactory::createFromCollision(
        pair.bodyAIndex, pair.bodyBIndex,
        pair.result, stateA, stateB,
        comA, comB, pair.restitution);

    for (auto& c : constraints)
    {
      allConstraints.push_back(std::move(c));
    }
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
    inverseInertias.push_back(asset.getInverseInertiaTensor());
  }

  for (auto& envAsset : environmentalAssets_)
  {
    states.push_back(std::cref(envAsset.getInertialState()));
    inverseMasses.push_back(envAsset.getInverseMass());            // 0.0
    inverseInertias.push_back(envAsset.getInverseInertiaTensor()); // Zero matrix
  }

  // Build non-owning TwoBodyConstraint* vector for solver
  std::vector<TwoBodyConstraint*> constraintPtrs;
  constraintPtrs.reserve(allConstraints.size());
  for (auto& c : allConstraints)
  {
    constraintPtrs.push_back(c.get());
  }

  // ===== Phase 4: Solve PGS =====
  auto solveResult = contactSolver_.solveWithContacts(
      constraintPtrs, states, inverseMasses, inverseInertias,
      numBodies, dt);

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

}  // namespace msd_sim
