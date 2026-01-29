#include "msd-sim/src/Environment/WorldModel.hpp"
#include <iostream>
#include <stdexcept>
#include "msd-sim/src/Physics/CollisionResponse.hpp"
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

  // IMPORTANT: Collision response BEFORE physics integration
  // Order matters: collision impulses must be applied before velocity
  // integration
  updateCollisions();

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

void WorldModel::updateCollisions()
{
  // O(n²) pairwise collision detection and response
  // For typical scene sizes (< 100 objects), this is acceptable
  // Future optimization: broadphase spatial partitioning in separate ticket

  const size_t assetCount = inertialAssets_.size();

  for (size_t i = 0; i < assetCount; ++i)
  {
    for (size_t j = i + 1; j < assetCount; ++j)
    {
      AssetInertial& assetA = inertialAssets_[i];
      AssetInertial& assetB = inertialAssets_[j];

      // ===== Collision Detection =====
      auto result = collisionHandler_.checkCollision(assetA, assetB);
      if (!result)
      {
        continue;  // No collision
      }

      // ===== Combine Restitution Coefficients =====
      double combinedE = CollisionResponse::combineRestitution(
        assetA.getCoefficientOfRestitution(),
        assetB.getCoefficientOfRestitution());

      // ===== Apply Lagrangian Constraint Response (Frictionless) =====
      // Computes Lagrange multiplier for non-penetration constraint
      // and applies constraint forces along contact normal only.
      CollisionResponse::applyConstraintResponse(
        assetA, assetB, *result, combinedE);
      CollisionResponse::applyPositionStabilization(
        assetA, assetB, *result);
    }
  }

  // ===== Inertial vs Environment Collisions =====
  // Dynamic objects colliding with static environment
  for (auto& inertial : inertialAssets_)
  {
    for (auto& environment : environmentalAssets_)
    {
      // Collision detection (inertial is A, environment is B)
      auto result = collisionHandler_.checkCollision(inertial, environment);
      if (!result)
      {
        continue;  // No collision
      }

      // Combine restitution (using default for environment)
      double combinedE = CollisionResponse::combineRestitution(
        inertial.getCoefficientOfRestitution(),
        CollisionResponse::kEnvironmentRestitution);

      // ===== Apply Lagrangian Constraint Response (Frictionless) =====
      CollisionResponse::applyConstraintResponseStatic(
        inertial, environment, *result, combinedE);
      CollisionResponse::applyPositionStabilizationStatic(
        inertial, environment, *result);
    }
  }
  // Ticket: 0027_collision_response_system (refactored to Lagrangian mechanics)
  // Frictionless constraints only - tangential forces removed
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
