#include "msd-sim/src/Environment/WorldModel.hpp"
#include <iostream>
#include <stdexcept>

namespace msd_sim
{


// ========== Object Management ==========

const AssetInertial& WorldModel::spawnObject(uint32_t assetId,
                                             ConvexHull& hull,
                                             const ReferenceFrame& origin)
{
  auto instanceId = getInertialAssetId();
  inertialAssets_.emplace_back(assetId, instanceId, hull, 1.0, origin);
  return inertialAssets_.back();
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

void WorldModel::update(std::chrono::milliseconds deltaTime)
{
  // Convert to seconds for physics calculations
  double dt = deltaTime.count() / 1000.0;

  // Update all platforms (agent logic + visual sync)
  for (auto& platform : platforms_)
  {
    platform.update(time_ + deltaTime);
  }

  // Update physics for all dynamic objects
  updatePhysics(dt);

  // Detect and resolve collisions
  updateCollisions();

  // Update simulation time
  time_ += deltaTime;

  // Debug output
  std::cout << "Simulation Time: " << time_.count() << " ms" << std::endl;
}

// ========== Private Methods ==========

void WorldModel::updatePhysics(double dt)
{
  for (auto& asset : inertialAssets_)
  {
    // ===== Apply Gravity =====
    // F_gravity = m * g (where g is acceleration vector)
    CoordinateRate gravityForce = gravity_ * asset.getMass();
    asset.applyForce(gravityForce);

    // ===== Linear Integration (Semi-Implicit Euler) =====
    // Get accumulated forces
    const Coordinate& netForce = asset.getAccumulatedForce();

    // Compute linear acceleration: a = F_net / m
    Coordinate linearAccel = netForce / asset.getMass();

    // Update velocity: v_new = v_old + a * dt
    InertialState& state = asset.getInertialState();
    state.velocity += linearAccel * dt;
    state.acceleration = linearAccel;

    // Update position using NEW velocity: x_new = x_old + v_new * dt
    state.position += state.velocity * dt;

    // ===== Angular Integration (Semi-Implicit Euler) =====
    // Get accumulated torque
    const Coordinate& netTorque = asset.getAccumulatedTorque();

    // Compute angular acceleration: α = I⁻¹ * τ
    Eigen::Vector3d angularAccel = asset.getInverseInertiaTensor() * netTorque;

    // Update angular velocity: ω_new = ω_old + α * dt
    AngularRate& omega = state.angularVelocity;
    omega += angularAccel * dt;
    state.angularAcceleration = AngularRate{angularAccel};

    // Update orientation: θ_new = θ_old + ω_new * dt
    state.orientation += omega * dt;

    // ===== Synchronize ReferenceFrame =====
    // ReferenceFrame must match InertialState for collision detection
    // and rendering to work correctly
    ReferenceFrame& frame = asset.getReferenceFrame();
    frame.setOrigin(state.position);
    frame.setRotation(state.orientation);

    // ===== Clear Forces for Next Frame =====
    asset.clearForces();
  }
  // Ticket: 0023_force_application_system
}

void WorldModel::updateCollisions()
{
}


uint32_t WorldModel::getInertialAssetId()
{
  return ++inertialAssetIdCounter_;
}

// ========== Gravity Configuration (ticket 0023a) ==========

const Coordinate& WorldModel::getGravity() const
{
  return gravity_;
}

}  // namespace msd_sim
