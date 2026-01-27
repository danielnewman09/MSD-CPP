#include "msd-sim/src/Environment/WorldModel.hpp"
#include <iostream>
#include <stdexcept>
#include "msd-sim/src/Physics/CollisionResponse.hpp"

namespace msd_sim
{


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

    // Update orientation using proper Euler angle rate transformation
    // Angular velocity is in world frame, but Euler angle rates require
    // body-frame angular velocity with a Jacobian transformation.
    //
    // For ZYX Euler angles (R = Rz * Ry * Rx):
    // ω_body = R^T * ω_world
    //
    // Then Euler angle rates are:
    // [roll_dot ]   [1, sin(roll)*tan(pitch),  cos(roll)*tan(pitch)] [ωx_body]
    // [pitch_dot] = [0, cos(roll),            -sin(roll)           ] [ωy_body]
    // [yaw_dot  ]   [0, sin(roll)/cos(pitch),  cos(roll)/cos(pitch)] [ωz_body]

    // Convert world-frame ω to body-frame
    const Eigen::Matrix3d& R = asset.getReferenceFrame().getRotation();
    Eigen::Vector3d omega_body = R.transpose() * omega;

    // Get current Euler angles
    double roll = state.orientation.roll();
    double pitch = state.orientation.pitch();

    // Compute trig functions
    double sr = std::sin(roll);
    double cr = std::cos(roll);
    double sp = std::sin(pitch);
    double cp = std::cos(pitch);

    // Avoid gimbal lock singularity (pitch near ±90°)
    constexpr double kGimbalLockThreshold = 0.9999;
    if (std::abs(cp) < (1.0 - kGimbalLockThreshold))
    {
      // Near gimbal lock - fall back to simple integration
      // This is a known limitation of Euler angles
      state.orientation += omega * dt;
    }
    else
    {
      double tp = sp / cp;  // tan(pitch)

      // Compute Euler angle rates from body-frame angular velocity
      double roll_dot = omega_body.x() + sr * tp * omega_body.y() +
                        cr * tp * omega_body.z();
      double pitch_dot = cr * omega_body.y() - sr * omega_body.z();
      double yaw_dot =
        (sr / cp) * omega_body.y() + (cr / cp) * omega_body.z();

      // Integrate Euler angles
      state.orientation.setRoll(roll + roll_dot * dt);
      state.orientation.setPitch(pitch + pitch_dot * dt);
      state.orientation.setYaw(state.orientation.yaw() + yaw_dot * dt);
    }
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

      // ===== Apply Manifold-Aware Collision Response (Ticket: 0029) =====
      CollisionResponse::applyImpulseManifold(
        assetA, assetB, *result, combinedE);
      CollisionResponse::applyPositionCorrectionManifold(
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

      // ===== Apply Manifold-Aware Collision Response (Ticket: 0029) =====
      CollisionResponse::applyImpulseManifoldStatic(
        inertial, environment, *result, combinedE);
      CollisionResponse::applyPositionCorrectionStatic(
        inertial, environment, *result);
    }
  }
  // Ticket: 0027_collision_response_system, 0029_contact_manifold_generation
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
