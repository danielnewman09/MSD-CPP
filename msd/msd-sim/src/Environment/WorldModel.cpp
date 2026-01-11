// Ticket: 0021_worldmodel_asset_refactor
// Design: docs/designs/worldmodel-asset-refactor/design.md

#include "msd-sim/src/Environment/WorldModel.hpp"
#include <iostream>
#include <stdexcept>

namespace msd_sim
{

// ========== Environment Asset Management ==========

size_t WorldModel::addEnvironmentAsset(AssetEnvironment&& asset)
{
  size_t index = environmentAssets_.size();
  environmentAssets_.push_back(std::move(asset));
  return index;
}

const AssetEnvironment& WorldModel::getEnvironmentAsset(size_t index) const
{
  if (index >= environmentAssets_.size())
  {
    throw std::out_of_range(
      "Environment asset index out of range: " + std::to_string(index) +
      " >= " + std::to_string(environmentAssets_.size()));
  }
  return environmentAssets_[index];
}

AssetEnvironment& WorldModel::getEnvironmentAsset(size_t index)
{
  if (index >= environmentAssets_.size())
  {
    throw std::out_of_range(
      "Environment asset index out of range: " + std::to_string(index) +
      " >= " + std::to_string(environmentAssets_.size()));
  }
  return environmentAssets_[index];
}

void WorldModel::removeEnvironmentAsset(size_t index)
{
  if (index >= environmentAssets_.size())
  {
    throw std::out_of_range(
      "Environment asset index out of range: " + std::to_string(index) +
      " >= " + std::to_string(environmentAssets_.size()));
  }

  environmentAssets_.erase(
    environmentAssets_.begin() +
    static_cast<std::vector<AssetEnvironment>::difference_type>(index));
}

// ========== Inertial Asset Management ==========

size_t WorldModel::addInertialAsset(AssetInertial&& asset)
{
  size_t index = inertialAssets_.size();
  inertialAssets_.push_back(std::move(asset));
  return index;
}

const AssetInertial& WorldModel::getInertialAsset(size_t index) const
{
  if (index >= inertialAssets_.size())
  {
    throw std::out_of_range(
      "Inertial asset index out of range: " + std::to_string(index) +
      " >= " + std::to_string(inertialAssets_.size()));
  }
  return inertialAssets_[index];
}

AssetInertial& WorldModel::getInertialAsset(size_t index)
{
  if (index >= inertialAssets_.size())
  {
    throw std::out_of_range(
      "Inertial asset index out of range: " + std::to_string(index) +
      " >= " + std::to_string(inertialAssets_.size()));
  }
  return inertialAssets_[index];
}

void WorldModel::removeInertialAsset(size_t index)
{
  if (index >= inertialAssets_.size())
  {
    throw std::out_of_range(
      "Inertial asset index out of range: " + std::to_string(index) +
      " >= " + std::to_string(inertialAssets_.size()));
  }

  inertialAssets_.erase(
    inertialAssets_.begin() +
    static_cast<std::vector<AssetInertial>::difference_type>(index));
}

// ========== Boundary Management ==========

void WorldModel::setBoundary(ConvexHull boundary)
{
  simulationBoundary_ = std::move(boundary);
}

// ========== Object Management (DEPRECATED) ==========

size_t WorldModel::spawnObject(Object&& object)
{
  size_t index = objects_.size();
  objects_.push_back(std::move(object));
  rebuildIndexCaches();
  return index;
}

const Object& WorldModel::getObject(size_t index) const
{
  if (index >= objects_.size())
  {
    throw std::out_of_range(
      "Object index out of range: " + std::to_string(index) +
      " >= " + std::to_string(objects_.size()));
  }
  return objects_[index];
}

Object& WorldModel::getObject(size_t index)
{
  if (index >= objects_.size())
  {
    throw std::out_of_range(
      "Object index out of range: " + std::to_string(index) +
      " >= " + std::to_string(objects_.size()));
  }
  return objects_[index];
}

void WorldModel::removeObject(size_t index)
{
  if (index >= objects_.size())
  {
    throw std::out_of_range(
      "Object index out of range: " + std::to_string(index) +
      " >= " + std::to_string(objects_.size()));
  }

  objects_.erase(
    objects_.begin() +
    static_cast<std::vector<Object>::difference_type>(index));
  rebuildIndexCaches();
}

void WorldModel::clearObjects()
{
  objects_.clear();
  physicsObjectIndices_.clear();
  collisionObjectIndices_.clear();
  renderObjectIndices_.clear();
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
  // Direct iteration over inertial assets (typed storage)
  for (auto& asset : inertialAssets_)
  {
    // Get dynamic state and reference frame
    DynamicState& state = asset.getDynamicState();
    ReferenceFrame& transform = asset.getReferenceFrame();

    // Simple Euler integration (can be replaced with better integrator)
    // Update velocity: v = v + a * dt
    Coordinate newLinearVel =
      state.getLinearVelocity() + state.getLinearAcceleration() * dt;
    state.setLinearVelocity(newLinearVel);

    Eigen::Vector3d newAngularVel =
      state.getAngularVelocity() + state.getAngularAcceleration() * dt;
    state.setAngularVelocity(newAngularVel);

    // Update position: p = p + v * dt
    Coordinate newPosition = transform.getOrigin() + newLinearVel * dt;
    transform.setOrigin(newPosition);

    // Update orientation using angular velocity
    // For small dt, quaternion update: q = q + 0.5 * dt * ω * q
    // This is a simplified rotation update - could use proper quaternion
    // integration For now, we'll skip orientation integration and leave it for
    // a more sophisticated integrator

    // Note: Force clearing is now handled by physics integrators if needed
    // AssetInertial doesn't have force accumulation in this implementation
  }

  // DEPRECATED: Legacy object-based physics update
  // Iterate only over objects with physics components
  for (size_t idx : physicsObjectIndices_)
  {
    Object& obj = objects_[idx];
    PhysicsComponent& physics = obj.getPhysics();
    ReferenceFrame& transform = obj.getTransform();

    // Get current state
    DynamicState& state = physics.getDynamicState();

    // Simple Euler integration (can be replaced with better integrator)
    // Update velocity: v = v + a * dt
    Coordinate newLinearVel =
      state.getLinearVelocity() + state.getLinearAcceleration() * dt;
    state.setLinearVelocity(newLinearVel);

    Eigen::Vector3d newAngularVel =
      state.getAngularVelocity() + state.getAngularAcceleration() * dt;
    state.setAngularVelocity(newAngularVel);

    // Update position: p = p + v * dt
    Coordinate newPosition = transform.getOrigin() + newLinearVel * dt;
    transform.setOrigin(newPosition);

    // Update orientation using angular velocity
    // For small dt, quaternion update: q = q + 0.5 * dt * ω * q
    // This is a simplified rotation update - could use proper quaternion
    // integration For now, we'll skip orientation integration and leave it for
    // a more sophisticated integrator

    // Clear forces for next frame (they'll be reapplied by force generators)
    physics.clearForces();
  }
}

void WorldModel::updateCollisions()
{
  // Typed storage approach: Iterate moving objects against static environment
  // Only inertial assets move, so only check them against environment and boundary

  // Check inertial assets against environment assets
  for (size_t i = 0; i < inertialAssets_.size(); ++i)
  {
    const AssetInertial& movingAsset = inertialAssets_[i];
    const ConvexHull& movingHull = movingAsset.getCollisionHull();

    // Check against all environment assets (static)
    for (size_t j = 0; j < environmentAssets_.size(); ++j)
    {
      const AssetEnvironment& staticAsset = environmentAssets_[j];
      const ConvexHull& staticHull = staticAsset.getCollisionHull();

      // TODO: Transform hulls to world space before intersection test
      // For now, assume hulls are already in world space

      // Narrow-phase: GJK collision detection
      bool collision = movingHull.intersects(staticHull);

      if (collision)
      {
        std::cout << "Collision detected between inertial asset " << i
                  << " and environment asset " << j << std::endl;

        // TODO: Implement collision response
        // 1. Compute contact manifold (contact points, normal, penetration depth)
        // 2. Apply impulse-based collision resolution
        // 3. Update velocities of moving asset
      }
    }

    // Check against simulation boundary (if present)
    if (hasBoundary())
    {
      const ConvexHull& boundary = simulationBoundary_.value();

      // TODO: Transform moving hull to world space
      bool collision = movingHull.intersects(boundary);

      if (collision)
      {
        std::cout << "Collision detected between inertial asset " << i
                  << " and simulation boundary" << std::endl;

        // TODO: Implement boundary collision response
        // Typically: reflect velocity or stop at boundary
      }
    }
  }

  // Check inertial assets against each other (dynamic-dynamic collisions)
  for (size_t i = 0; i < inertialAssets_.size(); ++i)
  {
    for (size_t j = i + 1; j < inertialAssets_.size(); ++j)
    {
      const AssetInertial& assetA = inertialAssets_[i];
      const AssetInertial& assetB = inertialAssets_[j];

      const ConvexHull& hullA = assetA.getCollisionHull();
      const ConvexHull& hullB = assetB.getCollisionHull();

      // TODO: Transform hulls to world space
      bool collision = hullA.intersects(hullB);

      if (collision)
      {
        std::cout << "Collision detected between inertial assets " << i
                  << " and " << j << std::endl;

        // TODO: Implement collision response for dynamic-dynamic collisions
        // 1. Compute contact manifold
        // 2. Apply impulse to both objects
        // 3. Update velocities of both assets
      }
    }
  }

  // DEPRECATED: Legacy collision detection
  // Simple broad-phase: check all pairs of collision objects
  // TODO: Implement spatial partitioning (octree, BVH, etc.) for better
  // performance

  const size_t numCollisionObjects = collisionObjectIndices_.size();

  for (size_t i = 0; i < numCollisionObjects; ++i)
  {
    for (size_t j = i + 1; j < numCollisionObjects; ++j)
    {
      size_t idxA = collisionObjectIndices_[i];
      size_t idxB = collisionObjectIndices_[j];

      Object& objA = objects_[idxA];
      Object& objB = objects_[idxB];

      // Get collision hulls and transforms
      const ConvexHull& hullA = objA.getCollisionHull();
      const ConvexHull& hullB = objB.getCollisionHull();
      // const ReferenceFrame& transformA = objA.getTransform();
      // const ReferenceFrame& transformB = objB.getTransform();

      // Broad-phase: Check bounding boxes first
      // TODO: Implement proper AABB transformation and intersection test
      // For now, we'll skip to narrow-phase

      // Narrow-phase: GJK collision detection
      bool collision = hullA.intersects(hullB);

      if (collision)
      {
        // TODO: Implement collision response
        // For now, just log the collision
        std::cout << "Collision detected between object " << idxA
                  << " and object " << idxB << std::endl;

        // Future work:
        // 1. Compute contact manifold (contact points, normal, penetration
        // depth)
        // 2. Apply impulse-based collision resolution
        // 3. Update velocities of both objects (if they have physics
        // components)
      }
    }
  }
}

void WorldModel::rebuildIndexCaches()
{
  physicsObjectIndices_.clear();
  collisionObjectIndices_.clear();
  renderObjectIndices_.clear();

  for (size_t i = 0; i < objects_.size(); ++i)
  {
    const Object& obj = objects_[i];

    if (obj.hasPhysics())
    {
      physicsObjectIndices_.push_back(i);
    }

    if (obj.hasCollision())
    {
      collisionObjectIndices_.push_back(i);
    }

    if (obj.hasVisualGeometry())
    {
      renderObjectIndices_.push_back(i);
    }
  }
}

}  // namespace msd_sim
