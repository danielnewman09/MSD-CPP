#include "msd-sim/src/Environment/WorldModel.hpp"
#include <iostream>
#include <stdexcept>

namespace msd_sim
{

// ========== Object Management ==========

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

  objects_.erase(objects_.begin() + index);
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

// void WorldModel::addPlatform(Platform&& platform)
// {
//   platforms_.push_back(std::move(platform));
// }

// ========== Simulation Update ==========

void WorldModel::update(std::chrono::milliseconds deltaTime)
{
  // Convert to seconds for physics calculations
  double dt = deltaTime.count() / 1000.0;

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
