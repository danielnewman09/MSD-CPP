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

void WorldModel::updatePhysics(double /*dt*/)
{
  // TODO: implement physics update
}

void WorldModel::updateCollisions()
{
}


uint32_t WorldModel::getInertialAssetId()
{
  return ++inertialAssetIdCounter_;
}


}  // namespace msd_sim
