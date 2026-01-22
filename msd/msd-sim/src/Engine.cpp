// Ticket: 0004_gui_framerate
// Ticket: 0005_camera_controller_sim
// Design: docs/designs/input-state-management/design.md
// Design: docs/designs/0005_camera_controller_sim/design.md

#include <exception>
#include <iostream>

#include "msd-sim/src/Agent/InputControlAgent.hpp"
#include "msd-sim/src/Engine.hpp"

namespace msd_sim
{

Engine::Engine(const std::string& dbPath)
  : assetRegistry_{dbPath}, worldModel_{}
{
}

msd_assets::AssetRegistry& Engine::getAssetRegistry()
{
  return assetRegistry_;
}


const AssetInertial& Engine::spawnInertialObject(const std::string assetName,
                                                 const Coordinate& position,
                                                 const AngularCoordinate& orientation)
{
  ReferenceFrame objectFrame{position, orientation};

  const auto& asset = assetRegistry_.getAsset(assetName);

  if (!asset.has_value())
  {
    throw std::runtime_error("Could not spawn object with name: " + assetName +
                             ". It was not found in the asset registry");
  }

  // Cache asset reference to avoid repeated accessor calls
  const auto& assetRef = asset->get();

  // try_emplace only constructs the ConvexHull if key doesn't exist
  // Returns iterator to existing or newly inserted element
  auto [hullIt, inserted] = registryHullMap_.try_emplace(
    assetRef.getId(), assetRef.getCollisionGeometry()->get().getVertices());

  return worldModel_.spawnObject(assetRef.getId(), hullIt->second, objectFrame);
}

uint32_t Engine::spawnPlayerPlatform(const std::string& assetName,
                                     const Coordinate& position,
                                     const AngularCoordinate& orientation)
{
  ReferenceFrame objectFrame{position, orientation};

  const auto& asset = assetRegistry_.getAsset(assetName);

  if (!asset.has_value())
  {
    throw std::runtime_error("Could not spawn object with name: " + assetName +
                             ". It was not found in the asset registry");
  }

  // Cache asset reference to avoid repeated accessor calls
  const auto& assetRef = asset->get();

  // try_emplace only constructs the ConvexHull if key doesn't exist
  // Returns iterator to existing or newly inserted element
  auto [hullIt, inserted] = registryHullMap_.try_emplace(
    assetRef.getId(), assetRef.getCollisionGeometry()->get().getVertices());

  // Create Platform with InputControlAgent
  uint32_t platformId = worldModel_.getNextPlatformId();
  Platform platform{platformId,
                    worldModel_.getInertialAssetId(),
                    assetRef.getId(),
                    hullIt->second,
                    1.0,
                    objectFrame};
  platform.setAgent(std::make_unique<InputControlAgent>());

  worldModel_.addPlatform(std::move(platform));
  playerPlatformId_ = platformId;

  return platformId;
}

void Engine::setPlayerInputCommands(const InputCommands& commands)
{
  if (!playerPlatformId_.has_value())
  {
    return;
  }

  // Find platform by ID
  for (auto& platform : worldModel_.getPlatforms())
  {
    if (platform.getId() == *playerPlatformId_)
    {
      // Update visual object's transform via MotionController
      auto& inertialAsset = platform.getInertialAsset();
      auto& motionController = platform.getMotionController();

      // Calculate deltaTime since last update (assume 16ms for now - should
      // be passed from update loop)
      // TODO: Pass deltaTime from the update loop instead of hardcoding
      std::chrono::milliseconds deltaTime{16};

      motionController.updateTransform(
        inertialAsset.getReferenceFrame(), commands, deltaTime);

      break;
    }
  }
}

void Engine::update(std::chrono::milliseconds simTime)
{
  worldModel_.update(simTime);
}
}  // namespace msd_sim