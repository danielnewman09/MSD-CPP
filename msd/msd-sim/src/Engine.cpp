// Ticket: 0004_gui_framerate
// Ticket: 0005_camera_controller_sim
// Design: docs/designs/input-state-management/design.md
// Design: docs/designs/0005_camera_controller_sim/design.md

#include <exception>
#include <iostream>

#include "msd-sim/src/Engine.hpp"
#include "msd-sim/src/Agent/InputControlAgent.hpp"

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


void Engine::spawnInertialObject(const std::string assetName,
                                 const Coordinate& position,
                                 const EulerAngles& orientation)
{
  ReferenceFrame objectFrame{position, orientation};

  const auto& asset = assetRegistry_.getAsset(assetName);

  if (!asset.has_value())
  {
    throw std::runtime_error("Could not spawn object with name: " + assetName +
                             ". It was not found in the asset registry");
  }

  worldModel_.spawnObject(
    Object::createInertial(asset->get(), objectFrame, 1.0));
}

uint32_t Engine::spawnPlayerPlatform(const std::string& assetName,
                                     const Coordinate& position,
                                     const EulerAngles& orientation)
{
  // Create visual object
  ReferenceFrame objectFrame{position, orientation};
  const auto& asset = assetRegistry_.getAsset(assetName);

  if (!asset.has_value())
  {
    throw std::runtime_error("Could not spawn player platform with asset: " +
                             assetName +
                             ". Asset not found in registry");
  }

  auto object = Object::createGraphical(asset->get(), objectFrame);
  size_t objIndex = worldModel_.spawnObject(std::move(object));

  // Create Platform with InputControlAgent
  uint32_t platformId = worldModel_.getNextPlatformId();
  Platform platform{platformId};
  platform.setAgent(std::make_unique<InputControlAgent>());
  platform.getState().position = position;
  platform.getState().angularPosition = orientation;
  platform.setVisualObject(worldModel_.getObject(objIndex));

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
      // Check if platform has visual object linked
      if (platform.hasVisualObject())
      {
        // Update visual object's transform via MotionController
        auto& visualObject = platform.getVisualObject();
        auto& motionController = platform.getMotionController();

        // Calculate deltaTime since last update (assume 16ms for now - should be passed from update loop)
        // TODO: Pass deltaTime from the update loop instead of hardcoding
        std::chrono::milliseconds deltaTime{16};

        motionController.updateTransform(visualObject.getTransform(), commands, deltaTime);
      }
      break;
    }
  }
}

void Engine::update(std::chrono::milliseconds simTime)
{
  worldModel_.update(simTime);
}
}  // namespace msd_sim