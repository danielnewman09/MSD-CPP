
#include <exception>
#include <iostream>

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

void Engine::update(std::chrono::milliseconds simTime)
{
  worldModel_.update(simTime);
}
}  // namespace msd_sim