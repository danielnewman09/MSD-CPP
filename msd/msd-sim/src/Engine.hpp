#ifndef MSD_ENGINE_HPP
#define MSD_ENGINE_HPP

#include <chrono>

#include "msd-assets/src/AssetRegistry.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"

namespace msd_sim
{

class Engine
{
public:
  Engine(const std::string& dbPath);

  void update(std::chrono::milliseconds simTime);

  void spawnInertialObject(const std::string assetName,
                           const Coordinate& position,
                           const EulerAngles& orientation);

  msd_assets::AssetRegistry& getAssetRegistry();


private:
  msd_assets::AssetRegistry assetRegistry_;
  WorldModel worldModel_;
};

}  // namespace msd_sim

#endif  // MSD_ENGINE_HPP