// Ticket: 0004_gui_framerate
// Design: docs/designs/input-state-management/design.md

#ifndef MSD_ENGINE_HPP
#define MSD_ENGINE_HPP

#include <chrono>
#include <optional>

#include "msd-assets/src/AssetRegistry.hpp"
#include "msd-sim/src/Agent/InputCommands.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"

namespace msd_sim
{

class Engine
{
public:
  Engine(const std::string& dbPath);

  void update(std::chrono::milliseconds simTime);

  const AssetInertial& spawnInertialObject(const std::string assetName,
                                           const Coordinate& position,
                                           const AngularCoordinate& orientation);

  /**
   * @brief Spawn a static environment object from an asset
   * @param assetName Name of asset in registry
   * @param position World position
   * @param orientation World orientation
   * @return Reference to spawned environment object
   *
   * Environment objects are static (immovable) and participate in collision
   * detection. Dynamic objects will bounce off them.
   */
  const AssetEnvironment& spawnEnvironmentObject(
      const std::string& assetName,
      const Coordinate& position,
      const AngularCoordinate& orientation);

  /**
   * @brief Spawn a player-controlled platform with visual object
   * @param assetName Name of visual asset
   * @param position Initial position
   * @param orientation Initial orientation
   * @return Platform ID
   *
   * Creates a Platform with InputControlAgent and links it to a visual Object
   * in WorldModel. Sets this platform as the player platform for input
   * commands.
   */
  uint32_t spawnPlayerPlatform(const std::string& assetName,
                               const Coordinate& position,
                               const AngularCoordinate& orientation);

  /**
   * @brief Set input commands for the player platform
   * @param commands Input commands from GUI
   *
   * Propagates input commands to the player platform's InputControlAgent.
   * If no player platform exists, this method does nothing.
   */
  void setPlayerInputCommands(const InputCommands& commands);

  msd_assets::AssetRegistry& getAssetRegistry();

  /**
   * @brief Get access to the world model
   * @return Reference to world model
   */
  WorldModel& getWorldModel()
  {
    return worldModel_;
  }

  const WorldModel& getWorldModel() const
  {
    return worldModel_;
  }


private:
  msd_assets::AssetRegistry assetRegistry_;

  std::unordered_map<uint32_t, ConvexHull> registryHullMap_;

  // Floor cube hull (not from registry, created programmatically)
  ConvexHull floorHull_;

  WorldModel worldModel_;
  std::optional<uint32_t> playerPlatformId_;
};

}  // namespace msd_sim

#endif  // MSD_ENGINE_HPP