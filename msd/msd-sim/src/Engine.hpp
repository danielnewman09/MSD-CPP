// Ticket: 0004_gui_framerate
// Ticket: 0021_worldmodel_asset_refactor
// Design: docs/designs/input-state-management/design.md
// Design: docs/designs/worldmodel-asset-refactor/design.md

#ifndef MSD_ENGINE_HPP
#define MSD_ENGINE_HPP

#include <chrono>
#include <optional>
#include <vector>

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

  /**
   * @brief Spawn a dynamic physics object (DEPRECATED)
   * @deprecated Use spawnInertialAsset for new typed storage
   */
  [[deprecated("Use spawnInertialAsset() for new typed storage")]]
  void spawnInertialObject(const std::string assetName,
                           const Coordinate& position,
                           const EulerAngles& orientation);

  /**
   * @brief Spawn a dynamic physics asset into WorldModel
   * @param assetName Name of asset in registry
   * @param position Initial position
   * @param orientation Initial orientation
   * @param mass Mass in kilograms (must be positive)
   * @return Index of spawned asset in WorldModel
   * @throws std::runtime_error if asset not found
   * @throws std::invalid_argument if asset has no collision geometry
   */
  size_t spawnInertialAsset(const std::string& assetName,
                            const Coordinate& position,
                            const EulerAngles& orientation,
                            double mass = 1.0);

  /**
   * @brief Spawn a static environment asset into WorldModel
   * @param assetName Name of asset in registry
   * @param position Position in world space
   * @param orientation Orientation in world space
   * @return Index of spawned asset in WorldModel
   * @throws std::runtime_error if asset not found
   * @throws std::invalid_argument if asset has no collision geometry
   */
  size_t spawnEnvironmentAsset(const std::string& assetName,
                                const Coordinate& position,
                                const EulerAngles& orientation);

  /**
   * @brief Set simulation boundary from ConvexHull
   * @param boundary Boundary hull
   */
  void setSimulationBoundary(const ConvexHull& boundary);

  /**
   * @brief Set simulation boundary from point cloud
   * @param boundaryPoints Vertices defining boundary hull
   * @throws std::runtime_error if point cloud is degenerate
   */
  void setSimulationBoundary(const std::vector<Coordinate>& boundaryPoints);

  /**
   * @brief Spawn a player-controlled platform with visual object
   * @param assetName Name of visual asset
   * @param position Initial position
   * @param orientation Initial orientation
   * @return Platform ID
   *
   * Creates a Platform with InputControlAgent and links it to a visual Object
   * in WorldModel. Sets this platform as the player platform for input commands.
   */
  uint32_t spawnPlayerPlatform(const std::string& assetName,
                               const Coordinate& position,
                               const EulerAngles& orientation);

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
  WorldModel& getWorldModel() { return worldModel_; }


private:
  msd_assets::AssetRegistry assetRegistry_;
  WorldModel worldModel_;
  std::optional<uint32_t> playerPlatformId_;
};

}  // namespace msd_sim

#endif  // MSD_ENGINE_HPP