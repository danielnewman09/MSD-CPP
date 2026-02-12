#ifndef MSD_SIM_PHYSICS_ASSET_DYNAMIC_STATE_HPP
#define MSD_SIM_PHYSICS_ASSET_DYNAMIC_STATE_HPP

#include <vector>

#include "msd-sim/src/DataTypes/ExternalForce.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"
#include "msd-transfer/src/AssetDynamicStateRecord.hpp"

namespace msd_sim
{

/**
 * @brief Per-frame mutable state of a dynamic rigid body.
 *
 * Groups the three quantities that change every simulation frame:
 * - Kinematic state (position, velocity, orientation, etc.)
 * - Individual external force contributions
 *
 * Individual forces are stored per-entry for diagnostics and recording;
 * use ExternalForce::accumulate() for net force/torque sums.
 *
 * Mirrors the transfer-layer AssetDynamicStateRecord for clean
 * domain ↔ persistence mapping.
 *
 * @ticket 0056h_asset_dynamic_state_struct
 */
struct AssetDynamicState
{
  InertialState inertialState;
  std::vector<ExternalForce> externalForces;

  /**
   * @brief Clear all individual force entries.
   *
   * Called at the end of each physics step after integration.
   */
  void clearForces() { externalForces.clear(); }

  /**
   * @brief Convert to a transfer record for database persistence.
   *
   * Each individual external force is stored as a separate record entry
   * via a one-to-many relationship.
   *
   * @param bodyId The instance ID of the owning AssetInertial
   * @return Transfer record containing all dynamic state data
   */
  [[nodiscard]] msd_transfer::AssetDynamicStateRecord toRecord(
    uint32_t bodyId) const
  {
    msd_transfer::AssetDynamicStateRecord record;
    record.body_id = bodyId;
    record.kinematicState = inertialState.toRecord();
    for (const auto& ef : externalForces)
    {
      record.externalForces.data.push_back(ef.toRecord());
    }
    return record;
  }

  /**
   * @brief Reconstruct from a transfer record.
   *
   * Each stored force record is reconstructed as an individual entry.
   *
   * @param record The transfer record to reconstruct from
   * @return AssetDynamicState with all fields populated
   */
  static AssetDynamicState fromRecord(
    const msd_transfer::AssetDynamicStateRecord& record)
  {
    AssetDynamicState state;
    state.inertialState =
      InertialState::fromRecord(record.kinematicState);
    for (const auto& efr : record.externalForces.data)
    {
      state.externalForces.push_back(ExternalForce::fromRecord(efr));
    }
    return state;
  }
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_ASSET_DYNAMIC_STATE_HPP
