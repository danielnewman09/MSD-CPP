#ifndef MSD_SIM_PHYSICS_ASSET_DYNAMIC_STATE_HPP
#define MSD_SIM_PHYSICS_ASSET_DYNAMIC_STATE_HPP

#include "msd-sim/src/DataTypes/ForceVector.hpp"
#include "msd-sim/src/DataTypes/TorqueVector.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"
#include "msd-transfer/src/AssetDynamicStateRecord.hpp"

namespace msd_sim
{

/**
 * @brief Per-frame mutable state of a dynamic rigid body.
 *
 * Groups the three quantities that change every simulation frame:
 * - Kinematic state (position, velocity, orientation, etc.)
 * - Accumulated force
 * - Accumulated torque
 *
 * Mirrors the transfer-layer AssetDynamicStateRecord for clean
 * domain ↔ persistence mapping.
 *
 * @ticket 0056h_asset_dynamic_state_struct
 */
struct AssetDynamicState
{
  InertialState inertialState;
  ForceVector accumulatedForce{0.0, 0.0, 0.0};
  TorqueVector accumulatedTorque{0.0, 0.0, 0.0};

  /**
   * @brief Reset accumulated force and torque to zero.
   *
   * Called at the end of each physics step after integration.
   */
  void clearForces()
  {
    accumulatedForce = ForceVector{0.0, 0.0, 0.0};
    accumulatedTorque = TorqueVector{0.0, 0.0, 0.0};
  }

  /**
   * @brief Convert to a transfer record for database persistence.
   *
   * @param bodyId The instance ID of the owning AssetInertial
   * @return Transfer record containing all dynamic state data
   */
  [[nodiscard]] msd_transfer::AssetDynamicStateRecord toRecord(
    uint32_t bodyId) const
  {
    msd_transfer::AssetDynamicStateRecord record;
    record.body_id = bodyId;

    auto stateRecord = inertialState.toRecord();
    record.position = stateRecord.position;
    record.velocity = stateRecord.velocity;
    record.acceleration = stateRecord.acceleration;
    record.orientation = stateRecord.orientation;
    record.quaternionRate = stateRecord.quaternionRate;
    record.angularAcceleration = stateRecord.angularAcceleration;

    record.accumulatedForce = accumulatedForce.toRecord();
    record.accumulatedTorque = accumulatedTorque.toRecord();

    return record;
  }

  /**
   * @brief Reconstruct from a transfer record.
   *
   * @param record The transfer record to reconstruct from
   * @return AssetDynamicState with all fields populated
   */
  static AssetDynamicState fromRecord(
    const msd_transfer::AssetDynamicStateRecord& record)
  {
    // Build an InertialStateRecord from the flattened fields
    msd_transfer::InertialStateRecord stateRecord;
    stateRecord.position = record.position;
    stateRecord.velocity = record.velocity;
    stateRecord.acceleration = record.acceleration;
    stateRecord.orientation = record.orientation;
    stateRecord.quaternionRate = record.quaternionRate;
    stateRecord.angularAcceleration = record.angularAcceleration;

    AssetDynamicState state;
    state.inertialState = InertialState::fromRecord(stateRecord);
    state.accumulatedForce = ForceVector::fromRecord(record.accumulatedForce);
    state.accumulatedTorque =
      TorqueVector::fromRecord(record.accumulatedTorque);
    return state;
  }
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_ASSET_DYNAMIC_STATE_HPP
