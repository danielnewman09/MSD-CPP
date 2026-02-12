// Ticket: 0056a_collision_force_transfer_records

#ifndef MSD_SIM_PHYSICS_ASSET_STATIC_STATE_HPP
#define MSD_SIM_PHYSICS_ASSET_STATIC_STATE_HPP

#include <cstdint>
#include <limits>

#include "msd-sim/src/Physics/RigidBody/MaterialProperties.hpp"
#include "msd-transfer/src/AssetInertialStaticRecord.hpp"

namespace msd_sim
{

/**
 * @brief Spawn-time static properties of a dynamic rigid body.
 *
 * Groups the quantities that are set once at construction and do not change
 * per-frame: mass and surface material properties. Mirrors the transfer-layer
 * AssetInertialStaticRecord for clean domain-persistence mapping.
 *
 * @ticket 0056a_collision_force_transfer_records
 */
struct AssetStaticState
{
  double mass{std::numeric_limits<double>::quiet_NaN()};
  MaterialProperties material;

  /**
   * @brief Convert to a transfer record for database persistence.
   *
   * @param bodyId The instance ID of the owning AssetInertial
   * @return Transfer record with body_id, mass, restitution, friction
   */
  [[nodiscard]] msd_transfer::AssetInertialStaticRecord toRecord(
    uint32_t bodyId) const
  {
    msd_transfer::AssetInertialStaticRecord record;
    record.body_id = bodyId;
    record.mass = mass;
    record.restitution = material.coefficientOfRestitution;
    record.friction = material.frictionCoefficient;
    return record;
  }

  /**
   * @brief Reconstruct from a transfer record.
   *
   * @param record The transfer record to reconstruct from
   * @return AssetStaticState with all fields populated
   */
  static AssetStaticState fromRecord(
    const msd_transfer::AssetInertialStaticRecord& record)
  {
    AssetStaticState state;
    state.mass = record.mass;
    state.material.coefficientOfRestitution = record.restitution;
    state.material.frictionCoefficient = record.friction;
    return state;
  }
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_ASSET_STATIC_STATE_HPP
