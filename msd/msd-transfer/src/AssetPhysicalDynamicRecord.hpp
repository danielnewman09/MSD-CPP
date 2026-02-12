// Ticket: 0056a_collision_force_transfer_records
// Per-frame position and orientation for any AssetPhysical

#ifndef MSD_TRANSFER_ASSET_PHYSICAL_DYNAMIC_RECORD_HPP
#define MSD_TRANSFER_ASSET_PHYSICAL_DYNAMIC_RECORD_HPP

#include <cstdint>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>

#include "msd-transfer/src/CoordinateRecord.hpp"
#include "msd-transfer/src/QuaternionDRecord.hpp"
#include "msd-transfer/src/SimulationFrameRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Per-frame position and orientation of an AssetPhysical
 *
 * Captures the ReferenceFrame state (origin + quaternion) for any physical
 * asset at a given simulation frame. Enables environment asset replay
 * (environment objects have no InertialState but do have a ReferenceFrame).
 *
 * @see msd_sim::ReferenceFrame
 * @see msd_sim::AssetPhysical
 * @ticket 0056a_collision_force_transfer_records
 */
struct AssetPhysicalDynamicRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_id{0};

  // Position (origin of reference frame)
  CoordinateRecord position;

  // Orientation (quaternion from reference frame)
  QuaternionDRecord orientation;

  // Frame FK for temporal association
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(AssetPhysicalDynamicRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_id, position, orientation, frame));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_ASSET_PHYSICAL_DYNAMIC_RECORD_HPP
