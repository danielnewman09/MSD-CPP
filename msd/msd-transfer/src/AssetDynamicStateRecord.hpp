#ifndef MSD_TRANSFER_ASSET_DYNAMIC_STATE_RECORD_HPP
#define MSD_TRANSFER_ASSET_DYNAMIC_STATE_RECORD_HPP

#include <cstdint>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBRepeatedFieldTransferObject.hpp>

#include "msd-transfer/src/ExternalForceRecord.hpp"
#include "msd-transfer/src/InertialStateRecord.hpp"
#include "msd-transfer/src/SimulationFrameRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Complete per-frame dynamic state of an AssetInertial
 *
 * Captures the full kinematic state (via InertialStateRecord) plus
 * accumulated force and torque for a single body at a single simulation
 * frame.
 *
 * @see msd_sim::AssetDynamicState
 * @see msd_sim::AssetInertial
 */
struct AssetDynamicStateRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_id{0};

  // Kinematic state (composition, not flattened)
  InertialStateRecord kinematicState;

  // Individual external force entries (one-to-many)
  cpp_sqlite::RepeatedFieldTransferObject<ExternalForceRecord> externalForces;

  // Frame FK for temporal association
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(AssetDynamicStateRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_id,
                       kinematicState,
                       externalForces,
                       frame));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_ASSET_DYNAMIC_STATE_RECORD_HPP
