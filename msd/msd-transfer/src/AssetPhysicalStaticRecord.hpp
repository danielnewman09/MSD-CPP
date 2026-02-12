// Ticket: 0056a_collision_force_transfer_records
// Per-body identity metadata (recorded once at spawn)

#ifndef MSD_TRANSFER_ASSET_PHYSICAL_STATIC_RECORD_HPP
#define MSD_TRANSFER_ASSET_PHYSICAL_STATIC_RECORD_HPP

#include <cstdint>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Per-body identity metadata from AssetPhysical
 *
 * Records the identity of a physical asset at spawn time:
 * instance ID, reference asset ID, and whether it is an environment object.
 *
 * Unlike per-frame records, this has NO ForeignKey to SimulationFrameRecord.
 *
 * @see msd_sim::AssetPhysical
 * @ticket 0056a_collision_force_transfer_records
 */
struct AssetPhysicalStaticRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_id{0};
  uint32_t asset_id{0};
  uint32_t is_environment{0};  // Boolean as uint32_t for SQLite
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(AssetPhysicalStaticRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_id, asset_id, is_environment));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_ASSET_PHYSICAL_STATIC_RECORD_HPP
