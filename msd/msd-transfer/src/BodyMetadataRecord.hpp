// Ticket: 0056a_collision_force_transfer_records
// Per-body static metadata (recorded once at spawn)

#ifndef MSD_TRANSFER_BODY_METADATA_RECORD_HPP
#define MSD_TRANSFER_BODY_METADATA_RECORD_HPP

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <limits>

namespace msd_transfer
{

/**
 * @brief Per-body static metadata
 *
 * Records static properties of bodies at spawn time.
 * Unlike other records, this is NOT per-frame — no ForeignKey to SimulationFrameRecord.
 *
 * @ticket 0056a_collision_force_transfer_records
 */
struct BodyMetadataRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_id{0};
  uint32_t asset_id{0};
  double mass{std::numeric_limits<double>::quiet_NaN()};
  double restitution{std::numeric_limits<double>::quiet_NaN()};
  double friction{std::numeric_limits<double>::quiet_NaN()};
  uint32_t is_environment{0}; // Boolean as uint32_t for SQLite

  // No frame FK — recorded once, not per-frame
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(BodyMetadataRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_id,
                       asset_id,
                       mass,
                       restitution,
                       friction,
                       is_environment));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_BODY_METADATA_RECORD_HPP
