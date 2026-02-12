// Ticket: 0056a_collision_force_transfer_records
// Per-body physics properties (recorded once at spawn)

#ifndef MSD_TRANSFER_ASSET_INERTIAL_STATIC_RECORD_HPP
#define MSD_TRANSFER_ASSET_INERTIAL_STATIC_RECORD_HPP

#include <cstdint>
#include <limits>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Per-body user-set physics properties from AssetInertial
 *
 * Records the physics properties (mass, restitution, friction) of an
 * inertial asset at spawn time.
 *
 * Unlike per-frame records, this has NO ForeignKey to SimulationFrameRecord.
 *
 * @see msd_sim::AssetInertial
 * @ticket 0056a_collision_force_transfer_records
 */
struct AssetInertialStaticRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_id{0};
  double mass{std::numeric_limits<double>::quiet_NaN()};
  double restitution{std::numeric_limits<double>::quiet_NaN()};
  double friction{std::numeric_limits<double>::quiet_NaN()};
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(AssetInertialStaticRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_id, mass, restitution, friction));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_ASSET_INERTIAL_STATIC_RECORD_HPP
