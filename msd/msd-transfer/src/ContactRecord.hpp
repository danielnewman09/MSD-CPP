// Ticket: 0056a_collision_force_transfer_records
// Per-contact collision data for visualization and replay

#ifndef MSD_TRANSFER_CONTACT_RECORD_HPP
#define MSD_TRANSFER_CONTACT_RECORD_HPP

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>
#include <limits>

#include "msd-transfer/src/CoordinateRecord.hpp"
#include "msd-transfer/src/SimulationFrameRecord.hpp"
#include "msd-transfer/src/Vector3DRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Per-contact-point collision data
 *
 * Records detailed contact geometry for collision visualization.
 * Up to 4 contacts per collision pair per frame.
 *
 * Uses composed sub-records (CoordinateRecord, Vector3DRecord) instead
 * of flat scalars for consistency with ExternalForceRecord pattern.
 *
 * Per-body material properties (restitution, friction) are recorded
 * separately in AssetInertialStaticRecord.
 *
 * @ticket 0056a_collision_force_transfer_records
 */
struct ContactRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_a_id{0};     // Instance ID of body A
  uint32_t body_b_id{0};     // Instance ID of body B
  uint32_t contact_index{0}; // Contact index within manifold [0,3]

  CoordinateRecord pointA;   // Contact point on body A (world space)
  CoordinateRecord pointB;   // Contact point on body B (world space)
  Vector3DRecord normal;     // Contact normal (A → B)

  double depth{std::numeric_limits<double>::quiet_NaN()};

  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(ContactRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_a_id,
                       body_b_id,
                       contact_index,
                       pointA,
                       pointB,
                       normal,
                       depth,
                       frame));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_CONTACT_RECORD_HPP
