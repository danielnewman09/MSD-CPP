// Ticket: 0056a_collision_force_transfer_records
// Per-contact geometry sub-record for CollisionResultRecord

#ifndef MSD_TRANSFER_CONTACT_POINT_RECORD_HPP
#define MSD_TRANSFER_CONTACT_POINT_RECORD_HPP

#include <limits>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

#include "msd-transfer/src/CoordinateRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Per-contact-point geometry sub-record
 *
 * Pure geometry record storing contact locations on both bodies.
 * Body/frame context lives in the parent CollisionResultRecord.
 *
 * Replaces ContactRecord which had body IDs, frame FK, and
 * contact_index baked in as a standalone per-row record.
 *
 * @ticket 0056a_collision_force_transfer_records
 */
struct ContactPointRecord : public cpp_sqlite::BaseTransferObject
{
  CoordinateRecord pointA;  // Contact point on body A (world space)
  CoordinateRecord pointB;  // Contact point on body B (world space)
  double depth{std::numeric_limits<double>::quiet_NaN()};
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(ContactPointRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (pointA, pointB, depth));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_CONTACT_POINT_RECORD_HPP
