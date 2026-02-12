// Ticket: 0056a_collision_force_transfer_records
// Full collision pair record with RepeatedField contacts

#ifndef MSD_TRANSFER_COLLISION_RESULT_RECORD_HPP
#define MSD_TRANSFER_COLLISION_RESULT_RECORD_HPP

#include <cstdint>
#include <limits>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBRepeatedFieldTransferObject.hpp>

#include "msd-transfer/src/ContactPointRecord.hpp"
#include "msd-transfer/src/SimulationFrameRecord.hpp"
#include "msd-transfer/src/Vector3DRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Complete collision result record for a body pair at one frame
 *
 * Captures collision geometry (normal, penetration depth) and a manifold
 * of up to 4 contact points via a RepeatedField of ContactPointRecord.
 *
 * Body/frame context is stored here; individual ContactPointRecords
 * are pure geometry sub-records.
 *
 * @ticket 0056a_collision_force_transfer_records
 */
struct CollisionResultRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_a_id{0};
  uint32_t body_b_id{0};

  Vector3DRecord normal;
  double penetrationDepth{std::numeric_limits<double>::quiet_NaN()};

  cpp_sqlite::RepeatedFieldTransferObject<ContactPointRecord> contacts;

  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(CollisionResultRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_a_id,
                       body_b_id,
                       normal,
                       penetrationDepth,
                       contacts,
                       frame));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_COLLISION_RESULT_RECORD_HPP
