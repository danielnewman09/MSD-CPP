// Ticket: 0056a_collision_force_transfer_records
// Per-contact collision data for visualization and replay

#ifndef MSD_TRANSFER_CONTACT_RECORD_HPP
#define MSD_TRANSFER_CONTACT_RECORD_HPP

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>
#include <limits>

#include "msd-transfer/src/SimulationFrameRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Per-contact-point collision data
 *
 * Records detailed contact geometry for collision visualization.
 * Up to 4 contacts per collision pair per frame.
 *
 * @ticket 0056a_collision_force_transfer_records
 */
struct ContactRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_a_id{0};     // Instance ID of body A
  uint32_t body_b_id{0};     // Instance ID of body B
  uint32_t contact_index{0}; // Contact index within manifold [0,3]

  double point_a_x{std::numeric_limits<double>::quiet_NaN()};
  double point_a_y{std::numeric_limits<double>::quiet_NaN()};
  double point_a_z{std::numeric_limits<double>::quiet_NaN()};

  double point_b_x{std::numeric_limits<double>::quiet_NaN()};
  double point_b_y{std::numeric_limits<double>::quiet_NaN()};
  double point_b_z{std::numeric_limits<double>::quiet_NaN()};

  double normal_x{std::numeric_limits<double>::quiet_NaN()};
  double normal_y{std::numeric_limits<double>::quiet_NaN()};
  double normal_z{std::numeric_limits<double>::quiet_NaN()};

  double depth{std::numeric_limits<double>::quiet_NaN()};
  double restitution{std::numeric_limits<double>::quiet_NaN()};
  double friction{std::numeric_limits<double>::quiet_NaN()};

  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(ContactRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_a_id,
                       body_b_id,
                       contact_index,
                       point_a_x,
                       point_a_y,
                       point_a_z,
                       point_b_x,
                       point_b_y,
                       point_b_z,
                       normal_x,
                       normal_y,
                       normal_z,
                       depth,
                       restitution,
                       friction,
                       frame));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_CONTACT_RECORD_HPP
