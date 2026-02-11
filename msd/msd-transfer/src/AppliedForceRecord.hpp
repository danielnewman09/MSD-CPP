// Ticket: 0056a_collision_force_transfer_records
// Per-body applied forces (gravity, external, potential fields)

#ifndef MSD_TRANSFER_APPLIED_FORCE_RECORD_HPP
#define MSD_TRANSFER_APPLIED_FORCE_RECORD_HPP

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>
#include <limits>

#include "msd-transfer/src/SimulationFrameRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Per-body applied forces
 *
 * Records forces applied to bodies from gravity, external forces,
 * and potential energy fields.
 *
 * Force types:
 * - 0: Gravity
 * - 1: External force
 * - 2: Potential energy field
 *
 * @ticket 0056a_collision_force_transfer_records
 */
struct AppliedForceRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_id{0};
  uint32_t force_type{0}; // 0=gravity, 1=external, 2=potential

  double force_x{0.0};
  double force_y{0.0};
  double force_z{0.0};

  double torque_x{0.0};
  double torque_y{0.0};
  double torque_z{0.0};

  double point_x{std::numeric_limits<double>::quiet_NaN()};
  double point_y{std::numeric_limits<double>::quiet_NaN()};
  double point_z{std::numeric_limits<double>::quiet_NaN()};

  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(AppliedForceRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_id,
                       force_type,
                       force_x,
                       force_y,
                       force_z,
                       torque_x,
                       torque_y,
                       torque_z,
                       point_x,
                       point_y,
                       point_z,
                       frame));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_APPLIED_FORCE_RECORD_HPP
