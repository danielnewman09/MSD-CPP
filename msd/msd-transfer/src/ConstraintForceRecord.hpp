// Ticket: 0056a_collision_force_transfer_records
// Per-body solved constraint forces from collision solver

#ifndef MSD_TRANSFER_CONSTRAINT_FORCE_RECORD_HPP
#define MSD_TRANSFER_CONSTRAINT_FORCE_RECORD_HPP

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>

#include "msd-transfer/src/SimulationFrameRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Per-body constraint forces from solver
 *
 * Records the net constraint force and torque applied to each body
 * by the constraint solver (output of collision resolution).
 *
 * @ticket 0056a_collision_force_transfer_records
 */
struct ConstraintForceRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_id{0};

  double force_x{0.0};
  double force_y{0.0};
  double force_z{0.0};

  double torque_x{0.0};
  double torque_y{0.0};
  double torque_z{0.0};

  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(ConstraintForceRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_id,
                       force_x,
                       force_y,
                       force_z,
                       torque_x,
                       torque_y,
                       torque_z,
                       frame));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_CONSTRAINT_FORCE_RECORD_HPP
