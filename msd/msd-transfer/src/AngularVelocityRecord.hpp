#ifndef MSD_TRANSFER_ANGULAR_VELOCITY_RECORD_HPP
#define MSD_TRANSFER_ANGULAR_VELOCITY_RECORD_HPP

#include <limits>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Database record for a 3D angular velocity vector [rad/s]
 *
 * Stores the pitch, roll, yaw components of an msd_sim::AngularVelocity as
 * individual scalar doubles for human-readable database inspection and
 * queryability.
 */
struct AngularVelocityRecord : public cpp_sqlite::BaseTransferObject
{
  double pitch{std::numeric_limits<double>::quiet_NaN()};
  double roll{std::numeric_limits<double>::quiet_NaN()};
  double yaw{std::numeric_limits<double>::quiet_NaN()};
};

BOOST_DESCRIBE_STRUCT(AngularVelocityRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (pitch, roll, yaw));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_ANGULAR_VELOCITY_RECORD_HPP
