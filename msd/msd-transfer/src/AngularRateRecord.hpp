#ifndef MSD_TRANSFER_ANGULAR_RATE_RECORD_HPP
#define MSD_TRANSFER_ANGULAR_RATE_RECORD_HPP

#include <limits>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Database record for a 3D angular rate vector
 *
 * Stores the pitch, roll, yaw components of an msd_sim::AngularRate as
 * individual scalar doubles for human-readable database inspection and
 * queryability.
 *
 * Units depend on context:
 * - Angular velocity: rad/s
 * - Angular acceleration: rad/s²
 */
struct AngularRateRecord : public cpp_sqlite::BaseTransferObject
{
  double pitch{std::numeric_limits<double>::quiet_NaN()};
  double roll{std::numeric_limits<double>::quiet_NaN()};
  double yaw{std::numeric_limits<double>::quiet_NaN()};
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(AngularRateRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (pitch, roll, yaw));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_ANGULAR_RATE_RECORD_HPP
