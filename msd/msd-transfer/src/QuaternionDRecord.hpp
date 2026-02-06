#ifndef MSD_TRANSFER_QUATERNIOND_RECORD_HPP
#define MSD_TRANSFER_QUATERNIOND_RECORD_HPP

#include <limits>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Database record for a quaternion (double precision)
 *
 * Stores the w, x, y, z components of an msd_sim::QuaternionD as individual
 * scalar doubles for human-readable database inspection and queryability.
 *
 * Uses Eigen/Hamilton convention: q = w + xi + yj + zk
 * where w is the scalar (real) part and (x, y, z) is the vector (imaginary) part.
 */
struct QuaternionDRecord : public cpp_sqlite::BaseTransferObject
{
  double w{std::numeric_limits<double>::quiet_NaN()};
  double x{std::numeric_limits<double>::quiet_NaN()};
  double y{std::numeric_limits<double>::quiet_NaN()};
  double z{std::numeric_limits<double>::quiet_NaN()};
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(QuaternionDRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (w, x, y, z));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_QUATERNIOND_RECORD_HPP
