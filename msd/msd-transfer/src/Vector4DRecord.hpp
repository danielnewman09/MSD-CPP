#ifndef MSD_TRANSFER_VECTOR4D_RECORD_HPP
#define MSD_TRANSFER_VECTOR4D_RECORD_HPP

#include <limits>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Database record for a generic 4D vector
 *
 * Stores the x, y, z, w components of an msd_sim::Vector4D as individual
 * scalar doubles for human-readable database inspection and queryability.
 *
 * Commonly used for:
 * - Quaternions (w, x, y, z ordering in physics)
 * - Homogeneous coordinates (x, y, z, w)
 * - RGBA colors
 */
struct Vector4DRecord : public cpp_sqlite::BaseTransferObject
{
  double x{std::numeric_limits<double>::quiet_NaN()};
  double y{std::numeric_limits<double>::quiet_NaN()};
  double z{std::numeric_limits<double>::quiet_NaN()};
  double w{std::numeric_limits<double>::quiet_NaN()};
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(Vector4DRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (x, y, z, w));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_VECTOR4D_RECORD_HPP
