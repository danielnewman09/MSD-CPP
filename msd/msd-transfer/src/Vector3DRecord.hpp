#ifndef MSD_TRANSFER_VECTOR3D_RECORD_HPP
#define MSD_TRANSFER_VECTOR3D_RECORD_HPP

#include <limits>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Database record for a generic 3D vector
 *
 * Stores the x, y, z components of an msd_sim::Vector3D as individual
 * scalar doubles for human-readable database inspection and queryability.
 *
 * Use this record for generic 3D vector data. For domain-specific types
 * (positions, velocities), use their respective records (CoordinateRecord, etc.).
 */
struct Vector3DRecord : public cpp_sqlite::BaseTransferObject
{
  double x{std::numeric_limits<double>::quiet_NaN()};
  double y{std::numeric_limits<double>::quiet_NaN()};
  double z{std::numeric_limits<double>::quiet_NaN()};
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(Vector3DRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (x, y, z));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_VECTOR3D_RECORD_HPP
