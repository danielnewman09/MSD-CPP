#ifndef MSD_TRANSFER_ANGULAR_COORDINATE_RECORD_HPP
#define MSD_TRANSFER_ANGULAR_COORDINATE_RECORD_HPP

#include <limits>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Database record for a 3D angular coordinate (orientation angles)
 *
 * Stores the pitch, roll, yaw components of an msd_sim::AngularCoordinate as
 * individual scalar doubles for human-readable database inspection and
 * queryability.
 *
 * All angles are in radians.
 *
 * Axis convention (ZYX intrinsic rotation):
 * - pitch: Rotation around Y-axis
 * - roll:  Rotation around X-axis
 * - yaw:   Rotation around Z-axis
 */
struct AngularCoordinateRecord : public cpp_sqlite::BaseTransferObject
{
  double pitch{std::numeric_limits<double>::quiet_NaN()};
  double roll{std::numeric_limits<double>::quiet_NaN()};
  double yaw{std::numeric_limits<double>::quiet_NaN()};
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(AngularCoordinateRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (pitch, roll, yaw));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_ANGULAR_COORDINATE_RECORD_HPP
