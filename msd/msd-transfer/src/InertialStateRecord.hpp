#ifndef MSD_TRANSFER_INERTIAL_STATE_RECORD_HPP
#define MSD_TRANSFER_INERTIAL_STATE_RECORD_HPP

#include <cstdint>
#include <limits>
#include <vector>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Database record for a complete kinematic state
 *
 * Stores the 14-component state vector of an msd_sim::InertialState:
 * - Linear components: position, velocity, acceleration (3 doubles each)
 * - Quaternion orientation (4 doubles as BLOB)
 * - Quaternion rate (4 doubles as BLOB)
 * - Angular acceleration (3 doubles)
 *
 * All linear values are in meters [m], meters per second [m/s],
 * or meters per second squared [m/s²].
 * All angular values are in radians or radians per second.
 *
 * @ticket 0030_lagrangian_quaternion_physics
 */
struct InertialStateRecord : public cpp_sqlite::BaseTransferObject
{
  // Linear position [m]
  double position_x{std::numeric_limits<double>::quiet_NaN()};
  double position_y{std::numeric_limits<double>::quiet_NaN()};
  double position_z{std::numeric_limits<double>::quiet_NaN()};

  // Linear velocity [m/s]
  double velocity_x{std::numeric_limits<double>::quiet_NaN()};
  double velocity_y{std::numeric_limits<double>::quiet_NaN()};
  double velocity_z{std::numeric_limits<double>::quiet_NaN()};

  // Linear acceleration [m/s²]
  double acceleration_x{std::numeric_limits<double>::quiet_NaN()};
  double acceleration_y{std::numeric_limits<double>::quiet_NaN()};
  double acceleration_z{std::numeric_limits<double>::quiet_NaN()};

  // Quaternion orientation (x, y, z, w) as BLOB [32 bytes]
  std::vector<uint8_t> orientation_quat{};

  // Quaternion rate Q̇ (x, y, z, w) as BLOB [32 bytes]
  std::vector<uint8_t> quaternion_rate{};

  // Angular acceleration [rad/s²]
  double angular_accel_pitch{std::numeric_limits<double>::quiet_NaN()};
  double angular_accel_roll{std::numeric_limits<double>::quiet_NaN()};
  double angular_accel_yaw{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace msd_transfer

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(
  msd_transfer::InertialStateRecord,
  (cpp_sqlite::BaseTransferObject),
  (position_x,
   position_y,
   position_z,
   velocity_x,
   velocity_y,
   velocity_z,
   acceleration_x,
   acceleration_y,
   acceleration_z,
   orientation_quat,
   quaternion_rate,
   angular_accel_pitch,
   angular_accel_roll,
   angular_accel_yaw));

#endif  // MSD_TRANSFER_INERTIAL_STATE_RECORD_HPP
