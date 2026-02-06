// Ticket: vector_datatype_refactor
// Transfer record for InertialState

#ifndef MSD_TRANSFER_INERTIAL_STATE_RECORD_HPP
#define MSD_TRANSFER_INERTIAL_STATE_RECORD_HPP

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

#include "msd-transfer/src/AngularRateRecord.hpp"
#include "msd-transfer/src/CoordinateRecord.hpp"
#include "msd-transfer/src/QuaternionDRecord.hpp"
#include "msd-transfer/src/Vector3DRecord.hpp"
#include "msd-transfer/src/Vector4DRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Database record for complete kinematic state
 *
 * Stores the 14-component state vector (X, Q, Ẋ, Q̇) plus acceleration
 * for an msd_sim::InertialState. Uses composition of existing record types
 * for each member.
 *
 * Components:
 * - position: 3D spatial coordinate [m]
 * - velocity: Linear velocity [m/s]
 * - acceleration: Linear acceleration [m/s²]
 * - orientation: Unit quaternion (w, x, y, z)
 * - quaternionRate: Quaternion time derivative Q̇
 * - angularAcceleration: Angular acceleration [rad/s²]
 *
 * @see msd_sim::InertialState
 */
struct InertialStateRecord : public cpp_sqlite::BaseTransferObject
{
  CoordinateRecord position;
  Vector3DRecord velocity;
  Vector3DRecord acceleration;
  QuaternionDRecord orientation;
  Vector4DRecord quaternionRate;
  AngularRateRecord angularAcceleration;
};

}  // namespace msd_transfer

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(msd_transfer::InertialStateRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (position,
                       velocity,
                       acceleration,
                       orientation,
                       quaternionRate,
                       angularAcceleration));

#endif  // MSD_TRANSFER_INERTIAL_STATE_RECORD_HPP
