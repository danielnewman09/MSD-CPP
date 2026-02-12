// Ticket: vector_datatype_refactor
// Ticket: 0056i_static_asset_recording_and_fk
// Transfer record for InertialState

#ifndef MSD_TRANSFER_INERTIAL_STATE_RECORD_HPP
#define MSD_TRANSFER_INERTIAL_STATE_RECORD_HPP

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>

#include "msd-transfer/src/AccelerationRecord.hpp"
#include "msd-transfer/src/AngularAccelerationRecord.hpp"
#include "msd-transfer/src/AssetInertialStaticRecord.hpp"
#include "msd-transfer/src/CoordinateRecord.hpp"
#include "msd-transfer/src/QuaternionDRecord.hpp"
#include "msd-transfer/src/SimulationFrameRecord.hpp"
#include "msd-transfer/src/Vector3DRecord.hpp"
#include "msd-transfer/src/Vector4DRecord.hpp"
#include "msd-transfer/src/VelocityRecord.hpp"

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
 * - body: Foreign key to AssetInertialStaticRecord for body identification
 * - frame: Foreign key to SimulationFrameRecord for timestamping
 *
 * @see msd_sim::InertialState
 * @ticket 0038_simulation_data_recorder
 * @ticket 0056i_static_asset_recording_and_fk
 */
struct InertialStateRecord : public cpp_sqlite::BaseTransferObject
{
  CoordinateRecord position;
  VelocityRecord velocity;
  AccelerationRecord acceleration;
  QuaternionDRecord orientation;
  Vector4DRecord quaternionRate;
  AngularAccelerationRecord angularAcceleration;
  cpp_sqlite::ForeignKey<AssetInertialStaticRecord> body;
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(InertialStateRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (position,
                       velocity,
                       acceleration,
                       orientation,
                       quaternionRate,
                       angularAcceleration,
                       body,
                       frame));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_INERTIAL_STATE_RECORD_HPP
