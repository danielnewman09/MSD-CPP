#ifndef MSD_TRANSFER_ASSET_DYNAMIC_STATE_RECORD_HPP
#define MSD_TRANSFER_ASSET_DYNAMIC_STATE_RECORD_HPP

#include <cstdint>
#include <limits>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>

#include "msd-transfer/src/AngularRateRecord.hpp"
#include "msd-transfer/src/CoordinateRecord.hpp"
#include "msd-transfer/src/ForceVectorRecord.hpp"
#include "msd-transfer/src/QuaternionDRecord.hpp"
#include "msd-transfer/src/SimulationFrameRecord.hpp"
#include "msd-transfer/src/TorqueVectorRecord.hpp"
#include "msd-transfer/src/Vector3DRecord.hpp"
#include "msd-transfer/src/Vector4DRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Complete per-frame dynamic state of an AssetInertial
 *
 * Captures the full kinematic state (position, velocity, acceleration,
 * orientation, quaternion rate, angular acceleration) plus accumulated
 * force and torque for a single body at a single simulation frame.
 *
 * Combines InertialState kinematic data with force/torque accumulators
 * into a single record per body per frame.
 *
 * @see msd_sim::AssetInertial
 * @see msd_sim::InertialState
 */
struct AssetDynamicStateRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_id{0};

  // Kinematic state (from InertialState)
  CoordinateRecord position;
  Vector3DRecord velocity;
  Vector3DRecord acceleration;
  QuaternionDRecord orientation;
  Vector4DRecord quaternionRate;
  AngularRateRecord angularAcceleration;

  // Force/torque accumulators
  ForceVectorRecord accumulatedForce;
  TorqueVectorRecord accumulatedTorque;

  // Frame FK for temporal association
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(AssetDynamicStateRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_id,
                       position,
                       velocity,
                       acceleration,
                       orientation,
                       quaternionRate,
                       angularAcceleration,
                       accumulatedForce,
                       accumulatedTorque,
                       frame));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_ASSET_DYNAMIC_STATE_RECORD_HPP
