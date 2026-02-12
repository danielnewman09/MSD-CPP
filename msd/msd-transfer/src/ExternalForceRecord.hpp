#ifndef MSD_TRANSFER_EXTERNAL_FORCE_RECORD_HPP
#define MSD_TRANSFER_EXTERNAL_FORCE_RECORD_HPP

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

#include "msd-transfer/src/CoordinateRecord.hpp"
#include "msd-transfer/src/ForceVectorRecord.hpp"
#include "msd-transfer/src/TorqueVectorRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Database record bundling force and torque as composed sub-records
 *
 * Groups a ForceVectorRecord and TorqueVectorRecord into a single composite
 * record. Used by AssetDynamicStateRecord to represent the external
 * force/torque accumulator for a rigid body.
 *
 * @see msd_sim::ExternalForce
 */
struct ExternalForceRecord : public cpp_sqlite::BaseTransferObject
{
  ForceVectorRecord force;
  TorqueVectorRecord torque;
  CoordinateRecord applicationPoint;
};

BOOST_DESCRIBE_STRUCT(ExternalForceRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (force, torque, applicationPoint));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_EXTERNAL_FORCE_RECORD_HPP
