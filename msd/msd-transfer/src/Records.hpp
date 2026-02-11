#ifndef MSD_TRANSFER_RECORDS_HPP
#define MSD_TRANSFER_RECORDS_HPP

/**
 * @file Records.hpp
 * @brief Convenience header including all database transfer objects
 *
 * This header provides a single include point for all msd-transfer records.
 * Use this when you need access to the complete database schema.
 */

#include "msd-transfer/src/AngularCoordinateRecord.hpp"
#include "msd-transfer/src/AngularRateRecord.hpp"
#include "msd-transfer/src/AppliedForceRecord.hpp"
#include "msd-transfer/src/BodyMetadataRecord.hpp"
#include "msd-transfer/src/ConstraintForceRecord.hpp"
#include "msd-transfer/src/ContactRecord.hpp"
#include "msd-transfer/src/CoordinateRecord.hpp"
#include "msd-transfer/src/EnergyRecord.hpp"
#include "msd-transfer/src/InertialStateRecord.hpp"
#include "msd-transfer/src/MaterialRecord.hpp"
#include "msd-transfer/src/MeshRecord.hpp"
#include "msd-transfer/src/PhysicsTemplateRecord.hpp"
#include "msd-transfer/src/SimulationFrameRecord.hpp"
#include "msd-transfer/src/SolverDiagnosticRecord.hpp"
#include "msd-transfer/src/SystemEnergyRecord.hpp"
#include "msd-transfer/src/Vector3DRecord.hpp"
#include "msd-transfer/src/Vector4DRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Type alias for cpp_sqlite Database
 *
 * Convenience alias to avoid repeating namespace qualification.
 */
using Database = cpp_sqlite::Database;

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_RECORDS_HPP
