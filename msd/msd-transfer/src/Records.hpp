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
#include "msd-transfer/src/CoordinateRecord.hpp"
#include "msd-transfer/src/MaterialRecord.hpp"
#include "msd-transfer/src/MeshRecord.hpp"
#include "msd-transfer/src/PhysicsTemplateRecord.hpp"

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
