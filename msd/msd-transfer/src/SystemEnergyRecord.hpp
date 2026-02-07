// Ticket: 0039a_energy_tracking_diagnostic_infrastructure

#ifndef MSD_TRANSFER_SYSTEM_ENERGY_RECORD_HPP
#define MSD_TRANSFER_SYSTEM_ENERGY_RECORD_HPP

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>

#include "msd-transfer/src/SimulationFrameRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Database record for system-level energy summary
 *
 * Stores aggregated energy values across all rigid bodies for a given
 * simulation frame. Includes energy change detection for anomaly
 * identification during collision debugging.
 *
 * Fields:
 * - total_linear_ke: Sum of all body linear kinetic energies [J]
 * - total_rotational_ke: Sum of all body rotational kinetic energies [J]
 * - total_potential_e: Sum of all body potential energies [J]
 * - total_system_e: Total system mechanical energy [J]
 * - delta_e: Energy change from previous frame [J]
 * - energy_injection: 1 if delta_e exceeds tolerance (anomaly flag)
 * - collision_active: 1 if any collision detected this frame
 *
 * @note energy_injection and collision_active use uint32_t for SQLite
 * compatibility (SQLite has no native boolean type).
 *
 * @see msd_sim::EnergyTracker
 * @ticket 0039a_energy_tracking_diagnostic_infrastructure
 */
struct SystemEnergyRecord : public cpp_sqlite::BaseTransferObject
{
  double total_linear_ke{0.0};     // Sum of all body linear KE [J]
  double total_rotational_ke{0.0}; // Sum of all body rotational KE [J]
  double total_potential_e{0.0};   // Sum of all body PE [J]
  double total_system_e{0.0};      // Total system energy [J]
  double delta_e{0.0};             // Change from previous frame [J]
  uint32_t energy_injection{0};    // 1 = anomaly detected (uint32_t for SQLite)
  uint32_t collision_active{0};    // 1 = collision this frame (uint32_t for SQLite)
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(SystemEnergyRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (total_linear_ke,
                       total_rotational_ke,
                       total_potential_e,
                       total_system_e,
                       delta_e,
                       energy_injection,
                       collision_active,
                       frame));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_SYSTEM_ENERGY_RECORD_HPP
