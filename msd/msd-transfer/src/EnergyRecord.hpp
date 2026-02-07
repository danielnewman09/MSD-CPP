// Ticket: 0039a_energy_tracking_diagnostic_infrastructure

#ifndef MSD_TRANSFER_ENERGY_RECORD_HPP
#define MSD_TRANSFER_ENERGY_RECORD_HPP

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>

#include "msd-transfer/src/SimulationFrameRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Database record for per-body energy values
 *
 * Stores computed energy components for a single rigid body at a given
 * simulation frame. Uses ForeignKey<SimulationFrameRecord> for temporal
 * association with the recording frame.
 *
 * Energy components:
 * - linear_ke: Linear kinetic energy KE = 0.5 * m * v^2 [J]
 * - rotational_ke: Rotational kinetic energy KE = 0.5 * omega^T * I_world * omega [J]
 * - potential_e: Gravitational potential energy PE = -m * (g . r) [J]
 * - total_e: Total mechanical energy E = KE_lin + KE_rot + PE [J]
 *
 * @see msd_sim::EnergyTracker
 * @ticket 0039a_energy_tracking_diagnostic_infrastructure
 */
struct EnergyRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t body_id{0};       // Stable body identifier (AssetInertial::getInstanceId())
  double linear_ke{0.0};     // Linear kinetic energy [J]
  double rotational_ke{0.0}; // Rotational kinetic energy [J]
  double potential_e{0.0};   // Gravitational potential energy [J]
  double total_e{0.0};       // Total mechanical energy [J]
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(EnergyRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_id,
                       linear_ke,
                       rotational_ke,
                       potential_e,
                       total_e,
                       frame));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_ENERGY_RECORD_HPP
