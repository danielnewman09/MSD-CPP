// Ticket: 0038_simulation_data_recorder
// Design: docs/designs/0038_simulation_data_recorder/design.md

#ifndef MSD_TRANSFER_SIMULATION_FRAME_RECORD_HPP
#define MSD_TRANSFER_SIMULATION_FRAME_RECORD_HPP

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Database record for simulation frame timestamping
 *
 * Each simulation frame has exactly one SimulationFrameRecord.
 * Per-frame records (InertialStateRecord, etc.) reference this
 * frame via ForeignKey<SimulationFrameRecord> for temporal association.
 *
 * The id field is inherited from BaseTransferObject and serves as the
 * primary key for frame identification.
 *
 * @see docs/designs/0038_simulation_data_recorder/0038_simulation_data_recorder.puml
 * @ticket 0038_simulation_data_recorder
 */
struct SimulationFrameRecord : public cpp_sqlite::BaseTransferObject
{
  double simulation_time{0.0};  // Simulation time [seconds]
  double wall_clock_time{0.0};  // Wall clock time [seconds since epoch]
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(SimulationFrameRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (simulation_time, wall_clock_time));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_SIMULATION_FRAME_RECORD_HPP
