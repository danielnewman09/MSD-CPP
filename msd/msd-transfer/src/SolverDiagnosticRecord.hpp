// Ticket: 0056a_collision_force_transfer_records
// Per-frame solver diagnostics for convergence debugging

#ifndef MSD_TRANSFER_SOLVER_DIAGNOSTIC_RECORD_HPP
#define MSD_TRANSFER_SOLVER_DIAGNOSTIC_RECORD_HPP

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>
#include <limits>

#include "msd-transfer/src/SimulationFrameRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Per-frame solver diagnostics
 *
 * Records constraint solver statistics for debugging convergence issues
 * and analyzing solver performance.
 *
 * @ticket 0056a_collision_force_transfer_records
 */
struct SolverDiagnosticRecord : public cpp_sqlite::BaseTransferObject
{
  uint32_t iterations{0};
  double residual{std::numeric_limits<double>::quiet_NaN()};
  uint32_t converged{0};      // Boolean as uint32_t for SQLite
  uint32_t num_constraints{0};
  uint32_t num_contacts{0};

  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(SolverDiagnosticRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (iterations,
                       residual,
                       converged,
                       num_constraints,
                       num_contacts,
                       frame));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_SOLVER_DIAGNOSTIC_RECORD_HPP
