#ifndef MSD_TRANSFER_FORCE_VECTOR_RECORD_HPP
#define MSD_TRANSFER_FORCE_VECTOR_RECORD_HPP

#include <limits>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Database record for a 3D force vector [N]
 *
 * Stores the x, y, z components of an msd_sim::ForceVector as individual
 * scalar doubles for human-readable database inspection and queryability.
 */
struct ForceVectorRecord : public cpp_sqlite::BaseTransferObject
{
  double x{std::numeric_limits<double>::quiet_NaN()};
  double y{std::numeric_limits<double>::quiet_NaN()};
  double z{std::numeric_limits<double>::quiet_NaN()};
};

BOOST_DESCRIBE_STRUCT(ForceVectorRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (x, y, z));

}  // namespace msd_transfer

#endif  // MSD_TRANSFER_FORCE_VECTOR_RECORD_HPP
