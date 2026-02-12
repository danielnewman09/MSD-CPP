#ifndef MSD_SIM_DATATYPES_EXTERNAL_FORCE_HPP
#define MSD_SIM_DATATYPES_EXTERNAL_FORCE_HPP

#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/ForceVector.hpp"
#include "msd-sim/src/DataTypes/TorqueVector.hpp"
#include "msd-transfer/src/ExternalForceRecord.hpp"

namespace msd_sim
{

/**
 * @brief A single external force contribution applied to a rigid body.
 *
 * Stores the linear force, resulting torque, and the world-space
 * application point. Multiple ExternalForce entries are collected
 * per frame; use accumulate() to compute the net result.
 */
struct ExternalForce
{
  ForceVector force{0.0, 0.0, 0.0};
  TorqueVector torque{0.0, 0.0, 0.0};
  Coordinate applicationPoint{0.0, 0.0, 0.0};

  /**
   * @brief Convert to a transfer record for database persistence.
   */
  [[nodiscard]] msd_transfer::ExternalForceRecord toRecord() const
  {
    msd_transfer::ExternalForceRecord record;
    record.force = force.toRecord();
    record.torque = torque.toRecord();
    record.applicationPoint = applicationPoint.toRecord();
    return record;
  }

  /**
   * @brief Reconstruct from a transfer record.
   */
  static ExternalForce fromRecord(
    const msd_transfer::ExternalForceRecord& record)
  {
    ExternalForce ef;
    ef.force = ForceVector::fromRecord(record.force);
    ef.torque = TorqueVector::fromRecord(record.torque);
    ef.applicationPoint = Coordinate::fromRecord(record.applicationPoint);
    return ef;
  }

  /**
   * @brief Sum all individual forces into a single net result.
   *
   * @param forces Vector of individual force contributions
   * @return ExternalForce with summed force/torque and mean application point
   */
  static ExternalForce accumulate(const std::vector<ExternalForce>& forces)
  {
    ExternalForce result;
    if (forces.empty())
    {
      return result;
    }

    Coordinate pointSum{0.0, 0.0, 0.0};
    for (const auto& f : forces)
    {
      result.force += f.force;
      result.torque += f.torque;
      pointSum += f.applicationPoint;
    }
    result.applicationPoint =
      pointSum / static_cast<double>(forces.size());
    return result;
  }
};

}  // namespace msd_sim

#endif  // MSD_SIM_DATATYPES_EXTERNAL_FORCE_HPP
