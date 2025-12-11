#ifndef MSD_SIM_PHYSICS_FORCE_ACCUMULATOR_HPP
#define MSD_SIM_PHYSICS_FORCE_ACCUMULATOR_HPP

#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Physics/Force.hpp"
#include <vector>

namespace msd_sim {

/**
 * @brief Accumulates forces acting on a rigid body and computes net effects.
 *
 * The ForceAccumulator collects all forces applied to a rigid body during
 * a simulation step, then computes the net force (for linear acceleration)
 * and net torque (for angular acceleration) about the center of mass.
 */
class ForceAccumulator
{
public:
  /**
   * @brief Default constructor.
   */
  ForceAccumulator();

  /**
   * @brief Add a force to the accumulator.
   *
   * @param force The force to add
   */
  void addForce(const Force& force);

  /**
   * @brief Add multiple forces to the accumulator.
   *
   * @param forces Vector of forces to add
   */
  void addForces(const std::vector<Force>& forces);

  /**
   * @brief Remove all accumulated forces.
   */
  void clear();

  /**
   * @brief Get the number of accumulated forces.
   *
   * @return Number of forces currently stored
   */
  size_t getForceCount() const;

  /**
   * @brief Get all accumulated forces.
   *
   * @return Const reference to the vector of forces
   */
  const std::vector<Force>& getForces() const;

  /**
   * @brief Calculate the net force vector (sum of all force vectors).
   *
   * F_net = Σ F_i
   *
   * @return The net force vector in Newtons [N]
   */
  Coordinate calculateNetForce() const;

  /**
   * @brief Calculate the net torque about a reference point (typically center
   * of mass).
   *
   * τ_net = Σ (r_i × F_i)
   * where r_i is the vector from the reference point to the application point
   * of force i.
   *
   * @param referencePoint The point about which to calculate torque
   *                       (usually center of mass)
   * @return The net torque vector in Newton-meters [N⋅m]
   */
  Coordinate calculateNetTorque(const Coordinate& referencePoint) const;

  /**
   * @brief Calculate both net force and net torque in a single pass.
   *
   * More efficient than calling calculateNetForce() and calculateNetTorque()
   * separately when both are needed.
   *
   * @param referencePoint The point about which to calculate torque
   * @param outNetForce Output parameter for net force [N]
   * @param outNetTorque Output parameter for net torque [N⋅m]
   */
  void calculateNetForceAndTorque(const Coordinate& referencePoint,
                                   Coordinate& outNetForce,
                                   Coordinate& outNetTorque) const;

private:
  std::vector<Force> forces_;  // Collection of all applied forces
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_FORCE_ACCUMULATOR_HPP
