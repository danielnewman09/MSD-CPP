// Ticket: 0035d_friction_hardening_and_validation
// Design: docs/designs/0035d_friction_hardening_and_validation/design.md

#ifndef MSD_SIM_TEST_ENERGY_MONITOR_HPP
#define MSD_SIM_TEST_ENERGY_MONITOR_HPP

#include <vector>
#include <Eigen/Dense>
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

/**
 * @brief Compute kinetic energy for test assertions validating M6 energy dissipation
 *
 * Static-only utility class providing energy computation for integration tests.
 * Computes linear and angular kinetic energy from inertial states and validates
 * that friction never injects energy (E(t+1) <= E(t)).
 *
 * Thread safety: Pure static functions, thread-safe
 * Error handling: No exceptions (returns bool for validation)
 *
 * @see docs/designs/0035d_friction_hardening_and_validation/design.md
 * @ticket 0035d_friction_hardening_and_validation
 */
class EnergyMonitor
{
public:
  /**
   * @brief Compute linear kinetic energy: 0.5 * sum(m_i * v_i^2)
   *
   * @param states Inertial states for all bodies
   * @param masses Mass for each body [kg]
   * @return Total linear kinetic energy [J]
   */
  static double computeLinearKE(
      const std::vector<InertialState>& states,
      const std::vector<double>& masses);

  /**
   * @brief Compute angular kinetic energy: 0.5 * sum(omega_i^T * I_i * omega_i)
   *
   * @param states Inertial states for all bodies
   * @param inertias Inertia tensor for each body [kg*m^2]
   * @return Total angular kinetic energy [J]
   */
  static double computeAngularKE(
      const std::vector<InertialState>& states,
      const std::vector<Eigen::Matrix3d>& inertias);

  /**
   * @brief Compute total kinetic energy (linear + angular)
   *
   * @param states Inertial states for all bodies
   * @param masses Mass for each body [kg]
   * @param inertias Inertia tensor for each body [kg*m^2]
   * @return Total kinetic energy [J]
   */
  static double computeTotalKE(
      const std::vector<InertialState>& states,
      const std::vector<double>& masses,
      const std::vector<Eigen::Matrix3d>& inertias);

  /**
   * @brief Validate energy is non-increasing
   *
   * Returns true if E_after <= E_before + tolerance.
   * Friction should dissipate energy, never inject it.
   *
   * @param E_before Energy before timestep [J]
   * @param E_after Energy after timestep [J]
   * @param tolerance Numerical tolerance (default 1e-6) [J]
   * @return true if energy decreased or stayed constant
   */
  static bool validateNonIncreasing(
      double E_before,
      double E_after,
      double tolerance = 1e-6);

  // Static-only utility (deleted special members)
  ~EnergyMonitor() = delete;
  EnergyMonitor() = delete;
  EnergyMonitor(const EnergyMonitor&) = delete;
  EnergyMonitor(EnergyMonitor&&) = delete;
  EnergyMonitor& operator=(const EnergyMonitor&) = delete;
  EnergyMonitor& operator=(EnergyMonitor&&) = delete;
};

}  // namespace msd_sim

#endif  // MSD_SIM_TEST_ENERGY_MONITOR_HPP
