// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md

#ifndef MSD_SIM_PHYSICS_POTENTIAL_ENERGY_HPP
#define MSD_SIM_PHYSICS_POTENTIAL_ENERGY_HPP

#include <Eigen/Dense>
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

/**
 * @brief Abstract interface for environmental potential energy fields
 *
 * This interface enables extensible potential energy computation for rigid body
 * dynamics. Implementations compute generalized forces from energy gradients.
 *
 * Lagrangian formulation: L = T - V where T is kinetic energy, V is potential
 * energy Generalized forces: F = -∂V/∂X  (linear force from position gradient)
 *                     τ = -∂V/∂Q  (torque from orientation gradient)
 *
 * Note: These are ENVIRONMENTAL potentials (gravity, magnetic fields) that
 * apply uniformly to all objects. Per-object forces (thrusters, springs) would
 * be handled separately in AssetInertial.
 *
 * Thread safety: Read-only methods after construction (thread-safe)
 *
 * @see
 * docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml
 * @ticket 0030_lagrangian_quaternion_physics
 */
class PotentialEnergy
{
public:
  virtual ~PotentialEnergy() = default;

  /**
   * @brief Compute linear force from potential energy gradient
   * @param state Current inertial state
   * @param mass Object mass [kg]
   * @return Generalized force F = -∂V/∂X [N]
   */
  virtual Coordinate computeForce(const InertialState& state,
                                  double mass) const = 0;

  /**
   * @brief Compute torque from potential energy gradient
   * @param state Current inertial state
   * @param inertia Inertia tensor in world frame [kg·m²]
   * @return Generalized torque τ = -∂V/∂Q [N·m]
   */
  virtual Coordinate computeTorque(const InertialState& state,
                                   const Eigen::Matrix3d& inertia) const = 0;

  /**
   * @brief Compute potential energy
   * @param state Current inertial state
   * @param mass Object mass [kg]
   * @return Potential energy V [J]
   */
  virtual double computeEnergy(const InertialState& state,
                               double mass) const = 0;

protected:
  PotentialEnergy() = default;
  PotentialEnergy(const PotentialEnergy&) = default;
  PotentialEnergy& operator=(const PotentialEnergy&) = default;
  PotentialEnergy(PotentialEnergy&&) noexcept = default;
  PotentialEnergy& operator=(PotentialEnergy&&) noexcept = default;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_POTENTIAL_ENERGY_HPP
