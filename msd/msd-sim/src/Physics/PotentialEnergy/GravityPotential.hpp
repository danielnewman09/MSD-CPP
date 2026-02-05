// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md

#ifndef MSD_SIM_PHYSICS_GRAVITY_POTENTIAL_HPP
#define MSD_SIM_PHYSICS_GRAVITY_POTENTIAL_HPP

#include "msd-sim/src/Physics/PotentialEnergy/PotentialEnergy.hpp"

namespace msd_sim
{

/**
 * @brief Uniform gravitational field potential energy
 *
 * Implements V = m * g * z where z is the vertical position component.
 * The gravitational vector g is typically (0, 0, -9.81) in z-up convention.
 *
 * Uniform gravity produces constant force F = m*g but no torque (orientation-independent).
 *
 * @see docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml
 * @ticket 0030_lagrangian_quaternion_physics
 */
class GravityPotential : public PotentialEnergy
{
public:
  /**
   * @brief Construct gravitational field with specified acceleration vector
   * @param gravityVector Gravitational acceleration [m/s²], e.g. (0, 0, -9.81)
   */
  explicit GravityPotential(Coordinate  gravityVector);

  ~GravityPotential() override = default;

  // PotentialEnergy interface implementation
  [[nodiscard]] Coordinate computeForce(const InertialState& state, double mass) const override;
  [[nodiscard]] Coordinate computeTorque(const InertialState& state,
                           const Eigen::Matrix3d& inertia) const override;
  [[nodiscard]] double computeEnergy(const InertialState& state, double mass) const override;

  // Gravity configuration
  void setGravity(const Coordinate& gravityVector);
  [[nodiscard]] const Coordinate& getGravity() const;

  // Rule of Five
  GravityPotential(const GravityPotential&) = default;
  GravityPotential& operator=(const GravityPotential&) = default;
  GravityPotential(GravityPotential&&) noexcept = default;
  GravityPotential& operator=(GravityPotential&&) noexcept = default;

private:
  Coordinate g_{0.0, 0.0, -9.81};  // Gravitational acceleration [m/s²]
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_GRAVITY_POTENTIAL_HPP
