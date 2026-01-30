// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md

#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"

namespace msd_sim
{

GravityPotential::GravityPotential(const Coordinate& gravityVector)
  : g_{gravityVector}
{
}

Coordinate GravityPotential::computeForce(const InertialState& /* state */,
                                          double mass) const
{
  // Uniform gravitational force: F = m * g
  // Force is independent of position (constant field)
  return g_ * mass;
}

Coordinate GravityPotential::computeTorque(const InertialState& /* state */,
                                           const Eigen::Matrix3d& /* inertia */) const
{
  // Uniform gravity produces no torque (orientation-independent)
  // τ = -∂V/∂Q = 0 since V does not depend on orientation Q
  return Coordinate{0.0, 0.0, 0.0};
}

double GravityPotential::computeEnergy(const InertialState& state,
                                       double mass) const
{
  // Gravitational potential energy: V = -m * g · r
  // For uniform field g = (0, 0, -9.81) at height z=10:
  //   V = -m * (0, 0, -9.81) · (0, 0, 10) = -m * (-98.1) = +98.1 * m
  // This gives positive potential energy for objects above the origin,
  // which correctly reflects their potential to do work by falling.
  return -mass * g_.dot(state.position);
}

void GravityPotential::setGravity(const Coordinate& gravityVector)
{
  g_ = gravityVector;
}

const Coordinate& GravityPotential::getGravity() const
{
  return g_;
}

}  // namespace msd_sim
