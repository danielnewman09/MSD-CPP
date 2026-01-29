// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md

#ifndef MSD_SIM_PHYSICS_INTEGRATOR_HPP
#define MSD_SIM_PHYSICS_INTEGRATOR_HPP

#include <Eigen/Dense>
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"
#include "msd-sim/src/Physics/Constraints/QuaternionConstraint.hpp"

namespace msd_sim
{

/**
 * @brief Abstract interface for numerical integration of rigid body dynamics
 *
 * Decouples the integration scheme from the physics computation, enabling:
 * - Swappable integrators (Euler, RK4, Verlet, etc.)
 * - Isolated testing of integration math
 * - WorldModel as pure orchestrator
 *
 * Thread safety: Implementations should be stateless and thread-safe
 *
 * @see docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml
 * @ticket 0030_lagrangian_quaternion_physics
 */
class Integrator
{
public:
  virtual ~Integrator() = default;

  /**
   * @brief Integrate state forward by one timestep
   * @param state Current inertial state (modified in place)
   * @param force Net force in world frame [N]
   * @param torque Net torque in world frame [N·m]
   * @param mass Object mass [kg]
   * @param inverseInertia Inverse inertia tensor in body frame [1/(kg·m²)]
   * @param constraint Quaternion constraint for normalization
   * @param dt Timestep [s]
   */
  virtual void step(InertialState& state,
                    const Coordinate& force,
                    const Coordinate& torque,
                    double mass,
                    const Eigen::Matrix3d& inverseInertia,
                    QuaternionConstraint& constraint,
                    double dt) = 0;

protected:
  Integrator() = default;
  Integrator(const Integrator&) = default;
  Integrator& operator=(const Integrator&) = default;
  Integrator(Integrator&&) noexcept = default;
  Integrator& operator=(Integrator&&) noexcept = default;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_INTEGRATOR_HPP
