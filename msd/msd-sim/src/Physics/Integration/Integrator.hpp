// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md
// Modified: 0031_generalized_lagrange_constraints (constraint vector)

#ifndef MSD_SIM_PHYSICS_INTEGRATOR_HPP
#define MSD_SIM_PHYSICS_INTEGRATOR_HPP

#include <Eigen/Dense>
#include <vector>
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

// Forward declaration
class Constraint;

/**
 * @brief Abstract interface for numerical integration of rigid body dynamics
 *
 * Decouples the integration scheme from the physics computation, enabling:
 * - Swappable integrators (Euler, RK4, Verlet, etc.)
 * - Isolated testing of integration math
 * - WorldModel as pure orchestrator
 *
 * Ticket 0031 breaking change: Replaced QuaternionConstraint& with
 * std::vector<Constraint*>& to support arbitrary constraint combinations.
 *
 * Thread safety: Implementations should be stateless and thread-safe
 *
 * @see
 * docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml
 * @see
 * docs/designs/0031_generalized_lagrange_constraints/0031_generalized_lagrange_constraints.puml
 * @ticket 0030_lagrangian_quaternion_physics
 * @ticket 0031_generalized_lagrange_constraints
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
   * @param constraints Vector of constraint pointers (non-owning)
   * @param dt Timestep [s]
   *
   * Ticket 0031 breaking change: Parameter changed from single
   * QuaternionConstraint& to std::vector<Constraint*>& to support multiple
   * constraints per object.
   */
  virtual void step(InertialState& state,
                    const Coordinate& force,
                    const Coordinate& torque,
                    double mass,
                    const Eigen::Matrix3d& inverseInertia,
                    const std::vector<Constraint*>& constraints,
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
