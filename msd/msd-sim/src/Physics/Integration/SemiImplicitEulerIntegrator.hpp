// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md

#ifndef MSD_SIM_PHYSICS_SEMI_IMPLICIT_EULER_INTEGRATOR_HPP
#define MSD_SIM_PHYSICS_SEMI_IMPLICIT_EULER_INTEGRATOR_HPP

#include "msd-sim/src/Physics/Integration/Integrator.hpp"

namespace msd_sim
{

/**
 * @brief Semi-implicit Euler integrator (symplectic)
 *
 * Integration order:
 * 1. Update velocities: v_new = v_old + a * dt
 * 2. Update positions: x_new = x_old + v_new * dt (uses NEW velocity)
 * 3. Enforce quaternion constraint with Baumgarte stabilization
 *
 * Properties:
 * - First-order accurate
 * - Symplectic (preserves phase space volume)
 * - Better energy conservation than explicit Euler
 * - Simple and computationally efficient
 *
 * @see docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml
 * @ticket 0030_lagrangian_quaternion_physics
 */
class SemiImplicitEulerIntegrator : public Integrator
{
public:
  SemiImplicitEulerIntegrator() = default;
  ~SemiImplicitEulerIntegrator() override = default;

  void step(InertialState& state,
            const Coordinate& force,
            const Coordinate& torque,
            double mass,
            const Eigen::Matrix3d& inverseInertia,
            QuaternionConstraint& constraint,
            double dt) override;

  // Rule of Five
  SemiImplicitEulerIntegrator(const SemiImplicitEulerIntegrator&) = default;
  SemiImplicitEulerIntegrator& operator=(const SemiImplicitEulerIntegrator&) = default;
  SemiImplicitEulerIntegrator(SemiImplicitEulerIntegrator&&) noexcept = default;
  SemiImplicitEulerIntegrator& operator=(SemiImplicitEulerIntegrator&&) noexcept = default;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_SEMI_IMPLICIT_EULER_INTEGRATOR_HPP
