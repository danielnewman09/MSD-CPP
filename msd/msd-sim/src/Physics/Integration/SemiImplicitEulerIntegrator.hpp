// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md
// Modified: 0031_generalized_lagrange_constraints (ConstraintSolver)
// Modified: 0045_constraint_solver_unification (Removed ConstraintSolver)

#ifndef MSD_SIM_PHYSICS_SEMI_IMPLICIT_EULER_INTEGRATOR_HPP
#define MSD_SIM_PHYSICS_SEMI_IMPLICIT_EULER_INTEGRATOR_HPP

#include "msd-sim/src/Physics/Integration/Integrator.hpp"

namespace msd_sim
{

/**
 * @brief Semi-implicit Euler integrator (symplectic)
 *
 * Integration order:
 * 1. Compute accelerations: a = F_ext / m, α = I^-1 * τ_ext
 * 2. Update velocities: v_new = v_old + a * dt
 * 3. Update positions: x_new = x_old + v_new * dt (uses NEW velocity)
 * 4. Integrate quaternion: Q_new = Q_old + Q̇ * dt
 * 5. Normalize quaternion (position-level drift correction)
 *
 * Properties:
 * - First-order accurate
 * - Symplectic (preserves phase space volume)
 * - Better energy conservation than explicit Euler
 *
 * Ticket 0045 breaking change: Removed constraint solving from integrator.
 * Quaternion normalization is now handled via state.orientation.normalize()
 * instead of UnitQuaternionConstraint.
 *
 * @see
 * docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml
 * @see
 * docs/designs/0045_constraint_solver_unification/design.md
 * @ticket 0030_lagrangian_quaternion_physics
 * @ticket 0045_constraint_solver_unification
 */
class SemiImplicitEulerIntegrator : public Integrator
{
public:
  /**
   * @brief Construct integrator
   */
  SemiImplicitEulerIntegrator() = default;

  ~SemiImplicitEulerIntegrator() override = default;

  void step(InertialState& state,
            const Coordinate& force,
            const Coordinate& torque,
            double mass,
            const Eigen::Matrix3d& inverseInertia,
            double dt) override;

  // Rule of Five
  SemiImplicitEulerIntegrator(const SemiImplicitEulerIntegrator&) = default;
  SemiImplicitEulerIntegrator& operator=(const SemiImplicitEulerIntegrator&) =
    default;
  SemiImplicitEulerIntegrator(SemiImplicitEulerIntegrator&&) noexcept = default;
  SemiImplicitEulerIntegrator& operator=(
    SemiImplicitEulerIntegrator&&) noexcept = default;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_SEMI_IMPLICIT_EULER_INTEGRATOR_HPP
