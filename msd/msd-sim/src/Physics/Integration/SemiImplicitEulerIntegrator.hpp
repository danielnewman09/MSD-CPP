// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md
// Modified: 0031_generalized_lagrange_constraints (ConstraintSolver)

#ifndef MSD_SIM_PHYSICS_SEMI_IMPLICIT_EULER_INTEGRATOR_HPP
#define MSD_SIM_PHYSICS_SEMI_IMPLICIT_EULER_INTEGRATOR_HPP

#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Integration/Integrator.hpp"

namespace msd_sim
{

/**
 * @brief Semi-implicit Euler integrator (symplectic) with constraint
 * enforcement
 *
 * Integration order:
 * 1. Compute unconstrained accelerations: a_free = F_ext / m, α_free = I^-1 *
 * τ_ext
 * 2. Solve constraint system: result = solver.solve(constraints, ...)
 * 3. Apply constraint forces: a_total = a_free + F_c / m, α_total = α_free +
 * I^-1 * τ_c
 * 4. Update velocities: v_new = v_old + a_total * dt
 * 5. Update positions: x_new = x_old + v_new * dt (uses NEW velocity)
 * 6. Integrate quaternion: Q_new = Q_old + Q̇ * dt
 * 7. Normalize quaternion (implicit constraint enforcement)
 *
 * Properties:
 * - First-order accurate
 * - Symplectic (preserves phase space volume)
 * - Better energy conservation than explicit Euler
 * - Supports arbitrary constraint combinations via ConstraintSolver
 *
 * Ticket 0031 breaking change: Uses ConstraintSolver instead of direct
 * QuaternionConstraint::enforceConstraint() call.
 *
 * @see
 * docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml
 * @see
 * docs/designs/0031_generalized_lagrange_constraints/0031_generalized_lagrange_constraints.puml
 * @ticket 0030_lagrangian_quaternion_physics
 * @ticket 0031_generalized_lagrange_constraints
 */
class SemiImplicitEulerIntegrator : public Integrator
{
public:
  /**
   * @brief Construct integrator with default solver
   */
  SemiImplicitEulerIntegrator();

  /**
   * @brief Construct integrator with custom solver
   * @param solver ConstraintSolver instance (copied)
   */
  explicit SemiImplicitEulerIntegrator(ConstraintSolver solver);

  ~SemiImplicitEulerIntegrator() override = default;

  void step(InertialState& state,
            const Coordinate& force,
            const Coordinate& torque,
            double mass,
            const Eigen::Matrix3d& inverseInertia,
            const std::vector<Constraint*>& constraints,
            double dt) override;

  // Rule of Five
  SemiImplicitEulerIntegrator(const SemiImplicitEulerIntegrator&) = default;
  SemiImplicitEulerIntegrator& operator=(const SemiImplicitEulerIntegrator&) =
    default;
  SemiImplicitEulerIntegrator(SemiImplicitEulerIntegrator&&) noexcept = default;
  SemiImplicitEulerIntegrator& operator=(
    SemiImplicitEulerIntegrator&&) noexcept = default;

private:
  ConstraintSolver solver_;  // Constraint solver instance
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_SEMI_IMPLICIT_EULER_INTEGRATOR_HPP
