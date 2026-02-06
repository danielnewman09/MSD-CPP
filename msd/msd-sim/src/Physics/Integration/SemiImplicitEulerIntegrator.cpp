// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md
// Modified: 0031_generalized_lagrange_constraints (ConstraintSolver)

#include "msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

SemiImplicitEulerIntegrator::SemiImplicitEulerIntegrator() = default;

SemiImplicitEulerIntegrator::SemiImplicitEulerIntegrator(
  ConstraintSolver solver)
  : solver_{solver}
{
}

void SemiImplicitEulerIntegrator::step(
  InertialState& state,
  const Coordinate& force,
  const Coordinate& torque,
  double mass,
  const Eigen::Matrix3d& inverseInertiaWorld,
  const std::vector<Constraint*>& constraints,
  double dt)
{
  // ===== Compute Unconstrained Accelerations =====

  const Coordinate linearAccelFree = force / mass;
  const AngularRate angularAccelFree{inverseInertiaWorld * torque};

  // ===== Solve Constraint System =====

  const ConstraintSolver::SolveResult result = msd_sim::ConstraintSolver::solve(
    constraints, state, force, torque, mass, inverseInertiaWorld, dt);

  // If solver didn't converge, proceed without constraint forces (graceful
  // degradation)
  Coordinate constraintLinearForce{0.0, 0.0, 0.0};
  Coordinate constraintAngularTorque{0.0, 0.0, 0.0};
  if (result.converged)
  {
    constraintLinearForce = result.linearConstraintForce;
    constraintAngularTorque = result.angularConstraintForce;
  }

  // ===== Apply Constraint Forces =====

  Coordinate const linearAccelTotal =
    linearAccelFree + (constraintLinearForce / mass);
  AngularRate const angularAccelTotal{
    angularAccelFree + (inverseInertiaWorld * constraintAngularTorque)};

  state.acceleration = linearAccelTotal;
  state.angularAcceleration = angularAccelTotal;

  // ===== Semi-Implicit Euler Integration =====

  // Update velocity: v_new = v_old + a_total * dt
  state.velocity += linearAccelTotal * dt;

  // Update position using NEW velocity: x_new = x_old + v_new * dt
  state.position += state.velocity * dt;

  // Convert angular acceleration to quaternion rate change
  AngularRate omega = state.getAngularVelocity();
  omega += angularAccelTotal * dt;
  state.setAngularVelocity(omega);

  // Integrate quaternion: Q_new = Q_old + Q̇ * dt
  Vector4D qVec = state.orientation.coeffs();
  qVec += state.quaternionRate * dt;
  state.orientation.coeffs() = qVec;

  // ===== Implicit Constraint Enforcement =====
  // Normalize quaternion to maintain |Q|=1 within machine precision
  state.orientation.normalize();
}

}  // namespace msd_sim
