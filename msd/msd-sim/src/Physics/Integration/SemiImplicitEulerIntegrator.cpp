// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md
// Modified: 0031_generalized_lagrange_constraints (ConstraintSolver)
// Modified: 0045_constraint_solver_unification (Removed constraint solving)

#include "msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

void SemiImplicitEulerIntegrator::step(
  InertialState& state,
  const Coordinate& force,
  const Coordinate& torque,
  double mass,
  const Eigen::Matrix3d& inverseInertiaWorld,
  double dt)
{
  // ===== Compute Accelerations =====

  const Coordinate linearAccel = force / mass;
  const AngularAcceleration angularAccel{inverseInertiaWorld * torque};

  state.acceleration = linearAccel;
  state.angularAcceleration = angularAccel;

  // ===== Semi-Implicit Euler Integration =====

  // Update velocity: v_new = v_old + a * dt
  state.velocity += linearAccel * dt;

  // Update position using NEW velocity: x_new = x_old + v_new * dt
  state.position += state.velocity * dt;

  // Convert angular acceleration to quaternion rate change
  AngularVelocity omega = state.getAngularVelocity();
  omega += angularAccel * dt;
  state.setAngularVelocity(omega);

  // Integrate quaternion: Q_new = Q_old + Q̇ * dt
  Vector4D qVec = state.orientation.coeffs();
  qVec += state.quaternionRate * dt;
  state.orientation.coeffs() = qVec;

  // ===== Position-Level Drift Correction =====
  // Normalize quaternion to maintain |Q|=1 within machine precision
  // Replaces velocity-level UnitQuaternionConstraint from ticket 0031
  state.orientation.normalize();
}

}  // namespace msd_sim
