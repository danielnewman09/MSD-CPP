// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md

#include "msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp"

namespace msd_sim
{

void SemiImplicitEulerIntegrator::step(InertialState& state,
                                        const Coordinate& force,
                                        const Coordinate& torque,
                                        double mass,
                                        const Eigen::Matrix3d& inverseInertia,
                                        QuaternionConstraint& constraint,
                                        double dt)
{
  // ===== Linear Integration (Semi-Implicit Euler) =====

  // Step 1: Compute linear acceleration: a = F_net / m
  Coordinate linearAccel = force / mass;
  state.acceleration = linearAccel;

  // Step 2: Update velocity: v_new = v_old + a * dt
  state.velocity += linearAccel * dt;

  // Step 3: Update position using NEW velocity: x_new = x_old + v_new * dt
  state.position += state.velocity * dt;

  // ===== Angular Integration (Semi-Implicit Euler) =====

  // Step 1: Compute angular acceleration: α = I⁻¹ * τ
  AngularRate angularAccel = inverseInertia * torque;
  state.angularAcceleration = angularAccel;

  // Step 2: Convert current Q̇ to ω for integration
  AngularRate omega = state.getAngularVelocity();

  // Step 3: Update angular velocity: ω_new = ω_old + α * dt
  omega += angularAccel * dt;

  // Step 4: Convert updated ω back to Q̇ for quaternion integration
  state.setAngularVelocity(omega);

  // Step 5: Integrate quaternion: Q_new = Q_old + Q̇ * dt
  Eigen::Vector4d Q_vec = state.orientation.coeffs();
  Q_vec += state.quaternionRate * dt;
  state.orientation.coeffs() = Q_vec;

  // ===== Enforce Quaternion Constraint =====
  // Apply Baumgarte stabilization to maintain |Q|=1
  constraint.enforceConstraint(state.orientation, state.quaternionRate);
}

}  // namespace msd_sim
