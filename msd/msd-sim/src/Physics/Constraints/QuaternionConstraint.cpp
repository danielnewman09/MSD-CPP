// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md

#include "msd-sim/src/Physics/Constraints/QuaternionConstraint.hpp"
#include <cmath>

namespace msd_sim
{

QuaternionConstraint::QuaternionConstraint(double alpha, double beta)
  : alpha_{alpha}, beta_{beta}
{
}

void QuaternionConstraint::enforceConstraint(Eigen::Quaterniond& Q,
                                              Eigen::Vector4d& Qdot)
{
  // Step 1: Normalize quaternion
  // g(Q) = QᵀQ - 1 should be 0
  double norm = Q.norm();
  if (norm > 1e-10)  // Avoid division by zero
  {
    Q.coeffs() /= norm;
  }

  // Step 2: Project Qdot onto tangent space (perpendicular to Q)
  // Constraint: Q̇ ⊥ Q  ⟺  QᵀQ̇ = 0
  // Projection: Q̇_perp = Q̇ - (QᵀQ̇)Q
  Eigen::Vector4d Q_vec = Q.coeffs();
  double dot_product = Q_vec.dot(Qdot);
  Qdot -= dot_product * Q_vec;

  // Step 3: Apply Baumgarte stabilization
  // λ = -α*g - β*ġ where:
  //   g = QᵀQ - 1 (position violation)
  //   ġ = 2QᵀQ̇ (velocity violation)
  //
  // Constraint force: F_c = Gᵀλ where G = ∂g/∂Q = 2Q
  // This adds correction term to Q̇: Q̇_corrected = Q̇ + F_c * dt
  //
  // Since we're called after integration, apply correction directly
  double g = positionViolation(Q);
  double g_dot = velocityViolation(Q, Qdot);
  double lambda = -alpha_ * g - beta_ * g_dot;

  // Apply constraint correction to Qdot
  // F_c = Gᵀλ = 2Qλ
  Eigen::Vector4d constraint_force = 2.0 * lambda * Q_vec;
  Qdot += constraint_force;
}

Eigen::Vector4d QuaternionConstraint::computeConstraintForce(
  const Eigen::Quaterniond& Q,
  const Eigen::Vector4d& Qdot) const
{
  // Compute Lagrange multiplier
  double g = positionViolation(Q);
  double g_dot = velocityViolation(Q, Qdot);
  double lambda = -alpha_ * g - beta_ * g_dot;

  // Constraint force: F_c = Gᵀλ where G = ∂g/∂Q = 2Q
  Eigen::Vector4d Q_vec = Q.coeffs();
  return 2.0 * lambda * Q_vec;
}

void QuaternionConstraint::setAlpha(double alpha)
{
  alpha_ = alpha;
}

void QuaternionConstraint::setBeta(double beta)
{
  beta_ = beta;
}

double QuaternionConstraint::getAlpha() const
{
  return alpha_;
}

double QuaternionConstraint::getBeta() const
{
  return beta_;
}

double QuaternionConstraint::positionViolation(
  const Eigen::Quaterniond& Q) const
{
  // g(Q) = QᵀQ - 1
  double norm_squared = Q.squaredNorm();
  return norm_squared - 1.0;
}

double QuaternionConstraint::velocityViolation(
  const Eigen::Quaterniond& Q,
  const Eigen::Vector4d& Qdot) const
{
  // ġ = d/dt(QᵀQ - 1) = 2QᵀQ̇
  Eigen::Vector4d Q_vec = Q.coeffs();
  return 2.0 * Q_vec.dot(Qdot);
}

}  // namespace msd_sim
