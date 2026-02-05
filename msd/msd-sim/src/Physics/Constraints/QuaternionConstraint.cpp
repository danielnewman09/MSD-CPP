// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md

#include "msd-sim/src/Physics/Constraints/QuaternionConstraint.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>

namespace msd_sim
{

QuaternionConstraint::QuaternionConstraint(double alpha, double beta)
  : alpha_{alpha}, beta_{beta}
{
}

void QuaternionConstraint::enforceConstraint(Eigen::Quaterniond& Q,
                                             Eigen::Vector4d& Qdot) const
{
  // Step 1: Normalize quaternion
  // g(Q) = QᵀQ - 1 should be 0
  double const norm = Q.norm();
  if (norm > 1e-10)  // Avoid division by zero
  {
    Q.coeffs() /= norm;
  }

  // Step 2: Project Qdot onto tangent space (perpendicular to Q)
  // Constraint: Q̇ ⊥ Q  ⟺  QᵀQ̇ = 0
  // Projection: Q̇_perp = Q̇ - (QᵀQ̇)Q
  Eigen::Vector4d const qVec = Q.coeffs();
  double const dotProduct = qVec.dot(Qdot);
  Qdot -= dotProduct * qVec;

  // Step 3: Apply Baumgarte stabilization
  // λ = -α*g - β*ġ where:
  //   g = QᵀQ - 1 (position violation)
  //   ġ = 2QᵀQ̇ (velocity violation)
  //
  // Constraint force: F_c = Gᵀλ where G = ∂g/∂Q = 2Q
  // This adds correction term to Q̇: Q̇_corrected = Q̇ + F_c * dt
  //
  // Since we're called after integration, apply correction directly
  double const g = positionViolation(Q);
  double const gDot = velocityViolation(Q, Qdot);
  double const lambda = (-alpha_ * g) - (beta_ * gDot);

  // Apply constraint correction to Qdot
  // F_c = Gᵀλ = 2Qλ
  Eigen::Vector4d const constraintForce = 2.0 * lambda * qVec;
  Qdot += constraintForce;
}

Eigen::Vector4d QuaternionConstraint::computeConstraintForce(
  const Eigen::Quaterniond& Q,
  const Eigen::Vector4d& Qdot) const
{
  // Compute Lagrange multiplier
  double const g = positionViolation(Q);
  double const gDot = velocityViolation(Q, Qdot);
  double const lambda = (-alpha_ * g) - (beta_ * gDot);

  // Constraint force: F_c = Gᵀλ where G = ∂g/∂Q = 2Q
  const Eigen::Vector4d& qVec = Q.coeffs();
  return 2.0 * lambda * qVec;
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

double QuaternionConstraint::positionViolation(const Eigen::Quaterniond& Q)
{
  // g(Q) = QᵀQ - 1
  double const normSquared = Q.squaredNorm();
  return normSquared - 1.0;
}

double QuaternionConstraint::velocityViolation(const Eigen::Quaterniond& Q,
                                               const Eigen::Vector4d& Qdot)
{
  // ġ = d/dt(QᵀQ - 1) = 2QᵀQ̇
  const Eigen::Vector4d& qVec = Q.coeffs();
  return 2.0 * qVec.dot(Qdot);
}

}  // namespace msd_sim
