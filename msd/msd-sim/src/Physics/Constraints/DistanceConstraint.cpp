// Ticket: 0031_generalized_lagrange_constraints
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md

#include "msd-sim/src/Physics/Constraints/DistanceConstraint.hpp"
#include <stdexcept>
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

DistanceConstraint::DistanceConstraint(double targetDistance,
                                       double alpha,
                                       double beta)
  : targetDistance_{targetDistance}, alpha_{alpha}, beta_{beta}
{
  if (targetDistance <= 0.0)
  {
    throw std::invalid_argument(
      "DistanceConstraint: targetDistance must be positive, got " +
      std::to_string(targetDistance));
  }
}

int DistanceConstraint::dimension() const
{
  return 1;  // Single scalar constraint: |X|² - d² = 0
}

Eigen::VectorXd DistanceConstraint::evaluate(const InertialState& state,
                                             double /* time */) const
{
  // Constraint: C(X) = |X|² - d²
  const Coordinate& X = state.position;

  // Compute |X|² = X·X (dot product)
  double X_squared = X.dot(X);

  // Constraint violation: |X|² - d²
  Eigen::VectorXd C(1);
  C(0) = X_squared - (targetDistance_ * targetDistance_);

  return C;
}

Eigen::MatrixXd DistanceConstraint::jacobian(const InertialState& state,
                                             double /* time */) const
{
  // Jacobian: J = ∂C/∂q where C = |X|² - d²
  // ∂C/∂X = 2·X^T
  //
  // The full state Jacobian is (1 × 7) for position-level DOFs:
  // J = [∂C/∂X, ∂C/∂Q] = [2x 2y 2z, 0 0 0 0]
  //
  // Since the constraint only depends on position, ∂C/∂Q = 0

  const Coordinate& X = state.position;

  Eigen::MatrixXd J(1, 7);
  J.setZero();

  // ∂C/∂X = 2·X^T
  J(0, 0) = 2.0 * X.x();  // ∂C/∂x
  J(0, 1) = 2.0 * X.y();  // ∂C/∂y
  J(0, 2) = 2.0 * X.z();  // ∂C/∂z

  // ∂C/∂Q = 0 (constraint doesn't depend on orientation)
  // J(0, 3) = 0;  // Already zero from setZero()
  // J(0, 4) = 0;
  // J(0, 5) = 0;
  // J(0, 6) = 0;

  return J;
}

Eigen::VectorXd DistanceConstraint::partialTimeDerivative(
  const InertialState& /* state */,
  double /* time */) const
{
  // Time derivative: ∂C/∂t = 0 (time-independent constraint)
  Eigen::VectorXd result(1);
  result(0) = 0.0;
  return result;
}

std::string DistanceConstraint::typeName() const
{
  return "DistanceConstraint";
}

double DistanceConstraint::alpha() const
{
  return alpha_;
}

double DistanceConstraint::beta() const
{
  return beta_;
}

double DistanceConstraint::getTargetDistance() const
{
  return targetDistance_;
}

}  // namespace msd_sim
