// Ticket: 0031_generalized_lagrange_constraints
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md

#include "msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

UnitQuaternionConstraint::UnitQuaternionConstraint(double alpha, double beta)
  : alpha_{alpha}, beta_{beta}
{
}

int UnitQuaternionConstraint::dimension() const
{
  return 1;  // Single scalar constraint: Q^T·Q - 1 = 0
}

Eigen::VectorXd UnitQuaternionConstraint::evaluate(const InertialState& state,
                                                     double /* time */) const
{
  // Constraint: C(Q) = Q^T·Q - 1 = 0
  const Eigen::Quaterniond& Q = state.orientation;

  // Compute Q^T·Q (dot product of quaternion with itself)
  double Q_dot_Q = Q.w() * Q.w() + Q.x() * Q.x() + Q.y() * Q.y() + Q.z() * Q.z();

  // Constraint violation: Q^T·Q - 1
  Eigen::VectorXd C(1);
  C(0) = Q_dot_Q - 1.0;

  return C;
}

Eigen::MatrixXd UnitQuaternionConstraint::jacobian(const InertialState& state,
                                                     double /* time */) const
{
  // Jacobian: J = ∂C/∂q where C = Q^T·Q - 1
  // ∂C/∂Q = 2·Q^T
  //
  // The full state Jacobian is (1 × 7) for position-level DOFs:
  // J = [∂C/∂X, ∂C/∂Q] = [0 0 0, 2w 2x 2y 2z]
  //
  // Since the constraint only depends on orientation, ∂C/∂X = 0

  const Eigen::Quaterniond& Q = state.orientation;

  Eigen::MatrixXd J(1, 7);
  J.setZero();

  // ∂C/∂X = 0 (constraint doesn't depend on position)
  // J(0, 0) = 0;  // Already zero from setZero()
  // J(0, 1) = 0;
  // J(0, 2) = 0;

  // ∂C/∂Q = 2·Q^T
  J(0, 3) = 2.0 * Q.w();  // ∂C/∂q_w
  J(0, 4) = 2.0 * Q.x();  // ∂C/∂q_x
  J(0, 5) = 2.0 * Q.y();  // ∂C/∂q_y
  J(0, 6) = 2.0 * Q.z();  // ∂C/∂q_z

  return J;
}

Eigen::VectorXd UnitQuaternionConstraint::partialTimeDerivative(
    const InertialState& /* state */,
    double /* time */) const
{
  // Time derivative: ∂C/∂t = 0 (time-independent constraint)
  Eigen::VectorXd result(1);
  result(0) = 0.0;
  return result;
}

std::string UnitQuaternionConstraint::typeName() const
{
  return "UnitQuaternionConstraint";
}

double UnitQuaternionConstraint::alpha() const
{
  return alpha_;
}

double UnitQuaternionConstraint::beta() const
{
  return beta_;
}

void UnitQuaternionConstraint::setAlpha(double alpha)
{
  alpha_ = alpha;
}

void UnitQuaternionConstraint::setBeta(double beta)
{
  beta_ = beta;
}

}  // namespace msd_sim
