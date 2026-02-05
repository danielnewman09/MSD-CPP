// Ticket: 0031_generalized_lagrange_constraints
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md

#include "msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <string>
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
  const Eigen::Quaterniond& q = state.orientation;

  // Compute Q^T·Q (dot product of quaternion with itself)
  double const qDotQ = (q.w() * q.w()) + (q.x() * q.x()) + (q.y() * q.y()) + (q.z() * q.z());

  // Constraint violation: Q^T·Q - 1
  Eigen::VectorXd c(1);
  c(0) = qDotQ - 1.0;

  return c;
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

  const Eigen::Quaterniond& q = state.orientation;

  Eigen::MatrixXd j(1, 7);
  j.setZero();

  // ∂C/∂X = 0 (constraint doesn't depend on position)
  // J(0, 0) = 0;  // Already zero from setZero()
  // J(0, 1) = 0;
  // J(0, 2) = 0;

  // ∂C/∂Q = 2·Q^T
  j(0, 3) = 2.0 * q.w();  // ∂C/∂q_w
  j(0, 4) = 2.0 * q.x();  // ∂C/∂q_x
  j(0, 5) = 2.0 * q.y();  // ∂C/∂q_y
  j(0, 6) = 2.0 * q.z();  // ∂C/∂q_z

  return j;
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
