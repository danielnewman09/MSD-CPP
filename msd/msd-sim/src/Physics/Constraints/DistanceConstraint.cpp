// Ticket: 0043_constraint_hierarchy_refactor
// Design: docs/designs/0043_constraint_hierarchy_refactor/design.md

#include "msd-sim/src/Physics/Constraints/DistanceConstraint.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <stdexcept>
#include <string>
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

DistanceConstraint::DistanceConstraint(double targetDistance,
                                       size_t bodyAIndex,
                                       double alpha,
                                       double beta)
  : Constraint{bodyAIndex, 0, alpha, beta},
    targetDistance_{targetDistance}
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

Eigen::VectorXd DistanceConstraint::evaluate(const InertialState& stateA,
                                             const InertialState& /* stateB */,
                                             double /* time */) const
{
  // Constraint: C(X) = |X|² - d²
  // Single-body constraint — uses stateA, ignores stateB
  const Coordinate& x = stateA.position;

  // Compute |X|² = X·X (dot product)
  double const xSquared = x.dot(x);

  // Constraint violation: |X|² - d²
  Eigen::VectorXd c(1);
  c(0) = xSquared - (targetDistance_ * targetDistance_);

  return c;
}

Eigen::MatrixXd DistanceConstraint::jacobian(const InertialState& stateA,
                                             const InertialState& /* stateB */,
                                             double /* time */) const
{
  // Jacobian: J = ∂C/∂q where C = |X|² - d²
  // ∂C/∂X = 2·X^T
  //
  // The full state Jacobian is (1 × 7) for position-level DOFs:
  // J = [∂C/∂X, ∂C/∂Q] = [2x 2y 2z, 0 0 0 0]
  //
  // Since the constraint only depends on position, ∂C/∂Q = 0
  // Single-body constraint — uses stateA, ignores stateB

  const Coordinate& x = stateA.position;

  Eigen::MatrixXd j(1, 7);
  j.setZero();

  // ∂C/∂X = 2·X^T
  j(0, 0) = 2.0 * x.x();  // ∂C/∂x
  j(0, 1) = 2.0 * x.y();  // ∂C/∂y
  j(0, 2) = 2.0 * x.z();  // ∂C/∂z

  // ∂C/∂Q = 0 (constraint doesn't depend on orientation)
  // J(0, 3) = 0;  // Already zero from setZero()
  // J(0, 4) = 0;
  // J(0, 5) = 0;
  // J(0, 6) = 0;

  return j;
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

LambdaBounds DistanceConstraint::lambdaBounds() const
{
  return LambdaBounds::bilateral();
}

std::string DistanceConstraint::typeName() const
{
  return "DistanceConstraint";
}

double DistanceConstraint::getTargetDistance() const
{
  return targetDistance_;
}

void DistanceConstraint::recordState(msd_transfer::ConstraintRecordVisitor& /* visitor */,
                                      uint32_t /* bodyAId */,
                                      uint32_t /* bodyBId */) const
{
  // Stub implementation: DistanceConstraint is a single-body example constraint
  // that is not currently used in production (vestigial from ticket 0043).
  // No recording needed as this constraint is never instantiated by CollisionPipeline.
  // Ticket: 0057_contact_tangent_recording
}

}  // namespace msd_sim
